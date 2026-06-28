/*

    Simple Satellite Operations  packet_db.c

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "packet_db.h"

#include "sso_paths.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

// /dev/urandom-backed run-id: 8 random bytes, hex-encoded. Not strong
// crypto — just an opaque label for grouping rows from one process
// launch. /dev/urandom is the safest portable source on Linux + macOS
// for non-crypto randomness too (avoids the rand() global state and
// the seeding-from-time-mod-32 trap).
int packet_db_make_run_id(char *buf, size_t cap)
{
    if (buf == NULL || cap < 17) return -1;
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd < 0) return -1;
    uint8_t bytes[8];
    ssize_t n = read(fd, bytes, sizeof bytes);
    close(fd);
    if (n != (ssize_t)sizeof bytes) return -1;
    static const char hex[] = "0123456789abcdef";
    for (int i = 0; i < 8; ++i) {
        buf[2 * i + 0] = hex[(bytes[i] >> 4) & 0xF];
        buf[2 * i + 1] = hex[bytes[i] & 0xF];
    }
    buf[16] = '\0';
    return 0;
}

packet_db_t *packet_db_setup(const char *user_path,
                             int no_db,
                             char *run_id_out, size_t run_id_cap)
{
    if (run_id_out != NULL && run_id_cap > 0) run_id_out[0] = '\0';
    if (no_db) return NULL;
    char default_buf[1024];
    const char *path = user_path;
    if (path == NULL) {
        if (packet_db_default_path(default_buf, sizeof default_buf) != 0) {
            fprintf(stderr,
                    "packet_db: cannot resolve default DB path "
                    "($SSO_PACKET_DB or $HOME unset). Pass --db=<path> "
                    "or --no-db to silence.\n");
            return NULL;
        }
        path = default_buf;
    }
    packet_db_t *db = packet_db_open(path);
    if (db == NULL) {
        fprintf(stderr,
                "packet_db: open(%s) failed (sqlite3 not built in?). "
                "Use --no-db to skip.\n", path);
        return NULL;
    }
    if (run_id_out != NULL && run_id_cap > 0) {
        if (packet_db_make_run_id(run_id_out, run_id_cap) != 0) {
            run_id_out[0] = '\0';
        }
    }
    return db;
}

int packet_db_default_path(char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return -1;
    const char *override = getenv("SSO_PACKET_DB");
    if (override != NULL && override[0] != '\0') {
        if (strlen(override) + 1 > cap) return -1;
        memcpy(buf, override, strlen(override) + 1);
        return 0;
    }
    // Otherwise the shared FrontierSat tree, so every operator sees the
    // same DB. sso_packet_db_path() resolves to $FRONTIERSAT_ROOT if
    // set, else /FrontierSat, and never returns empty. A dev host that
    // wants a different location sets FRONTIERSAT_ROOT (or
    // $SSO_PACKET_DB above).
    const char *shared = sso_packet_db_path();
    if (shared == NULL || shared[0] == '\0') return -1;
    if (strlen(shared) + 1 > cap) return -1;
    sso_mkdir_p_for_file(shared);
    memcpy(buf, shared, strlen(shared) + 1);
    return 0;
}

#ifdef WITH_SQLITE3

// SHA1 via the EVP_* API — the legacy SHA1_Init/Update/Final and the
// one-shot SHA1() helper are -Wdeprecated-declarations on OpenSSL 3.0+
// (Ubuntu 22.04 etc.). EVP is the supported successor and works on
// older OpenSSL too.
#include <openssl/evp.h>
#include <sqlite3.h>

#define PACKET_DB_SHA1_LEN 20

static void sha1_digest(const void *data, size_t len, uint8_t out[PACKET_DB_SHA1_LEN])
{
    // Zero first so out is defined even if anything below fails -- this is on
    // the dedup hot path, so a NULL ctx (allocation failure) must not deref.
    memset(out, 0, PACKET_DB_SHA1_LEN);
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    if (ctx == NULL) return;
    if (EVP_DigestInit_ex(ctx, EVP_sha1(), NULL) == 1
        && EVP_DigestUpdate(ctx, data, len) == 1) {
        EVP_DigestFinal_ex(ctx, out, NULL);
    }
    EVP_MD_CTX_free(ctx);
}

struct packet_db {
    sqlite3      *db;
    sqlite3_stmt *insert_stmt;
    sqlite3_stmt *update_gaps_stmt;
    sqlite3_stmt *update_force_stmt;
    sqlite3_stmt *update_replay_ts_stmt;
    sqlite3_stmt *register_tle_stmt;
    sqlite3_stmt *select_tle_id_stmt;
    sqlite3_stmt *insert_sent_tcmd_stmt;
};

// Fresh-DB schema. Existing DBs from user_version=1 get the new columns
// via the ALTER TABLE migration in packet_db_open after this CREATE
// runs.
static const char SCHEMA_SQL[] =
    "CREATE TABLE IF NOT EXISTS tle (\n"
    "  id              INTEGER PRIMARY KEY AUTOINCREMENT,\n"
    "  satellite       TEXT NOT NULL,\n"
    "  catalog_number  INTEGER,\n"
    "  epoch_year      INTEGER,\n"
    "  epoch_day       REAL,\n"
    "  line1           TEXT NOT NULL,\n"
    "  line2           TEXT NOT NULL,\n"
    "  sha1            BLOB NOT NULL UNIQUE,\n"
    "  first_seen      TEXT NOT NULL\n"
    ");\n"
    "CREATE TABLE IF NOT EXISTS packet (\n"
    "  id              INTEGER PRIMARY KEY AUTOINCREMENT,\n"
    "  ts_received     TEXT NOT NULL,\n"
    "  satellite       TEXT,\n"
    "  packet_type     INTEGER NOT NULL,\n"
    "  packet_type_name TEXT NOT NULL,\n"
    "  csp_src         INTEGER, csp_dst INTEGER,\n"
    "  csp_dport       INTEGER, csp_sport INTEGER,\n"
    "  csp_prio        INTEGER, csp_flags INTEGER,\n"
    "  payload         BLOB NOT NULL,\n"
    "  payload_sha1    BLOB NOT NULL,\n"
    "  golay_errs      INTEGER,\n"
    "  rs_errs         INTEGER,\n"
    // hmac_ok: legacy. RX integrity is now the CSP CRC32 (crc_status), not
    // HMAC, so new rows always store -1 ("not checked"). Kept so old rows
    // and any future uplink-side use still have a column to read.
    "  hmac_ok         INTEGER,\n"
    "  crc_status      INTEGER,\n"
    "  source_tool     TEXT NOT NULL,\n"
    "  source_run      TEXT,\n"
    "  audio_offset_s  REAL,\n"
    "  decoded_summary TEXT,\n"
    "  az_deg            REAL,\n"
    "  el_deg            REAL,\n"
    "  range_km          REAL,\n"
    "  range_rate_km_s   REAL,\n"
    "  doppler_hz_offset REAL,\n"
    "  tle_id            INTEGER REFERENCES tle(id),\n"
    "  session_dir       TEXT,\n"
    "  capture_origin    TEXT,\n"
    "  UNIQUE (payload_sha1, source_tool, source_run)\n"
    ");\n"
    "CREATE INDEX IF NOT EXISTS idx_packet_ts        ON packet(ts_received);\n"
    "CREATE INDEX IF NOT EXISTS idx_packet_type      ON packet(packet_type);\n"
    "CREATE INDEX IF NOT EXISTS idx_packet_satellite ON packet(satellite);\n"
    // Telecommands transmitted by simple_sat_ops, keyed by the @tssent
    // value the satellite echoes back in its tcmd_response. Lets a
    // reviewer resolve a received response to the command (and arguments)
    // that produced it. A separate, additive table: it never alters the
    // packet table, so existing DBs gain it via CREATE TABLE IF NOT
    // EXISTS on the next open with no migration step.
    "CREATE TABLE IF NOT EXISTS sent_tcmd (\n"
    "  id              INTEGER PRIMARY KEY AUTOINCREMENT,\n"
    "  ts_sent_ms      INTEGER NOT NULL,\n"
    "  tsexec_ms       INTEGER,\n"
    "  command_text    TEXT NOT NULL,\n"
    "  tx_freq_hz      INTEGER,\n"
    "  tx_gain_db      REAL,\n"
    "  source_tool     TEXT NOT NULL,\n"
    "  source_run      TEXT,\n"
    "  ts_transmitted  TEXT NOT NULL,\n"
    "  UNIQUE (ts_sent_ms, source_run)\n"
    ");\n"
    "CREATE INDEX IF NOT EXISTS idx_sent_tcmd_ts ON sent_tcmd(ts_sent_ms);\n";
// The (payload_sha1, source_tool) UNIQUE INDEX is added by the V3
// migration step below — putting it here would fail on an existing V2
// DB whose duplicates haven't been collapsed yet.

// Idempotent ALTERs for existing DBs created at schema version 1.
// "duplicate column name" errors are caught and ignored — re-running
// is a no-op.
static const char *const MIGRATION_V2_ALTERS[] = {
    "ALTER TABLE packet ADD COLUMN az_deg            REAL",
    "ALTER TABLE packet ADD COLUMN el_deg            REAL",
    "ALTER TABLE packet ADD COLUMN range_km          REAL",
    "ALTER TABLE packet ADD COLUMN range_rate_km_s   REAL",
    "ALTER TABLE packet ADD COLUMN doppler_hz_offset REAL",
    "ALTER TABLE packet ADD COLUMN tle_id            INTEGER REFERENCES tle(id)",
    "ALTER TABLE packet ADD COLUMN session_dir       TEXT",
    NULL
};

// V2 -> V3 migration: tighten dedup to (payload_sha1, source_tool).
// The original schema's UNIQUE(payload_sha1, source_tool, source_run)
// allowed a re-run of the same capture (different random source_run)
// to insert duplicate rows. Step 1 deletes the duplicates, keeping
// the row with the smallest id (the first one observed). Step 2 adds
// a UNIQUE INDEX that future INSERT OR IGNOREs will collide against,
// so the dedup is now per-tool rather than per-run. The original
// table-level UNIQUE constraint stays in place (subset of the new
// index, harmless).
static const char *const MIGRATION_V3_STEPS[] = {
    "DELETE FROM packet WHERE id NOT IN ("
    "  SELECT MIN(id) FROM packet GROUP BY payload_sha1, source_tool"
    ")",
    "CREATE UNIQUE INDEX IF NOT EXISTS idx_packet_sha1_tool "
    "  ON packet(payload_sha1, source_tool)",
    NULL
};

// V3 -> V4 migration: add capture_origin and widen the dedup key to
// include it. Without the widening, the same beacon decoded from our
// local WAV and from a SatNOGS .ogg would collide and one of the two
// rows would be dropped on insert — but we explicitly want both,
// since the point of pulling in SatNOGS audio is to compare what each
// site heard. Step 1 adds the column (idempotent — caught by
// "duplicate column name"). Step 2 backfills legacy NULLs to
// 'cts_ground' so re-running over the same captures with the new
// --capture-origin=cts_ground flag doesn't conflict with the old
// NULL-tagged row. Steps 3-4 swap the V3 (sha1, tool) index for the
// new (sha1, tool, capture_origin) one.
static const char *const MIGRATION_V4_STEPS[] = {
    "ALTER TABLE packet ADD COLUMN capture_origin TEXT",
    "UPDATE packet SET capture_origin = 'cts_ground' "
    "  WHERE capture_origin IS NULL",
    "DROP INDEX IF EXISTS idx_packet_sha1_tool",
    "CREATE UNIQUE INDEX IF NOT EXISTS idx_packet_sha1_tool_origin "
    "  ON packet(payload_sha1, source_tool, capture_origin)",
    NULL
};

static const char INSERT_SQL[] =
    "INSERT OR IGNORE INTO packet ("
    "  ts_received, satellite, packet_type, packet_type_name,"
    "  csp_src, csp_dst, csp_dport, csp_sport, csp_prio, csp_flags,"
    "  payload, payload_sha1,"
    "  golay_errs, rs_errs, hmac_ok, crc_status,"
    "  source_tool, source_run, audio_offset_s, decoded_summary,"
    "  az_deg, el_deg, range_km, range_rate_km_s, doppler_hz_offset,"
    "  tle_id, session_dir, capture_origin"
    ") VALUES ("
    "  ?1, ?2, ?3, ?4,"
    "  ?5, ?6, ?7, ?8, ?9, ?10,"
    "  ?11, ?12,"
    "  ?13, ?14, ?15, ?16,"
    "  ?17, ?18, ?19, ?20,"
    "  ?21, ?22, ?23, ?24, ?25,"
    "  ?26, ?27, ?28"
    ");";

// UPDATE for the rx_replay backfill path. Fills observer / tle / dir
// columns on the row matching payload_sha1, ONLY when those columns are
// currently NULL (default), or unconditionally when force is non-zero.
// Multiple rows may match (different source_tool / source_run) — we
// update all of them, since the observer state is a property of the
// physical packet, not of which tool decoded it.
static const char UPDATE_OBSERVER_GAPS_SQL[] =
    "UPDATE packet SET"
    "  az_deg = COALESCE(az_deg, ?1),"
    "  el_deg = COALESCE(el_deg, ?2),"
    "  range_km = COALESCE(range_km, ?3),"
    "  range_rate_km_s = COALESCE(range_rate_km_s, ?4),"
    "  doppler_hz_offset = COALESCE(doppler_hz_offset, ?5),"
    "  tle_id = COALESCE(tle_id, ?6),"
    "  session_dir = COALESCE(session_dir, ?7)"
    " WHERE payload_sha1 = ?8;";
static const char UPDATE_OBSERVER_FORCE_SQL[] =
    "UPDATE packet SET"
    "  az_deg = ?1, el_deg = ?2, range_km = ?3,"
    "  range_rate_km_s = ?4, doppler_hz_offset = ?5,"
    "  tle_id = ?6, session_dir = ?7"
    " WHERE payload_sha1 = ?8;";

// Targeted rewrite of ts_received and audio_offset_s on rx_replay
// rows. Scoped to source_tool = 'rx_replay' so live-decode rows
// (where ts_received already reflects the moment of reception) don't
// get clobbered by a backfill that's merely "WAV anchor + offset."
static const char UPDATE_REPLAY_TS_SQL[] =
    "UPDATE packet SET"
    "  ts_received = ?1, audio_offset_s = ?2"
    " WHERE payload_sha1 = ?3 AND source_tool = 'rx_replay';";

static const char REGISTER_TLE_SQL[] =
    "INSERT OR IGNORE INTO tle ("
    "  satellite, catalog_number, epoch_year, epoch_day,"
    "  line1, line2, sha1, first_seen"
    ") VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8);";

static const char SELECT_TLE_ID_SQL[] =
    "SELECT id FROM tle WHERE sha1 = ?1;";

static const char INSERT_SENT_TCMD_SQL[] =
    "INSERT OR IGNORE INTO sent_tcmd ("
    "  ts_sent_ms, tsexec_ms, command_text, tx_freq_hz, tx_gain_db,"
    "  source_tool, source_run, ts_transmitted"
    ") VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8);";

packet_db_t *packet_db_open(const char *path)
{
    if (path == NULL || path[0] == '\0') return NULL;
    sqlite3 *raw = NULL;
    // FULLMUTEX -> serialized threading mode for this connection, so the live
    // receiver can share one handle between the RX worker (packet inserts) and
    // the TX thread (sent_tcmd inserts) without each call racing inside SQLite.
    // Independent of the library-wide compile-time threading default.
    int rc = sqlite3_open_v2(path, &raw,
                             SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE
                             | SQLITE_OPEN_FULLMUTEX,
                             NULL);
    if (rc != SQLITE_OK) {
        if (raw != NULL) sqlite3_close(raw);
        errno = EIO;
        return NULL;
    }
    // Set the busy timeout BEFORE any locking statement. The WAL pragma,
    // schema apply, and the migration blocks below all take the write lock;
    // without a timeout in place a concurrent opener that hits the lock fails
    // the open outright (returns NULL) instead of waiting.
    //
    // 60 s, not 5 s: under parallel decode (decode_passes --jobs N) plus a
    // live receiver writing the same file, a long capture's inserts are
    // spread across a contended window, and a 5 s timeout let SQLITE_BUSY
    // drop whole passes (issue #52). The window matters more than the per-row
    // cost — single-row inserts still return in microseconds when the lock is
    // free; the timeout only bounds how long a contended insert waits before
    // giving up. A genuine deadlock still surfaces, just 60 s later.
    sqlite3_busy_timeout(raw, 60000);
    // WAL gives concurrent readers + one writer without the readers
    // blocking. Multiple receivers writing in parallel still serialise
    // on the single-writer rule, but that's fine at our packet rates.
    char *errmsg = NULL;
    if (sqlite3_exec(raw, "PRAGMA journal_mode=WAL;", NULL, NULL, &errmsg)
        != SQLITE_OK) {
        sqlite3_free(errmsg);
        sqlite3_close(raw);
        errno = EIO;
        return NULL;
    }
    // synchronous=NORMAL is the standard WAL companion: durable across
    // process crashes (fsync at checkpoint), not across OS crashes.
    // Cuts per-insert latency without affecting our use case (post-
    // pass review is fine with checkpoint-level durability).
    (void) sqlite3_exec(raw, "PRAGMA synchronous=NORMAL;", NULL, NULL, NULL);
    if (sqlite3_exec(raw, SCHEMA_SQL, NULL, NULL, &errmsg) != SQLITE_OK) {
        fprintf(stderr, "packet_db: schema apply failed: %s\n",
                errmsg ? errmsg : "(unknown)");
        sqlite3_free(errmsg);
        sqlite3_close(raw);
        errno = EIO;
        return NULL;
    }

    // Read the stored schema version so we run only the migration blocks
    // newer than it. Every block below is idempotent, but the V3 DELETE and
    // the V4 backfill UPDATE are full-table scans under a write lock -- on a
    // fully-migrated DB they did real work (scan + lock) on every open,
    // including every short-lived tool launch that contends with a live
    // receiver. A new DB reports 0 and migrates once; thereafter the version
    // matches and the ladder is skipped.
    int user_version = 0;
    {
        sqlite3_stmt *uv = NULL;
        if (sqlite3_prepare_v2(raw, "PRAGMA user_version;", -1, &uv, NULL)
                == SQLITE_OK
            && sqlite3_step(uv) == SQLITE_ROW) {
            user_version = sqlite3_column_int(uv, 0);
        }
        sqlite3_finalize(uv);
    }

    // V1 -> V2 migration: idempotent ALTER TABLE ADD COLUMNs. SQLite
    // returns "duplicate column name" when the column already exists;
    // those are caught and silently ignored so re-running is safe.
    if (user_version < 2) {
        for (int i = 0; MIGRATION_V2_ALTERS[i] != NULL; i++) {
            char *m_err = NULL;
            if (sqlite3_exec(raw, MIGRATION_V2_ALTERS[i], NULL, NULL, &m_err)
                != SQLITE_OK) {
                int dup = (m_err != NULL
                           && strstr(m_err, "duplicate column name") != NULL);
                if (!dup) {
                    fprintf(stderr, "packet_db: migration step '%s' failed: "
                            "%s\n", MIGRATION_V2_ALTERS[i],
                            m_err ? m_err : "(unknown)");
                    sqlite3_free(m_err);
                    sqlite3_close(raw);
                    errno = EIO;
                    return NULL;
                }
                sqlite3_free(m_err);
            }
        }
        // Stamp the version so the next open skips this block. A failed
        // stamp just means a future open re-runs the (idempotent) migration.
        (void) sqlite3_exec(raw, "PRAGMA user_version = 2;", NULL, NULL, NULL);
    }

    // V2 -> V3: collapse same-payload/same-tool duplicates and lock in
    // the tighter UNIQUE so future re-runs don't re-create them. Both
    // steps are idempotent — DELETE on a deduped table removes nothing
    // and CREATE UNIQUE INDEX IF NOT EXISTS is a no-op.
    if (user_version < 3) {
        for (int i = 0; MIGRATION_V3_STEPS[i] != NULL; i++) {
            char *m_err = NULL;
            if (sqlite3_exec(raw, MIGRATION_V3_STEPS[i], NULL, NULL, &m_err)
                != SQLITE_OK) {
                fprintf(stderr, "packet_db: migration step '%s' failed: "
                        "%s\n", MIGRATION_V3_STEPS[i],
                        m_err ? m_err : "(unknown)");
                sqlite3_free(m_err);
                sqlite3_close(raw);
                errno = EIO;
                return NULL;
            }
        }
        (void) sqlite3_exec(raw, "PRAGMA user_version = 3;", NULL, NULL, NULL);
    }

    // V3 -> V4: add capture_origin column, backfill legacy rows to
    // 'cts_ground', and widen the dedup index to include origin. The
    // ALTER step's "duplicate column name" error is caught and
    // ignored so a re-run is a no-op.
    if (user_version < 4) {
        for (int i = 0; MIGRATION_V4_STEPS[i] != NULL; i++) {
            char *m_err = NULL;
            if (sqlite3_exec(raw, MIGRATION_V4_STEPS[i], NULL, NULL, &m_err)
                != SQLITE_OK) {
                int dup = (m_err != NULL
                           && strstr(m_err, "duplicate column name") != NULL);
                if (!dup) {
                    fprintf(stderr, "packet_db: migration step '%s' failed: "
                            "%s\n", MIGRATION_V4_STEPS[i],
                            m_err ? m_err : "(unknown)");
                    sqlite3_free(m_err);
                    sqlite3_close(raw);
                    errno = EIO;
                    return NULL;
                }
                sqlite3_free(m_err);
            }
        }
        (void) sqlite3_exec(raw, "PRAGMA user_version = 4;", NULL, NULL, NULL);
    }

    packet_db_t *db = (packet_db_t *)calloc(1, sizeof *db);
    if (db == NULL) {
        sqlite3_close(raw);
        errno = ENOMEM;
        return NULL;
    }
    db->db = raw;

    struct { const char *sql; sqlite3_stmt **out; } stmts[] = {
        { INSERT_SQL,                &db->insert_stmt           },
        { UPDATE_OBSERVER_GAPS_SQL,  &db->update_gaps_stmt      },
        { UPDATE_OBSERVER_FORCE_SQL, &db->update_force_stmt     },
        { UPDATE_REPLAY_TS_SQL,      &db->update_replay_ts_stmt },
        { REGISTER_TLE_SQL,          &db->register_tle_stmt     },
        { SELECT_TLE_ID_SQL,         &db->select_tle_id_stmt    },
        { INSERT_SENT_TCMD_SQL,      &db->insert_sent_tcmd_stmt },
    };
    for (size_t i = 0; i < sizeof stmts / sizeof stmts[0]; i++) {
        if (sqlite3_prepare_v2(raw, stmts[i].sql, -1, stmts[i].out, NULL)
            != SQLITE_OK) {
            fprintf(stderr, "packet_db: prepare '%s' failed: %s\n",
                    stmts[i].sql, sqlite3_errmsg(raw));
            packet_db_close(db);
            errno = EIO;
            return NULL;
        }
    }
    return db;
}

static void bind_text_or_null(sqlite3_stmt *s, int idx, const char *txt)
{
    if (txt == NULL) sqlite3_bind_null(s, idx);
    else sqlite3_bind_text(s, idx, txt, -1, SQLITE_TRANSIENT);
}

static void bind_int_or_null(sqlite3_stmt *s, int idx, int v, int valid)
{
    if (valid) sqlite3_bind_int(s, idx, v);
    else sqlite3_bind_null(s, idx);
}

static void bind_double_or_null(sqlite3_stmt *s, int idx, double v)
{
    if (isnan(v)) sqlite3_bind_null(s, idx);
    else sqlite3_bind_double(s, idx, v);
}

int packet_db_insert(packet_db_t *db, const packet_db_record_t *rec)
{
    if (db == NULL || db->db == NULL || db->insert_stmt == NULL) return 0;
    if (rec == NULL || rec->ts_received == NULL
        || rec->source_tool == NULL || rec->payload == NULL
        || rec->packet_type_name == NULL) {
        return -1;
    }
    sqlite3_stmt *s = db->insert_stmt;

    // SHA1 of the raw payload, used for dedup. 20 bytes; stored as a
    // BLOB rather than hex so the index entries stay small.
    uint8_t sha[PACKET_DB_SHA1_LEN];
    sha1_digest(rec->payload, rec->payload_len, sha);

    sqlite3_reset(s);
    sqlite3_clear_bindings(s);

    bind_text_or_null(s,  1, rec->ts_received);
    bind_text_or_null(s,  2, rec->satellite);
    sqlite3_bind_int (s,  3, rec->packet_type);
    bind_text_or_null(s,  4, rec->packet_type_name);
    bind_int_or_null (s,  5, rec->csp_src,   rec->csp_present);
    bind_int_or_null (s,  6, rec->csp_dst,   rec->csp_present);
    bind_int_or_null (s,  7, rec->csp_dport, rec->csp_present);
    bind_int_or_null (s,  8, rec->csp_sport, rec->csp_present);
    bind_int_or_null (s,  9, rec->csp_prio,  rec->csp_present);
    bind_int_or_null (s, 10, rec->csp_flags, rec->csp_present);
    sqlite3_bind_blob(s, 11, rec->payload, (int)rec->payload_len,
                      SQLITE_TRANSIENT);
    sqlite3_bind_blob(s, 12, sha, sizeof sha, SQLITE_TRANSIENT);
    sqlite3_bind_int (s, 13, rec->golay_errs);
    sqlite3_bind_int (s, 14, rec->rs_errs);
    sqlite3_bind_int (s, 15, rec->hmac_ok);
    sqlite3_bind_int (s, 16, rec->crc_status);
    bind_text_or_null(s, 17, rec->source_tool);
    bind_text_or_null(s, 18, rec->source_run);
    bind_double_or_null(s, 19, rec->audio_offset_s);
    bind_text_or_null(s, 20, rec->decoded_summary);
    bind_double_or_null(s, 21, rec->az_deg);
    bind_double_or_null(s, 22, rec->el_deg);
    bind_double_or_null(s, 23, rec->range_km);
    bind_double_or_null(s, 24, rec->range_rate_km_s);
    bind_double_or_null(s, 25, rec->doppler_hz_offset);
    if (rec->tle_id > 0) sqlite3_bind_int64(s, 26, rec->tle_id);
    else sqlite3_bind_null(s, 26);
    bind_text_or_null(s, 27, rec->session_dir);
    bind_text_or_null(s, 28, rec->capture_origin);

    int rc = sqlite3_step(s);
    if (rc != SQLITE_DONE) {
        // INSERT OR IGNORE turns a duplicate into SQLITE_DONE, so reaching
        // here is a genuine failure: the row did NOT land. Distinguish
        // write-lock contention (retryable) from a hard error so the caller
        // — and the operator reading the log — can tell "try again" from
        // "something is broken".
        fprintf(stderr, "packet_db: insert failed: %s\n",
                sqlite3_errmsg(db->db));
        return (rc == SQLITE_BUSY || rc == SQLITE_LOCKED)
               ? PACKET_DB_INSERT_BUSY : PACKET_DB_INSERT_ERROR;
    }
    return PACKET_DB_INSERT_OK;
}

int packet_db_insert_sent_tcmd(packet_db_t *db, const sent_tcmd_record_t *rec)
{
    if (db == NULL || db->db == NULL || db->insert_sent_tcmd_stmt == NULL)
        return 0;
    if (rec == NULL || rec->command_text == NULL
        || rec->source_tool == NULL || rec->ts_transmitted == NULL) {
        return -1;
    }
    sqlite3_stmt *s = db->insert_sent_tcmd_stmt;
    sqlite3_reset(s);
    sqlite3_clear_bindings(s);

    sqlite3_bind_int64(s, 1, rec->ts_sent_ms);
    // tsexec_ms < 0 is the "not present" sentinel (real values are
    // positive unix-ms); store it as NULL.
    if (rec->tsexec_ms >= 0) sqlite3_bind_int64(s, 2, rec->tsexec_ms);
    else sqlite3_bind_null(s, 2);
    sqlite3_bind_text(s, 3, rec->command_text, -1, SQLITE_TRANSIENT);
    if (rec->tx_freq_hz > 0) sqlite3_bind_int64(s, 4, rec->tx_freq_hz);
    else sqlite3_bind_null(s, 4);
    bind_double_or_null(s, 5, rec->tx_gain_db);
    sqlite3_bind_text(s, 6, rec->source_tool, -1, SQLITE_TRANSIENT);
    bind_text_or_null(s, 7, rec->source_run);
    sqlite3_bind_text(s, 8, rec->ts_transmitted, -1, SQLITE_TRANSIENT);

    int rc = sqlite3_step(s);
    if (rc != SQLITE_DONE) {
        fprintf(stderr, "packet_db: sent_tcmd insert failed: %s\n",
                sqlite3_errmsg(db->db));
        return -1;
    }
    return 0;
}

void packet_db_close(packet_db_t *db)
{
    if (db == NULL) return;
    if (db->insert_stmt           != NULL) sqlite3_finalize(db->insert_stmt);
    if (db->update_gaps_stmt      != NULL) sqlite3_finalize(db->update_gaps_stmt);
    if (db->update_force_stmt     != NULL) sqlite3_finalize(db->update_force_stmt);
    if (db->update_replay_ts_stmt != NULL) sqlite3_finalize(db->update_replay_ts_stmt);
    if (db->register_tle_stmt     != NULL) sqlite3_finalize(db->register_tle_stmt);
    if (db->select_tle_id_stmt    != NULL) sqlite3_finalize(db->select_tle_id_stmt);
    if (db->insert_sent_tcmd_stmt != NULL) sqlite3_finalize(db->insert_sent_tcmd_stmt);
    if (db->db != NULL) sqlite3_close(db->db);
    free(db);
}

// Compute SHA1 of "line1\nline2" — the canonical TLE identity.
static void tle_sha1(const char *line1, const char *line2,
                     uint8_t out[PACKET_DB_SHA1_LEN])
{
    // Zero first so out is defined on any failure path (NULL ctx, etc.).
    memset(out, 0, PACKET_DB_SHA1_LEN);
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    if (ctx == NULL) return;
    if (EVP_DigestInit_ex(ctx, EVP_sha1(), NULL) == 1
        && EVP_DigestUpdate(ctx, line1, strlen(line1)) == 1
        && EVP_DigestUpdate(ctx, "\n", 1) == 1
        && EVP_DigestUpdate(ctx, line2, strlen(line2)) == 1) {
        EVP_DigestFinal_ex(ctx, out, NULL);
    }
    EVP_MD_CTX_free(ctx);
}

// Parse line1's catalog-number / epoch fields. TLE line 1 layout (NORAD):
//   col 0     '1'
//   col 2-6   5-digit catalog number (decimal)
//   col 18-19 epoch year (last two digits)
//   col 20-31 epoch day-of-year + fractional day
// All as ASCII text. Returns 0 on success, -1 if the line doesn't look
// like a TLE. The strlen >= 32 gate makes every fixed-offset read below
// in-bounds, so there's no overflow risk; a malformed line that still passes
// the gate is only a data-quality issue (garbage catalog/epoch), not unsafe.
static int parse_tle_line1(const char *line1,
                           int *out_catalog,
                           int *out_epoch_year,
                           double *out_epoch_day)
{
    if (line1 == NULL || strlen(line1) < 32 || line1[0] != '1') return -1;
    char buf[16];
    memcpy(buf, line1 + 2, 5); buf[5] = '\0';
    *out_catalog = atoi(buf);
    memcpy(buf, line1 + 18, 2); buf[2] = '\0';
    int yy = atoi(buf);
    *out_epoch_year = (yy < 57) ? 2000 + yy : 1900 + yy;
    memcpy(buf, line1 + 20, 12); buf[12] = '\0';
    *out_epoch_day = atof(buf);
    return 0;
}

long long packet_db_register_tle(packet_db_t *db,
                                 const char *satellite,
                                 const char *line1,
                                 const char *line2)
{
    if (db == NULL || db->register_tle_stmt == NULL || db->select_tle_id_stmt == NULL
        || satellite == NULL || line1 == NULL || line2 == NULL) {
        return 0;
    }
    uint8_t sha[PACKET_DB_SHA1_LEN];
    tle_sha1(line1, line2, sha);

    // Try to find existing row first — saves an INSERT round-trip in
    // the steady state where the same TLE is reused for many packets.
    sqlite3_reset(db->select_tle_id_stmt);
    sqlite3_clear_bindings(db->select_tle_id_stmt);
    sqlite3_bind_blob(db->select_tle_id_stmt, 1, sha, sizeof sha,
                      SQLITE_TRANSIENT);
    if (sqlite3_step(db->select_tle_id_stmt) == SQLITE_ROW) {
        long long id = sqlite3_column_int64(db->select_tle_id_stmt, 0);
        return id;
    }

    int catalog = 0, epoch_year = 0;
    double epoch_day = 0.0;
    int parsed = (parse_tle_line1(line1, &catalog, &epoch_year, &epoch_day) == 0);

    char ts_buf[40];
    time_t now = time(NULL);
    struct tm utc;
    gmtime_r(&now, &utc);
    strftime(ts_buf, sizeof ts_buf, "%Y-%m-%dT%H:%M:%SZ", &utc);

    sqlite3_stmt *s = db->register_tle_stmt;
    sqlite3_reset(s);
    sqlite3_clear_bindings(s);
    sqlite3_bind_text(s, 1, satellite, -1, SQLITE_TRANSIENT);
    if (parsed) sqlite3_bind_int(s, 2, catalog);   else sqlite3_bind_null(s, 2);
    if (parsed) sqlite3_bind_int(s, 3, epoch_year); else sqlite3_bind_null(s, 3);
    if (parsed) sqlite3_bind_double(s, 4, epoch_day); else sqlite3_bind_null(s, 4);
    sqlite3_bind_text(s, 5, line1, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 6, line2, -1, SQLITE_TRANSIENT);
    sqlite3_bind_blob(s, 7, sha, sizeof sha, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 8, ts_buf, -1, SQLITE_TRANSIENT);

    int step_rc = sqlite3_step(s);
    if (step_rc != SQLITE_DONE) {
        // Distinguish "the DB stayed contended past the busy_timeout" (a
        // transient we just lost the race for) from a genuine failure
        // (schema/constraint/IO). Both still return 0 = "no tle id", but the
        // log shouldn't make a busy DB look like corruption.
        if (step_rc == SQLITE_BUSY || step_rc == SQLITE_LOCKED) {
            fprintf(stderr, "packet_db: register_tle skipped: database busy past "
                    "the busy_timeout (%s)\n", sqlite3_errmsg(db->db));
        } else {
            fprintf(stderr, "packet_db: register_tle failed: %s\n",
                    sqlite3_errmsg(db->db));
        }
        return 0;
    }
    // INSERT OR IGNORE may have done nothing if a concurrent writer
    // beat us to it. Look up the row id either way.
    sqlite3_reset(db->select_tle_id_stmt);
    sqlite3_clear_bindings(db->select_tle_id_stmt);
    sqlite3_bind_blob(db->select_tle_id_stmt, 1, sha, sizeof sha,
                      SQLITE_TRANSIENT);
    if (sqlite3_step(db->select_tle_id_stmt) == SQLITE_ROW) {
        return sqlite3_column_int64(db->select_tle_id_stmt, 0);
    }
    return 0;
}

int packet_db_update_replay_ts(packet_db_t *db,
                               const uint8_t *payload, size_t payload_len,
                               const char *ts_iso,
                               double audio_offset_s)
{
    if (db == NULL || payload == NULL || ts_iso == NULL) return 0;
    sqlite3_stmt *s = db->update_replay_ts_stmt;
    if (s == NULL) return -1;

    uint8_t sha[PACKET_DB_SHA1_LEN];
    sha1_digest(payload, payload_len, sha);

    sqlite3_reset(s);
    sqlite3_clear_bindings(s);
    sqlite3_bind_text(s, 1, ts_iso, -1, SQLITE_TRANSIENT);
    bind_double_or_null(s, 2, audio_offset_s);
    sqlite3_bind_blob(s, 3, sha, sizeof sha, SQLITE_TRANSIENT);

    int rc = sqlite3_step(s);
    if (rc != SQLITE_DONE) {
        fprintf(stderr, "packet_db: update_replay_ts failed: %s\n",
                sqlite3_errmsg(db->db));
        return -1;
    }
    return sqlite3_changes(db->db);
}

int packet_db_update_observer(packet_db_t *db,
                              const uint8_t *payload, size_t payload_len,
                              double az_deg, double el_deg,
                              double range_km, double range_rate_km_s,
                              double doppler_hz_offset,
                              long long tle_id,
                              const char *session_dir,
                              int force)
{
    if (db == NULL || payload == NULL) return 0;
    sqlite3_stmt *s = force ? db->update_force_stmt : db->update_gaps_stmt;
    if (s == NULL) return -1;

    uint8_t sha[PACKET_DB_SHA1_LEN];
    sha1_digest(payload, payload_len, sha);

    sqlite3_reset(s);
    sqlite3_clear_bindings(s);
    bind_double_or_null(s, 1, az_deg);
    bind_double_or_null(s, 2, el_deg);
    bind_double_or_null(s, 3, range_km);
    bind_double_or_null(s, 4, range_rate_km_s);
    bind_double_or_null(s, 5, doppler_hz_offset);
    if (tle_id > 0) sqlite3_bind_int64(s, 6, tle_id);
    else sqlite3_bind_null(s, 6);
    bind_text_or_null(s, 7, session_dir);
    sqlite3_bind_blob(s, 8, sha, sizeof sha, SQLITE_TRANSIENT);

    int rc = sqlite3_step(s);
    if (rc != SQLITE_DONE) {
        fprintf(stderr, "packet_db: update_observer failed: %s\n",
                sqlite3_errmsg(db->db));
        return -1;
    }
    return sqlite3_changes(db->db);
}

#else // WITH_SQLITE3 not defined — stub mode

struct packet_db { int unused; };

packet_db_t *packet_db_open(const char *path)
{
    (void)path;
    return NULL;
}

int packet_db_insert(packet_db_t *db, const packet_db_record_t *rec)
{
    (void)db; (void)rec;
    return 0;
}

int packet_db_insert_sent_tcmd(packet_db_t *db, const sent_tcmd_record_t *rec)
{
    (void)db; (void)rec;
    return 0;
}

long long packet_db_register_tle(packet_db_t *db,
                                 const char *satellite,
                                 const char *line1,
                                 const char *line2)
{
    (void)db; (void)satellite; (void)line1; (void)line2;
    return 0;
}

int packet_db_update_observer(packet_db_t *db,
                              const uint8_t *payload, size_t payload_len,
                              double az_deg, double el_deg,
                              double range_km, double range_rate_km_s,
                              double doppler_hz_offset,
                              long long tle_id,
                              const char *session_dir,
                              int force)
{
    (void)db; (void)payload; (void)payload_len;
    (void)az_deg; (void)el_deg; (void)range_km;
    (void)range_rate_km_s; (void)doppler_hz_offset;
    (void)tle_id; (void)session_dir; (void)force;
    return 0;
}

int packet_db_update_replay_ts(packet_db_t *db,
                               const uint8_t *payload, size_t payload_len,
                               const char *ts_iso,
                               double audio_offset_s)
{
    (void)db; (void)payload; (void)payload_len;
    (void)ts_iso; (void)audio_offset_s;
    return 0;
}

void packet_db_close(packet_db_t *db)
{
    (void)db;
}

#endif // WITH_SQLITE3
