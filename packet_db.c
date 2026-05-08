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

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
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

// mkdir -p behaviour for one path. Ignores EEXIST. Used to make the
// parent of the default DB path on first ever open.
static int mkdir_p(const char *path, mode_t mode)
{
    if (path == NULL || path[0] == '\0') return -1;
    char buf[1024];
    size_t len = strlen(path);
    if (len >= sizeof buf) return -1;
    memcpy(buf, path, len + 1);
    for (size_t i = 1; i < len; ++i) {
        if (buf[i] != '/') continue;
        buf[i] = '\0';
        if (mkdir(buf, mode) != 0 && errno != EEXIST) return -1;
        buf[i] = '/';
    }
    if (mkdir(buf, mode) != 0 && errno != EEXIST) return -1;
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
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') return -1;
    char dir[1024];
    int rc = snprintf(dir, sizeof dir,
                      "%s/.local/share/simple_sat_ops", home);
    if (rc <= 0 || (size_t)rc >= sizeof dir) return -1;
    if (mkdir_p(dir, 0755) != 0) return -1;
    rc = snprintf(buf, cap, "%s/packets.db", dir);
    if (rc <= 0 || (size_t)rc >= cap) return -1;
    return 0;
}

#ifdef WITH_SQLITE3

#include <openssl/sha.h>
#include <sqlite3.h>

struct packet_db {
    sqlite3      *db;
    sqlite3_stmt *insert_stmt;
};

static const char SCHEMA_SQL[] =
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
    "  hmac_ok         INTEGER,\n"
    "  crc_status      INTEGER,\n"
    "  source_tool     TEXT NOT NULL,\n"
    "  source_run      TEXT,\n"
    "  audio_offset_s  REAL,\n"
    "  decoded_summary TEXT,\n"
    "  UNIQUE (payload_sha1, source_tool, source_run)\n"
    ");\n"
    "CREATE INDEX IF NOT EXISTS idx_packet_ts        ON packet(ts_received);\n"
    "CREATE INDEX IF NOT EXISTS idx_packet_type      ON packet(packet_type);\n"
    "CREATE INDEX IF NOT EXISTS idx_packet_satellite ON packet(satellite);\n"
    "PRAGMA user_version = 1;\n";

static const char INSERT_SQL[] =
    "INSERT OR IGNORE INTO packet ("
    "  ts_received, satellite, packet_type, packet_type_name,"
    "  csp_src, csp_dst, csp_dport, csp_sport, csp_prio, csp_flags,"
    "  payload, payload_sha1,"
    "  golay_errs, rs_errs, hmac_ok, crc_status,"
    "  source_tool, source_run, audio_offset_s, decoded_summary"
    ") VALUES ("
    "  ?1, ?2, ?3, ?4,"
    "  ?5, ?6, ?7, ?8, ?9, ?10,"
    "  ?11, ?12,"
    "  ?13, ?14, ?15, ?16,"
    "  ?17, ?18, ?19, ?20"
    ");";

packet_db_t *packet_db_open(const char *path)
{
    if (path == NULL || path[0] == '\0') return NULL;
    sqlite3 *raw = NULL;
    int rc = sqlite3_open_v2(path, &raw,
                             SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE,
                             NULL);
    if (rc != SQLITE_OK) {
        if (raw != NULL) sqlite3_close(raw);
        errno = EIO;
        return NULL;
    }
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
    if (sqlite3_exec(raw, SCHEMA_SQL, NULL, NULL, &errmsg) != SQLITE_OK) {
        fprintf(stderr, "packet_db: schema apply failed: %s\n",
                errmsg ? errmsg : "(unknown)");
        sqlite3_free(errmsg);
        sqlite3_close(raw);
        errno = EIO;
        return NULL;
    }

    packet_db_t *db = (packet_db_t *)calloc(1, sizeof *db);
    if (db == NULL) {
        sqlite3_close(raw);
        errno = ENOMEM;
        return NULL;
    }
    db->db = raw;
    if (sqlite3_prepare_v2(raw, INSERT_SQL, sizeof INSERT_SQL,
                           &db->insert_stmt, NULL) != SQLITE_OK) {
        fprintf(stderr, "packet_db: prepare failed: %s\n",
                sqlite3_errmsg(raw));
        sqlite3_close(raw);
        free(db);
        errno = EIO;
        return NULL;
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
    uint8_t sha[SHA_DIGEST_LENGTH];
    SHA1(rec->payload, rec->payload_len, sha);

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
    if (isnan(rec->audio_offset_s)) sqlite3_bind_null(s, 19);
    else sqlite3_bind_double(s, 19, rec->audio_offset_s);
    bind_text_or_null(s, 20, rec->decoded_summary);

    int rc = sqlite3_step(s);
    if (rc != SQLITE_DONE) {
        fprintf(stderr, "packet_db: insert failed: %s\n",
                sqlite3_errmsg(db->db));
        return -1;
    }
    return 0;
}

void packet_db_close(packet_db_t *db)
{
    if (db == NULL) return;
    if (db->insert_stmt != NULL) sqlite3_finalize(db->insert_stmt);
    if (db->db != NULL) sqlite3_close(db->db);
    free(db);
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

void packet_db_close(packet_db_t *db)
{
    (void)db;
}

#endif // WITH_SQLITE3
