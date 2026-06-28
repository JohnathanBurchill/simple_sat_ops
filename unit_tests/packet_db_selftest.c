/*

    Simple Satellite Operations  unit_tests/packet_db_selftest.c

    Coverage for src/db/packet_db.c. The DB is the cross-pass cache of
    every decoded packet across rx_replay / rx_decode / simple_sat_ops;
    a regression in schema migration, insert validation, or dedup means
    rows silently stop landing OR start double-counting. Operator only
    notices when a query gives the wrong answer hours later.

    What's covered:
      - packet_db_open on a fresh file creates schema and reaches
        user_version = 4 (current latest after V1→V2→V3→V4 migrations).
      - re-open on the same file is idempotent (re-runs migrations
        without error).
      - open rejects NULL / "" paths.
      - insert validates required fields (ts_received, source_tool,
        payload, packet_type_name).
      - insert with a valid record lands one row.
      - insert is deduped by (payload_sha1, source_tool, capture_origin):
        second insert with identical key produces no extra row.
      - changing source_tool or capture_origin lets a "duplicate"
        payload land as a separate row (per-decoder, per-origin
        attribution).
      - register_tle is idempotent: same (line1, line2) → same id;
        different TLE → different id.
      - NaN doubles map to NULL in the DB.
      - csp_present = 0 stores CSP columns as NULL.
      - make_run_id produces a 16-character lower-case hex string + NUL.
      - close() on NULL is a no-op.
      - a failed insert reports a negative return (never 0), so a dropped
        row can't masquerade as success (issue #52 silent-loss contract).
      - batch mode buffers rows in memory until packet_db_flush: nothing is
        written before the flush, batch_pending tracks the count, and the
        single-transaction flush lands every buffered row (issue #52 fix).
      - concurrent batch writers (flush per file) lose no rows and dedup
        correctly — the same exact-count oracle as the per-row path.

    Exit status: 0 = all tests passed, non-zero = failure.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include "packet_db.h"
#include "tap.h"

#include <ctype.h>
#include <math.h>
#include <sqlite3.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

// Build a tmpfile path that we'll feed packet_db_open. mkstemp creates
// + opens a placeholder; close + unlink it so packet_db_open's
// sqlite3_open_v2 owns the path cleanly. The buffer must be at least
// strlen("/tmp/packet_db_selftest_XXXXXX") + 1.
static int make_tmp_db_path(char *out, size_t cap)
{
    snprintf(out, cap, "/tmp/packet_db_selftest_XXXXXX");
    int fd = mkstemp(out);
    if (fd < 0) return -1;
    close(fd);
    unlink(out);
    return 0;
}

// Helper: count rows in a table on an external sqlite3 connection so
// we can audit what packet_db wrote.
static long count_rows(sqlite3 *db, const char *sql)
{
    sqlite3_stmt *s = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &s, NULL) != SQLITE_OK) return -1;
    long n = -1;
    if (sqlite3_step(s) == SQLITE_ROW) {
        n = (long) sqlite3_column_int64(s, 0);
    }
    sqlite3_finalize(s);
    return n;
}

// Helper: read PRAGMA user_version off a raw sqlite3 handle.
static int read_user_version(sqlite3 *db)
{
    sqlite3_stmt *s = NULL;
    if (sqlite3_prepare_v2(db, "PRAGMA user_version;", -1, &s, NULL)
        != SQLITE_OK) return -1;
    int v = -1;
    if (sqlite3_step(s) == SQLITE_ROW) v = sqlite3_column_int(s, 0);
    sqlite3_finalize(s);
    return v;
}

// Construct a fully-populated record so insert tests don't trip on
// uninitialised fields. Caller can mutate a copy.
static packet_db_record_t make_record(const uint8_t *payload, size_t plen,
                                      const char *source_tool)
{
    packet_db_record_t r = {
        .ts_received       = "2026-05-18T19:00:00Z",
        .satellite         = "CTS1",
        .packet_type       = 0x01,
        .packet_type_name  = "beacon_basic",
        .csp_src           = 1,
        .csp_dst           = 10,
        .csp_dport         = 10,
        .csp_sport         = 10,
        .csp_prio          = 3,
        .csp_flags         = 0,
        .csp_present       = 1,
        .payload           = payload,
        .payload_len       = plen,
        .golay_errs        = 0,
        .rs_errs           = 0,
        .hmac_ok           = -1,
        .crc_status        = -1,
        .source_tool       = source_tool,
        .source_run        = "deadbeef01020304",
        .audio_offset_s    = 0.0,
        .decoded_summary   = "beacon CTS1 st=NOMINAL_TX",
        .az_deg            = 123.45,
        .el_deg            = 67.8,
        .range_km          = 1200.0,
        .range_rate_km_s   = -2.1,
        .doppler_hz_offset = 8412.0,
        .tle_id            = 0,
        .session_dir       = "/tmp/test",
        .capture_origin    = "cts_ground",
    };
    return r;
}

// ------------------------------------------------------------------
// 1. Open + user_version reaches the latest after migrations.
// ------------------------------------------------------------------

static void test_open_fresh_creates_schema(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    tap_ok(db != NULL, "open: fresh tmpfile returns non-NULL handle");
    packet_db_close(db);

    // Verify schema by opening with raw sqlite3 and checking the
    // packet table + user_version.
    sqlite3 *raw = NULL;
    int rc = sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    tap_okf(rc == SQLITE_OK, "raw open verifies path (rc=%d)", rc);

    long n_packet = count_rows(raw,
        "SELECT count(*) FROM sqlite_master WHERE type='table' AND name='packet';");
    tap_okf(n_packet == 1, "schema: `packet` table exists (got count=%ld)",
            n_packet);
    long n_tle = count_rows(raw,
        "SELECT count(*) FROM sqlite_master WHERE type='table' AND name='tle';");
    tap_okf(n_tle == 1, "schema: `tle` table exists (got count=%ld)", n_tle);
    long n_origin_idx = count_rows(raw,
        "SELECT count(*) FROM sqlite_master "
        "WHERE type='index' AND name='idx_packet_sha1_tool_origin';");
    tap_okf(n_origin_idx == 1,
            "schema: V4 unique index exists (got count=%ld)", n_origin_idx);

    int uv = read_user_version(raw);
    tap_okf(uv == 4, "user_version reached V4 after open (got %d)", uv);
    sqlite3_close(raw);
    unlink(path);
}

// ------------------------------------------------------------------
// 2. Open is idempotent — re-running on the same file is a no-op.
// ------------------------------------------------------------------

static void test_open_idempotent(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db1 = packet_db_open(path);
    tap_ok(db1 != NULL, "open: first open succeeds");
    packet_db_close(db1);

    packet_db_t *db2 = packet_db_open(path);
    tap_ok(db2 != NULL, "open: second open succeeds (migrations idempotent)");
    packet_db_close(db2);

    sqlite3 *raw = NULL;
    sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    int uv = read_user_version(raw);
    tap_okf(uv == 4, "re-open: user_version still 4 (got %d)", uv);
    sqlite3_close(raw);
    unlink(path);
}

// ------------------------------------------------------------------
// 3. Open rejects NULL / "" paths.
// ------------------------------------------------------------------

static void test_open_rejects_bad_path(void)
{
    tap_ok(packet_db_open(NULL) == NULL,
           "open: NULL path returns NULL");
    tap_ok(packet_db_open("") == NULL,
           "open: empty path returns NULL");
}

// ------------------------------------------------------------------
// 4. Insert validates required fields.
// ------------------------------------------------------------------

static void test_insert_validation(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    if (!db) { tap_bail("open"); return; }

    uint8_t payload[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    packet_db_record_t r = make_record(payload, sizeof payload, "selftest");

    packet_db_record_t r_no_ts = r; r_no_ts.ts_received = NULL;
    tap_ok(packet_db_insert(db, &r_no_ts) == -1,
           "insert: missing ts_received returns -1");

    packet_db_record_t r_no_tool = r; r_no_tool.source_tool = NULL;
    tap_ok(packet_db_insert(db, &r_no_tool) == -1,
           "insert: missing source_tool returns -1");

    packet_db_record_t r_no_payload = r; r_no_payload.payload = NULL;
    tap_ok(packet_db_insert(db, &r_no_payload) == -1,
           "insert: missing payload returns -1");

    packet_db_record_t r_no_typename = r; r_no_typename.packet_type_name = NULL;
    tap_ok(packet_db_insert(db, &r_no_typename) == -1,
           "insert: missing packet_type_name returns -1");

    tap_ok(packet_db_insert(NULL, &r) == 0,
           "insert: NULL db is a silent no-op (returns 0)");
    tap_ok(packet_db_insert(db, NULL) == -1,
           "insert: NULL record returns -1");

    packet_db_close(db);
    unlink(path);
}

// ------------------------------------------------------------------
// 5. Insert lands a row; dedup ignores the same key on repeat.
// ------------------------------------------------------------------

static void test_insert_and_dedup(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    if (!db) { tap_bail("open"); return; }

    uint8_t payload[16];
    for (size_t i = 0; i < sizeof payload; ++i) payload[i] = (uint8_t) i;
    packet_db_record_t r = make_record(payload, sizeof payload, "selftest");

    tap_ok(packet_db_insert(db, &r) == 0,
           "insert: valid record returns 0");

    // Re-insert same (payload_sha1, source_tool, capture_origin) — must
    // be silently deduped (returns 0, no extra row).
    tap_ok(packet_db_insert(db, &r) == 0,
           "insert: duplicate key returns 0 (silent dedup)");

    packet_db_close(db);

    // Verify exactly one row.
    sqlite3 *raw = NULL;
    sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    long n = count_rows(raw, "SELECT count(*) FROM packet;");
    tap_okf(n == 1, "dedup: exactly 1 packet row (got %ld)", n);
    sqlite3_close(raw);
    unlink(path);
}

// ------------------------------------------------------------------
// 6. Different source_tool / capture_origin breaks the dedup tuple.
// ------------------------------------------------------------------

static void test_insert_tuple_attribution(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    if (!db) { tap_bail("open"); return; }

    // The packet table has TWO unique constraints in play after V4:
    //   - CREATE TABLE UNIQUE (payload_sha1, source_tool, source_run)
    //   - V4 index UNIQUE   (payload_sha1, source_tool, capture_origin)
    // A row only lands when it differs from every existing row on
    // BOTH tuples. The practical "different decoder OR different
    // capture origin OR different run" expectation thus needs each
    // record to vary source_run too — which is how real runs
    // naturally differ, every process has its own random run id.
    uint8_t payload[8] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11};
    packet_db_record_t r1 = make_record(payload, sizeof payload, "selftest");
    packet_db_record_t r2 = make_record(payload, sizeof payload, "rx_replay");
    r2.source_run = "rrrrrrrrrrrrrrrr";
    packet_db_record_t r3 = make_record(payload, sizeof payload, "selftest");
    r3.capture_origin = "satnogs";
    r3.source_run = "ssssssssssssssss";

    tap_ok(packet_db_insert(db, &r1) == 0, "tuple: tool=selftest");
    tap_ok(packet_db_insert(db, &r2) == 0, "tuple: tool=rx_replay");
    tap_ok(packet_db_insert(db, &r3) == 0, "tuple: origin=satnogs");

    packet_db_close(db);

    sqlite3 *raw = NULL;
    sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    long n = count_rows(raw, "SELECT count(*) FROM packet;");
    tap_okf(n == 3,
            "tuple: 3 distinct (tool, run, origin) tuples → 3 rows (got %ld)",
            n);
    sqlite3_close(raw);
    unlink(path);
}

// ------------------------------------------------------------------
// 7. csp_present=0 stores CSP columns as NULL.
// ------------------------------------------------------------------

static void test_insert_csp_absent_null(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    if (!db) { tap_bail("open"); return; }

    uint8_t payload[4] = {0x10, 0x20, 0x30, 0x40};
    packet_db_record_t r = make_record(payload, sizeof payload, "selftest");
    r.csp_present = 0;
    // Sentinel values should be IGNORED when csp_present=0.
    r.csp_src = 99; r.csp_dst = 99;
    tap_ok(packet_db_insert(db, &r) == 0, "csp_absent: insert returns 0");

    packet_db_close(db);

    sqlite3 *raw = NULL;
    sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    long n_null = count_rows(raw,
        "SELECT count(*) FROM packet WHERE csp_src IS NULL AND csp_dst IS NULL;");
    tap_okf(n_null == 1,
            "csp_absent: csp_src + csp_dst stored as NULL (got %ld)",
            n_null);
    sqlite3_close(raw);
    unlink(path);
}

// ------------------------------------------------------------------
// 8. NaN observer doubles map to NULL.
// ------------------------------------------------------------------

static void test_insert_nan_observer_null(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    if (!db) { tap_bail("open"); return; }

    uint8_t payload[4] = {0x11, 0x22, 0x33, 0x44};
    packet_db_record_t r = make_record(payload, sizeof payload, "selftest");
    r.az_deg = NAN;
    r.el_deg = NAN;
    r.range_km = NAN;
    r.range_rate_km_s = NAN;
    r.doppler_hz_offset = NAN;
    tap_ok(packet_db_insert(db, &r) == 0, "nan: insert with NaN doubles");

    packet_db_close(db);

    sqlite3 *raw = NULL;
    sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    long n_null = count_rows(raw,
        "SELECT count(*) FROM packet WHERE az_deg IS NULL "
        "AND el_deg IS NULL AND range_km IS NULL "
        "AND range_rate_km_s IS NULL AND doppler_hz_offset IS NULL;");
    tap_okf(n_null == 1,
            "nan: all 5 NaN doubles stored as NULL (got %ld)", n_null);
    sqlite3_close(raw);
    unlink(path);
}

// ------------------------------------------------------------------
// 9. register_tle is idempotent on (line1, line2).
// ------------------------------------------------------------------

static void test_register_tle_idempotent(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    if (!db) { tap_bail("open"); return; }

    // Real-looking ISS TLE — content doesn't have to validate, just be
    // the same bytes both times.
    const char *line1 = "1 25544U 98067A   24001.50000000  .00010000  00000-0  18000-3 0  9991";
    const char *line2 = "2 25544  51.6400 100.0000 0007000 100.0000 260.0000 15.50000000999991";
    long long id1 = packet_db_register_tle(db, "ISS",        line1, line2);
    long long id2 = packet_db_register_tle(db, "ISS-rename", line1, line2);
    tap_okf(id1 > 0, "register_tle: first call returns id=%lld > 0", id1);
    tap_okf(id1 == id2,
            "register_tle: same TLE → same id (%lld == %lld)", id1, id2);

    // Different line1 (catalog number tweaked) → new id.
    const char *line1b = "1 12345U 98067A   24001.50000000  .00010000  00000-0  18000-3 0  9991";
    long long id3 = packet_db_register_tle(db, "OTHER", line1b, line2);
    tap_okf(id3 > 0 && id3 != id1,
            "register_tle: different TLE → different id (%lld vs %lld)",
            id3, id1);

    // NULL satellite / lines → 0.
    tap_ok(packet_db_register_tle(db, NULL, line1, line2) == 0,
           "register_tle: NULL satellite returns 0");
    tap_ok(packet_db_register_tle(db, "ISS", NULL, line2) == 0,
           "register_tle: NULL line1 returns 0");

    packet_db_close(db);

    sqlite3 *raw = NULL;
    sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    long n = count_rows(raw, "SELECT count(*) FROM tle;");
    tap_okf(n == 2, "register_tle: 2 distinct TLE rows total (got %ld)", n);
    sqlite3_close(raw);
    unlink(path);
}

// ------------------------------------------------------------------
// 10. make_run_id format + buffer guard.
// ------------------------------------------------------------------

static void test_make_run_id(void)
{
    char buf[64];
    memset(buf, 'X', sizeof buf);
    int rc = packet_db_make_run_id(buf, sizeof buf);
    tap_okf(rc == 0, "run_id: rc=%d", rc);
    size_t len = strlen(buf);
    tap_okf(len == 16, "run_id: length = 16 (got %zu)", len);
    int all_hex = 1;
    for (size_t i = 0; i < 16; ++i) {
        char c = buf[i];
        int ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
        if (!ok) all_hex = 0;
    }
    tap_okf(all_hex == 1, "run_id: all chars lower-case hex (\"%s\")", buf);
    tap_okf(buf[16] == '\0',
            "run_id: NUL-terminated at position 16 (got 0x%02x)",
            (unsigned char) buf[16]);

    // Short buffer rejected.
    char tiny[8];
    tap_ok(packet_db_make_run_id(tiny, sizeof tiny) == -1,
           "run_id: cap < 17 returns -1");
    tap_ok(packet_db_make_run_id(NULL, 64) == -1,
           "run_id: NULL buffer returns -1");
}

// ------------------------------------------------------------------
// 11. close(NULL) is safe.
// ------------------------------------------------------------------

static void test_close_null_safe(void)
{
    packet_db_close(NULL);
    tap_ok(1, "close: NULL pointer no-op survives");
}

// ------------------------------------------------------------------
// 12. Parallel writers. The DB advertises WAL + busy_timeout + INSERT OR
//     IGNORE so several processes can write the same file at once without
//     loss or duplicate rows. Nothing exercised that. Fork a handful of
//     writers that contend on one DB: each hammers a SHARED record set
//     (identical across children -> must dedup to one row each, however
//     the inserts interleave) plus a per-child UNIQUE set (distinct
//     payloads -> all must survive). The final row counts don't depend on
//     the interleaving, so they're a deterministic oracle for "no lost
//     write, no false dedup" under real contention.
// ------------------------------------------------------------------
#define PW_CHILDREN 4
#define PW_SHARED   8
#define PW_UNIQUE   16

// Runs in a forked child: open an independent connection and insert.
// Returns 0 on success, non-zero on any open/insert failure. No TAP here
// (only the parent owns the TAP stream).
static int parallel_child_work(const char *path, int child_idx)
{
    packet_db_t *db = packet_db_open(path);
    if (db == NULL) {
        return 1;
    }
    // Shared set: identical for every child, inserted twice each to pile
    // on contention and exercise repeat-dedup. All collapse to PW_SHARED.
    for (int rep = 0; rep < 2; ++rep) {
        for (int k = 0; k < PW_SHARED; ++k) {
            uint8_t pl[3] = { 0xAA, (uint8_t)k, 0x00 };
            packet_db_record_t r = make_record(pl, sizeof pl, "pw-shared");
            if (packet_db_insert(db, &r) != 0) {
                packet_db_close(db);
                return 2;
            }
        }
    }
    // Unique set: payload carries the child index, so no child's payloads
    // collide with another's. All PW_CHILDREN*PW_UNIQUE must survive.
    for (int j = 0; j < PW_UNIQUE; ++j) {
        uint8_t pl[3] = { 0xBB, (uint8_t)child_idx, (uint8_t)j };
        packet_db_record_t r = make_record(pl, sizeof pl, "pw-unique");
        if (packet_db_insert(db, &r) != 0) {
            packet_db_close(db);
            return 3;
        }
    }
    packet_db_close(db);
    return 0;
}

typedef struct {
    const char *path;
    int idx;
    int rc;
} pw_thread_arg_t;

static void *parallel_writer_thread(void *p)
{
    pw_thread_arg_t *a = (pw_thread_arg_t *)p;
    a->rc = parallel_child_work(a->path, a->idx);
    return NULL;
}

static void test_parallel_writers(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    // Create the schema once up front so the writers only contend on
    // INSERTs (the property under test), not on first-time migration.
    packet_db_t *seed = packet_db_open(path);
    if (seed == NULL) {
        tap_bail("seed open"); unlink(path); return;
    }
    packet_db_close(seed);

    // Separate connections from separate threads contend on the WAL write
    // lock exactly as separate processes do -- SQLite's locking is per
    // connection, not per process -- so this drives the same busy_timeout +
    // INSERT OR IGNORE path the multi-process claim relies on, without the
    // fork()-without-exec abort sqlite triggers on macOS.
    pthread_t th[PW_CHILDREN];
    pw_thread_arg_t args[PW_CHILDREN];
    int spawned = 0;
    for (int c = 0; c < PW_CHILDREN; ++c) {
        args[c].path = path;
        args[c].idx  = c;
        args[c].rc   = -1;
        if (pthread_create(&th[c], NULL, parallel_writer_thread, &args[c]) != 0) {
            break;
        }
        spawned++;
    }
    for (int i = 0; i < spawned; ++i) {
        pthread_join(th[i], NULL);
    }
    tap_okf(spawned == PW_CHILDREN, "spawned %d concurrent writers (want %d)",
            spawned, PW_CHILDREN);

    int all_ok = 1, worst = 0;
    for (int i = 0; i < spawned; ++i) {
        if (args[i].rc != 0) {
            all_ok = 0;
            if (args[i].rc > worst) worst = args[i].rc;
        }
    }
    tap_okf(all_ok,
            "every concurrent insert returned success (busy_timeout absorbed "
            "contention; worst rc=%d)", worst);

    sqlite3 *raw = NULL;
    int rc = sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    if (rc != SQLITE_OK) {
        tap_okf(0, "raw open after parallel writes (rc=%d)", rc);
        unlink(path); return;
    }
    long shared = count_rows(raw,
        "SELECT count(*) FROM packet WHERE source_tool='pw-shared';");
    long uniq = count_rows(raw,
        "SELECT count(*) FROM packet WHERE source_tool='pw-unique';");
    long total = count_rows(raw, "SELECT count(*) FROM packet;");
    sqlite3_close(raw);

    tap_okf(shared == PW_SHARED,
            "shared records deduped under contention: %ld rows (want %d)",
            shared, PW_SHARED);
    tap_okf(uniq == (long)PW_CHILDREN * PW_UNIQUE,
            "every writer's unique records survived: %ld rows (want %d)",
            uniq, PW_CHILDREN * PW_UNIQUE);
    tap_okf(total == PW_SHARED + (long)PW_CHILDREN * PW_UNIQUE,
            "total rows exact, no lost write: %ld (want %d)",
            total, PW_SHARED + PW_CHILDREN * PW_UNIQUE);

    unlink(path);
    char side[80];
    snprintf(side, sizeof side, "%s-wal", path); unlink(side);
    snprintf(side, sizeof side, "%s-shm", path); unlink(side);
}

// ------------------------------------------------------------------
// 13. A failed insert is reported as a NEGATIVE return, never 0. This is
//     the contract decode_loop relies on to know a row was dropped: the
//     silent-loss bug (issue #52) survived because a dropped insert was
//     indistinguishable from success at this boundary, so the caller marked
//     the capture "done" and lost the packets. Force a deterministic step
//     failure by dropping the `packet` table out from under the prepared
//     statement via a second connection, then assert the insert returns < 0
//     (not PACKET_DB_INSERT_OK). The earlier success keeps the assertion
//     honest: it must flip from >= 0 to < 0, not be < 0 unconditionally.
// ------------------------------------------------------------------

static void test_insert_reports_failure(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    if (!db) { tap_bail("open"); unlink(path); return; }

    uint8_t payload[8] = {0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc};
    packet_db_record_t r = make_record(payload, sizeof payload, "selftest");
    tap_okf(packet_db_insert(db, &r) >= 0,
            "fail-report: healthy insert reports success (>= 0)");

    // Drop the table from an independent connection. The next step on
    // packet_db's prepared INSERT then fails ("no such table") — a
    // deterministic stand-in for any DB-level insert failure (contention,
    // I/O, schema change).
    sqlite3 *raw = NULL;
    int orc = sqlite3_open_v2(path, &raw, SQLITE_OPEN_READWRITE, NULL);
    if (orc != SQLITE_OK) {
        tap_bail("raw rw open"); packet_db_close(db); unlink(path); return;
    }
    sqlite3_busy_timeout(raw, 60000);
    int drc = sqlite3_exec(raw, "DROP TABLE packet;", NULL, NULL, NULL);
    tap_okf(drc == SQLITE_OK, "fail-report: dropped packet table (rc=%d)", drc);
    sqlite3_close(raw);

    // A different payload so this isn't a dedup no-op: the table is gone, so
    // the insert must fail and report a negative code (one stderr
    // "insert failed: no such table" line below is expected).
    uint8_t payload2[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    packet_db_record_t r2 = make_record(payload2, sizeof payload2, "selftest");
    int frc = packet_db_insert(db, &r2);
    tap_okf(frc < 0,
            "fail-report: insert against a broken DB returns < 0, not 0 (got %d)",
            frc);

    packet_db_close(db);
    unlink(path);
    char side[80];
    snprintf(side, sizeof side, "%s-wal", path); unlink(side);
    snprintf(side, sizeof side, "%s-shm", path); unlink(side);
}

// ------------------------------------------------------------------
// 14. Batch mode buffers until flush, then writes the whole file in one
//     transaction. This is the issue #52 fix: collapse one write-lock
//     acquisition per row into one per file so parallel decoders stop
//     dropping rows on contention. Oracle: nothing is in the DB before
//     flush (a second connection sees 0 rows), batch_pending tracks the
//     buffered count, and after flush every row is present. A mutation
//     that writes immediately (batch ignored) trips the pre-flush 0-rows
//     check; one that skips the write trips the post-flush count.
// ------------------------------------------------------------------

static void test_batch_buffers_until_flush(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    packet_db_t *db = packet_db_open(path);
    if (!db) { tap_bail("open"); unlink(path); return; }

    packet_db_set_batch(db, 1);

    enum { N = 5 };
    for (int i = 0; i < N; ++i) {
        uint8_t pl[3] = { 0xC0, (uint8_t)i, 0x00 };
        packet_db_record_t r = make_record(pl, sizeof pl, "batch");
        tap_okf(packet_db_insert(db, &r) == 0,
                "batch: insert %d buffers (returns 0)", i);
    }
    tap_okf(packet_db_batch_pending(db) == (size_t)N,
            "batch: %d rows pending before flush (got %zu)",
            N, packet_db_batch_pending(db));

    // Nothing should be in the DB yet — batch mode doesn't touch SQLite
    // until flush. Read via an independent connection.
    {
        sqlite3 *raw = NULL;
        sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
        long n = count_rows(raw, "SELECT count(*) FROM packet;");
        tap_okf(n == 0, "batch: 0 rows written before flush (got %ld)", n);
        sqlite3_close(raw);
    }

    int frc = packet_db_flush(db);
    tap_okf(frc == 0, "batch: flush returns 0 (got %d)", frc);
    tap_okf(packet_db_batch_pending(db) == 0,
            "batch: 0 rows pending after flush (got %zu)",
            packet_db_batch_pending(db));

    // A second flush with nothing buffered is a no-op success.
    tap_okf(packet_db_flush(db) == 0, "batch: empty flush is a no-op (0)");

    packet_db_close(db);

    sqlite3 *raw = NULL;
    sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    long n = count_rows(raw, "SELECT count(*) FROM packet;");
    tap_okf(n == N, "batch: all %d rows present after flush (got %ld)", N, n);
    sqlite3_close(raw);
    unlink(path);
    char side[80];
    snprintf(side, sizeof side, "%s-wal", path); unlink(side);
    snprintf(side, sizeof side, "%s-shm", path); unlink(side);
}

// ------------------------------------------------------------------
// 15. Concurrent batch writers. Mirror of test_parallel_writers but each
//     writer buffers and flushes once (the offline rx_replay path under
//     decode_passes --jobs N). Same exact-count oracle: shared records
//     dedup to PW_SHARED, every writer's unique records survive, no lost
//     write. Proves the batched flush is loss-free and dedup-correct under
//     real WAL contention — the property issue #52's fix must hold.
// ------------------------------------------------------------------

static int parallel_child_work_batch(const char *path, int child_idx)
{
    packet_db_t *db = packet_db_open(path);
    if (db == NULL) return 1;
    packet_db_set_batch(db, 1);
    // Shared set: identical for every child, buffered twice each. Within a
    // child's single flush transaction the repeat dedups; across children
    // the unique index dedups. All collapse to PW_SHARED.
    for (int rep = 0; rep < 2; ++rep) {
        for (int k = 0; k < PW_SHARED; ++k) {
            uint8_t pl[3] = { 0xAA, (uint8_t)k, 0x00 };
            packet_db_record_t r = make_record(pl, sizeof pl, "pwb-shared");
            if (packet_db_insert(db, &r) != 0) { packet_db_close(db); return 2; }
        }
    }
    // Unique set: payload carries the child index, so none collide.
    for (int j = 0; j < PW_UNIQUE; ++j) {
        uint8_t pl[3] = { 0xBB, (uint8_t)child_idx, (uint8_t)j };
        packet_db_record_t r = make_record(pl, sizeof pl, "pwb-unique");
        if (packet_db_insert(db, &r) != 0) { packet_db_close(db); return 3; }
    }
    // Until here nothing is written; this is the one contended lock grab.
    int frc = packet_db_flush(db);
    packet_db_close(db);
    return frc == 0 ? 0 : 4;
}

static void *parallel_writer_thread_batch(void *p)
{
    pw_thread_arg_t *a = (pw_thread_arg_t *)p;
    a->rc = parallel_child_work_batch(a->path, a->idx);
    return NULL;
}

static void test_batch_parallel_writers(void)
{
    char path[64];
    if (make_tmp_db_path(path, sizeof path) != 0) {
        tap_bail("mkstemp"); return;
    }
    // Create the schema once so writers only contend on the flush, not on
    // first-time migration.
    packet_db_t *seed = packet_db_open(path);
    if (seed == NULL) {
        tap_bail("seed open"); unlink(path); return;
    }
    packet_db_close(seed);

    pthread_t th[PW_CHILDREN];
    pw_thread_arg_t args[PW_CHILDREN];
    int spawned = 0;
    for (int c = 0; c < PW_CHILDREN; ++c) {
        args[c].path = path;
        args[c].idx  = c;
        args[c].rc   = -1;
        if (pthread_create(&th[c], NULL, parallel_writer_thread_batch,
                           &args[c]) != 0) {
            break;
        }
        spawned++;
    }
    for (int i = 0; i < spawned; ++i) {
        pthread_join(th[i], NULL);
    }
    tap_okf(spawned == PW_CHILDREN,
            "batch-parallel: spawned %d writers (want %d)",
            spawned, PW_CHILDREN);

    int all_ok = 1, worst = 0;
    for (int i = 0; i < spawned; ++i) {
        if (args[i].rc != 0) {
            all_ok = 0;
            if (args[i].rc > worst) worst = args[i].rc;
        }
    }
    tap_okf(all_ok,
            "batch-parallel: every writer's flush succeeded (worst rc=%d)",
            worst);

    sqlite3 *raw = NULL;
    int rc = sqlite3_open_v2(path, &raw, SQLITE_OPEN_READONLY, NULL);
    if (rc != SQLITE_OK) {
        tap_okf(0, "batch-parallel: raw open (rc=%d)", rc);
        unlink(path); return;
    }
    long shared = count_rows(raw,
        "SELECT count(*) FROM packet WHERE source_tool='pwb-shared';");
    long uniq = count_rows(raw,
        "SELECT count(*) FROM packet WHERE source_tool='pwb-unique';");
    long total = count_rows(raw, "SELECT count(*) FROM packet;");
    sqlite3_close(raw);

    tap_okf(shared == PW_SHARED,
            "batch-parallel: shared deduped: %ld rows (want %d)",
            shared, PW_SHARED);
    tap_okf(uniq == (long)PW_CHILDREN * PW_UNIQUE,
            "batch-parallel: every unique record survived: %ld (want %d)",
            uniq, PW_CHILDREN * PW_UNIQUE);
    tap_okf(total == PW_SHARED + (long)PW_CHILDREN * PW_UNIQUE,
            "batch-parallel: total exact, no lost write: %ld (want %d)",
            total, PW_SHARED + PW_CHILDREN * PW_UNIQUE);

    unlink(path);
    char side[80];
    snprintf(side, sizeof side, "%s-wal", path); unlink(side);
    snprintf(side, sizeof side, "%s-shm", path); unlink(side);
}

int main(void)
{
    test_open_fresh_creates_schema();
    test_open_idempotent();
    test_open_rejects_bad_path();
    test_insert_validation();
    test_insert_and_dedup();
    test_insert_tuple_attribution();
    test_insert_csp_absent_null();
    test_insert_nan_observer_null();
    test_register_tle_idempotent();
    test_make_run_id();
    test_close_null_safe();
    test_parallel_writers();
    test_insert_reports_failure();
    test_batch_buffers_until_flush();
    test_batch_parallel_writers();
    return tap_done();
}
