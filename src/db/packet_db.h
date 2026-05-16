/*

    Simple Satellite Operations  packet_db.h

    Append-only SQLite store for decoded AX100 packets. Every receiver
    (rx_live, rx_replay, b210_rx_tx, rx_decode) writes one row per
    decoded packet into a project-wide DB so cross-pass queries are
    possible without re-parsing per-session log files.

    The DB has one table, `packet`, with the raw payload bytes plus the
    pre-rendered firmware-interpreted body so LIKE queries against the
    text work without a binary decode step.

    Default DB path: $SSO_PACKET_DB if set, else
    $HOME/.local/share/simple_sat_ops/packets.db. Each receiver may
    override per-run via --db=<path> or skip writes entirely with --no-db.

    When the build doesn't have sqlite3 available, the implementation
    falls back to a no-op stub so existing receivers still link and run
    on stripped-down hosts. packet_db_open returns NULL in that mode and
    insert/close are no-ops.

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

#ifndef PACKET_DB_H
#define PACKET_DB_H

#include <stddef.h>
#include <stdint.h>

typedef struct packet_db packet_db_t;

// One decoded packet, ready to insert. Caller fills in what it has and
// leaves unknowns as NULL / sentinel values:
//   - csp_present == 0 means the CSP fields aren't valid; columns will
//     be stored as NULL.
//   - golay_errs / rs_errs / hmac_ok / crc_status follow the same
//     sentinels emit_frame uses (-1 = off / not checked, -2 = RS
//     uncorrectable, 0/1 = mismatch/ok). Stored as-is so queries can
//     filter on them directly.
//   - audio_offset_s is NaN when not from a file replay; stored as NULL
//     in that case.
//   - decoded_summary is the firmware-interpreted body (multi-line
//     text from beacon_print / tcmd_response_print / etc). NULL when
//     the dispatcher didn't recognise the packet.
typedef struct {
    const char *ts_received;
    const char *satellite;
    int         packet_type;
    const char *packet_type_name;
    int csp_src, csp_dst, csp_dport, csp_sport, csp_prio, csp_flags;
    int csp_present;
    const uint8_t *payload;
    size_t        payload_len;
    int golay_errs, rs_errs, hmac_ok, crc_status;
    const char *source_tool;
    const char *source_run;
    double      audio_offset_s;
    const char *decoded_summary;
    // Observer-frame state. NaN means "not known"; that maps to NULL
    // in the DB. Populated by b210_rx_tx (live) or rx_replay (when
    // run with --tle for backfill); other receivers leave them NaN.
    double      az_deg;
    double      el_deg;
    double      range_km;
    double      range_rate_km_s;
    double      doppler_hz_offset;
    // tle_id from packet_db_register_tle. 0 means "not known".
    long long   tle_id;
    // Run cwd / WAV directory. NULL means not known.
    const char *session_dir;
    // Provenance of the source audio: "cts_ground" for our B210
    // capture, "satnogs" for a SatNOGS-archive .ogg, NULL when
    // unknown (legacy rows / no flag supplied). Distinct from
    // source_tool, which identifies the decoder.
    const char *capture_origin;
} packet_db_record_t;

// Open the DB at `path`, creating it (with schema) if missing. Sets WAL
// journaling so concurrent readers/writers don't block each other.
// Returns NULL on failure (e.g. sqlite3 not available in this build, or
// the file couldn't be opened); errno is set on real I/O errors.
packet_db_t *packet_db_open(const char *path);

// Insert one row. Silently ignores duplicates (rows whose
// payload_sha1 + source_tool + source_run already exist) so re-runs of
// the same capture don't double-count. Returns 0 on success or silent
// dedup, -1 on real DB errors.
int packet_db_insert(packet_db_t *db, const packet_db_record_t *rec);

// Register a TLE (3-line: name, line1, line2) and return its row id.
// Idempotent — same TLE bytes produce the same id across runs (UNIQUE
// constraint on the SHA1 of the canonical "line1\nline2" form). The
// catalog number, epoch year, and epoch day are parsed out of line1
// for queryability; if parsing fails the row still gets stored with
// NULLs in those columns. Returns >0 (the id) on success, 0 on
// failure.
long long packet_db_register_tle(packet_db_t *db,
                                 const char *satellite,
                                 const char *line1,
                                 const char *line2);

// Backfill the observer / tle / session_dir columns on rows whose
// payload matches the supplied bytes (matched by SHA1). When `force`
// is 0, only NULL columns are updated (the default rx_replay --update
// behaviour: fill the gaps, don't trample). When `force` is non-zero,
// every column is overwritten. NaN inputs map to NULL the same way
// packet_db_insert handles them. tle_id == 0 means "leave as-is" in
// gaps mode; in force mode it sets the column to NULL. Returns the
// number of rows updated, or -1 on real DB errors.
int packet_db_update_observer(packet_db_t *db,
                              const uint8_t *payload, size_t payload_len,
                              double az_deg, double el_deg,
                              double range_km, double range_rate_km_s,
                              double doppler_hz_offset,
                              long long tle_id,
                              const char *session_dir,
                              int force);

// Rewrite ts_received and audio_offset_s on rows whose payload SHA1
// matches AND source_tool = 'rx_replay'. Used by rx_replay --update
// to fix the stale wall-clock-of-decode timestamp that prior runs
// stamped before the audio-clock anchor existed; the new value is
// the actual transmission UTC (anchor + offset) supplied here.
// Scoped to rx_replay rows so live-decode rows (whose ts_received is
// already correct) aren't trampled. Returns rows updated, -1 on
// error.
int packet_db_update_replay_ts(packet_db_t *db,
                               const uint8_t *payload, size_t payload_len,
                               const char *ts_iso,
                               double audio_offset_s);

void packet_db_close(packet_db_t *db);

// Resolve the default DB path into `buf`. Order of preference:
//   1. $SSO_PACKET_DB if set and non-empty.
//   2. $HOME/.local/share/simple_sat_ops/packets.db (parent dir created
//      if missing).
// Returns 0 on success, -1 if neither source could yield a path. The
// returned path is used as the default when a receiver doesn't pass an
// explicit --db= flag.
int packet_db_default_path(char *buf, size_t cap);

// Generate a random hex run-id (e.g. "a1b2c3d4e5f60718") into `buf`.
// Used to stamp the rows from one process invocation so dedup can tell
// "same packet, same run" from "same packet decoded twice in different
// runs". `buf` should be at least 17 bytes (16 hex + NUL). Returns 0
// on success or -1 if /dev/urandom couldn't be read.
int packet_db_make_run_id(char *buf, size_t cap);

// One-call helper for receivers. Picks the DB path (user_path if
// non-NULL, else the default from packet_db_default_path), opens it,
// and generates a run-id into run_id_out. When no_db is non-zero the
// helper short-circuits and returns NULL — the caller still passes
// the result to decode_loop_set_packet_db, which leaves the tap
// inactive. Logs to stderr on real open failures. Caller is
// responsible for packet_db_close on the returned handle (a no-op on
// NULL). run_id_out must be at least 17 bytes.
packet_db_t *packet_db_setup(const char *user_path,
                             int no_db,
                             char *run_id_out, size_t run_id_cap);

#endif // PACKET_DB_H
