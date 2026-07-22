/*

    Simple Satellite Operations  utils/mpi_reconstruct.c

    Rebuild an MPI science-data file from the bulk_file packets in the packet
    DB, recovering as much as possible and reporting the byte ranges that never
    arrived. This is the MPI counterpart to cam_reconstruct (boom-camera JPEGs)
    and gnss_reports/gnss_opm (the GNSS firehose): all three reassemble one
    downlinked file from the packets sitting in the store.

    The flight firmware (CTS-SAT-1, tag sat-1-rc3) records MPI science data to a
    file on flash. The operator names the file (mpi_enable_active_mode(<name>),
    usually with a timestamp), the MPI streams fixed data frames into it, and
    the file is later pulled to the ground with comms_bulk_file_downlink. Each
    MPI data frame is a run of raw bytes that begins with the four sync bytes
    0x0C 0xFF 0xFF 0x0C (see MPI_dataframe_t / MPI_validate_science_data_file in
    the firmware); a valid file is at least 20000 bytes and carries the sync
    word about once every ~158 bytes. Unlike the camera file there is no text
    wrapper -- the downloaded bytes ARE the science data -- so reconstruction is
    purely reassembly, with no on-ground reformatting.

    comms_bulk_file_downlink streams the flash file in fixed-size chunks (195
    file bytes per RF packet) tagged with a byte offset. Every received chunk
    lands in the packet DB as a bulk_file packet (packet_type 16) whose payload
    is [packet_type:1][file_offset:4 LE][data...]. simple_sat_ops does not know
    which flash file a chunk came from -- only its byte offset -- so this tool
    groups chunks into one download the same way packet_browser's Enter does:
    by a contiguous burst in TIME, not by decode run. Parallel decoders split a
    single RF pass into several source_runs, each catching a different subset of
    the same chunks, so scoping by run would show a sibling run's catches as
    holes. The burst model below (mirrored from packet_browser.c) keeps two
    time-adjacent chunks together only while the gap between them is consistent
    with the firmware streaming the bytes between their offsets at 9600 baud.

    What this tool does:
      1. Loads every bulk_file chunk with a sane file offset, splits them into
         download bursts by time, and reassembles each burst by offset (a byte
         is a gap until a chunk fills it; RS-clean chunks win, uncorrectable
         chunks only fill still-missing bytes).
      2. For each burst that looks like MPI science data (>= 20000 bytes and the
         0C FF FF 0C sync word seen repeatedly), writes MPI_yyyymmddThhmmss.bin
         to the output directory (the CWD by default). The timestamp is the UTC
         reception time of the burst's earliest chunk. Still-missing bytes are
         left as a fill value (0x00 by default) so the file stays byte-aligned.
      3. Reports which file byte ranges are still missing, snapped out to the
         195-byte download-chunk boundaries, and prints ready-to-use
         comms_bulk_file_downlink_start commands to re-fetch them. Feed those
         re-fetched chunks back into the DB and re-run to close the gaps.

    Read-only on the DB -- safe to run while a receiver is filling it.

    Usage:
      mpi_reconstruct [--db=<packet_db.sqlite>] [--since=<spec>] [--until=<spec>]
                      [--dir=<out-dir>] [--file-path=<sat-path>]
                      [--fill=00|ff] [--min-sync=<n>] [--quiet]

    With no --db the default store is used ($SSO_PACKET_DB, else the FrontierSat
    root's packet_db.sqlite). --since/--until scope the search (24h | 7d | 30m |
    ISO-8601). --file-path sets the on-satellite path printed in the re-download
    commands (the ground file is offset-addressed, so the flash name is a
    "<file_path>" placeholder unless given).

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

#include "argparse.h"
#include "gnss_frag.h"
#include "packet_db.h"
#include "sso_version.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef WITH_SQLITE3
int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "mpi_reconstruct")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "mpi_reconstruct: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

// Bulk-file packet geometry (mirrors packet_browser.c / beacon_cts1.h). A
// bulk_file payload is [packet_type:1][file_offset:4 LE][data...]; the firmware
// sends at most 195 file bytes per packet, so a lost byte range is always a
// whole multiple of the chunk size, and the ground appends a 4-byte CSP CRC32
// trailer we drop by clamping the data to 195.
#define BULK_FILE_PACKET_TYPE      16
#define BULK_FILE_HEADER_SIZE      5
#define BULK_FILE_MAX_DATA         195
#define BULK_FILE_MAX_PLAUSIBLE    (2 * 1024 * 1024)   // firmware caps a download at 1 MB

// A single comms_bulk_file_downlink telecommand can re-fetch at most this much
// contiguous data: the firmware clamps max_bytes to
// COMMS_bulk_file_downlink_max_allowable_total_bytes (1,000,000) in
// bulk_file_downlink.c. The re-download report merges the missing chunks into
// the fewest commands that stay within this span.
#define COMMS_BULK_DOWNLINK_MAX_BYTES  1000000L

// MPI science-data heuristics (mirror MPI_validate_science_data_file). A valid
// file is at least 20000 bytes and carries the 4-byte sync word once per frame
// (~every 152 bytes). Gaps drop the count, so require only a handful to call a
// download MPI -- enough to tell it apart from a camera or ASCII log file that
// never carries this pattern.
#define MPI_MIN_FILE_BYTES         20000
#define MPI_SYNC_WORD              "\x0C\xFF\xFF\x0C"
#define MPI_SYNC_WORD_LEN          4
#define MPI_DEFAULT_MIN_SYNC       8

// One MPI file is pulled over many attempts, sometimes across several passes.
// The chunks that actually carry the sync word positively identify the file;
// chunks streamed alongside them (in the same download) often fall between sync
// words and carry none, so we scope a download by TIME around the sync-bearing
// chunks. Sync-bearing chunks more than this far apart belong to different
// downloads (a later pass of another file); each cluster is reconstructed on
// its own. The window is padded by a margin so a download's sync-free lead-in
// and tail-out chunks are still swept in.
#define MPI_SESSION_GAP_MS         (15.0 * 60.0 * 1000.0)   // 15 minutes
#define MPI_SESSION_MARGIN_MS      (120.0 * 1000.0)         // 2 minutes

typedef struct {
    double   ts_ms;    // reception time, unix ms
    long     off;      // file offset (sane: 0 .. BULK_FILE_MAX_PLAUSIBLE)
    int      rs;       // rs_errs: >=0 clean/corrected, -2 uncorrectable
    long     dlen;     // data bytes (clamped to BULK_FILE_MAX_DATA)
    int      has_sync; // this chunk's own bytes contain the MPI sync word
    uint8_t *data;     // owned copy of the chunk's data bytes
    char     ts_iso[32];
} chunk_t;

typedef struct {
    const char *db_path;
    const char *since;
    const char *until;
    const char *out_dir;
    const char *sat_file_path;
    uint8_t     fill;
    int         min_sync;
    long        max_dl;   // per-command re-download cap (firmware's max_bytes)
    int         quiet;
} args_t;

#define OPTW 22

static int parse_args(args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (gnss_starts_with(arg, "--db=") || help) {
            if (help) parse_help_line(OPTW, "--db=<path>", "override default DB path ($SSO_PACKET_DB or the default)");
            else a->db_path = arg + 5;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--since=") || help) {
            if (help) parse_help_line(OPTW, "--since=<spec>", "24h | 7d | 30m | ISO-8601 (default: all)");
            else a->since = arg + 8;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--until=") || help) {
            if (help) parse_help_line(OPTW, "--until=<spec>", "same syntax as --since (default: now)");
            else a->until = arg + 8;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--dir=") || help) {
            if (help) parse_help_line(OPTW, "--dir=<path>", "output directory for the .bin (default: .)");
            else a->out_dir = arg + 6;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--file-path=") || help) {
            if (help) parse_help_line(OPTW, "--file-path=<p>", "on-satellite path for re-download commands");
            else a->sat_file_path = arg + 12;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--fill=") || help) {
            if (help) parse_help_line(OPTW, "--fill=00|ff", "byte for still-missing data (default: 00)");
            else {
                const char *v = arg + 7;
                char *end = NULL;
                long b = (strlen(v) == 2) ? strtol(v, &end, 16) : -1;
                if (b < 0 || b > 255 || (end && *end)) {
                    fprintf(stderr, "mpi_reconstruct: --fill must be two hex digits (e.g. 00 or ff)\n");
                    return PARSE_ERROR;
                }
                a->fill = (uint8_t) b;
            }
            matched = 1;
        }
        if (gnss_starts_with(arg, "--min-sync=") || help) {
            if (help) parse_help_line(OPTW, "--min-sync=<n>", "min sync words to call a burst MPI (default 8)");
            else a->min_sync = (int) strtol(arg + 11, NULL, 10);
            matched = 1;
        }
        if (gnss_starts_with(arg, "--max-download=") || help) {
            if (help) parse_help_line(OPTW, "--max-download=<n>", "per-command re-download cap in bytes (default 1000000)");
            else {
                long v = strtol(arg + 15, NULL, 0);
                if (v < BULK_FILE_MAX_DATA) {
                    fprintf(stderr, "mpi_reconstruct: --max-download must be >= %d\n", BULK_FILE_MAX_DATA);
                    return PARSE_ERROR;
                }
                a->max_dl = v;
            }
            matched = 1;
        }
        if (strcmp(arg, "--quiet") == 0 || help) {
            if (help) parse_help_line(OPTW, "--quiet", "only print the files written");
            else a->quiet = 1;
            matched = 1;
        }
        if ((strcmp(arg, "-V") == 0 || strcmp(arg, "--version") == 0) || help) {
            if (help) parse_help_line(OPTW, "-V, --version", "print version and exit");
            // -V is handled in main via sso_version_handle before parsing.
            matched = 1;
        }
        if (!help && !matched) {
            fprintf(stderr, "mpi_reconstruct: unknown option '%s' (try --help)\n", arg);
            return PARSE_ERROR;
        }
    }
    return help ? PARSE_HELP : PARSE_OK;
}

// Little-endian uint32 file offset out of a bulk_file payload.
static long bulk_offset(const uint8_t *pl)
{
    return (long) pl[1] | ((long) pl[2] << 8)
         | ((long) pl[3] << 16) | ((long) pl[4] << 24);
}

// Load every bulk_file chunk with a sane offset (optionally within the
// since/until window), oldest first. Returns a malloc'd array in *out and its
// length; -1 on error.
static int load_chunks(sqlite3 *db, const char *since_iso, const char *until_iso,
                       chunk_t **out, int *out_n)
{
    // ts_ms as unix ms, same expression packet_browser uses for burst timing.
    const char *sql =
        "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, "
        "       ts_received, payload, rs_errs "
        "FROM packet WHERE packet_type=16 "
        "  AND (?1 = '' OR ts_received >= ?1) "
        "  AND (?2 = '' OR ts_received <= ?2) "
        "ORDER BY ts_received, id";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "mpi_reconstruct: query failed: %s\n", sqlite3_errmsg(db));
        return -1;
    }
    sqlite3_bind_text(st, 1, since_iso ? since_iso : "", -1, SQLITE_STATIC);
    sqlite3_bind_text(st, 2, until_iso ? until_iso : "", -1, SQLITE_STATIC);

    chunk_t *cs = NULL;
    int n = 0, cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const uint8_t *pl = (const uint8_t *) sqlite3_column_blob(st, 2);
        int pl_len = sqlite3_column_bytes(st, 2);
        if (pl == NULL || pl_len < BULK_FILE_HEADER_SIZE + 1) continue;
        long off = bulk_offset(pl);
        if (off < 0 || off > BULK_FILE_MAX_PLAUSIBLE) continue;   // bit-flipped offset

        long dlen = pl_len - BULK_FILE_HEADER_SIZE;
        if (dlen > BULK_FILE_MAX_DATA) dlen = BULK_FILE_MAX_DATA;  // drop CSP CRC32 trailer
        if (off + dlen > BULK_FILE_MAX_PLAUSIBLE) continue;

        if (n == cap) {
            int ncap = cap ? cap * 2 : 256;
            chunk_t *t = (chunk_t *) realloc(cs, (size_t) ncap * sizeof *cs);
            if (t == NULL) { fprintf(stderr, "out of memory\n"); goto fail; }
            cs = t; cap = ncap;
        }
        chunk_t *c = &cs[n];
        memset(c, 0, sizeof *c);
        c->ts_ms = sqlite3_column_double(st, 0);
        c->off   = off;
        c->rs    = sqlite3_column_int(st, 3);
        c->dlen  = dlen;
        c->data  = (uint8_t *) malloc((size_t) dlen);
        if (c->data == NULL) { fprintf(stderr, "out of memory\n"); goto fail; }
        memcpy(c->data, pl + BULK_FILE_HEADER_SIZE, (size_t) dlen);
        // Does this chunk's own bytes carry the MPI sync word? Used to find the
        // download that is MPI and scope it in time (see the session loop).
        c->has_sync = 0;
        for (long k = 0; k + MPI_SYNC_WORD_LEN <= dlen; k++)
            if (memcmp(c->data + k, MPI_SYNC_WORD, MPI_SYNC_WORD_LEN) == 0) { c->has_sync = 1; break; }
        const char *ts = (const char *) sqlite3_column_text(st, 1);
        snprintf(c->ts_iso, sizeof c->ts_iso, "%s", ts ? ts : "");
        n++;
    }
    sqlite3_finalize(st);
    *out = cs;
    *out_n = n;
    return 0;

fail:
    for (int i = 0; i < n; i++) free(cs[i].data);
    free(cs);
    sqlite3_finalize(st);
    return -1;
}

// Reassemble the chunks named by idx[0..nidx) by absolute file offset into a
// freshly allocated buffer. Bulk offsets are absolute file positions, so chunks
// from different passes of the same download merge here regardless of when they
// arrived. Every byte starts as a gap; RS-clean chunks are placed first
// (phase 0) and uncorrectable chunks (rs_errs == -2) fill only bytes still
// missing (phase 1), so good data is never clobbered. present[] marks filled
// bytes. *out_min_off is the lowest offset that carried data (start of the
// received region). *out_conflicts counts bytes where two RS-clean chunks
// disagreed -- a sign the window mixes more than one file. Returns the buffer
// size (file span), or -1 on error; *out_buf / *out_present are owned by the
// caller.
static long reassemble(const chunk_t *cs, const int *idx, int nidx, uint8_t fill,
                       uint8_t **out_buf, uint8_t **out_present, long *out_min_off,
                       long *out_conflicts)
{
    long size = 0, min_off = -1;
    for (int j = 0; j < nidx; j++) {
        const chunk_t *c = &cs[idx[j]];
        if (c->off + c->dlen > size) size = c->off + c->dlen;
    }
    if (size <= 0) return -1;

    uint8_t *buf     = (uint8_t *) malloc((size_t) size);
    uint8_t *present = (uint8_t *) calloc((size_t) size, 1);
    if (buf == NULL || present == NULL) { free(buf); free(present); return -1; }
    memset(buf, fill, (size_t) size);

    long conflicts = 0;
    for (int phase = 0; phase < 2; phase++) {
        for (int j = 0; j < nidx; j++) {
            const chunk_t *c = &cs[idx[j]];
            int clean = (c->rs >= 0);              // rs_errs == -2 is uncorrectable
            if ((phase == 0) != clean) continue;    // phase 0 clean, phase 1 the rest
            long off = c->off;
            for (long k = 0; k < c->dlen && off + k < size; k++) {
                if (present[off + k]) {
                    // Two RS-clean chunks landing different bytes here means the
                    // window holds more than one file at this offset.
                    if (phase == 0 && buf[off + k] != c->data[k]) conflicts++;
                    if (phase == 1) continue;        // never clobber good data
                }
                buf[off + k] = c->data[k];
                present[off + k] = 1;
            }
            if (min_off < 0 || off < min_off) min_off = off;
        }
    }
    if (min_off < 0) min_off = 0;

    *out_buf = buf;
    *out_present = present;
    *out_min_off = min_off;
    *out_conflicts = conflicts;
    return size;
}

// Count non-overlapping MPI sync words in buf.
static long count_sync(const uint8_t *buf, long size)
{
    long n = 0;
    for (long i = 0; i + MPI_SYNC_WORD_LEN <= size; ) {
        if (memcmp(buf + i, MPI_SYNC_WORD, MPI_SYNC_WORD_LEN) == 0) {
            n++;
            i += MPI_SYNC_WORD_LEN;
        } else {
            i++;
        }
    }
    return n;
}

// "YYYYMMDDTHHMMSS" from an ISO ts_received ("2026-07-21T20:39:57.745Z"),
// keeping the 'T'. Falls back to "unknown" if fewer than 14 digits are present.
static void stamp_from_iso(const char *iso, char *out, size_t outn)
{
    char d[15] = "";   // 14 digits YYYYMMDDHHMMSS
    int nd = 0;
    for (const char *s = iso; *s && nd < 14; s++)
        if (*s >= '0' && *s <= '9') d[nd++] = *s;
    if (nd >= 14)
        snprintf(out, outn, "%.8sT%.6s", d, d + 8);
    else
        snprintf(out, outn, "unknown");
}

// Write buf to <dir>/MPI_<stamp>.bin; store the path in out_path. Returns 0 on
// success, -1 on error.
static int write_mpi_file(const char *dir, const char *stamp,
                          const uint8_t *buf, long size,
                          char *out_path, size_t out_path_n)
{
    snprintf(out_path, out_path_n, "%s/MPI_%s.bin",
             (dir && dir[0]) ? dir : ".", stamp);
    FILE *f = fopen(out_path, "wb");
    if (f == NULL) { perror(out_path); return -1; }
    if (size > 0 && fwrite(buf, 1, (size_t) size, f) != (size_t) size) {
        perror("fwrite"); fclose(f); return -1;
    }
    fclose(f);
    return 0;
}

// Emit the request that covers [cs, ce), tiled so no single command exceeds the
// firmware's per-command cap. `tile` is a whole number of 195-byte packets, so
// every command's length is a multiple of 195 -- except the one that reaches
// the file's end, whose last packet is genuinely partial. ce is clamped to the
// reconstructed file size. Accumulates the byte and command totals.
static void emit_run(const char *sat_file_path, long cs, long ce, long size,
                     long tile, int quiet, long *download, long *n_cmds)
{
    if (ce > size) ce = size;
    for (long p = cs; p < ce; p += tile) {
        long q = (p + tile < ce) ? p + tile : ce;
        if (!quiet)
            printf("    offset %8ld  len %7ld   "
                   "comms_bulk_file_downlink_start(%s,%ld,%ld)\n",
                   p, q - p, sat_file_path, p, q - p);
        *download += q - p;
        (*n_cmds)++;
    }
}

// Report the missing data as re-download telecommands. The firmware transmits
// whole 195-byte packets and a chunk is received or not (CRC pass/fail), so
// every request asks for a whole number of packets: a command starts at the
// first missing byte and its length rounds up to a multiple of 195. Missing
// runs whose whole-packet requests would touch or overlap are coalesced into
// one command (this only re-fetches present bytes that share a packet with a
// gap -- unavoidable, since a partial packet can't be requested); a run of
// fully-received packets between two gaps is left alone. A request wider than
// the firmware's per-command cap (max_dl) is tiled into back-to-back commands.
// Note the merged chunks come from passes on different 195-byte grids, so the
// gaps -- and thus the request offsets -- need not sit on any single grid.
static void report_gaps(const uint8_t *present, long size,
                        const char *sat_file_path, long max_dl, int quiet)
{
    const long PKT = BULK_FILE_MAX_DATA;         // 195 bytes per downlink packet
    long tile = (max_dl / PKT) * PKT;            // per-command cap, whole packets
    if (tile < PKT) tile = PKT;

    if (!quiet) printf("  re-download (fewest telecommands, whole 195-byte packets):\n");
    long n_cmds = 0, missing = 0, download = 0;
    long run_start = -1;
    long cs = -1, ce = -1;   // pending request [cs, ce); ce - cs is a multiple of 195

    // Iterate one past the end so a gap that runs to EOF is closed.
    for (long i = 0; i <= size; i++) {
        int gap = (i < size) && !present[i];
        if (gap && run_start < 0) {
            run_start = i;
        } else if (!gap && run_start >= 0) {
            long a = run_start, b = i;
            missing += b - a;
            long want_ce = a + ((b - a + PKT - 1) / PKT) * PKT;   // a + ceil(len/195)*195
            if (cs < 0) {
                cs = a; ce = want_ce;
            } else if (a <= ce) {
                // This gap starts inside the pending request's packets: extend
                // to cover it, keeping the length a whole number of packets.
                long ext = cs + ((b - cs + PKT - 1) / PKT) * PKT;
                if (ext > ce) ce = ext;
            } else {
                emit_run(sat_file_path, cs, ce, size, tile, quiet, &download, &n_cmds);
                cs = a; ce = want_ce;
            }
            run_start = -1;
        }
    }
    if (cs >= 0)
        emit_run(sat_file_path, cs, ce, size, tile, quiet, &download, &n_cmds);

    if (!quiet) {
        if (n_cmds == 0) printf("    (none -- file is complete)\n");
        printf("  missing     : %ld bytes; re-download %ld bytes in %ld telecommand%s\n",
               missing, download, n_cmds, n_cmds == 1 ? "" : "s");
    }
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "mpi_reconstruct")) return 0;

    args_t cfg = {0};
    cfg.out_dir = ".";
    cfg.sat_file_path = "<file_path>";
    cfg.fill = 0x00;
    cfg.min_sync = MPI_DEFAULT_MIN_SYNC;
    cfg.max_dl = COMMS_BULK_DOWNLINK_MAX_BYTES;
    switch (parse_args(&cfg, argc, argv, 0)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
        default: break;
    }

    char db_default[1024];
    const char *db_path = cfg.db_path;
    if (db_path == NULL) {
        if (packet_db_default_path(db_default, sizeof db_default) != 0) {
            fprintf(stderr, "mpi_reconstruct: no DB path "
                            "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = db_default;
    }

    char since_iso[40] = {0}, until_iso[40] = {0};
    if (cfg.since && gnss_parse_time_spec(cfg.since, since_iso, sizeof since_iso) != 0) {
        fprintf(stderr, "mpi_reconstruct: bad --since=%s\n", cfg.since); return 1;
    }
    if (cfg.until && gnss_parse_time_spec(cfg.until, until_iso, sizeof until_iso) != 0) {
        fprintf(stderr, "mpi_reconstruct: bad --until=%s\n", cfg.until); return 1;
    }

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "mpi_reconstruct: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 1;
    }

    chunk_t *cs = NULL;
    int n = 0;
    if (load_chunks(db, since_iso, until_iso, &cs, &n) != 0) {
        sqlite3_close(db);
        return 1;
    }
    sqlite3_close(db);

    if (n == 0) {
        fprintf(stderr, "mpi_reconstruct: no bulk_file packets found%s.\n",
                (cfg.since || cfg.until) ? " in that time window" : "");
        free(cs);
        return 1;
    }

    // Reusable index buffer for the chunks gathered into one session's window.
    int *idx = (int *) malloc((size_t) n * sizeof *idx);
    if (idx == NULL) {
        fprintf(stderr, "out of memory\n");
        for (int i = 0; i < n; i++) free(cs[i].data);
        free(cs);
        return 1;
    }

    // Walk the time-ordered chunks and cluster the sync-bearing ones into
    // download sessions (a gap over MPI_SESSION_GAP_MS between two sync-bearing
    // chunks starts a new session -- a later download of another file). Each
    // session is one MPI file; its chunks may still span several passes, all
    // merged by absolute offset. Chunks belonging to other files (a camera or
    // ASCII log pulled at a different time) carry no sync word and fall outside
    // every session window, so they are never mixed in.
    int rc = 0, n_files = 0;
    int i = 0;
    while (i < n) {
        if (!cs[i].has_sync) { i++; continue; }

        double first_sync = cs[i].ts_ms, last_sync = cs[i].ts_ms;
        int last_i = i;
        for (int k = i + 1; k < n; k++) {
            if (!cs[k].has_sync) continue;
            if (cs[k].ts_ms - last_sync > MPI_SESSION_GAP_MS) break;
            last_sync = cs[k].ts_ms; last_i = k;
        }

        // Gather every chunk within the sync span padded by the margin, so the
        // download's sync-free lead-in and tail-out chunks come along too.
        double wlo = first_sync - MPI_SESSION_MARGIN_MS;
        double whi = last_sync + MPI_SESSION_MARGIN_MS;
        int nidx = 0, first = -1, last = -1;
        for (int k = 0; k < n; k++) {
            if (cs[k].ts_ms < wlo || cs[k].ts_ms > whi) continue;
            idx[nidx++] = k;
            if (first < 0) first = k;
            last = k;
        }
        i = last_i + 1;   // resume after this session's sync chunks

        uint8_t *buf = NULL, *present = NULL;
        long min_off = 0, conflicts = 0;
        long size = reassemble(cs, idx, nidx, cfg.fill, &buf, &present, &min_off, &conflicts);
        long sync = (size > 0) ? count_sync(buf, size) : 0;
        if (size < MPI_MIN_FILE_BYTES || sync < cfg.min_sync) {
            free(buf); free(present);
            continue;   // this cluster does not hold enough MPI to be a file
        }

        long present_bytes = 0;
        for (long b = 0; b < size; b++) if (present[b]) present_bytes++;

        char stamp[32];
        stamp_from_iso(cs[first].ts_iso, stamp, sizeof stamp);
        char out_path[1200];
        if (write_mpi_file(cfg.out_dir, stamp, buf, size, out_path, sizeof out_path) != 0) {
            rc = 1;
        } else if (cfg.quiet) {
            printf("%s\n", out_path);
            n_files++;
        } else {
            printf("MPI file    : %s\n", out_path);
            printf("  received  : %.24s .. %.24s, %d packets, %ld sync words\n",
                   cs[first].ts_iso, cs[last].ts_iso, nidx, sync);
            printf("  file      : %ld bytes (offset 0 .. %ld)\n", size, size);
            printf("  recovered : %ld of %ld bytes (%.1f%%); data spans offset %ld .. %ld\n",
                   present_bytes, size,
                   100.0 * (double) present_bytes / (double) size, min_off, size);
            if (conflicts > 0)
                printf("  warning   : %ld byte(s) where two clean chunks disagree -- this window\n"
                       "              may mix more than one file; scope it with --since/--until\n",
                       conflicts);
            report_gaps(present, size, cfg.sat_file_path, cfg.max_dl, cfg.quiet);
            printf("\n");
            n_files++;
        }
        free(buf); free(present);
    }

    free(idx);
    for (int i2 = 0; i2 < n; i2++) free(cs[i2].data);
    free(cs);

    if (n_files == 0) {
        fprintf(stderr, "mpi_reconstruct: no MPI science-data download found%s.\n",
                (cfg.since || cfg.until) ? " in that time window" : "");
        return 1;
    }
    return rc;
}

#endif // WITH_SQLITE3
