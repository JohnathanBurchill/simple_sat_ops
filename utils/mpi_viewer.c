/*

    Simple Satellite Operations  utils/mpi_viewer.c

    A small raylib GUI for inspecting MPI science imagery straight out of the
    packet DB. The left panel lists MPI *experiments* -- one row per time the MPI
    was run and recorded (a distinct science-data file on the satellite), not per
    download pass. The MPI has been run only a handful of times this mission, but
    each recording is downlinked over many passes; this tool merges all of an
    experiment's passes back into one stream.

    Grouping (experiments, not downloads): the flight firmware writes JSON markers
    into each science file -- a {"mpi_start":...,"timestamp_ms":N} header at
    offset 0 and a {"uptime_ms":...,"timestamp_ms":N} footer after every
    20480-byte buffer flush. Those markers survive into the bulk_file packets. We
    cluster download bursts into experiments by the embedded marker time (all
    markers of one recording fall within its <=15-minute window; different
    experiments are days apart), then reassemble every contributing burst by
    absolute file offset into one stream. Reassembly by offset also deletes
    duplicates: the same byte range re-downloaded on a later pass just overwrites
    the same offsets.

    Capture times (not download times): each footer's timestamp_ms is the wall
    time that ~20 KB block finished filling on the satellite. We locate every
    marker in the reassembled stream, build an (offset -> time) curve, and assign
    each frame a capture time by interpolating along it. An image's time is the
    time of its first frame.

    Frame finding: the reconstructed byte stream is NOT a clean fixed 152-byte
    grid -- packets from passes on different offset grids leave the frame
    alignment drifting, so a global grid catches only the handful of frames whose
    sync happens to land on it. Instead we locate EVERY occurrence of the sync
    word 0C FF FF 0C; each one starts a 152-byte frame wherever it sits.

    Artifact removal: the 20480-byte flush is not frame-aligned, so once per
    ~20 KB a ~150-byte JSON footer is spliced into the middle of a frame. That
    frame's sync word is still found, but its 152 bytes read JSON text plus
    shifted pixels -- a garbage row. We drop any frame whose body overlaps a
    marker blob (~1 frame per 134), which removes the routine periodic artifact.

    Each frame carries aux/housekeeping fields then a strip of image pixels. One
    full inner-dome voltage sweep is one image; a new sweep begins each time the
    scan index (aux byte 13) returns to a multiple of the scan period (auto-
    detected, 16). Within an image a frame's column is its inner-dome target
    voltage (the commanded bias): the 0 V background frame (setpoint 65535, no
    ion signal on the CCD) sits at the far right and increasingly negative bias
    voltages stack to the left, most negative at the far left. The tool plays the
    images back like a movie, grayscale, with an adjustable DN range and
    nearest-neighbour zoom, and shows the aux data beside each image.

    Aux frame layout (authoritative, from the MPI storeTelemetryAuxData packer):
        0..3   sync word 0C FF FF 0C
        4..5   frame counter                 (big-endian uint16)
        6..7   temperature (raw ADC)         (big-endian uint16)
        8      firmware version              (uint8)
        9..10  camera board CCD ADC          (big-endian uint16)
        11..12 inner dome target voltage     (big-endian uint16)
        13     inner dome scan index         (uint8) -- the image row
        14..15 inner dome ADC reading        (big-endian uint16)
        16     first pixel index             (uint8)
        17     last pixel index              (uint8)
        18..19 integration period            (big-endian uint16)
        22..151 pixels, 65 big-endian uint16 (per the First Light notebook)

    Data coverage: under the aux panel sits a map of the whole reconstructed
    file -- byte 0 at the top left, the last byte at the bottom right, reading
    left to right then down. Green is a byte that came down, black one that
    never did. Each pixel stands for a slice of the file and goes black as soon
    as any byte in its slice is missing, so a single lost packet still shows.

    Re-download export: press d to write the telecommands that fetch whatever of
    the selected experiment never came down -- the holes snapped out to the
    file's 195-byte downlink packet grid and merged into the fewest commands, as
    a file for `simple_sat_ops --tc-file=`. The name of the file on the
    satellite comes from the sent_tcmd command log.

    How long the file is: a bulk_file packet carries a byte offset but never the
    file's length, so taking the length to be the largest offset seen hands the
    answer to whichever offset field was worst corrupted. One chunk claiming
    offset 2051784 sized the 2026-07-21 experiment at 2051979 bytes, and a file
    that is complete showed as 27% recovered with a re-download list demanding
    1.5 MB that does not exist. The length is read out of the satellite's own
    "Bulk downlink complete" log lines instead.

    An offset field is only as good as the packet carrying it, and the same CSP
    CRC32 covers both, so only CRC-verified chunks are trusted to say where they
    belong. A chunk whose CRC failed still fills bytes nothing verified claimed,
    but only if its offset lands on the grid the verified ones establish, and
    every byte it contributes is marked: the coverage map paints those bytes
    amber, and any image with a frame resting on them is flagged. Both live in
    bulk_size.c.

    Read-only on the DB. Press F5 to re-read it and rebuild the experiment list.

    Usage:
      mpi_viewer [--db=<packet_db.sqlite>] [--list]

    With no --db the default store is used ($SSO_PACKET_DB, else the FrontierSat
    root's packet_db.sqlite).

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

#include <raylib.h>

#include "bulk_size.h"
#include "packet_db.h"
#include "sso_version.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef WITH_SQLITE3
int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "mpi_viewer")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "mpi_viewer: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

// Bulk-file packet geometry (mirrors mpi_reconstruct.c). A bulk_file payload is
// [packet_type:1][file_offset:4 LE][data...]; at most 195 file bytes per packet.
#define BULK_FILE_PACKET_TYPE   16
#define BULK_FILE_HEADER_SIZE   5
#define BULK_FILE_MAX_DATA      195
#define BULK_FILE_CRC_LEN       4    // CSP CRC32 trailer, stripped only when it verified
#define BULK_FILE_MAX_PLAUSIBLE (2 * 1024 * 1024)

// MPI science-data heuristics (mirror mpi_reconstruct.c / the firmware).
#define MPI_MIN_FILE_BYTES  20000
#define MPI_SYNC_WORD       "\x0C\xFF\xFF\x0C"
#define MPI_SYNC_WORD_LEN   4
#define MPI_MIN_SYNC        8
#define MPI_SESSION_GAP_MS  (15.0 * 60.0 * 1000.0)
#define MPI_SESSION_MARGIN_MS (120.0 * 1000.0)

// Experiment grouping: markers of one recording fall within its <=15-minute run,
// while separate experiments are days apart, so a 1-hour gap cleanly separates
// experiments while keeping one recording's markers together.
#define MPI_EXP_GAP_MS      (60.0 * 60.0 * 1000.0)
// A recording lasts at most MPI_max_recording_duration_sec (900 s) in firmware;
// allow a little slack when deciding which markers belong to one experiment.
#define MPI_REC_WINDOW_MS   (30.0 * 60.0 * 1000.0)
// Sane unix-ms range for a marker timestamp (2026-01-01 .. 2027-01-01). A
// bit-flipped timestamp usually lands outside this and is rejected.
#define MPI_MARKER_MIN_MS   1767225600000.0
#define MPI_MARKER_MAX_MS   1798761600000.0

// A single bulk-file download telecommand can fetch at most this much
// contiguous data: the firmware clamps max_bytes to
// COMMS_bulk_file_downlink_max_allowable_total_bytes (1,000,000) in
// bulk_file_downlink.c. The re-download export merges the missing chunks into
// the fewest commands that stay within this span (mirrors mpi_reconstruct.c).
#define COMMS_BULK_DOWNLINK_MAX_BYTES  1000000L

// Every MPI download this station has flown went through the stored blob rather
// than calling the firmware command directly, so the exported commands use the
// same wrapper. The blob forwards its three semicolon-separated arguments
// (file_path;start_offset;byte_count) to COMMS_bulk_file_downlink_start.
#define MPI_BLOB_PATH  "blobs/bulk_downlink_start_v2.blob"

// Room for an on-satellite path like "mpi_data/2026-08-08.mpi".
#define MPI_SAT_PATH_LEN  128

// A download command sent this long before a burst's first packet still counts
// as the command that started that download.
#define MPI_CMD_LOOKBACK_MS  (30.0 * 60.0 * 1000.0)

// On-wire MPI data frame geometry (measured; matches the aux packer above and
// the First Light notebook's pixel slice).
#define FRAME_STRIDE   152   // bytes per frame (sync .. last pixel)
#define PIX_START      22    // first pixel byte within a frame
#define NPIX           65    // big-endian uint16 pixels per frame
#define SCAN_IDX_OFF   13    // inner dome scan index byte (the image row)
#define MAX_FPI        64    // largest scan period we render

typedef struct {
    double   ts_ms;
    long     off;
    int      rs;
    int      crc_ok;    // the packet's CSP CRC32 checked out, so its offset is a fact
    long     dlen;
    int      has_sync;
    uint8_t *data;
    char     ts_iso[32];
} chunk_t;

typedef struct {
    char     utc[24];        // experiment start time "2026-07-21 17:23:00" (UTC)
    double   t_start_ms;     // experiment start (unix ms) from the mpi_start marker
    char     sat_path[MPI_SAT_PATH_LEN];  // the file on the satellite, "" if unknown
    double   t_first_recv_ms;// receive time of this experiment's earliest packet
    double   t_last_recv_ms; // receive time of its latest packet
    double   t_end_ms;       // last marker time (unix ms); recording end
    uint8_t *buf;            // reconstructed bytes (all passes merged)
    uint8_t *present;        // 1 where a real byte landed
    uint8_t *verified;       // 1 where that byte came from a CRC-verified packet
    long     size;
    int      size_from_log;  // size is the satellite's own byte count, not a guess
    int      size_exact;     // that byte count came from a download that hit EOF
    long     dropped;        // chunks set aside: off-grid or past the end of the file
    long     recovered;      // present bytes
    long     unverified;     // present bytes that no CRC vouches for
    int      nframes;        // number of sync words found
    long    *fr_off;         // byte offset of each frame's sync word
    int     *fr_ctr;         // running counter (bytes 4-5) per frame
    int     *fr_scan;        // inner dome scan index (byte 13) per frame
    int     *fr_tv;          // inner dome target voltage (bytes 11-12) per frame
    double  *fr_ts;          // capture time (unix ms) per frame, -1 if unknown
    int     *fr_bad;         // 1 if the frame body overlaps a JSON marker (drop it)
    int     *fr_unver;       // 1 if any of the frame's bytes came from a failed-CRC packet
} experiment_t;

// Release one experiment's buffers. Split out of free_experiments because a
// rebuild against the satellite-reported file length has to drop the first,
// shorter-lived reconstruction.
static void free_experiment(experiment_t *e)
{
    free(e->buf); free(e->present); free(e->verified);
    free(e->fr_off); free(e->fr_ctr);
    free(e->fr_scan); free(e->fr_tv);
    free(e->fr_ts); free(e->fr_bad); free(e->fr_unver);
}

// Little-endian uint32 file offset out of a bulk_file payload.
static long bulk_offset(const uint8_t *pl)
{
    return (long) pl[1] | ((long) pl[2] << 8)
         | ((long) pl[3] << 16) | ((long) pl[4] << 24);
}

// Big-endian uint16 at buf+i.
static int be16(const uint8_t *buf, long i)
{
    return (buf[i] << 8) | buf[i + 1];
}

// "2026-08-09 22:08:04.123" (UTC) from a unix-ms timestamp.
static void fmt_utc_ms(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    int ms = (int) (ts_ms - (double) secs * 1000.0);
    struct tm tmv;
    gmtime_r(&secs, &tmv);
    snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
}

// "2026-07-21 17:23:00" (UTC, seconds) from a unix-ms timestamp.
static void fmt_utc_s(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    struct tm tmv;
    gmtime_r(&secs, &tmv);
    snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

// "20260721T172300" (UTC) from a unix-ms timestamp, for export filenames.
static void fmt_stamp(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    struct tm tmv;
    gmtime_r(&secs, &tmv);
    snprintf(out, n, "%04d%02d%02dT%02d%02d%02d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

// ---- embedded JSON marker parsing ------------------------------------------
//
// The firmware stamps each science file with JSON markers (see the file header
// comment). Both the mpi_start header and the periodic footers carry the same
// "timestamp_ms":<unix ms> field (the buffer-fill wall time). We read those to
// group and to timestamp frames. The markers are subject to bit-flips like any
// downlinked byte, so parsing validates against a sane mission time range and we
// vote/median across many markers rather than trust one.

// Parse the decimal integer immediately following `key` within buf[start..lim).
// Skips the "": separators. Returns the value (as double, for ms), or -1 if the
// digits are corrupt (a short run) or the value is outside the mission range.
static double parse_uint_after(const uint8_t *buf, long start, long lim,
                               const char *key)
{
    long klen = (long) strlen(key);
    for (long i = start; i + klen <= lim; i++) {
        if (memcmp(buf + i, key, (size_t) klen) != 0) continue;
        long j = i + klen;
        while (j < lim && (buf[j] < '0' || buf[j] > '9')) {
            if (buf[j] == '}') return -1;   // ran off the end of this object
            j++;
        }
        double v = 0; int nd = 0;
        while (j < lim && buf[j] >= '0' && buf[j] <= '9') {
            v = v * 10.0 + (buf[j] - '0'); j++; nd++;
        }
        if (nd >= 13 && v >= MPI_MARKER_MIN_MS && v <= MPI_MARKER_MAX_MS) return v;
        return -1;
    }
    return -1;
}

// Given a hit of the "uptime_ms" key at buf+pos, recover the marker's unix-ms
// time from "timestamp_ms" (the true buffer-fill time). Returns -1 if the digits
// are corrupt. We deliberately do NOT fall back to the "timestamp" base field:
// that is the RTC reference epoch, ~5 minutes before timestamp_ms, so using it
// would bias the experiment start and the time curve early.
static double parse_marker_ts(const uint8_t *buf, long size, long pos)
{
    long lim = pos + 300; if (lim > size) lim = size;
    return parse_uint_after(buf, pos, lim, "timestamp_ms");
}

// Find every MPI JSON marker in buf. Returns count and fills *out_off (byte
// offset of the "uptime_ms" key) and *out_ts (unix ms), ascending by offset.
// Markers later than max_ts are rejected: a capture cannot be downlinked before
// it happens, so a timestamp past the newest packet is a bit-flip. GNSS log
// markers use the same JSON scheme, so a marker with "gnss" just before it is
// skipped. Caller frees *out_off / *out_ts.
static int scan_markers(const uint8_t *buf, long size, double max_ts,
                        long **out_off, double **out_ts)
{
    long *off = NULL; double *ts = NULL; int n = 0, cap = 0;
    for (long i = 0; i + 9 <= size; i++) {
        if (memcmp(buf + i, "uptime_ms", 9) != 0) continue;
        long g0 = i - 16; if (g0 < 0) g0 = 0;
        int is_gnss = 0;
        for (long j = g0; j + 4 <= i; j++)
            if (memcmp(buf + j, "gnss", 4) == 0 || memcmp(buf + j, "gnsq", 4) == 0) { is_gnss = 1; break; }
        if (is_gnss) { i += 8; continue; }
        double t = parse_marker_ts(buf, size, i);
        if (t < 0 || t > max_ts) { i += 8; continue; }
        if (n == cap) {
            cap = cap ? cap * 2 : 16;
            off = (long *) realloc(off, (size_t) cap * sizeof *off);
            ts  = (double *) realloc(ts, (size_t) cap * sizeof *ts);
        }
        off[n] = i; ts[n] = t; n++;
        i += 8;
    }
    *out_off = off; *out_ts = ts;
    return n;
}

// The byte span [*s, *e) occupied by the JSON object around a marker at `mo`
// (the "uptime_ms" position): scan back to the opening '{' and forward to the
// closing '}'. Bounded so corruption can't run away.
static void marker_span(const uint8_t *buf, long size, long mo, long *s, long *e)
{
    long a = mo, alim = mo - 40; if (alim < 0) alim = 0;
    while (a > alim && buf[a] != '{') a--;
    if (buf[a] != '{') a = mo - 2 < 0 ? 0 : mo - 2;   // fallback if '{' flipped
    long b = mo, blim = mo + 256; if (blim > size) blim = size;
    while (b < blim && buf[b] != '}') b++;
    if (b < blim && buf[b] == '}') b++;               // include the '}'
    else b = mo + 180 < size ? mo + 180 : size;       // fallback typical length
    *s = a; *e = b;
}

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *) a, y = *(const double *) b;
    return (x > y) - (x < y);
}

// Median of n times, or -1 if none. Robust central value for clustering.
static double median_time(const double *t, int n)
{
    if (n <= 0) return -1;
    double *c = (double *) malloc((size_t) n * sizeof *c);
    if (c == NULL) return t[0];
    memcpy(c, t, (size_t) n * sizeof *c);
    qsort(c, (size_t) n, sizeof *c, cmp_double);
    double m = c[n / 2];
    free(c);
    return m;
}

// Linear interpolation of the (offset -> time) curve at byte offset x. Clamps at
// the ends. off[] is ascending; n >= 1.
static double interp_time(const long *off, const double *ts, int n, long x)
{
    if (n <= 0) return -1;
    if (x <= off[0]) return ts[0];
    if (x >= off[n - 1]) return ts[n - 1];
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) { int m = (lo + hi) / 2; if (off[m] <= x) lo = m; else hi = m; }
    double span = (double) (off[hi] - off[lo]);
    if (span <= 0) return ts[lo];
    return ts[lo] + (ts[hi] - ts[lo]) * (double) (x - off[lo]) / span;
}

// ---- DB load / reassembly --------------------------------------------------

// Load every bulk_file chunk with a sane offset, oldest first (mirrors
// mpi_reconstruct's loader). Returns 0 on success.
static int load_chunks(sqlite3 *db, chunk_t **out, int *out_n)
{
    const char *sql =
        "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, "
        "       ts_received, payload, rs_errs, crc_status "
        "FROM packet WHERE packet_type=16 ORDER BY ts_received, id";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "mpi_viewer: query failed: %s\n", sqlite3_errmsg(db));
        return -1;
    }

    chunk_t *cs = NULL;
    int n = 0, cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const uint8_t *pl = (const uint8_t *) sqlite3_column_blob(st, 2);
        int pl_len = sqlite3_column_bytes(st, 2);
        if (pl == NULL || pl_len < BULK_FILE_HEADER_SIZE + 1) continue;
        long off = bulk_offset(pl);
        if (off < 0 || off > BULK_FILE_MAX_PLAUSIBLE) continue;
        // A verified packet had its 4-byte CSP CRC32 trailer stripped on the
        // way in; one that failed still carries it, and those 4 bytes are not
        // file data.
        int crc_ok = (sqlite3_column_int(st, 4) == 1);
        long dlen = pl_len - BULK_FILE_HEADER_SIZE - (crc_ok ? 0 : BULK_FILE_CRC_LEN);
        if (dlen <= 0) continue;
        if (dlen > BULK_FILE_MAX_DATA) dlen = BULK_FILE_MAX_DATA;
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
        c->crc_ok = crc_ok;
        c->dlen  = dlen;
        c->data  = (uint8_t *) malloc((size_t) dlen);
        if (c->data == NULL) { fprintf(stderr, "out of memory\n"); goto fail; }
        memcpy(c->data, pl + BULK_FILE_HEADER_SIZE, (size_t) dlen);
        c->has_sync = 0;
        for (long k = 0; k + MPI_SYNC_WORD_LEN <= dlen; k++)
            if (memcmp(c->data + k, MPI_SYNC_WORD, MPI_SYNC_WORD_LEN) == 0) { c->has_sync = 1; break; }
        const char *ts = (const char *) sqlite3_column_text(st, 1);
        snprintf(c->ts_iso, sizeof c->ts_iso, "%s", ts ? ts : "");
        n++;
    }
    sqlite3_finalize(st);
    *out = cs; *out_n = n;
    return 0;

fail:
    for (int i = 0; i < n; i++) free(cs[i].data);
    free(cs);
    sqlite3_finalize(st);
    return -1;
}

// ---- the ground station's command log --------------------------------------
//
// sent_tcmd holds every telecommand this station transmitted. MPI work shows up
// as a recording command and as download commands:
//
//   CTS1+mpi_enable_active_mode(mpi_data/2026-08-08.mpi)@...
//   CTS1+exec_blob_from_fs(blobs/bulk_downlink_start_v2.blob,0,mpi_data/...;0;0)@...
//   CTS1+comms_bulk_file_downlink_start(mpi_data/2026-08-08.mpi,160485,1170)@...
//
// so the file on the satellite is just the "mpi_data/....mpi" token in the text,
// and the command started a recording rather than a download when
// "mpi_enable_active_mode(" appears. This is best-effort: a download commanded
// from elsewhere, or a store with no sent_tcmd table, simply leaves an
// experiment's path blank and the export writes a placeholder instead.

typedef struct {
    double ts_ms;
    char   path[MPI_SAT_PATH_LEN];
    int    is_record;
} mpicmd_t;

// Copy the "mpi_data/....mpi" token out of a command into `path`. Returns 1 on
// success. The token ends at the argument separator (',' in the direct form,
// ';' in the blob form) or the closing parenthesis.
static int extract_mpi_path(const char *text, char *path, size_t n)
{
    const char *s = strstr(text, "mpi_data/");
    if (s == NULL) return 0;
    size_t k = 0;
    while (s[k] != '\0' && s[k] != ',' && s[k] != ';' && s[k] != ')' && k < n - 1) k++;
    // s starts with "mpi_data/", so k is at least 9 and this test is in bounds.
    if (strncmp(s + k - 4, ".mpi", 4) != 0) return 0;   // not a science file
    memcpy(path, s, k);
    path[k] = '\0';
    return 1;
}

// Load every MPI-related telecommand, oldest first. Returns the count (0 if the
// table is absent, which is not an error).
static int load_mpi_commands(sqlite3 *db, mpicmd_t **out)
{
    *out = NULL;
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT ts_sent_ms, command_text FROM sent_tcmd "
            "WHERE command_text LIKE '%mpi_data/%' ORDER BY ts_sent_ms",
            -1, &st, NULL) != SQLITE_OK)
        return 0;

    mpicmd_t *cmds = NULL;
    int n = 0, cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const char *text = (const char *) sqlite3_column_text(st, 1);
        if (text == NULL) continue;
        char path[MPI_SAT_PATH_LEN];
        if (!extract_mpi_path(text, path, sizeof path)) continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 64;
            mpicmd_t *t = (mpicmd_t *) realloc(cmds, (size_t) cap * sizeof *cmds);
            if (t == NULL) break;
            cmds = t;
        }
        cmds[n].ts_ms = (double) sqlite3_column_int64(st, 0);
        snprintf(cmds[n].path, sizeof cmds[n].path, "%s", path);
        cmds[n].is_record = (strstr(text, "mpi_enable_active_mode(") != NULL);
        n++;
    }
    sqlite3_finalize(st);
    *out = cmds;
    return n;
}

// Read every bulk-downlink log line the satellite sent into the size table, so
// each experiment's file can be reconstructed against its real length instead
// of against the largest offset that happened to arrive. Missing or unreadable
// log packets are not an error: an experiment whose file is not named there
// falls back to sizing itself from its chunks.
static void load_bulk_log(sqlite3 *db, bulk_size_table_t *sizes)
{
    // Log packets are packet_type 3; both lines of interest mention "downl",
    // which keeps the scan off the rest of the log traffic.
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, payload "
            "FROM packet WHERE packet_type = 3 AND payload LIKE '%downl%' "
            "ORDER BY ts_received, id", -1, &st, NULL) != SQLITE_OK)
        return;

    while (sqlite3_step(st) == SQLITE_ROW) {
        const void *pl = sqlite3_column_blob(st, 1);
        int pl_len = sqlite3_column_bytes(st, 1);
        if (pl == NULL || pl_len <= 0) continue;
        // A log payload is raw bytes, not a C string, and holds anything at all
        // once RS has failed on it, so copy it out NUL-terminated first.
        char text[512];
        int m = pl_len < (int) sizeof text - 1 ? pl_len : (int) sizeof text - 1;
        memcpy(text, pl, (size_t) m);
        text[m] = '\0';
        bulk_size_feed(sizes, text, sqlite3_column_double(st, 0));
    }
    sqlite3_finalize(st);
}

// Name the file an experiment was written to. The recording command carries the
// name and is sent at the moment the recording starts, so the mpi_enable_active_mode
// nearest the experiment's embedded start time is the reliable answer. Failing
// that (the command log predates the recording, or the recording was commanded
// elsewhere) fall back to whatever download command was sent while this
// experiment's packets were coming down. Leaves sat_path empty when neither
// matches.
static void label_experiment(experiment_t *e, const mpicmd_t *cmds, int ncmds)
{
    e->sat_path[0] = '\0';
    double best = -1;
    for (int k = 0; k < ncmds; k++) {
        if (!cmds[k].is_record) continue;
        double d = fabs(cmds[k].ts_ms - e->t_start_ms);
        if (d > MPI_REC_WINDOW_MS) continue;
        if (best < 0 || d < best) {
            best = d;
            snprintf(e->sat_path, sizeof e->sat_path, "%s", cmds[k].path);
        }
    }
    if (best >= 0) return;

    for (int k = 0; k < ncmds; k++) {
        if (cmds[k].is_record) continue;
        if (cmds[k].ts_ms > e->t_last_recv_ms) break;            // sorted by time
        if (cmds[k].ts_ms < e->t_first_recv_ms - MPI_CMD_LOOKBACK_MS) continue;
        snprintf(e->sat_path, sizeof e->sat_path, "%s", cmds[k].path);
    }
}

// Reassemble the chunks named by idx[0..nidx) by absolute offset (mirrors
// mpi_reconstruct). RS-clean chunks win; uncorrectable chunks only fill gaps.
// A chunk's offset field is covered by the same CRC32 as its data, so only a
// CRC-verified chunk knows where it belongs. Verified chunks therefore decide
// the file's extent, are laid down first, and are the only ones allowed to
// teach the download's 195-byte offset grid. A chunk that failed its CRC is
// still worth having -- it is often the only copy of those bytes -- so it fills
// bytes nothing verified has claimed, but only if its offset at least lands on
// the grid, and every byte it contributes is marked unverified so the viewer
// can flag the images built from it.
//
// known_size is the satellite-reported file length, or 0 when the log did not
// say. An exact length IS the buffer size, and a chunk claiming to start past
// the end of the file is discarded. A length the satellite could only put a
// floor under is used as a floor: the buffer still stretches to the last
// verified chunk, because truncating there would throw away real data.
// *out_dropped counts the chunks set aside.
static long reassemble(const chunk_t *cs, const int *idx, int nidx,
                       long known_size, int known_exact,
                       uint8_t **out_buf, uint8_t **out_present,
                       uint8_t **out_verified, long *out_dropped)
{
    long size = 0;

    long *offs = (long *) malloc((size_t) (nidx > 0 ? nidx : 1) * sizeof *offs);
    unsigned char *ver = (unsigned char *) malloc((size_t) (nidx > 0 ? nidx : 1));
    unsigned char *keep = (unsigned char *) malloc((size_t) (nidx > 0 ? nidx : 1));
    if (offs == NULL || ver == NULL || keep == NULL) {
        free(offs); free(ver); free(keep); return -1;
    }
    for (int j = 0; j < nidx; j++) {
        offs[j] = cs[idx[j]].off;
        ver[j]  = (unsigned char) (cs[idx[j]].crc_ok != 0);
    }
    unsigned char grid[195];
    bulk_grid_learn(offs, ver, nidx, grid);
    for (int j = 0; j < nidx; j++) keep[j] = ver[j] || bulk_grid_ok(grid, offs[j]);
    free(offs); free(ver);

    if (known_size > 0 && known_exact) {
        size = known_size;
    } else {
        size = known_size;   // 0 when nothing is known, else a floor
        for (int j = 0; j < nidx; j++) {
            const chunk_t *c = &cs[idx[j]];
            if (!c->crc_ok) continue;                 // only a verified offset sets the extent
            if (c->off + c->dlen > size) size = c->off + c->dlen;
        }
    }
    if (size <= 0) { free(keep); return -1; }
    for (int j = 0; j < nidx; j++)
        if (cs[idx[j]].off >= size) keep[j] = 0;

    long dropped = 0;
    for (int j = 0; j < nidx; j++) if (!keep[j]) dropped++;

    uint8_t *buf      = (uint8_t *) malloc((size_t) size);
    uint8_t *present  = (uint8_t *) calloc((size_t) size, 1);
    uint8_t *verified = (uint8_t *) calloc((size_t) size, 1);
    if (buf == NULL || present == NULL || verified == NULL) {
        free(buf); free(present); free(verified); free(keep); return -1;
    }
    memset(buf, 0, (size_t) size);

    for (int phase = 0; phase < 2; phase++) {
        for (int j = 0; j < nidx; j++) {
            if (!keep[j]) continue;
            const chunk_t *c = &cs[idx[j]];
            if ((phase == 0) != (c->crc_ok != 0)) continue;
            long off = c->off;
            for (long k = 0; k < c->dlen && off + k < size; k++) {
                if (present[off + k] && phase == 1) continue;
                buf[off + k] = c->data[k];
                present[off + k] = 1;
                if (phase == 0) verified[off + k] = 1;
            }
        }
    }
    free(keep);
    *out_buf = buf; *out_present = present; *out_verified = verified;
    *out_dropped = dropped;
    return size;
}

static long count_sync(const uint8_t *buf, long size)
{
    long n = 0;
    for (long i = 0; i + MPI_SYNC_WORD_LEN <= size; ) {
        if (memcmp(buf + i, MPI_SYNC_WORD, MPI_SYNC_WORD_LEN) == 0) { n++; i += MPI_SYNC_WORD_LEN; }
        else i++;
    }
    return n;
}

// Build one experiment from the union of its passes' chunks (idx[0..nidx)).
// start_hint is the experiment's approximate start (unix ms) from the clustering
// pass; max_ts is the newest packet time (upper bound on plausible markers).
// Returns 0 if the merged stream is not valid MPI.
static int build_experiment(const chunk_t *cs, const int *idx, int nidx,
                            double start_hint, double max_ts,
                            long known_size, int known_exact,
                            experiment_t *out)
{
    uint8_t *buf = NULL, *present = NULL, *verified = NULL;
    long dropped = 0;
    long size = reassemble(cs, idx, nidx, known_size, known_exact,
                           &buf, &present, &verified, &dropped);
    if (size < MPI_MIN_FILE_BYTES || count_sync(buf, size) < MPI_MIN_SYNC) {
        free(buf); free(present); free(verified);
        return 0;
    }

    // Find every sync word; each one starts a 152-byte frame (drop any whose
    // frame would run past EOF).
    long *offs = NULL;
    int n = 0, cap = 0;
    for (long o = 0; o + FRAME_STRIDE <= size; o++) {
        if (buf[o] == 0x0C && buf[o + 1] == 0xFF && buf[o + 2] == 0xFF && buf[o + 3] == 0x0C) {
            if (n == cap) {
                cap = cap ? cap * 2 : 1024;
                offs = (long *) realloc(offs, (size_t) cap * sizeof(long));
            }
            offs[n++] = o;
        }
    }
    if (n < MPI_MIN_SYNC) { free(offs); free(buf); free(present); free(verified); return 0; }

    memset(out, 0, sizeof *out);
    out->buf = buf; out->present = present; out->verified = verified;
    out->size = size;
    out->dropped = dropped;
    out->nframes = n;
    out->t_first_recv_ms = cs[idx[0]].ts_ms;
    out->t_last_recv_ms  = cs[idx[0]].ts_ms;
    for (int j = 1; j < nidx; j++) {
        double t = cs[idx[j]].ts_ms;
        if (t < out->t_first_recv_ms) out->t_first_recv_ms = t;
        if (t > out->t_last_recv_ms)  out->t_last_recv_ms  = t;
    }
    out->fr_off = offs;
    out->fr_ctr  = (int *) malloc((size_t) n * sizeof(int));
    out->fr_scan = (int *) malloc((size_t) n * sizeof(int));
    out->fr_tv   = (int *) malloc((size_t) n * sizeof(int));
    out->fr_ts   = (double *) malloc((size_t) n * sizeof(double));
    out->fr_bad  = (int *) calloc((size_t) n, sizeof(int));
    out->fr_unver = (int *) calloc((size_t) n, sizeof(int));
    if (out->fr_ctr == NULL || out->fr_scan == NULL || out->fr_tv == NULL
        || out->fr_ts == NULL || out->fr_bad == NULL || out->fr_unver == NULL) {
        free(out->fr_ctr); free(out->fr_scan); free(out->fr_tv);
        free(out->fr_ts); free(out->fr_bad); free(out->fr_unver);
        free(offs); free(buf); free(present); free(verified);
        return 0;
    }
    for (int k = 0; k < n; k++) {
        long o = offs[k];
        out->fr_ctr[k]  = be16(buf, o + 4);
        out->fr_scan[k] = buf[o + SCAN_IDX_OFF];
        out->fr_tv[k]   = be16(buf, o + 11);
        out->fr_ts[k]   = -1.0;
    }

    long recovered = 0, unverified = 0;
    for (long b = 0; b < size; b++) {
        if (!present[b]) continue;
        recovered++;
        if (!verified[b]) unverified++;
    }
    out->recovered = recovered;
    out->unverified = unverified;

    // A frame is suspect if any byte of it came from a packet whose CRC failed:
    // those bytes were placed on an offset nothing vouches for, and their
    // contents were never checked either.
    for (int k = 0; k < n; k++) {
        long o = offs[k];
        for (long b = o; b < o + FRAME_STRIDE && b < size; b++)
            if (present[b] && !verified[b]) { out->fr_unver[k] = 1; break; }
    }

    // Locate the JSON markers in the merged stream. They give both the
    // (offset -> time) curve and the byte spans to drop straddling frames.
    long *mo = NULL; double *mt = NULL;
    int nm = scan_markers(buf, size, max_ts, &mo, &mt);

    // Experiment start = the earliest marker time within a recording window of
    // the marker cluster's centre (rejects a bit-flipped low outlier). Falls
    // back to the clustering hint if no markers parsed here.
    double center = median_time(mt, nm);
    double start = start_hint > 0 ? start_hint : center;
    if (nm > 0 && center > 0) {
        double mn = 0;
        int have = 0;
        for (int k = 0; k < nm; k++) {
            if (fabs(mt[k] - center) > MPI_REC_WINDOW_MS) continue;
            if (!have || mt[k] < mn) { mn = mt[k]; have = 1; }
        }
        if (have) start = mn;
    }
    out->t_start_ms = start;
    if (start > 0) fmt_utc_s(start, out->utc, sizeof out->utc);
    else snprintf(out->utc, sizeof out->utc, "unknown");

    // Recording end = the latest marker still inside the recording window. The
    // firmware writes one every 20 KB it flushes, so the last one lands within
    // a few seconds of the stop, and the span is how long the MPI actually ran.
    out->t_end_ms = start;
    for (int k = 0; k < nm; k++) {
        if (mt[k] < start || mt[k] > start + MPI_REC_WINDOW_MS) continue;
        if (mt[k] > out->t_end_ms) out->t_end_ms = mt[k];
    }

    // Build the (offset -> time) curve: anchor offset 0 to the start, then add
    // each in-window marker keeping the curve strictly increasing in offset and
    // non-decreasing in time (drops corrupted out-of-order points).
    long *coff = (long *) malloc((size_t) (nm + 1) * sizeof(long));
    double *cts = (double *) malloc((size_t) (nm + 1) * sizeof(double));
    int cn = 0;
    if (coff != NULL && cts != NULL && start > 0) {
        coff[cn] = 0; cts[cn] = start; cn++;
        for (int k = 0; k < nm; k++) {
            if (mt[k] < start - 1000.0 || mt[k] > start + MPI_REC_WINDOW_MS) continue;
            if (mo[k] <= coff[cn - 1]) continue;
            if (mt[k] < cts[cn - 1]) continue;
            coff[cn] = mo[k]; cts[cn] = mt[k]; cn++;
        }
        for (int k = 0; k < n; k++)
            out->fr_ts[k] = interp_time(coff, cts, cn, offs[k]);
    }
    free(coff); free(cts);

    // Drop frames whose 152-byte body overlaps a marker blob (the mid-frame JSON
    // splice that causes the routine per-20 KB artifact).
    for (int m = 0; m < nm; m++) {
        long ms, me;
        marker_span(buf, size, mo[m], &ms, &me);
        for (int k = 0; k < n; k++) {
            long fo = offs[k];
            if (fo < me && fo + FRAME_STRIDE > ms) out->fr_bad[k] = 1;
        }
    }

    free(mo); free(mt);
    return 1;
}

// One download burst: the chunk indices reassembled together in a receive-time
// cluster, plus its embedded experiment time (repr_ms, -1 if none) and the first
// chunk's receive time (recv_ms, for placing marker-less bursts).
typedef struct {
    int   *idx;
    int    nidx;
    double repr_ms;
    double recv_ms;
    int    exp;      // assigned experiment id, -1 until clustered
} burst_t;

// Load all experiments from an open DB. Returns the experiment count and fills
// *out (caller frees via free_experiments).
static int load_experiments_from_db(sqlite3 *db, experiment_t **out)
{
    *out = NULL;
    chunk_t *cs = NULL; int n = 0;
    if (load_chunks(db, &cs, &n) != 0 || n == 0) { free(cs); return 0; }

    // Newest packet receive time bounds plausible marker timestamps: a capture
    // cannot be downlinked before it happens, so a later timestamp is a bit-flip.
    double max_recv = 0;
    for (int k = 0; k < n; k++) if (cs[k].ts_ms > max_recv) max_recv = cs[k].ts_ms;
    double max_ts = max_recv + 3600000.0;   // +1 h slack

    // Pass 1: cluster chunks into download bursts (sync-bearing, 15-min gap),
    // keep only those that reassemble to valid MPI, and read each burst's
    // embedded experiment time.
    int *idx = (int *) malloc((size_t) n * sizeof(int));
    burst_t *bs = NULL; int nb = 0, bcap = 0;
    if (idx == NULL) goto done;

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
        double wlo = first_sync - MPI_SESSION_MARGIN_MS;
        double whi = last_sync + MPI_SESSION_MARGIN_MS;
        int nidx = 0;
        for (int k = 0; k < n; k++) {
            if (cs[k].ts_ms < wlo || cs[k].ts_ms > whi) continue;
            idx[nidx++] = k;
        }
        i = last_i + 1;

        uint8_t *buf = NULL, *present = NULL, *verified = NULL;
        long dropped = 0;
        long size = reassemble(cs, idx, nidx, 0, 0, &buf, &present, &verified, &dropped);
        if (size >= MPI_MIN_FILE_BYTES && count_sync(buf, size) >= MPI_MIN_SYNC) {
            long *mo = NULL; double *mt = NULL;
            int nm = scan_markers(buf, size, max_ts, &mo, &mt);
            double repr = median_time(mt, nm);
            free(mo); free(mt);

            if (nb == bcap) {
                bcap = bcap ? bcap * 2 : 16;
                bs = (burst_t *) realloc(bs, (size_t) bcap * sizeof *bs);
            }
            bs[nb].idx = (int *) malloc((size_t) nidx * sizeof(int));
            memcpy(bs[nb].idx, idx, (size_t) nidx * sizeof(int));
            bs[nb].nidx = nidx;
            bs[nb].repr_ms = repr;
            bs[nb].recv_ms = first_sync;
            bs[nb].exp = -1;
            nb++;
        }
        free(buf); free(present); free(verified);
    }
    free(idx);

    if (nb == 0) goto done;

    // Pass 2: cluster bursts into experiments by embedded marker time. Sort the
    // bursts that have a marker time, then split whenever the gap exceeds
    // MPI_EXP_GAP_MS. Experiments come out in time order.
    int *ord = (int *) malloc((size_t) nb * sizeof(int));
    int nord = 0;
    for (int b = 0; b < nb; b++) if (bs[b].repr_ms > 0) ord[nord++] = b;
    // insertion sort ord[] by repr_ms (nb is tiny)
    for (int a = 1; a < nord; a++) {
        int key = ord[a]; double kv = bs[key].repr_ms; int b = a - 1;
        while (b >= 0 && bs[ord[b]].repr_ms > kv) { ord[b + 1] = ord[b]; b--; }
        ord[b + 1] = key;
    }
    int nexp = 0;
    double *exp_hint = (double *) malloc((size_t) (nord + 1) * sizeof(double));
    double prev = -1;
    for (int a = 0; a < nord; a++) {
        double t = bs[ord[a]].repr_ms;
        if (a == 0 || t - prev > MPI_EXP_GAP_MS) {
            exp_hint[nexp] = t;           // first (earliest) marker time of the cluster
            nexp++;
        }
        bs[ord[a]].exp = nexp - 1;
        prev = t;
    }

    // Marker-less bursts (rare) attach to the nearest experiment by receive time.
    for (int b = 0; b < nb; b++) {
        if (bs[b].exp >= 0) continue;
        int best = -1; double bestd = 0;
        for (int c = 0; c < nb; c++) {
            if (bs[c].exp < 0) continue;
            double d = fabs(bs[c].recv_ms - bs[b].recv_ms);
            if (best < 0 || d < bestd) { best = bs[c].exp; bestd = d; }
        }
        bs[b].exp = best;   // -1 if no experiment had markers at all
    }

    // The command log names the file each experiment was written to on the
    // satellite, which the re-download export needs. An absent or unreadable
    // sent_tcmd table just leaves the name blank.
    mpicmd_t *cmds = NULL;
    int ncmds = load_mpi_commands(db, &cmds);

    // What the satellite said each of those files is actually long.
    bulk_size_table_t sizes = {0};
    load_bulk_log(db, &sizes);

    // Pass 3: for each experiment, union its bursts' chunks (dedup) and build.
    experiment_t *exps = (experiment_t *) malloc((size_t) (nexp > 0 ? nexp : 1) * sizeof *exps);
    int *uidx = (int *) malloc((size_t) n * sizeof(int));
    char *seen = (char *) malloc((size_t) n);
    int nout = 0;
    if (exps != NULL && uidx != NULL && seen != NULL) {
        for (int e = 0; e < nexp; e++) {
            memset(seen, 0, (size_t) n);
            int nu = 0;
            for (int b = 0; b < nb; b++) {
                if (bs[b].exp != e) continue;
                for (int j = 0; j < bs[b].nidx; j++) {
                    int ci = bs[b].idx[j];
                    if (!seen[ci]) { seen[ci] = 1; uidx[nu++] = ci; }
                }
            }
            // Build once to read the markers, which is what names the file;
            // then, if the satellite has told us how long that file is, build
            // again against the real length. The first build's size is only the
            // largest offset that survived the grid filter.
            experiment_t ex;
            if (nu > 0 && build_experiment(cs, uidx, nu, exp_hint[e], max_ts, 0, 0, &ex)) {
                label_experiment(&ex, cmds, ncmds);
                int exact = 0;
                long known = bulk_size_lookup(&sizes, ex.sat_path, &exact);
                if (known > 0 && known != ex.size) {
                    experiment_t re;
                    if (build_experiment(cs, uidx, nu, exp_hint[e], max_ts,
                                         known, exact, &re)) {
                        label_experiment(&re, cmds, ncmds);
                        free_experiment(&ex);
                        ex = re;
                    }
                }
                // Only claim the length came from the satellite if it is the
                // length actually reconstructed against: a floor the chunks
                // already reach past is the chunks' answer, not the log's.
                if (known > 0 && known == ex.size) {
                    ex.size_from_log = 1;
                    ex.size_exact = exact;
                }
                exps[nout++] = ex;
            }
        }
    }
    free(cmds);
    bulk_size_free(&sizes);
    free(uidx); free(seen); free(ord); free(exp_hint);
    *out = exps;

    for (int b = 0; b < nb; b++) free(bs[b].idx);
    free(bs);
    for (int k = 0; k < n; k++) free(cs[k].data);
    free(cs);
    return nout;

done:
    for (int b = 0; b < nb; b++) free(bs[b].idx);
    free(bs);
    for (int k = 0; k < n; k++) free(cs[k].data);
    free(cs);
    return 0;
}

static void free_experiments(experiment_t *exps, int nexp)
{
    for (int k = 0; k < nexp; k++) free_experiment(&exps[k]);
    free(exps);
}

// Open the DB read-only, load experiments, close. Returns the count (0 on any
// failure). Used at startup and on manual refresh (F5).
static int reload_experiments(const char *db_path, experiment_t **out)
{
    *out = NULL;
    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "mpi_viewer: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 0;
    }
    int nexp = load_experiments_from_db(db, out);
    sqlite3_close(db);
    return nexp;
}

// ---- target-voltage setpoints ----------------------------------------------
//
// The inner dome TARGET voltage (aux bytes 11-12) is the commanded bias: a
// setpoint m = m*(65536/N) - 1 for m = 1..N (so 4095, 8191 .. 65535 for N=16),
// where m = N (65535) is the 0 V background and lower m are increasingly
// negative bias. It sets both N (the setpoint count, via detect_n_targets) and
// each frame's column (via col_of). Because the real values sit on a coarse
// grid, a bit-flipped voltage lands off-grid and is rejected -- unlike the raw
// scan-index byte, where any 0..31 looks valid -- which is why the column axis
// is keyed off the voltage. The firmware never commands m = N-1, so that
// setpoint gets no column and the image is N-1 columns wide.

// Setpoint index (0..n-1) of a target voltage on the n-step grid, or -1 if the
// value is off-grid (a corrupted byte).
static int tv_row(int tv, int n)
{
    int step = 65536 / n;
    int m = (tv + 1 + step / 2) / step;              // nearest setpoint
    if (m < 1 || m > n) return -1;
    int nearest = m * step - 1;
    int diff = tv - nearest; if (diff < 0) diff = -diff;
    if (diff > step / 8) return -1;                  // too far from any setpoint
    return m - 1;
}

// Number of target setpoints actually used (8 or 16). The 8-step scan uses only
// the even setpoints of the 16-grid, so if the odd ones (rows 0,2,4,..) show up
// the scan is 16-step.
static int detect_n_targets(const experiment_t *s)
{
    int odd = 0;
    for (int k = 0; k < s->nframes; k++) {
        int r = tv_row(s->fr_tv[k], 16);
        if (r >= 0 && (r & 1) == 0) odd++;           // r even <=> a 16-only setpoint
    }
    return odd >= 4 ? 16 : 8;
}

// ---- viewer state ----------------------------------------------------------

enum { SCALE_AUTO_IMAGE, SCALE_AUTO_SESSION, SCALE_MANUAL };

typedef struct {
    int    sel;            // selected experiment
    int    img_pos;        // position within the selected experiment's image list
    int   *fr_img;         // per-frame image index (<0 = dropped / corrupt frame)
    int    n_img;
    int    n_targets;      // detected setpoints per sweep (8 or 16)
    int    fpi;            // scan period: n_targets or the override (boundary + column map)
    int    ncols;          // image width in columns (fpi-1; the never-commanded setpoint gets none)
    int    fpi_override;   // 0 = use the detected count, else forced 8/16
    int    zoom;
    int    scale_mode;
    int    dn_min, dn_max;
    int    sess_min, sess_max;
    int    playing;
    float  ips;
    float  accum;
} view_t;

// Column of frame k within its image, from the inner-dome TARGET voltage (the
// commanded bias). The columns are the physical scan axis: the 0 V background
// frame (setpoint 65535, m = fpi -- no ion signal on the CCD, sent once per
// sweep) sits at the far RIGHT, and increasingly negative bias voltages stack to
// the LEFT, so the most negative bias (setpoint 4095, m = 1) is at the far left.
// The firmware never commands the setpoint just below background (m = fpi-1,
// e.g. 61439 for fpi=16), so it gets no column and the axis has no gap: the
// image is fpi-1 columns wide. Returns -1 for an off-grid (bit-flipped) voltage
// or that never-commanded setpoint, so the frame is dropped.
static int col_of(const view_t *v, const experiment_t *s, int k)
{
    int r = tv_row(s->fr_tv[k], v->fpi);      // 0..fpi-1 setpoint index, or -1
    if (r < 0) return -1;
    if (r == v->fpi - 1) return v->fpi - 2;   // background -> far right
    if (r == v->fpi - 2) return -1;           // never-commanded setpoint
    return r;                                  // 0..fpi-3 -> same column (left = most negative)
}

// Detect the setpoint count for the current experiment; fpi is the scan period
// (setpoints per sweep) and ncols the image width (one fewer, since the setpoint
// just below background is never commanded).
static void apply_scan(view_t *v, const experiment_t *s)
{
    v->n_targets = detect_n_targets(s);
    v->fpi = v->fpi_override ? v->fpi_override : v->n_targets;
    if (v->fpi < 2) v->fpi = 16;
    v->ncols = v->fpi - 1;
}

// Assign every frame to an image. Frames are in file order; the image boundary
// comes from the inner-dome scan index (the clean per-frame step counter) -- a
// new image begins each time the scan index returns to a multiple of fpi, i.e.
// at the start of a sweep -- while the column within the image comes from the
// target voltage (see col_of). Corrupt frames (an off-grid target voltage or a
// JSON-straddle body) and the partial leading sweep are dropped (fr_img < 0).
static void rebuild_image_list(view_t *v, const experiment_t *s)
{
    free(v->fr_img);
    v->fr_img = NULL; v->n_img = 0;
    if (s == NULL || s->nframes == 0) return;
    v->fr_img = (int *) malloc((size_t) s->nframes * sizeof(int));
    if (v->fr_img == NULL) return;

    int img = 0, started = 0;
    for (int k = 0; k < s->nframes; k++) {
        if (s->fr_bad[k] || col_of(v, s, k) < 0) { v->fr_img[k] = -2; continue; }   // corrupt frame
        int sc = s->fr_scan[k];
        int boundary = (sc >= 0 && sc < 4 * v->fpi && sc % v->fpi == 0);
        if (!started) {
            if (boundary) { started = 1; v->fr_img[k] = 0; }
            else v->fr_img[k] = -1;      // partial leading sweep, before the first boundary
            continue;
        }
        if (boundary) img++;
        v->fr_img[k] = img;
    }
    v->n_img = started ? img + 1 : 0;
    if (v->img_pos >= v->n_img) v->img_pos = v->n_img ? v->n_img - 1 : 0;
}

// Per-experiment DN extent over all present pixels (for auto-session scaling).
static void compute_session_extent(view_t *v, const experiment_t *s)
{
    int mn = 65535, mx = 0;
    for (int k = 0; k < s->nframes; k++) {
        long base = s->fr_off[k] + PIX_START;
        for (int j = 0; j < NPIX; j++) {
            int p = be16(s->buf, base + j * 2);
            if (p < mn) mn = p;
            if (p > mx) mx = p;
        }
    }
    if (mx <= mn) mx = mn + 1;
    v->sess_min = mn; v->sess_max = mx;
}

// Switch to a different experiment and refresh everything derived from it.
static void select_experiment(view_t *v, const experiment_t *s)
{
    v->img_pos = 0;
    v->playing = 0;
    apply_scan(v, s);
    rebuild_image_list(v, s);
    compute_session_extent(v, s);
}

// ---- TTF font (pattern from decode_inspector.c / ~/src/ved) ----------------
// Loads the bundled SourceCodePro-Regular.ttf (next to the binary, the project
// assets/ dir, or $XDG_DATA_HOME), falling back to raylib's bitmap default.

static Font  g_ui_font;
static int   g_ui_font_loaded = 0;
static float g_ui_font_spacing = 1.0f;

static int load_ui_font(void)
{
    const char *home = getenv("HOME");
    const char *xdg = getenv("XDG_DATA_HOME");
    char cands[5][1024];
    int n = 0;
    if (xdg && xdg[0])
        snprintf(cands[n++], sizeof cands[0], "%s/simple_sat_ops/SourceCodePro-Regular.ttf", xdg);
    if (home && home[0]) {
        snprintf(cands[n++], sizeof cands[0], "%s/.local/share/simple_sat_ops/SourceCodePro-Regular.ttf", home);
        snprintf(cands[n++], sizeof cands[0], "%s/src/simple_sat_ops/assets/SourceCodePro-Regular.ttf", home);
    }
    snprintf(cands[n++], sizeof cands[0], "assets/SourceCodePro-Regular.ttf");
    snprintf(cands[n++], sizeof cands[0], "../assets/SourceCodePro-Regular.ttf");

    // Load the printable-ASCII range at a large base size and let raylib scale
    // it down per draw with bilinear filtering, so small text stays crisp.
    int cp[95];
    for (int c = 0x20; c <= 0x7E; c++) cp[c - 0x20] = c;
    for (int i = 0; i < n; i++) {
        if (!FileExists(cands[i])) continue;
        g_ui_font = LoadFontEx(cands[i], 48, cp, 95);
        if (g_ui_font.texture.id != 0) {
            SetTextureFilter(g_ui_font.texture, TEXTURE_FILTER_BILINEAR);
            return 1;
        }
    }
    fprintf(stderr, "mpi_viewer: TTF font not found; using raylib default\n");
    return 0;
}

static void draw_text(const char *s, int x, int y, int size, Color c)
{
    if (g_ui_font_loaded)
        DrawTextEx(g_ui_font, s, (Vector2){ (float) x, (float) y }, (float) size, g_ui_font_spacing, c);
    else
        DrawText(s, x, y, size, c);
}

// Width of a string as draw_text would render it, for right-justifying.
static int text_width(const char *s, int size)
{
    if (g_ui_font_loaded)
        return (int) MeasureTextEx(g_ui_font, s, (float) size, g_ui_font_spacing).x;
    return MeasureText(s, size);
}

// Fire once on press, then rapidly while the key is held (after a short delay).
// *cooldown carries the time until the next repeat -- pass one float per key.
static int key_repeat(int key, float *cooldown)
{
    const float delay = 0.30f, rate = 0.03f;   // 0.3 s before autorepeat, then ~33/s
    if (IsKeyPressed(key)) { *cooldown = delay; return 1; }
    if (!IsKeyDown(key)) return 0;
    *cooldown -= GetFrameTime();
    if (*cooldown <= 0.0f) { *cooldown = rate; return 1; }
    return 0;
}

// ---- re-download telecommand export ----------------------------------------
//
// An experiment is reassembled from whatever came down, and what did not come
// down is a set of holes. This writes the telecommands that fetch exactly those
// holes, as a file the operator can hand to `simple_sat_ops --tc-file=`.
//
// The requests are aligned to the file's fixed 195-byte downlink packet grid
// (mirrors mpi_reconstruct's report, and the reasoning is the same). The file
// is a sequence of 195-byte packets from offset 0; a packet either arrived or
// failed its CRC, and you can only ask for whole packets. So a missing run is
// snapped OUT to packet boundaries -- start rounded down, end rounded up -- and
// every request offset and length is a multiple of 195 (a single missing byte
// pulls its one 195-byte packet). Requests for consecutive needed packets are
// coalesced into one command; a stretch of fully-received packets between two
// gaps is left alone. A request past the firmware's per-command cap is tiled
// into back-to-back whole-packet commands. The one length that can be a
// non-multiple of 195 is the command reaching the reconstructed file's
// genuinely partial last packet. (The chunks came from passes on different
// 195-byte grids, which is why the raw gaps land off-grid; the re-download uses
// the canonical grid.)

// Emit the request that covers [cs, ce), tiled so no single command exceeds the
// firmware's per-command cap. `tile` is a whole number of 195-byte packets, so
// every command's length is a multiple of 195 -- except the one that reaches
// the end. Pass f = NULL to count without writing. Accumulates the byte and
// command totals.
static void emit_run(FILE *f, const char *sat_path, long cs, long ce, long size,
                     long tile, long *download, long *n_cmds)
{
    if (ce > size) ce = size;
    for (long p = cs; p < ce; p += tile) {
        long q = (p + tile < ce) ? p + tile : ce;
        if (f != NULL)
            fprintf(f, "CTS1+exec_blob_from_fs(%s,0,%s;%ld;%ld)!\n",
                    MPI_BLOB_PATH, sat_path, p, q - p);
        *download += q - p;
        (*n_cmds)++;
    }
}

// Walk the experiment's holes and emit one command per coalesced run. Pass
// f = NULL to total the missing bytes, the bytes the commands would fetch and
// the command count without writing anything.
static void plan_redownload(FILE *f, const uint8_t *present, long size,
                            const char *sat_path,
                            long *missing, long *download, long *n_cmds)
{
    const long PKT = BULK_FILE_MAX_DATA;                         // 195 bytes per packet
    long tile = (COMMS_BULK_DOWNLINK_MAX_BYTES / PKT) * PKT;     // per-command cap
    if (tile < PKT) tile = PKT;

    *missing = 0; *download = 0; *n_cmds = 0;
    long run_start = -1;
    long cs = -1, ce = -1;   // pending request [cs, ce), snapped to the packet grid

    // Iterate one past the end so a gap that runs to EOF is closed.
    for (long i = 0; i <= size; i++) {
        int gap = (i < size) && !present[i];
        if (gap && run_start < 0) {
            run_start = i;
        } else if (!gap && run_start >= 0) {
            long a = run_start, b = i;
            *missing += b - a;
            long sa = (a / PKT) * PKT;                 // snap down to a packet start
            long sb = ((b + PKT - 1) / PKT) * PKT;     // snap up to a packet end
            if (sb > size) sb = size;                  // last packet is partial at EOF
            if (cs < 0) {
                cs = sa; ce = sb;
            } else if (sa <= ce) {
                // Same or adjacent packet(s) as the pending request: coalesce.
                // A fully-received packet in between makes sa > ce, so it splits.
                if (sb > ce) ce = sb;
            } else {
                emit_run(f, sat_path, cs, ce, size, tile, download, n_cmds);
                cs = sa; ce = sb;
            }
            run_start = -1;
        }
    }
    if (cs >= 0)
        emit_run(f, sat_path, cs, ce, size, tile, download, n_cmds);
}

// Write the selected experiment's re-download telecommands to the working
// directory. Puts the outcome, good or bad, in *status.
static void export_redownload(const experiment_t *s, const char *db_path,
                              char *status, size_t nstatus)
{
    // The satellite file name comes from the command log; without it the
    // commands still spell out the right offsets, so write them with a
    // placeholder for the operator to fill in rather than refusing.
    int have_path = s->sat_path[0] != '\0';
    const char *sat_path = have_path ? s->sat_path : "<file_path>";

    long missing = 0, download = 0, n_cmds = 0;
    plan_redownload(NULL, s->present, s->size, sat_path, &missing, &download, &n_cmds);
    if (n_cmds == 0) {
        if (s->size_from_log && s->size_exact)
            snprintf(status, nstatus,
                     "complete: all %ld bytes are down; no commands needed", s->size);
        else
            snprintf(status, nstatus,
                     "nothing missing below %ld bytes, but that is only as far as "
                     "the file is known to run; no commands needed", s->size);
        return;
    }

    char stamp[24];
    fmt_stamp(s->t_start_ms, stamp, sizeof stamp);
    char path[256];
    snprintf(path, sizeof path, "mpi_redownload_%s.txt", stamp);

    FILE *f = fopen(path, "w");
    if (f == NULL) { snprintf(status, nstatus, "cannot write %s", path); return; }

    char now[32];
    fmt_utc_s((double) time(NULL) * 1000.0, now, sizeof now);
    fprintf(f,
        "# Re-download telecommands for the MPI experiment recorded %s UTC.\n"
        "# Written by mpi_viewer at %s UTC from %s.\n"
        "#\n"
        "# on-satellite file : %s%s\n"
        "# file length       : %ld bytes (%s)\n"
        "# received          : %ld bytes (%.1f%%)\n"
        "# missing           : %ld bytes\n"
        "# these commands    : %ld, fetching %ld bytes\n"
        "#\n"
        "# Every request is a whole number of 195-byte downlink packets, so a\n"
        "# command can re-fetch a few bytes already in hand where a gap shares a\n"
        "# packet with received data. Runs longer than the firmware's %ld-byte\n"
        "# per-command limit are split.\n"
        "#\n",
        s->utc, now, db_path,
        sat_path, have_path ? "" : "   <-- NOT in the command log; fill this in",
        s->size,
        s->size_from_log
            ? (s->size_exact ? "the satellite's own byte count for this file"
                             : "the satellite's own byte count, a MINIMUM: every download"
                               " of this file so far was stopped by a byte cap")
            : "the largest offset received, a LOWER bound",
        s->recovered, 100.0 * (double) s->recovered / (double) s->size,
        missing, n_cmds, download,
        COMMS_BULK_DOWNLINK_MAX_BYTES);

    if (!s->size_from_log || !s->size_exact)
        fprintf(f,
            "# Nothing here proves the last byte above is the end of the file. To\n"
            "# find out, and to pick up any tail past it, send\n"
            "#   CTS1+exec_blob_from_fs(%s,0,%s;%ld;0)!\n"
            "# and let it run to the end; the satellite's \"Bulk downlink complete\"\n"
            "# log line then gives the length outright.\n"
            "#\n",
            MPI_BLOB_PATH, sat_path, s->size);

    if (s->dropped > 0)
        fprintf(f,
            "# %ld received packet(s) were set aside: their offset field does not\n"
            "# sit on a download grid, or points past the end of the file, so it\n"
            "# was corrupted in flight. The bytes they claimed are listed below as\n"
            "# missing, which they are.\n"
            "#\n",
            s->dropped);

    fprintf(f,
        "# Run this through agenda_check before flying it. exec_blob_from_fs is\n"
        "# marked recovery/expert in the firmware, so every line draws a readiness\n"
        "# warning; warnings never block startup.\n"
        "\n");

    long m2 = 0, d2 = 0, c2 = 0;
    plan_redownload(f, s->present, s->size, sat_path, &m2, &d2, &c2);

    int bad = ferror(f);
    if (fclose(f) != 0 || bad) {
        snprintf(status, nstatus, "write failed on %s", path);
        return;
    }
    snprintf(status, nstatus, "wrote %s: %ld command%s, %ld bytes%s",
             path, n_cmds, n_cmds == 1 ? "" : "s", download,
             have_path ? "" : " (file path is a placeholder)");
}

// ---- data-coverage patch ---------------------------------------------------
// The reconstructed file drawn as a patch of pixels: byte 0 at the top left,
// the last byte at the bottom right, reading left to right then down. Green
// where a byte came down, black where it never did. One pixel stands for a
// slice of the file and goes black as soon as any byte in its slice is missing,
// so even a one-packet hole is visible.

static void build_coverage(const experiment_t *s, Color *pix, int w, int h)
{
    long ncell = (long) w * (long) h;
    for (long i = 0; i < ncell; i++) {
        long b0 = (long) ((double) s->size * (double) i / (double) ncell);
        long b1 = (long) ((double) s->size * (double) (i + 1) / (double) ncell);
        if (b1 <= b0) b1 = b0 + 1;
        if (b1 > s->size) b1 = s->size;
        int missing = 0, unverified = 0;
        for (long b = b0; b < b1; b++) {
            if (!s->present[b]) { missing = 1; break; }
            if (!s->verified[b]) unverified = 1;
        }
        // Amber for a slice that is all there but rests on packets whose CRC
        // failed: the bytes are readable, nothing says they are right.
        pix[i] = missing      ? (Color){ 8, 8, 10, 255 }
               : unverified   ? (Color){ 206, 150, 42, 255 }
                              : (Color){ 89, 184, 85, 255 };
    }
}

// The byte range the currently shown image was reconstructed from, so the
// coverage map can mark where in the file you are looking. Returns 0 if the
// image has no frames.
static int image_byte_span(const view_t *v, const experiment_t *s, int img,
                           long *out_b0, long *out_b1)
{
    long b0 = -1, b1 = -1;
    for (int k = 0; k < s->nframes; k++) {
        if (v->fr_img == NULL || v->fr_img[k] != img) continue;
        long o = s->fr_off[k];
        if (b0 < 0 || o < b0) b0 = o;
        if (b1 < 0 || o + FRAME_STRIDE > b1) b1 = o + FRAME_STRIDE;
    }
    if (b0 < 0) return 0;
    if (b1 > s->size) b1 = s->size;
    *out_b0 = b0; *out_b1 = b1;
    return 1;
}

// The image whose bytes sit closest to file offset `byte`, for click-to-seek on
// the coverage map. Returns -1 if there are no images.
static int image_nearest_byte(const view_t *v, const experiment_t *s, long byte)
{
    int best = -1;
    long best_d = 0;
    for (int k = 0; k < s->nframes; k++) {
        if (v->fr_img == NULL || v->fr_img[k] < 0) continue;
        long o = s->fr_off[k];
        long d = byte < o ? o - byte
               : byte >= o + FRAME_STRIDE ? byte - (o + FRAME_STRIDE - 1) : 0;
        if (best < 0 || d < best_d) { best = v->fr_img[k]; best_d = d; }
        if (d == 0) break;
    }
    return best;
}

// ---- main ------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "mpi_viewer")) return 0;

    const char *db_arg = NULL;
    int list_only = 0;
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--db=", 5) == 0) db_arg = argv[i] + 5;
        else if (strcmp(argv[i], "--list") == 0) list_only = 1;
        else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: mpi_viewer [--db=<packet_db.sqlite>] [--list]\n"
                   "Inspect MPI science imagery reconstructed from the packet DB.\n"
                   "The left panel lists MPI experiments; F5 re-reads the DB.\n"
                   "Press d to write the selected experiment's missing data as\n"
                   "re-download telecommands, for simple_sat_ops --tc-file.\n"
                   "--list prints what each experiment reconstructed to and exits,\n"
                   "without opening a window.\n");
            return 0;
        } else {
            fprintf(stderr, "mpi_viewer: unknown option '%s' (try --help)\n", argv[i]);
            return 1;
        }
    }

    char db_default[1024];
    const char *db_path = db_arg;
    if (db_path == NULL) {
        if (packet_db_default_path(db_default, sizeof db_default) != 0) {
            fprintf(stderr, "mpi_viewer: no DB path (set $SSO_PACKET_DB or pass --db=)\n");
            return 1;
        }
        db_path = db_default;
    }

    fprintf(stderr, "mpi_viewer: reconstructing MPI experiments from %s ...\n", db_path);
    experiment_t *exps = NULL;
    int nexp = reload_experiments(db_path, &exps);
    if (nexp == 0) {
        fprintf(stderr, "mpi_viewer: no MPI science-data experiments found.\n");
        return 1;
    }
    fprintf(stderr, "mpi_viewer: %d MPI experiment%s.\n", nexp, nexp == 1 ? "" : "s");

    if (list_only) {
        for (int k = 0; k < nexp; k++) {
            const experiment_t *e = &exps[k];
            printf("%s UTC  %s  (%.1f min)\n", e->utc,
                   e->sat_path[0] ? e->sat_path : "(file not in the command log)",
                   (e->t_end_ms - e->t_start_ms) / 60000.0);
            printf("  file      : %ld bytes (%s)\n", e->size,
                   e->size_from_log
                       ? (e->size_exact ? "satellite-reported length"
                                        : "satellite-reported minimum")
                       : "largest offset received -- a lower bound");
            printf("  recovered : %ld bytes (%.1f%%), %ld missing\n",
                   e->recovered, 100.0 * (double) e->recovered / (double) e->size,
                   e->size - e->recovered);
            printf("  unverified: %ld bytes from packets whose CRC failed\n", e->unverified);
            printf("  frames    : %d", e->nframes);
            int nunver = 0;
            for (int f = 0; f < e->nframes; f++) if (e->fr_unver[f]) nunver++;
            if (nunver > 0) printf(" (%d touch unverified bytes)", nunver);
            printf("\n");
            if (e->dropped > 0)
                printf("  dropped   : %ld packet(s) with an off-grid or past-the-end offset\n",
                       e->dropped);
        }
        free_experiments(exps, nexp);
        return 0;
    }

    view_t v = {0};
    v.zoom = 8; v.ips = 8.0f;
    v.scale_mode = SCALE_AUTO_IMAGE;
    v.dn_min = 1800; v.dn_max = 2300;
    select_experiment(&v, &exps[v.sel]);

    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "mpi_viewer");
    SetTargetFPS(60);
    g_ui_font_loaded = load_ui_font();

    // A fixed grayscale texture holding the image rotated 90 CW: the scan rows
    // become columns (width MAX_FPI) and the 65 pixels become rows (height
    // NPIX). We fill the top-left fpi x NPIX region and let raylib scale it with
    // nearest-neighbour (POINT) filtering.
    unsigned char graybuf[MAX_FPI * NPIX] = {0};
    Image gimg = { .data = graybuf, .width = MAX_FPI, .height = NPIX,
                   .mipmaps = 1, .format = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE };
    Texture2D tex = LoadTextureFromImage(gimg);
    SetTextureFilter(tex, TEXTURE_FILTER_POINT);

    const int LEFT_W = 340;

    // Per-key autorepeat timers for the arrow keys (files + image scrubber) and
    // the manual DN-range keys.
    float rep_up = 0, rep_down = 0, rep_left = 0, rep_right = 0;
    float rep_z = 0, rep_x = 0, rep_c = 0, rep_v = 0;

    // Outcome of the last d export, shown for a few seconds.
    char  status[200] = "";
    float status_left = 0.0f;

    // Data-coverage map: one texture pixel per slice of the reconstructed file,
    // rebuilt when the experiment changes or the patch is resized.
    Color    *cov_pix = NULL;
    Texture2D cov_tex = {0};
    int cov_w = 0, cov_h = 0, cov_sel = -1;
    // Set while the pointer is scrubbing the coverage map, so a press that
    // began there keeps control until the button is let go -- and a press that
    // began anywhere else never takes it.
    int cov_drag = 0;

    while (!WindowShouldClose()) {
        experiment_t *s = &exps[v.sel];

        // ---- input ----
        // Letting the button go ends a coverage-map scrub. Checked here rather
        // than beside the map itself, which is not drawn at all in a window too
        // small for it -- a scrub must not survive that.
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) cov_drag = 0;
        if (key_repeat(KEY_DOWN, &rep_down) && v.sel < nexp - 1) { v.sel++; select_experiment(&v, &exps[v.sel]); s = &exps[v.sel]; }
        if (key_repeat(KEY_UP, &rep_up)     && v.sel > 0)        { v.sel--; select_experiment(&v, &exps[v.sel]); s = &exps[v.sel]; }
        if (key_repeat(KEY_RIGHT, &rep_right) && v.img_pos < v.n_img - 1) v.img_pos++;
        if (key_repeat(KEY_LEFT, &rep_left)   && v.img_pos > 0)           v.img_pos--;
        if (IsKeyPressed(KEY_SPACE)) v.playing = !v.playing;
        if (IsKeyPressed(KEY_COMMA)  && v.ips > 1.0f)  v.ips -= 1.0f;
        if (IsKeyPressed(KEY_PERIOD) && v.ips < 60.0f) v.ips += 1.0f;
        if (IsKeyPressed(KEY_F)) {
            v.fpi_override = v.fpi_override == 0 ? 8 : v.fpi_override == 8 ? 16 : 0;
            apply_scan(&v, s);
            rebuild_image_list(&v, s);
        }
        if (IsKeyPressed(KEY_S)) { v.zoom = v.zoom == 2 ? 4 : v.zoom == 4 ? 8 : v.zoom == 8 ? 16 : 2; }
        if (IsKeyPressed(KEY_A)) v.scale_mode = (v.scale_mode + 1) % 3;
        if (IsKeyPressed(KEY_D)) {
            export_redownload(s, db_path, status, sizeof status);
            status_left = 8.0f;
        }
        if (IsKeyPressed(KEY_R)) v.scale_mode = SCALE_AUTO_IMAGE;
        if (v.scale_mode == SCALE_MANUAL) {
            int step = IsKeyDown(KEY_LEFT_SHIFT) ? 500 : 100;
            if (key_repeat(KEY_Z, &rep_z)) v.dn_min -= step;
            if (key_repeat(KEY_X, &rep_x)) v.dn_min += step;
            if (key_repeat(KEY_C, &rep_c)) v.dn_max -= step;
            if (key_repeat(KEY_V, &rep_v)) v.dn_max += step;
            if (v.dn_min < 0) v.dn_min = 0;
            if (v.dn_max > 65535) v.dn_max = 65535;
            if (v.dn_max <= v.dn_min) v.dn_max = v.dn_min + 1;
        }
        // F5: re-read the DB and rebuild the experiment list, preserving the
        // current selection, image position and view settings where possible.
        if (IsKeyPressed(KEY_F5)) {
            int save_sel = v.sel, save_img = v.img_pos, save_play = v.playing;
            experiment_t *ne = NULL;
            int nn = reload_experiments(db_path, &ne);
            if (nn > 0) {
                free_experiments(exps, nexp);
                exps = ne; nexp = nn;
                if (save_sel >= nexp) save_sel = nexp - 1;
                v.sel = save_sel;
                select_experiment(&v, &exps[v.sel]);
                if (save_img >= v.n_img) save_img = v.n_img ? v.n_img - 1 : 0;
                v.img_pos = save_img;
                v.playing = save_play;
                s = &exps[v.sel];
                cov_sel = -1;   // the reloaded experiment needs a fresh coverage map
            } else if (ne != NULL) {
                free_experiments(ne, nn);
            }
        }
        if (IsKeyPressed(KEY_Q)) break;
        if (status_left > 0.0f) status_left -= GetFrameTime();

        // ---- playback ----
        if (v.playing && v.n_img > 0) {
            v.accum += GetFrameTime();
            if (v.accum >= 1.0f / v.ips) {
                v.accum = 0;
                v.img_pos++;
                if (v.img_pos >= v.n_img) v.img_pos = 0;   // loop
            }
        }

        // ---- resolve the current image and its DN window ----
        int lo = v.dn_min, hi = v.dn_max;
        if (v.scale_mode == SCALE_AUTO_SESSION) { lo = v.sess_min; hi = v.sess_max; }
        else if (v.scale_mode == SCALE_AUTO_IMAGE) {
            int mn = 65535, mx = 0;
            for (int k = 0; k < s->nframes; k++) {
                if (v.fr_img == NULL || v.fr_img[k] != v.img_pos) continue;
                long base = s->fr_off[k] + PIX_START;
                for (int j = 0; j < NPIX; j++) {
                    int p = be16(s->buf, base + j * 2);
                    if (p < mn) mn = p;
                    if (p > mx) mx = p;
                }
            }
            if (mx <= mn) mx = mn + 1;
            lo = mn; hi = mx;
        }

        // ---- rasterise the image into the grayscale texture ----
        memset(graybuf, 0, sizeof graybuf);
        double frame_time = -1.0;
        int rep_frame = -1, cols_present = 0;
        for (int k = 0; k < s->nframes; k++) {
            if (v.fr_img == NULL || v.fr_img[k] != v.img_pos) continue;
            int col = col_of(&v, s, k);
            if (col < 0 || col >= v.ncols) continue;
            cols_present++;
            if (rep_frame < 0) rep_frame = k;
            if (s->fr_ts[k] >= 0 && (frame_time < 0 || s->fr_ts[k] < frame_time)) frame_time = s->fr_ts[k];
            long base = s->fr_off[k] + PIX_START;
            for (int j = 0; j < NPIX; j++) {
                int p = be16(s->buf, base + j * 2);
                int g = p <= lo ? 0 : p >= hi ? 255 : (int) (255.0 * (p - lo) / (hi - lo));
                // Column = bias voltage (background right, most negative left); pixel -> row.
                graybuf[j * MAX_FPI + col] = (unsigned char) g;
            }
        }
        UpdateTexture(tex, graybuf);

        // ---- draw ----
        BeginDrawing();
        ClearBackground((Color){ 18, 18, 22, 255 });

        // left: experiment list
        DrawRectangle(0, 0, LEFT_W, GetScreenHeight(), (Color){ 28, 28, 34, 255 });
        draw_text("MPI experiments", 12, 10, 18, RAYWHITE);
        draw_text(TextFormat("%d run%s", nexp, nexp == 1 ? "" : "s"), LEFT_W - 78, 14, 12, GRAY);
        int row_h = 44, list_top = 40;
        int visible = (GetScreenHeight() - list_top) / row_h;
        int top = 0;
        if (v.sel >= visible) top = v.sel - visible + 1;
        for (int r = 0; r < visible && top + r < nexp; r++) {
            int si = top + r;
            experiment_t *ss = &exps[si];
            int y = list_top + r * row_h;
            if (si == v.sel) DrawRectangle(0, y, LEFT_W, row_h, (Color){ 44, 70, 110, 255 });
            double pct = 100.0 * (double) ss->recovered / (double) ss->size;
            draw_text(TextFormat("%s UTC", ss->utc), 10, y + 4, 15, si == v.sel ? RAYWHITE : LIGHTGRAY);
            // How long the MPI actually ran, right-justified on the date line.
            double mins = (ss->t_end_ms - ss->t_start_ms) / 60000.0;
            if (mins > 0) {
                const char *dur = TextFormat("%.1f min", mins);
                draw_text(dur, LEFT_W - 10 - text_width(dur, 12), y + 7, 12,
                          si == v.sel ? LIGHTGRAY : GRAY);
            }
            draw_text(TextFormat("%.0f KB  %.0f%%  %d frames",
                                 ss->size / 1024.0, pct, ss->nframes),
                      10, y + 23, 12, GRAY);
        }

        // Was any part of this image built from packets whose CRC failed?
        int img_unver = 0, img_unver_frames = 0, img_frames = 0;
        for (int k = 0; k < s->nframes; k++) {
            if (v.fr_img == NULL || v.fr_img[k] != v.img_pos) continue;
            img_frames++;
            if (s->fr_unver[k]) { img_unver = 1; img_unver_frames++; }
        }

        // right: title / time
        int rx = LEFT_W + 20, ry = 14;
        draw_text(TextFormat("Image %d / %d", v.n_img ? v.img_pos + 1 : 0, v.n_img), rx, ry, 20, RAYWHITE);
        char tstr[48] = "no timestamp";
        if (frame_time >= 0) fmt_utc_ms(frame_time, tstr, sizeof tstr);
        draw_text(TextFormat("%s UTC", tstr), rx, ry + 26, 20, (Color){ 120, 220, 160, 255 });

        // the image
        int iy = ry + 64;
        // On screen the width is the bias-voltage columns, the height the pixels.
        int dw = v.ncols * v.zoom, dh = NPIX * v.zoom;
        Rectangle src = { 0, 0, (float) v.ncols, (float) NPIX };
        Rectangle dst = { (float) rx, (float) iy, (float) dw, (float) dh };
        DrawRectangleLines(rx - 1, iy - 1, dw + 2, dh + 2,
                           img_unver ? (Color){ 206, 150, 42, 255 } : (Color){ 70, 70, 80, 255 });
        DrawTexturePro(tex, src, dst, (Vector2){ 0, 0 }, 0.0f, WHITE);
        if (img_unver)
            draw_text(TextFormat("UNVERIFIED  %d of %d frames rest on packets whose CRC failed",
                                 img_unver_frames, img_frames),
                      rx, iy + dh + 8, 14, (Color){ 206, 150, 42, 255 });

        // aux panel to the right of the image
        int ax = rx + dw + 30;
        if (ax < LEFT_W + 360) ax = LEFT_W + 360;
        int ay = iy;
        draw_text("Aux data", ax, ay, 18, RAYWHITE); ay += 26;
        if (rep_frame >= 0) {
            long o = s->fr_off[rep_frame];
            draw_text(TextFormat("frame counter : %d", be16(s->buf, o + 4)),  ax, ay, 15, LIGHTGRAY); ay += 20;
            draw_text(TextFormat("temperature   : %d (raw ADC)", be16(s->buf, o + 6)), ax, ay, 15, LIGHTGRAY); ay += 20;
            draw_text(TextFormat("firmware ver  : %d", s->buf[o + 8]),        ax, ay, 15, LIGHTGRAY); ay += 20;
            draw_text(TextFormat("camera CCD ADC: %d", be16(s->buf, o + 9)),  ax, ay, 15, LIGHTGRAY); ay += 20;
            draw_text(TextFormat("dome target V : %d", be16(s->buf, o + 11)), ax, ay, 15, LIGHTGRAY); ay += 20;
            draw_text(TextFormat("scan index    : %d", s->buf[o + 13]),       ax, ay, 15, LIGHTGRAY); ay += 20;
            draw_text(TextFormat("dome ADC read : %d", be16(s->buf, o + 14)), ax, ay, 15, LIGHTGRAY); ay += 20;
            draw_text(TextFormat("pixels        : %d .. %d", s->buf[o + 16], s->buf[o + 17]), ax, ay, 15, LIGHTGRAY); ay += 20;
            draw_text(TextFormat("integration   : %d", be16(s->buf, o + 18)), ax, ay, 15, LIGHTGRAY); ay += 26;
        } else { ay += 20; }
        draw_text(TextFormat("experiment    : %s UTC", s->utc), ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("satellite file: %s", s->sat_path[0] ? s->sat_path : "unknown"),
                  ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("targets/sweep : %d", v.n_targets), ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("columns/image : %d %s  (filled %d)",
                             v.ncols, v.fpi_override ? "(fixed)" : "(auto)", cols_present),
                  ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("zoom          : x%d", v.zoom), ax, ay, 15, LIGHTGRAY); ay += 20;
        const char *sm = v.scale_mode == SCALE_AUTO_IMAGE ? "auto (image)"
                       : v.scale_mode == SCALE_AUTO_SESSION ? "auto (experiment)" : "manual";
        draw_text(TextFormat("color scale   : %s", sm), ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("DN window     : %d .. %d", lo, hi), ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("playback      : %s  %.0f img/s", v.playing ? "PLAY" : "paused", v.ips),
                  ax, ay, 15, v.playing ? (Color){ 120, 220, 160, 255 } : GRAY); ay += 20;

        // data-coverage patch, under the aux panel: the whole reconstructed
        // file, green where the bytes came down and black where they never did
        ay += 8;
        draw_text("Data coverage", ax, ay, 18, RAYWHITE); ay += 24;
        int cw = GetScreenWidth() - ax - 20;
        if (cw > 340) cw = 340;
        int ch = GetScreenHeight() - 68 - ay;
        if (cw >= 40 && ch >= 24) {
            if (cw != cov_w || ch != cov_h) {
                Color *np = (Color *) realloc(cov_pix, (size_t) cw * (size_t) ch * sizeof *cov_pix);
                if (np != NULL) {
                    cov_pix = np; cov_w = cw; cov_h = ch;
                    if (cov_tex.id != 0) UnloadTexture(cov_tex);
                    build_coverage(s, cov_pix, cov_w, cov_h);
                    cov_sel = v.sel;
                    Image ci = { .data = cov_pix, .width = cov_w, .height = cov_h,
                                 .mipmaps = 1, .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
                    cov_tex = LoadTextureFromImage(ci);
                }
            } else if (cov_sel != v.sel) {
                build_coverage(s, cov_pix, cov_w, cov_h);
                cov_sel = v.sel;
                UpdateTexture(cov_tex, cov_pix);
            }
            if (cov_tex.id != 0) {
                DrawTexture(cov_tex, ax, ay, WHITE);
                DrawRectangleLines(ax - 1, ay - 1, cov_w + 2, cov_h + 2, (Color){ 70, 70, 80, 255 });

                // Playhead: the cells holding the bytes the shown image was
                // built from. The map reads left to right then down, so the
                // run wraps the way a line of selected text does -- part of a
                // row, then whole rows, then part of a row -- and is drawn that
                // way, one filled segment per row. A row of the map is one
                // pixel tall, so outlining the band instead would draw the band
                // plus a line above and a line below it that mark nothing.
                long pb0 = 0, pb1 = 0;
                if (v.n_img > 0 && image_byte_span(&v, s, v.img_pos, &pb0, &pb1)) {
                    long ncell = (long) cov_w * (long) cov_h;
                    long c0 = pb0 * ncell / s->size;
                    long c1 = (pb1 * ncell + s->size - 1) / s->size;
                    if (c1 <= c0) c1 = c0 + 1;
                    if (c1 > ncell) c1 = ncell;
                    int r0 = (int) (c0 / cov_w), r1 = (int) ((c1 - 1) / cov_w);
                    int x0 = (int) (c0 % cov_w), x1 = (int) ((c1 - 1) % cov_w) + 1;
                    for (int r = r0; r <= r1; r++) {
                        int a = (r == r0) ? x0 : 0;
                        int b = (r == r1) ? x1 : cov_w;
                        // An image can cover fewer cells than it takes to see.
                        if (b - a < 3) b = a + 3;
                        if (b > cov_w) { b = cov_w; a = cov_w - 3; }
                        if (a < 0) a = 0;
                        DrawRectangle(ax + a, ay + r, b - a, 1, RAYWHITE);
                    }
                }

                draw_text(TextFormat("%.1f%% of %.0f KB down, %.0f KB missing  (%s)",
                                     100.0 * (double) s->recovered / (double) s->size,
                                     s->size / 1024.0, (s->size - s->recovered) / 1024.0,
                                     s->size_from_log
                                         ? (s->size_exact ? "length from the satellite"
                                                          : "length is a satellite-reported minimum")
                                         : "length is the largest offset seen"),
                          ax, ay + cov_h + 6, 13, GRAY);
                if (s->unverified > 0)
                    draw_text(TextFormat("amber: %.0f KB rest on packets whose CRC failed",
                                         s->unverified / 1024.0),
                              ax, ay + cov_h + 22, 13, (Color){ 206, 150, 42, 255 });
            }

            // Press the map to jump to the image nearest that point in the
            // file, and keep dragging to scrub through the experiment. Only a
            // press that landed on the map starts a scrub; once it has, the
            // pointer is clamped to the map, so wandering off an edge keeps
            // scrubbing along that edge instead of stopping dead.
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                Vector2 m = GetMousePosition();
                if (m.x >= ax && m.x < ax + cov_w && m.y >= ay && m.y < ay + cov_h)
                    cov_drag = 1;
            }
            if (cov_drag && v.n_img > 0) {
                Vector2 m = GetMousePosition();
                int mx = (int) m.x - ax, my = (int) m.y - ay;
                if (mx < 0) mx = 0;
                if (mx > cov_w - 1) mx = cov_w - 1;
                if (my < 0) my = 0;
                if (my > cov_h - 1) my = cov_h - 1;
                long cell = (long) my * cov_w + mx;
                long byte = cell * s->size / ((long) cov_w * (long) cov_h);
                int img = image_nearest_byte(&v, s, byte);
                if (img >= 0) { v.img_pos = img; v.playing = 0; }
            }
        }

        // the d-export outcome, above the help footer
        if (status_left > 0.0f)
            draw_text(status, 12, GetScreenHeight() - 42, 14, (Color){ 120, 220, 160, 255 });

        // help footer
        const char *help =
            "Up/Down experiment   Left/Right image   click or drag coverage map"
            "   Space play/pause   ,/. speed   f steps/sweep   s zoom   a scale"
            "   z/x min  c/v max   d re-download commands   F5 refresh  q quit";
        draw_text(help, 12, GetScreenHeight() - 22, 12, (Color){ 150, 150, 160, 255 });

        EndDrawing();
    }

    UnloadTexture(tex);
    if (cov_tex.id != 0) UnloadTexture(cov_tex);
    if (g_ui_font_loaded) UnloadFont(g_ui_font);
    CloseWindow();

    free_experiments(exps, nexp);
    free(v.fr_img);
    free(cov_pix);
    return 0;
}

#endif // WITH_SQLITE3
