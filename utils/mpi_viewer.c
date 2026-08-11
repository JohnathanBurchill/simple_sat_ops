/*

    Simple Satellite Operations  utils/mpi_viewer.c

    A small raylib GUI for inspecting MPI science imagery straight out of the
    packet DB. It reconstructs every MPI download the same way mpi_reconstruct
    does (bulk_file chunks -> time-clustered sessions -> reassembled by offset),
    then finds the 152-byte data frames by searching for the sync word.

    Frame finding (matches First_light_JKB_20260722.nb): the reconstructed byte
    stream is NOT a clean fixed 152-byte grid -- packets from passes on different
    offset grids leave the frame alignment drifting, so a global grid catches
    only the handful of frames whose sync happens to land on it. Instead we
    locate EVERY occurrence of the sync word 0C FF FF 0C; each one starts a
    152-byte frame wherever it sits. This recovers all the received frames, not
    just the sync-aligned few.

    Each frame carries aux/housekeeping fields then a strip of image pixels. The
    inner-dome scan index (aux byte 13) is the frame's row within an image: one
    full inner-dome sweep is one image, and its length (the scan period, 16 for
    sawtooth / 32 for down-up) is auto-detected per download. The tool plays the
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

    Read-only on the DB.

    Usage:
      mpi_viewer [--db=<packet_db.sqlite>]

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
#define BULK_FILE_MAX_PLAUSIBLE (2 * 1024 * 1024)

// MPI science-data heuristics (mirror mpi_reconstruct.c / the firmware).
#define MPI_MIN_FILE_BYTES  20000
#define MPI_SYNC_WORD       "\x0C\xFF\xFF\x0C"
#define MPI_SYNC_WORD_LEN   4
#define MPI_MIN_SYNC        8
#define MPI_SESSION_GAP_MS  (15.0 * 60.0 * 1000.0)
#define MPI_SESSION_MARGIN_MS (120.0 * 1000.0)

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
    long     dlen;
    int      has_sync;
    uint8_t *data;
    char     ts_iso[32];
} chunk_t;

typedef struct {
    char     utc[24];        // "2026-08-09 22:08:04" display time (first frame)
    uint8_t *buf;            // reconstructed bytes
    uint8_t *present;        // 1 where a real byte landed
    long     size;
    long     recovered;      // present bytes
    int      nframes;        // number of sync words found
    long    *fr_off;         // byte offset of each frame's sync word
    int     *fr_ctr;         // running counter (bytes 4-5) per frame
    int     *fr_scan;        // inner dome scan index (byte 13) per frame
    int     *fr_tv;          // inner dome target voltage (bytes 11-12) per frame
    double  *fr_ts;          // reception time (unix ms) per frame, -1 if unknown
} session_t;

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

// Load every bulk_file chunk with a sane offset, oldest first (mirrors
// mpi_reconstruct's loader). Returns 0 on success.
static int load_chunks(sqlite3 *db, chunk_t **out, int *out_n)
{
    const char *sql =
        "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, "
        "       ts_received, payload, rs_errs "
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
        long dlen = pl_len - BULK_FILE_HEADER_SIZE;
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

// Reassemble the chunks named by idx[0..nidx) by absolute offset (mirrors
// mpi_reconstruct). RS-clean chunks win; uncorrectable chunks only fill gaps.
static long reassemble(const chunk_t *cs, const int *idx, int nidx,
                       uint8_t **out_buf, uint8_t **out_present)
{
    long size = 0;
    for (int j = 0; j < nidx; j++) {
        const chunk_t *c = &cs[idx[j]];
        if (c->off + c->dlen > size) size = c->off + c->dlen;
    }
    if (size <= 0) return -1;

    uint8_t *buf     = (uint8_t *) malloc((size_t) size);
    uint8_t *present = (uint8_t *) calloc((size_t) size, 1);
    if (buf == NULL || present == NULL) { free(buf); free(present); return -1; }
    memset(buf, 0, (size_t) size);

    for (int phase = 0; phase < 2; phase++) {
        for (int j = 0; j < nidx; j++) {
            const chunk_t *c = &cs[idx[j]];
            int clean = (c->rs >= 0);
            if ((phase == 0) != clean) continue;
            long off = c->off;
            for (long k = 0; k < c->dlen && off + k < size; k++) {
                if (present[off + k] && phase == 1) continue;
                buf[off + k] = c->data[k];
                present[off + k] = 1;
            }
        }
    }
    *out_buf = buf; *out_present = present;
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

// "2026-08-09 22:08:04" from an ISO ts_received ("2026-08-09T22:08:04.123Z").
static void utc_from_iso(const char *iso, char *out, size_t outn)
{
    char d[15] = "";
    int nd = 0;
    for (const char *p = iso; *p && nd < 14; p++)
        if (*p >= '0' && *p <= '9') d[nd++] = *p;
    if (nd >= 14)
        snprintf(out, outn, "%.4s-%.2s-%.2s %.2s:%.2s:%.2s",
                 d, d + 4, d + 6, d + 8, d + 10, d + 12);
    else
        snprintf(out, outn, "unknown");
}

// First index i in offs[0..n) with offs[i] >= x (offs sorted ascending).
static int lower_bound(const long *offs, int n, long x)
{
    int lo = 0, hi = n;
    while (lo < hi) { int mid = (lo + hi) / 2; if (offs[mid] < x) lo = mid + 1; else hi = mid; }
    return lo;
}

// Build a session from one reassembled burst, or return 0 if it is not MPI.
static int build_session(const chunk_t *cs, const int *idx, int nidx,
                         int first_chunk, session_t *out)
{
    uint8_t *buf = NULL, *present = NULL;
    long size = reassemble(cs, idx, nidx, &buf, &present);
    if (size < MPI_MIN_FILE_BYTES || count_sync(buf, size) < MPI_MIN_SYNC) {
        free(buf); free(present);
        return 0;
    }

    // Find every sync word; each one starts a 152-byte frame (drop any whose
    // frame would run past EOF). This is the notebook's SequencePosition step.
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
    if (n < MPI_MIN_SYNC) { free(offs); free(buf); free(present); return 0; }

    memset(out, 0, sizeof *out);
    out->buf = buf; out->present = present; out->size = size;
    out->nframes = n;
    out->fr_off = offs;
    out->fr_ctr  = (int *) malloc((size_t) n * sizeof(int));
    out->fr_scan = (int *) malloc((size_t) n * sizeof(int));
    out->fr_tv   = (int *) malloc((size_t) n * sizeof(int));
    out->fr_ts   = (double *) malloc((size_t) n * sizeof(double));
    if (out->fr_ctr == NULL || out->fr_scan == NULL || out->fr_tv == NULL || out->fr_ts == NULL) {
        free(out->fr_ctr); free(out->fr_scan); free(out->fr_tv); free(out->fr_ts);
        free(offs); free(buf); free(present);
        return 0;
    }
    for (int k = 0; k < n; k++) {
        long o = offs[k];
        out->fr_ctr[k]  = be16(buf, o + 4);
        out->fr_scan[k] = buf[o + SCAN_IDX_OFF];
        out->fr_tv[k]   = be16(buf, o + 11);
        out->fr_ts[k]   = -1.0;
    }

    long recovered = 0;
    for (long b = 0; b < size; b++) if (present[b]) recovered++;
    out->recovered = recovered;

    // Per-frame reception time: the earliest chunk whose data covers the sync
    // offset. offs is sorted, so binary-search the frames each chunk touches.
    for (int j = 0; j < nidx; j++) {
        const chunk_t *c = &cs[idx[j]];
        int klo = lower_bound(offs, n, c->off);
        for (int k = klo; k < n && offs[k] < c->off + c->dlen; k++)
            if (out->fr_ts[k] < 0 || c->ts_ms < out->fr_ts[k]) out->fr_ts[k] = c->ts_ms;
    }

    utc_from_iso(cs[first_chunk].ts_iso, out->utc, sizeof out->utc);
    return 1;
}

// Cluster the chunks into MPI download sessions (mirrors mpi_reconstruct's
// session walk) and build one session_t per MPI file. Returns the count.
static int load_sessions(sqlite3 *db, session_t **out)
{
    chunk_t *cs = NULL; int n = 0;
    if (load_chunks(db, &cs, &n) != 0 || n == 0) { free(cs); *out = NULL; return 0; }

    int *idx = (int *) malloc((size_t) n * sizeof(int));
    session_t *sess = NULL; int nsess = 0, cap = 0;
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
        int nidx = 0, first = -1;
        for (int k = 0; k < n; k++) {
            if (cs[k].ts_ms < wlo || cs[k].ts_ms > whi) continue;
            idx[nidx++] = k;
            if (first < 0) first = k;
        }
        i = last_i + 1;

        session_t s;
        if (build_session(cs, idx, nidx, first, &s)) {
            if (nsess == cap) {
                cap = cap ? cap * 2 : 16;
                sess = (session_t *) realloc(sess, (size_t) cap * sizeof *sess);
            }
            sess[nsess++] = s;
        }
    }

done:
    free(idx);
    for (int k = 0; k < n; k++) free(cs[k].data);
    free(cs);
    *out = sess;
    return nsess;
}

// ---- target-voltage grouping -----------------------------------------------
//
// The frame's row is set by the inner dome TARGET voltage (aux bytes 11-12), a
// commanded setpoint that steps systematically through N = 8 or 16 values,
// setpoint m = m*(65536/N) - 1 for m = 1..N (so 4095, 8191 .. 65535 for N=16;
// 65535 is the periodic 0 V background frame). Because the real values sit on
// that coarse grid, a bit-flip lands off-grid and is rejected -- unlike the raw
// scan-index byte where any 0..31 looks valid. The running counter is useless
// (~40% flipped, not monotonic), so grouping uses the target voltage plus the
// scan index only to mark image boundaries.

// Setpoint index (row, 0..n-1) of a target voltage on the n-step grid, or -1 if
// the value is off-grid (a corrupted byte).
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
static int detect_n_targets(const session_t *s)
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
    int    sel;            // selected session
    int    img_pos;        // position within the selected session's image list
    int   *fr_img;         // per-frame image index (<0 = dropped / corrupt frame)
    int    n_img;
    int    n_targets;      // detected setpoints per sweep (8 or 16)
    int    fpi;            // frames per image (rows): n_targets or the override
    int    fpi_override;   // 0 = use the detected count, else forced 8/16
    int    zoom;
    int    scale_mode;
    int    dn_min, dn_max;
    int    sess_min, sess_max;
    int    playing;
    float  ips;
    float  accum;
} view_t;

// row of frame k within its image = its target-voltage setpoint index; -1 if
// the target-voltage bytes are a flipped, off-grid value.
static int row_of(const view_t *v, const session_t *s, int k)
{
    return tv_row(s->fr_tv[k], v->fpi);
}

// Detect the setpoint count for the current session and set the row count (fpi).
static void apply_scan(view_t *v, const session_t *s)
{
    v->n_targets = detect_n_targets(s);
    v->fpi = v->fpi_override ? v->fpi_override : v->n_targets;
    if (v->fpi < 1) v->fpi = 16;
}

// Assign every frame to an image. Frames are in file order; the row comes from
// the target voltage, and a new image begins at every scan-index multiple of
// fpi (the scan index is the clean per-frame step counter -- a down-up sweep
// runs 0..2*fpi-1, so it splits into an up image and a down image of fpi rows
// each). Corrupt frames (off-grid target voltage) and the partial leading sweep
// are dropped (fr_img < 0).
static void rebuild_image_list(view_t *v, const session_t *s)
{
    free(v->fr_img);
    v->fr_img = NULL; v->n_img = 0;
    if (s == NULL || s->nframes == 0) return;
    v->fr_img = (int *) malloc((size_t) s->nframes * sizeof(int));
    if (v->fr_img == NULL) return;

    int img = 0, started = 0;
    for (int k = 0; k < s->nframes; k++) {
        if (row_of(v, s, k) < 0) { v->fr_img[k] = -2; continue; }   // corrupt frame
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

// Per-session DN extent over all present pixels (for auto-session scaling).
static void compute_session_extent(view_t *v, const session_t *s)
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

// Switch to a different download and refresh everything derived from it.
static void select_session(view_t *v, const session_t *s)
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
            fprintf(stderr, "mpi_viewer: loaded font %s\n", cands[i]);
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

// ---- main ------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "mpi_viewer")) return 0;

    const char *db_arg = NULL;
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--db=", 5) == 0) db_arg = argv[i] + 5;
        else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: mpi_viewer [--db=<packet_db.sqlite>]\n"
                   "Inspect MPI science imagery reconstructed from the packet DB.\n");
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

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "mpi_viewer: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 1;
    }
    fprintf(stderr, "mpi_viewer: reconstructing MPI downloads from %s ...\n", db_path);
    session_t *sess = NULL;
    int nsess = load_sessions(db, &sess);
    sqlite3_close(db);

    if (nsess == 0) {
        fprintf(stderr, "mpi_viewer: no MPI science-data downloads found.\n");
        return 1;
    }
    fprintf(stderr, "mpi_viewer: %d MPI download%s.\n", nsess, nsess == 1 ? "" : "s");

    view_t v = {0};
    v.zoom = 8; v.ips = 8.0f;
    v.scale_mode = SCALE_AUTO_IMAGE;
    v.dn_min = 1800; v.dn_max = 2300;
    select_session(&v, &sess[v.sel]);

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

    while (!WindowShouldClose()) {
        session_t *s = &sess[v.sel];

        // ---- input ----
        if (key_repeat(KEY_DOWN, &rep_down) && v.sel < nsess - 1) { v.sel++; select_session(&v, &sess[v.sel]); s = &sess[v.sel]; }
        if (key_repeat(KEY_UP, &rep_up)     && v.sel > 0)         { v.sel--; select_session(&v, &sess[v.sel]); s = &sess[v.sel]; }
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
        if (IsKeyPressed(KEY_Q)) break;

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
        int rep_frame = -1, rows_present = 0;
        for (int k = 0; k < s->nframes; k++) {
            if (v.fr_img == NULL || v.fr_img[k] != v.img_pos) continue;
            int row = row_of(&v, s, k);
            if (row < 0 || row >= v.fpi) continue;
            rows_present++;
            if (rep_frame < 0) rep_frame = k;
            if (s->fr_ts[k] >= 0 && (frame_time < 0 || s->fr_ts[k] < frame_time)) frame_time = s->fr_ts[k];
            long base = s->fr_off[k] + PIX_START;
            for (int j = 0; j < NPIX; j++) {
                int p = be16(s->buf, base + j * 2);
                int g = p <= lo ? 0 : p >= hi ? 255 : (int) (255.0 * (p - lo) / (hi - lo));
                // 90 CW: scan row -> column (top row to the right), pixel -> row.
                graybuf[j * MAX_FPI + (v.fpi - 1 - row)] = (unsigned char) g;
            }
        }
        UpdateTexture(tex, graybuf);

        // ---- draw ----
        BeginDrawing();
        ClearBackground((Color){ 18, 18, 22, 255 });

        // left: session list
        DrawRectangle(0, 0, LEFT_W, GetScreenHeight(), (Color){ 28, 28, 34, 255 });
        draw_text("MPI downloads", 12, 10, 18, RAYWHITE);
        draw_text(TextFormat("%d in DB", nsess), LEFT_W - 78, 14, 12, GRAY);
        int row_h = 44, list_top = 40;
        int visible = (GetScreenHeight() - list_top) / row_h;
        int top = 0;
        if (v.sel >= visible) top = v.sel - visible + 1;
        for (int r = 0; r < visible && top + r < nsess; r++) {
            int si = top + r;
            session_t *ss = &sess[si];
            int y = list_top + r * row_h;
            if (si == v.sel) DrawRectangle(0, y, LEFT_W, row_h, (Color){ 44, 70, 110, 255 });
            double pct = 100.0 * (double) ss->recovered / (double) ss->size;
            draw_text(TextFormat("%s UTC", ss->utc), 10, y + 4, 15, si == v.sel ? RAYWHITE : LIGHTGRAY);
            draw_text(TextFormat("%.0f KB  %.0f%%  %d frames",
                                 ss->size / 1024.0, pct, ss->nframes),
                      10, y + 23, 12, GRAY);
        }

        // right: title / time
        int rx = LEFT_W + 20, ry = 14;
        draw_text(TextFormat("Image %d / %d", v.n_img ? v.img_pos + 1 : 0, v.n_img), rx, ry, 20, RAYWHITE);
        char tstr[48] = "no timestamp";
        if (frame_time >= 0) fmt_utc_ms(frame_time, tstr, sizeof tstr);
        draw_text(TextFormat("%s UTC", tstr), rx, ry + 26, 20, (Color){ 120, 220, 160, 255 });

        // the image
        int iy = ry + 64;
        // Rotated 90 CW: on screen the width is the scan rows, the height the pixels.
        int dw = v.fpi * v.zoom, dh = NPIX * v.zoom;
        Rectangle src = { 0, 0, (float) v.fpi, (float) NPIX };
        Rectangle dst = { (float) rx, (float) iy, (float) dw, (float) dh };
        DrawRectangleLines(rx - 1, iy - 1, dw + 2, dh + 2, (Color){ 70, 70, 80, 255 });
        DrawTexturePro(tex, src, dst, (Vector2){ 0, 0 }, 0.0f, WHITE);

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
        draw_text(TextFormat("targets/sweep : %d", v.n_targets), ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("frames/image  : %d %s  (rows %d)",
                             v.fpi, v.fpi_override ? "(fixed)" : "(auto)", rows_present),
                  ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("zoom          : x%d", v.zoom), ax, ay, 15, LIGHTGRAY); ay += 20;
        const char *sm = v.scale_mode == SCALE_AUTO_IMAGE ? "auto (image)"
                       : v.scale_mode == SCALE_AUTO_SESSION ? "auto (session)" : "manual";
        draw_text(TextFormat("color scale   : %s", sm), ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("DN window     : %d .. %d", lo, hi), ax, ay, 15, LIGHTGRAY); ay += 20;
        draw_text(TextFormat("playback      : %s  %.0f img/s", v.playing ? "PLAY" : "paused", v.ips),
                  ax, ay, 15, v.playing ? (Color){ 120, 220, 160, 255 } : GRAY); ay += 20;

        // help footer
        const char *help =
            "Up/Down download   Left/Right image   Space play/pause   ,/. speed"
            "   f frames/image   s zoom   a scale   z/x min  c/v max  q quit";
        draw_text(help, 12, GetScreenHeight() - 22, 12, (Color){ 150, 150, 160, 255 });

        EndDrawing();
    }

    UnloadTexture(tex);
    if (g_ui_font_loaded) UnloadFont(g_ui_font);
    CloseWindow();

    for (int k = 0; k < nsess; k++) {
        free(sess[k].buf); free(sess[k].present);
        free(sess[k].fr_off); free(sess[k].fr_ctr);
        free(sess[k].fr_scan); free(sess[k].fr_tv); free(sess[k].fr_ts);
    }
    free(sess);
    free(v.fr_img);
    return 0;
}

#endif // WITH_SQLITE3
