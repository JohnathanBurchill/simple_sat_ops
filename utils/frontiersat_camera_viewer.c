/*

    Simple Satellite Operations  utils/frontiersat_camera_viewer.c

    A small raylib GUI for looking at FrontierSat's boom-camera pictures
    straight out of the packet DB. The left panel lists captures -- one row per
    picture the camera took -- and the right panel shows that picture. Each
    picture is usually downlinked over several passes, and this tool merges all
    of them back into one image before decoding it.

    Camera files (see cam_jpeg.h): the flight firmware writes a capture to a
    file that starts with the text header "START_CAM:\n" and then a run of
    fixed 67-byte camera sentences, "@IIIITTTThhh...hh\r\n", where IIII is the
    sentence index, TTTT the sentence count, and the 56 hex digits are 28 bytes
    of JPEG. So sentence k always begins at file offset 11 + 67k.

    Finding the camera packets: a download arrives as bulk_file packets, each
    carrying 195 file bytes at a stated file offset, mixed in with every other
    file the satellite sent (science data, logs). Rather than guess which
    download a packet belongs to from its timing, we test each packet against
    the camera-file layout: a packet is camera data when its '@' characters sit
    at file offsets 11 + 67k and its bytes are almost all hex digits. That test
    keeps the real camera packets and rejects the badly decoded ones whose
    "START_CAM:" header survived but whose body is mush.

    Grouping (captures, not downloads): the camera packets are first split into
    download sessions by reception time, then sessions of the SAME picture are
    merged. Two sessions are the same picture when the bytes they both carry
    agree -- different pictures disagree on about four bytes in five, the same
    picture on at most one in five (its only disagreements come from packets
    Reed-Solomon could not correct) -- or when the ground station's command log
    shows both were downloads of the same file on the satellite. The command
    log covers the case content cannot: a follow-up pass that re-fetches only
    the byte ranges the first pass missed shares no bytes with it at all.

    Merging matters: on the current store it takes seven of the nine pictures
    to every byte recovered, where the best single pass had left holes.

    The command log (the sent_tcmd table) also names the file on the satellite
    and, through the camera_capture command that made it, when the picture was
    taken -- which the file itself does not record. Both are best-effort: a
    download commanded from another ground station leaves the row unlabelled.

    Looking at the picture: scroll or pinch over it to zoom, and drag it to
    move it around. Zoom starts at -- and never goes below -- the whole picture
    fitted to the pane, so scrolling back out always lands on the whole picture
    again, and a picture bigger than the pane cannot be dragged off the edge.

    Read-only on the DB. Press F5 to re-read it and rebuild the capture list.

    Usage:
      frontiersat_camera_viewer [--db=<packet_db.sqlite>]

    With no --db the default store is used ($SSO_PACKET_DB, else the
    FrontierSat root's packet_db.sqlite).

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

#include "cam_jpeg.h"
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
    if (sso_version_handle(argc, argv, "frontiersat_camera_viewer")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "frontiersat_camera_viewer: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

#ifdef __APPLE__
// Trackpad pinch arrives as NSEventTypeMagnify, which raylib/GLFW don't
// forward. utils/pinch_macos.m installs an NSEvent local monitor that
// accumulates magnification deltas into this global; we read & reset it
// each frame.
extern float g_sso_pinch_delta;
extern void  sso_install_pinch_monitor(void);
#endif

// Bulk-file packet geometry (mirrors cam_reconstruct.c / mpi_viewer.c). A
// bulk_file payload is [packet_type:1][file_offset:4 LE][data...]; at most 195
// file bytes per packet.
#define BULK_FILE_PACKET_TYPE   16
#define BULK_FILE_HEADER_SIZE   5
#define BULK_FILE_MAX_DATA      195
#define BULK_FILE_MAX_PLAUSIBLE (2 * 1024 * 1024)

// Camera-file layout (see cam_jpeg.h): an 11-byte "START_CAM:\n" header, then
// 67-byte sentences, so sentence k starts at file offset 11 + 67k.
#define CAM_HEADER_LEN   11
#define CAM_SENTENCE_LEN 67

// A packet counts as camera data when at least one '@' sits on the sentence
// grid, no more '@' sit off it, and this fraction of its bytes are hex digits
// (or the sentence punctuation). The header packet spends 8 of its 195 bytes on
// "START_CAM:\n", and downlink bit errors cost a few more, so the bar sits well
// below 1; a wrongly decoded packet scores about 0.02.
#define CAM_MIN_HEX_FRACTION 0.85

// One download session is a run of camera packets no further apart than this.
// The firmware paces a download at about four packets a second, so a five
// minute hole means the download stopped and (usually) a later pass restarted
// it; sessions are merged back into pictures below anyway, so splitting too
// eagerly here costs nothing.
#define SESSION_GAP_MS  (5.0 * 60.0 * 1000.0)

// Same-picture test. Two sessions are compared over the bytes they both
// recovered: different pictures disagree on 53%..85% of them, the same picture
// on at most 22%, so the threshold sits in the middle of a wide gap. Fewer than
// MATCH_MIN_BYTES bytes in common is no evidence either way.
#define MATCH_MIN_BYTES      256
#define MATCH_MAX_MISMATCH   0.35

// How far back of a session's start a bulk-download command may sit and still
// be taken as the command that triggered it.
#define CMD_LOOKBACK_MS (3.0 * 60.0 * 60.0 * 1000.0)

// A merged capture must have at least this many camera sentences to be worth
// listing, which drops the odd stray packet that matched the layout by chance.
#define MIN_SENTENCES 4

#define MAX_SESSIONS_SHOWN 6
#define MAX_PATH_LEN 128

typedef struct {
    double   ts_ms;
    long     off;
    int      rs;
    long     dlen;
    uint8_t *data;
} chunk_t;

// One download session: the camera packets received in one uninterrupted run,
// reassembled by file offset.
typedef struct {
    int     *idx;               // chunk indices, ascending in time
    int      n;
    uint8_t *buf;
    uint8_t *present;
    long     size;
    double   t0, t1;            // first / last reception time (unix ms)
    int      grp;               // merge group id
    char     sat_path[MAX_PATH_LEN];   // file on the satellite, "" if unknown
} session_t;

// One picture, merged from every session that downloaded it.
typedef struct {
    char     sat_path[MAX_PATH_LEN];   // "camera/2026-08-20.img", or ""
    double   t_capture_ms;             // when the camera took it, -1 if unknown
    double   t_first_ms;               // when the first pass downloaded it
    char     capture_utc[24];
    char     first_utc[24];
    int      n_sessions;
    double   sess_ms[MAX_SESSIONS_SHOWN];
    int      chunks;
    uint8_t *buf;
    uint8_t *present;
    long     size;
    long     recovered;
    cam_jpeg_stats_t st;
    uint8_t *jpg;
    long     jpg_len;
    Texture2D tex;
    int      tex_ok;
    int      img_w, img_h;
} capture_t;

// Little-endian uint32 file offset out of a bulk_file payload.
static long bulk_offset(const uint8_t *pl)
{
    return (long) pl[1] | ((long) pl[2] << 8)
         | ((long) pl[3] << 16) | ((long) pl[4] << 24);
}

// "2026-08-09 22:08:04" (UTC) from a unix-ms timestamp.
static void fmt_utc_s(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    struct tm tmv;
    gmtime_r(&secs, &tmv);
    snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

// "20260809_220804" (UTC) from a unix-ms timestamp, for export filenames.
static void fmt_stamp(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    struct tm tmv;
    gmtime_r(&secs, &tmv);
    snprintf(out, n, "%04d%02d%02d_%02d%02d%02d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

// "08-20 07:22" (UTC), the compact form used in the session list.
static void fmt_utc_short(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    struct tm tmv;
    gmtime_r(&secs, &tmv);
    snprintf(out, n, "%02d-%02d %02d:%02d",
             tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min);
}

static int is_hex_digit(int c)
{
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

// True if a bulk_file packet holding `n` file bytes at file offset `off` is
// camera data: its '@' characters land on the sentence grid (file offsets
// 11 + 67k) and its bytes are hex digits or sentence punctuation. Everything
// else the satellite downlinks -- binary science data, JSON logs -- fails the
// hex test, and a packet whose bits were mangled fails it too.
static int chunk_is_camera(long off, const uint8_t *d, long n)
{
    int on_grid = 0, off_grid = 0, plausible = 0;
    for (long j = 0; j < n; j++) {
        int c = d[j];
        if (c == '@') {
            long phase = ((off + j - CAM_HEADER_LEN) % CAM_SENTENCE_LEN + CAM_SENTENCE_LEN)
                         % CAM_SENTENCE_LEN;
            if (phase == 0) on_grid++;
            else off_grid++;
        }
        if (is_hex_digit(c) || c == '@' || c == '\r' || c == '\n') plausible++;
    }
    return on_grid >= 1 && on_grid > off_grid
           && (double) plausible / (double) n >= CAM_MIN_HEX_FRACTION;
}

// ---- DB load / reassembly --------------------------------------------------

// Load every bulk_file packet that looks like camera data, oldest first.
// Returns 0 on success; the caller frees each chunk's data and the array.
static int load_camera_chunks(sqlite3 *db, chunk_t **out, int *out_n)
{
    const char *sql =
        "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, payload, rs_errs "
        "FROM packet WHERE packet_type=16 ORDER BY ts_received, id";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "frontiersat_camera_viewer: query failed: %s\n", sqlite3_errmsg(db));
        return -1;
    }

    chunk_t *cs = NULL;
    int n = 0, cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const uint8_t *pl = (const uint8_t *) sqlite3_column_blob(st, 1);
        int pl_len = sqlite3_column_bytes(st, 1);
        if (pl == NULL || pl_len < BULK_FILE_HEADER_SIZE + 1) continue;
        long off = bulk_offset(pl);
        if (off < 0 || off > BULK_FILE_MAX_PLAUSIBLE) continue;
        long dlen = pl_len - BULK_FILE_HEADER_SIZE;
        if (dlen > BULK_FILE_MAX_DATA) dlen = BULK_FILE_MAX_DATA;
        if (off + dlen > BULK_FILE_MAX_PLAUSIBLE) continue;
        if (!chunk_is_camera(off, pl + BULK_FILE_HEADER_SIZE, dlen)) continue;

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
        c->rs    = sqlite3_column_int(st, 2);
        c->dlen  = dlen;
        c->data  = (uint8_t *) malloc((size_t) dlen);
        if (c->data == NULL) { fprintf(stderr, "out of memory\n"); goto fail; }
        memcpy(c->data, pl + BULK_FILE_HEADER_SIZE, (size_t) dlen);
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

// Reassemble the chunks named by idx[0..nidx) by absolute file offset. Bytes
// never received stay at the '?' gap fill, matching what packet_browser and
// cam_reconstruct write. RS-clean packets are placed first so an uncorrectable
// packet only ever fills bytes still missing.
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
    memset(buf, '?', (size_t) size);

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

// Compare two sessions over the bytes both recovered. Fills *n_compared and
// *n_differ.
static void compare_sessions(const session_t *a, const session_t *b,
                             long *n_compared, long *n_differ)
{
    long lim = a->size < b->size ? a->size : b->size;
    long cmp = 0, differ = 0;
    for (long k = 0; k < lim; k++) {
        if (!a->present[k] || !b->present[k]) continue;
        cmp++;
        if (a->buf[k] != b->buf[k]) differ++;
    }
    *n_compared = cmp; *n_differ = differ;
}

// ---- the ground station's command log --------------------------------------
//
// sent_tcmd holds every telecommand this station transmitted. Camera work
// shows up in two forms, the older direct command and the newer stored blob:
//
//   CTS1+comms_bulk_file_downlink_start(camera/2026-07-08_cam.img,0,0)@...
//   CTS1+exec_blob_from_fs(blobs/bulk_downlink_start_v2.blob,0,camera/...img;0;0)@...
//   CTS1+camera_capture(camera/2026-08-20.img,...)@...
//
// so the file on the satellite is just the "camera/....img" token in the text,
// and the command is a capture rather than a download when "camera_capture("
// appears. Everything here is best-effort: a download commanded elsewhere, or
// a store with no sent_tcmd table at all, simply leaves the labels blank.

typedef struct {
    double ts_ms;
    char   path[MAX_PATH_LEN];
    int    is_capture;
} camcmd_t;

// Copy the "camera/....img" token out of a command into `path`. Returns 1 on
// success. The token ends at the argument separator (',' in the direct form,
// ';' in the blob form) or the closing parenthesis.
static int extract_camera_path(const char *text, char *path, size_t n)
{
    const char *s = strstr(text, "camera/");
    if (s == NULL) return 0;
    size_t k = 0;
    while (s[k] != '\0' && s[k] != ',' && s[k] != ';' && s[k] != ')' && k < n - 1) k++;
    if (k < 5 || strncmp(s + k - 4, ".img", 4) != 0) return 0;   // not a picture file
    memcpy(path, s, k);
    path[k] = '\0';
    return 1;
}

// Load every camera-related telecommand, oldest first. Returns the count (0 if
// the table is absent, which is not an error).
static int load_camera_commands(sqlite3 *db, camcmd_t **out)
{
    *out = NULL;
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT ts_sent_ms, command_text FROM sent_tcmd "
            "WHERE command_text LIKE '%camera/%' ORDER BY ts_sent_ms",
            -1, &st, NULL) != SQLITE_OK)
        return 0;

    camcmd_t *cmds = NULL;
    int n = 0, cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const char *text = (const char *) sqlite3_column_text(st, 1);
        if (text == NULL) continue;
        char path[MAX_PATH_LEN];
        if (!extract_camera_path(text, path, sizeof path)) continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 64;
            camcmd_t *t = (camcmd_t *) realloc(cmds, (size_t) cap * sizeof *cmds);
            if (t == NULL) break;
            cmds = t;
        }
        cmds[n].ts_ms = (double) sqlite3_column_int64(st, 0);
        snprintf(cmds[n].path, sizeof cmds[n].path, "%s", path);
        cmds[n].is_capture = (strstr(text, "camera_capture(") != NULL);
        n++;
    }
    sqlite3_finalize(st);
    *out = cmds;
    return n;
}

// The file this session was downloading, from the last download command sent
// before the session ended (and no longer than CMD_LOOKBACK_MS before it
// started). Leaves sat_path empty when nothing matches.
static void label_session(session_t *s, const camcmd_t *cmds, int ncmds)
{
    double best = -1;
    for (int k = 0; k < ncmds; k++) {
        if (cmds[k].is_capture) continue;
        if (cmds[k].ts_ms > s->t1) break;                    // sorted by time
        if (cmds[k].ts_ms < s->t0 - CMD_LOOKBACK_MS) continue;
        best = cmds[k].ts_ms;
        snprintf(s->sat_path, sizeof s->sat_path, "%s", cmds[k].path);
    }
    if (best < 0) s->sat_path[0] = '\0';
}

// When the camera took `path`, from the camera_capture command that made it.
// Returns -1 if the command log doesn't have it.
static double capture_time_of(const char *path, const camcmd_t *cmds, int ncmds)
{
    if (path == NULL || path[0] == '\0') return -1;
    for (int k = 0; k < ncmds; k++)
        if (cmds[k].is_capture && strcmp(cmds[k].path, path) == 0)
            return cmds[k].ts_ms;
    return -1;
}

// ---- building the capture list ---------------------------------------------

static void free_sessions(session_t *ss, int ns)
{
    for (int k = 0; k < ns; k++) {
        free(ss[k].idx); free(ss[k].buf); free(ss[k].present);
    }
    free(ss);
}

static void free_captures(capture_t *caps, int n)
{
    for (int k = 0; k < n; k++) {
        free(caps[k].buf); free(caps[k].present); free(caps[k].jpg);
    }
    free(caps);
}

// Split the camera packets into download sessions, merge the sessions that
// downloaded the same picture, and build one capture per picture. Returns the
// capture count and fills *out (caller frees via free_captures).
static int build_captures(const chunk_t *cs, int n, const camcmd_t *cmds, int ncmds,
                          capture_t **out)
{
    *out = NULL;
    if (n == 0) return 0;

    // Sessions: runs of packets no more than SESSION_GAP_MS apart.
    session_t *ss = NULL;
    int ns = 0;
    for (int i = 0; i < n; ) {
        int j = i;
        while (j + 1 < n && cs[j + 1].ts_ms - cs[j].ts_ms <= SESSION_GAP_MS) j++;
        session_t *t = (session_t *) realloc(ss, (size_t) (ns + 1) * sizeof *ss);
        if (t == NULL) { free_sessions(ss, ns); return 0; }
        ss = t;
        session_t *s = &ss[ns];
        memset(s, 0, sizeof *s);
        s->n = j - i + 1;
        s->idx = (int *) malloc((size_t) s->n * sizeof(int));
        if (s->idx == NULL) { free_sessions(ss, ns); return 0; }
        for (int k = 0; k < s->n; k++) s->idx[k] = i + k;
        s->size = reassemble(cs, s->idx, s->n, &s->buf, &s->present);
        s->t0 = cs[i].ts_ms;
        s->t1 = cs[j].ts_ms;
        s->grp = ns;
        label_session(s, cmds, ncmds);
        ns++;
        i = j + 1;
    }

    // Merge sessions of the same picture. Content decides when the two share
    // enough bytes to judge; otherwise the satellite file path from the command
    // log does, which is the only evidence for a follow-up pass that fetched
    // exactly the ranges the first pass missed. Content also has a veto: if the
    // shared bytes disagree these are different pictures, whatever the log says.
    for (int a = 0; a < ns; a++) {
        for (int b = a + 1; b < ns; b++) {
            long cmp = 0, differ = 0;
            compare_sessions(&ss[a], &ss[b], &cmp, &differ);
            int judged = cmp >= MATCH_MIN_BYTES;
            int contradicts = judged && (double) differ / (double) cmp > MATCH_MAX_MISMATCH;
            int same_path = ss[a].sat_path[0] != '\0'
                            && strcmp(ss[a].sat_path, ss[b].sat_path) == 0;
            int same = !contradicts && (judged || same_path);
            if (!same) continue;
            int ga = ss[a].grp, gb = ss[b].grp;
            int lo = ga < gb ? ga : gb, hi = ga < gb ? gb : ga;
            for (int k = 0; k < ns; k++) if (ss[k].grp == hi) ss[k].grp = lo;
        }
    }

    capture_t *caps = (capture_t *) malloc((size_t) ns * sizeof *caps);
    int *idx = (int *) malloc((size_t) n * sizeof(int));
    int nout = 0;
    if (caps == NULL || idx == NULL) {
        free(caps); free(idx); free_sessions(ss, ns);
        return 0;
    }

    for (int g = 0; g < ns; g++) {
        int nidx = 0, nsess = 0;
        capture_t c = {0};
        c.t_capture_ms = -1;
        for (int k = 0; k < ns; k++) {
            if (ss[k].grp != g) continue;
            for (int q = 0; q < ss[k].n; q++) idx[nidx++] = ss[k].idx[q];
            if (nsess < MAX_SESSIONS_SHOWN) c.sess_ms[nsess] = ss[k].t0;
            if (c.sat_path[0] == '\0' && ss[k].sat_path[0] != '\0')
                snprintf(c.sat_path, sizeof c.sat_path, "%s", ss[k].sat_path);
            if (nsess == 0) c.t_first_ms = ss[k].t0;
            nsess++;
        }
        if (nsess == 0) continue;   // group already folded into a lower one

        c.n_sessions = nsess;
        c.chunks = nidx;
        c.size = reassemble(cs, idx, nidx, &c.buf, &c.present);
        if (c.size <= 0) { free(c.buf); free(c.present); continue; }
        for (long b = 0; b < c.size; b++) if (c.present[b]) c.recovered++;

        c.jpg = cam_jpeg_decode(c.buf, c.size, 0x00, &c.st, &c.jpg_len);
        if (c.jpg == NULL || c.st.sentences_present < MIN_SENTENCES) {
            free(c.buf); free(c.present); free(c.jpg);
            continue;
        }

        c.t_capture_ms = capture_time_of(c.sat_path, cmds, ncmds);
        if (c.t_capture_ms > 0) fmt_utc_s(c.t_capture_ms, c.capture_utc, sizeof c.capture_utc);
        fmt_utc_s(c.t_first_ms, c.first_utc, sizeof c.first_utc);
        caps[nout++] = c;
    }

    free(idx);
    free_sessions(ss, ns);
    *out = caps;
    return nout;
}

// Open the DB read-only, build the capture list, close. Returns the count (0 on
// any failure). Used at startup and on manual refresh (F5).
static int reload_captures(const char *db_path, capture_t **out)
{
    *out = NULL;
    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "frontiersat_camera_viewer: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 0;
    }
    chunk_t *cs = NULL;
    int n = 0, ncaps = 0;
    if (load_camera_chunks(db, &cs, &n) == 0 && n > 0) {
        camcmd_t *cmds = NULL;
        int ncmds = load_camera_commands(db, &cmds);
        ncaps = build_captures(cs, n, cmds, ncmds, out);
        free(cmds);
    }
    for (int k = 0; k < n; k++) free(cs[k].data);
    free(cs);
    sqlite3_close(db);
    return ncaps;
}

// ---- textures --------------------------------------------------------------

// Decode each capture's JPEG into a texture. A picture with too many missing
// bytes can fail to decode; that capture keeps tex_ok = 0 and the viewer says
// so instead of drawing it. Must run after InitWindow.
static void build_textures(capture_t *caps, int n)
{
    for (int k = 0; k < n; k++) {
        capture_t *c = &caps[k];
        c->tex_ok = 0;
        if (c->jpg == NULL || c->jpg_len <= 0) continue;
        Image im = LoadImageFromMemory(".jpg", c->jpg, (int) c->jpg_len);
        if (im.width > 0 && im.height > 0) {
            c->tex = LoadTextureFromImage(im);
            SetTextureFilter(c->tex, TEXTURE_FILTER_BILINEAR);
            c->tex_ok = 1;
            c->img_w = im.width;
            c->img_h = im.height;
        }
        UnloadImage(im);
    }
}

static void unload_textures(capture_t *caps, int n)
{
    for (int k = 0; k < n; k++)
        if (caps[k].tex_ok) { UnloadTexture(caps[k].tex); caps[k].tex_ok = 0; }
}

// ---- TTF font (pattern from mpi_viewer.c / decode_inspector.c) --------------
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
    fprintf(stderr, "frontiersat_camera_viewer: TTF font not found; using raylib default\n");
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
    const float delay = 0.30f, rate = 0.06f;
    if (IsKeyPressed(key)) { *cooldown = delay; return 1; }
    if (!IsKeyDown(key)) return 0;
    *cooldown -= GetFrameTime();
    if (*cooldown <= 0.0f) { *cooldown = rate; return 1; }
    return 0;
}

// ---- where the picture sits in its pane ------------------------------------
//
// The picture is drawn at "fit" -- the largest scale the whole of it takes in
// the pane -- times the operator's zoom, and slid by pan. Zoom 1 is fit and is
// the floor, so scrolling back out always lands on the whole picture, and pan
// is held to the picture's own edges so a zoomed-in picture always covers the
// pane. The cursor-anchored zoom and the drag both come through here, so the
// clamping lives in one place.
#define CAM_ZOOM_MAX 40.0f

static Rectangle picture_dst(const capture_t *c, int px, int py, int pw, int ph,
                             float zoom, Vector2 *pan)
{
    float fit_x = (float) pw / (float) c->img_w;
    float fit_y = (float) ph / (float) c->img_h;
    float fit = fit_x < fit_y ? fit_x : fit_y;
    float dw = c->img_w * fit * zoom, dh = c->img_h * fit * zoom;
    // Nothing to slide while the whole picture fits.
    float lim_x = dw > pw ? (dw - pw) / 2.0f : 0.0f;
    float lim_y = dh > ph ? (dh - ph) / 2.0f : 0.0f;
    if (pan->x >  lim_x) pan->x =  lim_x;
    if (pan->x < -lim_x) pan->x = -lim_x;
    if (pan->y >  lim_y) pan->y =  lim_y;
    if (pan->y < -lim_y) pan->y = -lim_y;
    Rectangle dst = { px + (pw - dw) / 2.0f + pan->x,
                      py + (ph - dh) / 2.0f + pan->y, dw, dh };
    return dst;
}

// Zoom by `step` (a log-scale nudge) about the cursor: note which point of the
// picture the cursor is over, change the scale, then slide the picture so that
// same point is back under the cursor.
static void zoom_at_cursor(const capture_t *c, int px, int py, int pw, int ph,
                           Vector2 m, float step, float *zoom, Vector2 *pan)
{
    Rectangle before = picture_dst(c, px, py, pw, ph, *zoom, pan);
    float ux = (m.x - before.x) / before.width;
    float uy = (m.y - before.y) / before.height;
    float nz = *zoom * expf(step);
    if (nz < 1.0f) nz = 1.0f;
    if (nz > CAM_ZOOM_MAX) nz = CAM_ZOOM_MAX;
    if (nz == *zoom) return;
    *zoom = nz;
    Rectangle after = picture_dst(c, px, py, pw, ph, *zoom, pan);
    pan->x += (m.x - after.x) - ux * after.width;
    pan->y += (m.y - after.y) - uy * after.height;
}

// The export name for a capture: when the picture was taken, or failing that
// when the first pass downloaded it.
static void jpeg_name(const capture_t *c, char *out, size_t n)
{
    char stamp[24];
    fmt_stamp(c->t_capture_ms > 0 ? c->t_capture_ms : c->t_first_ms, stamp, sizeof stamp);
    snprintf(out, n, "fs_boomcam_%s.jpg", stamp);
}

// Write a capture's JPEG to `path`. Returns 0, or -1 with the reason in
// *status.
static int write_jpeg(const capture_t *c, const char *path, char *status, size_t nstatus)
{
    FILE *f = fopen(path, "wb");
    if (f == NULL) { snprintf(status, nstatus, "cannot write %s", path); return -1; }
    size_t w = fwrite(c->jpg, 1, (size_t) c->jpg_len, f);
    fclose(f);
    if (w != (size_t) c->jpg_len) {
        snprintf(status, nstatus, "short write on %s", path);
        return -1;
    }
    return 0;
}

// Write the selected capture's JPEG to the working directory.
static void save_jpeg(const capture_t *c, char *status, size_t nstatus)
{
    if (c->jpg == NULL || c->jpg_len <= 0) {
        snprintf(status, nstatus, "nothing to save");
        return;
    }
    char path[256];
    jpeg_name(c, path, sizeof path);
    if (write_jpeg(c, path, status, nstatus) == 0)
        snprintf(status, nstatus, "saved %s", path);
}

// Hand the picture to the desktop's image viewer -- Preview on macOS, whatever
// xdg-open picks on Linux. The copy goes to the temporary directory rather than
// the working directory, so this stays a quick look and `s` remains the way to
// keep one. The viewer is launched in the background so a program that does not
// return until its window closes cannot stall the render loop.
static void open_in_system_viewer(const capture_t *c, char *status, size_t nstatus)
{
    if (c->jpg == NULL || c->jpg_len <= 0) {
        snprintf(status, nstatus, "nothing to open");
        return;
    }
    const char *tmp = getenv("TMPDIR");
    if (tmp == NULL || tmp[0] == '\0') tmp = "/tmp";
    size_t tlen = strlen(tmp);
    while (tlen > 1 && tmp[tlen - 1] == '/') tlen--;      // TMPDIR often ends in '/'

    char name[64];
    jpeg_name(c, name, sizeof name);
    char path[512];
    snprintf(path, sizeof path, "%.*s/%s", (int) tlen, tmp, name);
    if (write_jpeg(c, path, status, nstatus) != 0) return;

#if defined(__APPLE__)
    const char *opener = "open";
#else
    const char *opener = "xdg-open";
#endif
    char cmd[640];
    snprintf(cmd, sizeof cmd, "%s '%s' >/dev/null 2>&1 &", opener, path);
    if (system(cmd) != 0) snprintf(status, nstatus, "%s failed on %s", opener, path);
    else snprintf(status, nstatus, "opened %s", name);
}

// ---- main ------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "frontiersat_camera_viewer")) return 0;

    const char *db_arg = NULL;
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--db=", 5) == 0) db_arg = argv[i] + 5;
        else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: frontiersat_camera_viewer [--db=<packet_db.sqlite>]\n"
                   "Look at FrontierSat's boom-camera pictures rebuilt from the packet DB.\n"
                   "The left panel lists captures, one row per picture; F5 re-reads the DB.\n");
            return 0;
        } else {
            fprintf(stderr, "frontiersat_camera_viewer: unknown option '%s' (try --help)\n", argv[i]);
            return 1;
        }
    }

    char db_default[1024];
    const char *db_path = db_arg;
    if (db_path == NULL) {
        if (packet_db_default_path(db_default, sizeof db_default) != 0) {
            fprintf(stderr, "frontiersat_camera_viewer: no DB path "
                            "(set $SSO_PACKET_DB or pass --db=)\n");
            return 1;
        }
        db_path = db_default;
    }

    fprintf(stderr, "frontiersat_camera_viewer: rebuilding camera captures from %s ...\n", db_path);
    capture_t *caps = NULL;
    int ncaps = reload_captures(db_path, &caps);
    if (ncaps == 0) {
        fprintf(stderr, "frontiersat_camera_viewer: no boom-camera captures found.\n");
        return 1;
    }
    fprintf(stderr, "frontiersat_camera_viewer: %d capture%s.\n",
            ncaps, ncaps == 1 ? "" : "s");

    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "frontiersat_camera_viewer");
    SetTargetFPS(60);
    g_ui_font_loaded = load_ui_font();
    build_textures(caps, ncaps);
#ifdef __APPLE__
    sso_install_pinch_monitor();
#endif

    const int LEFT_W = 340;

    int sel = 0;
    // How the picture is being looked at: 1 is the whole picture fitted to the
    // pane, pan is how far it has been dragged from the middle. Both go back
    // to the start with every change of picture.
    float zoom = 1.0f;
    Vector2 pan = { 0, 0 };
    int dragging = 0;
    char status[160] = "";
    float status_left = 0.0f;
    float rep_up = 0, rep_down = 0;

    while (!WindowShouldClose()) {
        capture_t *c = &caps[sel];

        // The picture's pane, which the mouse handling below and the drawing
        // further down both need.
        int sw = GetScreenWidth(), sh = GetScreenHeight();
        const int info_h = 152;
        int rx = LEFT_W + 20;
        int px = rx, py = 70;
        int pw = sw - rx - 20, ph = sh - py - info_h;
        if (pw < 40) pw = 40;
        if (ph < 40) ph = 40;

        // ---- input ----
        int moved = 0;
        if (key_repeat(KEY_DOWN, &rep_down) && sel < ncaps - 1) { sel++; moved = 1; }
        if (key_repeat(KEY_UP, &rep_up)     && sel > 0)         { sel--; moved = 1; }
        if (moved) { c = &caps[sel]; zoom = 1.0f; pan = (Vector2){ 0, 0 }; }

        // Scroll or pinch over the picture to zoom, drag it to move it about.
        Vector2 m = GetMousePosition();
        int over_pic = m.x >= px && m.x < px + pw && m.y >= py && m.y < py + ph;
        float step = 0.0f;
#ifdef __APPLE__
        step += g_sso_pinch_delta;
        g_sso_pinch_delta = 0.0f;
#endif
        if (over_pic) step += 0.14f * GetMouseWheelMove();
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) dragging = 0;
        if (over_pic && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) dragging = 1;
        if (c->tex_ok) {
            if (over_pic && step != 0.0f)
                zoom_at_cursor(c, px, py, pw, ph, m, step, &zoom, &pan);
            if (dragging) {
                Vector2 d = GetMouseDelta();
                pan.x += d.x;
                pan.y += d.y;
            }
        }

        if (IsKeyPressed(KEY_S)) { save_jpeg(c, status, sizeof status); status_left = 6.0f; }
        if (IsKeyPressed(KEY_O)) { open_in_system_viewer(c, status, sizeof status); status_left = 6.0f; }
        if (IsKeyPressed(KEY_F5)) {
            capture_t *nc = NULL;
            int nn = reload_captures(db_path, &nc);
            if (nn > 0) {
                unload_textures(caps, ncaps);
                free_captures(caps, ncaps);
                caps = nc; ncaps = nn;
                build_textures(caps, ncaps);
                if (sel >= ncaps) sel = ncaps - 1;
                c = &caps[sel];
                zoom = 1.0f;
                pan = (Vector2){ 0, 0 };
                snprintf(status, sizeof status, "reloaded: %d capture%s",
                         ncaps, ncaps == 1 ? "" : "s");
            } else {
                if (nc != NULL) free_captures(nc, nn);
                snprintf(status, sizeof status, "reload found no captures; keeping the old list");
            }
            status_left = 6.0f;
        }
        if (IsKeyPressed(KEY_Q)) break;
        if (status_left > 0.0f) status_left -= GetFrameTime();

        // ---- draw ----
        BeginDrawing();
        ClearBackground((Color){ 18, 18, 22, 255 });

        // left: capture list
        DrawRectangle(0, 0, LEFT_W, sh, (Color){ 28, 28, 34, 255 });
        draw_text("Camera captures", 12, 10, 18, RAYWHITE);
        draw_text(TextFormat("%d", ncaps), LEFT_W - 40, 14, 12, GRAY);
        int row_h = 44, list_top = 40;
        int visible = (sh - list_top) / row_h;
        int top = 0;
        if (sel >= visible) top = sel - visible + 1;
        for (int r = 0; r < visible && top + r < ncaps; r++) {
            int si = top + r;
            capture_t *cc = &caps[si];
            int y = list_top + r * row_h;
            if (si == sel) DrawRectangle(0, y, LEFT_W, row_h, (Color){ 44, 70, 110, 255 });
            double pct = 100.0 * (double) cc->recovered / (double) cc->size;
            if (cc->t_capture_ms > 0)
                draw_text(TextFormat("%s UTC", cc->capture_utc), 10, y + 4, 15,
                          si == sel ? RAYWHITE : LIGHTGRAY);
            else
                draw_text(TextFormat("%s UTC (downlink)", cc->first_utc), 10, y + 4, 15,
                          si == sel ? RAYWHITE : LIGHTGRAY);
            draw_text(TextFormat("%.1f KB  %.0f%%  %ld sentences",
                                 cc->jpg_len / 1024.0, pct, cc->st.sentences_present),
                      10, y + 23, 12, GRAY);
        }

        // right: title
        draw_text(c->sat_path[0] ? c->sat_path : "(satellite file unknown)",
                  rx, 12, 20, RAYWHITE);
        if (c->t_capture_ms > 0)
            draw_text(TextFormat("taken %s UTC", c->capture_utc), rx, 40, 17,
                      (Color){ 120, 220, 160, 255 });
        else
            draw_text("capture time unknown (no camera_capture command on record)",
                      rx, 40, 17, (Color){ 190, 170, 110, 255 });

        // the picture, in the pane above the info panel
        DrawRectangleLines(px - 1, py - 1, pw + 2, ph + 2, (Color){ 70, 70, 80, 255 });
        float shown_scale = 0.0f;
        if (c->tex_ok) {
            Rectangle src = { 0, 0, (float) c->img_w, (float) c->img_h };
            Rectangle dst = picture_dst(c, px, py, pw, ph, zoom, &pan);
            shown_scale = dst.width / (float) c->img_w;
            BeginScissorMode(px, py, pw, ph);
            DrawTexturePro(c->tex, src, dst, (Vector2){ 0, 0 }, 0.0f, WHITE);
            EndScissorMode();
        } else {
            draw_text("JPEG will not decode -- too many bytes missing.",
                      px + 16, py + ph / 2 - 20, 18, (Color){ 220, 140, 140, 255 });
            draw_text("Press s to save the partial file and try another decoder.",
                      px + 16, py + ph / 2 + 6, 15, GRAY);
        }

        // info panel below the picture: two columns of counts, then the
        // download sessions across the full width (the list can be long).
        int iy = sh - info_h + 10;
        int col2 = rx + 460;
        double pct = 100.0 * (double) c->recovered / (double) c->size;
        draw_text(TextFormat("file bytes   : %ld of %ld  (%.1f%%)", c->recovered, c->size, pct),
                  rx, iy, 15, LIGHTGRAY);
        draw_text(TextFormat("sentences    : %ld of %ld%s%s",
                             c->st.sentences_present, c->st.n_sentences,
                             c->st.partial_sentences ? TextFormat("  (%ld partial)",
                                                                 c->st.partial_sentences) : "",
                             c->st.saw_end_sentinel ? ", end marker" : ""),
                  rx, iy + 20, 15, LIGHTGRAY);
        draw_text(TextFormat("jpeg         : %ld bytes%s", c->jpg_len,
                             c->tex_ok ? TextFormat("  %d x %d", c->img_w, c->img_h)
                                       : "  (does not decode)"),
                  rx, iy + 40, 15, LIGHTGRAY);

        draw_text(TextFormat("packets      : %d", c->chunks), col2, iy, 15, LIGHTGRAY);
        draw_text(c->tex_ok ? TextFormat("zoom         : %.0f%%%s", shown_scale * 100.0f,
                                         zoom <= 1.0f ? "  (whole picture)" : "")
                            : "zoom         : -",
                  col2, iy + 20, 15, LIGHTGRAY);
        if (status_left > 0.0f)
            draw_text(status, col2, iy + 40, 15, (Color){ 120, 220, 160, 255 });

        {
            char line[256] = "";
            int shown = c->n_sessions < MAX_SESSIONS_SHOWN ? c->n_sessions : MAX_SESSIONS_SHOWN;
            for (int k = 0; k < shown; k++) {
                char t[16];
                fmt_utc_short(c->sess_ms[k], t, sizeof t);
                snprintf(line + strlen(line), sizeof line - strlen(line),
                         "%s%s", k ? "  " : "", t);
            }
            if (c->n_sessions > shown)
                snprintf(line + strlen(line), sizeof line - strlen(line),
                         "  +%d more", c->n_sessions - shown);
            draw_text(TextFormat("downloaded   : %d pass%s   %s",
                                 c->n_sessions, c->n_sessions == 1 ? "" : "es", line),
                      rx, iy + 60, 15, LIGHTGRAY);
        }

        // help footer
        draw_text("Up/Down capture   scroll or pinch zoom   drag to move   "
                  "o open   s save jpeg   F5 refresh   q quit",
                  12, sh - 22, 12, (Color){ 150, 150, 160, 255 });

        EndDrawing();
    }

    unload_textures(caps, ncaps);
    if (g_ui_font_loaded) UnloadFont(g_ui_font);
    CloseWindow();

    free_captures(caps, ncaps);
    return 0;
}

#endif // WITH_SQLITE3
