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
    download sessions by reception time, and again wherever the sentence count
    the packets declare changes -- one picture's download straight after
    another's, as on 2026-10-22 at 23:22, when the 15:07 picture's packets were
    filed under the 15:44 picture's. Then sessions of the SAME picture are
    merged. Two sessions are the same picture when the bytes they both carry
    agree -- different pictures disagree on about four bytes in five, the same
    picture on at most one in five (its only disagreements come from packets
    Reed-Solomon could not correct) -- or when the ground station's command log
    shows both were downloads of the same file on the satellite. The command
    log covers the case content cannot: a follow-up pass that re-fetches only
    the byte ranges the first pass missed shares no bytes with it at all.

    Merging matters: on the current store it takes seven of the nine pictures
    to every byte recovered, where the best single pass had left holes.

    A picture whose first sentences never arrived has no JPEG header, but
    every picture so far has had the same one, so the decoder (cam_jpeg.c)
    puts that standard header in and the picture still shows. The info line
    under the picture then reads "standard header". The 2026-10-22 15:44
    picture was the first that needed it.

    The command log (the sent_tcmd table) also names the file on the satellite
    and, through the camera_capture command that made it, when the picture was
    taken -- which the file itself does not record. Both are best-effort: a
    download commanded from another ground station leaves the row unlabelled.

    Looking at the picture: scroll or pinch over it to zoom, and drag it to
    move it around. Zoom starts at -- and never goes below -- the whole picture
    fitted to the pane, so scrolling back out always lands on the whole picture
    again, and a picture bigger than the pane cannot be dragged off the edge.

    Where it was taken: under the capture list is the Earth (utils/sat_globe.c,
    the same panel mpi_viewer carries), NASA's Blue Marble wrapped on a sphere
    and lit from where the Sun actually stood at that moment, so a picture taken
    over the night side has a dark Earth under it. Five minutes of ground track
    either side of the moment is drawn across it, with a dot on the point the
    satellite was over. Drag the globe to turn it, press with two fingers and
    slide to turn it about the satellite, scroll to zoom, g to put it back. When
    no camera_capture command is on record the capture time is not known, and
    the globe falls back to where the satellite was when the picture was first
    downloaded -- a different place entirely, which the panel's heading says.
    When an extended beacon or ADCS log record near the moment carries an
    attitude, p outlines on the globe the patch of ground the picture
    covered: the camera's field of view projected from the satellite along
    the way it was facing.

    The list runs newest picture first. f tags the picture on show as a
    favourite (a gold star in the list and after the title), and v switches the
    list between every picture and the favourites alone. The favourites, that
    switch, and the picture last looked at are kept in
    ~/.local/state/simple_sat_ops/frontiersat_camera_viewer.state, so the next
    run opens where this one left off. Pictures are remembered by the moment
    they belong to (the stamp their exported JPEG is named with), not by their
    place in the list, so a new picture arriving at the top moves nothing.

    Read-only on the DB. Press F5 to re-read it and rebuild the capture list.

    The cache: a picture taken (or, with no capture command on record, first
    downloaded) more than a week ago is not put together again at every
    start. It is kept, as built, in
    ~/.cache/simple_sat_ops/frontiersat_camera_viewer.cache
    (utils/viewer_cache.h has the rules), and only the rest is rebuilt, from
    the last week or so of packets and whatever the DB has gained since the
    cache was written. A download of an old picture that comes in later is
    not folded in by itself: the info panel says how many such downloads it
    has left out, and shift-F5 (or --rebuild-cache) rebuilds every picture
    from the whole DB and writes the cache afresh. The attitudes keep a cache
    of their own beside it, by the same rules. A new build of the viewer
    starts with a full rebuild, and deleting the cache files is always safe.

    Usage:
      frontiersat_camera_viewer [--db=<packet_db.sqlite>] [--rebuild-cache]

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

#include "beacon_attitude.h"
#include "cam_jpeg.h"
#include "packet_db.h"
#include "sat_globe.h"
#include "sso_paths.h"
#include "sso_version.h"
#include "viewer_cache.h"

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
// eagerly here costs nothing. A run is also cut where the declared sentence
// count changes (confirmed_total), since a gap alone does not separate two
// pictures downloaded back to back.
#define SESSION_GAP_MS  (5.0 * 60.0 * 1000.0)

// Same-picture test. Two sessions are compared over the bytes they both
// recovered: different pictures disagree on 53%..85% of them, the same picture
// on at most 22%, so the threshold sits in the middle of a wide gap. Fewer than
// MATCH_MIN_BYTES bytes in common is no evidence either way.
#define MATCH_MIN_BYTES      256
#define MATCH_MAX_MISMATCH   0.35

// The test only looks at the image data, from sentence 21 on. Every picture
// opens with the same JPEG tables -- in all 16 pictures from 2026-06-01 to
// 2026-10-11 the image data starts at JPEG byte 589, inside sentence 21 -- so
// two different pictures agree there. A 2026-10-16 download whose only bytes
// in common with the 2026-07-23 one lay in the tables (95% agreement over 390
// bytes) was merged into it before this cut-off existed.
#define MATCH_FIRST_OFFSET   (CAM_HEADER_LEN + 21 * CAM_SENTENCE_LEN)

// How far back of a session's start a bulk-download command may sit and still
// be taken as the command that triggered it.
#define CMD_LOOKBACK_MS (3.0 * 60.0 * 60.0 * 1000.0)

// A merged capture must have at least this many camera sentences to be worth
// listing, which drops the odd stray packet that matched the layout by chance.
#define MIN_SENTENCES 4

#define MAX_SESSIONS_SHOWN 6
#define MAX_PATH_LEN 128

// How much ground track the globe draws either side of the moment the picture
// belongs to. A picture is taken in an instant, so unlike an MPI recording it
// has no arc of its own; five minutes of flight each way is about 2000 km of
// track either side, enough to say which way the satellite was going and where
// it had come from without wrapping the arc round the whole Earth.
#define CAM_TRACK_HALF_MS (5.0 * 60.0 * 1000.0)

// The camera's field of view across the picture and down it, for the outline
// of what it covered that p draws on the globe. Measured from the curve of the
// Earth's limb in the 2026-08-06 picture: a focal length of about 705 pixels
// over the 640 x 480 frame. No lens distortion is allowed for.
#define CAM_FOV_ACROSS_DEG 49.0
#define CAM_FOV_DOWN_DEG   38.0

// The help line along the bottom of the window runs the full width, so the
// left column stops above it.
#define FOOTER_H 28

typedef struct {
    long long id;               // the packet's row in the DB
    double   ts_ms;
    long     off;
    int      rs;
    long     dlen;
    uint8_t *data;
    long     total;             // sentence count its sentences declare, -1 if unreadable
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
    // What the viewer remembers the picture by between runs: the moment
    // it belongs to (capture, else first downlink) as "20260820_072204",
    // the same stamp its exported JPEG is named with.
    char     id[24];
    int      fav;                      // tagged as a favourite
    char     sat_path[MAX_PATH_LEN];   // "camera/2026-08-20.img", or ""
    double   t_capture_ms;             // when the camera took it, -1 if unknown
    double   t_first_ms;               // when the first pass downloaded it
    char     capture_utc[24];
    char     first_utc[24];
    int      n_sessions;
    double   sess_ms[MAX_SESSIONS_SHOWN];
    int      chunks;
    long long max_id;                  // the newest of its packets' DB rows
    uint8_t *buf;
    uint8_t *present;
    long     size;
    long     recovered;
    cam_jpeg_stats_t st;
    uint8_t *jpg;
    long     jpg_len;
    Texture2D tex;
    // The same picture histogram-equalized (e) and with its colours
    // stretched to full scale (c).
    Texture2D tex_eq;
    Texture2D tex_st;
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

// The picture's sentence count, as declared by the sentences that start in a
// camera packet: the TTTT after each on-grid '@'. Every sentence of a picture
// carries the same count, so it tells two pictures apart (unless they happen
// to be the same length). A packet that failed its checksum can carry a
// wrong count, so the packet's sentences vote; with no clear winner, or no
// count readable at all, the result is -1.
static long chunk_declared_total(long off, const uint8_t *d, long n)
{
    long vals[4];
    int nv = 0;
    for (long j = 0; j + 9 <= n && nv < 4; j++) {
        if (d[j] != '@') continue;
        long phase = ((off + j - CAM_HEADER_LEN) % CAM_SENTENCE_LEN + CAM_SENTENCE_LEN)
                     % CAM_SENTENCE_LEN;
        if (phase != 0) continue;
        long v = 0;
        int ok = 1;
        for (int k = 5; k < 9 && ok; k++) {
            int c = d[j + k];
            if (!is_hex_digit(c)) ok = 0;
            else v = v * 16 + (c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10);
        }
        if (ok) vals[nv++] = v;
    }
    long best = -1;
    int best_n = 0, tied = 0;
    for (int a = 0; a < nv; a++) {
        int c = 0;
        for (int b = 0; b < nv; b++) if (vals[b] == vals[a]) c++;
        if (c > best_n) { best_n = c; best = vals[a]; tied = 0; }
        else if (c == best_n && vals[a] != best) tied = 1;
    }
    return tied ? -1 : best;
}

// ---- DB load / reassembly --------------------------------------------------

// Load every bulk_file packet that looks like camera data, oldest first.
// Returns 0 on success; the caller frees each chunk's data and the array.
// The camera packets among every bulk_file packet, or with `since` (an ISO
// time as ts_received writes it) among those received from then on plus every
// row newer than after_id -- the part a start with a cache has to rebuild.
static int load_camera_chunks(sqlite3 *db, const char *since, long long after_id,
                              chunk_t **out, int *out_n)
{
    const char *sql = since == NULL
        ? "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, payload, rs_errs, id "
          "FROM packet WHERE packet_type=16 ORDER BY ts_received, id"
        : "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, payload, rs_errs, id "
          "FROM packet WHERE packet_type=16 AND (ts_received >= ?1 OR id > ?2) "
          "ORDER BY ts_received, id";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "frontiersat_camera_viewer: query failed: %s\n", sqlite3_errmsg(db));
        return -1;
    }
    if (since != NULL) {
        sqlite3_bind_text(st, 1, since, -1, SQLITE_TRANSIENT);
        sqlite3_bind_int64(st, 2, after_id);
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
        c->id    = sqlite3_column_int64(st, 3);
        c->ts_ms = sqlite3_column_double(st, 0);
        c->off   = off;
        c->rs    = sqlite3_column_int(st, 2);
        c->dlen  = dlen;
        c->total = chunk_declared_total(off, pl + BULK_FILE_HEADER_SIZE, dlen);
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

// Compare two downloads over the image-data bytes both recovered. Fills
// *n_compared and *n_differ.
static void compare_bytes(const uint8_t *abuf, const uint8_t *apresent, long asize,
                          const uint8_t *bbuf, const uint8_t *bpresent, long bsize,
                          long *n_compared, long *n_differ)
{
    long lim = asize < bsize ? asize : bsize;
    long cmp = 0, differ = 0;
    for (long k = MATCH_FIRST_OFFSET; k < lim; k++) {
        if (!apresent[k] || !bpresent[k]) continue;
        cmp++;
        if (abuf[k] != bbuf[k]) differ++;
    }
    *n_compared = cmp; *n_differ = differ;
}

// Whether two downloads are of the same picture. Content decides when the two
// share enough bytes to judge; otherwise the satellite file path from the
// command log does, which is the only evidence for a follow-up pass that
// fetched exactly the ranges the first pass missed. Content also has a veto:
// if the shared bytes disagree these are different pictures, whatever the log
// says.
static int same_picture(const uint8_t *abuf, const uint8_t *apresent, long asize,
                        const char *apath,
                        const uint8_t *bbuf, const uint8_t *bpresent, long bsize,
                        const char *bpath)
{
    long cmp = 0, differ = 0;
    compare_bytes(abuf, apresent, asize, bbuf, bpresent, bsize, &cmp, &differ);
    int judged = cmp >= MATCH_MIN_BYTES;
    int contradicts = judged && (double) differ / (double) cmp > MATCH_MAX_MISMATCH;
    int same_path = apath[0] != '\0' && strcmp(apath, bpath) == 0;
    return !contradicts && (judged || same_path);
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

// The moment a picture belongs to: when it was taken, or failing that when the
// first pass downloaded it.
static double capture_moment(const capture_t *c)
{
    return c->t_capture_ms > 0 ? c->t_capture_ms : c->t_first_ms;
}

static int by_newest_first(const void *a, const void *b)
{
    double ta = capture_moment((const capture_t *) a);
    double tb = capture_moment((const capture_t *) b);
    return (ta < tb) - (ta > tb);
}

// Packet k's declared sentence count, when the next packet in the same run
// that declares one agrees with it; otherwise -1. One damaged packet's wrong
// count is never confirmed, so it cannot split a session on its own.
static long confirmed_total(const chunk_t *cs, int n, int k)
{
    if (cs[k].total < 0) return -1;
    for (int m = k + 1; m < n && cs[m].ts_ms - cs[m - 1].ts_ms <= SESSION_GAP_MS; m++)
        if (cs[m].total >= 0) return cs[m].total == cs[k].total ? cs[k].total : -1;
    return -1;
}

// Split the camera packets into download sessions, merge the sessions that
// downloaded the same picture, and build one capture per picture. Returns the
// capture count and fills *out (caller frees via free_captures).
static int build_captures(const chunk_t *cs, int n, const camcmd_t *cmds, int ncmds,
                          capture_t **out)
{
    *out = NULL;
    if (n == 0) return 0;

    // Sessions: runs of packets no more than SESSION_GAP_MS apart, cut again
    // where the picture's declared sentence count changes -- one picture's
    // download following straight on from another's.
    session_t *ss = NULL;
    int ns = 0;
    for (int i = 0; i < n; ) {
        int j = i;
        long picture = confirmed_total(cs, n, i);
        while (j + 1 < n && cs[j + 1].ts_ms - cs[j].ts_ms <= SESSION_GAP_MS) {
            long t = confirmed_total(cs, n, j + 1);
            if (t >= 0 && picture >= 0 && t != picture) break;
            if (picture < 0) picture = t;
            j++;
        }
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

    // Merge sessions of the same picture (same_picture says how that is told).
    for (int a = 0; a < ns; a++) {
        for (int b = a + 1; b < ns; b++) {
            if (!same_picture(ss[a].buf, ss[a].present, ss[a].size, ss[a].sat_path,
                              ss[b].buf, ss[b].present, ss[b].size, ss[b].sat_path))
                continue;
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
        for (int q = 0; q < nidx; q++)
            if (cs[idx[q]].id > c.max_id) c.max_id = cs[idx[q]].id;
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
        fmt_stamp(capture_moment(&c), c.id, sizeof c.id);
        caps[nout++] = c;
    }

    free(idx);
    free_sessions(ss, ns);
    // Newest picture at the top of the list, oldest at the bottom.
    qsort(caps, (size_t) nout, sizeof *caps, by_newest_first);
    *out = caps;
    return nout;
}

// Build the capture list from an open DB: every picture, or with `since` the
// ones the packets load_camera_chunks picks out make. Returns the count.
static int captures_from_db(sqlite3 *db, const char *since, long long after_id,
                            capture_t **out)
{
    *out = NULL;
    chunk_t *cs = NULL;
    int n = 0, ncaps = 0;
    if (load_camera_chunks(db, since, after_id, &cs, &n) == 0 && n > 0) {
        camcmd_t *cmds = NULL;
        int ncmds = load_camera_commands(db, &cmds);
        ncaps = build_captures(cs, n, cmds, ncmds, out);
        free(cmds);
    }
    for (int k = 0; k < n; k++) free(cs[k].data);
    free(cs);
    return ncaps;
}

// ---- the cache -------------------------------------------------------------
//
// Pictures taken (or, with no capture command on record, first downloaded)
// more than a week before the cache was written are kept in it, as built,
// and the rest are rebuilt from the DB at every start (see viewer_cache.h for
// the policy). A capture goes into the file as its struct followed by its
// file bytes, which of them arrived, and its JPEG; the textures are made
// from the JPEG once the window is up, as for a capture built from the DB.

#define CAM_CACHE_NAME "frontiersat_camera_viewer"
// The attitudes keep a cache of their own (beacon_attitude.h).
#define CAM_ATTITUDE_CACHE_NAME "frontiersat_camera_viewer_attitude"

// What a load says about where its captures came from, for the viewer to
// pass on.
typedef struct {
    int    from_cache;   // captures read out of the cache
    int    newer;        // newer downloads of cached pictures left out
    double cutoff_ms;    // the cache holds pictures from before this
} load_note_t;

static void write_cache(const char *db_path, const capture_t *caps, int n,
                        double cutoff_ms, long long max_id)
{
    viewer_cache_info_t info = { .cutoff_ms = cutoff_ms, .max_id = max_id };
    viewer_cache_writer_t w;
    if (viewer_cache_create(&w, CAM_CACHE_NAME, db_path, &info) != 0) {
        fprintf(stderr, "frontiersat_camera_viewer: cannot write the cache; "
                        "every start will rebuild\n");
        return;
    }
    int nc = 0;
    for (int k = 0; k < n; k++) if (capture_moment(&caps[k]) < cutoff_ms) nc++;
    viewer_cache_write(&w, &nc, sizeof nc);
    for (int k = 0; k < n; k++) {
        const capture_t *c = &caps[k];
        if (capture_moment(c) >= cutoff_ms) continue;
        viewer_cache_write(&w, c, sizeof *c);
        viewer_cache_write(&w, c->buf, (size_t) c->size);
        viewer_cache_write(&w, c->present, (size_t) c->size);
        viewer_cache_write(&w, c->jpg, (size_t) c->jpg_len);
    }
    if (viewer_cache_commit(&w) != 0)
        fprintf(stderr, "frontiersat_camera_viewer: cannot write the cache %s\n", w.path);
}

// The captures in an open cache file. Returns the count, or -1 if the file is
// damaged.
static int read_cache(FILE *f, capture_t **out)
{
    *out = NULL;
    int n = 0;
    if (viewer_cache_read(f, &n, sizeof n) != 0 || n < 0 || n > 100000) return -1;
    capture_t *caps = (capture_t *) calloc((size_t) (n > 0 ? n : 1), sizeof *caps);
    if (caps == NULL) return -1;
    for (int k = 0; k < n; k++) {
        capture_t *c = &caps[k];
        if (viewer_cache_read(f, c, sizeof *c) != 0 || c->size < 0 || c->jpg_len < 0) {
            memset(c, 0, sizeof *c);
            free_captures(caps, k);
            return -1;
        }
        // The writer's pointers and textures mean nothing here.
        c->tex_ok = 0;
        c->fav = 0;
        c->buf = viewer_cache_read_alloc(f, (size_t) c->size);
        c->present = c->buf ? viewer_cache_read_alloc(f, (size_t) c->size) : NULL;
        c->jpg = c->present ? viewer_cache_read_alloc(f, (size_t) c->jpg_len) : NULL;
        if (c->jpg == NULL) {
            free_captures(caps, k + 1);
            return -1;
        }
    }
    *out = caps;
    return n;
}

// Load the captures to show: the cached ones and the rest rebuilt from the
// DB, or with `rebuild` (or no usable cache) every one of them rebuilt, and
// the cache written afresh. Returns the count (0 on any failure). Used at
// startup and on F5 (shift-F5 rebuilds).
static int load_captures(const char *db_path, int rebuild, capture_t **out,
                         load_note_t *note)
{
    *out = NULL;
    memset(note, 0, sizeof *note);
    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "frontiersat_camera_viewer: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 0;
    }
    // Read before the packets are, so a row that lands during the load is
    // looked at again next time rather than never.
    long long max_id = viewer_cache_db_max_id(db);
    double cutoff = viewer_cache_now_ms() - VIEWER_CACHE_SETTLED_MS;

    viewer_cache_info_t ci = {0};
    capture_t *cached = NULL;
    int nc = -1;
    FILE *cf = (rebuild || max_id < 0) ? NULL
             : viewer_cache_open(CAM_CACHE_NAME, db_path, max_id, &ci);
    if (cf != NULL) {
        nc = read_cache(cf, &cached);
        fclose(cf);
    }

    if (nc < 0) {
        int n = captures_from_db(db, NULL, 0, out);
        sqlite3_close(db);
        if (n > 0 && max_id >= 0) write_cache(db_path, *out, n, cutoff, max_id);
        return n;
    }

    char since[32];
    viewer_cache_iso(ci.cutoff_ms - VIEWER_CACHE_MARGIN_MS, since, sizeof since);
    capture_t *fresh = NULL;
    int nf = captures_from_db(db, since, ci.max_id, &fresh);
    sqlite3_close(db);

    // A fresh capture from before the cutoff, or that is one of the cached
    // pictures, is part of one the cache already holds: the stretch of its
    // download that falls inside the window. Only packets the cache has not
    // seen make it worth telling the operator about.
    capture_t *all = (capture_t *) malloc((size_t) (nc + nf > 0 ? nc + nf : 1) * sizeof *all);
    if (all == NULL) {
        free_captures(cached, nc);
        free_captures(fresh, nf);
        return 0;
    }
    int n = 0;
    for (int k = 0; k < nc; k++) all[n++] = cached[k];
    for (int k = 0; k < nf; k++) {
        capture_t *c = &fresh[k];
        int known = 0;
        for (int j = 0; j < nc && !known; j++)
            known = same_picture(c->buf, c->present, c->size, c->sat_path,
                                 cached[j].buf, cached[j].present, cached[j].size,
                                 cached[j].sat_path);
        if (!known && capture_moment(c) >= ci.cutoff_ms) {
            all[n++] = *c;
            continue;
        }
        if (c->max_id > ci.max_id) note->newer++;
        free(c->buf); free(c->present); free(c->jpg);
    }
    free(cached);
    free(fresh);
    qsort(all, (size_t) n, sizeof *all, by_newest_first);
    note->from_cache = nc;
    note->cutoff_ms = ci.cutoff_ms;

    // Move the cutoff up to a week ago once it has fallen a day behind. The
    // pictures that cross it were rebuilt whole just now, every packet of
    // theirs having come down after the picture was taken. Held back while
    // there are newer downloads the cache has not taken in, which moving the
    // cutoff past them would forget.
    if (note->newer == 0 && cutoff - ci.cutoff_ms >= VIEWER_CACHE_ROLL_MS) {
        write_cache(db_path, all, n, cutoff, max_id);
        note->from_cache = 0;
        for (int k = 0; k < n; k++) if (capture_moment(&all[k]) < cutoff) note->from_cache++;
        note->cutoff_ms = cutoff;
    }
    *out = all;
    return n;
}

// The line that says the DB has downloads of cached pictures the cache has not
// taken in; "" when it has none.
static void newer_note(const load_note_t *note, char *out, size_t n)
{
    out[0] = '\0';
    if (note->newer == 0) return;
    char day[24];
    fmt_utc_s(note->cutoff_ms, day, sizeof day);
    day[10] = '\0';
    snprintf(out, n, "%d newer download%s of pictures from before %s not shown: "
             "shift-F5 rebuilds them", note->newer, note->newer == 1 ? "" : "s", day);
}

// ---- textures --------------------------------------------------------------

// Decode each capture's JPEG into a texture, with a histogram-equalized and a
// colour-stretched copy beside it, so that e and c switch between them at no
// cost. A picture with too many missing bytes can fail to decode; that capture
// keeps tex_ok = 0 and the viewer says so instead of drawing it. Must run
// after InitWindow.
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
            ImageFormat(&im, PIXELFORMAT_UNCOMPRESSED_R8G8B8);
            long npix = (long) im.width * im.height;
            Image st = ImageCopy(im);
            cam_jpeg_stretch_rgb8((uint8_t *) st.data, npix);
            c->tex_st = LoadTextureFromImage(st);
            SetTextureFilter(c->tex_st, TEXTURE_FILTER_BILINEAR);
            UnloadImage(st);
            cam_jpeg_equalize_rgb8((uint8_t *) im.data, npix);
            c->tex_eq = LoadTextureFromImage(im);
            SetTextureFilter(c->tex_eq, TEXTURE_FILTER_BILINEAR);
            c->tex_ok = 1;
            c->img_w = im.width;
            c->img_h = im.height;
        }
        UnloadImage(im);
    }
}

static void unload_textures(capture_t *caps, int n)
{
    for (int k = 0; k < n; k++) {
        if (!caps[k].tex_ok) continue;
        UnloadTexture(caps[k].tex);
        UnloadTexture(caps[k].tex_eq);
        UnloadTexture(caps[k].tex_st);
        caps[k].tex_ok = 0;
    }
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

// The favourite mark: a five-pointed star centred on (cx, cy), r the radius
// of its points. Each of the ten slices is drawn in both windings, since
// raylib drops a triangle whose corners come in the order it culls.
#define FAV_GOLD ((Color){ 240, 196, 64, 255 })

static void draw_star(int cx, int cy, float r, Color col)
{
    Vector2 o = { (float) cx, (float) cy }, p[11];
    for (int k = 0; k <= 10; k++) {
        float a = -PI / 2.0f + (float) k * PI / 5.0f;
        float rr = (k % 2) ? r * 0.42f : r;
        p[k] = (Vector2){ o.x + rr * cosf(a), o.y + rr * sinf(a) };
    }
    for (int k = 0; k < 10; k++) {
        DrawTriangle(o, p[k], p[k + 1], col);
        DrawTriangle(o, p[k + 1], p[k], col);
    }
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
    snprintf(out, n, "fs_boomcam_%s.jpg", c->id);
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

// ---- what the viewer remembers between runs --------------------------------
//
// A short text file of key=value lines:
//
//   selected=20260820_072204
//   favourites_only=1
//   favourite=20260601_181502
//   favourite=20260820_072204
//
// Pictures are named by their id (see capture_t), so the file means the same
// thing whichever database is open and however the list is ordered. The
// favourites are kept as names, not as flags on the open list, so a favourite
// that this database does not hold is carried over untouched rather than
// forgotten the next time the file is written.

#define STATE_RELPATH ".local/state/simple_sat_ops/frontiersat_camera_viewer.state"
#define MAX_FAVOURITES 512

typedef struct {
    char selected[24];
    int  favourites_only;
    char fav[MAX_FAVOURITES][24];
    int  nfav;
} viewer_state_t;

static int state_path(char *out, size_t n)
{
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') return -1;
    int w = snprintf(out, n, "%s/%s", home, STATE_RELPATH);
    return (w > 0 && (size_t) w < n) ? 0 : -1;
}

static int state_find_fav(const viewer_state_t *s, const char *id)
{
    for (int k = 0; k < s->nfav; k++)
        if (strcmp(s->fav[k], id) == 0) return k;
    return -1;
}

// A missing or unreadable file is a first run: everything starts empty.
static void state_load(viewer_state_t *s)
{
    memset(s, 0, sizeof *s);
    char path[1024];
    if (state_path(path, sizeof path) != 0) return;
    FILE *f = fopen(path, "r");
    if (f == NULL) return;
    char line[128];
    while (fgets(line, sizeof line, f) != NULL) {
        line[strcspn(line, "\r\n")] = '\0';
        char *eq = strchr(line, '=');
        if (eq == NULL) continue;
        *eq = '\0';
        const char *val = eq + 1;
        if (strcmp(line, "selected") == 0)
            snprintf(s->selected, sizeof s->selected, "%s", val);
        else if (strcmp(line, "favourites_only") == 0)
            s->favourites_only = atoi(val) != 0;
        else if (strcmp(line, "favourite") == 0 && val[0] != '\0'
                 && s->nfav < MAX_FAVOURITES && state_find_fav(s, val) < 0)
            snprintf(s->fav[s->nfav++], sizeof s->fav[0], "%s", val);
    }
    fclose(f);
}

// Written to a temporary name and renamed over the old file, so a crash
// part-way through never leaves the favourites half written. Returns 0, or -1
// when the file could not be written (the viewer carries on regardless).
static int state_save(const viewer_state_t *s)
{
    char path[1024], tmp[1100];
    if (state_path(path, sizeof path) != 0) return -1;
    if (sso_mkdir_p_for_file(path) != 0) return -1;
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (f == NULL) return -1;
    fprintf(f, "# frontiersat_camera_viewer: the last picture looked at and the favourites\n");
    fprintf(f, "selected=%s\n", s->selected);
    fprintf(f, "favourites_only=%d\n", s->favourites_only);
    for (int k = 0; k < s->nfav; k++) fprintf(f, "favourite=%s\n", s->fav[k]);
    int bad = ferror(f);
    if (fclose(f) != 0) bad = 1;
    if (bad || rename(tmp, path) != 0) { remove(tmp); return -1; }
    return 0;
}

static void mark_favourites(capture_t *caps, int n, const viewer_state_t *s)
{
    for (int k = 0; k < n; k++) caps[k].fav = state_find_fav(s, caps[k].id) >= 0;
}

// The rows the list shows, as capture indices: every picture, or only the
// favourites. Returns the row count.
static int build_rows(const capture_t *caps, int n, int favourites_only, int *rows)
{
    int nr = 0;
    for (int k = 0; k < n; k++)
        if (!favourites_only || caps[k].fav) rows[nr++] = k;
    return nr;
}

// The row a capture sits on, or -1 when the list is not showing it.
static int row_of(const int *rows, int nrows, int cap)
{
    for (int r = 0; r < nrows; r++) if (rows[r] == cap) return r;
    return -1;
}

static int capture_by_id(const capture_t *caps, int n, const char *id)
{
    for (int k = 0; k < n; k++) if (strcmp(caps[k].id, id) == 0) return k;
    return -1;
}

// ---- main ------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "frontiersat_camera_viewer")) return 0;

    const char *db_arg = NULL;
    int rebuild = 0;
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--db=", 5) == 0) db_arg = argv[i] + 5;
        else if (strcmp(argv[i], "--rebuild-cache") == 0) rebuild = 1;
        else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: frontiersat_camera_viewer [--db=<packet_db.sqlite>] [--rebuild-cache]\n"
                   "Look at FrontierSat's boom-camera pictures rebuilt from the packet DB.\n"
                   "The left panel lists captures, newest first, one row per picture; F5 re-reads the DB.\n"
                   "Pictures taken more than a week ago come out of a cache in\n"
                   "~/.cache/simple_sat_ops/ and are rebuilt only by shift-F5 or\n"
                   "--rebuild-cache; the viewer says when the DB has newer downloads\n"
                   "of them than the cache.\n"
                   "e shows the picture histogram-equalized, c with its colours stretched to\n"
                   "full scale; the same key again shows it as received.\n"
                   "f tags a favourite, v shows the favourites only. Favourites, that choice\n"
                   "and the last picture viewed are kept in ~/" STATE_RELPATH ".\n");
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
    load_note_t note;
    int ncaps = load_captures(db_path, rebuild, &caps, &note);
    if (ncaps == 0) {
        fprintf(stderr, "frontiersat_camera_viewer: no boom-camera captures found.\n");
        return 1;
    }
    fprintf(stderr, "frontiersat_camera_viewer: %d capture%s",
            ncaps, ncaps == 1 ? "" : "s");
    if (note.from_cache > 0) fprintf(stderr, ", %d of them from the cache", note.from_cache);
    fprintf(stderr, ".\n");
    char newer[160];
    newer_note(&note, newer, sizeof newer);
    if (newer[0]) fprintf(stderr, "frontiersat_camera_viewer: %s.\n", newer);

    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "frontiersat_camera_viewer");
    SetTargetFPS(60);
    g_ui_font_loaded = load_ui_font();
    build_textures(caps, ncaps);

    // The globe under the capture list. The map and the element sets are read
    // once here; the track itself is built the first time round the loop.
    globe_t globe = {0};
    globe.zoom = GLOBE_ZOOM_MIN;
    globe.fov_across_deg = CAM_FOV_ACROSS_DEG;
    globe.fov_down_deg   = CAM_FOV_DOWN_DEG;
    globe_set_font(g_ui_font, g_ui_font_loaded, g_ui_font_spacing);
    globe_load_map(&globe, "frontiersat_camera_viewer");
    if (globe_load_tles(db_path) == 0)
        fprintf(stderr, "frontiersat_camera_viewer: no FrontierSat TLEs in the "
                        "DB; the globe will show no ground track\n");
    // The attitudes the extended beacons and the ADCS's SD-card logs
    // carry, for drawing which way the camera was facing. With neither
    // near the moment, the globe shows the ground track on its own.
    beacon_attitude_load(db_path, CAM_ATTITUDE_CACHE_NAME, rebuild);

#ifdef __APPLE__
    sso_install_pinch_monitor();
#endif

    const int LEFT_W = 340;

    // What the last run left: the picture it was on, whether it was showing
    // only the favourites, and the favourites themselves. A picture that
    // has gone (another database, say) falls back to the newest.
    viewer_state_t vs;
    state_load(&vs);
    mark_favourites(caps, ncaps, &vs);
    int *rows = (int *) malloc((size_t) ncaps * sizeof(int));
    if (rows == NULL) { fprintf(stderr, "frontiersat_camera_viewer: out of memory\n"); return 1; }
    int nrows = build_rows(caps, ncaps, vs.favourites_only, rows);
    if (nrows == 0) {
        vs.favourites_only = 0;
        nrows = build_rows(caps, ncaps, 0, rows);
    }
    int sel = capture_by_id(caps, ncaps, vs.selected);
    if (sel < 0 || row_of(rows, nrows, sel) < 0) sel = rows[0];
    // How the picture is being looked at: 1 is the whole picture fitted to the
    // pane, pan is how far it has been dragged from the middle. Both go back
    // to the start with every change of picture.
    float zoom = 1.0f;
    Vector2 pan = { 0, 0 };
    int dragging = 0;
    // How the picture is shown: as received, histogram-equalized (e) or with
    // its colours stretched (c). It stays as set from one picture to the
    // next, and every run opens on the pictures as received.
    enum { SHOW_AS_RECEIVED, SHOW_EQUALIZED, SHOW_STRETCHED } show = SHOW_AS_RECEIVED;
    char status[160] = "";
    float status_left = 0.0f;
    float rep_up = 0, rep_down = 0;
    // Where the capture list is scrolled to, in pixels. The reader's, not
    // the selection's: it is pulled back only far enough to keep the
    // selected picture in view, so scrolling away to look at the rest of
    // the list stays where you put it.
    float list_px = 0.0f;
    int   last_sel = -1;

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

        // The left column: the capture list above, the globe below. Worked out
        // before the input, since a press on the globe and a press on the list
        // are told apart by where the two panels are. A window too short to
        // hold the globe gives the whole column to the list.
        const int row_h = 44, list_top = 40;
        int col_bottom = sh - FOOTER_H;
        int globe_h = GLOBE_PANEL_H;
        int room = col_bottom - list_top - 2 * row_h;
        if (globe_h > room) globe_h = room;
        if (globe_h < GLOBE_PANEL_MIN_H) globe_h = 0;
        int globe_y = col_bottom - globe_h;
        int list_h = (globe_h ? globe_y : col_bottom) - list_top;
        if (list_h < 0) list_h = 0;

        // ---- input ----
        // The globe takes its own drags and its own wheel, so scrolling over
        // it zooms rather than running the list underneath it.
        if (globe_h > 0) globe_input(&globe, 0, globe_y, LEFT_W, globe_h);

        // The capture list: the wheel scrolls it, a press picks a picture.
        // The same handling mpi_viewer's experiment list has, so the two
        // read alike.
        int moved = 0;
        {
            Vector2 lm = GetMousePosition();
            int over_list = lm.x >= 0 && lm.x < LEFT_W && lm.y >= list_top
                            && lm.y < list_top + list_h;
            if (over_list) list_px -= GetMouseWheelMove() * (float) row_h;
            if (over_list && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                int r = (int) ((lm.y - (float) list_top + list_px) / (float) row_h);
                if (r >= 0 && r < nrows && rows[r] != sel) { sel = rows[r]; moved = 1; }
            }
        }
        // Up and Down step through the rows on show, so with the
        // favourites filter on they skip from favourite to favourite.
        int srow = row_of(rows, nrows, sel);
        if (key_repeat(KEY_DOWN, &rep_down) && srow < nrows - 1) { sel = rows[srow + 1]; moved = 1; }
        if (key_repeat(KEY_UP, &rep_up)     && srow > 0)         { sel = rows[srow - 1]; moved = 1; }

        // f tags or untags the picture on show. With only the favourites
        // on show, untagging one takes it off the list, so the selection
        // moves to the row that slid into its place -- and untagging the
        // last one goes back to the whole list rather than to an empty one.
        if (IsKeyPressed(KEY_F)) {
            int k = state_find_fav(&vs, caps[sel].id);
            if (k >= 0) {
                memmove(vs.fav[k], vs.fav[k + 1], (size_t) (vs.nfav - k - 1) * sizeof vs.fav[0]);
                vs.nfav--;
                snprintf(status, sizeof status, "taken off the favourites");
            } else if (vs.nfav < MAX_FAVOURITES) {
                snprintf(vs.fav[vs.nfav++], sizeof vs.fav[0], "%s", caps[sel].id);
                snprintf(status, sizeof status, "added to the favourites");
            } else {
                snprintf(status, sizeof status, "favourites full (%d)", MAX_FAVOURITES);
            }
            mark_favourites(caps, ncaps, &vs);
            nrows = build_rows(caps, ncaps, vs.favourites_only, rows);
            if (nrows == 0) {
                vs.favourites_only = 0;
                nrows = build_rows(caps, ncaps, 0, rows);
                snprintf(status, sizeof status, "no favourites left; showing every picture");
            }
            if (row_of(rows, nrows, sel) < 0) {
                sel = rows[srow < nrows ? srow : nrows - 1];
                moved = 1;
            }
            last_sel = -1;
            if (state_save(&vs) != 0) snprintf(status, sizeof status, "could not write the state file");
            status_left = 6.0f;
        }
        // v switches between every picture and only the favourites.
        if (IsKeyPressed(KEY_V)) {
            int want = !vs.favourites_only;
            if (want && build_rows(caps, ncaps, 1, rows) == 0) {
                snprintf(status, sizeof status, "no favourites yet -- f tags the picture on show");
            } else {
                vs.favourites_only = want;
                snprintf(status, sizeof status, want ? "showing the favourites only"
                                                     : "showing every picture");
                if (state_save(&vs) != 0) snprintf(status, sizeof status, "could not write the state file");
            }
            nrows = build_rows(caps, ncaps, vs.favourites_only, rows);
            if (row_of(rows, nrows, sel) < 0) { sel = rows[0]; moved = 1; }
            last_sel = -1;
            status_left = 6.0f;
        }
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

        // e: the picture histogram-equalized; c: its colours stretched to
        // full scale. One filter at a time, and the same key again goes back
        // to the picture as received. Only the screen changes; s and o still
        // hand over the JPEG as received.
        if (IsKeyPressed(KEY_E) || IsKeyPressed(KEY_C)) {
            int want = IsKeyPressed(KEY_E) ? SHOW_EQUALIZED : SHOW_STRETCHED;
            show = show == want ? SHOW_AS_RECEIVED : want;
            snprintf(status, sizeof status, "%s",
                     show == SHOW_EQUALIZED ? "showing the picture histogram-equalized"
                     : show == SHOW_STRETCHED ? "showing the picture with its colours stretched"
                     : "showing the picture as received");
            status_left = 6.0f;
        }
        if (IsKeyPressed(KEY_S)) { save_jpeg(c, status, sizeof status); status_left = 6.0f; }
        if (IsKeyPressed(KEY_O)) { open_in_system_viewer(c, status, sizeof status); status_left = 6.0f; }
        // F5 re-reads the DB; shift-F5 rebuilds the cached pictures too, and
        // the cache with them.
        if (IsKeyPressed(KEY_F5)) {
            int full = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            capture_t *nc = NULL;
            load_note_t nnote;
            int nn = load_captures(db_path, full, &nc, &nnote);
            int *nr = nn > 0 ? (int *) realloc(rows, (size_t) nn * sizeof(int)) : NULL;
            if (nr != NULL) {
                // Stay on the same picture, found by name: a new picture at
                // the top of the list moves every row below it down one.
                char keep[24];
                snprintf(keep, sizeof keep, "%s", caps[sel].id);
                unload_textures(caps, ncaps);
                free_captures(caps, ncaps);
                caps = nc; ncaps = nn;
                rows = nr;
                build_textures(caps, ncaps);
                mark_favourites(caps, ncaps, &vs);
                nrows = build_rows(caps, ncaps, vs.favourites_only, rows);
                if (nrows == 0) {
                    vs.favourites_only = 0;
                    nrows = build_rows(caps, ncaps, 0, rows);
                }
                sel = capture_by_id(caps, ncaps, keep);
                if (sel < 0 || row_of(rows, nrows, sel) < 0) sel = rows[0];
                last_sel = -1;
                c = &caps[sel];
                zoom = 1.0f;
                pan = (Vector2){ 0, 0 };
                globe_load_tles(db_path);
                beacon_attitude_load(db_path, CAM_ATTITUDE_CACHE_NAME, full);
                globe.track_key[0] = 0;      // and a fresh ground track
                note = nnote;
                newer_note(&note, newer, sizeof newer);
                snprintf(status, sizeof status, "%s: %d capture%s",
                         full ? "rebuilt from the whole DB" : "reloaded",
                         ncaps, ncaps == 1 ? "" : "s");
            } else {
                if (nc != NULL) free_captures(nc, nn);
                snprintf(status, sizeof status, "reload found no captures; keeping the old list");
            }
            status_left = 6.0f;
        }
        // g: the globe back to how it opens -- the whole Earth, framed on the
        // track -- from wherever turning and zooming left it.
        if (IsKeyPressed(KEY_G)) globe_reset_view(&globe);
        // p: the outline of what the picture covered, on the globe or off.
        // It needs an attitude to project, so say when this picture has none.
        if (IsKeyPressed(KEY_P)) {
            globe.show_fov = !globe.show_fov;
            if (!globe.show_fov)
                snprintf(status, sizeof status, "picture outline off");
            else if (globe.att.have)
                snprintf(status, sizeof status, "picture outline on");
            else
                snprintf(status, sizeof status,
                         "picture outline on -- no attitude on record for this one");
            status_left = 6.0f;
        }
        if (IsKeyPressed(KEY_Q)) break;
        if (status_left > 0.0f) status_left -= GetFrameTime();

        // Where the satellite was when this picture was taken -- or, when no
        // camera_capture command is on record to say, when the first pass
        // downloaded it, which is a different place and is labelled as such.
        // Pull the list back to the selected row when the selection moves,
        // and keep whatever the reader scrolled to when it does not.
        if (sel != last_sel) {
            float want_top = (float) (row_of(rows, nrows, sel) * row_h);
            if (list_px > want_top) list_px = want_top;
            if (list_px < want_top + (float) row_h - (float) list_h)
                list_px = want_top + (float) row_h - (float) list_h;
            // Remembered as it changes, so a killed window still opens on
            // this picture next time.
            if (strcmp(vs.selected, caps[sel].id) != 0) {
                snprintf(vs.selected, sizeof vs.selected, "%s", caps[sel].id);
                state_save(&vs);
            }
            last_sel = sel;
        }
        float list_max = (float) (nrows * row_h - list_h);
        if (list_max < 0.0f) list_max = 0.0f;
        if (list_px > list_max) list_px = list_max;
        if (list_px < 0.0f) list_px = 0.0f;

        double dot_ms = c->t_capture_ms > 0 ? c->t_capture_ms : c->t_first_ms;

        // A different picture is a different piece of ground track, a different
        // orbit and a different Sun. Keyed on the row as well as the moment, so
        // two pictures that share a timestamp still each get their own track.
        const char *track_key = TextFormat("%d %.0f", sel, dot_ms);
        if (strcmp(globe.track_key, track_key) != 0) {
            globe_set_track(&globe, track_key,
                            dot_ms - CAM_TRACK_HALF_MS, dot_ms + CAM_TRACK_HALF_MS);
            // Which way it was facing when the shutter went, from the
            // nearest extended beacon or log record that carried an
            // attitude. Most pictures have none -- the blob has to have
            // been running, or a log covering the moment been downloaded,
            // and the ADCS has to have been estimating -- and then the
            // globe draws the track without a pointing direction.
            globe_attitude_t att = {0};
            beacon_attitude_nearest(dot_ms, BEACON_ATTITUDE_MAX_AGE_S, &att);
            globe_set_attitude(&globe, &att);
        }

        // ---- draw ----
        BeginDrawing();
        ClearBackground((Color){ 18, 18, 22, 255 });

        // left: capture list
        DrawRectangle(0, 0, LEFT_W, sh, (Color){ 28, 28, 34, 255 });
        draw_text(vs.favourites_only ? "Favourites" : "Camera captures", 12, 10, 18, RAYWHITE);
        {
            const char *count = vs.favourites_only ? TextFormat("%d of %d", nrows, ncaps)
                                                   : TextFormat("%d", ncaps);
            draw_text(count, LEFT_W - 12 - 8 * (int) strlen(count), 14, 12, GRAY);
        }
        // A row half off the bottom is cut at the globe's edge rather than
        // drawn over it.
        BeginScissorMode(0, list_top, LEFT_W, list_h);
        for (int r = 0; r < nrows; r++) {
            int si = rows[r];
            capture_t *cc = &caps[si];
            int y = list_top + r * row_h - (int) list_px;
            if (y + row_h < list_top || y > list_top + list_h) continue;
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
            if (cc->fav) draw_star(LEFT_W - 22, y + row_h / 2, 8, FAV_GOLD);
        }
        EndScissorMode();
        // A thumb down the right edge, only while there is more list than
        // room -- the same one mpi_viewer's experiment list carries.
        if (list_max > 0.0f) {
            float frac = (float) list_h / (float) (nrows * row_h);
            int th = (int) ((float) list_h * frac);
            if (th < 20) th = 20;
            int ty = list_top + (int) ((float) (list_h - th) * list_px / list_max);
            DrawRectangle(LEFT_W - 5, ty, 3, th, (Color){ 90, 90, 105, 255 });
        }

        // the globe, under the list
        if (globe_h > 0)
            globe_draw(&globe, 0, globe_y, LEFT_W, globe_h, dot_ms,
                       c->t_capture_ms > 0 ? "Ground track at capture"
                                           : "Ground track at downlink");

        // right: title, with the star after it when this is a favourite
        {
            const char *title = c->sat_path[0] ? c->sat_path : "(satellite file unknown)";
            draw_text(title, rx, 12, 20, RAYWHITE);
            if (c->fav) {
                int tw = g_ui_font_loaded
                       ? (int) MeasureTextEx(g_ui_font, title, 20, g_ui_font_spacing).x
                       : MeasureText(title, 20);
                draw_star(rx + tw + 18, 22, 9, FAV_GOLD);
            }
        }
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
            Texture2D pic = show == SHOW_EQUALIZED ? c->tex_eq
                          : show == SHOW_STRETCHED ? c->tex_st : c->tex;
            DrawTexturePro(pic, src, dst, (Vector2){ 0, 0 }, 0.0f, WHITE);
            EndScissorMode();
            // Said above the corner of the pane for as long as a filter is
            // on, since the picture on show is not what the camera recorded.
            if (show != SHOW_AS_RECEIVED) {
                const char *label = show == SHOW_EQUALIZED ? "histogram equalized"
                                                           : "colours stretched to full scale";
                int lw = g_ui_font_loaded
                       ? (int) MeasureTextEx(g_ui_font, label, 15, g_ui_font_spacing).x
                       : MeasureText(label, 15);
                draw_text(label, px + pw - lw, 42, 15, FAV_GOLD);
            }
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
        draw_text(TextFormat("jpeg         : %ld bytes%s%s", c->jpg_len,
                             c->tex_ok ? TextFormat("  %d x %d", c->img_w, c->img_h)
                                       : "  (does not decode)",
                             c->st.header_supplied ? ", standard header" : ""),
                  rx, iy + 40, 15, LIGHTGRAY);

        draw_text(TextFormat("packets      : %d", c->chunks), col2, iy, 15, LIGHTGRAY);
        draw_text(c->tex_ok ? TextFormat("zoom         : %.0f%%%s", shown_scale * 100.0f,
                                         zoom <= 1.0f ? "  (whole picture)" : "")
                            : "zoom         : -",
                  col2, iy + 20, 15, LIGHTGRAY);
        // The last action's outcome; otherwise whether the cache is behind
        // the DB, which stays up until it is rebuilt.
        if (status_left > 0.0f)
            draw_text(status, col2, iy + 40, 15, (Color){ 120, 220, 160, 255 });
        else if (newer[0])
            draw_text(newer, col2, iy + 40, 15, (Color){ 240, 196, 64, 255 });

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
        draw_text("Up/Down capture   scroll or pinch zoom   drag to move   e equalize   c stretch colours   "
                  "globe: drag turns, scroll zooms, g resets, p outlines picture   "
                  "f favourite   v favourites only   o open   s save jpeg   F5 refresh   shift-F5 rebuild   q quit",
                  12, sh - 22, 12, (Color){ 150, 150, 160, 255 });

        EndDrawing();
    }

    snprintf(vs.selected, sizeof vs.selected, "%s", caps[sel].id);
    state_save(&vs);

    globe_free(&globe);
    beacon_attitude_free();
    unload_textures(caps, ncaps);
    if (g_ui_font_loaded) UnloadFont(g_ui_font);
    CloseWindow();

    free(rows);
    free_captures(caps, ncaps);
    return 0;
}

#endif // WITH_SQLITE3
