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

    Each frame carries aux/housekeeping fields then a strip of image pixels, and
    the two are one frame apart: the pixels were integrated during the previous
    frame's slot, while the aux fields are stamped when the frame is packed, by
    which time the dome has already been commanded to the next setpoint. So a
    frame's bias is the target voltage the frame BEFORE it reported. What gives
    it away is the frame the MPI sends without subtracting its background: it
    comes back around 20700 DN where its neighbours read about 2010, one of the
    bright vertical stripes in the whereogram, and its own aux claims the next
    setpoint down even though it was taken at 0 V. Reading that aux at face
    value drew it one column in from the right instead of at the edge.

    One full inner-dome voltage sweep is one image: fpi frames (the scan period,
    auto-detected, 16) in fpi columns, one for each. The dome sits at 0 V for
    two slots running, straddling the turn of the sweep, and the second of those
    two is the reference: the instrument keeps that frame's counts as its
    no-ion-signal background and takes them off every frame that follows, until
    it estimates again -- every 256 frames, every 16 images, by default, and NOT
    once a sweep. So one image in 16 has its first frame come down raw, being
    the background itself; in the other 15 that frame is the same 0 V dwell with
    the last estimate taken off it, and reads no differently from the rest of
    the sweep (measured at 2012 DN against a sweep average of 2016). Either way
    it is the image's first frame, at the far LEFT. Increasingly negative bias
    voltages run to the right from
    there, most negative (setpoint 4095) next to last, and the far right column
    is the other frame of the dwell: 0 V read again with the background taken
    off it, which is the residue the subtraction leaves. Laying the sweep out in
    the order it was flown means left to right is earliest to latest, the same
    direction as the whereogram below, so an image reads as the stretch of
    whereogram its playback head is sitting on rather than as a mirror of it,
    and consecutive images tile that recording with nothing left over. A new
    image begins one frame past a multiple of the scan period, which the scan
    index (aux byte 13) counts off. The tool plays the images back like a movie, grayscale, with
    an adjustable DN range and nearest-neighbour zoom, and shows the aux data
    beside each image -- that aux belonging, as above, to the frame after the
    one the pixels were taken in.

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
        20..149 pixels, 65 big-endian uint16
        150..151 checksum                    (not a pixel)

    The pixels start at byte 20, not 22, and the last two bytes are a checksum.
    Both fit the 152 bytes exactly -- 65 words from 20 leaves the last two over,
    65 words from 22 uses them up -- and the First Light notebook's
    [[23;;-1;;2]] reads them from 22, which is where this tool read them until
    2026-08-30. The flight firmware settles it: MPITelemetry.h sets
    TM_AUXDATA_BUFFER_SIZE to 20 and TM_BUFFER_PIXEL_OFFSET to that same 20,
    storeTelemetryAuxData fills TM_buf[0..19] and stops, storeTelemetryPixelData
    writes pixel i to TM_BUFFER_PIXEL_OFFSET + 2*(i - FirstPixelIndex), and
    calculateTelemetryCrc puts a CCITT CRC-16 over everything before it into the
    last two bytes, most significant first. The data says the same: bytes 20..21
    sit at 2057 DN beside neighbours at 2112 and 2100, track the pixel next to
    them across frames at r = +0.99, and jump ninefold to 18695 with the rest of
    the strip on a background frame; bytes 150..151 average 32991 -- uniform
    across the 16-bit range, which is what a checksum looks like -- do not
    correlate with the pixel before them (r = -0.04), and go DOWN on a
    background frame where every pixel goes up. And the CRC itself checks out
    over bytes 0..149 on every whole frame of the 2026-07-21 recording and all
    but 2 of the 2026-08-09 one. Read from 22 the image was one pixel out,
    missing the first and carrying the checksum as its bottom row.

    Cleaning (b): the MPI subtracts a background of its own in the FPGA, and
    this undoes it and puts a better one in its place. Every 256 frames -- 16
    images -- the instrument keeps a 0 V frame as its no-ion-signal background
    and takes those counts off every frame until the next one, landing the
    result on a fixed 2000 DN offset. The frame it kept is sent as measured, so
    it arrives carrying the whole background: the bright vertical stripes across
    the whereogram. Step 1 undoes that exactly -- add the range's background
    frame back on and take the 2000 DN off -- which leaves raw counts, and the
    stripes go, the frame the instrument kept having been raw counts already.
    Step 2 detrends every frame across its own strip: a straight line fitted by
    least squares through its first two and last two pixels (0, 1, 63 and 64)
    comes off the strip pixel by pixel and the 2000 DN goes back on, which takes
    out the block shifts a range corrected with an estimate that had drifted by
    the end of it leaves behind, and any tilt across the strip with them.
    Step 3 subtracts a background again, but a local one: the first frame of an
    image is the 0 V dwell read back and so holds no ion signal, and the
    pixel-by-pixel median (or mean, e) of those over the five images centred on
    this one (n cycles 1..9) comes off every frame of the image, with the 2000
    DN put back on so a cleaned count reads on the scale the frames arrived on.
    Only frames fit to be a sample go into that window -- all their bytes
    arrived, the frame's own CRC checks out, no JSON marker spliced in -- and
    the estimate is normalised by however many survive rather than by the width
    of the window.
    What is left has the same form the instrument sends, raw - background +
    2000, and differs only in the background being one measured a few images
    away instead of one up to 255 frames stale.
    The toggle applies to the image and the whereogram together. shift-B, while
    cleaning is on, stops after step 1 or step 2 instead of running all three,
    which is how you see what each step did rather than only the end of it.

    Whereogram: under the aux panel sits the whole recording as one spectrum,
    every frame a column of its 65 intensities, time left to right and arrival
    direction up the vertical axis -- which is the direction the ions came in
    from, hence the name. Five times as wide as it is high. Its horizontal axis
    is the file rather than the frames in hand, so a stretch that never came
    down takes its own width instead of being closed up, and is marked by a red
    rule above the panel. m cycles the colour map, which the image shares.

    Time ruler: a press on the whereogram jumps to that moment in the recording
    and dragging scrubs, and while the button is down the drag also measures.
    The stretch between where the press landed and the pointer is marked on the
    panel, with the times of its two ends read out in UTC and the span between
    them. It measures the recording, not the screen: the ends are the capture
    times of the frames nearest those two bytes, so a stretch that never came
    down costs no time, the same way it takes up no columns.

    Between sessions: on exit the viewer writes which experiment was open, where
    the playhead sat, and every view setting to
    ~/.local/state/simple_sat_ops/mpi_viewer.state, and reads it back at
    startup. Plain key = value text, checked on the way in; delete it to come
    back up on the defaults.

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
    every byte it contributes is marked, and any image with a frame resting on
    them is flagged. Both live in
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
#include "sso_paths.h"
#include "sso_version.h"

#include <ctype.h>
#include <float.h>
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
#define PIX_START      20    // first pixel byte within a frame
#define NPIX           65    // big-endian uint16 pixels per frame
#define SCAN_IDX_OFF   13    // inner dome scan index byte (the image row)
#define MAX_FPI        64    // largest scan period we render

// The 0 V background frame's target-voltage setpoint. It carries no ion signal
// by construction, so it is not a measurement of where anything came from: the
// whereogram leaves it out, and so does the intensity range scaled for it.
#define MPI_BACKGROUND_TV  65535

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

// "17:23:04.1" (UTC) from a unix-ms timestamp: the time of day alone, tenths
// of a second, for the two ends of a drag across the whereogram. The date is
// already on the panel above, and one recording runs for minutes.
static void fmt_tod_ms(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    int tenths = (int) ((ts_ms - (double) secs * 1000.0) / 100.0);
    struct tm tmv;
    gmtime_r(&secs, &tmv);
    snprintf(out, n, "%02d:%02d:%02d.%d",
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec, tenths);
}

// How long a stretch of recording lasted: "37.5 s" under a minute, "2 m 05.3 s"
// over one.
static void fmt_span(double ms, char *out, size_t n)
{
    double sec = ms / 1000.0;
    if (sec < 60.0) {
        snprintf(out, n, "%.1f s", sec);
        return;
    }
    int min = (int) (sec / 60.0);
    snprintf(out, n, "%d m %04.1f s", min, sec - 60.0 * (double) min);
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
// is keyed off the voltage. The firmware never commands m = N-1, and it
// commands m = N twice a sweep, so a sweep's N frames fill N-1 columns.

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
    int    ncols;          // image width in columns (== fpi: every frame of the sweep gets one)
    int    fpi_override;   // 0 = use the detected count, else forced 8/16
    int    cmap;           // colour map index, shared by the image and the whereogram
    int    zoom;
    int    scale_mode;
    int    dn_min, dn_max;
    int    sess_min, sess_max;
    int    playing;
    float  ips;
    float  accum;
    int    clean_stage;    // how much of the cleaning to apply (see the stage enum)
    int    clean_step;     // the step b comes back on at, and shift-B moves
    int    clean_win;      // images in the sliding background window (odd)
    int    clean_mean;     // background estimator: 0 = median, 1 = mean
    float *clean_pix;      // nframes * NPIX cleaned counts, NULL until built
    unsigned char *clean_ok;  // per frame: a cleaned value exists
    int   *key_fr;         // the frames that carried a background estimate
    int    n_key;
    int    key_step;       // frames between background estimates (256 by default)
    int    n_pre_key;      // frames ahead of the first background estimate
    int    clean_frames;   // frames a cleaned value was built for
    int    bg_samples;     // images whose first frame was fit to be a sample
    int    bg_short;       // images whose window lost at least one to that test
} view_t;

// Column of frame k within its image, from the inner-dome TARGET voltage (the
// commanded bias) -- of the frame BEFORE it, because a frame's pixels were
// integrated during the previous frame's slot while its aux was stamped a slot
// later (see the file header). So the previous frame has to really be the
// previous one: its counter one less than this one's, and its own aux sound.
//
// The columns are the physical scan axis, laid out in the order the sweep runs:
// the 0 V background at the far LEFT, then increasingly negative bias voltages
// to the RIGHT, so the most negative (setpoint 4095, m = 1) is next to last.
// Left to right is therefore also earliest to latest, the same direction the
// whereogram runs, which is what lets an image be read as the stretch of
// whereogram its playback head is sitting on -- and every frame of the sweep is
// drawn, so consecutive images tile that stretch with nothing left over.
//
// The dome sits at 0 V for two slots per sweep, so 0 V is measured at both ends
// of the image: the raw background at the left, and at the far right the same
// 0 V read again with that background taken off it, which is the residue the
// subtraction leaves. Both are real frames of the sweep and both get a column,
// which is why an image is fpi columns wide, not fpi-1.
//
// Returns -1 for an off-grid (bit-flipped) voltage, for the setpoint the
// firmware never commands (m = fpi-1, e.g. 61439 for fpi=16), and for a frame
// whose predecessor did not come down -- without it there is no telling which
// column the frame belongs in.
static int col_of(const view_t *v, const experiment_t *s, int k)
{
    if (k == 0 || s->fr_bad[k - 1]) return -1;
    if (((s->fr_ctr[k - 1] + 1) & 0xFFFF) != s->fr_ctr[k]) return -1;
    int r = tv_row(s->fr_tv[k - 1], v->fpi);  // 0..fpi-1 setpoint index, or -1
    if (r < 0) return -1;
    if (r == v->fpi - 1) {
        // Both frames of the 0 V dwell. The raw one is recognisable because its
        // own aux has already moved on to the first negative setpoint, while
        // the subtracted one still reads 0 V in its own aux; the sweep starts
        // on the first and ends on the second.
        if (tv_row(s->fr_tv[k], v->fpi) == v->fpi - 1) return v->fpi - 1;
        return 0;                              // raw background -> far left
    }
    if (r == v->fpi - 2) return -1;           // never-commanded setpoint
    return v->fpi - 2 - r;                     // rightward = more negative
}

// Detect the setpoint count for the current experiment; fpi is the scan period
// (frames per sweep) and ncols the image width, which is the same thing: every
// frame of the sweep gets a column (see col_of).
static void apply_scan(view_t *v, const experiment_t *s)
{
    v->n_targets = detect_n_targets(s);
    v->fpi = v->fpi_override ? v->fpi_override : v->n_targets;
    if (v->fpi < 2) v->fpi = 16;
    v->ncols = v->fpi;
}

// Assign every frame to an image. Frames are in file order; the image boundary
// comes from the inner-dome scan index (the clean per-frame step counter) -- a
// new image begins one frame past a multiple of fpi, at the raw frame that
// carries the 0 V dwell's pixels -- while the column within the image comes
// from the target voltage (see col_of). Corrupt frames (an off-grid target
// voltage, a JSON-straddle body, a missing predecessor) and the partial leading
// sweep are dropped (fr_img < 0).
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
        int boundary = (sc >= 0 && sc < 4 * v->fpi && sc % v->fpi == 1);
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

// ---- image cleaning ---------------------------------------------------------
//
// The MPI subtracts a background of its own in the FPGA, and cleaning undoes
// that and puts a better one in its place.
//
// What the instrument does: every so often -- 256 frames, 16 images, by default
// -- it keeps the frame it takes at the 0 V setpoint as its no-ion-signal
// background, and takes those counts off every frame that follows until the
// next such estimate, landing the result on a fixed offset of about 2000 DN.
// So what arrives is raw - background + 2000, except on the frame it kept,
// which is sent as measured and so arrives carrying the whole background:
// about ten times what its neighbours read, and the bright vertical stripes
// across the whereogram.
//
// Step 1 undoes exactly that, frame by frame: add back the range's background
// frame and take off the 2000 DN offset, which leaves the raw counts. The
// stripes go, because the frame the instrument kept was already raw counts and
// now everything else is too. What is left is close to one level across the
// recording -- depressed where sunlight or an electron beam takes signal away
// and raised where rammed ions add it -- but it steps between one range of 256
// frames and the next: each range was corrected with an estimate of its own,
// and one that has drifted by the end of its range leaves that whole block of
// frames sitting off its neighbours.
//
// Step 2 detrends every frame across its own strip. A straight line is fitted
// by least squares through its first two and last two pixels -- 0, 1, 63 and
// 64 -- and comes off the strip pixel by pixel in floating point, with the 2000
// DN offset put back on. Those four pixels then average 2000 again, so no frame
// sits at a level of its own and the block shifts go with the levels they were
// riding on; a tilt across the strip goes with them.
//
// Step 3 subtracts a background again, but a local one that cannot go stale.
// The first frame of an image is the 0 V dwell read back, so by construction it
// holds no ion signal and is a measurement of the background alone. For each
// image we take the first frames of the clean_win images centred on it (5 by
// default, the window clipped where it runs off either end of the recording),
// combine them pixel by pixel, and subtract that from every frame of the image
// -- then put the same 2000 DN back on, so a cleaned count reads on the scale
// the frames arrived on rather than as a difference around zero.
//
// The result has the same form as what the instrument sends, raw - background
// + 2000, and differs only in the background being one measured a few images
// away rather than one that can be 255 frames stale.
//
// Only frames fit to be a background sample go into that window: every byte of
// them arrived, their own CCITT CRC-16 checks out, and no JSON marker was
// spliced into them. An image whose first frame fails
// any of that contributes nothing, and the estimate is normalised by what is
// left -- four samples over four, not four over five.
//
// Median or mean, by the e key. The median is the default and measures better
// on both recordings. Scoring each frame by how far its own level sits from the
// 2000 DN it should land on, the median leaves 24 and 21 DN across ordinary
// images where the mean leaves 59 and 34; on the one image per range whose
// first frame IS the estimate, and whose window therefore straddles a change of
// level, the median leaves 55 and 31 DN against the mean's 128 and 96. That is
// the median returning the level the middle image is on where the mean returns
// a blend of the two. The mean is kept because it is the better estimator when
// every value in the window really is the same quantity, and because being able
// to put the two side by side is how the above was measured.

// How much of the cleaning is being applied: nothing, or the pipeline stopped
// after any of its steps. b turns cleaning on and off, and shift-B picks which
// step to stop after, which is only a question worth asking while it is on.
// Seeing a step on its own is how you tell which of them did what -- step 1
// alone is the whole recording at its raw level, with the block shifts the
// ranges arrive on still standing, and step 2 is those shifts taken out.
// Adding a step later means one more name here and one more block in
// rebuild_clean; shift-B picks it up on its own.
enum { CLEAN_OFF, CLEAN_STEP1, CLEAN_STEP2, CLEAN_FULL, CLEAN_STAGES };

// Frames after a candidate whose level says what the instrument has been taking
// off them, and how far above that level a real background estimate stands.
#define CLEAN_REF_FRAMES  32
#define CLEAN_REF_RATIO   2
#define CLEAN_WIN_MAX     9

// The fixed offset the instrument's own background subtraction lands on, in DN.
// Step 1 takes it off to recover the raw counts, and steps 2 and 3 put it back
// so a cleaned frame reads on the same scale a raw one does: the same DN window
// serves both modes, and nothing has to go negative to say "less signal than
// the background". Measured at 1930 to 1950 DN on both recordings.
#define CLEAN_PEDESTAL    2000.0

static int cmp_int(const void *a, const void *b)
{
    int x = *(const int *) a, y = *(const int *) b;
    return (x > y) - (x < y);
}

// Median of n values, sorting the array it is handed. An even count takes the
// upper of the two middle values rather than averaging them: averaging would
// blend the two levels a window straddling a boundary holds, which is the one
// thing the median is here to avoid.
static int median_int(int *v, int n)
{
    qsort(v, (size_t) n, sizeof *v, cmp_int);
    return v[n / 2];
}

// The background estimate from the n samples in the window: their median, or
// their mean over however many there are. Either way the normalisation is the
// count that survived the fitness test, not the width of the window.
static double estimate_bg(double *v, int n, int use_mean)
{
    if (use_mean) {
        double sum = 0.0;
        for (int i = 0; i < n; i++) sum += v[i];
        return sum / (double) n;
    }
    qsort(v, (size_t) n, sizeof *v, cmp_double);
    return v[n / 2];
}

// A frame's overall level. The median of its pixels rather than their mean,
// because the last pixel of a frame reads tens of thousands where the other 64
// read a couple of thousand, and one value in 65 does not move a median.
static int frame_level(const experiment_t *s, int k)
{
    int p[NPIX];
    long base = s->fr_off[k] + PIX_START;
    for (int j = 0; j < NPIX; j++) p[j] = be16(s->buf, base + j * 2);
    return median_int(p, NPIX);
}

// The CCITT CRC-16 the instrument puts in the last two bytes of every frame,
// over the 150 bytes before them (avr-libc's _crc_ccitt_update seeded 0xFFFF,
// most significant byte first -- calculateTelemetryCrc in the flight
// MPITelemetry.c). This is the instrument's own word on whether the frame
// arrived intact, and it covers exactly the bytes being read -- which the CSP
// CRC32 on a packet does not, a frame being assembled from whatever packets its
// bytes happened to land in. Measured, it passes on 3611 of 3611 whole frames
// of the 2026-07-21 recording and 9025 of 9027 of the 2026-08-09 one.
static int frame_crc_ok(const experiment_t *s, int k)
{
    const uint8_t *f = s->buf + s->fr_off[k];
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < FRAME_STRIDE - 2; i++) {
        uint8_t t = f[i] ^ (uint8_t) (crc & 0xFF);
        t ^= (uint8_t) (t << 4);
        crc = (uint16_t) ((crc >> 8) ^ ((uint16_t) t << 8)
                          ^ ((uint16_t) t << 3) ^ ((uint16_t) t >> 4));
    }
    return crc == (uint16_t) be16(s->buf, s->fr_off[k] + FRAME_STRIDE - 2);
}

// Whether frame k is fit to stand as a background sample. Every byte of it has
// to have arrived -- a byte that never came down reads as zero, and zero is not
// a measurement of no signal -- its own CRC has to check out, and it must not
// be one of the frames a JSON marker was spliced into. A frame that fails any
// of this is left out of the window rather than averaged in.
static int bg_sample_ok(const experiment_t *s, int k)
{
    if (s->fr_bad[k]) return 0;
    long b1 = s->fr_off[k] + FRAME_STRIDE;
    for (long b = s->fr_off[k]; b < b1; b++)
        if (!s->present[b]) return 0;
    return frame_crc_ok(s, k);
}

static void free_clean(view_t *v)
{
    free(v->clean_pix); v->clean_pix = NULL;
    free(v->clean_ok);  v->clean_ok  = NULL;
    free(v->key_fr);    v->key_fr    = NULL;
    v->n_key = 0; v->key_step = 0; v->n_pre_key = 0;
    v->clean_frames = 0; v->bg_samples = 0; v->bg_short = 0;
}

// Find the frames the instrument kept as background estimates. A candidate is a
// 0 V frame (column 0, the image's first) standing well above the level the
// frames after it sit at. That test alone also catches a frame that reads high
// for some other reason -- a corrupt body, a bright event -- so the cadence has
// the last word: the estimates run at a fixed interval, so take that interval
// to be the median spacing of the candidates and keep only the beat most of
// them share. Fills keys[] (room for nframes) and returns how many.
static int find_background_frames(view_t *v, const experiment_t *s,
                                  const int *lvl, int *keys)
{
    int nkey = 0;
    for (int k = 0; k < s->nframes; k++) {
        if (lvl[k] < 0 || col_of(v, s, k) != 0) continue;
        int ref[CLEAN_REF_FRAMES], nr = 0;
        for (int m = k + 1; m < s->nframes && nr < CLEAN_REF_FRAMES; m++)
            if (lvl[m] >= 0) ref[nr++] = lvl[m];
        if (nr == 0) continue;
        if (lvl[k] >= CLEAN_REF_RATIO * median_int(ref, nr)) keys[nkey++] = k;
    }
    if (nkey < 2) return nkey;

    int *d = (int *) malloc((size_t) (nkey - 1) * sizeof *d);
    if (d == NULL) return nkey;
    for (int i = 1; i < nkey; i++)
        d[i - 1] = (s->fr_ctr[keys[i]] - s->fr_ctr[keys[i - 1]]) & 0xFFFF;
    int step = median_int(d, nkey - 1);
    free(d);
    if (step < 2) return nkey;
    v->key_step = step;
    if (nkey < 3) return nkey;

    int phase = 0, most = 0;
    for (int i = 0; i < nkey; i++) {
        int ph = s->fr_ctr[keys[i]] % step, cnt = 0;
        for (int m = 0; m < nkey; m++)
            if (s->fr_ctr[keys[m]] % step == ph) cnt++;
        if (cnt > most) { most = cnt; phase = ph; }
    }
    int w = 0;
    for (int i = 0; i < nkey; i++)
        if (s->fr_ctr[keys[i]] % step == phase) keys[w++] = keys[i];
    return w;
}

// Build the cleaned pixels for the selected experiment. Returns 0 if there is
// nothing to show, in which case the buffers are left free.
static int rebuild_clean(view_t *v, const experiment_t *s)
{
    free_clean(v);
    if (s == NULL || s->nframes == 0 || v->fr_img == NULL || v->n_img == 0) return 0;

    int nf = s->nframes;
    float *pix = (float *) malloc((size_t) nf * NPIX * sizeof *pix);
    unsigned char *ok = (unsigned char *) calloc((size_t) nf, 1);
    int *lvl = (int *) malloc((size_t) nf * sizeof *lvl);
    int *keys = (int *) malloc((size_t) nf * sizeof *keys);
    if (pix == NULL || ok == NULL || lvl == NULL || keys == NULL) {
        free(pix); free(ok); free(lvl); free(keys);
        return 0;
    }
    for (int k = 0; k < nf; k++) lvl[k] = s->fr_bad[k] ? -1 : frame_level(s, k);
    int nkey = find_background_frames(v, s, lvl, keys);
    free(lvl);

    // Step 1: undo the instrument's subtraction. A frame belongs to the last
    // background frame at or before it in the file, so a range whose own
    // estimate never came down carries the one before it instead of going
    // uncorrected -- the background drifts only a few hundred DN across a
    // range, and step 3 takes off whatever is left of it either way. The frame
    // the instrument kept is already raw counts and is left alone; every other
    // frame gets its background back and the 2000 DN offset taken off.
    int cur = -1, nx = 0;
    for (int k = 0; k < nf; k++) {
        if (nx < nkey && keys[nx] == k) { cur = k; nx++; }
        if (s->fr_bad[k]) continue;
        long src = s->fr_off[k] + PIX_START;
        long bg = cur >= 0 ? s->fr_off[cur] + PIX_START : -1;
        for (int j = 0; j < NPIX; j++) {
            double p = be16(s->buf, src + j * 2);
            if (bg >= 0 && cur != k)
                p += (double) be16(s->buf, bg + j * 2) - CLEAN_PEDESTAL;
            pix[(long) k * NPIX + j] = (float) p;
        }
        ok[k] = 1;
        if (cur < 0) {
            v->n_pre_key++;
            // Ahead of the first estimate, so there is nothing to add back and
            // the frame is still on the instrument's own scale rather than at
            // raw counts. Step 2 puts it on the same footing as everything
            // else, a fit through a frame's own edges caring nothing for the
            // scale it came in on, but step 1 alone has no answer for it, and
            // showing it beside corrected frames would read as a tenfold
            // depression that never happened.
            if (v->clean_stage == CLEAN_STEP1) ok[k] = 0;
        }
    }

    // Step 2: detrend every frame across its own strip. A straight line is
    // fitted by least squares through the first two and last two pixels -- 0,
    // 1, 63 and 64 -- and that line, evaluated at each pixel in turn, comes off
    // the whole strip in floating point, with the 2000 DN offset put back on.
    // A least-squares fit with an intercept leaves its four residuals summing
    // to zero, so those four pixels average exactly 2000 again, and a frame
    // arriving with a tilt across the strip as well as a level of its own loses
    // both rather than only the level.
    // This is what takes the 256-frame block shifts out: a range whose estimate
    // has gone stale by the end of it, and the frames ahead of the first
    // estimate that never had one, come in a few hundred DN off the ranges
    // either side of them, which the whereogram shows as blocks that begin and
    // end where the estimates do.
    if (v->clean_stage >= CLEAN_STEP2) {
        const int ex[4] = { 0, 1, NPIX - 2, NPIX - 1 };
        for (int k = 0; k < nf; k++) {
            if (!ok[k]) continue;
            float *p = pix + (long) k * NPIX;
            double sx = 0.0, sy = 0.0;
            for (int i = 0; i < 4; i++) { sx += ex[i]; sy += (double) p[ex[i]]; }
            double mx = sx / 4.0, my = sy / 4.0;
            double num = 0.0, den = 0.0;
            for (int i = 0; i < 4; i++) {
                double dx = (double) ex[i] - mx;
                num += dx * ((double) p[ex[i]] - my);
                den += dx * dx;
            }
            double slope = num / den;
            for (int j = 0; j < NPIX; j++) {
                double fit = my + slope * ((double) j - mx);
                p[j] = (float) ((double) p[j] - fit + CLEAN_PEDESTAL);
            }
        }
    }

    // Stop here if that is all the stage asks for: step 1 on its own is the
    // recording at its raw level, which is what says whether the instrument's
    // own subtraction was all that stood between it and being flat, and step 2
    // on top of it says whether the block shifts have gone with it.
    if (v->clean_stage < CLEAN_FULL) {
        for (int k = 0; k < nf; k++) if (ok[k]) v->clean_frames++;
        v->clean_pix = pix; v->clean_ok = ok;
        v->key_fr = keys; v->n_key = nkey;
        if (v->clean_frames == 0) { free_clean(v); return 0; }
        return 1;
    }

    // The first frame of each image, where it is fit to be a background sample:
    // the 0 V dwell read back, which is the background measured with no ion
    // signal on it.
    int *first = (int *) malloc((size_t) v->n_img * sizeof *first);
    double *bgest = (double *) malloc((size_t) v->n_img * NPIX * sizeof *bgest);
    unsigned char *have = (unsigned char *) calloc((size_t) v->n_img, 1);
    if (first == NULL || bgest == NULL || have == NULL) {
        free(first); free(bgest); free(have); free(pix); free(ok); free(keys);
        return 0;
    }
    for (int i = 0; i < v->n_img; i++) first[i] = -1;
    for (int k = 0; k < nf; k++) {
        int im = v->fr_img[k];
        if (im < 0 || im >= v->n_img || !ok[k] || first[im] >= 0) continue;
        if (col_of(v, s, k) == 0 && bg_sample_ok(s, k)) first[im] = k;
    }
    for (int i = 0; i < v->n_img; i++) if (first[i] >= 0) v->bg_samples++;

    // Step 3: the sliding background, one estimate per image.
    int h = (v->clean_win - 1) / 2;
    for (int i = 0; i < v->n_img; i++) {
        int a = i - h < 0 ? 0 : i - h;
        int b = i + h > v->n_img - 1 ? v->n_img - 1 : i + h;
        int src[CLEAN_WIN_MAX], ns = 0, width = 0;
        for (int m = a; m <= b && ns < CLEAN_WIN_MAX; m++) {
            width++;
            if (first[m] >= 0) src[ns++] = first[m];
        }
        have[i] = ns > 0;
        if (ns < width) v->bg_short++;
        double win[CLEAN_WIN_MAX];
        for (int j = 0; j < NPIX; j++) {
            for (int q = 0; q < ns; q++) win[q] = pix[(long) src[q] * NPIX + j];
            bgest[(long) i * NPIX + j] = ns > 0 ? estimate_bg(win, ns, v->clean_mean) : 0.0;
        }
    }
    free(first);

    // Take it off, and put the instrument's own offset back on. A frame the
    // image list dropped is cleaned too, on the background of the image it sits
    // in: its aux was corrupt, which says nothing about its pixels, and leaving
    // it out would open a hole in the whereogram that no gap in the data
    // accounts for.
    int last = -1;
    for (int k = 0; k < nf; k++) {
        if (!ok[k]) continue;
        int im = v->fr_img[k];
        if (im >= 0 && im < v->n_img) last = im;
        int use = im;
        if (use < 0 || use >= v->n_img) use = last < 0 ? 0 : last;
        if (!have[use]) { ok[k] = 0; continue; }
        for (int j = 0; j < NPIX; j++) {
            double p = (double) pix[(long) k * NPIX + j]
                     - bgest[(long) use * NPIX + j] + CLEAN_PEDESTAL;
            pix[(long) k * NPIX + j] = (float) p;
        }
        v->clean_frames++;
    }
    free(bgest); free(have);

    v->clean_pix = pix; v->clean_ok = ok;
    v->key_fr = keys; v->n_key = nkey;
    if (v->clean_frames == 0) { free_clean(v); return 0; }
    return 1;
}

// What the viewer shows for pixel j of frame k: the counts as they came down,
// or the cleaned value when cleaning is on. Only call it for a frame
// frame_shown() has passed -- that is what says there is a cleaned value here.
static double pix_val(const view_t *v, const experiment_t *s, int k, int j)
{
    if (v->clean_stage != CLEAN_OFF) return v->clean_pix[(long) k * NPIX + j];
    return be16(s->buf, s->fr_off[k] + PIX_START + j * 2);
}

// Whether frame k has anything to show in the current mode. Cleaning needs a
// background for the image the frame sits in; without one there is no cleaned
// frame, and drawing the raw one beside cleaned neighbours would read as a
// measurement of something it is not.
static int frame_shown(const view_t *v, const experiment_t *s, int k)
{
    (void) s;
    if (v->clean_stage == CLEAN_OFF) return 1;
    return v->clean_pix != NULL && v->clean_ok[k];
}

// Per-experiment DN extent over all present pixels (for auto-session scaling).
static void compute_session_extent(view_t *v, const experiment_t *s)
{
    double mn = 0, mx = 0;
    int any = 0;
    for (int k = 0; k < s->nframes; k++) {
        if (!frame_shown(v, s, k)) continue;
        for (int j = 0; j < NPIX; j++) {
            double p = pix_val(v, s, k, j);
            if (!any || p < mn) mn = p;
            if (!any || p > mx) mx = p;
            any = 1;
        }
    }
    if (!any) { mn = 0; mx = 1; }
    v->sess_min = (int) floor(mn);
    v->sess_max = (int) ceil(mx);
    if (v->sess_max <= v->sess_min) v->sess_max = v->sess_min + 1;
}

// Move to a cleaning stage and rebuild what it needs. The manual DN window
// carries across every stage that ends on the pedestal, since a cleaned count
// is put back on the level a raw one arrives at (see CLEAN_PEDESTAL); step 1 on
// its own sits at the raw counts, about ten times higher, so the auto scales are
// the ones to be in there. Returns 0 if the experiment has nothing to clean,
// having dropped back to showing it as it came down.
static int set_clean(view_t *v, const experiment_t *s, int stage)
{
    v->clean_stage = stage;
    if (stage != CLEAN_OFF && !rebuild_clean(v, s)) {
        v->clean_stage = CLEAN_OFF;
        free_clean(v);
        compute_session_extent(v, s);
        return 0;
    }
    if (stage == CLEAN_OFF) free_clean(v);
    compute_session_extent(v, s);
    return 1;
}

// Switch to a different experiment and refresh everything derived from it.
// Cleaning is rebuilt against the new experiment, and dropped if that one has
// no background estimates to work from.
static void select_experiment(view_t *v, const experiment_t *s)
{
    v->img_pos = 0;
    v->playing = 0;
    apply_scan(v, s);
    rebuild_image_list(v, s);
    if (v->clean_stage == CLEAN_OFF) compute_session_extent(v, s);
    else set_clean(v, s, v->clean_stage);
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

// ---- colour maps ------------------------------------------------------------
// An intensity has no colour of its own, so the choice is about what the eye
// picks out. Grey is the honest one; ion is the blue-to-orange these spectra
// are usually shown in; viridis and inferno are the perceptually even maps,
// which bring up faint structure grey flattens. Each is a handful of anchor
// colours with a straight ramp between them.

#define CMAP_STOPS 5

typedef struct {
    const char   *name;
    unsigned char rgb[CMAP_STOPS][3];
} cmap_t;

static const cmap_t g_cmaps[] = {
    { "grey",    { {   0,  0,  0 }, {  64, 64, 64 }, { 128,128,128 }, { 192,192,192 }, { 255,255,255 } } },
    { "ion",     { {  45, 75,200 }, { 110,140,225 }, { 215,225,240 }, { 235,160, 70 }, { 255,228,185 } } },
    { "viridis", { {  68,  1, 84 }, {  59, 82,139 }, {  33,145,140 }, {  94,201, 98 }, { 253,231, 37 } } },
    { "inferno", { {   0,  0,  4 }, {  87, 16,110 }, { 188, 55, 84 }, { 249,142,  9 }, { 252,255,164 } } },
};
#define N_CMAPS ((int) (sizeof g_cmaps / sizeof g_cmaps[0]))

static Color cmap_color(int map, unsigned char val)
{
    const cmap_t *c = &g_cmaps[((map % N_CMAPS) + N_CMAPS) % N_CMAPS];
    double t = (double) val / 255.0 * (CMAP_STOPS - 1);
    int i = (int) t;
    if (i > CMAP_STOPS - 2) i = CMAP_STOPS - 2;
    double f = t - (double) i;
    Color out = { 0, 0, 0, 255 };
    out.r = (unsigned char) (c->rgb[i][0] + f * ((int) c->rgb[i + 1][0] - (int) c->rgb[i][0]));
    out.g = (unsigned char) (c->rgb[i][1] + f * ((int) c->rgb[i + 1][1] - (int) c->rgb[i][1]));
    out.b = (unsigned char) (c->rgb[i][2] + f * ((int) c->rgb[i + 1][2] - (int) c->rgb[i][2]));
    return out;
}

// The window's own background, which is what a column with no data is painted
// in so that nothing reads as a measurement that was never made.
#define MPI_BG  ((Color){ 18, 18, 22, 255 })

// ---- whereogram -------------------------------------------------------------
//
// A column no frame landed in. Columns are fractions once cleaning is on, and a
// cleaned one can land below zero where a raw one never does -- a deep enough
// depression, or a corrupt frame -- so the sentinel has to be a value no
// reading can take.
#define WG_NONE  (-DBL_MAX)
//
// The whole recording at a glance: every frame a column of its 65 pixels, time
// running left to right, arrival direction up the vertical axis. It is the
// spectrum the MPI actually measured, so it doubles as the map of the file --
// which is why it replaced the coverage patch rather than sitting beside it.
//
// The horizontal axis is the file, not the frames we happen to hold: a column
// covers a fixed run of frame-sized slices, so a stretch that never came down
// takes up its own width instead of being closed up. That is what makes a gap
// visible at all. Those columns are painted in the background colour -- a
// colour map has no value that means "no data", and the darkest end of one
// would read as a real, low measurement -- and are marked instead by the red
// rule drawn above the panel.
//
// Several frames share a column once a recording is longer than the panel is
// wide, and they are averaged. Taking the brightest instead lets the one
// brightest frame in each column stand for all of them, which at ten frames a
// column paints a picture of the outliers rather than of the data.
//
// The colour scale works the same way as the image's, but on the whereogram's
// own terms: auto here means over the whole whereogram, since scaling a picture
// of the whole recording to one image of it would make it flicker while
// scrubbing and would leave one column not comparable with the next. Pass
// lo >= hi to scale automatically; *out_lo / *out_hi report the range used.
static void build_whereogram(const view_t *v, const experiment_t *s,
                             int cmap, int lo, int hi,
                             Color *pix, int w, unsigned char *missing,
                             int *out_lo, int *out_hi)
{
    long nslot = s->size / FRAME_STRIDE;
    if (nslot < 1) nslot = 1;

    // Which frame, if any, begins in each frame-sized slice of the file.
    int *at = (int *) malloc((size_t) nslot * sizeof *at);
    double *val = (double *) malloc((size_t) w * NPIX * sizeof *val);
    if (at == NULL || val == NULL) { free(at); free(val); return; }
    for (long q = 0; q < nslot; q++) at[q] = -1;
    for (int k = 0; k < s->nframes; k++) {
        long q = s->fr_off[k] / FRAME_STRIDE;
        if (q >= 0 && q < nslot) at[q] = k;
    }

    // First pass: what each column reads, averaged over the frames in it.
    for (int c = 0; c < w; c++) {
        long q0 = (long) c * nslot / w;
        long q1 = (long) (c + 1) * nslot / w;
        if (q1 <= q0) q1 = q0 + 1;
        if (q1 > nslot) q1 = nslot;

        double sum[NPIX] = {0};
        int nsum = 0, gap = 0;

        for (long q = q0; q < q1; q++) {
            long b0 = q * FRAME_STRIDE, b1 = b0 + FRAME_STRIDE;
            if (b1 > s->size) b1 = s->size;
            for (long b = b0; b < b1; b++)
                if (!s->present[b]) { gap = 1; break; }

            int k = at[q];
            if (k < 0 || s->fr_bad[k] || !frame_shown(v, s, k)) continue;
            for (int j = 0; j < NPIX; j++) sum[j] += pix_val(v, s, k, j);
            nsum++;
        }

        missing[c] = (unsigned char) gap;
        for (int j = 0; j < NPIX; j++)
            val[(long) j * w + c] = nsum > 0 ? sum[j] / nsum : WG_NONE;
    }
    free(at);

    // Auto scaling is over the whole whereogram -- but over what it holds, not
    // over its two most extreme samples. Taking the plain minimum and maximum
    // gave 0 .. 49296 and painted a flat blue rectangle, because a few corrupt
    // readings reach the ends of the 16-bit range. The 1st and 95th percentile
    // step past those and leave the range the readings actually occupy. They
    // are read off the sorted values rather
    // than a histogram of them, because a cleaned column is a fraction and can
    // land outside the 16-bit range a histogram would have to cover.
    if (lo >= hi) {
        double *sorted = (double *) malloc((size_t) w * NPIX * sizeof *sorted);
        long n = 0;
        if (sorted != NULL)
            for (long i = 0; i < (long) w * NPIX; i++)
                if (val[i] != WG_NONE) sorted[n++] = val[i];
        double mn = 0, mx = 1;
        if (n > 0) {
            qsort(sorted, (size_t) n, sizeof *sorted, cmp_double);
            mn = sorted[(n - 1) / 100];
            mx = sorted[(n - 1) * 95 / 100];
        }
        free(sorted);
        lo = (int) floor(mn);
        hi = (int) ceil(mx);
    }
    if (hi <= lo) hi = lo + 1;

    // Second pass: paint it.
    for (long i = 0; i < (long) w * NPIX; i++) {
        double p = val[i];
        if (p == WG_NONE) { pix[i] = MPI_BG; continue; }
        int g = p <= lo ? 0 : p >= hi ? 255 : (int) (255.0 * (p - lo) / (hi - lo));
        pix[i] = cmap_color(cmap, (unsigned char) g);
    }
    free(val);
    *out_lo = lo; *out_hi = hi;
}

// Where a file offset falls along the whereogram's horizontal axis, and the
// offset a column stands for -- the two directions of the same mapping, used to
// place the playback head and to turn a pointer position back into an image.
//
// The head is placed in panel pixels rather than in whole columns. An image is
// only a few pixels wide on a panel holding a whole recording, so rounding its
// byte span out to the columns that contain it drew a box with a margin of
// recording inside it that the image does not hold. The right edge is not
// pulled back inside the panel either, so passing the byte one past the span
// gives the box's exclusive right edge.
static int wg_x_of_byte(const experiment_t *s, int w, long byte)
{
    long nslot = s->size / FRAME_STRIDE;
    if (nslot < 1) nslot = 1;
    long q = byte / FRAME_STRIDE;
    if (q < 0) q = 0;
    if (q > nslot) q = nslot;
    return (int) (q * w / nslot);
}

static long wg_byte_of_col(const experiment_t *s, int w, int col)
{
    long nslot = s->size / FRAME_STRIDE;
    if (nslot < 1) nslot = 1;
    if (col < 0) col = 0;
    if (col > w - 1) col = w - 1;
    return ((long) col * nslot / w) * FRAME_STRIDE;
}

// The capture time of the frame nearest a byte of the recording, in unix ms,
// or -1 if no frame carries one. Frames whose time never came down are passed
// over rather than answered with: the ruler is reading the recording's own
// clock, and a frame without one has nothing to say about where it sat in
// time. The frames run up the file in order, so the first one past the byte is
// the last that can be nearer than what is already in hand.
static double time_at_byte(const experiment_t *s, long byte)
{
    double best_t = -1.0;
    long best_d = 0;
    for (int k = 0; k < s->nframes; k++) {
        if (s->fr_ts[k] < 0) continue;
        long o = s->fr_off[k];
        long d = byte < o ? o - byte : byte - o;
        if (best_t < 0 || d < best_d) { best_t = s->fr_ts[k]; best_d = d; }
        if (o > byte) break;
    }
    return best_t;
}

// The byte range the currently shown image was reconstructed from, so the
// whereogram can mark where in the recording you are looking. Returns 0 if the
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
// the whereogram. Returns -1 if there are no images.
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

// ---- session state ---------------------------------------------------------
//
// Where the viewer was left: which experiment was open, where the playhead sat
// in it, every view setting, and the size and place of the window. Written
// when the window closes and read back at startup, so opening the viewer again
// carries on from the last look rather than starting at the first experiment
// with the defaults.
//
// Plain "key = value" text under ~/.local/state/simple_sat_ops, beside the
// active TLE and the uplink key simple_sat_ops keeps there, so it reads and
// edits by hand. A key this build does not know is ignored and a key the file
// does not carry keeps its built-in default, which is what lets a file written
// by an older or a newer build still be worth reading. Every value is checked
// on the way in: it is a file on disk, so a stale or hand-edited one must
// still leave the viewer somewhere it could have got to on its own.

#define STATE_RELPATH  ".local/state/simple_sat_ops/mpi_viewer.state"
#define WIN_W_DEFAULT  1280
#define WIN_H_DEFAULT  800

// The window's own geometry, kept out of view_t because it belongs to the
// window rather than to what is being looked at. has_pos says the file carried
// a place to put it; without one the window opens wherever the platform likes.
typedef struct {
    int has_pos;
    int x, y, w, h;
} geom_t;

static int state_path(char *out, size_t cap)
{
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') return -1;
    if (snprintf(out, cap, "%s/%s", home, STATE_RELPATH) >= (int) cap) return -1;
    return 0;
}

// Whitespace off both ends of a line, in place, the newline with it.
static char *trim(char *s)
{
    while (*s != '\0' && isspace((unsigned char) *s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char) e[-1])) e--;
    *e = '\0';
    return s;
}

static int clamp_int(int x, int lo, int hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}

// Read the saved state into v, sel_utc (the experiment that was open) and g.
// Anything the file does not carry is left as the caller set it.
static void load_state(view_t *v, char *sel_utc, size_t utc_cap, geom_t *g)
{
    char path[1024];
    if (state_path(path, sizeof path) != 0) return;
    FILE *f = fopen(path, "r");
    if (f == NULL) return;
    char line[512];
    while (fgets(line, sizeof line, f) != NULL) {
        char *eq = strchr(line, '=');
        if (line[0] == '#' || eq == NULL) continue;
        *eq = '\0';
        char *key = trim(line), *val = trim(eq + 1);
        int n = atoi(val);
        if (strcmp(key, "experiment") == 0)
            snprintf(sel_utc, utc_cap, "%s", val);
        else if (strcmp(key, "image") == 0)
            v->img_pos = n < 0 ? 0 : n;
        else if (strcmp(key, "playing") == 0)
            v->playing = n != 0;
        else if (strcmp(key, "images_per_second") == 0) {
            double r = atof(val);
            v->ips = (float) (r < 1.0 ? 1.0 : r > 60.0 ? 60.0 : r);
        } else if (strcmp(key, "frames_per_sweep") == 0)
            v->fpi_override = (n == 8 || n == 16) ? n : 0;
        else if (strcmp(key, "zoom") == 0)
            v->zoom = (n == 2 || n == 4 || n == 8 || n == 16) ? n : 8;
        else if (strcmp(key, "colour_map") == 0)
            v->cmap = clamp_int(n, 0, N_CMAPS - 1);
        else if (strcmp(key, "scale_mode") == 0)
            v->scale_mode = clamp_int(n, SCALE_AUTO_IMAGE, SCALE_MANUAL);
        else if (strcmp(key, "dn_min") == 0)
            v->dn_min = clamp_int(n, 0, 65535);
        else if (strcmp(key, "dn_max") == 0)
            v->dn_max = clamp_int(n, 0, 65535);
        else if (strcmp(key, "clean_stage") == 0)
            v->clean_stage = clamp_int(n, CLEAN_OFF, CLEAN_STAGES - 1);
        else if (strcmp(key, "clean_step") == 0)
            v->clean_step = clamp_int(n, CLEAN_STEP1, CLEAN_STAGES - 1);
        else if (strcmp(key, "clean_window") == 0)
            // Odd widths only: the window is the images either side of this one.
            v->clean_win = clamp_int(n, 1, CLEAN_WIN_MAX) | 1;
        else if (strcmp(key, "clean_mean") == 0)
            v->clean_mean = n != 0;
        else if (strcmp(key, "window_w") == 0)
            g->w = (n >= 640 && n <= 8192) ? n : WIN_W_DEFAULT;
        else if (strcmp(key, "window_h") == 0)
            g->h = (n >= 400 && n <= 8192) ? n : WIN_H_DEFAULT;
        else if (strcmp(key, "window_x") == 0) { g->x = n; g->has_pos = 1; }
        else if (strcmp(key, "window_y") == 0) g->y = n;
    }
    fclose(f);
    if (v->dn_max <= v->dn_min) v->dn_max = v->dn_min + 1;
}

// Write the state back. Best effort: a session that cannot write its state was
// still a good session, so nothing here is reported.
static void save_state(const view_t *v, const experiment_t *s, const geom_t *g)
{
    char path[1024];
    if (state_path(path, sizeof path) != 0) return;
    if (sso_mkdir_p_for_file(path) != 0) return;
    FILE *f = fopen(path, "w");
    if (f == NULL) return;
    fprintf(f, "# mpi_viewer session state -- written on exit, read at startup.\n");
    fprintf(f, "# Delete this file to come back up on the defaults.\n");
    if (s->sat_path[0] != '\0') fprintf(f, "# file = %s\n", s->sat_path);
    fprintf(f, "experiment = %s\n", s->utc);
    fprintf(f, "image = %d\n", v->img_pos);
    fprintf(f, "playing = %d\n", v->playing);
    fprintf(f, "images_per_second = %g\n", (double) v->ips);
    fprintf(f, "frames_per_sweep = %d\n", v->fpi_override);
    fprintf(f, "zoom = %d\n", v->zoom);
    fprintf(f, "colour_map = %d\n", v->cmap);
    fprintf(f, "scale_mode = %d\n", v->scale_mode);
    fprintf(f, "dn_min = %d\n", v->dn_min);
    fprintf(f, "dn_max = %d\n", v->dn_max);
    fprintf(f, "clean_stage = %d\n", v->clean_stage);
    fprintf(f, "clean_step = %d\n", v->clean_step);
    fprintf(f, "clean_window = %d\n", v->clean_win);
    fprintf(f, "clean_mean = %d\n", v->clean_mean);
    fprintf(f, "window_w = %d\n", g->w);
    fprintf(f, "window_h = %d\n", g->h);
    fprintf(f, "window_x = %d\n", g->x);
    fprintf(f, "window_y = %d\n", g->y);
    fclose(f);
}

// The experiment the last session had open, found by its start time. Returns
// its index, or -1 if this list does not hold it -- a different database, or
// one whose experiments have since been split differently.
static int find_experiment(const experiment_t *exps, int nexp, const char *utc)
{
    if (utc[0] == '\0') return -1;
    for (int k = 0; k < nexp; k++)
        if (strcmp(exps[k].utc, utc) == 0) return k;
    return -1;
}

// Whether a saved window position still lands on a display. Positions are in
// the desktop's own coordinates, which span every monitor, so the test is
// against each monitor's own rectangle rather than against a size: a place on
// a screen that has since been unplugged would open the window where nobody
// can see it.
static int position_on_a_monitor(int x, int y)
{
    for (int m = 0; m < GetMonitorCount(); m++) {
        Vector2 o = GetMonitorPosition(m);
        if (x >= (int) o.x && y >= (int) o.y
            && x < (int) o.x + GetMonitorWidth(m)
            && y < (int) o.y + GetMonitorHeight(m))
            return 1;
    }
    return 0;
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
                   "Press b to show the cleaned imagery -- the instrument's own\n"
                   "background subtraction undone, every frame detrended across its\n"
                   "own edge pixels, and a sliding local background put in place of\n"
                   "the instrument's -- and shift-B, while it is on, to stop after an\n"
                   "earlier one of those steps. n sets how many images the sliding\n"
                   "one is taken over, and e whether they are combined by median or\n"
                   "mean.\n"
                   "Press d to write the selected experiment's missing data as\n"
                   "re-download telecommands, for simple_sat_ops --tc-file.\n"
                   "--list prints what each experiment reconstructed to and exits,\n"
                   "without opening a window.\n"
                   "Which experiment was open, where the playhead sat and every\n"
                   "view setting are kept in ~/.local/state/simple_sat_ops/\n"
                   "mpi_viewer.state and picked up next time; delete that file to\n"
                   "come back up on the defaults.\n");
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
    v.clean_win = 5;
    v.clean_step = CLEAN_FULL;

    // What the last session left, over those defaults. The view settings have
    // to be in place before the experiment is selected, since the scan period
    // and the cleaning stage are what select_experiment builds from; the
    // playhead has to wait until after it, because selecting an experiment
    // rewinds it to the first image.
    geom_t geom = { .w = WIN_W_DEFAULT, .h = WIN_H_DEFAULT };
    char sel_utc[24] = "";
    load_state(&v, sel_utc, sizeof sel_utc, &geom);
    int saved_img = v.img_pos, saved_play = v.playing;
    int resumed = find_experiment(exps, nexp, sel_utc);
    if (resumed >= 0) v.sel = resumed;
    select_experiment(&v, &exps[v.sel]);
    if (resumed >= 0) {
        v.img_pos = saved_img >= v.n_img ? (v.n_img ? v.n_img - 1 : 0) : saved_img;
        v.playing = saved_play;
        fprintf(stderr, "mpi_viewer: resuming %s UTC at image %d of %d.\n",
                exps[v.sel].utc, v.img_pos + 1, v.n_img);
    }

    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(geom.w, geom.h, "mpi_viewer");
    if (geom.has_pos && position_on_a_monitor(geom.x, geom.y))
        SetWindowPosition(geom.x, geom.y);
    SetTargetFPS(60);
    g_ui_font_loaded = load_ui_font();

    // A fixed texture holding the image rotated 90 CW: the scan rows become
    // columns (width MAX_FPI) and the 65 pixels become rows (height NPIX). We
    // fill the top-left fpi x NPIX region and let raylib scale it with
    // nearest-neighbour (POINT) filtering. Full colour rather than grey so the
    // colour map applies here as well as to the whereogram.
    Color imgbuf[MAX_FPI * NPIX] = {0};
    Image gimg = { .data = imgbuf, .width = MAX_FPI, .height = NPIX,
                   .mipmaps = 1, .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
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

    // The whereogram: one texture column per run of frames, NPIX rows, rebuilt
    // when the experiment, the colour map, the scale or the panel width change.
    Color    *cov_pix = NULL;
    Texture2D cov_tex = {0};
    int cov_w = 0, cov_valid = 0;
    unsigned cov_sig = 0;
    unsigned char *wg_missing = NULL;   // per column: some of its bytes never came down
    int wg_lo = -1, wg_hi = -1;         // the DN window its colours were built for
    // Set while the pointer is scrubbing the whereogram, so a press that
    // began there keeps control until the button is let go -- and a press that
    // began anywhere else never takes it. cov_anchor is the byte the press
    // landed on, which the time ruler measures from.
    int cov_drag = 0;
    long cov_anchor = 0;

    while (!WindowShouldClose()) {
        experiment_t *s = &exps[v.sel];

        // ---- input ----
        // Letting the button go ends a whereogram scrub. Checked here rather
        // than beside the panel itself, which is not drawn at all in a window
        // too small for it -- a scrub must not survive that.
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
            // Cleaning is built on the images and on which frame of each is the
            // 0 V one, so a different scan period rebuilds it.
            if (v.clean_stage != CLEAN_OFF) set_clean(&v, s, v.clean_stage);
            cov_valid = 0;
        }
        // b turns cleaning on and off, coming back on at whichever step it was
        // left showing. shift-B moves between the steps -- the instrument's own
        // subtraction undone, then every frame detrended across its own edges,
        // then the sliding local background taken off as well -- and is ignored
        // with cleaning off, there being no step to pick then.
        if (IsKeyPressed(KEY_B)) {
            int shifted = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            int want = -1;
            if (!shifted) {
                want = v.clean_stage == CLEAN_OFF ? v.clean_step : CLEAN_OFF;
            } else if (v.clean_stage != CLEAN_OFF) {
                v.clean_step = v.clean_step + 1 >= CLEAN_STAGES ? CLEAN_STEP1
                                                               : v.clean_step + 1;
                want = v.clean_step;
            }
            if (want >= 0) {
                if (!set_clean(&v, s, want)) {
                    snprintf(status, sizeof status,
                             "nothing to clean: no background frames in this experiment");
                    status_left = 6.0f;
                }
                cov_valid = 0;
            }
        }
        // n: how many images the sliding background is taken over.
        if (IsKeyPressed(KEY_N)) {
            v.clean_win = v.clean_win >= CLEAN_WIN_MAX ? 1 : v.clean_win + 2;
            if (v.clean_stage != CLEAN_OFF) set_clean(&v, s, v.clean_stage);
            cov_valid = 0;
        }
        // e: how those images are combined into the estimate.
        if (IsKeyPressed(KEY_E)) {
            v.clean_mean = !v.clean_mean;
            if (v.clean_stage != CLEAN_OFF) set_clean(&v, s, v.clean_stage);
            cov_valid = 0;
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
                cov_valid = 0;   // the reloaded experiment needs a fresh whereogram
            } else if (ne != NULL) {
                free_experiments(ne, nn);
            }
        }
        if (IsKeyPressed(KEY_M)) v.cmap = (v.cmap + 1) % N_CMAPS;
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
            double mn = 0, mx = 0;
            int any = 0;
            for (int k = 0; k < s->nframes; k++) {
                if (v.fr_img == NULL || v.fr_img[k] != v.img_pos) continue;
                if (!frame_shown(&v, s, k)) continue;
                for (int j = 0; j < NPIX; j++) {
                    double p = pix_val(&v, s, k, j);
                    if (!any || p < mn) mn = p;
                    if (!any || p > mx) mx = p;
                    any = 1;
                }
            }
            if (!any) { mn = 0; mx = 1; }
            lo = (int) floor(mn); hi = (int) ceil(mx);
            if (hi <= lo) hi = lo + 1;
        }

        // ---- rasterise the image into the grayscale texture ----
        for (int q = 0; q < MAX_FPI * NPIX; q++) imgbuf[q] = MPI_BG;
        double frame_time = -1.0;
        int rep_frame = -1, cols_present = 0;
        for (int k = 0; k < s->nframes; k++) {
            if (v.fr_img == NULL || v.fr_img[k] != v.img_pos) continue;
            int col = col_of(&v, s, k);
            if (col < 0 || col >= v.ncols) continue;
            if (rep_frame < 0) rep_frame = k;
            if (s->fr_ts[k] >= 0 && (frame_time < 0 || s->fr_ts[k] < frame_time)) frame_time = s->fr_ts[k];
            if (!frame_shown(&v, s, k)) continue;
            cols_present++;
            for (int j = 0; j < NPIX; j++) {
                double p = pix_val(&v, s, k, j);
                int g = p <= lo ? 0 : p >= hi ? 255 : (int) (255.0 * (p - lo) / (hi - lo));
                // Column = bias voltage (background left, most negative right); pixel -> row.
                imgbuf[j * MAX_FPI + col] = cmap_color(v.cmap, (unsigned char) g);
            }
        }
        UpdateTexture(tex, imgbuf);

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

        draw_text(TextFormat("colour map    : %s  (%d/%d)",
                             g_cmaps[v.cmap].name, v.cmap + 1, N_CMAPS),
                  ax, ay, 15, LIGHTGRAY); ay += 20;

        // Cleaning: what is being shown, and what it was built from. The
        // background-estimate count and their spacing are the check that the
        // right frames were picked out -- the instrument runs them every 256.
        if (v.clean_stage == CLEAN_OFF) {
            draw_text("cleaning      : off  (counts as they came down)", ax, ay, 15, GRAY);
            ay += 20;
        } else {
            if (v.clean_stage == CLEAN_STEP1)
                draw_text("cleaning      : step 1  (the instrument's own subtraction undone)",
                          ax, ay, 15, (Color){ 120, 220, 160, 255 });
            else if (v.clean_stage == CLEAN_STEP2)
                draw_text("cleaning      : steps 1+2  (frames detrended across their own edges)",
                          ax, ay, 15, (Color){ 120, 220, 160, 255 });
            else
                draw_text(TextFormat("cleaning      : steps 1+2+3  (%d-image %s)",
                                     v.clean_win, v.clean_mean ? "mean" : "median"),
                          ax, ay, 15, (Color){ 120, 220, 160, 255 });
            ay += 20;
            if (v.clean_stage == CLEAN_FULL) {
                draw_text(TextFormat("bg samples    : %d of %d images%s",
                                     v.bg_samples, v.n_img,
                                     v.bg_short ? TextFormat(", %d windows short", v.bg_short) : ""),
                          ax, ay, 15, LIGHTGRAY); ay += 20;
            }
            if (v.key_step > 0)
                draw_text(TextFormat("bg estimates  : %d frames, every %d",
                                     v.n_key, v.key_step),
                          ax, ay, 15, LIGHTGRAY);
            else
                draw_text(TextFormat("bg estimates  : %d frames", v.n_key),
                          ax, ay, 15, LIGHTGRAY);
            ay += 20;
            if (v.n_pre_key > 0) {
                draw_text(TextFormat("                %d frames ahead of the first",
                                     v.n_pre_key),
                          ax, ay, 15, (Color){ 206, 150, 42, 255 }); ay += 20;
            }
        }

        // The whereogram, under the aux panel: the whole recording as a
        // spectrum, one column per frame, arrival direction up the vertical
        // axis. Five times as wide as it is high, because the interesting axis
        // is time and 65 pixels is all the other one ever has.
        ay += 8;
        draw_text("Whereogram  (arrival direction against time)", ax, ay, 18, RAYWHITE);
        if (wg_hi > wg_lo) {
            const char *wr = TextFormat("DN %d .. %d", wg_lo, wg_hi);
            draw_text(wr, GetScreenWidth() - 20 - text_width(wr, 13), ay + 5, 13, GRAY);
        }
        ay += 24;
        int cw = GetScreenWidth() - ax - 20;
        int avail_h = GetScreenHeight() - 74 - ay;
        if (cw > avail_h * 5) cw = avail_h * 5;
        int ch = cw / 5;
        if (cw >= 80 && ch >= 20) {
            // Same scale control as the image: a manual window is taken as
            // given, and either auto mode scales the whereogram over the whole
            // whereogram. Scaling it to the shown image instead would make it
            // flicker while scrubbing and stop one column being comparable
            // with the next.
            int wlo = v.scale_mode == SCALE_MANUAL ? v.dn_min : 0;
            int whi = v.scale_mode == SCALE_MANUAL ? v.dn_max : 0;
            // What the built pixels depend on: any change here rebuilds them.
            unsigned sig = (unsigned) v.sel;
            sig = sig * 31u + (unsigned) v.cmap;
            sig = sig * 31u + (unsigned) v.scale_mode;
            sig = sig * 31u + (unsigned) v.clean_stage;
            sig = sig * 31u + (unsigned) v.clean_win;
            sig = sig * 31u + (unsigned) v.clean_mean;
            sig = sig * 31u + (unsigned) v.fpi;
            if (v.scale_mode == SCALE_MANUAL)
                sig = sig * 31u + (unsigned) (v.dn_min ^ (v.dn_max << 3));
            if (cw != cov_w) {
                Color *np = (Color *) realloc(cov_pix, (size_t) cw * NPIX * sizeof *cov_pix);
                unsigned char *nm = (unsigned char *) realloc(wg_missing, (size_t) cw);
                if (np != NULL && nm != NULL) {
                    cov_pix = np; wg_missing = nm; cov_w = cw;
                    if (cov_tex.id != 0) UnloadTexture(cov_tex);
                    build_whereogram(&v, s, v.cmap, wlo, whi, cov_pix, cov_w, wg_missing,
                                     &wg_lo, &wg_hi);
                    cov_sig = sig; cov_valid = 1;
                    Image ci = { .data = cov_pix, .width = cov_w, .height = NPIX,
                                 .mipmaps = 1, .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
                    cov_tex = LoadTextureFromImage(ci);
                    SetTextureFilter(cov_tex, TEXTURE_FILTER_POINT);
                }
            } else if (!cov_valid || cov_sig != sig) {
                build_whereogram(&v, s, v.cmap, wlo, whi, cov_pix, cov_w, wg_missing,
                                 &wg_lo, &wg_hi);
                cov_sig = sig; cov_valid = 1;
                UpdateTexture(cov_tex, cov_pix);
            }
            if (cov_tex.id != 0) {
                Rectangle wsrc = { 0, 0, (float) cov_w, (float) NPIX };
                Rectangle wdst = { (float) ax, (float) ay, (float) cw, (float) ch };
                DrawTexturePro(cov_tex, wsrc, wdst, (Vector2){ 0, 0 }, 0.0f, WHITE);
                DrawRectangleLines(ax - 1, ay - 1, cw + 2, ch + 2, (Color){ 70, 70, 80, 255 });

                // A red rule above the panel over every stretch that never came
                // down. Those columns are painted in the background colour, and
                // background is not a reading -- without the rule there would be
                // nothing to tell "no data" from "nothing arriving".
                for (int c = 0; c < cov_w; c++) {
                    if (!wg_missing[c]) continue;
                    int x = ax + c * cw / cov_w;
                    int xe = ax + (c + 1) * cw / cov_w;
                    if (xe <= x) xe = x + 1;
                    DrawRectangle(x, ay - 5, xe - x, 3, (Color){ 220, 60, 60, 255 });
                }

                // Playback head: the stretch of the recording the shown image
                // was built from, sitting exactly on it -- on the panel's own
                // rows and on the pixels its frames occupy, with none of the
                // outward rounding that used to leave a margin inside the box.
                // Three pixels is the narrowest it can be and still have an
                // inside; below that the image is thinner than its own outline.
                long pb0 = 0, pb1 = 0;
                if (v.n_img > 0 && image_byte_span(&v, s, v.img_pos, &pb0, &pb1)) {
                    int x0 = ax + wg_x_of_byte(s, cw, pb0);
                    int x1 = ax + wg_x_of_byte(s, cw, pb1);
                    if (x1 - x0 < 3) x1 = x0 + 3;
                    DrawRectangleLines(x0, ay, x1 - x0, ch, RAYWHITE);
                }

                draw_text(TextFormat("%.1f%% of %.0f KB down, %.0f KB missing  (%s)",
                                     100.0 * (double) s->recovered / (double) s->size,
                                     s->size / 1024.0, (s->size - s->recovered) / 1024.0,
                                     s->size_from_log
                                         ? (s->size_exact ? "length from the satellite"
                                                          : "length is a satellite-reported minimum")
                                         : "length is the largest offset seen"),
                          ax, ay + ch + 8, 13, GRAY);
                if (s->unverified > 0)
                    draw_text(TextFormat("%.0f KB of it rests on packets whose CRC failed",
                                         s->unverified / 1024.0),
                              ax, ay + ch + 24, 13, (Color){ 206, 150, 42, 255 });
            }

            // Press the whereogram to jump to the image at that moment in the
            // recording, and keep dragging to scrub. Only a press that landed
            // on it starts a scrub; once one has, the pointer is clamped to the
            // panel, so wandering off an edge keeps scrubbing along that edge
            // instead of stopping dead.
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                Vector2 m = GetMousePosition();
                if (m.x >= ax && m.x < ax + cw && m.y >= ay && m.y < ay + ch) {
                    cov_drag = 1;
                    cov_anchor = wg_byte_of_col(s, cov_w, ((int) m.x - ax) * cov_w / cw);
                }
            }
            if (cov_drag && v.n_img > 0) {
                Vector2 m = GetMousePosition();
                int mx = (int) m.x - ax;
                if (mx < 0) mx = 0;
                if (mx > cw - 1) mx = cw - 1;
                long here = wg_byte_of_col(s, cov_w, mx * cov_w / cw);
                int img = image_nearest_byte(&v, s, here);
                if (img >= 0) { v.img_pos = img; v.playing = 0; }

                // The time ruler: the stretch of recording between where the
                // press landed and where the pointer is now, marked on the
                // whereogram and read out in UTC above it. It measures the
                // recording rather than the screen -- the two ends are the
                // times of the frames nearest those two bytes -- so a stretch
                // where nothing came down costs no time, the same way it takes
                // up no columns.
                int rx0 = ax + wg_x_of_byte(s, cw, cov_anchor);
                int rx1 = ax + mx;
                if (rx1 < rx0) { int t = rx0; rx0 = rx1; rx1 = t; }
                DrawRectangle(rx0, ay, rx1 - rx0 < 1 ? 1 : rx1 - rx0, ch,
                              (Color){ 255, 255, 255, 40 });
                DrawLine(rx0, ay, rx0, ay + ch, RAYWHITE);
                DrawLine(rx1, ay, rx1, ay + ch, RAYWHITE);
                double ta = time_at_byte(s, cov_anchor), tb = time_at_byte(s, here);
                if (ta >= 0.0 && tb >= 0.0) {
                    if (tb < ta) { double t = ta; ta = tb; tb = t; }
                    char t0[16], t1[16], sp[32];
                    fmt_tod_ms(ta, t0, sizeof t0);
                    fmt_tod_ms(tb, t1, sizeof t1);
                    fmt_span(tb - ta, sp, sizeof sp);
                    const char *lab = TextFormat("%s to %s UTC   %s", t0, t1, sp);
                    int tw = text_width(lab, 13);
                    int lx = (rx0 + rx1) / 2 - tw / 2;
                    if (lx < ax) lx = ax;
                    if (lx + tw > ax + cw) lx = ax + cw - tw;
                    DrawRectangle(lx - 6, ay + 4, tw + 12, 20, (Color){ 18, 18, 22, 220 });
                    draw_text(lab, lx, ay + 7, 13, RAYWHITE);
                }
            }
        }

        // the d-export outcome, above the help footer
        if (status_left > 0.0f)
            draw_text(status, 12, GetScreenHeight() - 42, 14, (Color){ 120, 220, 160, 255 });

        // help footer
        const char *help =
            "Up/Down experiment   Left/Right image   drag whereogram: scrub + time span"
            "   Space play/pause   ,/. speed   f steps/sweep   s zoom   a scale"
            "   z/x min  c/v max   m colour map   b clean  B step   n bg window   e median/mean"
            "   d re-download commands   F5 refresh  q quit";
        draw_text(help, 12, GetScreenHeight() - 22, 12, (Color){ 150, 150, 160, 255 });

        EndDrawing();
    }

    // Where this session got to, for the next one. Read while the window is
    // still up, since its size and place are the window's to report.
    Vector2 wpos = GetWindowPosition();
    geom.x = (int) wpos.x; geom.y = (int) wpos.y;
    geom.w = GetScreenWidth(); geom.h = GetScreenHeight();
    save_state(&v, &exps[v.sel], &geom);

    UnloadTexture(tex);
    if (cov_tex.id != 0) UnloadTexture(cov_tex);
    if (g_ui_font_loaded) UnloadFont(g_ui_font);
    CloseWindow();

    free_experiments(exps, nexp);
    free(v.fr_img);
    free_clean(&v);
    free(cov_pix);
    free(wg_missing);
    return 0;
}

#endif // WITH_SQLITE3
