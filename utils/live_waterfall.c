/*

    Simple Satellite Operations  utils/live_waterfall.c

    Real-time spectrogram viewer for an .iq capture in progress. Tails
    the file simple_sat_ops is writing, FFTs new IQ samples as they
    arrive, and renders a viridis spectrogram in a tall narrow raylib
    window beside the terminal UI. Newest row at the top; older rows
    scroll downward so the visual matches the operator's mental model
    of "the latest sample just landed".

    Independent process. simple_sat_ops doesn't link raylib; it just
    fork+execs this binary when a recording starts. Quitting either
    side leaves the other running.

    CLI:
        live_waterfall <iq_path> [options]

    Options:
        --rate=<Hz>          IQ sample rate (default 96000)
        --fft=<N>            FFT size, power of two (default 1024)
        --row-ms=<ms>        Time per spectrogram row (default 100)
        --zoom-khz=<W>       Visible bandwidth around DC (default = full rate)
        --width=<px>         Window width override (default = monitor/6)
        --height=<px>        Window height override (default = full)

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#include <raylib.h>

#include "argparse.h"
#include "waterfall_core.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <setjmp.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------------------------------------------------------------------
// Tailer: keep an FD open on the .iq file, read whatever new bytes
// have landed since the last poll. Detects truncation (recording
// restarted) by stat()ing periodically.
// --------------------------------------------------------------------

typedef struct {
    char    path[1024];
    FILE   *fp;
    off_t   read_pos;          // bytes consumed from the file
    time_t  last_size_check;
} iq_tail_t;

static int iq_tail_open(iq_tail_t *t, const char *path)
{
    snprintf(t->path, sizeof t->path, "%s", path);
    t->fp = fopen(t->path, "rb");
    if (t->fp == NULL) {
        return -1;
    }
    t->read_pos = 0;
    t->last_size_check = time(NULL);
    return 0;
}

static void iq_tail_close(iq_tail_t *t)
{
    if (t->fp != NULL) {
        fclose(t->fp);
        t->fp = NULL;
    }
}

// Returns number of new IQ pairs read into `buf` (which must hold at
// least cap_pairs * 4 bytes). 0 means no new data yet; -1 means the
// file disappeared. -2 means truncation detected (caller should reset
// internal state).
static int iq_tail_read(iq_tail_t *t, int16_t *buf, size_t cap_pairs)
{
    if (t->fp == NULL) {
        // Try to (re)open. Quiet on failure — the operator may have
        // not started recording yet.
        t->fp = fopen(t->path, "rb");
        if (t->fp == NULL) return -1;
        t->read_pos = 0;
    }

    // Detect truncation by stat (a re-opened recording would start the
    // file fresh; we'd want to re-seek to 0).
    struct stat st;
    if (stat(t->path, &st) == 0) {
        if ((off_t) st.st_size < t->read_pos) {
            // File got smaller — truncated. Reset.
            t->read_pos = 0;
            rewind(t->fp);
            return -2;
        }
    }

    // Seek to where we left off (handles other writers; harmless if
    // we're already there).
    if (fseeko(t->fp, t->read_pos, SEEK_SET) != 0) {
        fclose(t->fp); t->fp = NULL;
        return -1;
    }
    size_t got = fread(buf, sizeof(int16_t) * 2, cap_pairs, t->fp);
    if (got > 0) {
        t->read_pos += (off_t)(got * sizeof(int16_t) * 2);
    }
    return (int) got;
}

// --------------------------------------------------------------------
// Spectrogram state: an integrating accumulator that gathers FFT
// frames over row_ms milliseconds, then commits one row of viridis
// pixels to the scrollback texture.
// --------------------------------------------------------------------

typedef struct {
    unsigned n_fft;
    int      spec_w;            // visible bins after zoom crop
    int      spec_h;            // total rows in the scrollback texture
    int      bin_lo;            // first visible bin (after fftshift)
    int      bin_hi;            // one past last visible bin

    float   *win;               // Hann window, length n_fft
    float   *re;                // FFT scratch
    float   *im;
    double  *row_accum;         // linear power per visible bin
    float   *row_db;            // spec_w scratch for the dB-converted row
    int      row_accum_count;   // FFT frames integrated into row_accum

    // Sample accumulator (we may receive partial-frame chunks).
    int16_t *frame_buf;         // length n_fft * 2 (interleaved)
    int      frame_filled;      // IQ pairs accumulated so far

    // dB-domain auto-scaling: rolling min / max with hysteresis so the
    // colormap doesn't twitch every frame.
    float    auto_db_min;
    float    auto_db_max;
    int      have_auto_db;

    // Scrollback image (RGB pixels). Newest row at index 0; older rows
    // at higher indices.
    uint8_t *rgb;
    int      rgb_stride;        // bytes per row = spec_w * 3

    // Wall-clock timestamp per row (UNIX seconds). Mirrored into
    // every scroll so a label drawn at y reads row_time[y] and shows
    // the time of THAT pixel — the labels appear to scroll with the
    // spectrogram. Filled with 0 until the row has been committed.
    time_t  *row_time;
} spec_state_t;

static int spec_init(spec_state_t *s, unsigned n_fft, int spec_w, int spec_h,
                     double sample_rate_hz, double zoom_hz)
{
    s->n_fft = n_fft;
    int N = (int) n_fft;
    // Frequency-axis crop: keep bins inside ±zoom_hz/2 around DC.
    // After fftshift, bin 0 = -fs/2, bin N/2 = DC, bin N-1 = +fs/2-bin.
    int dc = N / 2;
    int half_bins = (int)((zoom_hz / 2.0) / sample_rate_hz * (double) N);
    if (half_bins < 1) half_bins = 1;
    if (half_bins > N / 2 - 1) half_bins = N / 2 - 1;
    s->bin_lo = dc - half_bins;
    s->bin_hi = dc + half_bins + 1;
    // spec_w stays at the caller's window width regardless of how few
    // FFT bins we're showing — when zoom narrows below 1 bin/pixel,
    // spec_one_frame's nearest-bin mapping just repeats columns. Keeps
    // the spectrogram filling the layout area when the operator
    // narrows zoom on the fly.
    s->spec_w = spec_w;
    s->spec_h = spec_h;
    s->win       = (float *)  calloc(N, sizeof(float));
    s->re        = (float *)  calloc(N, sizeof(float));
    s->im        = (float *)  calloc(N, sizeof(float));
    s->row_accum = (double *) calloc((size_t) spec_w, sizeof(double));
    s->row_db    = (float *)  calloc((size_t) spec_w, sizeof(float));
    s->frame_buf = (int16_t *)calloc((size_t) N * 2, sizeof(int16_t));
    s->rgb       = (uint8_t *)calloc((size_t) spec_w * (size_t) spec_h * 3, 1);
    s->row_time  = (time_t *) calloc((size_t) spec_h, sizeof(time_t));
    s->rgb_stride = spec_w * 3;
    if (!s->win || !s->re || !s->im || !s->row_accum || !s->row_db
        || !s->frame_buf || !s->rgb || !s->row_time) return -1;
    for (int k = 0; k < N; ++k) {
        s->win[k] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * (float) k
                                        / (float)(N - 1)));
    }
    s->row_accum_count = 0;
    s->frame_filled    = 0;
    s->have_auto_db    = 0;
    s->auto_db_min     = 0.0f;
    s->auto_db_max     = 12.0f;
    return 0;
}

static void spec_free(spec_state_t *s)
{
    free(s->win); free(s->re); free(s->im);
    free(s->row_accum); free(s->row_db); free(s->frame_buf); free(s->rgb);
    free(s->row_time);
}

// Reset the spec_state for a new zoom width. The window dimensions
// (spec_w × spec_h) stay the same; only the FFT-bin range and the
// scrollback content change. Wipes the existing rgb buffer + row_time
// so the operator sees the new zoom from scratch instead of a mix of
// pre/post-zoom rows.
static void spec_set_zoom(spec_state_t *s, double sample_rate_hz,
                          double zoom_hz)
{
    int N = (int) s->n_fft;
    int dc = N / 2;
    int half_bins = (int)((zoom_hz / 2.0) / sample_rate_hz * (double) N);
    if (half_bins < 1) half_bins = 1;
    if (half_bins > N / 2 - 1) half_bins = N / 2 - 1;
    s->bin_lo = dc - half_bins;
    s->bin_hi = dc + half_bins + 1;
    // Clear scrollback.
    memset(s->rgb, 0,
           (size_t) s->spec_w * (size_t) s->spec_h * 3);
    memset(s->row_time, 0, (size_t) s->spec_h * sizeof(time_t));
    memset(s->row_accum, 0, (size_t) s->spec_w * sizeof(double));
    s->row_accum_count = 0;
    s->frame_filled    = 0;
    s->have_auto_db    = 0;
    s->auto_db_min     = 0.0f;
    s->auto_db_max     = 12.0f;
}

// Run one FFT on the accumulated frame, fold into row_accum. spec_w is
// generally <= the bin count; nearest-bin downsample if it's smaller.
static void spec_one_frame(spec_state_t *s)
{
    int N = (int) s->n_fft;
    for (int k = 0; k < N; ++k) {
        float w = s->win[k];
        float I = (float) s->frame_buf[k * 2 + 0];
        float Q = (float) s->frame_buf[k * 2 + 1];
        s->re[k] = I * w;
        s->im[k] = Q * w;
    }
    wf_fft_forward(s->re, s->im, (unsigned) N);
    int avail = s->bin_hi - s->bin_lo;
    for (int x = 0; x < s->spec_w; ++x) {
        // Map output pixel x to a source-bin range [b_lo, b_hi).
        int b_lo = s->bin_lo + (int)((double) x       * (double) avail / s->spec_w);
        int b_hi = s->bin_lo + (int)((double)(x + 1) * (double) avail / s->spec_w);
        if (b_hi <= b_lo) b_hi = b_lo + 1;
        double sum = 0.0;
        for (int b = b_lo; b < b_hi; ++b) {
            int src = (b + N / 2) % N;  // fftshift
            double mag2 = (double) s->re[src] * s->re[src]
                        + (double) s->im[src] * s->im[src];
            sum += mag2;
        }
        s->row_accum[x] += sum / (double)(b_hi - b_lo);
    }
    ++s->row_accum_count;
}

// Commit the integrated row to the scrollback: scroll old rows down,
// paint new row at index 0.
static void spec_commit_row(spec_state_t *s)
{
    if (s->row_accum_count == 0) return;
    int W = s->spec_w;
    int H = s->spec_h;
    // Convert linear-power accumulator to dB. row_db is a persistent
    // spec_w buffer allocated once in spec_init -- an alloca here would be
    // an unbounded, no-failure-path stack allocation scaling with window width.
    float *row_db = s->row_db;
    float min_db = INFINITY, max_db = -INFINITY;
    for (int x = 0; x < W; ++x) {
        double p = s->row_accum[x] / (double) s->row_accum_count;
        float db = (float)(10.0 * log10(p + 1e-9));
        row_db[x] = db;
        if (db < min_db) min_db = db;
        if (db > max_db) max_db = db;
    }

    // Auto-scaling with hysteresis: target db_min = recent floor (10th
    // percentile-ish via min), db_max = recent peak. Smooth toward
    // those over many rows so the colormap doesn't twitch.
    if (!s->have_auto_db) {
        s->auto_db_min = min_db;
        s->auto_db_max = max_db;
        if (s->auto_db_max < s->auto_db_min + 12.0f)
            s->auto_db_max = s->auto_db_min + 12.0f;
        s->have_auto_db = 1;
    } else {
        const float alpha_down = 0.05f;   // floor follows down fast
        const float alpha_up   = 0.01f;   // peak follows down slowly
        if (min_db < s->auto_db_min)
            s->auto_db_min = (1.0f - alpha_down) * s->auto_db_min
                           + alpha_down * min_db;
        else
            s->auto_db_min = (1.0f - alpha_up)   * s->auto_db_min
                           + alpha_up   * min_db;
        if (max_db > s->auto_db_max)
            s->auto_db_max = (1.0f - alpha_down) * s->auto_db_max
                           + alpha_down * max_db;
        else
            s->auto_db_max = (1.0f - alpha_up)   * s->auto_db_max
                           + alpha_up   * max_db;
        if (s->auto_db_max < s->auto_db_min + 12.0f)
            s->auto_db_max = s->auto_db_min + 12.0f;
    }
    float lo = s->auto_db_min, hi = s->auto_db_max;
    float scale = (hi > lo) ? (255.0f / (hi - lo)) : 1.0f;

    // Scroll: row[r] := row[r-1] for r in [H-1 .. 1].
    memmove(s->rgb + (size_t) s->rgb_stride,
            s->rgb,
            (size_t) s->rgb_stride * (size_t)(H - 1));
    // Scroll the timestamp array in lock-step so the labels follow
    // their row pixels as the spectrogram crawls down.
    memmove(s->row_time + 1, s->row_time, (size_t)(H - 1) * sizeof(time_t));
    s->row_time[0] = time(NULL);
    // Paint new row at index 0.
    uint8_t *dst = s->rgb;
    for (int x = 0; x < W; ++x) {
        float norm = (row_db[x] - lo) * scale;
        int idx = (int) norm;
        if (idx < 0)   idx = 0;
        if (idx > 255) idx = 255;
        dst[x * 3 + 0] = WF_VIRIDIS[idx][0];
        dst[x * 3 + 1] = WF_VIRIDIS[idx][1];
        dst[x * 3 + 2] = WF_VIRIDIS[idx][2];
    }
    // Reset row accumulator.
    memset(s->row_accum, 0, (size_t) W * sizeof(double));
    s->row_accum_count = 0;
}

// Feed new IQ pairs into the spectrogram: accumulate into the frame
// buffer; whenever a full FFT frame is collected, run spec_one_frame.
// The caller drives row commits on a timer.
static void spec_push_iq(spec_state_t *s, const int16_t *iq, int n_pairs)
{
    int off = 0;
    int N = (int) s->n_fft;
    while (off < n_pairs) {
        int need = N - s->frame_filled;
        int take = (n_pairs - off < need) ? (n_pairs - off) : need;
        memcpy(s->frame_buf + s->frame_filled * 2,
               iq + off * 2,
               (size_t) take * 2 * sizeof(int16_t));
        s->frame_filled += take;
        off += take;
        if (s->frame_filled == N) {
            spec_one_frame(s);
            s->frame_filled = 0;
        }
    }
}

// Return a "nice" tick step (1, 2, 5, 10, ...) approximating the
// requested ratio of the full range so labels land at round numbers.
// Mirrors pick_tick_step in gen_waterfall.c.
static double pick_tick_step(double range, int approx_n)
{
    if (range <= 0 || approx_n < 1) return 1.0;
    double raw = range / (double) approx_n;
    double mag = pow(10.0, floor(log10(raw)));
    double mul = raw / mag;
    if      (mul < 1.5) mul = 1.0;
    else if (mul < 3.5) mul = 2.0;
    else if (mul < 7.5) mul = 5.0;
    else                mul = 10.0;
    return mul * mag;
}

// --------------------------------------------------------------------
// CLI parsing.
// --------------------------------------------------------------------

// Parsed command-line configuration. parse_args() fills this; main() copies
// the fields out into working locals so the (large) render body is unchanged.
typedef struct {
    const char *iq_path;
    double rate_hz;
    unsigned n_fft;
    int row_ms;
    double zoom_khz;
    int cli_w;
    int cli_h;
} wf_args_t;

// Option column width: the widest label below ("--zoom-khz=<W>") + a small
// margin. See src/cli/argparse.h for the parse_args convention.
#define OPTW 16

// Parse argv into *a (help == 0), or print one right-aligned help line per
// option and return (help != 0). Each option is one self-contained block whose
// test carries "|| help", so help mode falls through and prints them all.
static int parse_args(wf_args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // <iq_path>: first non-option token. A lone "-" counts as a
        // positional. Declared first so it lists above the --options in help.
        if ((a->iq_path == NULL && (arg[0] != '-' || strcmp(arg, "-") == 0)) || help) {
            if (help) parse_help_line(OPTW, "<iq_path>", "IQ capture to tail (.iq, interleaved S16_LE)");
            else a->iq_path = arg;
            matched = 1;
        }
        if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0 || help) {
            if (help) parse_help_line(OPTW, "-h, --help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strncmp(arg, "--rate=", 7) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rate=<Hz>", "IQ rate (default 96000)");
            else a->rate_hz = atof(arg + 7);
            matched = 1;
        }
        if (strncmp(arg, "--fft=", 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--fft=<N>", "FFT size, power of two (default 1024)");
            else a->n_fft = (unsigned) atoi(arg + 6);
            matched = 1;
        }
        if (strncmp(arg, "--row-ms=", 9) == 0 || help) {
            if (help) parse_help_line(OPTW, "--row-ms=<ms>", "Time per spectrogram row (default 100)");
            else a->row_ms = atoi(arg + 9);
            matched = 1;
        }
        if (strncmp(arg, "--zoom-khz=", 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--zoom-khz=<W>", "Visible BW around DC (default = full rate)");
            else a->zoom_khz = atof(arg + 11);
            matched = 1;
        }
        if (strncmp(arg, "--width=", 8) == 0 || help) {
            if (help) parse_help_line(OPTW, "--width=<px>", "Window width override (default = monitor/3)");
            else a->cli_w = atoi(arg + 8);
            matched = 1;
        }
        if (strncmp(arg, "--height=", 9) == 0 || help) {
            if (help) parse_help_line(OPTW, "--height=<px>", "Window height override (default = 95% of monitor)");
            else a->cli_h = atoi(arg + 9);
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "live_waterfall: unknown option %s\n", arg);
            return PARSE_ERROR;
        }
    }
    return PARSE_OK;
}

// Trap signals (SIGSEGV / SIGABRT) raised inside InitWindow so a
// hostile X server / GLX setup doesn't core-dump the process before
// we can print a useful hint. Same pattern as decode_inspector.
static sigjmp_buf g_init_window_jmp;
static void init_window_crash_handler(int sig)
{
    (void) sig;
    siglongjmp(g_init_window_jmp, 1);
}

static int safe_init_window(int w, int h, const char *title)
{
    struct sigaction old_segv, old_abrt;
    struct sigaction trap = {0};
    trap.sa_handler = init_window_crash_handler;
    sigemptyset(&trap.sa_mask);
    sigaction(SIGSEGV, &trap, &old_segv);
    sigaction(SIGABRT, &trap, &old_abrt);

    int ok = 0;
    if (sigsetjmp(g_init_window_jmp, 1) == 0) {
        InitWindow(w, h, title);
        ok = IsWindowReady();
    }
    sigaction(SIGSEGV, &old_segv, NULL);
    sigaction(SIGABRT, &old_abrt, NULL);
    return ok ? 0 : -1;
}

// (No software-renderer fallback — `LIBGL_ALWAYS_SOFTWARE=1` is
// still OpenGL, just on the CPU. See decode_inspector.c for the
// long explanation.)

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "live_waterfall")) return 0;
    wf_args_t cfg = {
        .rate_hz = 96000.0,
        .n_fft   = 1024,
        .row_ms  = 100,
        // Default = full capture width. 0 means "the whole rate"; the clamp
        // below turns it into rate/1000 kHz, so it's full at any sample rate
        // (not a hard-coded 96). Operator can narrow mid-flight via stdin
        // (see :lo_bandwidth in simple_sat_ops) or pass --zoom-khz=<N>.
        .zoom_khz = 0.0,
        .cli_w   = 0,
        .cli_h   = 0,
    };
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 2;
    }
    if (cfg.iq_path == NULL) { return 2; }

    // Copy parsed config into the working locals the render body below uses.
    const char *iq_path  = cfg.iq_path;
    double rate_hz       = cfg.rate_hz;
    unsigned n_fft       = cfg.n_fft;
    int      row_ms      = cfg.row_ms;
    double   zoom_khz    = cfg.zoom_khz;
    int      cli_w       = cfg.cli_w;
    int      cli_h       = cfg.cli_h;
    if (!wf_is_pow2(n_fft) || n_fft < 16) {
        fprintf(stderr, "live_waterfall: --fft must be power of 2 >= 16\n");
        return 2;
    }
    if (row_ms < 1 || row_ms > 10000) row_ms = 100;
    if (zoom_khz <= 0 || zoom_khz > rate_hz / 1000.0) {
        zoom_khz = rate_hz / 1000.0;
    }

    SetTraceLogLevel(LOG_WARNING);
    if (safe_init_window(64, 64, "live_waterfall") != 0) {
        // No fallback: raylib is OpenGL-only and the GLX wire
        // protocol over SSH-forwarded X11 can't carry an OpenGL
        // 3.3 core context. See decode_inspector.c for the long
        // explanation.
        fprintf(stderr,
            "live_waterfall: cannot open a window.\n"
            "\n"
            "If you're connected over SSH with `-X` / `-Y`:\n"
            "  vanilla X11 forwarding only carries GLX 1.x and the\n"
            "  OpenGL 3.3 core profile raylib needs can't traverse it.\n"
            "  Rebuild raylib with -DOPENGL_VERSION=2.1 and relink\n"
            "  (see README.md), or run locally / via Xpra / VirtualGL.\n");
        return 1;
    }
    int monitor = GetCurrentMonitor();
    int mon_w = GetMonitorWidth(monitor);
    int mon_h = GetMonitorHeight(monitor);
    // Default window is monitor/3 wide — twice the old monitor/6 — so
    // 96 kHz of bandwidth is readable next to the operator UI without
    // the whole spectrum getting smushed into one bin per pixel.
    int win_w = (cli_w > 0) ? cli_w : (mon_w / 3);
    int win_h = (cli_h > 0) ? cli_h : (int)(mon_h * 0.95);
    if (win_w < 200) win_w = 200;
    if (win_h < 400) win_h = 400;
    SetWindowSize(win_w, win_h);
    SetWindowTitle("live_waterfall");
    SetTargetFPS(30);

    // Layout:
    //   left_pad  — time labels (HH:MM:SS) every label_step_rows
    //   top_bar_h — status (REC / waiting) + dB scale readout
    //   colorbar  — 14 px viridis strip on the right edge with dB labels
    //   bottom_bar_h — freq tick labels (kHz)
    const int top_bar_h    = 22;
    const int bottom_bar_h = 22;
    const int left_pad     = 60;    // room for "HH:MM:SS"
    const int cb_w         = 14;    // colorbar strip
    const int cb_label_w   = 28;    // room for "+0..-90" labels
    const int right_pad    = cb_w + cb_label_w + 4;
    int spec_w = win_w - left_pad - right_pad;
    int spec_h = win_h - top_bar_h - bottom_bar_h;
    if (spec_w < 64) spec_w = 64;
    if (spec_h < 64) spec_h = 64;

    spec_state_t S = {0};
    if (spec_init(&S, n_fft, spec_w, spec_h, rate_hz, zoom_khz * 1000.0) != 0) {
        fprintf(stderr, "live_waterfall: spec_init failed\n");
        CloseWindow();
        return 1;
    }
    // raylib texture for the spectrogram. We update it from S.rgb each
    // time we paint a new row.
    Image spec_img = {
        .data = S.rgb,
        .width = S.spec_w,
        .height = S.spec_h,
        .mipmaps = 1,
        .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8,
    };
    Texture2D spec_tex = LoadTextureFromImage(spec_img);
    SetTextureFilter(spec_tex, TEXTURE_FILTER_POINT);

    iq_tail_t tail = {0};
    iq_tail_open(&tail, iq_path);   // may fail silently — we'll retry

    // FFT frame period (s) → frames per row.
    double frame_period_s = (double) n_fft / rate_hz;
    double row_period_s   = row_ms / 1000.0;
    double frames_per_row = row_period_s / frame_period_s;
    if (frames_per_row < 1.0) frames_per_row = 1.0;
    int target_frames_per_row = (int)(frames_per_row + 0.5);
    if (target_frames_per_row < 1) target_frames_per_row = 1;

    const int IQ_CHUNK_PAIRS = 8192;
    int16_t *iq_chunk = (int16_t *) malloc(IQ_CHUNK_PAIRS * 2 * sizeof(int16_t));
    if (!iq_chunk) {
        fprintf(stderr, "live_waterfall: out of memory\n");
        UnloadTexture(spec_tex);
        spec_free(&S);
        CloseWindow();
        return 1;
    }

    // Stdin command channel: simple_sat_ops's --live-waterfall launch
    // wires its child's stdin to a pipe so colon-commands like
    // :wf_zoom_khz <N> can adjust this viewer at runtime. Standalone
    // invocations from a shell never write commands; the non-blocking
    // read just returns EAGAIN forever and we draw normally.
    int stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (stdin_flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK);
    }
    char ctl_buf[256] = {0};
    size_t ctl_buf_len = 0;
    double current_zoom_khz = zoom_khz;

    while (!WindowShouldClose()) {
        // 0. Drain any pending control commands from stdin. Line-based
        // protocol; commands today:
        //     bandwidth <N>  → reset spec_state to ±N/2 kHz visible
        //     zoom <N>       → legacy alias for `bandwidth` (same effect)
        ssize_t rn;
        while ((rn = read(STDIN_FILENO, ctl_buf + ctl_buf_len,
                          sizeof ctl_buf - ctl_buf_len - 1)) > 0) {
            ctl_buf_len += (size_t) rn;
            ctl_buf[ctl_buf_len] = '\0';
            char *line = ctl_buf;
            char *nl;
            while ((nl = strchr(line, '\n')) != NULL) {
                *nl = '\0';
                double n;
                if ((sscanf(line, " bandwidth %lf", &n) == 1
                     || sscanf(line, " zoom %lf",    &n) == 1)
                    && n > 0.0 && n <= rate_hz / 1000.0) {
                    current_zoom_khz = n;
                    spec_set_zoom(&S, rate_hz, n * 1000.0);
                    fprintf(stderr,
                        "live_waterfall: bandwidth set to %g kHz\n", n);
                }
                line = nl + 1;
            }
            size_t remain = ctl_buf_len - (size_t)(line - ctl_buf);
            memmove(ctl_buf, line, remain);
            ctl_buf_len = remain;
        }

        // 1. Drain new IQ samples into the spec accumulator.
        int read_count;
        do {
            read_count = iq_tail_read(&tail, iq_chunk, IQ_CHUNK_PAIRS);
            if (read_count == -2) {
                // Truncation — reset spec state.
                memset(S.row_accum, 0, (size_t) S.spec_w * sizeof(double));
                S.row_accum_count = 0;
                S.frame_filled    = 0;
                S.have_auto_db    = 0;
            } else if (read_count > 0) {
                spec_push_iq(&S, iq_chunk, read_count);
            }
        } while (read_count > 0);

        // 2. Commit row if we've integrated enough FFT frames.
        while (S.row_accum_count >= target_frames_per_row) {
            spec_commit_row(&S);
            UpdateTexture(spec_tex, S.rgb);
        }

        // 3. Draw.
        BeginDrawing();
        ClearBackground(BLACK);

        // ---- Top status bar -----------------------------------------
        const char *status = (tail.fp != NULL) ? "REC" : "(waiting for .iq)";
        Color status_col = (tail.fp != NULL) ? (Color){80, 200, 80, 255}
                                              : (Color){200, 180, 80, 255};
        DrawText(status, left_pad, 4, 14, status_col);

        // ---- Spectrogram --------------------------------------------
        DrawTexture(spec_tex, left_pad, top_bar_h, WHITE);

        // ---- Time labels that scroll with the spectrogram ----------
        // Label each row whose row_time[] is on a 10-second boundary
        // AND is the FIRST row showing that second (i.e. row_time[y]
        // differs from row_time[y-1]). This produces labels anchored
        // to the actual time of the pixel row beside them — the
        // labels drift downward as new rows scroll in, instead of
        // floating at fixed y while their content jumps.
        const int label_period_s = 10;
        const Color time_col = (Color){180, 180, 180, 255};
        for (int y = 0; y < spec_h; ++y) {
            time_t t = S.row_time[y];
            if (t == 0) continue;
            if (((long) t % label_period_s) != 0) continue;
            if (y > 0 && S.row_time[y - 1] == t) continue;
            struct tm lt;
            localtime_r(&t, &lt);
            char buf[16];
            snprintf(buf, sizeof buf, "%02d:%02d:%02d",
                     lt.tm_hour, lt.tm_min, lt.tm_sec);
            int draw_y = top_bar_h + y - 5;
            DrawText(buf, 4, draw_y, 11, time_col);
            DrawLine(left_pad - 4, top_bar_h + y, left_pad - 1,
                     top_bar_h + y, time_col);
        }

        // ---- Frequency ticks at the bottom --------------------------
        // Round-number ticks at pick_tick_step(current_zoom_khz, ~5)
        // intervals. Spectrogram x covers ±current_zoom_khz/2 across
        // spec_w pixels (current_zoom_khz tracks runtime adjustments
        // via the stdin "zoom <N>" command).
        double f_lo  = -current_zoom_khz / 2.0;
        double f_hi  = +current_zoom_khz / 2.0;
        double f_step = pick_tick_step(current_zoom_khz, 5);
        double f0 = ceil(f_lo / f_step) * f_step;
        const Color tick_col  = (Color){160, 160, 160, 255};
        const Color flbl_col  = (Color){200, 200, 200, 255};
        for (double f = f0; f <= f_hi + 0.5 * f_step; f += f_step) {
            int x = left_pad + (int)((f - f_lo) / current_zoom_khz * spec_w);
            // Tick mark hanging below the spectrogram.
            DrawLine(x, top_bar_h + spec_h, x, top_bar_h + spec_h + 4,
                     tick_col);
            char buf[16];
            if (fabs(f) < 0.05) snprintf(buf, sizeof buf, "0");
            else if (fabs(f - floor(f + 0.5)) < 0.05)
                snprintf(buf, sizeof buf, "%+d", (int) f);
            else snprintf(buf, sizeof buf, "%+.1f", f);
            int tw = MeasureText(buf, 11);
            DrawText(buf, x - tw / 2,
                     top_bar_h + spec_h + 6, 11, flbl_col);
        }
        // "kHz" unit at far right.
        DrawText("kHz", left_pad + spec_w - 20,
                 top_bar_h + spec_h + 10, 10, GRAY);

        // ---- Colorbar on the right ----------------------------------
        // 14 px wide viridis strip with dB labels (auto_db_max at top,
        // auto_db_min at bottom). Hand-drawn one row at a time so we
        // don't need a second texture.
        int cb_x = win_w - right_pad + 2;
        int cb_top = top_bar_h;
        int cb_bot = top_bar_h + spec_h - 1;
        int cb_h   = cb_bot - cb_top + 1;
        for (int yy = 0; yy < cb_h; ++yy) {
            // Top of bar = high dB = bright viridis (index 255), so
            // map y=0 → idx=255, y=cb_h-1 → idx=0.
            int idx = (int)(255.0 * (1.0 - (double) yy / (cb_h - 1)));
            if (idx < 0)   idx = 0;
            if (idx > 255) idx = 255;
            Color c = {WF_VIRIDIS[idx][0], WF_VIRIDIS[idx][1], WF_VIRIDIS[idx][2], 255};
            DrawRectangle(cb_x, cb_top + yy, cb_w, 1, c);
        }
        // Frame around colorbar.
        DrawRectangleLines(cb_x, cb_top, cb_w, cb_h, GRAY);
        // dB labels (3 levels: max at top, mid, min at bottom).
        {
            char buf[24];
            float lo_db = S.auto_db_min, hi_db = S.auto_db_max;
            // Top label.
            snprintf(buf, sizeof buf, "%+.0f", hi_db);
            DrawText(buf, cb_x + cb_w + 3, cb_top - 2, 10, flbl_col);
            // Bottom label.
            snprintf(buf, sizeof buf, "%+.0f", lo_db);
            DrawText(buf, cb_x + cb_w + 3, cb_bot - 8, 10, flbl_col);
            // "dB" unit just above the bar.
            DrawText("dB", cb_x - 3, cb_top - 14, 10, GRAY);
        }

        EndDrawing();
    }

    free(iq_chunk);
    iq_tail_close(&tail);
    UnloadTexture(spec_tex);
    spec_free(&S);
    CloseWindow();
    return 0;
}
