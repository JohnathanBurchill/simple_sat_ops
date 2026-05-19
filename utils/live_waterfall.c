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
        --zoom-khz=<W>       Visible bandwidth around DC (default 30)
        --width=<px>         Window width override (default = monitor/6)
        --height=<px>        Window height override (default = full)

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#include <raylib.h>

#include <errno.h>
#include <math.h>
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
// Radix-2 in-place complex FFT — same as gen_waterfall.c.
// --------------------------------------------------------------------

static int is_pow2(unsigned n) { return n > 0 && (n & (n - 1)) == 0; }

static void fft_bit_reverse(float *re, float *im, unsigned n)
{
    unsigned j = 0;
    for (unsigned i = 1; i < n; ++i) {
        unsigned bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
}

static void fft_forward(float *re, float *im, unsigned n)
{
    fft_bit_reverse(re, im, n);
    for (unsigned len = 2; len <= n; len <<= 1) {
        double ang = -2.0 * M_PI / (double) len;
        double wr_step = cos(ang);
        double wi_step = sin(ang);
        unsigned half = len >> 1;
        for (unsigned i = 0; i < n; i += len) {
            double wr = 1.0, wi = 0.0;
            for (unsigned k = 0; k < half; ++k) {
                unsigned a = i + k;
                unsigned b = a + half;
                double tr = wr * re[b] - wi * im[b];
                double ti = wr * im[b] + wi * re[b];
                re[b] = (float)(re[a] - tr);
                im[b] = (float)(im[a] - ti);
                re[a] = (float)(re[a] + tr);
                im[a] = (float)(im[a] + ti);
                double nwr = wr * wr_step - wi * wi_step;
                double nwi = wr * wi_step + wi * wr_step;
                wr = nwr;
                wi = nwi;
            }
        }
    }
}

// --------------------------------------------------------------------
// Viridis 256-entry colormap (mirrored from gen_waterfall.c).
// --------------------------------------------------------------------

static const uint8_t VIRIDIS[256][3] = {
    { 68,  1, 84},{ 68,  2, 86},{ 69,  4, 87},{ 69,  5, 89},{ 70,  7, 90},
    { 70,  8, 92},{ 70, 10, 93},{ 70, 11, 94},{ 71, 13, 96},{ 71, 14, 97},
    { 71, 16, 99},{ 71, 17,100},{ 71, 19,101},{ 72, 20,103},{ 72, 22,104},
    { 72, 23,105},{ 72, 24,106},{ 72, 26,108},{ 72, 27,109},{ 72, 28,110},
    { 72, 29,111},{ 72, 31,112},{ 72, 32,113},{ 72, 33,115},{ 72, 35,116},
    { 72, 36,117},{ 72, 37,118},{ 72, 38,119},{ 72, 40,120},{ 72, 41,121},
    { 71, 42,122},{ 71, 44,122},{ 71, 45,123},{ 71, 46,124},{ 71, 47,125},
    { 70, 48,126},{ 70, 50,126},{ 70, 51,127},{ 70, 52,128},{ 69, 53,129},
    { 69, 55,129},{ 69, 56,130},{ 68, 57,131},{ 68, 58,131},{ 68, 59,132},
    { 67, 61,132},{ 67, 62,133},{ 66, 63,133},{ 66, 64,134},{ 66, 65,134},
    { 65, 66,135},{ 65, 68,135},{ 64, 69,136},{ 64, 70,136},{ 63, 71,136},
    { 63, 72,137},{ 62, 73,137},{ 62, 74,137},{ 62, 76,138},{ 61, 77,138},
    { 61, 78,138},{ 60, 79,138},{ 60, 80,139},{ 59, 81,139},{ 59, 82,139},
    { 58, 83,139},{ 58, 84,140},{ 57, 85,140},{ 57, 86,140},{ 56, 88,140},
    { 56, 89,140},{ 55, 90,140},{ 55, 91,141},{ 54, 92,141},{ 54, 93,141},
    { 53, 94,141},{ 53, 95,141},{ 52, 96,141},{ 52, 97,141},{ 51, 98,141},
    { 51, 99,141},{ 50,100,142},{ 50,101,142},{ 49,102,142},{ 49,103,142},
    { 49,104,142},{ 48,105,142},{ 48,106,142},{ 47,107,142},{ 47,108,142},
    { 46,109,142},{ 46,110,142},{ 46,111,142},{ 45,112,142},{ 45,113,142},
    { 44,113,142},{ 44,114,142},{ 44,115,142},{ 43,116,142},{ 43,117,142},
    { 42,118,142},{ 42,119,142},{ 42,120,142},{ 41,121,142},{ 41,122,142},
    { 41,123,142},{ 40,124,142},{ 40,125,142},{ 39,126,142},{ 39,127,142},
    { 39,128,142},{ 38,129,142},{ 38,130,142},{ 38,130,142},{ 37,131,142},
    { 37,132,142},{ 37,133,142},{ 36,134,142},{ 36,135,142},{ 35,136,142},
    { 35,137,142},{ 35,138,141},{ 34,139,141},{ 34,140,141},{ 34,141,141},
    { 33,142,141},{ 33,143,141},{ 33,144,141},{ 33,145,140},{ 32,146,140},
    { 32,146,140},{ 32,147,140},{ 31,148,140},{ 31,149,139},{ 31,150,139},
    { 31,151,139},{ 31,152,139},{ 31,153,138},{ 31,154,138},{ 30,155,138},
    { 30,156,137},{ 30,157,137},{ 31,158,137},{ 31,159,136},{ 31,160,136},
    { 31,161,136},{ 31,161,135},{ 31,162,135},{ 32,163,134},{ 32,164,134},
    { 33,165,133},{ 33,166,133},{ 34,167,133},{ 34,168,132},{ 35,169,131},
    { 36,170,131},{ 37,171,130},{ 37,172,130},{ 38,173,129},{ 39,173,129},
    { 40,174,128},{ 41,175,127},{ 42,176,127},{ 44,177,126},{ 45,178,125},
    { 46,179,124},{ 47,180,124},{ 49,181,123},{ 50,182,122},{ 52,182,121},
    { 53,183,121},{ 55,184,120},{ 56,185,119},{ 58,186,118},{ 59,187,117},
    { 61,188,116},{ 63,188,115},{ 64,189,114},{ 66,190,113},{ 68,191,112},
    { 70,192,111},{ 72,193,110},{ 74,193,109},{ 76,194,108},{ 78,195,107},
    { 80,196,106},{ 82,197,105},{ 84,197,104},{ 86,198,103},{ 88,199,101},
    { 90,200,100},{ 92,200, 99},{ 94,201, 98},{ 96,202, 96},{ 99,203, 95},
    {101,203, 94},{103,204, 92},{105,205, 91},{108,205, 90},{110,206, 88},
    {112,207, 87},{115,208, 86},{117,208, 84},{119,209, 83},{122,209, 81},
    {124,210, 80},{127,211, 78},{129,211, 77},{132,212, 75},{134,213, 73},
    {137,213, 72},{139,214, 70},{142,214, 69},{144,215, 67},{147,215, 65},
    {149,216, 64},{152,216, 62},{155,217, 60},{157,217, 59},{160,218, 57},
    {162,218, 55},{165,219, 54},{168,219, 52},{170,220, 50},{173,220, 48},
    {176,221, 47},{178,221, 45},{181,222, 43},{184,222, 41},{186,222, 40},
    {189,223, 38},{192,223, 37},{194,223, 35},{197,224, 33},{200,224, 32},
    {202,225, 31},{205,225, 29},{208,225, 28},{210,226, 27},{213,226, 26},
    {216,226, 25},{218,227, 25},{221,227, 24},{223,227, 24},{226,228, 24},
    {229,228, 25},{231,228, 25},{234,229, 26},{236,229, 27},{239,229, 28},
    {241,229, 29},{244,230, 30},{246,230, 32},{248,230, 33},{251,231, 35},
    {253,231, 37}
};

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
    int avail_w = s->bin_hi - s->bin_lo;
    // Render width is bounded by the available bins (oversampling has
    // no benefit) AND by the requested spec_w. We just shrink spec_w
    // to avail_w if it's wider.
    if (avail_w < spec_w) spec_w = avail_w;
    s->spec_w = spec_w;
    s->spec_h = spec_h;
    s->win       = (float *)  calloc(N, sizeof(float));
    s->re        = (float *)  calloc(N, sizeof(float));
    s->im        = (float *)  calloc(N, sizeof(float));
    s->row_accum = (double *) calloc((size_t) spec_w, sizeof(double));
    s->frame_buf = (int16_t *)calloc((size_t) N * 2, sizeof(int16_t));
    s->rgb       = (uint8_t *)calloc((size_t) spec_w * (size_t) spec_h * 3, 1);
    s->rgb_stride = spec_w * 3;
    if (!s->win || !s->re || !s->im || !s->row_accum
        || !s->frame_buf || !s->rgb) return -1;
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
    free(s->row_accum); free(s->frame_buf); free(s->rgb);
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
    fft_forward(s->re, s->im, (unsigned) N);
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
    // Convert linear-power accumulator to dB.
    float *row_db = (float *) alloca((size_t) W * sizeof(float));
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
    // Paint new row at index 0.
    uint8_t *dst = s->rgb;
    for (int x = 0; x < W; ++x) {
        float norm = (row_db[x] - lo) * scale;
        int idx = (int) norm;
        if (idx < 0)   idx = 0;
        if (idx > 255) idx = 255;
        dst[x * 3 + 0] = VIRIDIS[idx][0];
        dst[x * 3 + 1] = VIRIDIS[idx][1];
        dst[x * 3 + 2] = VIRIDIS[idx][2];
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

// --------------------------------------------------------------------
// CLI parsing.
// --------------------------------------------------------------------

static void usage(void)
{
    fprintf(stderr,
        "usage: live_waterfall <iq_path> [options]\n"
        "  --rate=<Hz>        IQ rate (default 96000)\n"
        "  --fft=<N>          FFT size, power of two (default 1024)\n"
        "  --row-ms=<ms>      Time per spectrogram row (default 100)\n"
        "  --zoom-khz=<W>     Visible BW around DC (default 30)\n"
        "  --width=<px>       Window width override\n"
        "  --height=<px>      Window height override\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) { usage(); return 2; }
    if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0) {
        usage(); return 0;
    }
    const char *iq_path  = argv[1];
    double rate_hz       = 96000.0;
    unsigned n_fft       = 1024;
    int      row_ms      = 100;
    double   zoom_khz    = 30.0;
    int      cli_w       = 0;
    int      cli_h       = 0;
    for (int i = 2; i < argc; ++i) {
        if      (strncmp(argv[i], "--rate=", 7) == 0)     rate_hz  = atof(argv[i] + 7);
        else if (strncmp(argv[i], "--fft=", 6) == 0)      n_fft    = (unsigned) atoi(argv[i] + 6);
        else if (strncmp(argv[i], "--row-ms=", 9) == 0)   row_ms   = atoi(argv[i] + 9);
        else if (strncmp(argv[i], "--zoom-khz=", 11) == 0) zoom_khz = atof(argv[i] + 11);
        else if (strncmp(argv[i], "--width=", 8) == 0)    cli_w    = atoi(argv[i] + 8);
        else if (strncmp(argv[i], "--height=", 9) == 0)   cli_h    = atoi(argv[i] + 9);
        else { fprintf(stderr, "live_waterfall: unknown option %s\n", argv[i]);
               usage(); return 2; }
    }
    if (!is_pow2(n_fft) || n_fft < 16) {
        fprintf(stderr, "live_waterfall: --fft must be power of 2 >= 16\n");
        return 2;
    }
    if (row_ms < 1 || row_ms > 10000) row_ms = 100;
    if (zoom_khz <= 0 || zoom_khz > rate_hz / 1000.0) {
        zoom_khz = rate_hz / 1000.0;
    }

    SetTraceLogLevel(LOG_WARNING);
    InitWindow(64, 64, "live_waterfall");
    int monitor = GetCurrentMonitor();
    int mon_w = GetMonitorWidth(monitor);
    int mon_h = GetMonitorHeight(monitor);
    int win_w = (cli_w > 0) ? cli_w : (mon_w / 6);
    int win_h = (cli_h > 0) ? cli_h : (int)(mon_h * 0.95);
    if (win_w < 200) win_w = 200;
    if (win_h < 400) win_h = 400;
    SetWindowSize(win_w, win_h);
    SetWindowTitle("live_waterfall");
    SetTargetFPS(30);

    // Layout: small top status bar, big spectrogram, freq labels at bottom.
    const int top_bar_h    = 22;
    const int bottom_bar_h = 18;
    const int left_pad     = 4;
    const int right_pad    = 4;
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

    while (!WindowShouldClose()) {
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
        // Top status bar.
        const char *status = (tail.fp != NULL) ? "REC" : "(waiting for .iq)";
        Color status_col = (tail.fp != NULL) ? (Color){80, 200, 80, 255}
                                              : (Color){200, 180, 80, 255};
        DrawText(status, left_pad, 4, 14, status_col);
        // Spectrogram.
        DrawTexture(spec_tex, left_pad, top_bar_h, WHITE);
        // Freq labels (just centre + edges).
        char buf[24];
        int fy = top_bar_h + spec_h + 2;
        snprintf(buf, sizeof buf, "-%g", zoom_khz / 2);
        DrawText(buf, left_pad, fy, 12, GRAY);
        snprintf(buf, sizeof buf, "0");
        DrawText(buf, left_pad + spec_w / 2 - 4, fy, 12, GRAY);
        snprintf(buf, sizeof buf, "+%g", zoom_khz / 2);
        int tw = MeasureText(buf, 12);
        DrawText(buf, left_pad + spec_w - tw, fy, 12, GRAY);
        // dB scale info, top-right.
        snprintf(buf, sizeof buf, "%+.0f..%+.0fdB",
                 S.auto_db_min, S.auto_db_max);
        int dw = MeasureText(buf, 12);
        DrawText(buf, win_w - right_pad - dw, 6, 12, LIGHTGRAY);
        EndDrawing();
    }

    free(iq_chunk);
    iq_tail_close(&tail);
    UnloadTexture(spec_tex);
    spec_free(&S);
    CloseWindow();
    return 0;
}
