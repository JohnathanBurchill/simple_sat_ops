/*

    Simple Satellite Operations  utils/decode_inspector.c

    Interactive raylib viewer that lets the operator mouse-draw
    time × frequency boxes around bursts in an .iq capture, then save
    them as anchors for rx_replay.

    Pipeline: the FFT / dB / detrend / notch / zoom logic lives in
    utils/waterfall_core.{c,h} and runs in-process at startup; no
    subprocess, no /tmp PNG. The resulting float dB grid is uploaded
    to a tiled R32F GPU texture and re-coloured live by a fragment
    shader (analytic viridis polynomial), so the colour-scale keys
    (',' '.' '<' '>' 'R') are uniform updates rather than full
    re-renders.

    Output:
        <iq_path>.boxes.csv          — t_start_s,t_end_s,f_lo_hz,f_hi_hz,label
        <iq_path>.box_anchors.csv    — burst.csv-compatible (rx_replay --anchor-csv)

    Keys:
        Mouse drag (LMB)     Draw a new box
        Click in a box       Select it
        Backspace / Delete   Delete the selected box
        T                    Cycle label of the selected box
        S                    Save both CSVs
        L                    Load <iq>.boxes.csv
        , / .                Decrease / increase color-scale floor (db-min)
        < / >                Decrease / increase color-scale ceiling (db-max)
        R                    Reset color scale to percentile auto-clip
        ↑/↓, PgUp/PgDn,      Scroll
        Home/End, wheel
        Q                    Quit

    CLI: same flag forms gen_waterfall accepts, plus:
        --width=<px>, --height=<px>  raylib window size

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#include <raylib.h>
#include <rlgl.h>

#include "golay24.h"
#include "modem.h"
#include "modem_fsk.h"
#include "pdf_writer.h"
#include "rs.h"
#include "sw_nco.h"
#include "waterfall_core.h"

#ifdef __APPLE__
// Trackpad pinch is delivered as NSEventTypeMagnify, which raylib/GLFW
// don't forward. utils/decode_inspector_macos.m installs an NSEvent local
// monitor that accumulates magnification deltas into this global; we
// read & reset it each frame.
extern float g_decode_inspector_pinch_delta;
extern void  decode_inspector_install_pinch_monitor(void);
#endif

#include <ctype.h>
#include <errno.h>
#include <libgen.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

// gen_waterfall's hard-coded image layout (PNG path, render_with_axes).
// Keep these in lockstep with utils/gen_waterfall.c.
#define WF_LM 80
#define WF_RM 124
#define WF_TM 12
#define WF_BM 28

// CCSDS RX scrambler XOR table — same constants ax100.c uses for
// descrambling on the receive side. Polynomial h(x) = x^8 + x^7 +
// x^5 + x^3 + 1, fbmask = 0xA9, initreg = 0xFF. Duplicated here so
// the decode inspector can run the descramble stage without pulling
// the full ax100 translation unit (and its OpenSSL transitives).
static const uint8_t DECMODE_CCSDS_TABLE[255] = {
    0xFF, 0x48, 0x0E, 0xC0, 0x9A, 0x0D, 0x70, 0xBC,
    0x8E, 0x2C, 0x93, 0xAD, 0xA7, 0xB7, 0x46, 0xCE,
    0x5A, 0x97, 0x7D, 0xCC, 0x32, 0xA2, 0xBF, 0x3E,
    0x0A, 0x10, 0xF1, 0x88, 0x94, 0xCD, 0xEA, 0xB1,
    0xFE, 0x90, 0x1D, 0x81, 0x34, 0x1A, 0xE1, 0x79,
    0x1C, 0x59, 0x27, 0x5B, 0x4F, 0x6E, 0x8D, 0x9C,
    0xB5, 0x2E, 0xFB, 0x98, 0x65, 0x45, 0x7E, 0x7C,
    0x14, 0x21, 0xE3, 0x11, 0x29, 0x9B, 0xD5, 0x63,
    0xFD, 0x20, 0x3B, 0x02, 0x68, 0x35, 0xC2, 0xF2,
    0x38, 0xB2, 0x4E, 0xB6, 0x9E, 0xDD, 0x1B, 0x39,
    0x6A, 0x5D, 0xF7, 0x30, 0xCA, 0x8A, 0xFC, 0xF8,
    0x28, 0x43, 0xC6, 0x22, 0x53, 0x37, 0xAA, 0xC7,
    0xFA, 0x40, 0x76, 0x04, 0xD0, 0x6B, 0x85, 0xE4,
    0x71, 0x64, 0x9D, 0x6D, 0x3D, 0xBA, 0x36, 0x72,
    0xD4, 0xBB, 0xEE, 0x61, 0x95, 0x15, 0xF9, 0xF0,
    0x50, 0x87, 0x8C, 0x44, 0xA6, 0x6F, 0x55, 0x8F,
    0xF4, 0x80, 0xEC, 0x09, 0xA0, 0xD7, 0x0B, 0xC8,
    0xE2, 0xC9, 0x3A, 0xDA, 0x7B, 0x74, 0x6C, 0xE5,
    0xA9, 0x77, 0xDC, 0xC3, 0x2A, 0x2B, 0xF3, 0xE0,
    0xA1, 0x0F, 0x18, 0x89, 0x4C, 0xDE, 0xAB, 0x1F,
    0xE9, 0x01, 0xD8, 0x13, 0x41, 0xAE, 0x17, 0x91,
    0xC5, 0x92, 0x75, 0xB4, 0xF6, 0xE8, 0xD9, 0xCB,
    0x52, 0xEF, 0xB9, 0x86, 0x54, 0x57, 0xE7, 0xC1,
    0x42, 0x1E, 0x31, 0x12, 0x99, 0xBD, 0x56, 0x3F,
    0xD2, 0x03, 0xB0, 0x26, 0x83, 0x5C, 0x2F, 0x23,
    0x8B, 0x24, 0xEB, 0x69, 0xED, 0xD1, 0xB3, 0x96,
    0xA5, 0xDF, 0x73, 0x0C, 0xA8, 0xAF, 0xCF, 0x82,
    0x84, 0x3C, 0x62, 0x25, 0x33, 0x7A, 0xAC, 0x7F,
    0xA4, 0x07, 0x60, 0x4D, 0x06, 0xB8, 0x5E, 0x47,
    0x16, 0x49, 0xD6, 0xD3, 0xDB, 0xA3, 0x67, 0x2D,
    0x4B, 0xBE, 0xE6, 0x19, 0x51, 0x5F, 0x9F, 0x05,
    0x08, 0x78, 0xC4, 0x4A, 0x66, 0xF5, 0x58,
};
#define DECMODE_CCSDS_TABLE_N \
    (sizeof DECMODE_CCSDS_TABLE / sizeof DECMODE_CCSDS_TABLE[0])

// Cap on how many post-Golay bytes the descrambler stage previews.
// 255 = the AX100 / RS(255,223) codeword length — enough to scroll
// across the entire on-wire payload when RS is in play.
#define DECMODE_DESCR_CAP 255

// ---------------------------------------------------------------------------
// CLI flags + companion helpers
// ---------------------------------------------------------------------------

static double parse_ut_from_path(const char *path)
{
    if (path == NULL) return 0.0;
    const char *p = strstr(path, "UT=");
    if (p == NULL) return 0.0;
    p += 3;
    int yr, mo, d, h, mi, s, ms = 0;
    int got = sscanf(p, "%4d%2d%2dT%2d%2d%2d.%3d",
                     &yr, &mo, &d, &h, &mi, &s, &ms);
    if (got < 6) return 0.0;
    struct tm utc = {0};
    utc.tm_year = yr - 1900; utc.tm_mon = mo - 1; utc.tm_mday = d;
    utc.tm_hour = h; utc.tm_min = mi; utc.tm_sec = s;
    time_t t = timegm(&utc);
    if (t == (time_t)-1) return 0.0;
    return (double) t + ms / 1000.0;
}

static int read_wav_samp_rate(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) return -1;
    unsigned char hdr[44];
    size_t got = fread(hdr, 1, 44, f);
    fclose(f);
    if (got != 44) return -1;
    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0)
        return -1;
    unsigned rate = (unsigned) hdr[24]
                  | ((unsigned) hdr[25] <<  8)
                  | ((unsigned) hdr[26] << 16)
                  | ((unsigned) hdr[27] << 24);
    return (int) rate;
}

// ---------------------------------------------------------------------------
// Boxes (annotations).
// ---------------------------------------------------------------------------

#define LABEL_MAX_LEN 32
#define BOXES_MAX     1024

static const char *LABEL_PRESETS[] = {
    "beacon", "bulk_download", "ack", "rfi", "unknown",
};
static const int N_LABEL_PRESETS =
    (int)(sizeof LABEL_PRESETS / sizeof LABEL_PRESETS[0]);

typedef struct {
    double t0_s, t1_s;
    double f_lo_hz, f_hi_hz;
    char   label[LABEL_MAX_LEN];
} box_t;

typedef struct {
    box_t  items[BOXES_MAX];
    int    n;
    int    selected;
} box_list_t;

static void box_normalize(box_t *b)
{
    if (b->t0_s > b->t1_s) {
        double t = b->t0_s; b->t0_s = b->t1_s; b->t1_s = t;
    }
    if (b->f_lo_hz > b->f_hi_hz) {
        double t = b->f_lo_hz; b->f_lo_hz = b->f_hi_hz; b->f_hi_hz = t;
    }
}

static int box_cmp_t0(const void *pa, const void *pb)
{
    const box_t *a = (const box_t *) pa;
    const box_t *b = (const box_t *) pb;
    if (a->t0_s < b->t0_s) return -1;
    if (a->t0_s > b->t0_s) return  1;
    // Stable secondary key on t1 so ties don't reorder on every sort.
    if (a->t1_s < b->t1_s) return -1;
    if (a->t1_s > b->t1_s) return  1;
    return 0;
}

// Sort boxes by t0_s ascending so they are numbered 1..n chronologically
// across the pass. Preserves the identity of the currently-selected box
// (by content-match) so the user's selection follows the data, not the
// old index.
static void boxes_sort(box_list_t *bl)
{
    if (bl->n <= 1) return;
    box_t sel_copy = {0};
    int   has_sel  = (bl->selected >= 0 && bl->selected < bl->n);
    if (has_sel) sel_copy = bl->items[bl->selected];
    qsort(bl->items, (size_t) bl->n, sizeof bl->items[0], box_cmp_t0);
    if (has_sel) {
        bl->selected = -1;
        for (int i = 0; i < bl->n; ++i) {
            const box_t *x = &bl->items[i];
            if (x->t0_s == sel_copy.t0_s
             && x->t1_s == sel_copy.t1_s
             && x->f_lo_hz == sel_copy.f_lo_hz
             && x->f_hi_hz == sel_copy.f_hi_hz
             && strcmp(x->label, sel_copy.label) == 0) {
                bl->selected = i;
                break;
            }
        }
    }
}

static int boxes_save(const box_list_t *bl, const char *iq_path)
{
    char path[1100];
    snprintf(path, sizeof path, "%.1000s.boxes.csv", iq_path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        fprintf(stderr, "decode_inspector: save failed: %s: %s\n",
                path, strerror(errno));
        return -1;
    }
    fprintf(f, "# t_start_s,t_end_s,f_lo_hz,f_hi_hz,label\n");
    for (int i = 0; i < bl->n; ++i) {
        const box_t *b = &bl->items[i];
        fprintf(f, "%.6f,%.6f,%.1f,%.1f,%s\n",
                b->t0_s, b->t1_s, b->f_lo_hz, b->f_hi_hz, b->label);
    }
    fclose(f);

    char anc_path[1100];
    snprintf(anc_path, sizeof anc_path, "%.1000s.box_anchors.csv", iq_path);
    f = fopen(anc_path, "w");
    if (f == NULL) {
        fprintf(stderr, "decode_inspector: save failed: %s: %s\n",
                anc_path, strerror(errno));
        return -1;
    }
    double ut_start = parse_ut_from_path(iq_path);
    long long ut_start_ms = (long long)(ut_start * 1000.0 + 0.5);
    fprintf(f, "# event,unix_time_ms,bright_bins,peak_excess_db,duration_ms,freq_hz\n");
    for (int i = 0; i < bl->n; ++i) {
        const box_t *b = &bl->items[i];
        double t_center = 0.5 * (b->t0_s + b->t1_s);
        double f_center = 0.5 * (b->f_lo_hz + b->f_hi_hz);
        double dur_ms   = (b->t1_s - b->t0_s) * 1000.0;
        long long u_ms  = ut_start_ms + (long long)(t_center * 1000.0 + 0.5);
        fprintf(f, "burst_start,%lld,32,30.0,0,%.0f\n", u_ms, f_center);
        fprintf(f, "burst_end,%lld,32,30.0,%.0f,%.0f\n",
                u_ms + (long long) dur_ms, dur_ms, f_center);
    }
    fclose(f);
    fprintf(stderr,
        "decode_inspector: saved %d box(es) to %s\n             anchors  to %s\n",
        bl->n, path, anc_path);
    return 0;
}

static int boxes_load(box_list_t *bl, const char *iq_path)
{
    char path[1100];
    snprintf(path, sizeof path, "%.1000s.boxes.csv", iq_path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "decode_inspector: load: %s: %s\n", path, strerror(errno));
        return -1;
    }
    bl->n = 0;
    bl->selected = -1;
    char line[512];
    while (fgets(line, sizeof line, f) != NULL && bl->n < BOXES_MAX) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0')
            continue;
        box_t b = {0};
        char lbl[LABEL_MAX_LEN] = {0};
        int got = sscanf(line, "%lf,%lf,%lf,%lf,%31s",
                         &b.t0_s, &b.t1_s, &b.f_lo_hz, &b.f_hi_hz, lbl);
        if (got < 4) continue;
        snprintf(b.label, sizeof b.label, "%s",
                 (got >= 5 && lbl[0]) ? lbl : "beacon");
        bl->items[bl->n++] = b;
    }
    fclose(f);
    // Older CSVs may have been written in insertion order. Sort so the
    // in-memory list is chronological from frame 1.
    if (bl->n > 1) {
        qsort(bl->items, (size_t) bl->n, sizeof bl->items[0], box_cmp_t0);
    }
    fprintf(stderr, "decode_inspector: loaded %d box(es) from %s\n",
            bl->n, path);
    return 0;
}

// ---------------------------------------------------------------------------
// Operator-flag passthru — same names gen_waterfall accepts; decode_inspector
// now consumes them directly into a wf_opts_t and runs the spectrogram
// in-process via waterfall_core.
// ---------------------------------------------------------------------------

#define MAX_PASSTHRU 32
static const char *PASSTHRU_FLAGS[] = {
    "--full-width", "--dc-notch", "--no-dc-notch", "--elapsed-time",
    NULL
};
static const char *PASSTHRU_PREFIXES[] = {
    "--fft-time-bin-s=", "--zoom-khz=", "--detrend=", "--detrend-tau-s=",
    "--center-hz=", "--dc-notch-bins=", "--db-min=", "--db-max=",
    "--power-offset=", "--start-utc=", "--fft=", "--hop=",
    "--marks-csv=", "--show-tm=",
    NULL
};

static int is_passthru(const char *arg)
{
    for (int i = 0; PASSTHRU_FLAGS[i]; ++i)
        if (strcmp(arg, PASSTHRU_FLAGS[i]) == 0) return 1;
    for (int i = 0; PASSTHRU_PREFIXES[i]; ++i) {
        size_t L = strlen(PASSTHRU_PREFIXES[i]);
        if (strncmp(arg, PASSTHRU_PREFIXES[i], L) == 0) return 1;
    }
    return 0;
}

// Translate the passthru flag list into a wf_opts_t. opt->out_rows is
// derived from --fft-time-bin-s (= seconds of IQ per output row) and
// the capture duration the caller passes in. Default time bin 0.5 s.
static int parse_wf_opts_from_passthru(const char **passthru, int n_passthru,
                                       int samp_rate, double duration_s,
                                       wf_opts_t *opt)
{
    memset(opt, 0, sizeof *opt);
    opt->fft_size       = 1024;
    opt->hop            = 256;
    double time_bin_s   = 0.5;
    opt->db_min         = -3.0f;
    opt->db_max         = 20.0f;
    opt->detrend_mode   = 0;
    opt->detrend_tau_s  = 0.0;
    opt->sample_rate    = samp_rate;
    opt->center_hz      = 0.0;
    opt->zoom_hz        = 30000.0;
    opt->dc_notch       = 0;
    opt->dc_notch_bins  = 2;
    snprintf(opt->power_unit, sizeof opt->power_unit, "dBFS");

    for (int i = 0; i < n_passthru; ++i) {
        const char *a = passthru[i];
        if (strncmp(a, "--fft=", 6) == 0) {
            opt->fft_size = atoi(a + 6);
            if (opt->hop > opt->fft_size) opt->hop = opt->fft_size / 4;
        } else if (strncmp(a, "--fft-time-bin-s=", 17) == 0) {
            time_bin_s = atof(a + 17);
            if (time_bin_s <= 0.0) time_bin_s = 0.5;
        } else if (strncmp(a, "--hop=", 6) == 0) {
            opt->hop = atoi(a + 6);
        } else if (strncmp(a, "--db-min=", 9) == 0) {
            opt->db_min = (float) atof(a + 9);
            opt->db_min_user_set = 1;
        } else if (strncmp(a, "--db-max=", 9) == 0) {
            opt->db_max = (float) atof(a + 9);
            opt->db_max_user_set = 1;
        } else if (strncmp(a, "--detrend=", 10) == 0) {
            const char *m = a + 10;
            if      (strcmp(m, "median") == 0) opt->detrend_mode = 0;
            else if (strcmp(m, "hpf")    == 0) opt->detrend_mode = 1;
            else if (strcmp(m, "none")   == 0) opt->detrend_mode = 2;
        } else if (strncmp(a, "--detrend-tau-s=", 16) == 0) {
            opt->detrend_tau_s = atof(a + 16);
        } else if (strncmp(a, "--center-hz=", 12) == 0) {
            opt->center_hz = atof(a + 12);
        } else if (strncmp(a, "--zoom-khz=", 11) == 0) {
            opt->zoom_hz = atof(a + 11) * 1000.0;
        } else if (strcmp(a, "--full-width") == 0) {
            opt->zoom_hz = 0.0;
        } else if (strcmp(a, "--dc-notch") == 0) {
            opt->dc_notch = 1;
        } else if (strcmp(a, "--no-dc-notch") == 0) {
            opt->dc_notch = 0;
        } else if (strncmp(a, "--dc-notch-bins=", 16) == 0) {
            opt->dc_notch_bins = atoi(a + 16);
        } else if (strncmp(a, "--power-offset=", 15) == 0) {
            opt->power_offset_db = (float) atof(a + 15);
            snprintf(opt->power_unit, sizeof opt->power_unit, "dBm");
        } else if (strncmp(a, "--marks-csv=", 12) == 0) {
            opt->marks_csv_path = a + 12;
        } else if (strncmp(a, "--show-tm=", 10) == 0) {
            opt->show_tm_csv_path = a + 10;
        } else if (strncmp(a, "--start-utc=", 12) == 0) {
            int yr, mo, d, h, mi, s, ms = 0;
            int got = sscanf(a + 12, "%4d%2d%2dT%2d%2d%2d.%3d",
                             &yr, &mo, &d, &h, &mi, &s, &ms);
            if (got >= 6) {
                struct tm utc = {0};
                utc.tm_year = yr - 1900; utc.tm_mon = mo - 1;
                utc.tm_mday = d; utc.tm_hour = h;
                utc.tm_min = mi; utc.tm_sec = s;
                time_t t = timegm(&utc);
                if (t != (time_t)-1) {
                    opt->start_utc = t;
                    opt->start_utc_subsec = ms / 1000.0;
                }
            }
        }
        // --elapsed-time is decode_inspector's render_opts concern, not wf_opts_t's.
    }
    // Resolve out_rows from the duration + time-bin hint. Clamp to a
    // sane range so a 14-second capture with time-bin=0.5 doesn't end
    // up with 28 rows (unreadable), and a huge multi-hour capture at
    // time-bin=0.01 doesn't try to allocate gigabyte-scale buffers.
    if (duration_s > 0.0 && time_bin_s > 0.0) {
        double rows = duration_s / time_bin_s;
        if (rows < 64.0)     rows = 64.0;
        if (rows > 200000.0) rows = 200000.0;
        opt->out_rows = (int)(rows + 0.5);
    } else {
        opt->out_rows = 1080;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Parse --zoom-khz / --center-hz / --elapsed-time / --start-utc from the
// passthru flags so we can compute the freq/time mapping locally.
// ---------------------------------------------------------------------------

typedef struct {
    double zoom_khz;     // 0 = full bandwidth
    double center_hz;    // RF tuning offset for label math (UI doesn't use it)
    int    elapsed_time; // 1 = render uses elapsed seconds rather than wall clock
    int    have_start_utc;
    double start_utc_s;  // user-overridden UT (seconds)
} render_opts_t;

static void parse_render_opts(const char **passthru, int n,
                              const char *iq_path, render_opts_t *r)
{
    r->zoom_khz = 0.0;
    r->center_hz = 0.0;
    r->elapsed_time = 0;
    r->have_start_utc = 0;
    r->start_utc_s = parse_ut_from_path(iq_path);
    if (r->start_utc_s > 0.0) r->have_start_utc = 1;
    for (int i = 0; i < n; ++i) {
        const char *a = passthru[i];
        if (strncmp(a, "--zoom-khz=", 11) == 0) {
            r->zoom_khz = atof(a + 11);
        } else if (strcmp(a, "--full-width") == 0) {
            r->zoom_khz = 0.0;
        } else if (strncmp(a, "--center-hz=", 12) == 0) {
            r->center_hz = atof(a + 12);
        } else if (strcmp(a, "--elapsed-time") == 0) {
            r->elapsed_time = 1;
        } else if (strncmp(a, "--start-utc=", 12) == 0) {
            int yr, mo, d, h, mi, s, ms = 0;
            int got = sscanf(a + 12, "%4d%2d%2dT%2d%2d%2d.%3d",
                             &yr, &mo, &d, &h, &mi, &s, &ms);
            if (got >= 6) {
                struct tm utc = {0};
                utc.tm_year = yr - 1900; utc.tm_mon = mo - 1;
                utc.tm_mday = d; utc.tm_hour = h;
                utc.tm_min = mi; utc.tm_sec = s;
                time_t t = timegm(&utc);
                if (t != (time_t)-1) {
                    r->start_utc_s = (double) t + ms / 1000.0;
                    r->have_start_utc = 1;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// CLI usage
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// TTF font loader (pattern lifted from ~/src/ved/ui.c). Looks for the
// bundled SourceCodePro-Regular.ttf next to the binary, in the project
// assets/ dir, or in $XDG_DATA_HOME/simple_sat_ops/. Falls back to
// raylib's bitmap default if none of those exist.
// ---------------------------------------------------------------------------

static Font g_ui_font;
static int  g_ui_font_loaded = 0;
static float g_ui_font_spacing = 1.0f;

static int load_ttf_from_known_paths(void)
{
    const char *home = getenv("HOME");
    char candidates[6][1024];
    int n = 0;
    const char *xdg = getenv("XDG_DATA_HOME");
    if (xdg && xdg[0]) {
        snprintf(candidates[n++], sizeof candidates[0],
                 "%s/simple_sat_ops/SourceCodePro-Regular.ttf", xdg);
    }
    if (home && home[0]) {
        snprintf(candidates[n++], sizeof candidates[0],
                 "%s/.local/share/simple_sat_ops/"
                 "SourceCodePro-Regular.ttf", home);
        snprintf(candidates[n++], sizeof candidates[0],
                 "%s/src/simple_sat_ops/assets/"
                 "SourceCodePro-Regular.ttf", home);
    }
    snprintf(candidates[n++], sizeof candidates[0],
             "assets/SourceCodePro-Regular.ttf");
    snprintf(candidates[n++], sizeof candidates[0],
             "../assets/SourceCodePro-Regular.ttf");
    // Build the codepoint list: printable ASCII (0x20..0x7E) plus the
    // handful of Latin-1 / math symbols the UI actually uses. LoadFontEx
    // with codepoints=NULL/count=0 falls back to ASCII-only, which is
    // why µ etc. were rendering as the "?" glyph.
    int codepoints[128];
    int n_cp = 0;
    for (int c = 0x20; c <= 0x7E; ++c) codepoints[n_cp++] = c;
    codepoints[n_cp++] = 0x00B5; // µ (micro sign)
    codepoints[n_cp++] = 0x00B0; // ° (degree)
    codepoints[n_cp++] = 0x00B1; // ± (plus-minus)
    codepoints[n_cp++] = 0x00B2; // ² (superscript two)
    codepoints[n_cp++] = 0x00B3; // ³ (superscript three)
    codepoints[n_cp++] = 0x2013; // – (en dash)
    codepoints[n_cp++] = 0x2014; // — (em dash)
    codepoints[n_cp++] = 0x03C0; // π (lowercase pi)
    codepoints[n_cp++] = 0x0394; // Δ (uppercase delta)
    codepoints[n_cp++] = 0x2264; // ≤ (less-than-or-equal)

    for (int i = 0; i < n; ++i) {
        if (FileExists(candidates[i])) {
            g_ui_font = LoadFontEx(candidates[i], 48,
                                   codepoints, n_cp);
            if (g_ui_font.texture.id != 0) {
                SetTextureFilter(g_ui_font.texture,
                                 TEXTURE_FILTER_BILINEAR);
                fprintf(stderr,
                    "decode_inspector: loaded font %s\n", candidates[i]);
                return 1;
            }
        }
    }
    fprintf(stderr,
        "decode_inspector: TTF font not found; using raylib default\n");
    return 0;
}

static void draw_text(const char *s, int x, int y, int size, Color c)
{
    if (g_ui_font_loaded) {
        DrawTextEx(g_ui_font, s, (Vector2){ (float)x, (float)y },
                   (float)size, g_ui_font_spacing, c);
    } else {
        DrawText(s, x, y, size, c);
    }
}

static int measure_text(const char *s, int size)
{
    if (g_ui_font_loaded) {
        return (int) MeasureTextEx(g_ui_font, s,
                                   (float)size, g_ui_font_spacing).x;
    }
    return MeasureText(s, size);
}

// Draw the 2-row ASM bit-comparison strip at `y_top`:
//   row 0 (expected): 16 preamble cells (gray) + 32 ASM cells (white)
//   row 1 (actual):   green where bits match expected, red where not
// Cell x positions are computed from each bit's *absolute time* so
// they survive pan/zoom without waiting for the next debounced
// recompute. Callers pass the captured ASM absolute time + bit
// period so the strip can land on the same physical bits regardless
// of which K-panel stage is showing.
static void draw_asm_bit_strip(
    int body_x0, int body_x1, int body_w,
    double wf_t_lo, double span_s,
    int y_top,
    const uint8_t *bits, size_t n_bits_total,
    size_t asm_offset,
    double asm_abs_time_s,
    double bit_period_s,
    int text_pt)
{
    if (bits == NULL || asm_abs_time_s < 0.0
        || bit_period_s <= 0.0 || span_s <= 0.0) return;
    const int n_pre = 16;
    const int n_asm = 32;
    const uint32_t ASM_EXPECTED = 0x930B51DEu;
    const int cell_h = 14;
    const int gap_y  = 2;
    int exp_y = y_top;
    int act_y = exp_y + cell_h + gap_y;
    Color exp_pre_bg = {110, 110, 120, 255};
    Color exp_asm_bg = {235, 235, 240, 255};
    Color edge       = {30,  30,  40,  255};
    Color match_bg   = {70,  190, 110, 255};
    Color miss_bg    = {220, 90,  100, 255};
    // Default cell width = one bit period's worth of screen pixels,
    // clamped so very wide zoom doesn't turn the strip into pixel-
    // wide bars and very high zoom doesn't fill the panel.
    int cell_w = (int)(bit_period_s / span_s * (double) body_w);
    if (cell_w < 4)  cell_w = 4;
    if (cell_w > 22) cell_w = 22;

    for (int i = 0; i < n_pre + n_asm; ++i) {
        int is_asm = (i >= n_pre);
        int bit_offset = i - n_pre;  // -16..+31
        int bit_idx = (int) asm_offset + bit_offset;
        if (bit_idx < 0 || bit_idx >= (int) n_bits_total) continue;
        double t_abs = asm_abs_time_s + (double) bit_offset * bit_period_s;
        int xc = body_x0 + (int)((t_abs - wf_t_lo) / span_s * (double) body_w);
        if (xc < body_x0 - 2 || xc > body_x1 + 2) continue;
        int xl = xc - cell_w / 2;

        int expected;
        if (is_asm) {
            expected = (ASM_EXPECTED >> (31 - (i - n_pre))) & 1u;
        } else {
            // asm_offset is byte-aligned; the last preamble byte is
            // 0xAA = 10101010 sent MSB-first, so distance-1 back is
            // bit 7 (=0), distance-2 is bit 6 (=1), etc.
            int dist_back = -bit_offset;  // 1..16
            expected = (dist_back & 1) ? 0 : 1;
        }
        uint8_t actual = bits[bit_idx] & 1u;

        Color exp_bg = is_asm ? exp_asm_bg : exp_pre_bg;
        Color exp_tx = is_asm ? (Color){20, 20, 30, 255}
                              : (Color){225, 225, 235, 255};
        DrawRectangle(xl, exp_y, cell_w, cell_h, exp_bg);
        DrawRectangleLines(xl, exp_y, cell_w, cell_h, edge);
        Color act_bg = (actual == expected) ? match_bg : miss_bg;
        DrawRectangle(xl, act_y, cell_w, cell_h, act_bg);
        DrawRectangleLines(xl, act_y, cell_w, cell_h, edge);
        if (cell_w >= 9 && text_pt > 0) {
            char b[4];
            snprintf(b, sizeof b, "%d", expected);
            int bw = measure_text(b, text_pt);
            draw_text(b, xc - bw / 2,
                      exp_y + (cell_h - text_pt) / 2,
                      text_pt, exp_tx);
            snprintf(b, sizeof b, "%d", actual);
            bw = measure_text(b, text_pt);
            draw_text(b, xc - bw / 2,
                      act_y + (cell_h - text_pt) / 2,
                      text_pt, (Color){20, 20, 30, 255});
        }
    }
}

// Pick how many decimals to show on a seconds label given the
// tick step (or the resolution of interest). Range: 0 (whole-second
// ticks) up to 7 (~100 ns precision — well below the 96 kSPS sample
// period at extreme zoom).
static int decimals_for_step(double step_s)
{
    if (!isfinite(step_s) || step_s <= 0.0) return 3;
    if (step_s >= 0.5)        return 0;
    if (step_s >= 0.05)       return 1;
    if (step_s >= 0.005)      return 2;
    if (step_s >= 0.0005)     return 3;
    if (step_s >= 0.00005)    return 4;
    if (step_s >= 0.000005)   return 5;
    if (step_s >= 0.0000005)  return 6;
    return 7;
}

// Format seconds as mm:ss.<n decimals>.
static void fmt_mmss_ndec(double t_s, int nd, char *buf, size_t cap)
{
    if (!isfinite(t_s)) {
        snprintf(buf, cap, "--:--%s%.*s",
                 nd > 0 ? "." : "", nd, "-------");
        return;
    }
    if (nd < 0) nd = 0;
    if (nd > 7) nd = 7;
    int sign = (t_s < 0.0) ? -1 : +1;
    double abs_s = fabs(t_s);
    int mm = (int)(abs_s / 60.0);
    double rem = abs_s - mm * 60.0;
    // Width: 2 digits + '.' + nd decimals when nd > 0, else 2.
    int width = (nd > 0) ? (3 + nd) : 2;
    snprintf(buf, cap, "%s%02d:%0*.*f",
             sign < 0 ? "-" : "", mm, width, nd, rem);
}

// Backwards-compatible mm:ss.sss (3 decimals) wrapper used where the
// caller doesn't know an appropriate step size — e.g. the box-info
// status line, where the box duration drives the precision (computed
// below at the call site).
static void fmt_mmss_ms(double t_s, char *buf, size_t cap)
{
    fmt_mmss_ndec(t_s, 3, buf, cap);
}

// HH-MM-SS.sss for filenames — colons are visually odd in Finder /
// can be problematic on FAT, so the hh/mm/ss separators are hyphens.
static void fmt_hhmmss_filename(double t_s, char *buf, size_t cap)
{
    if (!isfinite(t_s) || t_s < 0.0) t_s = 0.0;
    int hh = (int)(t_s / 3600.0);
    double rem = t_s - hh * 3600.0;
    int mm = (int)(rem / 60.0);
    double ss = rem - mm * 60.0;
    snprintf(buf, cap, "%02d-%02d-%06.3f", hh, mm, ss);
}

// ---------------------------------------------------------------------------
// IQ-sample buffer. The whole .iq file is mmap'd or read into memory
// once so the waveform panel can slice into it for any selected box
// without re-reading the file.
// ---------------------------------------------------------------------------

typedef struct {
    int16_t *samples;     // interleaved I,Q (2 int16 per pair)
    size_t   n_pairs;
    int      samp_rate;
} iq_buf_t;

// Reads a raw int16 I/Q file into b->samples. When `progress_pct_out`
// is non-NULL, the function writes a 0..1 fraction to it after each
// chunk so a UI thread can render a progress bar — the file is read in
// 8 MB chunks rather than one big fread() to make the granularity
// useful on multi-hundred-MB passes.
static int iq_buf_load_progress(iq_buf_t *b, const char *path, int samp_rate,
                                volatile float *progress_pct_out)
{
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "decode_inspector: open %s: %s\n",
                path, strerror(errno));
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (sz <= 0) { fclose(fp); return -1; }
    b->samples = (int16_t *) malloc((size_t) sz);
    if (b->samples == NULL) { fclose(fp); return -1; }

    const size_t chunk = 8u * 1024u * 1024u;
    size_t total      = (size_t) sz;
    size_t read_total = 0;
    char  *dst        = (char *) b->samples;
    while (read_total < total) {
        size_t want = chunk;
        if (read_total + want > total) want = total - read_total;
        size_t got = fread(dst + read_total, 1, want, fp);
        if (got == 0) {
            fclose(fp);
            free(b->samples); b->samples = NULL;
            return -1;
        }
        read_total += got;
        if (progress_pct_out != NULL) {
            *progress_pct_out = (float)((double) read_total / (double) total);
        }
    }
    fclose(fp);
    b->n_pairs   = total / 4;
    b->samp_rate = samp_rate;
    if (progress_pct_out != NULL) *progress_pct_out = 1.0f;
    return 0;
}

static int iq_buf_load(iq_buf_t *b, const char *path, int samp_rate)
{
    return iq_buf_load_progress(b, path, samp_rate, NULL);
}

// Pretty-print a duration in seconds using whichever unit (minutes,
// s, ms, µs, ns) lands the leading digit between 1 and 60. Three
// decimals of precision so a 1234 µs span still reads as "1234.000 µs"
// rather than "1.234 ms" — the operator picks the breakpoint when they
// drag.
static int fmt_duration_auto(double dur_s, char *out, size_t cap)
{
    double abs_d = fabs(dur_s);
    if (abs_d >= 60.0) {
        return snprintf(out, cap, "%.3f min", dur_s / 60.0);
    } else if (abs_d >= 1.0) {
        return snprintf(out, cap, "%.3f s", dur_s);
    } else if (abs_d >= 1.0e-3) {
        return snprintf(out, cap, "%.3f ms", dur_s * 1.0e3);
    } else if (abs_d >= 1.0e-6) {
        return snprintf(out, cap, "%.3f µs", dur_s * 1.0e6);
    } else {
        return snprintf(out, cap, "%.3f ns", dur_s * 1.0e9);
    }
}

// Format an unsigned 64-bit value with thousand separators (US-style
// commas — locale-independent so it doesn't depend on a setlocale()
// call at startup).  Returns the number of bytes written (excluding
// the terminator), or -1 if the buffer was too small.
static int fmt_thousands(char *out, size_t cap, uint64_t n)
{
    if (out == NULL || cap == 0) return -1;
    char tmp[32];
    int len = snprintf(tmp, sizeof tmp, "%llu", (unsigned long long) n);
    if (len < 0) return -1;
    int first_group = ((len - 1) % 3) + 1;
    int needed = len + (len - first_group) / 3;
    if ((size_t) needed + 1 > cap) return -1;
    int oi = 0;
    for (int i = 0; i < first_group; ++i) out[oi++] = tmp[i];
    for (int i = first_group; i < len; i += 3) {
        out[oi++] = ',';
        out[oi++] = tmp[i];
        out[oi++] = tmp[i + 1];
        out[oi++] = tmp[i + 2];
    }
    out[oi] = '\0';
    return oi;
}

static void iq_buf_free(iq_buf_t *b)
{
    free(b->samples); b->samples = NULL; b->n_pairs = 0;
}

// ---------------------------------------------------------------------------
// Background loader: IQ-file read + wf_compute on a worker thread.
// Lets InitWindow happen on the main thread first so the operator sees a
// "Loading..." / "Computing spectrogram..." splash instead of a dead
// terminal for the several seconds the FFT can take on a 200 MB pass.
// All GPU-touching work (texture upload, shader compile) stays on the
// main thread after pthread_join, because the OpenGL context belongs to
// it (macOS in particular pukes if you try to use GL from another
// thread).
// ---------------------------------------------------------------------------

typedef struct {
    const char     *iq_path;
    int             samp_rate;
    double          lo_shift_hz;    // sw NCO shift applied in-place to
                                    // iqb.samples between load and FFT;
                                    // 0 = pass-through
    wf_opts_t      *opt;
    // Outputs (set by the worker; readable by main once `done` is non-zero).
    iq_buf_t        iqb;
    float          *spec_db;
    int             spec_w;
    int             spec_h;
    // Progress + status. status_msg is under `lock` (variable-length).
    // phase + phase_progress are read lock-free from the splash — a
    // torn read just shows last-frame's bar position, which is fine.
    char            status_msg[128];
    int             phase;          // 0 = loading IQ, 1 = computing FFT
    volatile float  phase_progress; // 0..1, current phase only
    int             error;
    int             done;
    pthread_mutex_t lock;
} loader_ctx_t;

static void loader_set_status(loader_ctx_t *ctx, int phase, const char *fmt, ...)
{
    pthread_mutex_lock(&ctx->lock);
    ctx->phase = phase;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(ctx->status_msg, sizeof ctx->status_msg, fmt, ap);
    va_end(ap);
    pthread_mutex_unlock(&ctx->lock);
}

static void *loader_thread_fn(void *arg)
{
    loader_ctx_t *ctx = (loader_ctx_t *) arg;
    loader_set_status(ctx, 0, "Loading IQ samples...");
    ctx->phase_progress = 0.0f;
    if (iq_buf_load_progress(&ctx->iqb, ctx->iq_path, ctx->samp_rate,
                             &ctx->phase_progress) != 0) {
        pthread_mutex_lock(&ctx->lock);
        ctx->error = 1;
        snprintf(ctx->status_msg, sizeof ctx->status_msg,
                 "Failed to load %s", ctx->iq_path);
        ctx->done = 1;
        pthread_mutex_unlock(&ctx->lock);
        return NULL;
    }
    // Optional NCO-shift the loaded IQ so an off-DC carrier (e.g. an
    // old RAO capture recorded before carrier-trim-hz was populated)
    // lands at baseband. Spectrogram + K-panel decode both see the
    // shifted buffer.
    if (ctx->lo_shift_hz != 0.0 && ctx->iqb.samples != NULL
        && ctx->iqb.n_pairs > 0) {
        char pb[32];
        fmt_thousands(pb, sizeof pb, (uint64_t) ctx->iqb.n_pairs);
        loader_set_status(ctx, 0,
            "NCO-shifting IQ by %.3f kHz (%s pairs)...",
            ctx->lo_shift_hz / 1000.0, pb);
        sw_nco_t nco;
        sw_nco_init(&nco, (double) ctx->samp_rate);
        sw_nco_set_freq(&nco, ctx->lo_shift_hz);
        sw_nco_apply(&nco, ctx->iqb.samples, ctx->iqb.n_pairs);
    }

    char pair_buf[32];
    fmt_thousands(pair_buf, sizeof pair_buf,
                  (uint64_t) ctx->iqb.n_pairs);
    loader_set_status(ctx, 1, "Computing spectrogram (%s pairs)...",
                      pair_buf);
    ctx->phase_progress = 0.0f;
    // Hand the same progress field to wf_compute so the bar keeps
    // advancing through the FFT loop.
    ctx->opt->progress_pct_out = &ctx->phase_progress;
    if (wf_compute(ctx->iqb.samples, ctx->iqb.n_pairs,
                   ctx->opt, &ctx->spec_db,
                   &ctx->spec_w, &ctx->spec_h) != 0) {
        ctx->opt->progress_pct_out = NULL;
        pthread_mutex_lock(&ctx->lock);
        ctx->error = 1;
        snprintf(ctx->status_msg, sizeof ctx->status_msg,
                 "wf_compute failed");
        ctx->done = 1;
        pthread_mutex_unlock(&ctx->lock);
        return NULL;
    }
    ctx->opt->progress_pct_out = NULL;
    pthread_mutex_lock(&ctx->lock);
    ctx->done = 1;
    pthread_mutex_unlock(&ctx->lock);
    return NULL;
}

// ---------------------------------------------------------------------------
// Band-pass filter view: shift center_hz to DC, real LPF with bw_hz/2
// cutoff, leave the result at DC. Used by the waveform panel's 'F'
// toggle to view an IQ stream with the beacon's band kept and other
// energy (e.g. broadband 50 Hz line-coupled spikes) reduced. Runs on
// a worker thread so the UI stays responsive; output goes into a
// caller-allocated int16 buffer matched to the input length.
// ---------------------------------------------------------------------------

typedef struct {
    const int16_t *iq_in;
    size_t         n_pairs;
    int            samp_rate;
    double         center_hz;       // shift target for the LPF stage
    double         lower_hz;        // HPF cutoff (original frame); 0 = disabled
    double         upper_hz;        // LPF cutoff (shifted frame)
    int16_t       *iq_out;          // pre-allocated, n_pairs * 2 int16s
    volatile float progress_pct;    // 0..1, updated as the worker runs
    volatile int   done;
    int            error;
} filter_ctx_t;

// Build a length-N (odd) Hamming-windowed sinc low-pass with the given
// normalised cutoff (in units of fs). Returns a freshly-malloc'd array
// of N floats; caller frees.
static float *build_lpf_taps(int N, double cutoff_norm)
{
    if (N < 3 || (N & 1) == 0) return NULL;
    float *h = (float *) malloc((size_t) N * sizeof(float));
    if (h == NULL) return NULL;
    int M = N - 1;
    double sum = 0.0;
    for (int n = 0; n <= M; ++n) {
        double t = (double) n - (double) M / 2.0;
        double v = (fabs(t) < 1e-9)
            ? 2.0 * cutoff_norm
            : sin(2.0 * M_PI * cutoff_norm * t) / (M_PI * t);
        v *= 0.54 - 0.46 * cos(2.0 * M_PI * (double) n / (double) M);
        h[n] = (float) v;
        sum += v;
    }
    if (sum > 0.0) {
        for (int n = 0; n < N; ++n) h[n] /= (float) sum;
    }
    return h;
}

// Run a symmetric Hamming-windowed sinc FIR over an interleaved
// complex (I,Q) input. `taps[N]` is the real-valued kernel; the same
// kernel is applied to I and Q independently. If `mode_hpf` is set,
// the output is (delayed_input - LPF), giving a high-pass response
// with the same transition band. Output is group-delay-compensated:
// the last (N-1)/2 samples of the input are not written to. Reads
// from `in_iq` (complex floats) and writes to `out_iq` (complex
// floats); both are interleaved I,Q.
static void apply_fir_complex(const float *in_iq, size_t n_pairs,
                              const float *taps, int N, int mode_hpf,
                              float *out_iq,
                              volatile float *progress_pct_out,
                              float progress_lo, float progress_hi)
{
    int Mhalf = N / 2;
    float *dl_i = (float *) calloc((size_t) N, sizeof(float));
    float *dl_q = (float *) calloc((size_t) N, sizeof(float));
    if (dl_i == NULL || dl_q == NULL) {
        free(dl_i); free(dl_q);
        return;
    }
    int dl_pos = 0;
    size_t progress_step = (n_pairs >= 100) ? n_pairs / 100 : 1;
    for (size_t n = 0; n < n_pairs; ++n) {
        dl_i[dl_pos] = in_iq[n * 2 + 0];
        dl_q[dl_pos] = in_iq[n * 2 + 1];
        int newest = dl_pos;
        dl_pos = (dl_pos + 1) % N;

        double y_i = 0.0, y_q = 0.0;
        int idx = newest;
        for (int k = 0; k < N; ++k) {
            y_i += (double) taps[k] * dl_i[idx];
            y_q += (double) taps[k] * dl_q[idx];
            idx = (idx == 0) ? (N - 1) : (idx - 1);
        }

        ssize_t out_idx = (ssize_t) n - Mhalf;
        if (out_idx >= 0) {
            if (mode_hpf) {
                // High-pass: subtract the LPF from the delayed input.
                // dl[(newest - Mhalf) mod N] is the sample at out_idx.
                int o_idx = (newest - Mhalf + N) % N;
                out_iq[out_idx * 2 + 0] = dl_i[o_idx] - (float) y_i;
                out_iq[out_idx * 2 + 1] = dl_q[o_idx] - (float) y_q;
            } else {
                out_iq[out_idx * 2 + 0] = (float) y_i;
                out_iq[out_idx * 2 + 1] = (float) y_q;
            }
        }
        if (progress_pct_out != NULL && (n % progress_step) == 0) {
            float frac = (float)((double)(n + 1) / (double) n_pairs);
            *progress_pct_out = progress_lo
                + (progress_hi - progress_lo) * frac;
        }
    }
    // Zero the group-delay tail.
    for (size_t i = n_pairs - (size_t) Mhalf; i < n_pairs; ++i) {
        out_iq[i * 2 + 0] = 0.0f;
        out_iq[i * 2 + 1] = 0.0f;
    }
    free(dl_i); free(dl_q);
}

static void *filter_thread_fn(void *arg)
{
    filter_ctx_t *c = (filter_ctx_t *) arg;
    const int N_hpf = 257;
    const int N_lpf = 257;

    // Build LPF kernels for both stages. The HPF stage uses the same
    // sinc kernel internally; apply_fir_complex does the
    // (delayed_in - LPF) subtraction to turn it into a high-pass.
    float *h_hpf = NULL, *h_lpf = NULL;
    if (c->lower_hz > 0.0) {
        double cutoff_n = c->lower_hz / (double) c->samp_rate;
        if (cutoff_n <= 0.0 || cutoff_n >= 0.5) {
            c->error = 1; c->done = 1; return NULL;
        }
        h_hpf = build_lpf_taps(N_hpf, cutoff_n);
    }
    {
        double cutoff_n = c->upper_hz / (double) c->samp_rate;
        if (cutoff_n <= 0.0 || cutoff_n >= 0.5) {
            free(h_hpf);
            c->error = 1; c->done = 1; return NULL;
        }
        h_lpf = build_lpf_taps(N_lpf, cutoff_n);
    }
    if (h_lpf == NULL) {
        free(h_hpf);
        c->error = 1; c->done = 1; return NULL;
    }

    // Float intermediate buffer for the HPF stage (and the shift). One
    // float pair per IQ sample → 8 bytes × n_pairs. For a 9-minute
    // 96 kHz pass that's ~430 MB; comfortable on any modern Mac.
    float *iq_mid = (float *) malloc(c->n_pairs * 2 * sizeof(float));
    if (iq_mid == NULL) {
        free(h_hpf); free(h_lpf);
        c->error = 1; c->done = 1; return NULL;
    }

    // Stage 1: HPF in the ORIGINAL frame. With lower_hz == 0, the
    // stage is a no-op (just copy input → intermediate as floats).
    if (h_hpf != NULL) {
        // Copy input to a temporary float scratch so apply_fir_complex
        // can read uniform-format complex floats. Skip the copy by
        // adapting the inner loop; for now keep the code simple.
        float *scratch_in = (float *) malloc(c->n_pairs * 2 * sizeof(float));
        if (scratch_in == NULL) {
            free(iq_mid); free(h_hpf); free(h_lpf);
            c->error = 1; c->done = 1; return NULL;
        }
        for (size_t n = 0; n < c->n_pairs; ++n) {
            scratch_in[n * 2 + 0] = (float) c->iq_in[n * 2 + 0];
            scratch_in[n * 2 + 1] = (float) c->iq_in[n * 2 + 1];
        }
        apply_fir_complex(scratch_in, c->n_pairs,
                          h_hpf, N_hpf, /*mode_hpf=*/1,
                          iq_mid,
                          &c->progress_pct, 0.0f, 0.45f);
        free(scratch_in);
    } else {
        for (size_t n = 0; n < c->n_pairs; ++n) {
            iq_mid[n * 2 + 0] = (float) c->iq_in[n * 2 + 0];
            iq_mid[n * 2 + 1] = (float) c->iq_in[n * 2 + 1];
        }
        c->progress_pct = 0.45f;
    }

    // Stage 2: shift center_hz to DC, then LPF. Done in place into
    // iq_mid as we sweep through, with the result written into iq_out
    // (int16, clipped). The shift is a complex multiplication by
    // exp(-j*2π*center_hz*n/fs) — a recursive phasor update is cheap
    // and accurate enough for the few-million-sample range.
    double delta = -2.0 * M_PI * c->center_hz / (double) c->samp_rate;
    double cos_d = cos(delta), sin_d = sin(delta);
    double p_re = 1.0, p_im = 0.0;
    for (size_t n = 0; n < c->n_pairs; ++n) {
        double I = (double) iq_mid[n * 2 + 0];
        double Q = (double) iq_mid[n * 2 + 1];
        iq_mid[n * 2 + 0] = (float)(I * p_re - Q * p_im);
        iq_mid[n * 2 + 1] = (float)(I * p_im + Q * p_re);
        double np_re = p_re * cos_d - p_im * sin_d;
        double np_im = p_re * sin_d + p_im * cos_d;
        p_re = np_re; p_im = np_im;
    }
    // Final LPF stage. Write directly into iq_out via a small
    // wrapper that does the float → int16 clip after the convolution.
    float *iq_lpf_out = (float *) malloc(c->n_pairs * 2 * sizeof(float));
    if (iq_lpf_out == NULL) {
        free(iq_mid); free(h_hpf); free(h_lpf);
        c->error = 1; c->done = 1; return NULL;
    }
    apply_fir_complex(iq_mid, c->n_pairs,
                      h_lpf, N_lpf, /*mode_hpf=*/0,
                      iq_lpf_out,
                      &c->progress_pct, 0.50f, 0.99f);
    free(iq_mid);
    // Clip + write final output as int16.
    for (size_t n = 0; n < c->n_pairs; ++n) {
        int qi = (int) lrintf(iq_lpf_out[n * 2 + 0]);
        int qq = (int) lrintf(iq_lpf_out[n * 2 + 1]);
        if (qi >  32767) qi =  32767;
        if (qi < -32768) qi = -32768;
        if (qq >  32767) qq =  32767;
        if (qq < -32768) qq = -32768;
        c->iq_out[n * 2 + 0] = (int16_t) qi;
        c->iq_out[n * 2 + 1] = (int16_t) qq;
    }
    free(iq_lpf_out);

    free(h_hpf); free(h_lpf);
    c->progress_pct = 1.0f;
    c->done = 1;
    return NULL;
}

// Render the current waveform-panel view to a one-page PDF.
// iq_show_mode: 0=I+Q, 1=I only, 2=Q only, 3=phase only, 4=split.
static int pdf_write_waveform(const char *path,
                              const iq_buf_t *iqb,
                              double wf_t_lo, double wf_t_hi,
                              const char *iq_path,
                              const char *box_info,
                              int iq_show_mode)
{
    // Wider/shorter page than letter landscape — operator asked for
    // a less-tall plot. 11" × 6.5" in points = 792 × 468.
    const float page_w = 792.0f;
    const float page_h = 468.0f;
    pdfw_t *w = pdfw_begin(path, page_w, page_h);
    if (w == NULL) {
        fprintf(stderr, "decode_inspector: pdf open %s: %s\n",
                path, strerror(errno));
        return -1;
    }

    const float MARGIN = 36.0f;
    const float HEADER_H = 84.0f;
    const float FOOTER_H = 24.0f;
    pdfw_set_fill(w, (pdfw_rgb_t){250, 250, 252, 255});
    pdfw_rect_fill(w, MARGIN, MARGIN, page_w - 2*MARGIN, HEADER_H);
    pdfw_set_stroke(w, PDFW_DGREY);
    pdfw_lw(w, 0.7f);
    pdfw_rect_stroke(w, MARGIN, MARGIN, page_w - 2*MARGIN, HEADER_H);
    pdfw_set_fill(w, PDFW_BLACK);
    pdfw_text(w, MARGIN + 8, MARGIN + 6,
              "decode_inspector -- waveform export", 14.0f, 0);
    pdfw_text(w, MARGIN + 8, MARGIN + 26,
              iq_path ? iq_path : "", 9.0f, 1);

    char ts0[32], ts1[32];
    int hdr_nd = decimals_for_step((wf_t_hi - wf_t_lo) / 8.0);
    if (hdr_nd < 3) hdr_nd = 3;
    fmt_mmss_ndec(wf_t_lo, hdr_nd, ts0, sizeof ts0);
    fmt_mmss_ndec(wf_t_hi, hdr_nd, ts1, sizeof ts1);
    int64_t i_lo = (int64_t)(wf_t_lo * iqb->samp_rate);
    int64_t i_hi = (int64_t)(wf_t_hi * iqb->samp_rate);
    if (i_lo < 0) i_lo = 0;
    if (i_hi > (int64_t) iqb->n_pairs) i_hi = (int64_t) iqb->n_pairs;
    int64_t n_pairs_vis = i_hi - i_lo;
    char line3[256];
    snprintf(line3, sizeof line3,
        "window  %s -> %s   span %.*f s   %lld samples @ %d Hz",
        ts0, ts1, hdr_nd, wf_t_hi - wf_t_lo,
        (long long) n_pairs_vis, iqb->samp_rate);
    pdfw_text(w, MARGIN + 8, MARGIN + 42, line3, 9.0f, 1);
    if (box_info && box_info[0]) {
        pdfw_text(w, MARGIN + 8, MARGIN + 58, box_info, 9.0f, 1);
    }

    const float pL = MARGIN + 70.0f;
    const float pR = page_w - MARGIN - 8.0f;
    const float pT = MARGIN + HEADER_H + 20.0f;
    const float pB = page_h - MARGIN - FOOTER_H - 30.0f;
    const float plot_w = pR - pL;
    const float plot_h = pB - pT;
    pdfw_set_stroke(w, (pdfw_rgb_t){100, 100, 110, 255});
    pdfw_lw(w, 0.8f);
    pdfw_rect_stroke(w, pL, pT, plot_w, plot_h);

    // Mode flags + sub-rect layout (same logic as the on-screen panel).
    int show_iq    = (iq_show_mode != 3);
    int show_i     = show_iq && (iq_show_mode != 2);
    int show_q     = show_iq && (iq_show_mode != 1);
    int show_phase = (iq_show_mode == 3 || iq_show_mode == 4);
    int split      = (iq_show_mode == 4);
    float iq_y0_sub    = pT,  iq_y1_sub    = pB;
    float phase_y0_sub = pT,  phase_y1_sub = pB;
    if (split) {
        float gap = 8.0f;
        float mid = pT + plot_h * 0.5f;
        phase_y0_sub = pT;
        phase_y1_sub = mid - gap * 0.5f;
        iq_y0_sub    = mid + gap * 0.5f;
        iq_y1_sub    = pB;
    } else if (show_phase) {
        iq_y0_sub = iq_y1_sub = pT;
    } else {
        phase_y0_sub = phase_y1_sub = pT;
    }
    float iq_h_sub    = iq_y1_sub    - iq_y0_sub;
    float phase_h_sub = phase_y1_sub - phase_y0_sub;

    int amp_max = 1;
    if (show_iq && n_pairs_vis > 0) {
        int64_t step_s = (n_pairs_vis > 4096) ? n_pairs_vis / 2048 : 1;
        for (int64_t k = i_lo; k < i_hi; k += step_s) {
            int I = iqb->samples[k * 2 + 0];
            int Q = iqb->samples[k * 2 + 1];
            if (abs(I) > amp_max) amp_max = abs(I);
            if (abs(Q) > amp_max) amp_max = abs(Q);
        }
    }
    if (amp_max < 32) amp_max = 32;

    // IQ axis (centre line + ±amp/2, ±amp ticks).
    pdfw_set_fill(w, PDFW_BLACK);
    if (show_iq && iq_h_sub > 4.0f) {
        float iq_mid_y = iq_y0_sub + iq_h_sub * 0.5f;
        pdfw_set_stroke(w, PDFW_LGREY);
        pdfw_lw(w, 0.3f);
        pdfw_line(w, pL, iq_mid_y, pR, iq_mid_y);
        pdfw_set_stroke(w, PDFW_DGREY);
        pdfw_lw(w, 0.5f);
        for (int k = -2; k <= 2; ++k) {
            float y = iq_mid_y - (float)(k * iq_h_sub / 4.0f);
            pdfw_line(w, pL - 4, y, pL, y);
            char buf[24];
            snprintf(buf, sizeof buf, "%+d", (int)(k * amp_max / 2));
            float tw = pdfw_str_width(buf, 9.0f, 1);
            pdfw_text(w, pL - 6 - tw, y - 4, buf, 9.0f, 1);
        }
    }
    // Phase axis (centre line + -π, -π/2, 0, +π/2, +π ticks).
    if (show_phase && phase_h_sub > 4.0f) {
        float ph_mid_y = phase_y0_sub + phase_h_sub * 0.5f;
        pdfw_set_stroke(w, PDFW_LGREY);
        pdfw_lw(w, 0.3f);
        pdfw_line(w, pL, ph_mid_y, pR, ph_mid_y);
        pdfw_set_stroke(w, PDFW_DGREY);
        pdfw_lw(w, 0.5f);
        const char *plabels[5] = {"-pi", "-pi/2", "0", "+pi/2", "+pi"};
        for (int k = -2; k <= 2; ++k) {
            float y = ph_mid_y - (float)(k * phase_h_sub / 4.0f);
            pdfw_line(w, pL - 4, y, pL, y);
            float tw = pdfw_str_width(plabels[k + 2], 9.0f, 1);
            pdfw_text(w, pL - 6 - tw, y - 4, plabels[k + 2], 9.0f, 1);
        }
    }
    // Separator between the two sub-areas in split mode.
    if (split) {
        float sep_y = phase_y1_sub
                    + (iq_y0_sub - phase_y1_sub) * 0.5f;
        pdfw_set_stroke(w, PDFW_DGREY);
        pdfw_lw(w, 0.4f);
        pdfw_line(w, pL, sep_y, pR, sep_y);
    }

    double span = wf_t_hi - wf_t_lo;
    double raw_step = span / 8.0;
    double mag = pow(10.0, floor(log10(raw_step)));
    double mul = raw_step / mag;
    if      (mul < 1.5) mul = 1.0;
    else if (mul < 3.5) mul = 2.0;
    else if (mul < 7.5) mul = 5.0;
    else                mul = 10.0;
    double t_step = mul * mag;
    int    t_nd   = decimals_for_step(t_step);
    double t0_aligned = ceil(wf_t_lo / t_step) * t_step;
    pdfw_set_stroke(w, PDFW_DGREY);
    pdfw_lw(w, 0.5f);
    for (double t = t0_aligned; t <= wf_t_hi + 0.5 * t_step; t += t_step) {
        float x = pL + (float)((t - wf_t_lo) / span * plot_w);
        if (x < pL || x > pR) continue;
        pdfw_line(w, x, pB, x, pB + 4);
        char buf[40];
        fmt_mmss_ndec(t, t_nd, buf, sizeof buf);
        float tw = pdfw_str_width(buf, 9.0f, 1);
        pdfw_text(w, x - tw * 0.5f, pB + 6, buf, 9.0f, 1);
    }
    pdfw_text(w, (pL + pR) * 0.5f - pdfw_str_width("time", 9, 0) * 0.5f,
              pB + 22, "time", 9.0f, 0);

    // Y-axis labels in the left margin.
    if (show_iq && iq_h_sub > 4.0f) {
        pdfw_text(w, MARGIN + 4, iq_y0_sub - 12,
                  "amplitude (int16)", 9.0f, 0);
        float legy = iq_y0_sub + 4;
        if (show_i) { pdfw_text(w, MARGIN + 4, legy, "I = cyan",    9.0f, 0); legy += 12; }
        if (show_q) { pdfw_text(w, MARGIN + 4, legy, "Q = magenta", 9.0f, 0); }
    }
    if (show_phase && phase_h_sub > 4.0f) {
        pdfw_text(w, MARGIN + 4, phase_y0_sub - 12,
                  "phase (rad)", 9.0f, 0);
        pdfw_text(w, MARGIN + 4, phase_y0_sub + 4,
                  "phi = orange", 9.0f, 0);
    }

    if (n_pairs_vis > 0 && plot_w > 8.0f) {
        float y_scale = (iq_h_sub > 0.0f)
            ? (iq_h_sub * 0.5f) / (float) amp_max : 0.0f;
        float ph_scale = (phase_h_sub > 0.0f)
            ? (phase_h_sub * 0.5f) / (float) M_PI : 0.0f;
        float iq_mid_y = iq_y0_sub + iq_h_sub * 0.5f;
        float ph_mid_y = phase_y0_sub + phase_h_sub * 0.5f;
        pdfw_rgb_t I_col  = (pdfw_rgb_t){0,   170, 200, 255};
        pdfw_rgb_t Q_col  = (pdfw_rgb_t){190, 60,  180, 255};
        pdfw_rgb_t Ph_col = (pdfw_rgb_t){230, 130, 40,  255};
        int plot_w_px = (int) plot_w;
        if (n_pairs_vis <= (int64_t) plot_w_px * 2) {
            pdfw_lw(w, 0.5f);
            float prev_xi = -1, prev_yi = 0, prev_xq = -1, prev_yq = 0;
            float prev_xph = -1, prev_yph = 0;
            double prev_phi = 0.0;
            for (int64_t k = i_lo; k < i_hi; ++k) {
                double t = (double) k / iqb->samp_rate;
                float x = pL + (float)((t - wf_t_lo) / span * plot_w);
                if (x < pL || x > pR) continue;
                int I = iqb->samples[k * 2 + 0];
                int Q = iqb->samples[k * 2 + 1];
                if (show_iq) {
                    float yI = iq_mid_y - I * y_scale;
                    float yQ = iq_mid_y - Q * y_scale;
                    if (show_q && prev_xq > 0) {
                        pdfw_set_stroke(w, Q_col);
                        pdfw_line(w, prev_xq, prev_yq, x, yQ);
                    }
                    if (show_i && prev_xi > 0) {
                        pdfw_set_stroke(w, I_col);
                        pdfw_line(w, prev_xi, prev_yi, x, yI);
                    }
                    prev_xi = x; prev_yi = yI;
                    prev_xq = x; prev_yq = yQ;
                }
                if (show_phase) {
                    double phi = atan2((double) Q, (double) I);
                    float yPh = ph_mid_y - (float)(phi * ph_scale);
                    if (prev_xph > 0
                        && fabs(phi - prev_phi) <= M_PI) {
                        pdfw_set_stroke(w, Ph_col);
                        pdfw_line(w, prev_xph, prev_yph, x, yPh);
                    }
                    prev_xph = x; prev_yph = yPh; prev_phi = phi;
                }
            }
        } else {
            pdfw_lw(w, 0.5f);
            for (int x = 0; x < plot_w_px; ++x) {
                int64_t s0 = i_lo + (int64_t) x * n_pairs_vis / plot_w_px;
                int64_t s1 = i_lo + (int64_t)(x+1) * n_pairs_vis / plot_w_px;
                if (s1 <= s0) s1 = s0 + 1;
                if (s1 > i_hi) s1 = i_hi;
                int iMin =  INT_MAX, iMax = INT_MIN;
                int qMin =  INT_MAX, qMax = INT_MIN;
                double phMin =  1e9, phMax = -1e9;
                for (int64_t k = s0; k < s1; ++k) {
                    int I = iqb->samples[k * 2 + 0];
                    int Q = iqb->samples[k * 2 + 1];
                    if (show_iq) {
                        if (I < iMin) iMin = I;
                        if (I > iMax) iMax = I;
                        if (Q < qMin) qMin = Q;
                        if (Q > qMax) qMax = Q;
                    }
                    if (show_phase) {
                        double phi = atan2((double) Q, (double) I);
                        if (phi < phMin) phMin = phi;
                        if (phi > phMax) phMax = phi;
                    }
                }
                float xpx = pL + (float) x;
                if (show_iq && iq_h_sub > 0.0f) {
                    if (show_q) {
                        pdfw_set_stroke(w, Q_col);
                        pdfw_line(w, xpx, iq_mid_y - qMax * y_scale,
                                     xpx, iq_mid_y - qMin * y_scale);
                    }
                    if (show_i) {
                        pdfw_set_stroke(w, I_col);
                        pdfw_line(w, xpx, iq_mid_y - iMax * y_scale,
                                     xpx, iq_mid_y - iMin * y_scale);
                    }
                }
                if (show_phase && phase_h_sub > 0.0f && phMin <= phMax) {
                    pdfw_set_stroke(w, Ph_col);
                    pdfw_line(w, xpx, ph_mid_y - (float)(phMax * ph_scale),
                                 xpx, ph_mid_y - (float)(phMin * ph_scale));
                }
            }
        }
    }

    char foot[256];
    time_t now = time(NULL);
    struct tm utc;
    gmtime_r(&now, &utc);
    snprintf(foot, sizeof foot,
        "generated %04d-%02d-%02dT%02d:%02d:%02dZ  by decode_inspector",
        utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
        utc.tm_hour, utc.tm_min, utc.tm_sec);
    pdfw_set_fill(w, PDFW_GREY);
    pdfw_text(w, MARGIN, page_h - MARGIN - 12, foot, 9.0f, 1);

    return pdfw_end(w);
}

// ---------------------------------------------------------------------------
// Decode-mode (K panel) helpers
// ---------------------------------------------------------------------------

// Free every malloc'd buffer pointed to by `d` and zero the struct.
static void decmode_diag_free(fsk_diag_t *d)
{
    free(d->i_lpf); free(d->q_lpf);
    free(d->fm);    free(d->mf);
    free(d->strobes); free(d->strobe_t);
    free(d->bits); free(d->asm_hamming);
    memset(d, 0, sizeof *d);
}

// Grow the diag buffers (and the caller-owned out-bits scratch) so
// they can hold a recompute over a window of `lpf_cap` LPF samples
// and `strob_cap` strobes. Grow-only — zooming out reallocs, zooming
// back in reuses without churn. Returns 0 on success, -1 on alloc
// failure (in which case the partially-realloc'd buffers stay
// allocated; we don't try to roll back).
static int decmode_diag_grow(fsk_diag_t *d,
                             size_t lpf_cap, size_t strob_cap,
                             size_t *cap_lpf_io,
                             size_t *cap_strob_io,
                             uint8_t **out_scratch_io,
                             size_t *out_scratch_cap_io)
{
    if (lpf_cap > *cap_lpf_io) {
        float *ni  = realloc(d->i_lpf, lpf_cap * sizeof(float));
        float *nq  = realloc(d->q_lpf, lpf_cap * sizeof(float));
        float *nfm = realloc(d->fm,    lpf_cap * sizeof(float));
        float *nmf = realloc(d->mf,    lpf_cap * sizeof(float));
        if (!ni || !nq || !nfm || !nmf) {
            d->i_lpf = ni ? ni : d->i_lpf;
            d->q_lpf = nq ? nq : d->q_lpf;
            d->fm    = nfm ? nfm : d->fm;
            d->mf    = nmf ? nmf : d->mf;
            return -1;
        }
        d->i_lpf = ni; d->q_lpf = nq; d->fm = nfm; d->mf = nmf;
        *cap_lpf_io = lpf_cap;
    }
    if (strob_cap > *cap_strob_io) {
        float   *ns  = realloc(d->strobes,     strob_cap * sizeof(float));
        double  *nt  = realloc(d->strobe_t,    strob_cap * sizeof(double));
        uint8_t *nb  = realloc(d->bits,        strob_cap);
        uint8_t *nh  = realloc(d->asm_hamming, strob_cap);
        uint8_t *no  = realloc(*out_scratch_io, strob_cap);
        if (!ns || !nt || !nb || !nh || !no) return -1;
        d->strobes = ns; d->strobe_t = nt;
        d->bits = nb;   d->asm_hamming = nh;
        *out_scratch_io = no;
        *cap_strob_io = strob_cap;
        *out_scratch_cap_io = strob_cap;
    }
    return 0;
}

// Run the FSK diag chain over iqb[i_lo..i_hi]. Returns the rc from
// modem_fsk_iq_to_bits_diag (0 on sync, -1 on no sync — both leave
// the diag bundle populated with whatever it managed to compute).
static int decmode_recompute_window(const iq_buf_t *iqb,
                                    int64_t i_lo, int64_t i_hi,
                                    int sps, fsk_diag_t *d,
                                    uint8_t *out_scratch,
                                    size_t out_scratch_cap)
{
    if (iqb == NULL || iqb->samples == NULL) return -1;
    if (i_lo < 0) i_lo = 0;
    if (i_hi > (int64_t) iqb->n_pairs) i_hi = (int64_t) iqb->n_pairs;
    if (i_hi <= i_lo) return -1;
    size_t slice_n = (size_t)(i_hi - i_lo);
    modem_params_t p = {
        .samp_rate           = iqb->samp_rate,
        .bit_rate            = (int)(iqb->samp_rate / sps),
        .gain_db             = 0.0,
        .rx_disable_dc_block = 0,
    };
    size_t n_out = 0, sync_off = (size_t)-1;
    int polarity = -1;
    (void) out_scratch_cap;
    return modem_fsk_iq_to_bits_diag(
        iqb->samples + i_lo * 2, slice_n, &p,
        /*invert*/0, /*sync_max_ham*/4, /*min_bit_offset*/0,
        out_scratch, &n_out, &sync_off, &polarity, d);
}

static void usage(void)
{
    fprintf(stderr,
        "usage: decode_inspector <iq_path> [options]\n"
        "  All gen_waterfall options are accepted and forwarded\n"
        "  (--fft-time-bin-s, --zoom-khz, --detrend, --detrend-tau-s, --center-hz,\n"
        "   --dc-notch, --dc-notch-bins, --db-min, --db-max,\n"
        "   --power-offset, --start-utc, --elapsed-time, --fft, --hop,\n"
        "   --marks-csv, --show-tm, --full-width)\n"
        "Annotator-specific:\n"
        "  --rate=<Hz>          IQ rate (default = auto from companion .wav)\n"
        "  --width=<px>         Window width  (default 1280)\n"
        "  --height=<px>        Window height (default 900)\n"
        "  --filter-center-hz=F LPF shift target for the F-key view\n"
        "                       (default -770 Hz)\n"
        "  --filter-lower-hz=F  HPF cutoff in the original IQ frame\n"
        "                       (default 500 Hz; 0 disables HPF stage)\n"
        "  --filter-upper-hz=F  LPF cutoff around the shifted DC\n"
        "                       (default 6000 Hz)\n"
        "  --lo-shift-khz=N     NCO-shift the loaded IQ by -N kHz before\n"
        "                       spectrogram and decode. Positive N moves\n"
        "                       a signal at +N kHz baseband to DC. Use to\n"
        "                       bring an off-DC carrier on an old capture\n"
        "                       to baseband (e.g. -0.77 for the legacy\n"
        "                       RAO -770 Hz LO offset). Default 0.\n");
}

// Trap signals (SIGSEGV / SIGABRT) raised inside InitWindow so a
// hostile X server / GLX setup doesn't core-dump the process before
// we can print a useful hint. raylib forwards GLFW errors to its
// TraceLog as LOG_WARNING and on some builds glfwCreateWindow can
// hand back a half-valid handle that segfaults on the next deref;
// catching the signal lets us bail cleanly instead.
static sigjmp_buf g_init_window_jmp;
static void init_window_crash_handler(int sig)
{
    (void) sig;
    siglongjmp(g_init_window_jmp, 1);
}

// Wraps InitWindow with the trap above + an IsWindowReady() check.
// Returns 0 on success, non-zero on any failure (signal caught,
// glfwCreateWindow returned NULL, etc.). On failure the caller
// should print its own message and exit; this helper just unwinds
// the signal handlers cleanly.
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

// (No software-renderer fallback. `LIBGL_ALWAYS_SOFTWARE=1` just runs
// the OpenGL pipeline on the CPU via mesa/llvmpipe; it doesn't get us
// out of OpenGL. In particular the GLX wire protocol round-trip to
// the X server still happens, so the SSH-X11 case isn't helped at
// all. A true no-OpenGL path would need OSMesa + XPutImage or a
// different renderer entirely, and raylib doesn't expose that. If
// you're hitting this on a headless host with no GPU driver, set
// LIBGL_ALWAYS_SOFTWARE=1 in your environment by hand.)

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(); return 0;
        }
    }
    if (argc < 2) { usage(); return 2; }
    const char *iq_path = argv[1];

    int    samp_rate = 0;
    int    win_w     = 1280;
    int    win_h     = 900;

    // Filter knobs (overridable per-launch; the F-key handler reads
    // these). Defaults are tuned for the FrontierSat downlink at
    // -770 Hz on a 96 kHz IQ.
    double filter_center_hz_cli = -770.0;
    double filter_lower_hz_cli  = 500.0;
    double filter_upper_hz_cli  = 6000.0;

    // Optional global IQ NCO shift applied once at load, before the
    // spectrogram and before the K-panel decode see the buffer. Lets
    // the operator bring an off-DC carrier on an old capture to
    // baseband without re-running the RX chain. Sign matches rx_replay
    // --lo-shift-khz: positive value moves a signal at +N kHz down to
    // DC.
    double lo_shift_hz_cli = 0.0;

    const char *passthru[MAX_PASSTHRU];
    int n_passthru = 0;

    for (int i = 2; i < argc; ++i) {
        const char *a = argv[i];
        if (strncmp(a, "--rate=", 7) == 0) {
            samp_rate = atoi(a + 7);
        } else if (strncmp(a, "--width=", 8) == 0) {
            win_w = atoi(a + 8);
        } else if (strncmp(a, "--height=", 9) == 0) {
            win_h = atoi(a + 9);
        } else if (strncmp(a, "--filter-center-hz=", 19) == 0) {
            filter_center_hz_cli = atof(a + 19);
        } else if (strncmp(a, "--filter-lower-hz=", 18) == 0) {
            filter_lower_hz_cli = atof(a + 18);
        } else if (strncmp(a, "--filter-upper-hz=", 18) == 0) {
            filter_upper_hz_cli = atof(a + 18);
        } else if (strncmp(a, "--lo-shift-khz=", 15) == 0) {
            lo_shift_hz_cli = atof(a + 15) * 1000.0;
        } else if (is_passthru(a)) {
            if (n_passthru >= MAX_PASSTHRU) {
                fprintf(stderr,
                    "decode_inspector: too many gen_waterfall flags (>%d)\n",
                    MAX_PASSTHRU);
                return 2;
            }
            passthru[n_passthru++] = a;
        } else {
            fprintf(stderr, "decode_inspector: unknown option '%s'\n", a);
            usage();
            return 2;
        }
    }

    // Auto-detect rate from companion .wav.
    if (samp_rate <= 0) {
        size_t plen = strlen(iq_path);
        if (plen > 3 && strcmp(iq_path + plen - 3, ".iq") == 0) {
            char wav_path[1100];
            snprintf(wav_path, sizeof wav_path, "%.1000s.wav", iq_path);
            int r = read_wav_samp_rate(wav_path);
            if (r > 0) samp_rate = r;
        }
        if (samp_rate <= 0) samp_rate = 96000;
    }
    fprintf(stderr, "decode_inspector: rate=%d Hz\n", samp_rate);

    // Compute duration from file size.
    struct stat st;
    if (stat(iq_path, &st) != 0) {
        fprintf(stderr, "decode_inspector: stat %s: %s\n",
                iq_path, strerror(errno));
        return 1;
    }
    size_t n_pairs = (size_t) st.st_size / 4;
    double duration_s = (double) n_pairs / (double) samp_rate;
    {
        char pb[32];
        fmt_thousands(pb, sizeof pb, (uint64_t) n_pairs);
        fprintf(stderr, "decode_inspector: duration=%.3fs (%s pairs)\n",
                duration_s, pb);
    }

    // Pull --db-min / --db-max out of passthru into mutable locals so
    // the operator can adjust them at runtime ( ',' / '.' / '<' / '>' /
    // 'R' keys). These get folded into wf_opts_t just before the
    // in-process spectrogram runs.
    double wf_db_min     = 0.0;
    double wf_db_max     = 0.0;
    int    wf_db_min_set = 0;
    int    wf_db_max_set = 0;
    {
        int dst = 0;
        for (int i = 0; i < n_passthru; ++i) {
            if (strncmp(passthru[i], "--db-min=", 9) == 0) {
                wf_db_min     = atof(passthru[i] + 9);
                wf_db_min_set = 1;
            } else if (strncmp(passthru[i], "--db-max=", 9) == 0) {
                wf_db_max     = atof(passthru[i] + 9);
                wf_db_max_set = 1;
            } else {
                passthru[dst++] = passthru[i];
            }
        }
        n_passthru = dst;
    }

    render_opts_t ropts;
    parse_render_opts(passthru, n_passthru, iq_path, &ropts);

    // Translate the same passthru flags into the wf_opts_t that
    // waterfall_core takes. db-min / db-max overrides (already pulled
    // out into mutable locals above) get re-applied here.
    wf_opts_t wf_opt;
    parse_wf_opts_from_passthru(passthru, n_passthru, samp_rate,
                                duration_s, &wf_opt);
    if (wf_db_min_set) { wf_opt.db_min = (float) wf_db_min; wf_opt.db_min_user_set = 1; }
    if (wf_db_max_set) { wf_opt.db_max = (float) wf_db_max; wf_opt.db_max_user_set = 1; }
    // Bridge ropts → wf_opt: parse_render_opts also picks up
    // UT=YYYYMMDDTHHMMSS from the file path, which is how every
    // sso-captured .iq carries its start time. Threading that into
    // wf_opt.start_utc is what makes the left/right time axes render
    // in local HH:MM:SS rather than elapsed seconds. --elapsed-time
    // forces elapsed mode even when the UT is known.
    if (ropts.have_start_utc && !ropts.elapsed_time) {
        wf_opt.start_utc = (time_t) ropts.start_utc_s;
        wf_opt.start_utc_subsec = ropts.start_utc_s - (double) wf_opt.start_utc;
    }

    // ----- raylib window + IQ load + in-process spectrogram -----
    SetTraceLogLevel(LOG_WARNING);
    if (safe_init_window(win_w, win_h, "decode_inspector") != 0) {
        // Couldn't get an OpenGL context. raylib is OpenGL-only, so
        // there isn't a software fallback we can take inside the
        // process — `LIBGL_ALWAYS_SOFTWARE=1` just selects mesa's CPU
        // OpenGL implementation, which still has to traverse GLX to
        // the X server (no help when SSH forwarding is the bottleneck).
        fprintf(stderr,
            "decode_inspector: cannot open a window.\n"
            "\n"
            "If you're connected over SSH with `-X` / `-Y`:\n"
            "  vanilla X11 forwarding only carries the GLX 1.x wire\n"
            "  protocol; the OpenGL 3.3 core context raylib normally\n"
            "  requests can't be created remotely (the\n"
            "  GLX_ARB_create_context_profile errors above are this).\n"
            "  LIBGL_ALWAYS_SOFTWARE=1 doesn't help — it switches mesa\n"
            "  to the CPU pipeline locally, but the GLX request still\n"
            "  round-trips your X server.\n"
            "\n"
            "  The fix is to rebuild raylib with OpenGL 2.1 (the older\n"
            "  profile travels fine over GLX 1.x) and relink. See\n"
            "  README.md for the exact command.\n"
            "\n"
            "Alternatives if you don't want to rebuild raylib:\n"
            "  - Copy the .iq off the remote and run decode_inspector\n"
            "    on your local desktop (lowest friction).\n"
            "  - Use Xpra (`xpra start :100 --start-child=decode_inspector`)\n"
            "    or VirtualGL + VNC — both render on the server side\n"
            "    and ship framebuffer pixels rather than GL calls.\n"
            "\n"
            "If you're truly headless (no DISPLAY, no SSH `-X`): there\n"
            "is no way to put a window on the screen. Run locally.\n");
        return 1;
    }
    SetTargetFPS(60);
    SetExitKey(0);
#ifdef __APPLE__
    decode_inspector_install_pinch_monitor();
#endif
    g_ui_font_loaded = load_ttf_from_known_paths();

    // Spawn the loader thread that handles iq_buf_load + wf_compute, and
    // run a tiny "Loading..." render loop on the main thread until it
    // finishes. Doing the FFT off the main thread keeps the window
    // responsive (and showing progress) for the few seconds a long-pass
    // FFT can take.
    fprintf(stderr,
        "decode_inspector: computing spectrogram (fft=%d, rows=%d, "
        "zoom=%.1f kHz, detrend=%d)...\n",
        wf_opt.fft_size, wf_opt.out_rows,
        wf_opt.zoom_hz / 1e3, wf_opt.detrend_mode);

    loader_ctx_t lctx = {0};
    lctx.iq_path     = iq_path;
    lctx.samp_rate   = samp_rate;
    lctx.lo_shift_hz = lo_shift_hz_cli;
    lctx.opt         = &wf_opt;
    snprintf(lctx.status_msg, sizeof lctx.status_msg, "Loading IQ samples...");
    pthread_mutex_init(&lctx.lock, NULL);

    pthread_t loader_thread;
    if (pthread_create(&loader_thread, NULL,
                       loader_thread_fn, &lctx) != 0) {
        fprintf(stderr, "decode_inspector: pthread_create: %s\n",
                strerror(errno));
        pthread_mutex_destroy(&lctx.lock);
        CloseWindow(); return 1;
    }

    {
        // STATUS_PT is declared further down inside the render-state
        // block; mirror its value here for the splash sizing.
        int splash_pt = 14 * 3;
        int quit_pressed = 0;
        for (;;) {
            pthread_mutex_lock(&lctx.lock);
            int done = lctx.done;
            int err  = lctx.error;
            char msg[128];
            memcpy(msg, lctx.status_msg, sizeof msg);
            pthread_mutex_unlock(&lctx.lock);

            // Operator can ask to quit during the splash; we still have
            // to wait for wf_compute to finish (no cancellation point),
            // but we'll exit cleanly the moment it does.
            if (WindowShouldClose() || IsKeyPressed(KEY_Q)) quit_pressed = 1;

            // Snapshot the progress fields (lock-free reads are fine —
            // a torn value just shows last-frame's bar; both fields
            // settle within milliseconds). The bar reflects the
            // CURRENT phase's progress (0..1), not a weighted overall —
            // the FFT phase dominates wall-clock time and weighting it
            // to start at 50 % made every launch feel half-loaded
            // already. The status text labels which phase is active.
            float phase_pp = lctx.phase_progress;
            if (phase_pp < 0.0f) phase_pp = 0.0f;
            if (phase_pp > 1.0f) phase_pp = 1.0f;
            float overall = phase_pp;

            BeginDrawing();
            ClearBackground(BLACK);
            int win_sw = GetScreenWidth();
            int win_sh = GetScreenHeight();
            int tw = measure_text(msg, splash_pt);
            draw_text(msg,
                      (win_sw - tw) / 2,
                      (win_sh - splash_pt) / 2,
                      splash_pt,
                      (Color){240, 240, 240, 255});

            // Progress bar: 60 % of the window wide, sits a comfortable
            // gap below the status text. Outline + filled inner rect.
            {
                int bar_w  = (win_sw * 6) / 10;
                int bar_h  = 22;
                int bar_x  = (win_sw - bar_w) / 2;
                int bar_y  = (win_sh - splash_pt) / 2 + splash_pt + 24;
                int fill_w = (int)((float) bar_w * overall + 0.5f);
                if (fill_w < 0)        fill_w = 0;
                if (fill_w > bar_w)    fill_w = bar_w;
                DrawRectangle(bar_x, bar_y, fill_w, bar_h,
                              (Color){180, 200, 230, 255});
                DrawRectangleLines(bar_x, bar_y, bar_w, bar_h,
                                   (Color){140, 140, 150, 255});
                char pct_buf[16];
                snprintf(pct_buf, sizeof pct_buf, "%.0f %%",
                         overall * 100.0f);
                int pct_pt = 14;
                int pct_tw = measure_text(pct_buf, pct_pt);
                draw_text(pct_buf,
                          (win_sw - pct_tw) / 2,
                          bar_y + bar_h + 6,
                          pct_pt,
                          (Color){200, 200, 210, 255});
            }
            if (quit_pressed) {
                const char *q = "Quit pending -- exiting when FFT finishes...";
                int qpt = 14;
                int qw  = measure_text(q, qpt);
                draw_text(q,
                          (win_sw - qw) / 2,
                          (win_sh - splash_pt) / 2 + splash_pt + 80,
                          qpt,
                          (Color){200, 200, 200, 255});
            }
            EndDrawing();

            if (done) break;
        }
        pthread_join(loader_thread, NULL);
        pthread_mutex_destroy(&lctx.lock);
        if (lctx.error || quit_pressed) {
            if (lctx.error) {
                fprintf(stderr, "decode_inspector: %s\n", lctx.status_msg);
            }
            if (lctx.spec_db) free(lctx.spec_db);
            iq_buf_free(&lctx.iqb);
            CloseWindow();
            return lctx.error ? 1 : 0;
        }
    }

    iq_buf_t iqb     = lctx.iqb;
    float   *spec_db = lctx.spec_db;
    int      spec_w  = lctx.spec_w;
    int      spec_h  = lctx.spec_h;
    {
        char pb[32];
        fmt_thousands(pb, sizeof pb, (uint64_t) iqb.n_pairs);
        fprintf(stderr,
            "decode_inspector: IQ buffer loaded (%s pairs)\n", pb);
    }
    if (spec_w < 16 || spec_h < 16) {
        fprintf(stderr,
            "decode_inspector: spectrogram too small (%dx%d)\n",
            spec_w, spec_h);
        free(spec_db); iq_buf_free(&iqb); CloseWindow(); return 1;
    }
    // wf_compute returns row 0 = earliest sample. gen_waterfall's
    // render_with_axes then flips that so its PNG reads "newest at
    // top of spec, earliest at bottom" — the convention every other
    // bit of decode_inspector already assumes (box→pixel math and the
    // waveform-panel time mapping both use 1 - t/duration). Flip the
    // float grid here, once, so the GPU texture is in the same
    // orientation gen_waterfall's PNG was.
    {
        float *tmp_row = (float *) malloc((size_t) spec_w * sizeof(float));
        if (tmp_row != NULL) {
            for (int r = 0; r < spec_h / 2; ++r) {
                int r2 = spec_h - 1 - r;
                memcpy(tmp_row,
                       spec_db + (size_t) r * (size_t) spec_w,
                       (size_t) spec_w * sizeof(float));
                memcpy(spec_db + (size_t) r  * (size_t) spec_w,
                       spec_db + (size_t) r2 * (size_t) spec_w,
                       (size_t) spec_w * sizeof(float));
                memcpy(spec_db + (size_t) r2 * (size_t) spec_w,
                       tmp_row,
                       (size_t) spec_w * sizeof(float));
            }
            free(tmp_row);
        }
    }
    double display_bw_hz = wf_opt.display_bw_hz;
    int img_w = WF_LM + spec_w + WF_RM;
    int img_h = WF_TM + spec_h + WF_BM;

    // Operator-mutable dB range. Track the absolute-dBFS values the
    // shader uses for the colorbar labels, plus a "set" flag so 'R'
    // can drop back to the percentile auto-clip from the cached grid.
    float dbmin_abs = wf_opt.display_db_lo
                    + wf_opt.display_db_floor + wf_opt.power_offset_db;
    float dbmax_abs = wf_opt.display_db_hi
                    + wf_opt.display_db_floor + wf_opt.power_offset_db;
    wf_db_min = dbmin_abs;
    wf_db_max = dbmax_abs;

    // Slice the dB grid into row tiles small enough to fit under any
    // reasonable GL_MAX_TEXTURE_SIZE (8K-16K on common GPUs). Each
    // tile is an R32F texture; the fragment shader samples it and
    // looks up the colour in a 256×1 viridis LUT.
    const int TILE_H = 4096;
    int n_tiles = (spec_h + TILE_H - 1) / TILE_H;
    Texture2D *tiles = (Texture2D *) calloc((size_t) n_tiles,
                                            sizeof(Texture2D));
    if (tiles == NULL) {
        fprintf(stderr, "decode_inspector: oom on tile array\n");
        free(spec_db); iq_buf_free(&iqb); CloseWindow(); return 1;
    }
    for (int t = 0; t < n_tiles; ++t) {
        int y0 = t * TILE_H;
        int h  = (y0 + TILE_H <= spec_h) ? TILE_H : (spec_h - y0);
        Image im = {
            .data    = spec_db + (size_t) y0 * (size_t) spec_w,
            .width   = spec_w,
            .height  = h,
            .mipmaps = 1,
            .format  = PIXELFORMAT_UNCOMPRESSED_R32,
        };
        tiles[t] = LoadTextureFromImage(im);
        if (tiles[t].id == 0) {
            fprintf(stderr,
                "decode_inspector: tile %d upload failed (%dx%d)\n",
                t, spec_w, h);
        } else {
            SetTextureFilter(tiles[t], TEXTURE_FILTER_POINT);
        }
    }
    fprintf(stderr,
        "decode_inspector: spec %dx%d, %d tile(s) of up to %d rows, "
        "BW %.1f kHz, floor=%.1f dBFS\n",
        spec_w, spec_h, n_tiles, TILE_H, display_bw_hz / 1e3,
        wf_opt.display_db_floor + wf_opt.power_offset_db);

    // Fragment shader: take the R32F sample (dB), normalise against the
    // operator's current [db_min, db_max] range, and evaluate viridis
    // analytically (Inigo Quilez's 6-term polynomial fit — matches
    // matplotlib's viridis within ~1% per channel and avoids the
    // sampler-completeness pitfalls that a separate LUT texture runs
    // into on Apple's OpenGL 4.1 stack).
    //
    // Two source variants so the same binary works on both raylib
    // builds: GLSL 330 for the default 3.3-core build (Apple/Mac
    // dev hosts) and GLSL 120 for the 2.1 build that's needed when
    // SSH X11 forwarding can't carry the 3.3 core profile (see
    // README.md for the raylib rebuild command). rlGetVersion()
    // picks at runtime.
    static const char *VIRIDIS_BODY =
        "vec3 viridis(float t) {\n"
        "    vec3 c0 = vec3( 0.2777273272234177,  0.005407344544966578,  0.3340998053353061);\n"
        "    vec3 c1 = vec3( 0.1050930431667207,  1.4046135298985746,    1.3845901625946856);\n"
        "    vec3 c2 = vec3(-0.3308618287255563,  0.2148475594682130,    0.0950951630282366);\n"
        "    vec3 c3 = vec3(-4.6342304989834860, -5.7991009733515850,  -19.3324409562798700);\n"
        "    vec3 c4 = vec3( 6.2282699363470810, 14.1799333668050900,   56.6905526006810500);\n"
        "    vec3 c5 = vec3( 4.7763849976702880,-13.7451453777460100,  -65.3530326333723400);\n"
        "    vec3 c6 = vec3(-5.4354558559346310,  4.6458526121785350,   26.3124352495832000);\n"
        "    return c0 + t*(c1 + t*(c2 + t*(c3 + t*(c4 + t*(c5 + t*c6)))));\n"
        "}\n";
    static const char *FRAG_HEAD_330 =
        "#version 330\n"
        "in vec2 fragTexCoord;\n"
        "in vec4 fragColor;\n"
        "uniform sampler2D texture0;\n"
        // db_min / db_max are in the SAME median-subtracted space the
        // texture carries (the operator's absolute dBFS minus the
        // current layer's display_db_floor + power_offset). When we
        // switch between coarse and detail layers — they have different
        // medians — we re-push db_min / db_max accordingly.
        "uniform float db_min;\n"
        "uniform float db_max;\n"
        "out vec4 finalColor;\n";
    static const char *FRAG_MAIN_330 =
        "void main() {\n"
        "    float v = texture(texture0, fragTexCoord).r;\n"
        "    float t = (v - db_min) / max(db_max - db_min, 1e-6);\n"
        "    t = clamp(t, 0.0, 1.0);\n"
        "    finalColor = vec4(viridis(t), 1.0) * fragColor;\n"
        "}\n";
    static const char *FRAG_HEAD_120 =
        "#version 120\n"
        "varying vec2 fragTexCoord;\n"
        "varying vec4 fragColor;\n"
        "uniform sampler2D texture0;\n"
        "uniform float db_min;\n"
        "uniform float db_max;\n";
    static const char *FRAG_MAIN_120 =
        "void main() {\n"
        "    float v = texture2D(texture0, fragTexCoord).r;\n"
        "    float t = (v - db_min) / max(db_max - db_min, 1e-6);\n"
        "    t = clamp(t, 0.0, 1.0);\n"
        "    gl_FragColor = vec4(viridis(t), 1.0) * fragColor;\n"
        "}\n";
    int gl_version = rlGetVersion();
    int use_120 = (gl_version == RL_OPENGL_21);
    char frag_src[4096];
    snprintf(frag_src, sizeof frag_src, "%s%s%s",
             use_120 ? FRAG_HEAD_120 : FRAG_HEAD_330,
             VIRIDIS_BODY,
             use_120 ? FRAG_MAIN_120 : FRAG_MAIN_330);
    Shader wf_shader = LoadShaderFromMemory(NULL, frag_src);
    int loc_dbmin    = GetShaderLocation(wf_shader, "db_min");
    int loc_dbmax    = GetShaderLocation(wf_shader, "db_max");

    // Pre-baked colorbar (1×256 RGBA8). Top row = brightest viridis,
    // bottom row = darkest — i.e. row index i carries WF_VIRIDIS[255-i].
    // Apple's GL is happy with plain RGBA8 textures, so the colorbar
    // doesn't fight with the R32F + shader path.
    Texture2D cb_texture;
    {
        uint8_t cb_pixels[256 * 4];
        for (int i = 0; i < 256; ++i) {
            int j = 255 - i;
            cb_pixels[i * 4 + 0] = WF_VIRIDIS[j][0];
            cb_pixels[i * 4 + 1] = WF_VIRIDIS[j][1];
            cb_pixels[i * 4 + 2] = WF_VIRIDIS[j][2];
            cb_pixels[i * 4 + 3] = 255;
        }
        Image cb_img = {
            .data    = cb_pixels,
            .width   = 1,
            .height  = 256,
            .mipmaps = 1,
            .format  = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
        };
        cb_texture = LoadTextureFromImage(cb_img);
        SetTextureFilter(cb_texture, TEXTURE_FILTER_BILINEAR);
        SetTextureWrap(cb_texture, TEXTURE_WRAP_CLAMP);
    }
    // Push the initial dB range, in the coarse layer's median-subtracted
    // space. The detail draw re-pushes its own range right before
    // drawing.
    {
        float dbmin_co = dbmin_abs
            - wf_opt.display_db_floor - wf_opt.power_offset_db;
        float dbmax_co = dbmax_abs
            - wf_opt.display_db_floor - wf_opt.power_offset_db;
        SetShaderValue(wf_shader, loc_dbmin, &dbmin_co,
                       SHADER_UNIFORM_FLOAT);
        SetShaderValue(wf_shader, loc_dbmax, &dbmax_co,
                       SHADER_UNIFORM_FLOAT);
    }

    // Boxes.
    box_list_t boxes = {0};
    boxes.selected = -1;
    boxes_load(&boxes, iq_path);

    // ----- view state -----
    // The rendered PNG is sampled into the window with a zoom factor:
    //   screen_x = (img_x - view_x) * zoom
    //   screen_y = (img_y - view_y) * zoom
    // zoom=1 is 1:1 pixels (gen_waterfall axes/labels at their original
    // typography). zoom>1 zooms in on the cursor; <1 zooms out.
    // view_x may be negative when the image is narrower than the
    // window — that's how horizontal centring is expressed: the screen
    // origin lies to the left of the image's left edge.
    float zoom    = 1.0f;
    float view_x  = 0.0f;     // image pixel at screen x=0
    float view_y  = 0.0f;     // image pixel at screen y=0
    int   first_frame = 1;    // set initial fit-to-window view on frame 1
    // Which signal to show in the waveform panel + PDF.
    //   0 = I+Q (default)
    //   1 = I only
    //   2 = Q only
    //   3 = phase only (atan2(Q,I), -π..+π)
    //   4 = split: phase on top, I+Q on bottom
    // Cycle with the `i` key.
    int iq_show_mode = 0;
    // Smooth +/- zoom animation. When the user presses + (or =) the
    // target is set to 2× the current zoom centred on the cursor;
    // the per-frame interpolation eases over `anim_dur` seconds.
    float anim_t      = 0.0f;
    float anim_dur    = 1.00f;
    int   anim_active = 0;
    float anim_z_from = 0.0f, anim_z_to = 0.0f;
    // Anchor: image-pixel that was under the cursor at the moment +/-
    // was pressed, and the cursor's screen-pixel coords at that moment.
    // We recompute view_x/view_y each animation frame from the current
    // zoom so that THIS image point keeps sitting under that same
    // screen position throughout the animation — i.e. exactly the
    // pinch-zoom behaviour, just driven by a key + linear ramp.
    float anim_anchor_sx = 0.0f, anim_anchor_sy = 0.0f;
    float anim_anchor_ix = 0.0f, anim_anchor_iy = 0.0f;
    int   dragging = 0;       // 1 = drawing a new box
    Vector2 drag_start = {0, 0};
    // Waveform-panel measurement drag — anchored in TIME so the
    // rectangle still reads the right duration even if the operator
    // pinches/scrolls mid-drag. Cleared on mouse-up.
    int    wf_drag           = 0;
    double wf_drag_t_anchor  = 0.0;

    // Band-pass-filter view (F key). Two stages: HPF in the original
    // frame (cuts everything below filter_lower_hz around DC, i.e.
    // LO leakage / very-low-freq drift), then shift filter_center_hz
    // to DC + LPF (cuts everything above filter_upper_hz around the
    // new DC, i.e. outside the beacon band).
    int       filter_show         = 0;
    int       filter_built        = 0;
    int       filter_building     = 0;
    int16_t  *filtered_iq         = NULL;
    pthread_t filter_thread       = (pthread_t) 0;
    filter_ctx_t fctx             = {0};
    double    filter_center_hz    = filter_center_hz_cli;
    double    filter_lower_hz     = filter_lower_hz_cli;
    double    filter_upper_hz     = filter_upper_hz_cli;
    char  status[256] = "";
    int   label_cycle_idx = 0;

    // Drag-to-resize state. `resize_box` = box index being resized, or
    // -1 if no resize in progress. `resize_edge_x/y` are -1, 0, +1 to
    // mean "drag the low side / no drag / drag the high side" of that
    // axis. (1,0) → right edge; (-1,-1) → top-left corner; etc.
    int resize_box   = -1;
    int resize_edge_x = 0;
    int resize_edge_y = 0;
    // Edge-grab tolerance in screen pixels.
    const int HIT_TOL = 8;
    // Label font size — large enough to read at typical viewing
    // distance over the spectrogram colours.
    const int LABEL_PT = 16;
    const int STATUS_PT = 14;

    // Waveform-panel state: toggle with W. When on, the bottom 240 px
    // of the window become an I/Q time-series plot of the time range
    // currently visible in the spectrogram above. The panel has no
    // independent zoom / pan — pinch / scroll on the spectrogram move
    // both the waterfall and the time-series in lockstep.
    int   wf_open       = 0;
    int   wf_panel_h    = 240;

    // Decode side panel: D presses spawn rx_replay over the IQ slice
    // covering the currently-visible time range and capture its stderr
    // into this buffer. The panel persists on the right until Esc.
    int    decode_open      = 0;
    char  *decode_text      = NULL;
    size_t decode_text_len  = 0;
    int    decode_panel_w   = 520;
    int    decode_scroll    = 0;   // first visible line (panel scroll pos)

    // Decode-mode panel state: toggle with K. Mutually exclusive with
    // the W (waveform) panel — they share the bottom slot. Walks the
    // decode chain from docs/decoding/DECODING.md stage-by-stage;
    // [ / ] cycle stages. The decoder runs on the FULL time range
    // visible in the spectrogram (no fixed-window slice). Recompute
    // fires after a short debounce on zoom/pan so a fast scroll
    // doesn't trigger one per frame.
    #define DECMODE_N_STAGES 10
    static const char *DECMODE_STAGE_NAMES[DECMODE_N_STAGES] = {
        "raw IQ",
        "IQ LPF",
        "FM discriminator",
        "matched filter",
        "Gardner / eye",
        "slicer bits",
        "ASM correlation",
        "Golay length",
        "CCSDS descrambler",
        "Reed-Solomon (255,223)",
    };
    int    decmode_open      = 0;
    int    decmode_stage     = 0;
    int    decmode_panel_h   = 420;       // taller than wf so eye plot
                                          // fits as a 1:1 square
    double decmode_recompute_after = -1.0;
    double decmode_last_t_lo = -1.0;
    double decmode_last_t_hi = -1.0;
    int    decmode_have_result = 0;

    // Diagnostic bundle + scratch buffers. Grow monotonically as the
    // visible window expands; never shrink. The fsk_diag_t struct holds
    // pointers into these.
    fsk_diag_t decmode_diag      = {0};
    size_t     decmode_cap_lpf   = 0;     // i_lpf/q_lpf/fm/mf capacity
    size_t     decmode_cap_strob = 0;     // strobes/strobe_t/bits/asm_h capacity
    uint8_t   *decmode_out_scratch = NULL;
    size_t     decmode_out_scratch_cap = 0;
    int        decmode_input_sps     = 0;

    // Stage 7 (Golay length header) — computed after each recompute
    // when ASM was found. decmode_golay_rc:
    //   -2 = no ASM lock or not enough bits past ASM
    //   -1 = Golay uncorrectable (>3 bit errors)
    //    0 = decoded ok; data12 + errors valid
    uint32_t   decmode_golay_word24 = 0;
    uint16_t   decmode_golay_data12 = 0;
    int        decmode_golay_errors = -1;
    int        decmode_golay_rc     = -2;

    // ASM lock in absolute time, captured at recompute time so the
    // marker + bit-comparison strip can pan/zoom smoothly with the
    // spectrogram between debounced recomputes. -1.0 = no lock.
    double     decmode_asm_abs_time_s = -1.0;
    double     decmode_bit_period_s   = 0.0;

    // Stage 8 (CCSDS descrambler) — first DECMODE_DESCR_CAP bytes
    // immediately after the Golay header, packed MSB-first, plus
    // their CCSDS-descrambled values. decmode_descr_n = 0 → not
    // computed (no Golay lock, or not enough bits past Golay).
    uint8_t    decmode_descr_scrambled[DECMODE_DESCR_CAP];
    uint8_t    decmode_descr_descrambled[DECMODE_DESCR_CAP];
    size_t     decmode_descr_n          = 0;
    // Byte offset shown at the left edge of the descrambler grid.
    // Driven by wheel input while stage 8 is open; clamped to
    // [0, decmode_descr_n - max_cells] each frame. Persists across
    // waterfall pan/zoom recomputes so scrolling doesn't snap back.
    int        decmode_descr_view_off   = 0;
    // Byte-cell width in pixels. Pinch over the descrambler stage
    // adjusts this; wheel scroll is independent.
    float      decmode_descr_cell_w     = 30.0f;

    // Stage 9 (Reed-Solomon 255,223): the descrambled bytes get
    // zero-padded on the left to RS_N=255, then rs_decode runs in
    // place. decmode_rs_in is the codeword as RS saw it (pre-
    // correction); decmode_rs_out is what RS produced. _errors is
    // the number of corrected bytes (0..16), -1 if uncorrectable,
    // -2 if RS could not run (no Golay lock, no descrambled bytes,
    // or input too short).
    uint8_t    decmode_rs_in[RS_N];
    uint8_t    decmode_rs_out[RS_N];
    int        decmode_rs_locs[RS_NROOTS];
    int        decmode_rs_errors        = -2;
    size_t     decmode_rs_pad_len       = 0;
    int        decmode_rs_view_off      = 0;
    float      decmode_rs_cell_w        = 30.0f;

    while (!WindowShouldClose()) {
        int sw = GetScreenWidth();
        int sh = GetScreenHeight();
        Vector2 m = GetMousePosition();

        // ----- input -----
        // Capture pinch + wheel ONCE per frame so we can route them
        // to whichever panel the cursor is over.
        float pinch = 0.0f;
#ifdef __APPLE__
        pinch = g_decode_inspector_pinch_delta;
        g_decode_inspector_pinch_delta = 0.0f;
#endif
        Vector2 wheel_v = GetMouseWheelMoveV();
        // Bottom slot: wf and decmode are mutually exclusive; whichever
        // one is open captures pinch/wheel input over its area.
        int bottom_panel_h_now = wf_open ? wf_panel_h
                               : decmode_open ? decmode_panel_h : 0;
        int in_panel_for_input =
            bottom_panel_h_now > 0
            && (int) m.y >= sh - bottom_panel_h_now && (int) m.y < sh;
        // Decode side panel occupies the right strip when open.
        int in_decode_panel_input =
            decode_open && (int) m.x >= sw - decode_panel_w && (int) m.x < sw;
        if (in_decode_panel_input && wheel_v.y != 0.0f) {
            decode_scroll -= (int)(wheel_v.y * 3.0f);
            if (decode_scroll < 0) decode_scroll = 0;
        }

        // Panel-driven zoom: when the cursor is over the W or K
        // panel (the time-series view), pinch and wheel act on the
        // *time* axis, anchored on the cursor's panel-x mapping.
        // The spectrogram's view_y/zoom are the single source of
        // truth for the visible time window, so the panel handlers
        // just modify those.
        // Byte-grid stages (8 = CCSDS descrambler, 9 = Reed-Solomon)
        // consume wheel + pinch for their own scroll/zoom instead
        // of panning time. Wheel scrolls the byte offset shown at
        // the left edge of the grid; pinch resizes the byte cells.
        // Each stage has its own offset/cell-width state so the
        // operator's view position carries across when they cycle
        // between the two.
        int stage_eats_panel_input = in_panel_for_input
            && decmode_open
            && (decmode_stage == 8 || decmode_stage == 9);
        if (stage_eats_panel_input && (wheel_v.x != 0.0f
                                       || wheel_v.y != 0.0f)) {
            int delta = (int)((wheel_v.x + wheel_v.y) * 2.0f);
            if (decmode_stage == 8) {
                decmode_descr_view_off += delta;
                if (decmode_descr_view_off < 0)
                    decmode_descr_view_off = 0;
                int max_off = (int) decmode_descr_n - 1;
                if (max_off < 0) max_off = 0;
                if (decmode_descr_view_off > max_off)
                    decmode_descr_view_off = max_off;
            } else {
                decmode_rs_view_off += delta;
                if (decmode_rs_view_off < 0)
                    decmode_rs_view_off = 0;
                if (decmode_rs_view_off > RS_N - 1)
                    decmode_rs_view_off = RS_N - 1;
            }
        }
        if (stage_eats_panel_input && pinch != 0.0f) {
            float *cw = (decmode_stage == 8)
                ? &decmode_descr_cell_w : &decmode_rs_cell_w;
            float new_cw = *cw * expf(pinch);
            if (new_cw < 12.0f)  new_cw = 12.0f;
            if (new_cw > 100.0f) new_cw = 100.0f;
            *cw = new_cw;
        }
        if (in_panel_for_input && !stage_eats_panel_input
            && (pinch != 0.0f
                || wheel_v.y != 0.0f
                || wheel_v.x != 0.0f)) {
            int spec_screen_w_in = decode_open
                ? (sw - decode_panel_w) : sw;
            if (spec_screen_w_in < 64) spec_screen_w_in = 64;
            // Time math must match the renderer's spec_bot_y_screen:
            // both W and K derive their time range from the W-panel
            // slot (sh - wf_panel_h) so panels stay synchronised.
            // The K panel happens to be taller and overlaps the
            // bottom of the spec visually; using decmode_panel_h
            // here would give a different visible time range than
            // the renderer and the zoom-to-cursor anchor would
            // drift accordingly.
            int bar_h_in = 2 * (STATUS_PT + 6);
            int spec_bot_y_in = bottom_panel_h_now > 0
                ? (sh - wf_panel_h)
                : (sh - bar_h_in);
            if (spec_bot_y_in < 32) spec_bot_y_in = 32;
            int spec_screen_h_in = spec_bot_y_in;
            double vis_h_in = (double) spec_screen_h_in
                              / (double) zoom;
            double vt_top = (double) view_y;
            double vt_bot = vt_top + vis_h_in;
            if (vt_top < WF_TM) vt_top = WF_TM;
            if (vt_bot > WF_TM + spec_h) vt_bot = WF_TM + spec_h;
            if (vt_bot < vt_top) vt_bot = vt_top;
            double ft_top = (vt_top - WF_TM) / (double) spec_h;
            double ft_bot = (vt_bot - WF_TM) / (double) spec_h;
            double t_hi_in = (1.0 - ft_top) * duration_s;
            double t_lo_in = (1.0 - ft_bot) * duration_s;
            if (t_hi_in < t_lo_in) {
                double tmp = t_lo_in;
                t_lo_in = t_hi_in;
                t_hi_in = tmp;
            }
            double span_in = t_hi_in - t_lo_in;
            if (span_in > 1e-12) {
                // K and W panels both leave a 96 px left margin for
                // y-axis labels and an 8 px right margin. Map the
                // cursor's screen x to the actual *plot* area so a
                // cursor sitting on the left edge of the trace
                // corresponds to frac_x = 0 (not ~0.07).
                const int pL_panel = 96;
                const int pR_panel = 8;
                int plot_x0_in = pL_panel;
                int plot_x1_in = spec_screen_w_in - pR_panel;
                int plot_w_in  = plot_x1_in - plot_x0_in;
                if (plot_w_in < 16) plot_w_in = 16;
                double frac_x = ((double) m.x - (double) plot_x0_in)
                                / (double) plot_w_in;
                if (frac_x < 0.0) frac_x = 0.0;
                if (frac_x > 1.0) frac_x = 1.0;
                double t_cursor = t_lo_in + frac_x * span_in;
                // Spec image-y where t_cursor lives — this is the
                // pivot we want to keep at the cursor's panel x
                // position through the zoom.
                double img_y_t = WF_TM
                    + (1.0 - t_cursor / duration_s)
                      * (double) spec_h;
                // Effective spec screen-y of that pivot.
                double scr_y_eff = (img_y_t - (double) view_y)
                                   * (double) zoom;
                if (pinch != 0.0f) {
                    float img_x_under = view_x + m.x / zoom;
                    float new_zoom = zoom * expf(pinch);
                    if (new_zoom < 0.1f)        new_zoom = 0.1f;
                    if (new_zoom > 1.0e6f)      new_zoom = 1.0e6f;
                    zoom = new_zoom;
                    view_x = img_x_under - m.x / zoom;
                    view_y = (float) (img_y_t
                                      - scr_y_eff / (double) zoom);
                }
                if (wheel_v.y != 0.0f || wheel_v.x != 0.0f) {
                    // Both axes pan time on the panel — the panel's
                    // time axis is left-to-right so wheel.x is the
                    // natural pan, and wheel.y carries on working
                    // for users with vertical-only mice. Freq does
                    // not get a pan from the panel; the panel has
                    // no freq axis to pan along.
                    view_y -= (wheel_v.x + wheel_v.y) * 12.0f / zoom;
                }
            }
        }

        // Spectrogram-view pinch/scroll (only when cursor is NOT
        // in the waveform panel — the panel uses identical input
        // semantics but applies to its own time-zoom/pan state).
        if (!in_panel_for_input && !in_decode_panel_input && pinch != 0.0f) {
            float img_x_under = view_x + m.x / zoom;
            float img_y_under = view_y + m.y / zoom;
            float new_zoom = zoom * expf(pinch);
            // Allow extreme zoom — enough that the waveform panel
            // (which mirrors the spec view's visible time range) can
            // reach the bit level (~10 samples at 96 kSPS). At a
            // 4096-row render of a 784-s pass, one image row covers
            // ~0.19 s; to show 10 samples (~0.1 ms) needs roughly
            // 5×10⁴–10⁵× zoom. We cap at 1e6 to leave headroom and
            // avoid FP precision issues in the view-coord math.
            if (new_zoom < 0.1f)        new_zoom = 0.1f;
            if (new_zoom > 1.0e6f)      new_zoom = 1.0e6f;
            zoom = new_zoom;
            view_x = img_x_under - m.x / zoom;
            view_y = img_y_under - m.y / zoom;
        }
        if (!in_panel_for_input && !in_decode_panel_input
            && (wheel_v.x != 0.0f || wheel_v.y != 0.0f)) {
            view_x -= wheel_v.x * 12.0f / zoom;
            view_y -= wheel_v.y * 12.0f / zoom;
        }
        // Right-click drag = freehand pan (any direction).
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 d = GetMouseDelta();
            view_x -= d.x / zoom;
            view_y -= d.y / zoom;
        }
        // Arrow keys = pan; PgUp/PgDn = big vertical step; Home/End = endpoints.
        float pan_step = 8.0f / zoom;
        if (IsKeyDown(KEY_UP))    view_y -= pan_step;
        if (IsKeyDown(KEY_DOWN))  view_y += pan_step;
        if (IsKeyDown(KEY_LEFT))  view_x -= pan_step;
        if (IsKeyDown(KEY_RIGHT)) view_x += pan_step;
        float page_step = (sh / zoom) * 0.8f;
        if (IsKeyPressed(KEY_PAGE_DOWN)) view_y += page_step;
        if (IsKeyPressed(KEY_PAGE_UP))   view_y -= page_step;
        if (IsKeyPressed(KEY_HOME)) view_y = 0;
        if (IsKeyPressed(KEY_END))  view_y = img_h - sh / zoom;
        // Zoom resets — back to fit-image-in-window, centred.
        if (IsKeyPressed(KEY_ZERO) || IsKeyPressed(KEY_KP_0)) {
            first_frame = 1;
            anim_active = 0;
        }
        // +/= zooms in by 5×, − zooms out by 1/5×, both anchoring on
        // the point currently under the cursor (same semantics as
        // trackpad pinch — the image point stays put, the world
        // around it just scales). Animation is a straight-line ramp
        // on `zoom`; view_x/view_y are recomputed from the anchor
        // each frame so the anchor doesn't drift.
        int zoom_in_press  = IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD);
        int zoom_out_press = IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT);
        if (zoom_in_press || zoom_out_press) {
            float new_zoom = zoom * (zoom_in_press ? 5.0f : 0.2f);
            if (new_zoom < 0.1f)        new_zoom = 0.1f;
            if (new_zoom > 1.0e6f)      new_zoom = 1.0e6f;
            anim_anchor_sx = m.x;
            anim_anchor_sy = m.y;
            anim_anchor_ix = view_x + m.x / zoom;
            anim_anchor_iy = view_y + m.y / zoom;
            anim_z_from = zoom;
            anim_z_to   = new_zoom;
            anim_t = 0.0f;
            anim_active = 1;
        }
        // Per-frame animation step — linear ramp on zoom; recompute
        // view so anim_anchor_i stays under anim_anchor_s for the
        // whole animation (no wobble toward a moving target).
        if (anim_active) {
            anim_t += GetFrameTime();
            float u = anim_t / anim_dur;
            if (u >= 1.0f) { u = 1.0f; anim_active = 0; }
            zoom   = anim_z_from + u * (anim_z_to - anim_z_from);
            view_x = anim_anchor_ix - anim_anchor_sx / zoom;
            view_y = anim_anchor_iy - anim_anchor_sy / zoom;
        }
        // Clamp view to image extents. The spec drawing area on
        // screen goes from y=0 to y=spec_screen_h; the waveform
        // panel (when open) and the status bar replace the bottom
        // of the window so the spec doesn't render behind them.
        // Likewise spec_screen_w shrinks when the decode side panel
        // is open so the spec doesn't render behind that strip.
        int   bar_h_clamp = 2 * (STATUS_PT + 6);
        // wf and decmode share the bottom slot — exactly one can be
        // open at a time (the K and W handlers enforce that). Pick
        // whichever panel height is non-zero, falling back to the
        // status bar at the foot when neither is open.
        int   bottom_panel_h = wf_open ? wf_panel_h
                             : decmode_open ? decmode_panel_h : 0;
        int   spec_screen_h = bottom_panel_h
            ? (sh - bottom_panel_h) : (sh - bar_h_clamp);
        if (spec_screen_h < 32) spec_screen_h = 32;
        int   spec_screen_w = decode_open ? (sw - decode_panel_w) : sw;
        if (spec_screen_w < 64) spec_screen_w = 64;

        // First frame: zoom so the entire image height fits in the
        // current spec area, and centre the image horizontally in the
        // (spec portion of the) window. Negative view_x represents a
        // centring offset.
        if (first_frame) {
            zoom = (float) spec_screen_h / (float) img_h;
            if (zoom < 0.001f) zoom = 1.0f;
            view_y = 0.0f;
            float img_screen_w = (float) img_w * zoom;
            view_x = (img_screen_w < spec_screen_w)
                ? -((float) spec_screen_w / zoom - (float) img_w) * 0.5f
                : 0.0f;
            first_frame = 0;
        }

        float visible_w = spec_screen_w / zoom;
        float visible_h = spec_screen_h / zoom;
        // Horizontal: if image fits in window, lock to a centred view
        // (visualised as negative view_x). Otherwise allow pan within
        // image extents.
        if (visible_w >= img_w) {
            view_x = -(visible_w - (float) img_w) * 0.5f;
        } else {
            if (view_x < 0) view_x = 0;
            if (view_x > img_w - visible_w) view_x = img_w - visible_w;
        }
        if (view_y < 0) view_y = 0;
        if (visible_h >= img_h) view_y = 0;
        else if (view_y > img_h - visible_h) view_y = img_h - visible_h;

        if (IsKeyPressed(KEY_S)) {
            boxes_save(&boxes, iq_path);
            snprintf(status, sizeof status,
                "saved %d box(es) (boxes.csv + box_anchors.csv)", boxes.n);
        }
        if (IsKeyPressed(KEY_L)) {
            boxes_load(&boxes, iq_path);
            snprintf(status, sizeof status, "loaded %d box(es)", boxes.n);
        }
        if (IsKeyPressed(KEY_W)) {
            wf_open = !wf_open;
            if (wf_open) decmode_open = 0; // share the bottom slot
            snprintf(status, sizeof status,
                "waveform panel %s (follows spectrogram time range)",
                wf_open ? "ON" : "OFF");
        }
        // K: toggle decode-mode panel. Mutually exclusive with W. When
        // turning on, force a recompute on the next frame (debounce
        // target = now). Brackets cycle stages whenever the panel is
        // open (overriding the box-advance binding further down).
        if (IsKeyPressed(KEY_K)) {
            decmode_open = !decmode_open;
            if (decmode_open) {
                wf_open = 0;
                decmode_recompute_after = GetTime();
                decmode_last_t_lo = -1.0;
                decmode_last_t_hi = -1.0;
            }
            snprintf(status, sizeof status,
                "decode mode %s — stage %d/%d  %s",
                decmode_open ? "ON" : "OFF",
                decmode_stage + 1, DECMODE_N_STAGES,
                decmode_open ? DECMODE_STAGE_NAMES[decmode_stage] : "");
        }
        if (decmode_open
            && (IsKeyPressed(KEY_RIGHT_BRACKET)
                || IsKeyPressed(KEY_LEFT_BRACKET))) {
            int dir = IsKeyPressed(KEY_RIGHT_BRACKET) ? 1
                    : -1;
            decmode_stage = (decmode_stage + DECMODE_N_STAGES + dir)
                            % DECMODE_N_STAGES;
            snprintf(status, sizeof status,
                "decode stage %d/%d — %s",
                decmode_stage + 1, DECMODE_N_STAGES,
                DECMODE_STAGE_NAMES[decmode_stage]);
        }
        if (IsKeyPressed(KEY_I)) {
            iq_show_mode = (iq_show_mode + 1) % 5;
            const char *mode_label =
                (iq_show_mode == 0) ? "I + Q"
              : (iq_show_mode == 1) ? "I only"
              : (iq_show_mode == 2) ? "Q only"
              : (iq_show_mode == 3) ? "phase"
              :                       "phase + I/Q";
            snprintf(status, sizeof status,
                "waveform channels: %s", mode_label);
        }
        // F: band-pass-filter view. First press kicks off the filter
        // worker (mixes filter_center_hz to DC, real LPF with bw/2
        // cutoff); subsequent presses toggle the waveform-panel
        // between raw and filtered IQ. The live state (building %,
        // raw / FILTERED) lives in its own bar above the main status
        // bar (see render below); status[] is only used here for
        // one-time error / file-write announcements.
        if (IsKeyPressed(KEY_F) && iqb.samples != NULL) {
            if (!filter_built && !filter_building) {
                filtered_iq = (int16_t *) malloc(
                    iqb.n_pairs * 2 * sizeof(int16_t));
                if (filtered_iq != NULL) {
                    memset(&fctx, 0, sizeof fctx);
                    fctx.iq_in     = iqb.samples;
                    fctx.n_pairs   = iqb.n_pairs;
                    fctx.samp_rate = samp_rate;
                    fctx.center_hz = filter_center_hz;
                    fctx.lower_hz  = filter_lower_hz;
                    fctx.upper_hz  = filter_upper_hz;
                    fctx.iq_out    = filtered_iq;
                    if (pthread_create(&filter_thread, NULL,
                                       filter_thread_fn, &fctx) == 0) {
                        filter_building = 1;
                    } else {
                        free(filtered_iq); filtered_iq = NULL;
                        snprintf(status, sizeof status,
                            "filter: pthread_create failed");
                    }
                } else {
                    snprintf(status, sizeof status,
                        "filter: out of memory");
                }
            } else if (filter_built) {
                filter_show = !filter_show;
            }
            // While filter_building, F is a no-op (the dedicated bar
            // already shows progress).
        }
        // Reap the filter thread if it's finished. Done once per
        // frame so the toggle is available the instant the worker
        // returns.
        if (filter_building && fctx.done) {
            pthread_join(filter_thread, NULL);
            filter_building = 0;
            if (fctx.error) {
                free(filtered_iq); filtered_iq = NULL;
                snprintf(status, sizeof status, "filter: failed");
            } else {
                filter_built = 1;
                filter_show  = 1;
                // Write the cleaned IQ to /tmp so rx_replay can take
                // it. Path is logged to stderr; status[] gets just the
                // basename so it doesn't smear across the help line.
                char fn[1200];
                const char *base = strrchr(iq_path, '/');
                base = (base != NULL) ? base + 1 : iq_path;
                snprintf(fn, sizeof fn,
                    "/tmp/%.480s.filt%+0.0fHz_hp%.0f_lp%.0f.iq",
                    base, filter_center_hz,
                    filter_lower_hz, filter_upper_hz);
                FILE *fp = fopen(fn, "wb");
                if (fp != NULL) {
                    fwrite(filtered_iq, sizeof(int16_t) * 2,
                           iqb.n_pairs, fp);
                    fclose(fp);
                    const char *short_base = strrchr(fn, '/');
                    short_base = short_base ? short_base + 1 : fn;
                    // Width-bound short_base so gcc's truncation
                    // checker is happy — short_base can be up to ~1200
                    // bytes (size of fn[]), status is 256.
                    snprintf(status, sizeof status,
                        "wrote /tmp/%.200s", short_base);
                    fprintf(stderr,
                        "decode_inspector: filtered IQ -> %s\n", fn);
                } else {
                    snprintf(status, sizeof status,
                        "filter write failed: %s", strerror(errno));
                }
            }
        }
        // Color-scale controls — instantaneous now: the keystroke just
        // nudges dbmin_abs / dbmax_abs and pushes the new uniforms; the
        // GPU re-colours on the next frame. 'R' drops back to the
        // percentile auto-clip computed from the cached dB grid
        // (cheap; no FFT redo).
        {
            int shift_down = IsKeyDown(KEY_LEFT_SHIFT)
                          || IsKeyDown(KEY_RIGHT_SHIFT);
            int changed = 0;
            const float STEP_DB = 3.0f;
            if (IsKeyPressed(KEY_COMMA)) {
                if (shift_down) dbmax_abs -= STEP_DB;
                else            dbmin_abs -= STEP_DB;
                changed = 1;
            } else if (IsKeyPressed(KEY_PERIOD)) {
                if (shift_down) dbmax_abs += STEP_DB;
                else            dbmin_abs += STEP_DB;
                changed = 1;
            } else if (IsKeyPressed(KEY_R)) {
                float lo_internal = 0.0f, hi_internal = 0.0f;
                wf_auto_db_range(spec_db, spec_w, spec_h,
                                 &lo_internal, &hi_internal);
                dbmin_abs = lo_internal
                          + wf_opt.display_db_floor + wf_opt.power_offset_db;
                dbmax_abs = hi_internal
                          + wf_opt.display_db_floor + wf_opt.power_offset_db;
                changed = 1;
            }
            if (changed) {
                if (dbmax_abs < dbmin_abs + 1.0f) dbmax_abs = dbmin_abs + 1.0f;
                // Color key uniforms always reflect the coarse layer;
                // detail draw re-pushes its own values.
                float dbmin_co = dbmin_abs
                    - wf_opt.display_db_floor - wf_opt.power_offset_db;
                float dbmax_co = dbmax_abs
                    - wf_opt.display_db_floor - wf_opt.power_offset_db;
                SetShaderValue(wf_shader, loc_dbmin, &dbmin_co,
                               SHADER_UNIFORM_FLOAT);
                SetShaderValue(wf_shader, loc_dbmax, &dbmax_co,
                               SHADER_UNIFORM_FLOAT);
                wf_db_min = dbmin_abs;
                wf_db_max = dbmax_abs;
                wf_db_min_set = wf_db_max_set = 1;
                snprintf(status, sizeof status,
                    "color: db-min=%.1f dBFS  db-max=%.1f dBFS  "
                    "(',/.': floor  '<>': ceiling  'R': auto)",
                    dbmin_abs, dbmax_abs);
            }
        }
        // ']' → advance to the START of the next box (in time).
        // '[' → step back to the START of the previous box.
        // Zoom and horizontal pan stay put — only view_y moves so the
        // selected box's t0_s sits in the middle of the spec area.
        //
        // Boxes are kept sorted chronologically (see boxes_sort). When
        // a box is selected we walk by INDEX (selected ± 1) — that's
        // robust to the view-clamp pulling view_y back to 0 when the
        // whole pass fits in the window, which used to lock the anchor
        // at duration/2 forever. With no selection we fall back to a
        // t0-based search anchored on the time at the centre of the view.
        // The [ and ] keys advance / rewind through boxes, but when
        // the decode-mode panel is open they're stage navigation
        // (handled higher up); skip the box-walk in that case.
        int walk_fwd  = !decmode_open && IsKeyPressed(KEY_RIGHT_BRACKET);
        int walk_back = !decmode_open && IsKeyPressed(KEY_LEFT_BRACKET);
        if ((walk_fwd || walk_back) && boxes.n > 0) {
            int target = -1;
            if (boxes.selected >= 0 && boxes.selected < boxes.n) {
                int step = walk_fwd ? +1 : -1;
                int cand = boxes.selected + step;
                if (cand >= 0 && cand < boxes.n) target = cand;
            } else {
                // No selection — find the first / last box past the
                // centre of the current view.
                double center_img_y =
                    (double) view_y + (double) visible_h * 0.5;
                double ref_t = (1.0 - (center_img_y - WF_TM)
                                / (double) spec_h) * duration_s;
                const double eps = 1e-6;
                if (walk_fwd) {
                    for (int i = 0; i < boxes.n; ++i) {
                        if (boxes.items[i].t0_s > ref_t + eps) {
                            target = i; break;
                        }
                    }
                } else {
                    for (int i = boxes.n - 1; i >= 0; --i) {
                        if (boxes.items[i].t0_s < ref_t - eps) {
                            target = i; break;
                        }
                    }
                }
            }
            if (target >= 0) {
                double best_t = boxes.items[target].t0_s;
                double target_img_y =
                    WF_TM + (1.0 - best_t / duration_s) * (double) spec_h;
                view_y = (float)(target_img_y - (double) visible_h * 0.5);
                boxes.selected = target;
                snprintf(status, sizeof status,
                    "%s box %d/%d (%s, t0=%.3fs)",
                    walk_back ? "rewind to" : "advance to",
                    target + 1, boxes.n,
                    boxes.items[target].label, best_t);
            } else {
                int at_end =
                    (boxes.selected >= 0 && boxes.selected < boxes.n);
                snprintf(status, sizeof status,
                    "%s",
                    walk_fwd
                      ? (at_end ? "at last box" : "no next box")
                      : (at_end ? "at first box" : "no previous box"));
            }
        }

        // P → write the current waveform panel to a vector PDF
        // alongside the .iq. Deferred until after wf_t_lo/wf_t_hi
        // and box_info are computed later in the frame.
        int pdf_requested = IsKeyPressed(KEY_P);
        if (IsKeyPressed(KEY_Q)) break;

        // Esc closes the decode side panel (does NOT quit — SetExitKey(0)
        // disabled raylib's default Esc-to-exit at startup).
        if (IsKeyPressed(KEY_ESCAPE)) {
            if (decode_open) {
                decode_open = 0;
                snprintf(status, sizeof status, "decode panel closed");
            }
        }

        // D → spawn rx_replay over the IQ slice covering the currently-
        // visible time range and show its output in the side panel.
        // Synchronous (typically a few hundred ms for the slice sizes
        // operators use); the UI freezes for that long. Esc clears.
        if (IsKeyPressed(KEY_D) && iqb.samples != NULL) {
            // Visible time range, mirroring how wf_t_lo / wf_t_hi are
            // computed for the waveform panel further down.
            double vis_top_img = (double) view_y;
            double vis_bot_img = (double) view_y + (double) visible_h;
            if (vis_top_img < WF_TM) vis_top_img = WF_TM;
            if (vis_bot_img > WF_TM + spec_h) vis_bot_img = WF_TM + spec_h;
            if (vis_bot_img < vis_top_img) vis_bot_img = vis_top_img;
            double frac_top = (vis_top_img - WF_TM) / (double) spec_h;
            double frac_bot = (vis_bot_img - WF_TM) / (double) spec_h;
            double t_hi = (1.0 - frac_top) * duration_s;
            double t_lo = (1.0 - frac_bot) * duration_s;
            if (t_hi < t_lo) { double tmp = t_lo; t_lo = t_hi; t_hi = tmp; }
            // Pad ±1 s so rx_replay's 1.5 s sliding window always has
            // enough samples even when the operator zoomed into one
            // burst. Clamp to file extents.
            double t0 = t_lo - 1.0;
            double t1 = t_hi + 1.0;
            if (t0 < 0.0) t0 = 0.0;
            if (t1 > duration_s) t1 = duration_s;
            int64_t i_lo = (int64_t)(t0 * iqb.samp_rate);
            int64_t i_hi = (int64_t)(t1 * iqb.samp_rate);
            if (i_lo < 0) i_lo = 0;
            if (i_hi > (int64_t) iqb.n_pairs) i_hi = (int64_t) iqb.n_pairs;
            int64_t n_pairs_dec = i_hi - i_lo;
            if (n_pairs_dec < (int64_t) iqb.samp_rate) {
                snprintf(status, sizeof status,
                    "decode: window too small (%.3fs)", t1 - t0);
            } else {
                // Source: filtered IQ when F-toggle is on AND the
                // filter worker has finished; raw IQ otherwise.
                // The temp filename includes the source tag so it's
                // visible from the rx_replay command line and any
                // outside inspection of /tmp.
                const int16_t *src_iq =
                    (filter_show && filter_built && filtered_iq != NULL)
                    ? filtered_iq : iqb.samples;
                const char *src_label =
                    (src_iq == filtered_iq) ? "filt" : "raw";
                char tmp_iq[160], tmp_txt[160];
                snprintf(tmp_iq, sizeof tmp_iq,
                    "/tmp/iqa_decode_%d_%s.iq",
                    (int) getpid(), src_label);
                snprintf(tmp_txt, sizeof tmp_txt,
                    "/tmp/iqa_decode_%d_%s.txt",
                    (int) getpid(), src_label);
                FILE *fo = fopen(tmp_iq, "wb");
                if (fo == NULL) {
                    snprintf(status, sizeof status,
                        "decode: open %s: %s",
                        tmp_iq, strerror(errno));
                } else {
                    size_t want_pairs = (size_t) n_pairs_dec;
                    fwrite(src_iq + i_lo * 2,
                           sizeof(int16_t),
                           want_pairs * 2, fo);
                    fclose(fo);
                    char cmd[512];
                    snprintf(cmd, sizeof cmd,
                        "rx_replay %s --iq --rate=%d >%s 2>&1",
                        tmp_iq, iqb.samp_rate, tmp_txt);
                    int rc = system(cmd);
                    // Slurp the captured output.
                    FILE *fi = fopen(tmp_txt, "r");
                    if (fi != NULL) {
                        fseek(fi, 0, SEEK_END);
                        long sz = ftell(fi);
                        fseek(fi, 0, SEEK_SET);
                        free(decode_text);
                        decode_text = NULL;
                        decode_text_len = 0;
                        if (sz > 0) {
                            decode_text = (char *) malloc((size_t)(sz + 1));
                            if (decode_text != NULL) {
                                size_t r = fread(decode_text, 1, (size_t) sz, fi);
                                decode_text[r] = '\0';
                                decode_text_len = r;
                            }
                        }
                        fclose(fi);
                    }
                    unlink(tmp_iq);
                    unlink(tmp_txt);
                    decode_open = 1;
                    decode_scroll = 0;
                    if (rc == 0) {
                        snprintf(status, sizeof status,
                            "decode: rx_replay over %.2fs window "
                            "(t=%.2fs..%.2fs, %s IQ)",
                            t1 - t0, t0, t1, src_label);
                    } else {
                        snprintf(status, sizeof status,
                            "decode: rx_replay rc=%d (is it on PATH?, %s IQ)",
                            rc, src_label);
                    }
                }
            }
        }

        // Cursor → image px → (t, f).
        float img_x_f = view_x + m.x / zoom;
        float img_y_f = view_y + m.y / zoom;
        int   img_x   = (int) img_x_f;
        int   img_y   = (int) img_y_f;
        // When the waveform panel is open, exclude its area from
        // "in_spec" so clicks/drags in the panel don't also drive
        // box drawing / selection / resize on the spectrogram above.
        // Same applies to the decode-mode panel — it shares the slot.
        int   in_panel_now =
            (wf_open && (int) m.y >= sh - wf_panel_h && (int) m.y < sh)
         || (decmode_open && (int) m.y >= sh - decmode_panel_h && (int) m.y < sh);
        int   in_decode_now =
            decode_open && (int) m.x >= spec_screen_w;
        // Spec is drawn from screen (0,0) to (spec_screen_w, spec_screen_h);
        // below that is the waveform panel (when open) or the status bar,
        // and to the right is the decode side panel (when open).
        int   in_spec = (img_x >= WF_LM && img_x < WF_LM + spec_w
                      && img_y >= WF_TM && img_y < WF_TM + spec_h
                      && (int) m.x < spec_screen_w
                      && (int) m.y < spec_screen_h
                      && !in_panel_now && !in_decode_now);
        double cursor_t = 0.0, cursor_f = 0.0;
        if (in_spec) {
            double col_frac = (img_x_f - WF_LM) / (double) spec_w;
            double row_frac_top = (img_y_f - WF_TM) / (double) spec_h;
            cursor_t = (1.0 - row_frac_top) * duration_s;
            cursor_f = (col_frac - 0.5) * display_bw_hz;
        }

        // Box currently under cursor (smallest box if multiple overlap).
        int hovered = -1;
        if (in_spec) {
            double best_area = 0.0;
            for (int i = 0; i < boxes.n; ++i) {
                const box_t *b = &boxes.items[i];
                if (cursor_t < b->t0_s || cursor_t > b->t1_s) continue;
                if (cursor_f < b->f_lo_hz || cursor_f > b->f_hi_hz) continue;
                double area = (b->t1_s - b->t0_s) * (b->f_hi_hz - b->f_lo_hz);
                if (hovered < 0 || area < best_area) {
                    hovered = i; best_area = area;
                }
            }
        }

        // Delete deletes the HOVERED box, not the selected one.
        if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressed(KEY_DELETE)) {
            if (hovered >= 0 && hovered < boxes.n) {
                int i = hovered;
                for (int j = i; j < boxes.n - 1; ++j)
                    boxes.items[j] = boxes.items[j + 1];
                --boxes.n;
                if (boxes.selected == i) boxes.selected = -1;
                else if (boxes.selected > i) --boxes.selected;
                snprintf(status, sizeof status,
                    "deleted box %d (%d remaining)", i, boxes.n);
                hovered = -1;
            }
        }

        // T cycles the label of the hovered (or, if none hovered, the
        // selected) box.
        if (IsKeyPressed(KEY_T)) {
            int target = (hovered >= 0) ? hovered : boxes.selected;
            if (target >= 0 && target < boxes.n) {
                box_t *b = &boxes.items[target];
                int cur = -1;
                for (int k = 0; k < N_LABEL_PRESETS; ++k) {
                    if (strcmp(b->label, LABEL_PRESETS[k]) == 0) {
                        cur = k; break;
                    }
                }
                int next = (cur + 1) % N_LABEL_PRESETS;
                snprintf(b->label, sizeof b->label, "%s",
                         LABEL_PRESETS[next]);
                snprintf(status, sizeof status, "box %d label: %s",
                         target, b->label);
            }
        }

        // Edge / corner hit test on the hovered box. Reports which
        // sides are within HIT_TOL screen-pixels of the cursor, so a
        // click on the edge starts a resize-drag instead of a select.
        int edge_hit_x = 0;  // -1 left, +1 right, 0 none
        int edge_hit_y = 0;  // -1 top,  +1 bottom, 0 none
        if (hovered >= 0) {
            const box_t *b = &boxes.items[hovered];
            int x0_img = WF_LM + (int)(((b->f_lo_hz / display_bw_hz) + 0.5) * spec_w);
            int x1_img = WF_LM + (int)(((b->f_hi_hz / display_bw_hz) + 0.5) * spec_w);
            int y0_img = WF_TM + (int)((1.0 - b->t0_s / duration_s) * spec_h);
            int y1_img = WF_TM + (int)((1.0 - b->t1_s / duration_s) * spec_h);
            int xL = (x0_img < x1_img) ? x0_img : x1_img;
            int xR = (x0_img < x1_img) ? x1_img : x0_img;
            int yT = (y0_img < y1_img) ? y0_img : y1_img;
            int yB = (y0_img < y1_img) ? y1_img : y0_img;
            int sx_l = (int)((xL - view_x) * zoom);
            int sx_r = (int)((xR - view_x) * zoom);
            int sy_t = (int)((yT - view_y) * zoom);
            int sy_b = (int)((yB - view_y) * zoom);
            if (abs((int) m.x - sx_l) <= HIT_TOL) edge_hit_x = -1;
            else if (abs((int) m.x - sx_r) <= HIT_TOL) edge_hit_x = +1;
            if (abs((int) m.y - sy_t) <= HIT_TOL) edge_hit_y = -1;
            else if (abs((int) m.y - sy_b) <= HIT_TOL) edge_hit_y = +1;
        }
        // Mouse-cursor feedback for the hit kind.
        if (resize_box >= 0) {
            int ax = (resize_edge_x != 0), ay = (resize_edge_y != 0);
            if (ax && ay)      SetMouseCursor(MOUSE_CURSOR_RESIZE_NWSE);
            else if (ax)       SetMouseCursor(MOUSE_CURSOR_RESIZE_EW);
            else if (ay)       SetMouseCursor(MOUSE_CURSOR_RESIZE_NS);
            else               SetMouseCursor(MOUSE_CURSOR_DEFAULT);
        } else if (hovered >= 0 && (edge_hit_x || edge_hit_y)) {
            int diag = (edge_hit_x != 0 && edge_hit_y != 0);
            // ↘ for TL+BR, ↗ for TR+BL — raylib only ships NWSE/NESW.
            int nwse = diag && ((edge_hit_x == -1 && edge_hit_y == -1)
                              || (edge_hit_x == +1 && edge_hit_y == +1));
            if (diag) SetMouseCursor(nwse ? MOUSE_CURSOR_RESIZE_NWSE
                                          : MOUSE_CURSOR_RESIZE_NESW);
            else if (edge_hit_x != 0) SetMouseCursor(MOUSE_CURSOR_RESIZE_EW);
            else                      SetMouseCursor(MOUSE_CURSOR_RESIZE_NS);
        } else {
            SetMouseCursor(MOUSE_CURSOR_DEFAULT);
        }

        if (in_spec && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            if (hovered >= 0 && (edge_hit_x || edge_hit_y)) {
                // Start resize drag.
                resize_box    = hovered;
                resize_edge_x = edge_hit_x;
                resize_edge_y = edge_hit_y;
                boxes.selected = hovered;
                snprintf(status, sizeof status,
                    "resizing box %d (%s%s)",
                    hovered,
                    edge_hit_y < 0 ? "top"   : edge_hit_y > 0 ? "bottom" : "",
                    edge_hit_x < 0 ? (edge_hit_y ? "-left"  : "left")
                                   : edge_hit_x > 0 ? (edge_hit_y ? "-right" : "right")
                                                    : "");
            } else if (hovered >= 0) {
                boxes.selected = hovered;
                const box_t *b = &boxes.items[hovered];
                snprintf(status, sizeof status,
                    "selected box %d (%s, t=%.3f..%.3fs, f=%.0f..%.0f Hz)",
                    hovered, b->label, b->t0_s, b->t1_s,
                    b->f_lo_hz, b->f_hi_hz);
            } else {
                dragging = 1;
                drag_start = m;
                boxes.selected = -1;
            }
        }
        // Live update during resize drag.
        if (resize_box >= 0 && IsMouseButtonDown(MOUSE_LEFT_BUTTON) && in_spec) {
            box_t *b = &boxes.items[resize_box];
            float ix = view_x + m.x / zoom;
            float iy = view_y + m.y / zoom;
            double col_frac = (ix - WF_LM) / (double) spec_w;
            double row_frac_top = (iy - WF_TM) / (double) spec_h;
            double t_at = (1.0 - row_frac_top) * duration_s;
            double f_at = (col_frac - 0.5) * display_bw_hz;
            // Box is stored as (t0_s, t1_s) — order isn't normalised
            // here; we adjust the EDGE the user grabbed. After release
            // we re-normalise so t0_s ≤ t1_s, f_lo_hz ≤ f_hi_hz.
            if (resize_edge_x == -1) b->f_lo_hz = f_at;
            else if (resize_edge_x == +1) b->f_hi_hz = f_at;
            if (resize_edge_y == -1) b->t1_s = t_at;  // top edge = later t
            else if (resize_edge_y == +1) b->t0_s = t_at;  // bottom = earlier t
        }
        if (resize_box >= 0 && IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            // raylib reports IsMouseButtonDown == false on the frame
            // the button is released, so the live-update block above
            // is skipped for THIS frame. Capture the release-frame
            // mouse position here so the user's final movement
            // actually lands on the box (without this, the box sticks
            // at the previous-frame position — what the user saw as
            // "start/stop times aren't the most recently adjusted").
            if (in_spec) {
                box_t *b = &boxes.items[resize_box];
                float ix = view_x + m.x / zoom;
                float iy = view_y + m.y / zoom;
                double col_frac = (ix - WF_LM) / (double) spec_w;
                double row_frac_top = (iy - WF_TM) / (double) spec_h;
                double t_at = (1.0 - row_frac_top) * duration_s;
                double f_at = (col_frac - 0.5) * display_bw_hz;
                if (resize_edge_x == -1) b->f_lo_hz = f_at;
                else if (resize_edge_x == +1) b->f_hi_hz = f_at;
                if (resize_edge_y == -1) b->t1_s = t_at;
                else if (resize_edge_y == +1) b->t0_s = t_at;
            }
            box_normalize(&boxes.items[resize_box]);
            // Take a copy of the just-resized box BEFORE sorting, so we
            // can report its position after re-indexing.
            box_t resized_copy = boxes.items[resize_box];
            boxes.selected = resize_box;
            boxes_sort(&boxes);  // t0 may have moved; renumber chronologically.
            int new_idx = (boxes.selected >= 0) ? boxes.selected : resize_box;
            snprintf(status, sizeof status,
                "resized box %d (t=%.3f..%.3fs, f=%.0f..%.0f Hz)",
                new_idx,
                resized_copy.t0_s, resized_copy.t1_s,
                resized_copy.f_lo_hz, resized_copy.f_hi_hz);
            resize_box = -1; resize_edge_x = 0; resize_edge_y = 0;
        }
        if (dragging && IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            dragging = 0;
            double t0, t1, f0, f1;
            {
                float sx = drag_start.x;
                float sy = drag_start.y;
                float ix = view_x + sx / zoom;
                float iy = view_y + sy / zoom;
                double col_frac = (ix - WF_LM) / (double) spec_w;
                double row_frac_top = (iy - WF_TM) / (double) spec_h;
                t0 = (1.0 - row_frac_top) * duration_s;
                f0 = (col_frac - 0.5) * display_bw_hz;
            }
            {
                float ix = view_x + m.x / zoom;
                float iy = view_y + m.y / zoom;
                double col_frac = (ix - WF_LM) / (double) spec_w;
                double row_frac_top = (iy - WF_TM) / (double) spec_h;
                t1 = (1.0 - row_frac_top) * duration_s;
                f1 = (col_frac - 0.5) * display_bw_hz;
            }
            box_t b = {0};
            b.t0_s = t0; b.t1_s = t1;
            b.f_lo_hz = f0; b.f_hi_hz = f1;
            box_normalize(&b);
            const char *lbl = LABEL_PRESETS[label_cycle_idx % N_LABEL_PRESETS];
            snprintf(b.label, sizeof b.label, "%s", lbl);
            if ((b.t1_s - b.t0_s) > 0.005
             && (b.f_hi_hz - b.f_lo_hz) > 10.0
             && boxes.n < BOXES_MAX) {
                boxes.items[boxes.n] = b;
                boxes.selected = boxes.n;
                ++boxes.n;
                // Renumber chronologically so ']' / '[' walk 1..n in
                // time order and the user's mental model matches.
                boxes_sort(&boxes);
                int new_idx = (boxes.selected >= 0)
                              ? boxes.selected : boxes.n - 1;
                snprintf(status, sizeof status,
                    "added box %d (%s, %.3fs × %.0fHz)",
                    new_idx, b.label,
                    b.t1_s - b.t0_s, b.f_hi_hz - b.f_lo_hz);
            }
        }

        // ----- render -----
        BeginDrawing();
        ClearBackground(BLACK);

        // The spec area lives in image-space at
        //   x ∈ [WF_LM, WF_LM + spec_w)
        //   y ∈ [WF_TM, WF_TM + spec_h)
        // (the WF_* margins are still here so the axes/labels/colorbar
        // in task 5 have somewhere to sit). Tiles tile the spec area
        // vertically into TILE_H-row R32F slabs; the fragment shader
        // does the dB → viridis mapping live, so dragging the dB range
        // costs one SetShaderValue / frame and no recomputation.
        double spec_left_img  = (double) WF_LM;
        double spec_right_img = (double)(WF_LM + spec_w);
        double spec_top_img   = (double) WF_TM;
        double spec_bot_img   = (double)(WF_TM + spec_h);

        double src_x_img = (double) view_x;
        if (src_x_img < spec_left_img) src_x_img = spec_left_img;
        double src_right_img = (double) view_x + visible_w;
        if (src_right_img > spec_right_img) src_right_img = spec_right_img;
        double src_w_img = src_right_img - src_x_img;

        double view_y_top = (double) view_y;
        if (view_y_top < spec_top_img) view_y_top = spec_top_img;
        double view_y_bot = (double) view_y + visible_h;
        if (view_y_bot > spec_bot_img) view_y_bot = spec_bot_img;

        if (src_w_img > 0.0 && view_y_bot > view_y_top) {
            int t_first = (int)((view_y_top - spec_top_img) / TILE_H);
            int t_last  = (int)((view_y_bot - 1.0 - spec_top_img) / TILE_H);
            if (t_first < 0)         t_first = 0;
            if (t_last  >= n_tiles)  t_last  = n_tiles - 1;
            BeginShaderMode(wf_shader);
            for (int t = t_first; t <= t_last; ++t) {
                if (tiles[t].id == 0) continue;
                double tile_top_img = spec_top_img + (double)(t * TILE_H);
                int    tile_rows    = (t * TILE_H + TILE_H <= spec_h)
                                       ? TILE_H : (spec_h - t * TILE_H);
                double tile_bot_img = tile_top_img + (double) tile_rows;
                double vis_top_img = (view_y_top > tile_top_img)
                                     ? view_y_top : tile_top_img;
                double vis_bot_img = (view_y_bot < tile_bot_img)
                                     ? view_y_bot : tile_bot_img;
                if (vis_bot_img <= vis_top_img) continue;
                Rectangle src = {
                    .x = (float)(src_x_img - spec_left_img),
                    .y = (float)(vis_top_img - tile_top_img),
                    .width  = (float) src_w_img,
                    .height = (float)(vis_bot_img - vis_top_img),
                };
                Rectangle dst = {
                    .x = (float)((src_x_img - (double) view_x) * zoom),
                    .y = (float)((vis_top_img - (double) view_y) * zoom),
                    .width  = (float)(src_w_img * zoom),
                    .height = (float)((vis_bot_img - vis_top_img) * zoom),
                };
                DrawTexturePro(tiles[t], src, dst, (Vector2){0, 0}, 0.0f, WHITE);
            }
            EndShaderMode();
        }

        // ----- axes / labels / colorbar (drawn at image-space tick
        // positions; both text size and tick lengths scale with zoom
        // so the axes grow with the spec — same behaviour gen_waterfall
        // got "for free" from baking labels into the PNG). -----
        {
            const int   AXIS_PT_BASE = STATUS_PT;
            // Scaled label point sizes. Clamp to a minimum so labels
            // don't collapse to 0 px at extreme zoom-out.
            int lpt_axis = (int)((float) AXIS_PT_BASE * zoom + 0.5f);
            int lpt_sub  = (int)((float)(AXIS_PT_BASE - 2) * zoom + 0.5f);
            if (lpt_axis < 2) lpt_axis = 2;
            if (lpt_sub  < 2) lpt_sub  = 2;
            const Color AXIS_COL = (Color){200, 200, 200, 255};
            const Color TICK_COL = (Color){160, 160, 160, 255};

            // Bottom freq axis — ticks every pick_tick_step Hz across
            // the rendered bandwidth (display_bw_hz).
            {
                int axis_y_img    = WF_TM + spec_h + 3;
                int axis_y_screen = (int)(((double) axis_y_img
                                           - (double) view_y) * zoom);
                int tick_len      = (int)(4.0f * zoom + 0.5f);
                if (tick_len < 2) tick_len = 2;
                if (axis_y_screen >= 0 && axis_y_screen < sh - lpt_axis - 2) {
                    double f_step = wf_pick_tick_step(display_bw_hz, 8);
                    double f_lo   = -display_bw_hz / 2.0;
                    double f_hi   =  display_bw_hz / 2.0;
                    double f_first = ceil(f_lo / f_step) * f_step;
                    for (double f = f_first; f <= f_hi + 1e-6; f += f_step) {
                        float x_img = WF_LM + (float)(((f / display_bw_hz)
                                                      + 0.5) * spec_w);
                        int sx = (int)(((double) x_img - (double) view_x) * zoom);
                        if (sx < 0 || sx >= sw) continue;
                        DrawLine(sx, axis_y_screen,
                                 sx, axis_y_screen + tick_len, TICK_COL);
                        char buf[32];
                        double f_label = f + wf_opt.center_hz;
                        wf_fmt_freq(f_label, buf, sizeof buf);
                        int tw = measure_text(buf, lpt_axis);
                        draw_text(buf, sx - tw / 2,
                                  axis_y_screen + tick_len + 2,
                                  lpt_axis, AXIS_COL);
                    }
                }
            }

            // Left + right time axes, mirroring gen_waterfall's
            // three-tier hierarchy:
            //   1 s  ultra-minor  — short white pip (only when there's
            //                       at least ~3 px per second to keep
            //                       adjacent ticks from merging).
            //   20 s minor        — medium gray tick with a ":SS"
            //                       sub-label (e.g. ":20", ":40").
            //   major             — long gray tick with full HH:MM:SS,
            //                       interval chosen by wf_pick_time_step.
            // When wf_opt.start_utc is set, the 20 s + major sequences
            // align to wall-clock boundaries (so labels read ":00 / :20
            // / :40 / 12:11:00" instead of arbitrary capture-relative
            // offsets).
            {
                int TICK_1S  = (int)(3.0f  * zoom + 0.5f);
                int TICK_20S = (int)(6.0f  * zoom + 0.5f);
                int TICK_MAJ = (int)(10.0f * zoom + 0.5f);
                if (TICK_1S  < 1) TICK_1S  = 1;
                if (TICK_20S < 2) TICK_20S = 2;
                if (TICK_MAJ < 3) TICK_MAJ = 3;
                // Time-axis labels run 25 % smaller than the freq +
                // colorbar labels so the HH:MM:SS columns don't crowd
                // the spec edges at high zoom.
                int SS_PT   = (int)((float) lpt_sub  * 0.75f + 0.5f);
                int AXIS_PT = (int)((float) lpt_axis * 0.75f + 0.5f);
                if (SS_PT   < 2) SS_PT   = 2;
                if (AXIS_PT < 2) AXIS_PT = 2;
                const Color FINE_COL = (Color){255, 255, 255, 255};

                int axis_x_img_left  = WF_LM - 1;
                int axis_x_img_right = WF_LM + spec_w + 1;
                int axis_x_screen_left =
                    (int)(((double) axis_x_img_left
                           - (double) view_x) * zoom);
                int axis_x_screen_right =
                    (int)(((double) axis_x_img_right
                           - (double) view_x) * zoom);
                int draw_left  = (axis_x_screen_left  > 30
                               && axis_x_screen_left  < sw);
                int draw_right = (axis_x_screen_right > 0
                               && axis_x_screen_right < sw - 50);

                double t_step = wf_pick_time_step(duration_s, 10);
                double major_offset   = 0.0;
                double minor20_offset = 0.0;
                if (wf_opt.start_utc != 0) {
                    long step_maj = (long)(t_step + 0.5);
                    if (step_maj > 0) {
                        long mod = ((long) wf_opt.start_utc) % step_maj;
                        if (mod < 0) mod += step_maj;
                        major_offset =
                            (double)((step_maj - mod) % step_maj);
                    }
                    long mod20 = ((long) wf_opt.start_utc) % 20L;
                    if (mod20 < 0) mod20 += 20L;
                    minor20_offset = (double)((20L - mod20) % 20L);
                }

                double px_per_s = (duration_s > 0.0)
                                  ? (double) spec_h / duration_s : 0.0;
                int draw_1s = (px_per_s * zoom >= 3.0);

                if (draw_left || draw_right) {
                    // 1 s pips first so the longer minor / major ticks
                    // overpaint them at coincident positions.
                    if (draw_1s) {
                        for (double t = 0.0;
                             t <= duration_s + 0.5; t += 1.0)
                        {
                            float y_img = WF_TM
                                + (float)((1.0 - t / duration_s) * spec_h);
                            int sy = (int)(((double) y_img
                                            - (double) view_y) * zoom);
                            if (sy < 0 || sy >= sh) continue;
                            if (draw_left) {
                                DrawLine(axis_x_screen_left - TICK_1S, sy,
                                         axis_x_screen_left,            sy,
                                         FINE_COL);
                            }
                            if (draw_right) {
                                DrawLine(axis_x_screen_right,            sy,
                                         axis_x_screen_right + TICK_1S,  sy,
                                         FINE_COL);
                            }
                        }
                    }

                    // 20 s minor ticks with ":SS" labels. Skipped where
                    // they'd land on a major tick (the major loop draws
                    // a longer tick + full label at that spot).
                    for (double t = minor20_offset;
                         t <= duration_s + 0.5; t += 20.0)
                    {
                        double mod = fmod(t - major_offset, t_step);
                        if (fabs(mod) < 1.0 || fabs(mod - t_step) < 1.0) {
                            continue;
                        }
                        float y_img = WF_TM
                            + (float)((1.0 - t / duration_s) * spec_h);
                        int sy = (int)(((double) y_img
                                        - (double) view_y) * zoom);
                        if (sy < 0 || sy >= sh) continue;
                        char ssbuf[8];
                        if (wf_opt.start_utc != 0) {
                            long sec = (((long) wf_opt.start_utc
                                         + (long)(t + 0.5)) % 60L + 60L)
                                       % 60L;
                            snprintf(ssbuf, sizeof ssbuf, ":%02ld", sec);
                        } else {
                            snprintf(ssbuf, sizeof ssbuf, ":%02d",
                                     (int)(t + 0.5) % 60);
                        }
                        int ssw = measure_text(ssbuf, SS_PT);
                        if (draw_left) {
                            DrawLine(axis_x_screen_left - TICK_20S, sy,
                                     axis_x_screen_left,            sy,
                                     TICK_COL);
                            draw_text(ssbuf,
                                      axis_x_screen_left - TICK_MAJ - 2
                                          - ssw,
                                      sy - SS_PT / 2,
                                      SS_PT, AXIS_COL);
                        }
                        if (draw_right) {
                            DrawLine(axis_x_screen_right,             sy,
                                     axis_x_screen_right + TICK_20S,  sy,
                                     TICK_COL);
                            draw_text(ssbuf,
                                      axis_x_screen_right + TICK_MAJ + 3,
                                      sy - SS_PT / 2,
                                      SS_PT, AXIS_COL);
                        }
                    }

                    // Majors with full HH:MM:SS labels.
                    for (double t = major_offset;
                         t <= duration_s + 0.5 * t_step; t += t_step)
                    {
                        float y_img = WF_TM
                            + (float)((1.0 - t / duration_s) * spec_h);
                        int sy = (int)(((double) y_img
                                        - (double) view_y) * zoom);
                        if (sy < 0 || sy >= sh) continue;
                        char buf[32];
                        wf_fmt_time(wf_opt.start_utc, t, buf, sizeof buf);
                        int tw = measure_text(buf, AXIS_PT);
                        if (draw_left) {
                            DrawLine(axis_x_screen_left - TICK_MAJ, sy,
                                     axis_x_screen_left,            sy,
                                     TICK_COL);
                            draw_text(buf,
                                      axis_x_screen_left - TICK_MAJ - 2
                                          - tw,
                                      sy - AXIS_PT / 2,
                                      AXIS_PT, AXIS_COL);
                        }
                        if (draw_right) {
                            DrawLine(axis_x_screen_right,             sy,
                                     axis_x_screen_right + TICK_MAJ,  sy,
                                     TICK_COL);
                            draw_text(buf,
                                      axis_x_screen_right + TICK_MAJ + 3,
                                      sy - AXIS_PT / 2,
                                      AXIS_PT, AXIS_COL);
                        }
                    }
                }
            }

            // Right colorbar + dB tick labels. The colorbar sits in the
            // outer half of the right margin (60 px past the spec edge,
            // mirroring gen_waterfall's PNG layout) so the right time
            // axis has its own gutter. The 1×256 RGBA8 strip is
            // stretched to spec_h; dB labels are derived live from
            // dbmin_abs / dbmax_abs so they update the instant the
            // operator nudges the colormap.
            {
                int cb_w_img  = 16;
                int cb_x_img  = WF_LM + spec_w + 60;
                int cb_y_img  = WF_TM;
                int cb_h_img  = spec_h;
                double sx_d = ((double) cb_x_img - (double) view_x) * zoom;
                double sy_d = ((double) cb_y_img - (double) view_y) * zoom;
                double sw_d = (double) cb_w_img * zoom;
                double sh_d = (double) cb_h_img * zoom;
                int    tick_len = (int)(4.0f * zoom + 0.5f);
                int    label_gap = (int)(6.0f * zoom + 0.5f);
                if (tick_len  < 2) tick_len  = 2;
                if (label_gap < 3) label_gap = 3;
                if (sx_d < sw && sx_d + sw_d > 0 &&
                    sy_d < sh && sy_d + sh_d > 0) {
                    Rectangle src = { 0, 0, 1, 256 };
                    Rectangle dst = { (float) sx_d, (float) sy_d,
                                      (float) sw_d, (float) sh_d };
                    DrawTexturePro(cb_texture, src, dst,
                                   (Vector2){0, 0}, 0.0f, WHITE);

                    double db_range = (double) dbmax_abs - (double) dbmin_abs;
                    if (db_range > 0.0) {
                        double db_step  = wf_pick_tick_step(db_range, 6);
                        double db_first = ceil((double) dbmin_abs / db_step)
                                          * db_step;
                        int cb_right_screen = (int)(sx_d + sw_d);
                        for (double db = db_first; db <= dbmax_abs + 1e-6;
                             db += db_step)
                        {
                            double f = (db - (double) dbmin_abs) / db_range;
                            float y_img = WF_TM
                                + (float)((1.0 - f) * spec_h);
                            int sy = (int)(((double) y_img
                                            - (double) view_y) * zoom);
                            if (sy < 0 || sy >= sh) continue;
                            DrawLine(cb_right_screen,            sy,
                                     cb_right_screen + tick_len, sy,
                                     TICK_COL);
                            char buf[32];
                            snprintf(buf, sizeof buf, "%.0f", db);
                            draw_text(buf,
                                      cb_right_screen + label_gap,
                                      sy - lpt_axis / 2,
                                      lpt_axis, AXIS_COL);
                        }
                    }
                    // Unit label above the colorbar.
                    int unit_y_screen = (int)(((double)(WF_TM - 1)
                                               - (double) view_y) * zoom)
                                       - lpt_axis - 2;
                    if (unit_y_screen < 0) unit_y_screen = 0;
                    draw_text(wf_opt.power_unit,
                              (int) sx_d, unit_y_screen,
                              lpt_axis, AXIS_COL);
                }
            }
        }

        // Boxes: (t, f) → image px → screen px = (img_px - view_xy) * zoom.
        for (int i = 0; i < boxes.n; ++i) {
            const box_t *b = &boxes.items[i];
            float x0_img = WF_LM + (float)(((b->f_lo_hz / display_bw_hz) + 0.5) * spec_w);
            float x1_img = WF_LM + (float)(((b->f_hi_hz / display_bw_hz) + 0.5) * spec_w);
            float y0_img = WF_TM + (float)((1.0 - b->t0_s / duration_s) * spec_h);
            float y1_img = WF_TM + (float)((1.0 - b->t1_s / duration_s) * spec_h);
            float x_lo_i = (x0_img < x1_img) ? x0_img : x1_img;
            float x_hi_i = (x0_img < x1_img) ? x1_img : x0_img;
            float y_lo_i = (y0_img < y1_img) ? y0_img : y1_img;
            float y_hi_i = (y0_img < y1_img) ? y1_img : y0_img;
            int x_lo = (int)((x_lo_i - view_x) * zoom);
            int x_hi = (int)((x_hi_i - view_x) * zoom);
            int y_lo = (int)((y_lo_i - view_y) * zoom);
            int y_hi = (int)((y_hi_i - view_y) * zoom);
            if (x_hi < 0 || x_lo >= sw || y_hi < 0 || y_lo >= sh) continue;
            Color col;
            int   thickness;
            if (i == hovered)              { col = ORANGE;  thickness = 3; }
            else if (i == boxes.selected)  { col = YELLOW;  thickness = 2; }
            else                           { col = SKYBLUE; thickness = 1; }
            // Use DrawRectangleLinesEx for non-1 thickness.
            Rectangle r = { (float) x_lo, (float) y_lo,
                            (float)(x_hi - x_lo), (float)(y_hi - y_lo) };
            DrawRectangleLinesEx(r, (float) thickness, col);
            char tag[64];
            snprintf(tag, sizeof tag, "%d:%s", i, b->label);
            int label_y = y_lo - (LABEL_PT + 2);
            if (label_y < 2) label_y = y_lo + 2;
            draw_text(tag, x_lo + 4, label_y, LABEL_PT, col);
        }

        // Ongoing drag rectangle (in screen space — drag_start was a
        // screen-px coordinate at the start of the drag, and we want
        // the rectangle to track the cursor in screen space).
        if (dragging) {
            int x_lo = (int)((drag_start.x < m.x) ? drag_start.x : m.x);
            int x_hi = (int)((drag_start.x < m.x) ? m.x : drag_start.x);
            int y_lo = (int)((drag_start.y < m.y) ? drag_start.y : m.y);
            int y_hi = (int)((drag_start.y < m.y) ? m.y : drag_start.y);
            DrawRectangleLines(x_lo, y_lo, x_hi - x_lo, y_hi - y_lo, RED);
        }

        // ----- waveform panel (toggle W) -----
        // Reserves the bottom wf_panel_h pixels of the window. Inside
        // the panel, the mouse wheel zooms the time-axis (centred on
        // cursor) and click-drag pans. Outside the panel, those inputs
        // still drive the spectrogram view above. We process panel
        // input AFTER spectrogram input (the panel input handlers
        // overwrite the spectrogram-input view changes for mouse
        // events whose y is in the panel).
        int wf_y0 = sh - wf_panel_h;
        // The waveform panel shows whatever time range is visible in
        // the spectrogram above. gen_waterfall flips the spec so
        // image-row WF_TM is the NEWEST sample (t = duration) and
        // image-row WF_TM + spec_h is the OLDEST (t = 0). Each row
        // covers a time interval of duration/spec_h; the top edge of
        // the top row is at t = duration and the bottom edge of the
        // bottom row is at t = 0. We map fractional image-y to time
        // using the FRACTION of the spec height (not a discrete row
        // index) to avoid an off-by-one row-width gap at each end.
        int bar_h_for_wf = 2 * (STATUS_PT + 6);
        // Time-range derivation uses the W-panel's slot whether W or
        // K is open, so the time series and decode panels always
        // cover the same time range — operators can switch between
        // them and compare features by time without the bottom of
        // the range shifting. (The K panel is physically taller so
        // its top edge sits inside the W-slot region; the time math
        // pretends the spec extends down to the W bottom either way.)
        // When neither bottom panel is open, the spec stretches down
        // to the status bar and we use that.
        int spec_bot_y_screen =
            (wf_open || decmode_open)
                ? wf_y0
                : (sh - bar_h_for_wf);
        double vis_top_img = (double) view_y;
        double vis_bot_img = (double) view_y
                           + (double) spec_bot_y_screen / (double) zoom;
        // Clamp to the spec area; if the view extends into either of
        // gen_waterfall's margins (top axis labels / bottom freq axis +
        // colorbar), the panel just shows the inside-spec portion.
        if (vis_top_img < WF_TM) vis_top_img = WF_TM;
        if (vis_bot_img > WF_TM + spec_h) vis_bot_img = WF_TM + spec_h;
        if (vis_bot_img < vis_top_img)    vis_bot_img = vis_top_img;
        double frac_top = (vis_top_img - WF_TM) / (double) spec_h;
        double frac_bot = (vis_bot_img - WF_TM) / (double) spec_h;
        double wf_t_hi = (1.0 - frac_top) * duration_s;
        double wf_t_lo = (1.0 - frac_bot) * duration_s;
        if (wf_t_hi < wf_t_lo) {
            double tmp = wf_t_lo; wf_t_lo = wf_t_hi; wf_t_hi = tmp;
        }

        // Decode-mode debounced recompute. When the visible time range
        // changes (scroll, zoom, pan), arm the timer; when it fires,
        // slice iqb to [wf_t_lo, wf_t_hi] and rerun the FSK chain.
        // The fixed 9600 baud assumption matches rx_replay's default
        // and the FrontierSat link.
        if (decmode_open && iqb.samples != NULL) {
            if (fabs(wf_t_lo - decmode_last_t_lo) > 1e-9
             || fabs(wf_t_hi - decmode_last_t_hi) > 1e-9) {
                decmode_recompute_after = GetTime() + 0.25;
                decmode_last_t_lo = wf_t_lo;
                decmode_last_t_hi = wf_t_hi;
            }
            if (decmode_recompute_after > 0.0
                && GetTime() >= decmode_recompute_after
                && wf_t_hi > wf_t_lo) {
                decmode_recompute_after = -1.0;
                int64_t i_lo = (int64_t)(wf_t_lo * iqb.samp_rate);
                int64_t i_hi = (int64_t)(wf_t_hi * iqb.samp_rate);
                if (i_lo < 0) i_lo = 0;
                if (i_hi > (int64_t) iqb.n_pairs)
                    i_hi = (int64_t) iqb.n_pairs;
                // Hand the decoder at least 0.3 s of IQ starting at the
                // visible window's left edge — the matched filter,
                // Gardner loop, and ASM correlator all need room to
                // work even when the operator has zoomed in tight. The
                // render side still clips to [wf_t_lo, wf_t_hi] so the
                // K-panel time axis matches the W panel.
                int64_t min_dec_n = (int64_t)(0.3 * iqb.samp_rate);
                if (i_hi - i_lo < min_dec_n)
                    i_hi = i_lo + min_dec_n;
                if (i_hi > (int64_t) iqb.n_pairs)
                    i_hi = (int64_t) iqb.n_pairs;
                size_t slice_n = (i_hi > i_lo)
                    ? (size_t)(i_hi - i_lo) : 0;
                const int bit_rate = 9600;
                int sps = (iqb.samp_rate > 0
                           && (iqb.samp_rate % bit_rate) == 0)
                          ? iqb.samp_rate / bit_rate : 0;
                if (sps >= 2 && slice_n > 32u * (size_t) sps) {
                    size_t lpf_cap = (slice_n > 30u)
                        ? (slice_n - 30u) : 0;
                    size_t strob_cap =
                        modem_fsk_diag_max_strobes(slice_n, sps);
                    if (decmode_diag_grow(
                            &decmode_diag, lpf_cap, strob_cap,
                            &decmode_cap_lpf, &decmode_cap_strob,
                            &decmode_out_scratch,
                            &decmode_out_scratch_cap) == 0) {
                        (void) decmode_recompute_window(
                            &iqb, i_lo, i_hi, sps, &decmode_diag,
                            decmode_out_scratch,
                            decmode_out_scratch_cap);
                        decmode_input_sps   = sps;
                        decmode_have_result = 1;
                        // Stage 7: Golay(24,12) over the 24 bits right
                        // after the ASM. Pack MSB-first to match the
                        // way ax100.c lifts those 3 bytes off the wire.
                        decmode_golay_word24 = 0;
                        decmode_golay_data12 = 0;
                        decmode_golay_errors = -1;
                        decmode_golay_rc     = -2;
                        // Capture the ASM lock in absolute time so the
                        // marker + bit-strip can survive zoom/pan
                        // without waiting for the next recompute.
                        decmode_asm_abs_time_s = -1.0;
                        decmode_bit_period_s   =
                            (sps > 0) ? ((double) sps / (double) iqb.samp_rate)
                                      : 0.0;
                        if (decmode_diag.asm_offset != (size_t) -1
                            && decmode_diag.strobe_t != NULL) {
                            // Stay in the same coordinate system the
                            // trace renderer uses: strobe_t is in
                            // MF-sample units, and the trace draws
                            // strobe k at wf_t_lo + st[k]/sr. The
                            // marker (and the bit strip cells, which
                            // step in symbol periods from this time)
                            // must land on the trace's dip, so we
                            // ignore the LPF+discrim+MF group delay
                            // here — adding it would offset the
                            // marker by ~4 bits at 9600 baud / 96 kSPS.
                            decmode_asm_abs_time_s =
                                ((double) i_lo
                                 + decmode_diag.strobe_t[decmode_diag.asm_offset])
                                / (double) iqb.samp_rate;
                        }
                        if (decmode_diag.asm_offset != (size_t) -1
                            && decmode_diag.bits != NULL
                            && decmode_diag.n_strobes
                                 >= decmode_diag.asm_offset + 32 + 24) {
                            uint32_t g = 0;
                            size_t base = decmode_diag.asm_offset + 32;
                            for (int k = 0; k < 24; ++k) {
                                g = (g << 1)
                                  | (decmode_diag.bits[base + k] & 1u);
                            }
                            decmode_golay_word24 = g;
                            uint16_t d12 = 0;
                            int errs = 0;
                            int rc = golay24_decode(g, &d12, &errs);
                            decmode_golay_rc     = rc;
                            decmode_golay_data12 = d12;
                            decmode_golay_errors = errs;
                        }
                        // Stage 8: CCSDS descrambler. Pack the
                        // first DECMODE_DESCR_CAP bytes after the
                        // Golay header (MSB-first within each byte),
                        // then XOR with the CCSDS table. If the
                        // Golay length came back smaller than the
                        // cap, only descramble that many.
                        decmode_descr_n = 0;
                        // decmode_descr_view_off intentionally
                        // persists — operator-driven byte scroll
                        // shouldn't snap back when the waterfall
                        // recomputes from a pan/zoom event. The
                        // renderer clamps it each frame against the
                        // new buffer length.
                        if (decmode_diag.asm_offset != (size_t) -1
                            && decmode_diag.bits != NULL) {
                            size_t bbase =
                                decmode_diag.asm_offset + 32 + 24;
                            size_t want = DECMODE_DESCR_CAP;
                            if (decmode_golay_rc == 0
                                && decmode_golay_data12 > 0
                                && (size_t) decmode_golay_data12
                                   < want) {
                                want = (size_t) decmode_golay_data12;
                            }
                            for (size_t i = 0; i < want; ++i) {
                                if (bbase + (i + 1) * 8
                                    > decmode_diag.n_strobes) break;
                                uint8_t b = 0;
                                for (int k = 0; k < 8; ++k) {
                                    b = (uint8_t)((b << 1)
                                        | (decmode_diag.bits[
                                              bbase + i * 8 + k] & 1u));
                                }
                                decmode_descr_scrambled[i] = b;
                                decmode_descr_descrambled[i] =
                                    (uint8_t)(b
                                              ^ DECMODE_CCSDS_TABLE[
                                                  i % DECMODE_CCSDS_TABLE_N]);
                                decmode_descr_n = i + 1;
                            }
                        }
                        // Stage 9: Reed-Solomon. Mirror ax100.c's
                        // pycsp path — left-zero-pad to RS_N, run
                        // rs_decode in place, keep both the
                        // pre-correction codeword and the
                        // post-correction codeword so the K panel
                        // can show which positions RS moved.
                        decmode_rs_errors = -2;
                        decmode_rs_pad_len = 0;
                        if (decmode_descr_n > (size_t) RS_NROOTS) {
                            size_t use_n = (decmode_descr_n > (size_t) RS_N)
                                ? (size_t) RS_N : decmode_descr_n;
                            size_t pad = (size_t) RS_N - use_n;
                            memset(decmode_rs_in, 0, pad);
                            memcpy(decmode_rs_in + pad,
                                   decmode_descr_descrambled,
                                   use_n);
                            memcpy(decmode_rs_out, decmode_rs_in,
                                   RS_N);
                            decmode_rs_pad_len = pad;
                            int rc_rs = rs_decode(decmode_rs_out,
                                                  decmode_rs_locs);
                            decmode_rs_errors = rc_rs;
                        }
                    }
                }
            }
        }
        // Box-info status line content.
        char box_info[256] = "";
        int  box_for_status = (boxes.selected >= 0) ? boxes.selected : hovered;
        if (box_for_status >= 0 && box_for_status < boxes.n) {
            const box_t *b = &boxes.items[box_for_status];
            double dt   = b->t1_s - b->t0_s;
            double dbw  = (b->f_hi_hz - b->f_lo_hz);
            // Pick fractional-second precision from the box DURATION:
            // a 10 ms box gets 4 decimals; a 10 µs box gets 7. The
            // duration field shows the same precision.
            int nd = decimals_for_step(fabs(dt) / 10.0);
            if (nd < 3) nd = 3;
            char ts0[32], ts1[32];
            fmt_mmss_ndec(b->t0_s, nd, ts0, sizeof ts0);
            fmt_mmss_ndec(b->t1_s, nd, ts1, sizeof ts1);
            snprintf(box_info, sizeof box_info,
                "box %d [%s]  t %s -> %s  (%.*f s)   f %+.0f -> %+.0f Hz  (%.2f kHz)",
                box_for_status, b->label,
                ts0, ts1, nd, dt,
                b->f_lo_hz, b->f_hi_hz, dbw / 1000.0);
        }

        // Deferred PDF export — fires once when P was pressed this
        // frame. Uses the time range and box-info we just computed.
        if (pdf_requested && iqb.samples != NULL) {
            char t0fn[32], t1fn[32];
            fmt_hhmmss_filename(wf_t_lo, t0fn, sizeof t0fn);
            fmt_hhmmss_filename(wf_t_hi, t1fn, sizeof t1fn);
            char pdf_path[1100];
            snprintf(pdf_path, sizeof pdf_path,
                     "%.900s.timeseries.%s_%s.pdf",
                     iq_path, t0fn, t1fn);
            int rc = pdf_write_waveform(pdf_path, &iqb,
                                        wf_t_lo, wf_t_hi,
                                        iq_path, box_info,
                                        iq_show_mode);
            if (rc == 0) {
                // Status bar is narrow — show just the file name, not
                // the full pdf_path (avoids snprintf truncation warning).
                const char *base = strrchr(pdf_path, '/');
                base = (base != NULL) ? base + 1 : pdf_path;
                snprintf(status, sizeof status, "wrote %.200s", base);
            } else {
                snprintf(status, sizeof status,
                    "PDF write failed");
            }
        }

        // Status bar — two lines now: top = box info, bottom = cursor
        // / keys / status messages.
        int bar_h = 2 * (STATUS_PT + 6);

        // Filter-state strip — sits just above the main status bar
        // when the filter is building or built, so its live readout
        // doesn't fight with the keys-help line below.
        int filter_strip_h = 0;
        if (filter_building || filter_built) {
            char fmsg[160];
            Color fcol;
            if (filter_building) {
                snprintf(fmsg, sizeof fmsg,
                    "filtering... %.0f %%  "
                    "(center=%.0f Hz, HPF=%.0f Hz, LPF=%.0f Hz)",
                    fctx.progress_pct * 100.0f,
                    filter_center_hz,
                    filter_lower_hz, filter_upper_hz);
                fcol = (Color){200, 200, 130, 255};
            } else {
                snprintf(fmsg, sizeof fmsg,
                    "waveform: %s  "
                    "(center=%.0f Hz, HPF=%.0f Hz, LPF=%.0f Hz)",
                    filter_show ? "FILTERED" : "raw",
                    filter_center_hz,
                    filter_lower_hz, filter_upper_hz);
                fcol = filter_show
                    ? (Color){170, 220, 255, 255}
                    : (Color){180, 180, 190, 255};
            }
            filter_strip_h = STATUS_PT + 8;
            DrawRectangle(0, sh - bar_h - filter_strip_h,
                          spec_screen_w, filter_strip_h,
                          (Color){0, 0, 0, 210});
            draw_text(fmsg, 6, sh - bar_h - filter_strip_h + 4,
                      STATUS_PT, fcol);
        }

        // Status bar runs only as wide as the spec area so it doesn't
        // sit under the decode side panel.
        DrawRectangle(0, sh - bar_h, spec_screen_w, bar_h, (Color){0, 0, 0, 210});
        if (box_info[0]) {
            draw_text(box_info, 6, sh - bar_h + 4, STATUS_PT,
                      (boxes.selected >= 0 && boxes.selected == box_for_status)
                        ? YELLOW : ORANGE);
        }
        char info[512];
        if (in_spec) {
            snprintf(info, sizeof info,
                "cursor t=%.3fs  f=%+.0f Hz  zoom=%.2fx  boxes=%d  hover=%d sel=%d  | drag=new pinch=zoom +/-=5x@cursor-anchor scroll=pan W=wave I=ch F=filter P=pdf T=label S=save L=load ,./<>=color R=auto-color 0=reset Del=del-hover Q=quit",
                cursor_t, cursor_f, (double) zoom,
                boxes.n, hovered, boxes.selected);
        } else {
            snprintf(info, sizeof info,
                "zoom=%.2fx  boxes=%d  | drag=new pinch=zoom +/-=5x@cursor-anchor scroll=pan W=wave I=ch F=filter P=pdf T=label S=save L=load ,./<>=color R=auto-color 0=reset Del=del-hover Q=quit",
                (double) zoom, boxes.n);
        }
        draw_text(info, 6, sh - bar_h + STATUS_PT + 8, STATUS_PT, LIGHTGRAY);
        if (status[0]) {
            int tw = measure_text(status, STATUS_PT);
            draw_text(status, sw - tw - 8,
                      sh - bar_h + STATUS_PT + 8, STATUS_PT, YELLOW);
        }

        // ----- waveform panel rendering (above the status bar) -----
        // Time range is derived from the spectrogram view above (see
        // wf_t_lo / wf_t_hi computed earlier this frame). The panel
        // has no independent zoom/pan — pinch & scroll on the
        // spectrogram move both views together.
        if (wf_open && iqb.samples != NULL) {
            // Background — only the area to the left of the decode panel
            // (if open) so the two panels don't overlap.
            int wf_right = spec_screen_w;
            DrawRectangle(0, wf_y0, wf_right, wf_panel_h, (Color){15, 15, 20, 230});
            DrawLine(0, wf_y0, wf_right, wf_y0, GRAY);

            // Layout inside the panel — bigger margins so the larger
            // TTF labels fit.
            int pL = 96, pR = 8, pT = 6, pB = 52;
            int plot_x0 = pL;
            int plot_x1 = wf_right - pR;
            int plot_y0 = wf_y0 + pT;
            int plot_y1 = sh - bar_h - pB;
            int plot_w  = plot_x1 - plot_x0;
            int plot_h  = plot_y1 - plot_y0;
            if (plot_w > 16 && plot_h > 16 && wf_t_hi > wf_t_lo) {
                // Measurement-drag input + render. LMB press inside the
                // plot rect anchors a TIME (not a screen-x), so a
                // pinch-zoom or pan mid-drag still reads the right
                // duration. The rectangle paints behind everything else
                // in the panel — drawn first, then the traces sit on
                // top. Mouse-up clears the drag.
                int in_wf_plot = (int) m.x >= plot_x0
                              && (int) m.x <  plot_x1
                              && (int) m.y >= plot_y0
                              && (int) m.y <  plot_y1;
                if (in_wf_plot && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                    double frac = ((double) m.x - plot_x0) / (double) plot_w;
                    wf_drag_t_anchor = wf_t_lo
                        + frac * (wf_t_hi - wf_t_lo);
                    wf_drag = 1;
                }
                if (wf_drag && IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
                    wf_drag = 0;
                }
                if (wf_drag) {
                    double frac_now = ((double) m.x - plot_x0)
                                    / (double) plot_w;
                    if (frac_now < 0.0) frac_now = 0.0;
                    if (frac_now > 1.0) frac_now = 1.0;
                    double t_now = wf_t_lo
                        + frac_now * (wf_t_hi - wf_t_lo);
                    double frac_anchor = (wf_drag_t_anchor - wf_t_lo)
                        / (wf_t_hi - wf_t_lo);
                    if (frac_anchor < 0.0) frac_anchor = 0.0;
                    if (frac_anchor > 1.0) frac_anchor = 1.0;
                    int x_anchor = plot_x0
                        + (int)(frac_anchor * plot_w + 0.5);
                    int x_now    = plot_x0
                        + (int)(frac_now    * plot_w + 0.5);
                    int x_lo = (x_anchor < x_now) ? x_anchor : x_now;
                    int x_hi = (x_anchor < x_now) ? x_now : x_anchor;
                    DrawRectangle(x_lo, plot_y0,
                                  x_hi - x_lo, plot_h,
                                  (Color){90, 110, 150, 70});
                    char dbuf[64];
                    fmt_duration_auto(t_now - wf_drag_t_anchor,
                                      dbuf, sizeof dbuf);
                    int dpt = 18;
                    int dw  = measure_text(dbuf, dpt);
                    int tx  = (x_lo + x_hi) / 2 - dw / 2;
                    int ty  = plot_y0 + 6;
                    if (tx < plot_x0)            tx = plot_x0;
                    if (tx + dw > plot_x1)       tx = plot_x1 - dw;
                    DrawRectangle(tx - 4, ty - 2, dw + 8, dpt + 4,
                                  (Color){0, 0, 0, 180});
                    draw_text(dbuf, tx, ty, dpt,
                              (Color){240, 240, 250, 255});
                }

                // Sample bounds.
                int64_t i_lo = (int64_t)(wf_t_lo * iqb.samp_rate);
                int64_t i_hi = (int64_t)(wf_t_hi * iqb.samp_rate);
                if (i_lo < 0) i_lo = 0;
                if (i_hi > (int64_t) iqb.n_pairs) i_hi = iqb.n_pairs;
                int64_t n_pairs_vis = i_hi - i_lo;

                // When the F-toggle is on and the worker has finished,
                // read from the filtered IQ buffer; otherwise use the
                // raw one. The two buffers are identical-length so the
                // index math stays the same.
                const int16_t *display_iq =
                    (filter_show && filter_built && filtered_iq != NULL)
                    ? filtered_iq : iqb.samples;

                // Mode flags + sub-rect layout. iq_h/phase_h = 0 means
                // that channel isn't shown this frame. Split mode puts
                // phase on top, I/Q on bottom with a small visual gap.
                int show_iq    = (iq_show_mode != 3);
                int show_i     = show_iq && (iq_show_mode != 2);
                int show_q     = show_iq && (iq_show_mode != 1);
                int show_phase = (iq_show_mode == 3 || iq_show_mode == 4);
                int split      = (iq_show_mode == 4);
                int iq_y0_sub    = plot_y0;
                int iq_y1_sub    = plot_y1;
                int phase_y0_sub = plot_y0;
                int phase_y1_sub = plot_y1;
                if (split) {
                    int gap = 6;
                    int mid = plot_y0 + plot_h / 2;
                    phase_y0_sub = plot_y0;
                    phase_y1_sub = mid - gap / 2;
                    iq_y0_sub    = mid + gap / 2;
                    iq_y1_sub    = plot_y1;
                } else if (show_phase) {
                    // phase-only: phase takes the whole plot, iq area
                    // is zero-height so the iq render block skips.
                    iq_y0_sub = iq_y1_sub = plot_y0;
                } else {
                    phase_y0_sub = phase_y1_sub = plot_y0;
                }
                int iq_h_sub    = iq_y1_sub    - iq_y0_sub;
                int phase_h_sub = phase_y1_sub - phase_y0_sub;

                // Find the magnitude scale (auto). Take the max |I|,|Q|.
                // Skip the scan when IQ isn't shown — saves work in
                // phase-only mode.
                int amp_max = 1;
                if (show_iq && n_pairs_vis > 0) {
                    int64_t step_s = (n_pairs_vis > 4096)
                        ? n_pairs_vis / 2048 : 1;
                    for (int64_t k = i_lo; k < i_hi; k += step_s) {
                        int I = display_iq[k * 2 + 0];
                        int Q = display_iq[k * 2 + 1];
                        if (abs(I) > amp_max) amp_max = abs(I);
                        if (abs(Q) > amp_max) amp_max = abs(Q);
                    }
                }
                if (amp_max < 32) amp_max = 32;

                const int AMP_PT = 19;
                // IQ y-axis (centred on 0; ticks at ±amp_max/2 and ±amp_max).
                if (show_iq && iq_h_sub > 8) {
                    int iq_mid_y = iq_y0_sub + iq_h_sub / 2;
                    DrawLine(plot_x0, iq_mid_y, plot_x1, iq_mid_y,
                             (Color){50, 50, 60, 255});
                    for (int k = -2; k <= 2; ++k) {
                        int y = iq_mid_y - (k * iq_h_sub / 4);
                        DrawLine(plot_x0 - 4, y, plot_x0, y, GRAY);
                        char buf[24];
                        snprintf(buf, sizeof buf, "%+d",
                                 (int)(k * amp_max / 2));
                        int tw = measure_text(buf, AMP_PT);
                        draw_text(buf, plot_x0 - 6 - tw, y - AMP_PT/2,
                                  AMP_PT, GRAY);
                    }
                }
                // Phase y-axis (centred on 0; ticks at -π, -π/2, 0, +π/2, +π).
                if (show_phase && phase_h_sub > 8) {
                    int ph_mid_y = phase_y0_sub + phase_h_sub / 2;
                    DrawLine(plot_x0, ph_mid_y, plot_x1, ph_mid_y,
                             (Color){50, 50, 60, 255});
                    const char *plabels[5] = {"-π", "-π/2", "0", "+π/2", "+π"};
                    for (int k = -2; k <= 2; ++k) {
                        int y = ph_mid_y - (k * phase_h_sub / 4);
                        DrawLine(plot_x0 - 4, y, plot_x0, y, GRAY);
                        int tw = measure_text(plabels[k + 2], AMP_PT);
                        draw_text(plabels[k + 2],
                                  plot_x0 - 6 - tw, y - AMP_PT/2,
                                  AMP_PT, GRAY);
                    }
                }

                // Time-axis ticks (drawn at the bottom of the panel).
                double span = wf_t_hi - wf_t_lo;
                double raw_step = span / 6.0;
                double mag = pow(10.0, floor(log10(raw_step)));
                double mul = raw_step / mag;
                if      (mul < 1.5) mul = 1.0;
                else if (mul < 3.5) mul = 2.0;
                else if (mul < 7.5) mul = 5.0;
                else                mul = 10.0;
                double step = mul * mag;
                double t0_aligned = ceil(wf_t_lo / step) * step;
                const int T_PT = 19;
                // Fractional-second precision scales with the tick
                // step so labels at deep zoom can read sub-millisecond.
                int t_nd = decimals_for_step(step);
                for (double t = t0_aligned; t <= wf_t_hi + 0.5*step; t += step) {
                    int x = plot_x0
                          + (int)((t - wf_t_lo) / span * plot_w);
                    if (x < plot_x0 || x > plot_x1) continue;
                    DrawLine(x, plot_y1, x, plot_y1 + 6, GRAY);
                    char buf[40];
                    fmt_mmss_ndec(t, t_nd, buf, sizeof buf);
                    int tw = measure_text(buf, T_PT);
                    draw_text(buf, x - tw/2, plot_y1 + 8, T_PT, GRAY);
                }

                // Draw the waveforms. Decimate when n_pairs_vis > plot_w
                // by taking per-column min/max (so individual bit
                // transitions stay visible). Phase rendering uses
                // atan2(Q,I) into [-π,+π]; at the wrap (±π) the sparse
                // line plot lifts the pen and the dense min/max paints
                // a column-tall bar (visually = "phase wrapped here").
                const Color COLOR_I     = {80, 200, 220, 255};
                const Color COLOR_Q     = {200, 90, 220, 200};
                const Color COLOR_I_D   = {80, 200, 220, 220};
                const Color COLOR_Q_D   = {200, 90, 220, 160};
                const Color COLOR_PHASE = {255, 180, 80, 255};
                const Color COLOR_PHASE_D = {255, 180, 80, 200};
                if (n_pairs_vis > 0
                    && (n_pairs_vis <= (int64_t) plot_w * 2)) {
                    // Sparse: line-to-line plot.
                    int prev_xi = -1, prev_yi = 0, prev_xq = -1, prev_yq = 0;
                    int prev_xph = -1, prev_yph = 0;
                    double prev_phi = 0.0;
                    int iq_mid_y    = iq_y0_sub + iq_h_sub / 2;
                    int ph_mid_y    = phase_y0_sub + phase_h_sub / 2;
                    float y_scale = (iq_h_sub > 0)
                        ? (float) iq_h_sub / 2.0f / (float) amp_max
                        : 0.0f;
                    float ph_scale = (phase_h_sub > 0)
                        ? (float) phase_h_sub / 2.0f / (float) M_PI
                        : 0.0f;
                    for (int64_t k = i_lo; k < i_hi; ++k) {
                        double t = (double) k / iqb.samp_rate;
                        int x = plot_x0
                              + (int)((t - wf_t_lo) / span * plot_w);
                        if (x < plot_x0 || x > plot_x1) continue;
                        int I = display_iq[k * 2 + 0];
                        int Q = display_iq[k * 2 + 1];
                        if (show_iq) {
                            int yI = iq_mid_y - (int)(I * y_scale);
                            int yQ = iq_mid_y - (int)(Q * y_scale);
                            if (show_q && prev_xq >= 0)
                                DrawLine(prev_xq, prev_yq, x, yQ, COLOR_Q);
                            if (show_i && prev_xi >= 0)
                                DrawLine(prev_xi, prev_yi, x, yI, COLOR_I);
                            prev_xi = x; prev_yi = yI;
                            prev_xq = x; prev_yq = yQ;
                        }
                        if (show_phase) {
                            double phi = atan2((double) Q, (double) I);
                            int yPh = ph_mid_y - (int)(phi * ph_scale);
                            if (prev_xph >= 0
                                && fabs(phi - prev_phi) <= M_PI)
                                DrawLine(prev_xph, prev_yph,
                                         x, yPh, COLOR_PHASE);
                            prev_xph = x; prev_yph = yPh; prev_phi = phi;
                        }
                    }
                } else if (n_pairs_vis > 0) {
                    // Dense: per-column min/max.
                    int iq_mid_y    = iq_y0_sub + iq_h_sub / 2;
                    int ph_mid_y    = phase_y0_sub + phase_h_sub / 2;
                    float y_scale = (iq_h_sub > 0)
                        ? (float) iq_h_sub / 2.0f / (float) amp_max
                        : 0.0f;
                    float ph_scale = (phase_h_sub > 0)
                        ? (float) phase_h_sub / 2.0f / (float) M_PI
                        : 0.0f;
                    for (int x = 0; x < plot_w; ++x) {
                        int64_t s0 = i_lo + (int64_t) x * n_pairs_vis / plot_w;
                        int64_t s1 = i_lo + (int64_t)(x+1) * n_pairs_vis / plot_w;
                        if (s1 <= s0) s1 = s0 + 1;
                        if (s1 > i_hi) s1 = i_hi;
                        int iMin =  INT_MAX, iMax = INT_MIN;
                        int qMin =  INT_MAX, qMax = INT_MIN;
                        double phMin =  1e9, phMax = -1e9;
                        for (int64_t k = s0; k < s1; ++k) {
                            int I = display_iq[k * 2 + 0];
                            int Q = display_iq[k * 2 + 1];
                            if (show_iq) {
                                if (I < iMin) iMin = I;
                                if (I > iMax) iMax = I;
                                if (Q < qMin) qMin = Q;
                                if (Q > qMax) qMax = Q;
                            }
                            if (show_phase) {
                                double phi = atan2((double) Q, (double) I);
                                if (phi < phMin) phMin = phi;
                                if (phi > phMax) phMax = phi;
                            }
                        }
                        int xpx = plot_x0 + x;
                        if (show_iq && iq_h_sub > 0) {
                            if (show_q)
                                DrawLine(xpx, iq_mid_y - (int)(qMax * y_scale),
                                         xpx, iq_mid_y - (int)(qMin * y_scale),
                                         COLOR_Q_D);
                            if (show_i)
                                DrawLine(xpx, iq_mid_y - (int)(iMax * y_scale),
                                         xpx, iq_mid_y - (int)(iMin * y_scale),
                                         COLOR_I_D);
                        }
                        if (show_phase && phase_h_sub > 0 && phMin <= phMax) {
                            DrawLine(xpx, ph_mid_y - (int)(phMax * ph_scale),
                                     xpx, ph_mid_y - (int)(phMin * ph_scale),
                                     COLOR_PHASE_D);
                        }
                    }
                }

                // Border around plot (+ a separator line between the
                // two sub-areas in split mode so the eye doesn't try to
                // continue a phase trace into the I/Q area).
                DrawRectangleLines(plot_x0, plot_y0,
                                   plot_w, plot_h, DARKGRAY);
                if (split) {
                    int sep_y = phase_y1_sub
                              + (iq_y0_sub - phase_y1_sub) / 2;
                    DrawLine(plot_x0, sep_y, plot_x1, sep_y,
                             (Color){70, 70, 80, 255});
                }

                // Cursor indicator: bright full-height line through
                // the plot, with arrowheads at top and bottom, marking
                // where the spectrogram cursor's time falls inside the
                // panel. The line has a dark backing stroke so it's
                // legible over both the cyan/violet traces and the
                // dark plot background. Drawn while the cursor is in
                // the spec AND its time is inside the panel window.
                if (in_spec
                    && cursor_t >= wf_t_lo - 1e-12
                    && cursor_t <= wf_t_hi + 1e-12) {
                    int cx = plot_x0
                           + (int)((cursor_t - wf_t_lo) / span * plot_w);
                    if (cx >= plot_x0 && cx <= plot_x1) {
                        // Dark backing — 1 px wider than the bright
                        // line on each side so it shows up against
                        // anything underneath.
                        DrawLine(cx - 1, plot_y0, cx - 1, plot_y1 + 1, BLACK);
                        DrawLine(cx + 1, plot_y0, cx + 1, plot_y1 + 1, BLACK);
                        // Bright opaque guide line.
                        DrawLine(cx, plot_y0, cx, plot_y1 + 1, YELLOW);
                        // Bottom arrowhead — apex inside the plot, base
                        // on the axis. 12 px × 14 px.
                        {
                            Vector2 apex = {(float) cx, (float)(plot_y1 - 1)};
                            Vector2 bl = {(float)(cx - 7), (float)(plot_y1 + 11)};
                            Vector2 br = {(float)(cx + 7), (float)(plot_y1 + 11)};
                            DrawTriangle(apex, br, bl, YELLOW);
                            DrawTriangleLines(apex, br, bl, BLACK);
                        }
                        // Top arrowhead — apex pointing down into the
                        // plot, base just above the top border.
                        {
                            Vector2 apex = {(float) cx, (float)(plot_y0 + 1)};
                            Vector2 bl = {(float)(cx - 7), (float)(plot_y0 - 11)};
                            Vector2 br = {(float)(cx + 7), (float)(plot_y0 - 11)};
                            DrawTriangle(apex, bl, br, YELLOW);
                            DrawTriangleLines(apex, bl, br, BLACK);
                        }
                    }
                }
                // When the mouse is in the panel itself, draw a small
                // crosshair at the mouse position so the operator can
                // pinpoint exactly which sample they're over (the OS
                // arrow tip is hard to read against the I/Q traces).
                if (in_panel_now
                    && (int) m.x >= plot_x0 && (int) m.x <= plot_x1
                    && (int) m.y >= plot_y0 && (int) m.y <= plot_y1) {
                    int cx = (int) m.x;
                    int cy = (int) m.y;
                    const int arm = 12;
                    const int gap = 3;
                    DrawLine(cx - arm - 1, cy, cx - gap + 1, cy, BLACK);
                    DrawLine(cx + gap - 1, cy, cx + arm + 1, cy, BLACK);
                    DrawLine(cx, cy - arm - 1, cx, cy - gap + 1, BLACK);
                    DrawLine(cx, cy + gap - 1, cx, cy + arm + 1, BLACK);
                    DrawLine(cx - arm, cy, cx - gap, cy, YELLOW);
                    DrawLine(cx + gap, cy, cx + arm, cy, YELLOW);
                    DrawLine(cx, cy - arm, cx, cy - gap, YELLOW);
                    DrawLine(cx, cy + gap, cx, cy + arm, YELLOW);
                    DrawRectangle(cx - 1, cy - 1, 3, 3, YELLOW);
                }

                // Title (span + sample count). Color matches body text.
                const int TITLE_PT = 17;
                const char *chans =
                    (iq_show_mode == 0) ? "I cyan  Q violet"
                  : (iq_show_mode == 1) ? "I only (cyan)"
                  : (iq_show_mode == 2) ? "Q only (violet)"
                  : (iq_show_mode == 3) ? "phase (orange, ±π)"
                  :                       "phase top / I+Q bottom";
                char title[256];
                snprintf(title, sizeof title,
                    "waveform  span=%.3f s  (%lld samples)   %s   [i=cycle]",
                    span, (long long) n_pairs_vis, chans);
                draw_text(title, plot_x0 + 6, plot_y0 + 2, TITLE_PT,
                          (Color){200, 200, 220, 255});
            }
        }

        // ----- decode-mode panel rendering (K, mutually exclusive
        // with the waveform panel; same bottom slot but taller) -----
        if (decmode_open) {
            int dm_y0    = sh - decmode_panel_h;
            int dm_right = spec_screen_w;
            DrawRectangle(0, dm_y0, dm_right, decmode_panel_h,
                          (Color){15, 15, 20, 230});
            DrawLine(0, dm_y0, dm_right, dm_y0, GRAY);

            // Title strip — stage label + nav hint.
            int title_h = 30;
            char title_str[256];
            snprintf(title_str, sizeof title_str,
                "[stage %d/%d]  %s   ( [ / ] navigate )",
                decmode_stage + 1, DECMODE_N_STAGES,
                DECMODE_STAGE_NAMES[decmode_stage]);
            draw_text(title_str, 12, dm_y0 + 6, 18,
                      (Color){210, 210, 230, 255});
            DrawLine(0, dm_y0 + title_h, dm_right, dm_y0 + title_h,
                     (Color){40, 40, 50, 255});

            // Inner plot rect — pT below the title bar, leaving room
            // at the bottom for tick labels above the status bar.
            int pL = 96, pR = 8, pT = title_h + 8, pB = 52;
            int plot_x0 = pL;
            int plot_x1 = dm_right - pR;
            int plot_y0 = dm_y0 + pT;
            int plot_y1 = sh - bar_h - pB;
            int plot_w  = plot_x1 - plot_x0;
            int plot_h  = plot_y1 - plot_y0;

            int has_data = decmode_have_result
                        && decmode_diag.n_fm > 0
                        && wf_t_hi > wf_t_lo;
            if (plot_w > 16 && plot_h > 16 && has_data) {
                const int AMP_PT = 17;
                const int T_PT   = 17;
                int sps   = decmode_input_sps;
                double samp_rate_d = (double) iqb.samp_rate;
                // All stages share the same time x-axis (the
                // visible spectrogram window). Stage 6 additionally
                // gets a bit-number axis below the time labels.
                double span_s = wf_t_hi - wf_t_lo;

                // Stage 4 (eye) wants a 1:1 plot rect — collapse the
                // body to a square subrect on the left of the body
                // area. Other stages use the full body width.
                int body_x0 = plot_x0, body_x1 = plot_x1;
                int body_y0 = plot_y0, body_y1 = plot_y1;
                int body_w  = plot_w,  body_h  = plot_h;
                if (decmode_stage == 4) {
                    int side = (body_h < body_w) ? body_h : body_w;
                    body_w = side; body_h = side;
                    body_x0 = plot_x0;
                    body_x1 = body_x0 + body_w;
                    body_y0 = plot_y0;
                    body_y1 = body_y0 + body_h;
                }

                // ------ Stages 0..3, 5: y-axis (amp) + time x-axis ------
                if (decmode_stage <= 3 || decmode_stage == 5) {
                    // Auto-scale amplitude from the stage's buffer.
                    double amax = 1.0;
                    if (decmode_stage == 0) {
                        int64_t i_lo = (int64_t)(wf_t_lo * iqb.samp_rate);
                        int64_t i_hi = (int64_t)(wf_t_hi * iqb.samp_rate);
                        if (i_lo < 0) i_lo = 0;
                        if (i_hi > (int64_t) iqb.n_pairs) i_hi = iqb.n_pairs;
                        int64_t step = ((i_hi - i_lo) > 4096)
                            ? (i_hi - i_lo) / 2048 : 1;
                        for (int64_t k = i_lo; k < i_hi; k += step) {
                            double a = fabs((double) iqb.samples[k*2+0]);
                            double b = fabs((double) iqb.samples[k*2+1]);
                            if (a > amax) amax = a;
                            if (b > amax) amax = b;
                        }
                    } else if (decmode_stage == 1) {
                        size_t n = decmode_diag.n_pairs_lpf;
                        // Auto-scale off the visible portion only.
                        {
                            int64_t n_vis = (int64_t)(span_s * samp_rate_d);
                            if (n_vis < 0) n_vis = 0;
                            if ((int64_t) n > n_vis) n = (size_t) n_vis;
                        }
                        size_t step = (n > 4096) ? n / 2048 : 1;
                        for (size_t k = 0; k < n; k += step) {
                            double a = fabs((double) decmode_diag.i_lpf[k]);
                            double b = fabs((double) decmode_diag.q_lpf[k]);
                            if (a > amax) amax = a;
                            if (b > amax) amax = b;
                        }
                    } else {
                        const float *src = (decmode_stage == 2)
                            ? decmode_diag.fm : decmode_diag.mf;
                        size_t n = (decmode_stage == 2)
                            ? decmode_diag.n_fm : decmode_diag.n_mf;
                        // Auto-scale off the visible portion only.
                        {
                            int64_t n_vis = (int64_t)(span_s * samp_rate_d);
                            if (n_vis < 0) n_vis = 0;
                            if ((int64_t) n > n_vis) n = (size_t) n_vis;
                        }
                        size_t step = (n > 4096) ? n / 2048 : 1;
                        for (size_t k = 0; k < n; k += step) {
                            double a = fabs((double) src[k]);
                            if (a > amax) amax = a;
                        }
                        if (decmode_stage == 5) amax = 2.0; // ±π/20 rails fit
                    }
                    if (amax < 1e-6) amax = 1e-6;

                    int mid_y = body_y0 + body_h / 2;
                    float y_scale = (float)(body_h * 0.5 / amax);
                    DrawLine(body_x0, mid_y, body_x1, mid_y,
                             (Color){50, 50, 60, 255});
                    for (int k = -2; k <= 2; ++k) {
                        int y = mid_y - (k * body_h / 4);
                        DrawLine(body_x0 - 4, y, body_x0, y, GRAY);
                        char buf[24];
                        if (decmode_stage <= 1) {
                            snprintf(buf, sizeof buf, "%+d",
                                (int)(k * amax / 2));
                        } else {
                            snprintf(buf, sizeof buf, "%+.2f",
                                (double)(k * amax / 2.0));
                        }
                        int tw = measure_text(buf, AMP_PT);
                        draw_text(buf, body_x0 - 6 - tw, y - AMP_PT/2,
                                  AMP_PT, GRAY);
                    }

                    // Plot the data. For dense data fall back to
                    // per-column min/max; for sparse, line-to-line.
                    if (decmode_stage == 0 || decmode_stage == 1) {
                        // Two-trace (I + Q)
                        Color colI = {80, 200, 220, 255};
                        Color colQ = {200, 90, 220, 200};
                        const int16_t *src_iq16 = NULL;
                        const float *src_i = NULL, *src_q = NULL;
                        size_t n_pairs_total = 0;
                        int64_t i_lo_idx = 0, i_hi_idx = 0;
                        double samp_rate_data = samp_rate_d;
                        if (decmode_stage == 0) {
                            src_iq16 = iqb.samples;
                            n_pairs_total = iqb.n_pairs;
                            i_lo_idx = (int64_t)(wf_t_lo * iqb.samp_rate);
                            i_hi_idx = (int64_t)(wf_t_hi * iqb.samp_rate);
                            if (i_lo_idx < 0) i_lo_idx = 0;
                            if (i_hi_idx > (int64_t) n_pairs_total)
                                i_hi_idx = n_pairs_total;
                        } else {
                            src_i = decmode_diag.i_lpf;
                            src_q = decmode_diag.q_lpf;
                            n_pairs_total = decmode_diag.n_pairs_lpf;
                            i_lo_idx = 0;
                            // Clip to visible time so the dense binning
                            // doesn't compress the >=0.3 s decoder
                            // extension into the K panel.
                            int64_t n_vis = (int64_t)(span_s * samp_rate_data);
                            if (n_vis < 0) n_vis = 0;
                            if (n_vis > (int64_t) n_pairs_total)
                                n_vis = (int64_t) n_pairs_total;
                            i_hi_idx = n_vis;
                        }
                        int64_t nvis = i_hi_idx - i_lo_idx;
                        if (nvis > 0) {
                            if (nvis <= (int64_t) body_w * 2) {
                                int prev_x = -1, prev_yI = 0, prev_yQ = 0;
                                for (int64_t k = i_lo_idx; k < i_hi_idx; ++k) {
                                    double t = (double) k / samp_rate_data;
                                    double t_show = (decmode_stage == 0)
                                        ? t : (wf_t_lo + (double)(k - i_lo_idx) / samp_rate_data);
                                    int x = body_x0
                                          + (int)((t_show - wf_t_lo) / span_s * body_w);
                                    if (x < body_x0 || x > body_x1) continue;
                                    double I = (decmode_stage == 0)
                                        ? src_iq16[k*2+0] : src_i[k];
                                    double Q = (decmode_stage == 0)
                                        ? src_iq16[k*2+1] : src_q[k];
                                    int yI = mid_y - (int)(I * y_scale);
                                    int yQ = mid_y - (int)(Q * y_scale);
                                    if (prev_x >= 0) {
                                        DrawLine(prev_x, prev_yQ, x, yQ, colQ);
                                        DrawLine(prev_x, prev_yI, x, yI, colI);
                                    }
                                    prev_x = x; prev_yI = yI; prev_yQ = yQ;
                                }
                            } else {
                                for (int x = 0; x < body_w; ++x) {
                                    int64_t s0 = i_lo_idx + (int64_t) x * nvis / body_w;
                                    int64_t s1 = i_lo_idx + (int64_t)(x+1) * nvis / body_w;
                                    if (s1 <= s0) s1 = s0 + 1;
                                    if (s1 > i_hi_idx) s1 = i_hi_idx;
                                    double iMin =  1e30, iMax = -1e30;
                                    double qMin =  1e30, qMax = -1e30;
                                    for (int64_t k = s0; k < s1; ++k) {
                                        double I = (decmode_stage == 0)
                                            ? src_iq16[k*2+0] : src_i[k];
                                        double Q = (decmode_stage == 0)
                                            ? src_iq16[k*2+1] : src_q[k];
                                        if (I < iMin) iMin = I;
                                        if (I > iMax) iMax = I;
                                        if (Q < qMin) qMin = Q;
                                        if (Q > qMax) qMax = Q;
                                    }
                                    int xpx = body_x0 + x;
                                    DrawLine(xpx, mid_y - (int)(qMax * y_scale),
                                             xpx, mid_y - (int)(qMin * y_scale),
                                             (Color){200,90,220,160});
                                    DrawLine(xpx, mid_y - (int)(iMax * y_scale),
                                             xpx, mid_y - (int)(iMin * y_scale),
                                             (Color){80,200,220,220});
                                }
                            }
                        }
                    } else {
                        // Single-trace (FM disc, MF, or slicer underlay)
                        const float *src = (decmode_stage == 2)
                            ? decmode_diag.fm : decmode_diag.mf;
                        size_t n = (decmode_stage == 2)
                            ? decmode_diag.n_fm : decmode_diag.n_mf;
                        // Clip the buffer to the visible time span so
                        // the dense renderer doesn't squash the >=0.3 s
                        // decoder extension into the K panel.
                        {
                            int64_t n_vis = (int64_t)(span_s * samp_rate_d);
                            if (n_vis < 0) n_vis = 0;
                            if ((int64_t) n > n_vis) n = (size_t) n_vis;
                        }
                        Color col = (decmode_stage == 2)
                            ? (Color){180, 220, 130, 255}
                            : (Color){255, 200, 80, 255};
                        // Stage 5 overlays bit cells on the MF.
                        if (n > 0) {
                            if ((int64_t) n <= (int64_t) body_w * 2) {
                                int prev_x = -1, prev_y = 0;
                                for (size_t k = 0; k < n; ++k) {
                                    double t = wf_t_lo
                                        + (double) k / samp_rate_d;
                                    int x = body_x0
                                          + (int)((t - wf_t_lo) / span_s * body_w);
                                    if (x < body_x0 || x > body_x1) continue;
                                    int y = mid_y - (int)(src[k] * y_scale);
                                    if (prev_x >= 0)
                                        DrawLine(prev_x, prev_y, x, y, col);
                                    prev_x = x; prev_y = y;
                                }
                            } else {
                                for (int x = 0; x < body_w; ++x) {
                                    int64_t s0 = (int64_t) x * (int64_t) n / body_w;
                                    int64_t s1 = (int64_t)(x+1) * (int64_t) n / body_w;
                                    if (s1 <= s0) s1 = s0 + 1;
                                    if (s1 > (int64_t) n) s1 = n;
                                    double mn = 1e30, mx = -1e30;
                                    for (int64_t k = s0; k < s1; ++k) {
                                        double v = src[k];
                                        if (v < mn) mn = v;
                                        if (v > mx) mx = v;
                                    }
                                    int xpx = body_x0 + x;
                                    DrawLine(xpx, mid_y - (int)(mx * y_scale),
                                             xpx, mid_y - (int)(mn * y_scale),
                                             col);
                                }
                            }
                        }
                        // Stage 5: overlay bit cells at the strobe
                        // positions, color-coded.
                        if (decmode_stage == 5
                            && decmode_diag.bits != NULL) {
                            size_t ns = decmode_diag.n_strobes;
                            double samp_per_sym = (double) sps;
                            for (size_t k = 0; k < ns; ++k) {
                                double mf_idx = decmode_diag.strobe_t[k];
                                double t = wf_t_lo
                                    + mf_idx / samp_rate_d;
                                int x = body_x0
                                      + (int)((t - wf_t_lo) / span_s * body_w);
                                if (x < body_x0 || x > body_x1) continue;
                                int up = decmode_diag.bits[k] ? 1 : 0;
                                int y_rail = up
                                    ? (mid_y - (int)(0.157f * 2.0 * y_scale))
                                    : (mid_y + (int)(0.157f * 2.0 * y_scale));
                                DrawCircle(x, y_rail, 2,
                                    up ? (Color){120, 220, 120, 220}
                                       : (Color){220, 120, 120, 220});
                            }
                            (void) samp_per_sym;
                        }
                    }
                    // Bit-by-bit ASM comparison strip for the
                    // phase / post-discriminator stages where
                    // individual bits manifest visually (FM disc,
                    // matched filter, slicer). Stages 0/1 (raw IQ)
                    // skip this — they show amplitude, not phase.
                    if (decmode_stage == 2 || decmode_stage == 3
                        || decmode_stage == 5) {
                        int strip_top = body_y1 - 2 * 14 - 2 - 4;
                        draw_asm_bit_strip(
                            body_x0, body_x1, body_w,
                            wf_t_lo, span_s, strip_top,
                            decmode_diag.bits,
                            decmode_diag.n_strobes,
                            decmode_diag.asm_offset,
                            decmode_asm_abs_time_s,
                            decmode_bit_period_s,
                            AMP_PT - 4);
                    }
                }

                // ------ Stage 4: eye diagram ------
                else if (decmode_stage == 4) {
                    // Centre line + ±1 reference rails
                    int mid_y = body_y0 + body_h / 2;
                    float y_scale = (float)(body_h * 0.45);
                    DrawLine(body_x0, mid_y, body_x1, mid_y,
                             (Color){50, 50, 60, 255});
                    DrawLine(body_x0, mid_y - (int)(y_scale * 0.157f * 2),
                             body_x1, mid_y - (int)(y_scale * 0.157f * 2),
                             (Color){80, 80, 100, 180});
                    DrawLine(body_x0, mid_y + (int)(y_scale * 0.157f * 2),
                             body_x1, mid_y + (int)(y_scale * 0.157f * 2),
                             (Color){80, 80, 100, 180});

                    // Each strobe defines a symbol centre; draw a
                    // polyline from t_centre - sps to t_centre + sps
                    // (a 2-symbol window). Alpha-blend so high-density
                    // areas (the eye opening) brighten visibly.
                    const Color eye_col = {255, 180, 80, 64};
                    size_t ns = decmode_diag.n_strobes;
                    const float *mf = decmode_diag.mf;
                    size_t mf_n = decmode_diag.n_mf;
                    double sps_d = (double) sps;
                    if (ns > 0 && mf_n > 0) {
                        for (size_t k = 0; k < ns; ++k) {
                            double centre = decmode_diag.strobe_t[k];
                            double i0d = centre - sps_d;
                            double i1d = centre + sps_d;
                            if (i0d < 0.0) i0d = 0.0;
                            if (i1d >= (double) mf_n) i1d = (double) mf_n - 1.0;
                            int i0 = (int) i0d;
                            int i1 = (int) i1d;
                            if (i1 <= i0) continue;
                            int prev_x = -1, prev_y = 0;
                            for (int j = i0; j <= i1; ++j) {
                                double rel = ((double) j - centre) / sps_d;
                                // rel in [-1, +1] maps to body_x0..body_x1
                                int x = body_x0
                                      + (int)(((rel + 1.0) * 0.5) * body_w);
                                int y = mid_y - (int)(mf[j] * y_scale * 0.5);
                                if (prev_x >= 0)
                                    DrawLine(prev_x, prev_y, x, y, eye_col);
                                prev_x = x; prev_y = y;
                            }
                        }
                    }
                    // Y-axis labels at ±1 and 0
                    char buf[16];
                    snprintf(buf, sizeof buf, "+1");
                    draw_text(buf, body_x0 - 22,
                              mid_y - (int)(y_scale * 0.5) - AMP_PT/2,
                              AMP_PT, GRAY);
                    snprintf(buf, sizeof buf, "0");
                    draw_text(buf, body_x0 - 12, mid_y - AMP_PT/2,
                              AMP_PT, GRAY);
                    snprintf(buf, sizeof buf, "-1");
                    draw_text(buf, body_x0 - 22,
                              mid_y + (int)(y_scale * 0.5) - AMP_PT/2,
                              AMP_PT, GRAY);
                }

                // ------ Stage 6: ASM Hamming-distance trace ------
                else if (decmode_stage == 6) {
                    size_t n_bits = decmode_diag.n_strobes;
                    size_t n_h = (n_bits >= 32) ? (n_bits - 31) : 0;
                    int y_top = body_y0 + 6;
                    int y_bot = body_y1 - 6;
                    int h_axis = y_bot - y_top;
                    // Y axis: Hamming distance 0..32, with the accept
                    // threshold (4) drawn as a red line.
                    for (int k = 0; k <= 4; ++k) {
                        int hd = k * 8;  // ticks at 0, 8, 16, 24, 32
                        int y = y_bot
                              - (int)((double) hd / 32.0 * (double) h_axis);
                        DrawLine(body_x0 - 4, y, body_x0, y, GRAY);
                        char buf[16];
                        snprintf(buf, sizeof buf, "%d", hd);
                        int tw = measure_text(buf, AMP_PT);
                        draw_text(buf, body_x0 - 6 - tw, y - AMP_PT/2,
                                  AMP_PT, GRAY);
                    }
                    int y_thr = y_bot
                              - (int)((double) 4 / 32.0 * (double) h_axis);
                    DrawLine(body_x0, y_thr, body_x1, y_thr,
                             (Color){220, 80, 80, 200});
                    draw_text("accept (≤4)", body_x1 - 100,
                              y_thr + 4, AMP_PT,
                              (Color){220, 80, 80, 220});
                    // Trace — x is the TIME of each Hamming-trace
                    // strobe, so the ASM dip sits at the same time
                    // coordinate as the matching IQ feature in
                    // stages 0..5. strobe_t is in MF-sample units;
                    // the +30 (LPF) + 1 (discrim) + sps-1 (MF) offset
                    // shifts MF-index ↔ IQ-index, but for visual
                    // alignment with the spectrogram a sub-millisecond
                    // shift is invisible so we drop it.
                    if (n_h > 0 && decmode_diag.strobe_t != NULL) {
                        const uint8_t *h = decmode_diag.asm_hamming;
                        const double *st = decmode_diag.strobe_t;
                        double sr = samp_rate_d;
                        // Clip n_h to strobes that fall inside the
                        // visible time span. strobe_t is monotonic, so
                        // a linear scan from the right gives the count.
                        // Without this, the >=0.3 s decoder extension
                        // would push the trace into the dense path even
                        // when only a few strobes are visible, leaving
                        // the panel almost empty at high zoom.
                        double t_lim = sr * span_s;
                        while (n_h > 0 && st[n_h - 1] > t_lim) --n_h;
                        // Sparse: line-to-line plot.
                        if ((int64_t) n_h <= (int64_t) body_w * 2) {
                            int prev_x = -1, prev_y = 0;
                            for (size_t k = 0; k < n_h; ++k) {
                                int x = body_x0
                                      + (int)(st[k] / (sr * span_s) * body_w);
                                if (x < body_x0) { prev_x = -1; continue; }
                                if (x > body_x1) break;
                                int y = y_bot
                                      - (int)((double) h[k] / 32.0 * h_axis);
                                if (prev_x >= 0)
                                    DrawLine(prev_x, prev_y, x, y,
                                        (Color){180, 220, 250, 230});
                                prev_x = x; prev_y = y;
                            }
                        } else {
                            // Dense: per-pixel min/max binning.
                            int *col_min = malloc((size_t) body_w * sizeof(int));
                            int *col_max = malloc((size_t) body_w * sizeof(int));
                            if (col_min != NULL && col_max != NULL) {
                                for (int x = 0; x < body_w; ++x) {
                                    col_min[x] = 33;
                                    col_max[x] = -1;
                                }
                                for (size_t k = 0; k < n_h; ++k) {
                                    int x = (int)(st[k] / (sr * span_s) * body_w);
                                    if (x < 0 || x >= body_w) continue;
                                    if (h[k] < col_min[x]) col_min[x] = h[k];
                                    if (h[k] > col_max[x]) col_max[x] = h[k];
                                }
                                for (int x = 0; x < body_w; ++x) {
                                    if (col_min[x] > 32) continue;
                                    int y_mn = y_bot
                                        - (int)((double) col_min[x] / 32.0 * h_axis);
                                    int y_mx = y_bot
                                        - (int)((double) col_max[x] / 32.0 * h_axis);
                                    DrawLine(body_x0 + x, y_mn,
                                             body_x0 + x, y_mx,
                                             (Color){120, 180, 220, 180});
                                }
                            }
                            free(col_min); free(col_max);
                        }
                        // Marker at the ASM lock — positioned from
                        // the captured absolute time so it tracks
                        // the actual ASM through pan/zoom even before
                        // the next debounced recompute fires.
                        if (decmode_asm_abs_time_s >= 0.0) {
                            int x = body_x0
                                  + (int)((decmode_asm_abs_time_s
                                           - wf_t_lo)
                                          / span_s * (double) body_w);
                            if (x >= body_x0 && x <= body_x1) {
                                DrawLine(x, body_y0, x, body_y1, YELLOW);
                                char buf[64];
                                snprintf(buf, sizeof buf,
                                    "ASM @ bit %zu  (min=%d)",
                                    decmode_diag.asm_offset,
                                    decmode_diag.asm_dist);
                                draw_text(buf, x + 6, body_y0 + 4,
                                          AMP_PT, YELLOW);
                            }
                        }
                        // Expected-vs-actual bit comparison strip.
                        // Shared helper so the same strip lands on
                        // stages 2, 3, 5, 6 — anywhere the phase /
                        // post-discriminator signal is shown.
                        {
                            int strip_top = y_bot - 2 * 14 - 2 - 2;
                            draw_asm_bit_strip(
                                body_x0, body_x1, body_w,
                                wf_t_lo, span_s, strip_top,
                                decmode_diag.bits,
                                decmode_diag.n_strobes,
                                decmode_diag.asm_offset,
                                decmode_asm_abs_time_s,
                                decmode_bit_period_s,
                                AMP_PT - 4);
                        }
                    }
                }

                // ------ Stage 7: Golay(24,12) length header ------
                else if (decmode_stage == 7) {
                    // Layout: 24 cells across the upper third of the
                    // body (parity | data), result text in the middle,
                    // raw-word bit pattern in the lower third.
                    int margin_x = 24;
                    int cells_y0 = body_y0 + 30;
                    int cells_h  = 56;
                    int avail_w  = body_w - 2 * margin_x;
                    int gap_mid  = 18;
                    int half_w   = (avail_w - gap_mid) / 2;
                    int cell_w   = half_w / 12;
                    int parity_x0 = body_x0 + margin_x;
                    int data_x0   = parity_x0 + 12 * cell_w + gap_mid;

                    // Captions
                    draw_text("parity (12 bits)",
                              parity_x0, cells_y0 - 18,
                              T_PT, (Color){170, 180, 220, 220});
                    draw_text("data (12 bits)",
                              data_x0, cells_y0 - 18,
                              T_PT, (Color){200, 200, 150, 230});

                    // Draw the 24 cells. Golay convention from
                    // ax100.c: word = (parity << 12) | data. So bits
                    // 23..12 of word24 are parity (drawn left),
                    // bits 11..0 are data (drawn right).
                    if (decmode_golay_rc != -2) {
                        Color cell_on_parity = {180, 190, 240, 255};
                        Color cell_on_data   = {230, 220, 130, 255};
                        Color cell_off       = {60, 60, 80, 255};
                        Color cell_edge      = {30, 30, 40, 255};
                        for (int i = 0; i < 24; ++i) {
                            int bit = (decmode_golay_word24
                                       >> (23 - i)) & 1u;
                            int is_parity = (i < 12);
                            int xi = is_parity
                                ? (parity_x0 + i * cell_w)
                                : (data_x0   + (i - 12) * cell_w);
                            Color c = bit
                                ? (is_parity ? cell_on_parity
                                             : cell_on_data)
                                : cell_off;
                            DrawRectangle(xi + 1, cells_y0,
                                          cell_w - 2, cells_h, c);
                            DrawRectangleLines(xi + 1, cells_y0,
                                               cell_w - 2, cells_h,
                                               cell_edge);
                            char b[4];
                            snprintf(b, sizeof b, "%d", bit);
                            int bw = measure_text(b, AMP_PT);
                            Color tcol = bit
                                ? (Color){20, 20, 30, 255}
                                : (Color){170, 170, 190, 255};
                            draw_text(b, xi + 1 + (cell_w - 2 - bw)/2,
                                      cells_y0 + (cells_h - AMP_PT)/2,
                                      AMP_PT, tcol);
                        }
                    }

                    // Result text below the cells.
                    int txt_y = cells_y0 + cells_h + 24;
                    char buf[128];
                    if (decmode_golay_rc == -2) {
                        draw_text("no ASM lock — Golay decode skipped",
                                  body_x0 + margin_x, txt_y,
                                  AMP_PT, LIGHTGRAY);
                    } else {
                        snprintf(buf, sizeof buf,
                            "raw word    0x%06X",
                            decmode_golay_word24 & 0xFFFFFFu);
                        draw_text(buf, body_x0 + margin_x, txt_y,
                                  AMP_PT,
                                  (Color){220, 220, 230, 255});

                        snprintf(buf, sizeof buf,
                            "data 12b    0x%03X  ( = %u bytes )",
                            decmode_golay_data12 & 0xFFFu,
                            (unsigned) decmode_golay_data12);
                        draw_text(buf, body_x0 + margin_x,
                                  txt_y + AMP_PT + 6, AMP_PT,
                                  (Color){230, 220, 140, 255});

                        if (decmode_golay_rc == 0) {
                            snprintf(buf, sizeof buf,
                                "decode ok   %d bit error%s corrected",
                                decmode_golay_errors,
                                decmode_golay_errors == 1 ? "" : "s");
                            Color col = (decmode_golay_errors <= 1)
                                ? (Color){140, 220, 140, 255}
                                : (decmode_golay_errors == 2)
                                ? (Color){220, 220, 140, 255}
                                : (Color){240, 180, 120, 255};
                            draw_text(buf, body_x0 + margin_x,
                                      txt_y + 2*(AMP_PT + 6),
                                      AMP_PT, col);
                        } else {
                            draw_text(
                                "decode FAIL  (>3 bit errors — uncorrectable)",
                                body_x0 + margin_x,
                                txt_y + 2*(AMP_PT + 6),
                                AMP_PT, (Color){240, 110, 110, 255});
                        }
                    }
                }

                // ------ Stage 8: CCSDS descrambler ------
                else if (decmode_stage == 8) {
                    // Three rows of byte cells:
                    //   1) scrambled (on-wire) bytes
                    //   2) CCSDS XOR table bytes
                    //   3) descrambled bytes (row 1 XOR row 2)
                    // Plus an ASCII strip on the bottom showing
                    // printable descrambled bytes — gives the
                    // operator a quick "is this a beacon string?"
                    // sanity check.
                    int margin_x = 24;
                    int avail_w  = body_w - 2 * margin_x;
                    int cell_h   = 26;
                    int gap_y    = 4;
                    int cell_w   = (int) decmode_descr_cell_w;
                    if (cell_w < 12) cell_w = 12;
                    int max_cells = (avail_w > 0)
                        ? (avail_w / cell_w) : 0;
                    if (max_cells > DECMODE_DESCR_CAP)
                        max_cells = DECMODE_DESCR_CAP;
                    // Clamp the scroll offset each frame so a
                    // shrunk window or shorter Golay length can't
                    // leave us scrolled past the end.
                    int off_v = decmode_descr_view_off;
                    int max_off_v = (int) decmode_descr_n - max_cells;
                    if (max_off_v < 0) max_off_v = 0;
                    if (off_v > max_off_v) {
                        off_v = max_off_v;
                        decmode_descr_view_off = off_v;
                    }
                    if (off_v < 0) {
                        off_v = 0;
                        decmode_descr_view_off = 0;
                    }
                    int n_show = (int) decmode_descr_n - off_v;
                    if (n_show > max_cells) n_show = max_cells;
                    if (n_show < 0) n_show = 0;
                    int row_x0 = body_x0 + margin_x;
                    int row1_y = body_y0 + 32;
                    int row2_y = row1_y + cell_h + gap_y;
                    int row3_y = row2_y + cell_h + gap_y;
                    int ascii_y = row3_y + cell_h + gap_y + 4;

                    // Row captions on the left.
                    draw_text("scrambled",
                              body_x0 + margin_x - 4 - 70,
                              row1_y + (cell_h - AMP_PT)/2,
                              AMP_PT, (Color){200, 200, 220, 220});
                    draw_text("CCSDS XOR",
                              body_x0 + margin_x - 4 - 70,
                              row2_y + (cell_h - AMP_PT)/2,
                              AMP_PT, (Color){170, 170, 200, 220});
                    draw_text("descrambled",
                              body_x0 + margin_x - 4 - 88,
                              row3_y + (cell_h - AMP_PT)/2,
                              AMP_PT, (Color){200, 220, 200, 220});

                    if (decmode_descr_n == 0) {
                        draw_text(
                            "no Golay lock — descrambler input is "
                            "not byte-aligned yet",
                            body_x0 + margin_x, body_y0 + 80,
                            AMP_PT, LIGHTGRAY);
                    } else {
                        Color edge       = {30, 30, 40, 255};
                        Color row1_bg    = {60, 60, 70, 255};
                        Color row2_bg    = {45, 45, 60, 255};
                        Color row3_bg    = {55, 80, 70, 255};
                        Color row1_tx    = {220, 220, 230, 255};
                        Color row2_tx    = {170, 180, 210, 255};
                        Color row3_tx    = {220, 240, 200, 255};

                        for (int i = 0; i < n_show; ++i) {
                            int src = off_v + i;
                            int x = row_x0 + i * cell_w;
                            DrawRectangle(x, row1_y, cell_w - 2, cell_h, row1_bg);
                            DrawRectangleLines(x, row1_y, cell_w - 2, cell_h, edge);
                            DrawRectangle(x, row2_y, cell_w - 2, cell_h, row2_bg);
                            DrawRectangleLines(x, row2_y, cell_w - 2, cell_h, edge);
                            DrawRectangle(x, row3_y, cell_w - 2, cell_h, row3_bg);
                            DrawRectangleLines(x, row3_y, cell_w - 2, cell_h, edge);

                            char b[4];
                            int tp = AMP_PT - 2;
                            snprintf(b, sizeof b, "%02X",
                                     decmode_descr_scrambled[src]);
                            int bw = measure_text(b, tp);
                            draw_text(b, x + (cell_w - 2 - bw)/2,
                                      row1_y + (cell_h - tp)/2,
                                      tp, row1_tx);
                            snprintf(b, sizeof b, "%02X",
                                     DECMODE_CCSDS_TABLE[
                                         (size_t) src % DECMODE_CCSDS_TABLE_N]);
                            bw = measure_text(b, tp);
                            draw_text(b, x + (cell_w - 2 - bw)/2,
                                      row2_y + (cell_h - tp)/2,
                                      tp, row2_tx);
                            snprintf(b, sizeof b, "%02X",
                                     decmode_descr_descrambled[src]);
                            bw = measure_text(b, tp);
                            draw_text(b, x + (cell_w - 2 - bw)/2,
                                      row3_y + (cell_h - tp)/2,
                                      tp, row3_tx);
                        }
                        // Byte-index ribbon above row 1 — light tick
                        // every 4 cells, with the byte number written
                        // every 8 so the operator can tell at a
                        // glance "I'm at byte 64", "byte 128", etc.
                        for (int i = 0; i < n_show; ++i) {
                            int src = off_v + i;
                            int x = row_x0 + i * cell_w;
                            if ((src & 3) == 0) {
                                DrawLine(x + (cell_w - 2)/2,
                                         row1_y - 6,
                                         x + (cell_w - 2)/2,
                                         row1_y - 2,
                                         (Color){120, 130, 150, 220});
                            }
                            if ((src & 7) == 0) {
                                char nb[16];
                                snprintf(nb, sizeof nb, "%d", src);
                                int nw = measure_text(nb, AMP_PT - 4);
                                draw_text(nb,
                                          x + (cell_w - 2 - nw)/2,
                                          row1_y - AMP_PT - 2,
                                          AMP_PT - 4,
                                          (Color){170, 180, 210, 220});
                            }
                        }
                        // ASCII strip — replace non-printable with '.'.
                        char ascii_buf[DECMODE_DESCR_CAP + 1] = {0};
                        for (int i = 0; i < n_show; ++i) {
                            uint8_t c =
                                decmode_descr_descrambled[off_v + i];
                            ascii_buf[i] =
                                (c >= 0x20 && c < 0x7F) ? (char) c : '.';
                        }
                        ascii_buf[n_show] = '\0';
                        draw_text("ASCII:",
                                  body_x0 + margin_x - 4 - 56,
                                  ascii_y + 2,
                                  AMP_PT - 2,
                                  (Color){170, 200, 200, 220});
                        draw_text(ascii_buf,
                                  row_x0, ascii_y,
                                  AMP_PT,
                                  (Color){180, 220, 220, 230});

                        // Footer: show the byte range currently on
                        // screen, plus the Golay-decoded total.
                        char status_buf[192];
                        int last = off_v + n_show;
                        if (decmode_golay_rc == 0) {
                            snprintf(status_buf, sizeof status_buf,
                                "showing bytes [%d, %d) of %u "
                                "(Golay said %u on the wire)  —  "
                                "scroll the panel to scan further",
                                off_v, last,
                                (unsigned) decmode_descr_n,
                                (unsigned) decmode_golay_data12);
                        } else {
                            snprintf(status_buf, sizeof status_buf,
                                "showing bytes [%d, %d) of %u  —  "
                                "Golay uncorrectable, length unknown",
                                off_v, last,
                                (unsigned) decmode_descr_n);
                        }
                        draw_text(status_buf,
                                  row_x0,
                                  ascii_y + AMP_PT + 12,
                                  AMP_PT - 2,
                                  (Color){180, 180, 200, 220});
                    }
                }

                // ------ Stage 9: Reed-Solomon (255,223) ------
                else if (decmode_stage == 9) {
                    // Two rows of byte cells across the 255-byte
                    // codeword: the input (pre-correction) and the
                    // output (post-correction). Cells whose value
                    // changed are tinted yellow on the output row so
                    // the operator can spot every correction at a
                    // glance. A red vertical separator marks the
                    // data/parity boundary at byte 223. Scroll +
                    // pinch reuse the descrambler-stage state so
                    // the operator's view position carries across.
                    int margin_x = 24;
                    int avail_w  = body_w - 2 * margin_x;
                    int cell_h   = 26;
                    int gap_y    = 4;
                    int cell_w   = (int) decmode_rs_cell_w;
                    if (cell_w < 12) cell_w = 12;
                    int max_cells = (avail_w > 0)
                        ? (avail_w / cell_w) : 0;
                    if (max_cells > RS_N) max_cells = RS_N;
                    int off_v = decmode_rs_view_off;
                    int max_off_v = RS_N - max_cells;
                    if (max_off_v < 0) max_off_v = 0;
                    if (off_v > max_off_v) {
                        off_v = max_off_v;
                        decmode_rs_view_off = off_v;
                    }
                    if (off_v < 0) {
                        off_v = 0;
                        decmode_rs_view_off = 0;
                    }
                    int n_show = RS_N - off_v;
                    if (n_show > max_cells) n_show = max_cells;
                    if (n_show < 0) n_show = 0;

                    int row_x0 = body_x0 + margin_x;
                    int row1_y = body_y0 + 40;
                    int row2_y = row1_y + cell_h + gap_y;
                    int ascii_y = row2_y + cell_h + gap_y + 4;

                    draw_text("input",
                              body_x0 + margin_x - 4 - 50,
                              row1_y + (cell_h - AMP_PT)/2,
                              AMP_PT, (Color){200, 200, 220, 220});
                    draw_text("corrected",
                              body_x0 + margin_x - 4 - 80,
                              row2_y + (cell_h - AMP_PT)/2,
                              AMP_PT, (Color){200, 220, 200, 220});

                    if (decmode_rs_errors == -2) {
                        draw_text(
                            "no descrambled payload — need at least 33 "
                            "bytes (RS parity + 1) before RS can run",
                            body_x0 + margin_x, body_y0 + 80,
                            AMP_PT, LIGHTGRAY);
                    } else {
                        Color edge       = {30, 30, 40, 255};
                        Color row1_bg    = {60, 60, 70, 255};
                        Color row2_bg    = {55, 80, 70, 255};
                        Color corrected_bg = {200, 170, 60, 255};
                        Color row1_tx    = {220, 220, 230, 255};
                        Color row2_tx    = {220, 240, 200, 255};
                        Color corr_tx    = {30, 25, 10, 255};

                        // Quick lookup for corrected positions.
                        int corr_lo[RS_N] = {0};
                        if (decmode_rs_errors > 0) {
                            for (int j = 0; j < decmode_rs_errors
                                            && j < RS_NROOTS; ++j) {
                                int p = decmode_rs_locs[j];
                                if (p >= 0 && p < RS_N) corr_lo[p] = 1;
                            }
                        }

                        for (int i = 0; i < n_show; ++i) {
                            int src = off_v + i;
                            int x = row_x0 + i * cell_w;
                            DrawRectangle(x, row1_y,
                                          cell_w - 2, cell_h, row1_bg);
                            DrawRectangleLines(x, row1_y,
                                          cell_w - 2, cell_h, edge);
                            Color bg2 = corr_lo[src]
                                ? corrected_bg : row2_bg;
                            DrawRectangle(x, row2_y,
                                          cell_w - 2, cell_h, bg2);
                            DrawRectangleLines(x, row2_y,
                                          cell_w - 2, cell_h, edge);

                            int tp = AMP_PT - 2;
                            char b[4];
                            snprintf(b, sizeof b, "%02X",
                                     decmode_rs_in[src]);
                            int bw = measure_text(b, tp);
                            draw_text(b, x + (cell_w - 2 - bw)/2,
                                      row1_y + (cell_h - tp)/2,
                                      tp, row1_tx);
                            snprintf(b, sizeof b, "%02X",
                                     decmode_rs_out[src]);
                            bw = measure_text(b, tp);
                            Color tx2 = corr_lo[src]
                                ? corr_tx : row2_tx;
                            draw_text(b, x + (cell_w - 2 - bw)/2,
                                      row2_y + (cell_h - tp)/2,
                                      tp, tx2);
                        }

                        // Byte-index ribbon above row 1.
                        for (int i = 0; i < n_show; ++i) {
                            int src = off_v + i;
                            int x = row_x0 + i * cell_w;
                            if ((src & 3) == 0) {
                                DrawLine(x + (cell_w - 2)/2,
                                         row1_y - 6,
                                         x + (cell_w - 2)/2,
                                         row1_y - 2,
                                         (Color){120, 130, 150, 220});
                            }
                            if ((src & 7) == 0) {
                                char nb[16];
                                snprintf(nb, sizeof nb, "%d", src);
                                int nw = measure_text(nb, AMP_PT - 4);
                                draw_text(nb,
                                          x + (cell_w - 2 - nw)/2,
                                          row1_y - AMP_PT - 2,
                                          AMP_PT - 4,
                                          (Color){170, 180, 210, 220});
                            }
                        }

                        // Data / parity boundary at byte 223 (= start
                        // of the 32-byte RS parity tail).
                        const int parity_start = RS_K;
                        if (parity_start >= off_v
                            && parity_start < off_v + n_show) {
                            int x_sep = row_x0
                                + (parity_start - off_v) * cell_w - 1;
                            DrawLine(x_sep, row1_y - 14,
                                     x_sep, row2_y + cell_h + 2,
                                     (Color){220, 120, 80, 255});
                            draw_text("parity →",
                                      x_sep + 6, row1_y - 14,
                                      AMP_PT - 4,
                                      (Color){220, 150, 100, 230});
                        }

                        // ASCII strip from the corrected codeword.
                        char ascii_buf[RS_N + 1] = {0};
                        for (int i = 0; i < n_show; ++i) {
                            uint8_t c = decmode_rs_out[off_v + i];
                            ascii_buf[i] =
                                (c >= 0x20 && c < 0x7F) ? (char) c : '.';
                        }
                        ascii_buf[n_show] = '\0';
                        draw_text("ASCII:",
                                  body_x0 + margin_x - 4 - 56,
                                  ascii_y + 2,
                                  AMP_PT - 2,
                                  (Color){170, 200, 200, 220});
                        draw_text(ascii_buf,
                                  row_x0, ascii_y,
                                  AMP_PT,
                                  (Color){180, 220, 220, 230});

                        // Footer: status + correction count.
                        char status_buf[224];
                        int last = off_v + n_show;
                        if (decmode_rs_errors >= 0) {
                            snprintf(status_buf, sizeof status_buf,
                                "showing codeword bytes [%d, %d) of 255  "
                                "—  %d correction%s applied "
                                "(pad %u, data 0..222, parity 223..254)",
                                off_v, last,
                                decmode_rs_errors,
                                decmode_rs_errors == 1 ? "" : "s",
                                (unsigned) decmode_rs_pad_len);
                        } else {
                            snprintf(status_buf, sizeof status_buf,
                                "showing codeword bytes [%d, %d) of 255  "
                                "—  RS uncorrectable (>16 byte errors)",
                                off_v, last);
                        }
                        draw_text(status_buf,
                                  row_x0,
                                  ascii_y + AMP_PT + 12,
                                  AMP_PT - 2,
                                  decmode_rs_errors >= 0
                                      ? (Color){180, 180, 200, 220}
                                      : (Color){240, 110, 110, 255});
                    }
                }

                DrawRectangleLines(body_x0, body_y0,
                                   body_w, body_h, DARKGRAY);

                // Time-axis ticks — same renderer the W panel uses,
                // shared by the time-domain stages (0-6).
                if (decmode_stage != 7 && decmode_stage != 8
                    && decmode_stage != 9) {
                    double raw_step = span_s / 6.0;
                    double mag = pow(10.0, floor(log10(raw_step)));
                    double mul = raw_step / mag;
                    if      (mul < 1.5) mul = 1.0;
                    else if (mul < 3.5) mul = 2.0;
                    else if (mul < 7.5) mul = 5.0;
                    else                mul = 10.0;
                    double step = mul * mag;
                    double t0_aligned = ceil(wf_t_lo / step) * step;
                    int t_nd = decimals_for_step(step);
                    for (double t = t0_aligned;
                         t <= wf_t_hi + 0.5 * step; t += step) {
                        int x = body_x0
                              + (int)((t - wf_t_lo) / span_s * body_w);
                        if (x < body_x0 || x > body_x1) continue;
                        DrawLine(x, body_y1, x, body_y1 + 6, GRAY);
                        char buf[40];
                        fmt_mmss_ndec(t, t_nd, buf, sizeof buf);
                        int tw = measure_text(buf, T_PT);
                        draw_text(buf, x - tw/2, body_y1 + 8,
                                  T_PT, GRAY);
                    }
                }

                // Stage 6 also gets a secondary bit-number axis below
                // the time labels. Tick step is the largest power-of-2
                // multiple of 1 such that 5..20 ticks fit; 8 is the
                // default when it fits, but at very wide zoom we
                // coarsen to 16/32/64/... and at very narrow zoom we
                // refine to 4/2/1.
                if (decmode_stage == 6
                    && decmode_diag.n_strobes >= 32
                    && decmode_diag.strobe_t != NULL) {
                    size_t n_h = decmode_diag.n_strobes - 31;
                    const double *st = decmode_diag.strobe_t;
                    double sr = samp_rate_d;
                    static const int cands[] = {
                        2048, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1
                    };
                    int n_cands = (int)(sizeof cands / sizeof cands[0]);
                    int step_bits = 8;
                    for (int i = 0; i < n_cands; ++i) {
                        int t = (int)(n_h / (size_t) cands[i]);
                        if (t >= 5 && t <= 20) {
                            step_bits = cands[i];
                            break;
                        }
                    }
                    int bit_y_line = body_y1 + 26;
                    DrawLine(body_x0, bit_y_line, body_x1, bit_y_line,
                             (Color){60, 80, 110, 200});
                    for (size_t b = 0; b < n_h; b += (size_t) step_bits) {
                        int x = body_x0
                              + (int)(st[b] / (sr * span_s) * body_w);
                        if (x < body_x0 || x > body_x1) continue;
                        DrawLine(x, bit_y_line - 4, x, bit_y_line,
                                 (Color){120, 160, 200, 220});
                        char buf[32];
                        snprintf(buf, sizeof buf, "%zu", b);
                        int tw = measure_text(buf, T_PT - 2);
                        draw_text(buf, x - tw/2, bit_y_line + 2,
                                  T_PT - 2, (Color){170, 200, 220, 220});
                    }
                    // Axis caption to the left.
                    const char *cap = "bit";
                    int capw = measure_text(cap, T_PT - 2);
                    draw_text(cap, body_x0 - 6 - capw,
                              bit_y_line + 2, T_PT - 2,
                              (Color){170, 200, 220, 220});
                }
            } else {
                // Either no data yet, or the window is too small.
                const char *msg = (!decmode_have_result)
                    ? "computing decode chain (zoom the spectrogram to refresh)..."
                    : "no data — pick an IQ region in the spectrogram above";
                draw_text(msg, plot_x0, plot_y0 + 20, 16, LIGHTGRAY);
            }
            // Cursor indicator: yellow vertical guide showing where
            // the spectrogram cursor's time falls on the K-panel
            // x-axis, with top + bottom arrowheads (same as the W
            // panel). When the mouse is in the K panel itself, draw
            // a small crosshair at the mouse position. Skipped on
            // stage 4 (eye, symbol-relative) and stage 7 (Golay, no
            // time axis).
            if (decmode_stage != 4 && decmode_stage != 7
                && decmode_stage != 8 && decmode_stage != 9
                && wf_t_hi > wf_t_lo
                && plot_w > 16 && plot_h > 16) {
                double span = wf_t_hi - wf_t_lo;
                if (in_spec
                    && cursor_t >= wf_t_lo - 1e-12
                    && cursor_t <= wf_t_hi + 1e-12) {
                    int cx = plot_x0
                           + (int)((cursor_t - wf_t_lo) / span * plot_w);
                    if (cx >= plot_x0 && cx <= plot_x1) {
                        DrawLine(cx - 1, plot_y0, cx - 1, plot_y1 + 1, BLACK);
                        DrawLine(cx + 1, plot_y0, cx + 1, plot_y1 + 1, BLACK);
                        DrawLine(cx,     plot_y0, cx,     plot_y1 + 1, YELLOW);
                        Vector2 ba = {(float) cx, (float)(plot_y1 - 1)};
                        Vector2 bl = {(float)(cx - 7), (float)(plot_y1 + 11)};
                        Vector2 br = {(float)(cx + 7), (float)(plot_y1 + 11)};
                        DrawTriangle(ba, br, bl, YELLOW);
                        DrawTriangleLines(ba, br, bl, BLACK);
                        Vector2 ta = {(float) cx, (float)(plot_y0 + 1)};
                        Vector2 tl = {(float)(cx - 7), (float)(plot_y0 - 11)};
                        Vector2 tr = {(float)(cx + 7), (float)(plot_y0 - 11)};
                        DrawTriangle(ta, tl, tr, YELLOW);
                        DrawTriangleLines(ta, tl, tr, BLACK);
                    }
                }
                int in_k_panel_now =
                    decmode_open
                    && (int) m.y >= sh - decmode_panel_h
                    && (int) m.y <  sh
                    && (int) m.x >= 0
                    && (int) m.x <  dm_right;
                if (in_k_panel_now
                    && (int) m.x >= plot_x0 && (int) m.x <= plot_x1
                    && (int) m.y >= plot_y0 && (int) m.y <= plot_y1) {
                    int cx = (int) m.x;
                    int cy = (int) m.y;
                    DrawLine(cx - 6, cy, cx + 6, cy, YELLOW);
                    DrawLine(cx, cy - 6, cx, cy + 6, YELLOW);
                }
            }
        }

        // ----- decode side panel (D to (re)decode, Esc to close) -----
        if (decode_open) {
            int dpx = spec_screen_w;
            int dpw = decode_panel_w;
            DrawRectangle(dpx, 0, dpw, sh, (Color){12, 12, 18, 255});
            DrawLine(dpx, 0, dpx, sh, GRAY);

            const int DEC_TITLE_PT = 16;
            const int DEC_BODY_PT  = 13;
            const int dec_line_h   = DEC_BODY_PT + 3;
            draw_text("Decode  [Esc=close  D=rerun  wheel=scroll]",
                      dpx + 8, 6, DEC_TITLE_PT,
                      (Color){200, 220, 240, 255});
            int body_y0 = 6 + DEC_TITLE_PT + 10;
            int body_y_max = sh - 4;

            // Count lines for scroll clamping.
            int n_lines = 0;
            if (decode_text != NULL) {
                for (const char *p = decode_text; *p != '\0'; ++p) {
                    if (*p == '\n') ++n_lines;
                }
                if (decode_text_len > 0
                    && decode_text[decode_text_len - 1] != '\n') {
                    ++n_lines;
                }
            }
            int max_scroll =
                n_lines - (body_y_max - body_y0) / dec_line_h;
            if (max_scroll < 0) max_scroll = 0;
            if (decode_scroll > max_scroll) decode_scroll = max_scroll;

            // Walk lines, render those in [decode_scroll, ...].
            if (decode_text != NULL) {
                const char *p = decode_text;
                int line_idx = 0;
                int y = body_y0;
                while (*p != '\0' && y < body_y_max) {
                    const char *eol = strchr(p, '\n');
                    int len = (eol != NULL) ? (int)(eol - p) : (int) strlen(p);
                    if (line_idx >= decode_scroll) {
                        char buf[512];
                        int copy = len;
                        if (copy >= (int)(sizeof buf)) copy = (int) sizeof buf - 1;
                        memcpy(buf, p, (size_t) copy);
                        buf[copy] = '\0';
                        draw_text(buf, dpx + 8, y, DEC_BODY_PT,
                                  (Color){210, 210, 215, 255});
                        y += dec_line_h;
                    }
                    ++line_idx;
                    if (eol == NULL) break;
                    p = eol + 1;
                }
            } else {
                draw_text("(no output)", dpx + 8, body_y0, DEC_BODY_PT,
                          (Color){140, 140, 150, 255});
            }
            // Scroll indicator at bottom-right of panel.
            if (n_lines > 0) {
                char sb[64];
                snprintf(sb, sizeof sb,
                    "%d / %d lines", decode_scroll, n_lines);
                int tw = measure_text(sb, DEC_BODY_PT);
                draw_text(sb, dpx + dpw - tw - 8, sh - DEC_BODY_PT - 4,
                          DEC_BODY_PT, (Color){140, 140, 160, 255});
            }
        }

        EndDrawing();
    }

    // Make sure the band-pass-filter worker finishes before we let
    // iqb.samples drop out from under it.
    if (filter_building) {
        pthread_join(filter_thread, NULL);
        filter_building = 0;
    }
    if (filtered_iq != NULL) free(filtered_iq);

    for (int t = 0; t < n_tiles; ++t) {
        if (tiles[t].id != 0) UnloadTexture(tiles[t]);
    }
    free(tiles);
    UnloadTexture(cb_texture);
    UnloadShader(wf_shader);
    free(spec_db);
    iq_buf_free(&iqb);
    free(decode_text);
    decmode_diag_free(&decmode_diag);
    free(decmode_out_scratch);
    if (g_ui_font_loaded) UnloadFont(g_ui_font);
    CloseWindow();
    return 0;
}
