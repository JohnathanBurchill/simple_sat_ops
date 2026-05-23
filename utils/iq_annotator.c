/*

    Simple Satellite Operations  utils/iq_annotator.c

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

#include "pdf_writer.h"
#include "waterfall_core.h"

#ifdef __APPLE__
// Trackpad pinch is delivered as NSEventTypeMagnify, which raylib/GLFW
// don't forward. utils/iq_annotator_macos.m installs an NSEvent local
// monitor that accumulates magnification deltas into this global; we
// read & reset it each frame.
extern float g_iq_annotator_pinch_delta;
extern void  iq_annotator_install_pinch_monitor(void);
#endif

#include <ctype.h>
#include <errno.h>
#include <libgen.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
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
        fprintf(stderr, "iq_annotator: save failed: %s: %s\n",
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
        fprintf(stderr, "iq_annotator: save failed: %s: %s\n",
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
        "iq_annotator: saved %d box(es) to %s\n             anchors  to %s\n",
        bl->n, path, anc_path);
    return 0;
}

static int boxes_load(box_list_t *bl, const char *iq_path)
{
    char path[1100];
    snprintf(path, sizeof path, "%.1000s.boxes.csv", iq_path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "iq_annotator: load: %s: %s\n", path, strerror(errno));
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
    fprintf(stderr, "iq_annotator: loaded %d box(es) from %s\n",
            bl->n, path);
    return 0;
}

// ---------------------------------------------------------------------------
// Operator-flag passthru — same names gen_waterfall accepts; iq_annotator
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
        // --elapsed-time is iq_annotator's render_opts concern, not wf_opts_t's.
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
    for (int i = 0; i < n; ++i) {
        if (FileExists(candidates[i])) {
            g_ui_font = LoadFontEx(candidates[i], 48, NULL, 0);
            if (g_ui_font.texture.id != 0) {
                SetTextureFilter(g_ui_font.texture,
                                 TEXTURE_FILTER_BILINEAR);
                fprintf(stderr,
                    "iq_annotator: loaded font %s\n", candidates[i]);
                return 1;
            }
        }
    }
    fprintf(stderr,
        "iq_annotator: TTF font not found; using raylib default\n");
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
        fprintf(stderr, "iq_annotator: open %s: %s\n",
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

// Render the current waveform-panel view to a one-page PDF.
// iq_show_mode: 0=both, 1=I only, 2=Q only.
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
        fprintf(stderr, "iq_annotator: pdf open %s: %s\n",
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
              "iq_annotator -- waveform export", 14.0f, 0);
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

    int amp_max = 1;
    if (n_pairs_vis > 0) {
        int64_t step_s = (n_pairs_vis > 4096) ? n_pairs_vis / 2048 : 1;
        for (int64_t k = i_lo; k < i_hi; k += step_s) {
            int I = iqb->samples[k * 2 + 0];
            int Q = iqb->samples[k * 2 + 1];
            if (abs(I) > amp_max) amp_max = abs(I);
            if (abs(Q) > amp_max) amp_max = abs(Q);
        }
    }
    if (amp_max < 32) amp_max = 32;

    float mid_y = pT + plot_h * 0.5f;
    pdfw_set_stroke(w, PDFW_LGREY);
    pdfw_lw(w, 0.3f);
    pdfw_line(w, pL, mid_y, pR, mid_y);

    pdfw_set_stroke(w, PDFW_DGREY);
    pdfw_lw(w, 0.5f);
    pdfw_set_fill(w, PDFW_BLACK);
    for (int k = -2; k <= 2; ++k) {
        float y = mid_y - (float)(k * plot_h / 4);
        pdfw_line(w, pL - 4, y, pL, y);
        char buf[24];
        snprintf(buf, sizeof buf, "%+d", (int)(k * amp_max / 2));
        float tw = pdfw_str_width(buf, 9.0f, 1);
        pdfw_text(w, pL - 6 - tw, y - 4, buf, 9.0f, 1);
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

    int show_i = (iq_show_mode != 2);
    int show_q = (iq_show_mode != 1);
    pdfw_text(w, MARGIN + 4, pT - 14,
              "amplitude (int16)", 9.0f, 0);
    if (show_i) pdfw_text(w, MARGIN + 4, pT + 4,  "I = cyan",    9.0f, 0);
    if (show_q) pdfw_text(w, MARGIN + 4, pT + 16, "Q = magenta", 9.0f, 0);

    if (n_pairs_vis > 0 && plot_w > 8.0f) {
        float y_scale = (plot_h * 0.5f) / (float) amp_max;
        pdfw_rgb_t I_col = (pdfw_rgb_t){0, 170, 200, 255};
        pdfw_rgb_t Q_col = (pdfw_rgb_t){190, 60, 180, 255};
        int plot_w_px = (int) plot_w;
        if (n_pairs_vis <= (int64_t) plot_w_px * 2) {
            pdfw_lw(w, 0.5f);
            float prev_xi = -1, prev_yi = 0, prev_xq = -1, prev_yq = 0;
            for (int64_t k = i_lo; k < i_hi; ++k) {
                double t = (double) k / iqb->samp_rate;
                float x = pL + (float)((t - wf_t_lo) / span * plot_w);
                if (x < pL || x > pR) continue;
                float yI = mid_y - iqb->samples[k*2+0] * y_scale;
                float yQ = mid_y - iqb->samples[k*2+1] * y_scale;
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
        } else {
            pdfw_lw(w, 0.5f);
            for (int x = 0; x < plot_w_px; ++x) {
                int64_t s0 = i_lo + (int64_t) x * n_pairs_vis / plot_w_px;
                int64_t s1 = i_lo + (int64_t)(x+1) * n_pairs_vis / plot_w_px;
                if (s1 <= s0) s1 = s0 + 1;
                if (s1 > i_hi) s1 = i_hi;
                int iMin =  INT_MAX, iMax = INT_MIN;
                int qMin =  INT_MAX, qMax = INT_MIN;
                for (int64_t k = s0; k < s1; ++k) {
                    int I = iqb->samples[k*2+0];
                    int Q = iqb->samples[k*2+1];
                    if (I < iMin) iMin = I;
                    if (I > iMax) iMax = I;
                    if (Q < qMin) qMin = Q;
                    if (Q > qMax) qMax = Q;
                }
                float xpx = pL + (float) x;
                if (show_q) {
                    pdfw_set_stroke(w, Q_col);
                    pdfw_line(w, xpx, mid_y - qMax * y_scale,
                                 xpx, mid_y - qMin * y_scale);
                }
                if (show_i) {
                    pdfw_set_stroke(w, I_col);
                    pdfw_line(w, xpx, mid_y - iMax * y_scale,
                                 xpx, mid_y - iMin * y_scale);
                }
            }
        }
    }

    char foot[256];
    time_t now = time(NULL);
    struct tm utc;
    gmtime_r(&now, &utc);
    snprintf(foot, sizeof foot,
        "generated %04d-%02d-%02dT%02d:%02d:%02dZ  by iq_annotator",
        utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
        utc.tm_hour, utc.tm_min, utc.tm_sec);
    pdfw_set_fill(w, PDFW_GREY);
    pdfw_text(w, MARGIN, page_h - MARGIN - 12, foot, 9.0f, 1);

    return pdfw_end(w);
}

static void usage(void)
{
    fprintf(stderr,
        "usage: iq_annotator <iq_path> [options]\n"
        "  All gen_waterfall options are accepted and forwarded\n"
        "  (--fft-time-bin-s, --zoom-khz, --detrend, --detrend-tau-s, --center-hz,\n"
        "   --dc-notch, --dc-notch-bins, --db-min, --db-max,\n"
        "   --power-offset, --start-utc, --elapsed-time, --fft, --hop,\n"
        "   --marks-csv, --show-tm, --full-width)\n"
        "Annotator-specific:\n"
        "  --rate=<Hz>          IQ rate (default = auto from companion .wav)\n"
        "  --width=<px>         Window width  (default 1280)\n"
        "  --height=<px>        Window height (default 900)\n");
}

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
        } else if (is_passthru(a)) {
            if (n_passthru >= MAX_PASSTHRU) {
                fprintf(stderr,
                    "iq_annotator: too many gen_waterfall flags (>%d)\n",
                    MAX_PASSTHRU);
                return 2;
            }
            passthru[n_passthru++] = a;
        } else {
            fprintf(stderr, "iq_annotator: unknown option '%s'\n", a);
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
    fprintf(stderr, "iq_annotator: rate=%d Hz\n", samp_rate);

    // Compute duration from file size.
    struct stat st;
    if (stat(iq_path, &st) != 0) {
        fprintf(stderr, "iq_annotator: stat %s: %s\n",
                iq_path, strerror(errno));
        return 1;
    }
    size_t n_pairs = (size_t) st.st_size / 4;
    double duration_s = (double) n_pairs / (double) samp_rate;
    {
        char pb[32];
        fmt_thousands(pb, sizeof pb, (uint64_t) n_pairs);
        fprintf(stderr, "iq_annotator: duration=%.3fs (%s pairs)\n",
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
    InitWindow(win_w, win_h, "iq_annotator");
    SetTargetFPS(60);
    SetExitKey(0);
#ifdef __APPLE__
    iq_annotator_install_pinch_monitor();
#endif
    g_ui_font_loaded = load_ttf_from_known_paths();

    // Spawn the loader thread that handles iq_buf_load + wf_compute, and
    // run a tiny "Loading..." render loop on the main thread until it
    // finishes. Doing the FFT off the main thread keeps the window
    // responsive (and showing progress) for the few seconds a long-pass
    // FFT can take.
    fprintf(stderr,
        "iq_annotator: computing spectrogram (fft=%d, rows=%d, "
        "zoom=%.1f kHz, detrend=%d)...\n",
        wf_opt.fft_size, wf_opt.out_rows,
        wf_opt.zoom_hz / 1e3, wf_opt.detrend_mode);

    loader_ctx_t lctx = {0};
    lctx.iq_path   = iq_path;
    lctx.samp_rate = samp_rate;
    lctx.opt       = &wf_opt;
    snprintf(lctx.status_msg, sizeof lctx.status_msg, "Loading IQ samples...");
    pthread_mutex_init(&lctx.lock, NULL);

    pthread_t loader_thread;
    if (pthread_create(&loader_thread, NULL,
                       loader_thread_fn, &lctx) != 0) {
        fprintf(stderr, "iq_annotator: pthread_create: %s\n",
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
                fprintf(stderr, "iq_annotator: %s\n", lctx.status_msg);
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
            "iq_annotator: IQ buffer loaded (%s pairs)\n", pb);
    }
    if (spec_w < 16 || spec_h < 16) {
        fprintf(stderr,
            "iq_annotator: spectrogram too small (%dx%d)\n",
            spec_w, spec_h);
        free(spec_db); iq_buf_free(&iqb); CloseWindow(); return 1;
    }
    // wf_compute returns row 0 = earliest sample. gen_waterfall's
    // render_with_axes then flips that so its PNG reads "newest at
    // top of spec, earliest at bottom" — the convention every other
    // bit of iq_annotator already assumes (box→pixel math and the
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
        fprintf(stderr, "iq_annotator: oom on tile array\n");
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
                "iq_annotator: tile %d upload failed (%dx%d)\n",
                t, spec_w, h);
        } else {
            SetTextureFilter(tiles[t], TEXTURE_FILTER_POINT);
        }
    }
    fprintf(stderr,
        "iq_annotator: spec %dx%d, %d tile(s) of up to %d rows, "
        "BW %.1f kHz, floor=%.1f dBFS\n",
        spec_w, spec_h, n_tiles, TILE_H, display_bw_hz / 1e3,
        wf_opt.display_db_floor + wf_opt.power_offset_db);

    // Fragment shader: take the R32F sample (dB), normalise against the
    // operator's current [db_min, db_max] range, and evaluate viridis
    // analytically (Inigo Quilez's 6-term polynomial fit — matches
    // matplotlib's viridis within ~1% per channel and avoids the
    // sampler-completeness pitfalls that a separate LUT texture runs
    // into on Apple's OpenGL 4.1 stack).
    static const char *FRAG_SRC =
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
        "out vec4 finalColor;\n"
        "vec3 viridis(float t) {\n"
        "    const vec3 c0 = vec3( 0.2777273272234177,  0.005407344544966578,  0.3340998053353061);\n"
        "    const vec3 c1 = vec3( 0.1050930431667207,  1.4046135298985746,    1.3845901625946856);\n"
        "    const vec3 c2 = vec3(-0.3308618287255563,  0.2148475594682130,    0.0950951630282366);\n"
        "    const vec3 c3 = vec3(-4.6342304989834860, -5.7991009733515850,  -19.3324409562798700);\n"
        "    const vec3 c4 = vec3( 6.2282699363470810, 14.1799333668050900,   56.6905526006810500);\n"
        "    const vec3 c5 = vec3( 4.7763849976702880,-13.7451453777460100,  -65.3530326333723400);\n"
        "    const vec3 c6 = vec3(-5.4354558559346310,  4.6458526121785350,   26.3124352495832000);\n"
        "    return c0 + t*(c1 + t*(c2 + t*(c3 + t*(c4 + t*(c5 + t*c6)))));\n"
        "}\n"
        "void main() {\n"
        "    float v = texture(texture0, fragTexCoord).r;\n"
        "    float t = (v - db_min) / max(db_max - db_min, 1e-6);\n"
        "    t = clamp(t, 0.0, 1.0);\n"
        "    finalColor = vec4(viridis(t), 1.0) * fragColor;\n"
        "}\n";
    Shader wf_shader = LoadShaderFromMemory(NULL, FRAG_SRC);
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
    // Which channels to show in the waveform panel + PDF.
    // 0 = both (default), 1 = I only, 2 = Q only.  Cycle with the
    // `i` key: both → I → Q → both → ...
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

    while (!WindowShouldClose()) {
        int sw = GetScreenWidth();
        int sh = GetScreenHeight();
        Vector2 m = GetMousePosition();

        // ----- input -----
        // Capture pinch + wheel ONCE per frame so we can route them
        // to whichever panel the cursor is over.
        float pinch = 0.0f;
#ifdef __APPLE__
        pinch = g_iq_annotator_pinch_delta;
        g_iq_annotator_pinch_delta = 0.0f;
#endif
        Vector2 wheel_v = GetMouseWheelMoveV();
        int in_panel_for_input =
            wf_open && (int) m.y >= sh - wf_panel_h && (int) m.y < sh;
        // Decode side panel occupies the right strip when open.
        int in_decode_panel_input =
            decode_open && (int) m.x >= sw - decode_panel_w && (int) m.x < sw;
        if (in_decode_panel_input && wheel_v.y != 0.0f) {
            decode_scroll -= (int)(wheel_v.y * 3.0f);
            if (decode_scroll < 0) decode_scroll = 0;
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
        int   spec_screen_h = wf_open
            ? (sh - wf_panel_h) : (sh - bar_h_clamp);
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
            snprintf(status, sizeof status,
                "waveform panel %s (follows spectrogram time range)",
                wf_open ? "ON" : "OFF");
        }
        if (IsKeyPressed(KEY_I)) {
            iq_show_mode = (iq_show_mode + 1) % 3;
            const char *mode_label =
                (iq_show_mode == 0) ? "I + Q"
              : (iq_show_mode == 1) ? "I only" : "Q only";
            snprintf(status, sizeof status,
                "waveform channels: %s", mode_label);
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
        int walk_fwd  = IsKeyPressed(KEY_RIGHT_BRACKET);
        int walk_back = IsKeyPressed(KEY_LEFT_BRACKET);
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
                char tmp_iq[128], tmp_txt[128];
                snprintf(tmp_iq, sizeof tmp_iq,
                    "/tmp/iqa_decode_%d.iq", (int) getpid());
                snprintf(tmp_txt, sizeof tmp_txt,
                    "/tmp/iqa_decode_%d.txt", (int) getpid());
                FILE *fo = fopen(tmp_iq, "wb");
                if (fo == NULL) {
                    snprintf(status, sizeof status,
                        "decode: open %s: %s",
                        tmp_iq, strerror(errno));
                } else {
                    size_t want_pairs = (size_t) n_pairs_dec;
                    fwrite(iqb.samples + i_lo * 2,
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
                            "(t=%.2fs..%.2fs)", t1 - t0, t0, t1);
                    } else {
                        snprintf(status, sizeof status,
                            "decode: rx_replay rc=%d (is it on PATH?)", rc);
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
        int   in_panel_now =
            wf_open && (int) m.y >= sh - wf_panel_h && (int) m.y < sh;
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
                const int SS_PT     = lpt_sub;
                const int AXIS_PT   = lpt_axis;
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
        int spec_bot_y_screen =
            (wf_open ? wf_y0 : (sh - bar_h_for_wf));
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
                "cursor t=%.3fs  f=%+.0f Hz  zoom=%.2fx  boxes=%d  hover=%d sel=%d  | drag=new pinch=zoom +/-=5x@cursor-anchor scroll=pan W=wave I=ch P=pdf T=label S=save L=load ,./<>=color R=auto-color 0=reset Del=del-hover Q=quit",
                cursor_t, cursor_f, (double) zoom,
                boxes.n, hovered, boxes.selected);
        } else {
            snprintf(info, sizeof info,
                "zoom=%.2fx  boxes=%d  | drag=new pinch=zoom +/-=5x@cursor-anchor scroll=pan W=wave I=ch P=pdf T=label S=save L=load ,./<>=color R=auto-color 0=reset Del=del-hover Q=quit",
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
                // Sample bounds.
                int64_t i_lo = (int64_t)(wf_t_lo * iqb.samp_rate);
                int64_t i_hi = (int64_t)(wf_t_hi * iqb.samp_rate);
                if (i_lo < 0) i_lo = 0;
                if (i_hi > (int64_t) iqb.n_pairs) i_hi = iqb.n_pairs;
                int64_t n_pairs_vis = i_hi - i_lo;

                // Find the magnitude scale (auto). Take the max |I|,|Q|.
                int amp_max = 1;
                if (n_pairs_vis > 0) {
                    int64_t step = (n_pairs_vis > 4096)
                        ? n_pairs_vis / 2048 : 1;
                    for (int64_t k = i_lo; k < i_hi; k += step) {
                        int I = iqb.samples[k * 2 + 0];
                        int Q = iqb.samples[k * 2 + 1];
                        if (abs(I) > amp_max) amp_max = abs(I);
                        if (abs(Q) > amp_max) amp_max = abs(Q);
                    }
                }
                if (amp_max < 32) amp_max = 32;

                // Centre amp axis on 0; one tick per integer 1/4 of scale.
                DrawLine(plot_x0, plot_y0 + plot_h/2,
                         plot_x1, plot_y0 + plot_h/2,
                         (Color){50, 50, 60, 255});
                for (int k = -2; k <= 2; ++k) {
                    int y = plot_y0 + plot_h/2 - (k * plot_h / 4);
                    DrawLine(plot_x0 - 4, y, plot_x0, y, GRAY);
                    char buf[24];
                    snprintf(buf, sizeof buf, "%+d",
                             (int)(k * amp_max / 2));
                    const int AMP_PT = 19;
                    int tw = measure_text(buf, AMP_PT);
                    draw_text(buf, plot_x0 - 6 - tw, y - AMP_PT/2,
                              AMP_PT, GRAY);
                }

                // Time-axis ticks.
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
                // transitions stay visible).
                if (n_pairs_vis > 0) {
                    int mid_y = plot_y0 + plot_h / 2;
                    float y_scale = (float) plot_h / 2.0f / (float) amp_max;
                    int show_i = (iq_show_mode != 2);  // 0 both, 1 I only
                    int show_q = (iq_show_mode != 1);  // 0 both, 2 Q only
                    if (n_pairs_vis <= (int64_t) plot_w * 2) {
                        // Point/line plot for sparse data.
                        int prev_xi = -1, prev_yi = 0, prev_xq = -1, prev_yq = 0;
                        for (int64_t k = i_lo; k < i_hi; ++k) {
                            double t = (double) k / iqb.samp_rate;
                            int x = plot_x0
                                  + (int)((t - wf_t_lo) / span * plot_w);
                            if (x < plot_x0 || x > plot_x1) continue;
                            int yI = mid_y - (int)(iqb.samples[k*2+0] * y_scale);
                            int yQ = mid_y - (int)(iqb.samples[k*2+1] * y_scale);
                            // Underlay Q in violet first, I in cyan on top.
                            if (show_q && prev_xq >= 0)
                                DrawLine(prev_xq, prev_yq, x, yQ, (Color){200,90,220,200});
                            if (show_i && prev_xi >= 0)
                                DrawLine(prev_xi, prev_yi, x, yI, (Color){80,200,220,255});
                            prev_xi = x; prev_yi = yI;
                            prev_xq = x; prev_yq = yQ;
                        }
                    } else {
                        // Per-column min/max for dense data.
                        for (int x = 0; x < plot_w; ++x) {
                            int64_t s0 = i_lo + (int64_t) x * n_pairs_vis / plot_w;
                            int64_t s1 = i_lo + (int64_t)(x+1) * n_pairs_vis / plot_w;
                            if (s1 <= s0) s1 = s0 + 1;
                            if (s1 > i_hi) s1 = i_hi;
                            int iMin =  INT_MAX, iMax = INT_MIN;
                            int qMin =  INT_MAX, qMax = INT_MIN;
                            for (int64_t k = s0; k < s1; ++k) {
                                int I = iqb.samples[k*2+0];
                                int Q = iqb.samples[k*2+1];
                                if (I < iMin) iMin = I;
                                if (I > iMax) iMax = I;
                                if (Q < qMin) qMin = Q;
                                if (Q > qMax) qMax = Q;
                            }
                            int xpx = plot_x0 + x;
                            if (show_q)
                                DrawLine(xpx, mid_y - (int)(qMax * y_scale),
                                         xpx, mid_y - (int)(qMin * y_scale),
                                         (Color){200,90,220,160});
                            if (show_i)
                                DrawLine(xpx, mid_y - (int)(iMax * y_scale),
                                         xpx, mid_y - (int)(iMin * y_scale),
                                         (Color){80,200,220,220});
                        }
                    }
                }

                // Border around plot.
                DrawRectangleLines(plot_x0, plot_y0,
                                   plot_w, plot_h, DARKGRAY);

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
                  : "Q only (violet)";
                char title[256];
                snprintf(title, sizeof title,
                    "I/Q  span=%.3f s  (%lld samples)   %s   [i=cycle]",
                    span, (long long) n_pairs_vis, chans);
                draw_text(title, plot_x0 + 6, plot_y0 + 2, TITLE_PT,
                          (Color){200, 200, 220, 255});
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

    for (int t = 0; t < n_tiles; ++t) {
        if (tiles[t].id != 0) UnloadTexture(tiles[t]);
    }
    free(tiles);
    UnloadTexture(cb_texture);
    UnloadShader(wf_shader);
    free(spec_db);
    iq_buf_free(&iqb);
    free(decode_text);
    if (g_ui_font_loaded) UnloadFont(g_ui_font);
    CloseWindow();
    return 0;
}
