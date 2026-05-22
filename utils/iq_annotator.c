/*

    Simple Satellite Operations  utils/iq_annotator.c

    Interactive raylib viewer that lets the operator mouse-draw
    time × frequency boxes around bursts in an .iq capture, then save
    them as anchors for rx_replay.

    Rendering is delegated to gen_waterfall: iq_annotator shells out to
    gen_waterfall to produce a PNG with the operator's chosen options
    (--rows, --zoom-khz, --detrend, --detrend-tau-s, --center-hz,
    --dc-notch, --dc-notch-bins, --db-min, --db-max, --power-offset,
    --start-utc, --elapsed-time), loads the PNG as a raylib texture,
    and overlays interactive box drawing + cursor read-outs on top.
    The waterfall image looks identical to whatever gen_waterfall
    would have written to disk.

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
        ↑/↓, PgUp/PgDn,      Scroll
        Home/End, wheel
        Q                    Quit

    CLI: same flag forms gen_waterfall accepts, plus:
        --width=<px>, --height=<px>  raylib window size
        --tmp-png=<path>             override the rendered PNG path
                                     (default: /tmp/iq_annotator_<pid>.png)

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#include <raylib.h>

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
    fprintf(stderr, "iq_annotator: loaded %d box(es) from %s\n",
            bl->n, path);
    return 0;
}

// ---------------------------------------------------------------------------
// Spawn gen_waterfall to render the PNG with the operator's flags.
// ---------------------------------------------------------------------------

#define MAX_PASSTHRU 32
static const char *PASSTHRU_FLAGS[] = {
    // exact-match flags
    "--full-width", "--dc-notch", "--no-dc-notch", "--elapsed-time",
    NULL
};
static const char *PASSTHRU_PREFIXES[] = {
    // prefix flags ("--name=value")
    "--rows=", "--zoom-khz=", "--detrend=", "--detrend-tau-s=",
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

// Build and run: gen_waterfall <iq> <rate> <png> <passthru flags...>
static int run_gen_waterfall(const char *iq_path, int samp_rate,
                             const char *png_path,
                             const char **passthru, int n_passthru)
{
    // Use system() with a constructed command line; pass each token
    // through shellquote since paths can contain spaces.
    // Simpler: use execvp via fork.
    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "iq_annotator: fork: %s\n", strerror(errno));
        return -1;
    }
    if (pid == 0) {
        // child
        char rate_buf[16];
        snprintf(rate_buf, sizeof rate_buf, "%d", samp_rate);
        const char *argv_fixed[] = {
            "gen_waterfall", iq_path, rate_buf, png_path,
        };
        int n_fixed = (int)(sizeof argv_fixed / sizeof argv_fixed[0]);
        int total = n_fixed + n_passthru + 1;
        const char **argv2 = (const char **) calloc((size_t) total, sizeof(char *));
        if (argv2 == NULL) _exit(127);
        int j = 0;
        for (int i = 0; i < n_fixed; ++i) argv2[j++] = argv_fixed[i];
        for (int i = 0; i < n_passthru; ++i) argv2[j++] = passthru[i];
        argv2[j] = NULL;
        execvp("gen_waterfall", (char *const *) argv2);
        fprintf(stderr, "iq_annotator: exec gen_waterfall: %s\n",
                strerror(errno));
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        fprintf(stderr, "iq_annotator: waitpid: %s\n", strerror(errno));
        return -1;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        fprintf(stderr,
            "iq_annotator: gen_waterfall failed (status=%d)\n",
            WIFEXITED(status) ? WEXITSTATUS(status) : -1);
        return -1;
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

static int iq_buf_load(iq_buf_t *b, const char *path, int samp_rate)
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
    if (fread(b->samples, 1, (size_t) sz, fp) != (size_t) sz) {
        fclose(fp); free(b->samples); b->samples = NULL; return -1;
    }
    fclose(fp);
    b->n_pairs   = (size_t) sz / 4;
    b->samp_rate = samp_rate;
    return 0;
}

static void iq_buf_free(iq_buf_t *b)
{
    free(b->samples); b->samples = NULL; b->n_pairs = 0;
}

// ---------------------------------------------------------------------------
// Minimal vector PDF writer (pattern lifted from ~/src/lorentz_tracer's
// pdf_export.c, cut down to the bare minimum we need for one figure:
// a single page, Standard-14 fonts (Helvetica + Courier), stroked lines
// + rectangles, filled rectangles, ASCII text only, full opacity. The
// content stream is buffered in memory until pdf_end() so we know its
// length. Coordinates use raylib screen-pixel space (Y down); the
// PDFY macro flips to PDF page space (Y up).
// ---------------------------------------------------------------------------

#define PDFMAX_OBJ 64

typedef struct {
    FILE *fp;
    long  bytes;
    long  obj_off[PDFMAX_OBJ];
    int   n_objects;
    char *cs;
    size_t cs_len, cs_cap;
    float page_w, page_h;
    int catalog_obj, pages_obj, page_obj, content_obj;
    int font_helv, font_cour;
} pdfw_t;

static void pdfw_cs_append(pdfw_t *w, const char *s, size_t n)
{
    if (w->cs_len + n + 1 > w->cs_cap) {
        size_t nc = w->cs_cap ? w->cs_cap * 2 : 8192;
        while (nc < w->cs_len + n + 1) nc *= 2;
        char *p = (char *) realloc(w->cs, nc);
        if (p == NULL) return;
        w->cs = p; w->cs_cap = nc;
    }
    memcpy(w->cs + w->cs_len, s, n);
    w->cs_len += n;
    w->cs[w->cs_len] = '\0';
}

__attribute__((format(printf, 2, 3)))
static void pdfw_csf(pdfw_t *w, const char *fmt, ...)
{
    char buf[1024];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0 && n < (int) sizeof buf) pdfw_cs_append(w, buf, (size_t) n);
}

__attribute__((format(printf, 2, 3)))
static void pdfw_writef(pdfw_t *w, const char *fmt, ...)
{
    char buf[1024];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) { fwrite(buf, 1, (size_t) n, w->fp); w->bytes += n; }
}

#define PDFY(W, Y) ((W)->page_h - (float)(Y))

static pdfw_t *pdfw_begin(const char *path, float page_w, float page_h)
{
    pdfw_t *w = (pdfw_t *) calloc(1, sizeof(*w));
    if (w == NULL) return NULL;
    w->fp = fopen(path, "wb");
    if (w->fp == NULL) { free(w); return NULL; }
    w->page_w = page_w; w->page_h = page_h;
    static const char hdr[] = "%PDF-1.4\n%\xe2\xe3\xcf\xd3\n";
    fwrite(hdr, 1, sizeof hdr - 1, w->fp);
    w->bytes = (long)(sizeof hdr - 1);
    return w;
}

static int pdfw_next_obj(pdfw_t *w)
{
    if (w->n_objects + 1 >= PDFMAX_OBJ) return -1;
    return ++w->n_objects;
}

static void pdfw_set_stroke(pdfw_t *w, Color c)
{
    pdfw_csf(w, "%.4f %.4f %.4f RG\n", c.r/255.0, c.g/255.0, c.b/255.0);
}
static void pdfw_set_fill(pdfw_t *w, Color c)
{
    pdfw_csf(w, "%.4f %.4f %.4f rg\n", c.r/255.0, c.g/255.0, c.b/255.0);
}
static void pdfw_lw(pdfw_t *w, float lw) { pdfw_csf(w, "%.3f w\n", lw); }
static void pdfw_line(pdfw_t *w, float x1, float y1, float x2, float y2)
{
    pdfw_csf(w, "%.4f %.4f m %.4f %.4f l S\n",
             x1, PDFY(w, y1), x2, PDFY(w, y2));
}
static void pdfw_rect_stroke(pdfw_t *w, float x, float y, float ww, float hh)
{
    pdfw_csf(w, "%.4f %.4f %.4f %.4f re S\n",
             x, PDFY(w, y + hh), ww, hh);
}
static void pdfw_rect_fill(pdfw_t *w, float x, float y, float ww, float hh)
{
    pdfw_csf(w, "%.4f %.4f %.4f %.4f re f\n",
             x, PDFY(w, y + hh), ww, hh);
}
static void pdfw_text(pdfw_t *w, float x, float y_top,
                      const char *s, float fsz, int mono)
{
    if (s == NULL || !*s) return;
    float baseline_y = y_top + fsz * 0.8f;
    pdfw_csf(w, "BT /F%d %.2f Tf %.4f %.4f Td (", mono ? 1 : 0,
             fsz, x, PDFY(w, baseline_y));
    for (const char *p = s; *p; ++p) {
        unsigned char c = (unsigned char) *p;
        if (c == '(' || c == ')' || c == '\\') {
            char b[2] = {'\\', (char) c}; pdfw_cs_append(w, b, 2);
        } else if (c < 0x20 || c > 0x7E) {
            pdfw_cs_append(w, "?", 1);
        } else {
            pdfw_cs_append(w, (const char *) &c, 1);
        }
    }
    pdfw_cs_append(w, ") Tj ET\n", 8);
}
static float pdfw_str_width(const char *s, float fsz, int mono)
{
    if (s == NULL) return 0.0f;
    int n = (int) strlen(s);
    return n * (mono ? 0.60f : 0.55f) * fsz;
}

static int pdfw_end(pdfw_t *w)
{
    w->catalog_obj = pdfw_next_obj(w);
    w->pages_obj   = pdfw_next_obj(w);
    w->page_obj    = pdfw_next_obj(w);
    w->content_obj = pdfw_next_obj(w);
    w->font_helv   = pdfw_next_obj(w);
    w->font_cour   = pdfw_next_obj(w);
    if (w->font_cour < 0) { fclose(w->fp); free(w->cs); free(w); return -1; }

    w->obj_off[w->catalog_obj] = w->bytes;
    pdfw_writef(w, "%d 0 obj\n<< /Type /Catalog /Pages %d 0 R >>\nendobj\n",
                w->catalog_obj, w->pages_obj);
    w->obj_off[w->pages_obj] = w->bytes;
    pdfw_writef(w,
        "%d 0 obj\n<< /Type /Pages /Count 1 /Kids [%d 0 R] >>\nendobj\n",
        w->pages_obj, w->page_obj);
    w->obj_off[w->page_obj] = w->bytes;
    pdfw_writef(w,
        "%d 0 obj\n<< /Type /Page /Parent %d 0 R "
        "/MediaBox [0 0 %.4f %.4f] /Contents %d 0 R "
        "/Resources << /Font << /F0 %d 0 R /F1 %d 0 R >> >> "
        ">>\nendobj\n",
        w->page_obj, w->pages_obj, w->page_w, w->page_h,
        w->content_obj, w->font_helv, w->font_cour);
    w->obj_off[w->content_obj] = w->bytes;
    pdfw_writef(w, "%d 0 obj\n<< /Length %zu >>\nstream\n",
                w->content_obj, w->cs_len);
    if (w->cs_len > 0) {
        fwrite(w->cs, 1, w->cs_len, w->fp);
        w->bytes += (long) w->cs_len;
    }
    pdfw_writef(w, "\nendstream\nendobj\n");
    w->obj_off[w->font_helv] = w->bytes;
    pdfw_writef(w,
        "%d 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica "
        "/Encoding /WinAnsiEncoding >>\nendobj\n", w->font_helv);
    w->obj_off[w->font_cour] = w->bytes;
    pdfw_writef(w,
        "%d 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Courier "
        "/Encoding /WinAnsiEncoding >>\nendobj\n", w->font_cour);

    long xref_off = w->bytes;
    pdfw_writef(w, "xref\n0 %d\n0000000000 65535 f \n",
                w->n_objects + 1);
    for (int i = 1; i <= w->n_objects; ++i)
        pdfw_writef(w, "%010ld 00000 n \n", w->obj_off[i]);
    pdfw_writef(w,
        "trailer\n<< /Size %d /Root %d 0 R >>\nstartxref\n%ld\n%%%%EOF\n",
        w->n_objects + 1, w->catalog_obj, xref_off);
    fclose(w->fp);
    free(w->cs);
    free(w);
    return 0;
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
    pdfw_set_fill(w, (Color){250, 250, 252, 255});
    pdfw_rect_fill(w, MARGIN, MARGIN, page_w - 2*MARGIN, HEADER_H);
    pdfw_set_stroke(w, (Color){80, 80, 80, 255});
    pdfw_lw(w, 0.7f);
    pdfw_rect_stroke(w, MARGIN, MARGIN, page_w - 2*MARGIN, HEADER_H);
    pdfw_set_fill(w, BLACK);
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
    pdfw_set_stroke(w, (Color){100, 100, 110, 255});
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
    pdfw_set_stroke(w, (Color){200, 200, 210, 255});
    pdfw_lw(w, 0.3f);
    pdfw_line(w, pL, mid_y, pR, mid_y);

    pdfw_set_stroke(w, (Color){80, 80, 80, 255});
    pdfw_lw(w, 0.5f);
    pdfw_set_fill(w, BLACK);
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
        Color I_col = (Color){0, 170, 200, 255};
        Color Q_col = (Color){190, 60, 180, 255};
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
    pdfw_set_fill(w, (Color){120, 120, 120, 255});
    pdfw_text(w, MARGIN, page_h - MARGIN - 12, foot, 9.0f, 1);

    return pdfw_end(w);
}

static void usage(void)
{
    fprintf(stderr,
        "usage: iq_annotator <iq_path> [options]\n"
        "  All gen_waterfall options are accepted and forwarded\n"
        "  (--rows, --zoom-khz, --detrend, --detrend-tau-s, --center-hz,\n"
        "   --dc-notch, --dc-notch-bins, --db-min, --db-max,\n"
        "   --power-offset, --start-utc, --elapsed-time, --fft, --hop,\n"
        "   --marks-csv, --show-tm, --full-width)\n"
        "Annotator-specific:\n"
        "  --rate=<Hz>          IQ rate (default = auto from companion .wav)\n"
        "  --width=<px>         Window width  (default 1280)\n"
        "  --height=<px>        Window height (default 900)\n"
        "  --tmp-png=<path>     Rendered PNG path (default /tmp/iq_annotator_<pid>.png)\n");
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
    char   png_path[1100] = "";

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
        } else if (strncmp(a, "--tmp-png=", 10) == 0) {
            snprintf(png_path, sizeof png_path, "%s", a + 10);
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
    fprintf(stderr, "iq_annotator: duration=%.3fs (%zu pairs)\n",
            duration_s, n_pairs);

    if (png_path[0] == '\0') {
        snprintf(png_path, sizeof png_path,
                 "/tmp/iq_annotator_%d.png", (int) getpid());
    }
    fprintf(stderr, "iq_annotator: rendering with gen_waterfall → %s\n",
            png_path);
    if (run_gen_waterfall(iq_path, samp_rate, png_path,
                          passthru, n_passthru) != 0) {
        return 1;
    }

    render_opts_t ropts;
    parse_render_opts(passthru, n_passthru, iq_path, &ropts);

    // ----- raylib window + load PNG -----
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(win_w, win_h, "iq_annotator");
    SetTargetFPS(60);
    SetExitKey(0);
#ifdef __APPLE__
    iq_annotator_install_pinch_monitor();
#endif
    g_ui_font_loaded = load_ttf_from_known_paths();

    // Load the raw IQ samples once — the waveform panel slices into
    // this buffer for any selected box. Half a gig is fine on modern
    // hardware and avoids per-zoom file I/O.
    iq_buf_t iqb = {0};
    if (iq_buf_load(&iqb, iq_path, samp_rate) != 0) {
        fprintf(stderr,
            "iq_annotator: failed to load %s for waveform panel\n",
            iq_path);
    } else {
        fprintf(stderr,
            "iq_annotator: IQ buffer loaded (%zu pairs)\n",
            iqb.n_pairs);
    }

    Image img = LoadImage(png_path);
    if (img.data == NULL) {
        fprintf(stderr, "iq_annotator: LoadImage failed: %s\n", png_path);
        CloseWindow(); return 1;
    }
    int img_w = img.width;
    int img_h = img.height;

    // GL drivers cap texture dimensions (8K-16K on common GPUs); a
    // huge --rows produces a PNG taller than that and the upload
    // silently fails ("texture unloadable"). Slice the PNG into
    // TILE_H-tall row tiles, each uploaded as its own texture, and
    // composite at draw time. With TILE_H = 4096, even --rows=131072
    // is fine (32 tiles × ~15 MB on a 1228-px-wide PNG).
    const int TILE_H = 4096;
    int n_tiles = (img_h + TILE_H - 1) / TILE_H;
    Texture2D *tiles = (Texture2D *) calloc((size_t) n_tiles,
                                            sizeof(Texture2D));
    if (tiles == NULL) {
        fprintf(stderr, "iq_annotator: oom on tile array\n");
        UnloadImage(img); CloseWindow(); return 1;
    }
    for (int t = 0; t < n_tiles; ++t) {
        int y0  = t * TILE_H;
        int h   = (y0 + TILE_H <= img_h) ? TILE_H : (img_h - y0);
        Image sub = ImageFromImage(img, (Rectangle){0, (float) y0,
                                                     (float) img_w,
                                                     (float) h});
        tiles[t] = LoadTextureFromImage(sub);
        UnloadImage(sub);
        if (tiles[t].id == 0) {
            fprintf(stderr,
                "iq_annotator: tile %d upload failed (%dx%d)\n",
                t, img_w, h);
        } else {
            // Point filtering keeps cells crisp at extreme zoom.
            SetTextureFilter(tiles[t], TEXTURE_FILTER_POINT);
        }
    }
    UnloadImage(img);
    fprintf(stderr,
        "iq_annotator: PNG split into %d tile(s) of up to %d rows\n",
        n_tiles, TILE_H);

    int spec_w = img_w - WF_LM - WF_RM;
    int spec_h = img_h - WF_TM - WF_BM;
    if (spec_w < 16 || spec_h < 16) {
        fprintf(stderr,
            "iq_annotator: rendered image too small (%dx%d)\n",
            img_w, img_h);
        for (int t = 0; t < n_tiles; ++t) {
            if (tiles[t].id != 0) UnloadTexture(tiles[t]);
        }
        free(tiles);
        CloseWindow(); return 1;
    }

    double display_bw_hz = (ropts.zoom_khz > 0.0)
        ? ropts.zoom_khz * 1000.0
        : (double) samp_rate;

    fprintf(stderr,
        "iq_annotator: PNG %dx%d, spec %dx%d, BW %.1f kHz\n",
        img_w, img_h, spec_w, spec_h, display_bw_hz / 1e3);

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
    float zoom    = 1.0f;
    float view_x  = 0.0f;     // image pixel at screen x=0
    float view_y  = 0.0f;     // image pixel at screen y=0
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

        // Spectrogram-view pinch/scroll (only when cursor is NOT
        // in the waveform panel — the panel uses identical input
        // semantics but applies to its own time-zoom/pan state).
        if (!in_panel_for_input && pinch != 0.0f) {
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
        if (!in_panel_for_input
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
        // Zoom resets.
        if (IsKeyPressed(KEY_ZERO) || IsKeyPressed(KEY_KP_0)) {
            zoom = 1.0f; view_x = 0; view_y = 0;
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
        int   bar_h_clamp = 2 * (STATUS_PT + 6);
        int   spec_screen_h = wf_open
            ? (sh - wf_panel_h) : (sh - bar_h_clamp);
        if (spec_screen_h < 32) spec_screen_h = 32;
        float visible_w = sw / zoom;
        float visible_h = spec_screen_h / zoom;
        if (view_x < 0) view_x = 0;
        if (view_y < 0) view_y = 0;
        if (visible_w >= img_w) view_x = 0;
        else if (view_x > img_w - visible_w) view_x = img_w - visible_w;
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
        // P → write the current waveform panel to a vector PDF
        // alongside the .iq. Deferred until after wf_t_lo/wf_t_hi
        // and box_info are computed later in the frame.
        int pdf_requested = IsKeyPressed(KEY_P);
        if (IsKeyPressed(KEY_Q)) break;

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
        // Spec is drawn from screen y=0 down to spec_screen_h; below
        // that is the panel (when open) or the status bar.
        int   in_spec = (img_x >= WF_LM && img_x < WF_LM + spec_w
                      && img_y >= WF_TM && img_y < WF_TM + spec_h
                      && (int) m.y < spec_screen_h
                      && !in_panel_now);
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
            box_normalize(&boxes.items[resize_box]);
            snprintf(status, sizeof status,
                "resized box %d (t=%.3f..%.3fs, f=%.0f..%.0f Hz)",
                resize_box,
                boxes.items[resize_box].t0_s, boxes.items[resize_box].t1_s,
                boxes.items[resize_box].f_lo_hz, boxes.items[resize_box].f_hi_hz);
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
                snprintf(status, sizeof status,
                    "added box %d (%s, %.3fs × %.0fHz)",
                    boxes.n - 1, b.label,
                    b.t1_s - b.t0_s, b.f_hi_hz - b.f_lo_hz);
            }
        }

        // ----- render -----
        BeginDrawing();
        ClearBackground(BLACK);

        // Draw the PNG by tile. Each tile covers TILE_H rows of the
        // image (yielding sub-textures small enough to stay under the
        // GPU's max texture size); composite the visible portion of
        // each tile that intersects the view.
        float src_w_clamped = (view_x + visible_w > img_w)
            ? (img_w - view_x) : visible_w;
        if (src_w_clamped < 0) src_w_clamped = 0;
        double view_y_top = view_y;
        double view_y_bot = view_y + visible_h;
        if (view_y_bot > img_h) view_y_bot = img_h;
        int t_first = (int)(view_y_top / TILE_H);
        int t_last  = (int)((view_y_bot - 1) / TILE_H);
        if (t_first < 0)         t_first = 0;
        if (t_last  >= n_tiles)  t_last  = n_tiles - 1;
        for (int t = t_first; t <= t_last; ++t) {
            if (tiles[t].id == 0) continue;
            int tile_y_img = t * TILE_H;
            int tile_h     = (tile_y_img + TILE_H <= img_h)
                             ? TILE_H : (img_h - tile_y_img);
            double vis_top_img = (view_y_top > tile_y_img)
                                 ? view_y_top : (double) tile_y_img;
            double vis_bot_img = (view_y_bot < tile_y_img + tile_h)
                                 ? view_y_bot : (double)(tile_y_img + tile_h);
            if (vis_bot_img <= vis_top_img) continue;
            Rectangle src = {
                .x = (float) view_x,
                .y = (float)(vis_top_img - tile_y_img),
                .width  = src_w_clamped,
                .height = (float)(vis_bot_img - vis_top_img),
            };
            Rectangle dst = {
                .x = 0.0f,
                .y = (float)((vis_top_img - view_y_top) * zoom),
                .width  = src_w_clamped * zoom,
                .height = (float)((vis_bot_img - vis_top_img) * zoom),
            };
            DrawTexturePro(tiles[t], src, dst, (Vector2){0, 0}, 0.0f, WHITE);
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
                snprintf(status, sizeof status,
                    "wrote %s", pdf_path);
            } else {
                snprintf(status, sizeof status,
                    "PDF write failed");
            }
        }

        // Status bar — two lines now: top = box info, bottom = cursor
        // / keys / status messages.
        int bar_h = 2 * (STATUS_PT + 6);
        DrawRectangle(0, sh - bar_h, sw, bar_h, (Color){0, 0, 0, 210});
        if (box_info[0]) {
            draw_text(box_info, 6, sh - bar_h + 4, STATUS_PT,
                      (boxes.selected >= 0 && boxes.selected == box_for_status)
                        ? YELLOW : ORANGE);
        }
        char info[512];
        if (in_spec) {
            snprintf(info, sizeof info,
                "cursor t=%.3fs  f=%+.0f Hz  zoom=%.2fx  boxes=%d  hover=%d sel=%d  | drag=new pinch=zoom +/-=5x@cursor-anchor scroll=pan W=wave I=ch P=pdf T=label S=save L=load 0=reset Del=del-hover Q=quit",
                cursor_t, cursor_f, (double) zoom,
                boxes.n, hovered, boxes.selected);
        } else {
            snprintf(info, sizeof info,
                "zoom=%.2fx  boxes=%d  | drag=new pinch=zoom +/-=5x@cursor-anchor scroll=pan W=wave I=ch P=pdf T=label S=save L=load 0=reset Del=del-hover Q=quit",
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
            // Background.
            DrawRectangle(0, wf_y0, sw, wf_panel_h, (Color){15, 15, 20, 230});
            DrawLine(0, wf_y0, sw, wf_y0, GRAY);

            // Layout inside the panel — bigger margins so the larger
            // TTF labels fit.
            int pL = 96, pR = 8, pT = 6, pB = 52;
            int plot_x0 = pL;
            int plot_x1 = sw - pR;
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

                // Cursor indicator: short upward-pointing arrow on the
                // time axis at the spectrogram cursor's time position,
                // so the operator can see where the mouse over the
                // waterfall above maps into the waveform below. Only
                // drawn while the cursor is in the spectrogram and its
                // time falls inside the panel's visible window.
                if (in_spec
                    && cursor_t >= wf_t_lo - 1e-12
                    && cursor_t <= wf_t_hi + 1e-12) {
                    int cx = plot_x0
                           + (int)((cursor_t - wf_t_lo) / span * plot_w);
                    if (cx >= plot_x0 && cx <= plot_x1) {
                        // Triangle: tip up (apex INSIDE plot, base on
                        // the axis line below). 10 px tall × 12 px wide.
                        Vector2 apex = {(float) cx, (float)(plot_y1 - 1)};
                        Vector2 bl   = {(float)(cx - 6), (float)(plot_y1 + 9)};
                        Vector2 br   = {(float)(cx + 6), (float)(plot_y1 + 9)};
                        // raylib's DrawTriangle wants CCW order.
                        DrawTriangle(apex, br, bl, YELLOW);
                        // Thin guide line up into the plot for context.
                        DrawLine(cx, plot_y0, cx, plot_y1,
                                 (Color){255, 230, 60, 60});
                    }
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

        EndDrawing();
    }

    for (int t = 0; t < n_tiles; ++t) {
        if (tiles[t].id != 0) UnloadTexture(tiles[t]);
    }
    free(tiles);
    iq_buf_free(&iqb);
    if (g_ui_font_loaded) UnloadFont(g_ui_font);
    CloseWindow();
    return 0;
}
