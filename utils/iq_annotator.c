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

#include <errno.h>
#include <libgen.h>
#include <math.h>
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

    Image img = LoadImage(png_path);
    if (img.data == NULL) {
        fprintf(stderr, "iq_annotator: LoadImage failed: %s\n", png_path);
        CloseWindow(); return 1;
    }
    Texture2D tex = LoadTextureFromImage(img);
    if (tex.id == 0) {
        fprintf(stderr, "iq_annotator: LoadTextureFromImage failed\n");
        UnloadImage(img); CloseWindow(); return 1;
    }
    int img_w = img.width;
    int img_h = img.height;
    UnloadImage(img);

    int spec_w = img_w - WF_LM - WF_RM;
    int spec_h = img_h - WF_TM - WF_BM;
    if (spec_w < 16 || spec_h < 16) {
        fprintf(stderr,
            "iq_annotator: rendered image too small (%dx%d)\n",
            img_w, img_h);
        UnloadTexture(tex); CloseWindow(); return 1;
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
    int   dragging = 0;
    Vector2 drag_start = {0, 0};
    char  status[256] = "";
    int   label_cycle_idx = 0;

    while (!WindowShouldClose()) {
        int sw = GetScreenWidth();
        int sh = GetScreenHeight();
        Vector2 m = GetMousePosition();

        // ----- input -----
        // Trackpad pinch → zoom centred on cursor. NSEvent.magnification
        // accumulates per-event deltas in [-1, +1]; convert to a
        // multiplicative zoom factor via exp() so successive pinches
        // compose smoothly. raylib/GLFW drop magnify events, so we
        // pull the accumulated delta from the macOS-side global.
        float pinch = 0.0f;
#ifdef __APPLE__
        pinch = g_iq_annotator_pinch_delta;
        g_iq_annotator_pinch_delta = 0.0f;
#endif
        if (pinch != 0.0f) {
            float img_x_under = view_x + m.x / zoom;
            float img_y_under = view_y + m.y / zoom;
            float new_zoom = zoom * expf(pinch);
            if (new_zoom < 0.1f)  new_zoom = 0.1f;
            if (new_zoom > 32.0f) new_zoom = 32.0f;
            zoom = new_zoom;
            view_x = img_x_under - m.x / zoom;
            view_y = img_y_under - m.y / zoom;
        }
        // Two-finger scroll (trackpad) or mouse wheel → pan. Vertical
        // wheel pans the y-axis; horizontal wheel pans x. macOS sends
        // both axes via GetMouseWheelMoveV.
        Vector2 wheel_v = GetMouseWheelMoveV();
        if (wheel_v.x != 0.0f || wheel_v.y != 0.0f) {
            // Sign: wheel.y > 0 = scroll up. We want scroll-up to
            // reveal earlier rows (move view UP), which means
            // view_y decreasing.
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
        }
        // Clamp view to image extents.
        float visible_w = sw / zoom;
        float visible_h = sh / zoom;
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
        if (IsKeyPressed(KEY_Q)) break;

        // Cursor → image px → (t, f).
        float img_x_f = view_x + m.x / zoom;
        float img_y_f = view_y + m.y / zoom;
        int   img_x   = (int) img_x_f;
        int   img_y   = (int) img_y_f;
        int   in_spec = (img_x >= WF_LM && img_x < WF_LM + spec_w
                      && img_y >= WF_TM && img_y < WF_TM + spec_h);
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

        if (in_spec && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            if (hovered >= 0) {
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

        // Draw the PNG: source = (view_x, view_y, visible_w, visible_h);
        // dest = the whole window. Clamp source to image bounds so we
        // don't sample undefined texels.
        float src_w_clamped = (view_x + visible_w > img_w)
            ? (img_w - view_x) : visible_w;
        float src_h_clamped = (view_y + visible_h > img_h)
            ? (img_h - view_y) : visible_h;
        if (src_w_clamped < 0) src_w_clamped = 0;
        if (src_h_clamped < 0) src_h_clamped = 0;
        Rectangle src = {
            .x = view_x, .y = view_y,
            .width = src_w_clamped, .height = src_h_clamped,
        };
        Rectangle dst = {
            .x = 0, .y = 0,
            .width  = src_w_clamped * zoom,
            .height = src_h_clamped * zoom,
        };
        DrawTexturePro(tex, src, dst, (Vector2){0, 0}, 0.0f, WHITE);

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
            DrawText(tag, x_lo + 2, y_lo - 12, 10, col);
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

        // Status bar.
        int bar_h = 24;
        DrawRectangle(0, sh - bar_h, sw, bar_h, (Color){0, 0, 0, 200});
        char info[512];
        if (in_spec) {
            snprintf(info, sizeof info,
                "cursor t=%.3fs  f=%+.0f Hz   zoom=%.2fx   hover=%d sel=%d   keys: drag=new pinch=zoom scroll=pan T=label S=save L=load 0=reset Del=del-hover Q=quit",
                cursor_t, cursor_f, (double) zoom, hovered, boxes.selected);
        } else {
            snprintf(info, sizeof info,
                "zoom=%.2fx   boxes=%d   keys: drag=new pinch=zoom scroll=pan T=label S=save L=load 0=reset Del=del-hover Q=quit",
                (double) zoom, boxes.n);
        }
        DrawText(info, 6, sh - bar_h + 6, 11, LIGHTGRAY);
        if (status[0]) {
            int tw = MeasureText(status, 11);
            DrawText(status, sw - tw - 8, sh - bar_h + 6, 11, YELLOW);
        }

        EndDrawing();
    }

    UnloadTexture(tex);
    CloseWindow();
    return 0;
}
