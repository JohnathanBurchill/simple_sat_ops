/*

    Simple Satellite Operations  utils/telemetry_browser.c

    The beacons as curves. A small raylib GUI that reads every beacon in
    the packet database -- the satellite's own and the extended-beacon
    blob's -- and plots any of its telemetry fields against time, one
    stacked panel per field, so a question like "was the battery colder
    on the passes where the solar array dropped out" is a matter of
    ticking two boxes.

    The field list on the left is the catalogue in src/beacon/
    beacon_series.c, grouped by subsystem. Fields the basic beacon does
    not carry are marked "ext": only the extended-beacon blob downlinks
    the ADCS state, the attitude and the per-channel solar
    measurements, so those series exist only for the stretch of the
    mission where the blob was running.

    What is NOT plotted, deliberately: a field whose value in a packet
    is a subsystem's "no reading" marker (the blob's -9999, a dead
    thermistor's INT16_MAX, the MPI's -99 for "not active"). Those are
    dropped rather than drawn, so a gap in a curve means the satellite
    had nothing to tell us rather than that it reported minus ten
    thousand volts.

    Time: the axis is real time, and the beacons arrive in clumps -- a
    few minutes of them per pass, then hours of nothing. So no line is
    drawn across a gap longer than a few minutes, and [ and ] jump from
    one clump to the next, which is the navigation that actually
    matches the data. The moments are ground reception times, the same
    clock the rest of these tools use.

    Keys:
      up/down      move in the field list        space/enter  plot or unplot it
      left/right   pan through time              scroll       zoom about the cursor
      [ ]          previous / next pass          a            all of a group
      g            the whole record in view      n            none
      e            write the visible window to telemetry.csv
      F5           re-read the database          q            quit

    Read-only on the database and safe to run while a receiver fills it.

    Usage:
      telemetry_browser [--db=<packet_db.sqlite>] [--fields=<k,k,...>]

    With no --db the default store is used ($SSO_PACKET_DB, else the
    FrontierSat root's packet_db.sqlite). --fields opens with those
    field keys plotted instead of the default few; the key of each field
    is the first word of its row in the list.

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

#include "beacon_cts1.h"
#include "beacon_series.h"
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
    if (sso_version_handle(argc, argv, "telemetry_browser")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "telemetry_browser: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

// The left column, the help line along the bottom, and how tall one
// row of the field list is.
#define LEFT_W    270
#define FOOTER_H  26
#define ROW_H     19

// No line is drawn between two samples further apart than this: the
// beacons come a clump per pass, and joining one pass to the next
// draws a straight line across hours of nothing that reads as data.
// Five minutes is longer than any beacon interval seen in flight (the
// satellite's own is about 30 s, the blob's whatever it was started
// with) and far shorter than the gap between passes.
#define GAP_MS    (5.0 * 60.0 * 1000.0)

// Two samples further apart than this belong to different passes, for
// the [ and ] keys. Longer than GAP_MS so a beacon that went missing
// mid-pass does not split it in two.
#define PASS_GAP_MS (20.0 * 60.0 * 1000.0)

// How much of a pass is in view when [ or ] jumps to one, and the
// narrowest and widest the window can be zoomed.
#define PASS_PAD_MS   (4.0 * 60.0 * 1000.0)
#define MIN_SPAN_MS   (10.0 * 1000.0)

// How many series can be plotted at once. More than about six panels
// makes each one too short to read anything off, so the limit is a
// kindness as much as an allocation.
#define MAX_PLOTS 8

// One beacon: when it arrived and its bytes. The whole payload is kept
// rather than the fields pulled out at load time, so ticking a new
// field costs no reload.
typedef struct {
    double   ts_ms;
    uint16_t len;
    uint8_t  is_ext;
    uint8_t  payload[204];   // 198 or 202 for extended, 130 or 134 for basic
} sample_t;

static sample_t *g_s   = NULL;
static int       g_n   = 0;
static int       g_nb  = 0;   // of those: basic beacons
static int       g_nx  = 0;   // of those: extended ones

// The passes, as index ranges into g_s.
typedef struct { int first, last; } pass_t;
static pass_t *g_pass  = NULL;
static int     g_npass = 0;

// ---- the font (the pattern the other raylib tools use) ---------------------

static Font  g_font;
static int   g_font_loaded = 0;
static float g_font_spacing = 1.0f;

static int load_ui_font(void)
{
    const char *home = getenv("HOME");
    const char *xdg = getenv("XDG_DATA_HOME");
    char cands[5][1024];
    int n = 0;
    if (xdg && xdg[0])
        snprintf(cands[n++], sizeof cands[0],
                 "%s/simple_sat_ops/SourceCodePro-Regular.ttf", xdg);
    if (home && home[0]) {
        snprintf(cands[n++], sizeof cands[0],
                 "%s/.local/share/simple_sat_ops/SourceCodePro-Regular.ttf", home);
        snprintf(cands[n++], sizeof cands[0],
                 "%s/src/simple_sat_ops/assets/SourceCodePro-Regular.ttf", home);
    }
    snprintf(cands[n++], sizeof cands[0], "assets/SourceCodePro-Regular.ttf");
    snprintf(cands[n++], sizeof cands[0], "../assets/SourceCodePro-Regular.ttf");

    int cp[95];
    for (int c = 0x20; c <= 0x7E; c++) cp[c - 0x20] = c;
    for (int i = 0; i < n; i++) {
        if (!FileExists(cands[i])) continue;
        g_font = LoadFontEx(cands[i], 48, cp, 95);
        if (g_font.texture.id != 0) {
            SetTextureFilter(g_font.texture, TEXTURE_FILTER_BILINEAR);
            return 1;
        }
    }
    fprintf(stderr, "telemetry_browser: TTF font not found; "
                    "using raylib default\n");
    return 0;
}

static void draw_text(const char *s, int x, int y, int size, Color c)
{
    if (g_font_loaded)
        DrawTextEx(g_font, s, (Vector2){ (float) x, (float) y }, (float) size,
                   g_font_spacing, c);
    else
        DrawText(s, x, y, size, c);
}

static int text_width(const char *s, int size)
{
    if (g_font_loaded)
        return (int) MeasureTextEx(g_font, s, (float) size, g_font_spacing).x;
    return MeasureText(s, size);
}

static void fmt_utc(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    struct tm tm;
    gmtime_r(&secs, &tm);
    strftime(out, n, "%Y-%m-%d %H:%M:%S", &tm);
}

static void fmt_utc_short(double ts_ms, char *out, size_t n)
{
    time_t secs = (time_t) (ts_ms / 1000.0);
    struct tm tm;
    gmtime_r(&secs, &tm);
    strftime(out, n, "%m-%d %H:%M", &tm);
}

// ---- loading ---------------------------------------------------------------

static void free_samples(void)
{
    free(g_s);    g_s = NULL;    g_n = 0; g_nb = 0; g_nx = 0;
    free(g_pass); g_pass = NULL; g_npass = 0;
}

// Split the loaded samples into passes: runs of beacons no further
// apart than PASS_GAP_MS.
static void build_passes(void)
{
    free(g_pass);
    g_pass = NULL;
    g_npass = 0;
    if (g_n == 0) return;
    int cap = 0;
    int i = 0;
    while (i < g_n) {
        int j = i;
        while (j + 1 < g_n && g_s[j + 1].ts_ms - g_s[j].ts_ms <= PASS_GAP_MS) j++;
        if (g_npass == cap) {
            int ncap = cap ? cap * 2 : 128;
            pass_t *t = (pass_t *) realloc(g_pass, (size_t) ncap * sizeof *t);
            if (t == NULL) break;
            g_pass = t;
            cap = ncap;
        }
        g_pass[g_npass].first = i;
        g_pass[g_npass].last  = j;
        g_npass++;
        i = j + 1;
    }
}

// Every beacon in the database, in time order. Both kinds: the basic
// beacon is recognised by its stored type, the extended one by its
// shape, because the extended beacons decoded before the ground station
// knew that packet type are stored as "unknown" (the same reason
// packet_browser has its own predicate for them).
static int load_samples(const char *db_path)
{
    free_samples();

    sqlite3 *db = NULL;
    // Read-write, not read-only: under SQLITE_OPEN_READONLY a WAL
    // database falls back to journal emulation whose read lock blocks
    // the receiver. Nothing here writes.
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READWRITE, NULL) != SQLITE_OK) {
        fprintf(stderr, "telemetry_browser: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "?");
        if (db != NULL) sqlite3_close(db);
        return -1;
    }
    sqlite3_busy_timeout(db, 5000);

    // Frames whose decode went wrong are left out. A series is read as
    // numbers, and one frame of noise in it is a spike of twenty
    // million degrees through an otherwise readable temperature curve;
    // the operator's live panel makes the opposite trade on purpose
    // (see beacon_magic_intact). Reed-Solomon giving up and the CSP
    // CRC32 failing are both grounds here, and beacon_magic_intact
    // below catches the rest.
    //
    // The two kinds of beacon are bracketed together so those
    // exclusions apply to both: AND binds tighter than OR, and without
    // the brackets they would have applied only to the second.
    const char *sql =
        "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, payload "
        "FROM packet "
        "WHERE rs_errs != -2 AND crc_status != 0 "
        "  AND ((packet_type = 1 AND length(payload) IN (130, 134)) "
        "       OR (length(payload) IN (198, 202) "
        "           AND substr(payload, 1, 1) = x'20' "
        "           AND (substr(payload, 2, 4) = x'43545331' "
        "                OR substr(payload, 127, 2) = x'2058'))) "
        "ORDER BY ts_received";

    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "telemetry_browser: query failed: %s\n",
                sqlite3_errmsg(db));
        sqlite3_close(db);
        return -1;
    }

    int cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const uint8_t *pl = (const uint8_t *) sqlite3_column_blob(st, 1);
        int pln = sqlite3_column_bytes(st, 1);
        if (pl == NULL || pln <= 0 || pln > (int) sizeof g_s[0].payload) continue;
        const int is_ext = beacon_is_extended(pl, (size_t) pln);
        const int is_bas = beacon_is_basic(pl, (size_t) pln);
        if (!is_ext && !is_bas) continue;
        // The last of the decode checks: both magic fields exactly as
        // the satellite writes them. This is what keeps the 130 frames
        // in the operational store that are the right length and pass
        // their CRC but hold noise out of the curves.
        if (!beacon_magic_intact(pl, (size_t) pln)) continue;

        if (g_n == cap) {
            int ncap = cap ? cap * 2 : 4096;
            sample_t *t = (sample_t *) realloc(g_s, (size_t) ncap * sizeof *t);
            if (t == NULL) break;
            g_s = t;
            cap = ncap;
        }
        sample_t *s = &g_s[g_n++];
        s->ts_ms  = sqlite3_column_double(st, 0);
        s->len    = (uint16_t) pln;
        s->is_ext = (uint8_t) (is_ext ? 1 : 0);
        memcpy(s->payload, pl, (size_t) pln);
        if (is_ext) g_nx++; else g_nb++;
    }
    sqlite3_finalize(st);
    sqlite3_close(db);
    build_passes();
    return g_n;
}

// ---- the view --------------------------------------------------------------

// Which fields are plotted, in the order they were picked, and where
// the list cursor sits. plots holds indices into BEACON_SERIES_FIELDS.
static int    g_plot[MAX_PLOTS];
static int    g_nplot = 0;
static int    g_cursor = 0;
static float  g_list_scroll = 0.0f;

// The time window on screen.
static double g_t0 = 0.0, g_t1 = 0.0;

static char   g_status[200] = "";
static float  g_status_left = 0.0f;

static int is_plotted(int field_idx)
{
    for (int i = 0; i < g_nplot; i++) if (g_plot[i] == field_idx) return 1;
    return 0;
}

static void toggle_plot(int field_idx)
{
    for (int i = 0; i < g_nplot; i++) {
        if (g_plot[i] != field_idx) continue;
        for (int k = i; k + 1 < g_nplot; k++) g_plot[k] = g_plot[k + 1];
        g_nplot--;
        return;
    }
    if (g_nplot >= MAX_PLOTS) {
        snprintf(g_status, sizeof g_status,
                 "%d panels is as many as will fit; unpick one first", MAX_PLOTS);
        g_status_left = 4.0f;
        return;
    }
    g_plot[g_nplot++] = field_idx;
}

static void view_all(void)
{
    if (g_n == 0) { g_t0 = 0; g_t1 = 1; return; }
    g_t0 = g_s[0].ts_ms;
    g_t1 = g_s[g_n - 1].ts_ms;
    const double pad = (g_t1 - g_t0) * 0.02 + 1000.0;
    g_t0 -= pad;
    g_t1 += pad;
}

// Put pass p in the window. Clamped to the passes that exist, so
// holding ] at the end of the record does nothing rather than walking
// off it.
static void view_pass(int p)
{
    if (g_npass == 0) return;
    if (p < 0) p = 0;
    if (p >= g_npass) p = g_npass - 1;
    g_t0 = g_s[g_pass[p].first].ts_ms - PASS_PAD_MS;
    g_t1 = g_s[g_pass[p].last].ts_ms  + PASS_PAD_MS;
}

// The pass nearest the middle of the window, which is what [ and ]
// step from.
static int current_pass(void)
{
    if (g_npass == 0) return 0;
    const double mid = 0.5 * (g_t0 + g_t1);
    int best = 0;
    double bestd = 1e30;
    for (int p = 0; p < g_npass; p++) {
        const double c = 0.5 * (g_s[g_pass[p].first].ts_ms
                              + g_s[g_pass[p].last].ts_ms);
        const double d = fabs(c - mid);
        if (d < bestd) { bestd = d; best = p; }
    }
    return best;
}

// ---- plotting --------------------------------------------------------------

// The colours the panels cycle through. Chosen to stay apart on the
// dark background and to read in the cursor line beside each other.
static const Color SERIES_C[] = {
    { 120, 200, 255, 255 },   // pale blue
    { 255, 180,  90, 255 },   // amber
    { 150, 240, 170, 255 },   // green
    { 240, 140, 200, 255 },   // pink
    { 230, 230, 130, 255 },   // yellow
    { 180, 170, 255, 255 },   // violet
    { 255, 140, 130, 255 },   // salmon
    { 140, 230, 230, 255 },   // cyan
};

// A rounded step for the y axis: 1, 2 or 5 times a power of ten, the
// largest that still gives at least two labelled lines.
static double nice_step(double span, int want)
{
    if (span <= 0.0 || want < 1) return 1.0;
    const double raw = span / (double) want;
    const double mag = pow(10.0, floor(log10(raw)));
    const double norm = raw / mag;
    if (norm < 1.5) return mag;
    if (norm < 3.5) return 2.0 * mag;
    if (norm < 7.5) return 5.0 * mag;
    return 10.0 * mag;
}

// Draw one field's panel. Returns 1 if anything was in view.
static int draw_panel(const bs_field_t *f, Color c, int x, int y, int w, int h,
                      int have_cursor, double cursor_ms)
{
    DrawRectangle(x, y, w, h, (Color){ 26, 26, 32, 255 });
    DrawRectangleLines(x, y, w, h, (Color){ 52, 52, 62, 255 });

    const int pad_l = 58, pad_r = 10, pad_t = 17, pad_b = 4;
    const int px = x + pad_l, py = y + pad_t;
    const int pw = w - pad_l - pad_r, ph = h - pad_t - pad_b;
    if (pw < 10 || ph < 10) return 0;

    // One pass over the samples in the window to find the range. Doing
    // it per frame keeps the scaling honest as the window moves -- the
    // point of a plot like this is to see the shape of what is in view,
    // not of the whole four months.
    double lo = 1e300, hi = -1e300;
    int    seen = 0;
    for (int i = 0; i < g_n; i++) {
        if (g_s[i].ts_ms < g_t0 || g_s[i].ts_ms > g_t1) continue;
        double v = 0.0;
        if (!beacon_series_value(f, g_s[i].payload, g_s[i].len, &v)) continue;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        seen++;
    }

    char head[160];
    if (seen == 0) {
        snprintf(head, sizeof head, "%s  (%s)  -- nothing in this window",
                 f->label, f->unit[0] ? f->unit : "count");
        draw_text(head, x + 8, y + 3, 12, GRAY);
        return 0;
    }

    // A flat series would otherwise be a line on the axis with no
    // scale; give it a little room either side so the value reads.
    if (hi - lo < 1e-9) {
        const double pad = (fabs(hi) > 1.0) ? fabs(hi) * 0.05 : 0.5;
        lo -= pad;
        hi += pad;
    } else {
        const double pad = (hi - lo) * 0.08;
        lo -= pad;
        hi += pad;
    }

    #define SX(t) (px + (int) ((double) pw * ((t) - g_t0) / (g_t1 - g_t0)))
    #define SY(v) (py + ph - (int) ((double) ph * ((v) - lo) / (hi - lo)))

    // The y grid, labelled on the left.
    const double step = nice_step(hi - lo, 3);
    for (double gv = ceil(lo / step) * step; gv <= hi; gv += step) {
        const int gy = SY(gv);
        if (gy < py || gy > py + ph) continue;
        DrawLine(px, gy, px + pw, gy, (Color){ 44, 44, 54, 255 });
        char lab[32];
        // Enough decimals for the step to show, and no more.
        const int dp = step >= 10.0 ? 0 : step >= 1.0 ? 1 : step >= 0.1 ? 2 : 3;
        snprintf(lab, sizeof lab, "%.*f", dp, gv);
        draw_text(lab, x + pad_l - 6 - text_width(lab, 11), gy - 5, 11, GRAY);
    }

    // The series. A dot per sample, and a line between neighbours close
    // enough in time to belong to the same pass.
    int    have_prev = 0;
    double prev_t = 0.0;
    Vector2 prev = {0};
    for (int i = 0; i < g_n; i++) {
        const double t = g_s[i].ts_ms;
        if (t < g_t0 || t > g_t1) { have_prev = 0; continue; }
        double v = 0.0;
        if (!beacon_series_value(f, g_s[i].payload, g_s[i].len, &v)) {
            have_prev = 0;
            continue;
        }
        const Vector2 p = { (float) SX(t), (float) SY(v) };
        if (have_prev && t - prev_t <= GAP_MS)
            DrawLineEx(prev, p, 1.6f, c);
        // At four months in one window the dots merge into a band, which
        // is the right picture; when zoomed into a pass they are the
        // individual beacons.
        if (pw > 0 && (double) pw / (double) seen > 3.0)
            DrawCircleV(p, 2.0f, c);
        prev = p;
        prev_t = t;
        have_prev = 1;
    }

    // The heading: the field, its unit, and the range in view.
    snprintf(head, sizeof head, "%s  (%s)      %.3g to %.3g",
             f->label, f->unit[0] ? f->unit : "count", lo, hi);
    draw_text(head, x + 8, y + 3, 12, c);

    // The cursor's own value: the sample nearest it, so the number
    // shown is one the satellite actually sent rather than an
    // interpolation between two of them.
    if (have_cursor) {
        int best = -1;
        double bestd = 1e300, bestv = 0.0;
        for (int i = 0; i < g_n; i++) {
            if (g_s[i].ts_ms < g_t0 || g_s[i].ts_ms > g_t1) continue;
            double v = 0.0;
            if (!beacon_series_value(f, g_s[i].payload, g_s[i].len, &v)) continue;
            const double d = fabs(g_s[i].ts_ms - cursor_ms);
            if (d < bestd) { bestd = d; best = i; bestv = v; }
        }
        if (best >= 0) {
            const Vector2 p = { (float) SX(g_s[best].ts_ms),
                                (float) SY(bestv) };
            DrawCircleLines((int) p.x, (int) p.y, 5.0f, RAYWHITE);
            char val[48];
            snprintf(val, sizeof val, "%.4g %s", bestv, f->unit);
            const int tw = text_width(val, 12);
            int vx = (int) p.x + 8;
            if (vx + tw + 6 > x + w) vx = (int) p.x - 8 - tw;
            DrawRectangle(vx - 3, (int) p.y - 8, tw + 6, 16,
                          (Color){ 18, 18, 22, 210 });
            draw_text(val, vx, (int) p.y - 6, 12, RAYWHITE);
        }
    }
    #undef SX
    #undef SY
    return 1;
}

// ---- export ----------------------------------------------------------------

// The visible window, as one CSV row per beacon and one column per
// plotted field. A field with no reading in that beacon is left empty
// rather than filled with a zero.
static int export_csv(const char *path)
{
    FILE *fp = fopen(path, "w");
    if (fp == NULL) return -1;
    fprintf(fp, "ts_utc,beacon");
    for (int k = 0; k < g_nplot; k++)
        fprintf(fp, ",%s", BEACON_SERIES_FIELDS[g_plot[k]].key);
    fputc('\n', fp);

    long rows = 0;
    for (int i = 0; i < g_n; i++) {
        if (g_s[i].ts_ms < g_t0 || g_s[i].ts_ms > g_t1) continue;
        char ts[32];
        fmt_utc(g_s[i].ts_ms, ts, sizeof ts);
        fprintf(fp, "%sZ,%s", ts, g_s[i].is_ext ? "ext" : "basic");
        for (int k = 0; k < g_nplot; k++) {
            double v = 0.0;
            if (beacon_series_value(&BEACON_SERIES_FIELDS[g_plot[k]],
                                    g_s[i].payload, g_s[i].len, &v))
                fprintf(fp, ",%.6g", v);
            else
                fputc(',', fp);
        }
        fputc('\n', fp);
        rows++;
    }
    fclose(fp);
    return (int) rows;
}

// ---- main ------------------------------------------------------------------

// Fire once on press, then rapidly while held.
static int key_repeat(int key, float *cooldown)
{
    const float delay = 0.30f, rate = 0.05f;
    if (IsKeyPressed(key)) { *cooldown = delay; return 1; }
    if (!IsKeyDown(key)) return 0;
    *cooldown -= GetFrameTime();
    if (*cooldown <= 0.0f) { *cooldown = rate; return 1; }
    return 0;
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "telemetry_browser")) return 0;

    const char *db_arg = NULL;
    const char *fields_arg = NULL;
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--db=", 5) == 0) db_arg = argv[i] + 5;
        else if (strncmp(argv[i], "--fields=", 9) == 0) fields_arg = argv[i] + 9;
        else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: telemetry_browser [--db=<packet_db.sqlite>]"
                   " [--fields=<key,key,...>]\n"
                   "Plot FrontierSat's beacon telemetry against time, out of"
                   " the packet DB.\n"
                   "Both beacons are read: the satellite's own and the"
                   " extended-beacon blob's.\n\n"
                   "Fields (key -- what it is):\n");
            bs_group_t last = (bs_group_t) -1;
            for (size_t k = 0; k < BEACON_SERIES_FIELD_N; k++) {
                const bs_field_t *f = &BEACON_SERIES_FIELDS[k];
                if (f->group != last) {
                    printf("\n  %s:\n", beacon_series_group_name(f->group));
                    last = f->group;
                }
                printf("    %-13s %s%s%s%s\n", f->key, f->label,
                       f->unit[0] ? " (" : "", f->unit, f->unit[0] ? ")" : "");
            }
            printf("\nFields marked in the list as \"ext\" come only from the"
                   " extended-beacon blob.\n");
            return 0;
        } else {
            fprintf(stderr, "telemetry_browser: unknown option '%s'"
                            " (try --help)\n", argv[i]);
            return 1;
        }
    }

    char db_default[1024];
    const char *db_path = db_arg;
    if (db_path == NULL) {
        if (packet_db_default_path(db_default, sizeof db_default) != 0) {
            fprintf(stderr, "telemetry_browser: no DB path "
                            "(set $SSO_PACKET_DB or pass --db=)\n");
            return 1;
        }
        db_path = db_default;
    }

    fprintf(stderr, "telemetry_browser: reading beacons from %s ...\n", db_path);
    if (load_samples(db_path) < 0) return 1;
    if (g_n == 0) {
        fprintf(stderr, "telemetry_browser: no beacons in the database.\n");
        return 1;
    }
    fprintf(stderr, "telemetry_browser: %d beacons (%d basic, %d extended)"
                    " over %d passes.\n", g_n, g_nb, g_nx, g_npass);

    // What to open with: the operator's pick, or a few fields that say
    // most about the satellite's health in one screen.
    if (fields_arg != NULL) {
        char buf[512];
        snprintf(buf, sizeof buf, "%s", fields_arg);
        for (char *tok = strtok(buf, ","); tok != NULL; tok = strtok(NULL, ",")) {
            const bs_field_t *f = beacon_series_find(tok);
            if (f == NULL) {
                fprintf(stderr, "telemetry_browser: no field '%s'"
                                " (--help lists them)\n", tok);
                return 1;
            }
            toggle_plot((int) (f - BEACON_SERIES_FIELDS));
        }
    } else {
        const char *opening[] = { "batt_v", "pcu_in", "obc_t" };
        for (size_t i = 0; i < sizeof opening / sizeof opening[0]; i++) {
            const bs_field_t *f = beacon_series_find(opening[i]);
            if (f != NULL) toggle_plot((int) (f - BEACON_SERIES_FIELDS));
        }
    }
    // Open on the last pass: the freshest telemetry is what an operator
    // coming off a pass wants to see, and the whole record is one key
    // away.
    view_pass(g_npass - 1);

    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "telemetry_browser");
    SetTargetFPS(60);
    g_font_loaded = load_ui_font();

    float rep_up = 0, rep_down = 0, rep_left = 0, rep_right = 0;

    while (!WindowShouldClose()) {
        const int sw = GetScreenWidth(), sh = GetScreenHeight();
        const int plot_x = LEFT_W + 10;
        const int plot_w = sw - plot_x - 12;
        const int plot_top = 44;
        const int plot_bot = sh - FOOTER_H - 20;   // room for the time axis
        const int plot_h = plot_bot - plot_top;

        // ---- input ----
        const Vector2 m = GetMousePosition();
        const int over_plots = m.x >= plot_x && m.x < plot_x + plot_w
                            && m.y >= plot_top && m.y < plot_bot;
        const int over_list = m.x < LEFT_W;

        if (key_repeat(KEY_DOWN, &rep_down)
            && g_cursor < (int) BEACON_SERIES_FIELD_N - 1) g_cursor++;
        if (key_repeat(KEY_UP, &rep_up) && g_cursor > 0) g_cursor--;
        if (IsKeyPressed(KEY_SPACE) || IsKeyPressed(KEY_ENTER))
            toggle_plot(g_cursor);
        if (IsKeyPressed(KEY_N)) g_nplot = 0;
        if (IsKeyPressed(KEY_A)) {
            // Every field of the group the cursor is in, as far as the
            // panel limit allows.
            const bs_group_t grp = BEACON_SERIES_FIELDS[g_cursor].group;
            g_nplot = 0;
            for (size_t k = 0; k < BEACON_SERIES_FIELD_N && g_nplot < MAX_PLOTS; k++)
                if (BEACON_SERIES_FIELDS[k].group == grp) g_plot[g_nplot++] = (int) k;
        }

        // Panning and zooming in time.
        const double span = g_t1 - g_t0;
        if (key_repeat(KEY_LEFT, &rep_left))  { g_t0 -= span * 0.1; g_t1 -= span * 0.1; }
        if (key_repeat(KEY_RIGHT, &rep_right)) { g_t0 += span * 0.1; g_t1 += span * 0.1; }
        if (IsKeyPressed(KEY_LEFT_BRACKET))  view_pass(current_pass() - 1);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) view_pass(current_pass() + 1);
        if (IsKeyPressed(KEY_G)) view_all();

        if (over_plots) {
            const float wheel = GetMouseWheelMove();
            if (wheel != 0.0f) {
                // Zoom about the pointer, so the moment under it stays
                // under it.
                const double frac = (double) (m.x - plot_x) / (double) plot_w;
                const double at = g_t0 + span * frac;
                const double k = exp(-0.18 * (double) wheel);
                double ns = span * k;
                if (ns < MIN_SPAN_MS) ns = MIN_SPAN_MS;
                g_t0 = at - ns * frac;
                g_t1 = g_t0 + ns;
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                const Vector2 d = GetMouseDelta();
                const double per_px = span / (double) plot_w;
                g_t0 -= (double) d.x * per_px;
                g_t1 -= (double) d.x * per_px;
            }
        }
        if (over_list) g_list_scroll -= GetMouseWheelMove() * 3.0f * ROW_H;
        if (over_list && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            const int row = (int) ((m.y - plot_top + g_list_scroll) / ROW_H);
            // The rows carry group headings between them, so the hit
            // test walks the same layout the drawing does.
            int yy = 0;
            bs_group_t last = (bs_group_t) -1;
            for (size_t k = 0; k < BEACON_SERIES_FIELD_N; k++) {
                if (BEACON_SERIES_FIELDS[k].group != last) {
                    last = BEACON_SERIES_FIELDS[k].group;
                    yy++;
                }
                if (yy == row) { g_cursor = (int) k; toggle_plot((int) k); break; }
                yy++;
            }
        }

        if (IsKeyPressed(KEY_E)) {
            if (g_nplot == 0) {
                snprintf(g_status, sizeof g_status,
                         "nothing to write: pick a field first");
            } else {
                const int rows = export_csv("telemetry.csv");
                if (rows < 0)
                    snprintf(g_status, sizeof g_status,
                             "could not write telemetry.csv");
                else
                    snprintf(g_status, sizeof g_status,
                             "wrote telemetry.csv: %d beacons, %d field%s",
                             rows, g_nplot, g_nplot == 1 ? "" : "s");
            }
            g_status_left = 6.0f;
        }
        if (IsKeyPressed(KEY_F5)) {
            const double save0 = g_t0, save1 = g_t1;
            const int got = load_samples(db_path);
            if (got > 0) {
                g_t0 = save0; g_t1 = save1;
                snprintf(g_status, sizeof g_status,
                         "reloaded: %d beacons (%d basic, %d extended)",
                         g_n, g_nb, g_nx);
            } else {
                snprintf(g_status, sizeof g_status, "reload found no beacons");
            }
            g_status_left = 6.0f;
        }
        if (IsKeyPressed(KEY_Q)) break;
        if (g_status_left > 0.0f) g_status_left -= GetFrameTime();

        // Hold the window inside the record, with a screen's worth of
        // slack either side so the ends are reachable.
        if (g_n > 1) {
            const double slack = (g_t1 - g_t0);
            const double lo = g_s[0].ts_ms - slack;
            const double hi = g_s[g_n - 1].ts_ms + slack;
            if (g_t0 < lo) { const double d = lo - g_t0; g_t0 += d; g_t1 += d; }
            if (g_t1 > hi) { const double d = g_t1 - hi; g_t0 -= d; g_t1 -= d; }
        }

        // ---- draw ----
        BeginDrawing();
        ClearBackground((Color){ 18, 18, 22, 255 });

        // The field list.
        DrawRectangle(0, 0, LEFT_W, sh, (Color){ 28, 28, 34, 255 });
        draw_text("Telemetry fields", 12, 10, 18, RAYWHITE);
        draw_text(TextFormat("%d/%d", g_nplot, MAX_PLOTS),
                  LEFT_W - 52, 14, 12, GRAY);

        BeginScissorMode(0, plot_top - 4, LEFT_W, sh - FOOTER_H - plot_top + 4);
        {
            int y = plot_top - (int) g_list_scroll;
            bs_group_t last = (bs_group_t) -1;
            for (size_t k = 0; k < BEACON_SERIES_FIELD_N; k++) {
                const bs_field_t *f = &BEACON_SERIES_FIELDS[k];
                if (f->group != last) {
                    last = f->group;
                    draw_text(beacon_series_group_name(f->group), 10, y + 3, 12,
                              (Color){ 120, 150, 190, 255 });
                    y += ROW_H;
                }
                const int on = is_plotted((int) k);
                if ((int) k == g_cursor)
                    DrawRectangle(0, y, LEFT_W, ROW_H, (Color){ 44, 70, 110, 255 });
                // The marker doubles as the series colour, so a panel
                // and its row are tied together by eye.
                if (on) {
                    int slot = 0;
                    for (int i = 0; i < g_nplot; i++) if (g_plot[i] == (int) k) slot = i;
                    DrawRectangle(8, y + 5, 8, 8,
                                  SERIES_C[slot % (int) (sizeof SERIES_C
                                                         / sizeof SERIES_C[0])]);
                } else {
                    DrawRectangleLines(8, y + 5, 8, 8, (Color){ 90, 90, 100, 255 });
                }
                draw_text(f->key, 24, y + 3, 12,
                          on ? RAYWHITE : (Color){ 170, 170, 180, 255 });
                if (f->ext_only)
                    draw_text("ext", LEFT_W - 30, y + 4, 10,
                              (Color){ 120, 160, 200, 255 });
                y += ROW_H;
            }
            const float max_scroll = (float) y + g_list_scroll
                                   - (float) (sh - FOOTER_H);
            if (g_list_scroll > max_scroll) g_list_scroll = max_scroll;
            if (g_list_scroll < 0.0f) g_list_scroll = 0.0f;
        }
        EndScissorMode();

        // What the cursor row is, spelled out under the list heading --
        // the list itself only has room for the key.
        {
            const bs_field_t *f = &BEACON_SERIES_FIELDS[g_cursor];
            char d[120];
            snprintf(d, sizeof d, "%s%s%s%s", f->label,
                     f->unit[0] ? "  (" : "", f->unit, f->unit[0] ? ")" : "");
            draw_text(d, 12, 27, 12, (Color){ 150, 150, 160, 255 });
        }

        // The plots.
        const int have_cursor = over_plots;
        const double cursor_ms = g_t0 + (g_t1 - g_t0)
                               * (double) (m.x - plot_x) / (double) plot_w;
        if (g_nplot == 0) {
            draw_text("Pick a field on the left (space) to plot it.",
                      plot_x + 10, plot_top + 12, 14, GRAY);
        } else {
            const int gap = 6;
            const int each = (plot_h - gap * (g_nplot - 1)) / g_nplot;
            for (int i = 0; i < g_nplot; i++) {
                const int py = plot_top + i * (each + gap);
                draw_panel(&BEACON_SERIES_FIELDS[g_plot[i]],
                           SERIES_C[i % (int) (sizeof SERIES_C
                                               / sizeof SERIES_C[0])],
                           plot_x, py, plot_w, each, have_cursor, cursor_ms);
            }
            // The cursor, drawn over every panel at once so values at
            // the same moment line up down the screen.
            if (have_cursor) {
                DrawLine((int) m.x, plot_top, (int) m.x, plot_bot,
                         (Color){ 255, 255, 255, 60 });
            }
        }

        // The time axis under the panels: a handful of labelled ticks.
        {
            const int ticks = plot_w / 130;
            for (int i = 0; i <= ticks; i++) {
                const double t = g_t0 + (g_t1 - g_t0) * (double) i / (double) (ticks ? ticks : 1);
                const int tx = plot_x + plot_w * i / (ticks ? ticks : 1);
                DrawLine(tx, plot_bot, tx, plot_bot + 4, (Color){ 90, 90, 100, 255 });
                char lab[32];
                fmt_utc_short(t, lab, sizeof lab);
                draw_text(lab, tx - text_width(lab, 11) / 2, plot_bot + 5, 11, GRAY);
            }
            // How wide the window is, since two dates a minute apart and
            // two a month apart look much the same on the axis.
            const double s = (g_t1 - g_t0) / 1000.0;
            char wspan[64];
            if (s < 180.0)        snprintf(wspan, sizeof wspan, "%.0f s", s);
            else if (s < 10800.0) snprintf(wspan, sizeof wspan, "%.1f min", s / 60.0);
            else if (s < 259200.0) snprintf(wspan, sizeof wspan, "%.1f h", s / 3600.0);
            else                  snprintf(wspan, sizeof wspan, "%.1f days", s / 86400.0);
            draw_text(wspan, plot_x + plot_w - text_width(wspan, 11), plot_top - 15,
                      11, GRAY);
        }

        // The window, in words, above the panels; and the moment under
        // the pointer while it is over them.
        {
            char a[32], b[32], line[200];
            fmt_utc(g_t0, a, sizeof a);
            fmt_utc(g_t1, b, sizeof b);
            if (have_cursor) {
                char cur[32];
                fmt_utc(cursor_ms, cur, sizeof cur);
                snprintf(line, sizeof line, "%sZ  ..  %sZ        cursor %sZ",
                         a, b, cur);
            } else {
                snprintf(line, sizeof line, "%sZ  ..  %sZ", a, b);
            }
            draw_text(line, plot_x, plot_top - 16, 12,
                      (Color){ 200, 200, 210, 255 });
        }

        // The footer: the keys, and whatever the last action had to say.
        DrawRectangle(0, sh - FOOTER_H, sw, FOOTER_H, (Color){ 24, 24, 30, 255 });
        if (g_status_left > 0.0f && g_status[0] != '\0') {
            draw_text(g_status, 12, sh - FOOTER_H + 7, 12,
                      (Color){ 255, 210, 130, 255 });
        } else {
            draw_text("up/down field   space plot   a group   n none   "
                      "drag/left/right pan   scroll zoom   [ ] pass   "
                      "g all   e csv   F5 reload   q quit",
                      12, sh - FOOTER_H + 7, 12, (Color){ 140, 140, 150, 255 });
        }

        EndDrawing();
    }

    CloseWindow();
    free_samples();
    return 0;
}

#endif
