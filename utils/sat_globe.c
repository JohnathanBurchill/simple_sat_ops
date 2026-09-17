/*

    Simple Satellite Operations  utils/sat_globe.c

    The ephemeris globe panel (see sat_globe.h): the Earth as it was at a
    moment of the mission, lit from where the Sun actually stood, with the
    satellite's ground track drawn across it and a dot where the satellite was.

    The disc is drawn on the CPU, one ray per panel pixel into a unit sphere,
    rather than as a 3D model: the terminator, the map lookup and the test for
    whether a piece of track has gone round the far side all fall out of the
    same few lines, and it needs no shader. It is redrawn only when the view or
    the Sun moves; the track and the dot are drawn over it every frame.

    select_ephemeris rewrites a TLE's units in place and sgp4sdp4's flags are
    module-level rather than per-object, so the elements are converted once,
    when the track changes, and every sample after that propagates that one
    converted set.

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

#include "sat_globe.h"

#include "adcs_mag.h"
#include "prediction.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef WITH_SQLITE3
#include <sqlite3.h>
#endif

// ---- the caller's font ------------------------------------------------------

static Font  g_font;
static int   g_font_loaded = 0;
static float g_font_spacing = 1.0f;

void globe_set_font(Font f, int loaded, float spacing)
{
    g_font = f;
    g_font_loaded = loaded;
    g_font_spacing = spacing;
}

static void draw_text(const char *s, int x, int y, int size, Color c)
{
    if (g_font_loaded)
        DrawTextEx(g_font, s, (Vector2){ (float) x, (float) y }, (float) size, g_font_spacing, c);
    else
        DrawText(s, x, y, size, c);
}

// Width of a string as draw_text would render it, for right-justifying.
static int text_width(const char *s, int size)
{
    if (g_font_loaded)
        return (int) MeasureTextEx(g_font, s, (float) size, g_font_spacing).x;
    return MeasureText(s, size);
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

// ---- the TLE table ---------------------------------------------------------
//
// Every element set the DB holds for FrontierSat, so a moment can be
// matched to the one closest to it in time. Read once at startup and again
// whenever the caller re-reads the DB.

#define GLOBE_CATALOG  69015   // FrontierSat

typedef struct {
    long long catalog;
    int64_t   epoch_ms;
    char      line1[80];
    char      line2[80];
} tle_row_t;

static tle_row_t *g_tles = NULL;
static int64_t   *g_tle_epochs = NULL;   // parallel to g_tles, for the search
static int        g_ntles = 0;

static void free_tle_rows(void)
{
    free(g_tles); g_tles = NULL;
    free(g_tle_epochs); g_tle_epochs = NULL;
    g_ntles = 0;
}

// Load FrontierSat's element sets, each epoch worked out as unix ms. Returns
// how many came back.
int globe_load_tles(const char *db_path)
{
    free_tle_rows();
#ifndef WITH_SQLITE3
    // No DB support in this build, so no element sets and no ground track.
    (void) db_path;
    return 0;
#else
    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        if (db != NULL) sqlite3_close(db);
        return 0;
    }
    const char *sql =
        "SELECT catalog_number, epoch_year, epoch_day, line1, line2 FROM tle "
        "WHERE catalog_number = ?1 AND epoch_year IS NOT NULL "
        "AND epoch_day IS NOT NULL";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        sqlite3_close(db);
        return 0;
    }
    sqlite3_bind_int64(st, 1, GLOBE_CATALOG);
    int cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        int64_t epoch_ms = 0;
        if (adcs_tle_epoch_unix_ms(sqlite3_column_int(st, 1),
                                   sqlite3_column_double(st, 2), &epoch_ms) != 0)
            continue;
        if (g_ntles == cap) {
            int nc = cap ? cap * 2 : 64;
            tle_row_t *tt = realloc(g_tles, (size_t) nc * sizeof *tt);
            int64_t   *et = realloc(g_tle_epochs, (size_t) nc * sizeof *et);
            if (tt != NULL) g_tles = tt;
            if (et != NULL) g_tle_epochs = et;
            if (tt == NULL || et == NULL) break;
            cap = nc;
        }
        tle_row_t *r = &g_tles[g_ntles];
        r->catalog  = sqlite3_column_int64(st, 0);
        r->epoch_ms = epoch_ms;
        snprintf(r->line1, sizeof r->line1, "%s", (const char *) sqlite3_column_text(st, 3));
        snprintf(r->line2, sizeof r->line2, "%s", (const char *) sqlite3_column_text(st, 4));
        g_tle_epochs[g_ntles] = epoch_ms;
        g_ntles++;
    }
    sqlite3_finalize(st);
    sqlite3_close(db);
    return g_ntles;
#endif
}

// ---- sphere geometry -------------------------------------------------------

// Unit vector, Earth-fixed, of a point at (lat, lon) in degrees. X points at
// 0 N 0 E, Y at 0 N 90 E, Z at the north pole.
static void ll_vec(double lat_deg, double lon_deg, double v[3])
{
    double la = lat_deg * (M_PI / 180.0), lo = lon_deg * (M_PI / 180.0);
    double c = cos(la);
    v[0] = c * cos(lo);
    v[1] = c * sin(lo);
    v[2] = sin(la);
}

// The screen basis of a globe centred on (lat0, lon0): east across the panel,
// north up it, and out of it towards the viewer.
static void globe_basis(double lat0, double lon0,
                        double e[3], double n[3], double o[3])
{
    double la = lat0 * (M_PI / 180.0), lo = lon0 * (M_PI / 180.0);
    double sla = sin(la), cla = cos(la), slo = sin(lo), clo = cos(lo);
    e[0] = -slo;        e[1] =  clo;        e[2] = 0.0;
    n[0] = -sla * clo;  n[1] = -sla * slo;  n[2] = cla;
    o[0] =  cla * clo;  o[1] =  cla * slo;  o[2] = sla;
}

// The disc's radius in panel pixels.
static double globe_radius(int w, int h, double zoom)
{
    return GLOBE_FILL * (double) (w < h ? w : h) * zoom;
}

// Where a point at (lat, lon, altitude) lands on a panel of w x h whose disc is
// centred (ox, oy) from the middle of it, and whether the globe itself is in
// front of the point -- which it is when the point is behind the plane of the
// disc and inside its outline.
static void globe_project(const double e[3], const double n[3], const double o[3],
                          double R, int w, int h, double ox, double oy,
                          double lat, double lon, double alt_km,
                          Vector2 *out, int *hidden)
{
    double p[3];
    ll_vec(lat, lon, p);
    double rr = 1.0 + alt_km / GLOBE_EARTH_KM;
    double x = rr * (p[0] * e[0] + p[1] * e[1] + p[2] * e[2]);
    double y = rr * (p[0] * n[0] + p[1] * n[1] + p[2] * n[2]);
    double z = rr * (p[0] * o[0] + p[1] * o[1] + p[2] * o[2]);
    out->x = (float) ((double) w * 0.5 + ox + R * x);
    out->y = (float) ((double) h * 0.5 + oy - R * y);
    *hidden = (z < 0.0 && x * x + y * y < 1.0);
}

// Where the satellite's dot sits on a w x h panel as the globe stands now.
// The pivot works off the difference between two of these across a change, so
// a globe with no orbit to draw reports a fixed point and moves nothing.
static void globe_anchor_at(const globe_t *g, int w, int h, Vector2 *out)
{
    if (!g->have_anchor) { *out = (Vector2){ 0.0f, 0.0f }; return; }
    double e[3], n[3], o[3];
    int hidden = 0;
    globe_basis(g->lat0, g->lon0, e, n, o);
    globe_project(e, n, o, globe_radius(w, h, g->zoom), w, h, g->ox, g->oy,
                  g->anchor_lat, g->anchor_lon, g->anchor_alt, out, &hidden);
}

// Hold the satellite's dot where it was across a change of view: given where it
// sat before the change, slide the disc by however far the change moved it.
static void globe_hold_anchor(globe_t *g, int w, int h, Vector2 before)
{
    Vector2 after;
    globe_anchor_at(g, w, h, &after);
    g->ox += before.x - after.x;
    g->oy += before.y - after.y;
}

// ---- the world map ---------------------------------------------------------

// One 2x2 box average of an RGB image, for the next level down.
static void halve_rgb(const unsigned char *src, int sw, int sh, unsigned char *dst)
{
    int dw = sw / 2, dh = sh / 2;
    for (int y = 0; y < dh; y++) {
        for (int x = 0; x < dw; x++) {
            const unsigned char *a = src + ((size_t) (2 * y) * sw + 2 * x) * 3;
            const unsigned char *b = a + (size_t) sw * 3;
            unsigned char *d = dst + ((size_t) y * dw + x) * 3;
            for (int c = 0; c < 3; c++)
                d[c] = (unsigned char) ((a[c] + a[c + 3] + b[c] + b[c + 3] + 2) / 4);
        }
    }
}

// Bilinear lookup, wrapping in longitude and clamping in latitude, with u and
// v running 0..1 across the map. Falls back to a plain ocean blue if the map
// never loaded, so the panel still shows the lighting and the track.
static void map_sample(const globe_t *g, int lv, double u, double v, double out[3])
{
    if (g->nlv == 0) {
        out[0] = 38.0; out[1] = 62.0; out[2] = 96.0;
        return;
    }
    const maplevel_t *L = &g->lv[lv];
    double fx = u * L->w - 0.5, fy = v * L->h - 0.5;
    int x0 = (int) floor(fx), y0 = (int) floor(fy);
    double tx = fx - x0, ty = fy - y0;
    int x1 = ((x0 + 1) % L->w + L->w) % L->w;
    x0 = (x0 % L->w + L->w) % L->w;
    int y1 = y0 + 1;
    if (y0 < 0) y0 = 0; else if (y0 > L->h - 1) y0 = L->h - 1;
    if (y1 < 0) y1 = 0; else if (y1 > L->h - 1) y1 = L->h - 1;
    const unsigned char *p00 = L->rgb + ((size_t) y0 * L->w + x0) * 3;
    const unsigned char *p10 = L->rgb + ((size_t) y0 * L->w + x1) * 3;
    const unsigned char *p01 = L->rgb + ((size_t) y1 * L->w + x0) * 3;
    const unsigned char *p11 = L->rgb + ((size_t) y1 * L->w + x1) * 3;
    for (int c = 0; c < 3; c++) {
        double top = p00[c] + (p10[c] - p00[c]) * tx;
        double bot = p01[c] + (p11[c] - p01[c]) * tx;
        out[c] = top + (bot - top) * ty;
    }
}

// Load the equirectangular world map and halve it a few times over. Looked for
// the same places as the bundled font. Returns 0 if none of them had it, which
// leaves the globe drawn as a plain sphere rather than not drawn at all.
int globe_load_map(globe_t *g, const char *tool)
{
    const char *home = getenv("HOME");
    const char *xdg = getenv("XDG_DATA_HOME");
    char cands[5][1024];
    int n = 0;
    if (xdg != NULL && xdg[0] != '\0')
        snprintf(cands[n++], sizeof cands[0], "%s/simple_sat_ops/%s", xdg, GLOBE_MAP_FILE);
    if (home != NULL && home[0] != '\0') {
        snprintf(cands[n++], sizeof cands[0], "%s/.local/share/simple_sat_ops/%s", home, GLOBE_MAP_FILE);
        snprintf(cands[n++], sizeof cands[0], "%s/src/simple_sat_ops/assets/%s", home, GLOBE_MAP_FILE);
    }
    snprintf(cands[n++], sizeof cands[0], "assets/%s", GLOBE_MAP_FILE);
    snprintf(cands[n++], sizeof cands[0], "../assets/%s", GLOBE_MAP_FILE);

    Image im = {0};
    for (int i = 0; i < n && im.data == NULL; i++) {
        if (!FileExists(cands[i])) continue;
        im = LoadImage(cands[i]);
    }
    if (im.data == NULL) {
        fprintf(stderr, "%s: %s not found; the globe will be blank\n",
                tool, GLOBE_MAP_FILE);
        return 0;
    }
    ImageFormat(&im, PIXELFORMAT_UNCOMPRESSED_R8G8B8);

    size_t bytes = (size_t) im.width * im.height * 3;
    g->lv[0].rgb = malloc(bytes);
    if (g->lv[0].rgb == NULL) { UnloadImage(im); return 0; }
    memcpy(g->lv[0].rgb, im.data, bytes);
    g->lv[0].w = im.width;
    g->lv[0].h = im.height;
    g->nlv = 1;
    UnloadImage(im);

    while (g->nlv < GLOBE_MAP_LEVELS) {
        const maplevel_t *src = &g->lv[g->nlv - 1];
        int dw = src->w / 2, dh = src->h / 2;
        if (dw < 16 || dh < 8) break;
        unsigned char *dst = malloc((size_t) dw * dh * 3);
        if (dst == NULL) break;
        halve_rgb(src->rgb, src->w, src->h, dst);
        g->lv[g->nlv].rgb = dst;
        g->lv[g->nlv].w = dw;
        g->lv[g->nlv].h = dh;
        g->nlv++;
    }
    return 1;
}

// ---- where the Sun and the satellite were ----------------------------------

// The sub-solar point, Earth-fixed. Calculate_Solar_Position works in the
// inertial frame, so the longitude comes from taking Greenwich sidereal time
// off the Sun's right ascension -- the same step Calculate_LatLonAlt makes for
// the satellite.
static void sun_subpoint(double jul_utc, double *lat_deg, double *lon_deg)
{
    vector_t sol = {0};
    Calculate_Solar_Position(jul_utc, &sol);
    double r = sqrt(sol.x * sol.x + sol.y * sol.y + sol.z * sol.z);
    double lon = atan2(sol.y, sol.x) - ThetaG_JD(jul_utc);
    lon = fmod(lon + M_PI, 2.0 * M_PI);
    if (lon < 0.0) lon += 2.0 * M_PI;
    *lat_deg = asin(sol.z / r) * (180.0 / M_PI);
    *lon_deg = (lon - M_PI) * (180.0 / M_PI);
}

// The sub-satellite point at a moment, from the element set already converted
// into the globe. Returns 0 if no element set is loaded.
int globe_subpoint(const globe_t *g, double unix_ms,
                          double *lat, double *lon, double *alt_km)
{
    if (!g->have_tle) return 0;
    prediction_t pred = {0};
    pred.observer_ephem.position_geodetic.lat = RAO_LATITUDE  * (M_PI / 180.0);
    pred.observer_ephem.position_geodetic.lon = RAO_LONGITUDE * (M_PI / 180.0);
    pred.observer_ephem.position_geodetic.alt = RAO_ALTITUDE / 1000.0;
    pred.satellite_ephem.tle = g->tle;
    update_satellite_position(&pred, julian_date_from_unix_seconds(unix_ms / 1000.0));
    double lo = pred.satellite_ephem.longitude;
    if (lo > 180.0) lo -= 360.0;
    *lat    = pred.satellite_ephem.latitude;
    *lon    = lo;
    *alt_km = pred.satellite_ephem.altitude_km;
    return 1;
}

// Face the middle of the track the globe is showing, with the disc back in the
// middle of the panel. The zoom is left alone: a new track re-frames the view,
// but does not undo how far in the reader had gone.
static void globe_frame_track(globe_t *g)
{
    g->ox = g->oy = 0.0;
    if (g->ntrk > 0) {
        const subpoint_t *m = &g->trk[g->ntrk / 2];
        g->lat0 = m->lat;
        g->lon0 = m->lon;
    }
    g->valid = 0;
}

// The whole view back to how it first opens -- framed on the track and zoomed
// out to the whole Earth -- which is what g does, and the way back from having
// turned or zoomed to somewhere unhelpful.
void globe_reset_view(globe_t *g)
{
    g->zoom = GLOBE_ZOOM_MIN;
    globe_frame_track(g);
}

// Point the globe at a new stretch of track: pick the element set whose epoch
// is closest to it, walk the ground track from one end to the other, work out
// where the Sun was in the middle of it, and turn the globe to face the middle
// of the track.
void globe_set_track(globe_t *g, const char *key, double t0_ms, double t1_ms)
{
    snprintf(g->track_key, sizeof g->track_key, "%s", key);
    g->ntrk = 0;
    g->have_tle = 0;
    g->have_anchor = 0;
    g->valid = 0;

    // A new track is framed in the middle of itself -- unless this is the one
    // the last session had open, whose view came back with it.
    int frame_it = !g->keep_view;
    g->keep_view = 0;

    double t0 = t0_ms;
    double t1 = t1_ms > t0 ? t1_ms : t0 + 60000.0;
    double mid = 0.5 * (t0 + t1);
    sun_subpoint(julian_date_from_unix_seconds(mid / 1000.0), &g->sun_lat, &g->sun_lon);

    int idx = adcs_closest_index(g_tle_epochs, g_ntles, (int64_t) mid);
    if (idx < 0) {
        snprintf(g->tle_note, sizeof g->tle_note, "no TLE in the database");
        return;
    }
    const tle_row_t *row = &g_tles[idx];

    // Convert_Satellite_Data reads the two lines out of one 2 x 69 char buffer.
    char lines[2 * 69 + 1] = {0};
    size_t l1 = strlen(row->line1), l2 = strlen(row->line2);
    if (l1 > 69) l1 = 69;
    if (l2 > 69) l2 = 69;
    memcpy(lines, row->line1, l1);
    memcpy(lines + 69, row->line2, l2);
    if (!Good_Elements(lines)) {
        snprintf(g->tle_note, sizeof g->tle_note, "the nearest TLE will not parse");
        return;
    }
    Convert_Satellite_Data(lines, &g->tle);
    // select_ephemeris rewrites the elements in place and sets module-level
    // flags, so it runs once per element set, on a cleared slate.
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&g->tle);
    g->have_tle = 1;

    // How far the track sat from the element set's epoch, which is what
    // says how much to trust the track. In hours while it is under a day,
    // where a tenth of a day would round most of the answer away.
    double age_h = (mid - (double) row->epoch_ms) / 3600000.0;
    char ep[24];
    fmt_utc_s((double) row->epoch_ms, ep, sizeof ep);
    ep[16] = '\0';   // to the minute; the seconds of a TLE epoch say nothing here
    snprintf(g->tle_note, sizeof g->tle_note, "TLE %lld  %s  %+.1f %c",
             row->catalog, ep,
             fabs(age_h) < 24.0 ? age_h : age_h / 24.0,
             fabs(age_h) < 24.0 ? 'h' : 'd');

    for (int i = 0; i < GLOBE_TRACK_PTS; i++) {
        double t = t0 + (t1 - t0) * (double) i / (double) (GLOBE_TRACK_PTS - 1);
        double lat = 0, lon = 0, alt = 0;
        if (!globe_subpoint(g, t, &lat, &lon, &alt)) break;
        g->trk[g->ntrk].lat    = (float) lat;
        g->trk[g->ntrk].lon    = (float) lon;
        g->trk[g->ntrk].alt_km = (float) alt;
        g->ntrk++;
    }
    if (frame_it) globe_frame_track(g);
}

// ---- drawing ---------------------------------------------------------------

// Ray-cast the lit sphere into g->pix and hand it to the texture. One ray per
// texture pixel: the ones that miss are left transparent, the ones that graze
// the outline get a fraction of an alpha so the limb does not come out jagged.
//
// ss is how many texture pixels go to a panel pixel. A ray costs an atan2 and
// an arcsine, so a screen-resolution disc on a retina panel is four times the
// work of a panel-resolution one and too slow to keep up with a drag; the
// caller renders coarse while the view is moving and sharp once it settles.
static void globe_render(globe_t *g, int w, int h, int ss)
{
    int pw = w * ss, ph = h * ss;
    if (pw != g->tex_w || ph != g->tex_h) {
        Color *np = realloc(g->pix, (size_t) pw * ph * sizeof *np);
        if (np == NULL) return;
        g->pix = np;
        if (g->tex.id != 0) UnloadTexture(g->tex);
        Image im = { .data = g->pix, .width = pw, .height = ph,
                     .mipmaps = 1, .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
        memset(g->pix, 0, (size_t) pw * ph * sizeof *g->pix);
        g->tex = LoadTextureFromImage(im);
        SetTextureFilter(g->tex, TEXTURE_FILTER_BILINEAR);
        g->tex_w = pw; g->tex_h = ph;
    }

    double R = globe_radius(w, h, g->zoom) * ss;
    double cx = (double) pw * 0.5 + g->ox * ss;
    double cy = (double) ph * 0.5 + g->oy * ss;

    // The level whose texels are about a screen pixel across: 360 degrees of
    // map spans roughly 2*pi*R pixels over the middle of the disc.
    int lv = 0;
    while (lv + 1 < g->nlv && (double) g->lv[lv + 1].w >= 2.0 * M_PI * R) lv++;

    double e[3], n[3], o[3], sun[3];
    globe_basis(g->lat0, g->lon0, e, n, o);
    ll_vec(g->sun_lat, g->sun_lon, sun);

    for (int py = 0; py < ph; py++) {
        double sy = (cy - ((double) py + 0.5)) / R;
        Color *row = &g->pix[(size_t) py * pw];
        for (int px = 0; px < pw; px++) {
            double sx = (((double) px + 0.5) - cx) / R;
            double d2 = sx * sx + sy * sy;
            double cov = R - sqrt(d2) * R + 0.5;
            if (cov <= 0.0) { row[px] = (Color){ 0, 0, 0, 0 }; continue; }
            if (cov > 1.0) cov = 1.0;

            double sz = sqrt(d2 < 1.0 ? 1.0 - d2 : 0.0);
            double p[3];
            for (int c = 0; c < 3; c++) p[c] = sx * e[c] + sy * n[c] + sz * o[c];
            double pz = p[2] > 1.0 ? 1.0 : p[2] < -1.0 ? -1.0 : p[2];
            double u = atan2(p[1], p[0]) / (2.0 * M_PI) + 0.5;
            double vv = 0.5 - asin(pz) / M_PI;

            double rgb[3];
            map_sample(g, lv, u, vv, rgb);

            // Lambert, but with the terminator softened over a few degrees so
            // it does not come out as a hard line the way a bare dot product
            // would.
            double f = (p[0] * sun[0] + p[1] * sun[1] + p[2] * sun[2]
                        + GLOBE_TERMINATOR) / (2.0 * GLOBE_TERMINATOR);
            f = f < 0.0 ? 0.0 : f > 1.0 ? 1.0 : f;
            f = f * f * (3.0 - 2.0 * f);
            double lit = GLOBE_NIGHT + (1.0 - GLOBE_NIGHT) * f;

            row[px] = (Color){ (unsigned char) (rgb[0] * lit + 0.5),
                               (unsigned char) (rgb[1] * lit + 0.5),
                               (unsigned char) (rgb[2] * lit + 0.5),
                               (unsigned char) (cov * 255.0 + 0.5) };
        }
    }
    UpdateTexture(g->tex, g->pix);

    g->r_lat0 = g->lat0; g->r_lon0 = g->lon0; g->r_zoom = g->zoom;
    g->r_sunlat = g->sun_lat; g->r_sunlon = g->sun_lon;
    g->r_ox = g->ox; g->r_oy = g->oy;
    g->r_ss = ss;
    g->valid = 1;
}

// The globe panel: the lit Earth, the ground track, and a dot where the
// satellite was at now_ms -- the moment whatever is on screen belongs to.
// title is the heading along the top of the panel.
void globe_draw(globe_t *g, int x, int y, int w, int h, double now_ms,
                const char *title)
{
    DrawRectangle(x, y, w, h, (Color){ 22, 22, 27, 255 });
    DrawLine(x, y, x + w, y, (Color){ 56, 56, 66, 255 });
    draw_text(title, x + 12, y + 4, 13, LIGHTGRAY);
    if (g->tle_note[0] != '\0')
        draw_text(g->tle_note, x + w - 10 - text_width(g->tle_note, 12), y + 5, 12, GRAY);

    int dy = y + GLOBE_TITLE_H, dh = h - GLOBE_TITLE_H;
    if (dh < 32) return;

    // Coarse while the view is still moving, then once more at the screen's own
    // resolution when it has settled -- which on a retina display is where the
    // coastlines come back.
    if (g->settle > 0.0f) g->settle -= GetFrameTime();
    int ss = 1;
    if (g->settle <= 0.0f) {
        ss = (int) GetWindowScaleDPI().x;
        if (ss < 1) ss = 1;
        if (ss > 2) ss = 2;
    }
    if (!g->valid || g->tex_w != w * ss || g->tex_h != dh * ss || g->r_ss != ss
        || g->r_lat0 != g->lat0 || g->r_lon0 != g->lon0 || g->r_zoom != g->zoom
        || g->r_sunlat != g->sun_lat || g->r_sunlon != g->sun_lon
        || g->r_ox != g->ox || g->r_oy != g->oy)
        globe_render(g, w, dh, ss);
    if (g->tex.id == 0) return;
    DrawTexturePro(g->tex,
                   (Rectangle){ 0, 0, (float) g->tex_w, (float) g->tex_h },
                   (Rectangle){ (float) x, (float) dy, (float) w, (float) dh },
                   (Vector2){ 0, 0 }, 0.0f, WHITE);

    double R = globe_radius(w, dh, g->zoom);
    BeginScissorMode(x, dy, w, dh);

    // A faint outline, so the unlit limb still reads against the panel.
    DrawCircleLines(x + w / 2 + (int) g->ox, dy + dh / 2 + (int) g->oy,
                    (float) R, (Color){ 90, 110, 140, 120 });

    double e[3], n[3], o[3];
    globe_basis(g->lat0, g->lon0, e, n, o);

    // The track. The far-side stretches are drawn faintly rather than dropped,
    // so a pass that goes over the horizon still reads as one arc.
    const Color near_c = { 255, 150, 60, 255 };
    const Color far_c  = { 255, 150, 60, 70 };
    for (int i = 1; i < g->ntrk; i++) {
        Vector2 a, b;
        int ha = 0, hb = 0;
        globe_project(e, n, o, R, w, dh, g->ox, g->oy,
                      g->trk[i - 1].lat, g->trk[i - 1].lon,
                      g->trk[i - 1].alt_km, &a, &ha);
        globe_project(e, n, o, R, w, dh, g->ox, g->oy,
                      g->trk[i].lat, g->trk[i].lon,
                      g->trk[i].alt_km, &b, &hb);
        a.x += x; a.y += dy;
        b.x += x; b.y += dy;
        DrawLineEx(a, b, 2.0f, (ha && hb) ? far_c : near_c);
    }

    // Where it was at now_ms. This is also the point the next
    // frame's turning and zooming pivot on.
    double lat = 0, lon = 0, alt = 0;
    int have_now = globe_subpoint(g, now_ms, &lat, &lon, &alt);
    g->have_anchor = have_now;
    if (have_now) {
        g->anchor_lat = lat; g->anchor_lon = lon; g->anchor_alt = alt;
        Vector2 p;
        int hidden = 0;
        globe_project(e, n, o, R, w, dh, g->ox, g->oy, lat, lon, alt, &p, &hidden);
        p.x += x; p.y += dy;
        if (hidden) {
            DrawCircleLines((int) p.x, (int) p.y, 5.0f, (Color){ 255, 255, 255, 70 });
        } else {
            DrawCircleV(p, 6.0f, (Color){ 255, 150, 60, 160 });
            DrawCircleV(p, 3.0f, RAYWHITE);
        }
    }
    EndScissorMode();

    // Where that is on the ground, along the bottom of the disc.
    if (have_now) {
        const char *ll = TextFormat("%.1f %c   %.1f %c   %.0f km",
                                    fabs(lat), lat >= 0 ? 'N' : 'S',
                                    fabs(lon), lon >= 0 ? 'E' : 'W', alt);
        int tw = text_width(ll, 13);
        DrawRectangle(x + 6, y + h - 24, tw + 14, 20, (Color){ 18, 18, 22, 200 });
        draw_text(ll, x + 13, y + h - 21, 13, RAYWHITE);
    }
}

// Drag turns the globe, the wheel (or two fingers) zooms it. A drag that began
// on the globe keeps turning it wherever the pointer wanders, and one that
// began anywhere else never does.
//
// An ordinary drag turns the globe about its own middle -- the plain thing to
// do when you just want to look around. Pressing a Mac trackpad with two
// fingers down is the secondary click, so a two-finger press and slide arrives
// as a right-button drag, and that one turns the globe about the satellite
// instead: the dot is noted before the turn and the disc slid afterwards by
// however far the dot moved, which puts it back where it was and swings the
// Earth around it. Zooming holds the dot the same way, so closing in goes to
// the satellite rather than to whatever the middle of the disc happened to be.
void globe_input(globe_t *g, int x, int y, int w, int h)
{
    int dh = h - GLOBE_TITLE_H;
    Vector2 m = GetMousePosition();
    int over = m.x >= x && m.x < x + w && m.y >= y + GLOBE_TITLE_H && m.y < y + h;
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)
        && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) g->drag = 0;
    if (over && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        g->drag = 1; g->drag_pivot = 0; g->drag_at = m;
    }
    if (over && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        g->drag = 1; g->drag_pivot = 1; g->drag_at = m;
    }

    int turning = g->drag && (m.x != g->drag_at.x || m.y != g->drag_at.y);
    float wheel = over ? GetMouseWheelMove() : 0.0f;
    if (!turning && wheel == 0.0f) return;

    if (turning) {
        Vector2 before = {0};
        if (g->drag_pivot) globe_anchor_at(g, w, dh, &before);
        double R = globe_radius(w, dh, g->zoom);
        g->lon0 -= (double) (m.x - g->drag_at.x) * (180.0 / M_PI) / R;
        g->lat0 += (double) (m.y - g->drag_at.y) * (180.0 / M_PI) / R;
        if (g->lat0 >  89.9) g->lat0 =  89.9;
        if (g->lat0 < -89.9) g->lat0 = -89.9;
        g->lon0 = fmod(g->lon0 + 180.0, 360.0);
        if (g->lon0 < 0.0) g->lon0 += 360.0;
        g->lon0 -= 180.0;
        g->drag_at = m;
        if (g->drag_pivot) globe_hold_anchor(g, w, dh, before);
    }
    if (wheel != 0.0f) {
        Vector2 before;
        globe_anchor_at(g, w, dh, &before);
        g->zoom *= exp(0.14 * (double) wheel);
        if (g->zoom < GLOBE_ZOOM_MIN) g->zoom = GLOBE_ZOOM_MIN;
        if (g->zoom > GLOBE_ZOOM_MAX) g->zoom = GLOBE_ZOOM_MAX;
        globe_hold_anchor(g, w, dh, before);
    }
    g->valid = 0;
    g->settle = GLOBE_SETTLE;
}

void globe_free(globe_t *g)
{
    for (int i = 0; i < g->nlv; i++) free(g->lv[i].rgb);
    g->nlv = 0;
    free(g->pix);
    g->pix = NULL;
    if (g->tex.id != 0) UnloadTexture(g->tex);
    free_tle_rows();
}

