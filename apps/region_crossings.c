/*

    Simple Satellite Operations  region_crossings.c

    List the UTC intervals when FrontierSat is predicted to cross three
    space-physics regions, each defined as a band of magnetic latitude
    (either hemisphere) within a window of magnetic local time:

      cusp     70-80 deg, 09-15 MLT   the dayside cusp and cleft
      aurora   62-72 deg, 18-06 MLT   the nightside auroral oval
      STEVE    55-62 deg, 18-24 MLT   the pre-midnight subauroral zone

    Magnetic latitude is quasi-dipole (QD) latitude at the satellite's own
    altitude, from IGRF-14 field-line tracing, and MLT follows apexpy (see
    src/orbit/magcoords.h). The orbit is SGP4 from a TLE: by default the
    FrontierSat TLE in the packet database whose epoch is nearest the
    middle of the window, so past and future days both use the most
    relevant elements. Each row gives the start and stop times, the mean
    altitude, local solar time and MLT, and the geographic and magnetic
    latitude at the start and stop.

    The band edges are typical, quiet-to-moderate values: the oval and the
    subauroral zone move equatorward with activity. Each is adjustable.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include "argparse.h"
#include "magcoords.h"
#include "prediction.h"
#include "sso_paths.h"
#include "sso_version.h"

#include <sgp4sdp4.h>

#ifdef WITH_SQLITE3
#include <sqlite3.h>
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#define FRONTIERSAT_CATALOG 69015
#define TLE_LINE_CHARS      69
#define DEG                 (M_PI / 180.0)
#define SEC_PER_DAY         86400.0
// Boundary times are refined by bisection to this, in seconds
#define REFINE_S            0.5
// Warn when any part of the window is further than this from the TLE epoch
#define TLE_MAX_AGE_DAYS    7.0

typedef struct {
    const char *name;
    // Band of |QD latitude|: lo <= |lat| < hi, deg
    double mlat_lo, mlat_hi;
    // MLT window, hours; wraps through midnight when lo > hi
    double mlt_lo, mlt_hi;
} region_t;

enum { R_CUSP, R_AURORA, R_STEVE, N_REGIONS };

typedef struct {
    double jd;
    double lat, lon, alt;
    double qdlat, mlt, lt;
} sample_t;

typedef struct {
    int region;
    int hemi;
    double jd0, jd1;
    sample_t s0, s1;
    double alt_mean, lt_mean, mlt_mean;
} crossing_t;

typedef struct {
    region_t regions[N_REGIONS];
    char tle_path[512];
    char sat[64];
    char day[16];
    char start[32];
    char stop[32];
    double hours;
    double step_s;
} args_t;

typedef struct {
    prediction_t pred;
    igrf_model_t igrf;
} ctx_t;

// ---- time -------------------------------------------------------------------

static double now_jd(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return julian_date_from_unix_seconds((double) tv.tv_sec + tv.tv_usec * 1e-6);
}

// "YYYY-MM-DD", "YYYY-MM-DDTHH:MM" or "YYYY-MM-DDTHH:MM:SS" (UTC; a space
// may stand in for the T). Returns 0 and the Julian date, or -1.
static int parse_utc(const char *s, double *jd)
{
    int y, mo, d, h = 0, mi = 0, sec = 0;
    char sep = 0, tail = 0;
    int n = sscanf(s, "%d-%d-%d%c%d:%d:%d%c", &y, &mo, &d, &sep, &h, &mi, &sec, &tail);
    if (n == 3) {
        h = mi = sec = 0;
    } else if (!((n == 6 || n == 7) && (sep == 'T' || sep == ' '))) {
        return -1;
    }
    if (mo < 1 || mo > 12 || d < 1 || d > 31 || h < 0 || h > 23
        || mi < 0 || mi > 59 || sec < 0 || sec > 60)
        return -1;
    struct tm tm = {0};
    tm.tm_year = y - 1900;
    tm.tm_mon = mo - 1;
    tm.tm_mday = d;
    tm.tm_hour = h;
    tm.tm_min = mi;
    tm.tm_sec = sec;
    time_t t = timegm(&tm);
    *jd = julian_date_from_unix_seconds((double) t);
    return 0;
}

// Format a Julian date as "YYYY-MM-DD HH:MM:SS" (date_too) or "HH:MM:SS",
// rounded to the nearest second.
static void fmt_utc(double jd, int date_too, char *buf, size_t n)
{
    time_t t = (time_t) floor((jd - 2440587.5) * SEC_PER_DAY + 0.5);
    struct tm tm;
    gmtime_r(&t, &tm);
    strftime(buf, n, date_too ? "%Y-%m-%d %H:%M:%S" : "%H:%M:%S", &tm);
}

// ---- TLE --------------------------------------------------------------------

// Elements from two card lines into ctx->pred, ready to propagate. Returns
// 0, or -1 if the lines are not a valid set.
static int set_elements(ctx_t *c, const char *name, const char *l1, const char *l2)
{
    char set[2 * TLE_LINE_CHARS + 1] = {0};
    size_t a = strlen(l1), b = strlen(l2);
    if (a > TLE_LINE_CHARS) a = TLE_LINE_CHARS;
    if (b > TLE_LINE_CHARS) b = TLE_LINE_CHARS;
    memcpy(set, l1, a);
    memcpy(set + TLE_LINE_CHARS, l2, b);
    if (!Good_Elements(set)) return -1;
    memset(&c->pred.satellite_ephem.tle, 0, sizeof c->pred.satellite_ephem.tle);
    snprintf(c->pred.satellite_ephem.tle.sat_name,
             sizeof c->pred.satellite_ephem.tle.sat_name, "%s", name);
    Convert_Satellite_Data(set, &c->pred.satellite_ephem.tle);
    return 0;
}

#ifdef WITH_SQLITE3
// The FrontierSat TLE in the packet database whose epoch is nearest
// jd_target. Fills the elements and a description of where they came from.
static int tle_from_db(ctx_t *c, double jd_target, char *src, size_t src_n)
{
    const char *path = sso_packet_db_path();
    sqlite3 *db = NULL;
    if (sqlite3_open_v2(path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "Cannot open the packet database %s: %s\n", path,
                db ? sqlite3_errmsg(db) : "out of memory");
        sqlite3_close(db);
        return -1;
    }
    sqlite3_stmt *st = NULL;
    const char *sql = "SELECT id, line1, line2 FROM tle WHERE catalog_number = ?1";
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "Cannot read TLEs from %s: %s\n", path, sqlite3_errmsg(db));
        sqlite3_close(db);
        return -1;
    }
    sqlite3_bind_int(st, 1, FRONTIERSAT_CATALOG);

    long long best_id = -1;
    double best_dt = INFINITY;
    char l1[80] = {0}, l2[80] = {0};
    while (sqlite3_step(st) == SQLITE_ROW) {
        const char *a = (const char *) sqlite3_column_text(st, 1);
        const char *b = (const char *) sqlite3_column_text(st, 2);
        if (!a || !b || set_elements(c, "FrontierSat", a, b) != 0) continue;
        double dt = fabs(Julian_Date_of_Epoch(c->pred.satellite_ephem.tle.epoch) - jd_target);
        if (dt < best_dt) {
            best_dt = dt;
            best_id = sqlite3_column_int64(st, 0);
            snprintf(l1, sizeof l1, "%s", a);
            snprintf(l2, sizeof l2, "%s", b);
        }
    }
    sqlite3_finalize(st);
    sqlite3_close(db);

    if (best_id < 0) {
        fprintf(stderr, "No FrontierSat (catalog %d) TLEs in %s\n",
                FRONTIERSAT_CATALOG, path);
        return -1;
    }
    set_elements(c, "FrontierSat", l1, l2);
    snprintf(src, src_n, "packet database tle id %lld", best_id);
    return 0;
}
#endif

// ---- geometry ---------------------------------------------------------------

// Rotate an inertial (TEME) vector to Earth-fixed by Greenwich sidereal time.
static void eci_to_ecef(double jd, double x, double y, double z, double out[3])
{
    double g = ThetaG_JD(jd);
    double cg = cos(g), sg = sin(g);
    out[0] = cg * x + sg * y;
    out[1] = -sg * x + cg * y;
    out[2] = z;
}

static void compute_sample(ctx_t *c, double jd, sample_t *s)
{
    update_satellite_position(&c->pred, jd);
    const vector_t *p = &c->pred.satellite_ephem.position;
    double x[3];
    eci_to_ecef(jd, p->x, p->y, p->z, x);

    vector_t sun_eci = {0};
    Calculate_Solar_Position(jd, &sun_eci);
    double sun[3];
    eci_to_ecef(jd, sun_eci.x, sun_eci.y, sun_eci.z, sun);

    s->jd = jd;
    magc_ecef_to_geodetic(x, &s->lat, &s->lon, &s->alt);
    double alon;
    magc_qd(&c->igrf, x, s->alt, &s->qdlat, &alon, NULL);
    s->mlt = magc_mlt(&c->igrf, alon, sun);
    // Local solar time from the subsolar longitude
    double sslon = atan2(sun[1], sun[0]) / DEG;
    s->lt = fmod(12.0 + (s->lon - sslon) / 15.0 + 48.0, 24.0);
}

// +1 or -1 (hemisphere) when the sample is inside the region, else 0.
static int in_region(const region_t *r, const sample_t *s)
{
    double a = fabs(s->qdlat);
    if (a < r->mlat_lo || a >= r->mlat_hi) return 0;
    int in_mlt = r->mlt_lo <= r->mlt_hi
               ? (s->mlt >= r->mlt_lo && s->mlt < r->mlt_hi)
               : (s->mlt >= r->mlt_lo || s->mlt < r->mlt_hi);
    if (!in_mlt) return 0;
    return s->qdlat >= 0.0 ? 1 : -1;
}

// The membership changes between jd_a and jd_b (in_a says whether jd_a is
// inside region r in hemisphere hemi). Bisect to REFINE_S and return the
// boundary sample on the inside.
static void refine(ctx_t *c, const region_t *r, int hemi, double jd_a, int in_a,
                   double jd_b, sample_t *inside)
{
    double lo = jd_a, hi = jd_b;
    while ((hi - lo) * SEC_PER_DAY > REFINE_S) {
        double mid = 0.5 * (lo + hi);
        sample_t s;
        compute_sample(c, mid, &s);
        int in_mid = in_region(r, &s) == hemi;
        if (in_mid == in_a) lo = mid;
        else hi = mid;
    }
    compute_sample(c, in_a ? lo : hi, inside);
}

// Circular mean of hours in [0,24)
static double mean_hours(const double *h, int n)
{
    double sx = 0.0, sy = 0.0;
    for (int i = 0; i < n; ++i) {
        sx += cos(h[i] * 15.0 * DEG);
        sy += sin(h[i] * 15.0 * DEG);
    }
    double m = atan2(sy, sx) / (15.0 * DEG);
    return m < 0.0 ? m + 24.0 : m;
}

// Mean altitude, local time and MLT over a crossing, sampled evenly at no
// coarser than the scan step and always including both ends.
static void crossing_means(ctx_t *c, crossing_t *x, double step_s)
{
    double span_s = (x->jd1 - x->jd0) * SEC_PER_DAY;
    int n = (int) ceil(span_s / step_s) + 1;
    if (n < 2) n = 2;
    double *lt = malloc((size_t) n * sizeof *lt);
    double *mlt = malloc((size_t) n * sizeof *mlt);
    if (!lt || !mlt) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
    double alt = 0.0;
    for (int i = 0; i < n; ++i) {
        sample_t s;
        if (i == 0) s = x->s0;
        else if (i == n - 1) s = x->s1;
        else compute_sample(c, x->jd0 + (x->jd1 - x->jd0) * i / (n - 1), &s);
        alt += s.alt;
        lt[i] = s.lt;
        mlt[i] = s.mlt;
    }
    x->alt_mean = alt / n;
    x->lt_mean = mean_hours(lt, n);
    x->mlt_mean = mean_hours(mlt, n);
    free(lt);
    free(mlt);
}

static int by_start(const void *a, const void *b)
{
    const crossing_t *x = a, *y = b;
    return (x->jd0 > y->jd0) - (x->jd0 < y->jd0);
}

// ---- arguments --------------------------------------------------------------

#define OPTW 22

// "lo,hi" into two doubles
static int parse_pair(const char *s, double *lo, double *hi)
{
    char tail = 0;
    return sscanf(s, "%lf,%lf%c", lo, hi, &tail) == 2 ? 0 : -1;
}

static int region_option(const char *arg, const char *flag, double *lo,
                         double *hi, double max)
{
    size_t L = strlen(flag);
    if (parse_pair(arg + L, lo, hi) != 0 || *lo < 0.0 || *hi > max
        || (max == 90.0 && *lo >= *hi)) {
        fprintf(stderr, "region_crossings: %.*s wants lo,hi within 0..%g, got '%s'\n",
                (int) L - 1, flag, max, arg + L);
        return -1;
    }
    return 0;
}

static int parse_args(args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        if ((a->tle_path[0] == '\0' && (arg[0] != '-' || strcmp(arg, "-") == 0)) || help) {
            if (help) parse_help_line(OPTW, "[tle-file]", "TLE file to use instead of the packet database's FrontierSat TLE nearest the window");
            else snprintf(a->tle_path, sizeof a->tle_path, "%s", arg);
            matched = 1;
        }
        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "-h, --help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "-V") == 0 || strcmp(arg, "--version") == 0 || help) {
            // Handled by sso_version_handle in main; listed here for --help.
            if (help) parse_help_line(OPTW, "-V, --version", "print the build commit and exit");
            matched = 1;
        }
        if (strncmp(arg, "--sat=", 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sat=<name>", "satellite name in the TLE file, matched from the start of the name line (default FrontierSat)");
            else snprintf(a->sat, sizeof a->sat, "%s", arg + 6);
            matched = 1;
        }
        if (strncmp(arg, "--day=", 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--day=<YYYY-MM-DD>", "start at 00:00 UTC on that day, past or future (default window 24 h)");
            else snprintf(a->day, sizeof a->day, "%s", arg + 6);
            matched = 1;
        }
        if (strncmp(arg, "--start=", 8) == 0 || help) {
            if (help) parse_help_line(OPTW, "--start=<UTC>", "window start, YYYY-MM-DD[THH:MM[:SS]] (default now)");
            else snprintf(a->start, sizeof a->start, "%s", arg + 8);
            matched = 1;
        }
        if (strncmp(arg, "--stop=", 7) == 0 || help) {
            if (help) parse_help_line(OPTW, "--stop=<UTC>", "window stop, same form (default start + --hours)");
            else snprintf(a->stop, sizeof a->stop, "%s", arg + 7);
            matched = 1;
        }
        if (strncmp(arg, "--hours=", 8) == 0 || help) {
            if (help) parse_help_line(OPTW, "--hours=<h>", "window length from the start or day when there is no --stop (default 24)");
            else a->hours = atof(arg + 8);
            matched = 1;
        }
        if (strncmp(arg, "--step=", 7) == 0 || help) {
            if (help) parse_help_line(OPTW, "--step=<s>", "scan step in seconds; shorter visits can be missed (default 10)");
            else a->step_s = atof(arg + 7);
            matched = 1;
        }
        if (strncmp(arg, "--cusp-mlat=", 12) == 0 || help) {
            if (help) parse_help_line(OPTW, "--cusp-mlat=<lo,hi>", "cusp band of |magnetic latitude|, deg (default 70,80)");
            else if (region_option(arg, "--cusp-mlat=", &a->regions[R_CUSP].mlat_lo, &a->regions[R_CUSP].mlat_hi, 90.0)) return PARSE_ERROR;
            matched = 1;
        }
        if (strncmp(arg, "--cusp-mlt=", 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--cusp-mlt=<lo,hi>", "cusp MLT window, hours (default 9,15; 0,24 for any MLT)");
            else if (region_option(arg, "--cusp-mlt=", &a->regions[R_CUSP].mlt_lo, &a->regions[R_CUSP].mlt_hi, 24.0)) return PARSE_ERROR;
            matched = 1;
        }
        if (strncmp(arg, "--aurora-mlat=", 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--aurora-mlat=<lo,hi>", "nightside auroral band, deg (default 62,72)");
            else if (region_option(arg, "--aurora-mlat=", &a->regions[R_AURORA].mlat_lo, &a->regions[R_AURORA].mlat_hi, 90.0)) return PARSE_ERROR;
            matched = 1;
        }
        if (strncmp(arg, "--aurora-mlt=", 13) == 0 || help) {
            if (help) parse_help_line(OPTW, "--aurora-mlt=<lo,hi>", "auroral MLT window, wrapping through midnight (default 18,6)");
            else if (region_option(arg, "--aurora-mlt=", &a->regions[R_AURORA].mlt_lo, &a->regions[R_AURORA].mlt_hi, 24.0)) return PARSE_ERROR;
            matched = 1;
        }
        if (strncmp(arg, "--steve-mlat=", 13) == 0 || help) {
            if (help) parse_help_line(OPTW, "--steve-mlat=<lo,hi>", "subauroral (STEVE) band, deg (default 55,62)");
            else if (region_option(arg, "--steve-mlat=", &a->regions[R_STEVE].mlat_lo, &a->regions[R_STEVE].mlat_hi, 90.0)) return PARSE_ERROR;
            matched = 1;
        }
        if (strncmp(arg, "--steve-mlt=", 12) == 0 || help) {
            if (help) parse_help_line(OPTW, "--steve-mlt=<lo,hi>", "subauroral MLT window (default 18,24, pre-midnight)");
            else if (region_option(arg, "--steve-mlt=", &a->regions[R_STEVE].mlt_lo, &a->regions[R_STEVE].mlt_hi, 24.0)) return PARSE_ERROR;
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "region_crossings: unknown option '%s' (try --help)\n", arg);
            return PARSE_ERROR;
        }
    }
    if (help) {
        printf("\nMagnetic latitude is quasi-dipole latitude at the satellite (IGRF-14);\n"
               "MLT follows apexpy; LT is local solar time. A region holds when\n"
               "lo <= |magnetic latitude| < hi and the MLT is inside its window.\n");
    }
    return PARSE_OK;
}

// ---- main -------------------------------------------------------------------

static void print_region_line(const region_t *r)
{
    printf("  %-7s %4.1f-%4.1f deg   MLT %04.1f-%04.1f\n", r->name,
           r->mlat_lo, r->mlat_hi, r->mlt_lo, r->mlt_hi);
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "region_crossings")) return 0;

    args_t a = {0};
    a.regions[R_CUSP]   = (region_t) { "cusp",   70.0, 80.0,  9.0, 15.0 };
    a.regions[R_AURORA] = (region_t) { "aurora", 62.0, 72.0, 18.0,  6.0 };
    a.regions[R_STEVE]  = (region_t) { "STEVE",  55.0, 62.0, 18.0, 24.0 };
    snprintf(a.sat, sizeof a.sat, "FrontierSat");
    a.hours = 24.0;
    a.step_s = 10.0;
    switch (parse_args(&a, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }

    // ---- window
    if (a.day[0] && a.start[0]) {
        fprintf(stderr, "region_crossings: give --day or --start, not both\n");
        return 1;
    }
    double jd0, jd1;
    const char *start_text = a.day[0] ? a.day : a.start;
    if (start_text[0]) {
        if (parse_utc(start_text, &jd0) != 0) {
            fprintf(stderr, "region_crossings: cannot read the date '%s' (YYYY-MM-DD[THH:MM[:SS]])\n", start_text);
            return 1;
        }
    } else {
        jd0 = now_jd();
    }
    if (a.stop[0]) {
        if (parse_utc(a.stop, &jd1) != 0) {
            fprintf(stderr, "region_crossings: cannot read the date '%s' (YYYY-MM-DD[THH:MM[:SS]])\n", a.stop);
            return 1;
        }
    } else {
        jd1 = jd0 + a.hours / 24.0;
    }
    if (jd1 <= jd0) {
        fprintf(stderr, "region_crossings: the window stops before it starts\n");
        return 1;
    }
    if (a.step_s <= 0.0) {
        fprintf(stderr, "region_crossings: --step must be positive\n");
        return 1;
    }

    // ---- orbit
    ctx_t c;
    memset(&c, 0, sizeof c);
    char tle_src[600];
    if (a.tle_path[0]) {
        c.pred.tles_filename = a.tle_path;
        c.pred.satellite_ephem.name = a.sat;
        if (load_tle(&c.pred) != 0) return 1;
        snprintf(tle_src, sizeof tle_src, "%s", a.tle_path);
    } else {
#ifdef WITH_SQLITE3
        if (tle_from_db(&c, 0.5 * (jd0 + jd1), tle_src, sizeof tle_src) != 0) return 1;
#else
        fprintf(stderr, "region_crossings: built without SQLite, so give a TLE file\n");
        return 1;
#endif
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&c.pred.satellite_ephem.tle);
    double jd_epoch = Julian_Date_of_Epoch(c.pred.satellite_ephem.tle.epoch);

    double year = igrf_decimal_year(0.5 * (jd0 + jd1));
    igrf_init(&c.igrf, year);

    // ---- header
    char t0[32], t1[32], te[32];
    fmt_utc(jd0, 1, t0, sizeof t0);
    fmt_utc(jd1, 1, t1, sizeof t1);
    fmt_utc(jd_epoch, 1, te, sizeof te);
    printf("%s region crossings, %s to %s UTC\n", c.pred.satellite_ephem.tle.sat_name, t0, t1);
    printf("TLE epoch: %s UTC, ", te);
    if (jd_epoch < jd0)
        printf("%.1f days before the window starts", jd0 - jd_epoch);
    else if (jd_epoch > jd1)
        printf("%.1f days after the window ends", jd_epoch - jd1);
    else
        printf("inside the window");
    printf(" (%s)\n", tle_src);
    // How far the TLE is propagated to reach the far end of the window,
    // forward or back: SGP4 accuracy falls off with that distance.
    double reach = fmax(fabs(jd1 - jd_epoch), fabs(jd0 - jd_epoch));
    if (reach > TLE_MAX_AGE_DAYS)
        printf("WARNING: the window reaches %.1f days from the TLE epoch (more than %.0f);"
               " crossing times lose accuracy as a TLE ages.\n", reach, TLE_MAX_AGE_DAYS);
    if (year < IGRF_YEAR_FIRST || year > IGRF_YEAR_LAST)
        printf("Note: %.1f is outside IGRF-14's %.0f-%.0f span; the field is extrapolated.\n",
               year, IGRF_YEAR_FIRST, IGRF_YEAR_LAST);
    printf("Regions (|quasi-dipole latitude|, MLT):\n");
    for (int r = 0; r < N_REGIONS; ++r) print_region_line(&a.regions[r]);
    printf("\n");

    // ---- scan
    crossing_t *list = NULL;
    int n = 0, cap = 0;
    int open[N_REGIONS] = {0}, hemi[N_REGIONS] = {0};
    crossing_t cur[N_REGIONS];
    memset(cur, 0, sizeof cur);
    double step = a.step_s / SEC_PER_DAY;
    long nsteps = (long) ceil((jd1 - jd0) / step);
    double jd_prev = jd0;
    for (long i = 0; i <= nsteps; ++i) {
        double jd = (i == nsteps) ? jd1 : jd0 + i * step;
        sample_t s;
        compute_sample(&c, jd, &s);
        for (int r = 0; r < N_REGIONS; ++r) {
            const region_t *rg = &a.regions[r];
            int h = in_region(rg, &s);
            if (open[r] && h != hemi[r]) {
                refine(&c, rg, hemi[r], jd_prev, 1, jd, &cur[r].s1);
                cur[r].jd1 = cur[r].s1.jd;
                if (n == cap) {
                    cap = cap ? 2 * cap : 64;
                    crossing_t *g = realloc(list, (size_t) cap * sizeof *g);
                    if (!g) { fprintf(stderr, "Out of memory\n"); return 1; }
                    list = g;
                }
                list[n++] = cur[r];
                open[r] = 0;
            }
            if (!open[r] && h != 0) {
                memset(&cur[r], 0, sizeof cur[r]);
                cur[r].region = r;
                cur[r].hemi = h;
                if (i == 0) cur[r].s0 = s;
                else refine(&c, rg, h, jd_prev, 0, jd, &cur[r].s0);
                cur[r].jd0 = cur[r].s0.jd;
                open[r] = 1;
                hemi[r] = h;
            }
        }
        jd_prev = jd;
        if (i == nsteps) {
            // Close whatever is still open at the window's end
            for (int r = 0; r < N_REGIONS; ++r) {
                if (!open[r] || cur[r].jd0 >= jd) continue;
                cur[r].s1 = s;
                cur[r].jd1 = jd;
                if (n == cap) {
                    cap = cap ? 2 * cap : 64;
                    crossing_t *g = realloc(list, (size_t) cap * sizeof *g);
                    if (!g) { fprintf(stderr, "Out of memory\n"); return 1; }
                    list = g;
                }
                list[n++] = cur[r];
            }
        }
    }

    if (n > 0) qsort(list, (size_t) n, sizeof *list, by_start);
    for (int i = 0; i < n; ++i) crossing_means(&c, &list[i], a.step_s);

    // ---- table
    if (n == 0) {
        printf("No crossings in this window.\n");
        free(list);
        return 0;
    }
    printf("%-19s  %-8s  %5s  %-6s  %-4s  %6s  %5s  %5s  %13s  %13s\n",
           "Start (UTC)", "Stop", "Dur", "Region", "Hemi", "Alt km",
           "LT h", "MLT h", "Lat start/end", "MLat start/end");
    for (int i = 0; i < n; ++i) {
        const crossing_t *x = &list[i];
        char s0[32], s1[32], dur[48];
        fmt_utc(x->jd0, 1, s0, sizeof s0);
        fmt_utc(x->jd1, 0, s1, sizeof s1);
        long secs = lround((x->jd1 - x->jd0) * SEC_PER_DAY);
        snprintf(dur, sizeof dur, "%ld:%02ld", secs / 60, secs % 60);
        printf("%-19s  %-8s  %5s  %-6s  %-4s  %6.0f  %5.1f  %5.1f  %6.1f %6.1f  %6.1f %6.1f\n",
               s0, s1, dur, a.regions[x->region].name, x->hemi > 0 ? "N" : "S",
               x->alt_mean, x->lt_mean, x->mlt_mean,
               x->s0.lat, x->s1.lat, x->s0.qdlat, x->s1.qdlat);
    }
    free(list);
    return 0;
}
