/*

    Simple Satellite Operations  tle_keps.c

    Summarise the orbital elements ("keps") for one or more objects in a
    TLE file: the mean Keplerian elements straight from the two-line set,
    plus the geometry they imply (semi-major axis, apogee/perigee
    altitude, period), the J2 nodal precession rate, and the local time of
    the ascending/descending node (LTAN/LTDN).

    With no arguments it summarises the newest dated FrontierSat TLE, the
    one fetch_tle.sh drops at <root>/TLEs/YYYYMMDD/tle-YYYYMMDD.tle. Give a
    TLE file and/or one or more satellite-name prefixes to pick others.

    Read-only and static: it reports the elements at their epoch, so it
    needs no observer location, no current-time propagation, and no
    hardware. It does NOT run SGP4/SDP4 (and so never calls the
    elements-rewriting select_ephemeris): everything shown is either read
    directly from the TLE or derived in closed form from it. The live
    ephemeris, Doppler, and next pass are tle_compare's / next_in_queue's
    job. Builds anywhere libsgp4sdp4 is present; no ncurses or audio.

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

#include "sso_paths.h"

#include <sgp4sdp4.h>

#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

// Physical constants. These mirror the values sgp4sdp4 uses internally
// (xj2, an Earth radius of 6378.135-6378.137 km) but are defined here so
// the tool does not need the library's SGP4SDP4_CONSTANTS macro block.
#define GM_KM3_S2        398600.4418    // Earth gravitational parameter (WGS84)
#define EARTH_RADIUS_KM  6378.137       // WGS84 equatorial radius
#define J2_HARMONIC      1.0826158e-3   // Earth J2 (matches sgp4sdp4 xj2)

// Nodal precession rate of a sun-synchronous orbit: the node must drift
// east at the rate the mean sun moves along the equator, 360 deg per
// tropical year. Within this tolerance of it we flag the orbit sun-sync.
#define SUNSYNC_DEG_DAY  0.9856473
#define SUNSYNC_TOL      0.05

// One parsed two-line set, in the raw TLE units Convert_Satellite_Data
// leaves behind (angles in degrees, mean motion in rev/day): we never
// call select_ephemeris, which would rewrite them to radians/rad-min.
typedef struct {
    tle_t tle;
} kep_t;

// ---- TLE file reading -------------------------------------------------------

// Read the next non-blank line, stripped of trailing CR/LF and spaces.
// Returns 0 at end of file.
static int read_line(FILE *fp, char *buf, size_t cap)
{
    while (fgets(buf, (int) cap, fp)) {
        size_t n = strlen(buf);
        while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r'
                     || buf[n - 1] == ' ' || buf[n - 1] == '\t'))
            buf[--n] = '\0';
        if (n == 0) continue;
        return 1;
    }
    return 0;
}

// True if the line looks like TLE card 1 or card 2 ("1 " / "2 " prefix).
static int is_element_line(const char *s, char card)
{
    return s[0] == card && s[1] == ' ';
}

// Read every object in a TLE file into a grown array. Handles the usual
// 3-line groups (name, card 1, card 2), the bare "0 " 3LE name prefix,
// and nameless 2-line files. Each group is validated with Good_Elements
// and decoded with Convert_Satellite_Data, exactly as load_tle does for
// the single-object case. Returns 0 on success (out/count filled), -1 if
// the file can't be opened. A file with no valid sets yields count 0.
static int read_all_tles(const char *path, kep_t **out, int *count)
{
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;

    kep_t *arr = NULL;
    int n = 0, cap = 0;

    char name[160], l1[160], l2[160];
    while (read_line(fp, name, sizeof name)) {
        const char *nm = name;
        // A bare two-line set has no name line: the line we just read is
        // already card 1. Otherwise strip an optional "0 " 3LE prefix.
        if (is_element_line(name, '1')) {
            snprintf(l1, sizeof l1, "%s", name);
            if (!read_line(fp, l2, sizeof l2)) break;
            nm = "";
        } else {
            if (nm[0] == '0' && nm[1] == ' ') nm += 2;
            if (!read_line(fp, l1, sizeof l1)) break;
            if (!read_line(fp, l2, sizeof l2)) break;
        }

        // sgp4sdp4 wants card 1 at [0..68] and card 2 at [69..137].
        char set[139];
        memset(set, 0, sizeof set);
        size_t a = strlen(l1), b = strlen(l2);
        if (a > 69) a = 69;
        if (b > 69) b = 69;
        memcpy(set, l1, a);
        memcpy(set + 69, l2, b);
        set[138] = '\0';

        if (!Good_Elements(set)) continue;   // skip a malformed group

        if (n == cap) {
            int ncap = cap ? cap * 2 : 64;
            kep_t *grown = realloc(arr, (size_t) ncap * sizeof *arr);
            if (!grown) { free(arr); fclose(fp); return -1; }
            arr = grown;
            cap = ncap;
        }
        memset(&arr[n], 0, sizeof arr[n]);
        snprintf(arr[n].tle.sat_name, sizeof arr[n].tle.sat_name, "%s", nm);
        Convert_Satellite_Data(set, &arr[n].tle);
        n++;
    }

    fclose(fp);
    *out = arr;
    *count = n;
    return 0;
}

// ---- default TLE path -------------------------------------------------------

// True if s is exactly eight decimal digits (a YYYYMMDD day directory).
static int is_yyyymmdd(const char *s)
{
    if (strlen(s) != 8) return 0;
    for (int i = 0; i < 8; ++i)
        if (!isdigit((unsigned char) s[i])) return 0;
    return 1;
}

// Resolve the newest dated FrontierSat TLE:
// <root>/TLEs/<latest YYYYMMDD>/tle-<YYYYMMDD>.tle. Falls back to any
// *.tle in that day directory if the canonical name is absent, and to a
// tle-*.tle directly under TLEs/ if there are no day directories.
// Returns 0 and fills out on success, -1 if nothing suitable was found.
static int newest_dated_tle(char *out, size_t cap)
{
    const char *tles = sso_tles_dir();
    DIR *d = opendir(tles);
    if (!d) return -1;

    char latest[16] = {0};
    struct dirent *e;
    while ((e = readdir(d))) {
        if (!is_yyyymmdd(e->d_name)) continue;
        // Confirm it is a directory (d_type is not portable enough alone).
        char sub[512];
        snprintf(sub, sizeof sub, "%s/%s", tles, e->d_name);
        struct stat st;
        if (stat(sub, &st) != 0 || !S_ISDIR(st.st_mode)) continue;
        if (strcmp(e->d_name, latest) > 0)
            snprintf(latest, sizeof latest, "%s", e->d_name);
    }
    closedir(d);

    if (latest[0]) {
        // Canonical name first.
        struct stat st;
        snprintf(out, cap, "%s/%s/tle-%s.tle", tles, latest, latest);
        if (stat(out, &st) == 0 && S_ISREG(st.st_mode)) return 0;
        // Otherwise the first *.tle in that day directory.
        char sub[512];
        snprintf(sub, sizeof sub, "%s/%s", tles, latest);
        DIR *dd = opendir(sub);
        if (dd) {
            struct dirent *f;
            char first[256] = {0};
            while ((f = readdir(dd))) {
                size_t L = strlen(f->d_name);
                if (L > 4 && strcmp(f->d_name + L - 4, ".tle") == 0
                    && (first[0] == '\0' || strcmp(f->d_name, first) < 0))
                    snprintf(first, sizeof first, "%s", f->d_name);
            }
            closedir(dd);
            if (first[0]) {
                snprintf(out, cap, "%s/%s/%s", tles, latest, first);
                return 0;
            }
        }
    }

    // No day directories: a tle-*.tle sitting directly under TLEs/.
    DIR *d2 = opendir(tles);
    if (d2) {
        struct dirent *f;
        char best[256] = {0};
        while ((f = readdir(d2))) {
            size_t L = strlen(f->d_name);
            if (L > 4 && strncmp(f->d_name, "tle-", 4) == 0
                && strcmp(f->d_name + L - 4, ".tle") == 0
                && strcmp(f->d_name, best) > 0)
                snprintf(best, sizeof best, "%s", f->d_name);
        }
        closedir(d2);
        if (best[0]) {
            snprintf(out, cap, "%s/%s", tles, best);
            return 0;
        }
    }
    return -1;
}

// ---- derived quantities -----------------------------------------------------

typedef struct {
    double sma_km;          // semi-major axis
    double apogee_km;       // apogee altitude above mean equatorial radius
    double perigee_km;      // perigee altitude
    double period_min;      // orbital period
    double node_rate_dd;    // J2 secular nodal precession, deg/day
    int    sun_sync;        // node rate within tolerance of sun-sync
    double jd_epoch;        // epoch as a Julian date
    double ltan_h;          // local time of ascending node, hours [0,24)
    double ltdn_h;          // local time of descending node
} derived_t;

// Right ascension of the (apparent) sun, in degrees [0,360), at a Julian
// date, via sgp4sdp4's solar model. The solar vector is geocentric
// equatorial, so RA = atan2(y, x) in the same frame RAAN is measured in.
static double sun_ra_deg(double jd)
{
    vector_t sun = {0};
    Calculate_Solar_Position(jd, &sun);
    double ra = atan2(sun.y, sun.x) * 180.0 / M_PI;
    if (ra < 0.0) ra += 360.0;
    return ra;
}

static double wrap24(double h)
{
    h = fmod(h, 24.0);
    if (h < 0.0) h += 24.0;
    return h;
}

static void compute_derived(const tle_t *t, derived_t *d)
{
    memset(d, 0, sizeof *d);

    double e = t->eo;
    double n_rev_day = t->xno;                      // mean motion, rev/day
    if (n_rev_day > 0.0) {
        double n_rad_s = n_rev_day * 2.0 * M_PI / 86400.0;
        d->sma_km = cbrt(GM_KM3_S2 / (n_rad_s * n_rad_s));
        d->apogee_km  = d->sma_km * (1.0 + e) - EARTH_RADIUS_KM;
        d->perigee_km = d->sma_km * (1.0 - e) - EARTH_RADIUS_KM;
        d->period_min = 1440.0 / n_rev_day;

        // Secular J2 regression of the ascending node:
        //   d(RAAN)/dt = -1.5 n J2 (Re/p)^2 cos(i),  p = a(1 - e^2).
        double incl = t->xincl * M_PI / 180.0;       // degrees -> radians
        double p = d->sma_km * (1.0 - e * e);
        if (p > 0.0) {
            double f = EARTH_RADIUS_KM / p;
            double rate_rad_s = -1.5 * n_rad_s * J2_HARMONIC * f * f * cos(incl);
            d->node_rate_dd = rate_rad_s * (180.0 / M_PI) * 86400.0;
            d->sun_sync = fabs(d->node_rate_dd - SUNSYNC_DEG_DAY) < SUNSYNC_TOL;
        }
    }

    // Local time of the node: LTAN = (RAAN - RA_sun)/15 + 12 hours, with
    // both angles in the equatorial frame. Uses the apparent sun, so it is
    // local apparent solar time (within the equation of time, <~16 min, of
    // mean local time). Evaluated at the element epoch.
    d->jd_epoch = Julian_Date_of_Epoch(t->epoch);
    double ra_sun = sun_ra_deg(d->jd_epoch);
    d->ltan_h = wrap24((t->xnodeo - ra_sun) / 15.0 + 12.0);
    d->ltdn_h = wrap24(d->ltan_h + 12.0);
}

// ---- formatting helpers -----------------------------------------------------

static double now_jul_utc(void)
{
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    return Julian_Date(&utc, &tv);
}

static void fmt_utc(double jul, char *buf, size_t n)
{
    time_t t = (time_t) ((jul - 2440587.5) * 86400.0 + 0.5);
    struct tm g;
    if (gmtime_r(&t, &g) == NULL) { snprintf(buf, n, "----------"); return; }
    strftime(buf, n, "%Y-%m-%d %H:%M:%SZ", &g);
}

static void fmt_hm(double hours, char *buf, size_t n)
{
    int total_min = (int) (hours * 60.0 + 0.5);
    total_min %= 1440;
    if (total_min < 0) total_min += 1440;
    snprintf(buf, n, "%02d:%02d", total_min / 60, total_min % 60);
}

// Expand the 8-char international designator "YYNNNPPP" to the readable
// COSPAR id "YYYY-NNNP". Falls back to the raw (trimmed) field if it does
// not parse. The two-digit year rolls over at 57 (pre-2000 launches).
static void fmt_cospar(const char *idesg, char *buf, size_t n)
{
    char trimmed[16] = {0};
    snprintf(trimmed, sizeof trimmed, "%s", idesg);
    for (int i = (int) strlen(trimmed) - 1; i >= 0 && isspace((unsigned char) trimmed[i]); --i)
        trimmed[i] = '\0';

    if (strlen(trimmed) >= 5
        && isdigit((unsigned char) trimmed[0]) && isdigit((unsigned char) trimmed[1])
        && isdigit((unsigned char) trimmed[2]) && isdigit((unsigned char) trimmed[3])
        && isdigit((unsigned char) trimmed[4])) {
        int yy = (trimmed[0] - '0') * 10 + (trimmed[1] - '0');
        int year = (yy < 57) ? 2000 + yy : 1900 + yy;
        snprintf(buf, n, "%04d-%.3s%s", year, trimmed + 2, trimmed + 5);
    } else if (trimmed[0]) {
        snprintf(buf, n, "%s", trimmed);
    } else {
        snprintf(buf, n, "(none)");
    }
}

// Object display name: the TLE name line, or the catalog number when the
// set carried no name.
static void obj_name(const tle_t *t, char *buf, size_t n)
{
    if (t->sat_name[0])
        snprintf(buf, n, "%s", t->sat_name);
    else
        snprintf(buf, n, "(catalog %d)", t->catnr);
}

// ---- report (default, human) ------------------------------------------------

#define LBL 18   // label column width

static void print_report(const kep_t *o, double jul_now)
{
    const tle_t *t = &o->tle;
    derived_t d;
    compute_derived(t, &d);

    char name[160], cospar[24], epoch[32];
    obj_name(t, name, sizeof name);
    fmt_cospar(t->idesg, cospar, sizeof cospar);
    fmt_utc(d.jd_epoch, epoch, sizeof epoch);
    double age_d = jul_now - d.jd_epoch;

    char ltan[8], ltdn[8];
    fmt_hm(d.ltan_h, ltan, sizeof ltan);
    fmt_hm(d.ltdn_h, ltdn, sizeof ltdn);

    printf("%s   (NORAD %d, intl %s)\n", name, t->catnr, cospar);
    printf("  %-*s %s   (age %.2f d)\n", LBL, "epoch (UTC)", epoch, age_d);
    printf("  %-*s %d        rev at epoch %d\n", LBL, "element set", t->elset, t->revnum);

    printf("  mean elements (Keplerian):\n");
    printf("    %-*s %10.4f deg\n",  LBL, "inclination",    t->xincl);
    printf("    %-*s %10.4f deg\n",  LBL, "RAAN (asc node)", t->xnodeo);
    printf("    %-*s %10.7f\n",      LBL, "eccentricity",   t->eo);
    printf("    %-*s %10.4f deg\n",  LBL, "arg of perigee", t->omegao);
    printf("    %-*s %10.4f deg\n",  LBL, "mean anomaly",   t->xmo);
    printf("    %-*s %12.8f rev/day\n", LBL, "mean motion", t->xno);
    printf("    %-*s %10.3e rev/day^2  (n-dot/2)\n", LBL, "mm 1st deriv", t->xndt2o);
    printf("    %-*s %10.4e 1/Re\n",  LBL, "B* drag",       t->bstar);

    printf("  derived:\n");
    printf("    %-*s %10.1f km\n",   LBL, "semi-major axis", d.sma_km);
    printf("    %-*s %10.1f km\n",   LBL, "apogee altitude", d.apogee_km);
    printf("    %-*s %10.1f km\n",   LBL, "perigee altitude", d.perigee_km);
    printf("    %-*s %10.2f min   (%.4f orbits/day)\n",
           LBL, "orbital period", d.period_min, t->xno);
    printf("    %-*s %+10.4f deg/day%s\n",
           LBL, "node precession", d.node_rate_dd,
           d.sun_sync ? "   (sun-synchronous)" : "");
    printf("    %-*s %10s   (asc node, apparent-sun local time)\n",
           LBL, "LTAN", ltan);
    printf("    %-*s %10s   (desc node)\n", LBL, "LTDN", ltdn);
}

// ---- CSV (one row per object, for the passes sheet) -------------------------

static void print_csv_header(void)
{
    printf("name,norad,cospar,epoch_utc,age_days,incl_deg,raan_deg,ecc,"
           "argp_deg,ma_deg,mean_motion_rev_day,sma_km,apogee_km,perigee_km,"
           "period_min,node_rate_deg_day,sun_sync,ltan,ltdn\n");
}

// Quote a CSV field if it contains a comma or quote (satellite names can).
static void csv_field(const char *s)
{
    if (strpbrk(s, ",\"\n")) {
        putchar('"');
        for (const char *p = s; *p; ++p) {
            if (*p == '"') putchar('"');
            putchar(*p);
        }
        putchar('"');
    } else {
        fputs(s, stdout);
    }
}

static void print_csv_row(const kep_t *o, double jul_now)
{
    const tle_t *t = &o->tle;
    derived_t d;
    compute_derived(t, &d);

    char name[160], cospar[24], epoch[32], ltan[8], ltdn[8];
    obj_name(t, name, sizeof name);
    fmt_cospar(t->idesg, cospar, sizeof cospar);
    fmt_utc(d.jd_epoch, epoch, sizeof epoch);
    fmt_hm(d.ltan_h, ltan, sizeof ltan);
    fmt_hm(d.ltdn_h, ltdn, sizeof ltdn);

    csv_field(name);
    printf(",%d,", t->catnr);
    csv_field(cospar);
    printf(",%s,%.3f,%.4f,%.4f,%.7f,%.4f,%.4f,%.8f,%.1f,%.1f,%.1f,%.2f,%.4f,%d,%s,%s\n",
           epoch, jul_now - d.jd_epoch,
           t->xincl, t->xnodeo, t->eo, t->omegao, t->xmo, t->xno,
           d.sma_km, d.apogee_km, d.perigee_km, d.period_min,
           d.node_rate_dd, d.sun_sync, ltan, ltdn);
}

// ---- main -------------------------------------------------------------------

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options] [tle-file] [name ...]\n"
        "\n"
        "Summarise the orbital elements (\"keps\") of objects in a TLE file:\n"
        "the mean Keplerian elements, the geometry they imply (semi-major\n"
        "axis, apogee/perigee altitude, period), the J2 nodal precession\n"
        "rate, and the local time of the ascending/descending node.\n"
        "\n"
        "With no file it uses the newest dated FrontierSat TLE\n"
        "(<root>/TLEs/YYYYMMDD/tle-YYYYMMDD.tle). With no names it reports\n"
        "every object in the file; otherwise it reports each object whose\n"
        "name line starts with one of the given prefixes (case-sensitive).\n"
        "\n"
        "Options:\n"
        "  --tle <path>   TLE file to read (also accepted as a positional)\n"
        "  --csv          one comma-separated row per object (for a sheet),\n"
        "                 instead of the human report\n"
        "  -h, --help     this text\n"
        "  -V, --version  print the build commit and exit\n",
        prog);
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "tle_keps")) return 0;

    char tle_path[512] = {0};
    int csv = 0;

    // Positionals: the first one that names an existing regular file is the
    // TLE file; the rest are satellite-name prefixes.
    const char *names[256];
    int n_names = 0;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(argv[0]);
            return EXIT_SUCCESS;
        } else if (strcmp(a, "--csv") == 0) {
            csv = 1;
        } else if (strcmp(a, "--tle") == 0 && i + 1 < argc) {
            snprintf(tle_path, sizeof tle_path, "%s", argv[++i]);
        } else if (strncmp(a, "--tle=", 6) == 0) {
            snprintf(tle_path, sizeof tle_path, "%s", a + 6);
        } else if (a[0] == '-' && a[1] != '\0') {
            fprintf(stderr, "Unknown option: %s\n", a);
            usage(argv[0]);
            return EXIT_FAILURE;
        } else {
            struct stat st;
            if (tle_path[0] == '\0' && stat(a, &st) == 0 && S_ISREG(st.st_mode)) {
                snprintf(tle_path, sizeof tle_path, "%s", a);
            } else if (n_names < (int) (sizeof names / sizeof names[0])) {
                names[n_names++] = a;
            }
        }
    }

    if (tle_path[0] == '\0' && newest_dated_tle(tle_path, sizeof tle_path) != 0) {
        fprintf(stderr,
            "No TLE file given and no dated TLE found under %s.\n"
            "Pass a file (e.g. %s TLEs/amateur.tle \"ISS (ZARYA)\")\n"
            "or set FRONTIERSAT_ROOT to point at your data tree.\n",
            sso_tles_dir(), argv[0]);
        return EXIT_FAILURE;
    }

    kep_t *objs = NULL;
    int n = 0;
    if (read_all_tles(tle_path, &objs, &n) != 0) {
        fprintf(stderr, "Error opening %s\n", tle_path);
        return EXIT_FAILURE;
    }
    if (n == 0) {
        fprintf(stderr, "No valid TLE sets in %s\n", tle_path);
        free(objs);
        return EXIT_FAILURE;
    }

    double jul_now = now_jul_utc();

    if (csv) print_csv_header();
    else {
        char now_s[32];
        fmt_utc(jul_now, now_s, sizeof now_s);
        printf("tle_keps   %s\n%s\n\n", tle_path, now_s);
    }

    int shown = 0;
    if (n_names == 0) {
        // No names: every object in the file.
        for (int i = 0; i < n; ++i) {
            if (csv) print_csv_row(&objs[i], jul_now);
            else { if (shown) putchar('\n'); print_report(&objs[i], jul_now); }
            shown++;
        }
    } else {
        // Each requested prefix, in the order asked, first match wins.
        for (int k = 0; k < n_names; ++k) {
            size_t L = strlen(names[k]);
            int hit = -1;
            for (int i = 0; i < n; ++i) {
                if (strncmp(objs[i].tle.sat_name, names[k], L) == 0) { hit = i; break; }
            }
            if (hit < 0) {
                if (csv) { csv_field(names[k]); printf(",,,(not found)\n"); }
                else {
                    if (shown) putchar('\n');
                    printf("%s   (not found in %s)\n", names[k], tle_path);
                }
            } else {
                if (csv) print_csv_row(&objs[hit], jul_now);
                else { if (shown) putchar('\n'); print_report(&objs[hit], jul_now); }
            }
            shown++;
        }
    }

    free(objs);
    return EXIT_SUCCESS;
}
