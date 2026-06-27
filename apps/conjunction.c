/*

    Simple Satellite Operations  conjunction.c

    Closest-approach (conjunction) finder for two satellites given their
    TLEs. It propagates both objects forward (one week by default) with the
    same SGP4/SDP4 core the rest of the toolchain uses, finds the first time
    they pass within a threshold distance, and reports that encounter: the
    miss distance and its breakdown into radial (height), along-track, and
    cross-track separation in the primary's orbit frame; the time of closest
    approach in UTC and local time and how long until it happens; the
    relative speed; and Foster's (1992) 2-D probability of collision for an
    assumed (operator-supplied) covariance and hard-body size.

    A TLE carries no covariance, so the Pc is only as good as the assumed
    1-sigma uncertainties -- it is a screening aid, not a flight-safety
    product. Each object's TLE age is printed so a stale element set is
    obvious.

    Read-only and observer-independent: a conjunction is a fact about the
    two orbits, not about any ground station, so it takes no location. Build
    needs only the SGP4SDP4 library (no ncurses, no UHD, no ALSA), so it runs
    on any host that builds next_in_queue.

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

#include "argparse.h"
#include "conjunction.h"
#include "duration_fmt.h"
#include "prediction.h"
#include "sso_version.h"

#include <sgp4sdp4.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define OPTW 26

// One object: its loaded + once-converted elements, deep-space verdict, and
// TLE epoch. Mirrors the multi-satellite handling in tle_compare.c: the
// sgp4sdp4 library keeps the chosen ephemeris in module-level globals, so when
// two satellites share the process we restore the converted elements and
// re-assert the deep-space flag (without re-running the destructive
// select_ephemeris) before each propagation.
typedef struct {
    const char  *file;          // TLE file this object is read from
    const char  *name;          // requested name prefix (case-sensitive)
    prediction_t pred;          // holds the loaded tle + scratch state
    tle_t        tle_ready;     // converted elements, captured once
    int          deep_space;    // 1 if select_ephemeris flagged SDP4
    double       epoch_jul;     // TLE epoch as a Julian date
    char         namebuf[64];   // backing store for pred.satellite_ephem.name
} object_t;

typedef struct {
    const char *file1;
    const char *name1;
    const char *file2;          // NULL => same file as file1
    const char *name2;
    double days;                // forward search window
    double step_sec;            // coarse scan step
    double threshold_km;        // conjunction distance gate
    int    all;                 // report every event below threshold
    double hbr_m;               // combined hard-body radius, metres
    // Per-object 1-sigma position uncertainty in RTN, metres.
    double sig1_r, sig1_a, sig1_c;
    double sig2_r, sig2_a, sig2_c;
    int    plot;                // write a gnuplot 3-D view of the encounter
    const char *plot_out;       // output base / PNG path (NULL => "conjunction")
    double plot_window_sec;     // half-window around TCA shown in the plot
    int    n_positional;        // count of bare (non-option) tokens seen
} args_t;

// Default per-object 1-sigma position uncertainty (metres, RTN). A placeholder
// representative of a recent TLE -- in-track dominant -- not a measured
// covariance. Document loudly that Pc is only as good as this.
#define DEF_SIG_R 200.0
#define DEF_SIG_A 1000.0
#define DEF_SIG_C 200.0
#define DEF_HBR_M 20.0

// ---- time helpers ----------------------------------------------------------

static double now_jul_utc(void)
{
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    return Julian_Date(&utc, &tv);
}

static time_t jul_to_unix(double jul_utc)
{
    return (time_t) ((jul_utc - 2440587.5) * 86400.0 + 0.5);
}

static void fmt_utc(double jul_utc, char *buf, size_t n)
{
    time_t t = jul_to_unix(jul_utc);
    struct tm ut;
    if (gmtime_r(&t, &ut) == NULL) { snprintf(buf, n, "----------"); return; }
    strftime(buf, n, "%Y-%m-%d %H:%M:%S UTC", &ut);
}

// Split a Julian date into whole UTC seconds + milliseconds, without the
// round-to-nearest-second jul_to_unix() does -- whole-second rounding would
// hide the refined TCA precision (a 15 km/s closing speed moves ~15 m per ms).
static void jul_split_ms(double jul_utc, time_t *whole, int *ms)
{
    double secs = (jul_utc - 2440587.5) * 86400.0;
    time_t w = (time_t) floor(secs);
    int m = (int) ((secs - (double) w) * 1000.0 + 0.5);
    if (m >= 1000) { m -= 1000; w += 1; }
    *whole = w;
    *ms = m;
}

// Time of closest approach with milliseconds: "2026-06-28 07:18:33.531 UTC".
static void fmt_utc_ms(double jul_utc, char *buf, size_t n)
{
    time_t w; int ms;
    jul_split_ms(jul_utc, &w, &ms);
    struct tm ut;
    if (gmtime_r(&w, &ut) == NULL) { snprintf(buf, n, "----------"); return; }
    char s[32];
    strftime(s, sizeof s, "%Y-%m-%d %H:%M:%S", &ut);
    snprintf(buf, n, "%s.%03d UTC", s, ms);
}

static void fmt_local_ms(double jul_utc, char *buf, size_t n)
{
    time_t w; int ms;
    jul_split_ms(jul_utc, &w, &ms);
    struct tm lt;
    if (localtime_r(&w, &lt) == NULL) { snprintf(buf, n, "----------"); return; }
    char s[32], z[16];
    strftime(s, sizeof s, "%Y-%m-%d %H:%M:%S", &lt);
    strftime(z, sizeof z, "%Z", &lt);
    snprintf(buf, n, "%s.%03d %s", s, ms, z);
}

// ---- unit-convenient distance formatting -----------------------------------

// A separation that may be small: show metres under 1 km, kilometres above.
// Keeps the sign so the radial/along/cross components read as offsets.
static void fmt_len(double km, char *buf, size_t n)
{
    if (fabs(km) < 1.0)
        snprintf(buf, n, "%+.1f m", km * 1000.0);
    else
        snprintf(buf, n, "%+.3f km", km);
}

// An unsigned magnitude (the miss distance): metres under 1 km else km.
static void fmt_mag(double km, char *buf, size_t n)
{
    if (km < 1.0)
        snprintf(buf, n, "%.1f m", km * 1000.0);
    else
        snprintf(buf, n, "%.3f km", km);
}

// ---- propagation (mirrors tle_compare.c) -----------------------------------

// One-time per object after load_tle: convert the elements (select_ephemeris
// rewrites units in place and decides SGP4 vs SDP4) and capture the converted
// copy + the deep-space verdict + the epoch. Must run exactly once.
static void setup_object(object_t *o)
{
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&o->pred.satellite_ephem.tle);
    o->deep_space = isFlagSet(DEEP_SPACE_EPHEM_FLAG) ? 1 : 0;
    o->tle_ready  = o->pred.satellite_ephem.tle;
    o->epoch_jul  = Julian_Date_of_Epoch(o->tle_ready.epoch);
}

// Restore this object's once-converted elements and re-assert its deep-space
// flag before propagating it, WITHOUT re-running select_ephemeris (which is
// not idempotent). ClearFlag also drops the *_INITIALIZED bits so the
// propagator re-derives its constants for this object on the next call.
static void prep_object(object_t *o)
{
    o->pred.satellite_ephem.tle = o->tle_ready;
    ClearFlag(ALL_FLAGS);
    if (o->deep_space) SetFlag(DEEP_SPACE_EPHEM_FLAG);
}

// ECI position + velocity (km, km/s) at a Julian date, straight from the
// propagator. Calls prep_object first so it is safe to interleave with the
// other object in the same process.
static void sat_state(object_t *o, double jul, double r[3], double v[3])
{
    prep_object(o);
    double tsince = (jul - o->epoch_jul) * 1440.0;
    vector_t pos = {0}, vel = {0};
    if (o->deep_space) SDP4(tsince, &o->pred.satellite_ephem.tle, &pos, &vel);
    else               SGP4(tsince, &o->pred.satellite_ephem.tle, &pos, &vel);
    Convert_Sat_State(&pos, &vel);
    r[0] = pos.x; r[1] = pos.y; r[2] = pos.z;
    v[0] = vel.x; v[1] = vel.y; v[2] = vel.z;
}

// 3-D separation (km) between the two objects at a Julian date.
static double separation_km(object_t *a, object_t *b, double jul)
{
    double ra[3], va[3], rb[3], vb[3];
    sat_state(a, jul, ra, va);
    sat_state(b, jul, rb, vb);
    double dx = rb[0] - ra[0], dy = rb[1] - ra[1], dz = rb[2] - ra[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Golden-section minimisation of separation_km over a bracket [lo, hi] (days)
// that contains a single minimum. Returns the time (Julian date) of closest
// approach. The relative-distance curve is smooth and unimodal across one
// encounter, so this converges to the true TCA well below the coarse step.
static double refine_tca(object_t *a, object_t *b, double lo, double hi)
{
    const double GR = 0.6180339887498949;   // 1/phi
    double c = hi - GR * (hi - lo);
    double d = lo + GR * (hi - lo);
    double fc = separation_km(a, b, c);
    double fd = separation_km(a, b, d);
    // Tolerance in Julian days. At a ~15 km/s closing speed every 0.01 s of TCA
    // error is ~150 m of relative travel along the velocity, which tilts the
    // miss vector off perpendicular and corrupts the radial/along/cross split
    // (and, for a close miss, the miss magnitude). Refine to ~0.1 ms (~1.5 m),
    // just above the floor a double-precision Julian date imposes: a JD near
    // 2.46e6 resolves to ~5e-10 day (~0.7 m at this speed), and sat_state's
    // tsince = (jul - epoch) subtraction inherits that granularity, so chasing
    // a tighter tolerance would only be tracking floating-point noise.
    const double tol = 1.0e-4 / 86400.0;
    int iter = 0;
    while ((hi - lo) > tol && iter++ < 200) {
        if (fc < fd) {
            hi = d; d = c; fd = fc;
            c = hi - GR * (hi - lo);
            fc = separation_km(a, b, c);
        } else {
            lo = c; c = d; fc = fd;
            d = lo + GR * (hi - lo);
            fd = separation_km(a, b, d);
        }
    }
    return 0.5 * (lo + hi);
}

// ---- per-event report ------------------------------------------------------

static void report_event(const args_t *cfg, object_t *a, object_t *b,
                         double jul_tca, double jul_now, int index)
{
    double ra[3], va[3], rb[3], vb[3];
    sat_state(a, jul_tca, ra, va);
    sat_state(b, jul_tca, rb, vb);

    double radial, along, cross, range;
    conj_rtn_components(ra, va, rb, &radial, &along, &cross, &range);

    double rrel[3] = { rb[0] - ra[0], rb[1] - ra[1], rb[2] - ra[2] };
    double vrel[3] = { vb[0] - va[0], vb[1] - va[1], vb[2] - va[2] };
    double rel_speed = sqrt(vrel[0] * vrel[0] + vrel[1] * vrel[1] + vrel[2] * vrel[2]);

    // Combined ECI covariance (km^2): each object's RTN sigmas rotated into
    // ECI at TCA, then summed.
    double cov1[9], cov2[9], cov[9];
    conj_cov_rtn_to_eci(ra, va, cfg->sig1_r / 1000.0, cfg->sig1_a / 1000.0,
                        cfg->sig1_c / 1000.0, cov1);
    conj_cov_rtn_to_eci(rb, vb, cfg->sig2_r / 1000.0, cfg->sig2_a / 1000.0,
                        cfg->sig2_c / 1000.0, cov2);
    for (int i = 0; i < 9; ++i) cov[i] = cov1[i] + cov2[i];
    double pc = conj_foster_pc(rrel, vrel, cov, cfg->hbr_m / 1000.0);

    char utc[40], loc[40], until[40], miss[32], cr[32], ca[32], cc[32];
    fmt_utc_ms(jul_tca, utc, sizeof utc);
    fmt_local_ms(jul_tca, loc, sizeof loc);
    format_duration_compact((jul_tca - jul_now) * 86400.0, until, sizeof until);
    fmt_mag(range, miss, sizeof miss);
    fmt_len(radial, cr, sizeof cr);
    fmt_len(along,  ca, sizeof ca);
    fmt_len(cross,  cc, sizeof cc);

    if (index > 0) printf("\n--- close approach #%d ---\n", index);
    printf("  miss distance      %s\n", miss);
    printf("    radial (height)  %s\n", cr);
    printf("    along-track      %s\n", ca);
    printf("    cross-track      %s\n", cc);
    printf("  relative speed     %.3f km/s\n", rel_speed);
    printf("  time of closest approach\n");
    printf("    UTC              %s\n", utc);
    printf("    local            %s\n", loc);
    printf("    from now         %s\n", until);
    printf("  collision probability (Foster 1992)\n");
    printf("    Pc               %.3e\n", pc);
    printf("    hard-body radius %.1f m (combined)\n", cfg->hbr_m);
    printf("    assumed 1-sigma  %s: R %.0f m  I %.0f m  C %.0f m\n",
           a->name, cfg->sig1_r, cfg->sig1_a, cfg->sig1_c);
    printf("    assumed 1-sigma  %s: R %.0f m  I %.0f m  C %.0f m\n",
           b->name, cfg->sig2_r, cfg->sig2_a, cfg->sig2_c);
}

// ---- gnuplot 3-D encounter view --------------------------------------------

// Strip a trailing ".png" from the user's --plot-out so we can derive sibling
// .gp / .dat names from the same base.
static void plot_base_name(const char *out, char *base, size_t n)
{
    if (!out || !out[0]) { snprintf(base, n, "conjunction"); return; }
    snprintf(base, n, "%s", out);
    size_t len = strlen(base);
    if (len > 4 && strcmp(base + len - 4, ".png") == 0) base[len - 4] = '\0';
}

// Write a self-contained gnuplot script + data file for a 3-D perspective view
// of the encounter, then run gnuplot to render a PNG. Coordinates are ECI in
// km but recentred on the encounter midpoint, and the window is kept short
// (default +-90 s around TCA) so the two tracks are nearly straight, cross
// near the origin, and the miss distance is a visible gap rather than a
// sub-pixel intersection. Both trajectories are drawn (primary solid, secondary
// dashed) with a filled arrow along each velocity for the direction of motion,
// the two closest-approach points marked, and a dashed segment + label for the
// miss vector. Returns 0 if gnuplot rendered the PNG, -1 otherwise (the script
// and data are still left on disk so the operator can render them by hand).
static int write_conjunction_plot(const args_t *cfg, object_t *a, object_t *b,
                                  double jul_tca)
{
    char base[512], datp[600], gpp[600], pngp[600];
    plot_base_name(cfg->plot_out, base, sizeof base);
    snprintf(datp, sizeof datp, "%s.dat", base);
    snprintf(gpp,  sizeof gpp,  "%s.gp",  base);
    snprintf(pngp, sizeof pngp, "%s.png", base);

    double ra[3], va[3], rb[3], vb[3];
    sat_state(a, jul_tca, ra, va);
    sat_state(b, jul_tca, rb, vb);
    double mid[3] = { (ra[0] + rb[0]) / 2.0, (ra[1] + rb[1]) / 2.0, (ra[2] + rb[2]) / 2.0 };
    double dx = rb[0] - ra[0], dy = rb[1] - ra[1], dz = rb[2] - ra[2];
    double miss = sqrt(dx * dx + dy * dy + dz * dz);
    double sa = sqrt(va[0] * va[0] + va[1] * va[1] + va[2] * va[2]);
    double sb = sqrt(vb[0] * vb[0] + vb[1] * vb[1] + vb[2] * vb[2]);
    double vrel = sqrt((vb[0] - va[0]) * (vb[0] - va[0])
                     + (vb[1] - va[1]) * (vb[1] - va[1])
                     + (vb[2] - va[2]) * (vb[2] - va[2]));

    double win = cfg->plot_window_sec;
    const int N = 401;

    FILE *df = fopen(datp, "w");
    if (!df) { fprintf(stderr, "conjunction: cannot write %s\n", datp); return -1; }
    fprintf(df, "# index 0: %s track  (X Y Z km, recentred on encounter midpoint)\n", a->name);
    for (int i = 0; i < N; ++i) {
        double s = -win + 2.0 * win * (double) i / (double) (N - 1);
        double r[3], v[3];
        sat_state(a, jul_tca + s / 86400.0, r, v);
        fprintf(df, "%.6f %.6f %.6f\n", r[0] - mid[0], r[1] - mid[1], r[2] - mid[2]);
    }
    fprintf(df, "\n\n# index 1: %s track\n", b->name);
    for (int i = 0; i < N; ++i) {
        double s = -win + 2.0 * win * (double) i / (double) (N - 1);
        double r[3], v[3];
        sat_state(b, jul_tca + s / 86400.0, r, v);
        fprintf(df, "%.6f %.6f %.6f\n", r[0] - mid[0], r[1] - mid[1], r[2] - mid[2]);
    }
    fprintf(df, "\n\n# index 2: closest-approach points\n");
    fprintf(df, "%.6f %.6f %.6f\n", ra[0] - mid[0], ra[1] - mid[1], ra[2] - mid[2]);
    fprintf(df, "%.6f %.6f %.6f\n", rb[0] - mid[0], rb[1] - mid[1], rb[2] - mid[2]);
    fclose(df);

    // Endpoints (recentred) for the on-plot arrows and labels.
    double pa[3] = { ra[0] - mid[0], ra[1] - mid[1], ra[2] - mid[2] };
    double pb[3] = { rb[0] - mid[0], rb[1] - mid[1], rb[2] - mid[2] };
    // Direction-of-motion arrows ~30% of the half-arc (win * speed) long.
    double la_km = 0.30 * win * sa, lb_km = 0.30 * win * sb;
    double a2[3] = { pa[0] + (sa > 0 ? va[0] / sa : 0) * la_km,
                     pa[1] + (sa > 0 ? va[1] / sa : 0) * la_km,
                     pa[2] + (sa > 0 ? va[2] / sa : 0) * la_km };
    double b2[3] = { pb[0] + (sb > 0 ? vb[0] / sb : 0) * lb_km,
                     pb[1] + (sb > 0 ? vb[1] / sb : 0) * lb_km,
                     pb[2] + (sb > 0 ? vb[2] / sb : 0) * lb_km };
    double mm[3] = { (pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0, (pa[2] + pb[2]) / 2.0 };

    char missstr[32], utc[40];
    fmt_mag(miss, missstr, sizeof missstr);
    fmt_utc(jul_tca, utc, sizeof utc);

    FILE *gf = fopen(gpp, "w");
    if (!gf) { fprintf(stderr, "conjunction: cannot write %s\n", gpp); return -1; }
    fprintf(gf, "# gnuplot script generated by conjunction; render with: gnuplot %s\n", gpp);
    fprintf(gf, "set terminal pngcairo size 1100,850 enhanced font 'Helvetica,11'\n");
    fprintf(gf, "set output '%s'\n", pngp);
    fprintf(gf, "set title \"Conjunction: %s vs %s\\n"
                "miss %s at %s  (relative speed %.2f km/s)\"\n",
            a->name, b->name, missstr, utc, vrel);
    fprintf(gf, "set xlabel 'X (km, ECI, recentred on encounter)'\n");
    fprintf(gf, "set ylabel 'Y (km)'\n");
    fprintf(gf, "set zlabel 'Z (km)'\n");
    fprintf(gf, "set view 62, 28\n");
    fprintf(gf, "set view equal xyz\n");
    fprintf(gf, "set ticslevel 0\n");
    fprintf(gf, "set grid\n");
    // Legend outside the plot box so it never collides with the title or the
    // tracks (which can sweep through any corner depending on the geometry).
    fprintf(gf, "set key outside right center box\n");
    // Direction-of-motion arrows.
    fprintf(gf, "set arrow 1 from %.4f,%.4f,%.4f to %.4f,%.4f,%.4f head filled size screen 0.025,20 lw 2 lc rgb 'dark-blue'\n",
            pa[0], pa[1], pa[2], a2[0], a2[1], a2[2]);
    fprintf(gf, "set arrow 2 from %.4f,%.4f,%.4f to %.4f,%.4f,%.4f head filled size screen 0.025,20 lw 2 lc rgb 'dark-red'\n",
            pb[0], pb[1], pb[2], b2[0], b2[1], b2[2]);
    // Miss vector (dashed, no head) with a label at its midpoint.
    fprintf(gf, "set arrow 3 from %.4f,%.4f,%.4f to %.4f,%.4f,%.4f nohead dt 3 lw 1 lc rgb 'black'\n",
            pa[0], pa[1], pa[2], pb[0], pb[1], pb[2]);
    fprintf(gf, "set label 1 \"miss %s\" at %.4f,%.4f,%.4f center font ',9'\n",
            missstr, mm[0], mm[1], mm[2]);
    fprintf(gf, "splot '%s' index 0 with lines lw 2 lc rgb 'dark-blue' title '%s', \\\n",
            datp, a->name);
    fprintf(gf, "      '%s' index 1 with lines lw 2 dt 2 lc rgb 'dark-red' title '%s', \\\n",
            datp, b->name);
    fprintf(gf, "      '%s' index 2 with points pt 7 ps 1.4 lc rgb 'black' title 'closest approach'\n",
            datp);
    fclose(gf);

    char cmd[1400];
    snprintf(cmd, sizeof cmd, "gnuplot '%s'", gpp);
    int rc = system(cmd);
    if (rc == 0) {
        printf("  3-D plot           %s\n", pngp);
        printf("    (gnuplot script %s, data %s)\n", gpp, datp);
        return 0;
    }
    printf("  3-D plot: wrote %s + %s, but gnuplot did not run (rc=%d).\n",
           gpp, datp, rc);
    printf("            render it with:  gnuplot %s\n", gpp);
    return -1;
}

// ---- argument parsing ------------------------------------------------------

// Parse a "R,I,C" triple (metres) into three doubles. Returns 1 on success.
static int parse_rtn(const char *s, double *r, double *i, double *c)
{
    return sscanf(s, "%lf,%lf,%lf", r, i, c) == 3;
}

static int parse_args(args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // Positionals: <tle-file> <name1> <name2>, declared first so they
        // list above the options in help.
        if ((arg[0] != '-' || strcmp(arg, "-") == 0) || help) {
            if (help) {
                parse_help_line(OPTW, "<tle-file>", "TLE file holding the object(s)");
                parse_help_line(OPTW, "<name1>", "primary object name prefix (case-sensitive)");
                parse_help_line(OPTW, "<name2>", "secondary object name prefix");
                matched = 1;
            } else {
                switch (a->n_positional) {
                    case 0: a->file1 = arg; break;
                    case 1: a->name1 = arg; break;
                    case 2: a->name2 = arg; break;
                    default:
                        fprintf(stderr, "conjunction: unexpected extra argument '%s'\n", arg);
                        return PARSE_ERROR;
                }
                a->n_positional++;
                matched = 1;
            }
        }
        if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help, -h", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "--help-full") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help-full", "help plus worked examples");
            else { parse_args(a, argc, argv, HELP_FULL); return PARSE_HELP; }
            matched = 1;
        }
        if (strncmp(arg, "--tle2=", 7) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tle2=<path>", "read <name2> from a second TLE file");
            else a->file2 = arg + 7;
            matched = 1;
        }
        if (strncmp(arg, "--days=", 7) == 0 || help) {
            if (help) parse_help_line(OPTW, "--days=<N>", "forward search window in days (default 7)");
            else a->days = atof(arg + 7);
            matched = 1;
        }
        if (strncmp(arg, "--step=", 7) == 0 || help) {
            if (help) parse_help_line(OPTW, "--step=<sec>", "coarse scan step in seconds (default 10)");
            else a->step_sec = atof(arg + 7);
            matched = 1;
        }
        if (strncmp(arg, "--threshold-km=", 15) == 0 || help) {
            if (help) parse_help_line(OPTW, "--threshold-km=<km>", "conjunction distance gate (default 100)");
            else a->threshold_km = atof(arg + 15);
            matched = 1;
        }
        if (strcmp(arg, "--all") == 0 || help) {
            if (help) parse_help_line(OPTW, "--all", "report every approach below the threshold, not just the first");
            else a->all = 1;
            matched = 1;
        }
        if (strncmp(arg, "--hbr-m=", 8) == 0 || help) {
            if (help) parse_help_line(OPTW, "--hbr-m=<m>", "combined hard-body radius, metres (default 20)");
            else a->hbr_m = atof(arg + 8);
            matched = 1;
        }
        if (strncmp(arg, "--sigma1-rtn=", 13) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sigma1-rtn=R,I,C", "object 1 1-sigma position, RTN metres");
            else if (!parse_rtn(arg + 13, &a->sig1_r, &a->sig1_a, &a->sig1_c)) {
                fprintf(stderr, "conjunction: --sigma1-rtn wants R,I,C (e.g. 200,1000,200)\n");
                return PARSE_ERROR;
            }
            matched = 1;
        }
        if (strncmp(arg, "--sigma2-rtn=", 13) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sigma2-rtn=R,I,C", "object 2 1-sigma position, RTN metres");
            else if (!parse_rtn(arg + 13, &a->sig2_r, &a->sig2_a, &a->sig2_c)) {
                fprintf(stderr, "conjunction: --sigma2-rtn wants R,I,C (e.g. 200,1000,200)\n");
                return PARSE_ERROR;
            }
            matched = 1;
        }
        if (strncmp(arg, "--sigma-m=", 10) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sigma-m=<m>", "shorthand: isotropic 1-sigma (metres) for BOTH objects");
            else {
                double s = atof(arg + 10);
                a->sig1_r = a->sig1_a = a->sig1_c = s;
                a->sig2_r = a->sig2_a = a->sig2_c = s;
            }
            matched = 1;
        }
        if (strcmp(arg, "--plot") == 0 || help) {
            if (help) parse_help_line(OPTW, "--plot", "render a gnuplot 3-D view of the encounter (PNG)");
            else a->plot = 1;
            matched = 1;
        }
        if (strncmp(arg, "--plot-out=", 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--plot-out=<path>", "plot output base/PNG path (implies --plot; default conjunction.png)");
            else { a->plot_out = arg + 11; a->plot = 1; }
            matched = 1;
        }
        if (strncmp(arg, "--plot-window-sec=", 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--plot-window-sec=<s>", "half-window around TCA shown in the plot (default 90)");
            else a->plot_window_sec = atof(arg + 18);
            matched = 1;
        }
        if (strcmp(arg, "-V") == 0 || strcmp(arg, "--version") == 0 || help) {
            if (help) parse_help_line(OPTW, "-V, --version", "print the build commit and exit");
            // -V/--version is handled by sso_version_handle in main; this
            // block exists only so the option appears in --help.
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "conjunction: unable to parse '%s'\n", arg);
            return PARSE_ERROR;
        }
    }
    if (help >= HELP_FULL) {
        printf("\nExamples:\n");
        printf("  # Both objects in one file, FrontierSat as the primary:\n");
        printf("  conjunction unit_tests/fixtures/conjunction.tle FrontierSat SPACEMOBILE-004\n");
        printf("  # Search a fortnight at a finer step, tighter conjunction gate:\n");
        printf("  conjunction unit_tests/fixtures/conjunction.tle FrontierSat SPACEMOBILE-004 --days=14 --step=5 --threshold-km=25\n");
        printf("  # Secondary from a different file, with a supplied covariance:\n");
        printf("  conjunction ours.tle FrontierSat --tle2=other.tle DEBRIS-123 --sigma2-rtn=500,3000,500\n");
    }
    return PARSE_OK;
}

static int load_object(object_t *o, const char *file, const char *name)
{
    memset(o, 0, sizeof *o);
    o->file = file;
    o->name = name;
    snprintf(o->namebuf, sizeof o->namebuf, "%s", name);
    o->pred.tles_filename = (char *) file;
    o->pred.satellite_ephem.name = o->namebuf;
    if (load_tle(&o->pred) != 0) return -1;       // load_tle prints the reason
    setup_object(o);
    return 0;
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "conjunction")) return 0;

    args_t cfg = {0};
    cfg.days = 7.0;
    cfg.step_sec = 10.0;
    cfg.threshold_km = 100.0;
    cfg.hbr_m = DEF_HBR_M;
    cfg.plot_window_sec = 90.0;
    cfg.sig1_r = cfg.sig2_r = DEF_SIG_R;
    cfg.sig1_a = cfg.sig2_a = DEF_SIG_A;
    cfg.sig1_c = cfg.sig2_c = DEF_SIG_C;

    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }

    if (!cfg.file1 || !cfg.name1 || !cfg.name2) {
        fprintf(stderr, "Usage: conjunction <tle-file> <name1> <name2> [options]\n"
                        "       (try --help)\n");
        return 1;
    }
    if (!(cfg.days > 0.0))      { fprintf(stderr, "conjunction: --days must be > 0\n"); return 1; }
    if (!(cfg.step_sec > 0.0))  { fprintf(stderr, "conjunction: --step must be > 0\n"); return 1; }
    if (!(cfg.threshold_km > 0.0)) { fprintf(stderr, "conjunction: --threshold-km must be > 0\n"); return 1; }
    if (!(cfg.plot_window_sec > 0.0)) { fprintf(stderr, "conjunction: --plot-window-sec must be > 0\n"); return 1; }

    object_t a, b;
    if (load_object(&a, cfg.file1, cfg.name1) != 0) return 1;
    if (load_object(&b, cfg.file2 ? cfg.file2 : cfg.file1, cfg.name2) != 0) return 1;

    double jul_now = now_jul_utc();
    double jul_end = jul_now + cfg.days;

    // Header: who, how stale, and the search setup.
    char now_s[40], e1[40], e2[40], age1[32], age2[32];
    fmt_utc(jul_now, now_s, sizeof now_s);
    fmt_utc(a.epoch_jul, e1, sizeof e1);
    fmt_utc(b.epoch_jul, e2, sizeof e2);
    format_age_compact((jul_now - a.epoch_jul) * 86400.0, age1, sizeof age1);
    format_age_compact((jul_now - b.epoch_jul) * 86400.0, age2, sizeof age2);

    printf("Conjunction search\n");
    printf("  now                %s\n", now_s);
    printf("  primary            %s  (NORAD %d)\n", a.tle_ready.sat_name, a.tle_ready.catnr);
    printf("    TLE epoch        %s   (age %s)\n", e1, age1);
    printf("  secondary          %s  (NORAD %d)\n", b.tle_ready.sat_name, b.tle_ready.catnr);
    printf("    TLE epoch        %s   (age %s)\n", e2, age2);
    printf("  window             %.3g days   step %.3g s   threshold %.3g km\n",
           cfg.days, cfg.step_sec, cfg.threshold_km);
    if (a.tle_ready.catnr == b.tle_ready.catnr)
        printf("  NOTE: both objects share NORAD ID %d -- comparing an object with itself?\n",
               a.tle_ready.catnr);
    printf("\n");

    // Coarse scan tracking the previous two samples to spot a local minimum.
    // Every coarse local minimum is refined to the true time of closest
    // approach with golden-section search; the first refined minimum below the
    // threshold is reported (continuing only when --all is set). The smallest
    // refined minimum over the whole window is kept too, so the "no
    // conjunction" path can report the real closest approach -- the coarse
    // sample nearest a fast pass can sit several km off the true minimum, so
    // we must report the refined value, not the nearest sample.
    double step_jul = cfg.step_sec / 86400.0;
    double t_prev2 = 0.0, t_prev1 = 0.0;
    double d_prev2 = 0.0, d_prev1 = 0.0;
    int    have2 = 0, have1 = 0;
    double best_min = 1e300, best_min_t = jul_now;   // smallest refined minimum
    int    have_best = 0;
    double coarse_min = 1e300, coarse_min_t = jul_now;  // safety net if no local min
    int    found = 0, event_index = 0;

    for (double t = jul_now; t <= jul_end + 0.5 * step_jul; t += step_jul) {
        double d = separation_km(&a, &b, t);
        if (d < coarse_min) { coarse_min = d; coarse_min_t = t; }

        // Interior local minimum: middle sample is the lowest of three.
        if (have2 && have1 && d_prev1 <= d_prev2 && d_prev1 <= d) {
            double tca = refine_tca(&a, &b, t_prev2, t);
            double d_tca = separation_km(&a, &b, tca);
            if (d_tca < best_min) { best_min = d_tca; best_min_t = tca; have_best = 1; }
            if (d_tca <= cfg.threshold_km) {
                if (!found)
                    printf("CONJUNCTION FOUND (within %.3g km)\n", cfg.threshold_km);
                found = 1;
                report_event(&cfg, &a, &b, tca, jul_now, cfg.all ? ++event_index : 0);
                if (!cfg.all) break;
            }
        }

        t_prev2 = t_prev1; d_prev2 = d_prev1; have2 = have1;
        t_prev1 = t;       d_prev1 = d;       have1 = 1;
    }

    // The time we plot is the closest refined approach in the window (which,
    // in default first-match mode, is exactly the conjunction we reported);
    // fall back to the coarse minimum only if nothing was bracketed.
    double plot_tca = have_best ? best_min_t : coarse_min_t;

    if (!found) {
        // Prefer the refined global minimum; fall back to the coarse minimum
        // only if no interior local minimum was bracketed (e.g. the objects
        // are still approaching at the end of the window).
        double cl   = have_best ? best_min   : coarse_min;
        double cl_t = have_best ? best_min_t : coarse_min_t;
        char gm[32], gutc[40], gloc[40], guntil[40];
        fmt_mag(cl, gm, sizeof gm);
        fmt_utc_ms(cl_t, gutc, sizeof gutc);
        fmt_local_ms(cl_t, gloc, sizeof gloc);
        format_duration_compact((cl_t - jul_now) * 86400.0, guntil, sizeof guntil);
        printf("No conjunction within %.3g km over the next %.3g days.\n",
               cfg.threshold_km, cfg.days);
        printf("  closest approach   %s\n", gm);
        printf("    UTC              %s\n", gutc);
        printf("    local            %s\n", gloc);
        printf("    from now         %s\n", guntil);
    }

    if (cfg.plot) {
        printf("\n");
        write_conjunction_plot(&cfg, &a, &b, plot_tca);
    }

    return found ? 0 : 2;
}
