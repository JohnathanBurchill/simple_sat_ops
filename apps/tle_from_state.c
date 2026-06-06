/*

    Simple Satellite Operations  tle_from_state.c

    Build a NORAD two-line element set from a single position/velocity
    state vector, the kind a satellite's GNSS receiver reports. The
    intended use is FrontierSat: downlink a fix, drop it in a file, and
    get a TLE that next_in_queue and simple_sat_ops can track with.

    Input is a small text file (path on the command line, or stdin):

        2026-06-05T12:34:56Z              epoch, UTC
        -1234567.0, 2345678.0, 6789012.0  position x,y,z      (metres)
        5.0, 5.0, 5.0                     position 1-sigma    (metres)
        -1234.5, 6543.2, 1234.5           velocity vx,vy,vz   (m/s)
        0.05, 0.05, 0.05                  velocity 1-sigma    (m/s)

    Blank lines and lines beginning with '#' are ignored; the three
    components of a vector may be separated by commas or spaces. The
    position and velocity are Earth-fixed (ECEF / ITRF, the GNSS
    standard) unless --frame=eci is given.

    The conversion is in three steps:

      1. ECEF -> inertial. Rotate the vectors about the polar axis by the
         Greenwich sidereal angle (ThetaG_JD) and add the Earth-rotation
         term to the velocity. This is the same single-rotation model the
         bundled sgp4sdp4 uses for ground tracks (no polar motion or
         nutation), so the result is consistent with the rest of the
         suite even though it is not rigorous TEME.

      2. A two-body solve (r,v -> classical elements) gives a starting
         guess for the mean elements.

      3. A differential correction iterates the mean elements until this
         build's own SGP4, evaluated at the epoch, reproduces the
         measured state. That makes the TLE self-consistent with the
         propagator next_in_queue / simple_sat_ops actually run. If it
         cannot converge the tool falls back to the two-body elements and
         says so.

    A drag term cannot be observed from one state vector, so B* and the
    mean-motion derivatives are written as zero. The position/velocity
    sigmas are used only as a quality gate: if the fitted orbit cannot
    reproduce the measured state to within them the tool warns (or, with
    --strict-sigma, refuses to emit a TLE).

    The TLE goes to stdout; everything else (the fit log, the orbit
    summary, the gate result) goes to stderr, so stdout can be redirected
    straight into a .tle file.

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

// SGP4SDP4_CONSTANTS exposes the library's own physical constants
// (xkmper, twopi, de2ra, mfactor, ...) so our unit conversions match the
// propagator exactly rather than re-deriving slightly different numbers.
#define SGP4SDP4_CONSTANTS
#include <sgp4sdp4.h>

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Earth gravitational parameter for the two-body starting guess. The
// final elements come from the SGP4 fit, which uses the library's own
// gravity model, so this only has to be close enough to seed Newton.
#define GM_KM3_S2            398600.4418
#define EARTH_RADIUS_MEAN_KM 6371.0       // for the apogee/perigee readout

// Indices into the working element vector, all in the units SGP4 reads
// after select_ephemeris would have run (radians, radians/minute).
enum { EL_INCL, EL_RAAN, EL_ECC, EL_ARGP, EL_MA, EL_N, EL_COUNT };

// ---- small vector helpers (plain double[3], not the library vector) --------

static double v3_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double v3_norm(const double a[3])
{
    return sqrt(v3_dot(a, a));
}

static void v3_cross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// acos that tolerates arguments a hair outside [-1,1] from rounding.
static double safe_acos(double x)
{
    if (x > 1.0) x = 1.0;
    if (x < -1.0) x = -1.0;
    return acos(x);
}

static double wrap_2pi(double a)
{
    a = fmod(a, twopi);
    if (a < 0.0) a += twopi;
    return a;
}

// ---- input file -------------------------------------------------------------

// Next non-blank, non-comment line, trailing whitespace/CR/LF stripped.
// Returns 0 at end of input.
static int next_line(FILE *fp, char *buf, size_t cap)
{
    while (fgets(buf, (int) cap, fp)) {
        size_t n = strlen(buf);
        while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r'
                     || buf[n - 1] == ' ' || buf[n - 1] == '\t'))
            buf[--n] = '\0';
        // Skip leading blanks to test for a comment / empty line.
        const char *p = buf;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '#') continue;
        return 1;
    }
    return 0;
}

// Parse three numbers from a line; commas and/or whitespace separate them.
static int parse_triplet(const char *line, double out[3])
{
    char tmp[256];
    snprintf(tmp, sizeof tmp, "%s", line);
    for (char *c = tmp; *c; ++c)
        if (*c == ',') *c = ' ';
    return sscanf(tmp, "%lf %lf %lf", &out[0], &out[1], &out[2]) == 3;
}

// Parse "YYYY-MM-DDThh:mm:ss[.fff][Z]" (a space may replace the 'T', the
// 'Z' is optional) into broken-down UTC. Returns 0 on success.
static int parse_epoch(const char *str, int *year, int *mon, int *day,
                       int *hh, int *mm, double *ss)
{
    char tmp[64];
    snprintf(tmp, sizeof tmp, "%.*s", (int) (sizeof tmp - 1), str);
    for (char *c = tmp; *c; ++c) {
        if (*c == 'T' || *c == 't') *c = ' ';
        if (*c == 'Z' || *c == 'z') *c = '\0';
    }
    int n = sscanf(tmp, "%d-%d-%d %d:%d:%lf", year, mon, day, hh, mm, ss);
    if (n < 3) return -1;
    if (n < 6) { *hh = (n < 4) ? 0 : *hh; *mm = (n < 5) ? 0 : *mm; *ss = (n < 6) ? 0.0 : *ss; }
    if (*mon < 1 || *mon > 12 || *day < 1 || *day > 31) return -1;
    if (*hh < 0 || *hh > 23 || *mm < 0 || *mm > 59 || *ss < 0.0 || *ss >= 61.0) return -1;
    return 0;
}

// ---- frame conversion -------------------------------------------------------

// ECEF -> inertial at sidereal angle theta (radians). Position is a pure
// rotation about z; velocity also picks up the frame's rotation rate
// (mfactor = 7.292115e-5 rad/s), matching Calculate_User_PosVel's
// stationary-observer term in reverse.
static void ecef_to_eci(double theta, const double r_ef[3], const double v_ef[3],
                        double r_ci[3], double v_ci[3])
{
    double cs = cos(theta), sn = sin(theta);
    r_ci[0] = cs * r_ef[0] - sn * r_ef[1];
    r_ci[1] = sn * r_ef[0] + cs * r_ef[1];
    r_ci[2] = r_ef[2];
    v_ci[0] = cs * v_ef[0] - sn * v_ef[1] - mfactor * r_ci[1];
    v_ci[1] = sn * v_ef[0] + cs * v_ef[1] + mfactor * r_ci[0];
    v_ci[2] = v_ef[2];
}

// ---- two-body solve (starting guess) ---------------------------------------

// r,v in km and km/s -> mean elements in SGP4-internal units. argp/raan
// fall back to zero when the orbit is too circular/equatorial to define
// them; the fit refines from there.
static void rv_to_elements(const double r[3], const double v[3], double el[EL_COUNT])
{
    double rmag = v3_norm(r), vmag = v3_norm(v);
    double rdotv = v3_dot(r, v);

    double h[3];
    v3_cross(r, v, h);
    double hmag = v3_norm(h);

    double node[3] = { -h[1], h[0], 0.0 };
    double nmag = v3_norm(node);

    // Eccentricity vector e = ((v^2 - mu/r) r - (r.v) v) / mu.
    double evec[3];
    double t1 = vmag * vmag - GM_KM3_S2 / rmag;
    for (int i = 0; i < 3; ++i)
        evec[i] = (t1 * r[i] - rdotv * v[i]) / GM_KM3_S2;
    double e = v3_norm(evec);

    double energy = vmag * vmag / 2.0 - GM_KM3_S2 / rmag;
    double a = -GM_KM3_S2 / (2.0 * energy);   // km

    double incl = safe_acos(h[2] / hmag);

    double raan = 0.0;
    if (nmag > 1e-9) {
        raan = safe_acos(node[0] / nmag);
        if (node[1] < 0.0) raan = twopi - raan;
    }

    double argp = 0.0;
    if (nmag > 1e-9 && e > 1e-9) {
        argp = safe_acos(v3_dot(node, evec) / (nmag * e));
        if (evec[2] < 0.0) argp = twopi - argp;
    }

    // True anomaly, then eccentric and mean anomaly.
    double nu;
    if (e > 1e-9) {
        nu = safe_acos(v3_dot(evec, r) / (e * rmag));
        if (rdotv < 0.0) nu = twopi - nu;
    } else if (nmag > 1e-9) {
        // Near-circular: measure from the ascending node (arg. of latitude).
        nu = safe_acos(v3_dot(node, r) / (nmag * rmag));
        if (r[2] < 0.0) nu = twopi - nu;
    } else {
        nu = atan2(r[1], r[0]);
    }
    double E = atan2(sqrt(1.0 - e * e) * sin(nu), e + cos(nu));
    double M = wrap_2pi(E - e * sin(E));

    double n_rad_s = sqrt(GM_KM3_S2 / (a * a * a));

    el[EL_INCL] = incl;
    el[EL_RAAN] = raan;
    el[EL_ECC]  = e;
    el[EL_ARGP] = argp;
    el[EL_MA]   = M;
    el[EL_N]    = n_rad_s * 60.0;        // rad/minute
}

// ---- SGP4 evaluation at epoch ----------------------------------------------

// Run this build's SGP4 for the given mean elements (internal units) at
// tsince = 0 and return the inertial state in km and km/s. Clearing all
// flags first forces SGP4 to re-initialise from these elements (its setup
// is cached behind SGP4_INITIALIZED_FLAG). Returns -1 for a deep-space
// period, which this tool does not handle.
static int sgp4_state(const double el[EL_COUNT], double r[3], double v[3])
{
    if (el[EL_N] <= 0.0) return -1;
    if (twopi / el[EL_N] >= 225.0) return -1;   // SDP4 territory

    tle_t t;
    memset(&t, 0, sizeof t);
    t.xincl  = el[EL_INCL];
    t.xnodeo = el[EL_RAAN];
    t.eo     = el[EL_ECC] < 1e-6 ? 1e-6 : el[EL_ECC];   // avoid /eo in init
    t.omegao = el[EL_ARGP];
    t.xmo    = el[EL_MA];
    t.xno    = el[EL_N];

    vector_t pos = {0}, vel = {0};
    ClearFlag(ALL_FLAGS);
    SGP4(0.0, &t, &pos, &vel);
    Convert_Sat_State(&pos, &vel);   // Earth radii -> km, er/min -> km/s

    r[0] = pos.x; r[1] = pos.y; r[2] = pos.z;
    v[0] = vel.x; v[1] = vel.y; v[2] = vel.z;
    return 0;
}

// Residual g = SGP4(el) - measured, stacked [dr(km); dv(km/s)].
static int residual(const double el[EL_COUNT], const double r_meas[3],
                    const double v_meas[3], double g[6])
{
    double r[3], v[3];
    if (sgp4_state(el, r, v) != 0) return -1;
    for (int i = 0; i < 3; ++i) {
        g[i]     = r[i] - r_meas[i];
        g[i + 3] = v[i] - v_meas[i];
    }
    return 0;
}

static double res_pos(const double g[6]) { return sqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]); }
static double res_vel(const double g[6]) { return sqrt(g[3]*g[3] + g[4]*g[4] + g[5]*g[5]); }

// Finite-difference step per element, sized for clean central differences.
static double fd_step(int j)
{
    switch (j) {
        case EL_ECC: return 1e-7;
        case EL_N:   return 1e-9;     // rad/min
        default:     return 1e-6;     // angles, rad
    }
}

// Solve A x = b (n=6) by Gaussian elimination with partial pivoting.
// Returns -1 if the matrix is singular. A and b are overwritten.
static int solve6(double A[6][6], double b[6], double x[6])
{
    const int n = 6;
    for (int col = 0; col < n; ++col) {
        int best = col;
        double bestv = fabs(A[col][col]);
        for (int r = col + 1; r < n; ++r)
            if (fabs(A[r][col]) > bestv) { bestv = fabs(A[r][col]); best = r; }
        if (bestv < 1e-18) return -1;
        if (best != col) {
            for (int c = 0; c < n; ++c) { double t = A[col][c]; A[col][c] = A[best][c]; A[best][c] = t; }
            double t = b[col]; b[col] = b[best]; b[best] = t;
        }
        for (int r = col + 1; r < n; ++r) {
            double m = A[r][col] / A[col][col];
            for (int c = col; c < n; ++c) A[r][c] -= m * A[col][c];
            b[r] -= m * b[col];
        }
    }
    for (int i = n - 1; i >= 0; --i) {
        double acc = b[i];
        for (int c = i + 1; c < n; ++c) acc -= A[i][c] * x[c];
        x[i] = acc / A[i][i];
    }
    return 0;
}

static void normalize_elements(double el[EL_COUNT])
{
    el[EL_RAAN] = wrap_2pi(el[EL_RAAN]);
    el[EL_ARGP] = wrap_2pi(el[EL_ARGP]);
    el[EL_MA]   = wrap_2pi(el[EL_MA]);
    if (el[EL_INCL] < 0.0) el[EL_INCL] = 0.0;
    if (el[EL_INCL] > M_PI) el[EL_INCL] = M_PI;
    if (el[EL_ECC] < 0.0) el[EL_ECC] = 0.0;
    if (el[EL_ECC] > 0.99) el[EL_ECC] = 0.99;
}

// Differential correction: drive SGP4(el, t=0) to the measured state by
// Newton steps with a numerical Jacobian and simple backtracking. On
// return el holds the best elements found; *out_pos/*out_vel are the
// final residual norms (km, km/s). Returns 1 if it converged, 0 if not,
// -1 if SGP4 could not be evaluated (deep space).
static int fit_elements(double el[EL_COUNT], const double r_meas[3],
                        const double v_meas[3], int max_iter, int verbose,
                        double *out_pos, double *out_vel)
{
    double g[6];
    if (residual(el, r_meas, v_meas, g) != 0) return -1;
    double best_el[EL_COUNT];
    memcpy(best_el, el, sizeof best_el);
    double best_pos = res_pos(g);

    const double tol_pos = 1e-4;   // km  (0.1 m -- far below any GNSS sigma)
    const double tol_vel = 1e-7;   // km/s

    int converged = (best_pos < tol_pos && res_vel(g) < tol_vel);

    for (int iter = 0; iter < max_iter && !converged; ++iter) {
        // Jacobian by central differences.
        double J[6][6];
        for (int j = 0; j < EL_COUNT; ++j) {
            double h = fd_step(j);
            double save = el[j];
            double gp[6], gm[6];
            el[j] = save + h;
            if (residual(el, r_meas, v_meas, gp) != 0) { el[j] = save; return -1; }
            el[j] = save - h;
            if (residual(el, r_meas, v_meas, gm) != 0) { el[j] = save; return -1; }
            el[j] = save;
            for (int i = 0; i < 6; ++i) J[i][j] = (gp[i] - gm[i]) / (2.0 * h);
        }

        double A[6][6], b[6], delta[6];
        memcpy(A, J, sizeof A);
        for (int i = 0; i < 6; ++i) b[i] = -g[i];
        if (solve6(A, b, delta) != 0) break;   // singular -> stop, keep best

        // Backtracking line search: accept the largest fraction of the
        // step that does not increase the position residual.
        double cur = res_pos(g);
        double lambda = 1.0;
        double trial[EL_COUNT], gtrial[6];
        int accepted = 0;
        for (int ls = 0; ls < 12; ++ls) {
            for (int j = 0; j < EL_COUNT; ++j) trial[j] = el[j] + lambda * delta[j];
            normalize_elements(trial);
            if (residual(trial, r_meas, v_meas, gtrial) == 0 && res_pos(gtrial) < cur) {
                accepted = 1;
                break;
            }
            lambda *= 0.5;
        }
        if (!accepted) break;   // cannot improve -> stop, keep best

        memcpy(el, trial, sizeof trial);
        memcpy(g, gtrial, sizeof g);

        double pr = res_pos(g);
        if (pr < best_pos) { best_pos = pr; memcpy(best_el, el, sizeof best_el); }
        if (verbose)
            fprintf(stderr, "  iter %2d: |dr| = %11.6f km  |dv| = %12.9f km/s\n",
                    iter + 1, pr, res_vel(g));
        converged = (pr < tol_pos && res_vel(g) < tol_vel);
    }

    memcpy(el, best_el, sizeof best_el);
    double gf[6];
    residual(el, r_meas, v_meas, gf);
    *out_pos = res_pos(gf);
    *out_vel = res_vel(gf);
    return converged ? 1 : 0;
}

// ---- TLE formatting ---------------------------------------------------------

// mod-10 checksum (digits as themselves, '-' as 1) over the first 68
// columns, exactly as Checksum_Good verifies it.
static int tle_checksum(const char *line68)
{
    int sum = 0;
    for (int i = 0; i < 68; ++i) {
        char c = line68[i];
        if (c >= '0' && c <= '9') sum += c - '0';
        else if (c == '-') sum += 1;
    }
    return sum % 10;
}

// Overwrite len characters at off (no terminator); caller keeps the line
// space-filled and NUL-terminated at 69.
static void place(char *line, int off, const char *src, int len)
{
    memcpy(line + off, src, (size_t) len);
}

// Build card 1 and card 2 into two 69-char strings. Field offsets mirror
// Convert_Satellite_Data so the library re-reads exactly what we write;
// the result is checked with Good_Elements by the caller.
static void format_tle(int catnr, char classd, const char *idesg, double epoch_field,
                       int elset, int revnum, const double el_deg[EL_COUNT],
                       double n_rev_day, char card1[70], char card2[70])
{
    // Wide enough to hold the worst case a "%f" double can produce (~320
    // chars for DBL_MAX); the field values here are all range-bounded, but
    // sizing for the maximum keeps gcc's -Wformat-truncation=2 quiet so the
    // ground build stays warning-free. Each field is then copied into the
    // card at its fixed column width by place().
    char tmp[340];

    memset(card1, ' ', 69); card1[69] = '\0';
    card1[0] = '1';
    snprintf(tmp, sizeof tmp, "%5d", catnr);   place(card1, 2, tmp, 5);
    card1[7] = classd;
    snprintf(tmp, sizeof tmp, "%-8.8s", idesg); place(card1, 9, tmp, 8);
    snprintf(tmp, sizeof tmp, "%014.8f", epoch_field); place(card1, 18, tmp, 14);
    place(card1, 33, " .00000000", 10);    // n-dot/2 = 0
    place(card1, 44, " 00000+0", 8);       // n-ddot/6 = 0
    place(card1, 53, " 00000+0", 8);       // B* = 0
    card1[62] = '0';                       // ephemeris type
    snprintf(tmp, sizeof tmp, "%4d", elset); place(card1, 64, tmp, 4);
    card1[68] = (char) ('0' + tle_checksum(card1));

    memset(card2, ' ', 69); card2[69] = '\0';
    card2[0] = '2';
    snprintf(tmp, sizeof tmp, "%5d", catnr); place(card2, 2, tmp, 5);
    snprintf(tmp, sizeof tmp, "%8.4f", el_deg[EL_INCL]); place(card2, 8, tmp, 8);
    snprintf(tmp, sizeof tmp, "%8.4f", el_deg[EL_RAAN]); place(card2, 17, tmp, 8);
    long ecc7 = lround(el_deg[EL_ECC] * 1e7);
    if (ecc7 < 0) ecc7 = 0;
    if (ecc7 > 9999999) ecc7 = 9999999;
    snprintf(tmp, sizeof tmp, "%07ld", ecc7); place(card2, 26, tmp, 7);
    snprintf(tmp, sizeof tmp, "%8.4f", el_deg[EL_ARGP]); place(card2, 34, tmp, 8);
    snprintf(tmp, sizeof tmp, "%8.4f", el_deg[EL_MA]);   place(card2, 43, tmp, 8);
    snprintf(tmp, sizeof tmp, "%11.8f", n_rev_day);      place(card2, 52, tmp, 11);
    snprintf(tmp, sizeof tmp, "%5d", revnum);            place(card2, 63, tmp, 5);
    card2[68] = (char) ('0' + tle_checksum(card2));
}

// ---- main -------------------------------------------------------------------

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options] [state-file]\n"
        "\n"
        "Build a NORAD two-line element set from one position/velocity state\n"
        "vector (e.g. a FrontierSat GNSS fix) and write it to stdout. With no\n"
        "file, the state is read from stdin.\n"
        "\n"
        "The file has five data lines (blank and '#' lines ignored; commas or\n"
        "spaces separate components):\n"
        "  1  epoch, UTC          YYYY-MM-DDThh:mm:ssZ\n"
        "  2  position x,y,z      metres\n"
        "  3  position 1-sigma    metres\n"
        "  4  velocity vx,vy,vz   metres/second\n"
        "  5  velocity 1-sigma    metres/second\n"
        "Position and velocity are Earth-fixed (ECEF/ITRF) unless --frame=eci.\n"
        "\n"
        "Options:\n"
        "  --name <s>        name line (default FrontierSat; --no-name omits it)\n"
        "  --no-name         emit a bare two-line set, no name line\n"
        "  --catnr <n>       catalogue number, 1..99999 (default 69015)\n"
        "  --intl <s>        international designator, e.g. 25001A (default blank)\n"
        "  --elset <n>       element set number, 0..9999 (default 1)\n"
        "  --revnum <n>      revolution number at epoch, 0..99999 (default 0)\n"
        "  --frame <f>       ecef (default) or eci: frame of the input vectors\n"
        "  --sigma-k <x>     gate multiplier: warn if residual > x*sigma (default 1)\n"
        "  --strict-sigma    exit nonzero and emit no TLE if the gate fails\n"
        "  --max-iter <n>    differential-correction iteration cap (default 60)\n"
        "  -v, --verbose     print each fit iteration to stderr\n"
        "  -h, --help        this text\n"
        "  -V, --version     print the build commit and exit\n"
        "\n"
        "B* drag and the mean-motion derivatives are zero (not observable from\n"
        "one state). The TLE goes to stdout; the fit log, orbit summary and\n"
        "quality-gate result go to stderr.\n",
        prog);
}

#include "sso_version.h"

// Match argv[*i] against a value-bearing option in either form: "--opt
// value" (advancing *i past the value) or "--opt=value". Returns the
// value (possibly "") when it matches, NULL otherwise.
static const char *match_opt(char **argv, int argc, int *i, const char *opt)
{
    const char *a = argv[*i];
    size_t len = strlen(opt);
    if (strncmp(a, opt, len) != 0) return NULL;
    if (a[len] == '=') return a + len + 1;
    if (a[len] != '\0') return NULL;            // e.g. --name vs --no-name
    return (*i + 1 < argc) ? argv[++(*i)] : "";
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "tle_from_state")) return 0;

    const char *path = NULL;
    const char *name = "FrontierSat";
    const char *intl = "";
    int emit_name = 1;
    int catnr = 69015, elset = 1, revnum = 0, max_iter = 60, verbose = 0;
    int frame_eci = 0, strict_sigma = 0;
    double sigma_k = 1.0;
    char classd = 'U';

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i], *val;
        if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(argv[0]); return EXIT_SUCCESS; }
        else if (!strcmp(a, "-v") || !strcmp(a, "--verbose")) verbose = 1;
        else if (!strcmp(a, "--no-name")) emit_name = 0;
        else if (!strcmp(a, "--strict-sigma")) strict_sigma = 1;
        else if ((val = match_opt(argv, argc, &i, "--name"))) name = val;
        else if ((val = match_opt(argv, argc, &i, "--intl"))) intl = val;
        else if ((val = match_opt(argv, argc, &i, "--catnr"))) catnr = atoi(val);
        else if ((val = match_opt(argv, argc, &i, "--elset"))) elset = atoi(val);
        else if ((val = match_opt(argv, argc, &i, "--revnum"))) revnum = atoi(val);
        else if ((val = match_opt(argv, argc, &i, "--max-iter"))) max_iter = atoi(val);
        else if ((val = match_opt(argv, argc, &i, "--sigma-k"))) sigma_k = atof(val);
        else if ((val = match_opt(argv, argc, &i, "--frame"))) {
            if (!strcmp(val, "eci")) frame_eci = 1;
            else if (!strcmp(val, "ecef")) frame_eci = 0;
            else { fprintf(stderr, "--frame must be ecef or eci\n"); return EXIT_FAILURE; }
        }
        else if (a[0] == '-' && a[1] != '\0') {
            fprintf(stderr, "Unknown option: %s\n", a);
            usage(argv[0]);
            return EXIT_FAILURE;
        }
        else if (!path) path = a;
        else { fprintf(stderr, "Unexpected argument: %s\n", a); return EXIT_FAILURE; }
    }

    if (catnr < 1 || catnr > 99999) { fprintf(stderr, "--catnr out of range (1..99999)\n"); return EXIT_FAILURE; }
    if (elset < 0 || elset > 9999) { fprintf(stderr, "--elset out of range (0..9999)\n"); return EXIT_FAILURE; }
    if (revnum < 0 || revnum > 99999) { fprintf(stderr, "--revnum out of range (0..99999)\n"); return EXIT_FAILURE; }

    FILE *fp = path ? fopen(path, "r") : stdin;
    if (!fp) { fprintf(stderr, "Cannot open %s\n", path); return EXIT_FAILURE; }

    // Read the five data lines.
    char l_epoch[256], l_pos[256], l_dpos[256], l_vel[256], l_dvel[256];
    int ok = next_line(fp, l_epoch, sizeof l_epoch)
          && next_line(fp, l_pos,  sizeof l_pos)
          && next_line(fp, l_dpos, sizeof l_dpos)
          && next_line(fp, l_vel,  sizeof l_vel)
          && next_line(fp, l_dvel, sizeof l_dvel);
    if (path) fclose(fp);
    if (!ok) {
        fprintf(stderr, "Need five data lines: epoch, position, position-sigma, "
                        "velocity, velocity-sigma.\n");
        return EXIT_FAILURE;
    }

    int year, mon, day, hh, mm;
    double ss;
    if (parse_epoch(l_epoch, &year, &mon, &day, &hh, &mm, &ss) != 0) {
        fprintf(stderr, "Bad epoch '%s' (want YYYY-MM-DDThh:mm:ssZ)\n", l_epoch);
        return EXIT_FAILURE;
    }
    double r_in[3], sig_r[3], v_in[3], sig_v[3];
    if (!parse_triplet(l_pos, r_in) || !parse_triplet(l_dpos, sig_r)
        || !parse_triplet(l_vel, v_in) || !parse_triplet(l_dvel, sig_v)) {
        fprintf(stderr, "Could not parse three numbers from a position/velocity line.\n");
        return EXIT_FAILURE;
    }

    // metres -> km, m/s -> km/s.
    for (int i = 0; i < 3; ++i) { r_in[i] /= 1000.0; v_in[i] /= 1000.0; }
    double sig_r_m = sqrt(sig_r[0]*sig_r[0] + sig_r[1]*sig_r[1] + sig_r[2]*sig_r[2]);
    double sig_v_m = sqrt(sig_v[0]*sig_v[0] + sig_v[1]*sig_v[1] + sig_v[2]*sig_v[2]);

    // Epoch as day-of-year + fraction, the Julian date, and the TLE field.
    // Computed directly (not via Julian_Date) so it round-trips exactly
    // through Julian_Date_of_Epoch when the suite reads this TLE back.
    int doy = DOY(year, mon, day);
    double day_frac = (hh * 3600.0 + mm * 60.0 + ss) / 86400.0;
    double epoch_doy = doy + day_frac;
    double jd_epoch = Julian_Date_of_Year((double) year) + epoch_doy;
    double epoch_field = (year % 100) * 1000.0 + epoch_doy;

    // ECEF -> inertial (or pass through if the input is already inertial).
    double r_eci[3], v_eci[3];
    if (frame_eci) {
        memcpy(r_eci, r_in, sizeof r_eci);
        memcpy(v_eci, v_in, sizeof v_eci);
    } else {
        ecef_to_eci(ThetaG_JD(jd_epoch), r_in, v_in, r_eci, v_eci);
    }

    // Starting guess, then fit.
    double el[EL_COUNT];
    rv_to_elements(r_eci, v_eci, el);
    double el_guess[EL_COUNT];
    memcpy(el_guess, el, sizeof el_guess);

    if (twopi / el[EL_N] >= 225.0) {
        fprintf(stderr, "Orbital period >= 225 min: this is a deep-space orbit, which\n"
                        "this tool does not fit (SGP4 only). Aborting.\n");
        return EXIT_FAILURE;
    }

    if (verbose) fprintf(stderr, "differential correction:\n");
    double res_p, res_v;
    int fit = fit_elements(el, r_eci, v_eci, max_iter, verbose, &res_p, &res_v);

    int used_fallback = 0;
    if (fit < 0) {
        fprintf(stderr, "SGP4 could not be evaluated for these elements. Aborting.\n");
        return EXIT_FAILURE;
    }
    if (fit == 0 && res_p > 5.0) {
        // Could not get close; fall back to the two-body elements.
        memcpy(el, el_guess, sizeof el);
        used_fallback = 1;
        double g[6];
        residual(el, r_eci, v_eci, g);
        res_p = res_pos(g);
        res_v = res_vel(g);
        fprintf(stderr, "WARNING: differential correction did not converge; falling back\n"
                        "         to the two-body (osculating) elements. SGP4 will not\n"
                        "         reproduce the input state. Residual |dr| = %.3f km.\n", res_p);
    }

    normalize_elements(el);

    // Internal units -> TLE units (degrees, rev/day).
    double el_deg[EL_COUNT];
    el_deg[EL_INCL] = el[EL_INCL] / de2ra;
    el_deg[EL_RAAN] = el[EL_RAAN] / de2ra;
    el_deg[EL_ECC]  = el[EL_ECC];
    el_deg[EL_ARGP] = el[EL_ARGP] / de2ra;
    el_deg[EL_MA]   = el[EL_MA] / de2ra;
    double n_rev_day = el[EL_N] * 1440.0 / twopi;

    char card1[70], card2[70];
    format_tle(catnr, classd, intl, epoch_field, elset, revnum, el_deg, n_rev_day,
               card1, card2);

    // Self-check: the library must accept exactly what we wrote.
    char set[139];
    memset(set, 0, sizeof set);
    memcpy(set, card1, 69);
    memcpy(set + 69, card2, 69);
    if (!Good_Elements(set)) {
        fprintf(stderr, "Internal error: generated TLE failed Good_Elements. Not emitting.\n");
        return EXIT_FAILURE;
    }

    // Orbit summary + fit/gate report -> stderr.
    double a_km = cbrt(GM_KM3_S2 / (el[EL_N] / 60.0 * el[EL_N] / 60.0));
    double period_min = twopi / el[EL_N];
    double apogee = a_km * (1.0 + el[EL_ECC]) - EARTH_RADIUS_MEAN_KM;
    double perigee = a_km * (1.0 - el[EL_ECC]) - EARTH_RADIUS_MEAN_KM;
    double res_p_m = res_p * 1000.0, res_v_m = res_v * 1000.0;

    fprintf(stderr,
        "tle_from_state: %s\n"
        "  epoch (UTC)      %04d-%02d-%02dT%02d:%02d:%06.3fZ  (DOY %.6f)\n"
        "  input frame      %s\n"
        "  inclination      %9.4f deg      RAAN   %9.4f deg\n"
        "  eccentricity     %9.7f          arg.p  %9.4f deg\n"
        "  mean anomaly     %9.4f deg      mean motion %.8f rev/day\n"
        "  period           %9.2f min      perigee %.1f km  apogee %.1f km\n"
        "  fit              %s, |dr| = %.3f m, |dv| = %.4f m/s\n",
        used_fallback ? "two-body fallback (see warning)" : "converged",
        year, mon, day, hh, mm, ss, epoch_doy,
        frame_eci ? "ECI (as given)" : "ECEF/ITRF -> inertial via GMST",
        el_deg[EL_INCL], el_deg[EL_RAAN], el_deg[EL_ECC], el_deg[EL_ARGP],
        el_deg[EL_MA], n_rev_day, period_min, perigee, apogee,
        used_fallback ? "FALLBACK" : (fit ? "converged" : "best-effort"),
        res_p_m, res_v_m);

    // Quality gate against the supplied sigmas (magnitudes, frame-agnostic).
    // The limit is sigma_k times the 1-sigma magnitude; a zero sigma means
    // "no information", so that axis is not gated.
    double pos_limit = sigma_k * sig_r_m;
    double vel_limit = sigma_k * sig_v_m;
    int gate_fail = 0;
    if (sig_r_m > 0.0 && res_p_m > pos_limit) gate_fail = 1;
    if (sig_v_m > 0.0 && res_v_m > vel_limit) gate_fail = 1;
    if (gate_fail) {
        fprintf(stderr,
            "  QUALITY GATE: FAIL -- cannot reproduce the state within %.3gx sigma\n"
            "                position |dr| %.3f m   vs limit %.3f m\n"
            "                velocity |dv| %.4f m/s vs limit %.4f m/s\n",
            sigma_k, res_p_m, pos_limit, res_v_m, vel_limit);
        if (strict_sigma) {
            fprintf(stderr, "  --strict-sigma set: no TLE emitted.\n");
            return EXIT_FAILURE;
        }
    } else {
        fprintf(stderr,
            "  quality gate:    pass (|dr| %.3f m <= %.3f m, |dv| %.4f m/s <= %.4f m/s)\n",
            res_p_m, pos_limit, res_v_m, vel_limit);
    }

    // The TLE itself -> stdout.
    if (emit_name && name[0]) printf("%s\n", name);
    printf("%s\n%s\n", card1, card2);
    return EXIT_SUCCESS;
}
