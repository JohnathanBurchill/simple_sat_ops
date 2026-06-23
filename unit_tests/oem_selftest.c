/*

    Simple Satellite Operations  unit_tests/oem_selftest.c

    Exercises src/orbit/oem.c: the CCSDS OEM text parser (meta fields,
    ephemeris rows, the Gregorian->Julian date conversion, START/STOP
    defaulting, and the error paths) and oem_sample_at (cubic Hermite
    interpolation inside the window, two-body Kepler extrapolation
    outside it). oem_load_from_ssm is a popen() wrapper and is not
    exercised here.

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

#include "tap.h"
#include "oem.h"

#include <math.h>
#include <string.h>

#define MU_EARTH 398600.4418   // km^3/s^2, matches oem.c

static int approx(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

static double vec_maxerr(const double a[3], const double b[3])
{
    double m = 0.0;
    for (int k = 0; k < 3; ++k) {
        double e = fabs(a[k] - b[k]);
        if (e > m) m = e;
    }
    return m;
}

static double vec_norm(const double a[3])
{
    return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

// ---- oem_parse ---------------------------------------------------------

static void test_parse_basic(void)
{
    // Header lines before META are skipped; CENTER_NAME inside META is an
    // unknown key and ignored; a COVARIANCE block is skipped wholesale.
    static const char *doc =
        "CCSDS_OEM_VERS = 3.0\n"
        "CREATION_DATE = 2025-06-01T00:00:00\n"
        "ORIGINATOR = SSM\n"
        "\n"
        "META_START\n"
        "OBJECT_NAME = FRONTIERSAT\n"
        "OBJECT_ID = 2025-999A\n"
        "CENTER_NAME = EARTH\n"
        "REF_FRAME = ITRF2014\n"
        "TIME_SYSTEM = UTC\n"
        "START_TIME = 2000-01-01T00:00:00\n"
        "STOP_TIME = 2000-01-01T00:02:00\n"
        "META_STOP\n"
        "\n"
        "2000-01-01T00:00:00 1000.0 2000.0 3000.0 1.0 2.0 3.0\n"
        "2000-01-01T00:01:00 1060.0 2120.0 3180.0 1.0 2.0 3.0\n"
        "2000-01-01T00:02:00 1120.0 2240.0 3360.0 1.0 2.0 3.0\n"
        "\n"
        "COVARIANCE_START\n"
        "EPOCH = 2000-01-01T00:00:00\n"
        "CX_X = 1.234\n"
        "COVARIANCE_STOP\n";

    oem_table_t t;
    int rc = oem_parse(doc, &t);
    tap_okf(rc == 0, "parse of a well-formed OEM succeeds");
    if (rc != 0) { return; }

    tap_okf(strcmp(t.object_name, "FRONTIERSAT") == 0, "OBJECT_NAME parsed ('%s')", t.object_name);
    tap_okf(strcmp(t.object_id, "2025-999A") == 0, "OBJECT_ID parsed ('%s')", t.object_id);
    tap_okf(strcmp(t.ref_frame, "ITRF2014") == 0, "REF_FRAME parsed ('%s')", t.ref_frame);
    tap_okf(strcmp(t.time_system, "UTC") == 0, "TIME_SYSTEM parsed ('%s')", t.time_system);
    tap_okf(t.n_samples == 3, "three ephemeris rows parsed; covariance skipped (got %zu)", t.n_samples);

    // Independent date oracle: 2000-01-01T00:00:00 UTC == JD 2451544.5
    // (the J2000.0 epoch 2000-01-01T12:00 is 2451545.0, twelve hours later).
    tap_okf(approx(t.start_jul_utc, 2451544.5, 1e-9),
            "START_TIME -> JD 2451544.5 (got %.9f)", t.start_jul_utc);
    tap_okf(approx(t.stop_jul_utc, 2451544.5 + 2.0/1440.0, 1e-9),
            "STOP_TIME -> JD +2 min (got %.9f)", t.stop_jul_utc);

    // First and last sample geometry, and the middle sample to confirm rows
    // are not transposed or off-by-one.
    double r0[3] = {1000.0, 2000.0, 3000.0}, v0[3] = {1.0, 2.0, 3.0};
    double r2[3] = {1120.0, 2240.0, 3360.0};
    tap_okf(vec_maxerr(t.samples[0].r_ecef, r0) < 1e-9, "sample 0 position parsed");
    tap_okf(vec_maxerr(t.samples[0].v_ecef, v0) < 1e-9, "sample 0 velocity parsed");
    tap_okf(approx(t.samples[1].r_ecef[1], 2120.0, 1e-9), "sample 1 Y parsed (mid-row sanity)");
    tap_okf(vec_maxerr(t.samples[2].r_ecef, r2) < 1e-9, "sample 2 position parsed");
    tap_okf(approx(t.samples[0].jul_utc, 2451544.5, 1e-9), "sample 0 epoch matches START_TIME");

    oem_free(&t);
}

static void test_parse_defaults(void)
{
    // No START_TIME / STOP_TIME in META: they default to the first and last
    // sample epochs.
    static const char *doc =
        "META_START\n"
        "OBJECT_NAME = X\n"
        "META_STOP\n"
        "2000-01-01T00:00:00 1 2 3 0.1 0.2 0.3\n"
        "2000-01-01T01:00:00 4 5 6 0.4 0.5 0.6\n";

    oem_table_t t;
    int rc = oem_parse(doc, &t);
    tap_okf(rc == 0, "parse with no START/STOP succeeds");
    if (rc != 0) return;
    tap_okf(approx(t.start_jul_utc, t.samples[0].jul_utc, 1e-12),
            "START defaults to first sample epoch");
    tap_okf(approx(t.stop_jul_utc, t.samples[t.n_samples - 1].jul_utc, 1e-12),
            "STOP defaults to last sample epoch");
    oem_free(&t);
}

static void test_parse_skips_bad_rows(void)
{
    // A malformed ephemeris row (too few fields) is skipped, not fatal; the
    // valid rows around it still parse.
    static const char *doc =
        "META_START\n"
        "META_STOP\n"
        "2000-01-01T00:00:00 1 2 3 0.1 0.2 0.3\n"
        "this is not an ephemeris row\n"
        "2000-01-01T00:01:00 4 5 6 0.4 0.5 0.6\n";
    oem_table_t t;
    int rc = oem_parse(doc, &t);
    tap_okf(rc == 0, "parse tolerates a junk ephemeris row");
    tap_okf(rc == 0 && t.n_samples == 2, "junk row skipped, two good rows kept (got %zu)",
            rc == 0 ? t.n_samples : (size_t)0);
    if (rc == 0) oem_free(&t);
}

static void test_parse_errors(void)
{
    oem_table_t t;

    tap_ok(oem_parse(NULL, &t) == -1, "NULL text rejected");
    tap_ok(oem_parse("META_START\nMETA_STOP\n", NULL) == -1, "NULL output rejected");

    // Fewer than two ephemeris points is an error.
    static const char *one =
        "META_START\nMETA_STOP\n"
        "2000-01-01T00:00:00 1 2 3 0.1 0.2 0.3\n";
    tap_ok(oem_parse(one, &t) == -1, "single ephemeris point rejected");

    // A second META block is unsupported.
    static const char *two_meta =
        "META_START\nMETA_STOP\n"
        "2000-01-01T00:00:00 1 2 3 0.1 0.2 0.3\n"
        "2000-01-01T00:01:00 4 5 6 0.4 0.5 0.6\n"
        "META_START\n";
    tap_ok(oem_parse(two_meta, &t) == -1, "second META block rejected");

    // Malformed START_TIME.
    static const char *bad_start =
        "META_START\n"
        "START_TIME = not-a-timestamp\n"
        "META_STOP\n"
        "2000-01-01T00:00:00 1 2 3 0.1 0.2 0.3\n"
        "2000-01-01T00:01:00 4 5 6 0.4 0.5 0.6\n";
    tap_ok(oem_parse(bad_start, &t) == -1, "malformed START_TIME rejected");

    // A well-formed row count but an unparseable epoch in an ephemeris row.
    static const char *bad_epoch =
        "META_START\nMETA_STOP\n"
        "notatime 1 2 3 0.1 0.2 0.3\n"
        "2000-01-01T00:01:00 4 5 6 0.4 0.5 0.6\n";
    tap_ok(oem_parse(bad_epoch, &t) == -1, "ephemeris row with bad epoch rejected");
}

// ---- oem_sample_at: cubic Hermite (in window) --------------------------

// A fixed cubic per axis (km vs seconds), with v as its exact analytic
// derivative (km/s). Cubic Hermite reproduces any cubic exactly, so this is
// an independent oracle: feed knot states sampled from this cubic, then the
// interpolated state at any interior time must equal the cubic evaluated
// directly.
static void cubic_state(double ts, double r[3], double v[3])
{
    r[0] = 1000.0 + 3.0*ts - 0.002*ts*ts + 1e-5*ts*ts*ts;
    v[0] = 3.0 - 0.004*ts + 3e-5*ts*ts;
    r[1] = -2000.0 + 1.5*ts + 0.001*ts*ts - 5e-6*ts*ts*ts;
    v[1] = 1.5 + 0.002*ts - 1.5e-5*ts*ts;
    r[2] = 500.0 - 2.0*ts + 0.0005*ts*ts + 2e-6*ts*ts*ts;
    v[2] = -2.0 + 0.001*ts + 6e-6*ts*ts;
}

static void test_sample_cubic(void)
{
    // Knots at 0/40/80/120 s. The Julian base is a small fake value on
    // purpose: it keeps Julian-Date double-rounding (~5e-10 day at a real JD
    // of 2.45e6) from masking the interpolation math, so the tolerances
    // below reflect the algorithm, not time representation.
    const double JBASE = 1000.0;
    const double knot_s[4] = {0.0, 40.0, 80.0, 120.0};
    oem_sample_t s[4];
    memset(s, 0, sizeof s);
    for (int i = 0; i < 4; ++i) {
        s[i].jul_utc = JBASE + knot_s[i] / 86400.0;
        cubic_state(knot_s[i], s[i].r_ecef, s[i].v_ecef);
    }
    oem_table_t t;
    memset(&t, 0, sizeof t);
    t.samples = s;
    t.n_samples = 4;
    t.capacity = 4;
    t.start_jul_utc = s[0].jul_utc;
    t.stop_jul_utc = s[3].jul_utc;

    // Endpoint exactness: at a knot the interpolant must return that knot's
    // state to roundoff (the basis is exactly 1/0 there).
    double r[3], v[3], er[3], ev[3];
    tap_okf(oem_sample_at(&t, s[0].jul_utc, r, v) == 0, "sample at first knot succeeds");
    cubic_state(0.0, er, ev);
    tap_okf(vec_maxerr(r, er) < 1e-9, "first knot position is exact");
    tap_okf(vec_maxerr(v, ev) < 1e-9, "first knot velocity is exact");
    tap_okf(oem_sample_at(&t, s[3].jul_utc, r, v) == 0, "sample at last knot succeeds");
    cubic_state(120.0, er, ev);
    tap_okf(vec_maxerr(r, er) < 1e-9, "last knot position is exact");
    tap_okf(vec_maxerr(v, ev) < 1e-9, "last knot velocity is exact");

    // Interior queries spanning all three intervals (exercises the bracket
    // search). Each must reproduce the cubic. Tolerances are generous of the
    // analytic-exact value purely to absorb floating roundoff.
    const double q_s[3] = {15.0, 55.0, 95.0};
    for (int i = 0; i < 3; ++i) {
        double jq = JBASE + q_s[i] / 86400.0;
        int rc = oem_sample_at(&t, jq, r, v);
        cubic_state(q_s[i], er, ev);
        double pe = vec_maxerr(r, er);
        double ve = vec_maxerr(v, ev);
        tap_okf(rc == 0, "interior sample at t=%.0fs succeeds", q_s[i]);
        tap_okf(rc == 0 && pe < 1e-6, "t=%.0fs position matches the cubic (maxerr %.2e km)", q_s[i], pe);
        tap_okf(rc == 0 && ve < 1e-8, "t=%.0fs velocity matches the cubic (maxerr %.2e km/s)", q_s[i], ve);
    }
}

// ---- oem_sample_at: two-body Kepler (outside window) -------------------

static void make_circular_table(oem_table_t *t, oem_sample_t s[2],
                                 double jbase, double speed_scale)
{
    // Two snapshots of a circular orbit anchored ON the rotation (z) axis:
    // r = (0,0,7000) km, v in +X. Putting the radius on the z-axis makes
    // omega x r = 0, so the ITRF velocity here equals the ECI velocity and
    // the orbit is genuinely circular in the inertial frame where the
    // propagation happens (a circular *ITRF* state would be an ellipse in
    // ECI, since ITRF->ECI adds omega x r ~ 0.5 km/s). speed_scale = 1.0
    // gives the circular speed; > sqrt(2) gives an escape (hyperbolic) state.
    double vcirc = sqrt(MU_EARTH / 7000.0);
    memset(s, 0, 2 * sizeof s[0]);
    for (int i = 0; i < 2; ++i) {
        s[i].jul_utc = jbase + (double)i * 60.0 / 86400.0;
        s[i].r_ecef[0] = 0.0; s[i].r_ecef[1] = 0.0; s[i].r_ecef[2] = 7000.0;
        s[i].v_ecef[0] = vcirc * speed_scale; s[i].v_ecef[1] = 0.0; s[i].v_ecef[2] = 0.0;
    }
    memset(t, 0, sizeof *t);
    t->samples = s;
    t->n_samples = 2;
    t->capacity = 2;
    t->start_jul_utc = s[0].jul_utc;
    t->stop_jul_utc = s[1].jul_utc;
}

static void test_sample_kepler(void)
{
    const double JBASE = 2451545.0;
    oem_table_t t;
    oem_sample_t s[2];
    make_circular_table(&t, s, JBASE, 1.0);

    double r[3], v[3];

    // Mean motion of the circular orbit (rad/s). For r0 on the z-axis moving
    // in +X, the inertial position is 7000*(sin(n*dt), 0, cos(n*dt)); the
    // z-component is left untouched by the z-axis Earth rotation, so
    // r_ecef[2] == 7000*cos(n*dt) is a gmst-INDEPENDENT check on the orbital
    // phase. |r| alone is not enough -- it depends only on f^2, so it can't
    // see an f-coefficient sign flip that mirrors the position; the z phase
    // pins the direction and catches it.
    double n = sqrt(MU_EARTH / (7000.0 * 7000.0 * 7000.0));

    // Forward extrapolation 600 s past the window. A circular orbit keeps
    // |r| constant, and the ITRF<->ECI transform is a rotation (preserves
    // magnitude), so |r_ecef| must stay 7000 km regardless of how far we go.
    double dt1 = 600.0;
    int rc = oem_sample_at(&t, t.stop_jul_utc + dt1 / 86400.0, r, v);
    double rn = vec_norm(r);
    tap_okf(rc == 0, "forward Kepler extrapolation succeeds");
    tap_okf(rc == 0 && approx(rn, 7000.0, 1e-3), "circular |r| preserved forward (got %.6f km)", rn);
    tap_okf(rc == 0 && approx(r[2], 7000.0 * cos(n * dt1), 1e-2),
            "forward orbital phase correct: z = 7000*cos(n*dt) (got %.4f, want %.4f)",
            r[2], 7000.0 * cos(n * dt1));

    // The state must actually have moved off the anchor (not a no-op).
    double anchor[3] = {0.0, 0.0, 7000.0};
    tap_okf(rc == 0 && vec_maxerr(r, anchor) > 1000.0,
            "forward extrapolation moved the satellite (>1000 km from anchor)");

    // A quarter period (~1457 s) onward: still on the circle, and a quarter
    // turn means z ~ 0.
    double dt2 = 1457.0;
    rc = oem_sample_at(&t, t.stop_jul_utc + dt2 / 86400.0, r, v);
    rn = vec_norm(r);
    tap_okf(rc == 0 && approx(rn, 7000.0, 1e-3), "circular |r| preserved at ~T/4 (got %.6f km)", rn);
    tap_okf(rc == 0 && approx(r[2], 7000.0 * cos(n * dt2), 1e-2),
            "orbital phase at ~T/4 correct: z ~ 0 (got %.4f)", r[2]);

    // Backward extrapolation before the window uses the first sample.
    rc = oem_sample_at(&t, t.start_jul_utc - 600.0 / 86400.0, r, v);
    rn = vec_norm(r);
    tap_okf(rc == 0, "backward Kepler extrapolation succeeds");
    tap_okf(rc == 0 && approx(rn, 7000.0, 1e-3), "circular |r| preserved backward (got %.6f km)", rn);

    // Hyperbolic anchor (escape speed): the Kepler solver refuses, so an
    // out-of-window query fails rather than returning a bogus state.
    oem_table_t th;
    oem_sample_t sh[2];
    make_circular_table(&th, sh, JBASE, 2.0);   // 2x circular speed > escape
    rc = oem_sample_at(&th, th.stop_jul_utc + 600.0 / 86400.0, r, v);
    tap_okf(rc == -1, "extrapolation from a non-bound (hyperbolic) state is refused");
}

static void test_sample_errors(void)
{
    double r[3], v[3];
    tap_ok(oem_sample_at(NULL, 2451545.0, r, v) == -1, "NULL table rejected");

    oem_sample_t s1[1];
    memset(s1, 0, sizeof s1);
    s1[0].jul_utc = 2451545.0;
    oem_table_t t1;
    memset(&t1, 0, sizeof t1);
    t1.samples = s1;
    t1.n_samples = 1;
    t1.start_jul_utc = t1.stop_jul_utc = 2451545.0;
    tap_ok(oem_sample_at(&t1, 2451545.0, r, v) == -1, "table with one sample rejected");
}

int main(void)
{
    test_parse_basic();
    test_parse_defaults();
    test_parse_skips_bad_rows();
    test_parse_errors();
    test_sample_cubic();
    test_sample_kepler();
    test_sample_errors();
    return tap_done();
}
