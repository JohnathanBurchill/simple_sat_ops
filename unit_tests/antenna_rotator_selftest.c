/*

    Simple Satellite Operations  unit_tests/antenna_rotator_selftest.c

    Unit tests for the pure helpers in antenna_rotator.c:
      - antenna_rotator_wrap_to_pm180
      - antenna_rotator_accumulate_unwrapped
      - antenna_rotator_home_unwrapped_target
      - antenna_rotator_to_mech_coords

    None of these touch the serial port, so the binary builds and runs on
    any host (no SPID rotator, no ALSA, no UHD). The serial-I/O paths in
    antenna_rotator.c link as dead code.

    Exit status: 0 = all tests passed, non-zero = failure.

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

#include "antenna_rotator.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define check(cond, what) tap_ok((cond), (what))

static int approx(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

#define CHECK_APPROX(actual, expected, msg)                                \
    do {                                                                   \
        double _a = (actual);                                              \
        double _e = (expected);                                            \
        int _ok = approx(_a, _e, 1e-9);                                    \
        tap_ok(_ok, (msg));                                                \
        if (!_ok) tap_diag("expected %.9f, got %.9f", _e, _a);             \
    } while (0)

// ---------------------------------------------------------------- wrap_to_pm180

// Contract: result lies in (-180, 180]. 180 is fixed; -180 maps to +180.
static void test_wrap_to_pm180(void)
{
    fprintf(stderr, "wrap_to_pm180:\n");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(0.0), 0.0,
                 "0 -> 0");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(179.9), 179.9,
                 "179.9 -> 179.9");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(180.0), 180.0,
                 "180 stays 180 (upper boundary inclusive)");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(180.1), -179.9,
                 "180.1 -> -179.9");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(-180.0), 180.0,
                 "-180 maps up to +180 (lower boundary exclusive)");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(-179.9), -179.9,
                 "-179.9 -> -179.9");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(360.0), 0.0,
                 "360 -> 0");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(-360.0), 0.0,
                 "-360 -> 0");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(540.0), 180.0,
                 "540 -> 180");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(-540.0), 180.0,
                 "-540 -> 180 (via -180 -> +180 step)");
    CHECK_APPROX(antenna_rotator_wrap_to_pm180(720.0), 0.0,
                 "720 -> 0");
}

// ----------------------------------------------------- accumulate_unwrapped

// Contract: track the minimal-delta path forward from prev so a continuous
// stream of wrapped predictions accumulates into a monotone-friendly unwrapped
// trajectory. Used by the per-tick predicted-az -> commanded-az pipeline so
// the SPID never sees a 359 -> 0 step as -359 degrees of travel.
static void test_accumulate_unwrapped(void)
{
    fprintf(stderr, "accumulate_unwrapped:\n");
    CHECK_APPROX(antenna_rotator_accumulate_unwrapped(0.0, 0.0), 0.0,
                 "no-op when prev == pred");
    CHECK_APPROX(antenna_rotator_accumulate_unwrapped(350.0, 10.0), 370.0,
                 "350 -> 10 unwraps forward to 370 (delta = +20)");
    CHECK_APPROX(antenna_rotator_accumulate_unwrapped(10.0, 350.0), -10.0,
                 "10 -> 350 unwraps backward to -10 (delta = -20)");
    CHECK_APPROX(antenna_rotator_accumulate_unwrapped(370.0, 10.0), 370.0,
                 "370 -> wrapped 10 (= unwrapped 370) does not move");
    CHECK_APPROX(antenna_rotator_accumulate_unwrapped(-10.0, 10.0), 10.0,
                 "monotone forward across 0");
    CHECK_APPROX(antenna_rotator_accumulate_unwrapped(179.0, -179.0), 181.0,
                 "179 -> -179 unwraps to 181, not back to -179");
    CHECK_APPROX(antenna_rotator_accumulate_unwrapped(-179.0, 179.0), -181.0,
                 "-179 -> 179 unwraps to -181");

    // Multi-step trajectory: spinning eastward through 0 should accumulate
    // monotonically rather than oscillate.
    double prev = 350.0;
    double track[] = { 355.0, 0.0, 5.0, 10.0, 15.0 };
    double expected[] = { 355.0, 360.0, 365.0, 370.0, 375.0 };
    int monotone_ok = 1;
    for (int i = 0; i < 5; ++i) {
        prev = antenna_rotator_accumulate_unwrapped(prev, track[i]);
        if (!approx(prev, expected[i], 1e-9)) {
            monotone_ok = 0;
            fprintf(stderr, "    step %d: expected %.3f, got %.3f\n",
                    i, expected[i], prev);
        }
    }
    check(monotone_ok, "eastward sweep 350 -> 15 unwraps to 350 -> 375");
}

// -------------------------------------------------- home_unwrapped_target

// Contract: choose the in-range co-terminal of `home` closest to 0 so the
// home command unwinds toward the mechanical centre instead of winding
// another revolution. Range used here is [ANTENNA_ROTATOR_MINIMUM_AZIMUTH,
// ANTENNA_ROTATOR_MAXIMUM_AZIMUTH] = [-179, 539].
static void test_home_unwrapped_target(void)
{
    fprintf(stderr, "home_unwrapped_target:\n");
    CHECK_APPROX(antenna_rotator_home_unwrapped_target(0.0, 0.0), 0.0,
                 "home from 0 stays at 0");
    CHECK_APPROX(antenna_rotator_home_unwrapped_target(370.0, 0.0), 0.0,
                 "home from +370 picks 0 (unwind one rev)");
    CHECK_APPROX(antenna_rotator_home_unwrapped_target(-100.0, 0.0), 0.0,
                 "home from -100 picks 0");
    CHECK_APPROX(antenna_rotator_home_unwrapped_target(500.0, 0.0), 0.0,
                 "home from +500 picks 0 (not +360)");
    CHECK_APPROX(antenna_rotator_home_unwrapped_target(250.0, 180.0), 180.0,
                 "home=180 from 250 picks 180 (no -180 in range)");
    CHECK_APPROX(antenna_rotator_home_unwrapped_target(200.0, 90.0), 90.0,
                 "home=90 from 200 picks 90, not 450");
    // Result must stay within the rotator's mechanical range.
    double r = antenna_rotator_home_unwrapped_target(539.0, 0.0);
    check(r >= ANTENNA_ROTATOR_MINIMUM_AZIMUTH
              && r <= ANTENNA_ROTATOR_MAXIMUM_AZIMUTH,
          "result is always inside [-179, 539]");
}

// --------------------------------------------------------- to_mech_coords

// Contract (per the header doc): with flip=0 or MAX_ELEVATION<=90, the
// mapping is a pass-through. With flip=1 and MAX_ELEVATION>90, mech_az
// is lerped from aos_az (progress=0) to (los_az + 180) (progress=1)
// along the shortest arc, then the sat is projected onto that meridian.
// For zenith passes (los_az = aos_az + 180), the lerp is constant and
// the mapping collapses to the simpler aos_az-hold.
//
// Most of the assertions below pin to the zenith case via flip_zenith()
// so the aos_az-hold geometry is preserved. test_to_mech_coords_flip_lerp
// covers the los_az != aos_az + 180 case explicitly.

// Helper: invoke the flip mapping in the zenith-pass case (los_az
// implicit at aos+180, progress=0). The lerp delta is 0, so mech_az is
// held at aos_az regardless of progress -- matches the simpler hold
// behavior the early tests were written against.
static void flip_zenith(double aos_az, double sat_az, double sat_el,
                        double *out_az, double *out_el, int *out_half)
{
    antenna_rotator_to_mech_coords(1, aos_az, aos_az + 180.0, 0.0,
                                   sat_az, sat_el,
                                   out_az, out_el, out_half);
}
static void test_to_mech_coords_passthrough(void)
{
    fprintf(stderr, "to_mech_coords (pass-through):\n");
    double out_az = -1, out_el = -1;
    int half = -1;
    // flip=0: passthrough. aos_az/los_az/progress ignored.
    antenna_rotator_to_mech_coords(0, 270.0, 90.0, 0.5, 120.0, 30.0,
                                   &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 120.0, "flip=0: out_az == sat_az");
    CHECK_APPROX(out_el, 30.0,  "flip=0: out_el == sat_el");
    check(half == 0, "flip=0: half == 0");

    // flip=1 at AOS (progress=0): mech_az = aos_az regardless of los_az.
    antenna_rotator_to_mech_coords(1, 100.0, 350.0, 0.0, 100.0, 5.0,
                                   &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 100.0, "flip=1 progress=0 sat==aos: out_az == aos_az");
    CHECK_APPROX(out_el, 5.0,   "flip=1 progress=0 sat==aos: out_el == sat_el");
    check(half == 0, "flip=1 progress=0 sat==aos: half == 0");
}

// Projection contract under flip mode: mech_az is held at aos_az and the
// sat is projected orthogonally onto the boom's meridian. mech_el =
// atan2(sin(el), cos(el)*cos(sat_az - aos_az)) in degrees, ranging 0..180.
// out_half is always 0 (no half-transition under the new mapping).

static void test_to_mech_coords_flip_on_meridian(void)
{
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90) {
        fprintf(stderr, "to_mech_coords (flip on-meridian): SKIP "
                        "(ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90)\n");
        return;
    }
    fprintf(stderr, "to_mech_coords (flip, sat on boom meridian):\n");
    double out_az, out_el;
    int half;

    // Forward meridian: sat_az == aos_az -> mech_el == sat_el, no error.
    flip_zenith(100.0, 100.0, 45.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 100.0, "sat at (aos, 45): mech_az = aos_az");
    CHECK_APPROX(out_el,  45.0, "sat at (aos, 45): mech_el = 45");
    check(half == 0, "sat at (aos, 45): half = 0");

    flip_zenith(100.0, 100.0, 89.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_el, 89.0, "sat at (aos, 89): mech_el = 89");

    // Zenith: ambiguous sat_az, but with sat_az = aos_az the projection
    // pins to mech_el = 90.
    flip_zenith(100.0, 100.0, 90.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_el, 90.0, "sat at (aos, 90): mech_el = 90 (zenith)");

    // Back meridian: sat_az = aos_az + 180 -> mech_el = 180 - sat_el,
    // with the rotator interpreting >90 deg as back-pointing.
    flip_zenith(100.0, 280.0, 0.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 100.0, "sat at (aos+180, 0): mech_az = aos_az");
    CHECK_APPROX(out_el, 180.0, "sat at (aos+180, 0): mech_el = 180 (LOS)");

    flip_zenith(100.0, 280.0, 30.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_el, 150.0, "sat at (aos+180, 30): mech_el = 180 - 30 = 150");

    flip_zenith(100.0, 280.0, 89.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_el, 91.0, "sat at (aos+180, 89): mech_el = 91");

    // AOS wrap: aos=10, sat=350 -> daz = -20. Same projection as daz=+20.
    flip_zenith(10.0, 350.0, 5.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 10.0, "aos=10 sat=350: mech_az = aos_az = 10");
    // y = cos(5)*cos(-20) = 0.9962*0.9397 = 0.9362; z = sin(5) = 0.0872
    // mech_el = atan2(0.0872, 0.9362) = 5.319 deg
    CHECK_APPROX(out_el, 5.31910333780531,
                 "aos=10 sat=350 el=5: mech_el = atan2(sin(5), cos(5)*cos(-20))");
}

static void test_to_mech_coords_flip_off_meridian(void)
{
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90) {
        fprintf(stderr, "to_mech_coords (flip off-meridian): SKIP "
                        "(ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90)\n");
        return;
    }
    fprintf(stderr, "to_mech_coords (flip, sat off boom meridian):\n");
    double out_az, out_el;
    int half;

    // Sat at daz=+90: along-meridian component vanishes, projection
    // pins to mech_el = 90 deg (boom straight up). Worst-case pointing
    // error = (90 - sat_el).
    flip_zenith(100.0, 190.0, 30.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 100.0, "sat at (aos+90, 30): mech_az = aos_az");
    CHECK_APPROX(out_el,  90.0, "sat at (aos+90, 30): mech_el = 90 (boom up, "
                                "60 deg pointing error)");

    flip_zenith(100.0, 190.0, 88.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_el, 90.0, "sat at (aos+90, 88): mech_el = 90 "
                                "(2 deg pointing error -- the high-el case "
                                "the flip mapping is designed for)");

    // Sat at daz=-90: symmetric.
    flip_zenith(100.0, 10.0, 60.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_el, 90.0, "sat at (aos-90, 60): mech_el = 90 (symmetric)");

    // Sat at daz=+45, el=30: partial projection, mech_el ~ 39.2 deg.
    flip_zenith(100.0, 145.0, 30.0, &out_az, &out_el, &half);
    double expect_el_45_30 = atan2(sin(30.0 * M_PI / 180.0),
                                   cos(30.0 * M_PI / 180.0)
                                       * cos(45.0 * M_PI / 180.0))
                             * 180.0 / M_PI;
    CHECK_APPROX(out_el, expect_el_45_30,
                 "sat at (aos+45, 30): mech_el = atan2(sin(30), cos(30)*cos(45))");

    // Past the old half boundary (daz=+91): mech_el just past 90 deg,
    // continuous with daz=+89 result.
    flip_zenith(100.0, 191.0, 80.0, &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 100.0,
                 "sat at (aos+91, 80): mech_az still = aos_az (no jump)");
    check(out_el > 90.0 && out_el < 91.0,
          "sat at (aos+91, 80): mech_el just past 90 (back-pointing onset)");
}

// Continuity is the headline win: a small change in sat_az/sat_el across
// the old half boundary now gives a small change in mech_az/mech_el, not
// a 180 deg jump. main.c's flip_half reseed branch becomes inert under
// the new mapping (it never fires because out_half is always 0).
static void test_to_mech_coords_flip_continuity(void)
{
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90) {
        fprintf(stderr, "to_mech_coords (flip continuity): SKIP "
                        "(ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90)\n");
        return;
    }
    fprintf(stderr, "to_mech_coords (flip, continuity across daz=+/-90):\n");
    double az_a, el_a, az_b, el_b;
    int half_a, half_b;

    flip_zenith(100.0, 189.0, 70.0, &az_a, &el_a, &half_a);
    flip_zenith(100.0, 191.0, 70.0, &az_b, &el_b, &half_b);
    CHECK_APPROX(az_a, az_b, "mech_az unchanged across daz=+/-90 boundary");
    check(fabs(el_a - el_b) < 1.0,
          "mech_el delta < 1 deg for 2 deg sat_az step across boundary");
    check(half_a == 0 && half_b == 0,
          "half is always 0 (mapping is continuous)");

    // Sweep daz from 88 to 92 in 0.5 deg steps; mech_el must change
    // monotonically, with no step exceeding the sat_az step.
    double prev_el = NAN;
    int monotone_ok = 1;
    double max_step = 0.0;
    for (int i = 0; i <= 8; ++i) {
        double daz = 88.0 + 0.5 * (double)i;
        flip_zenith(100.0, 100.0 + daz, 70.0, &az_a, &el_a, &half_a);
        if (!isnan(prev_el)) {
            double step = el_a - prev_el;
            if (step < 0) { monotone_ok = 0; }
            if (fabs(step) > max_step) max_step = fabs(step);
        }
        prev_el = el_a;
    }
    check(monotone_ok, "mech_el monotone non-decreasing as sat_az sweeps "
                       "across boundary");
    check(max_step < 2.0,
          "no single 0.5 deg sat_az step produces a > 2 deg mech_el jump");
}

// LOS-interp behavior: when los_az != aos_az + 180 the lerp slowly
// rotates mech_az across the pass so the boom converges on the
// satellite at both AOS and LOS.
static void test_to_mech_coords_flip_lerp(void)
{
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90) {
        fprintf(stderr, "to_mech_coords (flip lerp): SKIP "
                        "(ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90)\n");
        return;
    }
    fprintf(stderr, "to_mech_coords (flip, AOS-LOS lerp):\n");
    double out_az, out_el;
    int half;

    // Non-zenith pass: aos=190, los=350 (diff 160 deg, los+180=170 deg).
    // Lerp shortest arc: aos=190 -> los_target=170, delta=-20 deg.
    const double aos = 190.0;
    const double los = 350.0;

    // progress=0: mech_az = aos_az.
    antenna_rotator_to_mech_coords(1, aos, los, 0.0, aos, 0.0,
                                   &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 190.0, "progress=0: mech_az = aos_az");
    CHECK_APPROX(out_el, 0.0,   "progress=0 sat at AOS: mech_el = 0");

    // progress=1: mech_az = (los_az + 180) mod 360 = 170. Sat at LOS
    // (350, 0) maps to mech_el = 180 (back-pointing horizon).
    antenna_rotator_to_mech_coords(1, aos, los, 1.0, los, 0.0,
                                   &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 170.0, "progress=1: mech_az = (los_az + 180) mod 360");
    CHECK_APPROX(out_el, 180.0, "progress=1 sat at LOS: mech_el = 180 "
                                "(boom points at LOS via back-form)");

    // progress=0.5: mech_az halfway along shortest arc = 180.
    antenna_rotator_to_mech_coords(1, aos, los, 0.5, 180.0, 88.0,
                                   &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 180.0, "progress=0.5: mech_az on shortest-arc midpoint");
    // Sat (180, 88), boom meridian at 180 -> sat is on the meridian.
    // daz = 0, y = cos(88)*1, z = sin(88), mech_el = atan2(sin88, cos88) = 88.
    CHECK_APPROX(out_el, 88.0, "progress=0.5 sat on meridian: mech_el = sat_el");

    // Wraparound: aos near 360, los near 0. shortest arc small.
    antenna_rotator_to_mech_coords(1, 350.0, 10.0, 1.0, 10.0, 0.0,
                                   &out_az, &out_el, &half);
    // los_target = 10 + 180 = 190. delta = 190 - 350 = -160 -> +200 wrap?
    // wrap_to_pm180(-160) = -160 (in range). mech_az = 350 + 1*(-160) = 190.
    CHECK_APPROX(out_az, 190.0, "aos=350 los=10 progress=1: mech_az = 190 "
                                "(short arc -160 deg)");

    // Progress clamping: < 0 behaves as 0.
    antenna_rotator_to_mech_coords(1, aos, los, -0.5, aos, 0.0,
                                   &out_az, &out_el, &half);
    CHECK_APPROX(out_az, aos, "progress=-0.5 clamps to 0");

    // Progress clamping: > 1 behaves as 1.
    antenna_rotator_to_mech_coords(1, aos, los, 1.5, los, 0.0,
                                   &out_az, &out_el, &half);
    CHECK_APPROX(out_az, 170.0, "progress=1.5 clamps to 1");

    // Zenith pass invariance: los = aos + 180 -> delta = 0 -> mech_az
    // constant regardless of progress.
    antenna_rotator_to_mech_coords(1, 100.0, 280.0, 0.0, 100.0, 5.0,
                                   &out_az, &out_el, &half);
    double az_p0 = out_az;
    antenna_rotator_to_mech_coords(1, 100.0, 280.0, 1.0, 100.0, 5.0,
                                   &out_az, &out_el, &half);
    CHECK_APPROX(out_az, az_p0, "zenith pass: mech_az constant across progress");
}

// Simulate a near-zenith pass and report the largest one-tick mech_az step
// flip mode produces. Under the zenith-case lerp (los=aos+180, delta=0)
// mech_az stays fixed at aos_az for the whole pass; non-zenith passes
// would show a small linear ramp instead.
static void test_to_mech_coords_high_pass_slew(void)
{
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90) {
        fprintf(stderr, "to_mech_coords (high-pass slew): SKIP "
                        "(ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90)\n");
        return;
    }
    fprintf(stderr, "to_mech_coords (high-pass slew report):\n");

    // Synthetic 88-degree pass: rises due south (aos_az = 180), apex near
    // zenith with sat_az sweeping rapidly across due-east to due-west, sets
    // due north. Discretize at 1-second cadence over the apex region.
    const double aos_az = 180.0;
    const double max_el = 88.0;
    // Parametrize sat_az(t) and sat_el(t) along a great-circle pass with
    // AOS due south and LOS due north. t in [-1, 1]; t = 0 is apex.
    const int N = 401;
    double prev_mech_az = NAN;
    double max_step = 0.0;
    int max_step_idx = -1;
    double max_step_el = 0.0;
    for (int i = 0; i < N; ++i) {
        double t = -1.0 + 2.0 * (double)i / (double)(N - 1);
        // Elevation profile: parabola peaked at apex.
        double el = max_el * (1.0 - t * t);
        if (el < 0.0) el = 0.0;
        // Azimuth profile: equator-of-orbit angle measured from AOS. As the
        // sat climbs, the sub-satellite track stays near the AOS meridian
        // until apex, where it jumps 180 degrees across to the LOS side in
        // a region whose width shrinks with (90 - max_el). Model that with
        // a smooth steepened arctan; the steepness is what stresses flip.
        double k = 60.0; // steepness factor; bigger = tighter apex
        double phi = atan(k * t) / atan(k);
        double az = aos_az + 90.0 * (phi + 1.0); // sweeps aos..aos+180
        while (az >= 360.0) az -= 360.0;
        while (az < 0.0)    az += 360.0;

        double mech_az, mech_el;
        int half;
        flip_zenith(aos_az, az, el, &mech_az, &mech_el, &half);

        if (!isnan(prev_mech_az)) {
            double step = fabs(antenna_rotator_wrap_to_pm180(
                mech_az - prev_mech_az));
            if (step > max_step) {
                max_step = step;
                max_step_idx = i;
                max_step_el = el;
            }
        }
        prev_mech_az = mech_az;
    }

    fprintf(stderr, "    max one-tick mech_az step in flip mode: "
                    "%.6f deg (at tick %d, el=%.2f, dt=%.3f s)\n",
            max_step, max_step_idx, max_step_el,
            2.0 / (double)(N - 1));
    check(max_step < 1e-9,
          "flip mode holds mech_az fixed: max one-tick step ~ 0");
}

// ---------------------------------------------------------- socketpair stub
//
// antenna_rotator_command() does plain write()+read() on its fd. A
// socketpair gives us a bidirectional fd pair with no tty/pty fuss: the
// rotator-side fd is sv[0], and the test fixture writes pre-cooked
// responses to / reads back command bytes from sv[1]. We skip
// antenna_rotator_connect() (no tcsetattr, no /dev/* open) by setting
// connected=1 and fd directly.

static int rotator_pair_init(antenna_rotator_t *r, int *test_fd)
{
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0) {
        return -1;
    }
    memset(r, 0, sizeof(*r));
    r->fd = sv[0];
    r->connected = 1;
    *test_fd = sv[1];
    return 0;
}

static void rotator_pair_close(antenna_rotator_t *r, int test_fd)
{
    if (r->connected) {
        close(r->fd);
        r->connected = 0;
    }
    if (test_fd >= 0) {
        close(test_fd);
    }
}

// Build a 12-byte SPID Rot2Prog status response for (az, el).
// Encoding (matches the parser in antenna_rotator_command):
//   [0]  start byte 0x57 ('W') -- ignored by parser
//   [1..4]  AZ digits: ((az+360)*10) split into thousands/hundreds/tens/ones
//   [5]  PH (AZ pulses/deg) -- ignored
//   [6..9]  EL digits, same encoding
//   [10] PV -- ignored
//   [11] terminator 0x20 -- ignored
// Each digit byte is the raw value 0..9, NOT ASCII (unlike SET).
static void encode_status_response(uint8_t resp[AR_RESPONSE_LEN],
                                   double az, double el)
{
    int az_q = (int)((az + 360.0) * 10.0 + 0.5);
    int el_q = (int)((el + 360.0) * 10.0 + 0.5);
    resp[0]  = 0x57;
    resp[1]  = (uint8_t)((az_q / 1000) % 10);
    resp[2]  = (uint8_t)((az_q / 100)  % 10);
    resp[3]  = (uint8_t)((az_q / 10)   % 10);
    resp[4]  = (uint8_t)( az_q         % 10);
    resp[5]  = 0;
    resp[6]  = (uint8_t)((el_q / 1000) % 10);
    resp[7]  = (uint8_t)((el_q / 100)  % 10);
    resp[8]  = (uint8_t)((el_q / 10)   % 10);
    resp[9]  = (uint8_t)( el_q         % 10);
    resp[10] = 0;
    resp[11] = 0x20;
}

// -------------------------------------------------- set_unwrapped limits

// Contract: set_unwrapped rejects out-of-range targets BEFORE touching
// the serial port, and does not mutate rotator state on rejection.
static void test_set_unwrapped_limits(void)
{
    fprintf(stderr, "set_unwrapped (limit checks):\n");
    antenna_rotator_t r;
    memset(&r, 0, sizeof(r));
    // connected=0: any I/O attempt would return BAD_RESPONSE, so a
    // limit-rejection return value of *_LIMIT proves the gate fired first.

    int rc = antenna_rotator_set_unwrapped(&r,
        ANTENNA_ROTATOR_MINIMUM_AZIMUTH - 1.0, 45.0);
    check(rc == ANTENNA_ROTATOR_AZIMUTH_LIMIT, "az < min rejected");

    rc = antenna_rotator_set_unwrapped(&r,
        ANTENNA_ROTATOR_MAXIMUM_AZIMUTH + 1.0, 45.0);
    check(rc == ANTENNA_ROTATOR_AZIMUTH_LIMIT, "az > max rejected");

    rc = antenna_rotator_set_unwrapped(&r, 0.0,
        ANTENNA_ROTATOR_MINIMUM_ELEVATION - 1.0);
    check(rc == ANTENNA_ROTATOR_ELEVATION_LIMIT, "el < min rejected");

    rc = antenna_rotator_set_unwrapped(&r, 0.0,
        ANTENNA_ROTATOR_MAXIMUM_ELEVATION + 1.0);
    check(rc == ANTENNA_ROTATOR_ELEVATION_LIMIT, "el > max rejected");

    // Boundaries inclusive: az == min and az == max should pass the
    // limit gate and reach the I/O path (which then fails with
    // BAD_RESPONSE because connected=0).
    rc = antenna_rotator_set_unwrapped(&r,
        ANTENNA_ROTATOR_MINIMUM_AZIMUTH, 0.0);
    check(rc == ANTENNA_ROTATOR_BAD_RESPONSE,
          "az == min passes limit gate");
    rc = antenna_rotator_set_unwrapped(&r,
        ANTENNA_ROTATOR_MAXIMUM_AZIMUTH, 0.0);
    check(rc == ANTENNA_ROTATOR_BAD_RESPONSE,
          "az == max passes limit gate");

    // State must not be mutated on limit rejection.
    r.target_azimuth           = 12.0;
    r.target_azimuth_unwrapped = 12.0;
    r.target_elevation         = 5.0;
    r.unwrapped_target_valid   = 1;
    antenna_rotator_set_unwrapped(&r,
        ANTENNA_ROTATOR_MAXIMUM_AZIMUTH + 1.0, 5.0);
    int unchanged = (r.target_azimuth == 12.0)
                 && (r.target_azimuth_unwrapped == 12.0)
                 && (r.target_elevation == 5.0)
                 && (r.unwrapped_target_valid == 1);
    check(unchanged, "state unchanged after AZIMUTH_LIMIT");
}

// ----------------------------------------------------- increase_azimuth

// Contract: base = target_azimuth_unwrapped when unwrapped_target_valid,
// else target_azimuth. Add `angle` and check the result is in range.
static void test_increase_azimuth_base_selection(void)
{
    fprintf(stderr, "increase_azimuth (base selection + limits):\n");
    antenna_rotator_t r;
    memset(&r, 0, sizeof(r));
    // connected=0: passing the limit gate falls into I/O and yields
    // BAD_RESPONSE, which is the signal we use to say "limit passed".
    r.target_azimuth           = 100.0;
    r.target_azimuth_unwrapped = 460.0;
    r.unwrapped_target_valid   = 1;
    r.target_elevation         = 10.0;

    // unwrapped base = 460; +10 = 470 is in [-179, 539] -> limit ok.
    int rc = antenna_rotator_increase_azimuth(&r, 10.0);
    check(rc == ANTENNA_ROTATOR_BAD_RESPONSE,
          "unwrapped base 460 + 10 = 470 passes limit gate");

    // 460 + 100 = 560 > 539 -> AZ limit.
    rc = antenna_rotator_increase_azimuth(&r, 100.0);
    check(rc == ANTENNA_ROTATOR_AZIMUTH_LIMIT,
          "unwrapped base 460 + 100 = 560 rejected (> max 539)");

    // Drop unwrapped_target_valid; base falls back to target_azimuth (100).
    r.unwrapped_target_valid = 0;
    rc = antenna_rotator_increase_azimuth(&r, 100.0);
    check(rc == ANTENNA_ROTATOR_BAD_RESPONSE,
          "fallback base = target_azimuth=100; 100 + 100 = 200 in range");

    rc = antenna_rotator_increase_azimuth(&r, -300.0);
    check(rc == ANTENNA_ROTATOR_AZIMUTH_LIMIT,
          "fallback base 100 - 300 = -200 < min -179, rejected");
}

// ---------------------------------------------- command() wire protocol

// Contract: SET command writes 13 bytes: 'W' AZ4 0x00 EL4 0x00 cmd ' '
// where AZ4 and EL4 are ASCII digit encodings of (value + 360). Returns
// OK without reading (rotator does not respond to SET).
static void test_set_command_wire_format(void)
{
    fprintf(stderr, "command(SET) wire format:\n");
    antenna_rotator_t r;
    int test_fd;
    if (rotator_pair_init(&r, &test_fd) != 0) {
        check(0, "socketpair() failed");
        return;
    }

    double az = 10.0, el = 5.0;
    int rc = antenna_rotator_command(&r, ANTENNA_ROTATOR_SET, &az, &el);
    check(rc == ANTENNA_ROTATOR_OK, "SET returns OK without reading");

    uint8_t buf[AR_CMD_LEN] = {0};
    ssize_t n = read(test_fd, buf, AR_CMD_LEN);
    check(n == AR_CMD_LEN, "SET wrote exactly 13 bytes");
    check(buf[0] == 'W', "SET [0] = 'W'");
    check(buf[1] == '0' && buf[2] == '3' && buf[3] == '7' && buf[4] == '0',
          "SET az=10 -> '0370' ASCII (az+360=370)");
    check(buf[5] == 0x00, "SET [5] = 0x00 separator");
    check(buf[6] == '0' && buf[7] == '3' && buf[8] == '6' && buf[9] == '5',
          "SET el=5 -> '0365' ASCII (el+360=365)");
    check(buf[10] == 0x00, "SET [10] = 0x00 separator");
    check(buf[11] == (uint8_t)ANTENNA_ROTATOR_SET,
          "SET [11] = 0x2F (SET cmd)");
    check(buf[12] == ' ', "SET [12] = 0x20 (' ')");

    rotator_pair_close(&r, test_fd);
}

// Boundary-value coverage on the BCD encoding: -179, 0, 539.
static void test_set_command_boundary_values(void)
{
    fprintf(stderr, "command(SET) boundary BCD encoding:\n");
    struct {
        double az, el;
        char az_str[5], el_str[5];
        const char *label;
    } cases[] = {
        { -179.0, -5.0, "0181", "0355", "az=-179 el=-5 (mins)" },
        {    0.0,  0.0, "0360", "0360", "az=0 el=0" },
        {  539.0, 180.0, "0899", "0540", "az=539 el=180 (maxes)" },
    };
    for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
        antenna_rotator_t r;
        int test_fd;
        if (rotator_pair_init(&r, &test_fd) != 0) {
            check(0, "socketpair() failed");
            return;
        }
        double a = cases[i].az, e = cases[i].el;
        antenna_rotator_command(&r, ANTENNA_ROTATOR_SET, &a, &e);
        uint8_t buf[AR_CMD_LEN] = {0};
        read(test_fd, buf, AR_CMD_LEN);
        int ok = (buf[1] == (uint8_t)cases[i].az_str[0])
              && (buf[2] == (uint8_t)cases[i].az_str[1])
              && (buf[3] == (uint8_t)cases[i].az_str[2])
              && (buf[4] == (uint8_t)cases[i].az_str[3])
              && (buf[6] == (uint8_t)cases[i].el_str[0])
              && (buf[7] == (uint8_t)cases[i].el_str[1])
              && (buf[8] == (uint8_t)cases[i].el_str[2])
              && (buf[9] == (uint8_t)cases[i].el_str[3]);
        check(ok, cases[i].label);
        if (!ok) {
            fprintf(stderr, "    got az='%c%c%c%c' el='%c%c%c%c'\n",
                    buf[1], buf[2], buf[3], buf[4],
                    buf[6], buf[7], buf[8], buf[9]);
        }
        rotator_pair_close(&r, test_fd);
    }
}

// Contract: STATUS writes a 13-byte query, reads back a 12-byte response,
// and parses the BCD digits into (az, el) in degrees.
static void test_status_command_round_trip(void)
{
    fprintf(stderr, "command(STATUS) round-trip:\n");
    antenna_rotator_t r;
    int test_fd;
    if (rotator_pair_init(&r, &test_fd) != 0) {
        check(0, "socketpair() failed");
        return;
    }

    uint8_t resp[AR_RESPONSE_LEN];
    encode_status_response(resp, 12.5, 42.0);
    ssize_t w = write(test_fd, resp, AR_RESPONSE_LEN);
    check(w == AR_RESPONSE_LEN, "test fixture wrote 12-byte response");
    // Half-close so the rotator's read loop sees EOF after our 12 bytes
    // (defensive: protects against the read loop spinning on a partial
    // chunk).
    shutdown(test_fd, SHUT_WR);

    double az = 0.0, el = 0.0;
    int rc = antenna_rotator_command(&r, ANTENNA_ROTATOR_STATUS, &az, &el);
    check(rc == ANTENNA_ROTATOR_OK, "STATUS returns OK");
    CHECK_APPROX(az, 12.5, "STATUS parsed az = 12.5");
    CHECK_APPROX(el, 42.0, "STATUS parsed el = 42.0");

    uint8_t cmdbuf[AR_CMD_LEN] = {0};
    ssize_t n = read(test_fd, cmdbuf, AR_CMD_LEN);
    check(n == AR_CMD_LEN, "STATUS wrote 13-byte query");
    check(cmdbuf[11] == (uint8_t)ANTENNA_ROTATOR_STATUS,
          "STATUS query [11] = 0x1F");

    rotator_pair_close(&r, test_fd);
}

// Disconnected I/O must not block on the (zero-initialized) fd and must
// return BAD_RESPONSE. This guards against the comment in command():
// "fd may be zero-initialized (= stdin)".
static void test_command_when_disconnected(void)
{
    fprintf(stderr, "command (disconnected):\n");
    antenna_rotator_t r;
    memset(&r, 0, sizeof(r));
    r.connected = 0;
    double az = 0.0, el = 0.0;
    int rc = antenna_rotator_command(&r, ANTENNA_ROTATOR_STATUS, &az, &el);
    check(rc == ANTENNA_ROTATOR_BAD_RESPONSE,
          "STATUS on connected=0 returns BAD_RESPONSE (no read on stdin)");
    rc = antenna_rotator_command(&r, ANTENNA_ROTATOR_SET, &az, &el);
    check(rc == ANTENNA_ROTATOR_BAD_RESPONSE,
          "SET on connected=0 returns BAD_RESPONSE");
}

// ----------------------------------------------- seed_from_status state

static void test_seed_from_status(void)
{
    fprintf(stderr, "seed_from_status (state population):\n");
    antenna_rotator_t r;
    int test_fd;
    if (rotator_pair_init(&r, &test_fd) != 0) {
        check(0, "socketpair() failed");
        return;
    }

    uint8_t resp[AR_RESPONSE_LEN];
    encode_status_response(resp, -45.0, 30.0);
    write(test_fd, resp, AR_RESPONSE_LEN);
    shutdown(test_fd, SHUT_WR);

    int rc = antenna_rotator_seed_from_status(&r);
    check(rc == ANTENNA_ROTATOR_OK, "seed_from_status returns OK");
    CHECK_APPROX(r.azimuth,                 -45.0, "seed: azimuth = -45");
    CHECK_APPROX(r.elevation,                30.0, "seed: elevation = 30");
    CHECK_APPROX(r.target_azimuth,          -45.0, "seed: target_azimuth = -45");
    CHECK_APPROX(r.target_azimuth_unwrapped, -45.0, "seed: target_azimuth_unwrapped = -45");
    CHECK_APPROX(r.target_elevation,         30.0, "seed: target_elevation = 30");
    check(r.unwrapped_target_valid == 1,
          "seed: unwrapped_target_valid = 1");

    rotator_pair_close(&r, test_fd);
}

// --------------------------------------------- set_unwrapped state update

// Round-trip test of the success path: send a command, verify state
// fields are updated, and confirm the on-wire bytes match the input.
static void test_set_unwrapped_state_update(void)
{
    fprintf(stderr, "set_unwrapped (state on success):\n");
    antenna_rotator_t r;
    int test_fd;
    if (rotator_pair_init(&r, &test_fd) != 0) {
        check(0, "socketpair() failed");
        return;
    }

    // Az outside [0, 360) on purpose -- the whole point of unwrapped
    // tracking is that we send values like 425 (one rev past 65).
    int rc = antenna_rotator_set_unwrapped(&r, 425.0, 75.0);
    check(rc == ANTENNA_ROTATOR_OK, "set_unwrapped(425, 75) returns OK");
    CHECK_APPROX(r.target_azimuth_unwrapped, 425.0,
                 "target_azimuth_unwrapped = 425");
    CHECK_APPROX(r.target_azimuth, 425.0,
                 "target_azimuth = 425 (same as unwrapped form)");
    CHECK_APPROX(r.target_elevation, 75.0,
                 "target_elevation = 75");
    check(r.unwrapped_target_valid == 1,
          "unwrapped_target_valid set");

    uint8_t buf[AR_CMD_LEN] = {0};
    read(test_fd, buf, AR_CMD_LEN);
    check(buf[1]=='0' && buf[2]=='7' && buf[3]=='8' && buf[4]=='5',
          "on-wire az = '0785' (425 + 360 = 785)");
    check(buf[6]=='0' && buf[7]=='4' && buf[8]=='3' && buf[9]=='5',
          "on-wire el = '0435' (75 + 360 = 435)");
    check(buf[11] == (uint8_t)ANTENNA_ROTATOR_SET,
          "on-wire cmd byte = SET (0x2F)");

    rotator_pair_close(&r, test_fd);
}

// ----------------------------------------- pass simulation (integration)

// Replay main.c's tracking-loop math against a synthetic near-zenith pass
// and tally what the rotator would actually be commanded to do. Two
// metrics confirm the fix:
//
//   max_commanded_step:  largest single-tick |delta_az| that triggers a
//                        SET. With mech_az held at aos_az for the whole
//                        pass, this is 0 -- no SET command ever moves
//                        the azimuth.
//
//   max_physical_jump:   largest |new_target - last_issued_target| at
//                        a SET. main.c's half-transition reseed no longer
//                        fires (out_half is 0 throughout), so the rotator
//                        never has to make up a 180 deg gap. Also 0.
//
// Pre-fix these were ~14 deg and ~166 deg; post-fix they are 0/0. If
// these checks ever regress, the rotator will start chasing sat_az
// across apex again.
#define SSO_TEST_DELTA_THRESHOLD_DEG 1.0   // mirrors MAX_DELTA_*_DEGREES in main.c
static void test_pass_simulation_flip_baseline(void)
{
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90) {
        fprintf(stderr, "pass_simulation_flip_baseline: SKIP\n");
        return;
    }
    fprintf(stderr, "pass_simulation (flip-mode baseline):\n");

    // Synthetic 88-degree pass, AOS due south, LOS due north, 8 min at 1 Hz.
    const double aos_az  = 180.0;
    const double max_el  = 88.0;
    const int    N       = 481;

    int    flip_pass    = (max_el >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD);
    double flip_aos_az  = aos_az;
    int    flip_half    = 0;

    // Shadow only the fields the tracking loop drives.
    double target_az_unwrapped = aos_az;
    double target_el           = 0.0;
    // The position the rotator was last *told* to go to. Updated only on
    // an actual SET command, not on a silent half-transition reseed.
    double last_issued_target  = aos_az;
    int    last_issued_valid   = 0;

    int    set_commands       = 0;
    double max_commanded_step = 0.0;
    double max_el_step        = 0.0;
    double max_physical_jump  = 0.0;
    double total_az_travel    = 0.0;
    int    half_transitions   = 0;

    for (int i = 0; i < N; ++i) {
        double t = -1.0 + 2.0 * (double)i / (double)(N - 1);

        double sat_el = max_el * (1.0 - t * t);
        if (sat_el < 0.0) sat_el = 0.0;

        double k   = 60.0;
        double phi = atan(k * t) / atan(k);
        double sat_az = aos_az + 90.0 * (phi + 1.0);
        while (sat_az >= 360.0) sat_az -= 360.0;
        while (sat_az < 0.0)    sat_az += 360.0;

        double mech_az, mech_el;
        int half;
        // Synthetic pass models a zenith trajectory (aos -> aos+180 over
        // t=-1..1). Match that with los_az = aos_az + 180 (lerp delta=0).
        double progress = (double)i / (double)(N - 1);
        antenna_rotator_to_mech_coords(flip_pass, flip_aos_az,
                                       flip_aos_az + 180.0, progress,
                                       sat_az, sat_el,
                                       &mech_az, &mech_el, &half);

        if (flip_pass && half != flip_half) {
            // Reseed the unwrap accumulator at the half boundary so it
            // does not bridge a 180-degree jump. Note this DOES NOT send
            // a SET, so last_issued_target stays at the first-half final
            // value -- the bug.
            target_az_unwrapped = mech_az;
            flip_half = half;
            ++half_transitions;
        }

        double next_az = antenna_rotator_accumulate_unwrapped(
                            target_az_unwrapped, mech_az);
        double next_el = mech_el;
        if (next_el < ANTENNA_ROTATOR_MINIMUM_ELEVATION) {
            next_el = ANTENNA_ROTATOR_MINIMUM_ELEVATION;
        } else if (next_el > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
            next_el = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
        }

        double delta_az = next_az - target_az_unwrapped;
        double delta_el = next_el - target_el;

        if (fabs(delta_az) >= SSO_TEST_DELTA_THRESHOLD_DEG
            || fabs(delta_el) >= SSO_TEST_DELTA_THRESHOLD_DEG) {
            ++set_commands;
            if (fabs(delta_az) > max_commanded_step) {
                max_commanded_step = fabs(delta_az);
            }
            if (fabs(delta_el) > max_el_step) {
                max_el_step = fabs(delta_el);
            }
            if (last_issued_valid) {
                double jump = fabs(next_az - last_issued_target);
                if (jump > max_physical_jump) max_physical_jump = jump;
            }
            total_az_travel += fabs(delta_az);
            target_az_unwrapped = next_az;
            target_el           = next_el;
            last_issued_target  = next_az;
            last_issued_valid   = 1;
        }
    }

    fprintf(stderr, "    SET commands issued:    %d / %d ticks\n",
            set_commands, N);
    fprintf(stderr, "    max commanded az step:  %.2f deg\n",
            max_commanded_step);
    fprintf(stderr, "    max physical az jump:   %.2f deg "
                    "(target_n vs last_issued)\n", max_physical_jump);
    fprintf(stderr, "    max commanded el step:  %.2f deg\n", max_el_step);
    fprintf(stderr, "    total az travel:        %.2f deg\n", total_az_travel);
    fprintf(stderr, "    half transitions:       %d\n", half_transitions);

    check(flip_pass == 1,
          "flip mode engaged at max_el = 88 (>= threshold 75)");
    check(half_transitions == 0,
          "no flip-half transitions (out_half is always 0 under new mapping)");

    // FIX assertions: both numbers must be essentially zero. Anything
    // greater than a degree means the flip mapping is back to chasing
    // sat_az somewhere.
    check(max_commanded_step < 1e-9,
          "FIX: max commanded az step ~ 0 (mech_az held at aos_az)");
    check(max_physical_jump < 1e-9,
          "FIX: max physical az jump ~ 0 (no half-transition reseed)");
    // The pass should still produce reasonable mech_el motion.
    check(set_commands > 0,
          "elevation tracking still issues SETs across the pass");
    CHECK_APPROX(total_az_travel, 0.0,
                 "FIX: total az travel = 0 deg");
}

// The two-step home's final leg must NOT fire on the post-SET target echo
// (a STATUS reading still sitting at the commanded mid waypoint), only on
// real feedback that has unwound into the zone where the short path to the
// target runs the same way as the unwind. This is the decision behind the old
// "r winds to 360" bug. antenna_rotator_home_leg2_ready encodes it (and
// tracking_tick calls it); tol here matches tracking.c HOME_ECHO_TOLERANCE_DEG.
static void test_home_leg2_echo_rejection(void)
{
    fprintf(stderr, "home two-step leg-2 echo rejection:\n");
    const double tol = 2.0;

    // Geometry A: unwind negative (mid 170 -> final 0).
    // The mid waypoint is itself in-zone here, so a reading at/near it tests
    // the echo gate specifically, not the zone gate.
    check(antenna_rotator_home_leg2_ready(170.0, 170.0, 0.0, tol) == 0,
          "echo exactly at the mid waypoint does not fire leg 2");
    check(antenna_rotator_home_leg2_ready(171.0, 170.0, 0.0, tol) == 0,
          "reading within the echo tolerance of the mid does not fire");
    check(antenna_rotator_home_leg2_ready(172.0, 170.0, 0.0, tol) == 0,
          "reading exactly at the echo tolerance does not fire (strict >)");
    check(antenna_rotator_home_leg2_ready(80.0, 170.0, 0.0, tol) == 1,
          "real feedback unwound into the zone fires leg 2");
    check(antenna_rotator_home_leg2_ready(250.0, 170.0, 0.0, tol) == 0,
          "real feedback wound the wrong way (out of zone) does not fire");
    check(antenna_rotator_home_leg2_ready(0.0, 170.0, 0.0, tol) == 1,
          "reading at the final target (remaining 0) fires leg 2");

    // Geometry B: unwind positive (mid 180 -> final 350), the mirror image.
    check(antenna_rotator_home_leg2_ready(180.0, 180.0, 350.0, tol) == 0,
          "echo at the mid waypoint (positive unwind) does not fire");
    check(antenna_rotator_home_leg2_ready(270.0, 180.0, 350.0, tol) == 1,
          "real feedback in the zone (positive unwind) fires leg 2");
    check(antenna_rotator_home_leg2_ready(90.0, 180.0, 350.0, tol) == 0,
          "real feedback wound the wrong way (positive unwind) does not fire");
}

int main(void)
{
    test_wrap_to_pm180();
    test_accumulate_unwrapped();
    test_home_unwrapped_target();
    test_home_leg2_echo_rejection();
    test_to_mech_coords_passthrough();
    test_to_mech_coords_flip_on_meridian();
    test_to_mech_coords_flip_off_meridian();
    test_to_mech_coords_flip_continuity();
    test_to_mech_coords_flip_lerp();
    test_to_mech_coords_high_pass_slew();
    test_set_unwrapped_limits();
    test_increase_azimuth_base_selection();
    test_set_command_wire_format();
    test_set_command_boundary_values();
    test_status_command_round_trip();
    test_command_when_disconnected();
    test_seed_from_status();
    test_set_unwrapped_state_update();
    test_pass_simulation_flip_baseline();
    return tap_done();
}
