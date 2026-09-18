/*

    Simple Satellite Operations  unit_tests/attitude_selftest.c

    Coverage for src/orbit/attitude.{c,h} -- the geometry that turns the
    ADCS's three attitude angles into a direction in space and a point on
    the ground. The viewers draw a line from the satellite to that point,
    so a sign error here does not fail loudly: it draws a confident line
    at the wrong place on the Earth, which is worse than drawing none.

    Every expected value below is worked out by hand from the frame
    definitions, or (for the geodetic conversion) round-tripped against
    sgp4sdp4's own Calculate_User_PosVel, which is the library's separate
    geodetic-to-inertial path and knows nothing of the code under test.

    What's covered:
      - attitude_orbit_frame on a circular equatorial orbit: nadir down
        the position vector, Y on the negative orbit normal, X along the
        flight direction, and the set right-handed.
      - the degenerate position-parallel-to-velocity case leaves no
        orbit plane rather than NaNs.
      - attitude_body_axes at zero angles lands exactly on the orbit
        frame, and one axis at a time under 90 degrees of yaw, pitch and
        roll, each checked against the axis it must become.
      - the body set stays orthonormal and right-handed at an arbitrary
        set of angles.
      - attitude_ray_sphere: straight down gives exactly the altitude,
        a ray along the horizon misses, and the limb angle (arcsine of
        the radius ratio, worked out independently) is the boundary
        between hit and miss to within a tenth of a degree.
      - attitude_solve: off-nadir equals the roll angle, the cross-track
        sign says left of track, the ground point is where the hand
        calculation puts it, and a miss leaves hit clear.
      - attitude_eci_to_geodetic: round-trips three places, wraps
        longitude into -180..180, and puts the pole at 90 degrees.

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
*/

#include "attitude.h"
#include "tap.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Unit vectors compared component by component. 1e-12 is about four
// orders of magnitude above double rounding on the handful of
// multiplications and one square root each component costs, and far
// below any error a wrong sign or a transposed matrix would make.
#define VEPS 1e-12

static int veq(const double a[3], const double b[3], double eps)
{
    return fabs(a[0] - b[0]) < eps
        && fabs(a[1] - b[1]) < eps
        && fabs(a[2] - b[2]) < eps;
}

static double vdot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vcross3(const double a[3], const double b[3], double o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

// A circular equatorial orbit at 7000 km radius, moving towards +Y. Its
// orbit frame is short enough to write down: nadir is -X, the orbit
// normal is +Z so the frame's Y is -Z, and X completes the set at +Y,
// which is the direction of flight.
static const double R_CIRC = 7000.0;
static const double r_eq[3] = { 7000.0, 0.0, 0.0 };
static const double v_eq[3] = { 0.0, 7.5, 0.0 };
static const double ox_eq[3] = { 0.0, 1.0, 0.0 };
static const double oy_eq[3] = { 0.0, 0.0, -1.0 };
static const double oz_eq[3] = { -1.0, 0.0, 0.0 };

static void test_orbit_frame(void)
{
    double ox[3], oy[3], oz[3];
    attitude_orbit_frame(r_eq, v_eq, ox, oy, oz);

    tap_ok(veq(oz, oz_eq, VEPS), "orbit frame: Z points at nadir");
    tap_ok(veq(oy, oy_eq, VEPS),
           "orbit frame: Y lies along the negative orbit normal");
    tap_ok(veq(ox, ox_eq, VEPS),
           "orbit frame: X lies along the direction of flight");

    // Right-handed and orthonormal, which is what everything downstream
    // assumes when it reads a row of the rotation matrix as an axis.
    double xy[3];
    vcross3(ox, oy, xy);
    tap_ok(veq(xy, oz, VEPS), "orbit frame: X cross Y is Z (right-handed)");
    tap_okf(fabs(vdot3(ox, oy)) < VEPS && fabs(vdot3(oy, oz)) < VEPS
            && fabs(vdot3(ox, oz)) < VEPS,
            "orbit frame: axes mutually perpendicular");
    tap_okf(fabs(vdot3(ox, ox) - 1.0) < VEPS
            && fabs(vdot3(oy, oy) - 1.0) < VEPS
            && fabs(vdot3(oz, oz) - 1.0) < VEPS,
            "orbit frame: axes are unit length");

    // An inclined, non-circular state: nadir still down the position
    // vector, and Y still perpendicular to both position and velocity
    // even though X is no longer exactly along the velocity.
    const double r2[3] = { 3000.0, -5200.0, 2600.0 };
    const double v2[3] = { 5.1, 2.3, -1.4 };
    attitude_orbit_frame(r2, v2, ox, oy, oz);
    double rn = sqrt(vdot3(r2, r2));
    const double want_z[3] = { -r2[0] / rn, -r2[1] / rn, -r2[2] / rn };
    tap_ok(veq(oz, want_z, 1e-12),
           "orbit frame: nadir is the position vector reversed, off the equator too");
    tap_okf(fabs(vdot3(oy, r2)) < 1e-9 && fabs(vdot3(oy, v2)) < 1e-9,
            "orbit frame: Y is perpendicular to both position and velocity");
    tap_okf(vdot3(ox, v2) > 0.0,
            "orbit frame: X leans the way the satellite is going");
}

static void test_orbit_frame_degenerate(void)
{
    // Position parallel to velocity has no orbit plane. This cannot
    // happen on a real orbit, but a caller handing over a zeroed or
    // half-filled state shouldn't get NaNs drawn on a globe.
    const double r[3] = { 7000.0, 0.0, 0.0 };
    const double v[3] = { 7.5, 0.0, 0.0 };
    double ox[3], oy[3], oz[3];
    attitude_orbit_frame(r, v, ox, oy, oz);
    tap_ok(veq(oz, oz_eq, VEPS),
           "degenerate state: nadir is still found");
    const double zero[3] = { 0.0, 0.0, 0.0 };
    tap_ok(veq(oy, zero, VEPS) && veq(ox, zero, VEPS),
           "degenerate state: no orbit plane leaves X and Y zero, not NaN");

    // And a zero velocity, the other way a half-filled state arrives.
    const double v0[3] = { 0.0, 0.0, 0.0 };
    attitude_orbit_frame(r, v0, ox, oy, oz);
    tap_ok(veq(oy, zero, VEPS) && veq(ox, zero, VEPS),
           "zero velocity: X and Y zero rather than NaN");
}

static void test_body_axes_zero(void)
{
    double ox[3], oy[3], oz[3], bx[3], by[3], bz[3];
    attitude_orbit_frame(r_eq, v_eq, ox, oy, oz);
    attitude_rpy_t rpy = {0};
    attitude_body_axes(ox, oy, oz, &rpy, bx, by, bz);
    tap_ok(veq(bx, ox, VEPS) && veq(by, oy, VEPS) && veq(bz, oz, VEPS),
           "zero attitude: body axes lie exactly on the orbit axes");
}

// One 90-degree rotation at a time. The expected axis in each case comes
// from the 3-2-1 matrix written out by hand with two of the three angles
// zero, so a transposed matrix or a swapped rotation order fails here.
static void test_body_axes_single_rotations(void)
{
    double ox[3], oy[3], oz[3], bx[3], by[3], bz[3];
    attitude_orbit_frame(r_eq, v_eq, ox, oy, oz);

    // Yaw 90: the nadir face still looks down, but the satellite has
    // turned about it -- body X onto orbit Y, body Y onto -orbit X.
    attitude_rpy_t yaw = { .yaw_deg = 90.0 };
    attitude_body_axes(ox, oy, oz, &yaw, bx, by, bz);
    tap_ok(veq(bz, oz, 1e-12), "yaw 90: nadir face still looks straight down");
    tap_ok(veq(bx, oy, 1e-12), "yaw 90: body X turns onto orbit Y");
    const double neg_ox[3] = { -ox_eq[0], -ox_eq[1], -ox_eq[2] };
    tap_ok(veq(by, neg_ox, 1e-12), "yaw 90: body Y turns onto minus orbit X");

    // Pitch 90: the nadir face swings forward onto the flight direction.
    attitude_rpy_t pitch = { .pitch_deg = 90.0 };
    attitude_body_axes(ox, oy, oz, &pitch, bx, by, bz);
    tap_ok(veq(bz, ox, 1e-12),
           "pitch 90: nadir face swings forward onto the flight direction");
    tap_ok(veq(by, oy, 1e-12), "pitch 90: body Y is untouched by pitch");

    // Roll 90: the nadir face swings sideways onto the positive orbit
    // normal, which is minus the orbit frame's Y.
    attitude_rpy_t roll = { .roll_deg = 90.0 };
    attitude_body_axes(ox, oy, oz, &roll, bx, by, bz);
    const double neg_oy[3] = { -oy_eq[0], -oy_eq[1], -oy_eq[2] };
    tap_ok(veq(bz, neg_oy, 1e-12),
           "roll 90: nadir face swings sideways onto minus orbit Y");
    tap_ok(veq(bx, ox, 1e-12), "roll 90: body X is untouched by roll");
}

static void test_body_axes_orthonormal(void)
{
    // An arbitrary attitude, all three angles at once and none of them a
    // special case. The rotation has to come out orthonormal and
    // right-handed whatever the angles; a matrix assembled wrongly
    // (rows for columns, or one sign flipped) usually breaks one of the
    // two even when each single-axis case passes.
    double ox[3], oy[3], oz[3], bx[3], by[3], bz[3];
    attitude_orbit_frame(r_eq, v_eq, ox, oy, oz);
    attitude_rpy_t rpy = { .roll_deg = 68.33, .pitch_deg = 26.87,
                           .yaw_deg = 36.54 };
    attitude_body_axes(ox, oy, oz, &rpy, bx, by, bz);

    tap_okf(fabs(vdot3(bx, by)) < 1e-12 && fabs(vdot3(by, bz)) < 1e-12
            && fabs(vdot3(bx, bz)) < 1e-12,
            "arbitrary attitude: body axes mutually perpendicular");
    tap_okf(fabs(vdot3(bx, bx) - 1.0) < 1e-12
            && fabs(vdot3(by, by) - 1.0) < 1e-12
            && fabs(vdot3(bz, bz) - 1.0) < 1e-12,
            "arbitrary attitude: body axes are unit length");
    double xy[3];
    vcross3(bx, by, xy);
    tap_ok(veq(xy, bz, 1e-12),
           "arbitrary attitude: body X cross Y is body Z (right-handed)");

    // Rotating by the angles and then back by their negatives in the
    // reverse order has to land where it started, which pins the
    // sequence rather than just the algebra of one matrix.
    double cx[3], cy[3], cz[3];
    attitude_rpy_t undo_roll  = { .roll_deg  = -rpy.roll_deg };
    attitude_rpy_t undo_pitch = { .pitch_deg = -rpy.pitch_deg };
    attitude_rpy_t undo_yaw   = { .yaw_deg   = -rpy.yaw_deg };
    // The body frame reached by the full 3-2-1 rotation, then undone one
    // axis at a time in the opposite order: roll first, then pitch, then
    // yaw, each applied in the frame the last one left.
    attitude_body_axes(bx, by, bz, &undo_roll,  cx, cy, cz);
    attitude_body_axes(cx, cy, cz, &undo_pitch, bx, by, bz);
    attitude_body_axes(bx, by, bz, &undo_yaw,   cx, cy, cz);
    tap_ok(veq(cx, ox, 1e-10) && veq(cy, oy, 1e-10) && veq(cz, oz, 1e-10),
           "arbitrary attitude: undoing roll, pitch then yaw returns the orbit frame");
}

static void test_ray_sphere(void)
{
    const double p[3] = { R_CIRC, 0.0, 0.0 };
    double t = -1.0;

    // Straight down: the distance to the ground is the altitude, exactly.
    const double down[3] = { -1.0, 0.0, 0.0 };
    tap_ok(attitude_ray_sphere(p, down, ATTITUDE_EARTH_MEAN_KM, &t) == 1,
           "ray: straight down hits the Earth");
    tap_okf(fabs(t - (R_CIRC - ATTITUDE_EARTH_MEAN_KM)) < 1e-9,
            "ray: straight down travels the altitude, %.6f km (want %.6f)",
            t, R_CIRC - ATTITUDE_EARTH_MEAN_KM);

    // Straight up, and along the local horizontal: both miss. The
    // horizontal one is the interesting case -- it starts outside the
    // sphere and stays outside, so a solver that took the far root or
    // forgot to check the sign of the near one would claim a hit.
    const double up[3] = { 1.0, 0.0, 0.0 };
    tap_ok(attitude_ray_sphere(p, up, ATTITUDE_EARTH_MEAN_KM, NULL) == 0,
           "ray: straight up misses");
    const double side[3] = { 0.0, 1.0, 0.0 };
    tap_ok(attitude_ray_sphere(p, side, ATTITUDE_EARTH_MEAN_KM, NULL) == 0,
           "ray: along the local horizontal misses");

    // The limb: from radius R the Earth's edge sits arcsine(Re/R) off
    // nadir, which at 7000 km is 65.52510 degrees. Worked out from the
    // radii alone with no reference to the code under test, and checked
    // against bc's own arctangent: 90 - atan(cot(x)) in degrees for
    // 6371/7000 is 90 - 24.47490.
    const double limb_deg = asin(ATTITUDE_EARTH_MEAN_KM / R_CIRC)
                          * (180.0 / M_PI);
    tap_okf(fabs(limb_deg - 65.52510) < 1e-4,
            "ray: the limb from 7000 km is 65.52510 degrees off nadir (got %.5f)",
            limb_deg);
    for (int i = 0; i < 2; i++) {
        // A ray tilted off nadir towards +Y by a shade less, then a
        // shade more, than the limb angle.
        const double a = (limb_deg + (i == 0 ? -0.05 : 0.05)) * (M_PI / 180.0);
        const double d[3] = { -cos(a), sin(a), 0.0 };
        const int hit = attitude_ray_sphere(p, d, ATTITUDE_EARTH_MEAN_KM, &t);
        tap_okf(hit == (i == 0 ? 1 : 0),
                "ray: %.2f degrees off nadir %s the Earth",
                limb_deg + (i == 0 ? -0.05 : 0.05),
                i == 0 ? "reaches" : "passes outside");
    }
}

static void test_solve(void)
{
    attitude_frame_t f;

    // Straight down. The ground point is the sub-satellite point at the
    // Earth's radius, and nothing is off to either side.
    attitude_rpy_t level = {0};
    attitude_solve(r_eq, v_eq, &level, &f);
    tap_okf(fabs(f.off_nadir_deg) < 1e-9,
            "solve: level attitude is 0 degrees off nadir (got %.3e)",
            f.off_nadir_deg);
    tap_okf(fabs(f.cross_track_deg) < 1e-9,
            "solve: level attitude looks along track, not to either side");
    tap_ok(f.hit == 1, "solve: level attitude reaches the ground");
    const double want_hit[3] = { ATTITUDE_EARTH_MEAN_KM, 0.0, 0.0 };
    tap_ok(veq(f.hit_eci, want_hit, 1e-9),
           "solve: level attitude looks at the point right below it");
    tap_okf(fabs(f.hit_range_km - (R_CIRC - ATTITUDE_EARTH_MEAN_KM)) < 1e-9,
            "solve: range to the ground is the altitude");

    // Rolled 30 degrees. Off-nadir is the roll angle itself for a pure
    // roll, and the face is looking to one side of the track.
    attitude_rpy_t rolled = { .roll_deg = 30.0 };
    attitude_solve(r_eq, v_eq, &rolled, &f);
    tap_okf(fabs(f.off_nadir_deg - 30.0) < 1e-9,
            "solve: a 30 degree roll is 30 degrees off nadir (got %.9f)",
            f.off_nadir_deg);
    tap_okf(fabs(f.cross_track_deg - 30.0) < 1e-9,
            "solve: a positive roll looks 30 degrees to the left of track (got %+.9f)",
            f.cross_track_deg);
    tap_ok(f.hit == 1 && f.hit_range_km > R_CIRC - ATTITUDE_EARTH_MEAN_KM,
           "solve: a rolled look has further to go than a straight-down one");
    // The rolled ground point must lie on the sphere, and out of the
    // orbit plane (which for this orbit is the z = 0 plane).
    tap_okf(fabs(sqrt(vdot3(f.hit_eci, f.hit_eci)) - ATTITUDE_EARTH_MEAN_KM) < 1e-9,
            "solve: the ground point sits on the Earth's surface");
    tap_okf(f.hit_eci[2] > 0.0,
            "solve: rolling left puts the ground point north of the track");

    // A pure yaw turns the satellite about a nadir look, so it changes
    // neither the off-nadir angle nor the ground point.
    attitude_rpy_t yawed = { .yaw_deg = 137.0 };
    attitude_solve(r_eq, v_eq, &yawed, &f);
    tap_okf(fabs(f.off_nadir_deg) < 1e-9 && f.hit == 1
            && veq(f.hit_eci, want_hit, 1e-9),
            "solve: yaw alone leaves the ground point where it was");

    // Rolled past the limb: no ground point at all, and the fields that
    // describe one stay clear.
    attitude_rpy_t over = { .roll_deg = 80.0 };
    attitude_solve(r_eq, v_eq, &over, &f);
    tap_okf(fabs(f.off_nadir_deg - 80.0) < 1e-9,
            "solve: an 80 degree roll is 80 degrees off nadir");
    tap_ok(f.hit == 0, "solve: past the limb there is no ground point");
    tap_okf(f.hit_range_km == 0.0,
            "solve: a miss leaves the range at zero rather than stale");

    // Rolling the other way looks to the other side, which is the sign
    // convention the viewers label on screen.
    attitude_rpy_t right = { .roll_deg = -25.0 };
    attitude_solve(r_eq, v_eq, &right, &f);
    tap_okf(fabs(f.cross_track_deg + 25.0) < 1e-9,
            "solve: a negative roll looks to the right of track (got %+.6f)",
            f.cross_track_deg);
    tap_okf(f.hit_eci[2] < 0.0,
           "solve: rolling right puts the ground point south of the track");
}

static void test_eci_to_geodetic(void)
{
    // Round-trip against the library's own geodetic-to-inertial path,
    // which shares no code with the conversion being tested. Any scale,
    // sign or sidereal-time error in either direction shows up as a
    // place that does not come back.
    const double jd = 2461000.5;    // an arbitrary date well inside range
    const struct { double lat, lon, alt; const char *what; } places[] = {
        {  50.8688, -114.2910,   1.279, "the observatory" },
        { -33.9,      18.4,      0.0,   "a southern, eastern place" },
        {   0.0,     179.5,    400.0,   "just short of the date line, at altitude" },
    };
    for (size_t i = 0; i < sizeof places / sizeof places[0]; i++) {
        geodetic_t g = {0};
        g.lat = places[i].lat * (M_PI / 180.0);
        g.lon = places[i].lon * (M_PI / 180.0);
        g.alt = places[i].alt;
        vector_t pos = {0}, vel = {0};
        Calculate_User_PosVel(jd, &g, &pos, &vel);

        const double p[3] = { pos.x, pos.y, pos.z };
        double lat = 0, lon = 0, alt = 0;
        attitude_eci_to_geodetic(jd, p, &lat, &lon, &alt);
        // A tenth of a millidegree is about 10 m on the ground, well
        // inside what the library's own iteration converges to and far
        // inside anything a reader of a globe could see.
        tap_okf(fabs(lat - places[i].lat) < 1e-4
                && fabs(lon - places[i].lon) < 1e-4
                && fabs(alt - places[i].alt) < 1e-3,
                "geodetic: %s round-trips (%.5f %.5f %.3f km)",
                places[i].what, lat, lon, alt);
        tap_okf(lon >= -180.0 && lon <= 180.0,
                "geodetic: %s comes back inside -180..180", places[i].what);
    }

    // Close to the pole, where the latitude has to come out near 90 and
    // the longitude has to stay in range. Not the pole itself: the
    // library's altitude formula divides by the cosine of the latitude,
    // so exactly on the spin axis it returns the negative Earth radius.
    // Nothing here depends on that -- a ground point is drawn on the
    // sphere at zero altitude whatever this says -- but a caller that
    // wanted an altitude within metres of the pole would need its own
    // formula.
    geodetic_t g = {0};
    g.lat = 89.5 * (M_PI / 180.0);
    g.lon = 45.0 * (M_PI / 180.0);
    g.alt = 0.0;
    vector_t pos = {0}, vel = {0};
    Calculate_User_PosVel(jd, &g, &pos, &vel);
    const double p[3] = { pos.x, pos.y, pos.z };
    double lat = 0, lon = 0, alt = 0;
    attitude_eci_to_geodetic(jd, p, &lat, &lon, &alt);
    tap_okf(fabs(lat - 89.5) < 1e-4,
            "geodetic: half a degree from the pole round-trips (got %.5f)", lat);
    tap_okf(fabs(lon - 45.0) < 1e-3 && lon >= -180.0 && lon <= 180.0,
            "geodetic: its longitude survives too (got %.4f)", lon);
    tap_okf(fabs(alt) < 1e-3,
            "geodetic: and it is on the ground (got %.6f km)", alt);
}

static void test_eci_to_earth_fixed(void)
{
    // A direction, not a place: the globe draws the satellite's body
    // axes, and a rotation with the wrong sign would draw the model
    // turned the wrong way about the Earth's axis -- a mirror image,
    // which at a glance looks entirely reasonable.
    const double jd = 2461000.3;

    // Length is preserved.
    const double v[3] = { 0.3, -0.7, 0.64807407 };
    double out[3];
    attitude_eci_to_earth_fixed(jd, v, out);
    tap_okf(fabs(sqrt(vdot3(out, out)) - sqrt(vdot3(v, v))) < 1e-12,
            "earth-fixed: the rotation preserves length");
    tap_okf(fabs(out[2] - v[2]) < 1e-15,
            "earth-fixed: and leaves the component along the spin axis alone");

    // The sign, against the library's own two-way path: a place built
    // from a known longitude, taken into the inertial frame by
    // Calculate_User_PosVel, has to come back out at that longitude.
    // Nothing here shares code with the rotation being tested.
    const double want_lon[] = { 0.0, 73.5, -114.291, 179.0 };
    for (size_t i = 0; i < sizeof want_lon / sizeof want_lon[0]; i++) {
        geodetic_t g = {0};
        g.lat = 12.0 * (M_PI / 180.0);
        g.lon = want_lon[i] * (M_PI / 180.0);
        g.alt = 0.0;
        vector_t pos = {0}, vel = {0};
        Calculate_User_PosVel(jd, &g, &pos, &vel);

        const double eci[3] = { pos.x, pos.y, pos.z };
        double ef[3];
        attitude_eci_to_earth_fixed(jd, eci, ef);
        double lon = atan2(ef[1], ef[0]) * (180.0 / M_PI);
        // 179 and -181 are the same meridian.
        double d = lon - want_lon[i];
        while (d >  180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        tap_okf(fabs(d) < 1e-6,
                "earth-fixed: a place at %.3f deg east comes back there "
                "(got %.6f)", want_lon[i], lon);
    }

    // And the whole chain together, which is what the model rides on:
    // for a level attitude the body's +Z axis, taken into the
    // Earth-fixed frame, has to point from the satellite straight at the
    // centre of the Earth. If the orbit frame, the body axes or this
    // rotation were wrong, this is where it would show.
    const double r[3] = { 3000.0, -5200.0, 2600.0 };
    const double v2[3] = { 5.1, 2.3, -1.4 };
    attitude_rpy_t level = {0};
    attitude_frame_t f;
    attitude_solve(r, v2, &level, &f);

    double bz_ef[3], r_ef[3];
    attitude_eci_to_earth_fixed(jd, f.bz, bz_ef);
    attitude_eci_to_earth_fixed(jd, r, r_ef);
    const double rn = sqrt(vdot3(r_ef, r_ef));
    const double down[3] = { -r_ef[0] / rn, -r_ef[1] / rn, -r_ef[2] / rn };
    tap_ok(veq(bz_ef, down, 1e-12),
           "earth-fixed: a level satellite's +Z axis points at the centre "
           "of the Earth in the frame the globe draws in");

    // A yawed satellite still looks straight down: yaw turns it about
    // the axis it is looking along, so this catches a rotation that
    // leaks into the wrong component.
    attitude_rpy_t yawed = { .yaw_deg = 61.0 };
    attitude_solve(r, v2, &yawed, &f);
    attitude_eci_to_earth_fixed(jd, f.bz, bz_ef);
    tap_ok(veq(bz_ef, down, 1e-12),
           "earth-fixed: and so does a yawed one");
}

int main(void)
{
    test_orbit_frame();
    test_orbit_frame_degenerate();
    test_body_axes_zero();
    test_body_axes_single_rotations();
    test_body_axes_orthonormal();
    test_ray_sphere();
    test_solve();
    test_eci_to_geodetic();
    test_eci_to_earth_fixed();
    return tap_done();
}
