/*

    Simple Satellite Operations  attitude.h

    Where the satellite was looking. The ADCS reports its attitude as
    three angles of the body frame with respect to the orbit frame; this
    turns those angles, plus an orbit, into directions in space and a
    point on the ground -- which is what a viewer needs to draw a line
    from the satellite to whatever was under its camera.

    Frames, spelled out, because everything here depends on them:

      Orbit frame (what the ADCS measures against). The CubeSpace orbit
      reference frame: +Z points at nadir, +Y along the negative orbit
      normal, and +X completes the right-handed set, which for a near-
      circular orbit is very nearly the direction of flight.

      Body frame. At zero roll, pitch and yaw the body axes lie on the
      orbit ones, so +Z body is the face that looks straight down. That
      is the face the ADCS's nadir sensor is on -- the boom camera, the
      one whose pictures frontiersat_camera_viewer shows (see the
      CubeSpace ADCS notes in the flight-firmware repo: "Camera 2: Nadir
      sensor. Pointed at earth. Has a clear view of the boom.").

      The rotation from orbit to body is the aerospace 3-2-1 sequence:
      yaw about Z, then pitch about the new Y, then roll about the new X.

    A caveat worth keeping in mind: the 3-2-1 order and the orbit frame's
    handedness are CubeSpace's documented convention, but we have not
    confirmed them against flight data (doing so needs an independent
    attitude source -- the magnetometer against a field model would be
    the way). Every viewer that draws a pointing direction says on
    screen that it is derived from the ADCS estimate, and the estimate is
    only filled in estimation modes 3 through 6. If the convention later
    turns out to differ, attitude_body_axes is the one function to fix.

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

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <sgp4sdp4.h>

// Mean Earth radius, for putting a ray's end on the ground. The mean
// rather than the equatorial one because the ray is aimed at a sphere,
// and over a whole orbit the mean is the smaller error either way. This
// is the same figure tle_keps uses for apogee and perigee heights.
#define ATTITUDE_EARTH_MEAN_KM 6371.0

// The three angles as the ADCS reports them, in degrees.
typedef struct {
    double roll_deg, pitch_deg, yaw_deg;
} attitude_rpy_t;

// The body axes in the inertial frame, and where the nadir-face axis
// meets the ground.
typedef struct {
    // Body +X, +Y, +Z as unit vectors in the same inertial frame the
    // position and velocity came in.
    double bx[3], by[3], bz[3];

    // Angle between body +Z and nadir, in degrees. Zero means the face
    // is looking straight down; 90 means it is looking at the horizon.
    double off_nadir_deg;

    // Where body +Z meets the Earth. hit is 0 when the axis points away
    // from the Earth or past its limb, and then the rest is untouched.
    int    hit;
    double hit_eci[3];      // the point itself, inertial frame
    double hit_range_km;    // how far along the ray it lies

    // Where the satellite is looking relative to its own flight: the
    // angle from the body +Z axis to the plane containing nadir and the
    // direction of flight, positive to the left of track. Useful for
    // saying "it was looking off to the side" without a picture.
    double cross_track_deg;
} attitude_frame_t;

// The orbit frame's axes in the inertial frame, from an inertial
// position and velocity (km and km/s; SGP4's own output will do). Writes
// unit vectors. A position and velocity that are parallel (which cannot
// happen on a real orbit) leave oy and ox zero.
void attitude_orbit_frame(const double r[3], const double v[3],
                          double ox[3], double oy[3], double oz[3]);

// The body axes in the inertial frame, given the orbit frame's axes and
// the three angles. This is the one place the 3-2-1 convention lives.
void attitude_body_axes(const double ox[3], const double oy[3],
                        const double oz[3], const attitude_rpy_t *rpy,
                        double bx[3], double by[3], double bz[3]);

// Where a ray from p along unit vector d first meets a sphere of radius
// R about the origin. Returns 1 and the distance along the ray in
// *t_km, or 0 if the ray misses the sphere or points away from it.
int attitude_ray_sphere(const double p[3], const double d[3],
                        double R_km, double *t_km);

// Everything above in one call: from an inertial state and the ADCS
// angles, work out the body axes, the off-nadir angle, and the point on
// the ground the nadir face was looking at.
void attitude_solve(const double r[3], const double v[3],
                    const attitude_rpy_t *rpy, attitude_frame_t *out);

// An inertial point as a place on the Earth: latitude and longitude in
// degrees (longitude in -180..180) and altitude in km, at the given
// Julian date. Thin wrapper over sgp4sdp4's Calculate_LatLonAlt, so the
// callers that draw on a globe do not each repeat the unit shuffling.
void attitude_eci_to_geodetic(double jul_utc, const double p[3],
                              double *lat_deg, double *lon_deg,
                              double *alt_km);

// The same rotation, for a direction rather than a place: an inertial
// vector into the Earth-fixed frame at the given Julian date, by turning
// it back through the Greenwich sidereal angle. Length is preserved, so
// a unit vector stays one.
//
// A place can go through attitude_eci_to_geodetic and come back as a
// latitude and longitude, but a direction cannot -- the difference of
// two nearby places is not a direction on a sphere. A body axis drawn
// on the globe needs this.
void attitude_eci_to_earth_fixed(double jul_utc, const double v[3],
                                 double out[3]);

#endif // ATTITUDE_H
