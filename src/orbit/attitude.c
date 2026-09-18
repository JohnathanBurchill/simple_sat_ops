/*

    Simple Satellite Operations  attitude.c

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

#include "attitude.h"

#include <math.h>
#include <string.h>

static double vdot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vcross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// Scale to unit length. A zero-length input is left alone, which is how
// a degenerate orbit frame comes back as zeros rather than as NaNs.
static void vnorm(double v[3])
{
    double n = sqrt(vdot(v, v));
    if (n <= 0.0) return;
    v[0] /= n; v[1] /= n; v[2] /= n;
}

void attitude_orbit_frame(const double r[3], const double v[3],
                          double ox[3], double oy[3], double oz[3])
{
    memset(ox, 0, 3 * sizeof *ox);
    memset(oy, 0, 3 * sizeof *oy);
    memset(oz, 0, 3 * sizeof *oz);

    // Z at nadir: straight down the position vector.
    oz[0] = -r[0]; oz[1] = -r[1]; oz[2] = -r[2];
    vnorm(oz);

    // Y along the negative orbit normal. r x v is the normal, so the
    // orbit frame's Y is the other way.
    double n[3];
    vcross(r, v, n);
    double nn = sqrt(vdot(n, n));
    if (nn <= 0.0) return;   // r parallel to v: no orbit plane to work in
    oy[0] = -n[0] / nn; oy[1] = -n[1] / nn; oy[2] = -n[2] / nn;

    // X completes the right-handed set, which puts it along the flight
    // direction for a circular orbit and within a fraction of a degree
    // of it for one as round as this satellite's.
    vcross(oy, oz, ox);
    vnorm(ox);
}

void attitude_body_axes(const double ox[3], const double oy[3],
                        const double oz[3], const attitude_rpy_t *rpy,
                        double bx[3], double by[3], double bz[3])
{
    const double d2r = M_PI / 180.0;
    const double cr = cos(rpy->roll_deg  * d2r), sr = sin(rpy->roll_deg  * d2r);
    const double cp = cos(rpy->pitch_deg * d2r), sp = sin(rpy->pitch_deg * d2r);
    const double cy = cos(rpy->yaw_deg   * d2r), sy = sin(rpy->yaw_deg   * d2r);

    // The 3-2-1 rotation matrix M taking a vector's orbit-frame
    // components to its body-frame ones, M = Rx(roll) Ry(pitch) Rz(yaw).
    // A body axis expressed in the orbit frame is then a row of M: body
    // +X is M's first row, +Y its second, +Z its third. (Because
    // e_orbit = M^T e_body, and M^T's columns are M's rows.)
    const double m[3][3] = {
        { cp * cy,                cp * sy,               -sp     },
        { sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp },
        { cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp },
    };

    // Each row, read back out into the inertial frame through the orbit
    // frame's own axes.
    double *b[3] = { bx, by, bz };
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            b[i][k] = m[i][0] * ox[k] + m[i][1] * oy[k] + m[i][2] * oz[k];
        }
        vnorm(b[i]);
    }
}

int attitude_ray_sphere(const double p[3], const double d[3],
                        double R_km, double *t_km)
{
    // |p + t d|^2 = R^2 with d a unit vector, so t^2 + 2 (p.d) t +
    // (p.p - R^2) = 0.
    const double b = vdot(p, d);
    const double c = vdot(p, p) - R_km * R_km;
    const double disc = b * b - c;
    if (disc < 0.0) return 0;              // the ray misses entirely
    const double t = -b - sqrt(disc);      // the near intersection
    if (t <= 0.0) return 0;                // behind the start, or inside
    if (t_km != NULL) *t_km = t;
    return 1;
}

void attitude_solve(const double r[3], const double v[3],
                    const attitude_rpy_t *rpy, attitude_frame_t *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof *out);

    double ox[3], oy[3], oz[3];
    attitude_orbit_frame(r, v, ox, oy, oz);
    attitude_body_axes(ox, oy, oz, rpy, out->bx, out->by, out->bz);

    // How far off straight down the nadir face was looking. Clamped
    // before the arccosine: two unit vectors can dot to 1 + 1e-16.
    double c = vdot(out->bz, oz);
    if (c >  1.0) c =  1.0;
    if (c < -1.0) c = -1.0;
    out->off_nadir_deg = acos(c) * (180.0 / M_PI);

    // Which side of the flight path it was looking. The component along
    // the orbit frame's Y is the across-track one, and Y is the negative
    // orbit normal, so a positive component is to the right of flight;
    // report it the other way round so positive reads as left, the way
    // a bearing off the nose does.
    out->cross_track_deg = -asin(vdot(out->bz, oy)) * (180.0 / M_PI);

    double t = 0.0;
    if (attitude_ray_sphere(r, out->bz, ATTITUDE_EARTH_MEAN_KM, &t)) {
        out->hit = 1;
        out->hit_range_km = t;
        for (int k = 0; k < 3; k++) out->hit_eci[k] = r[k] + t * out->bz[k];
    }
}

void attitude_eci_to_earth_fixed(double jul_utc, const double v[3],
                                 double out[3])
{
    // Calculate_LatLonAlt takes the Earth-fixed longitude to be the
    // inertial right ascension less the Greenwich sidereal angle, so the
    // Earth-fixed frame is the inertial one turned by -theta about Z.
    // Turning a vector by that same angle is this.
    const double th = ThetaG_JD(jul_utc);
    const double c = cos(th), s = sin(th);
    const double x = v[0], y = v[1];
    out[0] =  x * c + y * s;
    out[1] = -x * s + y * c;
    out[2] =  v[2];
}

void attitude_eci_to_geodetic(double jul_utc, const double p[3],
                              double *lat_deg, double *lon_deg,
                              double *alt_km)
{
    vector_t pos = {0};
    geodetic_t geo = {0};
    pos.x = p[0]; pos.y = p[1]; pos.z = p[2];
    pos.w = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    Calculate_LatLonAlt(jul_utc, &pos, &geo);
    double lon = geo.lon * (180.0 / M_PI);
    // Calculate_LatLonAlt returns 0..2pi; the globe wants -180..180.
    while (lon > 180.0)  lon -= 360.0;
    while (lon < -180.0) lon += 360.0;
    if (lat_deg != NULL) *lat_deg = geo.lat * (180.0 / M_PI);
    if (lon_deg != NULL) *lon_deg = lon;
    if (alt_km  != NULL) *alt_km  = geo.alt;
}
