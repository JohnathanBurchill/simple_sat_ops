/*

    Simple Satellite Operations  magcoords.c

    IGRF-14 main field, quasi-dipole latitude / apex longitude by field-line
    tracing, and magnetic local time. See magcoords.h for the conventions.

    The coefficient table below is IGRF-14's 2025.0 main field (degree 13)
    and its 2025-2030 secular variation (degree 8; zero beyond), copied
    from the IAGA file igrf14coeffs.txt
    (https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf14coeffs.txt). The
    field is synthesised with Gauss-normalised associated Legendre
    functions and Schmidt factors folded into the coefficients, the
    arrangement of Davis (2004), "Mathematical Modeling of Earth's
    Magnetic Field".

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

#include "magcoords.h"

#include <math.h>
#include <string.h>

// WGS84 ellipsoid
#define WGS84_A_KM 6378.137
#define WGS84_F    (1.0 / 298.257223563)

// Field-line tracing. Each RK4 step is this fraction of the current
// radius: the dipole part of the field bends on the scale of the radius,
// and the higher-degree terms that bend faster are tiny in amplitude.
// Against steps of 0.005 over 40-83 deg latitude, 0.04 is within 0.0003
// deg of QD latitude and 0.004 deg of apex longitude.
#define TRACE_STEP_FRAC 0.04
// Beyond this radius the field is a dipole to well under 0.1 percent, so
// the trace stops and the dipole formula finishes the job.
#define TRACE_RMAX_KM   (100.0 * IGRF_A_KM)
#define TRACE_MAX_STEPS 20000

#define DEG (M_PI / 180.0)

typedef struct { char gh; int n, m; double v2025, sv; } igrf_coef_t;

static const igrf_coef_t IGRF14[] = {
    {'g',  1,  0,   -29350.0,   12.6},
    {'g',  1,  1,    -1410.3,   10.0},
    {'h',  1,  1,     4545.5,  -21.5},
    {'g',  2,  0,    -2556.2,  -11.2},
    {'g',  2,  1,     2950.9,   -5.3},
    {'h',  2,  1,    -3133.6,  -27.3},
    {'g',  2,  2,     1648.7,   -8.3},
    {'h',  2,  2,     -814.2,  -11.1},
    {'g',  3,  0,     1360.9,   -1.5},
    {'g',  3,  1,    -2404.2,   -4.4},
    {'h',  3,  1,      -56.9,    3.8},
    {'g',  3,  2,     1243.8,    0.4},
    {'h',  3,  2,      237.6,   -0.2},
    {'g',  3,  3,      453.4,  -15.6},
    {'h',  3,  3,     -549.6,   -3.9},
    {'g',  4,  0,      894.7,   -1.7},
    {'g',  4,  1,      799.6,   -2.3},
    {'h',  4,  1,      278.6,   -1.3},
    {'g',  4,  2,       55.8,   -5.8},
    {'h',  4,  2,     -134.0,    4.1},
    {'g',  4,  3,     -281.1,    5.4},
    {'h',  4,  3,      212.0,    1.6},
    {'g',  4,  4,       12.0,   -6.8},
    {'h',  4,  4,     -375.4,   -4.1},
    {'g',  5,  0,     -232.9,    0.6},
    {'g',  5,  1,      369.0,    1.3},
    {'h',  5,  1,       45.3,   -0.5},
    {'g',  5,  2,      187.2,    0.0},
    {'h',  5,  2,      220.0,    2.1},
    {'g',  5,  3,     -138.7,    0.7},
    {'h',  5,  3,     -122.9,    0.5},
    {'g',  5,  4,     -141.9,    2.3},
    {'h',  5,  4,       42.9,    1.7},
    {'g',  5,  5,       20.9,    1.0},
    {'h',  5,  5,      106.2,    1.9},
    {'g',  6,  0,       64.3,   -0.2},
    {'g',  6,  1,       63.8,   -0.3},
    {'h',  6,  1,      -18.4,    0.3},
    {'g',  6,  2,       76.7,    0.8},
    {'h',  6,  2,       16.8,   -1.6},
    {'g',  6,  3,     -115.7,    1.2},
    {'h',  6,  3,       48.9,   -0.4},
    {'g',  6,  4,      -40.9,   -0.8},
    {'h',  6,  4,      -59.8,    0.8},
    {'g',  6,  5,       14.9,    0.4},
    {'h',  6,  5,       10.9,    0.7},
    {'g',  6,  6,      -60.8,    0.9},
    {'h',  6,  6,       72.8,    0.9},
    {'g',  7,  0,       79.6,   -0.1},
    {'g',  7,  1,      -76.9,   -0.1},
    {'h',  7,  1,      -48.9,    0.6},
    {'g',  7,  2,       -8.8,   -0.1},
    {'h',  7,  2,      -14.4,    0.5},
    {'g',  7,  3,       59.3,    0.5},
    {'h',  7,  3,       -1.0,   -0.7},
    {'g',  7,  4,       15.8,   -0.1},
    {'h',  7,  4,       23.5,    0.0},
    {'g',  7,  5,        2.5,   -0.8},
    {'h',  7,  5,       -7.4,   -0.9},
    {'g',  7,  6,      -11.2,   -0.8},
    {'h',  7,  6,      -25.1,    0.5},
    {'g',  7,  7,       14.3,    0.9},
    {'h',  7,  7,       -2.2,   -0.3},
    {'g',  8,  0,       23.1,   -0.1},
    {'g',  8,  1,       10.9,    0.2},
    {'h',  8,  1,        7.2,   -0.3},
    {'g',  8,  2,      -17.5,    0.0},
    {'h',  8,  2,      -12.6,    0.4},
    {'g',  8,  3,        2.0,    0.4},
    {'h',  8,  3,       11.5,   -0.3},
    {'g',  8,  4,      -21.8,   -0.1},
    {'h',  8,  4,       -9.7,    0.4},
    {'g',  8,  5,       16.9,    0.3},
    {'h',  8,  5,       12.7,   -0.5},
    {'g',  8,  6,       14.9,    0.1},
    {'h',  8,  6,        0.7,   -0.6},
    {'g',  8,  7,      -16.8,    0.0},
    {'h',  8,  7,       -5.2,    0.3},
    {'g',  8,  8,        1.0,    0.3},
    {'h',  8,  8,        3.9,    0.2},
    {'g',  9,  0,        4.7,    0.0},
    {'g',  9,  1,        8.0,    0.0},
    {'h',  9,  1,      -24.8,    0.0},
    {'g',  9,  2,        3.0,    0.0},
    {'h',  9,  2,       12.1,    0.0},
    {'g',  9,  3,       -0.2,    0.0},
    {'h',  9,  3,        8.3,    0.0},
    {'g',  9,  4,       -2.5,    0.0},
    {'h',  9,  4,       -3.4,    0.0},
    {'g',  9,  5,      -13.1,    0.0},
    {'h',  9,  5,       -5.3,    0.0},
    {'g',  9,  6,        2.4,    0.0},
    {'h',  9,  6,        7.2,    0.0},
    {'g',  9,  7,        8.6,    0.0},
    {'h',  9,  7,       -0.6,    0.0},
    {'g',  9,  8,       -8.7,    0.0},
    {'h',  9,  8,        0.8,    0.0},
    {'g',  9,  9,      -12.8,    0.0},
    {'h',  9,  9,        9.8,    0.0},
    {'g', 10,  0,       -1.3,    0.0},
    {'g', 10,  1,       -6.4,    0.0},
    {'h', 10,  1,        3.3,    0.0},
    {'g', 10,  2,        0.2,    0.0},
    {'h', 10,  2,        0.1,    0.0},
    {'g', 10,  3,        2.0,    0.0},
    {'h', 10,  3,        2.5,    0.0},
    {'g', 10,  4,       -1.0,    0.0},
    {'h', 10,  4,        5.4,    0.0},
    {'g', 10,  5,       -0.5,    0.0},
    {'h', 10,  5,       -9.0,    0.0},
    {'g', 10,  6,       -0.9,    0.0},
    {'h', 10,  6,        0.4,    0.0},
    {'g', 10,  7,        1.5,    0.0},
    {'h', 10,  7,       -4.2,    0.0},
    {'g', 10,  8,        0.9,    0.0},
    {'h', 10,  8,       -3.8,    0.0},
    {'g', 10,  9,       -2.6,    0.0},
    {'h', 10,  9,        0.9,    0.0},
    {'g', 10, 10,       -3.9,    0.0},
    {'h', 10, 10,       -9.0,    0.0},
    {'g', 11,  0,        3.0,    0.0},
    {'g', 11,  1,       -1.4,    0.0},
    {'h', 11,  1,        0.0,    0.0},
    {'g', 11,  2,       -2.5,    0.0},
    {'h', 11,  2,        2.8,    0.0},
    {'g', 11,  3,        2.4,    0.0},
    {'h', 11,  3,       -0.6,    0.0},
    {'g', 11,  4,       -0.6,    0.0},
    {'h', 11,  4,        0.1,    0.0},
    {'g', 11,  5,        0.0,    0.0},
    {'h', 11,  5,        0.5,    0.0},
    {'g', 11,  6,       -0.6,    0.0},
    {'h', 11,  6,       -0.3,    0.0},
    {'g', 11,  7,       -0.1,    0.0},
    {'h', 11,  7,       -1.2,    0.0},
    {'g', 11,  8,        1.1,    0.0},
    {'h', 11,  8,       -1.7,    0.0},
    {'g', 11,  9,       -1.0,    0.0},
    {'h', 11,  9,       -2.9,    0.0},
    {'g', 11, 10,       -0.1,    0.0},
    {'h', 11, 10,       -1.8,    0.0},
    {'g', 11, 11,        2.6,    0.0},
    {'h', 11, 11,       -2.3,    0.0},
    {'g', 12,  0,       -2.0,    0.0},
    {'g', 12,  1,       -0.1,    0.0},
    {'h', 12,  1,       -1.2,    0.0},
    {'g', 12,  2,        0.4,    0.0},
    {'h', 12,  2,        0.6,    0.0},
    {'g', 12,  3,        1.2,    0.0},
    {'h', 12,  3,        1.0,    0.0},
    {'g', 12,  4,       -1.2,    0.0},
    {'h', 12,  4,       -1.5,    0.0},
    {'g', 12,  5,        0.6,    0.0},
    {'h', 12,  5,        0.0,    0.0},
    {'g', 12,  6,        0.5,    0.0},
    {'h', 12,  6,        0.6,    0.0},
    {'g', 12,  7,        0.5,    0.0},
    {'h', 12,  7,       -0.2,    0.0},
    {'g', 12,  8,       -0.1,    0.0},
    {'h', 12,  8,        0.8,    0.0},
    {'g', 12,  9,       -0.5,    0.0},
    {'h', 12,  9,        0.1,    0.0},
    {'g', 12, 10,       -0.2,    0.0},
    {'h', 12, 10,       -0.9,    0.0},
    {'g', 12, 11,       -1.2,    0.0},
    {'h', 12, 11,        0.1,    0.0},
    {'g', 12, 12,       -0.7,    0.0},
    {'h', 12, 12,        0.2,    0.0},
    {'g', 13,  0,        0.2,    0.0},
    {'g', 13,  1,       -0.9,    0.0},
    {'h', 13,  1,       -0.9,    0.0},
    {'g', 13,  2,        0.6,    0.0},
    {'h', 13,  2,        0.7,    0.0},
    {'g', 13,  3,        0.7,    0.0},
    {'h', 13,  3,        1.2,    0.0},
    {'g', 13,  4,       -0.2,    0.0},
    {'h', 13,  4,       -0.3,    0.0},
    {'g', 13,  5,        0.5,    0.0},
    {'h', 13,  5,       -1.3,    0.0},
    {'g', 13,  6,        0.1,    0.0},
    {'h', 13,  6,       -0.1,    0.0},
    {'g', 13,  7,        0.7,    0.0},
    {'h', 13,  7,        0.2,    0.0},
    {'g', 13,  8,        0.0,    0.0},
    {'h', 13,  8,       -0.2,    0.0},
    {'g', 13,  9,        0.3,    0.0},
    {'h', 13,  9,        0.5,    0.0},
    {'g', 13, 10,        0.2,    0.0},
    {'h', 13, 10,        0.6,    0.0},
    {'g', 13, 11,        0.4,    0.0},
    {'h', 13, 11,       -0.6,    0.0},
    {'g', 13, 12,       -0.5,    0.0},
    {'h', 13, 12,       -0.3,    0.0},
    {'g', 13, 13,       -0.4,    0.0},
    {'h', 13, 13,       -0.5,    0.0},
};

double igrf_decimal_year(double jd)
{
    // JD 2451544.5 is 2000-01-01T00:00Z
    return 2000.0 + (jd - 2451544.5) / 365.25;
}

static void cross3(const double a[3], const double b[3], double c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

static double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double norm3(const double a[3])
{
    return sqrt(dot3(a, a));
}

void igrf_init(igrf_model_t *m, double decimal_year)
{
    memset(m, 0, sizeof *m);
    m->year = decimal_year;
    double dt = decimal_year - 2025.0;
    for (size_t i = 0; i < sizeof IGRF14 / sizeof IGRF14[0]; ++i) {
        const igrf_coef_t *c = &IGRF14[i];
        double v = c->v2025 + c->sv * dt;
        if (c->gh == 'g') m->g[c->n][c->m] = v;
        else              m->h[c->n][c->m] = v;
    }

    // Centered-dipole axes from the degree-1 terms. The dipole moment
    // points along (g11, h11, g10); the north geomagnetic pole is opposite.
    double pole[3] = { -m->g[1][1], -m->h[1][1], -m->g[1][0] };
    double pn = norm3(pole);
    for (int k = 0; k < 3; ++k) m->dip_z[k] = pole[k] / pn;
    const double zgeo[3] = { 0.0, 0.0, 1.0 };
    cross3(zgeo, m->dip_z, m->dip_y);
    double yn = norm3(m->dip_y);
    for (int k = 0; k < 3; ++k) m->dip_y[k] /= yn;
    cross3(m->dip_y, m->dip_z, m->dip_x);

    // Fold the Schmidt semi-normalisation into the coefficients so the
    // synthesis can use the simpler Gauss-normalised Legendre recursion.
    double S[IGRF_NMAX + 1][IGRF_NMAX + 1] = {{0}};
    S[0][0] = 1.0;
    for (int n = 1; n <= IGRF_NMAX; ++n) {
        S[n][0] = S[n - 1][0] * (2.0 * n - 1.0) / n;
        for (int k = 1; k <= n; ++k)
            S[n][k] = S[n][k - 1]
                    * sqrt((n - k + 1.0) * (k == 1 ? 2.0 : 1.0) / (n + k));
    }
    for (int n = 1; n <= IGRF_NMAX; ++n)
        for (int k = 0; k <= n; ++k) {
            m->g[n][k] *= S[n][k];
            m->h[n][k] *= S[n][k];
        }
}

void igrf_field_sph(const igrf_model_t *m, double r_km, double theta,
                    double phi, int nmax,
                    double *br, double *btheta, double *bphi)
{
    if (nmax > IGRF_NMAX) nmax = IGRF_NMAX;
    double c = cos(theta), s = sin(theta);
    // The longitude term carries a 1/sin(theta); every P(n,m) with m > 0
    // carries sin^m, so the ratio is finite and a floor only guards the
    // exact pole.
    if (s < 1e-12) s = 1e-12;

    // Gauss-normalised P(n,m)(cos theta) and dP/dtheta. Zero-filled so the
    // P(n-2,m) term reads 0 where m > n-2 (its factor K is also 0 there).
    double P[IGRF_NMAX + 1][IGRF_NMAX + 1] = {{0}};
    double dP[IGRF_NMAX + 1][IGRF_NMAX + 1] = {{0}};
    P[0][0] = 1.0;
    for (int n = 1; n <= nmax; ++n) {
        for (int k = 0; k <= n; ++k) {
            if (k == n) {
                P[n][n] = s * P[n - 1][n - 1];
                dP[n][n] = s * dP[n - 1][n - 1] + c * P[n - 1][n - 1];
            } else if (n == 1) {
                P[1][0] = c;
                dP[1][0] = -s;
            } else {
                double K = ((n - 1.0) * (n - 1.0) - (double) k * k)
                         / ((2.0 * n - 1.0) * (2.0 * n - 3.0));
                P[n][k] = c * P[n - 1][k] - K * P[n - 2][k];
                dP[n][k] = c * dP[n - 1][k] - s * P[n - 1][k] - K * dP[n - 2][k];
            }
        }
    }

    double cosm[IGRF_NMAX + 1], sinm[IGRF_NMAX + 1];
    for (int k = 0; k <= nmax; ++k) {
        cosm[k] = cos(k * phi);
        sinm[k] = sin(k * phi);
    }

    double ar = IGRF_A_KM / r_km;
    double arn = ar * ar;
    double sr = 0.0, st = 0.0, sp = 0.0;
    for (int n = 1; n <= nmax; ++n) {
        // (a/r)^(n+2)
        arn *= ar;
        for (int k = 0; k <= n; ++k) {
            double gc = m->g[n][k] * cosm[k] + m->h[n][k] * sinm[k];
            sr += (n + 1.0) * arn * gc * P[n][k];
            st -= arn * gc * dP[n][k];
            sp += arn * k * (m->g[n][k] * sinm[k] - m->h[n][k] * cosm[k]) * P[n][k];
        }
    }
    *br = sr;
    *btheta = st;
    *bphi = sp / s;
}

void igrf_field_ecef(const igrf_model_t *m, const double x_km[3], int nmax,
                     double b[3])
{
    double r = norm3(x_km);
    double theta = acos(x_km[2] / r);
    double phi = atan2(x_km[1], x_km[0]);
    double br, bt, bp;
    igrf_field_sph(m, r, theta, phi, nmax, &br, &bt, &bp);
    double st = sin(theta), ct = cos(theta), sp = sin(phi), cp = cos(phi);
    double horiz = br * st + bt * ct;
    b[0] = horiz * cp - bp * sp;
    b[1] = horiz * sp + bp * cp;
    b[2] = br * ct - bt * st;
}

void magc_geodetic_to_ecef(double lat_deg, double lon_deg, double alt_km,
                           double x_km[3])
{
    double e2 = WGS84_F * (2.0 - WGS84_F);
    double sl = sin(lat_deg * DEG), cl = cos(lat_deg * DEG);
    double N = WGS84_A_KM / sqrt(1.0 - e2 * sl * sl);
    x_km[0] = (N + alt_km) * cl * cos(lon_deg * DEG);
    x_km[1] = (N + alt_km) * cl * sin(lon_deg * DEG);
    x_km[2] = (N * (1.0 - e2) + alt_km) * sl;
}

void magc_ecef_to_geodetic(const double x_km[3], double *lat_deg,
                           double *lon_deg, double *alt_km)
{
    double e2 = WGS84_F * (2.0 - WGS84_F);
    double p = hypot(x_km[0], x_km[1]);
    double lat = atan2(x_km[2], p * (1.0 - e2));
    for (int i = 0; i < 8; ++i) {
        double sl = sin(lat);
        double N = WGS84_A_KM / sqrt(1.0 - e2 * sl * sl);
        lat = atan2(x_km[2] + e2 * N * sl, p);
    }
    double sl = sin(lat);
    *lat_deg = lat / DEG;
    *lon_deg = atan2(x_km[1], x_km[0]) / DEG;
    // Height along the normal; valid at the poles, unlike p/cos(lat) - N
    *alt_km = p * cos(lat) + x_km[2] * sl
            - WGS84_A_KM * sqrt(1.0 - e2 * sl * sl);
}

void magc_dipole_latlon(const igrf_model_t *m, const double x[3],
                        double *lat_deg, double *lon_deg)
{
    double r = norm3(x);
    double z = dot3(x, m->dip_z) / r;
    if (z > 1.0) z = 1.0;
    if (z < -1.0) z = -1.0;
    *lat_deg = asin(z) / DEG;
    *lon_deg = atan2(dot3(x, m->dip_y), dot3(x, m->dip_x)) / DEG;
}

// Unit tangent sgn * B / |B| at x.
static void trace_dir(const igrf_model_t *m, const double x[3], double sgn,
                      double d[3])
{
    double b[3];
    igrf_field_ecef(m, x, IGRF_NMAX, b);
    double bn = norm3(b);
    for (int k = 0; k < 3; ++k) d[k] = sgn * b[k] / bn;
}

static void rk4_step(const igrf_model_t *m, const double x[3], double h,
                     double sgn, double out[3])
{
    double k1[3], k2[3], k3[3], k4[3], t[3];
    trace_dir(m, x, sgn, k1);
    for (int k = 0; k < 3; ++k) t[k] = x[k] + 0.5 * h * k1[k];
    trace_dir(m, t, sgn, k2);
    for (int k = 0; k < 3; ++k) t[k] = x[k] + 0.5 * h * k2[k];
    trace_dir(m, t, sgn, k3);
    for (int k = 0; k < 3; ++k) t[k] = x[k] + h * k3[k];
    trace_dir(m, t, sgn, k4);
    for (int k = 0; k < 3; ++k)
        out[k] = x[k] + h / 6.0 * (k1[k] + 2.0 * k2[k] + 2.0 * k3[k] + k4[k]);
}

void magc_qd(const igrf_model_t *m, const double x_km[3], double alt_km,
             double *qdlat_deg, double *apex_lon_deg, double *apex_height_km)
{
    double b[3];
    igrf_field_ecef(m, x_km, IGRF_NMAX, b);
    double r0 = norm3(x_km);
    double br = dot3(b, x_km) / r0;
    // The field points down in the northern magnetic hemisphere. Trace
    // along sgn * B, the direction that climbs away from the Earth.
    int north = br < 0.0;
    double sgn = br > 0.0 ? 1.0 : -1.0;

    double prev[3] = { x_km[0], x_km[1], x_km[2] };
    double cur[3] = { x_km[0], x_km[1], x_km[2] };
    double rc = r0, rprev = r0;
    int have_prev = 0;
    double apex[3] = { x_km[0], x_km[1], x_km[2] };
    // Shrinks only while the first step overshoots the apex (a point near
    // the magnetic equator, on a field line shorter than one step)
    double frac = TRACE_STEP_FRAC;

    for (int i = 0; i < TRACE_MAX_STEPS; ++i) {
        double nxt[3];
        rk4_step(m, cur, frac * rc, sgn, nxt);
        double rn = norm3(nxt);
        if (rn > TRACE_RMAX_KM) {
            // Far out the line is a dipole line r = L cos^2(lat), whose
            // apex is in the dipole equator at the same dipole longitude.
            double dlat, dlon;
            magc_dipole_latlon(m, nxt, &dlat, &dlon);
            double cl = cos(dlat * DEG);
            double L = rn / (cl * cl);
            for (int k = 0; k < 3; ++k)
                apex[k] = L * (cos(dlon * DEG) * m->dip_x[k]
                             + sin(dlon * DEG) * m->dip_y[k]);
            break;
        }
        if (rn <= rc && !have_prev && frac * rc > 0.01) {
            frac *= 0.5;
            continue;
        }
        if (rn <= rc) {
            // Passed the top. Fit a parabola through the last three points
            // (equally spaced to within the slow change of the step) and
            // take the position at its peak radius. A start within 10 m of
            // the apex stands in for it.
            if (have_prev) {
                double curv = 2.0 * rc - rprev - rn;
                double u = curv > 0.0 ? (rn - rprev) / (2.0 * curv) : 0.0;
                for (int k = 0; k < 3; ++k)
                    apex[k] = cur[k] + 0.5 * u * (nxt[k] - prev[k])
                            + 0.5 * u * u * (nxt[k] - 2.0 * cur[k] + prev[k]);
            }
            break;
        }
        memcpy(prev, cur, sizeof prev);
        memcpy(cur, nxt, sizeof cur);
        rprev = rc;
        rc = rn;
        have_prev = 1;
    }

    double alat_dip, alon;
    magc_dipole_latlon(m, apex, &alat_dip, &alon);
    double glat, glon, hA;
    magc_ecef_to_geodetic(apex, &glat, &glon, &hA);
    if (hA < alt_km) hA = alt_km;
    double ratio = (QD_RE_KM + alt_km) / (QD_RE_KM + hA);
    double lat = acos(sqrt(ratio)) / DEG;
    *qdlat_deg = north ? lat : -lat;
    *apex_lon_deg = alon;
    if (apex_height_km) *apex_height_km = hA;
}

double magc_mlt(const igrf_model_t *m, double apex_lon_deg,
                const double sun_ecef[3])
{
    double slat, slon;
    magc_dipole_latlon(m, sun_ecef, &slat, &slon);
    double mlt = fmod((180.0 + apex_lon_deg - slon) / 15.0, 24.0);
    if (mlt < 0.0) mlt += 24.0;
    return mlt;
}
