/*

    Simple Satellite Operations  unit_tests/magcoords_selftest.c

    Tests for src/orbit/magcoords.c: the IGRF-14 field, quasi-dipole (QD)
    latitude and apex longitude by field-line tracing, and magnetic local
    time.

    Field oracle: IRI-2026's Fortran IGRF (igrf.for FELDG, with its own
    igrf2025.dat / igrf2025s.dat), run once on 2026-10-06 at decimal year
    2026.75 with the WGS84 ellipsoid, geodetic north/east/down in nT. An
    independent implementation of the same IGRF-14 coefficients.

    Coordinate oracle: apexpy 2.1.1 (IGRF-14), built from source with
    gfortran-15 and run once on 2026-10-06 at decimal year 2026.75. Per
    point, its Fortran tracer (fortranapex.apex) gave the apex radius A
    (geodetic apex height 6378.137*(A-1)) and the apex longitude. The QD
    latitude column is acos(sqrt((6371.009 + h)/(6371.009 + hA))) from
    that apex height, the definition apexpy's own fitted QD uses. MLT rows
    are Apex.mlon2mlt at 2026-10-06T12:00Z with apexpy's subsolar point.

    apexpy is not exact. Its IGRF divides positions by 6371.009 km rather
    than IGRF's 6371.2 km reference radius (its field is about 0.009
    percent weak), and it traces with a fixed step of about 1000 km. Run on
    a dipole-only coefficient file, its apex heights came out 0.08 to 0.14
    percent above the analytic dipole apex, where this tracer lands within
    0.1 km. The apexpy tolerances below allow for that.

    Analytic oracles: a dipole-only model, whose field lines are
    r = L cos^2(dipole latitude) with the apex in the dipole equator at the
    starting dipole longitude, and WGS84 points of known geodetic height.

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
#include "tap.h"

#include <math.h>
#include <stdio.h>

#define DEG (M_PI / 180.0)

typedef struct {
    double lat, lon, alt;
    double bn, be, bd;
    double hA, alon, qd;
} ref_t;

static const ref_t REF[] = {
    {  51.08, -114.13,   0.0,  15846.81,  3747.40,  52710.20,    15580.4,  -50.1914,  57.4026 },
    {  51.08, -114.13, 500.0,  12658.06,  2628.35,  41493.47,    17261.9,  -49.9070,  57.3707 },
    {  75.00, -100.00, 500.0,   3148.16,  -224.56,  46123.89,   299525.3,  -40.8632,  81.3804 },
    {  78.00,   15.00, 500.0,   5781.15,   889.78,  44661.01,   106985.5,  106.6685,  75.7473 },
    {  65.00, -147.00, 500.0,   9748.89,  2439.65,  43741.71,    33032.2,  -90.1698,  65.3178 },
    { -70.00,  130.00, 500.0,  -3406.27, -1707.57, -50684.81,   719125.1, -140.2447, -84.4153 },
    { -65.00,  140.00, 500.0,    -23.73,  1171.69, -51884.71,   149310.1, -127.3434, -77.8727 },
    { -75.00,    0.00, 500.0,  13124.47, -5872.43, -30881.25,    32719.9,   43.7140, -65.2128 },
    { -60.00,  -60.00, 500.0,  14872.64,  2178.89, -22971.56,     8648.2,   10.4480, -47.4391 },
    { -30.00,  -50.00, 500.0,  13300.61, -3764.98, -12603.92,     1980.5,   16.4051, -24.8999 },
    {  10.00,    0.00, 500.0,  25289.94,  -886.21,  -1187.82,      503.0,   74.7105,  -1.2029 },
    {  60.00,  100.00, 800.0,   9543.11,  -293.76,  41110.36,    16160.2,  174.4645,  55.6564 },
    { -55.00,  170.00, 500.0,   8860.41,  6688.53, -47602.28,    24705.1,  -97.3373, -61.9517 },
    {  82.00,  -60.00, 500.0,   2519.29, -1637.00,  45589.66,  1638903.5,   59.3111,  86.2948 },
};
#define NREF ((int) (sizeof REF / sizeof REF[0]))

static double wrap180(double d)
{
    d = fmod(d + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

static void test_geodetic(void)
{
    // A point on the equator at the WGS84 equatorial radius, and one on
    // the axis at the polar radius b = a(1-f), are both at height 0.
    double x0[3] = { 6378.137, 0.0, 0.0 };
    double x1[3] = { 0.0, 0.0, 6378.137 * (1.0 - 1.0 / 298.257223563) + 100.0 };
    double lat, lon, h;
    magc_ecef_to_geodetic(x0, &lat, &lon, &h);
    tap_okf(fabs(lat) < 1e-9 && fabs(h) < 1e-9,
            "equator point at the equatorial radius: lat %.2e h %.2e km", lat, h);
    magc_ecef_to_geodetic(x1, &lat, &lon, &h);
    tap_okf(fabs(lat - 90.0) < 1e-9 && fabs(h - 100.0) < 1e-6,
            "point 100 km above the pole: lat %.9f h %.6f km", lat, h);
    // Round trip at a mid latitude, where the normal and the radius differ.
    double x[3];
    magc_geodetic_to_ecef(51.08, -114.13, 512.3, x);
    magc_ecef_to_geodetic(x, &lat, &lon, &h);
    tap_okf(fabs(lat - 51.08) < 1e-9 && fabs(lon + 114.13) < 1e-9
            && fabs(h - 512.3) < 1e-6,
            "geodetic round trip: %.9f %.9f %.6f", lat, lon, h);
}

static void test_field(const igrf_model_t *m)
{
    // IRI computes in single precision; 0.2 nT is about 4e-6 of the field.
    // A wrong Schmidt factor, sign, coefficient or reference radius (the
    // 6371.009 km slip costs 5 nT here) shifts the field by more.
    double worst = 0.0;
    for (int i = 0; i < NREF; ++i) {
        const ref_t *r = &REF[i];
        double x[3], b[3];
        magc_geodetic_to_ecef(r->lat, r->lon, r->alt, x);
        igrf_field_ecef(m, x, IGRF_NMAX, b);
        double sl = sin(r->lat * DEG), cl = cos(r->lat * DEG);
        double so = sin(r->lon * DEG), co = cos(r->lon * DEG);
        double bn = -sl * co * b[0] - sl * so * b[1] + cl * b[2];
        double be = -so * b[0] + co * b[1];
        double bd = -cl * co * b[0] - cl * so * b[1] - sl * b[2];
        double d = fmax(fabs(bn - r->bn), fmax(fabs(be - r->be), fabs(bd - r->bd)));
        if (d > worst) worst = d;
        tap_okf(d < 0.2, "IGRF at %.2f %.2f %.0f km: N %.2f E %.2f D %.2f nT (IRI %.2f %.2f %.2f)",
                r->lat, r->lon, r->alt, bn, be, bd, r->bn, r->be, r->bd);
    }
    tap_diag("worst field difference from IRI: %.3f nT", worst);
}

static void test_qd(const igrf_model_t *m)
{
    // Allow for apexpy's own error (see the header): its apex heights sit
    // 0.08 to 0.14 percent high on a dipole, so ours may be up to 0.25
    // percent below and 0.05 percent above. A 0.15 percent apex error
    // moves QD latitude by 0.02 deg at 57 deg, so QD agrees within 0.03 deg
    // and apex longitude within 0.02 deg. A sign or frame slip is degrees.
    for (int i = 0; i < NREF; ++i) {
        const ref_t *r = &REF[i];
        double x[3], qd, alon, hA;
        magc_geodetic_to_ecef(r->lat, r->lon, r->alt, x);
        magc_qd(m, x, r->alt, &qd, &alon, &hA);
        double dl = fabs(qd - r->qd);
        double dlon = fabs(wrap180(alon - r->alon));
        double rel = (hA - r->hA) / r->hA;
        // Beyond an apex radius of about 185,000 km (QD above about 79 deg
        // at 500 km) apexpy's 200 steps of about 1300 km run out before the
        // apex, and it finishes with a dipole formula from there; ours
        // traces on to 100 Earth radii. QD then agrees within 0.05 deg and
        // apex longitude within 0.5 deg, under 4 km of arc that close to
        // the pole.
        int far = fabs(r->qd) > 79.0;
        int ok = far ? (dl < 0.05 && dlon < 0.5)
                     : (dl < 0.03 && dlon < 0.02 && rel > -2.5e-3 && rel < 5e-4);
        tap_okf(ok, "QD at %.2f %.2f %.0f km: lat %.4f (apexpy %.4f) apex lon %.4f (%.4f) apex h %.1f (%.1f) km",
                r->lat, r->lon, r->alt, qd, r->qd, alon, r->alon, hA, r->hA);
    }
}

static void test_dipole(const igrf_model_t *full)
{
    // Dipole-only model: zero every term above degree 1, keep the frame.
    // The tracer must land on the closed-form apex: QD latitude and apex
    // longitude within 0.001 deg, apex height within 5e-5 (under 0.001 deg
    // of QD latitude at these latitudes). Tracing error is far below the
    // apexpy comparison that follows, so it gets the tighter test.
    igrf_model_t m = *full;
    for (int n = 2; n <= IGRF_NMAX; ++n)
        for (int k = 0; k <= n; ++k) m.g[n][k] = m.h[n][k] = 0.0;

    const double pts[][3] = {
        { 55.0, -114.0, 500.0 }, { 72.0, 20.0, 450.0 }, { -66.0, 140.0, 600.0 },
        { -80.0, -40.0, 500.0 }, { 30.0, 60.0, 500.0 },
    };
    for (size_t i = 0; i < sizeof pts / sizeof pts[0]; ++i) {
        double x[3], qd, alon, hA, dlat, dlon;
        magc_geodetic_to_ecef(pts[i][0], pts[i][1], pts[i][2], x);
        magc_qd(&m, x, pts[i][2], &qd, &alon, &hA);
        magc_dipole_latlon(&m, x, &dlat, &dlon);
        double r = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
        double L = r / (cos(dlat * DEG) * cos(dlat * DEG));
        double apex[3];
        for (int k = 0; k < 3; ++k)
            apex[k] = L * (cos(dlon * DEG) * m.dip_x[k] + sin(dlon * DEG) * m.dip_y[k]);
        double glat, glon, hA_exact;
        magc_ecef_to_geodetic(apex, &glat, &glon, &hA_exact);
        double want = acos(sqrt((QD_RE_KM + pts[i][2]) / (QD_RE_KM + hA_exact))) / DEG;
        if (dlat < 0.0) want = -want;
        tap_okf(fabs(qd - want) < 1e-3 && fabs(wrap180(alon - dlon)) < 1e-3
                && fabs(hA - hA_exact) < 5e-5 * hA_exact,
                "dipole field line from %.0f %.0f: QD %.5f want %.5f, apex lon %.5f want %.5f, apex h %.2f want %.2f km",
                pts[i][0], pts[i][1], qd, want, alon, dlon, hA, hA_exact);
    }
}

static void test_mlt(const igrf_model_t *m)
{
    // apexpy's subsolar point at 2026-10-06T12:00Z and its mlon2mlt.
    // apexpy takes the Sun's apex longitude from its fitted coordinates
    // 50 Earth radii out; 0.01 h (0.15 deg) covers that fit.
    const double sslat = -5.22580110557268, sslon = -2.973390710552394;
    const double cases[][2] = {
        { -60.0, 3.379308573404948 }, { 0.0, 7.3793085734049475 },
        { 95.0, 13.71264190673828 }, { 170.0, 18.71264190673828 },
    };
    double sun[3] = { cos(sslat * DEG) * cos(sslon * DEG),
                      cos(sslat * DEG) * sin(sslon * DEG), sin(sslat * DEG) };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
        double mlt = magc_mlt(m, cases[i][0], sun);
        double d = fabs(fmod(mlt - cases[i][1] + 36.0, 24.0) - 12.0);
        tap_okf(d < 0.01, "MLT of apex lon %.0f: %.4f h (apexpy %.4f)",
                cases[i][0], mlt, cases[i][1]);
    }
}

int main(void)
{
    igrf_model_t m;
    igrf_init(&m, 2026.75);

    // The north geomagnetic pole of IGRF-14 near 2026.75 is about 80.8 N,
    // 72.8 W. Independent of the tables: it is where dip_z points.
    double plat = asin(m.dip_z[2]) / DEG;
    double plon = atan2(m.dip_z[1], m.dip_z[0]) / DEG;
    tap_okf(fabs(plat - 80.8) < 0.1 && fabs(plon + 72.8) < 0.3,
            "north geomagnetic pole at %.2f %.2f", plat, plon);

    test_geodetic();
    test_field(&m);
    test_dipole(&m);
    test_qd(&m);
    test_mlt(&m);
    return tap_done();
}
