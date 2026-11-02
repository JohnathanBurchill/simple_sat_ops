/*

    Simple Satellite Operations  magcoords.h

    The geomagnetic main field and the magnetic coordinates space physics
    uses to place a satellite relative to the auroral oval and the cusp:

      - IGRF-14 (the 2025.0 main field plus its 2025-2030 secular
        variation), evaluated in geocentric spherical or Earth-fixed
        Cartesian coordinates.
      - Quasi-dipole (QD) latitude and apex longitude (Richmond 1995;
        Emmert et al. 2010), found by tracing the IGRF field line from the
        point up to its apex. These are the coordinates apexpy and the
        Swarm products report.
      - Magnetic local time from the apex longitude, with apexpy's
        convention: the Sun's longitude is taken in the centered-dipole
        frame (apexpy evaluates it 50 Earth radii out, where the field is
        that dipole).

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

#ifndef MAGCOORDS_H
#define MAGCOORDS_H

#define IGRF_NMAX       13
// IGRF reference radius, km
#define IGRF_A_KM       6371.2
// Mean Earth radius in the quasi-dipole latitude formula (apexpy's RE), km
#define QD_RE_KM        6371.009
// IGRF-14's secular variation is defined over this span of decimal years
#define IGRF_YEAR_FIRST 2025.0
#define IGRF_YEAR_LAST  2030.0

typedef struct igrf_model {
    double year;
    // Coefficients at `year`, pre-multiplied by the Schmidt factors so they
    // pair with Gauss-normalised associated Legendre functions. nT.
    double g[IGRF_NMAX + 1][IGRF_NMAX + 1];
    double h[IGRF_NMAX + 1][IGRF_NMAX + 1];
    // Centered-dipole frame in Earth-fixed coordinates: z toward the north
    // geomagnetic pole, y = (geographic north) x z, x = y x z. With these
    // axes the geographic north pole sits at magnetic longitude 180 deg.
    double dip_x[3], dip_y[3], dip_z[3];
} igrf_model_t;

// Fill the model for a decimal year. Outside IGRF_YEAR_FIRST..LAST the
// secular variation is extrapolated linearly; callers warn about that.
void igrf_init(igrf_model_t *m, double decimal_year);

// Decimal year of a Julian date (UTC), close enough for a field model.
double igrf_decimal_year(double jd);

// Field at geocentric radius r_km, colatitude theta and east longitude phi
// (radians), truncated at degree nmax. Outputs the spherical components
// (radial outward, colatitude southward, longitude eastward) in nT.
void igrf_field_sph(const igrf_model_t *m, double r_km, double theta,
                    double phi, int nmax,
                    double *br, double *btheta, double *bphi);

// Field at an Earth-fixed Cartesian position (km), as Earth-fixed
// Cartesian components (nT), truncated at degree nmax.
void igrf_field_ecef(const igrf_model_t *m, const double x_km[3], int nmax,
                     double b[3]);

// WGS84 geodetic <-> Earth-fixed Cartesian (km).
void magc_geodetic_to_ecef(double lat_deg, double lon_deg, double alt_km,
                           double x_km[3]);
void magc_ecef_to_geodetic(const double x_km[3], double *lat_deg,
                           double *lon_deg, double *alt_km);

// Centered-dipole latitude and longitude (deg) of an Earth-fixed direction.
void magc_dipole_latlon(const igrf_model_t *m, const double x[3],
                        double *lat_deg, double *lon_deg);

// Quasi-dipole latitude and apex longitude (deg) of an Earth-fixed point
// x_km whose geodetic height is alt_km, by tracing the IGRF field line to
// its apex. apex_height_km (may be NULL) returns the apex's geodetic
// height. Field lines that leave 100 Earth radii (QD latitude above about
// 84 deg) are finished with the dipole formula from that distance.
void magc_qd(const igrf_model_t *m, const double x_km[3], double alt_km,
             double *qdlat_deg, double *apex_lon_deg, double *apex_height_km);

// Magnetic local time (hours, [0,24)) of an apex longitude, given the
// Earth-fixed direction to the Sun (any length).
double magc_mlt(const igrf_model_t *m, double apex_lon_deg,
                const double sun_ecef[3]);

#endif // MAGCOORDS_H
