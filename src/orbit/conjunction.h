/*

    Simple Satellite Operations  src/orbit/conjunction.h

    Pure geometry + probability-of-collision math for the conjunction
    finder. No SGP4, no I/O: the caller supplies ECI position/velocity
    states (km, km/s) it has already propagated, and these routines turn
    two states into the numbers an operator wants at a close approach --
    the radial / along-track / cross-track separation, and Foster's (1992)
    2-D probability of collision. Kept separate from apps/conjunction.c so
    it can be unit-tested against closed-form oracles.

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

#ifndef SSO_ORBIT_CONJUNCTION_H
#define SSO_ORBIT_CONJUNCTION_H

// Build the RTN (radial / along-track / cross-track) unit axes of an object
// from its ECI position r and velocity v (km, km/s), each written as a 3-vector
// in ECI:
//   rhat  = r / |r|                      (radial, points away from Earth)
//   what  = (r x v) / |r x v|            (cross-track / orbit normal)
//   shat  = what x rhat                  (along-track; ~ velocity direction
//                                         for a near-circular orbit)
// Degenerate inputs (zero r, or r parallel to v) leave the axes as the ECI
// basis so callers never divide by zero.
void conj_rtn_axes(const double r[3], const double v[3],
                   double rhat[3], double shat[3], double what[3]);

// Decompose the position of object 2 relative to object 1 into object 1's
// RTN frame at one instant. r1/v1/r2 are ECI (km, km/s); the four outputs are
// in km:
//   *radial : along object 1's rhat. + = object 2 is higher (farther from Earth).
//   *along  : along object 1's shat. + = object 2 is ahead (along the track).
//   *cross  : along object 1's what (orbit normal).
//   *range  : the full 3-D separation |r2 - r1|.
// By construction radial^2 + along^2 + cross^2 == range^2.
void conj_rtn_components(const double r1[3], const double v1[3],
                         const double r2[3],
                         double *radial, double *along, double *cross,
                         double *range);

// Build a 3x3 position covariance (km^2, row-major) in ECI from per-axis 1-sigma
// position uncertainties given in the object's own RTN frame: sigma_r (radial),
// sigma_a (along-track), sigma_c (cross-track), all in km. r/v are the object's
// ECI state (km, km/s) used to orient the RTN frame. Writes 9 elements to cov.
void conj_cov_rtn_to_eci(const double r[3], const double v[3],
                         double sigma_r, double sigma_a, double sigma_c,
                         double cov[9]);

// Foster (1992) 2-D probability of collision, evaluated in the principal-axis
// frame of the combined covariance projected into the encounter plane:
//   sx, sy : 1-sigma std devs (km) along the two principal axes
//   xm, ym : the projected miss vector (km) in that same principal frame
//   R      : combined hard-body radius (km)
// Returns Pc in [0, 1]: the integral of the 2-D Gaussian over the hard-body
// disk of radius R centred at the origin. A non-positive R returns 0; a
// degenerate (zero) sigma falls back to deterministic containment (1 if the
// miss is inside the disk, else 0).
double conj_foster_pc_principal(double sx, double sy,
                                double xm, double ym, double R);

// Foster (1992) probability of collision from the full 3-D encounter geometry.
//   r_rel : relative position object2 - object1 at TCA (km, ECI)
//   v_rel : relative velocity object2 - object1 at TCA (km/s, ECI)
//   cov   : combined 3x3 position covariance (km^2, row-major) in the SAME
//           frame as r_rel/v_rel (sum the two objects' ECI covariances)
//   R     : combined hard-body radius (km)
// Projects cov and r_rel into the encounter plane (perpendicular to v_rel),
// diagonalises the 2x2, and calls conj_foster_pc_principal. Returns Pc in
// [0, 1]. A zero relative velocity (no defined encounter plane) returns 0.
double conj_foster_pc(const double r_rel[3], const double v_rel[3],
                      const double cov[9], double R);

#endif // SSO_ORBIT_CONJUNCTION_H
