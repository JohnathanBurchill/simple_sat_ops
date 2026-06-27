/*

    Simple Satellite Operations  src/orbit/conjunction.c

    Pure geometry + Foster (1992) probability of collision for the
    conjunction finder. See conjunction.h for the contract.

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

#include "conjunction.h"

#include <math.h>

// --- small 3-vector helpers -------------------------------------------------

static double v3_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double v3_norm(const double a[3])
{
    return sqrt(v3_dot(a, a));
}

static void v3_cross(const double a[3], const double b[3], double o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

// --- RTN frame --------------------------------------------------------------

void conj_rtn_axes(const double r[3], const double v[3],
                   double rhat[3], double shat[3], double what[3])
{
    // Fall back to the ECI basis on a degenerate state so no caller divides
    // by zero. Overwritten below whenever the geometry is well defined.
    rhat[0] = 1.0; rhat[1] = 0.0; rhat[2] = 0.0;
    shat[0] = 0.0; shat[1] = 1.0; shat[2] = 0.0;
    what[0] = 0.0; what[1] = 0.0; what[2] = 1.0;

    double rmag = v3_norm(r);
    if (rmag <= 0.0) return;
    double rh[3] = { r[0] / rmag, r[1] / rmag, r[2] / rmag };

    double w[3];
    v3_cross(r, v, w);
    double wmag = v3_norm(w);
    if (wmag <= 0.0) return;          // r parallel to v: orbit normal undefined
    double wh[3] = { w[0] / wmag, w[1] / wmag, w[2] / wmag };

    // shat = what x rhat completes the right-handed triad and is the
    // along-track direction (~ velocity for a near-circular orbit).
    double sh[3];
    v3_cross(wh, rh, sh);

    for (int i = 0; i < 3; ++i) {
        rhat[i] = rh[i];
        what[i] = wh[i];
        shat[i] = sh[i];
    }
}

void conj_rtn_components(const double r1[3], const double v1[3],
                         const double r2[3],
                         double *radial, double *along, double *cross,
                         double *range)
{
    double rhat[3], shat[3], what[3];
    conj_rtn_axes(r1, v1, rhat, shat, what);

    double d[3] = { r2[0] - r1[0], r2[1] - r1[1], r2[2] - r1[2] };
    if (radial) *radial = v3_dot(d, rhat);
    if (along)  *along  = v3_dot(d, shat);
    if (cross)  *cross  = v3_dot(d, what);
    if (range)  *range  = v3_norm(d);
}

// --- covariance -------------------------------------------------------------

void conj_cov_rtn_to_eci(const double r[3], const double v[3],
                         double sigma_r, double sigma_a, double sigma_c,
                         double cov[9])
{
    double rhat[3], shat[3], what[3];
    conj_rtn_axes(r, v, rhat, shat, what);

    // C = Q diag(sr^2, sa^2, sc^2) Q^T, where the columns of Q are the RTN
    // axes in ECI. Element (i,j) = sr^2 rhat_i rhat_j + sa^2 shat_i shat_j
    // + sc^2 what_i what_j.
    double vr = sigma_r * sigma_r;
    double va = sigma_a * sigma_a;
    double vc = sigma_c * sigma_c;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cov[3 * i + j] = vr * rhat[i] * rhat[j]
                           + va * shat[i] * shat[j]
                           + vc * what[i] * what[j];
        }
    }
}

// --- Foster (1992) probability of collision ---------------------------------

double conj_foster_pc_principal(double sx, double sy,
                                double xm, double ym, double R)
{
    if (R <= 0.0) return 0.0;

    // Degenerate covariance: the relative position is deterministic, so a
    // collision either happens (miss inside the hard-body disk) or it does
    // not. Avoids dividing by a zero sigma below.
    if (sx <= 0.0 || sy <= 0.0)
        return (sqrt(xm * xm + ym * ym) <= R) ? 1.0 : 0.0;

    // Integrate the 2-D Gaussian (centred at the miss vector, diagonal in
    // this principal frame) over the hard-body disk x^2 + y^2 <= R^2 centred
    // at the origin. Reduce the inner y-integral to an erf difference, then
    // substitute x = R sin(theta) so the disk's edge (where the integrand's
    // slope is otherwise vertical, sqrt(R^2-x^2)) maps to theta = +-pi/2 and
    // Simpson converges cleanly.
    const double SQRT_2PI = 2.5066282746310002;
    const double SQRT_2   = 1.4142135623730951;
    double inv_sx_root2pi = 1.0 / (sx * SQRT_2PI);
    double inv_sy_root2   = 1.0 / (sy * SQRT_2);

    const int N = 2000;                 // even, for composite Simpson
    double a = -M_PI / 2.0, b = M_PI / 2.0;
    double h = (b - a) / (double) N;
    double sum = 0.0;
    for (int i = 0; i <= N; ++i) {
        double th = a + (double) i * h;
        double ct = cos(th);
        double x  = R * sin(th);
        double yhi = R * ct;            // sqrt(R^2 - x^2)

        double ex = inv_sx_root2pi
                  * exp(-0.5 * ((x - xm) / sx) * ((x - xm) / sx));
        double inner = 0.5 * (erf((yhi - ym) * inv_sy_root2)
                            - erf((-yhi - ym) * inv_sy_root2));
        // The R*cos(theta) Jacobian from the substitution.
        double f = ex * inner * R * ct;

        double w = (i == 0 || i == N) ? 1.0 : (i % 2 ? 4.0 : 2.0);
        sum += w * f;
    }
    double pc = sum * h / 3.0;
    if (pc < 0.0) pc = 0.0;
    if (pc > 1.0) pc = 1.0;
    return pc;
}

// Symmetric 2x2 quadratic form e^T C e for a 3-vector axis e and a row-major
// 3x3 covariance.
static double quad3(const double e[3], const double cov[9])
{
    double ce[3];
    for (int i = 0; i < 3; ++i)
        ce[i] = cov[3 * i + 0] * e[0] + cov[3 * i + 1] * e[1] + cov[3 * i + 2] * e[2];
    return v3_dot(e, ce);
}

// e^T C f for two axes.
static double bilin3(const double e[3], const double cov[9], const double f[3])
{
    double cf[3];
    for (int i = 0; i < 3; ++i)
        cf[i] = cov[3 * i + 0] * f[0] + cov[3 * i + 1] * f[1] + cov[3 * i + 2] * f[2];
    return v3_dot(e, cf);
}

double conj_foster_pc(const double r_rel[3], const double v_rel[3],
                      const double cov[9], double R)
{
    double vmag = v3_norm(v_rel);
    if (vmag <= 0.0) return 0.0;        // no encounter plane without relative motion
    double vhat[3] = { v_rel[0] / vmag, v_rel[1] / vmag, v_rel[2] / vmag };

    // Two orthonormal in-plane axes p, q perpendicular to vhat. Seed with the
    // ECI axis least aligned with vhat to avoid a near-zero cross product.
    double seed[3] = { 1.0, 0.0, 0.0 };
    double ax = fabs(vhat[0]), ay = fabs(vhat[1]), az = fabs(vhat[2]);
    if (ay <= ax && ay <= az)      { seed[0] = 0.0; seed[1] = 1.0; seed[2] = 0.0; }
    else if (az <= ax && az <= ay) { seed[0] = 0.0; seed[1] = 0.0; seed[2] = 1.0; }

    double sdotv = v3_dot(seed, vhat);
    double p[3];
    for (int i = 0; i < 3; ++i) p[i] = seed[i] - sdotv * vhat[i];
    double pmag = v3_norm(p);
    if (pmag <= 0.0) return 0.0;
    for (int i = 0; i < 3; ++i) p[i] /= pmag;
    double q[3];
    v3_cross(vhat, p, q);              // unit by construction

    // Projected miss vector and 2x2 covariance in the {p, q} plane.
    double m1 = v3_dot(r_rel, p);
    double m2 = v3_dot(r_rel, q);
    double cpp = quad3(p, cov);
    double cqq = quad3(q, cov);
    double cpq = bilin3(p, cov, q);

    // Eigen-decompose the symmetric 2x2 [[cpp, cpq], [cpq, cqq]].
    double tr = cpp + cqq;
    double diff = cpp - cqq;
    double root = sqrt((diff / 2.0) * (diff / 2.0) + cpq * cpq);
    double l1 = tr / 2.0 + root;       // larger eigenvalue
    double l2 = tr / 2.0 - root;
    if (l1 < 0.0) l1 = 0.0;
    if (l2 < 0.0) l2 = 0.0;

    // Principal axis for l1: eigenvector (cpq, l1 - cpp), or the p-axis when
    // the 2x2 is already diagonal.
    double e1[2];
    if (fabs(cpq) > 1e-300) {
        e1[0] = cpq;
        e1[1] = l1 - cpp;
    } else {
        e1[0] = 1.0;
        e1[1] = 0.0;
    }
    double e1mag = sqrt(e1[0] * e1[0] + e1[1] * e1[1]);
    if (e1mag <= 0.0) { e1[0] = 1.0; e1[1] = 0.0; e1mag = 1.0; }
    e1[0] /= e1mag; e1[1] /= e1mag;
    double e2[2] = { -e1[1], e1[0] };

    double xm = m1 * e1[0] + m2 * e1[1];
    double ym = m1 * e2[0] + m2 * e2[1];

    return conj_foster_pc_principal(sqrt(l1), sqrt(l2), xm, ym, R);
}
