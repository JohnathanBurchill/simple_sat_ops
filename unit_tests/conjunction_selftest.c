/*

    Simple Satellite Operations  unit_tests/conjunction_selftest.c

    Tests for src/orbit/conjunction.c -- the pure geometry + Foster (1992)
    probability of collision used by the conjunction finder. No SGP4 here:
    these are closed-form-checkable math, so every assertion has an
    independent oracle that a real bug would break.

      - conj_rtn_axes / conj_rtn_components: hand-computed values for an
        axis-aligned orbit, the radial^2 + along^2 + cross^2 == range^2
        invariant for a tilted one, sign conventions, and the degenerate
        fallback.
      - conj_cov_rtn_to_eci: diagonal result when RTN aligns with ECI;
        rotation invariants a real implementation must satisfy regardless of
        orientation (symmetry, trace = sum of variances, the per-axis
        quadratic forms recover the input variances).
      - conj_foster_pc_principal: the circular zero-miss closed form
        Pc = 1 - exp(-R^2 / 2 sigma^2) (an EXTERNAL analytic oracle), a fully
        independent 2-D grid integrator for the elliptical / off-centre case
        the closed form can't reach, monotonicity, and the limit / degenerate
        cases.
      - conj_foster_pc: cross-checks the 3-D encounter-plane projection +
        diagonalisation against direct conj_foster_pc_principal calls for an
        isotropic covariance and an axis-aligned anisotropic one.

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
#include "tap.h"

#include <math.h>
#include <stdio.h>

// Independent 2-D oracle for the Foster principal integral: a midpoint grid
// over the hard-body disk. Crude (O(h) on the circular boundary) but a wholly
// separate method from the erf-reduction + sin-substitution Simpson in
// conjunction.c, so agreement between them catches a wrong normalisation,
// a swapped sigma, or a botched erf argument. N chosen so the boundary error
// is ~1e-3 absolute for sigma ~ R ~ 1.
static double pc_grid(double sx, double sy, double xm, double ym, double R)
{
    const int N = 1400;
    double h = 2.0 * R / (double) N;
    double norm = 1.0 / (2.0 * M_PI * sx * sy);
    double sum = 0.0;
    for (int i = 0; i < N; ++i) {
        double x = -R + ((double) i + 0.5) * h;
        for (int j = 0; j < N; ++j) {
            double y = -R + ((double) j + 0.5) * h;
            if (x * x + y * y <= R * R) {
                double e = (x - xm) * (x - xm) / (sx * sx)
                         + (y - ym) * (y - ym) / (sy * sy);
                sum += exp(-0.5 * e);
            }
        }
    }
    return sum * h * h * norm;
}

// ------------------------------------------------------------------ RTN frame

static void test_rtn(void)
{
    fprintf(stderr, "conj_rtn_axes / conj_rtn_components:\n");

    // Axis-aligned orbit: r along +x, v along +y. Then rhat=+x, what=+z
    // (r x v), shat = what x rhat = +y. A relative offset of (10,20,30) km
    // maps directly onto (radial, along, cross).
    double r1[3] = { 7000.0, 0.0, 0.0 };
    double v1[3] = { 0.0, 7.5, 0.0 };
    double rhat[3], shat[3], what[3];
    conj_rtn_axes(r1, v1, rhat, shat, what);
    tap_okf(fabs(rhat[0] - 1.0) < 1e-12 && fabs(rhat[1]) < 1e-12 && fabs(rhat[2]) < 1e-12,
            "rhat = +x for r along +x (got %.3f,%.3f,%.3f)", rhat[0], rhat[1], rhat[2]);
    tap_okf(fabs(shat[1] - 1.0) < 1e-12 && fabs(shat[0]) < 1e-12 && fabs(shat[2]) < 1e-12,
            "shat = +y (velocity direction) (got %.3f,%.3f,%.3f)", shat[0], shat[1], shat[2]);
    tap_okf(fabs(what[2] - 1.0) < 1e-12 && fabs(what[0]) < 1e-12 && fabs(what[1]) < 1e-12,
            "what = +z (orbit normal) (got %.3f,%.3f,%.3f)", what[0], what[1], what[2]);

    double r2[3] = { 7010.0, 20.0, 30.0 };
    double radial, along, cross, range;
    conj_rtn_components(r1, v1, r2, &radial, &along, &cross, &range);
    tap_okf(fabs(radial - 10.0) < 1e-9, "radial = +10 km (object higher) (got %.6f)", radial);
    tap_okf(fabs(along - 20.0) < 1e-9, "along = +20 km (object ahead) (got %.6f)", along);
    tap_okf(fabs(cross - 30.0) < 1e-9, "cross = +30 km (got %.6f)", cross);
    tap_okf(fabs(range - sqrt(1400.0)) < 1e-9, "range = sqrt(1400) km (got %.6f)", range);

    // Invariant on a tilted, non-circular geometry: the three signed
    // components are an orthonormal decomposition of the same vector, so
    // their squares must sum to the squared range exactly.
    double rt1[3] = { 3000.0, 4000.0, 5000.0 };
    double vt1[3] = { -4.0, 3.0, 2.0 };
    double rt2[3] = { 2987.0, 4015.0, 4960.0 };
    conj_rtn_components(rt1, vt1, rt2, &radial, &along, &cross, &range);
    double recomposed = sqrt(radial * radial + along * along + cross * cross);
    tap_okf(fabs(recomposed - range) < 1e-9,
            "radial^2+along^2+cross^2 == range^2 (%.9f vs %.9f)", recomposed, range);

    // Sign: an object purely lower (toward Earth) has negative radial.
    double rlow[3] = { 6990.0, 0.0, 0.0 };
    conj_rtn_components(r1, v1, rlow, &radial, &along, &cross, &range);
    tap_okf(radial < 0.0 && fabs(radial + 10.0) < 1e-9,
            "radial = -10 km for an object 10 km lower (got %.6f)", radial);

    // Degenerate r=0 falls back to the ECI basis (no divide-by-zero / NaN).
    double rz[3] = { 0.0, 0.0, 0.0 };
    conj_rtn_axes(rz, v1, rhat, shat, what);
    tap_okf(rhat[0] == 1.0 && shat[1] == 1.0 && what[2] == 1.0,
            "degenerate r=0 -> ECI basis fallback");
}

// ------------------------------------------------------------------ covariance

static void test_cov(void)
{
    fprintf(stderr, "conj_cov_rtn_to_eci:\n");

    // Axis-aligned orbit: RTN coincides with ECI, so the ECI covariance is
    // exactly diag(sr^2, sa^2, sc^2).
    double r[3] = { 7000.0, 0.0, 0.0 };
    double v[3] = { 0.0, 7.5, 0.0 };
    double cov[9];
    conj_cov_rtn_to_eci(r, v, 1.0, 2.0, 3.0, cov);
    tap_okf(fabs(cov[0] - 1.0) < 1e-9 && fabs(cov[4] - 4.0) < 1e-9 && fabs(cov[8] - 9.0) < 1e-9,
            "aligned: diagonal = (1,4,9) km^2 (got %.4f,%.4f,%.4f)", cov[0], cov[4], cov[8]);
    tap_okf(fabs(cov[1]) < 1e-12 && fabs(cov[2]) < 1e-12 && fabs(cov[5]) < 1e-12,
            "aligned: off-diagonal terms are zero");

    // Tilted orbit: the covariance is a rotation of diag(sr^2,sa^2,sc^2), so
    // it must (a) be symmetric, (b) preserve the trace (= sum of variances),
    // and (c) recover each input variance as the quadratic form along the
    // corresponding RTN axis. These hold for any orientation -- a real
    // rotation -- and fail for a transposed or mis-built rotation matrix.
    double rt[3] = { 3000.0, 4000.0, 5000.0 };
    double vt[3] = { -4.0, 3.0, 2.0 };
    double sr = 0.3, sa = 1.1, sc = 0.5;
    conj_cov_rtn_to_eci(rt, vt, sr, sa, sc, cov);
    int symmetric = fabs(cov[1] - cov[3]) < 1e-12
                 && fabs(cov[2] - cov[6]) < 1e-12
                 && fabs(cov[5] - cov[7]) < 1e-12;
    tap_ok(symmetric, "tilted: covariance is symmetric");
    double trace = cov[0] + cov[4] + cov[8];
    double want_trace = sr * sr + sa * sa + sc * sc;
    tap_okf(fabs(trace - want_trace) < 1e-9,
            "tilted: trace == sr^2+sa^2+sc^2 (%.6f vs %.6f)", trace, want_trace);

    double rhat[3], shat[3], what[3];
    conj_rtn_axes(rt, vt, rhat, shat, what);
    double qr = 0.0, qa = 0.0, qc = 0.0;
    for (int i = 0; i < 3; ++i) {
        double cr = 0, ca = 0, cc = 0;
        for (int j = 0; j < 3; ++j) {
            cr += cov[3 * i + j] * rhat[j];
            ca += cov[3 * i + j] * shat[j];
            cc += cov[3 * i + j] * what[j];
        }
        qr += rhat[i] * cr;
        qa += shat[i] * ca;
        qc += what[i] * cc;
    }
    tap_okf(fabs(qr - sr * sr) < 1e-9, "tilted: rhat^T C rhat == sr^2 (%.6f vs %.6f)", qr, sr * sr);
    tap_okf(fabs(qa - sa * sa) < 1e-9, "tilted: shat^T C shat == sa^2 (%.6f vs %.6f)", qa, sa * sa);
    tap_okf(fabs(qc - sc * sc) < 1e-9, "tilted: what^T C what == sc^2 (%.6f vs %.6f)", qc, sc * sc);
}

// ----------------------------------------------------- Foster principal Pc

static void test_pc_principal(void)
{
    fprintf(stderr, "conj_foster_pc_principal:\n");

    // Circular, zero miss -> the analytic Rayleigh result 1 - exp(-R^2/2s^2).
    // Simpson with the sin-substitution should match to well under 1e-5.
    double cases[][2] = { {1.0, 1.0}, {1.0, 0.5}, {2.0, 3.0}, {0.5, 0.2} };
    for (int k = 0; k < 4; ++k) {
        double s = cases[k][0], R = cases[k][1];
        double pc = conj_foster_pc_principal(s, s, 0.0, 0.0, R);
        double want = 1.0 - exp(-(R * R) / (2.0 * s * s));
        tap_okf(fabs(pc - want) < 1e-5,
                "circular zero-miss: s=%.2f R=%.2f Pc=%.8f (closed form %.8f)",
                s, R, pc, want);
    }

    // Elliptical + off-centre: cross-check the analytic-unreachable cases
    // against the independent grid integrator. 1%% relative (the grid's
    // boundary error dominates), guarded by a small absolute floor.
    double ell[][5] = {
        // sx,  sy,   xm,   ym,   R
        { 1.0, 2.0,  0.5,  0.0,  1.5 },
        { 0.8, 0.4,  0.3, -0.2,  0.6 },
        { 2.0, 1.0, -1.0,  0.5,  2.0 },
        { 1.0, 1.0,  1.2,  0.0,  1.0 },
    };
    for (int k = 0; k < 4; ++k) {
        double sx = ell[k][0], sy = ell[k][1], xm = ell[k][2], ym = ell[k][3], R = ell[k][4];
        double pc = conj_foster_pc_principal(sx, sy, xm, ym, R);
        double ref = pc_grid(sx, sy, xm, ym, R);
        double tol = 0.01 * ref + 1e-4;
        tap_okf(fabs(pc - ref) < tol,
                "elliptical/off-centre vs grid: sx=%.1f sy=%.1f m=(%.1f,%.1f) R=%.1f "
                "Pc=%.6f grid=%.6f", sx, sy, xm, ym, R, pc, ref);
    }

    // Monotone in R (zero miss): a bigger hard body cannot lower Pc.
    double p1 = conj_foster_pc_principal(1.0, 1.0, 0.0, 0.0, 0.5);
    double p2 = conj_foster_pc_principal(1.0, 1.0, 0.0, 0.0, 1.0);
    double p3 = conj_foster_pc_principal(1.0, 1.0, 0.0, 0.0, 2.0);
    tap_okf(p1 < p2 && p2 < p3, "Pc increases with R (%.4f < %.4f < %.4f)", p1, p2, p3);

    // Monotone decreasing as the miss grows.
    double m0 = conj_foster_pc_principal(1.0, 1.0, 0.0, 0.0, 1.0);
    double m1 = conj_foster_pc_principal(1.0, 1.0, 2.0, 0.0, 1.0);
    double m2 = conj_foster_pc_principal(1.0, 1.0, 5.0, 0.0, 1.0);
    tap_okf(m0 > m1 && m1 > m2, "Pc decreases as miss grows (%.4e > %.4e > %.4e)", m0, m1, m2);

    // Limits and degenerate guards.
    tap_ok(conj_foster_pc_principal(1.0, 1.0, 0.0, 0.0, 0.0) == 0.0, "R=0 -> Pc=0");
    tap_ok(conj_foster_pc_principal(1.0, 1.0, 0.0, 0.0, -1.0) == 0.0, "R<0 -> Pc=0");
    double big = conj_foster_pc_principal(1.0, 1.0, 0.0, 0.0, 50.0);
    tap_okf(big > 0.999999, "R >> sigma -> Pc -> 1 (got %.8f)", big);
    // Degenerate sigma -> deterministic containment.
    tap_ok(conj_foster_pc_principal(0.0, 1.0, 0.5, 0.0, 1.0) == 1.0,
           "sigma=0, miss inside R -> Pc=1");
    tap_ok(conj_foster_pc_principal(0.0, 1.0, 2.0, 0.0, 1.0) == 0.0,
           "sigma=0, miss outside R -> Pc=0");
}

// ----------------------------------------------------- Foster 3-D pipeline

static void test_pc_pipeline(void)
{
    fprintf(stderr, "conj_foster_pc (3-D projection):\n");

    double R = 0.05;            // 50 m combined hard body, in km

    // Isotropic covariance: any 2-D projection of sigma^2 * I is sigma^2 * I,
    // and with the miss perpendicular to the relative velocity the projected
    // miss magnitude equals |r_rel|. So the pipeline must equal a direct
    // principal call with that miss magnitude.
    double sigma = 1.0;         // km
    double cov_iso[9] = { sigma * sigma, 0, 0,
                          0, sigma * sigma, 0,
                          0, 0, sigma * sigma };
    double v_rel[3] = { 0.0, 0.0, 7.0 };          // along +z
    double r_rel[3] = { 0.6, 0.8, 0.0 };          // in x-y, perpendicular to v_rel
    double miss = sqrt(0.6 * 0.6 + 0.8 * 0.8);    // = 1.0 km
    double pc_pipe = conj_foster_pc(r_rel, v_rel, cov_iso, R);
    double pc_ref  = conj_foster_pc_principal(sigma, sigma, miss, 0.0, R);
    tap_okf(fabs(pc_pipe - pc_ref) < 1e-9,
            "isotropic: pipeline == principal (%.6e vs %.6e)", pc_pipe, pc_ref);

    // Anisotropic, axis-aligned: cov = diag(a^2,b^2,c^2), v_rel along x, so the
    // encounter plane is y-z and the projection picks out (b^2, c^2) with the
    // miss components (my, mz). Exercises the projection + 2x2 diagonalisation
    // with distinct eigenvalues against a direct principal call.
    double a = 0.5, b = 1.5, c = 0.9;             // km
    double cov_an[9] = { a * a, 0, 0,
                         0, b * b, 0,
                         0, 0, c * c };
    double v2[3] = { 7.5, 0.0, 0.0 };             // along +x
    double r2[3] = { 0.0, 0.7, -0.4 };            // my=0.7, mz=-0.4 in the plane
    double pc_pipe2 = conj_foster_pc(r2, v2, cov_an, R);
    double pc_ref2  = conj_foster_pc_principal(b, c, 0.7, -0.4, R);
    tap_okf(fabs(pc_pipe2 - pc_ref2) < 1e-9,
            "anisotropic axis-aligned: pipeline == principal (%.6e vs %.6e)",
            pc_pipe2, pc_ref2);

    // Zero relative velocity: no encounter plane is defined -> Pc = 0.
    double vz[3] = { 0.0, 0.0, 0.0 };
    tap_ok(conj_foster_pc(r_rel, vz, cov_iso, R) == 0.0, "zero relative velocity -> Pc=0");
}

int main(void)
{
    test_rtn();
    test_cov();
    test_pc_principal();
    test_pc_pipeline();
    return tap_done();
}
