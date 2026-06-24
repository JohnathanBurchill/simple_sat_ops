/*

    Simple Satellite Operations  shortarc.c

    Weighted batch least-squares orbit fit over a short arc. See shortarc.h.

    Internally everything runs in km / km/s (the natural scale for the gravity
    constants and well conditioned); the API is metres, converted at the edges.

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

#include "shortarc.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Earth constants -- identical to space_safety_manager's propagate.c so the
// fit and ssm's forward propagation use the same dynamics.
#define SA_MU   398600.4418     // km^3/s^2
#define SA_RE   6378.137        // km
#define SA_J2   1.08263e-3

// Two-body + J2 acceleration (km/s^2), no Earth-rotation terms. Mirror of
// ssm's j2_acceleration.
static void accel(const double pos[3], double acc[3])
{
    double r2 = pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2];
    double r = sqrt(r2);
    double r3 = r2 * r;
    double r5 = r3 * r2;

    acc[0] = -SA_MU * pos[0] / r3;
    acc[1] = -SA_MU * pos[1] / r3;
    acc[2] = -SA_MU * pos[2] / r3;

    double z2 = pos[2] * pos[2];
    double fac = 1.5 * SA_J2 * SA_MU * SA_RE * SA_RE / r5;
    double fxy = fac * (5.0 * z2 / r2 - 1.0);
    double fz = fac * (5.0 * z2 / r2 - 3.0);
    acc[0] -= fxy * pos[0];
    acc[1] -= fxy * pos[1];
    acc[2] -= fz * pos[2];
}

// One RK4 step of the 6-state [x,y,z,vx,vy,vz]. Mirror of ssm's rk4_step.
static void rk4_step(double s[6], double dt)
{
    double k1[6], k2[6], k3[6], k4[6], tmp[6], a[3];

    accel(s, a);
    k1[0] = s[3]; k1[1] = s[4]; k1[2] = s[5]; k1[3] = a[0]; k1[4] = a[1]; k1[5] = a[2];
    for (int i = 0; i < 6; i++) tmp[i] = s[i] + 0.5 * dt * k1[i];
    accel(tmp, a);
    k2[0] = tmp[3]; k2[1] = tmp[4]; k2[2] = tmp[5]; k2[3] = a[0]; k2[4] = a[1]; k2[5] = a[2];
    for (int i = 0; i < 6; i++) tmp[i] = s[i] + 0.5 * dt * k2[i];
    accel(tmp, a);
    k3[0] = tmp[3]; k3[1] = tmp[4]; k3[2] = tmp[5]; k3[3] = a[0]; k3[4] = a[1]; k3[5] = a[2];
    for (int i = 0; i < 6; i++) tmp[i] = s[i] + dt * k3[i];
    accel(tmp, a);
    k4[0] = tmp[3]; k4[1] = tmp[4]; k4[2] = tmp[5]; k4[3] = a[0]; k4[4] = a[1]; k4[5] = a[2];
    for (int i = 0; i < 6; i++)
        s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
}

// Advance state s by dt seconds in place (dt may be negative).
static void propagate(double s[6], double dt)
{
    if (dt == 0.0) return;
    const double max_step = 5.0;       // s
    int n = (int)ceil(fabs(dt) / max_step);
    if (n < 1) n = 1;
    double h = dt / n;
    for (int i = 0; i < n; i++) rk4_step(s, h);
}

// State transition matrix Phi[i][j] = d state_i(epoch+dt) / d state_j(epoch),
// by central finite differences about x0.
static void stm(const double x0[6], double dt, double Phi[6][6])
{
    static const double pert[6] = { 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6 }; // km, km/s
    for (int j = 0; j < 6; j++) {
        double sp[6], sm[6];
        memcpy(sp, x0, sizeof sp);
        memcpy(sm, x0, sizeof sm);
        sp[j] += pert[j];
        sm[j] -= pert[j];
        propagate(sp, dt);
        propagate(sm, dt);
        for (int i = 0; i < 6; i++) Phi[i][j] = (sp[i] - sm[i]) / (2.0 * pert[j]);
    }
}

// In-place 6x6 inverse by Gauss-Jordan with partial pivoting. Returns 0 on
// success, -1 if singular.
static int mat6_inverse(const double A[6][6], double inv[6][6])
{
    double m[6][12];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) { m[i][j] = A[i][j]; m[i][j + 6] = (i == j) ? 1.0 : 0.0; }
    }
    for (int c = 0; c < 6; c++) {
        int piv = c;
        double best = fabs(m[c][c]);
        for (int r = c + 1; r < 6; r++)
            if (fabs(m[r][c]) > best) { best = fabs(m[r][c]); piv = r; }
        if (best < 1e-18) return -1;
        if (piv != c)
            for (int j = 0; j < 12; j++) { double t = m[c][j]; m[c][j] = m[piv][j]; m[piv][j] = t; }
        double d = m[c][c];
        for (int j = 0; j < 12; j++) m[c][j] /= d;
        for (int r = 0; r < 6; r++) {
            if (r == c) continue;
            double f = m[r][c];
            for (int j = 0; j < 12; j++) m[r][j] -= f * m[c][j];
        }
    }
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) inv[i][j] = m[i][j + 6];
    return 0;
}

int shortarc_fit(const shortarc_obs_t *obs, int n, shortarc_fit_t *out)
{
    memset(out, 0, sizeof *out);
    if (obs == NULL || n < 1) return -1;

    // Observations and per-axis inverse-variance weights in km, km/s.
    double y[SHORTARC_MAX_OBS][6], w[SHORTARC_MAX_OBS][6], dt[SHORTARC_MAX_OBS];
    if (n > SHORTARC_MAX_OBS) {
        fprintf(stderr,
                "shortarc_fit: %d observations exceeds the %d-obs cap; "
                "using the first %d.\n", n, SHORTARC_MAX_OBS, SHORTARC_MAX_OBS);
        n = SHORTARC_MAX_OBS;
    }
    for (int i = 0; i < n; i++) {
        dt[i] = obs[i].dt;
        for (int k = 0; k < 3; k++) {
            y[i][k]     = obs[i].pos[k] / 1000.0;
            y[i][k + 3] = obs[i].vel[k] / 1000.0;
            double sp = obs[i].pos_sigma[k] / 1000.0;
            double sv = obs[i].vel_sigma[k] / 1000.0;
            w[i][k]     = (sp > 0.0) ? 1.0 / (sp * sp) : 0.0;
            w[i][k + 3] = (sv > 0.0) ? 1.0 / (sv * sv) : 0.0;
        }
    }

    // A priori: the observation nearest the epoch (smallest |dt|).
    int a0 = 0;
    for (int i = 1; i < n; i++) if (fabs(dt[i]) < fabs(dt[a0])) a0 = i;
    double x[6];
    memcpy(x, y[a0], sizeof x);

    double Ninv[6][6];
    int iter = 0, converged = 0;
    const int MAXIT = 8;
    for (; iter < MAXIT; iter++) {
        double N[6][6] = { { 0 } }, b[6] = { 0 };
        for (int i = 0; i < n; i++) {
            double xnom[6];
            memcpy(xnom, x, sizeof xnom);
            propagate(xnom, dt[i]);
            double Phi[6][6];
            stm(x, dt[i], Phi);
            double r[6];
            for (int k = 0; k < 6; k++) r[k] = y[i][k] - xnom[k];
            // N += Phi^T W Phi ; b += Phi^T W r   (W diagonal)
            for (int a = 0; a < 6; a++) {
                double pwa[6];
                for (int k = 0; k < 6; k++) pwa[k] = Phi[k][a] * w[i][k];
                for (int bcol = 0; bcol < 6; bcol++) {
                    double s = 0.0;
                    for (int k = 0; k < 6; k++) s += pwa[k] * Phi[k][bcol];
                    N[a][bcol] += s;
                }
                double sb = 0.0;
                for (int k = 0; k < 6; k++) sb += pwa[k] * r[k];
                b[a] += sb;
            }
        }
        if (mat6_inverse(N, Ninv) != 0) return -1;
        double dx[6];
        for (int a = 0; a < 6; a++) {
            double s = 0.0;
            for (int k = 0; k < 6; k++) s += Ninv[a][k] * b[k];
            dx[a] = s;
        }
        for (int k = 0; k < 6; k++) x[k] += dx[k];
        double dpos = sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]);
        double dvel = sqrt(dx[3] * dx[3] + dx[4] * dx[4] + dx[5] * dx[5]);
        if (dpos < 1e-9 && dvel < 1e-12) { iter++; converged = 1; break; }   // < ~1 um, 1 nm/s
    }
    // A short arc converges in a few iterations; running out of iterations means
    // the linearization broke down (e.g. the window is far too long for this
    // dynamics model). Don't emit a garbage state.
    if (!converged) return -1;

    // Post-fit residuals: chi-square (for covariance scaling) and position RMS.
    double chi2 = 0.0, sumsq = 0.0;
    for (int i = 0; i < n; i++) {
        double xnom[6];
        memcpy(xnom, x, sizeof xnom);
        propagate(xnom, dt[i]);
        for (int k = 0; k < 6; k++) {
            double r = y[i][k] - xnom[k];
            chi2 += r * r * w[i][k];
        }
        for (int k = 0; k < 3; k++) {
            double r = y[i][k] - xnom[k];
            sumsq += r * r;
        }
    }
    int dof = 6 * n - 6;
    double scale = 1.0;
    if (dof > 0) {
        double red = chi2 / dof;
        if (red > 1.0) scale = red;   // inflate optimistic formal covariance
    }

    // A valid covariance has positive variances; anything else means the fit
    // is degenerate. Combined with the finite checks below, this stops a bad
    // solution from ever reaching the OPM.
    for (int i = 0; i < 6; i++)
        if (!(Ninv[i][i] > 0.0) || !isfinite(Ninv[i][i])) return -1;
    for (int k = 0; k < 6; k++)
        if (!isfinite(x[k])) return -1;

    // Fill output, converting km -> m (every covariance entry scales by 1e6).
    for (int k = 0; k < 3; k++) { out->pos[k] = x[k] * 1000.0; out->vel[k] = x[k + 3] * 1000.0; }
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) out->cov[i][j] = Ninv[i][j] * scale * 1.0e6;
    // pos_rms is a literal root-mean-square of the position residuals over n
    // observations (a per-fix diagnostic). It deliberately divides by n, not
    // by the dof = 6n-6 used for the reduced chi-square above: that statistic
    // scales the formal covariance, this one reports the typical position miss.
    out->pos_rms = sqrt(sumsq / n) * 1000.0;
    out->n_obs = n;
    out->iterations = iter;
    out->ok = 1;
    return 0;
}
