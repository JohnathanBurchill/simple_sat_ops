/*

    Simple Satellite Operations  unit_tests/shortarc_selftest.c

    Tests for the short-arc least-squares orbit fit (src/orbit/shortarc.c).
    Real flight data has no closely-spaced valid GNSS fixes to fit, so the
    fixture is synthetic: a known epoch state is propagated (by an independent
    copy of the two-body+J2 integrator) to several earlier times to make the
    observations, then shortarc_fit must recover the epoch state. We check
    that (1) the integrator round-trips, (2) a single fix reduces to itself,
    (3) a clean arc is recovered to high precision, (4) the fit tightens the
    velocity uncertainty well below a single Doppler fix, and (5) a noisy arc
    is still recovered within a few sigma.

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
#include "tap.h"

#include <math.h>
#include <string.h>

#define check(cond, what) tap_ok((cond), (what))

// Independent replica of the dynamics (km, km/s) used only to synthesize the
// fixture arc -- same physics as the module so a clean arc recovers exactly.
#define MU 398600.4418
#define RE 6378.137
#define J2 1.08263e-3

static void accel(const double p[3], double a[3])
{
    double r2 = p[0]*p[0] + p[1]*p[1] + p[2]*p[2], r = sqrt(r2);
    double r3 = r2*r, r5 = r3*r2;
    a[0] = -MU*p[0]/r3; a[1] = -MU*p[1]/r3; a[2] = -MU*p[2]/r3;
    double z2 = p[2]*p[2], fac = 1.5*J2*MU*RE*RE/r5;
    double fxy = fac*(5.0*z2/r2 - 1.0), fz = fac*(5.0*z2/r2 - 3.0);
    a[0] -= fxy*p[0]; a[1] -= fxy*p[1]; a[2] -= fz*p[2];
}

static void step(double s[6], double dt)
{
    double k1[6],k2[6],k3[6],k4[6],t[6],a[3];
    accel(s,a); k1[0]=s[3];k1[1]=s[4];k1[2]=s[5];k1[3]=a[0];k1[4]=a[1];k1[5]=a[2];
    for (int i=0;i<6;i++) t[i]=s[i]+0.5*dt*k1[i];
    accel(t,a); k2[0]=t[3];k2[1]=t[4];k2[2]=t[5];k2[3]=a[0];k2[4]=a[1];k2[5]=a[2];
    for (int i=0;i<6;i++) t[i]=s[i]+0.5*dt*k2[i];
    accel(t,a); k3[0]=t[3];k3[1]=t[4];k3[2]=t[5];k3[3]=a[0];k3[4]=a[1];k3[5]=a[2];
    for (int i=0;i<6;i++) t[i]=s[i]+dt*k3[i];
    accel(t,a); k4[0]=t[3];k4[1]=t[4];k4[2]=t[5];k4[3]=a[0];k4[4]=a[1];k4[5]=a[2];
    for (int i=0;i<6;i++) s[i]+=dt/6.0*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
}

static void prop(double s[6], double dt)
{
    int n = (int)ceil(fabs(dt)/1.0); if (n<1) n=1;     // 1 s steps: finer than the module
    double h = dt/n;
    for (int i=0;i<n;i++) step(s,h);
}

// True epoch state (km, km/s): ~493 km, inclined so all axes and J2 engage.
static const double TRUE0[6] = { 6871.0, 0.0, 0.0, 0.0, 4.733, 5.967 };

static void make_obs(shortarc_obs_t *obs, int n, double cadence,
                     double sp, double sv, int noisy)
{
    // Deterministic, bounded "noise" pattern (no RNG), within +/-1 sigma.
    static const double pat[5] = { 0.6, -0.9, 0.3, -0.6, 0.8 };
    for (int i = 0; i < n; i++) {
        double dt = -cadence * (n - 1 - i);     // last obs at dt = 0 (the epoch)
        double s[6]; memcpy(s, TRUE0, sizeof s);
        prop(s, dt);
        obs[i].dt = dt;
        for (int k = 0; k < 3; k++) {
            double np = noisy ? sp * pat[(i + k) % 5]   : 0.0;
            double nv = noisy ? sv * pat[(i + k + 2) % 5] : 0.0;
            obs[i].pos[k] = s[k]   * 1000.0 + np;       // km -> m, + noise (m)
            obs[i].vel[k] = s[k+3] * 1000.0 + nv;
            obs[i].pos_sigma[k] = sp;
            obs[i].vel_sigma[k] = sv;
        }
    }
}

int main(void)
{
    // 1. Integrator round-trip (forward then back returns to start).
    double s[6]; memcpy(s, TRUE0, sizeof s);
    prop(s, 600.0); prop(s, -600.0);
    double back = 0.0;
    for (int k = 0; k < 3; k++) back += fabs(s[k] - TRUE0[k]);
    check(back < 1e-6, "integrator round-trips to < 1e-6 km");

    const double SP = 2.4, SV = 0.057;   // m, m/s (typical SINGLE fix)

    // 2. Single fix reduces to itself with diagonal covariance.
    {
        shortarc_obs_t o; make_obs(&o, 1, 60.0, SP, SV, 0);
        shortarc_fit_t f;
        check(shortarc_fit(&o, 1, &f) == 0 && f.ok, "n=1 fit succeeds");
        check(fabs(f.pos[0] - o.pos[0]) < 1e-6, "n=1 returns the fix position");
        check(fabs(sqrt(f.cov[0][0]) - SP) < 1e-6, "n=1 position sigma = input");
        check(fabs(sqrt(f.cov[3][3]) - SV) < 1e-9, "n=1 velocity sigma = input");
    }

    // 3. Clean arc (8 fixes, 60 s cadence = 7 min) recovers the epoch state.
    {
        shortarc_obs_t o[8]; make_obs(o, 8, 60.0, SP, SV, 0);
        shortarc_fit_t f;
        check(shortarc_fit(o, 8, &f) == 0 && f.ok, "clean-arc fit succeeds");
        double dp = 0.0, dv = 0.0;
        for (int k = 0; k < 3; k++) {
            dp += fabs(f.pos[k] - TRUE0[k] * 1000.0);
            dv += fabs(f.vel[k] - TRUE0[k + 3] * 1000.0);
        }
        check(dp < 0.5, "clean arc recovers position to < 0.5 m");
        check(dv < 5e-4, "clean arc recovers velocity to < 5e-4 m/s");
        check(f.pos_rms < 1e-3, "clean-arc residual RMS < 1 mm");

        // 4. The fit tightens velocity well below a single Doppler fix.
        double svx = sqrt(f.cov[3][3]);
        check(svx < SV / 3.0, "fit velocity sigma at least 3x better than one fix");
        check(sqrt(f.cov[0][0]) <= SP + 1e-6, "fit position sigma no worse than one fix");
    }

    // 5. Noisy arc still recovered within a few sigma.
    {
        shortarc_obs_t o[8]; make_obs(o, 8, 60.0, SP, SV, 1);
        shortarc_fit_t f;
        check(shortarc_fit(o, 8, &f) == 0 && f.ok, "noisy-arc fit succeeds");
        double dp = 0.0;
        for (int k = 0; k < 3; k++) dp += fabs(f.pos[k] - TRUE0[k] * 1000.0);
        check(dp < 3.0 * SP, "noisy arc recovers position within a few sigma");
        check(f.iterations >= 1 && f.iterations <= 8, "fit converged in a sane iteration count");
    }

    return tap_done();
}
