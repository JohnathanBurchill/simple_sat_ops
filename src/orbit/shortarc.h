/*

    Simple Satellite Operations  shortarc.h

    Short-arc least-squares orbit fit. Given several GNSS state vectors
    (position + velocity, each with a per-axis 1-sigma) gathered over a short
    interval, estimate one epoch state and its covariance by a weighted batch
    least-squares fit through the orbit dynamics. The point is velocity: a
    single GNSS fix knows velocity only to the Doppler level (~cm/s), but a
    span of position fixes pins it far better, which is what dominates the
    along-track error when the state is propagated for the SpaceX Space Safety
    upload.

    The dynamics here are two-body + J2 with NO Earth-rotation terms, applied
    directly to the Earth-fixed (ITRF) state -- deliberately identical to the
    space_safety_manager propagator (its j2_acceleration / rk4_step), so the
    fitted epoch state is consistent with the trajectory ssm later propagates
    from it. (Treating the ITRF state as inertial is ssm's simplification; we
    match it on purpose rather than being "more correct" and inconsistent.)

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

#ifndef SSO_SHORTARC_H
#define SSO_SHORTARC_H

// One GNSS observation. Distances are metres, velocities metres/second, all
// in the Earth-fixed (ITRF) frame. dt is the observation time relative to the
// fit epoch in seconds (negative for fixes before the epoch). The sigmas are
// the receiver per-axis 1-sigma (treated as a diagonal covariance).
typedef struct {
    double dt;             // s, relative to the fit epoch
    double pos[3];         // m, ITRF
    double vel[3];         // m/s, ITRF
    double pos_sigma[3];   // m, 1-sigma
    double vel_sigma[3];   // m/s, 1-sigma
} shortarc_obs_t;

// The fitted epoch state and its covariance. State order is
// [x, y, z, vx, vy, vz]; covariance units are m^2 / (m^2/s) / (m^2/s^2).
typedef struct {
    double pos[3];         // m, ITRF, at the fit epoch
    double vel[3];         // m/s, ITRF, at the fit epoch
    double cov[6][6];      // 6x6 covariance at the fit epoch
    double pos_rms;        // m, post-fit position residual RMS
    int    n_obs;          // observations used
    int    iterations;     // Gauss-Newton iterations taken
    int    ok;             // 1 if the fit converged and the normals inverted
} shortarc_fit_t;

// Fit an epoch state to obs[0..n-1] by weighted Gauss-Newton batch least
// squares. The a priori is the observation nearest the epoch (dt closest to
// 0). With n == 1 the result is just that fix and its diagonal covariance.
// Returns 0 on success (out->ok == 1), -1 on failure (singular normals, bad
// input); on -1 out->ok == 0.
int shortarc_fit(const shortarc_obs_t *obs, int n, shortarc_fit_t *out);

#endif
