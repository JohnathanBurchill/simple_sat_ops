/*

   Simple Satellite Operations  control/tracking.c

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

#include "tracking.h"
#include "state.h"

#include "antenna_rotator.h"
#include "antenna_rotator_async.h"
#include "panels.h"          // compute_predictions
#include "prediction.h"
#include "pursuit.h"
#include "scan_sky.h"
#include "cmd_line.h"
#include "sso_audit.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// Range-check, write target_* bookkeeping, and submit the wire-level SET
// to the rotator worker. Replaces the previous direct
// antenna_rotator_set_unwrapped() call; the target / wrap bookkeeping
// stays on the main thread, only the serial I/O moves to the worker.
int main_rotator_submit_set(state_t *state,
                                    double az_unwrapped, double elevation)
{
    if (az_unwrapped < ANTENNA_ROTATOR_MINIMUM_AZIMUTH
        || az_unwrapped > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
        return ANTENNA_ROTATOR_AZIMUTH_LIMIT;
    }
    if (elevation < ANTENNA_ROTATOR_MINIMUM_ELEVATION
        || elevation > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
        return ANTENNA_ROTATOR_ELEVATION_LIMIT;
    }
    state->antenna_rotator.target_azimuth_unwrapped = az_unwrapped;
    state->antenna_rotator.target_azimuth           = az_unwrapped;
    state->antenna_rotator.target_elevation         = elevation;
    state->antenna_rotator.unwrapped_target_valid   = 1;
    if (state->rot_async != NULL) {
        antenna_rotator_async_submit_set(state->rot_async, az_unwrapped, elevation);
    }
    return ANTENNA_ROTATOR_OK;
}

// Mirror of antenna_rotator_increase_azimuth() but routed through the
// async worker via main_rotator_submit_set(). Used by the `[` / `]`
// (5 deg) and `{` / `}` (1 deg, shifted) hotkeys.
int main_rotator_increase_azimuth(state_t *state, double delta)
{
    double base = state->antenna_rotator.unwrapped_target_valid
        ? state->antenna_rotator.target_azimuth_unwrapped
        : state->antenna_rotator.target_azimuth;
    return main_rotator_submit_set(state, base + delta,
                                    state->antenna_rotator.target_elevation);
}

// Same idea but stepping elevation. Used by the `,` / `.` (5 deg) and
// `<` / `>` (1 deg, shifted) hotkeys. The azimuth target is held — only
// the wire-level SET goes out, on the worker.
int main_rotator_increase_elevation(state_t *state, double delta)
{
    double az = state->antenna_rotator.unwrapped_target_valid
        ? state->antenna_rotator.target_azimuth_unwrapped
        : state->antenna_rotator.target_azimuth;
    double new_el = state->antenna_rotator.target_elevation + delta;
    return main_rotator_submit_set(state, az, new_el);
}

// --- Pursuit planner integration ----------------------------------
//
// At AOS we pre-sample the satellite trajectory in unwrapped mech
// coords into state->pursuit_track, then ask the planner (src/orbit/
// pursuit.c) for a rate-feasible whole-pass antenna trajectory. Each
// track-loop tick the loop reads the next waypoint via
// pursuit_aim_at() and submits it through the existing
// main_rotator_submit_set() path — the playback is just "aim at the
// next waypoint", and the worker's constant-rate slew interpolates
// the segment for us. The planner runs once per pass; mid-pass the
// only work is the O(log N) waypoint lookup.

// Free the pre-sampled trajectory arrays. Idempotent.
static void pursuit_track_free(pursuit_track_t *trk)
{
    if (trk == NULL) return;
    free(trk->t_jul);
    free(trk->az_unwrapped);
    free(trk->el);
    trk->t_jul        = NULL;
    trk->az_unwrapped = NULL;
    trk->el           = NULL;
    trk->n            = 0;
}

// pursuit_sat_sample_fn_t backing the planner. Linear-interpolates the
// pre-sampled track at `jul`. Saturates at the endpoints so iterations
// that wander a fraction of a second beyond AOS / LOS still produce a
// sensible answer.
static int pursuit_track_lookup(double jul, double *az, double *el, void *ctx)
{
    const pursuit_track_t *trk = (const pursuit_track_t *) ctx;
    if (trk == NULL || trk->n == 0) return -1;
    if (jul <= trk->t_jul[0]) {
        if (az) *az = trk->az_unwrapped[0];
        if (el) *el = trk->el[0];
        return 0;
    }
    if (jul >= trk->t_jul[trk->n - 1]) {
        if (az) *az = trk->az_unwrapped[trk->n - 1];
        if (el) *el = trk->el[trk->n - 1];
        return 0;
    }
    size_t lo = 0, hi = trk->n - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (trk->t_jul[mid] <= jul) lo = mid;
        else hi = mid;
    }
    double frac = (jul - trk->t_jul[lo])
                / (trk->t_jul[hi] - trk->t_jul[lo]);
    if (az) *az = trk->az_unwrapped[lo]
                + (trk->az_unwrapped[hi] - trk->az_unwrapped[lo]) * frac;
    if (el) *el = trk->el[lo]
                + (trk->el[hi] - trk->el[lo]) * frac;
    return 0;
}

// Sample the live prediction's satellite at 1 s intervals across the
// pass window, run it through the existing flip mech-coord mapping (so
// flip-mode passes get back-hemisphere mech_el up to 180), accumulate
// unwrapped azimuth in time order. We work on a memcpy of
// state->prediction so the live satellite_ephem.azimuth/elevation
// displayed in the UI is not perturbed. Returns 0 on success.
static int pursuit_track_build(const state_t *state,
                                double jul_aos, double jul_los,
                                int flip,
                                double aos_az, double los_az,
                                double aos_jul, double los_jul,
                                double a0_unwrapped,
                                pursuit_track_t *out)
{
    if (out == NULL) return -1;
    pursuit_track_free(out);
    double dt_days = 1.0 / 86400.0;     // 1 s sampling
    if (jul_los <= jul_aos) return -1;
    size_t n = (size_t) floor((jul_los - jul_aos) / dt_days) + 1;
    if (n < 2)    n = 2;
    if (n > 4096) n = 4096;             // sanity cap; ~68 min pass
    out->t_jul        = calloc(n, sizeof *out->t_jul);
    out->az_unwrapped = calloc(n, sizeof *out->az_unwrapped);
    out->el           = calloc(n, sizeof *out->el);
    if (out->t_jul == NULL || out->az_unwrapped == NULL
        || out->el == NULL) {
        pursuit_track_free(out);
        return -1;
    }
    out->n = n;

    prediction_t scratch;
    memcpy(&scratch, &state->prediction, sizeof scratch);

    double prev = a0_unwrapped;
    double span = jul_los - jul_aos;
    for (size_t i = 0; i < n; ++i) {
        double frac = (n == 1) ? 0.0 : (double) i / (double) (n - 1);
        double t = jul_aos + frac * span;
        out->t_jul[i] = t;
        update_satellite_position(&scratch, t);
        double sat_az = scratch.satellite_ephem.azimuth;
        double sat_el = scratch.satellite_ephem.elevation;
        double mech_az = sat_az;
        double mech_el = sat_el;
        if (flip) {
            double progress = 0.0;
            double pass_jul = los_jul - aos_jul;
            if (pass_jul > 0.0) progress = (t - aos_jul) / pass_jul;
            int half = 0;
            antenna_rotator_to_mech_coords(1, aos_az, los_az, progress,
                                            sat_az, sat_el,
                                            &mech_az, &mech_el, &half);
        }
        prev = antenna_rotator_accumulate_unwrapped(prev, mech_az);
        out->az_unwrapped[i] = prev;
        out->el[i] = mech_el;
    }
    return 0;
}

// Free the current plan + its sampled trajectory. Idempotent.
void main_pursuit_clear_plan(state_t *state)
{
    pursuit_plan_free(&state->pursuit_plan);
    pursuit_track_free(&state->pursuit_track);
}

// Build (or rebuild) the whole-pass plan from the current prediction.
// `jul_now` is used to clamp the plan start to the current time if
// we're already past AOS (mid-pass re-enter). Quietly does nothing
// when pursuit is disabled or prerequisites are missing — the caller
// just keeps the existing aim-where-sat-is-now logic.
void main_pursuit_build_plan(state_t *state, double jul_now)
{
    main_pursuit_clear_plan(state);
    if (state->without_rotator_pursuit)              return;
    if (state->pursuit_az_dps <= 0.0)                return;
    if (state->pursuit_el_dps <= 0.0)                return;
    if (!state->have_antenna_rotator)           return;
    if (!state->antenna_rotator.unwrapped_target_valid) return;

    double aos = state->prediction.predicted_ascension_jul_utc;
    double los = state->prediction.predicted_descent_jul_utc;
    if (aos <= 0.0 || los <= aos)               return;
    if (jul_now > aos) aos = jul_now;
    if (los - aos < 5.0 / 86400.0)              return;  // <5 s left

    int    flip    = state->antenna_rotator.flip_mode_pass;
    double aos_az  = state->antenna_rotator.flip_aos_az;
    double los_az  = state->antenna_rotator.flip_los_az;
    double aos_jul = state->antenna_rotator.flip_aos_jul;
    double los_jul = state->antenna_rotator.flip_los_jul;
    double a0      = state->antenna_rotator.target_azimuth_unwrapped;
    double e0      = state->antenna_rotator.target_elevation;

    if (pursuit_track_build(state, aos, los,
                             flip, aos_az, los_az, aos_jul, los_jul,
                             a0, &state->pursuit_track) != 0) {
        fprintf(stderr, "pursuit: track sampling failed; "
                        "falling back to aim-where-sat-is-now\n");
        main_pursuit_clear_plan(state);
        return;
    }

    pursuit_config_t cfg;
    pursuit_config_defaults(&cfg);
    cfg.jul_aos      = aos;
    cfg.jul_los      = los;
    cfg.r_az_dps     = state->pursuit_az_dps;
    cfg.r_el_dps     = state->pursuit_el_dps;
    cfg.a0_unwrapped = a0;
    cfg.e0           = e0;

    if (pursuit_plan_build(&cfg, pursuit_track_lookup, &state->pursuit_track,
                            &state->pursuit_plan) != 0) {
        fprintf(stderr, "pursuit: plan build failed; "
                        "falling back to aim-where-sat-is-now\n");
        main_pursuit_clear_plan(state);
        return;
    }
    // Sanity bound: a plan with > 30 deg max error is suspect; the
    // calibration may be wildly off. Discard and let the track loop
    // fall back rather than driving the antenna to bogus targets.
    if (state->pursuit_plan.max_error_deg > 30.0) {
        fprintf(stderr,
                "pursuit: plan max_err=%.1f deg > 30; disabled, "
                "falling back to aim-where-sat-is-now\n",
                state->pursuit_plan.max_error_deg);
        main_pursuit_clear_plan(state);
        return;
    }
    fprintf(stderr,
            "pursuit: plan built %zu waypoints, "
            "max_err=%.2f mean_err=%.2f deg, %d iter\n",
            state->pursuit_plan.n_waypoints,
            state->pursuit_plan.max_error_deg, state->pursuit_plan.mean_error_deg,
            state->pursuit_plan.iterations_used);
}

// Two paths point at the same TLE file? Canonicalise both with
// realpath() so /a/./tle and /a/tle compare equal; fall back to a
// plain string compare if either path can't be resolved (e.g. a
// relative path whose file was just removed). NULL/empty never match.
static int retarget_same_file(const char *a, const char *b)
{
    if (a == NULL || b == NULL || a[0] == '\0' || b[0] == '\0') return 0;
    char *ra = realpath(a, NULL);
    char *rb = realpath(b, NULL);
    int same = strcmp(ra ? ra : a, rb ? rb : b) == 0;
    free(ra);
    free(rb);
    return same;
}

// Read the FIRST satellite from a 3-line TLE file: its name line and the
// two element lines, packed into the 139-byte buffer sgp4sdp4 wants
// (two 69-char lines joined, NUL-padded). Stops at the first complete
// record. Returns 0 on success, -1 if the file can't be opened or a
// full name+line1+line2 triple isn't present.
static int retarget_read_first_tle(const char *path,
                                   char *name, size_t name_cap,
                                   char tle_out[139])
{
    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;
    char line[256];
    char l1[256] = {0};
    char l2[256] = {0};
    int  have_name = 0;
    name[0] = '\0';
    while (fgets(line, sizeof line, f) != NULL) {
        size_t n = strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'
                      || line[n - 1] == ' '  || line[n - 1] == '\t')) {
            line[--n] = '\0';
        }
        if (n == 0) continue;
        int is_elem = ((line[0] == '1' || line[0] == '2') && line[1] == ' ');
        if (!have_name && !is_elem) {
            snprintf(name, name_cap, "%s", line);
            have_name = 1;
            continue;
        }
        if (line[0] == '1' && line[1] == ' ' && l1[0] == '\0') {
            snprintf(l1, sizeof l1, "%s", line);
            continue;
        }
        if (line[0] == '2' && line[1] == ' ' && l2[0] == '\0') {
            snprintf(l2, sizeof l2, "%s", line);
            break;  // first complete record -> done
        }
    }
    fclose(f);
    if (!have_name || l1[0] == '\0' || l2[0] == '\0') return -1;
    size_t a = strlen(l1);
    size_t b = strlen(l2);
    if (a > 69) a = 69;
    if (b > 69) b = 69;
    memset(tle_out, 0, 139);
    memcpy(tle_out, l1, a);
    memcpy(tle_out + 69, l2, b);
    return 0;
}

// Swap the tracked satellite mid-pass to the first one in `path`.
//
// The new target's elements replace the live ephemeris; the SGP4/SDP4
// selection (global flag state) is re-picked for it. Pass geometry is
// recomputed from scratch so the display, flip decision and pursuit
// plan all reflect the new object. The antenna is NOT homed first: we
// clear the flip latch + per-pass tracking flag so the track loop
// re-derives and aims straight at the new sat's current sky position,
// and we keep target_azimuth_unwrapped so the azimuth accumulator picks
// the co-terminal nearest where the antenna already is (short slew, no
// trip through 0,0).
//
// A repeat :retarget on the same file is a no-op (RETARGET_SAME);
// different files swap even when they name the same satellite. Returns
// one of the RETARGET_* codes.
int retarget_to_tle(state_t *state, const char *path)
{
    if (path == NULL || path[0] == '\0') return RETARGET_BAD_ARG;
    if (retarget_same_file(path, state->target_tle_path)) return RETARGET_SAME;

    char name[64];
    char tle[139];
    if (retarget_read_first_tle(path, name, sizeof name, tle) != 0) {
        return RETARGET_READ_ERR;
    }
    if (!Good_Elements(tle)) return RETARGET_BAD_TLE;

    // Commit the new elements. select_ephemeris rewrites the TLE in
    // place, so it must be called exactly once on these freshly
    // converted elements -- clear the global flags first so the
    // SGP4/SDP4 choice is made fresh for this object.
    snprintf(state->target_name, sizeof state->target_name, "%s", name);
    Convert_Satellite_Data(tle, &state->prediction.satellite_ephem.tle);
    snprintf(state->prediction.satellite_ephem.tle.sat_name,
             sizeof state->prediction.satellite_ephem.tle.sat_name,
             "%s", name);
    state->prediction.satellite_ephem.name = state->target_name;
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state->prediction.satellite_ephem.tle);

    // Recompute pass geometry for the new target. Reset max-elevation to
    // the sentinel so compute_predictions walks back to AOS when we're
    // already mid-pass (otherwise it would only see the pass remainder).
    state->prediction.predicted_max_elevation = -180.0;
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_now = Julian_Date(&utc, &tv);
    update_satellite_position(&state->prediction, jul_now);
    compute_predictions(state, jul_now);

    // Re-aim without homing: drop the old plan and clear the per-pass
    // latches. An active track (satellite_tracking set) re-derives the
    // flip decision, rebuilds the pursuit plan, and slews to the new
    // target on the next tick. If we weren't tracking, only the target
    // changes -- nothing moves until the operator presses T.
    main_pursuit_clear_plan(state);
    state->antenna_rotator.tracking           = 0;
    state->antenna_rotator.flip_mode_pass     = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half          = 0;

    snprintf(state->target_tle_path, sizeof state->target_tle_path, "%s", path);
    return RETARGET_OK;
}

// Pull az/el from the async snapshot and write them through to az/el AND
// target_* on state. Used after a STOP / on tracking start when targets
// should reflect the physical position. Returns 0 on success, -1 if no
// good status has landed yet (or it's gone stale).
int main_rotator_refresh_targets_from_snapshot(state_t *state)
{
    if (state->rot_async == NULL) return -1;
    double az = 0.0, el = 0.0;
    int    ok = 0, stale_ms = 0;
    antenna_rotator_async_snapshot(state->rot_async, &az, &el, &ok, &stale_ms, NULL);
    if (!ok || stale_ms > 1500) return -1;
    state->antenna_rotator.azimuth                  = az;
    state->antenna_rotator.elevation                = el;
    state->antenna_rotator.target_azimuth           = az;
    state->antenna_rotator.target_elevation         = el;
    state->antenna_rotator.target_azimuth_unwrapped = az;
    state->antenna_rotator.unwrapped_target_valid   = 1;
    return 0;
}

void start_tracking(state_t *state)
{
    int antenna_rotator_result = 0;

    state->satellite_tracking = 1;
    state->doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control =
        state->antenna_rotator.antenna_should_be_controlled;
    // Clear the flip latch so the next tracking-enable re-decides for
    // the upcoming pass.
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;
    if (state->antenna_rotator.fixed_target) {
        antenna_rotator_result = main_rotator_submit_set(state,
            state->antenna_rotator.target_azimuth_unwrapped,
            state->antenna_rotator.target_elevation);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error setting antenna rotator position\n");
        } else {
            state->antenna_rotator.antenna_is_moving = 1;
        }
    }
}

void stop_tracking(state_t *state)
{
    state->satellite_tracking = 0;
    state->doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control = 0;
    if (state->run_with_antenna_rotator && state->rot_async != NULL) {
        antenna_rotator_async_submit_stop(state->rot_async);
        antenna_rotator_async_kick_status(state->rot_async);
        // Wait briefly for the next OK STATUS so target_* reflect the
        // position the antenna actually stopped at (not where the
        // satellite was). Bounded — the operator's 's' / 'r' keystroke
        // shouldn't hang if the controller is unresponsive.
        if (antenna_rotator_async_wait_next_good_status(state->rot_async, 750) == 0) {
            (void) main_rotator_refresh_targets_from_snapshot(state);
        }
    }
    state->antenna_rotator.antenna_is_moving = 0;
    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;
}

int point_to_stationary_target(state_t *state, double azimuth, double elevation)
{
    state->satellite_tracking = 0;
    state->antenna_rotator.antenna_is_under_control = 0;
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;

    // Re-sync the unwrap accumulator to the antenna's *physical* position
    // before computing the home move. target_azimuth_unwrapped can drift
    // from reality across a pass -- e.g. it can hold a negative co-terminal
    // that the wrapped status display still renders as a positive angle --
    // which made "home" drive the long way around instead of unwinding.
    // Seeding prev from the live status makes r/home always move from where
    // the antenna actually is. If no fresh status is available, fall back to
    // the last known target, and only fail if we have neither.
    if (main_rotator_refresh_targets_from_snapshot(state) != 0
        && !state->antenna_rotator.unwrapped_target_valid) {
        return ANTENNA_ROTATOR_BAD_RESPONSE;
    }

    double prev = state->antenna_rotator.target_azimuth_unwrapped;
    double final_az = antenna_rotator_home_unwrapped_target(prev, azimuth);
    double delta = final_az - prev;

    // Already at the target (azimuth and elevation both within the rotator's
    // deadband)? Do nothing instead of issuing a zero-length SET -- on this
    // controller a SET kicks off a couple of seconds of target-echo on STATUS
    // and a spurious "moving" state, for no actual move. delta is the raw
    // rotation the unwind needs, so a wound antenna (|delta| large) still
    // moves; only a genuine no-op is skipped.
    if (fabs(delta) <= MAX_DELTA_AZIMUTH_DEGREES
        && fabs(elevation - state->antenna_rotator.elevation)
               <= MAX_DELTA_ELEVATION_DEGREES) {
        state->antenna_rotator.target_azimuth_unwrapped = final_az;
        state->antenna_rotator.target_azimuth           = final_az;
        state->antenna_rotator.target_elevation         = elevation;
        state->antenna_rotator.unwrapped_target_valid   = 1;
        state->antenna_rotator.homing_in_progress       = 0;
        state->antenna_rotator.home_pending_final_az    = 0.0;
        cmd_set_status(state, "home: already at %.1f, %.1f deg -- no move",
                       prev, state->antenna_rotator.elevation);
        sso_audit_event("home", "already at target -- no move");
        return ANTENNA_ROTATOR_OK;
    }

    // Trace the home decision (audit log + a brief on-screen line) so the
    // unwind can be confirmed at the rotator: |delta| > 180 takes the
    // two-step unwind (mid waypoint, then the final leg once it's reached
    // the 0..180 zone -- see the loop), otherwise a direct move.
    {
        char det[128];
        snprintf(det, sizeof det, "from az=%.1f to %.1f delta=%+.1f (%s)",
                 prev, final_az, delta,
                 (fabs(delta) > 180.0) ? "two-step unwind" : "direct");
        sso_audit_event("home", det);
        cmd_set_status(state, "home: %.1f -> %.1f, %+.1f deg %s (%s)",
                       prev, final_az, delta, delta < 0.0 ? "CCW" : "CW",
                       (fabs(delta) > 180.0) ? "unwind" : "direct");
    }

    if (fabs(delta) > 180.0) {
        // Two-step home: halfway waypoint first to disambiguate the
        // direction of rotation; the main loop drives the second leg
        // once the antenna has stopped at the intermediate.
        double mid = prev + delta / 2.0;
        if (mid < ANTENNA_ROTATOR_MINIMUM_AZIMUTH)
            mid = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
        if (mid > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH)
            mid = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
        state->antenna_rotator.home_pending_final_az = final_az;
        state->antenna_rotator.homing_in_progress = 1;
        int rc = main_rotator_submit_set(state, mid, elevation);
        if (rc == ANTENNA_ROTATOR_OK) {
            state->antenna_rotator.antenna_is_moving = 1;
        }
        return rc;
    }

    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    int rc = main_rotator_submit_set(state, final_az, elevation);
    if (rc == ANTENNA_ROTATOR_OK) {
        state->antenna_rotator.antenna_is_moving = 1;
    }
    return rc;
}


void update_doppler_shifted_frequencies(state_t *state,
                                          double uplink_freq,
                                          double downlink_freq)
{
    double doppler_factor = 1.0
        - state->prediction.satellite_ephem.range_rate_km_s / 299792.458;
    state->doppler_uplink_frequency_hz   = uplink_freq   * doppler_factor;
    state->doppler_downlink_frequency_hz = downlink_freq * doppler_factor;
}


// HOME_ECHO_TOLERANCE_DEG: how close a STATUS azimuth must be to the
// just-commanded home waypoint to be treated as the controller's post-SET
// target echo -- which it reports for a couple of seconds before its
// feedback shows real motion -- rather than a real position reading. Gates
// the two-step home's final leg.
#define HOME_ECHO_TOLERANCE_DEG 2.0

// Per-tick antenna pointing. Detect motion-settle, drive the second leg of a
// two-step home, advance an active sky scan, and -- when a pass is in range --
// run the satellite-tracking / pursuit-plan aim loop (or release the rotator
// and tear down the planner at LOS). All pointing state lives on
// state->antenna_rotator; jul_utc is the current Julian date, t_now the
// monotonic-seconds clock (drives the scan dwell).
void tracking_tick(state_t *state, double jul_utc, double t_now)
{
    // These persist across ticks. last_az/last_el feed the settle check;
    // jul_idle_start is reserved for a future idle-window behavior.
    static double last_az = 0.0;
    static double last_el = 0.0;
    static double jul_idle_start = 0.0;
    double delta_az = 0.0;
    double delta_el = 0.0;

    double current_az = state->antenna_rotator.azimuth;
    double current_el = state->antenna_rotator.elevation;
        if (state->antenna_rotator.antenna_is_moving) {
            if (fabs(current_az - last_az) == 0
                && fabs(current_el - last_el) == 0) {
                state->antenna_rotator.antenna_is_moving = 0;
            }
            last_az = current_az;
            last_el = current_el;
        }

        // Drive the second leg of a two-step home. The first leg drops a mid
        // waypoint to start the antenna unwinding; the final 'go to target'
        // must wait until the antenna has unwound far enough that the
        // controller's SHORT path to the target runs the SAME way as the
        // unwind -- i.e. it has reached the 0..180 zone on the unwinding side.
        // Until then the short path is the opposite (winding) way, so issuing
        // the target now sends it back around and it winds up (330 -> 360).
        //
        // Complication: after a SET the controller's STATUS reports the
        // just-commanded target (the mid waypoint) for a couple of seconds
        // before its feedback shows real motion. So a reading that still
        // equals the commanded mid is that echo, not the real position --
        // ignore it. The first reading that DIFFERS is the antenna's true
        // position; act on that. Mid-slew the real position is far from the
        // mid waypoint, so there's no echo-vs-arrival ambiguity. (Unwinds
        // past a full turn, prev > 360, would need more than one waypoint;
        // a single pass winds < 360, so one mid waypoint suffices.)
        if (state->antenna_rotator.homing_in_progress
            && state->have_antenna_rotator) {
            double final_az  = state->antenna_rotator.home_pending_final_az;
            double mid_az    = state->antenna_rotator.target_azimuth_unwrapped;
            double from_mid  = fabs(antenna_rotator_wrap_to_pm180(current_az - mid_az));
            double unwind    = final_az - mid_az;   // sign = unwind direction
            double remaining = antenna_rotator_wrap_to_pm180(final_az - current_az);
            int in_zone = (remaining == 0.0)
                       || ((remaining > 0.0) == (unwind > 0.0));
            // from_mid > tol => the reading is real feedback, not the post-SET
            // target echo. The two-step always starts out of the unwind zone
            // (|prev| > 180), so the stale pre-SET reading can't fire early.
            if (from_mid > HOME_ECHO_TOLERANCE_DEG && in_zone) {
                int rc = main_rotator_submit_set(state, final_az, 0.0);
                if (rc == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                state->antenna_rotator.homing_in_progress = 0;
                state->antenna_rotator.home_pending_final_az = 0.0;
                char det[96];
                snprintf(det, sizeof det, "leg2 fired at az=%.1f -> %.1f",
                         current_az, final_az);
                sso_audit_event("home", det);
            }
        }
        // --scan-sky: drives a sky grid one target at a time, dwelling
        // SCAN_DWELL_S at each. Bypasses the satellite_tracking +
        // pass-timing gate below entirely, so the operator can scan
        // regardless of TLE / pass state-> 's' stops mid-scan.
        if (state->scan.active) {
            scan_sky_tick(state, t_now);
        }
        if (state->satellite_tracking
            && state->prediction.predicted_minutes_until_visible
                   < state->antenna_rotator.tracking_prep_time_minutes) {
            if (!state->in_pass) {
                state->in_pass = 1;
            }
            if (state->antenna_rotator.antenna_should_be_controlled
                && !state->antenna_rotator.tracking) {
                if (!state->antenna_rotator.fixed_target
                    && !state->antenna_rotator.flip_decision_made) {
                    state->antenna_rotator.flip_mode_pass = 0;
                    state->antenna_rotator.flip_half = 0;
                    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION > 90
                        && state->prediction.predicted_max_elevation
                               >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD) {
                        state->antenna_rotator.flip_mode_pass = 1;
                        // Prefer the prediction-derived AOS azimuth (the
                        // satellite_ephem.azimuth here may be a few deg
                        // off as we are still pre-AOS); fall back to the
                        // live position if the pass walk didn't capture
                        // an ascension sample.
                        double aos_az_pred =
                            state->prediction.predicted_ascension_azimuth;
                        state->antenna_rotator.flip_aos_az =
                            (aos_az_pred != 0.0)
                                ? aos_az_pred
                                : state->prediction.satellite_ephem.azimuth;
                        state->antenna_rotator.flip_los_az =
                            state->prediction.predicted_descent_azimuth;
                        state->antenna_rotator.flip_aos_jul =
                            state->prediction.predicted_ascension_jul_utc;
                        state->antenna_rotator.flip_los_jul =
                            state->prediction.predicted_descent_jul_utc;
                    }
                    state->antenna_rotator.flip_decision_made = 1;
                    state->antenna_rotator.tracking = 1;
                    // Pre-sample the trajectory and ask the planner
                    // for a rate-feasible whole-pass aim sequence. On
                    // any failure (no calibration, planner unhappy,
                    // --without-rotator-pursuit) the helper leaves
                    // state->pursuit_plan zero and the track loop below
                    // falls back to today's aim-where-sat-is-now path.
                    main_pursuit_build_plan(state, jul_utc);
                }
            }

            if (state->antenna_rotator.tracking
                && state->antenna_rotator.antenna_is_under_control) {
                if (!state->antenna_rotator.unwrapped_target_valid) {
                    if (main_rotator_refresh_targets_from_snapshot(state)
                        != 0) {
                        state->antenna_rotator.tracking = 0;
                        main_pursuit_clear_plan(state);
                    }
                } else if (!state->antenna_rotator.antenna_is_moving) {
                    double next_az = 0.0, next_el = 0.0;
                    double prev_unwrapped =
                        state->antenna_rotator.target_azimuth_unwrapped;
                    int    used_pursuit = 0;
                    if (state->pursuit_plan.waypoints != NULL
                        && pursuit_aim_at(&state->pursuit_plan, jul_utc,
                                          &next_az, &next_el) == 0) {
                        used_pursuit = 1;
                    }
                    if (!used_pursuit) {
                        double pred_az =
                            state->prediction.satellite_ephem.azimuth;
                        double pred_el =
                            state->prediction.satellite_ephem.elevation;
                        double mech_az = pred_az;
                        double mech_el = pred_el;
                        int half = 0;
                        // AOS->LOS progress: drives the boom-meridian
                        // lerp in flip mode. Clamped to [0, 1] inside
                        // the function. Ignored for non-flip passes.
                        double progress = 0.0;
                        double pass_jul =
                            state->antenna_rotator.flip_los_jul
                            - state->antenna_rotator.flip_aos_jul;
                        if (pass_jul > 0.0) {
                            progress = (jul_utc
                                        - state->antenna_rotator.flip_aos_jul)
                                       / pass_jul;
                        }
                        antenna_rotator_to_mech_coords(
                            state->antenna_rotator.flip_mode_pass,
                            state->antenna_rotator.flip_aos_az,
                            state->antenna_rotator.flip_los_az,
                            progress,
                            pred_az, pred_el,
                            &mech_az, &mech_el, &half);
                        if (state->antenna_rotator.flip_mode_pass
                            && half != state->antenna_rotator.flip_half) {
                            state->antenna_rotator.target_azimuth_unwrapped =
                                mech_az;
                            state->antenna_rotator.flip_half = half;
                            prev_unwrapped =
                                state->antenna_rotator.target_azimuth_unwrapped;
                        }
                        next_az = antenna_rotator_accumulate_unwrapped(
                            prev_unwrapped, mech_az);
                        next_el = mech_el;
                    }
                    if (next_el < ANTENNA_ROTATOR_MINIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MINIMUM_ELEVATION;
                    } else if (next_el > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                    }
                    delta_az = next_az - prev_unwrapped;
                    // With a plan in play the elevation is part of the
                    // trajectory; respect it even below the horizon.
                    // The pre-pursuit fallback keeps the existing
                    // "only chase el while the sat is visible" rule.
                    if (used_pursuit
                        || state->antenna_rotator.flip_mode_pass
                        || state->prediction.satellite_ephem.elevation >= 0) {
                        delta_el = next_el
                                   - state->antenna_rotator.target_elevation;
                    } else {
                        delta_el = 0.0;
                    }

                    if (fabs(delta_az) >= MAX_DELTA_AZIMUTH_DEGREES
                        || fabs(delta_el) >= MAX_DELTA_ELEVATION_DEGREES) {
                        if (next_az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH
                            || next_az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                            state->antenna_rotator.tracking = 0;
                            main_pursuit_clear_plan(state);
                        } else {
                            int rc = main_rotator_submit_set(
                                state, next_az, next_el);
                            if (rc != ANTENNA_ROTATOR_OK) {
                                fprintf(stderr,
                                        "Error setting antenna rotator position\n");
                            } else {
                                state->antenna_rotator.antenna_is_moving = 1;
                            }
                        }
                    }
                }
            }

            jul_idle_start = 0;
        } else {
            if (state->in_pass) {
                state->in_pass = 0;
                jul_idle_start = jul_utc;
            }
            if (state->antenna_rotator.tracking) {
                state->antenna_rotator.tracking = 0;
                state->antenna_rotator.flip_mode_pass = 0;
                state->antenna_rotator.flip_decision_made = 0;
                state->antenna_rotator.flip_half = 0;
                // Released the pass; tear down the planner so the
                // memory comes back and so the next pass / mid-pass
                // 'T' rebuilds against fresh state.
                main_pursuit_clear_plan(state);
            }
        }
        (void) jul_idle_start;  // reserved for any future idle-window behavior
}
