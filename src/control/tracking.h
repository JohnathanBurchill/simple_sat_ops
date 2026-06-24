/*

   Simple Satellite Operations  control/tracking.h

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

// Antenna pointing + satellite tracking: the rotator-set primitives, the
// whole-pass pursuit planner, the track/stop/home state transitions, the
// per-tick Doppler retune, and the mid-pass :retarget. Reaches the rotator
// through state->antenna_rotator + the async worker; the pure pursuit math
// is in orbit/pursuit.

#ifndef CONTROL_TRACKING_H
#define CONTROL_TRACKING_H

#include "rot_state.h"
#include "track_state.h"

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Pointing deadband: the main loop / point helper only commands a new
// rotator SET when the current error exceeds these (degrees).
#define MAX_DELTA_AZIMUTH_DEGREES   1.0
#define MAX_DELTA_ELEVATION_DEGREES 1.0

// :retarget <file> result codes. retarget_to_tle swaps the tracked
// satellite mid-pass; cmd_dispatch reports the outcome.
enum {
    RETARGET_OK        =  0,   // swapped; antenna re-aims on the next tick
    RETARGET_SAME      =  1,   // same file as the current target; no-op
    RETARGET_BAD_ARG   = -1,   // no path given
    RETARGET_READ_ERR  = -2,   // could not read a TLE from the file
    RETARGET_BAD_TLE   = -3,   // elements present but failed validation
};

// Rotator-set primitives (bounds-checked; post via the async worker).
int main_rotator_submit_set(rot_t *rot, double az_unwrapped, double elevation);
int main_rotator_increase_azimuth(rot_t *rot, double delta);
int main_rotator_increase_elevation(rot_t *rot, double delta);
// Refresh the rotator's current/target snapshot from the async worker.
int main_rotator_refresh_targets_from_snapshot(rot_t *rot);

// Whole-pass pursuit planner.
void main_pursuit_clear_plan(rot_t *rot);
void main_pursuit_build_plan(state_t *state, double jul_now);

// Tracking state transitions.
void start_tracking(state_t *state);
void stop_tracking(state_t *state);
int  point_to_stationary_target(state_t *state, double azimuth, double elevation);

// Recompute the Doppler-shifted up/downlink frequencies on the track state
// from the current range rate.
void update_doppler_shifted_frequencies(track_t *track, double uplink_freq, double downlink_freq);

// Per-tick antenna pointing: motion-settle detection, the two-step home's
// second leg, an active sky scan, and the satellite-tracking / pursuit aim
// loop (or rotator release at LOS). jul_utc is the current Julian date,
// t_now the monotonic-seconds clock.
void tracking_tick(state_t *state, double jul_utc, double t_now);

// Swap the tracked satellite to the first object in `path`. Returns a
// RETARGET_* code.
int retarget_to_tle(state_t *state, const char *path);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_TRACKING_H
