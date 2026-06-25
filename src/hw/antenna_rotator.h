/*

   Simple Satellite Operations  antenna_rotator.h

   Copyright (C) 2025, 2026  Johnathan K Burchill

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

#ifndef ANTENNA_ROTATOR_H
#define ANTENNA_ROTATOR_H

#include <termios.h>
#include <stdint.h>

#define AR_CMD_LEN 13
#define AR_RESPONSE_LEN 12

#define ANTENNA_ROTATOR_MINIMUM_AZIMUTH -179
#define ANTENNA_ROTATOR_MAXIMUM_AZIMUTH 539
#define ANTENNA_ROTATOR_MINIMUM_ELEVATION -5
// Set to 180 if the SPID controller is configured for over-the-top travel
// (MD-01/MD-02 H+V mode, EL endstops moved). 89 keeps the rotator in the
// classic 0..90 hemisphere; 180 unlocks flip-mode tracking for high passes.
#define ANTENNA_ROTATOR_MAXIMUM_ELEVATION 180

// Above this predicted max elevation the AZ slew at apex of a normal-mode
// pass starts to outrun the SPID, and flip mode (if MAX_ELEVATION > 90)
// trades a one-shot 180 deg AZ pre-position for a smooth EL sweep over the
// top. Tuned conservatively on the user-reported >80 deg pass.
#define ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD 75.0

enum ANTENNA_ROTATOR_STATUS {
    ANTENNA_ROTATOR_OK = 0,
    ANTENNA_ROTATOR_BAD_RESPONSE,
    ANTENNA_ROTATOR_ERROR,
    ANTENNA_ROTATOR_OPEN,
    ANTENNA_ROTATOR_ARGS,
    ANTENNA_ROTATOR_AZIMUTH_LIMIT,
    ANTENNA_ROTATOR_ELEVATION_LIMIT,
};

typedef enum {
    ANTENNA_ROTATOR_STOP = 0x0F,
    ANTENNA_ROTATOR_STATUS = 0x1F,
    ANTENNA_ROTATOR_SET = 0x2F,
} antenna_rotator_command_t;

typedef struct antenna_rotator 
{
    char *device_filename;
    speed_t serial_speed;
    uint8_t connected;
    int fd;
    struct termios tty;
    int is_required;
    double target_azimuth;
    double target_elevation;
    double azimuth;
    double elevation;
    // Last commanded extended-range azimuth; canonical for path planning.
    // target_azimuth is kept as the display-friendly (often wrapped) form.
    double target_azimuth_unwrapped;
    int unwrapped_target_valid;
    // Pending second leg when a home-return needs an intermediate waypoint.
    double home_pending_final_az;
    int homing_in_progress;
    int fixed_target;
    int tracking;
    double tracking_prep_time_minutes;
    int antenna_is_under_control;
    int antenna_should_be_controlled;
    int antenna_is_moving;
    // Set when entering a high-elevation pass (>= FLIP_ELEVATION_THRESHOLD)
    // and MAXIMUM_ELEVATION > 90. While set, predicted (az, el) sky targets
    // are routed through antenna_rotator_to_mech_coords with flip_aos_az,
    // so the boom can sweep mech_el 0->180 over the top.
    int flip_mode_pass;
    // Sky azimuth at the moment flip mode was decided. Used to classify
    // each subsequent prediction as "first half" (boom on same side as
    // satellite) or "second half" (sat has crossed to the back hemisphere
    // and the boom tracks via 180-el).
    double flip_aos_az;
    // Predicted LOS azimuth + AOS/LOS Julian dates, captured at the
    // same moment as flip_aos_az. Used by the flip mapping to lerp
    // mech_az from aos_az at progress=0 to (los_az + 180) at progress=1
    // so the boom converges on the satellite at LOS as well as AOS.
    double flip_los_az;
    double flip_aos_jul;
    double flip_los_jul;
    // One-shot latch: prevents the per-tick re-enable from re-running the
    // flip decision after a mid-pass bailout. Cleared at LOS / s / r.
    int flip_decision_made;
    // Cached half: 0 = first (sat near aos_az), 1 = second (sat across).
    // Set on the first valid tick under flip mode; transition triggers a
    // reseed of target_azimuth_unwrapped so the unwrap accumulator doesn't
    // try to bridge the 180 deg jump.
    int flip_half;
} antenna_rotator_t;

int antenna_rotator_init(antenna_rotator_t *antenna_rotator);
void antenna_rotator_connect(antenna_rotator_t *antenna_rotator);
void antenna_rotator_disconnect(antenna_rotator_t *antenna_rotator);
int antenna_rotator_command(antenna_rotator_t *antenna_rotator, antenna_rotator_command_t cmd, double *azimuth, double *elevation);
int antenna_rotator_increase_azimuth(antenna_rotator_t *antenna_rotator, double angle);
int antenna_rotator_point_to_target(antenna_rotator_t *antenna_rotator, double azimuth, double elevation);

double antenna_rotator_wrap_to_pm180(double delta_deg);
double antenna_rotator_accumulate_unwrapped(double prev_unwrapped, double prediction_az);
// Great-circle angle (deg) between two (az, el) directions. Used to show the
// antenna pointing error -- how far the rotator's actual position is from its
// commanded target. Both pairs are in the same (mechanical) frame, so this
// also reads correctly on flip passes. Azimuth wrap is handled (cos is
// periodic). Result is in [0, 180].
double antenna_rotator_pointing_error_deg(double az_a_deg, double el_a_deg,
                                          double az_b_deg, double el_b_deg);
double antenna_rotator_home_unwrapped_target(double prev_unwrapped, double home_az_wrapped);
// Decide whether the final leg of a two-step home should fire, given the
// latest STATUS azimuth. Two things must hold: the reading must differ from
// the commanded mid waypoint by more than echo_tol_deg (so it is real
// feedback, not the post-SET target echo the Rot2Prog reports for a couple
// of seconds), and it must be in the unwind zone (the short path from here to
// final_az runs the same way as the unwind). Returns 1 to fire, 0 to wait.
int antenna_rotator_home_leg2_ready(double current_az, double mid_az,
                                    double final_az, double echo_tol_deg);
int antenna_rotator_seed_from_status(antenna_rotator_t *antenna_rotator);
int antenna_rotator_set_unwrapped(antenna_rotator_t *antenna_rotator, double az_unwrapped, double elevation);
// Map a predicted sky direction (sat az/el, both in standard convention)
// to the mechanical (az, el) the rotator should drive.
//
// With flip == 0 or ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90 the mapping
// is a pass-through; aos_az/los_az/progress are ignored.
//
// With flip != 0 and MAXIMUM_ELEVATION > 90, the boom's meridian is
// linearly interpolated:
//
//   mech_az_meridian(progress) = aos_az + progress * delta
//   delta = shortest-arc (los_az + 180 - aos_az), in (-180, 180]
//
// At progress=0 the boom points along the AOS great circle (boom on
// target at AOS); at progress=1 it uses (los_az + 180) so mech_el=180
// looks down the AOS-to-LOS great circle in reverse (boom on target
// at LOS). progress is clamped to [0, 1].
//
// The sat direction is then projected orthogonally onto the current
// meridian: mech_el = atan2(sin(sat_el), cos(sat_el) * cos(sat_az -
// mech_az_meridian)), with values past 90 deg interpreted by the
// rotator as back-pointing. Off-meridian pointing error -- the
// component perpendicular to the boom's great circle -- is the
// trade-off and is not correctable without a roll axis.
//
// For zenith passes (los_az = aos_az + 180), delta = 0 and mech_az
// is held constant; the mapping collapses to the simpler aos_az-hold.
//
// *out_half is retained for caller compatibility but is now always 0.
void antenna_rotator_to_mech_coords(int flip, double aos_az, double los_az,
                                    double progress,
                                    double sat_az, double sat_el,
                                    double *out_az, double *out_el,
                                    int *out_half);

#endif // ANTENNA_ROTATOR_H
