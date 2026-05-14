/*

   Simple Satellite Operations  antenna_rotator.h

   Copyright (C) 2025  Johnathan K Burchill

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
int antenna_rotator_command(antenna_rotator_t *antenna_rotator, antenna_rotator_command_t cmd, double *azimuth, double *elevation);
int antenna_rotator_increase_azimuth(antenna_rotator_t *antenna_rotator, double angle);
int antenna_rotator_point_to_target(antenna_rotator_t *antenna_rotator, double azimuth, double elevation);

double antenna_rotator_wrap_to_pm180(double delta_deg);
double antenna_rotator_accumulate_unwrapped(double prev_unwrapped, double prediction_az);
double antenna_rotator_home_unwrapped_target(double prev_unwrapped, double home_az_wrapped);
int antenna_rotator_seed_from_status(antenna_rotator_t *antenna_rotator);
int antenna_rotator_set_unwrapped(antenna_rotator_t *antenna_rotator, double az_unwrapped, double elevation);
// Map a predicted sky direction (sat az/el, both in standard convention)
// to the mechanical (az, el) the rotator should drive.
//
// With flip == 0 or ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90 the mapping
// is a pass-through.
//
// With flip != 0 and MAXIMUM_ELEVATION > 90, mech_az is held at aos_az
// for the entire pass and the sat direction is projected orthogonally
// onto the boom's meridian (the great circle through (aos_az, 0) up
// through zenith and back down to (aos_az+180, 0)). mech_el comes out
// in 0..180 deg; values past 90 deg are the rotator's back-pointing
// representation. This trades a small off-meridian pointing error --
// bounded by (90 deg - sat_el) at the sat's daz=90 deg crossing -- for
// zero azimuth slew at apex. There's no AZ axis we can drive to fix
// the off-meridian component without a roll axis, so we accept it.
//
// *out_half is retained for caller compatibility but is now always 0:
// the new mapping is continuous, with no half-transition.
void antenna_rotator_to_mech_coords(int flip, double aos_az,
                                    double sat_az, double sat_el,
                                    double *out_az, double *out_el,
                                    int *out_half);

#endif // ANTENNA_ROTATOR_H
