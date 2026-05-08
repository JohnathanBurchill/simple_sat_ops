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
    // and MAXIMUM_ELEVATION > 90. While set, predicted (az,el) sky targets
    // are rewritten to mechanical (az+180, 180-el) so the boom travels up
    // and over zenith instead of slewing AZ fast at the apex.
    int flip_mode_pass;
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
// Map a predicted sky direction (sat az/el, both in standard convention) to
// the mechanical (az,el) the rotator should drive. With flip == 0 it's a
// pass-through. With flip != 0 it returns (az+180 mod 360, 180-el), valid
// only when MAXIMUM_ELEVATION supports over-the-top travel.
void antenna_rotator_to_mech_coords(int flip, double sky_az, double sky_el,
                                    double *out_az, double *out_el);

#endif // ANTENNA_ROTATOR_H
