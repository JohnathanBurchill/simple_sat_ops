/*

   Simple Satellite Operations  antenna_rotator.c

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

#include "antenna_rotator.h"
#include "qol.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <math.h>

int antenna_rotator_init(antenna_rotator_t *antenna_rotator)
{
    int antenna_rotator_result = ANTENNA_ROTATOR_OK;

    // Blocking serial connection
    antenna_rotator_connect(antenna_rotator);
    if (!antenna_rotator->connected) {
        fprintf(stderr, "Error opening antenna_rotator. Is it plugged into USB and powered?\n");
        return ANTENNA_ROTATOR_OPEN;
    }

    // TODO visual check and confirm good to go.

    return ANTENNA_ROTATOR_OK;
}

void antenna_rotator_connect(antenna_rotator_t *antenna_rotator)
{
    antenna_rotator->connected = 0;
    antenna_rotator->fd = open(antenna_rotator->device_filename, O_RDWR | O_NOCTTY | O_SYNC);
    if (antenna_rotator->fd == -1) {
        perror("Error opening antenna rotator serial port");
        return;
    }

    memset(&antenna_rotator->tty, 0, sizeof(antenna_rotator->tty));
    if (tcgetattr(antenna_rotator->fd, &antenna_rotator->tty) != 0) {
        perror("Error getting antenna rotator serial port attributes");
        close(antenna_rotator->fd);
        return;
    }

    cfsetospeed(&antenna_rotator->tty, antenna_rotator->serial_speed);
    cfsetispeed(&antenna_rotator->tty, antenna_rotator->serial_speed);

    antenna_rotator->tty.c_cflag = (antenna_rotator->tty.c_cflag & ~CSIZE) | CS8;
    antenna_rotator->tty.c_iflag &= ~IGNPAR;
    antenna_rotator->tty.c_lflag = 0;
    antenna_rotator->tty.c_oflag = 0;
    antenna_rotator->tty.c_cc[VMIN] = 0;
    // Wait up to 0.5 s
    antenna_rotator->tty.c_cc[VTIME] = 5;

    antenna_rotator->tty.c_cflag |= (CLOCAL | CREAD);
    antenna_rotator->tty.c_cflag &= ~(PARENB | PARODD);
    antenna_rotator->tty.c_cflag &= ~CSTOPB;
    antenna_rotator->tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(antenna_rotator->fd, TCSANOW, &antenna_rotator->tty) != 0) {
        perror("Error setting antenna rotator serial port attributes");
        close(antenna_rotator->fd);
        return;
    }
    // Flush buffers
    tcflush(antenna_rotator->fd, TCIOFLUSH);

    antenna_rotator->connected = 1;

    return;
}

void antenna_rotator_disconnect(antenna_rotator_t *antenna_rotator)
{
    if (antenna_rotator->connected) {
        close(antenna_rotator->fd);
        antenna_rotator->connected = 0;
    }

    return;
}

int antenna_rotator_command(antenna_rotator_t *antenna_rotator, antenna_rotator_command_t cmd, double *azimuth, double *elevation)
{
    // Refuse I/O on an unconnected rotator: fd may be zero-initialized (= stdin),
    // which would cause read() to block the UI thread waiting for keyboard input.
    if (!antenna_rotator->connected) {
        return ANTENNA_ROTATOR_BAD_RESPONSE;
    }
    uint8_t telemetry[AR_CMD_LEN] = {'W', '0', '0', '0', '0', 0x0, '0', '0', '0', '0', 0x0, (uint8_t)cmd, ' '};

    char az[5], el[5];

    if (cmd == ANTENNA_ROTATOR_SET) {
        if (azimuth == NULL || elevation == NULL) {
            return ANTENNA_ROTATOR_ARGS;
        }
        snprintf(az, 5, "%04.0f", (360.0 + *azimuth));
        snprintf(el, 5, "%04.0f", (360.0 + *elevation));
        for (int i = 0; i < 4; ++i) {
            telemetry[i + 1] = az[i];
            telemetry[i + 6] = el[i];
        }
    }
    printcmd("Antenna rotator command:", telemetry, AR_CMD_LEN);

    ssize_t bytes_sent = write(antenna_rotator->fd, telemetry, AR_CMD_LEN); 
    if (bytes_sent != AR_CMD_LEN) {
        return ANTENNA_ROTATOR_BAD_RESPONSE;
    }

    // Rot2ProG does not respond to a set command
    if (cmd == ANTENNA_ROTATOR_SET) {
        return ANTENNA_ROTATOR_OK;
    }

    uint8_t response[AR_RESPONSE_LEN] = {0};
    ssize_t bytes_received = -1;
    int remaining_buffer = AR_RESPONSE_LEN;
    int offset = 0;
    while (bytes_received != 0) {
        bytes_received = read(antenna_rotator->fd, response + offset, remaining_buffer);
        if (bytes_received == -1) {
            return ANTENNA_ROTATOR_BAD_RESPONSE;
        }
        offset += bytes_received;
        if (offset > AR_RESPONSE_LEN) {
            offset = AR_RESPONSE_LEN;
        }
        remaining_buffer -= bytes_received;
        if (remaining_buffer <= 0) {
            break;
        }
    }
    printcmd("Antenna rotator response", response, offset);

    if (offset != AR_RESPONSE_LEN) {
        return ANTENNA_ROTATOR_BAD_RESPONSE;
    }

    // All commands return the current position
    if (azimuth != NULL) {
        *azimuth = (double)(response[1] * 100 + response[2] * 10 + response[3]) + (double)response[4] / 10.0 - 360.0; 
    }
    if (elevation != NULL) {
        *elevation = (double)(response[6] * 100 + response[7] * 10 + response[8]) + (double)response[9] / 10.0 - 360.0; 
    }

    return ANTENNA_ROTATOR_OK;
}

int antenna_rotator_increase_azimuth(antenna_rotator_t *antenna_rotator, double angle)
{
    double base = antenna_rotator->unwrapped_target_valid
        ? antenna_rotator->target_azimuth_unwrapped
        : antenna_rotator->target_azimuth;
    double new_azimuth = base + angle;
    if (new_azimuth < ANTENNA_ROTATOR_MINIMUM_AZIMUTH || new_azimuth > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
        return ANTENNA_ROTATOR_AZIMUTH_LIMIT;
    }

    int rc = antenna_rotator_set_unwrapped(antenna_rotator, new_azimuth, antenna_rotator->target_elevation);
    if (rc != ANTENNA_ROTATOR_OK) {
        fprintf(stderr, "Error setting antenna rotator position\n");
        return ANTENNA_ROTATOR_BAD_RESPONSE;
    }
    return ANTENNA_ROTATOR_OK;
}

int antenna_rotator_point_to_target(antenna_rotator_t *antenna_rotator, double azimuth, double elevation)
{
    return antenna_rotator_set_unwrapped(antenna_rotator, azimuth, elevation);
}

double antenna_rotator_wrap_to_pm180(double d)
{
    while (d > 180.0)   d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

double antenna_rotator_accumulate_unwrapped(double prev, double pred_az)
{
    return prev + antenna_rotator_wrap_to_pm180(pred_az - prev);
}

// Pick the in-range co-terminal of `home_wrapped` (typically 0) with the
// smallest absolute value, tiebreaking toward `prev`. The aim is to leave the
// antenna physically at the home azimuth with as little accumulated rotation
// as the rotator's mechanical range allows -- i.e., always unwind, never wind
// another revolution. Used only by the home-return path.
double antenna_rotator_home_unwrapped_target(double prev, double home_wrapped)
{
    double best = home_wrapped;
    int have_best = 0;
    // Cover [-720, +900], comfortably larger than the [-179, +539] mechanical range.
    for (int k = -2; k <= 2; ++k) {
        double c = home_wrapped + 360.0 * (double)k;
        if (c < ANTENNA_ROTATOR_MINIMUM_AZIMUTH || c > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
            continue;
        }
        if (!have_best
            || fabs(c) < fabs(best)
            || (fabs(c) == fabs(best) && fabs(c - prev) < fabs(best - prev))) {
            best = c;
            have_best = 1;
        }
    }
    return best;
}

int antenna_rotator_seed_from_status(antenna_rotator_t *antenna_rotator)
{
    double az = 0.0, el = 0.0;
    int rc = antenna_rotator_command(antenna_rotator, ANTENNA_ROTATOR_STATUS, &az, &el);
    if (rc != ANTENNA_ROTATOR_OK) {
        return rc;
    }
    antenna_rotator->azimuth = az;
    antenna_rotator->elevation = el;
    antenna_rotator->target_azimuth = az;
    antenna_rotator->target_elevation = el;
    antenna_rotator->target_azimuth_unwrapped = az;
    antenna_rotator->unwrapped_target_valid = 1;
    return ANTENNA_ROTATOR_OK;
}

void antenna_rotator_to_mech_coords(int flip, double aos_az, double los_az,
                                    double progress,
                                    double sat_az, double sat_el,
                                    double *out_az, double *out_el,
                                    int *out_half)
{
    if (out_az == NULL || out_el == NULL) {
        return;
    }
    if (!flip || ANTENNA_ROTATOR_MAXIMUM_ELEVATION <= 90) {
        *out_az = sat_az;
        *out_el = sat_el;
        if (out_half) *out_half = 0;
        return;
    }
    if (progress < 0.0) progress = 0.0;
    if (progress > 1.0) progress = 1.0;
    // The LOS azimuth maps to mech_az = (los_az + 180) deg under the
    // rotator's back-pointing form (mech_el = 180 - sat_el looks down
    // the great circle in reverse). Lerp aos_az -> (los_az + 180)
    // along the shortest arc so for a near-zenith pass (los_az ~=
    // aos_az + 180) the lerp is essentially zero and mech_az is held
    // constant.
    double los_target = los_az + 180.0;
    double delta = los_target - aos_az;
    while (delta > 180.0)   delta -= 360.0;
    while (delta <= -180.0) delta += 360.0;
    double mech_az_meridian = aos_az + progress * delta;

    // Project sat onto the (rotated) boom meridian. y is along the
    // meridian (forward at mech_el=0, back at mech_el=180), z is up.
    double daz_rad = (sat_az - mech_az_meridian) * M_PI / 180.0;
    double el_rad  = sat_el * M_PI / 180.0;
    double y = cos(el_rad) * cos(daz_rad);
    double z = sin(el_rad);
    *out_az = mech_az_meridian;
    *out_el = atan2(z, y) * 180.0 / M_PI;
    if (out_half) *out_half = 0;
}

int antenna_rotator_set_unwrapped(antenna_rotator_t *antenna_rotator, double az_unwrapped, double elevation)
{
    if (az_unwrapped < ANTENNA_ROTATOR_MINIMUM_AZIMUTH || az_unwrapped > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
        return ANTENNA_ROTATOR_AZIMUTH_LIMIT;
    }
    if (elevation < ANTENNA_ROTATOR_MINIMUM_ELEVATION || elevation > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
        return ANTENNA_ROTATOR_ELEVATION_LIMIT;
    }
    double az = az_unwrapped;
    double el = elevation;
    int rc = antenna_rotator_command(antenna_rotator, ANTENNA_ROTATOR_SET, &az, &el);
    if (rc != ANTENNA_ROTATOR_OK) {
        return ANTENNA_ROTATOR_BAD_RESPONSE;
    }
    antenna_rotator->target_azimuth_unwrapped = az_unwrapped;
    antenna_rotator->target_azimuth = az_unwrapped;
    antenna_rotator->target_elevation = elevation;
    antenna_rotator->unwrapped_target_valid = 1;
    return ANTENNA_ROTATOR_OK;
}

