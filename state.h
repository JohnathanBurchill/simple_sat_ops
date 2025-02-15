/*

    Simple Satellite Operations  state.h

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

#ifndef STATE_H
#define STATE_H

#include "sgp4sdp4/sgp4sdp4.h"

#include <stdint.h>
#include <termios.h>

#define MAX_TLE_LINE_LENGTH 128

#define RADIO_MAX_MODEL_NAME_LEN 64
#define RADIO_MAX_DEVICE_FILENAME_LEN 1024

typedef struct ephemeres
{
    char *name;
    tle_t tle;
    vector_t position;
    vector_t velocity;
    double speed_km_s;
    geodetic_t position_geodetic;
    double azimuth;
    double elevation;
    double range_km;
    double range_rate_km_s;
    double latitude;
    double longitude;
    double altitude_km;
    vector_t observation_set;
} ephemeres_t;

typedef struct radio 
{
    char model_name[RADIO_MAX_MODEL_NAME_LEN];
    char *device_filename;
    uint32_t serial_speed;
    uint8_t connected;
    int fd;
    struct termios tty;
} radio_t;

typedef struct antenna_rotator 
{
    char model_name[RADIO_MAX_MODEL_NAME_LEN];
    char *device_filename;
    uint32_t serial_speed;
    uint8_t connected;
    int fd;
    struct termios tty;
} antenna_rotator_t;

typedef struct state 
{
    int n_options;
    int running;
    int verbose_level;
    char *tles_filename;
    double jul_epoch;
    double minutes_since_epoch;
    ephemeres_t observer;
    ephemeres_t satellite;
    radio_t radio;
    antenna_rotator_t antenna_rotator;
    double doppler_uplink_frequency;
    double doppler_downlink_frequency;
    double radio_vfo_main_frequency;
    double radio_vfo_sub_frequency;
    double predicted_minutes_until_visible;
    double predicted_max_elevation;
    double predicted_pass_duration_minutes;
    double predicted_minutes_above_0_degrees;
    double predicted_minutes_above_30_degrees;
    double predicted_ascension_azimuth;
    double predicted_ascension_jul_utc;
    int tracking;
    int in_pass;
    int run_with_radio;
    int run_with_rotator;
    int have_radio;
    int have_rotator;
} state_t;


void radio_connect(radio_t *radio);
void radio_disconnect(radio_t *radio);
int radio_set_satellite_mode(radio_t *radio, int sat_mode);

int antenna_rotator_get_position(antenna_rotator_t *antenna_rotator, double *azimuth_degrees, double *elevation_degrees);
int antenna_rotator_set_position(antenna_rotator_t *antenna_rotator, double azimuth_degrees, double elevation_degrees);

#endif // STATE_H
