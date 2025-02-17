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

#include "radio.h"
#include "antenna_rotator.h"

#include <stdint.h>
#include <termios.h>
#include <sgp4sdp4.h>


#define MAX_TLE_LINE_LENGTH 128
#define TRACKING_PREP_TIME_MINUTES 5.0

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
    double predicted_minutes_until_visible;
    double predicted_max_elevation;
    double predicted_pass_duration_minutes;
    double predicted_minutes_above_0_degrees;
    double predicted_minutes_above_30_degrees;
    double predicted_ascension_azimuth;
    double predicted_ascension_jul_utc;
    int tracking;
    double tracking_prep_time_minutes;
    int in_pass;
    int run_with_radio;
    int run_with_antenna_rotator;
    int have_radio;
    int have_antenna_rotator;
    int auto_sat;
} state_t;


#endif // STATE_H
