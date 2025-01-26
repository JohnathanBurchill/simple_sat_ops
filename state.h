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

#define MAX_TLE_LINE_LENGTH 128

#include <sgp4sdp4.h>
#include <hamlib/rig.h>
#include <hamlib/rotator.h>

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

typedef struct state {
    int n_options;
    int running;
    char *tles_filename;
    double jul_epoch;
    double minutes_since_epoch;
    ephemeres_t observer;
    ephemeres_t satellite;
    vector_t observer_satellite_relative_velocity;
    double observer_satellite_relative_speed;
    double doppler_uplink_frequency;
    double doppler_downlink_frequency;
    double predicted_minutes_until_visible;
    double predicted_max_elevation;
    double predicted_pass_duration_minutes;
    double predicted_minutes_above_0_degrees;
    double predicted_minutes_above_30_degrees;
    int tracking;
    int in_pass;
    RIG *rig;
    ROT *rot;
    int run_without_rig;
    int run_without_rotator;
    int have_rig;
    int have_rotator;
} state_t;

#endif // STATE_H
