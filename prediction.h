/*

    Simple Satellite Operations  prediction.h

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

#ifndef PREDICTION_H
#define PREDICTION_H

#include "ephemeres.h"

typedef struct criteria
{
    double min_altitude_km;
    double max_altitude_km;
    double min_minutes;
    double max_minutes;
    double min_elevation;
    double max_elevation;
    char *regex;
    int regex_ignore_case;
    int with_constellations;
} criteria_t;

typedef struct pass
{
    double minutes_away;
    double max_elevation;
    double max_altitude;
    double ascension_jul_utc;
    double ascension_azimuth;
    double pass_duration;
    char name[26];
    char tle[160];
} pass_t;

typedef struct prediction
{
    ephemeres_t observer_ephem;
    ephemeres_t satellite_ephem;
    int auto_sat;
    char *tles_filename;
    double jul_utc_start;
    double jul_epoch;
    double minutes_since_epoch;
    double predicted_minutes_until_visible;
    double predicted_max_elevation;
    double predicted_max_altitude;
    double predicted_pass_duration_minutes;
    double predicted_minutes_above_0_degrees;
    double predicted_minutes_above_30_degrees;
    double predicted_ascension_azimuth;
    double predicted_ascension_jul_utc;
} prediction_t;

/* RAO site observer location in Priddis, SW of Calgary */
#define RAO_LATITUDE  50.8688  // Latitude in degrees
#define RAO_LONGITUDE -114.2910 // Longitude in degrees
#define RAO_ALTITUDE  1279.0   // Altitude in meters

void update_satellite_position(prediction_t *state, double jul_utc);
void update_pass_predictions(prediction_t *external_state, double jul_utc_start, double delta_t_minutes);
void minutes_until_visible(prediction_t *external_state, double jul_utc_start, double jul_utc_stop, double delta_t_minutes);
int load_tle(prediction_t *state);
int find_passes(prediction_t *external_state, double jul_utc_start, double delta_t_minutes, criteria_t *criteria, int *count, int *number_checked, int reverse_order, int find_all);
const pass_t *get_pass(int index);
const size_t number_of_passes(void);
void free_passes(void);
int get_next_pass(prediction_t *state, double jul_utc_start, double jul_utc_stop, double delta_t_minutes);


#endif // PREDICTION_H
      
