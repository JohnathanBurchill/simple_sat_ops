/*

    Simple Satellite Operations  prediction.h

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
    // If non-NULL, only satellites whose names start with this string
    // are considered, bypassing the constellation filter.
    const char *name_prefix;
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
} pass_t;

// Forward decl: when non-NULL on prediction_t, state comes from a
// pre-propagated ephemeris (ITRF Cartesian), not from SGP4/TLE.
struct oem_table;

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
    double predicted_descent_azimuth;
    double predicted_descent_jul_utc;
    // Alternative state source. When non-NULL, update_satellite_position
    // interpolates from this table instead of running SGP4.
    struct oem_table *oem;
} prediction_t;

/* RAO site observer location in Priddis, SW of Calgary */
#define RAO_LATITUDE  50.8688  // Latitude in degrees
#define RAO_LONGITUDE -114.2910 // Longitude in degrees
#define RAO_ALTITUDE  1279.0   // Altitude in meters

// Convert Unix epoch seconds (UTC) to a Julian Date for the SGP4/SDP4
// propagator. JD 2440587.5 is the Unix epoch (1970-01-01T00:00:00Z); there
// are 86400 seconds per day, and sub-second precision is preserved.
//
// Use this to turn a wall-clock time into the jul_utc argument of
// update_satellite_position(). Do NOT build a struct tm with gmtime_r() and
// pass it to sgp4sdp4's Julian_Date(): that function expects a non-POSIX
// struct tm (full year in tm_year, 1-based tm_mon), so a POSIX struct tm
// yields a JD ~1900 years off, and SGP4 then returns a garbage ~1e19 km
// range (issue #53).
double julian_date_from_unix_seconds(double unix_seconds);

void update_satellite_position(prediction_t *state, double jul_utc);
void update_pass_predictions(prediction_t *external_state, double jul_utc_start, double delta_t_minutes);
void minutes_until_visible(prediction_t *external_state, double jul_utc_start, double jul_utc_stop, double delta_t_minutes);
// Loads the named satellite's TLE into state->satellite_ephem.tle. The
// elements are left in raw (pre-conversion) units: the caller MUST call
// sgp4sdp4's select_ephemeris() on the tle exactly once before propagating,
// and never twice — select_ephemeris() rewrites the element units in place,
// so a second call corrupts them (a real hazard in multi-satellite loops).
int load_tle(prediction_t *state);
// Fills out_path with "$HOME/.local/state/simple_sat_ops/active.tle".
// Returns 0 on success, -1 if $HOME is unset or the buffer is too small.
int tle_default_path(char *out_path, size_t out_cap);
int find_passes(prediction_t *external_state, double jul_utc_start, double delta_t_minutes, criteria_t *criteria, int *count, int *number_checked, int reverse_order, int find_all);
const pass_t *get_pass(int index);
size_t number_of_passes(void);
void free_passes(void);
int get_next_pass(prediction_t *state, double jul_utc_start, double jul_utc_stop, double delta_t_minutes);


#endif // PREDICTION_H
      
