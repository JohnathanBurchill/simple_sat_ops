/*

    Simple Satellite Operations  prediction.c

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

#include "prediction.h"
#include "state.h"

#include <string.h>
#include <time.h>
#include <hamlib/rig.h>
#include <hamlib/rotator.h>
#include <sgp4sdp4.h>
#include <ncurses.h>


void update_satellite_position(state_t *state, double jul_utc)
{
    // jul times are days
    state->jul_epoch = Julian_Date_of_Epoch(state->satellite.tle.epoch);
    state->minutes_since_epoch = (jul_utc - state->jul_epoch) * 1440.0;

    /* Propagate satellite position */
    /* Call NORAD routines according to deep-space flag */
    if(isFlagSet(DEEP_SPACE_EPHEM_FLAG)) {
        SDP4(state->minutes_since_epoch, &state->satellite.tle, &state->satellite.position, &state->satellite.velocity);
    } else {
        SGP4(state->minutes_since_epoch, &state->satellite.tle, &state->satellite.position, &state->satellite.velocity);
    }

    // pos and vel in km, km/s
    Convert_Sat_State(&state->satellite.position, &state->satellite.velocity);
    Magnitude(&state->satellite.velocity);
    Calculate_Obs(jul_utc, &state->satellite.position, &state->satellite.velocity, &state->observer.position_geodetic, &state->satellite.observation_set);
    Calculate_LatLonAlt(jul_utc, &state->satellite.position, &state->satellite.position_geodetic);
    state->satellite.azimuth = Degrees(state->satellite.observation_set.x);
    state->satellite.elevation = Degrees(state->satellite.observation_set.y);
    state->satellite.range_km = state->satellite.observation_set.z;
    state->satellite.range_rate_km_s = state->satellite.observation_set.w;
    state->satellite.latitude = Degrees(state->satellite.position_geodetic.lat);
    state->satellite.longitude = Degrees(state->satellite.position_geodetic.lon);
    state->satellite.altitude_km = state->satellite.position_geodetic.alt;
    state->satellite.speed_km_s = state->satellite.velocity.w;
    // Assumes ground station (not in a car, drone, balloon, plane, satellite, etc.)
    Calculate_User_PosVel(state->minutes_since_epoch, &state->observer.position_geodetic, &state->satellite.position, &state->observer.velocity);

    return;
}

void update_doppler_shifted_frequencies(state_t *state, double uplink_freq, double downlink_freq)
{
    Vec_Sub(&state->satellite.velocity, &state->observer.velocity, &state->observer_satellite_relative_velocity);
    state->observer_satellite_relative_speed = Dot(&state->observer_satellite_relative_velocity, &state->satellite.position) / state->satellite.position.w;  // Radial velocity
    state->doppler_uplink_frequency = uplink_freq * (1 + state->observer_satellite_relative_speed / 299792.458);  // Speed of light in km/s
    state->doppler_downlink_frequency = downlink_freq * (1 + state->observer_satellite_relative_speed / 299792.458);

    return;
}

// Overwrites the current satellite position
void update_pass_predictions(state_t *external_state, double jul_utc_start, double delta_t_minutes)
{
    state_t state = {0};
    memcpy(&state, external_state, sizeof *external_state);
    double jul_utc = jul_utc_start; 
    // Sets prediction to start of pass
    update_satellite_position(&state, jul_utc);
    double current_elevation = state.satellite.elevation;

    double max_elevation = current_elevation;
    double pass_duration = 0.0;
    double minutes_above_0_degrees = 0.0;
    double minutes_above_30_degrees = 0.0;
    int ascended = 0;
    while (current_elevation > -5.0) {
        update_satellite_position(&state, jul_utc + pass_duration / 1440.0);
        pass_duration += delta_t_minutes;
        if (current_elevation > 0.0) {
            minutes_above_0_degrees += delta_t_minutes;
            if (!ascended) {
                ascended = 1;
                external_state->predicted_ascension_jul_utc = jul_utc + pass_duration / 1440.0;
                external_state->predicted_ascension_azimuth = state.satellite.azimuth;
            }
        }
        if (current_elevation > 30.0) {
            minutes_above_30_degrees += delta_t_minutes;
        }
        current_elevation = state.satellite.elevation;
        if (max_elevation < current_elevation) {
            max_elevation = current_elevation;
        }
    }
    external_state->predicted_pass_duration_minutes = pass_duration;
    external_state->predicted_minutes_above_0_degrees = minutes_above_0_degrees;
    external_state->predicted_minutes_above_30_degrees = minutes_above_30_degrees;
    external_state->predicted_max_elevation = max_elevation;

    return;
}

void minutes_until_visible(state_t *external_state, double delta_t_minutes, double max_minutes)
{
    state_t state = {0};
    memcpy(&state, external_state, sizeof *external_state);
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc_start = Julian_Date(&utc, &tv);
    double jul_utc = jul_utc_start; 
    double max_jul_utc = jul_utc_start + max_minutes / 1440.0;
    update_satellite_position(&state, jul_utc);
    double elevation = state.satellite.elevation;
    if (elevation < 0) {
        // How long until it becomes visible?
        while (elevation < 0 && jul_utc < max_jul_utc) {
            jul_utc += delta_t_minutes / 1440.0;
            update_satellite_position(&state, jul_utc);
            elevation = state.satellite.elevation;
        }
    } else {
        // How long since it became visible?
        while (elevation > 0 && jul_utc < max_jul_utc) {
            jul_utc -= delta_t_minutes / 1440.0;
            update_satellite_position(&state, jul_utc);
            elevation = state.satellite.elevation;
        }
    }

    external_state->predicted_minutes_until_visible = (jul_utc - jul_utc_start) * 1440.0;

    return;
}


// Returns the first match on state->satellite.name
int load_tle(state_t *state)
{
    FILE *file = fopen(state->tles_filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error opening %s\n", state->tles_filename);
        return -1;
    }
    
    // 2 69-character lines plus a nul terminator
    char tle[139] = {0};
    char name[128] = {0}; 
    int found_satellite = 0;

    while (fgets(name, 128, file)) {
        // Remove newline
        name[strlen(name) - 1] = '\0';
        if (strncmp(state->satellite.name, name, strlen(state->satellite.name)) == 0) {
            // Errors caught in TLE check
            // Read 70 characters, including the newline
            fgets(tle, 71, file);
            // Read 69 characterers
            fgets(tle + 69, 70, file);
            tle[138] = '\0';
            snprintf(state->satellite.tle.sat_name, sizeof(state->satellite.tle.sat_name), "%s", name);
            found_satellite = 1;
            break;
        }
    }
    fclose(file);

    if (!found_satellite) {
        fprintf(stderr, "Satellite '%s' not found in %s\n", state->satellite.name, state->tles_filename);
        return -2;
    }

    if (!Good_Elements(tle)) {
        fprintf(stderr, "Invalid TLE\n");
        return -3;
    }
    Convert_Satellite_Data(tle, &state->satellite.tle);

    return 0;

}

