/*

    Simple Satellite Operations  next_in_queue.c

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

#include "state.h"
#include "prediction.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sgp4sdp4.h>

typedef struct
{
    double min_altitude_km;
    double max_altitude_km;
    double max_minutes;
} criteria_t;

// Returns the first match on state->satellite.name
int next_in_queue(state_t *external_state, double jul_utc_start, double delta_t_minutes, criteria_t *criteria, char **next_name, double *minutes_away)
{
    FILE *file = fopen(external_state->tles_filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error opening %s\n", external_state->tles_filename);
        return -1;
    }
    
    // 2 69-character lines plus a nul terminator
    char tle[160] = {0};
    char name[128] = {0}; 
    int found_satellite = 0;
    char next_up_name[128] = {0};
    double next_up_minutes_away = 1e10;

    state_t state = {0};
    memcpy(&state, external_state, sizeof *external_state);
    
    // TODO load into memory for speed
    // Check every TLE
    int count = 0;
    int number_checked = 0;
    while (fgets(name, 128, file)) {
        // Remove newline
        name[strlen(name) - 1] = '\0';
        // Errors caught in TLE check
        // Read 70 characters, including the newline
        fgets(tle, 71, file);
        // Read remaining characterers
        fgets(tle + 69, 71, file);
        tle[138] = '\0';
        // Calculate minutes away
        if (!Good_Elements(tle)) {
            fprintf(stderr, "Invalid TLE\n");
            fclose(file);
            return -3;
        }
        Convert_Satellite_Data(tle, &state.satellite.tle);
        ClearFlag(ALL_FLAGS);
        select_ephemeris(&state.satellite.tle);
        update_satellite_position(&state, jul_utc_start);

        if (state.satellite.altitude_km >= criteria->min_altitude_km && state.satellite.altitude_km <= criteria->max_altitude_km) {
            minutes_until_visible(&state, delta_t_minutes, criteria->max_minutes);
            if (state.predicted_minutes_until_visible > 0 && next_up_minutes_away > state.predicted_minutes_until_visible) {
                next_up_minutes_away = state.predicted_minutes_until_visible;
                snprintf(next_up_name, sizeof(next_up_name), "%s", name);
            }
            number_checked++;
            double m = state.predicted_minutes_until_visible;
            if (m > 1440.0) {
                printf("%s: %.0f days\n", name, m / 1440.0);
            } else if (m > 60.0) {
                printf("%s: %.0f hours\n", name, m / 60.0);
            } else {
                printf("%s: %.0f minutes\n", name, m);
            }
        }

        count++;

    }
    fclose(file);

    if (next_name) {
        *next_name = strdup(next_up_name);
    }
    if (minutes_away) {
        *minutes_away = next_up_minutes_away;
    }

    return count;
}

int main(int argc, char **argv)
{
    if (argc != 4) {
        fprintf(stderr, "usage: %s <file.tle> <min_alt_km> <max_alt_km>\n", argv[0]);
        return EXIT_FAILURE;
    }

    int status = 0;
    state_t state = {0};

    state.tles_filename = argv[1];
    double min_altitude_km = atof(argv[2]);
    double max_altitude_km = atof(argv[3]);

    /* Set up observer location */
    state.observer.position_geodetic.lat = RAO_LATITUDE * M_PI / 180.0;
    state.observer.position_geodetic.lon = RAO_LONGITUDE* M_PI / 180.0;
    state.observer.position_geodetic.alt = RAO_ALTITUDE / 1000.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);

    char *next_in_queue_name = NULL;
    double next_in_queue_minutes_away = -1e10; 

    printf("Checking %s for upcoming satellite passes\n", state.tles_filename);
    criteria_t criteria = {
        .min_altitude_km = min_altitude_km,
        .max_altitude_km = max_altitude_km,
        .max_minutes = 1440.0,
    };

    int number_checked = next_in_queue(&state, jul_utc, 1.0, &criteria,  &next_in_queue_name, &next_in_queue_minutes_away);
    if (number_checked > 0) {
        int n = strlen(next_in_queue_name);
        while(n > 0 && isspace(next_in_queue_name[n - 1])) {
            n--;
        }
        next_in_queue_name[n] = '\0';
        // refine the estimate
        state.satellite.name = next_in_queue_name;
        status = load_tle(&state);
        ClearFlag(ALL_FLAGS);
        select_ephemeris(&state.satellite.tle);
        minutes_until_visible(&state, 1/120.0, next_in_queue_minutes_away * 2.0);
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state, jul_utc);
        printf("%s in %.2f minutes\n", state.satellite.name, state.predicted_minutes_until_visible);
        printf("%15s : %.2f° N\n", "latitude", state.satellite.latitude);
        printf("%15s : %.2f° E\n", "longitude", state.satellite.longitude);
        printf("%15s : %.3f km\n", "altitude", state.satellite.altitude_km);
        printf("%15s : %.3f km/s\n", "speed", state.satellite.speed_km_s);
        printf("%15s : %.2f°\n", "elevation", state.satellite.elevation);
        printf("%15s : %.2f°\n", "azimuth", state.satellite.azimuth);
        printf("%15s : %.2f km\n", "range", state.satellite.range_km);
        printf("%15s : %.2f km/s\n", "range rate", state.satellite.range_rate_km_s);
    }

    free(next_in_queue_name);

    return 0;
}

