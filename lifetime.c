/*

    Simple Satellite Operations  lifetime.c

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
#include "sgp4sdp4/sgp4sdp4.h"

// Returns the first match on state->satellite.name
double lifetime(state_t *state, double jul_utc_start, double delta_t_minutes, double max_years, double min_alt_km)
{
    int status = load_tle(state);
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state->satellite.tle);
    double jul_utc = jul_utc_start;
    update_satellite_position(state, jul_utc);
    double years = 0.0;

    char filename[FILENAME_MAX] = {0};
    snprintf(filename, FILENAME_MAX, "/tmp/lifetime_%s.dat", state->satellite.name);
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        fprintf(stderr, "Unable to open %s for writing\n", filename);
        return -1;
    }

    while (years < max_years && state->satellite.altitude_km > min_alt_km) {
        update_satellite_position(state, jul_utc);
        fprintf(file, "%.6f %6.2f\n", years, state->satellite.altitude_km);
        jul_utc += delta_t_minutes / 1440.0;
        // Approx, sufficient for this problem
        years = (jul_utc - jul_utc_start) / 365.25;
    }

    fflush(file);
    fclose(file);

    return years;
}

void usage(FILE *dest, const char *name) 
{
    fprintf(dest, "usage: %s <tles_file> <satellite_name> <max_years>\n", name);
    return;
}

int main(int argc, char **argv)
{
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;

    int status = 0;
    state_t state = {0};

    for (int i = 0; i < argc; i++) {
        if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 1;
        }
    }

    if (argc - state.n_options != 4) {
        usage(stderr, argv[0]);
        return EXIT_FAILURE;
    }

    state.tles_filename = argv[1];
    state.satellite.name = argv[2];
    double max_years = atof(argv[3]);
    double min_alt_km = 100.0;

    /* Set up observer location */
    state.observer.position_geodetic.lat = site_latitude * M_PI / 180.0;
    state.observer.position_geodetic.lon = site_longitude * M_PI / 180.0;
    state.observer.position_geodetic.alt = site_altitude / 1000.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);

    double years = lifetime(&state, jul_utc, 1.0, max_years, min_alt_km);
    printf("Years above %.1f km: %.3f\n", min_alt_km, years);

    return 0;
}

