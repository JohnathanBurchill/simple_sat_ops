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

#include "prediction.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sgp4sdp4.h>

// Returns the first match on prediction->satellite_ephem.name
double lifetime(prediction_t *prediction, double jul_utc_start, double delta_t_minutes, double max_years, double min_alt_km)
{
    int status = load_tle(prediction);
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&prediction->satellite_ephem.tle);
    double jul_utc = jul_utc_start;
    update_satellite_position(prediction, jul_utc);
    double years = 0.0;

    char filename[FILENAME_MAX] = {0};
    snprintf(filename, FILENAME_MAX, "/tmp/lifetime_%s.dat", prediction->satellite_ephem.name);
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        fprintf(stderr, "Unable to open %s for writing\n", filename);
        return -1;
    }

    while (years < max_years && prediction->satellite_ephem.altitude_km > min_alt_km) {
        update_satellite_position(prediction, jul_utc);
        fprintf(file, "%.6f %6.2f\n", years, prediction->satellite_ephem.altitude_km);
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
    prediction_t prediction = {0};

    int n_options = 0;

    for (int i = 0; i < argc; i++) {
        if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 1;
        }
    }

    if (argc - n_options != 4) {
        usage(stderr, argv[0]);
        return EXIT_FAILURE;
    }

    prediction.tles_filename = argv[1];
    prediction.satellite_ephem.name = argv[2];
    double max_years = atof(argv[3]);
    double min_alt_km = 100.0;

    /* Set up observer location */
    prediction.observer_ephem.position_geodetic.lat = site_latitude * M_PI / 180.0;
    prediction.observer_ephem.position_geodetic.lon = site_longitude * M_PI / 180.0;
    prediction.observer_ephem.position_geodetic.alt = site_altitude / 1000.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);

    double years = lifetime(&prediction, jul_utc, 1.0, max_years, min_alt_km);
    printf("Years above %.1f km: %.3f\n", min_alt_km, years);

    return 0;
}

