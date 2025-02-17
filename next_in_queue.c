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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <sgp4sdp4.h>

void usage(FILE *dest, const char *name) 
{
    fprintf(dest, "usage: %s <tles_file> <min_alt_km> <max_alt_km>\n", name);
    return;
}

int main(int argc, char **argv)
{
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;

    int status = 0;
    int list_all = 0;
    int max_passes = -1;
    double min_minutes_away = 0.0;
    double max_minutes_away = 1440.0;
    double min_elevation = 0.0;
    double max_elevation = 90.0;
    state_t state = {0};
    char *regex = NULL;
    int regex_ignore_case = 0;
    int with_constellations = 1;
    int reverse = 0;

    for (int i = 0; i < argc; i++) {
        if (strncmp("--lat=", argv[i], 6) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_latitude = atof(argv[i] + 6);
        } else if (strncmp("--lon=", argv[i], 6) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_longitude = atof(argv[i] + 6);
        } else if (strncmp("--max-passes=", argv[i], 13) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 14) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_passes = atoi(argv[i] + 13);
        } else if (strncmp("--alt=", argv[i], 6) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_altitude = atof(argv[i] + 6);
        } else if (strcmp("--list", argv[i]) == 0) {
            state.n_options++;
            list_all = 1;
        } else if (strcmp("--reverse", argv[i]) == 0) {
            state.n_options++;
            reverse = 1;
        } else if (strcmp("--no-constellations", argv[i]) == 0) {
            state.n_options++;
            with_constellations = 0;
        } else if (strncmp("--min-elevation=", argv[i], 16) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_elevation = atof(argv[i] + 16);
        } else if (strncmp("--max-elevation=", argv[i], 16) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_elevation = atof(argv[i] + 16);
        } else if (strncmp("--min-minutes=", argv[i], 14) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_minutes_away = atof(argv[i] + 14);
        } else if (strncmp("--max-minutes=", argv[i], 14) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_minutes_away = atof(argv[i] + 14);
        } else if (strcmp("--ignore-case", argv[i]) == 0) {
            state.n_options++;
            regex_ignore_case = 1;
        } else if (strncmp("--regex=", argv[i], 8) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 9) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            regex = argv[i] + 8;
        } else if (strcmp("--help", argv[i]) == 0) {
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
    double min_altitude_km = atof(argv[2]);
    double max_altitude_km = atof(argv[3]);

    /* Set up observer location */
    state.observer.position_geodetic.lat = site_latitude * M_PI / 180.0;
    state.observer.position_geodetic.lon = site_longitude * M_PI / 180.0;
    state.observer.position_geodetic.alt = site_altitude / 1000.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);

    printf("Checking %s for upcoming satellite passes within the next %.1f minutes\n", state.tles_filename, max_minutes_away);
    criteria_t criteria = {
        .min_altitude_km = min_altitude_km,
        .max_altitude_km = max_altitude_km,
        .min_minutes = min_minutes_away,
        .max_minutes = max_minutes_away,
        .min_elevation = min_elevation,
        .max_elevation = max_elevation,
        .regex = regex,
        .regex_ignore_case = regex_ignore_case,
        .with_constellations = with_constellations,
    };

    int count = 0;
    int number_checked = 0;
    status = find_passes(&state, jul_utc, 1.0, &criteria, &count, &number_checked, reverse);
    const size_t n_passes = number_of_passes();

    if (n_passes > 0) {
        const pass_t *p = NULL;
        if (max_passes == -1) {
            max_passes = n_passes;
        }
        if (list_all) {
            if (!reverse) {
                printf("Found %lu upcoming passes from a total of %d satellites\n", n_passes, count);
                printf("%26s  %8s %8s %9s %9s\n", "Name", "in (min)", "dur (min)", "azi (deg)", "ele (deg)");
            }
            for (int i = 0; i < max_passes; i++) {
                p = get_pass(i);
                printf("%26s  %8.1f %9.1f %9.1f %9.1f\n", p->name, p->minutes_away, p->pass_duration, p->ascension_azimuth, p->max_elevation);
            }
            if (reverse) {
                printf("%26s  %8s %8s %9s %9s\n", "Name", "in (min)", "dur (min)", "azi (deg)", "ele (deg)");
                printf("Found %lu upcoming passes from a total of %d satellites\n", n_passes, count);
            }
        } else {
            printf("Searched %d satellites\n", count);
            p = get_pass(0);
            printf("%26s in %.1f minutes at azimuth %.1f deg. for %.1f minutes reaching elevation %.1f deg.\n", p->name, p->minutes_away, p->ascension_azimuth, p->pass_duration, p->max_elevation);
            // printf("%s in %.2f minutes\n", p->name, p->minutes_away);
            // printf("%15s : %.2f° N\n", "latitude", state.satellite.latitude);
            // printf("%15s : %.2f° E\n", "longitude", state.satellite.longitude);
            // printf("%15s : %.3f km\n", "altitude", state.satellite.altitude_km);
            // printf("%15s : %.3f km/s\n", "speed", state.satellite.speed_km_s);
            // printf("%15s : %.2f°\n", "elevation", state.satellite.elevation);
            // printf("%15s : %.2f°\n", "azimuth", state.satellite.azimuth);
            // printf("%15s : %.2f km\n", "range", state.satellite.range_km);
            // printf("%15s : %.2f km/s\n", "range rate", state.satellite.range_rate_km_s);
        }
    }

    free_passes();

    return 0;
}

