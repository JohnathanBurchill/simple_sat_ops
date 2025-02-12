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
#include <regex.h>
#include <sgp4sdp4.h>

typedef struct
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

typedef struct 
{
    double minutes_away;
    double max_elevation;
    double ascension_jul_utc;
    double ascension_azimuth;
    double pass_duration;
    char name[26];
    char tle[160];
} pass_t;

static pass_t *passes = NULL;
static size_t n_passes = 0;

// Returns the first match on state->satellite.name
int find_overflights(state_t *external_state, double jul_utc_start, double delta_t_minutes, criteria_t *criteria, int *count, int *number_checked)
{
    FILE *file = fopen(external_state->tles_filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error opening %s\n", external_state->tles_filename);
        return -1;
    }
    
    // 2 69-character lines plus a nul terminator
    char tle[160] = {0};
    char name[26] = {0}; 
    int found_satellite = 0;
    char next_up_name[26] = {0};
    double next_up_minutes_away = 1e10;

    state_t state = {0};
    memcpy(&state, external_state, sizeof *external_state);
    
    // Check every TLE
    int internal_count = 0;
    int internal_number_checked = 0;
    int ignore_tle = 0;
    regex_t pattern = {0};
    if (criteria->regex != NULL) {
        printf("regex: '%s'\n", criteria->regex);
        int flags = REG_EXTENDED;
        if (criteria->regex_ignore_case) {
            flags |= REG_ICASE;
        }
        int res = regcomp(&pattern, criteria->regex, REG_EXTENDED);
        if (res) {
            fprintf(stderr, "Error compiling regex\n");
            return -6;
        }
    }

    char *constellations[] = {
        "COSMOS", 
        "CENTISPACE", 
        "FLOCK 4", 
        "GAOFEN-", 
        "GEESAT-", 
        "GLOBALSTAR",
        "GONETS-",
        "HAWK-", 
        "ICEYE-",
        "IRIDIUM",
        "JILIN-",
        "LEMUR-",
        "NUSAT-",
        "ONEWEB-",
        "QIANFAN-",
        "SITRO-AIS",
        "STARLINK", 
        "YAOGAN",
    };
    int n_constellations = sizeof constellations / sizeof(char *);
    int skip_this = 0;


    while (fgets(name, 26, file)) {
        skip_this = 0;
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

        internal_count++;

        // Remove trailing whitespace
        int n = strlen(name);
        while(n > 0 && isspace(name[n - 1])) {
            n--;
        }
        name[n] = '\0';
        // Filter with regex
        if (!criteria->with_constellations) { 
            for (int i = 0; i < n_constellations; i++) {
                if (strncmp(constellations[i], name, strlen(constellations[i])) == 0) {
                    skip_this = 1;
                    break;
                }
            }
            if (skip_this) {
                continue;
            }
        }
        if (criteria->regex != NULL && (regexec(&pattern, name, 0, NULL, 0) == REG_NOMATCH)) {
            continue;
        }

        Convert_Satellite_Data(tle, &state.satellite.tle);
        ClearFlag(ALL_FLAGS);
        select_ephemeris(&state.satellite.tle);
        update_satellite_position(&state, jul_utc_start);

        // TODO filter on perigee / apogee instead of current altitude?
        if (state.satellite.altitude_km >= criteria->min_altitude_km && state.satellite.altitude_km <= criteria->max_altitude_km) {
            minutes_until_visible(&state, delta_t_minutes, criteria->max_minutes);
            if (state.predicted_minutes_until_visible > 0 && next_up_minutes_away > state.predicted_minutes_until_visible) {
                next_up_minutes_away = state.predicted_minutes_until_visible;
                snprintf(next_up_name, sizeof(next_up_name), "%s", name);
            }
            internal_number_checked++;
            if (state.predicted_minutes_until_visible >= 0 && state.predicted_minutes_until_visible < criteria->max_minutes) {
                update_pass_predictions(&state, jul_utc_start + state.predicted_minutes_until_visible / 1440.0, 0.25);
                if (state.predicted_minutes_above_0_degrees <= 0.0 || state.predicted_max_elevation > criteria->max_elevation || state.predicted_max_elevation < criteria->min_elevation) {
                    continue;
                }
                // Store this pass
                void *mem = realloc(passes, sizeof *passes * (n_passes + 1));
                if (mem == NULL) {
                    printf("Unable to allocate memory for the pass info.\n");
                    regfree(&pattern);
                    return -4;
                }
                passes = mem;
                n_passes++;
                memset(&passes[n_passes - 1], 0, sizeof *passes);
                pass_t *p = &passes[n_passes - 1];
                (void)strncpy(p->name, name, sizeof(p->name));

                (void)strncpy(p->tle, name, sizeof(p->tle));

                // Refine estimate:
                if (state.predicted_minutes_until_visible < 10.0 && delta_t_minutes > 1.0/60.) {
                    minutes_until_visible(&state, 1.0 / 60.0, criteria->max_minutes);
                }
                p->minutes_away = state.predicted_minutes_until_visible;
                p->pass_duration = state.predicted_pass_duration_minutes;
                p->ascension_jul_utc = state.predicted_ascension_jul_utc;
                p->ascension_azimuth = state.predicted_ascension_azimuth;
                p->max_elevation = state.predicted_max_elevation;
            }
        }
    }
    fclose(file);
    regfree(&pattern);

    if (count) {
        *count = internal_count;
    }
    if (number_checked) {
        *number_checked = internal_number_checked;
    }

    return 0;
}

void usage(FILE *dest, const char *name) 
{
    fprintf(dest, "usage: %s <tles_file> <min_alt_km> <max_alt_km>\n", name);
    return;
}

// Sort to give soonest pass first
int pass_sort_soonest_first(const void *a, const void *b)
{
    pass_t *p1 = (pass_t *)a;
    pass_t *p2 = (pass_t *)b;

    if (p1->minutes_away < p2->minutes_away) {
        return -1;
    } else if (p1->minutes_away > p2->minutes_away) {
        return 1;
    } else {
        return 0;
    }
}

int pass_sort_latest_first(const void *a, const void *b)
{
    return -pass_sort_soonest_first(a, b);
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
    status = find_overflights(&state, jul_utc, 1.0, &criteria, &count, &number_checked);

    if (n_passes > 0) {
        // Sort the list
        if (reverse) {
            qsort(passes, n_passes, sizeof *passes, pass_sort_latest_first);
        } else {
            qsort(passes, n_passes, sizeof *passes, pass_sort_soonest_first);
        }
        pass_t *p = NULL;
        if (max_passes == -1) {
            max_passes = n_passes;
        }
        if (list_all) {
            if (!reverse) {
                printf("Found %lu upcoming passes from a total of %d satellites\n", n_passes, count);
                printf("%26s  %8s %8s %9s %9s\n", "Name", "in (min)", "dur (min)", "azi (deg)", "ele (deg)");
            }
            for (int i = 0; i < max_passes; i++) {
                p = &passes[i];
                printf("%26s  %8.1f %9.1f %9.1f %9.1f\n", p->name, p->minutes_away, p->pass_duration, p->ascension_azimuth, p->max_elevation);
            }
            if (reverse) {
                printf("%26s  %8s %8s %9s %9s\n", "Name", "in (min)", "dur (min)", "azi (deg)", "ele (deg)");
                printf("Found %lu upcoming passes from a total of %d satellites\n", n_passes, count);
            }
        } else {
            printf("Searched %d satellites\n", count);
            p = &passes[0];
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

    free(passes);
    passes = NULL;
    n_passes = 0;

    return 0;
}

