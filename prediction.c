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

#include <string.h>
#include <time.h>
#include <sgp4sdp4.h>
#include <ncurses.h>
#include <regex.h>

static pass_t *passes = NULL;
static size_t n_passes = 0;

void update_satellite_position(prediction_t *prediction, double jul_utc)
{
    // jul times are days
    prediction->jul_epoch = Julian_Date_of_Epoch(prediction->satellite_ephem.tle.epoch);
    prediction->minutes_since_epoch = (jul_utc - prediction->jul_epoch) * 1440.0;

    /* Propagate satellite position */
    /* Call NORAD routines according to deep-space flag */
    if(isFlagSet(DEEP_SPACE_EPHEM_FLAG)) {
        SDP4(prediction->minutes_since_epoch, &prediction->satellite_ephem.tle, &prediction->satellite_ephem.position, &prediction->satellite_ephem.velocity);
    } else {
        SGP4(prediction->minutes_since_epoch, &prediction->satellite_ephem.tle, &prediction->satellite_ephem.position, &prediction->satellite_ephem.velocity);
    }

    // pos and vel in km, km/s
    Convert_Sat_State(&prediction->satellite_ephem.position, &prediction->satellite_ephem.velocity);
    Magnitude(&prediction->satellite_ephem.velocity);
    Calculate_Obs(jul_utc, &prediction->satellite_ephem.position, &prediction->satellite_ephem.velocity, &prediction->observer_ephem.position_geodetic, &prediction->satellite_ephem.observation_set);
    Calculate_LatLonAlt(jul_utc, &prediction->satellite_ephem.position, &prediction->satellite_ephem.position_geodetic);
    prediction->satellite_ephem.azimuth = Degrees(prediction->satellite_ephem.observation_set.x);
    prediction->satellite_ephem.elevation = Degrees(prediction->satellite_ephem.observation_set.y);
    prediction->satellite_ephem.range_km = prediction->satellite_ephem.observation_set.z;
    prediction->satellite_ephem.range_rate_km_s = prediction->satellite_ephem.observation_set.w;
    prediction->satellite_ephem.latitude = Degrees(prediction->satellite_ephem.position_geodetic.lat);
    prediction->satellite_ephem.longitude = Degrees(prediction->satellite_ephem.position_geodetic.lon);
    prediction->satellite_ephem.altitude_km = prediction->satellite_ephem.position_geodetic.alt;
    prediction->satellite_ephem.speed_km_s = prediction->satellite_ephem.velocity.w;
    // Assumes ground station (not in a car, drone, balloon, plane, satellite, etc.)
    Calculate_User_PosVel(prediction->minutes_since_epoch, &prediction->observer_ephem.position_geodetic, &prediction->satellite_ephem.position, &prediction->observer_ephem.velocity);

    return;
}

// Overwrites the current satellite position
void update_pass_predictions(prediction_t *external_prediction, double jul_utc_start, double delta_t_minutes)
{
    prediction_t prediction = {0};
    memcpy(&prediction, external_prediction, sizeof *external_prediction);
    double jul_utc = jul_utc_start; 
    // Sets prediction to start of pass
    update_satellite_position(&prediction, jul_utc);
    double current_elevation = prediction.satellite_ephem.elevation;
    double current_altitude = prediction.satellite_ephem.altitude_km;

    double max_elevation = current_elevation;
    double max_altitude = current_altitude;
    double pass_duration = 0.0;
    double minutes_above_0_degrees = 0.0;
    double minutes_above_30_degrees = 0.0;
    int ascended = 0;
    while (current_elevation > -5.0) {
        update_satellite_position(&prediction, jul_utc + pass_duration / 1440.0);
        pass_duration += delta_t_minutes;
        current_altitude = prediction.satellite_ephem.altitude_km;
        if (current_elevation > 0.0) {
            minutes_above_0_degrees += delta_t_minutes;
            if (!ascended) {
                ascended = 1;
                external_prediction->predicted_ascension_jul_utc = jul_utc + pass_duration / 1440.0;
                external_prediction->predicted_ascension_azimuth = prediction.satellite_ephem.azimuth;
            }
            if (max_altitude < current_altitude) {
                max_altitude = current_altitude;
            }
            if (current_elevation > 30.0) {
                minutes_above_30_degrees += delta_t_minutes;
            }
        }
        current_elevation = prediction.satellite_ephem.elevation;
        if (max_elevation < current_elevation) {
            max_elevation = current_elevation;
        }
    }
    external_prediction->predicted_pass_duration_minutes = pass_duration;
    external_prediction->predicted_minutes_above_0_degrees = minutes_above_0_degrees;
    external_prediction->predicted_minutes_above_30_degrees = minutes_above_30_degrees;
    external_prediction->predicted_max_elevation = max_elevation;
    external_prediction->predicted_max_altitude = max_altitude;

    return;
}

void minutes_until_visible(prediction_t *external_prediction, double jul_utc_start, double jul_utc_stop, double delta_t_minutes)
{
    prediction_t prediction = {0};
    memcpy(&prediction, external_prediction, sizeof *external_prediction);
    if (jul_utc_start == 0.0) {
        struct tm utc;
        struct timeval tv;
        UTC_Calendar_Now(&utc, &tv);
        jul_utc_start = Julian_Date(&utc, &tv);
    }
    double jul_utc = jul_utc_start; 
    update_satellite_position(&prediction, jul_utc);
    double elevation = prediction.satellite_ephem.elevation;
    if (elevation < 0) {
        // How long until it becomes visible?
        while (elevation < 0 && jul_utc < jul_utc_stop) {
            jul_utc += delta_t_minutes / 1440.0;
            update_satellite_position(&prediction, jul_utc);
            elevation = prediction.satellite_ephem.elevation;
        }
    } else {
        // How long since it became visible?
        while (elevation > 0 && jul_utc < jul_utc_stop) {
            jul_utc -= delta_t_minutes / 1440.0;
            update_satellite_position(&prediction, jul_utc);
            elevation = prediction.satellite_ephem.elevation;
        }
    }
    if (jul_utc > 0 && jul_utc < jul_utc_stop) {
        external_prediction->predicted_minutes_until_visible = (jul_utc - jul_utc_start) * 1440.0;
    } else {
        external_prediction->predicted_minutes_until_visible = -9999.0;
    }

    return;
}


// Returns the first match on prediction->satellite_ephem.name
int load_tle(prediction_t *prediction)
{
    FILE *file = fopen(prediction->tles_filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error opening %s\n", prediction->tles_filename);
        return -1;
    }
    
    // 2 69-character lines plus a nul terminator
    char tle[139] = {0};
    char name[128] = {0}; 
    int found_satellite = 0;
    char *ptr = NULL;

    while (fgets(name, 128, file)) {
        // Remove newline
        name[strlen(name) - 1] = '\0';
        if (strncmp(prediction->satellite_ephem.name, name, strlen(prediction->satellite_ephem.name)) == 0) {
            // Errors caught in TLE check
            // Read 70 characters, including the newline
            ptr = fgets(tle, 71, file);
            if (ptr == NULL) {
                break;
            }
            // Read 69 characterers
            ptr = fgets(tle + 69, 70, file);
            if (ptr == NULL) {
                break;
            }
            tle[138] = '\0';
            snprintf(prediction->satellite_ephem.tle.sat_name, sizeof(prediction->satellite_ephem.tle.sat_name), "%s", name);
            found_satellite = 1;
            break;
        }
    }
    fclose(file);

    if (!found_satellite) {
        fprintf(stderr, "Satellite '%s' not found in %s\n", prediction->satellite_ephem.name, prediction->tles_filename);
        return -2;
    }

    if (!Good_Elements(tle)) {
        fprintf(stderr, "Invalid TLE\n");
        return -3;
    }
    Convert_Satellite_Data(tle, &prediction->satellite_ephem.tle);

    return 0;

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


// Returns the first match on prediction->satellite_ephem.name
int find_passes(prediction_t *external_prediction, double jul_utc_start, double delta_t_minutes, criteria_t *criteria, int *count, int *number_checked, int reverse_order, int find_all)
{
    FILE *file = fopen(external_prediction->tles_filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error opening %s\n", external_prediction->tles_filename);
        return -1;
    }
    
    // 2 69-character lines plus a nul terminator
    char tle[160] = {0};
    char name[26] = {0}; 
    int found_satellite = 0;

    prediction_t prediction = {0};
    memcpy(&prediction, external_prediction, sizeof *external_prediction);
    
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
    char *ptr = NULL;

    while (fgets(name, 26, file)) {
        skip_this = 0;
        // Remove newline
        name[strlen(name) - 1] = '\0';
        // Errors caught in TLE check
        // Read 70 characters, including the newline
        ptr = fgets(tle, 71, file);
        if (ptr == NULL) {
            break;
        }
        // Read remaining characterers
        ptr = fgets(tle + 69, 71, file);
        if (ptr == NULL) {
            break;
        }
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

        Convert_Satellite_Data(tle, &prediction.satellite_ephem.tle);
        ClearFlag(ALL_FLAGS);
        select_ephemeris(&prediction.satellite_ephem.tle);
        update_satellite_position(&prediction, jul_utc_start);

        // TODO filter on perigee / apogee instead of current altitude?
        if (prediction.satellite_ephem.altitude_km >= criteria->min_altitude_km && prediction.satellite_ephem.altitude_km <= criteria->max_altitude_km) {
            internal_number_checked++;
            double utc_offset_minutes = 0;
            double minutes_until_visible = 0;
            double jul_utc_stop = jul_utc_start + criteria->max_minutes / 1400.0;
            while (get_next_pass(&prediction, jul_utc_start + utc_offset_minutes / 1440.0, jul_utc_stop, delta_t_minutes)) {
                minutes_until_visible = prediction.predicted_minutes_until_visible + utc_offset_minutes;
                utc_offset_minutes += prediction.predicted_minutes_until_visible + 60;
                if (prediction.predicted_minutes_above_0_degrees <= 0.0 || prediction.predicted_max_elevation > criteria->max_elevation || prediction.predicted_max_elevation < criteria->min_elevation) {
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
                p->minutes_away = minutes_until_visible;
                p->pass_duration = prediction.predicted_pass_duration_minutes;
                p->ascension_jul_utc = prediction.predicted_ascension_jul_utc;
                p->ascension_azimuth = prediction.predicted_ascension_azimuth;
                p->max_elevation = prediction.predicted_max_elevation;
                p->max_altitude = prediction.predicted_max_altitude;
                if (find_all == 0) {
                    break;
                }
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

    if (n_passes > 0) {
        // Sort the list
        if (reverse_order) {
            qsort(passes, n_passes, sizeof *passes, pass_sort_latest_first);
        } else {
            qsort(passes, n_passes, sizeof *passes, pass_sort_soonest_first);
        }
    }

    return 0;
}

const pass_t *get_pass(int index)
{
    const pass_t *p = NULL;
    if (index >= 0 && index < n_passes) {
        p = &(passes[index]);
    }

    return p;
}

const size_t number_of_passes(void)
{
    return n_passes;
}

void free_passes(void)
{
    free(passes);
    passes = NULL;
    n_passes = 0;
}

int get_next_pass(prediction_t *prediction, double jul_utc_start, double jul_utc_stop, double delta_t_minutes)
{
    int got_pass = 0;
    minutes_until_visible(prediction, jul_utc_start, jul_utc_stop, delta_t_minutes);
    if (prediction->predicted_minutes_until_visible >= 0) {
        // Refine estimate:
        if (prediction->predicted_minutes_until_visible < 10.0 && delta_t_minutes > 1.0/60.) {
            minutes_until_visible(prediction, jul_utc_start, jul_utc_stop, 1.0 / 60.0);
        }
        update_pass_predictions(prediction, jul_utc_start + prediction->predicted_minutes_until_visible / 1440.0, 0.25);
        got_pass = 1;
    }

    return got_pass;
}
