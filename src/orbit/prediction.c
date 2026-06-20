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
#include "oem.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sgp4sdp4.h>
#include <ncurses.h>
#include <regex.h>

// WGS84 ellipsoid constants — used for ITRF <-> geodetic conversions on
// the OEM state path. Values match standard WGS84; minor disagreement
// with sgp4sdp4's Earth model is below our pass-prediction tolerance.
#define WGS84_A   6378.137            // km, semi-major axis
#define WGS84_F   (1.0 / 298.257223563)
#define WGS84_E2  (2.0 * WGS84_F - WGS84_F * WGS84_F)

static void geodetic_to_ecef(double lat_rad, double lon_rad, double alt_km,
                             double out[3])
{
    double sl = sin(lat_rad), cl = cos(lat_rad);
    double so = sin(lon_rad), co = cos(lon_rad);
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sl * sl);
    out[0] = (N + alt_km) * cl * co;
    out[1] = (N + alt_km) * cl * so;
    out[2] = (N * (1.0 - WGS84_E2) + alt_km) * sl;
}

// Rotate an ITRF vector anchored at the observer origin into the local
// East-North-Up frame at the observer's geodetic latitude/longitude.
static void ecef_delta_to_enu(double obs_lat_rad, double obs_lon_rad,
                              const double d[3], double enu[3])
{
    double sl = sin(obs_lat_rad), cl = cos(obs_lat_rad);
    double so = sin(obs_lon_rad), co = cos(obs_lon_rad);
    enu[0] = -so * d[0] + co * d[1];
    enu[1] = -sl * co * d[0] - sl * so * d[1] + cl * d[2];
    enu[2] =  cl * co * d[0] + cl * so * d[1] + sl * d[2];
}

// Bowring's closed-form ECEF -> geodetic (WGS84), accurate to < 1 mm for
// altitudes we care about.
static void ecef_to_geodetic(const double r[3],
                             double *lat_rad, double *lon_rad, double *alt_km)
{
    double x = r[0], y = r[1], z = r[2];
    double p = sqrt(x * x + y * y);
    double b = WGS84_A * (1.0 - WGS84_F);
    double ep2 = (WGS84_A * WGS84_A - b * b) / (b * b);
    double theta = atan2(z * WGS84_A, p * b);
    double st = sin(theta), ct = cos(theta);
    double lat = atan2(z + ep2 * b * st * st * st,
                       p - WGS84_E2 * WGS84_A * ct * ct * ct);
    double sl = sin(lat);
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sl * sl);
    *lat_rad = lat;
    *lon_rad = atan2(y, x);
    *alt_km = p / cos(lat) - N;
}

// Populate prediction->satellite_ephem from an OEM sample. Out-of-window
// is signalled with elevation = -90° so the pass-finder's above-horizon
// walks terminate naturally.
static void fill_ephem_from_oem(prediction_t *prediction, double jul_utc)
{
    double r[3], v[3];
    if (oem_sample_at(prediction->oem, jul_utc, r, v) != 0) {
        prediction->satellite_ephem.azimuth = 0.0;
        prediction->satellite_ephem.elevation = -90.0;
        prediction->satellite_ephem.range_km = 0.0;
        prediction->satellite_ephem.range_rate_km_s = 0.0;
        prediction->satellite_ephem.altitude_km = 0.0;
        prediction->satellite_ephem.speed_km_s = 0.0;
        return;
    }

    double obs[3];
    geodetic_to_ecef(prediction->observer_ephem.position_geodetic.lat,
                     prediction->observer_ephem.position_geodetic.lon,
                     prediction->observer_ephem.position_geodetic.alt,
                     obs);
    double rng[3] = { r[0] - obs[0], r[1] - obs[1], r[2] - obs[2] };
    double range_km = sqrt(rng[0]*rng[0] + rng[1]*rng[1] + rng[2]*rng[2]);

    double enu[3];
    ecef_delta_to_enu(prediction->observer_ephem.position_geodetic.lat,
                      prediction->observer_ephem.position_geodetic.lon,
                      rng, enu);

    double az_rad = atan2(enu[0], enu[1]);
    if (az_rad < 0.0) az_rad += 2.0 * M_PI;
    double horiz = sqrt(enu[0]*enu[0] + enu[1]*enu[1]);
    double el_rad = atan2(enu[2], horiz);

    // Range-rate: sat moves in ECEF with v; observer is stationary.
    double rrate = 0.0;
    if (range_km > 0.0) {
        rrate = (v[0] * rng[0] + v[1] * rng[1] + v[2] * rng[2]) / range_km;
    }

    double sat_lat, sat_lon, sat_alt;
    ecef_to_geodetic(r, &sat_lat, &sat_lon, &sat_alt);

    ephemeres_t *se = &prediction->satellite_ephem;
    se->azimuth = Degrees(az_rad);
    se->elevation = Degrees(el_rad);
    se->range_km = range_km;
    se->range_rate_km_s = rrate;
    se->latitude = Degrees(sat_lat);
    se->longitude = Degrees(sat_lon);
    se->altitude_km = sat_alt;
    se->speed_km_s = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

int tle_default_path(char *out_path, size_t out_cap)
{
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') {
        return -1;
    }
    int n = snprintf(out_path, out_cap,
                     "%s/.local/state/simple_sat_ops/active.tle", home);
    if (n < 0 || (size_t)n >= out_cap) {
        return -1;
    }
    return 0;
}

static pass_t *passes = NULL;
static size_t n_passes = 0;

// Reads a line from f, trims trailing \r and/or \n, NUL-terminates.
// Returns 1 on success, 0 on EOF/error. Callers must size buf large
// enough to hold the longest expected line + terminator.
static int read_tle_line(char *buf, size_t size, FILE *f)
{
    if (fgets(buf, (int)size, f) == NULL) {
        return 0;
    }
    size_t n = strlen(buf);
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) {
        buf[--n] = '\0';
    }
    return 1;
}

void update_satellite_position(prediction_t *prediction, double jul_utc)
{
    // OEM-backed trajectory: interpolate ITRF state and derive az/el
    // directly; skip the TLE/SGP4 machinery.
    if (prediction->oem != NULL) {
        fill_ephem_from_oem(prediction, jul_utc);
        return;
    }

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
    // Capture the most recent visible sample so we can record LOS once
    // elevation drops below zero. predicted_descent_* is initialized
    // here so a partial / no-pass walk leaves the fields well-defined.
    external_prediction->predicted_descent_azimuth  = 0.0;
    external_prediction->predicted_descent_jul_utc  = 0.0;
    double last_visible_jul = 0.0;
    double last_visible_az  = 0.0;
    while (current_elevation > -5.0) {
        // Propagate to this step's time and read elevation, altitude, and
        // azimuth from that one sample. Reading the elevation gate from a
        // different sample than the azimuth/time it's paired with put the
        // AOS/LOS azimuths and the above-30-degrees count one step out of
        // phase; keep them coherent.
        double jul_now = jul_utc + pass_duration / 1440.0;
        update_satellite_position(&prediction, jul_now);
        current_elevation = prediction.satellite_ephem.elevation;
        current_altitude  = prediction.satellite_ephem.altitude_km;
        double current_azimuth = prediction.satellite_ephem.azimuth;
        if (current_elevation > 0.0) {
            minutes_above_0_degrees += delta_t_minutes;
            if (!ascended) {
                ascended = 1;
                external_prediction->predicted_ascension_jul_utc = jul_now;
                external_prediction->predicted_ascension_azimuth = current_azimuth;
            }
            last_visible_jul = jul_now;
            last_visible_az  = current_azimuth;
            if (max_altitude < current_altitude) {
                max_altitude = current_altitude;
            }
            if (current_elevation > 30.0) {
                minutes_above_30_degrees += delta_t_minutes;
            }
        }
        if (max_elevation < current_elevation) {
            max_elevation = current_elevation;
        }
        pass_duration += delta_t_minutes;
    }
    external_prediction->predicted_pass_duration_minutes = pass_duration;
    external_prediction->predicted_minutes_above_0_degrees = minutes_above_0_degrees;
    external_prediction->predicted_minutes_above_30_degrees = minutes_above_30_degrees;
    external_prediction->predicted_max_elevation = max_elevation;
    external_prediction->predicted_max_altitude = max_altitude;
    if (ascended) {
        external_prediction->predicted_descent_jul_utc = last_visible_jul;
        external_prediction->predicted_descent_azimuth = last_visible_az;
    }

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
    // A NULL name would crash the strlen/strncmp match below. Callers are
    // expected to set one, but fail cleanly rather than segfault if not.
    if (prediction->satellite_ephem.name == NULL) {
        fprintf(stderr, "load_tle: no satellite name set for %s\n",
                prediction->tles_filename ? prediction->tles_filename
                                          : "(no TLE file)");
        return -2;
    }
    FILE *file = fopen(prediction->tles_filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error opening %s\n", prediction->tles_filename);
        return -1;
    }

    // sgp4sdp4 expects two 69-char TLE lines joined by \n, NUL-terminated
    char tle[139] = {0};
    char name[128] = {0};
    char line1[80] = {0};
    char line2[80] = {0};
    int found_satellite = 0;

    while (read_tle_line(name, sizeof(name), file)) {
        if (strncmp(prediction->satellite_ephem.name, name, strlen(prediction->satellite_ephem.name)) == 0) {
            if (!read_tle_line(line1, sizeof(line1), file)) {
                break;
            }
            if (!read_tle_line(line2, sizeof(line2), file)) {
                break;
            }
            size_t l1 = strlen(line1);
            size_t l2 = strlen(line2);
            if (l1 > 69) l1 = 69;
            if (l2 > 69) l2 = 69;
            memset(tle, 0, sizeof(tle));
            memcpy(tle, line1, l1);
            memcpy(tle + 69, line2, l2);
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


// Append a pass_t to the passes list using the prediction's computed
// pass metrics. Returns 0 on success, -4 on OOM.
static int append_pass(const char *name, double minutes_until_visible,
                       const prediction_t *prediction)
{
    void *mem = realloc(passes, sizeof *passes * (n_passes + 1));
    if (mem == NULL) {
        fprintf(stderr, "Unable to allocate memory for the pass info.\n");
        return -4;
    }
    passes = mem;
    n_passes++;
    memset(&passes[n_passes - 1], 0, sizeof *passes);
    pass_t *p = &passes[n_passes - 1];
    (void)strncpy(p->name, name, sizeof(p->name) - 1);
    (void)strncpy(p->tle,  name, sizeof(p->tle)  - 1);
    p->minutes_away        = minutes_until_visible;
    p->pass_duration       = prediction->predicted_pass_duration_minutes;
    p->ascension_jul_utc   = prediction->predicted_ascension_jul_utc;
    p->ascension_azimuth   = prediction->predicted_ascension_azimuth;
    p->max_elevation       = prediction->predicted_max_elevation;
    p->max_altitude        = prediction->predicted_max_altitude;
    return 0;
}

// Returns the first match on prediction->satellite_ephem.name
int find_passes(prediction_t *external_prediction, double jul_utc_start, double delta_t_minutes, criteria_t *criteria, int *count, int *number_checked, int reverse_order, int find_all)
{
    // OEM trajectory path: single satellite, no file loop, no regex /
    // constellation filtering (the operator chose this specific trajectory).
    if (external_prediction->oem != NULL) {
        prediction_t prediction = {0};
        memcpy(&prediction, external_prediction, sizeof *external_prediction);
        int internal_number_checked = 0;
        const char *name = external_prediction->oem->object_name;
        if (name == NULL || name[0] == '\0') name = "UNKNOWN";

        // Don't cap jul_utc_stop at the OEM window — oem_sample_at()
        // transparently extrapolates past it via two-body Kepler.
        double jul_utc_stop = jul_utc_start + criteria->max_minutes / 1440.0;

        update_satellite_position(&prediction, jul_utc_start);
        if (prediction.satellite_ephem.altitude_km >= criteria->min_altitude_km &&
            prediction.satellite_ephem.altitude_km <= criteria->max_altitude_km) {
            internal_number_checked = 1;
            double utc_offset_minutes = 0;
            double minutes_until_visible = 0;
            while (get_next_pass(&prediction,
                                 jul_utc_start + utc_offset_minutes / 1440.0,
                                 jul_utc_stop, delta_t_minutes)) {
                minutes_until_visible = prediction.predicted_minutes_until_visible
                                      + utc_offset_minutes;
                utc_offset_minutes += prediction.predicted_minutes_until_visible + 60;
                if (prediction.predicted_minutes_above_0_degrees <= 0.0 ||
                    prediction.predicted_max_elevation > criteria->max_elevation ||
                    prediction.predicted_max_elevation < criteria->min_elevation) {
                    continue;
                }
                int rc = append_pass(name, minutes_until_visible, &prediction);
                if (rc != 0) return rc;
                if (find_all == 0) break;
            }
        }

        if (count) *count = 1;
        if (number_checked) *number_checked = internal_number_checked;
        if (n_passes > 0) {
            qsort(passes, n_passes, sizeof *passes,
                  reverse_order ? pass_sort_latest_first : pass_sort_soonest_first);
        }
        return 0;
    }

    FILE *file = fopen(external_prediction->tles_filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error opening %s\n", external_prediction->tles_filename);
        return -1;
    }
    
    // sgp4sdp4 expects two 69-char TLE lines joined by \n, NUL-terminated
    char tle[160] = {0};
    char name[128] = {0};
    char line1[80] = {0};
    char line2[80] = {0};

    prediction_t prediction = {0};
    memcpy(&prediction, external_prediction, sizeof *external_prediction);

    // Check every TLE
    int internal_count = 0;
    int internal_number_checked = 0;
    regex_t pattern = {0};
    if (criteria->regex != NULL) {
        int flags = REG_EXTENDED;
        if (criteria->regex_ignore_case) {
            flags |= REG_ICASE;
        }
        int res = regcomp(&pattern, criteria->regex, flags);
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

    while (read_tle_line(name, sizeof(name), file)) {
        skip_this = 0;
        if (!read_tle_line(line1, sizeof(line1), file)) {
            break;
        }
        if (!read_tle_line(line2, sizeof(line2), file)) {
            break;
        }
        size_t l1 = strlen(line1);
        size_t l2 = strlen(line2);
        if (l1 > 69) l1 = 69;
        if (l2 > 69) l2 = 69;
        memset(tle, 0, sizeof(tle));
        memcpy(tle, line1, l1);
        memcpy(tle + 69, line2, l2);
        // Calculate minutes away
        if (!Good_Elements(tle)) {
            fprintf(stderr, "Invalid TLE\n");
            regfree(&pattern);
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
        // Literal prefix match on the satellite name overrides regex and
        // constellation filtering — the user asked for this sat specifically.
        if (criteria->name_prefix != NULL) {
            if (strncmp(name, criteria->name_prefix, strlen(criteria->name_prefix)) != 0) {
                continue;
            }
        } else {
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
        }

        Convert_Satellite_Data(tle, &prediction.satellite_ephem.tle);
        ClearFlag(ALL_FLAGS);
        select_ephemeris(&prediction.satellite_ephem.tle);
        update_satellite_position(&prediction, jul_utc_start);

        // TODO filter on perigee / apogee instead of current altitude?
        if (prediction.satellite_ephem.altitude_km >= criteria->min_altitude_km && prediction.satellite_ephem.altitude_km <= criteria->max_altitude_km) {
            internal_number_checked++;
            // For fresh OPM-derived TLEs the epoch can be in the future
            // (e.g. ExoLaunch's deployment-time TLEs, used hours before
            // the actual deploy). SGP4 back-propagation from a fresh
            // post-deployment state isn't physically meaningful, so we
            // pin the per-TLE search start at the epoch when the epoch
            // is later than the user's t0. Pre-loading utc_offset_minutes
            // with the skip preserves the "minutes from t0" semantics of
            // predicted_minutes_until_visible + utc_offset_minutes.
            double utc_offset_minutes = 0;
            if (prediction.jul_epoch > jul_utc_start) {
                utc_offset_minutes = (prediction.jul_epoch - jul_utc_start) * 1440.0;
            }
            double minutes_until_visible = 0;
            double jul_utc_stop = jul_utc_start + criteria->max_minutes / 1440.0;
            while (get_next_pass(&prediction, jul_utc_start + utc_offset_minutes / 1440.0, jul_utc_stop, delta_t_minutes)) {
                minutes_until_visible = prediction.predicted_minutes_until_visible + utc_offset_minutes;
                utc_offset_minutes += prediction.predicted_minutes_until_visible + 60;
                if (prediction.predicted_minutes_above_0_degrees <= 0.0 || prediction.predicted_max_elevation > criteria->max_elevation || prediction.predicted_max_elevation < criteria->min_elevation) {
                    continue;
                }
                int rc = append_pass(name, minutes_until_visible, &prediction);
                if (rc != 0) {
                    regfree(&pattern);
                    fclose(file);
                    return rc;
                }
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
    if (index >= 0 && (size_t) index < n_passes) {
        p = &(passes[index]);
    }

    return p;
}

size_t number_of_passes(void)
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
