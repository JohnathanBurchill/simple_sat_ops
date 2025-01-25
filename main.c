#include "state.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <hamlib/rig.h>
#include <hamlib/rotator.h>
#include "sgp4sdp4.h"

/* RAO site observer location in Priddis, SW of Calgary */
#define RAO_LATITUDE  50.8812  // Latitude in degrees
#define RAO_LONGITUDE -114.2914 // Longitude in degrees
#define RAO_ALTITUDE  1250.0   // Altitude in meters

/* Satellite communication frequencies */
#define VHF_UPLINK_FREQ   145800000ULL  /* Uplink: 145.800 MHz */
#define UHF_DOWNLINK_FREQ 435300000ULL  /* Downlink: 435.300 MHz */

/* Convert satellite ECI position to Az/El for observer location */
void eci_to_azel(vector_t *pos, geodetic_t *observer, time_t timestamp, double *az, double *el) {
    vector_t obs_pos, obs_vel, range;
    double range_magnitude, sin_lat, cos_lat, sin_lon, cos_lon;
    double top_s, top_e, top_z;

    /* Calculate observer's position and velocity */
    Calculate_User_PosVel((double)timestamp, observer, &obs_pos, &obs_vel);

    /* Relative position (range vector) */
    Vec_Sub(pos, &obs_pos, &range);

    /* Magnitude of the range vector */
    Magnitude(&range);
    range_magnitude = range.w;

    /* Convert observer's latitude and longitude to radians */
    sin_lat = sin(observer->lat);
    cos_lat = cos(observer->lat);
    sin_lon = sin(observer->lon);
    cos_lon = cos(observer->lon);

    /* Compute topocentric coordinates */
    top_s = sin_lat * cos_lon * range.x + sin_lat * sin_lon * range.y - cos_lat * range.z;
    top_e = -sin_lon * range.x + cos_lon * range.y;
    top_z = cos_lat * cos_lon * range.x + cos_lat * sin_lon * range.y + sin_lat * range.z;

    /* Azimuth and elevation */
    *az = atan2(top_e, -top_s) * 180.0 / M_PI;
    if (*az < 0) *az += 360.0;
    *el = asin(top_z / range_magnitude) * 180.0 / M_PI;
}

void usage(FILE *dest, const char *name) 
{
    fprintf(dest, "usage: %s <tle_file>\n", name);
    return;
}

int main(int argc, char **argv) 
{
    state_t state = {0};

    for (int i = 0; i < argc; i++) {
        if (strcmp("--without-rig", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rig = 1;
        }
        else if (strcmp("--without-rotator", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rotator = 1;
        }
        else if (strcmp("--without-hardware", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rig = 1;
            state.run_without_rotator = 1;
        }
        else if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0]);
            return 0;
        }
        else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 1;
        }

    }
    if (argc - state.n_options != 2) {
        usage(stderr, argv[0]);
        return 1;
    }

    /* Open TLE file */
    state.tle_filename = argv[1];

    /* Parse TLE data */
    tle_t tle = {0};
    Input_Tle_Set(state.tle_filename, &tle);

    /* Initialize Hamlib for rig and rotator */
    rig_set_debug(RIG_DEBUG_NONE);
    RIG *rig = rig_init(RIG_MODEL_IC9700);
    if (!rig) {
        fprintf(stderr, "Failed to initialize rig support.\n");
        return 1;
    }
    ROT *rot = rot_init(ROT_MODEL_GS232A);
    if (!rot) {
        fprintf(stderr, "Failed to initialize rotator support.\n");
        return 1;
    }

    strncpy(rig->state.rigport.pathname, "/dev/ttyUSB1", sizeof(rig->state.rigport.pathname) - 1);
    rig->state.rigport.pathname[sizeof(rig->state.rigport.pathname) - 1] = '\0';
    if (rig_open(rig) != RIG_OK) {
        fprintf(stderr, "Error opening rig. Is it plugged into USB and powered?.\n");
        if (!state.run_without_rig) {
            rig_cleanup(rig);
            rot_cleanup(rot);
            return 1;
        }
    }
    state.have_rig = 1;

    strncpy(rot->state.rotport.pathname, "/dev/ttyUSB0", sizeof(rot->state.rotport.pathname) - 1);
    rot->state.rotport.pathname[sizeof(rot->state.rotport.pathname) - 1] = '\0';
        if (rot_open(rot) != RIG_OK) {
        fprintf(stderr, "Error opening rotator. Is it plugged into USB and powered?.\n");
        if (!state.run_without_rotator) {
            rig_cleanup(rig);
            rot_cleanup(rot);
            return 1;
        }
    }
    state.have_rotator = 1;

    /* Set up observer location */
    geodetic_t observer = {
        .lat = RAO_LATITUDE * M_PI / 180.0,
        .lon = RAO_LONGITUDE * M_PI / 180.0,
        .alt = RAO_ALTITUDE / 1000.0,
    };

    /* Tracking loop */
    int tracking = 0;  // 1 if tracking, 0 if idle
    time_t idle_start = 0;  // Time when the satellite was last tracked
    struct timeval tv = {0};
    double now = 0.0;

    while (1) {
        gettimeofday(&tv, NULL);
        now = tv.tv_sec + tv.tv_usec / 1e6;
        double az, el;
        vector_t pos, vel;
        double doppler_uplink, doppler_downlink;
        int ret;

        /* Propagate satellite position */
        SGP4((now - tle.epoch) * 1440.0 / 86400.0, &tle, &pos, &vel);

        /* Convert ECI to Az/El */
        eci_to_azel(&pos, &observer, now, &az, &el);

        printf("EL: %6.2f  Az: %6.2f\n", el, az);
        if (el > -5.0) { // Satellite is above -5 degrees elevation
            if (!tracking) {
                printf("Satellite is rising. Starting tracking...\n");
                tracking = 1;
            }

            /* Calculate Doppler shift */
            vector_t obs_pos, obs_vel, relative_vel;
            Calculate_User_PosVel((double)now, &observer, &obs_pos, &obs_vel);
            Vec_Sub(&vel, &obs_vel, &relative_vel);
            double relative_speed = Dot(&relative_vel, &pos) / pos.w;  // Radial velocity

            doppler_uplink = VHF_UPLINK_FREQ * (1 + relative_speed / 299792.458);  // Speed of light in km/s
            doppler_downlink = UHF_DOWNLINK_FREQ * (1 + relative_speed / 299792.458);

            /* Point rotator to Az/El */
            if (state.have_rotator || !state.run_without_rotator) {
                if ((ret = rot_set_position(rot, az, el)) != RIG_OK) {
                    fprintf(stderr, "Error setting rotor position: %s\n", rigerror(ret));
                }
            }

            /* Set rig frequencies with Doppler correction */
            if (state.have_rig || !state.run_without_rig) {
                if ((ret = rig_set_freq(rig, RIG_VFO_A, (unsigned long)doppler_uplink)) != RIG_OK ||
                    (ret = rig_set_freq(rig, RIG_VFO_B, (unsigned long)doppler_downlink)) != RIG_OK) {
                    fprintf(stderr, "Error setting rig frequency: %s\n", rigerror(ret));
                }
            }

            idle_start = 0;  // Reset idle timer
        } else { // Satellite is below -5 degrees elevation
            if (tracking) {
                printf("Satellite has set. Stopping tracking...\n");
                tracking = 0;
                idle_start = now;  // Start idle timer
            }

            /* Check if timeout has elapsed */
            if (idle_start > 0 && difftime(now, idle_start) > 600) {  // 10-minute timeout
                printf("Timeout reached. Exiting tracking loop.\n");
                break;
            }
        }

        /* Sleep for a short interval (e.g., 1 second) */
        sleep(1);
    }

    /* Cleanup */
    if (rig) {
        rig_close(rig);
        rig_cleanup(rig);
    }
    if (rot) {
        rot_close(rot);
        rot_cleanup(rot);
    }

    return 0;
}
