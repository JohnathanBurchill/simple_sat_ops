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

int main(int argc, char **argv) {
    RIG *rig;
    ROT *rot;
    int ret;

    if (argc < 3) {
        fprintf(stderr, "Usage: %s \"TLE Line 1\" \"TLE Line 2\"\n", argv[0]);
        return 1;
    }

    /* Parse TLE data */
    tle_t tle;
    Convert_Satellite_Data(argv[1], &tle);
    Convert_Satellite_Data(argv[2], &tle);

    /* Initialize Hamlib for rig and rotator */
    rig = rig_init(RIG_MODEL_IC9700);
    rot = rot_init(ROT_MODEL_GS232);

    if (!rig || !rot) {
        fprintf(stderr, "Failed to initialize rig or rotor.\n");
        return 1;
    }

    /* Set rig port */
    strncpy(rig->state.rigport.pathname, "/dev/ttyUSB1", sizeof(rig->state.rigport.pathname) - 1);
    rig->state.rigport.pathname[sizeof(rig->state.rigport.pathname) - 1] = '\0'; // Ensure null termination
    if ((ret = rig_open(rig)) != RIG_OK) {
        fprintf(stderr, "Error opening rig: %s\n", rigerror(ret));
        rig_cleanup(rig);
        return 1;
    }

    /* Set rig port */
    strncpy(rig->state.rigport.pathname, "/dev/ttyUSB1", sizeof(rig->state.rigport.pathname) - 1);
    rig->state.rigport.pathname[sizeof(rig->state.rigport.pathname) - 1] = '\0'; // Ensure null termination
    if ((ret = rig_open(rig)) != RIG_OK) {
        fprintf(stderr, "Error opening rig: %s\n", rigerror(ret));
        rig_cleanup(rig);
        return 1;
    }

    if ((ret = rig_open(rig)) != RIG_OK || (ret = rot_open(rot)) != RIG_OK) {
        fprintf(stderr, "Error opening rig or rotator: %s\n", rigerror(ret));
        return 1;
    }

    /* Set up observer location */
    geodetic_t observer = {
        .lat = RAO_LATITUDE * M_PI / 180.0, // Convert degrees to radians
        .lon = RAO_LONGITUDE * M_PI / 180.0, // Convert degrees to radians
        .alt = RAO_ALTITUDE / 1000.0 // Convert meters to kilometers
    };

    /* Tracking loop */
    for (int i = 0; i < 10; i++) {
        time_t now = time(NULL);
        double az, el;
        vector_t pos, vel;

        /* Propagate satellite position */
        SGP4((double)((now - tle.epoch) * 1440.0 / 86400.0), &tle, &pos, &vel);

        /* Convert ECI to Az/El */
        eci_to_azel(&pos, &observer, now, &az, &el);

        /* Point rotator to Az/El */
        if ((ret = rot_set_position(rot, az, el)) != RIG_OK) {
            fprintf(stderr, "Error setting rotor position: %s\n", rigerror(ret));
        }

        /* Optionally set rig frequencies */
        if ((ret = rig_set_freq(rig, RIG_VFO_A, VHF_UPLINK_FREQ)) != RIG_OK ||
            (ret = rig_set_freq(rig, RIG_VFO_B, UHF_DOWNLINK_FREQ)) != RIG_OK) {
            fprintf(stderr, "Error setting rig frequency: %s\n", rigerror(ret));
        }

        /* Sleep for 1 second */
        sleep(1);
    }

    /* Cleanup */
    rig_close(rig);
    rot_close(rot);
    rig_cleanup(rig);
    rot_cleanup(rot);

    return 0;
}
