#ifndef STATE_H
#define STATE_H

#define MAX_TLE_LINE_LENGTH 128

#include <sgp4sdp4.h>

typedef struct ephemeres
{
    vector_t position;
    vector_t velocity;
    double speed_km_s;
    geodetic_t position_geodetic;
    double azimuth;
    double elevation;
    double range_km;
    double range_rate_km_s;
    double latitude;
    double longitude;
    double altitude_km;
    vector_t observation_set;
} ephemeres_t;

typedef struct state {
    int n_options;
    char *tle_filename;
    tle_t tle;
    double jul_epoch;
    double minutes_since_epoch;
    int run_without_rig;
    int run_without_rotator;
    int have_rig;
    int have_rotator;
    ephemeres_t observer;
    ephemeres_t satellite;
    vector_t observer_satellite_relative_velocity;
    double observer_satellite_relative_speed;
    double doppler_uplink_frequency;
    double doppler_downlink_frequency;
} state_t;

#endif // STATE_H
