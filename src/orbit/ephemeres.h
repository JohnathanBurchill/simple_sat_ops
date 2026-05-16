#ifndef EPHEMERES_H
#define EPHEMERES_H

#include <sgp4sdp4.h>

typedef struct ephemeres
{
    char *name;
    tle_t tle;
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


#endif // EPHEMERES_H
