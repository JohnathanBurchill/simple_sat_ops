/*

    Simple Satellite Operations  ephemeres.h

    Copyright (C) 2026  Johnathan K Burchill

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
