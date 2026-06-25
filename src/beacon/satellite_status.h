/*

    Simple Satellite Operations  satellite_status.h

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

#ifndef SATELLITE_STATUS_H
#define SATELLITE_STATUS_H

#include <stdint.h>

typedef struct satellite_status {
    char name[64];
    char id[64];
    char f_uplink_mhz[64];
    char f_downlink_mhz[64];
    char f_beacon_mhz[64];
    char mode[64];
    char callsign[64];
    char status[64];
} satellite_status_t;

int parse_satellite_status_file(char *filename, satellite_status_t **status_list, int *n_entries);


#endif // SATELLITE_STATUS_H
