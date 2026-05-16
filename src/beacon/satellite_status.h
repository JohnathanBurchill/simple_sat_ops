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
