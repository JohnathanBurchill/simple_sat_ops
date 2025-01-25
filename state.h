#ifndef STATE_H
#define STATE_H

#define MAX_TLE_LINE_LENGTH 128

#include <sgp4sdp4.h>

typedef struct state {
    int n_options;
    char *tle_filename;
    tle_t tle;
    int run_without_rig;
    int run_without_rotator;
    int have_rig;
    int have_rotator;
} state_t;

#endif // STATE_H
