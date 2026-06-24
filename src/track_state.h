/*

   Simple Satellite Operations  src/track_state.h

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

// Orbit prediction + Doppler-frequency slice of state_t — see state.h.

#ifndef TRACK_STATE_H
#define TRACK_STATE_H

#include "prediction.h"

typedef struct track {
    int satellite_tracking;
    // 1 while the tracked satellite is within the pass window (set/cleared by
    // tracking_tick from the predicted minutes-until-visible).
    int in_pass;
    prediction_t prediction;

    // Tracked-satellite identity. target_tle_path is the file the current
    // satellite came from, so a repeat :retarget on the same file is a
    // no-op; seeded from the startup TLE path, updated on each retarget.
    // target_name is stable backing for satellite_ephem.name after a
    // retarget (the startup name points at argv / an apply_args buffer).
    char target_tle_path[1024];
    char target_name[64];

    // Nominal + Doppler-corrected frequencies (Hz). Computed each tick
    // by main.c from the prediction's range-rate. simple_sat_ops no
    // longer drives a radio directly — these are display-only and get
    // published into the IPC state events for any subscriber
    // (tx_frame_sdr can pick up the current uplink freq this way).
    double nominal_uplink_frequency_hz;
    double nominal_downlink_frequency_hz;
    double doppler_uplink_frequency_hz;
    double doppler_downlink_frequency_hz;
    int doppler_correction_enabled;
} track_t;

#endif // TRACK_STATE_H
