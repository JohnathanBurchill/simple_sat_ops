/*

    Simple Satellite Operations  state.h

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

#ifndef STATE_H
#define STATE_H

#include "antenna_rotator.h"
#include "prediction.h"
#include "telemetry.h"

#include <stdint.h>
#include <termios.h>

#define MAX_TLE_LINE_LENGTH 128
#define TRACKING_PREP_TIME_MINUTES 5.0

// SDR-only state. The CAT-radio + ALSA-audio fields are gone; the B210
// is now driven externally by b210_rx_tx / tx_frame_sdr, which talk
// to simple_sat_ops over the sso_ipc socket for operator coordination.
// What remains here is the satellite tracker, the rotator, and the
// nominal/Doppler-shifted frequency outputs computed for display + IPC
// broadcast.
typedef struct state
{
    // General
    int n_options;
    int running;
    int verbose_level;
    int in_pass;

    // Tracking
    int satellite_tracking;
    prediction_t prediction;

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

    // SDR LO offset (Hz) below the nominal downlink carrier. Without this,
    // the corrected signal sits at DC after software Doppler tracking and
    // gets eaten by the B210's DC blocker whenever Doppler crosses zero
    // (i.e. exactly at TCA, the worst possible time). Offsetting the LO
    // by ~25 kHz parks the signal in a clean part of the captured 96 kHz
    // baseband for the whole pass. Configurable via --lo-offset=<kHz>.
    double rx_lo_offset_hz;

    // Antenna rotator
    antenna_rotator_t antenna_rotator;
    int run_with_antenna_rotator;
    int have_antenna_rotator;

    // Telemetry overlay (still rendered alongside the prediction).
    telemetry_t telemetry;
} state_t;


#endif // STATE_H
