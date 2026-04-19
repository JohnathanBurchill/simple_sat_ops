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
#include "audio.h"
#include "radio.h"
#include "prediction.h"
#include "telemetry.h"

#include <stdint.h>
#include <termios.h>

#define MAX_TLE_LINE_LENGTH 128
#define TRACKING_PREP_TIME_MINUTES 5.0

typedef struct state 
{
    // General state
    int n_options;
    int running;
    int verbose_level;
    int in_pass;
    // Tracking
    int satellite_tracking;
    prediction_t prediction;
    // Radio control
    radio_t radio;
    int run_with_radio;
    int have_radio;
    // Antenna control
    antenna_rotator_t antenna_rotator;
    int run_with_antenna_rotator;
    int have_antenna_rotator;
    // Audio
    audio_t audio;
    // Telemetry 
    telemetry_t telemetry;
} state_t;


#endif // STATE_H
