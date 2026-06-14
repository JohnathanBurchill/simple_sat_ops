/*

   Simple Satellite Operations  ui/input.c

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

#include "input.h"
#include "state.h"

#include "antenna_rotator.h"  // ANTENNA_ROTATOR_OK
#include "auto_tcmd.h"
#include "cmd_line.h"
#include "prediction.h"
#include "scan_sky.h"
#include "sso_audit.h"
#include "tracking.h"
#include "tx_compose.h"

#include <ncurses.h>
#include <stdio.h>

void input_handle_keys(state_t *state, int *keyboard_unlocked)
{
    // Scratch for the jog keys' main_rotator_increase_* return codes.
    int antenna_rotator_result = 0;

    int key = getch();
    if (state->tx_compose_active) {
        if (!tx_compose_handle_key(state, key)) {
            tx_compose_close(state);
        }
    } else if (state->auto_tcmd_active) {
        if (!auto_tcmd_handle_key(state, key)) {
            auto_tcmd_close(state);
        }
    } else if (state->cmd.active) {
        cmd_handle_key(key, state);
    } else if (key == 'K') {
        *keyboard_unlocked = !*keyboard_unlocked;
    } else if (*keyboard_unlocked) {
        switch (key) {
            case ':':
                cmd_enter(state);
                break;
            case 'q':
                state->running = 0;
                break;
            case 'T':
                if (state->scan.mode) {
                    scan_sky_start(state);
                } else {
                    start_tracking(state);
                    if (state->antenna_rotator.fixed_target) {
                        char det[128];
                        snprintf(det, sizeof det,
                            "mode=fixed-target az=%.1f el=%.1f",
                            state->antenna_rotator.target_azimuth,
                            state->antenna_rotator.target_elevation);
                        sso_audit_event("track-on", det);
                    } else {
                        sso_audit_event("track-on",
                            state->prediction.satellite_ephem.tle.sat_name[0]
                                ? state->prediction.satellite_ephem.tle.sat_name : "");
                    }
                }
                break;
            case 's':
                if (state->scan.active) {
                    scan_sky_stop(state, "user");
                }
                stop_tracking(state);
                break;
            case 'r':
                stop_tracking(state);
                point_to_stationary_target(state, 0.0, 0.0);
                break;
            case '[':
                state->satellite_tracking = 0;
                state->antenna_rotator.antenna_is_under_control = 0;
                antenna_rotator_result = main_rotator_increase_azimuth(
                    state, -5.0);
                if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                flushinp();
                break;
            case ']':
                state->satellite_tracking = 0;
                state->antenna_rotator.antenna_is_under_control = 0;
                antenna_rotator_result = main_rotator_increase_azimuth(
                    state, 5.0);
                if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                flushinp();
                break;
            case '{':
                state->satellite_tracking = 0;
                state->antenna_rotator.antenna_is_under_control = 0;
                antenna_rotator_result = main_rotator_increase_azimuth(
                    state, -1.0);
                if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                flushinp();
                break;
            case '}':
                state->satellite_tracking = 0;
                state->antenna_rotator.antenna_is_under_control = 0;
                antenna_rotator_result = main_rotator_increase_azimuth(
                    state, 1.0);
                if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                flushinp();
                break;
            case ',':
                state->satellite_tracking = 0;
                state->antenna_rotator.antenna_is_under_control = 0;
                antenna_rotator_result = main_rotator_increase_elevation(
                    state, -5.0);
                if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                flushinp();
                break;
            case '.':
                state->satellite_tracking = 0;
                state->antenna_rotator.antenna_is_under_control = 0;
                antenna_rotator_result = main_rotator_increase_elevation(
                    state, 5.0);
                if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                flushinp();
                break;
            case '<':
                state->satellite_tracking = 0;
                state->antenna_rotator.antenna_is_under_control = 0;
                antenna_rotator_result = main_rotator_increase_elevation(
                    state, -1.0);
                if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                flushinp();
                break;
            case '>':
                state->satellite_tracking = 0;
                state->antenna_rotator.antenna_is_under_control = 0;
                antenna_rotator_result = main_rotator_increase_elevation(
                    state, 1.0);
                if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                    state->antenna_rotator.antenna_is_moving = 1;
                }
                flushinp();
                break;
            case 't':
                tx_compose_open(state);
                break;
            case 'A':
                auto_tcmd_open(state);
                break;
            default:
                break;
        }
    }
}
