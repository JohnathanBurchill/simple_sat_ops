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
#include "keybindings.h"
#include "state.h"

#include "antenna_rotator.h"  // ANTENNA_ROTATOR_OK
#include "auto_tcmd.h"
#include "cmd_line.h"
#include "prediction.h"
#include "scan_sky.h"
#include "sso_audit.h"
#include "tracking.h"
#include "tui.h"
#include "tx_compose.h"

#include <ncurses.h>
#include <stdio.h>

// --- Keyboard actions ---------------------------------------------
//
// One function per operator key (or key group). These are the bodies the
// keybindings table points at; the table lives in ui/keybindings.c and the
// dispatcher there calls these. Ported verbatim from the old key switch.

void kb_act_track(state_t *state, int key)
{
    (void) key;
    if (state->scan.mode) {
        scan_sky_start(state);
    } else {
        start_tracking(state);
        if (state->rot.antenna_rotator.fixed_target) {
            char det[128];
            snprintf(det, sizeof det,
                "mode=fixed-target az=%.1f el=%.1f",
                state->rot.antenna_rotator.target_azimuth,
                state->rot.antenna_rotator.target_elevation);
            sso_audit_event("track-on", det);
        } else {
            sso_audit_event("track-on",
                state->track.prediction.satellite_ephem.tle.sat_name[0]
                    ? state->track.prediction.satellite_ephem.tle.sat_name : "");
        }
    }
}

void kb_act_stop(state_t *state, int key)
{
    (void) key;
    if (state->scan.active) {
        scan_sky_stop(state, "user");
    }
    stop_tracking(state);
}

void kb_act_reset(state_t *state, int key)
{
    (void) key;
    stop_tracking(state);
    point_to_stationary_target(state, 0.0, 0.0);
}

// All eight antenna jog keys. The key picks the axis and the step; a manual
// jog drops the satellite-tracking / under-control flags first. flushinp
// discards any queued repeats so a held key can't pile up moves.
void kb_act_jog(state_t *state, int key)
{
    int result = 0;
    state->track.satellite_tracking = 0;
    state->rot.antenna_rotator.antenna_is_under_control = 0;
    switch (key) {
        case '[': result = main_rotator_increase_azimuth(&state->rot,   -5.0); break;
        case ']': result = main_rotator_increase_azimuth(&state->rot,    5.0); break;
        case '{': result = main_rotator_increase_azimuth(&state->rot,   -1.0); break;
        case '}': result = main_rotator_increase_azimuth(&state->rot,    1.0); break;
        case ',': result = main_rotator_increase_elevation(&state->rot, -5.0); break;
        case '.': result = main_rotator_increase_elevation(&state->rot,  5.0); break;
        case '<': result = main_rotator_increase_elevation(&state->rot, -1.0); break;
        case '>': result = main_rotator_increase_elevation(&state->rot,  1.0); break;
        default:  return;
    }
    if (result == ANTENNA_ROTATOR_OK) {
        state->rot.antenna_rotator.antenna_is_moving = 1;
    }
    flushinp();
}

void kb_act_tx_compose(state_t *state, int key)
{
    (void) key;
    tx_compose_open(state);
}

void kb_act_auto_tcmd(state_t *state, int key)
{
    (void) key;
    auto_tcmd_open(state);
}

void kb_act_lock(state_t *state, int key)
{
    (void) key;
    state->ui.keyboard_unlocked = !state->ui.keyboard_unlocked;
}

void kb_act_quit(state_t *state, int key)
{
    (void) key;
    state->app.running = 0;
}

void kb_act_cmdline(state_t *state, int key)
{
    (void) key;
    cmd_enter(&state->cmd);
}

// --- Router -------------------------------------------------------
//
// Read one key and hand it to whoever owns the screen: the active modal /
// command line, or -- when none is up -- the keybindings dispatcher, which
// applies the lock and routes to the action above.
void input_handle_keys(state_t *state)
{
    int key = getch();
    // Resize is global -- handle it before any modal / command-line routing
    // so the layout repaints cleanly no matter what is on screen. The main
    // loop repaints when it sees need_full_redraw.
    if (key == KEY_RESIZE) {
        tui_handle_resize();
        state->ui.need_full_redraw = 1;
        return;
    }
    if (state->tx.tx_compose_active) {
        if (!tx_compose_handle_key(state, key)) {
            tx_compose_close(&state->tx);
        }
    } else if (state->tx.auto_tcmd_active) {
        if (!auto_tcmd_handle_key(state, key)) {
            auto_tcmd_close(&state->tx);
        }
    } else if (state->cmd.active) {
        cmd_handle_key(key, state);
    } else {
        keybindings_dispatch(state, key);
    }
}
