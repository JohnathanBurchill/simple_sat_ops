/*

   Simple Satellite Operations  ui/auto_tcmd.h

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

// The auto-tcmd modal ('A' / --tc-file): drive a file of ASCII telecommands
// through the TX path automatically, queuing one state->tx_request per shot
// and stopping at LOS. State lives on state_t.auto_tcmd; the modal is ticked
// non-blocking alongside the main loop.

#ifndef UI_AUTO_TCMD_H
#define UI_AUTO_TCMD_H

#include "state.h"     // state_t, auto_tcmd_field_t

#ifdef __cplusplus
extern "C" {
#endif

// Open / close the modal window.
void auto_tcmd_open(state_t *state);
void auto_tcmd_close(state_t *state);

// Feed a key to the open modal. Returns 0 when the modal consumed the key,
// non-zero to let the caller handle it.
int auto_tcmd_handle_key(state_t *state, int key);

// Per-tick: advance the run state machine, queuing the next burst when due.
void auto_tcmd_tick(state_t *state);

// Snapshot the run progress for the status line / IPC broadcast. Returns 1
// when a run is active (fills sent/total/label), 0 otherwise.
int auto_tcmd_progress(state_t *state, int *sent, int *total, const char **label);

// 1 if the given modal field is a free-text field (the file-path field).
int auto_field_is_text(auto_tcmd_field_t f);

#ifdef __cplusplus
}
#endif

#endif // UI_AUTO_TCMD_H
