/*

   Simple Satellite Operations  ui/cmd_line.h

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

// The vi-style ':' command line at the bottom of the operator screen: entry,
// in-line editing + history + tab-completion, the command dispatcher (freq /
// gain / track / retarget / spectrum / modals / ...), the preview/executed
// broadcasts to viewers, and the prompt renderer. State lives on state_t.cmd.

#ifndef UI_CMD_LINE_H
#define UI_CMD_LINE_H

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Enter command mode (show the ':' prompt; route keys through the handler).
void cmd_enter(state_t *state);

// Feed a key to the active command line. Returns 0 when the line stays open,
// non-zero once it has closed (executed or cancelled).
int cmd_handle_key(int key, state_t *state);

// Draw the prompt + status line at the bottom of the screen.
void cmd_render(state_t *state);

// Broadcast the live (debounced) command-line buffer to viewers.
void cmd_broadcast_preview(state_t *state);

// Per-tick: fire the debounced preview broadcast when the draft has settled.
void cmd_pump(state_t *state);

// Set the one-line status shown beneath the prompt (printf-style). Also
// called from the tracking layer to report pointing/retarget outcomes.
void cmd_set_status(state_t *state, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // UI_CMD_LINE_H
