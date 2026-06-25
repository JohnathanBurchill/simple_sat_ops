/*

   Simple Satellite Operations  ui/input.h

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

// Top-level keyboard routing for the operator UI: read one key and dispatch
// it to whichever consumer is active -- the TX-compose modal, the auto-tcmd
// modal, the ":" command line, or (when none is up) the keybindings table,
// which owns the keyboard lock and the operator keys. The modals and the
// command line own their own key handling; this is just the router that
// decides who gets the keystroke.

#ifndef UI_INPUT_H
#define UI_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Read one key (getch) and act on it. The lock state lives on
// state->ui.keyboard_unlocked; 'K' flips it (via the keybindings table) and
// the other operator keys are only honoured while it is set.
void input_handle_keys(state_t *state);

#ifdef __cplusplus
}
#endif

#endif // UI_INPUT_H
