/*

   Simple Satellite Operations  ui/keybindings.h

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

// One table is the single source of truth for the operator's keyboard:
// which keys do what (dispatch), the on-screen legend (ui/panels.c), and
// the --help-full listing (apps/cli_args.c). Adding a key in one place used
// to mean updating three -- and the "Track" legend line and the t / A help
// lines had already drifted out of sync. Now every consumer reads this table.
//
// The action bodies live in ui/input.c (they call into tracking / scan /
// the TX modals); this header only declares them so the table can point at
// them. Keeping the table itself free of those dependencies lets the
// keybindings selftest link it with stub actions.

#ifndef UI_KEYBINDINGS_H
#define UI_KEYBINDINGS_H

#include <stddef.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Row flags.
#define KB_ALWAYS 0x01u   // honoured even while the keyboard is locked (K)
#define KB_HIDDEN 0x02u   // not shown in the legend or --help-full (the ":" command line)

typedef struct keybinding {
    // Every getch() character this row handles. dispatch scans it with
    // strchr, so a row can claim a pair -- e.g. "[]" for the az -5/+5 jog,
    // whose single action reads `key` to pick the direction.
    const char *keys;
    // Legend / help left column, e.g. "[/]".
    const char *combo;
    // Legend / help right column, e.g. "Jog azimuth -5/+5 deg".
    const char *desc;
    // Invoked with the state and the actual key pressed.
    void      (*action)(state_t *state, int key);
    unsigned    flags;
} keybinding_t;

// The table and its length. Callers iterate it to render the legend.
const keybinding_t *keybindings_table(size_t *out_n);

// Route one key to its action. Keys without KB_ALWAYS only fire while
// state->ui.keyboard_unlocked is set. Unbound keys (and ERR / NUL) are
// ignored. This is the unlocked-operator-keys branch the router delegates to.
void keybindings_dispatch(state_t *state, int key);

// Number of rows shown in the legend (i.e. not KB_HIDDEN) -- panels uses it
// to know how tall the legend block is before it paints.
int keybindings_visible_count(void);

// Print the KEYBOARD section of --help-full to fp, one line per visible row.
void keybindings_print_help(FILE *fp);

// Action bodies, defined in ui/input.c. Declared here so the table can
// reference them; the selftest provides its own stub definitions.
void kb_act_track(state_t *state, int key);
void kb_act_stop(state_t *state, int key);
void kb_act_reset(state_t *state, int key);
void kb_act_jog(state_t *state, int key);
void kb_act_tx_compose(state_t *state, int key);
void kb_act_auto_tcmd(state_t *state, int key);
void kb_act_lock(state_t *state, int key);
void kb_act_quit(state_t *state, int key);
void kb_act_cmdline(state_t *state, int key);

#ifdef __cplusplus
}
#endif

#endif // UI_KEYBINDINGS_H
