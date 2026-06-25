/*

   Simple Satellite Operations  ui/keybindings.c

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

#include "keybindings.h"
#include "state.h"

#include <string.h>

// The single source of truth. Order is the legend order. The jog rows each
// claim a key pair and share one action that reads `key` for the direction.
static const keybinding_t KEYBINDINGS[] = {
    { "T",  "T",   "Track satellite",              kb_act_track,      0 },
    { "s",  "s",   "Stop antenna immediately",     kb_act_stop,       0 },
    { "r",  "r",   "Reset to az=0 el=0",           kb_act_reset,      0 },
    { "[]", "[/]", "Jog azimuth -5/+5 deg",        kb_act_jog,        0 },
    { "{}", "{/}", "Jog azimuth -1/+1 deg",        kb_act_jog,        0 },
    { ",.", ",/.", "Jog elevation -5/+5 deg",      kb_act_jog,        0 },
    { "<>", "</>", "Jog elevation -1/+1 deg",      kb_act_jog,        0 },
    { "t",  "t",   "Compose TX command",           kb_act_tx_compose, 0 },
    { "A",  "A",   "Auto-TCMD (needs --tc-file=)", kb_act_auto_tcmd,  0 },
    { "K",  "K",   "Lock/unlock keyboard",         kb_act_lock,       KB_ALWAYS },
    { "q",  "q",   "Quit",                         kb_act_quit,       0 },
    { ":",  ":",   "Command line",                 kb_act_cmdline,    KB_HIDDEN },
};

static const size_t KEYBINDINGS_N = sizeof KEYBINDINGS / sizeof KEYBINDINGS[0];

const keybinding_t *keybindings_table(size_t *out_n)
{
    if (out_n) *out_n = KEYBINDINGS_N;
    return KEYBINDINGS;
}

void keybindings_dispatch(state_t *state, int key)
{
    // getch returns ERR (-1) when nothing is waiting; NUL can't be typed.
    // Guard both so strchr doesn't match a row's terminating '\0'. Special
    // keys (KEY_LEFT etc., > 255) simply won't match any printable row.
    if (key <= 0) return;
    for (size_t i = 0; i < KEYBINDINGS_N; ++i) {
        const keybinding_t *kb = &KEYBINDINGS[i];
        if (kb->keys && strchr(kb->keys, key)) {
            if ((kb->flags & KB_ALWAYS) || state->ui.keyboard_unlocked) {
                if (kb->action) kb->action(state, key);
            }
            return;
        }
    }
}

int keybindings_visible_count(void)
{
    int n = 0;
    for (size_t i = 0; i < KEYBINDINGS_N; ++i) {
        if (!(KEYBINDINGS[i].flags & KB_HIDDEN)) ++n;
    }
    return n;
}

void keybindings_print_help(FILE *fp)
{
    if (!fp) return;
    fprintf(fp,
            "\n"
            "KEYBOARD (unlocked by default, press K to toggle lock state)\n"
            "\n");
    for (size_t i = 0; i < KEYBINDINGS_N; ++i) {
        const keybinding_t *kb = &KEYBINDINGS[i];
        if (kb->flags & KB_HIDDEN) continue;
        fprintf(fp, "  %-5s  %s\n", kb->combo, kb->desc);
    }
}
