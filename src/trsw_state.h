/*

   Simple Satellite Operations  src/trsw_state.h

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

// T/R antenna switch slice of state_t — see state.h for the composition.

#ifndef TRSW_STATE_H
#define TRSW_STATE_H

#include "tr_switch.h"
#include "tr_switch_find.h"

// T/R antenna switch (USB-CDC).
// run_with_tr_switch defaults to 1: look for the device on start.
// With no --tr-switch-device= the port is worked out at startup (see
// tr_switch_find.h) and its path kept in device_path, which is what the
// driver's device_filename then points at. Absent hardware is a one-line
// warning, not an error.
typedef struct trsw {
    tr_switch_t tr_switch;
    char device_path[TR_SWITCH_PATH_MAX];
    int run_with_tr_switch;
    int have_tr_switch;
} trsw_t;

#endif // TRSW_STATE_H
