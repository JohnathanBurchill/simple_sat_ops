/*

   Simple Satellite Operations  src/op_state.h

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

// Operator slice of state_t: the IPC fan-out server + Unix user, the last
// broadcast snapshot, the pass output folder, and the low-disk warning.
// See state.h.

#ifndef OP_STATE_H
#define OP_STATE_H

#include "sso_ipc.h"    // sso_ipc_server_t

// Latest broadcast snapshot, kept so a newly-connecting viewer gets state
// in its WELCOME response without waiting up to 500 ms for the next
// periodic STATE broadcast. ipc_broadcast_state fills it after each send;
// ipc_on_event reads it to seed a WELCOME.
typedef struct last_state {
    int    valid;
    char   sat[64];
    double az;
    double el;
    long   freq_hz;
    double doppler;
    char   tle[256];
    double tgt_az;
    double tgt_el;
    int    flip;
    int    in_pass;
    int    tracking;
    int    has_rot;
    double jul;
    char   idesg[9];
    double epoch_min;
    double min_visible;
    double min_above_0;
    double min_above_30;
    double max_el;
    double pred_az;
    double pred_el;
    double alt_km;
    double lat_deg;
    double lon_deg;
    double speed_kms;
    double range_km;
    double rrate_kms;
} last_state_t;

typedef struct op {
    const char       *operator_user;   // Unix user running the operator
    sso_ipc_server_t *ipc;             // IPC fan-out server (operator side)
    last_state_t      last_state;      // last broadcast snapshot (WELCOME seed)
    char   pass_folder[256];           // this pass's output folder; "" until set
    char   low_disk_msg[80];           // non-empty -> low-disk warning to show
    double low_disk_last_t;            // last low-disk probe (monotonic s)
} op_t;

#endif // OP_STATE_H
