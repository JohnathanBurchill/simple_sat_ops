/*

   Simple Satellite Operations  control/operator_ipc.h

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

// Operator-side IPC glue: pack the live tracker/rotator/RX/auto-tcmd state
// into a STATE broadcast for viewers, and handle inbound viewer events
// (HELLO -> WELCOME seed, take-control hand-off).

#ifndef CONTROL_OPERATOR_IPC_H
#define CONTROL_OPERATOR_IPC_H

#include "sso_ipc.h"     // sso_ipc_server_t, sso_client_id_t, sso_event_t

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Broadcast a STATE event with the current pointing + Doppler-shifted
// downlink + RX-panel + auto-tcmd snapshot to all connected viewers.
void ipc_broadcast_state(state_t *s, double az, double el,
                         double downlink_freq, double doppler_delta_dl,
                         double jul_utc);

// IPC server event callback (registered with sso_ipc_server_on_event); the
// state_t is passed through the user pointer.
void ipc_on_event(sso_ipc_server_t *srv, sso_client_id_t id,
                  const sso_event_t *evt, void *user);

// Startup: audit-log the session + command line, and (in --control mode)
// bind the operator IPC socket, register ipc_on_event, and install the
// take-control yield handler. Returns 0 on success; EXIT_FAILURE if another
// operator is already running or the socket bind fails.
int ipc_operator_startup(state_t *state, int argc, char **argv);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_OPERATOR_IPC_H
