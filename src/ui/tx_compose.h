/*

   Simple Satellite Operations  ui/tx_compose.h

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

// The interactive TX-compose modal ('t'): edit a telecommand payload, power
// and the allow-tx gate, broadcast a debounced preview to viewers, and on
// commit stage a burst into state->tx_request for the main loop to transmit.

#ifndef UI_TX_COMPOSE_H
#define UI_TX_COMPOSE_H

#include "sso_ipc.h"     // sso_event_type_t

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Open / close the modal (allocate + draw / tear down the window).
void tx_compose_open(state_t *state);
void tx_compose_close(state_t *state);

// Feed a key to the open modal. Returns 0 when the modal consumed the key
// (including closing on Esc/Enter), non-zero to let the caller handle it.
int tx_compose_handle_key(state_t *state, int key);

// Per-tick: emit the debounced preview broadcast when the draft has settled.
void tx_compose_pump(state_t *state);

// Push a locally-generated TX event (SENT / NOT_SENT) into the log + IPC
// fan-out, mirroring what the burst worker produces.
void emit_tx_event_local(state_t *state, sso_event_type_t type,
                         const char *summary, const char *ack_status);

#ifdef __cplusplus
}
#endif

#endif // UI_TX_COMPOSE_H
