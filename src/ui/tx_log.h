/*

   Simple Satellite Operations  ui/tx_log.h

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

// TX event log: append each PREVIEW / SENT / NOT_SENT event to the in-memory
// ring on state_t (rendered by render_tx_log_panel) and to the persistent
// per-pass JSONL log. Shared by the operator (commit + emit paths) and the
// viewer (mirroring the broadcast).

#ifndef UI_TX_LOG_H
#define UI_TX_LOG_H

#include "sso_ipc.h"     // sso_event_t

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Record a TX event in state's ring buffer (collapsing a SENT/NOT_SENT onto
// the preceding PREVIEW) and append it to the on-disk JSONL log.
void tx_log_push(state_t *state, const sso_event_t *evt);

#ifdef __cplusplus
}
#endif

#endif // UI_TX_LOG_H
