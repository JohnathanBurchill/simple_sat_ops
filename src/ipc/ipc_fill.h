/*

   Simple Satellite Operations  ipc/ipc_fill.h

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

// Map the SGP4 prediction snapshot into a STATE event. Pulled out of the
// operator broadcast so the same fields are filled the same way by both
// the operator and the headless --viewer-stream relay (which propagates
// the TLE itself with no operator). Pure data copy, no ncurses, no
// hardware — so it can be unit-tested on its own.

#ifndef IPC_FILL_H
#define IPC_FILL_H

#include "prediction.h"   // prediction_t
#include "sso_ipc.h"      // sso_event_t

#ifdef __cplusplus
extern "C" {
#endif

// Copy the satellite name, idesg, and every prediction-derived number
// (epoch, pass timing, sky az/el, sub-point, range / range-rate) from
// `p` into the matching STATE fields of `evt`. Does not touch the
// hardware fields (rotator az/el, RX panel) or the source tag — the
// caller fills those for its mode. Safe with NULL args.
void ipc_fill_state_prediction(const prediction_t *p, sso_event_t *evt);

#ifdef __cplusplus
}
#endif

#endif // IPC_FILL_H
