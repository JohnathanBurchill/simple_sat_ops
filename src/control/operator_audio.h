/*

   Simple Satellite Operations  control/operator_audio.h

   Operator side of the live-audio relay (§9 of docs/VIEWER_STREAM_JSON.md).
   Holds a per-subscriber Ogg/Vorbis encoder: each viewer that sends an
   audio-ctl enable gets its own encoder fed from the RX session's PCM tap,
   so it receives a complete, self-contained stream from the headers on.

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

#ifndef CONTROL_OPERATOR_AUDIO_H
#define CONTROL_OPERATOR_AUDIO_H

#include "sso_ipc.h"   // sso_client_id_t, sso_event_t

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Cap on simultaneous audio subscribers. Bounds the per-tick encode cost
// on the single-threaded operator (each subscriber is one Vorbis encode).
#define SSO_AUDIO_MAX_SUBS 4

// Handle an inbound audio-ctl event from a viewer/relay (called from the
// operator's IPC on-event callback). enable=1 (re)creates that client's
// encoder and starts the RX tap; enable=0 tears it down. Always answers
// the requesting client with an audio-status event.
void operator_audio_handle_ctl(state_t *state, sso_client_id_t id,
                               const sso_event_t *evt);

// Main-loop pump: drain the RX audio ring and send each subscriber a chunk
// of its Ogg/Vorbis stream. Cheap no-op when there are no subscribers.
void operator_audio_pump(state_t *state);

// 1 if at least one viewer is listening live. The main loop ticks faster
// while this is true so audio flows with low latency.
int  operator_audio_active(void);

// Drop subscribers whose client has disconnected (mark-and-sweep against
// the live client list) and clear the RX tap when the last one leaves.
// Call periodically alongside the IPC service step.
void operator_audio_prune(state_t *state);

// Free all encoders and clear the RX tap. Call on operator shutdown.
void operator_audio_shutdown(state_t *state);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_OPERATOR_AUDIO_H
