/*

   Simple Satellite Operations  antenna_rotator_async.h

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

#ifndef ANTENNA_ROTATOR_ASYNC_H
#define ANTENNA_ROTATOR_ASYNC_H

#include "antenna_rotator.h"

// Worker-thread wrapper around the wire-level antenna_rotator_command()
// API. The worker owns the serial FD: it polls STATUS at status_period_s,
// issues SET / STOP commands posted by the main thread, and publishes the
// latest snapshot for the UI to read non-blockingly. This takes the per-
// tick 5-10 ms serial roundtrip (and the 500 ms VTIME hang on a missing
// cable) off the ncurses main loop.
//
// Modeled on src/pipeline/rx_session.c (submit / poll, pthread_cond_timed
// wait, latest-wins handoff). The wire-level antenna_rotator.{h,c} stays
// unchanged; only the call site moves to the worker thread.

typedef struct antenna_rotator_async antenna_rotator_async_t;

// Spawn the worker. `rot` is borrowed — the caller continues to own its
// lifetime, and the worker only touches `rot->fd` / `rot->connected` (it
// never writes to the target / wrap fields, which stay single-threaded on
// the main thread). status_period_s is the STATUS poll cadence; 0.5 is a
// reasonable default. Returns 0 on success, -1 on error.
int  antenna_rotator_async_open(antenna_rotator_async_t **out,
                                antenna_rotator_t *rot,
                                double status_period_s);

// Stop the worker, broadcast, join, free. Safe with NULL.
void antenna_rotator_async_close(antenna_rotator_async_t *ar);

// Latest-wins SET. Always accepts; a newer submission supersedes any
// still-queued older one. The worker emits the wire-level SET on its next
// iteration.
void antenna_rotator_async_submit_set(antenna_rotator_async_t *ar,
                                      double az_unwrapped, double elevation);

// Idempotent STOP. Drained on the worker's next iteration.
void antenna_rotator_async_submit_stop(antenna_rotator_async_t *ar);

// Force the worker to issue a STATUS read on its next iteration without
// waiting for the status period to elapse. Useful after a STOP to freshen
// the snapshot.
void antenna_rotator_async_kick_status(antenna_rotator_async_t *ar);

// Snapshot read for the UI. Any out-pointer may be NULL.
//
// out_ok           — 1 if the most recent STATUS attempt succeeded, 0 if
//                    it failed or we've never had one.
// out_stale_ms     — ms since the last GOOD STATUS reply. INT_MAX if we
//                    have never had one. The panel renders "?" when this
//                    crosses the operator's tolerance (e.g. 1500 ms).
// out_set_in_flight — reserved for a future "antenna_is_moving" hint;
//                     always 0 in this implementation.
void antenna_rotator_async_snapshot(const antenna_rotator_async_t *ar,
                                    double *out_az, double *out_el,
                                    int *out_ok, int *out_stale_ms,
                                    int *out_set_in_flight);

// Bounded blocking wait for the first OK STATUS reply. Returns 0 on
// success (snapshot is populated), -1 on timeout or close-during-wait.
// Used by startup seed-from-status in place of the synchronous wire-level
// seed.
int  antenna_rotator_async_wait_first_status(antenna_rotator_async_t *ar,
                                              int timeout_ms);

// Bounded blocking wait for the NEXT OK STATUS reply (one that lands
// after this call entered). Used after submit_stop + kick_status to
// refresh target_* with the position the antenna actually stopped at,
// not where the satellite was. Returns 0 on success, -1 on timeout.
int  antenna_rotator_async_wait_next_good_status(antenna_rotator_async_t *ar,
                                                  int timeout_ms);

#endif // ANTENNA_ROTATOR_ASYNC_H
