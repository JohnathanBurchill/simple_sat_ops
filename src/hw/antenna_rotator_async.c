/*

   Simple Satellite Operations  antenna_rotator_async.c

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

#include "antenna_rotator_async.h"
#include "antenna_rotator.h"

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct antenna_rotator_async {
    antenna_rotator_t *rot;            // borrowed
    pthread_t          thread;
    int                thread_started;
    int                stop_requested;
    pthread_mutex_t    mu;
    pthread_cond_t     cv;

    double             status_period_s;

    // Published snapshot. Worker writes under mu; readers (UI / track
    // loop) take a brief mu lock in snapshot(). The worker broadcasts cv
    // on every OK STATUS so wait_first_status / wait_next_good_status
    // wake up promptly.
    double snap_az;
    double snap_el;
    int    snap_ok;                    // last STATUS reply was OK
    double snap_last_good_mono_s;      // monotonic seconds of last good reply
    int    snap_set_in_flight;         // reserved; always 0 in this impl

    // Latest-wins SET slot. A second submit_set before the worker drains
    // overwrites the first; that's intentional (only the freshest target
    // matters).
    int    set_pending;
    double set_az;
    double set_el;

    // One-shots, drained each iteration.
    int    stop_cmd_pending;
    int    status_kick_pending;
};

static double now_mono_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double) ts.tv_sec + (double) ts.tv_nsec / 1e9;
}

// Build an absolute CLOCK_REALTIME deadline `delay_s` from now, suitable
// for pthread_cond_timedwait (which uses CLOCK_REALTIME by default).
static void abs_deadline(double delay_s, struct timespec *out)
{
    clock_gettime(CLOCK_REALTIME, out);
    if (delay_s < 0.0) delay_s = 0.0;
    long add_ns = (long)(delay_s * 1e9);
    out->tv_sec  += add_ns / 1000000000L;
    out->tv_nsec += add_ns % 1000000000L;
    // Normalize with while, not if: the canonical timespec carry that holds
    // regardless of how large the added nanoseconds are, so tv_nsec can never
    // be handed to pthread_cond_timedwait out of [0, 1e9) (EINVAL/busy-spin).
    while (out->tv_nsec >= 1000000000L) {
        out->tv_sec  += 1;
        out->tv_nsec -= 1000000000L;
    }
}

static void *rotator_worker(void *arg)
{
    antenna_rotator_async_t *ar = (antenna_rotator_async_t *) arg;
    double last_status_attempt = 0.0;

    for (;;) {
        // Wait for: stop, an incoming request, or the status period to
        // elapse. cond_timedwait is the only place this thread blocks
        // outside of antenna_rotator_command's serial read.
        pthread_mutex_lock(&ar->mu);
        for (;;) {
            if (ar->stop_requested) break;
            if (ar->stop_cmd_pending || ar->set_pending
                || ar->status_kick_pending) break;
            double remaining_s = ar->status_period_s
                                 - (now_mono_s() - last_status_attempt);
            if (remaining_s <= 0.0) break;
            struct timespec deadline;
            abs_deadline(remaining_s, &deadline);
            int rc = pthread_cond_timedwait(&ar->cv, &ar->mu, &deadline);
            if (rc == ETIMEDOUT) break;
        }
        if (ar->stop_requested) {
            pthread_mutex_unlock(&ar->mu);
            break;
        }

        int    do_stop  = ar->stop_cmd_pending;     ar->stop_cmd_pending    = 0;
        int    do_set   = ar->set_pending;
        double set_az   = ar->set_az;
        double set_el   = ar->set_el;
        ar->set_pending = 0;
        int    do_status = ar->status_kick_pending
                         || (now_mono_s() - last_status_attempt) >= ar->status_period_s;
        ar->status_kick_pending = 0;
        pthread_mutex_unlock(&ar->mu);

        // Off-lock serial I/O. STATUS can take ~500 ms on an unplugged
        // cable; we don't hold the mutex across it so the UI can still
        // read the (now-stale) snapshot without waiting.
        if (do_stop) {
            double az = 0.0, el = 0.0;
            (void) antenna_rotator_command(ar->rot,
                                           ANTENNA_ROTATOR_STOP, &az, &el);
        }
        if (do_set) {
            double az = set_az, el = set_el;
            (void) antenna_rotator_command(ar->rot,
                                           ANTENNA_ROTATOR_SET, &az, &el);
        }
        if (do_status) {
            double az = 0.0, el = 0.0;
            int rc = antenna_rotator_command(ar->rot,
                                             ANTENNA_ROTATOR_STATUS, &az, &el);
            double mono = now_mono_s();
            pthread_mutex_lock(&ar->mu);
            last_status_attempt = mono;
            if (rc == ANTENNA_ROTATOR_OK) {
                ar->snap_az = az;
                ar->snap_el = el;
                ar->snap_ok = 1;
                ar->snap_last_good_mono_s = mono;
                // Wake any wait_first_status / wait_next_good_status
                // callers; cheap when nobody's waiting.
                pthread_cond_broadcast(&ar->cv);
            } else {
                // Failed STATUS — keep the last good az/el for display, but
                // mark ok=0 and let snap_last_good_mono_s drift so stale_ms
                // grows. The UI renders "?" once it crosses its threshold.
                ar->snap_ok = 0;
            }
            pthread_mutex_unlock(&ar->mu);
        }
    }
    return NULL;
}

int antenna_rotator_async_open(antenna_rotator_async_t **out,
                                antenna_rotator_t *rot,
                                double status_period_s)
{
    if (out == NULL || rot == NULL) return -1;
    *out = NULL;

    antenna_rotator_async_t *ar = calloc(1, sizeof *ar);
    if (ar == NULL) return -1;
    ar->rot = rot;
    ar->status_period_s = (status_period_s > 0.0) ? status_period_s : 0.5;

    if (pthread_mutex_init(&ar->mu, NULL) != 0) {
        free(ar);
        return -1;
    }
    if (pthread_cond_init(&ar->cv, NULL) != 0) {
        pthread_mutex_destroy(&ar->mu);
        free(ar);
        return -1;
    }
    // Kick a first STATUS immediately so wait_first_status doesn't have
    // to wait a whole status_period.
    ar->status_kick_pending = 1;

    if (pthread_create(&ar->thread, NULL, rotator_worker, ar) != 0) {
        pthread_cond_destroy(&ar->cv);
        pthread_mutex_destroy(&ar->mu);
        free(ar);
        return -1;
    }
    ar->thread_started = 1;
    *out = ar;
    return 0;
}

void antenna_rotator_async_close(antenna_rotator_async_t *ar)
{
    if (ar == NULL) return;
    if (ar->thread_started) {
        pthread_mutex_lock(&ar->mu);
        ar->stop_requested = 1;
        pthread_cond_broadcast(&ar->cv);
        pthread_mutex_unlock(&ar->mu);
        pthread_join(ar->thread, NULL);
        ar->thread_started = 0;
    }
    pthread_cond_destroy(&ar->cv);
    pthread_mutex_destroy(&ar->mu);
    free(ar);
}

void antenna_rotator_async_submit_set(antenna_rotator_async_t *ar,
                                      double az_unwrapped, double elevation)
{
    if (ar == NULL) return;
    pthread_mutex_lock(&ar->mu);
    ar->set_pending = 1;
    ar->set_az = az_unwrapped;
    ar->set_el = elevation;
    pthread_cond_broadcast(&ar->cv);
    pthread_mutex_unlock(&ar->mu);
}

void antenna_rotator_async_submit_stop(antenna_rotator_async_t *ar)
{
    if (ar == NULL) return;
    pthread_mutex_lock(&ar->mu);
    ar->stop_cmd_pending = 1;
    pthread_cond_broadcast(&ar->cv);
    pthread_mutex_unlock(&ar->mu);
}

void antenna_rotator_async_kick_status(antenna_rotator_async_t *ar)
{
    if (ar == NULL) return;
    pthread_mutex_lock(&ar->mu);
    ar->status_kick_pending = 1;
    pthread_cond_broadcast(&ar->cv);
    pthread_mutex_unlock(&ar->mu);
}

void antenna_rotator_async_snapshot(const antenna_rotator_async_t *ar,
                                    double *out_az, double *out_el,
                                    int *out_ok, int *out_stale_ms,
                                    int *out_set_in_flight)
{
    if (ar == NULL) {
        if (out_az) *out_az = 0.0;
        if (out_el) *out_el = 0.0;
        if (out_ok) *out_ok = 0;
        if (out_stale_ms) *out_stale_ms = INT_MAX;
        if (out_set_in_flight) *out_set_in_flight = 0;
        return;
    }
    // Cast away const for the lock — snapshot is logically read-only,
    // but the mutex is a runtime resource. Standard pattern.
    pthread_mutex_t *mu = (pthread_mutex_t *) &ar->mu;
    pthread_mutex_lock(mu);
    if (out_az) *out_az = ar->snap_az;
    if (out_el) *out_el = ar->snap_el;
    if (out_ok) *out_ok = ar->snap_ok;
    if (out_stale_ms) {
        if (ar->snap_last_good_mono_s > 0.0) {
            double dt = now_mono_s() - ar->snap_last_good_mono_s;
            if (dt < 0.0) dt = 0.0;
            // Cap at ~24 days so we don't overflow the int return.
            if (dt > 2.0e6) dt = 2.0e6;
            *out_stale_ms = (int)(dt * 1000.0);
        } else {
            *out_stale_ms = INT_MAX;
        }
    }
    if (out_set_in_flight) *out_set_in_flight = ar->snap_set_in_flight;
    pthread_mutex_unlock(mu);
}

int antenna_rotator_async_wait_first_status(antenna_rotator_async_t *ar,
                                             int timeout_ms)
{
    if (ar == NULL) return -1;
    if (timeout_ms < 0) timeout_ms = 0;
    pthread_mutex_lock(&ar->mu);
    struct timespec deadline;
    abs_deadline((double) timeout_ms / 1000.0, &deadline);
    while (ar->snap_last_good_mono_s == 0.0 && !ar->stop_requested) {
        int rc = pthread_cond_timedwait(&ar->cv, &ar->mu, &deadline);
        if (rc == ETIMEDOUT) break;
    }
    int ok = (ar->snap_last_good_mono_s > 0.0 && !ar->stop_requested) ? 0 : -1;
    pthread_mutex_unlock(&ar->mu);
    return ok;
}

int antenna_rotator_async_wait_next_good_status(antenna_rotator_async_t *ar,
                                                 int timeout_ms)
{
    if (ar == NULL) return -1;
    if (timeout_ms < 0) timeout_ms = 0;
    pthread_mutex_lock(&ar->mu);
    double t_entry = ar->snap_last_good_mono_s;
    struct timespec deadline;
    abs_deadline((double) timeout_ms / 1000.0, &deadline);
    while (ar->snap_last_good_mono_s <= t_entry && !ar->stop_requested) {
        int rc = pthread_cond_timedwait(&ar->cv, &ar->mu, &deadline);
        if (rc == ETIMEDOUT) break;
    }
    int ok = (ar->snap_last_good_mono_s > t_entry
              && !ar->stop_requested) ? 0 : -1;
    pthread_mutex_unlock(&ar->mu);
    return ok;
}
