/*

   Simple Satellite Operations  ui/live_waterfall.h

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

// Manages the optional out-of-process raylib waterfall viewer
// (--live-waterfall): the child PID, the pipe to its stdin, and the
// spawn / reap / shutdown lifecycle. The operator UI owns one viewer at a
// time, pointed at the current recording's .iq file.

#ifndef UI_LIVE_WATERFALL_H
#define UI_LIVE_WATERFALL_H

#include <sys/types.h>  // pid_t

#ifdef __cplusplus
extern "C" {
#endif

struct rx_session;
typedef struct rx_session rx_session_t;

// Write end of the pipe whose read end is the viewer's stdin; -1 when no
// viewer is alive. The ":wf_*" colon commands write line-based commands here
// so a running viewer can be adjusted without a relaunch.
int live_waterfall_stdin_fd(void);

// Address of the child PID, handed to the crash handler so a device-loss
// abort that skips normal teardown can still reach the viewer.
pid_t *live_waterfall_pid_ref(void);

#ifdef SSO_WITH_SDR
// Once per ribbon tick: (re)launch the viewer when the recording's .iq path
// first appears or changes, and reap one the operator closed via its window.
void live_waterfall_poll(rx_session_t *rxs);
#endif

// Politely terminate the viewer at shutdown (SIGTERM, up to 5 s, then
// SIGKILL). No-op when none is running.
void live_waterfall_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // UI_LIVE_WATERFALL_H
