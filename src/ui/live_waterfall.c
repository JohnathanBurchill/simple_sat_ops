/*

   Simple Satellite Operations  ui/live_waterfall.c

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

#define _GNU_SOURCE

#include "live_waterfall.h"

#ifdef SSO_WITH_SDR
#include "rx_session.h"
#endif

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

// The single viewer's process state. pid/stdin_fd are referenced by the
// shutdown + crash paths even in a no-SDR build; the .iq path is only needed
// by the poll, which spawns the viewer, so it is gated with it.
static pid_t s_pid       = -1;
static int   s_stdin_fd  = -1;
#ifdef SSO_WITH_SDR
static char  s_iq[512]   = "";
#endif

int live_waterfall_stdin_fd(void) { return s_stdin_fd; }

pid_t *live_waterfall_pid_ref(void) { return &s_pid; }

#ifdef SSO_WITH_SDR
void live_waterfall_poll(rx_session_t *rxs)
{
    char iq_path[512] = "";
    int  iq_rate      = 0;
    rx_session_iq_snapshot(rxs,
                           iq_path, sizeof iq_path,
                           NULL, &iq_rate);
    if (iq_path[0]
        && strcmp(iq_path, s_iq) != 0) {
        // Tear down a viewer pointed at a stale path.
        if (s_pid > 0) {
            kill(s_pid, SIGTERM);
            waitpid(s_pid, NULL, 0);
            s_pid = -1;
        }
        if (s_stdin_fd >= 0) {
            close(s_stdin_fd);
            s_stdin_fd = -1;
        }
        snprintf(s_iq,
                 sizeof s_iq, "%s", iq_path);
        char rate_arg[32];
        snprintf(rate_arg, sizeof rate_arg,
                 "--rate=%d",
                 iq_rate > 0 ? iq_rate : 96000);
        // pipe()+dup2 so the parent can shove
        // line-based commands (e.g. "zoom 60\n") at the
        // viewer's stdin.
        int pfd[2] = {-1, -1};
        if (pipe(pfd) != 0) { pfd[0] = pfd[1] = -1; }
        pid_t pid = fork();
        if (pid == 0) {
            if (pfd[0] >= 0) {
                close(pfd[1]);
                dup2(pfd[0], STDIN_FILENO);
                close(pfd[0]);
            }
            char *args[] = {
                (char *) "live_waterfall",
                (char *) s_iq,
                rate_arg,
                NULL
            };
            execvp("live_waterfall", args);
            _exit(127);
        } else if (pid > 0) {
            s_pid = pid;
            if (pfd[0] >= 0) close(pfd[0]);
            s_stdin_fd = pfd[1];
        } else {
            if (pfd[0] >= 0) close(pfd[0]);
            if (pfd[1] >= 0) close(pfd[1]);
        }
    }
    // Reap a viewer that the operator closed via its
    // window — non-blocking so the main loop never stalls.
    if (s_pid > 0) {
        int status;
        pid_t r = waitpid(s_pid,
                          &status, WNOHANG);
        if (r == s_pid) {
            s_pid = -1;
            s_iq[0] = '\0';
            if (s_stdin_fd >= 0) {
                close(s_stdin_fd);
                s_stdin_fd = -1;
            }
        }
    }
}
#endif

void live_waterfall_shutdown(void)
{
    // Politely terminate the live raylib waterfall if we spawned one. 5 s
    // timeout via WNOHANG polling so the operator doesn't wait on a hung
    // viewer at shutdown.
    if (s_pid > 0) {
        kill(s_pid, SIGTERM);
        for (int t = 0; t < 50; ++t) {
            int status;
            pid_t r = waitpid(s_pid, &status, WNOHANG);
            if (r == s_pid) {
                s_pid = -1;
                break;
            }
            usleep(100000);
        }
        if (s_pid > 0) {
            kill(s_pid, SIGKILL);
            waitpid(s_pid, NULL, 0);
            s_pid = -1;
        }
    }
    if (s_stdin_fd >= 0) {
        close(s_stdin_fd);
        s_stdin_fd = -1;
    }
}
