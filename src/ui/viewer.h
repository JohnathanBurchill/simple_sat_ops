/*

   Simple Satellite Operations  ui/viewer.h

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

// Read-only viewer mode: connect to a running operator over the IPC socket,
// mirror its broadcast into a local state_t, and render the same panels the
// operator draws. Also offers the 'c'/'y' take-control hand-off. Entered when
// a bare invocation finds an operator already running.

#ifndef UI_VIEWER_H
#define UI_VIEWER_H

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Run the viewer event loop until the user quits or takes control (which
// re-execs as the operator). argv0 is the path to re-exec on take-control.
// Returns a process exit code.
int run_viewer(const char *argv0);

// Read the running operator's pid (from the IPC pidfile) into *out_pid.
// Returns 0 on success, non-zero if no operator pidfile is present. Used by
// the viewer take-control path and main's --control refusal message.
int read_operator_pid(pid_t *out_pid);

#ifdef __cplusplus
}
#endif

#endif // UI_VIEWER_H
