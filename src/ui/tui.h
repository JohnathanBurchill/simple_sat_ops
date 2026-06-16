/*

   Simple Satellite Operations  ui/tui.h

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

// Terminal lifecycle for the operator/viewer TUI: ncurses init, the
// stderr-to-logfile redirect that keeps library noise off the panels, and
// the crash / quit signal safety net that restores the terminal when an SDR
// is yanked off USB mid-stream.

#ifndef UI_TUI_H
#define UI_TUI_H

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Bring up ncurses (alt screen, colours, raw input) and divert stderr to the
// pass-folder log so backend/library errors never paint over the panels.
void init_window(state_t *state);

// Install the SIGABRT/SIGSEGV/SIGBUS crash handler (restores the terminal and
// re-raises) and the SIGINT/SIGTERM graceful-quit handler.
void install_signal_handlers(void);

// Arm the SIGUSR1 force-takeover handler. Operator-only — call after a
// successful IPC bind so tui_yield_requested() reports the takeover.
void tui_install_yield_handler(void);

// Restore stderr to the real terminal; call on teardown. Idempotent.
void tui_release_stderr(void);

// One-line closing status: did anything hit the redirected stderr log this
// run? Call last, after the TUI is down and stderr restored.
void tui_report_errors(void);

// Capture the cooked terminal modes BEFORE ncurses switches the tty to raw,
// so the crash handler can put them back deterministically.
void tui_save_termios(void);

// 1 once SIGINT/SIGTERM has been received — the main loop runs its normal
// teardown instead of dying raw.
int tui_should_quit(void);

// 1 once SIGUSR1 (force-takeover) has been received.
int tui_yield_requested(void);

// Register the live-waterfall child pid (by pointer, owned by the caller) so
// the crash handler can SIGKILL it before re-raising. Pass NULL to clear.
void tui_register_waterfall_pid(pid_t *pidp);

#ifdef __cplusplus
}
#endif

#endif // UI_TUI_H
