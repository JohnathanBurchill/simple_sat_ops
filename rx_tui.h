/*

    Simple Satellite Operations  rx_tui.h

    Optional ncurses display shared by rx_live and rx_replay. Default
    is the existing streaming-text mode; --ui on either tool switches
    over to a panelled view: latest beacon (broken out by subsystem
    when there's screen room), scrolling list of recent telecommand
    responses, frame counters, and an "age since last frame" status.

    File logging continues unchanged in --ui mode; the TUI is a
    glanceable view, not a replacement for the .log companion.

    The implementation is gated on RX_TUI_AVAILABLE (set by CMake when
    pkg-config finds ncurses). When that's not defined, all functions
    here are no-op stubs and rx_tui_init returns -1 so the caller can
    print a "ncurses not built in" message and exit.

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

#ifndef RX_TUI_H
#define RX_TUI_H

#include <stddef.h>
#include <stdint.h>

// Take over the terminal. Returns 0 on success, -1 if ncurses wasn't
// compiled in, -2 if initscr/colour setup failed. Safe to call once
// per process; subsequent calls return 0 without re-initialising.
int rx_tui_init(void);

// Restore the terminal. Safe to call when init failed or wasn't called.
void rx_tui_close(void);

// Set the header text (top reverse-video bar). Typical content: program
// name, audio device / file path, decoder params. Buffer is copied;
// caller may free immediately. Pass NULL to clear.
void rx_tui_set_header(const char *header);

// Set a one-line status row rendered directly under the title bar in
// the same reverse-video treatment. When the buffer is empty (NULL or
// ""), the row is hidden and the main panels reclaim the space — so
// callers that don't need it (rx_live, rx_replay) are unaffected.
// Used by b210_rx_live to keep the live Doppler-tracked frequency in
// a stable on-screen location across the run.
void rx_tui_set_status(const char *status);

// Feed a successfully-decoded frame. The TUI sniffs the payload itself
// to pick the right updater (beacon vs tcmd response vs other). ts is
// whatever timestamp string emit_frame would receive.
void rx_tui_observe_frame(const char *ts,
                          const uint8_t *packet, size_t packet_len,
                          int golay_errs, int hmac_ok, int use_hmac,
                          int rs_errs,
                          int crc_status);

// Push one signal-presence sample (called every audio chunk, typically
// 10..50 Hz). ratio_db is monitor_squelch's instantaneous signal/noise
// ratio; thresh_db is the gate-open threshold for visual context;
// gate_open is non-zero when the squelch is currently passing audio.
// First call switches on the activity ribbon at the bottom of the TUI;
// receivers that don't track signal (rx_replay, plain rx_live) never
// call this and the ribbon stays hidden.
void rx_tui_observe_signal(double ratio_db, double thresh_db, int gate_open);

// Pump input (q/Q -> quit requested, KEY_RESIZE -> redraw) and refresh
// the display. Returns 1 if quit was requested, 0 otherwise. Cheap
// enough to call every audio chunk.
int rx_tui_tick(void);

// rx_replay convenience: after the file is exhausted, block until the
// user presses q so they can read the final state. Refreshes once a
// second so "last RX age" keeps ticking. No-op when ncurses absent.
void rx_tui_hold_until_quit(void);

// Set the quit flag from outside the TUI. Async-signal-safe (a single
// volatile sig_atomic_t store), so callers can wire it into their
// SIGINT/SIGTERM handler so Ctrl-C escapes rx_tui_hold_until_quit
// without the user having to reach for q.
void rx_tui_request_quit(void);

// Optional in-TUI REPL. When a handler is registered, rx_tui paints a
// `> ` input row above the footer, routes keystrokes into a line
// editor (cursor keys, Home/End, Backspace, history Up/Down, Esc to
// clear), and calls fn(line, ctx) on Enter. While the REPL is active
// the q-as-quit shortcut is disabled — Ctrl-C and the literal `quit`
// command are the exits — so plain rx_live / rx_replay (which never
// register a handler) keep their existing behaviour. Pass fn=NULL to
// drop the REPL again.
typedef void (*rx_tui_command_fn)(const char *cmd, void *ctx);
void rx_tui_set_command_handler(rx_tui_command_fn fn, void *ctx);

// Persist accepted commands to <path> (one per line, append-only).
// Existing contents are loaded into the in-memory ring at the first
// call. Pass NULL to disable file persistence; the in-memory ring
// (most recent ~64 entries) keeps working either way.
void rx_tui_set_history_path(const char *path);

#endif // RX_TUI_H
