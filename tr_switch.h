/*

   Simple Satellite Operations  tr_switch.h

   Host-side driver for the CTS UHF RX/TX antenna switch
   (CalgaryToSpace/CTS-Ground-Station-UHF-Antenna-Switch, RP2040-Zero
   firmware speaking USB-CDC over /dev/ttyACM0).

   The firmware boots silent at log level 0. After open we send '1'
   to enable LOG_INFO heartbeats (cadence ~2500 ms) and 'a' to put
   the switch in serial-override AUTO so its TX detector — rather than
   the physical slide switch — drives the K1/K2 relays. From then on
   tr_switch_pump() drains complete '\n'-terminated lines, parses any
   line that begins with "Heartbeat: ", and stashes the latest
   state/mode/last_tx_ago_s.

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

#ifndef TR_SWITCH_H
#define TR_SWITCH_H

#include <termios.h>
#include <stdint.h>
#include <stddef.h>

#define TR_SWITCH_RX_BUF_BYTES 256
#define TR_SWITCH_STATE_MAX     8     // "TX" / "RX"
#define TR_SWITCH_MODE_MAX     16     // "AUTO" / "FORCE_TX" / "FORCE_RX" / "INVALID"

// Firmware heartbeat cadence is 2500 ms. Treat the link as stale once
// we go 3x that without a fresh line.
#define TR_SWITCH_HEARTBEAT_PERIOD_S  2.5
#define TR_SWITCH_STALE_AFTER_S       (3.0 * TR_SWITCH_HEARTBEAT_PERIOD_S)

typedef struct tr_switch {
    // Caller fills these before tr_switch_init().
    const char *device_filename;     // e.g. "/dev/ttyACM0"
    speed_t     serial_speed;        // B115200 — USB-CDC ignores it,
                                     //          but termios still needs a value

    // Set by tr_switch_init() / tr_switch_disconnect().
    int            fd;
    int            connected;
    struct termios tty;

    // Partial-line assembly buffer for the non-blocking reader.
    char   rx_buf[TR_SWITCH_RX_BUF_BYTES];
    size_t rx_len;

    // Latest parsed heartbeat fields. Empty strings until the first
    // heartbeat lands.
    char   state_str[TR_SWITCH_STATE_MAX];
    char   mode_str [TR_SWITCH_MODE_MAX];
    // last_tx_ago_s is from the (future) firmware extension; until
    // that lands the field is absent in the line and this stays NAN.
    // The literal "never" maps to +infinity so the UI can format it
    // as "—" without colliding with a real, very-large age.
    double last_tx_ago_s;
    // Monotonic seconds, set on every line received and on every
    // successful parse. The UI compares against the current monotonic
    // time to detect a stale link.
    double t_last_byte;
    double t_last_heartbeat;
    // Count of heartbeat lines successfully parsed since open. Useful
    // for the panel ("seen N") and for tests.
    unsigned long heartbeat_count;
} tr_switch_t;

// Open the device, drop into raw 8N1, send '1' + 'a'. Returns 0 on
// success, nonzero on failure. Failure is non-fatal — the caller
// should leave the switch as "not connected" and keep running.
int  tr_switch_init        (tr_switch_t *s);
void tr_switch_disconnect  (tr_switch_t *s);

// Non-blocking read pump. Drains whatever bytes are available, runs
// any complete lines through the heartbeat parser, and updates the
// status fields. Cheap to call every tick.
void tr_switch_pump        (tr_switch_t *s, double t_now);

// Operator commands. Each writes a single character; the firmware
// parser is single-character with no terminator.
//   level: 0..3
//   serial_mode: 't' / 'r' / 'a' / 's' (force TX / RX / AUTO / clear)
int  tr_switch_set_log_level   (tr_switch_t *s, int level);
int  tr_switch_set_serial_mode (tr_switch_t *s, char serial_mode);

// True if no heartbeat has landed within TR_SWITCH_STALE_AFTER_S.
// Returns 0 when not connected (so callers can treat the link as
// "no data, but not specifically stale").
int  tr_switch_is_stale    (const tr_switch_t *s, double t_now);

#endif // TR_SWITCH_H
