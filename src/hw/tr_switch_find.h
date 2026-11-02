/*

   Simple Satellite Operations  src/hw/tr_switch_find.h

   Work out which serial port the CTS UHF T/R antenna switch is on, so
   the operator doesn't have to name it, and remember the answer so the
   next run doesn't have to work it out again.

   The switch is an RP2040-Zero speaking USB-CDC, and the kernel hands
   those out in plug order: it is /dev/ttyACM0 on a machine with nothing
   else of the kind attached and /dev/ttyACM1 the day someone leaves a
   dev board plugged in. So the ports are found and then told apart.

   Finding them: /sys/class/tty/ttyACM* on Linux, /dev/cu.usbmodem* on
   macOS. Nothing outside that list is ever opened -- the rotator's own
   /dev/ttyUSB0 is not a USB-CDC port and cannot be reached from here.
   Linux hangs the USB descriptors off /sys/class/tty/<port>/device/..,
   so vendor, product, manufacturer and serial come back with each port;
   macOS publishes nothing beside the node, so its ports arrive bare.

   Telling them apart, the first time: the port is opened the way it is
   opened for real -- log level 1, serial AUTO -- and given
   TR_SWITCH_PROBE_SECONDS to produce a heartbeat. One heartbeat is the
   switch's own signature and nothing else on the bus emits it. A port
   that stays quiet is handed back and the next one tried.

   Telling them apart, every time after: what answered is written to
   ~/.local/state/simple_sat_ops/tr_switch.state, and the next run looks
   for that board first. This is deliberately learned rather than baked
   in. The firmware leaves the Pico SDK's stock USB descriptors in place,
   so its vendor and product strings would fit any RP2040 board on the
   machine; the serial is the one field that identifies this board, and
   no source file can know it in advance. A remembered serial that turns
   up in the scan is the switch and is opened straight away; a remembered
   port with no serial to confirm it (which is every port on macOS) is
   merely tried first, and still has to produce a heartbeat.

   The remembered board is only ever matched against ports the scan
   found, so a stale or hand-edited state file can misdirect the search
   but cannot make it open something that isn't a USB serial port. Delete
   the file to make the next run search from scratch.

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

#ifndef TR_SWITCH_FIND_H
#define TR_SWITCH_FIND_H

#include "tr_switch.h"

#include <stddef.h>

#define TR_SWITCH_PATH_MAX        64
#define TR_SWITCH_PRODUCT_MAX     64
#define TR_SWITCH_SERIAL_MAX      32
#define TR_SWITCH_MAX_CANDIDATES   8

// Where the answer is remembered, under $HOME.
#define TR_SWITCH_STATE_RELPATH ".local/state/simple_sat_ops/tr_switch.state"

// How long a port gets to produce a heartbeat before we give up on it.
// The firmware beats every TR_SWITCH_HEARTBEAT_PERIOD_S and the probe can
// open just after one has gone out, so the window has to hold a whole
// period with room to spare.
#define TR_SWITCH_PROBE_SECONDS 4.0

// One USB serial port that might be the switch.
typedef struct tr_switch_candidate {
    char     path[TR_SWITCH_PATH_MAX];        // "/dev/ttyACM1"
    char     product[TR_SWITCH_PRODUCT_MAX];  // USB product string, "" if unknown
    char     serial[TR_SWITCH_SERIAL_MAX];    // USB serial, "" if unknown
    unsigned vid, pid;                        // 0 when the OS doesn't say
} tr_switch_candidate_t;

// The board a previous run found.
typedef struct tr_switch_memo {
    char     serial[TR_SWITCH_SERIAL_MAX];
    char     product[TR_SWITCH_PRODUCT_MAX];
    char     path[TR_SWITCH_PATH_MAX];
    unsigned vid, pid;
} tr_switch_memo_t;

// How well the remembered board matches a port that is attached now.
typedef enum {
    TR_SWITCH_MEMO_NONE = 0,    // nothing remembered is attached
    TR_SWITCH_MEMO_PATH,        // the remembered port, but no serial to confirm it
    TR_SWITCH_MEMO_SERIAL,      // the remembered board itself
} tr_switch_memo_hit_t;

// Every USB serial port on the machine, in path order. Returns how many
// were written, at most `cap`.
//
// Both roots default to the real ones ("/sys" and "/dev") when NULL; a
// test passes its own tree instead. Linux ports (/sys/class/tty/ttyACM*)
// and macOS ports (/dev/cu.usbmodem*) are both looked for whatever the
// host is, since a machine only ever has one kind and a test wants both.
int tr_switch_scan(const char *sysfs_root, const char *dev_root,
                   tr_switch_candidate_t *out, int cap);

// $HOME/TR_SWITCH_STATE_RELPATH. Returns 0, or -1 with no $HOME.
int tr_switch_state_path(char *out, size_t cap);

// Read / write the remembered board. Plain "key = value" text, so it
// reads and edits by hand; an unknown key is ignored and a missing one
// leaves its field empty, which is what lets a file written by another
// build still be worth reading. Both return 0 on success, -1 otherwise
// (a missing file is a quiet -1: nothing has been found yet).
int tr_switch_memo_read (const char *path, tr_switch_memo_t *m);
int tr_switch_memo_write(const char *path, const tr_switch_memo_t *m);

// Which of `c` the memo points at, and how surely. `index` is set only
// when the result is not TR_SWITCH_MEMO_NONE. An empty memo matches
// nothing.
tr_switch_memo_hit_t tr_switch_memo_pick(const tr_switch_memo_t *m,
                                         const tr_switch_candidate_t *c, int n,
                                         int *index);

// Find the switch and open it, leaving `s` connected and its
// device_filename pointing at `path_out` (which the caller owns and must
// keep alive for as long as the link is). The board that answered is
// written to the state file, so the next run finds it first.
//
// `found`, when given, comes back holding the port that won, so the
// caller can name the board it opened; `remembered`, when given, comes
// back non-zero if that port was the remembered one. `looked_at`, when
// given, comes back holding the ports that were considered, comma
// separated, for the caller to name in a warning.
//
// Returns 0 with the switch open, -1 with nothing open.
int tr_switch_open_detected(tr_switch_t *s, char *path_out, size_t path_cap,
                            tr_switch_candidate_t *found, int *remembered,
                            char *looked_at, size_t looked_cap);

// The same search with the two directory roots and the state file handed
// in: what tr_switch_open_detected calls once it has resolved the real
// ones, and what the self-test drives with a simulated switch on a
// pseudo-terminal. NULL roots mean the real ones; a NULL state_file means
// remember nothing.
int tr_switch_open_found(tr_switch_t *s,
                         const char *sysfs_root, const char *dev_root,
                         const char *state_file,
                         char *path_out, size_t path_cap,
                         tr_switch_candidate_t *found, int *remembered,
                         char *looked_at, size_t looked_cap);

#endif // TR_SWITCH_FIND_H
