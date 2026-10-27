/*

    Simple Satellite Operations  src/ipc/sso_action_log.h

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

// sso_action_log.h — per-pass plain-text action log.
//
// Every operator action (keys, ":" commands, modal open/close), every
// rotator command put on the wire, and every tracking state change lands
// as one line in <pass_folder>/actions.log:
//
//   2026-10-27T14:03:22.123Z [rot]   SET az=123.4 el=45.6
//
// Where /var/log/sso/runs.log (sso_audit) answers "who ran what, when",
// this file answers "what happened during this pass", next to the pass's
// recordings and tx.log.
//
// Lines logged before the pass folder exists (startup, hardware bring-up)
// are held in a small in-memory buffer and written out once
// sso_action_log_set_dir() is called. In a process that never sets a
// directory (viewers, standalone tools) the calls are cheap no-ops apart
// from that buffer.
//
// Thread-safe: the rotator worker and the RX/TX workers log from their
// own threads. Each line is a single O_APPEND write(), so a second
// process logging into the same folder can't interleave mid-line.

#ifndef SSO_ACTION_LOG_H
#define SSO_ACTION_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

// Point the log at <dir>/actions.log (opened for append, created if
// missing), flushing any buffered lines into it. Calling again with a
// different directory closes the old file and moves to the new one; the
// same directory is a no-op. Returns 0 on success, -1 if the file can't
// be opened (logging then stays in buffer-only mode).
int sso_action_log_set_dir(const char *dir);

// Append one line. `cat` is a short subsystem tag ("ui", "cmd", "rot",
// "track", "tx", "rx", "scan", "pass", "app"); the message is printf-
// formatted. Newlines in the message are flattened to spaces.
void sso_action_log(const char *cat, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

// Close the file. Later sso_action_log() calls go back to buffering.
void sso_action_log_close(void);

#ifdef __cplusplus
}
#endif

#endif
