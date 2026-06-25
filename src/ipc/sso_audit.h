/*

    Simple Satellite Operations  src/ipc/sso_audit.h

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

// sso_audit.h — append-only invocation log for the sso tools.
//
// Each major tool (simple_sat_ops, b210_rx_tx, tx_frame_sdr,
// rx_replay, and any other hardware-touching binary) calls
// sso_audit_start() once at process entry. That writes one line to
// /var/log/sso/runs.log:
//
//   <ISO-ts>  <unix-user>  <tool>  <pid>  start <detail>
//
// and arms an atexit handler that, on normal exit, appends:
//
//   <ISO-ts>  <unix-user>  <tool>  <pid>  end exit=<code>
//
// `detail` is a free-form short string ("operator", "viewer", "replay",
// "force-claim from=jburchill"), already-shell-quote-safe; the writer
// escapes tabs and newlines defensively. The log path can be overridden
// with $SSO_AUDIT_LOG. When the default path isn't writable (no
// /var/log/sso on a dev host), the fallback is
// $HOME/.local/share/simple_sat_ops/runs.log.
//
// All four sso_audit_* entry points are reentrant w.r.t. signal
// handlers (write() into an open fd, no malloc) and safe to call from
// multiple processes simultaneously: a flock around each line write
// keeps records from interleaving.

#ifndef SSO_AUDIT_H
#define SSO_AUDIT_H

#ifdef __cplusplus
extern "C" {
#endif

// Record process start. `tool` is the short canonical name
// ("simple_sat_ops"). `detail` is appended to the line; pass "" if
// none. Registers atexit() handler that writes the matching `end`
// record. Returns 0 on success, -1 on log-open failure (which is
// non-fatal — the tool keeps running).
int sso_audit_start(const char *tool, const char *detail);

// Record an arbitrary intermediate event for this process. Tool name
// and pid come from the earlier sso_audit_start() call. Examples:
// sso_audit_event("force-claim", "from=jburchill"). Safe to call
// before sso_audit_start (the line just lacks tool name).
int sso_audit_event(const char *event, const char *detail);

// Override the exit code recorded in the atexit `end` line. Useful
// when the caller wants to record a meaningful failure code that
// differs from exit(). Last value wins. Default is 0.
void sso_audit_set_exit_code(int code);

// Returns the canonical Unix username for the current process: prefers
// $SUDO_USER over $USER over $LOGNAME, with /etc/passwd lookup as the
// last resort. Always returns a non-NULL pointer to a static buffer
// (may be "unknown"). Not thread-safe across the first call.
const char *sso_unix_user(void);

#ifdef __cplusplus
}
#endif

#endif
