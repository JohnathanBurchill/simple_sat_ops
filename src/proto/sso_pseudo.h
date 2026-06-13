/*

    Simple Satellite Operations  sso_pseudo.h

    simple_sat_ops-directed pseudo-commands. An agenda line (or a line typed
    into the 't' compose modal) that begins with "SSO+" is not a literal
    telecommand: it is a directive the ground software expands into a concrete
    "CTS1+...!" telecommand at TRANSMIT time, so the expansion can depend on
    live state (the wall clock) that is not known when the agenda is authored.

    The first such command is the time-sync:

        SSO+sync_sat_time_to_ground()!
            ->  CTS1+set_system_time(<now_ms+500>)@tssent=<pass-start ms>!

    where now_ms is the queue-time UTC in milliseconds (so each send carries a
    fresh time) and the @tssent dedup key is the pass-start value the caller
    supplies (one fixed value for the whole session). The clocks come in via a
    context struct so this stays a pure function of (cmd, ctx) -- the live
    transmit path passes real clocks; the linter passes placeholders.

    The command set is a table (sso_pseudo.c), mirroring tcmd_spec.c: a future
    SSO+ command is one expander function plus one row.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#ifndef SSO_PSEUDO_H
#define SSO_PSEUDO_H

#include <stddef.h>

#define SSO_PSEUDO_PREFIX     "SSO+"
#define SSO_PSEUDO_PREFIX_LEN 4

typedef enum {
    SSO_PSEUDO_OK = 0,        // expanded into out
    SSO_PSEUDO_NOT_PSEUDO,    // no "SSO+" prefix; out := verbatim copy of cmd
    SSO_PSEUDO_UNKNOWN_CMD,   // "SSO+" prefix but the name is not in the registry
    SSO_PSEUDO_BAD_SYNTAX,    // missing name / "()" / "!", or wrong arg count
    SSO_PSEUDO_TRUNCATED,     // expansion did not fit out_cap
} sso_pseudo_status_t;

// Clocks the caller supplies, captured at queue time. Kept in a struct so a
// future SSO+ command can use more without churning the signature.
typedef struct {
    // Fresh queue-time UTC in milliseconds. The time value(s) a command sets
    // derive from this, so each send carries a current time.
    long long now_ms;
    // The @tssent dedup key to stamp: the pass start, truncated to the minute,
    // fixed for the whole simple_sat_ops session. The flight firmware dedups on
    // this value, so a value constant across a session's sends makes the
    // satellite run the command once per pass (correct across hour/midnight
    // boundaries) yet a later session (a new minute) runs it again.
    long long tssent_ms;
} sso_pseudo_ctx_t;

// Cheap probe: 1 if `cmd` (already trimmed, inline comment stripped) begins
// with "SSO+". Does not validate the rest of the line.
int sso_pseudo_is_directed(const char *cmd);

// Expand one "SSO+<name>(args)!" line into its concrete "CTS1+...!"
// telecommand, using ctx for the queue-time clocks. On SSO_PSEUDO_NOT_PSEUDO
// (no "SSO+" prefix) `out` receives a verbatim copy of `cmd`, so a caller can
// use `out` unconditionally and key behaviour off the returned status. On any
// error status the caller must NOT transmit. `err`/`err_cap` (NULL/0 to skip)
// receive a human message on a non-OK, non-NOT_PSEUDO status.
sso_pseudo_status_t sso_pseudo_expand(const char *cmd, const sso_pseudo_ctx_t *ctx,
                                      char *out, size_t out_cap,
                                      char *err, size_t err_cap);

// Human label for a status (for messages / diagnostics).
const char *sso_pseudo_status_label(sso_pseudo_status_t st);

#endif // SSO_PSEUDO_H
