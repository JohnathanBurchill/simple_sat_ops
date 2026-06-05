/*

    Simple Satellite Operations  tcmd_lint.h

    Lint a telecommand (or a whole agenda file) against what the flight
    firmware will actually accept, before it goes on the air. The checks
    mirror the firmware's own parser (TCMD_parse_full_telecommand): the
    CTS1+ prefix, the single terminating '!', a known command name, the
    parenthesised argument list with the exact argument count the command
    expects, the length limits, and well-formed @tssent=/@tsexec= tags. The
    command set and per-command argument counts come from tcmd_spec.h, which
    is generated from the firmware. A command whose readiness level is not
    "operation" (ground-only, high-risk, etc.) is flagged as a warning.

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

#ifndef TCMD_LINT_H
#define TCMD_LINT_H

#include <stddef.h>
#include <stdio.h>

typedef enum {
    TCMD_LINT_OK    = 0,
    TCMD_LINT_WARN  = 1,
    TCMD_LINT_ERROR = 2,
} tcmd_lint_severity_t;

// Lint one telecommand string -- a single "CTS1+name(args)...!" line with any
// inline '# comment' already removed and surrounding whitespace trimmed.
// Every problem found is appended (semicolon-separated) to `msg` (pass NULL/0
// to skip the text). Returns the worst severity; a clean command returns
// TCMD_LINT_OK.
tcmd_lint_severity_t tcmd_lint_command(const char *cmd, char *msg, size_t msg_cap);

// Lint every telecommand in an agenda file: one command per line, blank lines
// and whole-line '#' comments skipped, inline '# ...' comments stripped (the
// same rule simple_sat_ops transmits by). Prints
// "<path>:<line>: error|warning: <msg>" for each flagged line to `out`.
// Returns the number of ERROR lines (0 = safe to proceed); *warn_count, if
// non-NULL, receives the WARNING count. Returns -1 if the file can't be read.
int tcmd_lint_file(const char *path, FILE *out, int *warn_count);

#endif // TCMD_LINT_H
