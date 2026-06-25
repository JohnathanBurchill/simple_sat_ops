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

    On top of the firmware-parser mirror there is a small ground-policy
    blacklist of substrings that make an otherwise-valid command dangerous to
    fly (see tcmd_dangerous_substrings[] in tcmd_lint.c). The first is the
    boot-time agenda filename "default_tcmd_agenda.txt": a command that writes
    or enqueues it can wedge the satellite in a boot loop. A blacklist hit is
    its own top severity, TCMD_LINT_DANGER -- worse than a parse error,
    because the command is perfectly well-formed and the satellite would run
    it. It blocks separately from structural errors and needs its own
    override. (Issue #43.)

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
    TCMD_LINT_OK     = 0,
    TCMD_LINT_WARN   = 1,
    TCMD_LINT_ERROR  = 2,
    // A well-formed command the satellite would happily run, but which is on
    // the ground brick-risk blacklist (e.g. touches default_tcmd_agenda.txt).
    // Ranked above ERROR so it wins the reported severity and gets its own
    // gate; see the header note and tcmd_dangerous_substrings[] in the .c.
    TCMD_LINT_DANGER = 3,
} tcmd_lint_severity_t;

// The longest telecommand (the whole "CTS1+...!" line, including any
// @tssent=/@tsexec= tags) that fits in one over-the-air frame. The AX100
// Reed-Solomon (255,223) block carries 223 payload bytes; the CSP v1
// header takes 4 and the HMAC-SHA1 trailer 4, leaving 223 - 4 - 4 = 215
// for the telecommand text. The firmware parser itself accepts more (up
// to 254 visible chars) but only over the wired umbilical -- nothing
// longer than this goes up over the radio, so the linter warns and the
// compose modal caps typing here. HMAC is always on operationally;
// without it the ceiling would be 219.
#define TCMD_RF_MAX_LEN 215

// The startup / auto-run gate decision, factored out as a pure function so the
// real gate (cli_tcmd_lint_gate), the auto-telecommand modal, and the unit
// tests all share one policy. Given the lint counts for an agenda and the
// operator's two override flags, it says whether to start and, if not, why.
typedef enum {
    TCMD_GATE_PROCEED      = 0,  // safe to start
    TCMD_GATE_BLOCK_DANGER = 1,  // a brick-risk command, not overridden
    TCMD_GATE_BLOCK_ERROR  = 2,  // a parse error, not overridden
} tcmd_gate_decision_t;

// Decide whether an agenda with `errors` parse errors and `dangers` brick-risk
// findings may start. Brick risk is checked first and has its OWN override
// (allow_dangers), kept separate from the parse-error override (allow_errors)
// so that accepting a typo never also accepts a boot-loop. Pure: no I/O, no
// globals -- trivially testable without a satellite or any hardware.
tcmd_gate_decision_t tcmd_lint_gate_decision(int errors, int dangers,
                                             int allow_errors, int allow_dangers);

// Lint one telecommand string -- a single "CTS1+name(args)...!" line with any
// inline '# comment' already removed and surrounding whitespace trimmed.
// Every problem found is appended (semicolon-separated) to `msg` (pass NULL/0
// to skip the text). Returns the worst severity; a clean command returns
// TCMD_LINT_OK.
tcmd_lint_severity_t tcmd_lint_command(const char *cmd, char *msg, size_t msg_cap);

// Lint every telecommand in an agenda file: one command per line, blank lines
// and whole-line '#' comments skipped, inline '# ...' comments stripped (the
// same rule simple_sat_ops transmits by). Prints
// "<path>:<line>: danger|error|warning: <msg>" for each flagged line to `out`.
// Returns the number of structural ERROR lines (0 = no parse errors); the two
// optional out-params receive the WARNING and DANGER counts. A DANGER line
// (brick-risk blacklist hit) is counted in *danger_count, not the return
// value, because it gates separately from parse errors. Any out-pointer may
// be NULL. Returns -1 if the file can't be read.
int tcmd_lint_file(const char *path, FILE *out, int *warn_count, int *danger_count);

#endif // TCMD_LINT_H
