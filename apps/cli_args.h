/*

   Simple Satellite Operations  cli_args.h

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

// simple_sat_ops startup wiring: parse argv into state_t (opening hardware,
// resolving paths, loading the HMAC keyfile and SDR config), and the
// --self-test configuration dump. The parse_args / HELP_* / PARSE_* contract
// is in argparse.h.

#ifndef CLI_ARGS_H
#define CLI_ARGS_H

#include "tx_state.h"

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Parse argc/argv into *state (help == HELP_OFF), or print help (HELP_BRIEF /
// HELP_FULL). Returns a PARSE_* code (see argparse.h).
int apply_args(state_t *state, int argc, char **argv, double jul_utc, int help);

// Print the resolved configuration snapshot in a stable "key: value" layout
// for --self-test. Called at the end of the bring-up (after the rotator and
// SDR are opened), so the hardware lines report the live opened state.
void self_test_report(const state_t *state, FILE *out, int argc, char **argv);

// Resolve and load the HMAC keyfile into the TX state (default path if
// --hmac-keyfile was not given). A missing or bad key only sets the display
// status; it is not fatal here — the TX path refuses to key the PA when the
// key is absent.
void cli_load_hmac_keyfile(tx_t *tx);

// Lint the --tc-file agenda (if any) against the firmware telecommand set
// before any PA-keying bring-up. Returns 0 to continue, EXIT_FAILURE on lint
// errors unless --ignore-at-your-peril-all-tc-errors is set.
int cli_tcmd_lint_gate(const tx_t *tx);

#ifdef __cplusplus
}
#endif

#endif // CLI_ARGS_H
