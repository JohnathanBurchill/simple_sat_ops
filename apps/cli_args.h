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

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Parse argc/argv into *state (help == HELP_OFF), or print help (HELP_BRIEF /
// HELP_FULL). Returns a PARSE_* code (see argparse.h).
int apply_args(state_t *state, int argc, char **argv, double jul_utc, int help);

// Print the resolved configuration snapshot (after parse + HMAC load) in a
// stable "key: value" layout for --self-test.
void self_test_report(const state_t *state, FILE *out, int argc, char **argv);

#ifdef __cplusplus
}
#endif

#endif // CLI_ARGS_H
