/*

   Simple Satellite Operations  src/app_state.h

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

// Process-wide run-mode slice of state_t: the loop-running flag, verbosity,
// and the one-shot mode flags set once in apply_args. See state.h.

#ifndef APP_STATE_H
#define APP_STATE_H

typedef struct app {
    int n_options;
    int running;
    int verbose_level;

    // Run mode + one-shot CLI flags (set once in apply_args / main).
    int control_mode;        // --control: this process is the operator
    int viewer_mode;         // bare invocation found a running operator
    int viewer_stream;       // --viewer-stream: headless JSON stream to stdout
    int self_test;           // --self-test: print a report and exit
    int testing_mode;        // --testing
} app_t;

#endif // APP_STATE_H
