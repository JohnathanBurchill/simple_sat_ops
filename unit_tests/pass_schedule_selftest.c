/*

    Simple Satellite Operations  unit_tests/pass_schedule_selftest.c

    Coverage for src/ipc/pass_schedule.c — the {"t":"passes"} upcoming-passes
    wire codec the --viewer-stream relay emits and the standalone viewer reads.
    A key-name or field-order slip here would silently break the viewer's pass
    list / alerts, so this pins the round-trip and the format.

    Exit status: 0 = all tests passed, non-zero = failure.

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

#include "pass_schedule.h"
#include "tap.h"

#include <math.h>
#include <string.h>

static int deq(double a, double b) { return fabs(a - b) <= 1e-3 + 1e-6 * fabs(b); }

int main(void)
{
    char line[16384];

    // --- a populated schedule round-trips --------------------------------
    {
        pass_schedule_t s;
        memset(&s, 0, sizeof s);
        snprintf(s.satellite, sizeof s.satellite, "ISS (ZARYA)");
        snprintf(s.idesg, sizeof s.idesg, "98067A");
        s.generated_unix = 1749941177.0;
        s.tle_epoch_min  = 9326060.0;
        s.count = 3;
        double seed[3][5] = {
            { 1749945600, 1749946140, 1749945870, 12.4, 118.0 },
            { 1749966000, 1749966600, 1749966300, 41.7, 205.0 },
            { 1750050000, 1750050720, 1750050360, 73.2, 288.0 },
        };
        for (int i = 0; i < 3; i++) {
            s.passes[i].aos_unix    = seed[i][0];
            s.passes[i].los_unix    = seed[i][1];
            s.passes[i].peak_unix   = seed[i][2];
            s.passes[i].peak_el_deg = seed[i][3];
            s.passes[i].peak_az_deg = seed[i][4];
        }

        tap_ok(pass_schedule_encode(&s, line, sizeof line) == 0, "encode returns 0");

        pass_schedule_t d;
        tap_ok(pass_schedule_decode(line, &d) == 0, "decode returns 0");
        tap_ok(strcmp(d.satellite, "ISS (ZARYA)") == 0, "satellite preserved");
        tap_ok(strcmp(d.idesg, "98067A") == 0, "idesg preserved");
        tap_ok(d.count == 3, "count preserved");
        tap_ok(deq(d.generated_unix, 1749941177.0), "generated_unix preserved");
        int passes_ok = 1;
        for (int i = 0; i < 3; i++) {
            passes_ok &= deq(d.passes[i].aos_unix,    seed[i][0]);
            passes_ok &= deq(d.passes[i].los_unix,    seed[i][1]);
            passes_ok &= deq(d.passes[i].peak_unix,   seed[i][2]);
            passes_ok &= deq(d.passes[i].peak_el_deg, seed[i][3]);
            passes_ok &= deq(d.passes[i].peak_az_deg, seed[i][4]);
        }
        tap_ok(passes_ok, "every pass's 5 fields round-trip");
    }

    // --- empty schedule ---------------------------------------------------
    {
        pass_schedule_t s;
        memset(&s, 0, sizeof s);
        snprintf(s.satellite, sizeof s.satellite, "FrontierSat");
        s.count = 0;
        pass_schedule_t d;
        tap_ok(pass_schedule_encode(&s, line, sizeof line) == 0, "encode empty returns 0");
        tap_ok(pass_schedule_decode(line, &d) == 0 && d.count == 0, "decode empty -> count 0");
    }

    // --- robustness -------------------------------------------------------
    {
        pass_schedule_t d;
        tap_ok(pass_schedule_decode("{\"t\":\"state\",\"sat\":\"X\"}", &d) == -1,
               "non-passes line rejected");
        tap_ok(pass_schedule_decode(NULL, &d) == -1, "NULL line rejected");
        // n says 3 but only 1 whole pass (5 numbers) is present -> clamp to 1.
        const char *partial =
            "{\"t\":\"passes\",\"sat\":\"A\",\"n\":3,\"p\":[100,200,150,10,90]}";
        tap_ok(pass_schedule_decode(partial, &d) == 0 && d.count == 1,
               "short array clamps count to whole passes");
        // encode into a too-small buffer fails cleanly.
        pass_schedule_t s;
        memset(&s, 0, sizeof s);
        s.count = 5;
        char tiny[16];
        tap_ok(pass_schedule_encode(&s, tiny, sizeof tiny) == -1,
               "encode into a too-small buffer rejected");
    }

    return tap_done();
}
