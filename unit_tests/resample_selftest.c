/*

    Simple Satellite Operations  unit_tests/resample_selftest.c

    Coverage for src/dsp/resample.c — the integer-factor linear
    upsampler ham_speak uses to lift 48 kHz mic audio to the 480 kHz TX
    rate before FM modulation. A regression here would mistime or
    mis-scale the transmitted voice.

    What's covered:
      - Output count is n*L when the buffer is big enough.
      - L=1 is an exact identity.
      - Linear interpolation produces the expected ramp, and the final
        input sample is held flat (it has no successor to interpolate to).
      - out_cap is honoured: a short buffer truncates and never writes
        past its end.
      - Bad arguments (n=0, L<1, NULL in/out) return 0 and write nothing.

    Exit status: 0 if all TAP assertions ok, non-zero otherwise.

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#include "resample.h"
#include "tap.h"

#include <stdint.h>
#include <string.h>

static void test_count_and_identity(void)
{
    int16_t in[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    int16_t out[80];

    size_t n = resample_up_linear(in, 8, 10, out, 80);
    tap_okf(n == 80, "x10 of 8 samples -> 80 (got %zu)", n);

    int16_t id[8];
    size_t m = resample_up_linear(in, 8, 1, id, 8);
    int same = (m == 8) && (memcmp(in, id, sizeof in) == 0);
    tap_ok(same, "L=1 is an exact identity");
}

static void test_interpolation(void)
{
    // 0 -> 100 over 4 steps, then the last sample (100) held flat.
    int16_t in[2] = { 0, 100 };
    int16_t out[8];
    size_t n = resample_up_linear(in, 2, 4, out, 8);

    int16_t want[8] = { 0, 25, 50, 75, 100, 100, 100, 100 };
    int ok = (n == 8) && (memcmp(out, want, sizeof want) == 0);
    tap_okf(ok, "linear interp ramp + last-sample hold "
            "(got %d,%d,%d,%d,%d,%d,%d,%d)",
            out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7]);
}

static void test_cap_clamp(void)
{
    int16_t in[2] = { 0, 100 };
    int16_t out[8];
    for (int i = 0; i < 8; ++i) out[i] = -999;   // sentinels

    size_t n = resample_up_linear(in, 2, 4, out, 3);   // cap < n*L
    int clamped = (n == 3)
               && out[0] == 0 && out[1] == 25 && out[2] == 50
               && out[3] == -999;                       // untouched
    tap_okf(clamped, "out_cap=3 writes exactly 3 and no more (n=%zu, out[3]=%d)",
            n, out[3]);
}

static void test_bad_args(void)
{
    int16_t in[4] = { 1, 2, 3, 4 };
    int16_t out[16];

    tap_ok(resample_up_linear(in, 0, 4, out, 16) == 0, "n=0 returns 0");
    tap_ok(resample_up_linear(in, 4, 0, out, 16) == 0, "L=0 returns 0");
    tap_ok(resample_up_linear(in, 4, -1, out, 16) == 0, "L<0 returns 0");
    tap_ok(resample_up_linear(NULL, 4, 4, out, 16) == 0, "NULL in returns 0");
    tap_ok(resample_up_linear(in, 4, 4, NULL, 16) == 0, "NULL out returns 0");
}

int main(void)
{
    test_count_and_identity();
    test_interpolation();
    test_cap_clamp();
    test_bad_args();
    return tap_done();
}
