/*

    Simple Satellite Operations  unit_tests/fir_decim_selftest.c

    Coverage for src/dsp/fir_decim.c. The decimator narrows the B210's
    raw IQ stream (e.g. 480 kS/s) to a sane post-decim rate (96 kS/s
    @ M=5) so the FM discriminator and AX100 decoder see only the
    band of interest. If anti-alias rejection regresses, out-of-band
    spurs fold into the post-decim spectrum and the squelch/decoder
    chase ghosts; if DC gain drifts, every downstream level meter is
    miscalibrated.

    What's covered:
      - Constructor parameter validation: NULL on M=0, ntaps=0, non-
        positive rates, fc_hz >= fs_in_hz/2.
      - Accessors return the configured M and ntaps.
      - DC sustained input passes with unity gain.
      - In-band tone passes with ≈ unity magnitude.
      - Out-of-band tone (would alias into the post-decim spectrum) is
        attenuated by > 30 dB.
      - Total output count after a long input matches floor(n_in / M)
        within a one-sample warmup tolerance.
      - State preserved across multiple push calls — splitting the
        input into two halves produces the same output as a single
        call of the full length.
      - push() with NULL pointers returns 0 (no crash).

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

#include "fir_decim.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Production-like config: 480 kS/s in, M=5 → 96 kS/s out, fc=18 kHz,
// 96-tap Hamming-windowed sinc. Matches apps/main.c's defaults.
#define FS_IN  480000.0
#define FC     18000.0
#define M_DEC  5u
#define NTAPS  96u
#define FS_OUT (FS_IN / (double) M_DEC)

// Synthesize a complex tone at f_hz with peak amplitude `amp`, fs samples
// per second, starting phase phi0. Fills n_pairs interleaved int16 I,Q.
static void synth_complex_tone(int16_t *iq, size_t n_pairs,
                               double f_hz, double fs, double amp,
                               double phi0)
{
    const double dphi = 2.0 * M_PI * f_hz / fs;
    double phase = phi0;
    for (size_t i = 0; i < n_pairs; ++i) {
        double I = amp * cos(phase);
        double Q = amp * sin(phase);
        if (I >  32767.0) I =  32767.0;
        if (I < -32768.0) I = -32768.0;
        if (Q >  32767.0) Q =  32767.0;
        if (Q < -32768.0) Q = -32768.0;
        iq[2 * i + 0] = (int16_t) lrint(I);
        iq[2 * i + 1] = (int16_t) lrint(Q);
        phase += dphi;
    }
}

// Mean |z| over the last `tail_pairs` samples of an IQ stream.
static double mean_abs_z(const int16_t *iq, size_t n_pairs, size_t tail_pairs)
{
    if (n_pairs == 0 || tail_pairs == 0) return 0.0;
    size_t start = (n_pairs > tail_pairs) ? (n_pairs - tail_pairs) : 0;
    size_t cnt = n_pairs - start;
    double sum = 0.0;
    for (size_t i = start; i < n_pairs; ++i) {
        double I = (double) iq[2 * i + 0];
        double Q = (double) iq[2 * i + 1];
        sum += sqrt(I * I + Q * Q);
    }
    return sum / (double) cnt;
}

// ------------------------------------------------------------------
// 1. Constructor parameter validation.
// ------------------------------------------------------------------

static void test_constructor_validation(void)
{
    fir_decim_iq_t *f;

    f = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    tap_ok(f != NULL, "ctor: valid params returns non-NULL");
    fir_decim_iq_free(f);

    tap_ok(fir_decim_iq_new(FS_IN, FC, NTAPS, 0u) == NULL,
           "ctor: M=0 rejected");
    tap_ok(fir_decim_iq_new(FS_IN, FC, 0u, M_DEC) == NULL,
           "ctor: ntaps=0 rejected");
    tap_ok(fir_decim_iq_new(0.0, FC, NTAPS, M_DEC) == NULL,
           "ctor: fs_in_hz=0 rejected");
    tap_ok(fir_decim_iq_new(-1.0, FC, NTAPS, M_DEC) == NULL,
           "ctor: fs_in_hz negative rejected");
    tap_ok(fir_decim_iq_new(FS_IN, 0.0, NTAPS, M_DEC) == NULL,
           "ctor: fc_hz=0 rejected");
    tap_ok(fir_decim_iq_new(FS_IN, FS_IN, NTAPS, M_DEC) == NULL,
           "ctor: fc_hz >= fs/2 rejected");
    tap_ok(fir_decim_iq_new(FS_IN, FS_IN * 0.5, NTAPS, M_DEC) == NULL,
           "ctor: fc_hz == fs/2 rejected (boundary)");
}

// ------------------------------------------------------------------
// 2. Accessors.
// ------------------------------------------------------------------

static void test_accessors(void)
{
    fir_decim_iq_t *f = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    if (!f) { tap_bail("ctor failed"); return; }
    tap_okf(fir_decim_iq_M(f) == M_DEC,
            "accessor M == %u (got %u)", M_DEC, fir_decim_iq_M(f));
    tap_okf(fir_decim_iq_ntaps(f) == NTAPS,
            "accessor ntaps == %u (got %u)", NTAPS, fir_decim_iq_ntaps(f));
    // NULL safety: return 0.
    tap_ok(fir_decim_iq_M(NULL) == 0,
           "accessor M(NULL) returns 0");
    tap_ok(fir_decim_iq_ntaps(NULL) == 0,
           "accessor ntaps(NULL) returns 0");
    fir_decim_iq_free(f);
}

// ------------------------------------------------------------------
// 3. DC unity gain.
// ------------------------------------------------------------------

static void test_dc_unity_gain(void)
{
    fir_decim_iq_t *f = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    if (!f) { tap_bail("ctor failed"); return; }

    const size_t n_in = 4096;
    const size_t cap_out = n_in / M_DEC + 8;
    int16_t *in  = (int16_t *) calloc(n_in * 2,    sizeof(int16_t));
    int16_t *out = (int16_t *) calloc(cap_out * 2, sizeof(int16_t));
    if (!in || !out) { tap_bail("oom"); free(in); free(out);
                       fir_decim_iq_free(f); return; }

    const int16_t DC_AMP = 10000;
    for (size_t i = 0; i < n_in; ++i) {
        in[2 * i + 0] = DC_AMP;
        in[2 * i + 1] = 0;
    }
    size_t n_out = fir_decim_iq_push(f, in, n_in, out, cap_out);
    tap_okf(n_out > 0, "DC: produced %zu output samples", n_out);

    // Skip the first half (warmup) and measure the tail.
    size_t tail = n_out / 2;
    double mean_I = 0.0, mean_Q = 0.0;
    size_t start = n_out - tail;
    for (size_t i = start; i < n_out; ++i) {
        mean_I += out[2 * i + 0];
        mean_Q += out[2 * i + 1];
    }
    mean_I /= (double) tail;
    mean_Q /= (double) tail;
    tap_okf(fabs(mean_I - DC_AMP) < 2.0,
            "DC: settled I mean %.2f ≈ %d", mean_I, DC_AMP);
    tap_okf(fabs(mean_Q) < 2.0,
            "DC: settled Q mean %.2f ≈ 0", mean_Q);

    free(in); free(out); fir_decim_iq_free(f);
}

// ------------------------------------------------------------------
// 4. In-band tone passes ≈ unity magnitude.
// ------------------------------------------------------------------

static void test_passband_tone(void)
{
    fir_decim_iq_t *f = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    if (!f) { tap_bail("ctor failed"); return; }

    const size_t n_in = 8192;
    const size_t cap_out = n_in / M_DEC + 8;
    int16_t *in  = (int16_t *) calloc(n_in * 2,    sizeof(int16_t));
    int16_t *out = (int16_t *) calloc(cap_out * 2, sizeof(int16_t));
    if (!in || !out) { tap_bail("oom"); free(in); free(out);
                       fir_decim_iq_free(f); return; }

    const double AMP = 12000.0;
    const double F_IN_BAND = 4000.0;  // well below fc=18 kHz
    synth_complex_tone(in, n_in, F_IN_BAND, FS_IN, AMP, 0.0);
    size_t n_out = fir_decim_iq_push(f, in, n_in, out, cap_out);

    double mag = mean_abs_z(out, n_out, n_out / 2);
    double ratio = mag / AMP;
    tap_okf(ratio > 0.97 && ratio < 1.03,
            "passband tone at %.0f Hz: |z|/A = %.4f (≈ 1.0)",
            F_IN_BAND, ratio);

    free(in); free(out); fir_decim_iq_free(f);
}

// ------------------------------------------------------------------
// 5. Out-of-band tone (would alias) is rejected.
// ------------------------------------------------------------------

static void test_stopband_rejection(void)
{
    fir_decim_iq_t *f = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    if (!f) { tap_bail("ctor failed"); return; }

    const size_t n_in = 16384;
    const size_t cap_out = n_in / M_DEC + 8;
    int16_t *in  = (int16_t *) calloc(n_in * 2,    sizeof(int16_t));
    int16_t *out = (int16_t *) calloc(cap_out * 2, sizeof(int16_t));
    if (!in || !out) { tap_bail("oom"); free(in); free(out);
                       fir_decim_iq_free(f); return; }

    // 100 kHz tone: above fs_out/2 = 48 kHz, so without rejection it
    // would alias into the post-decim band. Also well above fc=18 kHz
    // so the FIR's stopband applies fully.
    const double AMP = 16000.0;
    const double F_ALIAS = 100000.0;
    synth_complex_tone(in, n_in, F_ALIAS, FS_IN, AMP, 0.0);
    size_t n_out = fir_decim_iq_push(f, in, n_in, out, cap_out);

    double mag = mean_abs_z(out, n_out, n_out / 2);
    double atten_db = 20.0 * log10((mag + 1e-30) / AMP);
    tap_okf(atten_db < -30.0,
            "stopband tone at %.0f Hz: attenuation %.2f dB > 30 dB",
            F_ALIAS, -atten_db);

    free(in); free(out); fir_decim_iq_free(f);
}

// ------------------------------------------------------------------
// 6. Output count == floor(n_in / M) (within a one-sample warmup).
// ------------------------------------------------------------------

static void test_output_rate(void)
{
    fir_decim_iq_t *f = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    if (!f) { tap_bail("ctor failed"); return; }

    const size_t n_in = 10000;
    const size_t cap_out = n_in / M_DEC + 8;
    int16_t *in  = (int16_t *) calloc(n_in * 2,    sizeof(int16_t));
    int16_t *out = (int16_t *) calloc(cap_out * 2, sizeof(int16_t));
    if (!in || !out) { tap_bail("oom"); free(in); free(out);
                       fir_decim_iq_free(f); return; }
    for (size_t i = 0; i < n_in; ++i) { in[2*i] = 0; in[2*i+1] = 0; }
    size_t n_out = fir_decim_iq_push(f, in, n_in, out, cap_out);
    size_t expected = n_in / M_DEC;
    // The implementation emits the first output once phase reaches M,
    // so we expect exactly floor(n_in / M) for an initial run from a
    // fresh decimator (phase starts at 0).
    tap_okf(n_out == expected,
            "rate: %zu output for %zu input @ M=%u (expected %zu)",
            n_out, n_in, M_DEC, expected);
    free(in); free(out); fir_decim_iq_free(f);
}

// ------------------------------------------------------------------
// 7. State preserved across push() calls.
// ------------------------------------------------------------------

static void test_state_preserved_across_calls(void)
{
    const size_t n_in   = 4096;
    const size_t cap    = n_in / M_DEC + 8;
    int16_t *in   = (int16_t *) calloc(n_in * 2, sizeof(int16_t));
    int16_t *outA = (int16_t *) calloc(cap   * 2, sizeof(int16_t));
    int16_t *outB = (int16_t *) calloc(cap   * 2, sizeof(int16_t));
    if (!in || !outA || !outB) {
        tap_bail("oom"); free(in); free(outA); free(outB); return;
    }
    synth_complex_tone(in, n_in, 5000.0, FS_IN, 10000.0, 0.0);

    // One-shot reference.
    fir_decim_iq_t *fA = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    size_t nA = fir_decim_iq_push(fA, in, n_in, outA, cap);
    fir_decim_iq_free(fA);

    // Chunked: split at an arbitrary non-multiple-of-M boundary so the
    // decimation phase carries across.
    fir_decim_iq_t *fB = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    const size_t split = 1357;
    size_t nB1 = fir_decim_iq_push(fB, in, split,
                                   outB, cap);
    size_t nB2 = fir_decim_iq_push(fB, in + split * 2, n_in - split,
                                   outB + nB1 * 2, cap - nB1);
    size_t nB = nB1 + nB2;
    fir_decim_iq_free(fB);

    tap_okf(nA == nB,
            "split: total output count matches (one-shot=%zu chunked=%zu)",
            nA, nB);
    int max_diff = 0;
    for (size_t i = 0; i < nA * 2; ++i) {
        int d = outA[i] - outB[i];
        if (d < 0) d = -d;
        if (d > max_diff) max_diff = d;
    }
    tap_okf(max_diff == 0,
            "split: byte-identical output (max |Δ|=%d)", max_diff);

    free(in); free(outA); free(outB);
}

// ------------------------------------------------------------------
// 8. NULL-pointer / zero-args safety on push.
// ------------------------------------------------------------------

static void test_push_null_safety(void)
{
    fir_decim_iq_t *f = fir_decim_iq_new(FS_IN, FC, NTAPS, M_DEC);
    if (!f) { tap_bail("ctor failed"); return; }
    int16_t in[8] = {0};
    int16_t out[8];
    tap_ok(fir_decim_iq_push(NULL, in, 1, out, 4) == 0,
           "push: NULL filter returns 0");
    tap_ok(fir_decim_iq_push(f, NULL, 1, out, 4) == 0,
           "push: NULL input returns 0");
    tap_ok(fir_decim_iq_push(f, in, 1, NULL, 4) == 0,
           "push: NULL output returns 0");
    // free() on NULL is a no-op per posix; check it doesn't crash.
    fir_decim_iq_free(NULL);
    tap_ok(1, "free: NULL pointer no-op survives");
    fir_decim_iq_free(f);
}

int main(void)
{
    test_constructor_validation();
    test_accessors();
    test_dc_unity_gain();
    test_passband_tone();
    test_stopband_rejection();
    test_output_rate();
    test_state_preserved_across_calls();
    test_push_null_safety();
    return tap_done();
}
