/*

    Simple Satellite Operations  unit_tests/fm_mod_selftest.c

    Coverage for src/dsp/fm_mod.c — the narrowband-FM modulator and
    key-up/key-down ramp shared by the telecommand TX path and the
    ham_speak voice tool. A regression in the deviation scaling would
    put the wrong amount of energy on the air; a regression in the
    phase accumulator would click at every chunk boundary.

    What's covered:
      - fm_mod_init zeroes the phase; zero audio holds the carrier at a
        constant (I=22937, Q=0) point with no rotation.
      - Constant-envelope output: |I,Q| stays at the 22937 amplitude for
        arbitrary audio (FM is angle-only).
      - Deviation scaling: constant full-scale audio rotates the carrier
        at exactly +/-deviation Hz, recovered via an atan2 discriminator.
      - Chunk-boundary phase continuity: one block == two half blocks
        through the same fm_mod_t, byte-for-byte.
      - Phase stays bounded (no NaN / no envelope drift) over a long run
        that wraps many times.
      - fm_apply_ramp: env=0 at the first sample, an untouched middle,
        ramp_n=0 is a no-op, and ramp_n > n/2 clamps without overrun.
      - NULL-argument safety on every entry point.

    Exit status: 0 if all TAP assertions ok, non-zero otherwise.

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#include "fm_mod.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// atan2 FM discriminator: instantaneous frequency (Hz) between two
// consecutive IQ pairs, the same arg(z[k]*conj(z[k-1])) the receiver uses.
static double demod_hz(int16_t I, int16_t Q, int16_t pI, int16_t pQ, double fs)
{
    double dphi = atan2((double) Q * pI - (double) I * pQ,
                        (double) I * pI + (double) Q * pQ);
    return dphi * fs / (2.0 * M_PI);
}

static void test_init_and_zero_audio(void)
{
    fm_mod_t m;
    m.phi = 1.234;
    fm_mod_init(&m);
    tap_okf(m.phi == 0.0, "fm_mod_init zeroes phase (phi=%g)", m.phi);

    enum { N = 256 };
    int16_t pcm[N] = {0};        // silence
    int16_t iq[2 * N];
    fm_mod_block(&m, pcm, N, 5000.0, 48000.0, iq);

    int held = 1;
    for (int i = 0; i < N; ++i) {
        if (iq[2 * i] != 22937 || iq[2 * i + 1] != 0) { held = 0; break; }
    }
    tap_ok(held, "zero audio holds carrier at (22937, 0), no rotation");
}

static void test_constant_envelope(void)
{
    enum { N = 2000 };
    fm_mod_t m;
    fm_mod_init(&m);
    int16_t *pcm = malloc(N * sizeof(int16_t));
    int16_t *iq  = malloc(2 * N * sizeof(int16_t));
    // A 700 Hz tone at half scale — arbitrary varying audio.
    for (int i = 0; i < N; ++i)
        pcm[i] = (int16_t) lround(16000.0 * sin(2.0 * M_PI * 700.0 * i / 48000.0));
    fm_mod_block(&m, pcm, N, 5000.0, 48000.0, iq);

    int worst = 0;
    for (int i = 0; i < N; ++i) {
        double mag = sqrt((double) iq[2*i] * iq[2*i]
                        + (double) iq[2*i+1] * iq[2*i+1]);
        int err = (int) lround(fabs(mag - 22937.0));
        if (err > worst) worst = err;
    }
    tap_okf(worst <= 3, "constant-envelope: |IQ| within %d of 22937", worst);
    free(pcm); free(iq);
}

static void test_deviation_scaling(void)
{
    enum { N = 4800 };
    const double fs = 48000.0, dev = 5000.0;
    int16_t *pcm = malloc(N * sizeof(int16_t));
    int16_t *iq  = malloc(2 * N * sizeof(int16_t));

    // Full positive scale -> +dev; full negative -> -dev.
    fm_mod_t m;
    fm_mod_init(&m);
    for (int i = 0; i < N; ++i) pcm[i] = 32767;
    fm_mod_block(&m, pcm, N, dev, fs, iq);
    double sum = 0.0; int cnt = 0;
    for (int k = 1; k < N; ++k) {
        sum += demod_hz(iq[2*k], iq[2*k+1], iq[2*(k-1)], iq[2*(k-1)+1], fs);
        cnt++;
    }
    double mean_pos = sum / cnt;
    tap_okf(fabs(mean_pos - dev) < 5.0,
            "full + audio -> +%.0f Hz deviation (got %.2f)", dev, mean_pos);

    fm_mod_init(&m);
    for (int i = 0; i < N; ++i) pcm[i] = -32767;
    fm_mod_block(&m, pcm, N, dev, fs, iq);
    sum = 0.0; cnt = 0;
    for (int k = 1; k < N; ++k) {
        sum += demod_hz(iq[2*k], iq[2*k+1], iq[2*(k-1)], iq[2*(k-1)+1], fs);
        cnt++;
    }
    double mean_neg = sum / cnt;
    tap_okf(fabs(mean_neg + dev) < 5.0,
            "full - audio -> -%.0f Hz deviation (got %.2f)", dev, mean_neg);
    free(pcm); free(iq);
}

static void test_phase_continuity(void)
{
    enum { N = 1024 };
    int16_t *pcm = malloc(N * sizeof(int16_t));
    int16_t *iq_one = malloc(2 * N * sizeof(int16_t));
    int16_t *iq_two = malloc(2 * N * sizeof(int16_t));
    for (int i = 0; i < N; ++i)
        pcm[i] = (int16_t) lround(20000.0 * sin(2.0 * M_PI * 1234.0 * i / 48000.0));

    fm_mod_t a; fm_mod_init(&a);
    fm_mod_block(&a, pcm, N, 5000.0, 48000.0, iq_one);

    fm_mod_t b; fm_mod_init(&b);
    fm_mod_block(&b, pcm,         N / 2, 5000.0, 48000.0, iq_two);
    fm_mod_block(&b, pcm + N / 2, N / 2, 5000.0, 48000.0, iq_two + N);

    tap_ok(memcmp(iq_one, iq_two, 2 * N * sizeof(int16_t)) == 0,
           "one block == two half blocks through the same fm_mod_t");
    free(pcm); free(iq_one); free(iq_two);
}

static void test_long_run_bounded(void)
{
    enum { N = 200000 };
    int16_t *pcm = malloc(N * sizeof(int16_t));
    int16_t *iq  = malloc(2 * N * sizeof(int16_t));
    for (int i = 0; i < N; ++i) pcm[i] = 32767;   // wraps phase many times
    fm_mod_t m; fm_mod_init(&m);
    fm_mod_block(&m, pcm, N, 5000.0, 48000.0, iq);

    int bad = 0;
    for (int i = 0; i < N; ++i) {
        double mag = sqrt((double) iq[2*i] * iq[2*i]
                        + (double) iq[2*i+1] * iq[2*i+1]);
        if (isnan(mag) || fabs(mag - 22937.0) > 3.0) { bad = 1; break; }
    }
    tap_ok(!bad, "long run stays bounded: no NaN, envelope holds after wrap");
    free(pcm); free(iq);
}

static void test_ramp(void)
{
    enum { N = 400, R = 50 };
    int16_t base[2 * N];
    for (int i = 0; i < 2 * N; ++i) base[i] = 10000;

    // ramp_n = 0 is a no-op.
    int16_t a[2 * N];
    memcpy(a, base, sizeof base);
    fm_apply_ramp(a, N, 0);
    tap_ok(memcmp(a, base, sizeof base) == 0, "ramp_n=0 leaves IQ unchanged");

    // Normal ramp: first sample fully attenuated, a mid sample untouched.
    int16_t b[2 * N];
    memcpy(b, base, sizeof base);
    fm_apply_ramp(b, N, R);
    tap_okf(b[0] == 0 && b[1] == 0, "ramp zeroes the first sample (%d,%d)",
            b[0], b[1]);
    tap_ok(b[2 * (N / 2)] == 10000 && b[2 * (N / 2) + 1] == 10000,
           "ramp leaves the middle untouched");
    tap_okf(abs(b[2 * (N - 1)]) < 10000,
            "ramp attenuates the last sample (%d)", b[2 * (N - 1)]);

    // ramp_n > n/2 must clamp, not run off the end.
    int16_t c[2 * N];
    memcpy(c, base, sizeof base);
    fm_apply_ramp(c, N, N);   // clamped to N/2 internally
    tap_ok(c[0] == 0, "ramp_n > n/2 clamps without overrun");
}

static void test_null_safety(void)
{
    int16_t pcm[8] = {0};
    int16_t iq[16] = {0};
    fm_mod_t m; fm_mod_init(&m);
    fm_mod_init(NULL);
    fm_mod_block(NULL, pcm, 8, 5000.0, 48000.0, iq);
    fm_mod_block(&m, NULL, 8, 5000.0, 48000.0, iq);
    fm_mod_block(&m, pcm, 8, 5000.0, 48000.0, NULL);
    fm_mod_block(&m, pcm, 8, 5000.0, 0.0, iq);     // fs <= 0
    fm_apply_ramp(NULL, 8, 2);
    tap_ok(1, "NULL / zero-fs arguments are handled without crashing");
}

int main(void)
{
    test_init_and_zero_audio();
    test_constant_envelope();
    test_deviation_scaling();
    test_phase_continuity();
    test_long_run_bounded();
    test_ramp();
    test_null_safety();
    return tap_done();
}
