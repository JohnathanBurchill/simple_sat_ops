/*

    Simple Satellite Operations  unit_tests/biquad_selftest.c

    Coverage for src/dsp/biquad.c. The biquad is a 2nd-order RBJ-cookbook
    bandpass with constant 0 dB peak gain. It's used downstream of the
    FM discriminator on the operator's monitor path to isolate the
    beacon tone before the squelch envelope detector. Wrong coefficients
    here would silently kill or mis-band the squelch's idea of "signal
    present", which the operator then sees as no beacon detection.

    What's covered:
      - biquad_bpf zeros the delay-line state z1/z2.
      - Coefficients match a hand-computed reference for a known {f0,
        bw, fs} (regression pin on the cookbook formula).
      - Pole magnitudes < 1 (filter is stable in the design).
      - Steady-state magnitude response: ≈ 1.0 at f0, → 0 at DC and
        Nyquist, well attenuated in the stopband.
      - Impulse response is a damped sinusoid (peak energy in the
        leading samples, decays by the end of a long buffer).
      - Random noise driven through the filter stays bounded.
      - Q < 0.5 is floored to 0.5 (catches the regression where a wide
        BW with a small Q would otherwise blow up the coefficients).
      - Cascading two identical sections deepens stopband rejection.

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

#include "biquad.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// xorshift32 — deterministic noise source for the stability test.
static uint32_t xs_state = 0xfeedbeefu;
static uint32_t xs_next(void)
{
    uint32_t x = xs_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    xs_state = x;
    return x;
}
static double xs_unit(void)
{
    return ((double) xs_next() / (double) UINT32_MAX) * 2.0 - 1.0;
}

// Drive a settled sinusoid through the filter and return the peak
// magnitude of the output over the LAST `measure_n` samples (after the
// `warmup_n`-sample transient).
static double settled_peak(biquad_t *bq, double f_hz, double fs,
                           int warmup_n, int measure_n)
{
    double w = 2.0 * M_PI * f_hz / fs;
    double phase = 0.0;
    for (int i = 0; i < warmup_n; ++i) {
        (void) biquad_step(bq, sin(phase));
        phase += w;
    }
    double peak = 0.0;
    for (int i = 0; i < measure_n; ++i) {
        double y = biquad_step(bq, sin(phase));
        if (fabs(y) > peak) peak = fabs(y);
        phase += w;
    }
    return peak;
}

// ------------------------------------------------------------------
// 1. biquad_bpf zeros the delay-line state.
// ------------------------------------------------------------------

static void test_init_zeros_delay_line(void)
{
    biquad_t bq;
    // Stuff z1/z2 with garbage to verify biquad_bpf clears them.
    bq.z1 = 1.5; bq.z2 = -2.3;
    biquad_bpf(&bq, 1200.0, 600.0, 48000.0);
    tap_okf(bq.z1 == 0.0 && bq.z2 == 0.0,
            "init: z1=%.3g z2=%.3g (both 0)", bq.z1, bq.z2);
}

// ------------------------------------------------------------------
// 2. Coefficients match a hand-computed reference.
// ------------------------------------------------------------------

static void test_coefficients_reference(void)
{
    // f0=1200, bw=600, fs=48000 → w0=π/20, Q=2.
    //   alpha = sin(w0)/(2Q),  a0 = 1+alpha
    //   b0 = alpha/a0,  b2 = -alpha/a0,  b1 = 0
    //   a1 = -2*cos(w0)/a0,  a2 = (1-alpha)/a0
    const double fs = 48000.0;
    const double f0 = 1200.0;
    const double bw = 600.0;
    biquad_t bq;
    biquad_bpf(&bq, f0, bw, fs);

    double w0 = 2.0 * M_PI * f0 / fs;
    double Q  = f0 / bw;
    double alpha = sin(w0) / (2.0 * Q);
    double a0 = 1.0 + alpha;
    double exp_b0 =  alpha / a0;
    double exp_b1 =  0.0;
    double exp_b2 = -alpha / a0;
    double exp_a1 = -2.0 * cos(w0) / a0;
    double exp_a2 = (1.0 - alpha) / a0;

    const double tol = 1e-12;
    tap_okf(fabs(bq.b0 - exp_b0) < tol,
            "coeff b0=%.6g (expected %.6g)", bq.b0, exp_b0);
    tap_okf(fabs(bq.b1 - exp_b1) < tol,
            "coeff b1=%.6g (expected 0)", bq.b1);
    tap_okf(fabs(bq.b2 - exp_b2) < tol,
            "coeff b2=%.6g (expected %.6g)", bq.b2, exp_b2);
    tap_okf(fabs(bq.a1 - exp_a1) < tol,
            "coeff a1=%.6g (expected %.6g)", bq.a1, exp_a1);
    tap_okf(fabs(bq.a2 - exp_a2) < tol,
            "coeff a2=%.6g (expected %.6g)", bq.a2, exp_a2);
}

// ------------------------------------------------------------------
// 3. Pole magnitudes < 1 (filter stable in the design).
// ------------------------------------------------------------------

static void test_pole_magnitudes_stable(void)
{
    // Poles solve z² + a1 z + a2 = 0 → z = (-a1 ± √(a1²-4a2))/2.
    // For a BPF with these coefficients the discriminant is usually
    // negative (complex-conjugate pole pair); |z|² = a2 then.
    const struct { double f0, bw; } cases[] = {
        {  100.0,   50.0 }, // narrow LF
        { 1200.0,  600.0 }, // typical audio beacon
        { 6000.0, 2000.0 }, // wider band
        {18000.0, 4000.0 }, // near Nyquist
    };
    int n = sizeof cases / sizeof cases[0];
    int fails = 0;
    for (int i = 0; i < n; ++i) {
        biquad_t bq;
        biquad_bpf(&bq, cases[i].f0, cases[i].bw, 48000.0);
        double disc = bq.a1 * bq.a1 - 4.0 * bq.a2;
        double mag2;
        if (disc < 0.0) {
            // Complex-conjugate poles: |z|² = product of roots = a2.
            mag2 = bq.a2;
        } else {
            double s = sqrt(disc);
            double r1 = (-bq.a1 + s) / 2.0;
            double r2 = (-bq.a1 - s) / 2.0;
            double m1 = fabs(r1), m2 = fabs(r2);
            mag2 = (m1 > m2) ? (m1 * m1) : (m2 * m2);
        }
        if (mag2 >= 1.0) ++fails;
    }
    tap_okf(fails == 0,
            "all %d test configs have pole magnitude < 1 (%d failed)",
            n, fails);
}

// ------------------------------------------------------------------
// 4. Peak gain at f0 is ≈ 1.0 (constant 0-dB form).
// ------------------------------------------------------------------

static void test_gain_at_center(void)
{
    biquad_t bq;
    biquad_bpf(&bq, 1200.0, 600.0, 48000.0);
    double peak = settled_peak(&bq, 1200.0, 48000.0, 4000, 4000);
    tap_okf(fabs(peak - 1.0) < 0.02,
            "peak gain at f0 ≈ 1.0 (got %.5f)", peak);
}

// ------------------------------------------------------------------
// 5. DC steady-state rejection.
// ------------------------------------------------------------------

static void test_gain_at_dc(void)
{
    biquad_t bq;
    biquad_bpf(&bq, 1200.0, 600.0, 48000.0);
    // Run thousands of samples of DC=1.0 and measure the tail.
    for (int i = 0; i < 8000; ++i) (void) biquad_step(&bq, 1.0);
    double peak = 0.0;
    for (int i = 0; i < 1000; ++i) {
        double y = biquad_step(&bq, 1.0);
        if (fabs(y) > peak) peak = fabs(y);
    }
    tap_okf(peak < 1e-6,
            "DC steady-state output ≈ 0 (peak %.3g)", peak);
}

// ------------------------------------------------------------------
// 6. Nyquist (alternating ±1) steady-state rejection.
// ------------------------------------------------------------------

static void test_gain_at_nyquist(void)
{
    biquad_t bq;
    biquad_bpf(&bq, 1200.0, 600.0, 48000.0);
    double sign = 1.0;
    for (int i = 0; i < 8000; ++i) {
        (void) biquad_step(&bq, sign);
        sign = -sign;
    }
    double peak = 0.0;
    for (int i = 0; i < 1000; ++i) {
        double y = biquad_step(&bq, sign);
        if (fabs(y) > peak) peak = fabs(y);
        sign = -sign;
    }
    tap_okf(peak < 1e-6,
            "Nyquist steady-state output ≈ 0 (peak %.3g)", peak);
}

// ------------------------------------------------------------------
// 7. Stopband rejection: a tone an octave above f0 attenuates strongly.
// ------------------------------------------------------------------

static void test_stopband_rejection(void)
{
    biquad_t bq;
    biquad_bpf(&bq, 1200.0, 600.0, 48000.0);
    double peak = settled_peak(&bq, 4800.0, 48000.0, 4000, 4000);
    // 4800 Hz is 4×f0 → an octave + change above the band. With Q=2
    // and a 2nd-order BPF, the asymptotic skirt is ~−12 dB/octave.
    // We expect attenuation around −18 to −25 dB at this offset.
    double atten_db = 20.0 * log10(peak + 1e-30);
    tap_okf(atten_db < -15.0,
            "stopband at 4×f0 attenuated > 15 dB (got %.2f dB)",
            atten_db);
}

// ------------------------------------------------------------------
// 8. Impulse response decays (no runaway, ringing dies out).
// ------------------------------------------------------------------

static void test_impulse_response_decays(void)
{
    biquad_t bq;
    biquad_bpf(&bq, 1200.0, 600.0, 48000.0);
    const int N = 4096;
    double *h = (double *) malloc((size_t) N * sizeof(double));
    if (!h) { tap_bail("oom"); return; }
    h[0] = biquad_step(&bq, 1.0);
    for (int i = 1; i < N; ++i) {
        h[i] = biquad_step(&bq, 0.0);
    }
    // Peak should be in the first ~Q periods (≈ Q * fs/f0 = 80 samples).
    // Take the absolute peak across the whole response, and verify it
    // lives in the leading 200 samples — i.e., the ringing isn't
    // amplified later.
    int peak_idx = 0;
    double peak_mag = 0.0;
    for (int i = 0; i < N; ++i) {
        if (fabs(h[i]) > peak_mag) { peak_mag = fabs(h[i]); peak_idx = i; }
    }
    tap_okf(peak_idx < 200,
            "impulse: peak in leading 200 samples (got idx=%d)", peak_idx);

    // Tail energy is a small fraction of the head energy: the
    // last quarter of the buffer carries < 1% of the total energy.
    double e_head = 0.0, e_tail = 0.0;
    for (int i = 0;            i < N / 4;     ++i) e_head += h[i] * h[i];
    for (int i = 3 * N / 4;    i < N;         ++i) e_tail += h[i] * h[i];
    double ratio = (e_head > 0.0) ? (e_tail / e_head) : 1.0;
    tap_okf(ratio < 0.01,
            "impulse: tail/head energy ratio %.5f < 1%%", ratio);
    free(h);
}

// ------------------------------------------------------------------
// 9. Stability under random input.
// ------------------------------------------------------------------

static void test_stability_under_noise(void)
{
    biquad_t bq;
    biquad_bpf(&bq, 1200.0, 600.0, 48000.0);
    xs_state = 0xfeedbeefu;
    double max_abs = 0.0;
    int N = 200000;
    for (int i = 0; i < N; ++i) {
        double x = xs_unit();
        double y = biquad_step(&bq, x);
        double a = fabs(y);
        if (a > max_abs) max_abs = a;
    }
    // Worst-case BPF gain at f0 is 1, so |y| ≤ 1 in steady state — and
    // even an adversarial impulse can't push it past Q within ±1
    // input. We test a generous bound to catch only blow-ups.
    tap_okf(max_abs < 10.0,
            "noise drive stays bounded over %d samples (max |y|=%.3g)",
            N, max_abs);
}

// ------------------------------------------------------------------
// 10. Q < 0.5 is floored.
// ------------------------------------------------------------------

static void test_q_floor(void)
{
    // bw = 10×f0 → Q = 0.1; the function floors Q to 0.5.
    const double fs = 48000.0;
    const double f0 = 1200.0;
    biquad_t bq;
    biquad_bpf(&bq, f0, 10.0 * f0, fs);

    // Compute reference assuming the floored Q.
    double w0 = 2.0 * M_PI * f0 / fs;
    double Q  = 0.5;
    double alpha = sin(w0) / (2.0 * Q);
    double a0 = 1.0 + alpha;
    double exp_b0 = alpha / a0;
    double exp_a2 = (1.0 - alpha) / a0;
    tap_okf(fabs(bq.b0 - exp_b0) < 1e-12,
            "Q-floor: b0 matches Q=0.5 reference (got %.6g, want %.6g)",
            bq.b0, exp_b0);
    tap_okf(fabs(bq.a2 - exp_a2) < 1e-12,
            "Q-floor: a2 matches Q=0.5 reference (got %.6g, want %.6g)",
            bq.a2, exp_a2);
}

// ------------------------------------------------------------------
// 11. Cascading two identical sections deepens stopband rejection.
// ------------------------------------------------------------------

static void test_cascade_deepens_stopband(void)
{
    // One section's stopband at 4×f0 vs two sections (same coeffs).
    biquad_t bq1;
    biquad_t bq2a, bq2b;
    biquad_bpf(&bq1,  1200.0, 600.0, 48000.0);
    biquad_bpf(&bq2a, 1200.0, 600.0, 48000.0);
    biquad_bpf(&bq2b, 1200.0, 600.0, 48000.0);

    // Warm up + measure for the cascade.
    double w = 2.0 * M_PI * 4800.0 / 48000.0;
    double phase = 0.0;
    for (int i = 0; i < 4000; ++i) {
        double x = sin(phase);
        (void) biquad_step(&bq2a, x);
        // For the cascade reference we feed the output of bq2a into bq2b.
        (void) biquad_step(&bq2b, biquad_step(&bq2a, x));
        phase += w;
    }
    // We just stomped the warmup with two simultaneous chains — redo cleanly.
    biquad_bpf(&bq2a, 1200.0, 600.0, 48000.0);
    biquad_bpf(&bq2b, 1200.0, 600.0, 48000.0);
    phase = 0.0;
    for (int i = 0; i < 4000; ++i) {
        double x = sin(phase);
        (void) biquad_step(&bq2b, biquad_step(&bq2a, x));
        phase += w;
    }
    double peak2 = 0.0;
    for (int i = 0; i < 4000; ++i) {
        double x = sin(phase);
        double y = biquad_step(&bq2b, biquad_step(&bq2a, x));
        if (fabs(y) > peak2) peak2 = fabs(y);
        phase += w;
    }
    double peak1 = settled_peak(&bq1, 4800.0, 48000.0, 4000, 4000);
    double db1 = 20.0 * log10(peak1 + 1e-30);
    double db2 = 20.0 * log10(peak2 + 1e-30);
    tap_okf((db1 - db2) > 10.0,
            "cascade adds ≥ 10 dB stopband (1×: %.2f dB, 2×: %.2f dB, Δ=%.2f)",
            db1, db2, db1 - db2);
}

// ------------------------------------------------------------------
// 12. Degenerate bandwidth falls back to a passthrough, not all-stop.
// ------------------------------------------------------------------

static void test_degenerate_bandwidth_passthrough(void)
{
    // bw_hz <= 0 makes Q = f0/bw infinite (bw=0) or negative (bw<0). The
    // low-Q floor only catches the wide end, so the un-guarded coefficients
    // collapse to b0=b2=0 — a silent all-stop that kills the signal. The
    // guard must instead leave a unity passthrough (b0=1, the rest 0).
    const double bws[] = { 0.0, -100.0 };
    for (int i = 0; i < 2; ++i) {
        biquad_t bq;
        bq.z1 = 3.0; bq.z2 = -4.0;  // garbage the guard must also clear
        biquad_bpf(&bq, 1200.0, bws[i], 48000.0);
        tap_okf(bq.b0 == 1.0 && bq.b1 == 0.0 && bq.b2 == 0.0
                && bq.a1 == 0.0 && bq.a2 == 0.0 && bq.z1 == 0.0 && bq.z2 == 0.0,
                "bw=%.0f: passthrough coeffs (b0=%.3g b2=%.3g)",
                bws[i], bq.b0, bq.b2);
        // And it actually passes a sample through unchanged.
        double y = biquad_step(&bq, 0.75);
        tap_okf(fabs(y - 0.75) < 1e-12,
                "bw=%.0f: step passes input through (got %.6g)", bws[i], y);
    }
    // A non-positive sample rate is guarded the same way.
    biquad_t bq;
    biquad_bpf(&bq, 1200.0, 600.0, 0.0);
    tap_okf(bq.b0 == 1.0 && bq.b2 == 0.0,
            "fs=0: passthrough coeffs (b0=%.3g)", bq.b0);
}

int main(void)
{
    test_init_zeros_delay_line();
    test_coefficients_reference();
    test_pole_magnitudes_stable();
    test_gain_at_center();
    test_gain_at_dc();
    test_gain_at_nyquist();
    test_stopband_rejection();
    test_impulse_response_decays();
    test_stability_under_noise();
    test_q_floor();
    test_cascade_deepens_stopband();
    test_degenerate_bandwidth_passthrough();
    return tap_done();
}
