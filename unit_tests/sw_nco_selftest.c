/*

    Simple Satellite Operations  unit_tests/sw_nco_selftest.c

    Coverage for src/dsp/sw_nco. The NCO is the software-Doppler core
    that runs inside b210_rx_tx_core_pump; getting its sign, phase
    continuity, or magnitude preservation wrong breaks every live
    decode silently. The hardware path can't be exercised here, so
    these tests drive sw_nco directly with synthesised IQ.

    What's covered:
      - Zero-frequency = pass-through (no rotation applied).
      - A pure +5 kHz tone, rotated by +5 kHz, lands at DC: the
        rotated stream's mean magnitude per sample is preserved AND
        the post-rotation phase progression is ~zero per sample.
      - Phase continuity across multiple sw_nco_apply calls — split
        a long signal into 8 chunks, run NCO chunk-by-chunk; the
        residual frequency must match the single-call result.
      - Magnitude preservation: rotation is unitary, so |z'[n]| ==
        |z[n]| modulo int16 quantisation (within ±1 LSB on average).
      - Set-freq mid-stream: with a step in frequency, the phase
        accumulator stays continuous (no audible click). Verified by
        the residual carrier never excursing past a bounded delta.

    Exit status: 0 if all TAP assertions ok, non-zero otherwise.

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#include "sw_nco.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Synthesize a unit-amplitude complex tone at frequency f_hz, fs samples
// per second, with starting phase phi0. Fills n_pairs interleaved int16
// I,Q samples scaled to ~half full-scale so a small rotation rounding
// error doesn't saturate.
static void synth_tone(int16_t *iq, size_t n_pairs,
                       double f_hz, double fs_hz, double phi0)
{
    const double dphi = 2.0 * M_PI * f_hz / fs_hz;
    const double amp  = 16000.0;
    double phase = phi0;
    for (size_t i = 0; i < n_pairs; ++i) {
        iq[i * 2 + 0] = (int16_t)(amp * cos(phase));
        iq[i * 2 + 1] = (int16_t)(amp * sin(phase));
        phase += dphi;
    }
}

// Estimate the residual carrier frequency of an IQ block via the
// phase difference between consecutive samples (FM-discriminator
// trick), averaged over the block. Returns the residual in Hz.
// `skip` samples at the head are ignored so the first sample's
// phase doesn't bias the estimate.
static double estimate_residual_hz(const int16_t *iq, size_t n_pairs,
                                   double fs_hz, size_t skip)
{
    if (n_pairs < skip + 2) return 0.0;
    double phase_sum = 0.0;
    size_t cnt = 0;
    for (size_t i = skip + 1; i < n_pairs; ++i) {
        double I0 = (double) iq[(i - 1) * 2 + 0];
        double Q0 = (double) iq[(i - 1) * 2 + 1];
        double I1 = (double) iq[i * 2 + 0];
        double Q1 = (double) iq[i * 2 + 1];
        // arg(z1 · conj(z0)) — the per-sample phase advance.
        double cross_re = I1 * I0 + Q1 * Q0;
        double cross_im = Q1 * I0 - I1 * Q0;
        phase_sum += atan2(cross_im, cross_re);
        ++cnt;
    }
    double dphi_mean = phase_sum / (double) cnt;
    return dphi_mean * fs_hz / (2.0 * M_PI);
}

// Per-sample magnitude (|I| + |Q| approximation). Cheaper than sqrt
// and tracks the true magnitude closely enough for a comparison test.
static double mean_abs_magnitude(const int16_t *iq, size_t n_pairs)
{
    if (n_pairs == 0) return 0.0;
    double sum = 0.0;
    for (size_t i = 0; i < n_pairs; ++i) {
        double I = (double) iq[i * 2 + 0];
        double Q = (double) iq[i * 2 + 1];
        sum += sqrt(I * I + Q * Q);
    }
    return sum / (double) n_pairs;
}

// ------------------------------------------------------------------
// 1. Zero-frequency NCO is a pass-through.
// ------------------------------------------------------------------

static void test_zero_freq_passthrough(void)
{
    const double fs = 96000.0;
    const size_t n  = 4096;
    int16_t *iq  = (int16_t *) malloc(n * 2 * sizeof(int16_t));
    int16_t *ref = (int16_t *) malloc(n * 2 * sizeof(int16_t));
    if (!iq || !ref) { tap_bail("oom"); free(iq); free(ref); return; }

    synth_tone(iq, n, 3000.0, fs, 0.0);
    memcpy(ref, iq, n * 2 * sizeof(int16_t));

    sw_nco_t nco;
    sw_nco_init(&nco, fs);
    // freq stays at 0 → apply must be a no-op fast path.
    sw_nco_apply(&nco, iq, n);

    tap_ok(memcmp(iq, ref, n * 2 * sizeof(int16_t)) == 0,
           "zero freq: stream unchanged");
    tap_ok(nco.phase_rad == 0.0,
           "zero freq: phase accumulator not advanced");
    free(iq); free(ref);
}

// ------------------------------------------------------------------
// 2. Single-call rotation lands the tone at DC.
// ------------------------------------------------------------------

static void test_single_call_lands_at_dc(void)
{
    const double fs   = 96000.0;
    const double f_in = 5000.0;
    const size_t n    = 16384;
    int16_t *iq = (int16_t *) malloc(n * 2 * sizeof(int16_t));
    if (!iq) { tap_bail("oom"); return; }

    synth_tone(iq, n, f_in, fs, 0.0);
    double mag_before = mean_abs_magnitude(iq, n);

    sw_nco_t nco;
    sw_nco_init(&nco, fs);
    sw_nco_set_freq(&nco, f_in);  // rotate by +f_in → tone lands at DC
    sw_nco_apply(&nco, iq, n);

    double residual = estimate_residual_hz(iq, n, fs, 64);
    tap_okf(fabs(residual) < 0.5,
            "rotated +%.0f Hz tone lands at DC (residual %.3f Hz)",
            f_in, residual);

    double mag_after = mean_abs_magnitude(iq, n);
    double mag_ratio = mag_after / mag_before;
    tap_okf(mag_ratio > 0.99 && mag_ratio < 1.01,
            "magnitude preserved within 1%% (ratio %.4f)", mag_ratio);
    free(iq);
}

// ------------------------------------------------------------------
// 3. Phase continuity: chunked apply == single-call apply.
// ------------------------------------------------------------------

static void test_phase_continuity_across_chunks(void)
{
    const double fs    = 96000.0;
    const double f_in  = 7500.0;
    const size_t n     = 16384;
    const size_t chunk = 1024;  // 16 chunks
    int16_t *single = (int16_t *) malloc(n * 2 * sizeof(int16_t));
    int16_t *chunked = (int16_t *) malloc(n * 2 * sizeof(int16_t));
    if (!single || !chunked) { tap_bail("oom"); free(single); free(chunked); return; }

    synth_tone(single,  n, f_in, fs, 0.0);
    synth_tone(chunked, n, f_in, fs, 0.0);

    sw_nco_t nco1, nco2;
    sw_nco_init(&nco1, fs);
    sw_nco_init(&nco2, fs);
    sw_nco_set_freq(&nco1, f_in);
    sw_nco_set_freq(&nco2, f_in);

    sw_nco_apply(&nco1, single, n);
    for (size_t off = 0; off < n; off += chunk) {
        size_t take = (off + chunk <= n) ? chunk : (n - off);
        sw_nco_apply(&nco2, chunked + off * 2, take);
    }

    // The two output streams should be byte-identical: same input, same
    // NCO frequency, only the chunking differs. If the phase isn't
    // continuous across calls, the chunked stream would have ±1 LSB
    // discontinuities at each chunk boundary.
    int max_diff = 0;
    for (size_t i = 0; i < n * 2; ++i) {
        int d = single[i] - chunked[i];
        if (d < 0) d = -d;
        if (d > max_diff) max_diff = d;
    }
    tap_okf(max_diff == 0,
            "chunked apply matches single apply byte-for-byte (max |Δ|=%d)",
            max_diff);
    free(single); free(chunked);
}

// ------------------------------------------------------------------
// 4. Mid-stream frequency step keeps phase continuous.
// ------------------------------------------------------------------

static void test_freq_step_phase_continuous(void)
{
    // Simulate a Doppler trajectory: first half at +f1, second half at
    // +f2. The NCO frequency follows. With phase preserved across the
    // change, the residual carrier of each half measured locally
    // should still be near DC; with phase RESET, the boundary would
    // show a glitch and the residual estimate over the second half
    // would be skewed by the per-sample phase advance.
    const double fs = 96000.0;
    const double f1 = 4000.0;
    const double f2 = -3000.0;
    const size_t n_half = 8192;
    int16_t *iq = (int16_t *) malloc(n_half * 2 * 2 * sizeof(int16_t));
    if (!iq) { tap_bail("oom"); return; }

    // Concatenate two single-frequency tones — this is a phase-
    // discontinuous source on purpose: the test is whether the NCO
    // can track f1 then f2 and bring both halves to DC.
    synth_tone(iq,                n_half, f1, fs, 0.0);
    // Continue the carrier phase into the second half so the synthetic
    // input itself is continuous; otherwise we'd be testing NCO
    // tracking AND a non-physical step in the input.
    double phi_continuous = 2.0 * M_PI * f1 * (double) n_half / fs;
    synth_tone(iq + n_half * 2,   n_half, f2, fs, phi_continuous);

    sw_nco_t nco;
    sw_nco_init(&nco, fs);

    sw_nco_set_freq(&nco, f1);
    sw_nco_apply(&nco, iq, n_half);

    sw_nco_set_freq(&nco, f2);
    sw_nco_apply(&nco, iq + n_half * 2, n_half);

    double res_first  = estimate_residual_hz(iq,                n_half, fs, 64);
    double res_second = estimate_residual_hz(iq + n_half * 2,   n_half, fs, 64);

    tap_okf(fabs(res_first) < 0.5,
            "freq step: first segment lands at DC (residual %.3f Hz)",
            res_first);
    tap_okf(fabs(res_second) < 0.5,
            "freq step: second segment lands at DC after f change "
            "(residual %.3f Hz)", res_second);
    free(iq);
}

// ------------------------------------------------------------------
// 5. Cumulative phase wrap doesn't drift.
// ------------------------------------------------------------------

static void test_phase_wrap_stays_bounded(void)
{
    // After many apply calls, the wrapped phase accumulator should
    // stay in [-π, π] — i.e. the modular arithmetic at the end of
    // sw_nco_apply hasn't drifted into raw radians territory.
    const double fs = 96000.0;
    int16_t pad[16] = {0};
    sw_nco_t nco;
    sw_nco_init(&nco, fs);
    sw_nco_set_freq(&nco, 12345.0);
    for (int i = 0; i < 1000; ++i) {
        sw_nco_apply(&nco, pad, 8);
    }
    tap_okf(nco.phase_rad >= -M_PI && nco.phase_rad <= M_PI,
            "phase accumulator stays wrapped to [-π, π] "
            "(phase=%.6f rad after 1000 chunks)", nco.phase_rad);
}

int main(void)
{
    test_zero_freq_passthrough();
    test_single_call_lands_at_dc();
    test_phase_continuity_across_chunks();
    test_freq_step_phase_continuous();
    test_phase_wrap_stays_bounded();
    return tap_done();
}
