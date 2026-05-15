/*

    Simple Satellite Operations  utils/modem_iq_selftest.c

    Round-trip and sensitivity tests for modem_iq_to_bits. Synthesizes
    continuous-phase FSK at the same parameters our AX100 link uses
    (9600 bps, h=0.5, samp_rate=48000), feeds it through the IQ-domain
    demod, and checks:

      A. Clean signal — ASM found at the expected offset, post-ASM bits
         match the synthesizer's bit stream with zero errors.
      B. Same content + AWGN — ASM found, bit-error rate stays below a
         loose ceiling for SNRs in the "comfortable" regime.
      C. Silence — no ASM match returned.
      D. A/B sanity — modem_iq_to_bits is not strictly worse than
         modem_pcm16_to_bits on a high-SNR capture (i.e., we didn't
         introduce a regression in the easy case).

    SNR convention: per-IQ-sample input SNR in dB. Noise is iid Gaussian
    added to I and Q independently; signal amplitude is full-scale
    cos/sin × 0.5 so the post-FIR levels resemble real RF.

    No external test framework — exits 0 on full pass, non-zero on
    failure, prints one line per check.

    Copyright (C) 2026  Johnathan K Burchill

    GPLv3 or later.
*/

#define _GNU_SOURCE

#include "modem.h"
#include "modem_iq.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// AX100 32-bit attached sync marker, MSB first as a bit array.
static const uint8_t ASM_BITS[32] = {
    1,0,0,1,0,0,1,1,
    0,0,0,0,1,0,1,1,
    0,1,0,1,0,0,0,1,
    1,1,0,1,1,1,1,0,
};

// Modulator parameters.
typedef struct {
    int    samp_rate;
    int    bit_rate;
    double mod_index;       // h, e.g. 0.5 for MSK
    double amplitude;       // baseband amplitude before noise (0..1)
} mod_params_t;

// xorshift32 RNG so the tests are reproducible without seeding rand()
// (and don't perturb the global libc RNG state).
static uint32_t g_rng = 1u;
static uint32_t rng_next(void)
{
    uint32_t x = g_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng = x;
    return x;
}
static double rng_uniform01(void)
{
    return (double) rng_next() / 4294967296.0;
}
// Box-Muller, returning one Gaussian per call (caches the partner).
static double rng_gauss(void)
{
    static int    cached = 0;
    static double cache  = 0.0;
    if (cached) { cached = 0; return cache; }
    double u1 = rng_uniform01();
    double u2 = rng_uniform01();
    if (u1 < 1e-12) u1 = 1e-12;
    double r = sqrt(-2.0 * log(u1));
    double a = 2.0 * M_PI * u2;
    cache  = r * sin(a);
    cached = 1;
    return r * cos(a);
}

// CPFSK modulator. bits = MSB-first sequence; total samples produced =
// n_bits * sps. Output is int16 interleaved I,Q at the requested
// sample rate. Phase carries continuously across symbols (no GMSK pulse
// shaping — same coverage area as MSK / rectangular FSK; the receiver's
// boxcar matched filter doesn't care which pulse shape we used because
// the noise model dominates the tests).
static void cpfsk_modulate(const uint8_t *bits, size_t n_bits,
                           const mod_params_t *p,
                           int16_t *iq_out)
{
    int sps = p->samp_rate / p->bit_rate;
    double phase = 0.0;
    double dphi_per_sample = M_PI * p->mod_index / (double) sps;
    double amp = p->amplitude * 32767.0;
    for (size_t b = 0; b < n_bits; ++b) {
        double sym = (bits[b] & 1u) ? +1.0 : -1.0;
        for (int s = 0; s < sps; ++s) {
            iq_out[(b * (size_t) sps + (size_t) s) * 2 + 0] =
                (int16_t) lround(amp * cos(phase));
            iq_out[(b * (size_t) sps + (size_t) s) * 2 + 1] =
                (int16_t) lround(amp * sin(phase));
            phase += sym * dphi_per_sample;
        }
    }
}

// Add iid Gaussian noise to I and Q with per-axis standard deviation
// chosen for the requested signal-to-noise ratio in dB. SNR is defined
// over total (I² + Q²) energy.
static void add_awgn(int16_t *iq, size_t n_pairs,
                     double amplitude_full_scale,
                     double snr_db)
{
    // Signal power per sample = A². Noise power per sample = N².
    // SNR = 10·log10(A² / N²) → N = A · 10^(-SNR/20).
    double A = amplitude_full_scale * 32767.0;
    double N = A * pow(10.0, -snr_db / 20.0);
    // Distribute noise equally to I and Q axes: each axis gets N/√2 std.
    double sigma_axis = N / sqrt(2.0);
    for (size_t i = 0; i < n_pairs; ++i) {
        double I = (double) iq[i * 2 + 0] + sigma_axis * rng_gauss();
        double Q = (double) iq[i * 2 + 1] + sigma_axis * rng_gauss();
        if (I >  32767.0) I =  32767.0;
        if (I < -32768.0) I = -32768.0;
        if (Q >  32767.0) Q =  32767.0;
        if (Q < -32768.0) Q = -32768.0;
        iq[i * 2 + 0] = (int16_t) lround(I);
        iq[i * 2 + 1] = (int16_t) lround(Q);
    }
}

// FM-discriminate IQ → real-valued PCM int16 at the same rate. Used
// to feed modem_pcm16_to_bits with the equivalent signal the existing
// chain would see in a real B210 capture, so we can A/B-compare.
static void fm_discriminate(const int16_t *iq, size_t n_pairs,
                            int16_t *pcm_out, double fm_fullscale_hz,
                            int samp_rate)
{
    double k_scale = (double) samp_rate / (2.0 * M_PI * fm_fullscale_hz);
    k_scale *= 32767.0;
    if (n_pairs == 0) return;
    pcm_out[0] = 0;
    double prev_I = (double) iq[0];
    double prev_Q = (double) iq[1];
    for (size_t k = 1; k < n_pairs; ++k) {
        double I = (double) iq[k * 2 + 0];
        double Q = (double) iq[k * 2 + 1];
        double dphi = atan2(Q * prev_I - I * prev_Q,
                            I * prev_I + Q * prev_Q);
        double pcm_d = dphi * k_scale;
        if (pcm_d >  32767.0) pcm_d =  32767.0;
        if (pcm_d < -32768.0) pcm_d = -32768.0;
        pcm_out[k] = (int16_t) lround(pcm_d);
        prev_I = I;
        prev_Q = Q;
    }
}

// Build a known bit stream: preamble (alternating 0/1, M&M loves it),
// ASM, then a deterministic random payload. Returns total bit count.
static size_t build_test_frame(uint8_t *bits, size_t bit_cap,
                               size_t preamble_bits, size_t payload_bits)
{
    size_t n = 0;
    if (preamble_bits + 32 + payload_bits > bit_cap) return 0;
    for (size_t i = 0; i < preamble_bits; ++i) bits[n++] = (uint8_t)(i & 1u);
    for (size_t i = 0; i < 32; ++i)            bits[n++] = ASM_BITS[i];
    // Deterministic pseudo-random payload (xorshift seed reset for repeatability).
    uint32_t s = 0xC0FFEEu;
    for (size_t i = 0; i < payload_bits; ++i) {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        bits[n++] = (uint8_t)((s >> 7) & 1u);
    }
    return n;
}

static int check(int cond, const char *msg)
{
    if (cond) {
        printf("PASS  %s\n", msg);
        return 0;
    }
    printf("FAIL  %s\n", msg);
    return 1;
}

// ------------------------------------------------------------------
// Individual tests
// ------------------------------------------------------------------

static int test_clean_decode(void)
{
    const int samp_rate = 48000;
    const int bit_rate  = 9600;
    const size_t preamble_bits = 64;
    const size_t payload_bits  = 8 * 64;  // 64 bytes ≈ 1.4× ASM region
    const size_t total_bits = preamble_bits + 32 + payload_bits;
    int sps = samp_rate / bit_rate;
    size_t n_pairs = total_bits * (size_t) sps;

    uint8_t *bits     = malloc(total_bits);
    int16_t *iq       = malloc(n_pairs * 2 * sizeof(int16_t));
    uint8_t *out_bits = malloc(total_bits + 1024);
    if (!bits || !iq || !out_bits) { free(bits); free(iq); free(out_bits); return 1; }

    size_t nb = build_test_frame(bits, total_bits, preamble_bits, payload_bits);
    mod_params_t mp = { samp_rate, bit_rate, 0.5, 0.5 };
    cpfsk_modulate(bits, nb, &mp, iq);

    modem_params_t p;
    modem_params_defaults(&p);
    p.samp_rate = samp_rate;
    p.bit_rate  = bit_rate;
    p.rx_disable_dc_block = 1;  // synth has no DC; HPF would just add ringing

    size_t n_out = 0, sync_off = 0;
    int polarity = -1;
    int rc = modem_iq_to_bits(iq, n_pairs, &p,
                              0, /*sync_max_ham=*/0, /*min_offset=*/0,
                              out_bits, &n_out, &sync_off, &polarity);

    int fails = 0;
    fails += check(rc == 0, "clean: modem_iq_to_bits found ASM");
    if (rc == 0) {
        // ASM should be in the preamble's neighbourhood. Allow a few
        // bits of slack for M&M settling.
        size_t expected = preamble_bits;
        long delta = (long) sync_off - (long) expected;
        if (delta < 0) delta = -delta;
        fails += check(delta <= 6, "clean: ASM offset near expected");

        // Post-ASM bits should reproduce the payload exactly. The bits
        // out_bits[0..31] are the ASM itself; payload starts at index 32.
        size_t mismatches = 0;
        size_t check_bits = payload_bits;
        if (n_out < 32 + check_bits) check_bits = n_out >= 32 ? n_out - 32 : 0;
        for (size_t i = 0; i < check_bits; ++i) {
            uint8_t got = out_bits[32 + i] & 1u;
            uint8_t want = bits[preamble_bits + 32 + i] & 1u;
            if (got != want) ++mismatches;
        }
        char msg[128];
        snprintf(msg, sizeof msg, "clean: payload BER = %zu/%zu",
                 mismatches, check_bits);
        fails += check(mismatches == 0, msg);
    }

    free(bits); free(iq); free(out_bits);
    return fails;
}

static int test_silence(void)
{
    const int samp_rate = 48000;
    const int bit_rate  = 9600;
    const size_t n_pairs = (size_t) samp_rate;  // 1 s of silence
    int16_t *iq = calloc(n_pairs * 2, sizeof(int16_t));
    uint8_t *out_bits = malloc(n_pairs);
    if (!iq || !out_bits) { free(iq); free(out_bits); return 1; }

    modem_params_t p;
    modem_params_defaults(&p);
    p.samp_rate = samp_rate;
    p.bit_rate  = bit_rate;
    p.rx_disable_dc_block = 1;

    size_t n_out = 0, sync_off = 0;
    int polarity = -1;
    int rc = modem_iq_to_bits(iq, n_pairs, &p,
                              0, /*sync_max_ham=*/0, 0,
                              out_bits, &n_out, &sync_off, &polarity);

    int fails = 0;
    fails += check(rc != 0, "silence: no false ASM on zero IQ");

    free(iq); free(out_bits);
    return fails;
}

// Run both demods on the same synthesized signal at a moderate SNR and
// confirm the IQ chain isn't strictly worse than the PCM chain. This is
// a non-regression guard, not a sensitivity comparison — sensitivity
// numbers belong on real-RF captures, not synthetic AWGN at a single
// SNR.
static int test_iq_not_worse_than_pcm(void)
{
    const int samp_rate = 48000;
    const int bit_rate  = 9600;
    const size_t preamble_bits = 64;
    const size_t payload_bits  = 8 * 128;
    const size_t total_bits = preamble_bits + 32 + payload_bits;
    int sps = samp_rate / bit_rate;
    size_t n_pairs = total_bits * (size_t) sps;
    const double snr_db = 15.0;  // high-ish — both chains should be near-clean

    uint8_t *bits     = malloc(total_bits);
    int16_t *iq       = malloc(n_pairs * 2 * sizeof(int16_t));
    int16_t *pcm      = malloc(n_pairs * sizeof(int16_t));
    uint8_t *out_iq   = malloc(total_bits + 1024);
    uint8_t *out_pcm  = malloc(total_bits + 1024);
    if (!bits || !iq || !pcm || !out_iq || !out_pcm) {
        free(bits); free(iq); free(pcm); free(out_iq); free(out_pcm);
        return 1;
    }

    size_t nb = build_test_frame(bits, total_bits, preamble_bits, payload_bits);
    mod_params_t mp = { samp_rate, bit_rate, 0.5, 0.5 };
    cpfsk_modulate(bits, nb, &mp, iq);
    add_awgn(iq, n_pairs, mp.amplitude, snr_db);
    fm_discriminate(iq, n_pairs, pcm, 25000.0, samp_rate);

    modem_params_t p;
    modem_params_defaults(&p);
    p.samp_rate = samp_rate;
    p.bit_rate  = bit_rate;
    p.rx_disable_dc_block = 1;

    size_t n_iq = 0,  iq_off = 0;  int iq_pol = -1;
    size_t n_pcm = 0, pcm_off = 0; int pcm_pol = -1;
    int rc_iq  = modem_iq_to_bits(iq, n_pairs, &p, 0, 0, 0,
                                  out_iq, &n_iq, &iq_off, &iq_pol);
    int rc_pcm = modem_pcm16_to_bits(pcm, n_pairs, &p, 0, 0, 0,
                                     out_pcm, &n_pcm, &pcm_off, &pcm_pol);

    int fails = 0;
    fails += check(rc_pcm == 0, "ab: PCM chain found ASM at SNR=15 dB");
    fails += check(rc_iq  == 0, "ab: IQ chain found ASM at SNR=15 dB");

    if (rc_iq == 0 && rc_pcm == 0) {
        size_t check_bits = payload_bits;
        if (n_iq < 32 + check_bits)  check_bits = n_iq  >= 32 ? n_iq  - 32 : 0;
        if (n_pcm < 32 + check_bits) check_bits = n_pcm >= 32 ? n_pcm - 32 : 0;
        size_t mis_iq = 0, mis_pcm = 0;
        for (size_t i = 0; i < check_bits; ++i) {
            uint8_t want = bits[preamble_bits + 32 + i] & 1u;
            if ((out_iq[32 + i]  & 1u) != want) ++mis_iq;
            if ((out_pcm[32 + i] & 1u) != want) ++mis_pcm;
        }
        char msg[128];
        snprintf(msg, sizeof msg,
                 "ab: IQ BER %zu/%zu vs PCM BER %zu/%zu",
                 mis_iq, check_bits, mis_pcm, check_bits);
        // "Not strictly worse" — allow some slack since synthetic AWGN
        // can occasionally tilt either way. Generous ceiling: IQ BER
        // can be up to 4 errors over PCM before we call regression.
        fails += check(mis_iq <= mis_pcm + 4, msg);
    }

    free(bits); free(iq); free(pcm); free(out_iq); free(out_pcm);
    return fails;
}

// Low-SNR A/B at 5 dB — the regime where symbol-rate differential
// matters. The IQ chain should keep BER under ~10 % of payload; the
// PCM chain is allowed to drop sync at this floor (sync robustness
// is a separate axis from BER-after-sync).
static int test_low_snr_ab(void)
{
    const int samp_rate = 48000;
    const int bit_rate  = 9600;
    const size_t preamble_bits = 128;
    const size_t payload_bits  = 8 * 200;
    const size_t total_bits = preamble_bits + 32 + payload_bits;
    int sps = samp_rate / bit_rate;
    size_t n_pairs = total_bits * (size_t) sps;
    const double snr_db = 5.0;

    uint8_t *bits     = malloc(total_bits);
    int16_t *iq       = malloc(n_pairs * 2 * sizeof(int16_t));
    int16_t *pcm      = malloc(n_pairs * sizeof(int16_t));
    uint8_t *out_iq   = malloc(total_bits + 1024);
    uint8_t *out_pcm  = malloc(total_bits + 1024);
    if (!bits || !iq || !pcm || !out_iq || !out_pcm) {
        free(bits); free(iq); free(pcm); free(out_iq); free(out_pcm);
        return 1;
    }

    size_t nb = build_test_frame(bits, total_bits, preamble_bits, payload_bits);
    mod_params_t mp = { samp_rate, bit_rate, 0.5, 0.5 };
    cpfsk_modulate(bits, nb, &mp, iq);
    add_awgn(iq, n_pairs, mp.amplitude, snr_db);
    fm_discriminate(iq, n_pairs, pcm, 25000.0, samp_rate);

    modem_params_t p;
    modem_params_defaults(&p);
    p.samp_rate = samp_rate;
    p.bit_rate  = bit_rate;
    p.rx_disable_dc_block = 1;

    size_t n_iq = 0,  iq_off = 0;  int iq_pol = -1;
    size_t n_pcm = 0, pcm_off = 0; int pcm_pol = -1;
    int rc_iq  = modem_iq_to_bits(iq, n_pairs, &p, 0, 4, 0,
                                  out_iq, &n_iq, &iq_off, &iq_pol);
    int rc_pcm = modem_pcm16_to_bits(pcm, n_pairs, &p, 0, 4, 0,
                                     out_pcm, &n_pcm, &pcm_off, &pcm_pol);

    int fails = 0;
    fails += check(rc_iq == 0, "lowsnr: IQ chain found ASM at SNR=5 dB");
    if (rc_pcm != 0) {
        printf("INFO  lowsnr: PCM chain did not sync at SNR=5 dB\n");
    }

    if (rc_iq == 0) {
        size_t check_bits = payload_bits;
        if (n_iq < 32 + check_bits) check_bits = n_iq >= 32 ? n_iq - 32 : 0;
        size_t mis_iq = 0, mis_pcm = 0;
        for (size_t i = 0; i < check_bits; ++i) {
            uint8_t want = bits[preamble_bits + 32 + i] & 1u;
            if ((out_iq[32 + i] & 1u) != want) ++mis_iq;
            if (rc_pcm == 0 && (out_pcm[32 + i] & 1u) != want) ++mis_pcm;
        }
        char msg[160];
        if (rc_pcm == 0) {
            snprintf(msg, sizeof msg,
                     "lowsnr: IQ BER %zu/%zu vs PCM BER %zu/%zu",
                     mis_iq, check_bits, mis_pcm, check_bits);
        } else {
            snprintf(msg, sizeof msg,
                     "lowsnr: IQ BER %zu/%zu (PCM no-sync)",
                     mis_iq, check_bits);
        }
        size_t ceiling = check_bits / 10;
        fails += check(mis_iq <= ceiling, msg);
    }

    free(bits); free(iq); free(pcm); free(out_iq); free(out_pcm);
    return fails;
}

int main(void)
{
    g_rng = (uint32_t) time(NULL);
    printf("modem_iq_selftest: rng_seed=%u\n", g_rng);
    int fails = 0;
    fails += test_clean_decode();
    fails += test_silence();
    fails += test_iq_not_worse_than_pcm();
    fails += test_low_snr_ab();
    printf("modem_iq_selftest: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
