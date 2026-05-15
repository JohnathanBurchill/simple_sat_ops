/*

    Simple Satellite Operations  modem_iq.c

    Quasi-coherent IQ-domain demod. See modem_iq.h for the pipeline.
    The M&M + slicer + ASM-search logic at the tail mirrors what
    modem_pcm16_to_bits does — keeping the bit-detection identical
    means any improvement we measure against the FM-audio path comes
    purely from doing the matched filtering on IQ rather than on the
    discriminator output.

    Copyright (C) 2026  Johnathan K Burchill

    GPLv3 or later.
*/

#include "modem_iq.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ASM_BIG_ENDIAN_U32 0x930B51DEu

// Lowest-Hamming ASM finder. Duplicated rather than imported from
// modem.c so this module is independent. Same algorithm as
// find_u32_pattern_best in modem.c.
static size_t iq_find_asm_best(const uint8_t *bits, size_t n_bits,
                               uint32_t needle, int max_ham,
                               size_t min_offset, int *out_ham)
{
    if (out_ham) *out_ham = 33;
    if (n_bits < 32) return (size_t) -1;
    size_t start = min_offset;
    if (start > n_bits - 32) return (size_t) -1;
    uint32_t window = 0;
    for (size_t i = 0; i < 32; ++i) {
        window = (window << 1) | (bits[start + i] & 1u);
    }
    int best_ham = 33;
    size_t best_off = (size_t) -1;
    int h = (int) __builtin_popcount(window ^ needle);
    if (h <= max_ham) {
        best_ham = h;
        best_off = start;
    }
    for (size_t i = 32; i < n_bits - start; ++i) {
        window = (window << 1) | (bits[start + i] & 1u);
        h = (int) __builtin_popcount(window ^ needle);
        if (h <= max_ham && h < best_ham) {
            best_ham = h;
            best_off = start + i - 31;
            if (best_ham == 0) break;
        }
    }
    if (out_ham && best_off != (size_t) -1) *out_ham = best_ham;
    return best_off;
}

int modem_iq_to_bits(const int16_t *iq_pairs, size_t n_pairs,
                     const modem_params_t *p,
                     int invert_polarity,
                     int sync_max_ham,
                     size_t min_bit_offset,
                     uint8_t *out_bits, size_t *n_bits_out,
                     size_t *sync_bit_offset,
                     int *polarity_used)
{
    if (iq_pairs == NULL || p == NULL
        || out_bits == NULL || n_bits_out == NULL) {
        return -1;
    }
    if (p->samp_rate <= 0 || p->bit_rate <= 0
        || p->samp_rate % p->bit_rate != 0) {
        return -1;
    }
    int sps = p->samp_rate / p->bit_rate;
    if (sps <= 1 || n_pairs < (size_t) sps * 32u) return -1;

    // 1. AGC on complex baseband. RMS magnitude = sqrt(<I² + Q²>).
    //    Floor avoids division blow-up on near-silent input.
    double sum_sq = 0.0;
    for (size_t i = 0; i < n_pairs; ++i) {
        double I = (double) iq_pairs[i * 2 + 0];
        double Q = (double) iq_pairs[i * 2 + 1];
        sum_sq += I * I + Q * Q;
    }
    double rms = sqrt(sum_sq / (double) n_pairs);
    if (rms < 1.0) rms = 1.0;
    double agc_inv = 1.0 / rms;

    // 2. Matched filter on I and Q separately. Sliding boxcar of length
    //    sps — same NRZ-matched shape the PCM chain uses, but applied
    //    BEFORE the non-linear arg() collapses I,Q into a scalar.
    //    Output length = n_pairs - sps + 1 complex samples at sample rate.
    size_t mf_len = n_pairs - (size_t) sps + 1;
    float *Imf = (float *) malloc(mf_len * sizeof(float));
    float *Qmf = (float *) malloc(mf_len * sizeof(float));
    if (Imf == NULL || Qmf == NULL) {
        free(Imf); free(Qmf);
        return -1;
    }
    {
        const double inv_sps = 1.0 / (double) sps;
        double sumI = 0.0, sumQ = 0.0;
        for (int k = 0; k < sps; ++k) {
            sumI += (double) iq_pairs[k * 2 + 0];
            sumQ += (double) iq_pairs[k * 2 + 1];
        }
        Imf[0] = (float)(sumI * agc_inv * inv_sps);
        Qmf[0] = (float)(sumQ * agc_inv * inv_sps);
        for (size_t i = 1; i < mf_len; ++i) {
            sumI -= (double) iq_pairs[(i - 1) * 2 + 0];
            sumQ -= (double) iq_pairs[(i - 1) * 2 + 1];
            sumI += (double) iq_pairs[(i + (size_t) sps - 1) * 2 + 0];
            sumQ += (double) iq_pairs[(i + (size_t) sps - 1) * 2 + 1];
            Imf[i] = (float)(sumI * agc_inv * inv_sps);
            Qmf[i] = (float)(sumQ * agc_inv * inv_sps);
        }
    }

    // 3. Differential phase: y[k] = arg(z[k] * conj(z[k-1])). The first
    //    output is at index 1; index 0 is undefined (no prev). Use atan2
    //    so the result lies in [-π, π].
    //
    //    For ideal continuous-phase FSK at modulation index h, the
    //    per-sample phase advance is ±πh/sps and integrates to ±πh per
    //    symbol. At h=0.5 (MSK) a perfectly-strobed symbol differential
    //    is ±π/2 — well separated from 0.
    size_t df_len = mf_len > 0 ? mf_len - 1 : 0;
    float *dphi = (float *) malloc((df_len > 0 ? df_len : 1) * sizeof(float));
    if (dphi == NULL) { free(Imf); free(Qmf); return -1; }
    for (size_t i = 0; i < df_len; ++i) {
        double I0 = (double) Imf[i],     Q0 = (double) Qmf[i];
        double I1 = (double) Imf[i + 1], Q1 = (double) Qmf[i + 1];
        // z1 * conj(z0) = (I1 + jQ1)(I0 - jQ0)
        //               = (I1 I0 + Q1 Q0) + j(Q1 I0 - I1 Q0)
        double real_p = I1 * I0 + Q1 * Q0;
        double imag_p = Q1 * I0 - I1 * Q0;
        dphi[i] = (float) atan2(imag_p, real_p);
    }
    free(Imf); free(Qmf);

    // 4. Mueller-Müller decision-directed timing recovery on dphi.
    //    Identical loop to modem_pcm16_to_bits — keeps the bit-detection
    //    half of both chains apples-to-apples so any A/B difference is
    //    attributable to the front end.
    size_t max_strobes = df_len / (size_t) sps + 1;
    float *strobe = (float *) malloc(max_strobes * sizeof(float));
    if (strobe == NULL) { free(dphi); return -1; }
    {
        const double sps_d = (double) sps;
        const double Kp = 0.10;
        const double max_step = sps_d * 0.25;
        double pos = sps_d;
        double prev_y = 0.0, prev_dec = 0.0;
        int have_prev = 0;
        size_t n = 0;
        while (pos + 1.0 < (double) df_len && n < max_strobes) {
            size_t i = (size_t) pos;
            double frac = pos - (double) i;
            double y = (double) dphi[i] * (1.0 - frac)
                     + (double) dphi[i + 1] * frac;
            double dec = (y >= 0.0) ? 1.0 : -1.0;
            strobe[n++] = (float) y;
            double advance = sps_d;
            if (have_prev) {
                double ted = prev_dec * y - dec * prev_y;
                double adj = Kp * ted;
                if      (adj >  max_step) adj =  max_step;
                else if (adj < -max_step) adj = -max_step;
                advance += adj;
            }
            pos += advance;
            prev_y = y;
            prev_dec = dec;
            have_prev = 1;
        }
        max_strobes = n;
    }
    free(dphi);

    // 5. Slice + ASM search under each polarity. The radio-side FM
    //    convention can invert the sign of the differential phase
    //    (just as it inverts the FM audio); brute-force both and keep
    //    the lowest-Hamming match.
    uint8_t *tmp_bits = (uint8_t *) malloc(max_strobes ? max_strobes : 1);
    if (tmp_bits == NULL) { free(strobe); return -1; }
    int polarities[2];
    polarities[0] = invert_polarity ? 1 : 0;
    polarities[1] = invert_polarity ? 0 : 1;

    size_t best_sync = (size_t) -1;
    int best_polarity = -1;

    for (int pi = 0; pi < 2 && best_polarity < 0; ++pi) {
        int this_invert = polarities[pi];
        for (size_t i = 0; i < max_strobes; ++i) {
            int bit;
            if (this_invert) bit = (strobe[i] < 0.0f) ? 1 : 0;
            else             bit = (strobe[i] > 0.0f) ? 1 : 0;
            tmp_bits[i] = (uint8_t) bit;
        }
        int this_ham = 33;
        size_t off = iq_find_asm_best(tmp_bits, max_strobes,
                                      ASM_BIG_ENDIAN_U32, sync_max_ham,
                                      min_bit_offset, &this_ham);
        if (off != (size_t) -1) {
            size_t copy_bits = max_strobes - off;
            memcpy(out_bits, tmp_bits + off, copy_bits);
            *n_bits_out = copy_bits;
            best_sync = off;
            best_polarity = this_invert;
        }
    }
    if (polarity_used) *polarity_used = best_polarity;

    free(tmp_bits);
    free(strobe);

    if (best_polarity < 0) {
        if (sync_bit_offset) *sync_bit_offset = (size_t) -1;
        return -1;
    }
    if (sync_bit_offset) *sync_bit_offset = best_sync;
    return 0;
}
