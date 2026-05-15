/*

    Simple Satellite Operations  modem_viterbi.c

    MSK-MLSE Viterbi over the IQ baseband. See modem_viterbi.h for the
    full pipeline.

    Copyright (C) 2026  Johnathan K Burchill

    GPLv3 or later.
*/

#include "modem_viterbi.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ASM_BIG_ENDIAN_U32 0x930B51DEu

// Same lowest-Hamming ASM finder modem_iq.c uses. Duplicated here so
// this translation unit stays independent of modem_iq.c.
static size_t vit_find_asm_best(const uint8_t *bits, size_t n_bits,
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

int modem_iq_viterbi_to_bits(const int16_t *iq_pairs, size_t n_pairs,
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

    // 1. AGC on the complex baseband.
    double sum_sq = 0.0;
    for (size_t i = 0; i < n_pairs; ++i) {
        double I = (double) iq_pairs[i * 2 + 0];
        double Q = (double) iq_pairs[i * 2 + 1];
        sum_sq += I * I + Q * Q;
    }
    double rms = sqrt(sum_sq / (double) n_pairs);
    if (rms < 1.0) rms = 1.0;
    double agc_inv = 1.0 / rms;

    // 2. Boxcar matched filter on I and Q. mf_len = n_pairs - sps + 1.
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

    // 3. Symbol-rate differential phase: dphi[i] = arg(z_mf[i+sps] · conj(z_mf[i])).
    if (mf_len <= (size_t) sps) { free(Imf); free(Qmf); return -1; }
    size_t df_len = mf_len - (size_t) sps;
    float *dphi = (float *) malloc(df_len * sizeof(float));
    if (dphi == NULL) { free(Imf); free(Qmf); return -1; }
    for (size_t i = 0; i < df_len; ++i) {
        double I0 = (double) Imf[i],             Q0 = (double) Qmf[i];
        double I1 = (double) Imf[i + (size_t) sps],
               Q1 = (double) Qmf[i + (size_t) sps];
        dphi[i] = (float) atan2(Q1 * I0 - I1 * Q0,
                                I1 * I0 + Q1 * Q0);
    }

    // 4. Mueller-Müller timing on dphi (same loop as modem_iq.c). The
    //    side product is the symbol-rate complex sample y[n] = MF(pos+sps),
    //    fractionally interpolated, for the Viterbi to consume.
    size_t max_strobes = df_len / (size_t) sps + 1;
    float *yI = (float *) malloc(max_strobes * sizeof(float));
    float *yQ = (float *) malloc(max_strobes * sizeof(float));
    if (yI == NULL || yQ == NULL) {
        free(yI); free(yQ); free(dphi); free(Imf); free(Qmf);
        return -1;
    }
    size_t n_sym = 0;
    {
        const double sps_d = (double) sps;
        const double Kp = 0.10;
        const double max_step = sps_d * 0.25;
        double pos = sps_d;
        double prev_y = 0.0, prev_dec = 0.0;
        int have_prev = 0;
        while (pos + 1.0 < (double) df_len && n_sym < max_strobes) {
            size_t i = (size_t) pos;
            double frac = pos - (double) i;
            double y = (double) dphi[i] * (1.0 - frac)
                     + (double) dphi[i + 1] * frac;
            double dec = (y >= 0.0) ? 1.0 : -1.0;
            size_t j = i + (size_t) sps;
            if (j + 1 < mf_len) {
                yI[n_sym] = (float)((double) Imf[j] * (1.0 - frac)
                                  + (double) Imf[j + 1] * frac);
                yQ[n_sym] = (float)((double) Qmf[j] * (1.0 - frac)
                                  + (double) Qmf[j + 1] * frac);
            } else if (j < mf_len) {
                yI[n_sym] = Imf[j];
                yQ[n_sym] = Qmf[j];
            } else {
                break;
            }
            n_sym++;
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
    }
    free(dphi); free(Imf); free(Qmf);
    if (n_sym < 64) { free(yI); free(yQ); return -1; }

    // 5. Carrier-phase estimate via fourth-power. For MSK at h=1/2 the
    //    quartic operation removes the bit-by-bit ±π/2 modulation
    //    (since 4·(±π/2) ≡ 0 mod 2π) and the remainder is 4·φ₀ + n,
    //    so a sum-then-arg recovers φ₀ up to a π/2 ambiguity. That
    //    residual ambiguity is harmless: the Viterbi state graph is
    //    rotationally symmetric, so re-labelling the absolute state
    //    leaves transitions (and hence decoded bits) untouched.
    double sum4_re = 0.0, sum4_im = 0.0;
    for (size_t n = 0; n < n_sym; ++n) {
        double I = (double) yI[n], Q = (double) yQ[n];
        double I2 = I * I - Q * Q;
        double Q2 = 2.0 * I * Q;
        double I4 = I2 * I2 - Q2 * Q2;
        double Q4 = 2.0 * I2 * Q2;
        sum4_re += I4;
        sum4_im += Q4;
    }
    double phi_0 = atan2(sum4_im, sum4_re) / 4.0;
    double cos_p = cos(phi_0), sin_p = sin(phi_0);
    for (size_t n = 0; n < n_sym; ++n) {
        double I = (double) yI[n], Q = (double) yQ[n];
        double Ir =  I * cos_p + Q * sin_p;
        double Qr = -I * sin_p + Q * cos_p;
        yI[n] = (float) Ir;
        yQ[n] = (float) Qr;
    }

    // 6. 4-state Viterbi MLSE. State s ∈ {0,1,2,3} represents phase
    //    s·π/2. Predecessor under bit=1 (+π/2 transition) is (s+3)%4;
    //    under bit=0 (-π/2) is (s+1)%4. Branch metric Re{y·exp(-j·s·π/2)}
    //    simplifies to bm[0]=Re{y}, bm[1]=Im{y}, bm[2]=-Re{y}, bm[3]=-Im{y}.
    //    Path metrics are renormalised each step (subtract the running
    //    min) to keep floats from drifting on long captures.
    uint8_t *bt_pred = (uint8_t *) malloc(n_sym * 4u);
    uint8_t *bt_bit  = (uint8_t *) malloc(n_sym * 4u);
    if (bt_pred == NULL || bt_bit == NULL) {
        free(bt_pred); free(bt_bit); free(yI); free(yQ);
        return -1;
    }
    float pm[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
    float pmn[4];
    for (size_t n = 0; n < n_sym; ++n) {
        float I = yI[n], Q = yQ[n];
        float bm[4] = { I, Q, -I, -Q };
        for (int s = 0; s < 4; ++s) {
            int pa = (s + 3) & 3;
            int pb = (s + 1) & 3;
            if (pm[pa] >= pm[pb]) {
                pmn[s] = pm[pa] + bm[s];
                bt_pred[n * 4u + (size_t) s] = (uint8_t) pa;
                bt_bit [n * 4u + (size_t) s] = 1u;
            } else {
                pmn[s] = pm[pb] + bm[s];
                bt_pred[n * 4u + (size_t) s] = (uint8_t) pb;
                bt_bit [n * 4u + (size_t) s] = 0u;
            }
        }
        float mn = pmn[0];
        for (int s = 1; s < 4; ++s) if (pmn[s] < mn) mn = pmn[s];
        for (int s = 0; s < 4; ++s) pm[s] = pmn[s] - mn;
    }

    // Traceback from the highest-metric end state.
    int s_end = 0;
    for (int s = 1; s < 4; ++s) if (pm[s] > pm[s_end]) s_end = s;
    uint8_t *bits_raw = (uint8_t *) malloc(n_sym);
    if (bits_raw == NULL) {
        free(bt_pred); free(bt_bit); free(yI); free(yQ);
        return -1;
    }
    {
        int s = s_end;
        for (size_t k = n_sym; k-- > 0; ) {
            bits_raw[k] = bt_bit[k * 4u + (size_t) s] & 1u;
            s = bt_pred[k * 4u + (size_t) s];
        }
    }
    free(bt_pred); free(bt_bit); free(yI); free(yQ);

    // 7. ASM search under both polarities — same convention as
    //    modem_iq_to_bits so callers can swap chains without
    //    re-thinking how the polarity flag flows.
    uint8_t *tmp_bits = (uint8_t *) malloc(n_sym ? n_sym : 1);
    if (tmp_bits == NULL) { free(bits_raw); return -1; }
    int polarities[2];
    polarities[0] = invert_polarity ? 1 : 0;
    polarities[1] = invert_polarity ? 0 : 1;

    size_t best_sync = (size_t) -1;
    int best_polarity = -1;

    for (int pi = 0; pi < 2 && best_polarity < 0; ++pi) {
        int this_invert = polarities[pi];
        for (size_t i = 0; i < n_sym; ++i) {
            uint8_t b = bits_raw[i] & 1u;
            if (this_invert) b ^= 1u;
            tmp_bits[i] = b;
        }
        int this_ham = 33;
        size_t off = vit_find_asm_best(tmp_bits, n_sym,
                                       ASM_BIG_ENDIAN_U32, sync_max_ham,
                                       min_bit_offset, &this_ham);
        if (off != (size_t) -1) {
            size_t copy_bits = n_sym - off;
            memcpy(out_bits, tmp_bits + off, copy_bits);
            *n_bits_out = copy_bits;
            best_sync = off;
            best_polarity = this_invert;
        }
    }
    if (polarity_used) *polarity_used = best_polarity;

    free(tmp_bits);
    free(bits_raw);

    if (best_polarity < 0) {
        if (sync_bit_offset) *sync_bit_offset = (size_t) -1;
        return -1;
    }
    if (sync_bit_offset) *sync_bit_offset = best_sync;
    return 0;
}
