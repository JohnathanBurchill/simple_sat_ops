/*

    Simple Satellite Operations  modem_viterbi.c

    MSK-MLSE Viterbi over the IQ baseband. See modem_viterbi.h for the
    full pipeline.

    Copyright (C) 2026  Johnathan K Burchill

    GPLv3 or later.
*/

#include "modem_viterbi.h"

#include "asm_search.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

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
    if (sps <= 1 || n_pairs < (size_t) sps * (size_t) 32) return -1;

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

    // 3. Symbol-rate complex differential and carrier-frequency-offset
    //    (per-symbol bias) estimate. The complex differential
    //        d[i] = z_mf[i+sps] · conj(z_mf[i])
    //    has phase bias ± π/2 for MSK at h=1/2. Squaring d[i] strips
    //    the ±π/2 data modulation (lands at 2·bias ± π = 2·bias + π,
    //    a single phase regardless of bit), so atan2(-Σ d², -Σ d²)
    //    recovers 2·bias and half gives the bias mod π. The residual
    //    π-ambiguity propagates to a polarity flip in the decoded bits
    //    and is resolved by the ASM-search polarity loop in step 8.
    if (mf_len <= (size_t) sps) { free(Imf); free(Qmf); return -1; }
    size_t df_len = mf_len - (size_t) sps;
    float *dphi = (float *) malloc(df_len * sizeof(float));
    if (dphi == NULL) { free(Imf); free(Qmf); return -1; }

    double s2r = 0.0, s2i = 0.0;
    for (size_t i = 0; i < df_len; ++i) {
        double I0 = (double) Imf[i],             Q0 = (double) Qmf[i];
        double I1 = (double) Imf[i + (size_t) sps],
               Q1 = (double) Qmf[i + (size_t) sps];
        double a  = I1 * I0 + Q1 * Q0;
        double b  = Q1 * I0 - I1 * Q0;
        s2r += a * a - b * b;
        s2i += 2.0 * a * b;
    }
    // atan2(0,0) is undefined; an all-zero / pure-DC window gives
    // s2r==s2i==0, so take zero bias there instead of the libm corner case.
    double bias_mf = (s2r == 0.0 && s2i == 0.0)
                   ? 0.0 : atan2(-s2i, -s2r) / 2.0;

    // The MF-domain estimator has a small structural noise floor
    // (~0.005 rad on clean MSK) from the matched filter's transient
    // response at symbol boundaries. That is tiny per symbol but
    // accumulates across the window and corrupts the slicer that
    // M&M's TED relies on. Apply the correction only when it is
    // unambiguously larger than the noise floor; below the threshold
    // the carrier offset is small enough that raw dphi is already on
    // the right side of zero for M&M to work.
    const double BIAS_APPLY_RAD = 0.05;     // ~3°/sym, ~75 Hz @ sps=5/48k
    int apply_dphi_bias = (bias_mf >  BIAS_APPLY_RAD)
                        || (bias_mf < -BIAS_APPLY_RAD);
    double cb = cos(-bias_mf), sb = sin(-bias_mf);

    // dphi with bias rotated out (gated by threshold), ready for M&M.
    for (size_t i = 0; i < df_len; ++i) {
        double I0 = (double) Imf[i],             Q0 = (double) Qmf[i];
        double I1 = (double) Imf[i + (size_t) sps],
               Q1 = (double) Qmf[i + (size_t) sps];
        double a = I1 * I0 + Q1 * Q0;
        double b = Q1 * I0 - I1 * Q0;
        if (apply_dphi_bias) {
            double a_rot = a * cb - b * sb;
            double b_rot = a * sb + b * cb;
            dphi[i] = (float) atan2(b_rot, a_rot);
        } else {
            dphi[i] = (float) atan2(b, a);
        }
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

    // 5. Per-symbol derotation by n·bias. The carrier-frequency
    //    offset leaves an accumulated n·bias of phase rotation on
    //    the symbol-rate complex samples y[n] that feed the Viterbi.
    //    Re-estimate bias from y[n+1]·conj(y[n]) instead of reusing
    //    the MF-domain estimate from step 3 — once M&M has placed the
    //    strobes near symbol centres the differential is cleaner
    //    (no boxcar-MF transient ISI), so the 2nd-power phase here is
    //    a tighter estimate. Same threshold logic as step 3: on clean
    //    AWGN the estimate is essentially zero but a tiny residual
    //    would still accumulate to many radians across the Viterbi
    //    window, so don't apply unless we have confidence.
    if (n_sym >= 2) {
        double s2r_y = 0.0, s2i_y = 0.0;
        for (size_t n = 0; n + 1 < n_sym; ++n) {
            double I0 = (double) yI[n],     Q0 = (double) yQ[n];
            double I1 = (double) yI[n + 1], Q1 = (double) yQ[n + 1];
            double a  = I1 * I0 + Q1 * Q0;
            double b  = Q1 * I0 - I1 * Q0;
            s2r_y += a * a - b * b;
            s2i_y += 2.0 * a * b;
        }
        double bias_y = atan2(-s2i_y, -s2r_y) / 2.0;
        if (bias_y > BIAS_APPLY_RAD || bias_y < -BIAS_APPLY_RAD) {
            for (size_t n = 0; n < n_sym; ++n) {
                double ang = -(double) n * bias_y;
                double c   = cos(ang), s = sin(ang);
                double I   = (double) yI[n], Q = (double) yQ[n];
                yI[n] = (float)(I * c - Q * s);
                yQ[n] = (float)(I * s + Q * c);
            }
        }
    }

    // 6. Carrier-phase estimate via fourth-power. For MSK at h=1/2 the
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

    // 7. 4-state Viterbi MLSE. State s ∈ {0,1,2,3} represents phase
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

    // 8. ASM search under both polarities — same convention as
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
        size_t off = asm_find_best(tmp_bits, n_sym,
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
