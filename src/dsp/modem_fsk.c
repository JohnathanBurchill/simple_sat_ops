/*

    Simple Satellite Operations  modem_fsk.c

    FSK demod on int16 I,Q baseband. See modem_fsk.h for the full
    pipeline. Closely mirrors modem_pcm16_to_bits (modem.c) but
    operates on float buffers internally so we don't pay an int16
    round-trip on the FM-discriminated signal.

    Copyright (C) 2026  Johnathan K Burchill

    GPLv3 or later.
*/

#include "modem_fsk.h"

#include "asm_search.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// IQ low-pass filter (Hamming-windowed sinc) applied separately to
// I and Q before the FM discriminator. The atan2 inside the
// discriminator is nonlinear at medium SNR — out-of-band noise the
// downstream MF would otherwise reject ends up modulating into
// in-band noise through atan2's "click" behaviour, so trimming the
// noise bandwidth FIRST gains ~2-3 dB of effective SNR on real RF
// captures. Same approach gr_satellites uses (its quadrature_demod
// typically follows a bandpass filter).
//
// Default cutoff is 12 kHz — the empirical sweet spot on the
// 2026-05-15 RAO capture: tight enough to bring the t≈150/232
// near-TCA beacons cleanly through RS, wide enough that the FSK
// pulse shape (deviation 2400, baud 9600 → h=0.5 MSK) isn't
// distorted. Override with $FSK_IQ_LPF_HZ for sweeps.
#define FSK_IQ_LPF_LEN 31

// Build a Hamming-windowed sinc LPF kernel for the given cutoff at
// the given sample rate. fc_hz is the -6 dB cutoff; the transition
// band rolls off over roughly ±fs/(N+1) on either side. Returns the
// normalised kernel in `out` (DC gain = 1).
static void fsk_build_iq_lpf(double fc_hz, double fs, float out[FSK_IQ_LPF_LEN])
{
    int N = FSK_IQ_LPF_LEN;
    double sum = 0.0;
    for (int n = 0; n < N; ++n) {
        double t = (double) n - (double)(N - 1) / 2.0;
        double sinc_v;
        if (t == 0.0) {
            sinc_v = 1.0;
        } else {
            double x = 2.0 * fc_hz / fs * t;
            sinc_v = sin(M_PI * x) / (M_PI * x);
        }
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * (double) n / (double)(N - 1));
        double k = 2.0 * fc_hz / fs * sinc_v * w;
        out[n] = (float) k;
        sum += k;
    }
    if (sum != 0.0) {
        for (int n = 0; n < N; ++n) out[n] = (float)((double) out[n] / sum);
    }
}

// Read the LPF cutoff from $FSK_IQ_LPF_HZ if set, otherwise the
// default 12 kHz that empirically gave the cleanest decodes on the
// 2026-05-15 RAO capture (low-Doppler bursts at TCA). Cached after
// first lookup.
static double fsk_iq_lpf_cutoff_hz(void)
{
    static double cached = -1.0;
    if (cached < 0.0) {
        const char *env = getenv("FSK_IQ_LPF_HZ");
        if (env != NULL && *env != '\0') {
            cached = atof(env);
            if (cached < 1000.0 || cached > 22000.0) cached = 12000.0;
        } else {
            cached = 12000.0;
        }
    }
    return cached;
}


// =======================================================================
// Per-stage helpers. Each is a pure function over caller-supplied
// buffers; the orchestrator (modem_fsk_iq_to_bits_diag) owns the
// scratch allocations. Keeping these as static-internal lets the
// diagnostic entry point copy out intermediates without changing the
// public API. See the chain narrative at docs/decoding/DECODING.md.
// =======================================================================

// Stage 1: IQ low-pass filter (Hamming-windowed sinc, valid-only
// convolution). Outputs n_pairs - FSK_IQ_LPF_LEN + 1 samples.
static size_t fsk_stage_lpf(const int16_t *iq_pairs, size_t n_pairs,
                            int samp_rate,
                            float *out_i_lpf, float *out_q_lpf)
{
    if (n_pairs < (size_t) FSK_IQ_LPF_LEN + 2u) return 0;

    float kernel[FSK_IQ_LPF_LEN];
    fsk_build_iq_lpf(fsk_iq_lpf_cutoff_hz(), (double) samp_rate, kernel);

    size_t lpf_n = n_pairs - (size_t) FSK_IQ_LPF_LEN + 1u;
    for (size_t i = 0; i < lpf_n; ++i) {
        double accI = 0.0, accQ = 0.0;
        for (int k = 0; k < FSK_IQ_LPF_LEN; ++k) {
            double h = (double) kernel[k];
            accI += h * (double) iq_pairs[(i + (size_t) k) * 2 + 0];
            accQ += h * (double) iq_pairs[(i + (size_t) k) * 2 + 1];
        }
        out_i_lpf[i] = (float) accI;
        out_q_lpf[i] = (float) accQ;
    }
    return lpf_n;
}

// Stage 2: FM discriminator. fm[i] = arg(z[i+1] * conj(z[i])) — the
// differential phase between adjacent samples. Outputs lpf_n - 1.
static size_t fsk_stage_discriminate(const float *I_lpf, const float *Q_lpf,
                                     size_t lpf_n, float *out_fm)
{
    if (lpf_n < 2) return 0;
    size_t fm_n = lpf_n - 1;
    for (size_t i = 0; i < fm_n; ++i) {
        double I0 = (double) I_lpf[i],     Q0 = (double) Q_lpf[i];
        double I1 = (double) I_lpf[i + 1], Q1 = (double) Q_lpf[i + 1];
        out_fm[i] = (float) atan2(Q1 * I0 - I1 * Q0,
                                  I1 * I0 + Q1 * Q0);
    }
    return fm_n;
}

// Stage 3: 1-pole HPF (DC-block) on fm in place. Strips the carrier-
// offset / slow-Doppler DC pedestal that would bias the slicer.
static void fsk_stage_dc_block(float *fm, size_t fm_n, float alpha)
{
    if (fm_n == 0) return;
    float prev_x = 0.0f, prev_y = 0.0f;
    for (size_t i = 0; i < fm_n; ++i) {
        float x = fm[i];
        float y = x - prev_x + alpha * prev_y;
        fm[i] = y;
        prev_x = x;
        prev_y = y;
    }
}

// Stage 4: RMS AGC on fm in place. Brings the FSK signal to roughly
// unit-amplitude regardless of carrier strength, so downstream MF and
// slicer don't need a per-burst gain calibration.
static void fsk_stage_agc(float *fm, size_t fm_n)
{
    if (fm_n == 0) return;
    double sum_sq = 0.0;
    for (size_t i = 0; i < fm_n; ++i) {
        sum_sq += (double) fm[i] * (double) fm[i];
    }
    double rms = sqrt(sum_sq / (double) fm_n);
    if (rms < 1e-6) rms = 1e-6;
    double agc_inv = 1.0 / rms;
    for (size_t i = 0; i < fm_n; ++i) {
        fm[i] = (float)((double) fm[i] * agc_inv);
    }
}

// Stage 5: boxcar matched filter (running mean of length sps). Each
// output is the average instantaneous frequency over one symbol
// period. Output length = fm_n - sps + 1.
static size_t fsk_stage_matched_filter(const float *fm, size_t fm_n,
                                       int sps, float *out_mf)
{
    if (fm_n < (size_t) sps || sps < 1) return 0;
    size_t mf_n = fm_n - (size_t) sps + 1;
    const double inv_sps = 1.0 / (double) sps;
    double window = 0.0;
    for (int k = 0; k < sps; ++k) window += (double) fm[k];
    out_mf[0] = (float)(window * inv_sps);
    for (size_t i = 1; i < mf_n; ++i) {
        window -= (double) fm[i - 1];
        window += (double) fm[i + (size_t) sps - 1];
        out_mf[i] = (float)(window * inv_sps);
    }
    return mf_n;
}

// Stage 6: Gardner TED + Farrow cubic interpolator. Picks one
// symbol-rate sample per symbol, tracking timing drift. Returns the
// number of strobes written into out_strobes (and out_strobe_t when
// non-NULL — fractional sample index of each strobe).
static size_t fsk_stage_gardner_farrow(const float *mf, size_t mf_n,
                                       int sps,
                                       float *out_strobes,
                                       double *out_strobe_t,
                                       size_t max_strobes)
{
    if (mf_n < 4 || max_strobes == 0) return 0;
    const double sps_d   = (double) sps;
    const double half    = sps_d * 0.5;
    const double Kp      = 0.05;
    const double max_step = sps_d * 0.10;

    double pos = sps_d;
    double prev_y = 0.0;
    int    have_prev = 0;
    size_t n_sym = 0;
    while (pos + sps_d + 2.0 < (double) mf_n && pos > 1.0
           && n_sym < max_strobes) {
        size_t i_c  = (size_t) pos;
        double mu_c = pos - (double) i_c;
        if (i_c < 1 || i_c + 2 >= mf_n) break;
        double f_m1c = -mu_c * (mu_c - 1.0) * (mu_c - 2.0) / 6.0;
        double f_0c  =  (mu_c + 1.0) * (mu_c - 1.0) * (mu_c - 2.0) / 2.0;
        double f_1c  = -(mu_c + 1.0) * mu_c * (mu_c - 2.0) / 2.0;
        double f_2c  =  (mu_c + 1.0) * mu_c * (mu_c - 1.0) / 6.0;
        double y_curr = (double) mf[i_c - 1] * f_m1c
                      + (double) mf[i_c]     * f_0c
                      + (double) mf[i_c + 1] * f_1c
                      + (double) mf[i_c + 2] * f_2c;
        out_strobes[n_sym] = (float) y_curr;
        if (out_strobe_t != NULL) out_strobe_t[n_sym] = pos;
        ++n_sym;

        if (have_prev) {
            double pos_mid = pos - half;
            if (pos_mid > 1.0 && pos_mid + 2.0 < (double) mf_n) {
                size_t i_m  = (size_t) pos_mid;
                double mu_m = pos_mid - (double) i_m;
                double f_m1m = -mu_m * (mu_m - 1.0) * (mu_m - 2.0) / 6.0;
                double f_0m  =  (mu_m + 1.0) * (mu_m - 1.0) * (mu_m - 2.0) / 2.0;
                double f_1m  = -(mu_m + 1.0) * mu_m * (mu_m - 2.0) / 2.0;
                double f_2m  =  (mu_m + 1.0) * mu_m * (mu_m - 1.0) / 6.0;
                double y_mid = (double) mf[i_m - 1] * f_m1m
                             + (double) mf[i_m]     * f_0m
                             + (double) mf[i_m + 1] * f_1m
                             + (double) mf[i_m + 2] * f_2m;
                double ted = y_mid * (y_curr - prev_y);
                double adj = -Kp * ted;
                if      (adj >  max_step) adj =  max_step;
                else if (adj < -max_step) adj = -max_step;
                pos += sps_d + adj;
            } else {
                pos += sps_d;
            }
        } else {
            pos += sps_d;
        }
        prev_y = y_curr;
        have_prev = 1;
    }
    return n_sym;
}

// Stage 7a: slice strobes to hard-decision bits under the given
// polarity. invert=0 → bit = (s > 0); invert=1 → bit = (s < 0).
static void fsk_stage_slice(const float *strobes, size_t n_strobes,
                            int invert, uint8_t *out_bits)
{
    for (size_t i = 0; i < n_strobes; ++i) {
        int bit;
        if (invert) bit = (strobes[i] < 0.0f) ? 1 : 0;
        else        bit = (strobes[i] > 0.0f) ? 1 : 0;
        out_bits[i] = (uint8_t) bit;
    }
}

// Stage 7b: full Hamming-distance trace from the 32-bit ASM, per bit
// offset. Output length = n_bits - 31. asm_find_best() is the
// argmin (subject to a max-Hamming cap + min-offset filter); this is
// the underlying curve.
static void fsk_stage_asm_hamming(const uint8_t *bits, size_t n_bits,
                                  uint32_t needle, uint8_t *out_hamming)
{
    if (n_bits < 32) return;
    uint32_t window = 0;
    for (size_t i = 0; i < 32; ++i) {
        window = (window << 1) | (bits[i] & 1u);
    }
    out_hamming[0] = (uint8_t) __builtin_popcount(window ^ needle);
    for (size_t i = 32; i < n_bits; ++i) {
        window = (window << 1) | (bits[i] & 1u);
        out_hamming[i - 31] =
            (uint8_t) __builtin_popcount(window ^ needle);
    }
}

size_t modem_fsk_diag_max_strobes(size_t n_pairs, int sps)
{
    if (sps < 1) return 0;
    if (n_pairs < (size_t) FSK_IQ_LPF_LEN + 2u) return 0;
    size_t lpf_n = n_pairs - (size_t) FSK_IQ_LPF_LEN + 1u;
    if (lpf_n < 2) return 0;
    size_t fm_n = lpf_n - 1;
    if (fm_n < (size_t) sps) return 0;
    size_t mf_n = fm_n - (size_t) sps + 1u;
    return mf_n / (size_t) sps + 1u;
}

int modem_fsk_iq_to_bits(const int16_t *iq_pairs, size_t n_pairs,
                         const modem_params_t *p,
                         int invert_polarity,
                         int sync_max_ham,
                         size_t min_bit_offset,
                         uint8_t *out_bits, size_t *n_bits_out,
                         size_t *sync_bit_offset,
                         int *polarity_used)
{
    return modem_fsk_iq_to_bits_diag(iq_pairs, n_pairs, p,
                                     invert_polarity, sync_max_ham,
                                     min_bit_offset,
                                     out_bits, n_bits_out,
                                     sync_bit_offset, polarity_used,
                                     NULL);
}

int modem_fsk_iq_to_bits_diag(const int16_t *iq_pairs, size_t n_pairs,
                              const modem_params_t *p,
                              int invert_polarity,
                              int sync_max_ham,
                              size_t min_bit_offset,
                              uint8_t *out_bits, size_t *n_bits_out,
                              size_t *sync_bit_offset,
                              int *polarity_used,
                              fsk_diag_t *diag)
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
    if (n_pairs < (size_t) FSK_IQ_LPF_LEN + 2u) return -1;

    if (diag != NULL) {
        diag->n_pairs_lpf  = 0;
        diag->n_fm         = 0;
        diag->n_mf         = 0;
        diag->n_strobes    = 0;
        diag->asm_offset   = (size_t) -1;
        diag->asm_dist     = 33;
        diag->polarity_used = -1;
    }

    // Stage 1: IQ low-pass filter (see fsk_stage_lpf).
    size_t lpf_n_cap = n_pairs - (size_t) FSK_IQ_LPF_LEN + 1u;
    float *I_lpf = (float *) malloc(lpf_n_cap * sizeof(float));
    float *Q_lpf = (float *) malloc(lpf_n_cap * sizeof(float));
    if (I_lpf == NULL || Q_lpf == NULL) {
        free(I_lpf); free(Q_lpf);
        return -1;
    }
    size_t lpf_n = fsk_stage_lpf(iq_pairs, n_pairs, p->samp_rate,
                                 I_lpf, Q_lpf);
    if (diag != NULL) {
        diag->n_pairs_lpf = lpf_n;
        if (diag->i_lpf != NULL)
            memcpy(diag->i_lpf, I_lpf, lpf_n * sizeof(float));
        if (diag->q_lpf != NULL)
            memcpy(diag->q_lpf, Q_lpf, lpf_n * sizeof(float));
    }

    // Stage 2: FM discriminator.
    size_t fm_n_cap = (lpf_n > 0) ? lpf_n - 1 : 0;
    float *fm = (fm_n_cap > 0) ? (float *) malloc(fm_n_cap * sizeof(float))
                               : NULL;
    if (fm_n_cap > 0 && fm == NULL) {
        free(I_lpf); free(Q_lpf);
        return -1;
    }
    size_t fm_n = fsk_stage_discriminate(I_lpf, Q_lpf, lpf_n, fm);
    free(I_lpf); free(Q_lpf);

    // Stage 3 + 4: DC block + AGC (in place).
    if (!p->rx_disable_dc_block) {
        fsk_stage_dc_block(fm, fm_n, 0.995f);
    }
    fsk_stage_agc(fm, fm_n);

    if (diag != NULL) {
        diag->n_fm = fm_n;
        if (diag->fm != NULL && fm_n > 0)
            memcpy(diag->fm, fm, fm_n * sizeof(float));
    }

    // Stage 5: boxcar matched filter.
    if (fm_n < (size_t) sps) { free(fm); return -1; }
    size_t mf_n_cap = fm_n - (size_t) sps + 1u;
    float *mf = (float *) malloc(mf_n_cap * sizeof(float));
    if (mf == NULL) { free(fm); return -1; }
    size_t mf_n = fsk_stage_matched_filter(fm, fm_n, sps, mf);
    free(fm);

    if (diag != NULL) {
        diag->n_mf = mf_n;
        if (diag->mf != NULL && mf_n > 0)
            memcpy(diag->mf, mf, mf_n * sizeof(float));
    }

    // Stage 6: Gardner+Farrow timing recovery.
    size_t max_strobes = mf_n / (size_t) sps + 1u;
    float  *strobe   = (float *)  malloc(max_strobes * sizeof(float));
    double *strobe_t = (double *) malloc(max_strobes * sizeof(double));
    if (strobe == NULL || strobe_t == NULL) {
        free(strobe); free(strobe_t); free(mf);
        return -1;
    }
    size_t n_sym = fsk_stage_gardner_farrow(mf, mf_n, sps,
                                            strobe, strobe_t, max_strobes);
    free(mf);

    if (diag != NULL) {
        diag->n_strobes = n_sym;
        if (diag->strobes != NULL && n_sym > 0)
            memcpy(diag->strobes, strobe, n_sym * sizeof(float));
        if (diag->strobe_t != NULL && n_sym > 0)
            memcpy(diag->strobe_t, strobe_t, n_sym * sizeof(double));
    }

    // Stage 7: slice + ASM search under both polarities. The diag
    // bits/asm_hamming reflect the polarity that was actually USED
    // (i.e. the one that yielded sync, or the first one tried if
    // neither did).
    uint8_t *tmp_bits = (uint8_t *) malloc(n_sym ? n_sym : 1);
    if (tmp_bits == NULL) {
        free(strobe); free(strobe_t);
        return -1;
    }
    int polarities[2];
    polarities[0] = invert_polarity ? 1 : 0;
    polarities[1] = invert_polarity ? 0 : 1;

    size_t best_sync     = (size_t) -1;
    int    best_polarity = -1;
    int    best_ham      = 33;
    int    diag_polarity = -1;

    for (int pi = 0; pi < 2 && best_polarity < 0; ++pi) {
        int this_invert = polarities[pi];
        fsk_stage_slice(strobe, n_sym, this_invert, tmp_bits);
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
            best_ham = this_ham;
        }
        // Snap diag for THIS polarity if it's the one we ended on
        // (either we found sync, or it's the last polarity and we
        // still haven't snapped anything).
        if (diag != NULL
            && (best_polarity == this_invert || pi == 1
                || diag_polarity < 0)) {
            if (diag->bits != NULL && n_sym > 0)
                memcpy(diag->bits, tmp_bits, n_sym);
            if (diag->asm_hamming != NULL && n_sym >= 32) {
                fsk_stage_asm_hamming(tmp_bits, n_sym,
                                      ASM_BIG_ENDIAN_U32,
                                      diag->asm_hamming);
            }
            diag_polarity = this_invert;
        }
    }
    if (polarity_used) *polarity_used = best_polarity;
    if (diag != NULL) {
        diag->polarity_used = (best_polarity >= 0)
            ? best_polarity : diag_polarity;
        diag->asm_offset = best_sync;
        diag->asm_dist   = (best_polarity >= 0) ? best_ham : 33;
    }

    free(tmp_bits);
    free(strobe);
    free(strobe_t);

    if (best_polarity < 0) {
        if (sync_bit_offset) *sync_bit_offset = (size_t) -1;
        return -1;
    }
    if (sync_bit_offset) *sync_bit_offset = best_sync;
    return 0;
}
