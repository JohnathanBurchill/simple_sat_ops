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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ASM_BIG_ENDIAN_U32 0x930B51DEu

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

// Lowest-Hamming ASM finder, same convention as modem_iq.c /
// modem_viterbi.c (duplicated here so this TU is independent).
static size_t fsk_find_asm_best(const uint8_t *bits, size_t n_bits,
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

int modem_fsk_iq_to_bits(const int16_t *iq_pairs, size_t n_pairs,
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
    if (n_pairs < (size_t) FSK_IQ_LPF_LEN + 2u) return -1;

    // 1. Pre-FM-discriminator low-pass filter on I and Q. See the
    //    fsk_iq_lpf comment above for why this matters. Output length
    //    after a valid-only convolution is n_pairs - FSK_IQ_LPF_LEN + 1.
    float lpf_kernel[FSK_IQ_LPF_LEN];
    fsk_build_iq_lpf(fsk_iq_lpf_cutoff_hz(), (double) p->samp_rate,
                     lpf_kernel);

    size_t iq_lpf_n = n_pairs - (size_t) FSK_IQ_LPF_LEN + 1u;
    float *I_lpf = (float *) malloc(iq_lpf_n * sizeof(float));
    float *Q_lpf = (float *) malloc(iq_lpf_n * sizeof(float));
    if (I_lpf == NULL || Q_lpf == NULL) {
        free(I_lpf); free(Q_lpf);
        return -1;
    }
    for (size_t i = 0; i < iq_lpf_n; ++i) {
        double accI = 0.0, accQ = 0.0;
        for (int k = 0; k < FSK_IQ_LPF_LEN; ++k) {
            double h = (double) lpf_kernel[k];
            accI += h * (double) iq_pairs[(i + (size_t) k) * 2 + 0];
            accQ += h * (double) iq_pairs[(i + (size_t) k) * 2 + 1];
        }
        I_lpf[i] = (float) accI;
        Q_lpf[i] = (float) accQ;
    }

    // 2. FM discriminator on the filtered IQ. The instantaneous-
    //    frequency stream at the full IQ sample rate is the differential
    //    phase between consecutive samples:
    //        fm[i] = arg(z[i+1] * conj(z[i]))
    //    For a pure tone at baseband offset Δf this lands at the
    //    constant 2π·Δf/fs (DC after FM demod = carrier offset). The
    //    modulator adds ±2π·deviation/fs on top, so the FSK bits
    //    show up as two scalar levels separated by 2·(2π·dev/fs).
    //    Doppler-of-the-pass is the slow-varying DC — the HPF (step 3)
    //    strips it.
    size_t fm_n = iq_lpf_n - 1;
    float *fm = (float *) malloc(fm_n * sizeof(float));
    if (fm == NULL) { free(I_lpf); free(Q_lpf); return -1; }
    for (size_t i = 0; i < fm_n; ++i) {
        double I0 = (double) I_lpf[i],     Q0 = (double) Q_lpf[i];
        double I1 = (double) I_lpf[i + 1], Q1 = (double) Q_lpf[i + 1];
        fm[i] = (float) atan2(Q1 * I0 - I1 * Q0,
                              I1 * I0 + Q1 * Q0);
    }
    free(I_lpf); free(Q_lpf);

    // 3. DC-block (1-pole HPF, same shape as modem_pcm16). Strips
    //    the carrier-offset DC bias, including the slow Doppler
    //    drift that walks across the pass. Skipped when caller sets
    //    p->rx_disable_dc_block (useful for synthetic AWGN tests).
    if (!p->rx_disable_dc_block) {
        const float alpha = 0.995f;
        float prev_x = 0.0f, prev_y = 0.0f;
        for (size_t i = 0; i < fm_n; ++i) {
            float x = fm[i];
            float y = x - prev_x + alpha * prev_y;
            fm[i] = y;
            prev_x = x;
            prev_y = y;
        }
    }

    // 4. AGC: RMS-normalise so the MF and the slicer both see
    //    roughly unit-scale signal.
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

    // 5. Boxcar matched filter of length sps. After this each output
    //    sample is the average instantaneous frequency over one
    //    symbol period. At a perfectly-aligned strobe inside a
    //    constant-bit-run this lands at ±1 after AGC; on transitions
    //    it dips through zero.
    if (fm_n < (size_t) sps) { free(fm); return -1; }
    size_t mf_len = fm_n - (size_t) sps + 1;
    float *mf = (float *) malloc(mf_len * sizeof(float));
    if (mf == NULL) { free(fm); return -1; }
    {
        const double inv_sps = 1.0 / (double) sps;
        double window = 0.0;
        for (int k = 0; k < sps; ++k) window += (double) fm[k];
        mf[0] = (float)(window * inv_sps);
        for (size_t i = 1; i < mf_len; ++i) {
            window -= (double) fm[i - 1];
            window += (double) fm[i + (size_t) sps - 1];
            mf[i] = (float)(window * inv_sps);
        }
    }
    free(fm);

    // 6. Symbol-timing recovery: Gardner TED with 4-point Lagrange
    //    (Farrow cubic) fractional-delay interpolation, replacing the
    //    M&M + linear interpolation we used originally. This matches
    //    gr-satellites' digital.symbol_sync_ff(TED_GARDNER, ...
    //    IR_PFB_NO_MF) chain that the AIT gold-standard ground station
    //    uses (radio_ax100.py) and gains ~1-2 dB at marginal SNR on
    //    real RF — Gardner is a non-decision-directed TED, so it stays
    //    locked on bursts where the slicer would otherwise be feeding
    //    noisy decisions back into M&M. Farrow cubic is the standard
    //    closed-form alternative to PFB for sub-sample interpolation
    //    and is good to <0.1 dB of the full PFB for sps>=3.
    //
    //    Gardner TED: e(k) = y_mid · (y_curr - y_prev), where y_mid is
    //    the sample halfway between consecutive symbol centres and
    //    y_curr/y_prev are the symbol-centre samples themselves. For
    //    well-aligned timing e≈0; an early/late strobe gives a non-
    //    zero error whose sign indicates the direction.
    //
    //    Loop constants: P-only proportional controller with
    //    Kp = 0.05 (matches gr-satellites' clk_bw=0.06 / damping=1.0
    //    PI loop closely enough for the burst durations we see —
    //    integral term is overkill when burst length << 1/clk_bw).
    //    max_step = clk_limit = 0.004·sps = 0.02 samples per symbol —
    //    same as the gr-satellites default.
    size_t max_strobes = mf_len / (size_t) sps + 1;
    float *strobe = (float *) malloc(max_strobes * sizeof(float));
    if (strobe == NULL) { free(mf); return -1; }
    size_t n_sym = 0;
    {
        const double sps_d = (double) sps;
        const double half = sps_d * 0.5;
        const double Kp = 0.05;
        const double max_step = sps_d * 0.10;
        double pos = sps_d;
        double prev_y = 0.0;
        int have_prev = 0;
        while (pos + sps_d + 2.0 < (double) mf_len && pos > 1.0
               && n_sym < max_strobes) {
            // Farrow cubic at the current symbol centre.
            size_t i_c  = (size_t) pos;
            double mu_c = pos - (double) i_c;
            if (i_c < 1 || i_c + 2 >= mf_len) break;
            double f_m1c = -mu_c * (mu_c - 1.0) * (mu_c - 2.0) / 6.0;
            double f_0c  =  (mu_c + 1.0) * (mu_c - 1.0) * (mu_c - 2.0) / 2.0;
            double f_1c  = -(mu_c + 1.0) * mu_c * (mu_c - 2.0) / 2.0;
            double f_2c  =  (mu_c + 1.0) * mu_c * (mu_c - 1.0) / 6.0;
            double y_curr = (double) mf[i_c - 1] * f_m1c
                          + (double) mf[i_c]     * f_0c
                          + (double) mf[i_c + 1] * f_1c
                          + (double) mf[i_c + 2] * f_2c;
            strobe[n_sym++] = (float) y_curr;

            if (have_prev) {
                // Mid-symbol sample (halfway between previous and
                // current symbol centres) for Gardner TED.
                double pos_mid = pos - half;
                if (pos_mid > 1.0
                    && pos_mid + 2.0 < (double) mf_len) {
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
                    // Gardner TED: late sampling gives positive
                    // ted (see derivation in the textbook). Apply a
                    // negative gain so adj points toward earlier
                    // sampling on the next step.
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
    }
    free(mf);

    // 7. Slice + ASM search under both polarities (FM-discriminator
    //    polarity is a radio-side convention — gr_satellites finds
    //    the polarity that yields a valid sync, we do the same).
    uint8_t *tmp_bits = (uint8_t *) malloc(n_sym ? n_sym : 1);
    if (tmp_bits == NULL) { free(strobe); return -1; }
    int polarities[2];
    polarities[0] = invert_polarity ? 1 : 0;
    polarities[1] = invert_polarity ? 0 : 1;

    size_t best_sync = (size_t) -1;
    int best_polarity = -1;

    for (int pi = 0; pi < 2 && best_polarity < 0; ++pi) {
        int this_invert = polarities[pi];
        for (size_t i = 0; i < n_sym; ++i) {
            int bit;
            if (this_invert) bit = (strobe[i] < 0.0f) ? 1 : 0;
            else             bit = (strobe[i] > 0.0f) ? 1 : 0;
            tmp_bits[i] = (uint8_t) bit;
        }
        int this_ham = 33;
        size_t off = fsk_find_asm_best(tmp_bits, n_sym,
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
    free(strobe);

    if (best_polarity < 0) {
        if (sync_bit_offset) *sync_bit_offset = (size_t) -1;
        return -1;
    }
    if (sync_bit_offset) *sync_bit_offset = best_sync;
    return 0;
}
