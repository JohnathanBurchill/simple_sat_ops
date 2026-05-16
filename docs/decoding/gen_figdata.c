// Generate data files for DECODING.md figures.
//
// Drives the real C encoder pipeline (csp.c, ax100.c, golay24.c, rs.c)
// to produce a wire-format AX100 frame for the CSP packet
//     CSP{src=5, dst=10, dport=9, sport=0} "CTS1 beacon"
// then MSK-modulates the bits, simulates the channel + receiver DSP,
// and dumps .dat files that the companion .gp scripts plot.
//
// Build:
//   cc -O2 -I../.. gen_figdata.c ../../csp.c ../../ax100.c \
//      ../../golay24.c ../../rs.c -lcrypto -lm -o gen_figdata
//
// Outputs (in cwd):
//   wire_bytes.txt           hexdump-with-annotation of the wire stream
//   msk_trajectory.dat       bit / inst-freq / unwrapped-phase vs t
//   discriminator.dat        Δφ at SNR ∞, 10 dB, 0 dB
//   matched_filter.dat       Δφ + MF output at SNR 5 dB
//   eye_diagram.dat          gnuplot "every block" overlay of MF, SNR 8 dB
//   gardner_scurve.dat       TED average vs timing offset
//   asm_correlation.dat      sliding 32-bit Hamming distance to 0x930B51DE
//   scrambler_spectrum.dat   PSD of all-zero stream pre/post CCSDS scrambler
//   rs_cliff.dat             P(RS(255,223) success) vs bit-error rate

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <complex.h>

#include "csp.h"
#include "ax100.h"

#define FS       96000.0
#define BAUD     9600.0
#define SPS      10
#define H_MOD    0.5
#define DEV_HZ   (H_MOD * BAUD / 2.0)   // 2400 Hz
#define ASM_U32  0x930B51DEu

// CCSDS scrambler table excerpt (will be filled from ax100.c's exported
// symbol if visible; otherwise we open-code it here).
// ax100.c keeps CCSDS_SCRAMBLER_TABLE as file-static, so we mirror the
// first row here for the spectrum figure; the live encode/decode goes
// through ax100_frame so the symbol stays internal.
static uint8_t scrambler_xor_byte(size_t i);

// ============================================================
// MSK modulator
// ============================================================
static void msk_modulate(const uint8_t *bits, size_t n_bits,
                         double *iq_i, double *iq_q,
                         double *inst_f, double *phase_unwrapped)
{
    double phi = 0.0;
    size_t k = 0;
    for (size_t b = 0; b < n_bits; ++b) {
        double f = bits[b] ? +DEV_HZ : -DEV_HZ;
        double dphi = 2.0 * M_PI * f / FS;
        for (int s = 0; s < SPS; ++s) {
            phi += dphi;
            iq_i[k] = cos(phi);
            iq_q[k] = sin(phi);
            inst_f[k] = f;
            phase_unwrapped[k] = phi;
            ++k;
        }
    }
}

// ============================================================
// AWGN noise (Box-Muller)
// ============================================================
static double awgn_one(double sigma)
{
    static int has_spare = 0;
    static double spare;
    if (has_spare) { has_spare = 0; return spare * sigma; }
    double u1, u2;
    do { u1 = (rand() + 1.0) / ((double)RAND_MAX + 2.0); } while (u1 <= 1e-15);
    u2 = (rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double mag = sqrt(-2.0 * log(u1));
    spare = mag * sin(2.0 * M_PI * u2);
    has_spare = 1;
    return mag * cos(2.0 * M_PI * u2) * sigma;
}

static void add_awgn(const double *iq_in_i, const double *iq_in_q,
                     double *iq_out_i, double *iq_out_q,
                     size_t n, double snr_db)
{
    // Signal power per I or Q is 0.5 (since cos^2 averages to 1/2 and
    // the IQ pair has unit magnitude). So per-component noise variance
    // for target SNR is (0.5) / 10^(snr/10).
    double sigma = sqrt(0.5 * pow(10.0, -snr_db / 10.0));
    for (size_t i = 0; i < n; ++i) {
        iq_out_i[i] = iq_in_i[i] + awgn_one(sigma);
        iq_out_q[i] = iq_in_q[i] + awgn_one(sigma);
    }
}

// ============================================================
// FM discriminator: dphi[n] = arg(s[n] * conj(s[n-1]))
// ============================================================
static void fm_discriminate(const double *iq_i, const double *iq_q,
                            size_t n, double *dphi)
{
    dphi[0] = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double r  = iq_i[i] * iq_i[i-1] + iq_q[i] * iq_q[i-1];
        double im = iq_q[i] * iq_i[i-1] - iq_i[i] * iq_q[i-1];
        dphi[i] = atan2(im, r);
    }
}

// ============================================================
// Boxcar matched filter of length sps
// ============================================================
static size_t boxcar_mf(const double *x, size_t n, int len, double *y)
{
    if ((int)n < len) return 0;
    double sum = 0.0;
    for (int i = 0; i < len; ++i) sum += x[i];
    y[0] = sum / len;
    size_t out_n = n - (size_t)len + 1;
    for (size_t i = 1; i < out_n; ++i) {
        sum += x[i + (size_t)len - 1] - x[i - 1];
        y[i] = sum / len;
    }
    return out_n;
}

// ============================================================
// Helpers
// ============================================================
static FILE *open_dat(const char *path, const char *header)
{
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); exit(1); }
    if (header) fprintf(f, "# %s\n", header);
    return f;
}

static int popcount32(uint32_t x)
{
    int c = 0;
    while (x) { c += (int)(x & 1u); x >>= 1; }
    return c;
}

// Naive DFT-based PSD on a windowed signal; returns log10|X|^2 dB at
// each of `nbins` evenly-spaced normalized frequencies in [0, 0.5].
// Hann window applied. Slow but accurate; we only use ~4 kpts here.
static void psd_hann(const double *x, size_t n, int nbins, double *out_db)
{
    double sum_w2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double w = 0.5 * (1.0 - cos(2.0 * M_PI * i / (n - 1)));
        sum_w2 += w * w;
    }
    for (int k = 0; k < nbins; ++k) {
        double f_norm = 0.5 * (double)k / (double)(nbins - 1);
        double re = 0.0, im = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double w = 0.5 * (1.0 - cos(2.0 * M_PI * i / (n - 1)));
            double th = 2.0 * M_PI * f_norm * (double)i;
            re += w * x[i] * cos(th);
            im -= w * x[i] * sin(th);
        }
        double p = (re * re + im * im) / sum_w2;
        out_db[k] = 10.0 * log10(p + 1e-30);
    }
}

// Open-coded copy of CCSDS_SCRAMBLER_TABLE rows from ax100.c so this
// program can compare scrambled vs unscrambled bit streams without
// touching the encoder API. Built at startup by running an 8-bit LFSR
// with polynomial h(x) = x^8 + x^7 + x^5 + x^3 + 1, fbmask = 0xA9,
// initreg = 0xFF — same as ax100.c's table.
static uint8_t SCRAMBLER[255];
static int     scrambler_built = 0;
static void build_scrambler(void)
{
    if (scrambler_built) return;
    uint16_t reg = 0xFF;
    for (int byte = 0; byte < 255; ++byte) {
        uint8_t out = 0;
        for (int b = 0; b < 8; ++b) {
            // High bit of register is the output bit.
            uint8_t bit = (uint8_t)((reg >> 7) & 1u);
            out = (uint8_t)((out << 1) | bit);
            // Shift left; new low bit = parity of (reg AND fbmask).
            uint8_t fb = 0;
            uint16_t tap = (uint16_t)(reg & 0xA9u);
            while (tap) { fb ^= (uint8_t)(tap & 1u); tap >>= 1; }
            // Append the original high bit as feedback into low end as
            // per the standard self-synchronizing scheme used in AX100.
            // (Reproduces ax100.c's runtime LFSR sequence exactly; the
            // explicit table in ax100.c is the canonical source.)
            reg = (uint16_t)(((reg << 1) | fb) & 0xFFu);
        }
        SCRAMBLER[byte] = out;
    }
    scrambler_built = 1;
}

static uint8_t scrambler_xor_byte(size_t i) { build_scrambler(); return SCRAMBLER[i % 255]; }


// ============================================================
// MAIN
// ============================================================
int main(void)
{
    srand(1);

    // ---- 1. CSP packet ----
    csp_v1_header_t hdr = {
        .prio = CSP_PRIO_NORM, .src = 5, .dst = 10,
        .dport = 9, .sport = 0, .flags = 0,
    };
    const char *msg = "CTS1 beacon";
    uint8_t csp_pkt[64];
    ssize_t csp_len = csp_v1_encode(&hdr, (const uint8_t*)msg, strlen(msg),
                                    csp_pkt, sizeof csp_pkt);
    if (csp_len < 0) { fprintf(stderr, "csp encode failed\n"); return 1; }

    // ---- 2. AX100 frame ----
    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    opts.reed_solomon = 1;
    opts.prefill = 4;          // short preamble for tidier plots
    opts.tailfill = 0;

    uint8_t wire[512];
    ssize_t wire_len = ax100_frame(csp_pkt, (size_t)csp_len,
                                   &opts, wire, sizeof wire);
    if (wire_len < 0) { fprintf(stderr, "ax100 frame failed\n"); return 1; }

    // Wire byte layout for plotting/annotation
    int pre_n  = opts.prefill;                  // 4 bytes (0xAA)
    int asm_n  = opts.syncword ? 4 : 0;         // 4 bytes (0x93 0x0B 0x51 0xDE)
    int gly_n  = opts.len_field ? 3 : 0;        // 3 bytes (Golay length)
    int hdr_bits = (pre_n + asm_n + gly_n) * 8;

    // Annotated hexdump
    {
        FILE *f = open_dat("wire_bytes.txt", "Wire bytes (preamble | ASM | Golay len | scrambled payload+RS)");
        fprintf(f, "preamble (%d bytes):  ", pre_n);
        for (int i = 0; i < pre_n; ++i) fprintf(f, "%02X ", wire[i]);
        fprintf(f, "\nASM      (4 bytes):  ");
        for (int i = pre_n; i < pre_n + 4; ++i) fprintf(f, "%02X ", wire[i]);
        fprintf(f, "\nGolay    (3 bytes):  ");
        for (int i = pre_n + 4; i < pre_n + 7; ++i) fprintf(f, "%02X ", wire[i]);
        int data_off = pre_n + 7;
        int data_n = (int)wire_len - data_off;
        fprintf(f, "\npayload  (%d bytes, scrambled+RS):\n  ", data_n);
        for (int i = 0; i < data_n; ++i) {
            fprintf(f, "%02X ", wire[data_off + i]);
            if ((i + 1) % 16 == 0) fprintf(f, "\n  ");
        }
        fprintf(f, "\n");
        fclose(f);
    }

    // ---- 3. Bits, MSB first ----
    size_t n_bits = (size_t)wire_len * 8;
    uint8_t *bits = malloc(n_bits);
    for (size_t i = 0; i < (size_t)wire_len; ++i)
        for (int b = 7; b >= 0; --b)
            bits[i * 8 + (7 - b)] = (wire[i] >> b) & 1u;

    // ---- 4. MSK modulate (clean) ----
    size_t n_samp = n_bits * (size_t)SPS;
    double *iq_i      = malloc(n_samp * sizeof(double));
    double *iq_q      = malloc(n_samp * sizeof(double));
    double *inst_f    = malloc(n_samp * sizeof(double));
    double *phase_uw  = malloc(n_samp * sizeof(double));
    msk_modulate(bits, n_bits, iq_i, iq_q, inst_f, phase_uw);

    // ---- 5. Discriminator at three SNRs ----
    double *iq_i_clean = iq_i;
    double *iq_q_clean = iq_q;

    double *iq_i_10 = malloc(n_samp * sizeof(double));
    double *iq_q_10 = malloc(n_samp * sizeof(double));
    add_awgn(iq_i, iq_q, iq_i_10, iq_q_10, n_samp, 10.0);

    double *iq_i_0 = malloc(n_samp * sizeof(double));
    double *iq_q_0 = malloc(n_samp * sizeof(double));
    add_awgn(iq_i, iq_q, iq_i_0, iq_q_0, n_samp, 0.0);

    double *iq_i_5 = malloc(n_samp * sizeof(double));
    double *iq_q_5 = malloc(n_samp * sizeof(double));
    add_awgn(iq_i, iq_q, iq_i_5, iq_q_5, n_samp, 5.0);

    double *iq_i_8 = malloc(n_samp * sizeof(double));
    double *iq_q_8 = malloc(n_samp * sizeof(double));
    add_awgn(iq_i, iq_q, iq_i_8, iq_q_8, n_samp, 8.0);

    double *dphi_clean = malloc(n_samp * sizeof(double));
    double *dphi_10    = malloc(n_samp * sizeof(double));
    double *dphi_0     = malloc(n_samp * sizeof(double));
    double *dphi_5     = malloc(n_samp * sizeof(double));
    double *dphi_8     = malloc(n_samp * sizeof(double));
    fm_discriminate(iq_i_clean, iq_q_clean, n_samp, dphi_clean);
    fm_discriminate(iq_i_10,    iq_q_10,    n_samp, dphi_10);
    fm_discriminate(iq_i_0,     iq_q_0,     n_samp, dphi_0);
    fm_discriminate(iq_i_5,     iq_q_5,     n_samp, dphi_5);
    fm_discriminate(iq_i_8,     iq_q_8,     n_samp, dphi_8);

    // ---- 6. Matched filter (length sps) ----
    double *mf_clean = malloc(n_samp * sizeof(double));
    double *mf_5     = malloc(n_samp * sizeof(double));
    double *mf_8     = malloc(n_samp * sizeof(double));
    size_t mf_n = boxcar_mf(dphi_clean, n_samp, SPS, mf_clean);
    boxcar_mf(dphi_5, n_samp, SPS, mf_5);
    boxcar_mf(dphi_8, n_samp, SPS, mf_8);

    // ============================================================
    // FIGURE: MSK trajectory — first ~32 bits of the wire stream
    // ============================================================
    {
        FILE *f = open_dat("msk_trajectory.dat",
            "t_ms  bit_nrz  inst_f_Hz  phase_unwrapped_rad");
        int n_show_bits = 32;
        size_t k = 0;
        for (int b = 0; b < n_show_bits; ++b) {
            for (int s = 0; s < SPS; ++s, ++k) {
                double t_ms = (double)k / FS * 1000.0;
                double nrz = bits[b] ? +1.0 : -1.0;
                fprintf(f, "%.6f %g %.2f %.6f\n",
                        t_ms, nrz, inst_f[k], phase_uw[k]);
            }
        }
        fclose(f);
    }

    // ============================================================
    // FIGURE: discriminator at three SNRs
    // ============================================================
    {
        FILE *f = open_dat("discriminator.dat",
            "t_ms  dphi_clean  dphi_snr10dB  dphi_snr0dB  (rad/sample)");
        int n_show = 32 * SPS;
        for (int k = 0; k < n_show; ++k) {
            double t_ms = (double)k / FS * 1000.0;
            fprintf(f, "%.6f %.6f %.6f %.6f\n",
                    t_ms, dphi_clean[k], dphi_10[k], dphi_0[k]);
        }
        fclose(f);
    }

    // ============================================================
    // FIGURE: matched filter vs discriminator (SNR 5 dB)
    // ============================================================
    {
        FILE *f = open_dat("matched_filter.dat",
            "t_ms  dphi_snr5dB  mf_snr5dB  (rad/sample)");
        int n_show = 32 * SPS;
        for (int k = 0; k < n_show; ++k) {
            double t_ms = (double)k / FS * 1000.0;
            double mfv = (k < (int)mf_n) ? mf_5[k] : 0.0;
            fprintf(f, "%.6f %.6f %.6f\n", t_ms, dphi_5[k], mfv);
        }
        fclose(f);
    }

    // ============================================================
    // FIGURE: eye diagram (MF output, 2-symbol windows overlaid)
    // ============================================================
    // mf_8[k] averages dphi[k..k+SPS-1]. The k-th-symbol decision sits
    // at mf_8[k*SPS]. We overlay 2-symbol windows starting at each
    // decision instant — gnuplot reads blank-line-separated blocks.
    {
        FILE *f = open_dat("eye_diagram.dat",
            "fraction_of_symbol  mf_value  (one block per overlaid trace)");
        int n_traces = 120;
        int win = 2 * SPS;
        for (int t = 0; t < n_traces; ++t) {
            int i0 = t * SPS;
            if (i0 + win >= (int)mf_n) break;
            for (int s = 0; s < win; ++s) {
                double x = (double)s / SPS;   // 0..2 symbols
                fprintf(f, "%.6f %.6f\n", x, mf_8[i0 + s]);
            }
            fprintf(f, "\n");
        }
        fclose(f);
    }

    // ============================================================
    // FIGURE: Gardner S-curve
    // ============================================================
    // For each timing offset τ ∈ [-sps/2, +sps/2], compute the mean
    // Gardner TED over all symbols. TED[k] = y(k - 1/2) * (y(k) - y(k-1))
    // with samples linearly interpolated between MF taps so the curve
    // is smooth (not staircased to integer indices).
    {
        FILE *f = open_dat("gardner_scurve.dat",
            "tau_samples  mean_TED  (mean of Gardner timing-error detector)");
        int n_tau = 101;
        double tau_half = (double)SPS / 2.0;
        for (int it = 0; it < n_tau; ++it) {
            double tau = -tau_half + 2.0 * tau_half * (double)it / (n_tau - 1);
            double acc = 0.0;
            int cnt = 0;
            for (int k = 2; k < 800; ++k) {
                double p_curr = k * SPS + tau;
                double p_prev = (k - 1) * SPS + tau;
                double p_mid  = (k - 0.5) * SPS + tau;
                if (p_curr + 1 >= (double)mf_n) break;
                if (p_prev < 0) continue;
                int i0c = (int)p_curr; double fc = p_curr - i0c;
                int i0p = (int)p_prev; double fp = p_prev - i0p;
                int i0m = (int)p_mid;  double fm = p_mid  - i0m;
                double y_curr = mf_8[i0c] * (1 - fc) + mf_8[i0c + 1] * fc;
                double y_prev = mf_8[i0p] * (1 - fp) + mf_8[i0p + 1] * fp;
                double y_mid  = mf_8[i0m] * (1 - fm) + mf_8[i0m + 1] * fm;
                acc += y_mid * (y_curr - y_prev);
                ++cnt;
            }
            fprintf(f, "%.4f %.6e\n", tau, cnt ? acc / cnt : 0.0);
        }
        fclose(f);
    }

    // ============================================================
    // FIGURE: ASM sliding-window Hamming distance
    // ============================================================
    // Slide a 32-bit window across the wire-stream bits and count
    // Hamming distance to ASM = 0x930B51DE. Should dip to 0 exactly
    // at the ASM offset.
    {
        FILE *f = open_dat("asm_correlation.dat",
            "bit_offset  hamming_distance_to_ASM  (32-bit sliding window)");
        size_t maxoff = n_bits - 32;
        for (size_t off = 0; off <= maxoff; ++off) {
            uint32_t w = 0;
            for (int b = 0; b < 32; ++b)
                w = (w << 1) | bits[off + b];
            int h = popcount32(w ^ ASM_U32);
            fprintf(f, "%zu %d\n", off, h);
        }
        fclose(f);
    }

    // ============================================================
    // FIGURE: scrambler spectrum
    // ============================================================
    // Build a 4096-bit stream of all-zeros, then a scrambled version
    // (XOR each byte with CCSDS table). Convert to NRZ ±1 and compute
    // the PSD of each. Should show DC + harmonics for unscrambled,
    // flat (~white) for scrambled.
    {
        // "Worst-case structured payload" = alternating 0x00 / 0xFF.
        // Bit stream is then 00000000 11111111 00000000 11111111 ...,
        // a 16-bit-period square wave with fundamental at f = 1/16
        // cycles/bit and odd harmonics at 3/16, 5/16, 7/16, ....
        // The scrambler XORs this with the CCSDS table → ~white.
        int n_bytes = 512;          // 4096 bits
        int n_b = n_bytes * 8;
        double *nrz_unscr = calloc(n_b, sizeof(double));
        double *nrz_scr   = calloc(n_b, sizeof(double));
        for (int i = 0; i < n_bytes; ++i) {
            uint8_t byte_u = (i & 1) ? 0xFF : 0x00;
            uint8_t byte_s = byte_u ^ scrambler_xor_byte((size_t)i);
            for (int b = 7; b >= 0; --b) {
                int idx = i * 8 + (7 - b);
                nrz_unscr[idx] = ((byte_u >> b) & 1u) ? +1.0 : -1.0;
                nrz_scr[idx]   = ((byte_s >> b) & 1u) ? +1.0 : -1.0;
            }
        }
        int nbins = 256;
        double *psd_unscr = malloc(nbins * sizeof(double));
        double *psd_scr   = malloc(nbins * sizeof(double));
        psd_hann(nrz_unscr, n_b, nbins, psd_unscr);
        psd_hann(nrz_scr,   n_b, nbins, psd_scr);

        // Normalize each so its peak sits at 0 dB — comparing shapes,
        // not absolute power.
        double peak_u = -1e30, peak_s = -1e30;
        for (int k = 0; k < nbins; ++k) {
            if (psd_unscr[k] > peak_u) peak_u = psd_unscr[k];
            if (psd_scr[k]   > peak_s) peak_s = psd_scr[k];
        }
        FILE *f = open_dat("scrambler_spectrum.dat",
            "f_norm  psd_unscrambled_dB  psd_scrambled_dB  (peak-normalized)");
        for (int k = 0; k < nbins; ++k) {
            double fn = 0.5 * (double)k / (double)(nbins - 1);
            fprintf(f, "%.6f %.4f %.4f\n",
                    fn, psd_unscr[k] - peak_u, psd_scr[k] - peak_s);
        }
        fclose(f);
        free(nrz_unscr); free(nrz_scr); free(psd_unscr); free(psd_scr);
    }

    // ============================================================
    // FIGURE: Reed-Solomon decode cliff
    // ============================================================
    // Analytical: under i.i.d. bit-error rate p,
    //   p_byte = 1 - (1-p)^8
    //   P(decode_ok) = P( Binom(255, p_byte) <= 16 )
    // Compute the CDF in log domain for numerical stability.
    {
        FILE *f = open_dat("rs_cliff.dat",
            "bit_error_rate  P_decode_success  P_decode_failure");
        int n_p = 80;
        double p_min = 1e-5, p_max = 5e-2;
        for (int i = 0; i < n_p; ++i) {
            double frac = (double)i / (n_p - 1);
            double p = p_min * pow(p_max / p_min, frac);
            double pb = 1.0 - pow(1.0 - p, 8.0);
            // P(B(255, pb) <= 16) via recursion on log-pmf.
            // Use direct binomial sum with stable log-gamma terms.
            double log_pb = log(pb), log_qb = log(1.0 - pb);
            double max_logpmf = -INFINITY;
            double logpmf[17];
            for (int k = 0; k <= 16; ++k) {
                // log C(255, k) + k*log_pb + (255-k)*log_qb
                double log_c = lgamma(256.0) - lgamma(k + 1.0) - lgamma(255.0 - k + 1.0);
                double v = log_c + k * log_pb + (255.0 - k) * log_qb;
                logpmf[k] = v;
                if (v > max_logpmf) max_logpmf = v;
            }
            double s = 0.0;
            for (int k = 0; k <= 16; ++k) s += exp(logpmf[k] - max_logpmf);
            double P_ok = exp(max_logpmf) * s;
            if (P_ok > 1.0) P_ok = 1.0;
            if (P_ok < 0.0) P_ok = 0.0;
            fprintf(f, "%.6e %.6e %.6e\n", p, P_ok, 1.0 - P_ok);
        }
        fclose(f);
    }

    free(bits);
    free(iq_i); free(iq_q); free(inst_f); free(phase_uw);
    free(iq_i_10); free(iq_q_10);
    free(iq_i_0);  free(iq_q_0);
    free(iq_i_5);  free(iq_q_5);
    free(iq_i_8);  free(iq_q_8);
    free(dphi_clean); free(dphi_10); free(dphi_0); free(dphi_5); free(dphi_8);
    free(mf_clean); free(mf_5); free(mf_8);

    printf("ok: wire_len=%zd  CSP=%zd  msg='%s'\n",
           (ssize_t)wire_len, (ssize_t)csp_len, msg);
    printf("    header bits (preamble+ASM+Golay) = %d\n", hdr_bits);
    return 0;
}
