/*

    Simple Satellite Operations  modem.c

    Copyright (C) 2025, 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "modem.h"

#include "asm_search.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void modem_params_defaults(modem_params_t *p)
{
    if (p == NULL) return;
    p->samp_rate = 48000;
    p->bit_rate = 9600;
    p->gain_db = 0.0;
    // gauss_bt = 0 disables the Gaussian filter entirely (rectangular NRZ
    // pulses). The gold-reference gr-satellites TX chain in
    // gnu_radio/usrp_b210_gnu_radio/radio_ax100.py uses NRZ FSK
    // (chunks_to_symbols_bf([-1,1]) with no pulse shaping) and its
    // fsk_demodulator is matched to that. Callers that want GMSK can
    // override gauss_bt back to 0.5 explicitly.
    p->gauss_bt = 0.0;
    p->gauss_symbol_span = 4;
    p->rx_disable_dc_block = 0;
    p->fsk_iq_lpf_hz = 0.0;  // 0 = built-in default / $FSK_IQ_LPF_HZ override
}

ssize_t modem_bytes_to_pcm16(const uint8_t *frame, size_t frame_len,
                             const modem_params_t *p,
                             int16_t *out, size_t out_cap)
{
    if (frame == NULL || p == NULL || out == NULL || frame_len == 0) {
        return -1;
    }
    if (p->samp_rate <= 0 || p->bit_rate <= 0) {
        return -1;
    }
    if (p->samp_rate % p->bit_rate != 0) {
        // Integer sps keeps the oversampler + filter trivial.
        return -1;
    }
    int sps = p->samp_rate / p->bit_rate;
    if (sps <= 0) return -1;

    size_t n_bits = frame_len * 8;
    size_t n_samples = n_bits * (size_t)sps;
    if (n_samples > out_cap) {
        return -1;
    }

    // Baseband NRZ buffer (float for convolution).
    float *bb = (float *)malloc(n_samples * sizeof(float));
    if (bb == NULL) return -1;

    size_t idx = 0;
    for (size_t b = 0; b < frame_len; ++b) {
        uint8_t byte = frame[b];
        for (int bit = 7; bit >= 0; --bit) {
            float v = ((byte >> bit) & 1u) ? 1.0f : -1.0f;
            for (int s = 0; s < sps; ++s) {
                bb[idx++] = v;
            }
        }
    }

    // Optional Gaussian pulse-shape filter (unity DC gain).
    int n_taps = 0;
    float *taps = NULL;
    if (p->gauss_bt > 0.0 && p->gauss_symbol_span > 0) {
        n_taps = p->gauss_symbol_span * sps + 1;
        taps = (float *)malloc((size_t)n_taps * sizeof(float));
        if (taps == NULL) {
            free(bb);
            return -1;
        }
        double center = (n_taps - 1) / 2.0;
        double k = 2.0 * M_PI * p->gauss_bt / sqrt(log(2.0));
        double sum = 0.0;
        for (int i = 0; i < n_taps; ++i) {
            double t_nt = ((double)i - center) / (double)sps;
            double arg = k * t_nt;
            taps[i] = (float)exp(-0.5 * arg * arg);
            sum += taps[i];
        }
        if (sum <= 0.0) {
            free(taps);
            free(bb);
            return -1;
        }
        for (int i = 0; i < n_taps; ++i) {
            taps[i] = (float)(taps[i] / sum);
        }
    }

    float gain = (float)pow(10.0, p->gain_db / 20.0);
    int half = n_taps > 0 ? (n_taps - 1) / 2 : 0;

    for (size_t n = 0; n < n_samples; ++n) {
        double y;
        if (n_taps > 0) {
            y = 0.0;
            for (int j = 0; j < n_taps; ++j) {
                ssize_t src = (ssize_t)n + j - half;
                if (src >= 0 && (size_t)src < n_samples) {
                    y += (double)taps[j] * (double)bb[src];
                }
            }
        } else {
            y = (double)bb[n];
        }
        y *= (double)gain;
        if (y > 1.0) y = 1.0;
        else if (y < -1.0) y = -1.0;
        long v = lrint(y * 32767.0);
        if (v > 32767) v = 32767;
        else if (v < -32768) v = -32768;
        out[n] = (int16_t)v;
    }

    free(taps);
    free(bb);
    return (ssize_t)n_samples;
}

static void wav_write_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)( v        & 0xFF);
    p[1] = (uint8_t)((v >>  8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void wav_write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)( v       & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

int pcm16_write_wav(const char *path,
                    const int16_t *samples, size_t n_samples,
                    int samp_rate)
{
    if (path == NULL || samples == NULL || samp_rate <= 0) {
        return -1;
    }

    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        fprintf(stderr, "pcm16_write_wav: open(%s): %s\n", path, strerror(errno));
        return -1;
    }

    const uint16_t channels = 1;
    const uint16_t bits_per_sample = 16;
    const uint16_t block_align = channels * (bits_per_sample / 8);
    const uint32_t byte_rate = (uint32_t)samp_rate * block_align;
    const uint32_t data_size = (uint32_t)(n_samples * block_align);
    const uint32_t riff_size = 36u + data_size;

    uint8_t header[44];
    memcpy(header + 0, "RIFF", 4);
    wav_write_u32_le(header + 4, riff_size);
    memcpy(header + 8, "WAVE", 4);
    memcpy(header + 12, "fmt ", 4);
    wav_write_u32_le(header + 16, 16u);        // subchunk size
    wav_write_u16_le(header + 20, 1u);         // audio format = PCM
    wav_write_u16_le(header + 22, channels);
    wav_write_u32_le(header + 24, (uint32_t)samp_rate);
    wav_write_u32_le(header + 28, byte_rate);
    wav_write_u16_le(header + 32, block_align);
    wav_write_u16_le(header + 34, bits_per_sample);
    memcpy(header + 36, "data", 4);
    wav_write_u32_le(header + 40, data_size);

    if (fwrite(header, 1, sizeof(header), f) != sizeof(header)) {
        fprintf(stderr, "pcm16_write_wav: header write failed\n");
        fclose(f);
        return -1;
    }
    if (fwrite(samples, sizeof(int16_t), n_samples, f) != n_samples) {
        fprintf(stderr, "pcm16_write_wav: sample write failed\n");
        fclose(f);
        return -1;
    }
    if (fclose(f) != 0) {
        fprintf(stderr, "pcm16_write_wav: close(%s): %s\n", path, strerror(errno));
        return -1;
    }
    return 0;
}

// Demod chain (independent of gr-satellites; equivalent in shape to its
// fsk_demodulator):
//
//   PCM int16  →  DC-block (existing 1-pole HPF, optional)
//              →  AGC: divide by signal RMS so downstream gains are scale-free
//              →  Matched filter: sliding boxcar of length sps. For
//                 rectangular NRZ pulses this is the integrate-and-dump
//                 optimum filter; gives ~10·log10(sps) dB of SNR gain
//                 over single-sample slicing (~7 dB at sps=5).
//              →  Mueller-Müller decision-directed symbol-timing recovery:
//                 strobe one sample per symbol via linear interpolation,
//                 update the strobe position with TED = sgn(y[k-1])·y[k]
//                 - sgn(y[k])·y[k-1], proportional loop filter. M&M is
//                 sign-invariant under polarity flip, so a single timing
//                 loop produces the strobe samples that we then slice
//                 under each polarity.
//              →  Hard slicer at 0 (sign of the strobe sample)
//              →  ASM (0x930B51DE) search in the bit stream
//
// The change vs. the prior phase-search slicer: about 7 dB of bit-error
// rate margin, which is the difference between RS-correctable and
// RS-uncorrectable on a real-RF capture.

int modem_pcm16_to_bits(const int16_t *samples, size_t n_samples,
                        const modem_params_t *p,
                        int invert_polarity,
                        int sync_max_ham,
                        size_t min_bit_offset,
                        uint8_t *out_bits, size_t *n_bits_out,
                        size_t *sync_bit_offset,
                        int *polarity_used)
{
    if (samples == NULL || p == NULL || out_bits == NULL || n_bits_out == NULL) {
        return -1;
    }
    if (p->samp_rate <= 0 || p->bit_rate <= 0 ||
        p->samp_rate % p->bit_rate != 0) {
        return -1;
    }
    int sps = p->samp_rate / p->bit_rate;
    if (sps <= 1 || n_samples < (size_t)sps * (size_t)32) {
        return -1;
    }

    // 1. DC-block: 1-pole HPF, y[n] = x[n] - x[n-1] + α·y[n-1].
    //    α close to 1 → narrow low-cut. 0.995 at 48 kHz ≈ 40 Hz -3dB.
    //    Skipped when p->rx_disable_dc_block is set — useful for radio
    //    paths with no DC offset, where the HPF only adds baseline
    //    transients and group delay.
    float *dc_blocked = (float *)malloc(n_samples * sizeof(float));
    if (dc_blocked == NULL) return -1;
    if (p->rx_disable_dc_block) {
        for (size_t i = 0; i < n_samples; ++i) {
            dc_blocked[i] = (float)samples[i];
        }
    } else {
        const float alpha = 0.995f;
        float prev_x = 0.0f, prev_y = 0.0f;
        for (size_t i = 0; i < n_samples; ++i) {
            float x = (float)samples[i];
            float y = x - prev_x + alpha * prev_y;
            dc_blocked[i] = y;
            prev_x = x;
            prev_y = y;
        }
    }

    // 2. AGC: compute RMS over the whole window, normalize so the matched
    //    filter and timing-error detector both see roughly unit-scale signals.
    //    A small floor prevents division blow-ups on near-silent input.
    double sum_sq = 0.0;
    for (size_t i = 0; i < n_samples; ++i) {
        sum_sq += (double)dc_blocked[i] * (double)dc_blocked[i];
    }
    double rms = sqrt(sum_sq / (double)n_samples);
    if (rms < 1.0) rms = 1.0;
    double agc_inv = 1.0 / rms;

    // 3. Matched filter: sliding boxcar of length sps, output normalized
    //    so a perfectly-aligned in-symbol strobe lands at amplitude ~ ±1
    //    after AGC. Implemented as an O(N) running sum.
    size_t mf_len = n_samples - (size_t)sps + 1;
    float *mf = (float *)malloc(mf_len * sizeof(float));
    if (mf == NULL) {
        free(dc_blocked);
        return -1;
    }
    {
        double inv_sps = 1.0 / (double)sps;
        double window = 0.0;
        for (int k = 0; k < sps; ++k) window += (double)dc_blocked[k];
        mf[0] = (float)(window * agc_inv * inv_sps);
        for (size_t i = 1; i < mf_len; ++i) {
            window -= (double)dc_blocked[i - 1];
            window += (double)dc_blocked[i + (size_t)sps - 1];
            mf[i] = (float)(window * agc_inv * inv_sps);
        }
    }
    free(dc_blocked);

    // 4. Mueller-Müller timing recovery + strobe collection.
    //    Loop gain 0.10 — fast pull-in (the AX100 0x55 preamble has a
    //    transition every bit, so M&M typically locks within ~5 symbols),
    //    small enough to keep tracking jitter ~5 % of a symbol period at
    //    moderate SNR. Per-step adjustment is clamped to ±sps/4 so a
    //    pathological TED can't make the strobe skip or repeat a symbol.
    //
    //    The strobe sample at fractional sample position `pos` is linearly
    //    interpolated from the matched-filter output. We start one symbol
    //    in so M&M has valid history for its first TED computation.
    size_t max_strobes = mf_len / (size_t)sps + 1;
    float *strobe = (float *)malloc(max_strobes * sizeof(float));
    if (strobe == NULL) {
        free(mf);
        return -1;
    }
    {
        const double sps_d = (double)sps;
        const double Kp = 0.10;
        const double max_step = sps_d * 0.25;
        double pos = sps_d;
        double prev_y = 0.0, prev_dec = 0.0;
        int have_prev = 0;
        size_t n = 0;
        while (pos + 1.0 < (double)mf_len && n < max_strobes) {
            size_t i = (size_t)pos;
            double frac = pos - (double)i;
            double y = (double)mf[i] * (1.0 - frac) + (double)mf[i + 1] * frac;
            double dec = (y >= 0.0) ? 1.0 : -1.0;
            strobe[n++] = (float)y;

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
    free(mf);

    // 5. Slice strobe samples under each polarity and ASM-search.
    //    FM-discriminator polarity is a radio-side convention (IC-9700
    //    MONI vs. RTL-SDR, etc.), so we brute-force both. Normal-first
    //    when invert_polarity=0; preserves the prior preference of
    //    accepting any normal-polarity match before falling back to
    //    inverted, even if inverted gave a lower-Hamming match.
    uint8_t *tmp_bits = (uint8_t *)malloc(max_strobes ? max_strobes : 1);
    if (tmp_bits == NULL) {
        free(strobe);
        return -1;
    }
    int polarities[2];
    polarities[0] = invert_polarity ? 1 : 0;
    polarities[1] = invert_polarity ? 0 : 1;

    size_t best_sync = (size_t)-1;
    int best_polarity = -1;

    for (int pi = 0; pi < 2 && best_polarity < 0; ++pi) {
        int this_invert = polarities[pi];
        for (size_t i = 0; i < max_strobes; ++i) {
            int bit;
            if (this_invert) bit = (strobe[i] < 0.0f) ? 1 : 0;
            else             bit = (strobe[i] > 0.0f) ? 1 : 0;
            tmp_bits[i] = (uint8_t)bit;
        }
        int this_ham = 33;
        size_t off = asm_find_best(tmp_bits, max_strobes,
                                           ASM_BIG_ENDIAN_U32, sync_max_ham,
                                           min_bit_offset, &this_ham);
        if (off != (size_t)-1) {
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
        if (sync_bit_offset) *sync_bit_offset = (size_t)-1;
        return -1;
    }
    if (sync_bit_offset) *sync_bit_offset = best_sync;
    return 0;
}

size_t modem_bits_to_bytes(const uint8_t *bits, size_t n_bits, uint8_t *out)
{
    size_t n_bytes = (n_bits + 7) / 8;
    for (size_t i = 0; i < n_bytes; ++i) {
        uint8_t b = 0;
        for (int k = 0; k < 8; ++k) {
            size_t bit_idx = i * 8 + (size_t)k;
            if (bit_idx < n_bits) {
                b = (uint8_t)((b << 1) | (bits[bit_idx] & 1u));
            } else {
                b <<= 1;
            }
        }
        out[i] = b;
    }
    return n_bytes;
}
