/*

    Simple Satellite Operations  modem.c

    Copyright (C) 2025  Johnathan K Burchill

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
    p->gauss_bt = 0.5;
    p->gauss_symbol_span = 4;
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

// AX100 attached sync marker. Must match AX100_ASM_* in ax100.h — duplicated
// here so modem.c doesn't have to pull ax100.h (keeps it a pure DSP module).
#define ASM_BIG_ENDIAN_U32  0x930B51DEu

// Given a bit stream (MSB-first), find the offset in bits at which the
// 32-bit pattern `needle` matches within `max_ham` bit errors, starting
// the search at or past `min_offset`. Returns the bit offset of the
// first match, or (size_t)-1 if not found.
static size_t find_u32_pattern(const uint8_t *bits, size_t n_bits,
                               uint32_t needle, int max_ham,
                               size_t min_offset)
{
    if (n_bits < 32) return (size_t)-1;
    size_t start = min_offset;
    if (start > n_bits - 32) return (size_t)-1;
    uint32_t window = 0;
    for (size_t i = 0; i < 32; ++i) {
        window = (window << 1) | (bits[start + i] & 1u);
    }
    if ((int)__builtin_popcount(window ^ needle) <= max_ham) {
        return start;
    }
    for (size_t i = 32; i < n_bits - start; ++i) {
        window = (window << 1) | (bits[start + i] & 1u);
        if ((int)__builtin_popcount(window ^ needle) <= max_ham) {
            return start + i - 31;
        }
    }
    return (size_t)-1;
}

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
    if (sps <= 0 || n_samples < (size_t)sps * 32u) {
        return -1;
    }

    // DC-block: 1-pole HPF, y[n] = x[n] - x[n-1] + α·y[n-1].
    // α close to 1 → narrow low-cut. 0.995 at 48 kHz ≈ 40 Hz -3dB.
    float *dc_blocked = (float *)malloc(n_samples * sizeof(float));
    if (dc_blocked == NULL) return -1;
    const float alpha = 0.995f;
    float prev_x = 0.0f, prev_y = 0.0f;
    for (size_t i = 0; i < n_samples; ++i) {
        float x = (float)samples[i];
        float y = x - prev_x + alpha * prev_y;
        dc_blocked[i] = y;
        prev_x = x;
        prev_y = y;
    }

    // Pre-slice into per-sample bits for each of the `sps` possible mid-bit
    // phase offsets. The actual bit stream is every sps-th sample starting
    // at (phase + sps/2) for phase in [0, sps).
    size_t max_bits = n_samples / (size_t)sps;
    uint8_t *tmp_bits = (uint8_t *)malloc(max_bits);
    if (tmp_bits == NULL) {
        free(dc_blocked);
        return -1;
    }

    // Search over both polarity conventions unless the caller forced one
    // (invert_polarity=0 tries normal first then inverted; =1 tries
    // inverted first then normal). FM-discriminator polarity is a
    // radio-side convention — IC-9700 MONI differs from RTL-SDR, for
    // example — so it's easier to brute-force than document.
    int polarities[2];
    polarities[0] = invert_polarity ? 1 : 0;
    polarities[1] = invert_polarity ? 0 : 1;

    size_t best_sync = (size_t)-1;
    int best_phase = -1;
    int best_polarity = -1;

    for (int pi = 0; pi < 2 && best_phase < 0; ++pi) {
        int this_invert = polarities[pi];
        for (int phase = 0; phase < sps; ++phase) {
            size_t mid_offset = (size_t)phase + (size_t)sps / 2u;
            size_t n_bits = 0;
            for (size_t i = mid_offset; i < n_samples; i += (size_t)sps) {
                int bit;
                if (this_invert) {
                    bit = (dc_blocked[i] < 0.0f) ? 1 : 0;
                } else {
                    bit = (dc_blocked[i] > 0.0f) ? 1 : 0;
                }
                tmp_bits[n_bits++] = (uint8_t)bit;
            }
            size_t off = find_u32_pattern(tmp_bits, n_bits,
                                          ASM_BIG_ENDIAN_U32, sync_max_ham,
                                          min_bit_offset);
            if (off != (size_t)-1) {
                size_t copy_bits = n_bits - off;
                memcpy(out_bits, tmp_bits + off, copy_bits);
                *n_bits_out = copy_bits;
                best_sync = off;
                best_phase = phase;
                best_polarity = this_invert;
                break;
            }
        }
    }
    if (polarity_used) *polarity_used = best_polarity;

    free(tmp_bits);
    free(dc_blocked);

    if (best_phase < 0) {
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
