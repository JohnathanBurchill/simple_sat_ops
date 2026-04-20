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
