/*

    Simple Satellite Operations  utils/wav_read.c

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

#include "wav_read.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int wav_read_pcm16(const char *path,
                   int16_t **out_samples, size_t *out_n,
                   int *out_samp_rate, int *out_channels)
{
    if (path == NULL || out_samples == NULL || out_n == NULL) {
        return -1;
    }

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        fprintf(stderr, "wav_read: open(%s): %s\n", path, strerror(errno));
        return -1;
    }

    uint8_t header[12];
    if (fread(header, 1, sizeof(header), f) != sizeof(header)) {
        fprintf(stderr, "wav_read: %s: short read at RIFF header\n", path);
        fclose(f);
        return -1;
    }
    if (memcmp(header, "RIFF", 4) != 0 || memcmp(header + 8, "WAVE", 4) != 0) {
        fprintf(stderr, "wav_read: %s: not a RIFF/WAVE file\n", path);
        fclose(f);
        return -1;
    }

    uint16_t audio_format = 0, channels = 0, bits_per_sample = 0;
    uint32_t samp_rate = 0;
    int16_t *samples = NULL;
    size_t n_samples = 0;
    int got_fmt = 0, got_data = 0;

    while (!got_data) {
        uint8_t chunk_hdr[8];
        if (fread(chunk_hdr, 1, sizeof(chunk_hdr), f) != sizeof(chunk_hdr)) {
            fprintf(stderr, "wav_read: %s: unexpected EOF before data chunk\n", path);
            free(samples);
            fclose(f);
            return -1;
        }
        uint32_t chunk_size = read_u32_le(chunk_hdr + 4);

        if (memcmp(chunk_hdr, "fmt ", 4) == 0) {
            if (chunk_size < 16) {
                fprintf(stderr, "wav_read: %s: fmt chunk too small (%u)\n",
                        path, chunk_size);
                fclose(f);
                return -1;
            }
            uint8_t fmt_buf[16];
            if (fread(fmt_buf, 1, 16, f) != 16) {
                fprintf(stderr, "wav_read: %s: short read in fmt chunk\n", path);
                fclose(f);
                return -1;
            }
            audio_format    = read_u16_le(fmt_buf +  0);
            channels        = read_u16_le(fmt_buf +  2);
            samp_rate       = read_u32_le(fmt_buf +  4);
            bits_per_sample = read_u16_le(fmt_buf + 14);
            if (chunk_size > 16) {
                if (fseek(f, (long)(chunk_size - 16), SEEK_CUR) != 0) {
                    fprintf(stderr, "wav_read: %s: seek past fmt tail failed\n", path);
                    fclose(f);
                    return -1;
                }
            }
            got_fmt = 1;
        } else if (memcmp(chunk_hdr, "data", 4) == 0) {
            if (!got_fmt) {
                fprintf(stderr, "wav_read: %s: data chunk before fmt\n", path);
                fclose(f);
                return -1;
            }
            if (audio_format != 1) {
                fprintf(stderr,
                        "wav_read: %s: only PCM (fmt=1) supported (got fmt=%u)\n",
                        path, audio_format);
                fclose(f);
                return -1;
            }
            if (bits_per_sample != 16) {
                fprintf(stderr,
                        "wav_read: %s: only 16-bit PCM supported (got %u bits)\n",
                        path, bits_per_sample);
                fclose(f);
                return -1;
            }
            n_samples = chunk_size / sizeof(int16_t);
            samples = (int16_t *)malloc(n_samples * sizeof(int16_t));
            if (samples == NULL) {
                fprintf(stderr, "wav_read: out of memory for %zu samples\n", n_samples);
                fclose(f);
                return -1;
            }
            if (fread(samples, sizeof(int16_t), n_samples, f) != n_samples) {
                fprintf(stderr, "wav_read: %s: short read in data chunk\n", path);
                free(samples);
                fclose(f);
                return -1;
            }
            got_data = 1;
        } else {
            // Unknown chunk — skip it.
            if (fseek(f, (long)chunk_size, SEEK_CUR) != 0) {
                fprintf(stderr, "wav_read: %s: seek past unknown chunk failed\n", path);
                fclose(f);
                return -1;
            }
        }
    }
    fclose(f);

    *out_samples = samples;
    *out_n = n_samples;
    if (out_samp_rate) *out_samp_rate = (int)samp_rate;
    if (out_channels)  *out_channels  = (int)channels;
    return 0;
}
