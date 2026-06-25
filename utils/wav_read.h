/*

    Simple Satellite Operations  utils/wav_read.h

    Minimal WAV reader, sized just large enough for the RIFF/WAVE files
    that pcm16_write_wav() in modem.c produces. Supports 16-bit PCM only.

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

#ifndef WAV_READ_H
#define WAV_READ_H

#include <stdint.h>
#include <stddef.h>

// Reads a 16-bit PCM WAV file.
// On success, *out_samples points to a freshly malloc'd buffer of int16
// samples (interleaved if multi-channel); caller must free(). *out_n
// holds the total number of samples (across channels). *out_samp_rate
// and *out_channels receive the file's values.
// Returns 0 on success, -1 on error (with a diagnostic on stderr).
int wav_read_pcm16(const char *path,
                   int16_t **out_samples, size_t *out_n,
                   int *out_samp_rate, int *out_channels);

#endif // WAV_READ_H
