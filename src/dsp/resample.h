/*

    Simple Satellite Operations  src/dsp/resample.h

    Tiny integer-factor upsampler for the voice TX path. ham_speak
    records mic audio at 48 kHz and the SDR transmits at 480 kHz, so the
    audio is upsampled x10 before FM modulation. Linear interpolation is
    plenty for a narrowband voice channel (the first image sits ~26 dB
    down, far outside the 3 kHz voice band); a windowed-FIR variant can
    be added later if bench listening ever calls for it.

    Copyright (C) 2026  Johnathan K Burchill

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

#ifndef SSO_DSP_RESAMPLE_H
#define SSO_DSP_RESAMPLE_H

#include <stddef.h>
#include <stdint.h>

// Upsample `n` int16 samples by the integer factor L (>= 1) using
// linear interpolation, writing up to out_cap samples to `out`.
// Produces n*L samples (the last input sample is held flat for its
// run). Returns the number of samples written, or 0 on bad arguments.
// `out` must hold at least min(n*L, out_cap) samples.
size_t resample_up_linear(const int16_t *in, size_t n, int L,
                          int16_t *out, size_t out_cap);

#endif // SSO_DSP_RESAMPLE_H
