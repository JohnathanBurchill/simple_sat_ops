/*

   Simple Satellite Operations  fir_decim.h

   Lowpass FIR decimator for interleaved sc16 IQ. The B210 streams at
   240 kS/s minimum-ish; a 9600-baud GMSK signal is only ~12 kHz wide,
   so feeding raw IQ into an atan2 FM discriminator buries the carrier
   under 230+ kHz of noise. This module narrows the IQ to a sane post-
   decim rate (e.g. 240 kHz → 48 kHz, M=5) before the demod runs.

   Hamming-windowed-sinc design, computed at construction. Coefficients
   are normalised so DC gain is exactly 1 — output PCM levels match the
   no-decim case for a clean carrier.

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

#ifndef FIR_DECIM_H
#define FIR_DECIM_H

#include <stddef.h>
#include <stdint.h>

typedef struct fir_decim_iq fir_decim_iq_t;

// Build a Hamming-windowed-sinc lowpass FIR for interleaved sc16 IQ,
// decimating by integer factor M. fc_hz is the -6 dB cutoff at the
// input rate fs_in_hz; pick it well below 0.5 * (fs_in_hz / M) so the
// stopband suppresses what would otherwise alias into the output.
//
// Returns NULL on alloc failure or invalid params (M==0, ntaps==0,
// non-positive rates, or fc_hz >= fs_in_hz / 2).
fir_decim_iq_t *fir_decim_iq_new(double fs_in_hz, double fc_hz,
                                 unsigned ntaps, unsigned M);

void fir_decim_iq_free(fir_decim_iq_t *f);

// Push n_in_pairs IQ pairs (each 2 int16s, interleaved) through the
// filter. Up to floor((n_in_pairs + saved_phase) / M) output pairs are
// written to iq_out, capped by cap_out_pairs. State (delay line +
// decimation phase) is preserved across calls.
//
// Returns the number of output IQ pairs written.
size_t fir_decim_iq_push(fir_decim_iq_t *f,
                         const int16_t *iq_in, size_t n_in_pairs,
                         int16_t *iq_out, size_t cap_out_pairs);

// Read-back accessors.
unsigned fir_decim_iq_M    (const fir_decim_iq_t *f);
unsigned fir_decim_iq_ntaps(const fir_decim_iq_t *f);

#endif // FIR_DECIM_H
