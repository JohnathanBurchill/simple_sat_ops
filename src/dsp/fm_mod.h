/*

    Simple Satellite Operations  src/dsp/fm_mod.h

    Narrowband FM modulator: integrate an audio PCM stream into carrier
    phase and emit constant-amplitude sc16 IQ, plus a cosine key-up/
    key-down ramp. Lifted out of tx_burst.c so the voice tools
    (ham_speak) and the telecommand TX path share one modulator.

    The phase accumulator lives in a caller-owned fm_mod_t rather than a
    file-local static, so independent transmits can't silently share
    phase. Within one logical transmission, pass the same fm_mod_t to
    every block so chunk boundaries stay phase-continuous.

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

#ifndef SSO_DSP_FM_MOD_H
#define SSO_DSP_FM_MOD_H

#include <stddef.h>
#include <stdint.h>

// Carrier-phase accumulator for the FM modulator. Reset with
// fm_mod_init before a transmission; reuse across fm_mod_block calls
// that belong to the same transmission.
typedef struct fm_mod {
    double phi;   // running phase, radians, kept in (-pi, pi]
} fm_mod_t;

// Reset the phase accumulator to 0.
void fm_mod_init(fm_mod_t *m);

// Modulate n_pcm int16 audio samples into iq_out (which must hold at
// least 2*n_pcm int16: interleaved I,Q). Full-scale audio (+/-32767)
// maps to +/-dev_hz instantaneous deviation at sample rate fs. Output
// amplitude is the constant 22937 (~ -3 dBFS), same as the telecommand
// TX path. Phase carries across calls via *m.
void fm_mod_block(fm_mod_t *m, const int16_t *pcm, size_t n_pcm,
                  double dev_hz, double fs, int16_t *iq_out);

// Cosine ramp-up over the first ramp_n IQ pairs and ramp-down over the
// last ramp_n, to soften the key-up/key-down click. ramp_n is clamped
// to n_samps/2; ramp_n == 0 is a no-op.
void fm_apply_ramp(int16_t *iq, size_t n_samps, size_t ramp_n);

#endif // SSO_DSP_FM_MOD_H
