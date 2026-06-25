/*

   Simple Satellite Operations  fm_demod.h

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

// The narrowband-FM discriminator kernel: the per-sample atan2 phase-
// difference detector, its int16 PCM scaling constant, and the IQ squared-
// magnitude used by the squelch gates.
//
// The same atan2(...) discriminator and the same k_scale formula were
// written out three times — once in the live RX/TX core (b210_rx_tx_core.c)
// and twice in the capture tool (b210_rx_capture.c, the ALSA live monitor
// and the WAV writer). The loops around them differ on purpose (the core
// and live monitor carry phase state across chunks; the WAV pass runs once
// over a whole capture and counts clipped/squelched samples), so only the
// arithmetic is shared here and each caller keeps its own loop.
//
// Header-only (static inline) so it stays zero-overhead inside those tight
// per-sample loops and adds no link dependency.

#ifndef SSO_DSP_FM_DEMOD_H
#define SSO_DSP_FM_DEMOD_H

#include <math.h>
#include <stdint.h>

// Converts the per-sample phase step (radians) the discriminator produces
// into int16 PCM, calibrated so ±fullscale_hz of deviation maps to ±32767.
// It is rate-dependent: a given number of Hz of deviation is a larger phase
// step at a lower sample rate, so the scale tracks the sample rate.
// sample_rate_hz and fullscale_hz must both be > 0.
static inline double fm_demod_k_scale(double sample_rate_hz, double fullscale_hz)
{
    return sample_rate_hz * 32767.0 / (fullscale_hz * 2.0 * M_PI);
}

// Squared magnitude of one IQ sample, |I + jQ|². Shared by the squelch gates
// (which compare it against a squared threshold to avoid a sqrt per sample).
static inline double fm_iq_mag_sq(double I, double Q)
{
    return I * I + Q * Q;
}

// One FM-discriminator PCM sample: the argument of z[k]·conj(z[k-1]), scaled
// by k_scale and clipped to the int16 range. prev_I/prev_Q is the previous
// IQ sample, (I, Q) the current one. If clipped is non-NULL it is incremented
// when the sample saturates (the capture tool's clip meter); pass NULL when
// no count is wanted.
static inline int16_t fm_demod_pcm(double prev_I, double prev_Q,
                                   double I, double Q, double k_scale,
                                   int *clipped)
{
    double dphi  = atan2(Q * prev_I - I * prev_Q, I * prev_I + Q * prev_Q);
    double pcm_d = dphi * k_scale;
    if (pcm_d >  32767.0) { pcm_d =  32767.0; if (clipped) (*clipped)++; }
    if (pcm_d < -32768.0) { pcm_d = -32768.0; if (clipped) (*clipped)++; }
    return (int16_t) lround(pcm_d);
}

#endif // SSO_DSP_FM_DEMOD_H
