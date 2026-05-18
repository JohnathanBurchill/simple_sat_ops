/*

    Simple Satellite Operations  src/dsp/sw_nco.h

    Software numerically-controlled oscillator (NCO) that rotates a
    complex int16 IQ stream by exp(-j 2π f_hz · t). Used to apply
    Doppler correction after the SDR captures a fixed-LO baseband
    signal — the phase accumulator is continuous across both apply
    calls and frequency updates, so a smooth Doppler trajectory yields
    a phase-coherent corrected stream.

    Extracted from b210_rx_tx_core.c so it can be unit-tested without
    UHD. The pump path in b210_rx_tx_core composes one of these into
    its core struct and calls sw_nco_apply() right after decimation.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#ifndef SSO_SW_NCO_H
#define SSO_SW_NCO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double phase_rad;       // running phase, wrapped to [-π, π] after each apply
    double freq_hz;         // current rotation frequency; 0 = pass-through
    double sample_rate_hz;  // samples per second on the IQ stream
} sw_nco_t;

// Reset the NCO to zero phase, zero frequency, configured for fs.
// Required before the first apply.
void   sw_nco_init(sw_nco_t *nco, double sample_rate_hz);

// Set the rotation frequency (Hz). The phase accumulator is NOT
// reset — that's the whole point of a phase-coherent NCO. Pass 0 to
// turn the NCO into a pass-through (sw_nco_apply short-circuits).
void   sw_nco_set_freq(sw_nco_t *nco, double freq_hz);
double sw_nco_get_freq(const sw_nco_t *nco);

// Rotate `n_pairs` interleaved sc16 IQ samples in place by
// exp(-j 2π freq_hz · t / sample_rate_hz). Magnitude is preserved
// exactly (modulo int16 saturation on rotated samples that round past
// ±32767). Safe to call with n_pairs == 0.
void   sw_nco_apply(sw_nco_t *nco, int16_t *iq_inout, size_t n_pairs);

#ifdef __cplusplus
}
#endif

#endif // SSO_SW_NCO_H
