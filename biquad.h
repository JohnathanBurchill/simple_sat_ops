/*

    Simple Satellite Operations  biquad.h

    Tiny biquad helper shared by the post-FM-demod tools that need to
    isolate a frequency band on streaming PCM. Cookbook bandpass
    coefficients (constant-skirt-gain form). One section is 2nd order;
    cascade two for 4th order with the same f0/BW.

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

#ifndef BIQUAD_H
#define BIQUAD_H

typedef struct biquad {
    double b0, b1, b2, a1, a2;
    double z1, z2;
} biquad_t;

// Compute coefficients for a 2nd-order bandpass at f0 with bandwidth
// bw_hz, sampled at fs. Resets the delay line. Q is f0 / bw_hz, floored
// at 0.5 to keep the filter well-conditioned for very wide bands.
void biquad_bpf(biquad_t *bq, double f0, double bw_hz, double fs);

// Step one sample through the filter. Transposed direct form II.
double biquad_step(biquad_t *bq, double x);

#endif // BIQUAD_H
