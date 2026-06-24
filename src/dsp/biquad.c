/*

    Simple Satellite Operations  biquad.c

    Audio-EQ-cookbook constant-skirt-gain bandpass biquad.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include "biquad.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void biquad_bpf(biquad_t *bq, double f0, double bw_hz, double fs)
{
    // Degenerate inputs (non-positive bandwidth or sample rate, or NaN)
    // would make Q = f0/bw_hz infinite or negative. The low-Q floor below
    // only guards the wide-band end, so without this guard the coefficients
    // collapse to a SILENT all-stop filter (b0 = b2 = 0) that kills the
    // signal. Fall back to a unity passthrough instead, so a misconfigured
    // band leaves the signal observable rather than dead.
    if (!(bw_hz > 0.0) || !(fs > 0.0)) {
        bq->b0 = 1.0; bq->b1 = 0.0; bq->b2 = 0.0;
        bq->a1 = 0.0; bq->a2 = 0.0;
        bq->z1 = bq->z2 = 0.0;
        return;
    }
    double w0    = 2.0 * M_PI * f0 / fs;
    double cw    = cos(w0);
    double sw    = sin(w0);
    double Q     = f0 / bw_hz;
    if (Q < 0.5) Q = 0.5;
    double alpha = sw / (2.0 * Q);
    double a0    = 1.0 + alpha;
    bq->b0 = alpha / a0;
    bq->b1 = 0.0;
    bq->b2 = -alpha / a0;
    bq->a1 = -2.0 * cw / a0;
    bq->a2 = (1.0 - alpha) / a0;
    bq->z1 = bq->z2 = 0.0;
}

double biquad_step(biquad_t *bq, double x)
{
    double y = bq->b0 * x + bq->z1;
    bq->z1 = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2 = bq->b2 * x - bq->a2 * y;
    return y;
}
