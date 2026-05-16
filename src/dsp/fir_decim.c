/*

   Simple Satellite Operations  fir_decim.c

   Hamming-windowed-sinc lowpass FIR decimator for interleaved sc16 IQ.
   See fir_decim.h for the why.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "fir_decim.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct fir_decim_iq {
    unsigned   M;          // decimation factor
    unsigned   ntaps;
    float     *h;          // taps, length ntaps; sum(h) == 1 (DC gain == 1)
    float     *di;         // I delay line, circular, length ntaps
    float     *dq;         // Q delay line, circular, length ntaps
    unsigned   head;       // next slot to write (newest sample lives at head-1)
    unsigned   phase;      // counts 0..M-1; emit when phase wraps to 0
};

// Hamming-windowed sinc lowpass. Coefficients are computed unscaled
// then normalised so sum(h) == 1, which gives exactly unity DC gain
// regardless of the inevitable Hamming-window edge ripple.
static void design_lpf(float *h, unsigned n, double fc_norm)
{
    double mid = 0.5 * (double)(n - 1);
    double sum = 0.0;
    for (unsigned k = 0; k < n; k++) {
        double m = (double)k - mid;
        double s;
        if (m == 0.0) {
            s = 2.0 * fc_norm;
        } else {
            s = sin(2.0 * M_PI * fc_norm * m) / (M_PI * m);
        }
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * (double)k / (double)(n - 1));
        h[k] = (float)(s * w);
        sum += (double)h[k];
    }
    if (sum != 0.0) {
        for (unsigned k = 0; k < n; k++) {
            h[k] = (float)((double)h[k] / sum);
        }
    }
}

fir_decim_iq_t *fir_decim_iq_new(double fs_in_hz, double fc_hz,
                                 unsigned ntaps, unsigned M)
{
    if (ntaps == 0 || M == 0
        || fs_in_hz <= 0.0 || fc_hz <= 0.0
        || fc_hz >= 0.5 * fs_in_hz) {
        return NULL;
    }
    fir_decim_iq_t *f = (fir_decim_iq_t *)calloc(1, sizeof(*f));
    if (f == NULL) return NULL;
    f->M     = M;
    f->ntaps = ntaps;
    f->h  = (float *)calloc(ntaps, sizeof(float));
    f->di = (float *)calloc(ntaps, sizeof(float));
    f->dq = (float *)calloc(ntaps, sizeof(float));
    if (f->h == NULL || f->di == NULL || f->dq == NULL) {
        fir_decim_iq_free(f);
        return NULL;
    }
    design_lpf(f->h, ntaps, fc_hz / fs_in_hz);
    return f;
}

void fir_decim_iq_free(fir_decim_iq_t *f)
{
    if (f == NULL) return;
    free(f->h);
    free(f->di);
    free(f->dq);
    free(f);
}

unsigned fir_decim_iq_M    (const fir_decim_iq_t *f) { return f ? f->M     : 0; }
unsigned fir_decim_iq_ntaps(const fir_decim_iq_t *f) { return f ? f->ntaps : 0; }

size_t fir_decim_iq_push(fir_decim_iq_t *f,
                         const int16_t *iq_in, size_t n_in_pairs,
                         int16_t *iq_out, size_t cap_out_pairs)
{
    if (f == NULL || iq_in == NULL || iq_out == NULL) return 0;
    const unsigned N = f->ntaps;
    size_t n_out = 0;
    for (size_t i = 0; i < n_in_pairs; i++) {
        f->di[f->head] = (float)iq_in[2*i + 0];
        f->dq[f->head] = (float)iq_in[2*i + 1];
        f->head = (f->head + 1 == N) ? 0u : (f->head + 1);
        f->phase++;
        if (f->phase != f->M) continue;
        f->phase = 0;
        if (n_out >= cap_out_pairs) continue;

        // Convolve h[j] * x[k - j] for j = 0..N-1.
        // Newest sample (x[k]) sits at (head - 1) % N; walk backward.
        float yi = 0.0f, yq = 0.0f;
        unsigned idx = (f->head == 0u) ? (N - 1u) : (f->head - 1u);
        for (unsigned j = 0; j < N; j++) {
            yi += f->h[j] * f->di[idx];
            yq += f->h[j] * f->dq[idx];
            idx = (idx == 0u) ? (N - 1u) : (idx - 1u);
        }
        if (yi >  32767.0f) yi =  32767.0f;
        if (yi < -32768.0f) yi = -32768.0f;
        if (yq >  32767.0f) yq =  32767.0f;
        if (yq < -32768.0f) yq = -32768.0f;
        iq_out[2*n_out + 0] = (int16_t)lrintf(yi);
        iq_out[2*n_out + 1] = (int16_t)lrintf(yq);
        n_out++;
    }
    return n_out;
}
