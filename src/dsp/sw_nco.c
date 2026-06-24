/*

    Simple Satellite Operations  src/dsp/sw_nco.c

    Implementation of the software NCO. See sw_nco.h for the public
    contract — the only subtle bit is that sw_nco_set_freq() does NOT
    reset the phase accumulator, which is what keeps continuous Doppler
    trajectories phase-coherent across set_freq() calls.

    Copyright (C) 2026  Johnathan K Burchill — GPLv3 or later.
*/

#include "sw_nco.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void sw_nco_init(sw_nco_t *nco, double sample_rate_hz)
{
    if (nco == NULL) return;
    nco->phase_rad      = 0.0;
    nco->freq_hz        = 0.0;
    nco->sample_rate_hz = sample_rate_hz;
}

void sw_nco_set_freq(sw_nco_t *nco, double freq_hz)
{
    if (nco == NULL) return;
    nco->freq_hz = freq_hz;
}

double sw_nco_get_freq(const sw_nco_t *nco)
{
    return nco ? nco->freq_hz : 0.0;
}

void sw_nco_apply(sw_nco_t *nco, int16_t *iq_inout, size_t n_pairs)
{
    if (nco == NULL || iq_inout == NULL || n_pairs == 0) return;
    if (nco->sample_rate_hz <= 0.0) return;
    const double f = nco->freq_hz;
    if (f == 0.0) return;  // pass-through fast path

    const double dphi = -2.0 * M_PI * f / nco->sample_rate_hz;
    double phase = nco->phase_rad;
    // Deliberately a fresh cos/sin per sample rather than a multiplicative
    // phasor recurrence (c *= e^{j·dphi}). A recurrence is cheaper but drifts
    // in magnitude and would need renormalisation at points that depend on
    // the chunk size — which would break the byte-for-byte equality between a
    // single apply() and the same data split into chunks that the live decode
    // and sw_nco_selftest both rely on. Evaluating per sample makes each
    // output a pure function of its absolute phase, independent of chunking.
    for (size_t i = 0; i < n_pairs; ++i) {
        double cp = cos(phase);
        double sp = sin(phase);
        double I = (double) iq_inout[i * 2 + 0];
        double Q = (double) iq_inout[i * 2 + 1];
        double rotI = I * cp - Q * sp;
        double rotQ = I * sp + Q * cp;
        // Saturate to int16 — the rotated magnitude equals the input
        // magnitude exactly, but rounding can push a full-scale sample
        // ±1 past the int16 limit.
        if (rotI >  32767.0) rotI =  32767.0;
        else if (rotI < -32768.0) rotI = -32768.0;
        if (rotQ >  32767.0) rotQ =  32767.0;
        else if (rotQ < -32768.0) rotQ = -32768.0;
        iq_inout[i * 2 + 0] = (int16_t) rotI;
        iq_inout[i * 2 + 1] = (int16_t) rotQ;
        phase += dphi;
    }
    // Wrap once per chunk; cumulative drift over a 10-minute pass at
    // 96 kSPS is ~5.8e7 samples × 2π × |f|/fs, which would otherwise
    // grow into ~1e6 radians and start to lose precision.
    phase = fmod(phase, 2.0 * M_PI);
    if (phase >  M_PI) phase -= 2.0 * M_PI;
    if (phase < -M_PI) phase += 2.0 * M_PI;
    nco->phase_rad = phase;
}
