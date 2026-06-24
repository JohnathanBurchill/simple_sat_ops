/*

    Simple Satellite Operations  src/dsp/iq_burst.c

    Implementation of the broadband-burst detector. See iq_burst.h
    for what and why. Inline radix-2 complex FFT identical in form to
    the one in utils/gen_waterfall.c — keeping the module self-
    contained avoids dragging the waterfall renderer into the live
    receive path (and into the iq_burst selftest).

    Copyright (C) 2026  Johnathan K Burchill — GPLv3 or later.
*/

#include "iq_burst.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct iq_burst {
    unsigned n_fft;
    double   sample_rate_hz;
    double   threshold_db;
    double   alpha;          // floor IIR coefficient when bin is NOT bright

    // Hann window, length n_fft.
    float   *win;
    // Frame scratch: complex floats.
    float   *re;
    float   *im;
    // Per-bin floor in dB. Initialised on first completed frame.
    float   *floor_db;
    int      have_floor;

    // Sample accumulator. Buffers up int16 IQ pairs until we have a
    // full FFT frame's worth. Allocated as 2*n_fft int16s for
    // interleaved I,Q.
    int16_t *accum;
    unsigned accum_pairs;

    // Last snapshot.
    int           last_bright_bins;
    double        last_peak_excess_db;
    unsigned long frame_count;
};

static int is_pow2(unsigned n)
{
    return n > 0 && (n & (n - 1)) == 0;
}

// Radix-2 in-place complex FFT (Cooley-Tukey, iterative). Forward
// transform. n must be a power of two. Mirrored from gen_waterfall.c.
static void fft_bit_reverse(float *re, float *im, unsigned n)
{
    unsigned j = 0;
    for (unsigned i = 1; i < n; ++i) {
        unsigned bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
}

static void fft_forward(float *re, float *im, unsigned n)
{
    fft_bit_reverse(re, im, n);
    for (unsigned len = 2; len <= n; len <<= 1) {
        double ang = -2.0 * M_PI / (double) len;
        double wr_step = cos(ang);
        double wi_step = sin(ang);
        unsigned half = len >> 1;
        for (unsigned i = 0; i < n; i += len) {
            double wr = 1.0, wi = 0.0;
            for (unsigned k = 0; k < half; ++k) {
                unsigned a = i + k;
                unsigned b = a + half;
                double tr = wr * re[b] - wi * im[b];
                double ti = wr * im[b] + wi * re[b];
                re[b] = (float)(re[a] - tr);
                im[b] = (float)(im[a] - ti);
                re[a] = (float)(re[a] + tr);
                im[a] = (float)(im[a] + ti);
                double nwr = wr * wr_step - wi * wi_step;
                double nwi = wr * wi_step + wi * wr_step;
                wr = nwr;
                wi = nwi;
            }
        }
    }
}

iq_burst_t *iq_burst_new(unsigned n_fft, double sample_rate_hz,
                         double threshold_db,
                         double floor_tau_s)
{
    if (!is_pow2(n_fft) || n_fft < 16 || sample_rate_hz <= 0.0) return NULL;
    if (floor_tau_s <= 0.0) return NULL;

    iq_burst_t *b = (iq_burst_t *) calloc(1, sizeof *b);
    if (!b) return NULL;
    b->n_fft = n_fft;
    b->sample_rate_hz = sample_rate_hz;
    b->threshold_db = threshold_db;
    // Frame rate = sample_rate / n_fft (no overlap). α = 1 - exp(-T/τ)
    // where T is the seconds between frames.
    double frame_period_s = (double) n_fft / sample_rate_hz;
    b->alpha = 1.0 - exp(-frame_period_s / floor_tau_s);
    b->win      = (float *)  calloc(n_fft, sizeof(float));
    b->re       = (float *)  calloc(n_fft, sizeof(float));
    b->im       = (float *)  calloc(n_fft, sizeof(float));
    b->floor_db = (float *)  calloc(n_fft, sizeof(float));
    b->accum    = (int16_t *)calloc((size_t) n_fft * 2, sizeof(int16_t));
    if (!b->win || !b->re || !b->im || !b->floor_db || !b->accum) {
        iq_burst_free(b);
        return NULL;
    }
    for (unsigned k = 0; k < n_fft; ++k) {
        b->win[k] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * (float) k
                                        / (float)(n_fft - 1)));
    }
    b->last_peak_excess_db = -INFINITY;
    return b;
}

void iq_burst_free(iq_burst_t *b)
{
    if (!b) return;
    free(b->win);
    free(b->re);
    free(b->im);
    free(b->floor_db);
    free(b->accum);
    free(b);
}

// Process one completed FFT frame: window the accumulated IQ, FFT,
// compute power in dB per bin, update floor (asymmetric IIR), count
// bright bins.
static void process_frame(iq_burst_t *b)
{
    unsigned N = b->n_fft;
    for (unsigned k = 0; k < N; ++k) {
        float w = b->win[k];
        float I = (float) b->accum[k * 2 + 0];
        float Q = (float) b->accum[k * 2 + 1];
        b->re[k] = I * w;
        b->im[k] = Q * w;
    }
    fft_forward(b->re, b->im, N);

    // Power in dB per bin. `p` is a sum of squared int16-scale FFT bins,
    // so any non-zero bin has p >= 1; the 1e-6 term is far below that and
    // only does one thing — keep log10 finite when a bin is *exactly* zero.
    // It is not a calibrated noise floor: absolute level is irrelevant here
    // because we only ever compare a bin to its own running floor below.
    int bright = 0;
    float peak_excess = -INFINITY;
    for (unsigned k = 0; k < N; ++k) {
        double p = (double) b->re[k] * b->re[k]
                 + (double) b->im[k] * b->im[k];
        float p_db = (float)(10.0 * log10(p + 1e-6));
        if (!b->have_floor) {
            b->floor_db[k] = p_db;
            continue;
        }
        float excess = p_db - b->floor_db[k];
        if (excess > peak_excess) peak_excess = excess;
        if (excess > b->threshold_db) {
            // Bin is bright: count it AND freeze the floor. A carrier
            // that sits in one bin doesn't pull that bin's floor up;
            // a brief broadband burst doesn't drag any bin's floor up.
            ++bright;
        } else {
            // Bin is in the noise. Symmetric IIR brings the floor
            // toward the bin's current dB level. Over many frames of
            // stationary noise the floor settles at the per-bin mean.
            b->floor_db[k] = (float)((1.0 - b->alpha) * b->floor_db[k]
                                     + b->alpha * (double) p_db);
        }
    }
    if (!b->have_floor) {
        b->have_floor = 1;
        // First frame seeds the floor; no detections possible against
        // it (every bin would read 0 dB excess). Report 0.
        bright = 0;
        peak_excess = 0.0f;
    }
    b->last_bright_bins    = bright;
    b->last_peak_excess_db = (double) peak_excess;
    b->frame_count++;
}

void iq_burst_push(iq_burst_t *b, const int16_t *iq, size_t n_pairs)
{
    if (!b || !iq || n_pairs == 0) return;
    unsigned N = b->n_fft;
    size_t off = 0;
    while (off < n_pairs) {
        size_t need = (size_t) N - b->accum_pairs;
        size_t take = (n_pairs - off < need) ? (n_pairs - off) : need;
        memcpy(b->accum + b->accum_pairs * 2,
               iq + off * 2,
               take * 2 * sizeof(int16_t));
        b->accum_pairs += (unsigned) take;
        off += take;
        if (b->accum_pairs == N) {
            process_frame(b);
            b->accum_pairs = 0;
        }
    }
}

int iq_burst_bright_bins(const iq_burst_t *b)
{
    return b ? b->last_bright_bins : 0;
}

double iq_burst_peak_excess_db(const iq_burst_t *b)
{
    return b ? b->last_peak_excess_db : -INFINITY;
}

unsigned long iq_burst_frame_count(const iq_burst_t *b)
{
    return b ? b->frame_count : 0;
}
