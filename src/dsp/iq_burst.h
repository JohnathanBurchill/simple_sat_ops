/*

    Simple Satellite Operations  iq_burst.h

    Broadband-burst detector for the post-NCO IQ stream. Per FFT frame
    (length n_fft, Hann-windowed), maintains an asymmetric per-bin
    running noise floor (fast down, slow up — so a brief packet doesn't
    pull the floor up with itself), counts how many bins are
    simultaneously above floor + threshold_dB, and exposes that count
    plus the maximum per-bin excess as a snapshot.

    Why: a narrowband signal (a CW or Doppler-swept carrier) lights up
    1-2 FFT bins. A wideband packet (e.g. CTS1 9600-baud GFSK,
    occupying ~12 kHz) lights up tens to hundreds of bins. The
    bright-bin COUNT lets the operator-side ribbon tell those two
    apart without any audio-band gate.

    Copyright (C) 2026  Johnathan K Burchill — GPLv3 or later.
*/

#ifndef IQ_BURST_H
#define IQ_BURST_H

#include <stddef.h>
#include <stdint.h>

typedef struct iq_burst iq_burst_t;

// Allocate and initialise an iq_burst detector.
//   n_fft          — power of two, e.g. 512. Bin width = sample_rate / n_fft.
//   sample_rate_hz — IQ rate (the post-decim rate from b210_rx_tx_core,
//                    typically 96 kHz). Used for documentation only —
//                    all detection is on FFT-bin power vs floor.
//   threshold_db   — bin must be > floor + this to count as bright.
//                    Per-bin noise-power variance in IQ FFT is ~5.6 dB
//                    so threshold below 8 dB starts producing many
//                    false-positives per frame from noise alone.
//                    10 dB is a sensible default.
//   floor_tau_s    — IIR time constant for the per-bin noise-floor
//                    estimator. Long enough that a transient packet
//                    barely budges it; short enough that genuine drift
//                    (AGC, antenna temp) is tracked. 2.0 s default.
//                    The floor IS FROZEN per-bin when that bin is
//                    currently above threshold — so a carrier sitting
//                    in one bin doesn't poison the floor for that bin.
// Returns NULL on bad params / alloc failure.
iq_burst_t *iq_burst_new(unsigned n_fft, double sample_rate_hz,
                         double threshold_db,
                         double floor_tau_s);

void iq_burst_free(iq_burst_t *b);

// Feed interleaved sc16 IQ samples (I,Q,I,Q,...). Internally
// accumulates into an n_fft-sample frame; emits one FFT per filled
// frame and updates the latest snapshot. Multiple frames per push
// are processed in turn. State (floor, accumulator) persists across
// pushes.
void iq_burst_push(iq_burst_t *b, const int16_t *iq, size_t n_pairs);

// Latest snapshot — set on each completed FFT frame. Safe to read
// without locking from another thread (single-writer / single-reader
// on a double or int is atomic on the platforms we care about).

// Number of FFT bins that exceed the per-bin floor by threshold_db
// or more in the most recently completed frame. 0 if no frame has
// completed yet. A narrowband signal (CW / unswept carrier) reads
// 1-2; a wideband packet reads tens to hundreds.
int iq_burst_bright_bins(const iq_burst_t *b);

// dB excess of the brightest bin above its floor in the most recently
// completed frame. -INFINITY when no frame has completed yet.
double iq_burst_peak_excess_db(const iq_burst_t *b);

// Number of FFT frames processed since init. Useful for testing.
unsigned long iq_burst_frame_count(const iq_burst_t *b);

#endif // IQ_BURST_H
