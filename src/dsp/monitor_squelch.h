/*

    Simple Satellite Operations  monitor_squelch.h

    Carrier-presence squelch for FM-demoded PCM. Replaces the older
    FM-noise gate (which closed when high-band noise stayed loud,
    relying on FM capture to silence noise during a beacon). The new
    gate is the inverse: it OPENS when the data-band power
    (default 4500–5100 Hz, the GMSK preamble tone at half the bit
    rate) rises relative to out-of-band noise (default 8000 Hz to
    fs/2), which is the same signature beacon_detect uses to find
    bursts post-pass.

    The point of this gate is audible verification of carrier presence
    during a live pass — you hear a half-second of audio per beacon
    burst, silence in between, with no human in the loop.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#ifndef MONITOR_SQUELCH_H
#define MONITOR_SQUELCH_H

#include "biquad.h"

#include <stddef.h>
#include <stdint.h>

typedef enum {
    MSQ_OFF = 0,
    MSQ_AUTO_BOOTSTRAPPING,
    MSQ_AUTO_ENGAGED,
    MSQ_FIXED,
} msq_mode_t;

typedef struct monitor_squelch {
    msq_mode_t mode;

    // 4th-order BPF cascades for the two detector bands.
    biquad_t sig1, sig2;
    biquad_t noise1, noise2;

    // Smoothed |y|² for each band (1-pole IIR, ~30 ms time constant).
    double smooth_alpha;
    double sig_sq;
    double noise_sq;

    // Detection ratio in dB (10·log10(sig_sq/noise_sq)). Threshold is
    // crossed when `ratio_db > thresh_db`.
    double ratio_db;
    double thresh_db;

    // Gate stays open for hold_duration samples after each crossing,
    // so a 50 ms beacon yields ~hold_duration_samples of audible audio.
    size_t hold_samples_remaining;
    size_t hold_duration;

    // Bootstrap state for AUTO mode.
    size_t boot_target;
    size_t boot_count;
    double boot_sum_ratio_db;
    double auto_offset_db;        // threshold = boot_mean + auto_offset_db
    double boot_floor_db;         // remembered boot_mean for status string
} monitor_squelch_t;

typedef struct monitor_squelch_params {
    double rate_hz;          // PCM rate (required)

    // Detector bands. Defaults applied when zero/negative are given:
    //   sig:   [4500, 5100] Hz
    //   noise: [8000, fs/2 - 1000] Hz
    double sig_lo_hz;
    double sig_hi_hz;
    double noise_lo_hz;
    double noise_hi_hz;

    double smooth_tau_s;     // default 0.030
    double hold_s;           // default 0.5
    double boot_window_s;    // default 1.0
    double auto_offset_db;   // default 3.0

    msq_mode_t init_mode;    // OFF / AUTO_BOOTSTRAPPING / FIXED
    double init_thresh_db;   // for FIXED only
} monitor_squelch_params_t;

// Initialise. Defaults are applied for any non-positive p->* field.
// rate_hz must be positive.
void monitor_squelch_init(monitor_squelch_t *s,
                          const monitor_squelch_params_t *p);

void monitor_squelch_set_auto(monitor_squelch_t *s);
void monitor_squelch_set_off(monitor_squelch_t *s);
void monitor_squelch_set_fixed_db(monitor_squelch_t *s, double thresh_db);

// Render a one-line status. Caller passes a buffer of >= 80 chars.
void monitor_squelch_status(const monitor_squelch_t *s,
                            char *buf, size_t cap);

// Run n samples through the detector and gate. pcm_in and pcm_out may
// alias. Bootstrap samples are emitted as silence. After bootstrap the
// gate opens for `hold_s` whenever the ratio crosses thresh_db.
void monitor_squelch_process(monitor_squelch_t *s,
                             const int16_t *pcm_in,
                             int16_t *pcm_out,
                             size_t n);

#endif // MONITOR_SQUELCH_H
