/*

    Simple Satellite Operations  monitor_squelch.c

    See monitor_squelch.h for the ratio detector's "why".

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include "monitor_squelch.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void monitor_squelch_init(monitor_squelch_t *s,
                          const monitor_squelch_params_t *p)
{
    memset(s, 0, sizeof *s);

    double fs = p->rate_hz;
    // rate_hz is a documented precondition (> 0). Without a positive rate the
    // band math and smoothing constant are meaningless, so leave the squelch
    // OFF (memset already set mode = MSQ_OFF) rather than build garbage.
    if (!(fs > 0.0)) return;

    double sig_lo   = p->sig_lo_hz   > 0 ? p->sig_lo_hz   : 4500.0;
    double sig_hi   = p->sig_hi_hz   > 0 ? p->sig_hi_hz   : 5100.0;
    double noise_lo = p->noise_lo_hz > 0 ? p->noise_lo_hz : 8000.0;
    double noise_hi = p->noise_hi_hz > 0 ? p->noise_hi_hz : (0.5 * fs - 1000.0);
    double smooth_tau = p->smooth_tau_s  > 0 ? p->smooth_tau_s  : 0.030;
    double hold_s     = p->hold_s        > 0 ? p->hold_s        : 0.5;
    double boot_s     = p->boot_window_s > 0 ? p->boot_window_s : 1.0;
    double offset_db  = p->auto_offset_db != 0 ? p->auto_offset_db : 3.0;
    // Carrier-lockout. carrier_lockout_s defaults to 1.5 s; pass < 0
    // to disable (set to 0 internally → never trips). lockout_release_s
    // defaults to 0.5 s.
    double lockout_s  = (p->carrier_lockout_s < 0.0) ? 0.0
                      : (p->carrier_lockout_s == 0.0 ? 1.5 : p->carrier_lockout_s);
    double release_s  = p->lockout_release_s > 0.0 ? p->lockout_release_s : 0.5;

    // Keep every band edge below Nyquist and each high edge above its low
    // edge, so a sample rate too low for the default bands (fs <= ~18 kHz,
    // where the default noise_hi falls below noise_lo) can't hand biquad_bpf
    // a negative bandwidth. All no-ops at the live 48 / 96 kHz rates.
    double nyq = 0.5 * fs;
    if (sig_hi   >= nyq) sig_hi   = nyq - 1000.0;
    if (noise_hi >= nyq) noise_hi = nyq - 1000.0;
    if (sig_hi   <= sig_lo)   sig_lo   = 0.5 * sig_hi;
    if (noise_hi <= noise_lo) noise_lo = 0.5 * noise_hi;

    biquad_bpf(&s->sig1,   0.5 * (sig_lo + sig_hi),
               sig_hi - sig_lo, fs);
    biquad_bpf(&s->sig2,   0.5 * (sig_lo + sig_hi),
               sig_hi - sig_lo, fs);
    biquad_bpf(&s->noise1, 0.5 * (noise_lo + noise_hi),
               noise_hi - noise_lo, fs);
    biquad_bpf(&s->noise2, 0.5 * (noise_lo + noise_hi),
               noise_hi - noise_lo, fs);

    s->smooth_alpha   = exp(-1.0 / (smooth_tau * fs));
    s->hold_duration  = (size_t)(hold_s * fs);
    s->boot_target    = (size_t)(boot_s * fs);
    s->auto_offset_db = offset_db;
    s->carrier_lockout_samples = (size_t)(lockout_s * fs);
    s->lockout_release_samples = (size_t)(release_s * fs);

    if (p->init_mode == MSQ_AUTO_BOOTSTRAPPING) {
        s->mode = MSQ_AUTO_BOOTSTRAPPING;
    } else if (p->init_mode == MSQ_FIXED) {
        s->mode      = MSQ_FIXED;
        s->thresh_db = p->init_thresh_db;
    } else {
        s->mode = MSQ_OFF;
    }
}

void monitor_squelch_set_auto(monitor_squelch_t *s)
{
    s->mode             = MSQ_AUTO_BOOTSTRAPPING;
    s->boot_count       = 0;
    s->boot_sum_ratio_db = 0.0;
    s->boot_floor_db    = 0.0;
    s->thresh_db        = 0.0;
    s->hold_samples_remaining = 0;
    s->above_thresh_samples   = 0;
    s->below_thresh_samples   = 0;
    s->lockout_active         = 0;
}

void monitor_squelch_set_off(monitor_squelch_t *s)
{
    s->mode = MSQ_OFF;
}

void monitor_squelch_set_fixed_db(monitor_squelch_t *s, double thresh_db)
{
    s->mode      = MSQ_FIXED;
    s->thresh_db = thresh_db;
    s->hold_samples_remaining = 0;
    s->above_thresh_samples   = 0;
    s->below_thresh_samples   = 0;
    s->lockout_active         = 0;
}

void monitor_squelch_status(const monitor_squelch_t *s, char *buf, size_t cap)
{
    switch (s->mode) {
    case MSQ_OFF:
        snprintf(buf, cap, "off");
        break;
    case MSQ_AUTO_BOOTSTRAPPING:
        snprintf(buf, cap, "auto(boot %zu/%zu)",
                 s->boot_count, s->boot_target);
        break;
    case MSQ_AUTO_ENGAGED:
        snprintf(buf, cap, "auto(thr=%+.1fdB floor=%+.1fdB now=%+.1fdB)",
                 s->thresh_db, s->boot_floor_db, s->ratio_db);
        break;
    case MSQ_FIXED:
        snprintf(buf, cap, "thr=%+.1fdB now=%+.1fdB",
                 s->thresh_db, s->ratio_db);
        break;
    }
}

void monitor_squelch_process(monitor_squelch_t *s,
                             const int16_t *pcm_in,
                             int16_t *pcm_out, size_t n)
{
    const double alpha = s->smooth_alpha;
    const double inv_a = 1.0 - alpha;
    for (size_t i = 0; i < n; i++) {
        double x = (double)pcm_in[i];

        double sy = biquad_step(&s->sig1, x);
        sy = biquad_step(&s->sig2, sy);
        s->sig_sq = alpha * s->sig_sq + inv_a * sy * sy;

        double ny = biquad_step(&s->noise1, x);
        ny = biquad_step(&s->noise2, ny);
        s->noise_sq = alpha * s->noise_sq + inv_a * ny * ny;

        // Use sig_sq / noise_sq directly (ratio of |y|², so 10·log10
        // gives dB). Floor both at 1 PCM-unit² so log doesn't blow up
        // on a totally silent input.
        double sden = s->sig_sq   < 1.0 ? 1.0 : s->sig_sq;
        double nden = s->noise_sq < 1.0 ? 1.0 : s->noise_sq;
        s->ratio_db = 10.0 * log10(sden / nden);

        if (s->mode == MSQ_AUTO_BOOTSTRAPPING) {
            s->boot_sum_ratio_db += s->ratio_db;
            s->boot_count++;
            if (s->boot_count >= s->boot_target) {
                double mean = s->boot_sum_ratio_db / (double)s->boot_target;
                s->boot_floor_db = mean;
                s->thresh_db     = mean + s->auto_offset_db;
                s->mode          = MSQ_AUTO_ENGAGED;
            }
            pcm_out[i] = 0;
            continue;
        }

        // Carrier-lockout bookkeeping. Tracks how long the ratio has
        // stayed above (or below) the threshold across consecutive
        // samples; toggles lockout_active when the above-thresh run
        // exceeds carrier_lockout_samples, clears it once we get a
        // sustained below-thresh gap. Skipped when the feature is
        // disabled (carrier_lockout_samples == 0).
        if (s->carrier_lockout_samples > 0) {
            if (s->ratio_db > s->thresh_db) {
                s->above_thresh_samples++;
                s->below_thresh_samples = 0;
                if (s->above_thresh_samples > s->carrier_lockout_samples) {
                    s->lockout_active = 1;
                }
            } else {
                s->above_thresh_samples = 0;
                s->below_thresh_samples++;
                if (s->below_thresh_samples >= s->lockout_release_samples) {
                    s->lockout_active = 0;
                }
            }
        }

        int open;
        if (s->mode == MSQ_OFF) {
            open = 1;
        } else {
            // Only refresh the hold timer if we're not in a carrier
            // lockout. The lockout still lets any already-running hold
            // counter drain naturally, so the tail end of a real packet
            // that happened JUST before the lockout engaged is heard.
            if (s->ratio_db > s->thresh_db && !s->lockout_active) {
                s->hold_samples_remaining = s->hold_duration;
            }
            open = (s->hold_samples_remaining > 0);
            if (s->hold_samples_remaining > 0) {
                s->hold_samples_remaining--;
            }
        }
        pcm_out[i] = open ? pcm_in[i] : (int16_t)0;
    }
}
