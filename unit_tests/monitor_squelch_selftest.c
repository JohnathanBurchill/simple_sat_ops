/*

    Simple Satellite Operations  unit_tests/monitor_squelch_selftest.c

    Coverage for src/dsp/monitor_squelch.c. The squelch is the operator's
    audible carrier-presence indicator during a live pass — open ear,
    silent in between beacons, no human in the loop. The detector is a
    4th-order BPF cascade in two bands plus a 1-pole smoother; the gate
    is a hold-time state machine. A regression in the state machine
    silently breaks audible verification of carrier presence.

    What's covered:
      - init applies defaults for zero-valued params (smooth_alpha for
        the documented 30 ms tau; hold_duration for the 0.5 s hold;
        boot_target for the 1.0 s bootstrap window).
      - init with explicit params overrides the defaults.
      - set_off / set_fixed_db / set_auto switch mode and clear hold.
      - MSQ_OFF is pure pass-through.
      - MSQ_FIXED with silence stays closed (output 0).
      - MSQ_FIXED with an in-band tone opens the gate; output non-zero.
      - Hold timer: gate stays open for ≈ hold_duration samples after
        the trigger stops, then closes.
      - MSQ_AUTO_BOOTSTRAPPING transitions to MSQ_AUTO_ENGAGED after
        boot_target samples, with threshold == boot_mean + offset.
      - Output is silenced for the duration of bootstrap.
      - Status string format for each of the four modes.
      - pcm_in and pcm_out may alias (same buffer) — the docs promise.

    Exit status: 0 = all tests passed, non-zero = failure.

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

#include "monitor_squelch.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS 48000.0

// Fill `pcm` with a sinusoid at f_hz, amplitude `amp`, fs samples/sec.
static void fill_tone(int16_t *pcm, size_t n, double f_hz,
                      double fs, double amp)
{
    const double dphi = 2.0 * M_PI * f_hz / fs;
    double phase = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double v = amp * sin(phase);
        if (v >  32767.0) v =  32767.0;
        if (v < -32768.0) v = -32768.0;
        pcm[i] = (int16_t) lrint(v);
        phase += dphi;
    }
}

static size_t count_nonzero(const int16_t *pcm, size_t n)
{
    size_t c = 0;
    for (size_t i = 0; i < n; ++i) if (pcm[i] != 0) ++c;
    return c;
}

// ------------------------------------------------------------------
// 1. init applies defaults for zero-valued params.
// ------------------------------------------------------------------

static void test_init_defaults(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = { .rate_hz = FS };
    monitor_squelch_init(&s, &p);
    // smooth_alpha = exp(-1 / (tau * fs)) with tau = 0.030 s.
    double expected_alpha = exp(-1.0 / (0.030 * FS));
    tap_okf(fabs(s.smooth_alpha - expected_alpha) < 1e-12,
            "init: default smooth_alpha %.6f matches 30 ms tau (want %.6f)",
            s.smooth_alpha, expected_alpha);
    // hold_duration = 0.5 s * fs = 24000 samples.
    tap_okf(s.hold_duration == (size_t)(0.5 * FS),
            "init: default hold_duration == %zu (got %zu)",
            (size_t)(0.5 * FS), s.hold_duration);
    // boot_target = 1.0 s * fs = 48000 samples.
    tap_okf(s.boot_target == (size_t)(1.0 * FS),
            "init: default boot_target == %zu (got %zu)",
            (size_t)(1.0 * FS), s.boot_target);
    // auto_offset_db defaults to +3.
    tap_okf(fabs(s.auto_offset_db - 3.0) < 1e-12,
            "init: default auto_offset_db == 3.0 (got %.3f)",
            s.auto_offset_db);
    // Default init_mode (uninitialised in the params struct → 0 == MSQ_OFF).
    tap_ok(s.mode == MSQ_OFF, "init: default mode == MSQ_OFF");
}

// ------------------------------------------------------------------
// 2. init with explicit params overrides defaults.
// ------------------------------------------------------------------

static void test_init_explicit_params(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = {
        .rate_hz         = FS,
        .smooth_tau_s    = 0.010,    // 10 ms instead of 30
        .hold_s          = 0.100,    // 100 ms instead of 500
        .boot_window_s   = 0.020,    // 20 ms instead of 1000
        .auto_offset_db  = 6.0,      // 6 dB instead of 3
        .init_mode       = MSQ_FIXED,
        .init_thresh_db  = -4.0,
    };
    monitor_squelch_init(&s, &p);
    double expected_alpha = exp(-1.0 / (0.010 * FS));
    tap_okf(fabs(s.smooth_alpha - expected_alpha) < 1e-12,
            "override: smooth_alpha %.6f matches 10 ms tau",
            s.smooth_alpha);
    tap_okf(s.hold_duration == (size_t)(0.100 * FS),
            "override: hold_duration == %zu", s.hold_duration);
    tap_okf(s.boot_target == (size_t)(0.020 * FS),
            "override: boot_target == %zu", s.boot_target);
    tap_okf(fabs(s.auto_offset_db - 6.0) < 1e-12,
            "override: auto_offset_db == 6.0");
    tap_ok(s.mode == MSQ_FIXED, "override: mode == MSQ_FIXED");
    tap_okf(fabs(s.thresh_db - (-4.0)) < 1e-12,
            "override: thresh_db == -4.0 (got %.3f)", s.thresh_db);
}

// ------------------------------------------------------------------
// 3. set_off / set_fixed_db / set_auto.
// ------------------------------------------------------------------

static void test_set_mode_helpers(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = { .rate_hz = FS };
    monitor_squelch_init(&s, &p);
    s.hold_samples_remaining = 1234;

    monitor_squelch_set_fixed_db(&s, +5.5);
    tap_ok(s.mode == MSQ_FIXED, "set_fixed: mode == MSQ_FIXED");
    tap_okf(fabs(s.thresh_db - 5.5) < 1e-12,
            "set_fixed: thresh_db == %.3f", s.thresh_db);
    tap_ok(s.hold_samples_remaining == 0,
           "set_fixed: clears hold counter");

    s.hold_samples_remaining = 999;
    monitor_squelch_set_auto(&s);
    tap_ok(s.mode == MSQ_AUTO_BOOTSTRAPPING,
           "set_auto: mode == MSQ_AUTO_BOOTSTRAPPING");
    tap_ok(s.boot_count == 0 && s.boot_sum_ratio_db == 0.0,
           "set_auto: bootstrap counters reset");
    tap_ok(s.hold_samples_remaining == 0,
           "set_auto: clears hold counter");

    monitor_squelch_set_off(&s);
    tap_ok(s.mode == MSQ_OFF, "set_off: mode == MSQ_OFF");
}

// ------------------------------------------------------------------
// 4. MSQ_OFF is pass-through.
// ------------------------------------------------------------------

static void test_off_pass_through(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = { .rate_hz = FS, .init_mode = MSQ_OFF };
    monitor_squelch_init(&s, &p);

    int16_t in[256], out[256];
    fill_tone(in, 256, 4800.0, FS, 8000.0);
    monitor_squelch_process(&s, in, out, 256);
    int match = (memcmp(in, out, sizeof in) == 0);
    tap_ok(match, "MSQ_OFF: output bytes match input");
}

// ------------------------------------------------------------------
// 5. MSQ_FIXED with silence stays closed.
// ------------------------------------------------------------------

static void test_fixed_silence_closed(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = {
        .rate_hz = FS, .init_mode = MSQ_FIXED, .init_thresh_db = 3.0,
    };
    monitor_squelch_init(&s, &p);

    int16_t in[8192]  = {0};
    int16_t out[8192];
    monitor_squelch_process(&s, in, out, 8192);
    size_t nz = count_nonzero(out, 8192);
    tap_okf(nz == 0,
            "MSQ_FIXED + silence: output all zeros (nz=%zu)", nz);
}

// ------------------------------------------------------------------
// 6. MSQ_FIXED with an in-band tone opens the gate.
// ------------------------------------------------------------------

static void test_fixed_tone_opens(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = {
        .rate_hz = FS, .init_mode = MSQ_FIXED, .init_thresh_db = 3.0,
    };
    monitor_squelch_init(&s, &p);

    const size_t N = 8192;
    int16_t in[8192], out[8192];
    // 4800 Hz is dead-centre of the default sig band [4500, 5100].
    fill_tone(in, N, 4800.0, FS, 8000.0);
    monitor_squelch_process(&s, in, out, N);
    // After the smoother settles (~3*tau ≈ 4300 samples) the gate
    // must be open. Test by bytewise comparison against input on the
    // tail — a sine input has natural zero-crossings, so counting
    // nonzero outputs is meaningless. With the gate open, output ==
    // input exactly.
    int match = memcmp(out + N/2, in + N/2, (N/2) * sizeof(int16_t)) == 0;
    tap_ok(match,
           "MSQ_FIXED + in-band tone: tail pass-through == input (gate open)");
    tap_okf(s.ratio_db > 3.0,
            "MSQ_FIXED: ratio_db %.2f exceeds thresh +3.0", s.ratio_db);
}

// ------------------------------------------------------------------
// 7. Hold timer holds the gate open across a tone → silence transition.
// ------------------------------------------------------------------

static void test_hold_timer_keeps_gate_open(void)
{
    // smooth_tau is intentionally small here so sig_sq decays in a few
    // hundred samples once the tone stops — otherwise the smoother's
    // memory keeps re-tripping the threshold on every silent input
    // sample, the hold timer never counts down, and the test asserts a
    // race condition. The hold-timer behaviour we care about is the
    // counter contract: positive after tone, drains to 0 given enough
    // silence, re-triggers on a second tone burst.
    monitor_squelch_t s;
    monitor_squelch_params_t p = {
        .rate_hz        = FS, .init_mode    = MSQ_FIXED,
        .init_thresh_db = 3.0,
        .hold_s         = 0.050,    // 50 ms = 2400 samples
        .smooth_tau_s   = 0.001,    // 1 ms — decays in a few hundred samples
    };

    const size_t N_TONE = 4000;
    int16_t tone[4000], silent[8000], out[8000];
    fill_tone(tone, N_TONE, 4800.0, FS, 8000.0);
    memset(silent, 0, sizeof silent);

    // Process the tone alone, then read the counter. On the trigger
    // sample the counter is set to hold_duration then decremented in
    // the same iteration → end-of-tone value is hold_duration - 1.
    monitor_squelch_init(&s, &p);
    monitor_squelch_process(&s, tone, out, N_TONE);
    tap_okf(s.hold_samples_remaining == s.hold_duration - 1,
            "hold: counter == hold_duration-1 after tone "
            "(got %zu, want %zu)",
            s.hold_samples_remaining, s.hold_duration - 1);

    // Feed plenty of silence — well more than hold_duration plus the
    // smoother's settling. Counter must reach 0.
    monitor_squelch_process(&s, silent, out, sizeof silent / 2);
    tap_okf(s.hold_samples_remaining == 0,
            "hold: counter drains to 0 after sustained silence (got %zu)",
            s.hold_samples_remaining);

    // Second tone burst — counter must re-trigger to a positive value.
    monitor_squelch_process(&s, tone, out, N_TONE);
    tap_okf(s.hold_samples_remaining > 0,
            "hold: counter re-triggers on second tone burst (got %zu)",
            s.hold_samples_remaining);
}

// ------------------------------------------------------------------
// 8. Bootstrap transitions to ENGAGED after boot_target samples.
// ------------------------------------------------------------------

static void test_bootstrap_engages(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = {
        .rate_hz        = FS,
        .init_mode      = MSQ_AUTO_BOOTSTRAPPING,
        .boot_window_s  = 0.020,    // 960 samples — fast for test
        .auto_offset_db = 4.0,
    };
    monitor_squelch_init(&s, &p);
    tap_ok(s.mode == MSQ_AUTO_BOOTSTRAPPING,
           "boot: starts in MSQ_AUTO_BOOTSTRAPPING");

    size_t target = s.boot_target;
    // Feed one boot_target's worth of silence + 1 sample.
    int16_t *in  = (int16_t *) calloc(target + 16, sizeof(int16_t));
    int16_t *out = (int16_t *) calloc(target + 16, sizeof(int16_t));
    if (!in || !out) { tap_bail("oom"); free(in); free(out); return; }

    monitor_squelch_process(&s, in, out, target);
    tap_ok(s.mode == MSQ_AUTO_ENGAGED,
           "boot: transitions to MSQ_AUTO_ENGAGED at sample boot_target");
    // Silenced input → ratio ≈ 0 dB (both bands floored). So boot mean
    // ≈ 0 dB → threshold ≈ +4 dB.
    tap_okf(fabs(s.boot_floor_db) < 1.0,
            "boot: floor ≈ 0 dB on silence (got %.3f)", s.boot_floor_db);
    tap_okf(fabs(s.thresh_db - (s.boot_floor_db + 4.0)) < 1e-6,
            "boot: thresh == floor + offset_db (%.3f vs %.3f+4.0)",
            s.thresh_db, s.boot_floor_db);

    size_t nz = count_nonzero(out, target);
    tap_okf(nz == 0,
            "boot: all bootstrap output samples are zero (nz=%zu)", nz);
    free(in); free(out);
}

// ------------------------------------------------------------------
// 9. Status string format.
// ------------------------------------------------------------------

static void test_status_strings(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = { .rate_hz = FS };
    char buf[128];

    monitor_squelch_init(&s, &p);   // mode == MSQ_OFF
    monitor_squelch_status(&s, buf, sizeof buf);
    tap_okf(strcmp(buf, "off") == 0,
            "status MSQ_OFF: \"%s\" == \"off\"", buf);

    monitor_squelch_set_auto(&s);
    monitor_squelch_status(&s, buf, sizeof buf);
    tap_okf(strstr(buf, "auto(boot ") != NULL,
            "status MSQ_AUTO_BOOTSTRAPPING: \"%s\" contains \"auto(boot \"",
            buf);

    monitor_squelch_set_fixed_db(&s, -2.5);
    monitor_squelch_status(&s, buf, sizeof buf);
    tap_okf(strstr(buf, "thr=-2.5dB") != NULL,
            "status MSQ_FIXED: \"%s\" contains \"thr=-2.5dB\"", buf);

    // Manually push to ENGAGED to test that branch.
    s.mode = MSQ_AUTO_ENGAGED;
    s.thresh_db    = +5.0;
    s.boot_floor_db = +2.0;
    s.ratio_db     = +3.5;
    monitor_squelch_status(&s, buf, sizeof buf);
    tap_okf(strstr(buf, "auto(thr=+5.0dB") != NULL,
            "status MSQ_AUTO_ENGAGED: \"%s\" contains \"auto(thr=+5.0dB\"",
            buf);
}

// ------------------------------------------------------------------
// 10. pcm_in and pcm_out aliasing safe.
// ------------------------------------------------------------------

static void test_alias_safe(void)
{
    monitor_squelch_t s;
    monitor_squelch_params_t p = { .rate_hz = FS, .init_mode = MSQ_OFF };
    monitor_squelch_init(&s, &p);

    int16_t buf[256];
    fill_tone(buf, 256, 4800.0, FS, 6000.0);
    int16_t ref[256];
    memcpy(ref, buf, sizeof buf);
    // Same buffer for in and out.
    monitor_squelch_process(&s, buf, buf, 256);
    tap_ok(memcmp(buf, ref, sizeof buf) == 0,
           "alias-safe: MSQ_OFF passthrough still matches input");
}

int main(void)
{
    test_init_defaults();
    test_init_explicit_params();
    test_set_mode_helpers();
    test_off_pass_through();
    test_fixed_silence_closed();
    test_fixed_tone_opens();
    test_hold_timer_keeps_gate_open();
    test_bootstrap_engages();
    test_status_strings();
    test_alias_safe();
    return tap_done();
}
