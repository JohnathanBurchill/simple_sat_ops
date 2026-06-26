/*

    Simple Satellite Operations  unit_tests/frame_rssi_selftest.c

    Coverage for src/dsp/frame_rssi.c — the RMS-over-a-sample-span -> dBFS
    used as the rx_replay --forensics-report "rssi" field.

    External oracle: the expected dBFS are textbook decibel relationships,
    NOT numbers produced by this code, so a bug in the formula can't hide
    behind a self-consistent round trip:
      - half amplitude (16384/32768 = 0.5)        -> -6.0206 dB
      - quarter amplitude (8192/32768 = 0.25)     -> -12.0412 dB
      - 1/sqrt(2) (I==Q magnitude vs full scale)  -> -3.0103 dB
      - 0.5/sqrt(2) (half the window at half amp) -> -9.0309 dB
    Each value comes from 20*log10(ratio) worked out by hand below.

    The constant-amplitude buffers make the RMS exact, so the only error
    is double-precision log10/sqrt (~1e-12) and the chosen integer
    amplitudes (powers of two -> exact). EPS = 1e-3 is far above that and
    far below the gaps between the cases, so each assertion still goes red
    under a real mutation (RMS->peak, dropped sqrt, wrong reference, an
    ignored start/span, a broken end-clip). frame_rssi_dbfs returns the
    full-precision double; the one-decimal rounding lives in rx_replay's
    printf, not here.

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

#include "frame_rssi.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>

#define EPS 1e-3

static int deq(double a, double b) { return fabs(a - b) <= EPS; }

int main(void)
{
    // ---- PCM: constant amplitude -> known dBFS -------------------------
    {
        int16_t buf[64];
        for (int i = 0; i < 64; ++i) buf[i] = 16384;   // half scale
        double v = frame_rssi_dbfs(buf, 64, 0, 0, 64);
        // 20*log10(16384/32768) = 20*log10(0.5) = -6.0205999...
        tap_okf(deq(v, -6.0205999), "PCM half scale -> -6.02 dBFS (got %.6f)", v);

        for (int i = 0; i < 64; ++i) buf[i] = 8192;     // quarter scale
        v = frame_rssi_dbfs(buf, 64, 0, 0, 64);
        // 20*log10(0.25) = -12.0411998...
        tap_okf(deq(v, -12.0411998), "PCM quarter scale -> -12.04 dBFS (got %.6f)", v);
    }

    // ---- RMS, not peak: a lone full-scale spike reads low --------------
    {
        int16_t buf[100] = {0};
        buf[50] = 32767;                                // one spike in 100
        double v = frame_rssi_dbfs(buf, 100, 0, 0, 100);
        // rms = 32767/sqrt(100) = 3276.7 -> ratio ~0.1 -> ~ -20 dBFS.
        // A peak meter would report ~0 dBFS, so this pins RMS semantics.
        tap_okf(deq(v, -20.0003167), "lone spike averages to ~ -20 dBFS, not 0 (got %.6f)", v);
    }

    // ---- IQ: magnitude sqrt(I^2+Q^2) -----------------------------------
    {
        int16_t iq[2 * 32];
        for (int k = 0; k < 32; ++k) { iq[2*k] = 16384; iq[2*k+1] = 16384; }
        double v = frame_rssi_dbfs(iq, 32, 1, 0, 32);
        // |z| = 16384*sqrt(2); ratio = sqrt(2)/2 -> 20*log10(0.70710678) = -3.0103
        tap_okf(deq(v, -3.0102999), "IQ I==Q -> magnitude -3.01 dBFS (got %.6f)", v);

        for (int k = 0; k < 32; ++k) { iq[2*k] = 16384; iq[2*k+1] = 0; }
        v = frame_rssi_dbfs(iq, 32, 1, 0, 32);
        // |z| = 16384 (Q=0) -> same as PCM half scale.
        tap_okf(deq(v, -6.0205999), "IQ Q=0 magnitude == PCM half scale (got %.6f)", v);
    }

    // ---- start / span actually select the window ----------------------
    {
        int16_t buf[20];
        for (int i = 0; i < 10; ++i) buf[i] = 32767;    // loud first half
        for (int i = 10; i < 20; ++i) buf[i] = 8192;    // quiet second half
        // Window the quiet half only: a bug that ignores `start` would
        // pull in the loud region and read far higher than -12 dBFS.
        double v = frame_rssi_dbfs(buf, 20, 0, 10, 10);
        tap_okf(deq(v, -12.0411998), "start offset selects the quiet window (got %.6f)", v);

        // Half the window at half amplitude, half silent:
        // mean_sq = (10*16384^2)/20 -> ratio = 0.5/sqrt(2) -> -9.0309 dBFS.
        for (int i = 0; i < 10; ++i) buf[i] = 16384;
        for (int i = 10; i < 20; ++i) buf[i] = 0;
        v = frame_rssi_dbfs(buf, 20, 0, 0, 20);
        tap_okf(deq(v, -9.0308998), "half-amplitude half-silent window -> -9.03 dBFS (got %.6f)", v);
    }

    // ---- span clipped to the end of the buffer -------------------------
    {
        int16_t buf[10];
        for (int i = 0; i < 10; ++i) buf[i] = 16384;
        // Ask for far more than exists: must clip to the 10 real frames
        // and read the same -6.02, not run off the end.
        double clipped = frame_rssi_dbfs(buf, 10, 0, 0, 1000);
        double exact   = frame_rssi_dbfs(buf, 10, 0, 0, 10);
        tap_okf(deq(clipped, -6.0205999), "over-long span clips to buffer end (got %.6f)", clipped);
        tap_okf(deq(clipped, exact), "clipped span == exact span");
    }
    {
        int16_t iq[2 * 8];
        for (int k = 0; k < 8; ++k) { iq[2*k] = 16384; iq[2*k+1] = 0; }
        // IQ over-long span: clip at pair count, no read past the buffer.
        double v = frame_rssi_dbfs(iq, 8, 1, 0, 500);
        tap_okf(deq(v, -6.0205999), "IQ over-long span clips at pair count (got %.6f)", v);
    }

    // ---- floor: silence and sub-LSB RMS --------------------------------
    {
        int16_t buf[32] = {0};
        tap_ok(frame_rssi_dbfs(buf, 32, 0, 0, 32) == -90.0, "all-zero PCM -> -90 floor");

        int16_t spike[100] = {0};
        spike[0] = 1;   // rms = 1/sqrt(100) = 0.1 < 1 LSB -> floor
        tap_ok(frame_rssi_dbfs(spike, 100, 0, 0, 100) == -90.0,
               "sub-LSB RMS -> -90 floor (not -inf)");
    }

    // ---- guard / failure paths -----------------------------------------
    {
        int16_t buf[8] = { 16384,16384,16384,16384,16384,16384,16384,16384 };
        tap_ok(frame_rssi_dbfs(NULL, 8, 0, 0, 8)  == -90.0, "NULL buffer -> -90");
        tap_ok(frame_rssi_dbfs(buf, 8, 0, 0, 0)   == -90.0, "span 0 -> -90");
        tap_ok(frame_rssi_dbfs(buf, 8, 0, 8, 4)   == -90.0, "start == n_frames -> -90");
        tap_ok(frame_rssi_dbfs(buf, 8, 0, 99, 4)  == -90.0, "start past end -> -90");
    }

    return tap_done();
}
