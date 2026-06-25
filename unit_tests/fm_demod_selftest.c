/*

   Simple Satellite Operations  unit_tests/fm_demod_selftest.c

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Coverage for the shared FM-discriminator kernel src/dsp/fm_demod.h, which
// the live RX/TX core and the capture tool's two demod loops now call. The
// oracle is analytic, not a round-trip against the kernel itself: a complex
// tone of frequency f advances phase by exactly 2*pi*f/fs each sample, so
// the discriminator must report dphi = 2*pi*f/fs, which the calibrated
// k_scale maps to PCM = f/fullscale * 32767. That closed form would go red
// if the atan2 arguments were transposed/negated or the k_scale formula
// dropped a term, so the asserts bite on a real regression.

#include "fm_demod.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>

// One ideal IQ sample of unit amplitude at phase phi.
static void iq_at(double phi, double *I, double *Q)
{
    *I = cos(phi);
    *Q = sin(phi);
}

int main(void)
{
    const double fs        = 96000.0;
    const double fullscale = 25000.0;
    const double k_scale   = fm_demod_k_scale(fs, fullscale);

    // Calibration: a deviation of exactly fullscale_hz must map to full
    // scale. The phase step for that deviation is 2*pi*fullscale/fs, and
    // dphi * k_scale should land on 32767.
    double dphi_full = 2.0 * M_PI * fullscale / fs;
    tap_okf(fabs(dphi_full * k_scale - 32767.0) < 1e-6,
            "k_scale maps a full-scale deviation to 32767 (got %.3f)",
            dphi_full * k_scale);

    // A tone at a known fraction of full scale yields a constant PCM equal
    // to that fraction of 32767, for every sample of the run. Run several
    // samples to confirm it is steady (not just a lucky first sample).
    const double f_tone   = 0.4 * fullscale;          // 10 kHz
    const double dphi      = 2.0 * M_PI * f_tone / fs;
    const int16_t expected = (int16_t) lround(0.4 * 32767.0);  // 13107
    int steady_ok = 1;
    double pI, pQ;
    iq_at(0.0, &pI, &pQ);
    for (int k = 1; k <= 200; k++) {
        double I, Q;
        iq_at(k * dphi, &I, &Q);
        int16_t pcm = fm_demod_pcm(pI, pQ, I, Q, k_scale, NULL);
        if (pcm != expected) { steady_ok = 0; }
        pI = I; pQ = Q;
    }
    tap_okf(steady_ok,
            "constant tone at 0.4*fullscale yields steady PCM %d", expected);

    // Sign convention: an advancing (positive-frequency) tone gives positive
    // PCM; a retreating tone gives negative PCM. A transposed atan2 flips this.
    {
        double I0, Q0, I1, Q1;
        iq_at(0.0, &I0, &Q0);
        iq_at(dphi, &I1, &Q1);
        tap_okf(fm_demod_pcm(I0, Q0, I1, Q1, k_scale, NULL) > 0,
                "positive frequency step gives positive PCM");
        iq_at(-dphi, &I1, &Q1);
        tap_okf(fm_demod_pcm(I0, Q0, I1, Q1, k_scale, NULL) < 0,
                "negative frequency step gives negative PCM");
    }

    // Clipping + the optional clip counter. A near-pi phase step is far past
    // full scale, so it must saturate and bump the counter exactly once.
    {
        int clipped = 0;
        double I0, Q0, I1, Q1;
        iq_at(0.0, &I0, &Q0);
        iq_at(3.0, &I1, &Q1);   // +3.0 rad/sample, way over full scale
        int16_t hi = fm_demod_pcm(I0, Q0, I1, Q1, k_scale, &clipped);
        tap_okf(hi == 32767 && clipped == 1,
                "over-range positive step clips to 32767 and counts (pcm=%d, n=%d)",
                hi, clipped);
        iq_at(-3.0, &I1, &Q1);
        int16_t lo = fm_demod_pcm(I0, Q0, I1, Q1, k_scale, &clipped);
        tap_okf(lo == -32768 && clipped == 2,
                "over-range negative step clips to -32768 and counts (pcm=%d, n=%d)",
                lo, clipped);
    }

    // A NULL clip counter must be accepted (the core and live-monitor paths
    // pass NULL) and still clip the value.
    {
        double I0, Q0, I1, Q1;
        iq_at(0.0, &I0, &Q0);
        iq_at(3.0, &I1, &Q1);
        tap_okf(fm_demod_pcm(I0, Q0, I1, Q1, k_scale, NULL) == 32767,
                "NULL clip counter is accepted and value still clips");
    }

    // No phase change between samples reads as zero frequency -> zero PCM.
    {
        tap_okf(fm_demod_pcm(1.0, 0.0, 1.0, 0.0, k_scale, NULL) == 0,
                "stationary phase reads as 0 PCM");
    }

    // The squelch magnitude helper: a 3-4-5 triangle squares to 25, origin to 0.
    tap_okf(fm_iq_mag_sq(3.0, 4.0) == 25.0, "fm_iq_mag_sq(3,4) == 25");
    tap_okf(fm_iq_mag_sq(0.0, 0.0) == 0.0,  "fm_iq_mag_sq(0,0) == 0");

    return tap_done();
}
