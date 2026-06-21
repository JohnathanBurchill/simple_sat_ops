/*

    Simple Satellite Operations  src/dsp/fm_mod.c

    See fm_mod.h. The modulator and ramp were previously file-local
    statics in src/pipeline/tx_burst.c; the only behavioural change on
    extraction is that the phase accumulator is now caller-owned (was a
    function-local static that leaked phase between unrelated bursts).

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

#include "fm_mod.h"

#include <math.h>

void fm_mod_init(fm_mod_t *m)
{
    if (m == NULL) return;
    m->phi = 0.0;
}

void fm_mod_block(fm_mod_t *m, const int16_t *pcm, size_t n_pcm,
                  double dev_hz, double fs, int16_t *iq_out)
{
    if (m == NULL || pcm == NULL || iq_out == NULL || fs <= 0.0) return;
    const double k = 2.0 * M_PI * dev_hz / fs;
    const double inv = 1.0 / 32767.0;
    double phi = m->phi;
    for (size_t i = 0; i < n_pcm; i++) {
        double x = (double) pcm[i] * inv;
        phi += k * x;
        if (phi >  M_PI) phi -= 2.0 * M_PI;
        if (phi < -M_PI) phi += 2.0 * M_PI;
        iq_out[2 * i + 0] = (int16_t) lround(cos(phi) * 22937.0);
        iq_out[2 * i + 1] = (int16_t) lround(sin(phi) * 22937.0);
    }
    m->phi = phi;
}

void fm_apply_ramp(int16_t *iq, size_t n_samps, size_t ramp_n)
{
    if (iq == NULL || ramp_n == 0) return;
    if (ramp_n > n_samps / 2) ramp_n = n_samps / 2;
    for (size_t i = 0; i < ramp_n; i++) {
        double env_in  = 0.5 * (1.0 - cos(M_PI * (double) i / (double) ramp_n));
        double env_out = 0.5 * (1.0 + cos(M_PI * (double) i / (double) ramp_n));
        iq[2 * i + 0] = (int16_t) lround((double) iq[2 * i + 0] * env_in);
        iq[2 * i + 1] = (int16_t) lround((double) iq[2 * i + 1] * env_in);
        size_t k = n_samps - ramp_n + i;
        iq[2 * k + 0] = (int16_t) lround((double) iq[2 * k + 0] * env_out);
        iq[2 * k + 1] = (int16_t) lround((double) iq[2 * k + 1] * env_out);
    }
}
