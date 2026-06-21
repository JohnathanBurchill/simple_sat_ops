/*

    Simple Satellite Operations  src/dsp/resample.c

    See resample.h.

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

#include "resample.h"

#include <math.h>

size_t resample_up_linear(const int16_t *in, size_t n, int L,
                          int16_t *out, size_t out_cap)
{
    if (in == NULL || out == NULL || L < 1 || n == 0) return 0;

    size_t w = 0;
    for (size_t i = 0; i < n && w < out_cap; i++) {
        double cur  = (double) in[i];
        // Interpolate toward the next sample; the final sample has no
        // successor, so hold it flat for its run.
        double next = (i + 1 < n) ? (double) in[i + 1] : cur;
        for (int j = 0; j < L && w < out_cap; j++) {
            double frac = (double) j / (double) L;
            double v = cur + (next - cur) * frac;
            out[w++] = (int16_t) lround(v);
        }
    }
    return w;
}
