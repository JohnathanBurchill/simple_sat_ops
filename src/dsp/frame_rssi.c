/*

    Simple Satellite Operations  src/dsp/frame_rssi.c

    See frame_rssi.h. RMS-over-a-sample-span -> dBFS, the rx_replay
    --forensics-report "rssi" field. Mirrors rx_session's level meter.

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

#include <math.h>

double frame_rssi_dbfs(const int16_t *s, size_t n_frames,
                       int iq_mode, uint64_t start, uint64_t span)
{
    if (s == NULL || span == 0 || start >= n_frames) return -90.0;
    uint64_t end = start + span;
    if (end > n_frames) end = n_frames;
    double sum_sq = 0.0;
    uint64_t count = 0;
    for (uint64_t k = start; k < end; ++k) {
        if (iq_mode) {
            double i = (double)s[2 * k];
            double q = (double)s[2 * k + 1];
            sum_sq += i * i + q * q;
        } else {
            double v = (double)s[k];
            sum_sq += v * v;
        }
        ++count;
    }
    if (count == 0) return -90.0;
    double rms = sqrt(sum_sq / (double)count);
    if (rms < 1.0) return -90.0;
    return 20.0 * log10(rms / 32768.0);
}
