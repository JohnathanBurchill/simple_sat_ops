/*

   Simple Satellite Operations  adcs_sdlog.c

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

#include "adcs_sdlog.h"

#include <math.h>
#include <string.h>

// The first record's time has to land in these years (2026-01-01 to
// 2036-01-01) for the start of a file to read as a log. The ADCS clock was
// a year behind until it was first set on 2026-06-30, and no log from before
// then carries an attitude.
#define ADCS_SDLOG_T_MIN  1767225600.0
#define ADCS_SDLOG_T_MAX  2082758400.0

// The longest logging period the ADCS takes, its period field being a
// uint16 of seconds.
#define ADCS_SDLOG_PERIOD_MAX  65535L

static int frame_selected(const uint8_t *mask, int frame)
{
    return (mask[frame / 8] >> (frame % 8)) & 1;
}

int adcs_sdlog_parse_header(const uint8_t *d, size_t n, adcs_sdlog_t *log)
{
    adcs_sdlog_t none = {0};
    if (log != NULL) *log = none;
    if (d == NULL || log == NULL || n < ADCS_SDLOG_MASK_LEN + ADCS_SDLOG_STAMP_LEN)
        return 0;

    int nframes = 0;
    for (int f = 0; f < ADCS_SDLOG_MASK_LEN * 8; f++)
        nframes += frame_selected(d, f);
    if (nframes == 0) return 0;

    double t0 = adcs_sdlog_stamp(d + ADCS_SDLOG_MASK_LEN);
    if (t0 < ADCS_SDLOG_T_MIN || t0 >= ADCS_SDLOG_T_MAX) return 0;

    memcpy(log->mask, d, ADCS_SDLOG_MASK_LEN);
    log->nframes = nframes;
    log->rec_len = ADCS_SDLOG_STAMP_LEN + (long) ADCS_SDLOG_FRAME_LEN * nframes;
    log->t0 = t0;
    return 1;
}

long adcs_sdlog_record_offset(const adcs_sdlog_t *log, long i)
{
    return ADCS_SDLOG_MASK_LEN + i * log->rec_len;
}

int adcs_sdlog_frame_offset(const adcs_sdlog_t *log, int frame)
{
    if (frame < 0 || frame >= ADCS_SDLOG_MASK_LEN * 8) return -1;
    if (!frame_selected(log->mask, frame)) return -1;
    int before = 0;
    for (int f = 0; f < frame; f++) before += frame_selected(log->mask, f);
    return ADCS_SDLOG_STAMP_LEN + ADCS_SDLOG_FRAME_LEN * before;
}

double adcs_sdlog_stamp(const uint8_t *p)
{
    uint32_t s = (uint32_t) p[0] | ((uint32_t) p[1] << 8)
               | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
    unsigned ms = (unsigned) p[4] | ((unsigned) p[5] << 8);
    if (ms >= 1000) return -1.0;
    return (double) s + ms / 1000.0;
}

long adcs_sdlog_stamps_in(const adcs_sdlog_t *log, long off, size_t n,
                          long *first, long *last)
{
    long R = log->rec_len;
    long end = off + (long) n;
    // The first stamp starting at or after off, and the last one ending at
    // or before end.
    long i0 = off <= ADCS_SDLOG_MASK_LEN ? 0 : (off - ADCS_SDLOG_MASK_LEN + R - 1) / R;
    long room = end - ADCS_SDLOG_STAMP_LEN - ADCS_SDLOG_MASK_LEN;
    if (room < 0) return 0;
    long i1 = room / R;
    if (i1 < i0) return 0;
    if (first != NULL) *first = i0;
    if (last != NULL) *last = i1;
    return i1 - i0 + 1;
}

long adcs_sdlog_period_at(const adcs_sdlog_t *log, long i, double t)
{
    if (i < 1 || t <= log->t0) return 0;
    long p = lround((t - log->t0) / (double) i);
    if (p < 1 || p > ADCS_SDLOG_PERIOD_MAX) return 0;
    if (fabs(t - (log->t0 + (double) i * (double) p)) > ADCS_SDLOG_SCHEDULE_TOL_S)
        return 0;
    return p;
}

long adcs_sdlog_chunk_fits(const adcs_sdlog_t *log, long period, long off,
                           const uint8_t *d, size_t n)
{
    long i0 = 0, i1 = -1;
    long count = adcs_sdlog_stamps_in(log, off, n, &i0, &i1);
    for (long i = i0; i <= i1; i++) {
        double t = adcs_sdlog_stamp(d + (adcs_sdlog_record_offset(log, i) - off));
        if (t < 0.0) return -1;
        if (fabs(t - (log->t0 + (double) i * (double) period)) > ADCS_SDLOG_SCHEDULE_TOL_S)
            return -1;
    }
    return count;
}
