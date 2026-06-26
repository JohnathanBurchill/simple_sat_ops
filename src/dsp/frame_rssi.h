/*

    Simple Satellite Operations  src/dsp/frame_rssi.h

    Per-frame signal-level estimate for the rx_replay --forensics-report
    "rssi" field: the RMS amplitude of a span of recorded samples, in
    dBFS relative to int16 full scale (32768). It is the same formula and
    scale as the live RX panel's RMS level meter (rms/32768, -90 dBFS
    floor) so a forensics number is comparable to what the operator sees
    live — NOT a calibrated radio RSSI in dBm. IQ files use the magnitude
    sqrt(I^2+Q^2); mono PCM / FM-audio uses the sample value (for a
    demodulated .wav/.ogg this is the discriminated audio level, not RF
    power).

    Extracted from utils/rx_replay.c so it can be unit-tested without the
    rest of the decoder (the function is pure: samples in, dBFS out).

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

#ifndef SSO_FRAME_RSSI_H
#define SSO_FRAME_RSSI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// RMS level of `span` sample-frames starting at `start`, in dBFS
// relative to int16 full scale (20*log10(rms/32768)).
//
//   s        - sample buffer: interleaved I,Q int16 pairs when iq_mode,
//              else mono int16 PCM.
//   n_frames - number of sample-frames in s (IQ pairs, or PCM samples).
//   iq_mode  - 1: each frame is one I,Q pair, level uses the magnitude
//              sqrt(I^2+Q^2); 0: each frame is one PCM sample.
//   start    - index of the first frame to include.
//   span     - number of frames to average over; clipped to the end of
//              the buffer.
//
// Returns the floor -90.0 when s is NULL, span is 0, start is past the
// end, or the RMS is below 1 LSB (so a silent or out-of-range span reads
// as the floor rather than -inf).
double frame_rssi_dbfs(const int16_t *s, size_t n_frames,
                       int iq_mode, uint64_t start, uint64_t span);

#ifdef __cplusplus
}
#endif

#endif // SSO_FRAME_RSSI_H
