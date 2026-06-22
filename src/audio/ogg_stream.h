/*

    Simple Satellite Operations  audio/ogg_stream.h

    Shared Ogg/Vorbis encoder. Pulled out of utils/ham_listen.c so the
    same streaming encoder serves both ham_listen (--ogg-stdout) and the
    simple_sat_ops operator's live-audio relay to viewers.

    The encoder hands finished Ogg bytes to a caller-supplied sink
    callback, so the same code drives a pipe (ham_listen writes them to
    stdout), a socket, or a base64 framer (the operator wraps them in an
    SSO_EVT_AUDIO and sends them to a subscribed viewer). Ogg is a
    streaming container, so the byte stream survives a non-seekable pipe.

    The whole module is gated on libsndfile (HAVE_SNDFILE). Built without
    it, ogg_stream_open returns NULL and the callers report that audio is
    unavailable in this build.

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

#ifndef AUDIO_OGG_STREAM_H
#define AUDIO_OGG_STREAM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Sink for encoded Ogg bytes. Mirrors write(2): return the number of
// bytes consumed (normally n) or a negative value on error. An error
// latches the stream into a failed state that ogg_stream_write reports.
typedef long (*ogg_stream_sink_fn)(const uint8_t *bytes, size_t n, void *user);

typedef struct ogg_stream ogg_stream_t;

// Open a mono/stereo Ogg/Vorbis encoder at sample_rate, writing encoded
// bytes to `sink`. vbr_quality is the Vorbis VBR target in [0.0, 1.0]
// (clamped). Returns NULL on error or when built without libsndfile.
// The encoder's Ogg headers are produced into the sink on the first
// ogg_stream_write, so the sink sees a complete, self-contained stream.
ogg_stream_t *ogg_stream_open(int sample_rate, int channels,
                              double vbr_quality,
                              ogg_stream_sink_fn sink, void *user);

// Encode `frames` mono/interleaved int16 PCM samples. Returns 0 on
// success, -1 if the encoder is NULL or the sink has reported an error
// (the caller should stop and close).
int ogg_stream_write(ogg_stream_t *s, const int16_t *pcm, size_t frames);

// Total encoded bytes the sink has accepted so far (for rate reporting).
unsigned long long ogg_stream_bytes(const ogg_stream_t *s);

// Flush and close. Safe with NULL.
void ogg_stream_close(ogg_stream_t *s);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_OGG_STREAM_H
