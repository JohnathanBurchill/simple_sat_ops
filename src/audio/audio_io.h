/*

    Simple Satellite Operations  src/audio/audio_io.h

    Thin live-audio wrapper over the vendored miniaudio library, for the
    amateur-band voice tools (ham_listen plays the demodulated downlink
    to the speakers; ham_speak records the microphone before
    transmitting). Mono 16-bit PCM only, system default device, decoupled
    from the caller's thread by an internal PCM ring buffer.

    This header is deliberately miniaudio-free (opaque handles) so the
    tools don't need miniaudio on their include path — only audio_io.c
    and miniaudio_impl.c do.

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

#ifndef SSO_AUDIO_IO_H
#define SSO_AUDIO_IO_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

typedef struct audio_play    audio_play_t;
typedef struct audio_capture audio_capture_t;

// Open the system default output device for `channels`-channel 16-bit
// PCM at `rate` Hz and start it. Returns NULL on failure.
audio_play_t *audio_play_open(int rate, int channels);

// Queue `frames` PCM frames for playback. Blocks (briefly sleeping) when
// the internal ring buffer is full so the producer can't outrun the
// soundcard; gives up after a few seconds of stall. Returns the number
// of frames queued, or -1 on error.
int audio_play_write(audio_play_t *p, const int16_t *pcm, size_t frames);

// Let any queued audio drain (bounded wait), stop the device, and free.
void audio_play_close(audio_play_t *p);

// Open the system default input device for `channels`-channel 16-bit PCM
// at `rate` Hz and start capturing. Returns NULL on failure.
audio_capture_t *audio_capture_open(int rate, int channels);

// Copy up to `max_frames` already-captured PCM frames into `pcm`.
// Non-blocking: returns the number of frames available now (may be 0),
// or -1 on error.
ssize_t audio_capture_read(audio_capture_t *c, int16_t *pcm, size_t max_frames);

// Stop the device and free.
void audio_capture_close(audio_capture_t *c);

#endif // SSO_AUDIO_IO_H
