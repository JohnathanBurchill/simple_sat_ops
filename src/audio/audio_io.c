/*

    Simple Satellite Operations  src/audio/audio_io.c

    See audio_io.h. miniaudio runs the device on its own thread and calls
    our data callback; we bridge to the caller's thread through a
    single-producer/single-consumer PCM ring buffer (ma_pcm_rb):

      * playback: the caller writes (producer), the callback reads.
      * capture:  the callback writes, the caller reads (consumer).

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

#include "audio_io.h"
#include "miniaudio.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

// Ring-buffer depth, in seconds of audio. One second comfortably
// absorbs scheduling jitter on either side without adding noticeable
// latency to a voice channel.
#define RB_SECONDS 1

struct audio_play {
    ma_device device;
    ma_pcm_rb rb;
    ma_uint32 channels;
    int       device_ok;
    int       rb_ok;
};

struct audio_capture {
    ma_device device;
    ma_pcm_rb rb;
    ma_uint32 channels;
    int       device_ok;
    int       rb_ok;
};

static void sleep_ms(int ms)
{
    struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

// Playback callback (miniaudio thread): pull as many frames as the ring
// buffer holds into pOutput, zero-fill the remainder on underrun.
static void play_cb(ma_device *dev, void *out, const void *in, ma_uint32 frames)
{
    (void) in;
    audio_play_t *p = (audio_play_t *) dev->pUserData;
    int16_t *dst = (int16_t *) out;
    ma_uint32 left = frames;
    while (left > 0) {
        ma_uint32 n = left;
        void *buf = NULL;
        if (ma_pcm_rb_acquire_read(&p->rb, &n, &buf) != MA_SUCCESS || n == 0) break;
        memcpy(dst, buf, (size_t) n * p->channels * sizeof(int16_t));
        ma_pcm_rb_commit_read(&p->rb, n);
        dst  += (size_t) n * p->channels;
        left -= n;
    }
    if (left > 0) {
        memset(dst, 0, (size_t) left * p->channels * sizeof(int16_t));
    }
}

// Capture callback (miniaudio thread): push captured frames into the
// ring buffer; if the consumer has fallen behind and the buffer is
// full, drop the overflow rather than block the audio thread.
static void capture_cb(ma_device *dev, void *out, const void *in, ma_uint32 frames)
{
    (void) out;
    audio_capture_t *c = (audio_capture_t *) dev->pUserData;
    const int16_t *src = (const int16_t *) in;
    ma_uint32 left = frames;
    while (left > 0) {
        ma_uint32 n = left;
        void *buf = NULL;
        if (ma_pcm_rb_acquire_write(&c->rb, &n, &buf) != MA_SUCCESS || n == 0) break;
        memcpy(buf, src, (size_t) n * c->channels * sizeof(int16_t));
        ma_pcm_rb_commit_write(&c->rb, n);
        src  += (size_t) n * c->channels;
        left -= n;
    }
}

audio_play_t *audio_play_open(int rate, int channels)
{
    if (rate <= 0 || channels <= 0) return NULL;
    audio_play_t *p = calloc(1, sizeof *p);
    if (p == NULL) return NULL;
    p->channels = (ma_uint32) channels;

    if (ma_pcm_rb_init(ma_format_s16, p->channels,
                       (ma_uint32)(rate * RB_SECONDS),
                       NULL, NULL, &p->rb) != MA_SUCCESS) {
        free(p);
        return NULL;
    }
    p->rb_ok = 1;

    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format   = ma_format_s16;
    cfg.playback.channels = p->channels;
    cfg.sampleRate        = (ma_uint32) rate;
    cfg.dataCallback      = play_cb;
    cfg.pUserData         = p;

    if (ma_device_init(NULL, &cfg, &p->device) != MA_SUCCESS) {
        audio_play_close(p);
        return NULL;
    }
    p->device_ok = 1;

    if (ma_device_start(&p->device) != MA_SUCCESS) {
        audio_play_close(p);
        return NULL;
    }
    return p;
}

int audio_play_write(audio_play_t *p, const int16_t *pcm, size_t frames)
{
    if (p == NULL || pcm == NULL) return -1;
    size_t done  = 0;
    int    spins = 0;
    while (done < frames) {
        ma_uint32 want = (ma_uint32)(frames - done);
        void *buf = NULL;
        if (ma_pcm_rb_acquire_write(&p->rb, &want, &buf) != MA_SUCCESS) return -1;
        if (want == 0) {
            // Buffer full — let the callback drain it. Give up after a
            // long stall (~10 s) so a dead device can't wedge the caller.
            if (++spins > 2000) return (int) done;
            sleep_ms(5);
            continue;
        }
        spins = 0;
        memcpy(buf, pcm + done * p->channels,
               (size_t) want * p->channels * sizeof(int16_t));
        ma_pcm_rb_commit_write(&p->rb, want);
        done += want;
    }
    return (int) done;
}

void audio_play_close(audio_play_t *p)
{
    if (p == NULL) return;
    if (p->device_ok) {
        // Let queued audio finish playing (bounded wait, ~2 s).
        for (int i = 0; i < 400 && ma_pcm_rb_available_read(&p->rb) > 0; i++) {
            sleep_ms(5);
        }
        ma_device_uninit(&p->device);
    }
    if (p->rb_ok) ma_pcm_rb_uninit(&p->rb);
    free(p);
}

audio_capture_t *audio_capture_open(int rate, int channels)
{
    if (rate <= 0 || channels <= 0) return NULL;
    audio_capture_t *c = calloc(1, sizeof *c);
    if (c == NULL) return NULL;
    c->channels = (ma_uint32) channels;

    if (ma_pcm_rb_init(ma_format_s16, c->channels,
                       (ma_uint32)(rate * RB_SECONDS),
                       NULL, NULL, &c->rb) != MA_SUCCESS) {
        free(c);
        return NULL;
    }
    c->rb_ok = 1;

    ma_device_config cfg = ma_device_config_init(ma_device_type_capture);
    cfg.capture.format   = ma_format_s16;
    cfg.capture.channels = c->channels;
    cfg.sampleRate       = (ma_uint32) rate;
    cfg.dataCallback     = capture_cb;
    cfg.pUserData        = c;

    if (ma_device_init(NULL, &cfg, &c->device) != MA_SUCCESS) {
        audio_capture_close(c);
        return NULL;
    }
    c->device_ok = 1;

    if (ma_device_start(&c->device) != MA_SUCCESS) {
        audio_capture_close(c);
        return NULL;
    }
    return c;
}

ssize_t audio_capture_read(audio_capture_t *c, int16_t *pcm, size_t max_frames)
{
    if (c == NULL || pcm == NULL) return -1;
    size_t done = 0;
    while (done < max_frames) {
        ma_uint32 want = (ma_uint32)(max_frames - done);
        void *buf = NULL;
        if (ma_pcm_rb_acquire_read(&c->rb, &want, &buf) != MA_SUCCESS) return -1;
        if (want == 0) break;  // nothing more buffered right now
        memcpy(pcm + done * c->channels, buf,
               (size_t) want * c->channels * sizeof(int16_t));
        ma_pcm_rb_commit_read(&c->rb, want);
        done += want;
    }
    return (ssize_t) done;
}

void audio_capture_close(audio_capture_t *c)
{
    if (c == NULL) return;
    if (c->device_ok) ma_device_uninit(&c->device);
    if (c->rb_ok)     ma_pcm_rb_uninit(&c->rb);
    free(c);
}
