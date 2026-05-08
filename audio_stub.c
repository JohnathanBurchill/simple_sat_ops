/*

    Simple Satellite Operations  audio_stub.c

    No-op implementations of the audio.h API for hosts that don't have
    ALSA (macOS dev box). Lets simple_sat_ops link without libasound so
    the operator can drive the rotator + radio from a laptop, with
    --without-audio (or the auto-disabled audio_record default below)
    keeping the ALSA-using code paths inert at runtime.

    Every function returns an obvious failure (or a no-op for
    cleanup-style entry points) so any accidental call is loud rather
    than silent.

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

#include "audio.h"

#include <stdio.h>

static int complain(const char *op)
{
    fprintf(stderr, "audio: %s called but this build was made without "
            "ALSA. Use --without-audio.\n", op);
    return AUDIO_DEVICE_OPEN;
}

int init_audio_capture(audio_t *state)        { (void)state; return complain("init_audio_capture"); }
void audio_capture_cleanup(audio_t *state)    { (void)state; }
void *capture_audio(void *data)               { (void)data;  complain("capture_audio"); return NULL; }

int audio_playback_open(snd_pcm_t **handle, const char *device,
                        unsigned int rate_hz, unsigned int channels)
{
    (void)device; (void)rate_hz; (void)channels;
    if (handle) *handle = NULL;
    return complain("audio_playback_open");
}

struct audio_wav_writer { int unused; };
audio_wav_writer_t *audio_wav_writer_open(const char *path,
                                          unsigned int rate_hz,
                                          unsigned int channels)
{
    (void)path; (void)rate_hz; (void)channels;
    complain("audio_wav_writer_open");
    return NULL;
}
void audio_wav_writer_append(audio_wav_writer_t *w, const void *buf, size_t bytes)
{
    (void)w; (void)buf; (void)bytes;
}
void audio_wav_writer_close(audio_wav_writer_t *w) { (void)w; }

int audio_play_tone(snd_pcm_t *handle, audio_wav_writer_t *wav,
                    double freq_hz, double amplitude,
                    double duration_s, unsigned int rate_hz, unsigned int channels)
{
    (void)handle; (void)wav; (void)freq_hz; (void)amplitude;
    (void)duration_s; (void)rate_hz; (void)channels;
    return complain("audio_play_tone");
}

int audio_play_white_noise(snd_pcm_t *handle, audio_wav_writer_t *wav,
                           double amplitude, double duration_s,
                           unsigned int rate_hz, unsigned int channels,
                           uint64_t seed, double bandwidth_hz)
{
    (void)handle; (void)wav; (void)amplitude; (void)duration_s;
    (void)rate_hz; (void)channels; (void)seed; (void)bandwidth_hz;
    return complain("audio_play_white_noise");
}

void audio_playback_close(snd_pcm_t *handle) { (void)handle; }

int audio_capture_open(snd_pcm_t **handle, const char *device,
                       unsigned int rate_hz, unsigned int channels)
{
    (void)device; (void)rate_hz; (void)channels;
    if (handle) *handle = NULL;
    return complain("audio_capture_open");
}

int audio_capture(snd_pcm_t *handle, audio_wav_writer_t *wav, FILE *raw_out,
                  double duration_s, unsigned int rate_hz, unsigned int channels,
                  volatile sig_atomic_t *stop_flag)
{
    (void)handle; (void)wav; (void)raw_out; (void)duration_s;
    (void)rate_hz; (void)channels; (void)stop_flag;
    return complain("audio_capture");
}

void audio_capture_close(snd_pcm_t *handle) { (void)handle; }

int audio_generate_spectrogram(const char *wav_path)
{
    (void)wav_path;
    return -1;
}

int audio_find_alsa_card(const char *const *needles, int needle_count,
                         char *out_hint, size_t hint_cap)
{
    (void)needles; (void)needle_count;
    if (out_hint && hint_cap > 0) out_hint[0] = '\0';
    return -1;
}

int audio_card_usbid(int card_idx, char *out, size_t cap)
{
    (void)card_idx;
    if (out && cap > 0) out[0] = '\0';
    return -1;
}

int audio_find_alsa_card_by_usbid(const char *const *ids, int id_count)
{
    (void)ids; (void)id_count;
    return -1;
}

int audio_find_signalink_device(char *out, size_t cap)
{
    (void)out; (void)cap;
    return -1;
}

int audio_find_radio_device(const char *backend_hint, char *out, size_t cap)
{
    (void)backend_hint; (void)out; (void)cap;
    return -1;
}
