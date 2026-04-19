#ifndef AUDIO_H
#define AUDIO_H

#include "state.h"

#define AUDIO_DEVICE_NAME_MAIN "hw:3,0"
#define AUDIO_DEVICE_NAME_SUB "hw:4,0"
#define AUDIO_RATE_HZ 48000
#define AUDIO_CHANNELS 2
#define AUDIO_FORMAT SND_PCM_FORMAT_S16_LE
#define AUDIO_CAPTURE_DURATION_S 60
#define AUDIO_PLAYBACK_FRAMES_PER_CHUNK 1024

enum audio_device_enum {
    AUDIO_DEVICE_MAIN = 0,
    AUDIO_DEVICE_SUB = 1,
};

enum audio_errors {
    AUDIO_OK = 0,
    AUDIO_ARGS,
    AUDIO_MEMORY,
    AUDIO_DEVICE_OPEN,
    AUDIO_FILE_OPEN,
};

int init_audio_capture(state_t *state);
void audio_capture_cleanup(state_t *state);
void *capture_audio(void *data);

int audio_playback_open(snd_pcm_t **handle, const char *device,
                        unsigned int rate_hz, unsigned int channels);
int audio_play_tone(snd_pcm_t *handle, double freq_hz, double amplitude,
                    double duration_s, unsigned int rate_hz, unsigned int channels);
void audio_playback_close(snd_pcm_t *handle);


#endif // AUDIO_H
