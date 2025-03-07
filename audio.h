#ifndef AUDIO_H
#define AUDIO_H

#include "state.h"

#define AUDIO_DEVICE_NAME_MAIN "hw:3,0"
#define AUDIO_DEVICE_NAME_SUB "hw:4,0"
#define AUDIO_RATE_HZ 48000
#define AUDIO_CHANNELS 2
#define AUDIO_FORMAT SND_PCM_FORMAT_S16_LE
#define AUDIO_CAPTURE_DURATION_S 60 

enum audio_device_enum {
    AUDIO_DEVICE_MAIN = 0,
    AUDIO_DEVICE_SUB = 1,
};

int capture_audio(state_t *state, int audio_device);


#endif // AUDIO_H
