#ifndef AUDIO_H
#define AUDIO_H

#include <pthread.h>
#include <stdio.h>

// On Linux we get the real ALSA types. On hosts without ALSA (macOS dev
// boxes building only the radio_ctl / non-audio binaries) we provide
// just-enough opaque stubs so audio_t can still be sized and state.h
// remains includeable. Code that actually calls audio_* functions only
// gets compiled on hosts where the real header is present.
#if defined(__has_include)
#  if __has_include(<alsa/asoundlib.h>)
#    include <alsa/asoundlib.h>
#    define SSO_HAVE_ALSA 1
#  endif
#endif
#ifndef SSO_HAVE_ALSA
typedef void *snd_pcm_t;
typedef unsigned long snd_pcm_uframes_t;
#  ifndef SND_PCM_FORMAT_S16_LE
#    define SND_PCM_FORMAT_S16_LE 2
#  endif
#endif

// RX capture path — IC-9700 native USB CODEC at card 4 on the RAO box.
// The SignaLink (card 1) is the TX-side audio+PTT device and is not used
// for capture.
#define AUDIO_DEVICE_NAME_MAIN "plughw:4,0"
#define AUDIO_DEVICE_NAME_SUB "plughw:4,0"
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

typedef struct audio {
    snd_pcm_t *pcm_handle_main;
    snd_pcm_t *pcm_handle_sub;
    snd_pcm_uframes_t audio_frames;
    pthread_t audio_thread_main;
    pthread_t audio_thread_sub;
    char *audio_output_file_basename;
    char audio_output_filename_main[FILENAME_MAX];
    char audio_output_filename_sub[FILENAME_MAX];
    FILE *audio_file_main;
    FILE *audio_file_sub;
    char *audio_buffer_main;
    char *audio_buffer_sub;
    volatile int recording_audio;
    int audio_record;
} audio_t;

int init_audio_capture(audio_t *state);
void audio_capture_cleanup(audio_t *state);
void *capture_audio(void *data);

int audio_playback_open(snd_pcm_t **handle, const char *device,
                        unsigned int rate_hz, unsigned int channels);
int audio_play_tone(snd_pcm_t *handle, double freq_hz, double amplitude,
                    double duration_s, unsigned int rate_hz, unsigned int channels);
void audio_playback_close(snd_pcm_t *handle);


#endif // AUDIO_H
