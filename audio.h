#ifndef AUDIO_H
#define AUDIO_H

#include <alsa/asoundlib.h>

#define AUDIO_DEVICE_NAME_MAIN "plughw:1,0"
#define AUDIO_DEVICE_NAME_SUB "plughw:1,0"
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
