#include "audio.h"

#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <ncurses.h>

int capture_audio(state_t *state, int audio_device)
{
    snd_pcm_t *pcm_handle;
    snd_pcm_hw_params_t *params;
    int dir;
    unsigned int rate = AUDIO_RATE_HZ;
    snd_pcm_uframes_t frames = 32; // Number of frames per period
    int buffer_size = frames * AUDIO_CHANNELS * 2; // 2 bytes per sample
    char *buffer = (char *)malloc(buffer_size);

    char *audio_device_name = AUDIO_DEVICE_NAME_MAIN;
    if (audio_device == AUDIO_DEVICE_SUB) {
        audio_device_name = AUDIO_DEVICE_NAME_SUB;
    }
    
    // Open the PCM device for recording
    if (snd_pcm_open(&pcm_handle, audio_device_name, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        fprintf(stderr, "Error opening PCM device %s\n", audio_device_name);
        return 1;
    }

    // Allocate hardware parameters object
    snd_pcm_hw_params_malloc(&params);
    snd_pcm_hw_params_any(pcm_handle, params);

    // Set parameters: Interleaved mode, sample format, channels, rate
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, params, AUDIO_FORMAT);
    snd_pcm_hw_params_set_channels(pcm_handle, params, AUDIO_CHANNELS);
    snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, &dir);
    snd_pcm_hw_params_set_period_size_near(pcm_handle, params, &frames, &dir);

    // Apply the hardware parameters
    snd_pcm_hw_params(pcm_handle, params);
    snd_pcm_hw_params_free(params);
    
    // Open output file for raw PCM data
    FILE *file = fopen(state->audio_output_file, "wb");
    if (!file) {
        fprintf(stderr, "Error opening file for writing\n");
        return 1;
    }

    // Start capturing audio
    mvprintw(0, 0, "%s", "Recording...");
    clrtoeol();
    refresh();
    snd_pcm_sframes_t n_frames_read = 0;
    double seconds_read = 0.0;
    for (int i = 0; i < (AUDIO_RATE_HZ / frames) * AUDIO_CAPTURE_DURATION_S; i++) {
        n_frames_read = snd_pcm_readi(pcm_handle, buffer, frames);
        if (n_frames_read < 0) {
            snd_pcm_prepare(pcm_handle); // Recover from errors
        }
        // 128 bytes of data
        fwrite(buffer, 1, n_frames_read * AUDIO_CHANNELS * 2, file);
        seconds_read += (double)n_frames_read / (double)AUDIO_RATE_HZ;
        mvprintw(0, 0, "Recording...%6.1f s of %6.1f s ", seconds_read, (double)AUDIO_CAPTURE_DURATION_S);
        clrtoeol();
        refresh();
    }

    // Cleanup
    fclose(file);
    snd_pcm_drain(pcm_handle);
    snd_pcm_close(pcm_handle);
    free(buffer);

    mvprintw(0, 0, "%s", "Captured audio.");
    clrtoeol();
    refresh();
    
    return 0;
}
