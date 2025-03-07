#include "audio.h"

#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <time.h>

static int result_status = 0;

int init_audio_capture(state_t *state)
{
    
    snd_pcm_hw_params_t *params;
    int dir;
    unsigned int rate = AUDIO_RATE_HZ;
    state->audio_frames = 32; // Number of frames per period
    char *audio_device_name = AUDIO_DEVICE_NAME_MAIN;
    if (state->audio_device == AUDIO_DEVICE_SUB) {
        audio_device_name = AUDIO_DEVICE_NAME_SUB;
    }
    
    // Open the PCM device for recording
    if (snd_pcm_open(&state->pcm_handle, audio_device_name, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        result_status = AUDIO_DEVICE_OPEN;
        return AUDIO_DEVICE_OPEN;
    }

    // Allocate hardware parameters object
    snd_pcm_hw_params_malloc(&params);
    snd_pcm_hw_params_any(state->pcm_handle, params);

    // Set parameters: Interleaved mode, sample format, channels, rate
    snd_pcm_hw_params_set_access(state->pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(state->pcm_handle, params, AUDIO_FORMAT);
    snd_pcm_hw_params_set_channels(state->pcm_handle, params, AUDIO_CHANNELS);
    snd_pcm_hw_params_set_rate_near(state->pcm_handle, params, &rate, &dir);
    snd_pcm_hw_params_set_period_size_near(state->pcm_handle, params, &state->audio_frames, &dir);

    // Apply the hardware parameters
    snd_pcm_hw_params(state->pcm_handle, params);
    snd_pcm_hw_params_free(params);
    
    // Open output file for raw PCM data
    char output_file[FILENAME_MAX];
    struct tm utc = {0};
    time_t t = time(0);
    utc = *gmtime(&t);
    snprintf(state->audio_output_filename, FILENAME_MAX, "%s_%s_%04d%02d%02dT%02d%02d%02d.raw", state->audio_output_file_basename, state->audio_device == AUDIO_DEVICE_MAIN ? "Main" : "Sub", utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec);
    state->audio_file = fopen(state->audio_output_filename, "wb");
    if (state->audio_file == NULL) {
        return AUDIO_FILE_OPEN;
    }

    return AUDIO_OK;

}

void audio_capture_cleanup(state_t *state)
{
    fclose(state->audio_file);
    snd_pcm_drain(state->pcm_handle);
    snd_pcm_close(state->pcm_handle);

    return;

}

void *capture_audio(void *data)
{
    if (data == NULL) {
        result_status = AUDIO_ARGS;
        return &result_status;
    }

    state_t *state = (state_t*)data;

    int buffer_size = state->audio_frames * AUDIO_CHANNELS * 2; // 2 bytes per sample
    state->audio_buffer = malloc(buffer_size);
    if (state->audio_buffer == NULL) {
        result_status = AUDIO_MEMORY;
        return &result_status;
    }

    // Start capturing audio
    snd_pcm_sframes_t n_frames_read = 0;
    double seconds_read = 0.0;
    while (state->recording_audio) {
        n_frames_read = snd_pcm_readi(state->pcm_handle, state->audio_buffer, state->audio_frames);
        if (n_frames_read < 0) {
            snd_pcm_prepare(state->pcm_handle); // Recover from errors
        }
        // 128 bytes of data
        fwrite(state->audio_buffer, 1, n_frames_read * AUDIO_CHANNELS * 2, state->audio_file);
        seconds_read += (double)n_frames_read / (double)AUDIO_RATE_HZ;
    }

    free(state->audio_buffer);
    state->audio_buffer = NULL;

    result_status = AUDIO_OK;
    return &result_status;
}
