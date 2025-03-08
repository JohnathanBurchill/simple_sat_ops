#include "audio.h"

#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <time.h>

static int result_status = 0;

int init_audio_capture(state_t *state)
{
    snd_pcm_hw_params_t *params_main;
    snd_pcm_hw_params_t *params_sub;
    int dir;
    unsigned int rate = AUDIO_RATE_HZ;
    state->audio_frames = 32; // Number of frames per period
    
    // Open the PCM devices for recording
    if (snd_pcm_open(&state->pcm_handle_main, AUDIO_DEVICE_NAME_MAIN, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        result_status = AUDIO_DEVICE_OPEN;
        return AUDIO_DEVICE_OPEN;
    }
    if (snd_pcm_open(&state->pcm_handle_sub, AUDIO_DEVICE_NAME_SUB, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        result_status = AUDIO_DEVICE_OPEN;
        return AUDIO_DEVICE_OPEN;
    }

    snd_pcm_hw_params_malloc(&params_main);
    snd_pcm_hw_params_any(state->pcm_handle_main, params_main);
    snd_pcm_hw_params_set_access(state->pcm_handle_main, params_main, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(state->pcm_handle_main, params_main, AUDIO_FORMAT);
    snd_pcm_hw_params_set_channels(state->pcm_handle_main, params_main, AUDIO_CHANNELS);
    snd_pcm_hw_params_set_rate_near(state->pcm_handle_main, params_main, &rate, &dir);
    snd_pcm_hw_params_set_period_size_near(state->pcm_handle_main, params_main, &state->audio_frames, &dir);
    snd_pcm_hw_params(state->pcm_handle_main, params_main);
    snd_pcm_hw_params_free(params_main);

    snd_pcm_hw_params_malloc(&params_sub);
    snd_pcm_hw_params_any(state->pcm_handle_sub, params_sub);
    snd_pcm_hw_params_set_access(state->pcm_handle_sub, params_sub, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(state->pcm_handle_sub, params_sub, AUDIO_FORMAT);
    snd_pcm_hw_params_set_channels(state->pcm_handle_sub, params_sub, AUDIO_CHANNELS);
    snd_pcm_hw_params_set_rate_near(state->pcm_handle_sub, params_sub, &rate, &dir);
    snd_pcm_hw_params_set_period_size_near(state->pcm_handle_sub, params_sub, &state->audio_frames, &dir);
    snd_pcm_hw_params(state->pcm_handle_sub, params_sub);

    
    // Open output file for raw PCM data
    char output_file[FILENAME_MAX];
    struct tm utc = {0};
    time_t t = time(0);
    utc = *gmtime(&t);
    snprintf(state->audio_output_filename_main, FILENAME_MAX, "%s_Main_%04d%02d%02dT%02d%02d%02d.raw", state->audio_output_file_basename, utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec);
    state->audio_file_main = fopen(state->audio_output_filename_main, "wb");
    if (state->audio_file_main == NULL) {
        return AUDIO_FILE_OPEN;
    }
    snprintf(state->audio_output_filename_sub, FILENAME_MAX, "%s_Sub__%04d%02d%02dT%02d%02d%02d.raw", state->audio_output_file_basename, utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec);
    state->audio_file_sub = fopen(state->audio_output_filename_sub, "wb");
    if (state->audio_file_sub == NULL) {
        return AUDIO_FILE_OPEN;
    }

    return AUDIO_OK;

}

void audio_capture_cleanup(state_t *state)
{
    fclose(state->audio_file_main);
    snd_pcm_drain(state->pcm_handle_main);
    snd_pcm_close(state->pcm_handle_main);
    fclose(state->audio_file_sub);
    snd_pcm_drain(state->pcm_handle_sub);
    snd_pcm_close(state->pcm_handle_sub);

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
    state->audio_buffer_main = malloc(buffer_size);
    if (state->audio_buffer_main == NULL) {
        result_status = AUDIO_MEMORY;
        return &result_status;
    }
    state->audio_buffer_sub = malloc(buffer_size);
    if (state->audio_buffer_sub == NULL) {
        result_status = AUDIO_MEMORY;
        return &result_status;
    }

    // Start capturing audio
    snd_pcm_sframes_t n_frames_read = 0;
    double seconds_read = 0.0;
    while (state->recording_audio) {
        n_frames_read = snd_pcm_readi(state->pcm_handle_main, state->audio_buffer_main, state->audio_frames);
        if (n_frames_read < 0) {
            snd_pcm_prepare(state->pcm_handle_main); // Recover from errors
        }
        n_frames_read = snd_pcm_readi(state->pcm_handle_sub, state->audio_buffer_sub, state->audio_frames);
        if (n_frames_read < 0) {
            snd_pcm_prepare(state->pcm_handle_sub); // Recover from errors
        }
        // 128 bytes of data
        fwrite(state->audio_buffer_main, 1, n_frames_read * AUDIO_CHANNELS * 2, state->audio_file_main);
        fwrite(state->audio_buffer_sub, 1, n_frames_read * AUDIO_CHANNELS * 2, state->audio_file_sub);
        seconds_read += (double)n_frames_read / (double)AUDIO_RATE_HZ;
    }

    free(state->audio_buffer_main);
    state->audio_buffer_main = NULL;
    free(state->audio_buffer_sub);
    state->audio_buffer_sub = NULL;

    result_status = AUDIO_OK;
    return &result_status;
}
