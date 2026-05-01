#include "audio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alsa/asoundlib.h>
#include <math.h>
#include <time.h>

static int result_status = 0;

int init_audio_capture(audio_t *state)
{
    snd_pcm_hw_params_t *params_main;
    snd_pcm_hw_params_t *params_sub;
    int dir;
    unsigned int rate = AUDIO_RATE_HZ;
    state->audio_frames = 32; // Number of frames per period
    
    // Open the PCM devices for recording. Sub is skipped when it'd be the
    // same device as Main — ALSA exclusive capture won't share hardware.
    state->pcm_handle_sub = NULL;
    int rc = snd_pcm_open(&state->pcm_handle_main, AUDIO_DEVICE_NAME_MAIN, SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) {
        fprintf(stderr, "audio: snd_pcm_open(%s, CAPTURE) failed: %s\n",
                AUDIO_DEVICE_NAME_MAIN, snd_strerror(rc));
        result_status = AUDIO_DEVICE_OPEN;
        return AUDIO_DEVICE_OPEN;
    }
    int have_sub = strcmp(AUDIO_DEVICE_NAME_SUB, AUDIO_DEVICE_NAME_MAIN) != 0;
    if (have_sub) {
        rc = snd_pcm_open(&state->pcm_handle_sub, AUDIO_DEVICE_NAME_SUB, SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0) {
            fprintf(stderr, "audio: snd_pcm_open(%s, CAPTURE) failed: %s\n",
                    AUDIO_DEVICE_NAME_SUB, snd_strerror(rc));
            result_status = AUDIO_DEVICE_OPEN;
            return AUDIO_DEVICE_OPEN;
        }
    }

    snd_pcm_hw_params_malloc(&params_main);
    snd_pcm_hw_params_any(state->pcm_handle_main, params_main);
    snd_pcm_hw_params_set_access(state->pcm_handle_main, params_main, SND_PCM_ACCESS_RW_INTERLEAVED);
    if ((rc = snd_pcm_hw_params_set_format(state->pcm_handle_main, params_main, AUDIO_FORMAT)) < 0) {
        fprintf(stderr, "audio: set_format S16_LE on %s failed: %s\n",
                AUDIO_DEVICE_NAME_MAIN, snd_strerror(rc));
        snd_pcm_hw_params_free(params_main);
        return AUDIO_DEVICE_OPEN;
    }
    if ((rc = snd_pcm_hw_params_set_channels(state->pcm_handle_main, params_main, AUDIO_CHANNELS)) < 0) {
        fprintf(stderr, "audio: set_channels(%d) on %s failed: %s\n",
                AUDIO_CHANNELS, AUDIO_DEVICE_NAME_MAIN, snd_strerror(rc));
        snd_pcm_hw_params_free(params_main);
        return AUDIO_DEVICE_OPEN;
    }
    if ((rc = snd_pcm_hw_params_set_rate_near(state->pcm_handle_main, params_main, &rate, &dir)) < 0) {
        fprintf(stderr, "audio: set_rate_near(%u) on %s failed: %s\n",
                rate, AUDIO_DEVICE_NAME_MAIN, snd_strerror(rc));
        snd_pcm_hw_params_free(params_main);
        return AUDIO_DEVICE_OPEN;
    }
    snd_pcm_hw_params_set_period_size_near(state->pcm_handle_main, params_main, &state->audio_frames, &dir);
    if ((rc = snd_pcm_hw_params(state->pcm_handle_main, params_main)) < 0) {
        fprintf(stderr, "audio: hw_params apply on %s failed: %s\n",
                AUDIO_DEVICE_NAME_MAIN, snd_strerror(rc));
        snd_pcm_hw_params_free(params_main);
        return AUDIO_DEVICE_OPEN;
    }
    snd_pcm_hw_params_free(params_main);

    if (have_sub) {
        snd_pcm_hw_params_malloc(&params_sub);
        snd_pcm_hw_params_any(state->pcm_handle_sub, params_sub);
        snd_pcm_hw_params_set_access(state->pcm_handle_sub, params_sub, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(state->pcm_handle_sub, params_sub, AUDIO_FORMAT);
        snd_pcm_hw_params_set_channels(state->pcm_handle_sub, params_sub, AUDIO_CHANNELS);
        snd_pcm_hw_params_set_rate_near(state->pcm_handle_sub, params_sub, &rate, &dir);
        snd_pcm_hw_params_set_period_size_near(state->pcm_handle_sub, params_sub, &state->audio_frames, &dir);
        snd_pcm_hw_params(state->pcm_handle_sub, params_sub);
    }

    
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
    state->audio_file_sub = NULL;
    if (state->pcm_handle_sub != NULL) {
        snprintf(state->audio_output_filename_sub, FILENAME_MAX, "%s_Sub__%04d%02d%02dT%02d%02d%02d.raw", state->audio_output_file_basename, utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec);
        state->audio_file_sub = fopen(state->audio_output_filename_sub, "wb");
        if (state->audio_file_sub == NULL) {
            return AUDIO_FILE_OPEN;
        }
    }

    return AUDIO_OK;

}

void audio_capture_cleanup(audio_t *state)
{
    if (state->audio_file_main) fclose(state->audio_file_main);
    if (state->pcm_handle_main) {
        snd_pcm_drain(state->pcm_handle_main);
        snd_pcm_close(state->pcm_handle_main);
    }
    if (state->audio_file_sub) fclose(state->audio_file_sub);
    if (state->pcm_handle_sub) {
        snd_pcm_drain(state->pcm_handle_sub);
        snd_pcm_close(state->pcm_handle_sub);
    }

    return;

}

void *capture_audio(void *data)
{
    if (data == NULL) {
        result_status = AUDIO_ARGS;
        return &result_status;
    }

    audio_t *state = (audio_t*)data;

    int buffer_size = state->audio_frames * AUDIO_CHANNELS * 2; // 2 bytes per sample
    state->audio_buffer_main = malloc(buffer_size);
    if (state->audio_buffer_main == NULL) {
        result_status = AUDIO_MEMORY;
        return &result_status;
    }
    state->audio_buffer_sub = NULL;
    if (state->pcm_handle_sub != NULL) {
        state->audio_buffer_sub = malloc(buffer_size);
        if (state->audio_buffer_sub == NULL) {
            result_status = AUDIO_MEMORY;
            return &result_status;
        }
    }

    // Start capturing audio
    snd_pcm_sframes_t n_frames_read = 0;
    double seconds_read = 0.0;
    while (state->recording_audio) {
        n_frames_read = snd_pcm_readi(state->pcm_handle_main, state->audio_buffer_main, state->audio_frames);
        if (n_frames_read < 0) {
            snd_pcm_prepare(state->pcm_handle_main); // Recover from errors
        }
        if (n_frames_read > 0) {
            fwrite(state->audio_buffer_main, 1, n_frames_read * AUDIO_CHANNELS * 2, state->audio_file_main);
        }
        if (state->pcm_handle_sub != NULL) {
            snd_pcm_sframes_t n_sub = snd_pcm_readi(state->pcm_handle_sub, state->audio_buffer_sub, state->audio_frames);
            if (n_sub < 0) {
                snd_pcm_prepare(state->pcm_handle_sub);
            } else if (n_sub > 0) {
                fwrite(state->audio_buffer_sub, 1, n_sub * AUDIO_CHANNELS * 2, state->audio_file_sub);
            }
        }
        seconds_read += (double)n_frames_read / (double)AUDIO_RATE_HZ;
    }

    free(state->audio_buffer_main);
    state->audio_buffer_main = NULL;
    if (state->audio_buffer_sub) {
        free(state->audio_buffer_sub);
        state->audio_buffer_sub = NULL;
    }

    result_status = AUDIO_OK;
    return &result_status;
}

int audio_playback_open(snd_pcm_t **handle, const char *device,
                        unsigned int rate_hz, unsigned int channels)
{
    if (handle == NULL || device == NULL) {
        return AUDIO_ARGS;
    }
    if (snd_pcm_open(handle, device, SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        return AUDIO_DEVICE_OPEN;
    }

    snd_pcm_hw_params_t *params = NULL;
    snd_pcm_hw_params_malloc(&params);
    snd_pcm_hw_params_any(*handle, params);
    snd_pcm_hw_params_set_access(*handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(*handle, params, AUDIO_FORMAT);
    snd_pcm_hw_params_set_channels(*handle, params, channels);
    unsigned int rate = rate_hz;
    int dir = 0;
    snd_pcm_hw_params_set_rate_near(*handle, params, &rate, &dir);
    snd_pcm_uframes_t period = AUDIO_PLAYBACK_FRAMES_PER_CHUNK;
    snd_pcm_hw_params_set_period_size_near(*handle, params, &period, &dir);
    int apply = snd_pcm_hw_params(*handle, params);
    snd_pcm_hw_params_free(params);
    if (apply < 0) {
        snd_pcm_close(*handle);
        *handle = NULL;
        return AUDIO_DEVICE_OPEN;
    }
    return AUDIO_OK;
}

int audio_play_tone(snd_pcm_t *handle, double freq_hz, double amplitude,
                    double duration_s, unsigned int rate_hz, unsigned int channels)
{
    if (handle == NULL || channels == 0 || rate_hz == 0) {
        return AUDIO_ARGS;
    }
    if (amplitude < 0.0) amplitude = 0.0;
    if (amplitude > 1.0) amplitude = 1.0;

    const snd_pcm_uframes_t frames_per_chunk = AUDIO_PLAYBACK_FRAMES_PER_CHUNK;
    int16_t *buf = malloc(frames_per_chunk * channels * sizeof(int16_t));
    if (buf == NULL) {
        return AUDIO_MEMORY;
    }

    double phase = 0.0;
    const double phase_inc = 2.0 * M_PI * freq_hz / (double)rate_hz;
    const int16_t scale = (int16_t)(amplitude * 32767.0);
    const uint64_t total_frames = (uint64_t)(duration_s * (double)rate_hz);
    uint64_t frames_sent = 0;

    while (frames_sent < total_frames) {
        snd_pcm_uframes_t n = frames_per_chunk;
        if (n > total_frames - frames_sent) {
            n = (snd_pcm_uframes_t)(total_frames - frames_sent);
        }
        for (snd_pcm_uframes_t i = 0; i < n; i++) {
            int16_t sample = (int16_t)((double)scale * sin(phase));
            phase += phase_inc;
            if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
            for (unsigned int c = 0; c < channels; c++) {
                buf[i * channels + c] = sample;
            }
        }
        snd_pcm_sframes_t written = snd_pcm_writei(handle, buf, n);
        if (written == -EPIPE) {
            snd_pcm_prepare(handle);
            continue;
        }
        if (written < 0) {
            free(buf);
            return AUDIO_DEVICE_OPEN;
        }
        frames_sent += (uint64_t)written;
    }

    free(buf);
    return AUDIO_OK;
}

// xorshift64 — fast, decent statistical quality, no library dependency.
// Plenty for audio-grade noise; we don't need cryptographic randomness.
static inline uint64_t xorshift64_step(uint64_t *state)
{
    uint64_t x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    return x;
}

int audio_play_white_noise(snd_pcm_t *handle, double amplitude,
                           double duration_s, unsigned int rate_hz,
                           unsigned int channels, uint64_t seed)
{
    if (handle == NULL || channels == 0 || rate_hz == 0) {
        return AUDIO_ARGS;
    }
    if (amplitude < 0.0) amplitude = 0.0;
    if (amplitude > 1.0) amplitude = 1.0;

    const snd_pcm_uframes_t frames_per_chunk = AUDIO_PLAYBACK_FRAMES_PER_CHUNK;
    int16_t *buf = malloc(frames_per_chunk * channels * sizeof(int16_t));
    if (buf == NULL) {
        return AUDIO_MEMORY;
    }

    // xorshift64 deadlocks on a zero state; if the caller passed 0, derive a
    // non-zero seed from the wall clock so successive runs differ. Any other
    // value (including 1) is taken verbatim for repeatable captures.
    uint64_t state = seed;
    if (state == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        state = (uint64_t)ts.tv_sec * 1000000003ull
              ^ (uint64_t)ts.tv_nsec * 6364136223846793005ull;
        if (state == 0) state = 0x9E3779B97F4A7C15ull;
    }

    const double scale = amplitude * 32767.0;
    const uint64_t total_frames = (uint64_t)(duration_s * (double)rate_hz);
    uint64_t frames_sent = 0;

    while (frames_sent < total_frames) {
        snd_pcm_uframes_t n = frames_per_chunk;
        if (n > total_frames - frames_sent) {
            n = (snd_pcm_uframes_t)(total_frames - frames_sent);
        }
        for (snd_pcm_uframes_t i = 0; i < n; i++) {
            // Draw 32 bits and map to [-1, 1). Independent draw per frame
            // gives a flat PSD; channels carry the same sample so left/right
            // are coherent and the captured spectrum is unambiguous.
            uint32_t r = (uint32_t)(xorshift64_step(&state) >> 32);
            double u = ((double)r / 4294967295.0) * 2.0 - 1.0;
            int16_t sample = (int16_t)(scale * u);
            for (unsigned int c = 0; c < channels; c++) {
                buf[i * channels + c] = sample;
            }
        }
        snd_pcm_sframes_t written = snd_pcm_writei(handle, buf, n);
        if (written == -EPIPE) {
            snd_pcm_prepare(handle);
            continue;
        }
        if (written < 0) {
            free(buf);
            return AUDIO_DEVICE_OPEN;
        }
        frames_sent += (uint64_t)written;
    }

    free(buf);
    return AUDIO_OK;
}

void audio_playback_close(snd_pcm_t *handle)
{
    if (handle == NULL) return;
    snd_pcm_drain(handle);
    snd_pcm_close(handle);
}

// /proc/asound/cards is two lines per card:
//
//    N [shortname     ]: driver - longname
//                         details on second line
//
// We pair adjacent lines, strip leading whitespace, and substring-match
// the concatenation against the needle list. Returns the leading card
// index from the first matching pair.
int audio_find_alsa_card(const char *const *needles, int needle_count,
                         char *out_hint, size_t hint_cap)
{
    if (needles == NULL || needle_count <= 0) return -2;
    FILE *f = fopen("/proc/asound/cards", "r");
    if (f == NULL) return -2;
    char header[256];
    char details[256];
    int found = -1;
    while (fgets(header, sizeof header, f) != NULL) {
        if (fgets(details, sizeof details, f) == NULL) break;
        // Header starts with N or whitespace+N.
        const char *p = header;
        while (*p == ' ' || *p == '\t') p++;
        if (*p < '0' || *p > '9') continue;
        int idx = atoi(p);
        char combined[512];
        snprintf(combined, sizeof combined, "%s %s", header, details);
        for (int i = 0; i < needle_count; i++) {
            if (needles[i] != NULL && strstr(combined, needles[i]) != NULL) {
                found = idx;
                if (out_hint != NULL && hint_cap > 0) {
                    snprintf(out_hint, hint_cap, "%s", combined);
                    // Trim trailing newline for tidy log output.
                    size_t n = strlen(out_hint);
                    while (n > 0 && (out_hint[n-1] == '\n' || out_hint[n-1] == '\r')) {
                        out_hint[--n] = '\0';
                    }
                }
                break;
            }
        }
        if (found >= 0) break;
    }
    fclose(f);
    return found >= 0 ? found : -1;
}

int audio_find_signalink_device(char *out, size_t cap)
{
    if (out == NULL || cap < 16) return -2;
    // SignaLink USB uses TI/Burr-Brown PCM290x. Kernel reports it with
    // either string in the long-name field; either is enough to ID it
    // uniquely on our ground-station boxes (no other Burr-Brown audio
    // chips in use here).
    const char *needles[] = { "Burr-Brown", "PCM2901", "PCM2904" };
    int idx = audio_find_alsa_card(needles, 3, NULL, 0);
    if (idx < 0) return idx;
    snprintf(out, cap, "plughw:%d,0", idx);
    return 0;
}

int audio_find_radio_device(const char *backend_hint, char *out, size_t cap)
{
    if (out == NULL || cap < 16) return -2;
    // Yaesu rigs enumerate with "Yaesu" or specific model number (FT-991A,
    // FT-DX, etc.) in the long-name. ICOM rigs enumerate with "IC-9700"
    // or "ICOM". Caller can narrow via backend_hint, otherwise we search
    // both.
    const char *yaesu_needles[] = { "Yaesu", "YAESU", "FT-991A", "FT-DX" };
    const char *icom_needles[]  = { "IC-9700", "IC-705", "ICOM", "Icom" };

    int idx = -1;
    if (backend_hint == NULL || strcmp(backend_hint, "yaesu") == 0) {
        idx = audio_find_alsa_card(yaesu_needles,
                                   (int)(sizeof yaesu_needles / sizeof yaesu_needles[0]),
                                   NULL, 0);
        if (idx >= 0) { snprintf(out, cap, "plughw:%d,0", idx); return 0; }
    }
    if (backend_hint == NULL || strcmp(backend_hint, "icom") == 0) {
        idx = audio_find_alsa_card(icom_needles,
                                   (int)(sizeof icom_needles / sizeof icom_needles[0]),
                                   NULL, 0);
        if (idx >= 0) { snprintf(out, cap, "plughw:%d,0", idx); return 0; }
    }
    return -1;
}
