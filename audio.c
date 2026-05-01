#include "audio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alsa/asoundlib.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

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

// PCM S16_LE WAV writer. Header is a fixed 44 bytes (RIFF/fmt /data) with
// 32-bit length placeholders that get patched on close. We assume little-
// endian host (true on every box this project runs on); the multi-byte
// fields go in via memcpy of host-order ints, which matches the WAV
// spec's LE encoding without any swapping.
struct audio_wav_writer {
    FILE *fp;
    uint32_t data_bytes;
    unsigned int rate_hz;
    unsigned int channels;
};

audio_wav_writer_t *audio_wav_writer_open(const char *path,
                                          unsigned int rate_hz,
                                          unsigned int channels)
{
    if (path == NULL || channels == 0 || rate_hz == 0) return NULL;
    FILE *fp = fopen(path, "wb");
    if (fp == NULL) return NULL;
    uint8_t hdr[44] = {0};
    memcpy(hdr + 0, "RIFF", 4);
    /* hdr[4..7]   = 36 + data_bytes  (patched on close) */
    memcpy(hdr + 8, "WAVE", 4);
    memcpy(hdr + 12, "fmt ", 4);
    uint32_t fmt_chunk_size = 16;
    memcpy(hdr + 16, &fmt_chunk_size, 4);
    uint16_t fmt_pcm = 1;
    memcpy(hdr + 20, &fmt_pcm, 2);
    uint16_t ch16 = (uint16_t)channels;
    memcpy(hdr + 22, &ch16, 2);
    uint32_t rate32 = rate_hz;
    memcpy(hdr + 24, &rate32, 4);
    uint32_t byte_rate = rate_hz * channels * 2;
    memcpy(hdr + 28, &byte_rate, 4);
    uint16_t block_align = (uint16_t)(channels * 2);
    memcpy(hdr + 32, &block_align, 2);
    uint16_t bits = 16;
    memcpy(hdr + 34, &bits, 2);
    memcpy(hdr + 36, "data", 4);
    /* hdr[40..43] = data_bytes  (patched on close) */
    if (fwrite(hdr, 1, sizeof hdr, fp) != sizeof hdr) {
        fclose(fp);
        return NULL;
    }
    audio_wav_writer_t *w = malloc(sizeof *w);
    if (w == NULL) {
        fclose(fp);
        return NULL;
    }
    w->fp = fp;
    w->data_bytes = 0;
    w->rate_hz = rate_hz;
    w->channels = channels;
    return w;
}

void audio_wav_writer_close(audio_wav_writer_t *w)
{
    if (w == NULL) return;
    if (w->fp != NULL) {
        uint32_t riff_size = 36 + w->data_bytes;
        if (fseek(w->fp, 4, SEEK_SET) == 0) {
            fwrite(&riff_size, 4, 1, w->fp);
        }
        if (fseek(w->fp, 40, SEEK_SET) == 0) {
            fwrite(&w->data_bytes, 4, 1, w->fp);
        }
        fclose(w->fp);
    }
    free(w);
}

static void wav_writer_append(audio_wav_writer_t *w, const void *buf, size_t bytes)
{
    if (w == NULL || w->fp == NULL || bytes == 0) return;
    size_t n = fwrite(buf, 1, bytes, w->fp);
    w->data_bytes += (uint32_t)n;
}

int audio_play_tone(snd_pcm_t *handle, audio_wav_writer_t *wav,
                    double freq_hz, double amplitude,
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
        // Mirror to the synth WAV before the ALSA write — that way the
        // WAV captures the chunk we *intended* to send, even if ALSA
        // returns a short write or EPIPE and we re-try.
        wav_writer_append(wav, buf, (size_t)n * channels * sizeof(int16_t));
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

// Build a Hamming-windowed sinc lowpass FIR with N taps and normalized
// cutoff fc_norm = fc/fs (0 < fc_norm < 0.5). N must be odd. Sums to
// unity so DC gain is 1. ~80 dB rejection beyond the transition band
// for N=127, fc_norm = 0.3 (i.e. 14.4 kHz at 48 kHz).
static void make_lowpass_fir(double *taps, int N, double fc_norm)
{
    int M = N - 1;
    double sum = 0.0;
    for (int n = 0; n < N; n++) {
        int k = n - M / 2;
        double s;
        if (k == 0) {
            s = 2.0 * fc_norm;
        } else {
            double w = 2.0 * M_PI * fc_norm * (double)k;
            s = sin(w) / (M_PI * (double)k);
        }
        double window = 0.54 - 0.46 * cos(2.0 * M_PI * (double)n / (double)M);
        taps[n] = s * window;
        sum += taps[n];
    }
    // Normalise so DC gain = 1 (Hamming + sinc has small ripple).
    if (sum != 0.0) {
        for (int n = 0; n < N; n++) taps[n] /= sum;
    }
}

int audio_play_white_noise(snd_pcm_t *handle, audio_wav_writer_t *wav,
                           double amplitude, double duration_s,
                           unsigned int rate_hz, unsigned int channels,
                           uint64_t seed, double bandwidth_hz)
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

    // Optional FIR low-pass. 127 taps at fs=48k gives a transition band of
    // about 750 Hz and ~80 dB stopband rejection — sharp enough that the
    // spectrogram of the synth WAV has a near-vertical edge at the
    // requested cutoff. Convolution uses a circular buffer to avoid
    // shifting state on every sample.
    int filter_taps = 0;
    double *taps = NULL;
    double *fir_state = NULL;
    int fir_idx = 0;
    const double nyquist = (double)rate_hz / 2.0;
    if (bandwidth_hz > 0.0 && bandwidth_hz < nyquist) {
        filter_taps = 127;
        taps = malloc((size_t)filter_taps * sizeof(double));
        fir_state = calloc((size_t)filter_taps, sizeof(double));
        if (taps == NULL || fir_state == NULL) {
            free(taps); free(fir_state); free(buf);
            return AUDIO_MEMORY;
        }
        make_lowpass_fir(taps, filter_taps, bandwidth_hz / (double)rate_hz);
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
            uint32_t r = (uint32_t)(xorshift64_step(&state) >> 32);
            double u = ((double)r / 4294967295.0) * 2.0 - 1.0;
            if (filter_taps > 0) {
                // Push current sample into circular buffer, then convolve.
                fir_state[fir_idx] = u;
                double y = 0.0;
                int idx = fir_idx;
                for (int k = 0; k < filter_taps; k++) {
                    y += fir_state[idx] * taps[k];
                    idx = (idx == 0) ? filter_taps - 1 : idx - 1;
                }
                fir_idx = (fir_idx + 1) % filter_taps;
                u = y;
            }
            int16_t sample = (int16_t)(scale * u);
            for (unsigned int c = 0; c < channels; c++) {
                buf[i * channels + c] = sample;
            }
        }
        wav_writer_append(wav, buf, (size_t)n * channels * sizeof(int16_t));
        snd_pcm_sframes_t written = snd_pcm_writei(handle, buf, n);
        if (written == -EPIPE) {
            snd_pcm_prepare(handle);
            continue;
        }
        if (written < 0) {
            free(taps); free(fir_state); free(buf);
            return AUDIO_DEVICE_OPEN;
        }
        frames_sent += (uint64_t)written;
    }

    free(taps);
    free(fir_state);
    free(buf);
    return AUDIO_OK;
}

void audio_playback_close(snd_pcm_t *handle)
{
    if (handle == NULL) return;
    snd_pcm_drain(handle);
    snd_pcm_close(handle);
}

int audio_capture_open(snd_pcm_t **handle, const char *device,
                       unsigned int rate_hz, unsigned int channels)
{
    if (handle == NULL || device == NULL) {
        return AUDIO_ARGS;
    }
    if (snd_pcm_open(handle, device, SND_PCM_STREAM_CAPTURE, 0) < 0) {
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

int audio_capture(snd_pcm_t *handle,
                  audio_wav_writer_t *wav, FILE *raw_out,
                  double duration_s,
                  unsigned int rate_hz, unsigned int channels,
                  volatile sig_atomic_t *stop_flag)
{
    if (handle == NULL || channels == 0 || rate_hz == 0) {
        return AUDIO_ARGS;
    }

    const snd_pcm_uframes_t frames_per_chunk = AUDIO_PLAYBACK_FRAMES_PER_CHUNK;
    int16_t *buf = malloc(frames_per_chunk * channels * sizeof(int16_t));
    if (buf == NULL) {
        return AUDIO_MEMORY;
    }

    // duration_s <= 0 means "capture until stop_flag is set" — useful for
    // open-ended Ctrl-C captures. With duration_s > 0 we still honour
    // stop_flag so an early Ctrl-C cleanly truncates outputs at the last
    // chunk read, instead of either timing out the full duration or
    // SIGKILLing with a half-written WAV header.
    const uint64_t target_frames = (duration_s > 0.0)
        ? (uint64_t)(duration_s * (double)rate_hz)
        : 0;
    uint64_t frames_read = 0;

    while ((target_frames == 0 || frames_read < target_frames)
           && (stop_flag == NULL || *stop_flag == 0)) {
        snd_pcm_uframes_t want = frames_per_chunk;
        if (target_frames != 0) {
            uint64_t remain = target_frames - frames_read;
            if (want > remain) want = (snd_pcm_uframes_t)remain;
        }
        snd_pcm_sframes_t got = snd_pcm_readi(handle, buf, want);
        if (got == -EPIPE) {
            // Overrun. Recover and keep going.
            snd_pcm_prepare(handle);
            continue;
        }
        if (got < 0) {
            free(buf);
            return AUDIO_DEVICE_OPEN;
        }
        size_t bytes = (size_t)got * channels * sizeof(int16_t);
        if (wav != NULL) {
            wav_writer_append(wav, buf, bytes);
        }
        if (raw_out != NULL) {
            // Best-effort: a short write here only loses tail samples, the
            // capture itself keeps running. We don't error out so the WAV
            // (which will still be valid) gets a chance to close cleanly.
            (void)fwrite(buf, 1, bytes, raw_out);
        }
        frames_read += (uint64_t)got;
    }

    free(buf);
    return AUDIO_OK;
}

void audio_capture_close(snd_pcm_t *handle)
{
    if (handle == NULL) return;
    // drop, not drain: drain on a capture stream waits for the app to
    // read out everything still in the buffer, which we don't want here.
    snd_pcm_drop(handle);
    snd_pcm_close(handle);
}

int audio_generate_spectrogram(const char *wav_path)
{
    if (wav_path == NULL) return -1;
    size_t len = strlen(wav_path);
    char png_path[512];
    int n;
    if (len >= 4 && strcmp(wav_path + len - 4, ".wav") == 0) {
        n = snprintf(png_path, sizeof png_path, "%.*s.png",
                     (int)(len - 4), wav_path);
    } else {
        n = snprintf(png_path, sizeof png_path, "%s.png", wav_path);
    }
    if (n <= 0 || (size_t)n >= sizeof png_path) return -1;

    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "warning: fork() for ffmpeg spectrogram failed\n");
        return -1;
    }
    if (pid == 0) {
        // Child. Replace stdout/stderr with /dev/null so ffmpeg's normal
        // chatter doesn't pollute the parent's terminal — we already pass
        // -loglevel error, but the encoder still emits a few lines.
        char *args[] = {
            "ffmpeg",
            "-hide_banner", "-loglevel", "error", "-y",
            "-i", (char *)wav_path,
            "-lavfi",
            "showspectrumpic=s=1920x1080:mode=combined:color=intensity:legend=1",
            png_path,
            NULL,
        };
        execvp("ffmpeg", args);
        // execvp only returns on failure.
        fprintf(stderr, "warning: ffmpeg not found on PATH; "
                "skipping spectrogram for %s\n", wav_path);
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        return -1;
    }
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        fprintf(stderr, "spectrogram -> %s\n", png_path);
        return 0;
    }
    return -1;
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

int audio_card_usbid(int card_idx, char *out, size_t cap)
{
    if (out == NULL || cap < 12) return -1;
    char path[64];
    snprintf(path, sizeof path, "/proc/asound/card%d/usbid", card_idx);
    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;
    if (fgets(out, cap, f) == NULL) {
        fclose(f);
        return -1;
    }
    fclose(f);
    size_t n = strlen(out);
    while (n > 0 && (out[n-1] == '\n' || out[n-1] == '\r' || out[n-1] == ' ')) {
        out[--n] = '\0';
    }
    return n > 0 ? 0 : -1;
}

int audio_find_alsa_card_by_usbid(const char *const *ids, int id_count)
{
    if (ids == NULL || id_count <= 0) return -2;
    FILE *f = fopen("/proc/asound/cards", "r");
    if (f == NULL) return -2;
    char header[256], details[256];
    int found = -1;
    while (fgets(header, sizeof header, f) != NULL) {
        if (fgets(details, sizeof details, f) == NULL) break;
        const char *p = header;
        while (*p == ' ' || *p == '\t') p++;
        if (*p < '0' || *p > '9') continue;
        int idx = atoi(p);
        char usbid[16] = {0};
        if (audio_card_usbid(idx, usbid, sizeof usbid) != 0) continue;
        for (int i = 0; i < id_count; i++) {
            if (ids[i] != NULL && strcasecmp(ids[i], usbid) == 0) {
                found = idx;
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
    // SignaLink USB uses TI/Burr-Brown PCM290x. The VID is always 08bb
    // (Texas Instruments); the PID varies across hardware revisions.
    // We match by VID:PID first (stable across kernel-string changes,
    // identically-named devices, etc.) and fall back to substring
    // matching the kernel-reported card name in case a future revision
    // ships with a PID we haven't catalogued.
    const char *ids[] = { "08bb:2901", "08bb:2902", "08bb:2904" };
    int idx = audio_find_alsa_card_by_usbid(ids,
                                           (int)(sizeof ids / sizeof ids[0]));
    if (idx < 0) {
        const char *needles[] = { "Burr-Brown", "PCM2901", "PCM2902", "PCM2904" };
        idx = audio_find_alsa_card(needles,
                                   (int)(sizeof needles / sizeof needles[0]),
                                   NULL, 0);
    }
    if (idx < 0) return idx;
    snprintf(out, cap, "plughw:%d,0", idx);
    char usbid[16] = {0};
    if (audio_card_usbid(idx, usbid, sizeof usbid) == 0) {
        fprintf(stderr, "audio: SignaLink %s on card %d\n", usbid, idx);
    }
    return 0;
}

int audio_find_radio_device(const char *backend_hint, char *out, size_t cap)
{
    if (out == NULL || cap < 16) return -2;
    // VID:PID lists for radio USB Audio Class interfaces. These are the
    // values observed in the field; add more as you encounter them.
    //
    //   Yaesu FT-991A    : 0bda:481c (Realtek-licensed CMedia in some
    //                      lots) / 0d8c:0008 (CMedia branded). Confirm
    //                      on the Linux box with `cat /proc/asound/
    //                      card<N>/usbid` once the radio enumerates.
    //   ICOM IC-9700    : 0c26:001a / 0c26:0030 (ICOM Inc.).
    //
    // We always also fall back to substring matching by manufacturer
    // name in case the VID:PID isn't on our list yet.
    const char *yaesu_ids[]    = { "0bda:481c", "0d8c:0008", "0779:0001" };
    const char *icom_ids[]     = { "0c26:001a", "0c26:0030" };
    const char *yaesu_needles[] = { "Yaesu", "YAESU", "FT-991A", "FT-DX" };
    const char *icom_needles[]  = { "IC-9700", "IC-705", "ICOM", "Icom" };

    int idx = -1;
    const char *kind = NULL;

    if (backend_hint == NULL || strcmp(backend_hint, "yaesu") == 0) {
        idx = audio_find_alsa_card_by_usbid(yaesu_ids,
                                           (int)(sizeof yaesu_ids / sizeof yaesu_ids[0]));
        if (idx < 0) {
            idx = audio_find_alsa_card(yaesu_needles,
                                       (int)(sizeof yaesu_needles / sizeof yaesu_needles[0]),
                                       NULL, 0);
        }
        if (idx >= 0) kind = "Yaesu";
    }
    if (idx < 0 && (backend_hint == NULL || strcmp(backend_hint, "icom") == 0)) {
        idx = audio_find_alsa_card_by_usbid(icom_ids,
                                           (int)(sizeof icom_ids / sizeof icom_ids[0]));
        if (idx < 0) {
            idx = audio_find_alsa_card(icom_needles,
                                       (int)(sizeof icom_needles / sizeof icom_needles[0]),
                                       NULL, 0);
        }
        if (idx >= 0) kind = "ICOM";
    }
    if (idx < 0) return -1;

    snprintf(out, cap, "plughw:%d,0", idx);
    char usbid[16] = {0};
    if (audio_card_usbid(idx, usbid, sizeof usbid) == 0) {
        fprintf(stderr, "audio: %s radio %s on card %d\n",
                kind ? kind : "radio", usbid, idx);
    }
    return 0;
}
