#ifndef AUDIO_H
#define AUDIO_H

#include <pthread.h>
#include <stdint.h>
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

// WAV writer for capturing the synthesized signal as it streams to the
// audio card. PCM S16_LE; opaque handle. open() writes a placeholder
// header; close() patches the size fields and closes the file. Pass the
// returned handle (or NULL = don't write) to audio_play_tone /
// audio_play_white_noise; multiple play calls against the same writer
// concatenate into one growing WAV (useful for stepped sweeps).
typedef struct audio_wav_writer audio_wav_writer_t;
audio_wav_writer_t *audio_wav_writer_open(const char *path,
                                          unsigned int rate_hz,
                                          unsigned int channels);
void audio_wav_writer_close(audio_wav_writer_t *w);

int audio_play_tone(snd_pcm_t *handle, audio_wav_writer_t *wav,
                    double freq_hz, double amplitude,
                    double duration_s, unsigned int rate_hz, unsigned int channels);
// Uniform-PRNG white noise across the full Nyquist band. Each output sample is
// an independent draw, so the PSD is flat to within sampling noise and the
// captured spectrum reflects the inherent TX/RX bandshape. seed=0 picks a
// time-based seed; pass any non-zero value for deterministic output.
int audio_play_white_noise(snd_pcm_t *handle, audio_wav_writer_t *wav,
                           double amplitude, double duration_s,
                           unsigned int rate_hz, unsigned int channels,
                           uint64_t seed);
void audio_playback_close(snd_pcm_t *handle);

// Autodetect helpers for /proc/asound/cards. We've burned hours pointing
// tx_tone at the wrong USB CODEC; these scan kernel-reported card names
// for known fingerprints and return a "plughw:N,0" string ready to hand
// to ALSA. All take a caller-owned buffer; cap should be >= 32. Return 0
// on success, -1 if no matching card was found, -2 on I/O / parse error.

// SignaLink (Texas Instruments / Burr-Brown PCM2901 or PCM2904 USB CODEC).
int audio_find_signalink_device(char *out, size_t cap);

// Radio's native USB CODEC. backend_hint is one of "yaesu" / "icom" or
// NULL to search both. Yaesu radios enumerate as "Yaesu USB Audio";
// IC-9700 enumerates with "IC-9700" or "ICOM" in the kernel description.
int audio_find_radio_device(const char *backend_hint, char *out, size_t cap);

// Lower-level helper: scan /proc/asound/cards and return the card index
// of the first card whose two-line entry contains any of the needle
// substrings (case-sensitive). out_hint, if non-NULL, is filled with
// the matched card's full description (truncated to hint_cap-1).
int audio_find_alsa_card(const char *const *needles, int needle_count,
                         char *out_hint, size_t hint_cap);


#endif // AUDIO_H
