/*

    Simple Satellite Operations  utils/tx_frame.c

    Single-shot AX100 frame transmitter. Builds a CSP v1 packet, frames
    it for the AX100 (HMAC + scrambler + Golay length + sync word + pre/
    postamble), modulates to baseband PCM, then:

        1. CI-V PTT on
        2. optional pre-audio settle (guard-time)
        3. stream the baseband PCM to an ALSA playback device
        4. optional post-audio settle
        5. CI-V PTT off

    Intended for bench / commissioning TX tests. No pass-event scheduling,
    no current_telecommand.txt watcher, no looping. One frame per run.

    Copyright (C) 2025  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "ax100.h"
#include "csp.h"
#include "hmac_keyfile.h"
#include "modem.h"
#include "radio.h"

#include <alsa/asoundlib.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

// File-scope pointer so the signal handler can reach the open radio.
// Only touched via async-signal-safe write() — do not deref fields that
// could change; only read fd/connected which are stable post-connect.
static radio_t *g_radio_for_sigint = NULL;
static pid_t g_record_pid = 0;

static void stop_recorder(void)
{
    if (g_record_pid <= 0) return;
    kill(g_record_pid, SIGINT);
    int status = 0;
    waitpid(g_record_pid, &status, 0);
    g_record_pid = 0;
}

// Fork arecord against the USB CODEC into a WAV file. Format matches
// the tx_frame baseband (mono S16_LE at the modem sample rate) so the
// captured file lines up with uplink_test --out= for decoder diffing.
// warmup_ms: sleep after fork so arecord's DMA/CODEC startup transient
// (first ~100 ms of noise + silence) is past before PTT fires.
static int start_recorder(const char *device, const char *out_path,
                          unsigned int rate_hz, int warmup_ms)
{
    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "error: fork() for arecord failed: %s\n", strerror(errno));
        return -1;
    }
    if (pid == 0) {
        char rate_str[16];
        snprintf(rate_str, sizeof rate_str, "%u", rate_hz);
        char *argv_rec[] = {
            "arecord",
            "-q",
            "-D", (char *)device,
            "-f", "S16_LE",
            "-r", rate_str,
            "-c", "1",
            "-t", "wav",
            (char *)out_path,
            NULL,
        };
        execvp("arecord", argv_rec);
        fprintf(stderr, "error: execvp(arecord) failed: %s\n", strerror(errno));
        _exit(127);
    }
    g_record_pid = pid;
    if (warmup_ms > 0) {
        usleep((useconds_t)warmup_ms * 1000);
    }
    return 0;
}

static void on_sigint(int sig)
{
    (void)sig;
    // CI-V PTT off: FE FE A2 E0 1C 00 00 FD (transmit off)
    static const uint8_t ptt_off_cmd[] = {
        0xFE, 0xFE, 0xA2, 0xE0, 0x1C, 0x00, 0x00, 0xFD,
    };
    if (g_radio_for_sigint != NULL && g_radio_for_sigint->connected) {
        (void)!write(g_radio_for_sigint->fd, ptt_off_cmd, sizeof(ptt_off_cmd));
    }
    // Clean up the recorder child before we bail so its WAV is flushed.
    stop_recorder();
    _exit(130);  // 128 + SIGINT
}

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static ssize_t parse_payload_hex(const char *hex, uint8_t *out, size_t out_cap)
{
    size_t n = strlen(hex);
    if ((n & 1u) != 0) {
        fprintf(stderr, "payload hex: odd number of chars (%zu)\n", n);
        return -1;
    }
    size_t n_bytes = n / 2;
    if (n_bytes > out_cap) {
        fprintf(stderr, "payload hex: %zu bytes exceeds cap %zu\n", n_bytes, out_cap);
        return -1;
    }
    for (size_t i = 0; i < n_bytes; ++i) {
        int hi = hex_digit(hex[2 * i]);
        int lo = hex_digit(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) {
            fprintf(stderr, "payload hex: bad char at offset %zu\n", 2 * i);
            return -1;
        }
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return (ssize_t)n_bytes;
}

static void usage(FILE *out, const char *argv0)
{
    fprintf(out,
        "usage: %s (--payload-hex=<HEX> | --payload-ascii=<STR>) [options]\n"
        "\n"
        "Builds a CSP+AX100 frame, initializes the radio to FM-DATA on the\n"
        "FrontierSat UHF carrier, keys PTT via CI-V, streams the modulated\n"
        "baseband to an ALSA playback device, then unkeys. The DATA MOD\n"
        "source is NOT touched — set it on the front panel to match the\n"
        "audio cabling (USB for the 9700's native USB CODEC).\n"
        "\n"
        "Required (exactly one):\n"
        "  --payload-hex=<HEX>         Payload as hex\n"
        "  --payload-ascii=<STR>       Payload as literal ASCII bytes\n"
        "\n"
        "CSP header (defaults shown):\n"
        "  --src=<0..31>               source address (1)\n"
        "  --dst=<0..31>               destination address (2)\n"
        "  --dport=<0..63>             destination port (3)\n"
        "  --sport=<0..63>             source port (4)\n"
        "  --prio=<0..3>               priority, 2=norm (2)\n"
        "\n"
        "HMAC:\n"
        "  --keyfile=<path>            HMAC keyfile (default: $HOME/%s)\n"
        "  --no-hmac                   Skip HMAC (invalid for live ops; test-only)\n"
        "\n"
        "Radio (CI-V for PTT):\n"
        "  --radio-device=<path>       CI-V tty (default /dev/ttyUSB1)\n"
        "  --radio-serial-speed=<bps>  Serial speed (default 115200)\n"
        "  --freq-hz=<hz>              UHF simplex carrier (default %.0f)\n"
        "\n"
        "Audio (ALSA playback):\n"
        "  --audio-device=<device>     ALSA device (default plughw:4,0)\n"
        "  --pre-ms=<ms>               Delay after PTT on before audio starts (200)\n"
        "  --post-ms=<ms>              Delay after audio ends before PTT off (200)\n"
        "  --record=<wav>              Capture the USB CODEC to <wav> during TX\n"
        "                              (needs radio MONI on for useful content)\n"
        "  --record-warmup-ms=<ms>     Delay between arecord start and PTT on so\n"
        "                              its DMA/CODEC startup transient lands\n"
        "                              before TX audio (default 600)\n"
        "  --moni-level=<0..100>       MONI loopback gain, %% (CI-V 14 07; MONI\n"
        "                              itself stays a front-panel toggle)\n"
        "\n"
        "Safety / dry-run:\n"
        "  --dry-run                   Build the frame, print size, do not TX\n"
        "  --help                      This message\n",
        argv0, HMAC_KEYFILE_DEFAULT_RELPATH, FRONTIERSAT_CARRIER_HZ);
}

int main(int argc, char **argv)
{
    const char *payload_hex = NULL;
    const char *payload_ascii = NULL;
    const char *keyfile_path = NULL;
    const char *radio_device = "/dev/ttyUSB1";
    const char *audio_device = "plughw:4,0";
    const char *record_path = NULL;
    double freq_hz = FRONTIERSAT_CARRIER_HZ;
    speed_t radio_speed = B115200;
    int use_hmac = 1;
    int dry_run = 0;
    int pre_ms = 200;
    int post_ms = 200;
    int moni_level_pct = -1;  // < 0 = don't touch
    int record_warmup_ms = 600;

    csp_v1_header_t csp_hdr = {
        .prio = CSP_PRIO_NORM,
        .src = 1, .dst = 2,
        .dport = 3, .sport = 4,
        .flags = 0,
    };
    modem_params_t mp;
    modem_params_defaults(&mp);

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (starts_with(a, "--payload-hex="))      payload_hex   = a + 14;
        else if (starts_with(a, "--payload-ascii="))    payload_ascii = a + 16;
        else if (starts_with(a, "--keyfile="))          keyfile_path  = a + 10;
        else if (strcmp(a, "--no-hmac") == 0)           use_hmac = 0;
        else if (starts_with(a, "--src="))     csp_hdr.src   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dst="))     csp_hdr.dst   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dport="))   csp_hdr.dport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--sport="))   csp_hdr.sport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--prio="))    csp_hdr.prio  = (uint8_t)atoi(a + 7);
        else if (starts_with(a, "--radio-device="))     radio_device = a + 15;
        else if (starts_with(a, "--radio-serial-speed=")) {
            int bps = atoi(a + 21);
            if      (bps == 9600)   radio_speed = B9600;
            else if (bps == 19200)  radio_speed = B19200;
            else if (bps == 38400)  radio_speed = B38400;
            else if (bps == 57600)  radio_speed = B57600;
            else if (bps == 115200) radio_speed = B115200;
            else { fprintf(stderr, "unsupported baud: %d\n", bps); return 1; }
        }
        else if (starts_with(a, "--audio-device="))     audio_device = a + 15;
        else if (starts_with(a, "--freq-hz="))          freq_hz      = atof(a + 10);
        else if (starts_with(a, "--record="))           record_path  = a + 9;
        else if (starts_with(a, "--moni-level=")) {
            moni_level_pct = atoi(a + 13);
            if (moni_level_pct < 0 || moni_level_pct > 100) {
                fprintf(stderr, "--moni-level must be 0..100 (%%)\n");
                return 1;
            }
        }
        else if (starts_with(a, "--record-warmup-ms=")) {
            record_warmup_ms = atoi(a + 19);
            if (record_warmup_ms < 0) record_warmup_ms = 0;
        }
        else if (starts_with(a, "--pre-ms="))           pre_ms       = atoi(a + 9);
        else if (starts_with(a, "--post-ms="))          post_ms      = atoi(a + 10);
        else if (strcmp(a, "--dry-run") == 0)           dry_run = 1;
        else {
            fprintf(stderr, "unknown option: %s\n", a);
            usage(stderr, argv[0]);
            return 1;
        }
    }

    if ((payload_hex == NULL) == (payload_ascii == NULL)) {
        fprintf(stderr, "pass exactly one of --payload-hex or --payload-ascii\n");
        usage(stderr, argv[0]);
        return 1;
    }

    uint8_t payload[2048];
    ssize_t payload_len;
    if (payload_hex != NULL) {
        payload_len = parse_payload_hex(payload_hex, payload, sizeof(payload));
        if (payload_len < 0) return 1;
    } else {
        size_t n = strlen(payload_ascii);
        if (n > sizeof(payload)) {
            fprintf(stderr, "payload ascii: %zu bytes exceeds cap %zu\n", n, sizeof(payload));
            return 1;
        }
        memcpy(payload, payload_ascii, n);
        payload_len = (ssize_t)n;
    }

    uint8_t hmac_key[128];
    ssize_t hmac_key_len = 0;
    if (use_hmac) {
        char default_path[512];
        const char *path = keyfile_path;
        if (path == NULL) {
            if (hmac_keyfile_default_path(default_path, sizeof(default_path)) != 0) {
                fprintf(stderr, "HOME is unset; pass --keyfile=<path>\n");
                return 1;
            }
            path = default_path;
        }
        hmac_key_len = hmac_keyfile_load(path, hmac_key, sizeof(hmac_key));
        if (hmac_key_len < 0) return 1;
    }

    uint8_t csp_packet[2100];
    ssize_t csp_len = csp_v1_encode(&csp_hdr, payload, (size_t)payload_len,
                                    csp_packet, sizeof(csp_packet));
    if (csp_len < 0) { fprintf(stderr, "csp_v1_encode failed\n"); return 1; }

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (use_hmac) {
        opts.hmac_key = hmac_key;
        opts.hmac_key_len = (size_t)hmac_key_len;
    }
    uint8_t frame[4200];
    ssize_t frame_len = ax100_frame(csp_packet, (size_t)csp_len, &opts, frame, sizeof(frame));
    if (frame_len < 0) { fprintf(stderr, "ax100_frame failed\n"); return 1; }

    int sps = mp.samp_rate / mp.bit_rate;
    size_t n_samples = (size_t)frame_len * 8u * (size_t)sps;
    int16_t *samples = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if (samples == NULL) {
        fprintf(stderr, "out of memory for %zu samples\n", n_samples);
        return 1;
    }
    ssize_t n = modem_bytes_to_pcm16(frame, (size_t)frame_len, &mp, samples, n_samples);
    if (n < 0) { fprintf(stderr, "modem_bytes_to_pcm16 failed\n"); free(samples); return 1; }

    double tx_seconds = (double)n_samples / (double)mp.samp_rate;
    fprintf(stderr, "tx_frame: frame=%zd bytes, %zu samples, %.3f s on-air\n",
            frame_len, n_samples, tx_seconds);

    if (dry_run) {
        fprintf(stderr, "tx_frame: --dry-run, not keying PTT\n");
        free(samples);
        return 0;
    }

    snd_pcm_t *pcm = NULL;
    int rc = snd_pcm_open(&pcm, audio_device, SND_PCM_STREAM_PLAYBACK, 0);
    if (rc < 0) {
        fprintf(stderr, "snd_pcm_open(%s, PLAYBACK): %s\n", audio_device, snd_strerror(rc));
        free(samples);
        return 1;
    }
    rc = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                            1, (unsigned)mp.samp_rate, 1, 500000);
    if (rc < 0) {
        fprintf(stderr, "snd_pcm_set_params: %s\n", snd_strerror(rc));
        snd_pcm_close(pcm);
        free(samples);
        return 1;
    }

    radio_t radio = {0};
    radio.device_filename = (char *)radio_device;
    radio.serial_speed = radio_speed;
    radio.nominal_downlink_frequency = freq_hz;
    radio.sub_park_frequency = RADIO_SUB_PARK_HZ;
    rc = radio_init(&radio);
    if (rc != RADIO_OK) {
        fprintf(stderr, "radio_init(%s) failed (rc=%d)\n", radio_device, rc);
        if (radio.connected) radio_disconnect(&radio);
        snd_pcm_close(pcm);
        free(samples);
        return 1;
    }
    // radio_init leaves Main in plain FM (CI-V 0x06 clears the DATA flag).
    // Restore DATA so the modulator uses the flat audio path AX100 needs.
    rc = radio_set_data_mode(&radio, 1, RADIO_FILTER_FIL1);
    if (rc != RADIO_OK) {
        fprintf(stderr, "warning: could not re-enable DATA mode (rc=%d); "
                "radio will transmit in voice FM.\n", rc);
    }
    if (moni_level_pct >= 0) {
        int raw = (int)((moni_level_pct * 255 + 50) / 100);
        rc = radio_set_moni_level(&radio, raw);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: could not set MONI level (rc=%d)\n", rc);
        }
    }

    // Arm a SIGINT handler that writes PTT-off directly to the radio fd
    // so Ctrl-C during TX doesn't leave the radio keyed. Handler calls
    // _exit() — any ALSA cleanup is left to the kernel.
    g_radio_for_sigint = &radio;
    struct sigaction sa = {0};
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;  // no SA_RESTART: want ALSA blocking calls to return EINTR
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (record_path != NULL) {
        fprintf(stderr, "tx_frame: recording %s <- %s (warmup %d ms)\n",
                record_path, audio_device, record_warmup_ms);
        if (start_recorder(audio_device, record_path,
                           (unsigned)mp.samp_rate, record_warmup_ms) != 0) {
            snd_pcm_close(pcm);
            radio_disconnect(&radio);
            free(samples);
            return 1;
        }
    }

    fprintf(stderr, "tx_frame: PTT on\n");
    rc = radio_ptt(&radio, 1);
    if (rc != RADIO_OK) {
        fprintf(stderr, "radio_ptt(1) failed: %d\n", rc);
        stop_recorder();
        snd_pcm_close(pcm);
        radio_disconnect(&radio);
        free(samples);
        return 1;
    }
    if (pre_ms > 0) usleep((useconds_t)pre_ms * 1000);

    size_t written = 0;
    while (written < n_samples) {
        snd_pcm_sframes_t wrote = snd_pcm_writei(pcm, samples + written, n_samples - written);
        if (wrote < 0) {
            wrote = snd_pcm_recover(pcm, (int)wrote, 0);
        }
        if (wrote < 0) {
            fprintf(stderr, "snd_pcm_writei: %s\n", snd_strerror((int)wrote));
            break;
        }
        written += (size_t)wrote;
    }
    snd_pcm_drain(pcm);

    if (post_ms > 0) usleep((useconds_t)post_ms * 1000);

    fprintf(stderr, "tx_frame: PTT off\n");
    radio_ptt(&radio, 0);

    stop_recorder();
    snd_pcm_close(pcm);
    radio_disconnect(&radio);
    free(samples);
    return 0;
}
