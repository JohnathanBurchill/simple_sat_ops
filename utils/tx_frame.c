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
#include "radio_device_store.h"

#include <alsa/asoundlib.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
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

// Fork arecord against the USB CODEC into a headerless S16_LE raw file
// at the modem sample rate. Headerless so the file lines up byte-for-byte
// with `rtl_fm -M fm` output (same format, same rate) for decoder diffing.
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
            "-t", "raw",
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

// Build an auto-record filename "<tool>_UT=YYYYMMDDTHHMMSS.sss.raw" in the
// current working directory. Timestamp is UTC at the instant of generation.
// Returns 0 on success, -1 on overflow.
static int make_auto_record_path(const char *tool, char *out, size_t out_size)
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) {
        return -1;
    }
    struct tm utc;
    if (gmtime_r(&tv.tv_sec, &utc) == NULL) {
        return -1;
    }
    int n = snprintf(out, out_size,
                     "%s_UT=%04d%02d%02dT%02d%02d%02d.%03ld.raw",
                     tool,
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec,
                     (long)(tv.tv_usec / 1000));
    return (n > 0 && (size_t)n < out_size) ? 0 : -1;
}

static void on_sigint(int sig)
{
    (void)sig;
    // Backend-supplied PTT-off bytes (populated by radio backends during
    // init). Async-signal-safe write() only — no fprintf/malloc here.
    if (g_radio_for_sigint != NULL && g_radio_for_sigint->connected
        && g_radio_for_sigint->ptt_off_raw_len > 0) {
        (void)!write(g_radio_for_sigint->fd,
                     g_radio_for_sigint->ptt_off_raw,
                     g_radio_for_sigint->ptt_off_raw_len);
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
        "FrontierSat UHF carrier, keys PTT via CAT, streams the modulated\n"
        "baseband to an ALSA playback device, then unkeys. The DATA MOD\n"
        "source is set via --mod-input= (default acc = rear DATA jack).\n"
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
        "HMAC / FEC:\n"
        "  --keyfile=<path>            HMAC keyfile (default: $HOME/%s)\n"
        "  --no-hmac                   Skip HMAC (invalid for live ops; test-only)\n"
        "  --reed-solomon              RS(255,223) encode (DEFAULT for uplink,\n"
        "                              matches pycsplink). Adds 32 parity bytes;\n"
        "                              receiver gets up to 16 byte-errors of FEC.\n"
        "  --no-reed-solomon           Disable RS encode (smaller frame, no FEC).\n"
        "\n"
        "Radio (CAT for PTT):\n"
        "  --radio-type=<id>           yaesu-cat (default) | icom-civ | usrp-b210\n"
        "  --radio-device=<path>       CAT tty. Falls back to the saved default\n"
        "                              in ~/.local/share/simple_sat_ops/radio_device,\n"
        "                              then /dev/ttyUSB1.\n"
        "  --radio-serial-speed=<bps>  Serial speed integer. Falls back to the saved\n"
        "                              default, then 4800 (yaesu-cat) or 115200\n"
        "                              (icom-civ).\n"
        "  --freq-hz=<hz>              UHF simplex carrier (default %.0f)\n"
        "\n"
        "Modem:\n"
        "  --bit-rate=<bps>            Baud rate (default 9600). Must divide\n"
        "                              48000 Hz evenly. Try 2400 if the radio's\n"
        "                              FM-DATA passband filter kills the 4.8 kHz\n"
        "                              preamble fundamental at 9600 bps.\n"
        "\n"
        "Audio (ALSA playback):\n"
        "  --signalink-audio           (default) Auto-detect the SignaLink USB and\n"
        "                              use its plughw:N,0. Implies --mod-input=acc.\n"
        "  --radio-audio               Auto-detect the radio's native USB CODEC\n"
        "                              and use its plughw:N,0. Implies\n"
        "                              --mod-input=usb.\n"
        "  --audio-device=<device>     Explicit ALSA device, overriding either\n"
        "                              auto-detect.\n"
        "  --mod-input=<src>           Modulator audio input: usb|acc|mic|mic+acc|\n"
        "                              mic+usb|lan. Inferred from --signalink-audio /\n"
        "                              --radio-audio above; pass explicitly to\n"
        "                              override.\n"
        "  --pre-ms=<ms>               Delay after PTT on before audio starts (200)\n"
        "  --post-ms=<ms>              Delay after audio ends before PTT off (200)\n"
        "  --record=<path>             Capture audio device to <path> (headerless\n"
        "                              S16_LE; default: auto-generated\n"
        "                              tx_frame_UT=YYYYMMDDTHHMMSS.sss.raw in CWD)\n"
        "  --no-record                 Disable auto-recording\n"
        "  --record-warmup-ms=<ms>     Delay between arecord start and PTT on so\n"
        "                              its DMA/CODEC startup transient lands\n"
        "                              before TX audio (default 600)\n"
        "  --moni-level=<0..100>       MONI loopback gain, %%. Backend-specific;\n"
        "                              IC-9700 only on this branch.\n"
        "  --uplink-mod-level=<0..100> USB MOD level, %% — how loud your PCM is on\n"
        "                              the modulator.\n"
        "  --tx-power=<0..100>         RF power, %%. Untouched if omitted (uses the\n"
        "                              radio's current setting).\n"
        "  --allow-high-power          Required for --tx-power above 10%%.\n"
        "  --allow-tx                  Clear the default TX inhibit. Without this,\n"
        "                              PTT is gated and the radio is configured\n"
        "                              but never keyed.\n"
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
    const char *radio_device = NULL;
    const char *audio_device = NULL;
    const char *record_path = NULL;
    char auto_record_path[256];
    int no_record = 0;
    double freq_hz = FRONTIERSAT_CARRIER_HZ;
    int radio_speed_bps = -1;
    int use_hmac = 1;
    int use_rs = 1;  // default ON to match pycsplink uplink
    int dry_run = 0;
    int pre_ms = 200;
    int post_ms = 200;
    int moni_level_pct = -1;  // < 0 = don't touch
    int uplink_mod_level = -1;  // < 0 = don't touch (% 0..100)
    int tx_power_pct = -1;  // < 0 = don't touch (% 0..100)
    int allow_high_power = 0;
    int allow_tx = 0;
    int mod_input_override = -1;  // < 0 = leave whatever radio_uplink_prep picks
    char audio_device_buf[64] = {0};
    int audio_pick = 1;  // 0=explicit, 1=signalink (default), 2=radio
    radio_backend_type_t radio_backend = RADIO_BACKEND_YAESU_CAT;
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
        else if (strcmp(a, "--reed-solomon") == 0)      use_rs = 1;
        else if (strcmp(a, "--no-reed-solomon") == 0)   use_rs = 0;
        else if (starts_with(a, "--src="))     csp_hdr.src   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dst="))     csp_hdr.dst   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dport="))   csp_hdr.dport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--sport="))   csp_hdr.sport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--prio="))    csp_hdr.prio  = (uint8_t)atoi(a + 7);
        else if (starts_with(a, "--radio-device="))     radio_device = a + 15;
        else if (starts_with(a, "--radio-serial-speed=")) radio_speed_bps = atoi(a + 21);
        else if (starts_with(a, "--audio-device="))   { audio_device = a + 15; audio_pick = 0; }
        else if (strcmp(a, "--signalink-audio") == 0) audio_pick = 1;
        else if (strcmp(a, "--radio-audio") == 0)     audio_pick = 2;
        else if (starts_with(a, "--mod-input=")) {
            const char *s = a + 12;
            if      (strcmp(s, "mic")     == 0) mod_input_override = RADIO_DATA_MOD_SRC_MIC;
            else if (strcmp(s, "acc")     == 0) mod_input_override = RADIO_DATA_MOD_SRC_ACC;
            else if (strcmp(s, "mic+acc") == 0) mod_input_override = RADIO_DATA_MOD_SRC_MIC_ACC;
            else if (strcmp(s, "usb")     == 0) mod_input_override = RADIO_DATA_MOD_SRC_USB;
            else if (strcmp(s, "mic+usb") == 0) mod_input_override = RADIO_DATA_MOD_SRC_MIC_USB;
            else if (strcmp(s, "lan")     == 0) mod_input_override = RADIO_DATA_MOD_SRC_LAN;
            else { fprintf(stderr, "--mod-input: unknown '%s'\n", s); return 1; }
        }
        else if (starts_with(a, "--bit-rate=")) {
            int bps = atoi(a + 11);
            if (bps <= 0 || (48000 % bps) != 0) {
                fprintf(stderr, "--bit-rate must be a positive divisor of 48000\n");
                return 1;
            }
            mp.bit_rate = bps;
        }
        else if (starts_with(a, "--freq-hz="))          freq_hz      = atof(a + 10);
        else if (starts_with(a, "--record="))           record_path  = a + 9;
        else if (strcmp(a, "--no-record") == 0)         no_record = 1;
        else if (starts_with(a, "--moni-level=")) {
            moni_level_pct = atoi(a + 13);
            if (moni_level_pct < 0 || moni_level_pct > 100) {
                fprintf(stderr, "--moni-level must be 0..100 (%%)\n");
                return 1;
            }
        }
        else if (starts_with(a, "--uplink-mod-level=")) {
            uplink_mod_level = atoi(a + 19);
            if (uplink_mod_level < 0 || uplink_mod_level > 100) {
                fprintf(stderr, "--uplink-mod-level must be 0..100 (%%)\n");
                return 1;
            }
        }
        else if (starts_with(a, "--tx-power=")) {
            tx_power_pct = atoi(a + 11);
            if (tx_power_pct < 0 || tx_power_pct > 100) {
                fprintf(stderr, "--tx-power must be 0..100 (%%)\n");
                return 1;
            }
        }
        else if (strcmp(a, "--allow-high-power") == 0) allow_high_power = 1;
        else if (strcmp(a, "--allow-tx") == 0) allow_tx = 1;
        else if (starts_with(a, "--radio-type=")) {
            radio_backend_type_t t = radio_backend_type_from_string(a + 13);
            if (t == RADIO_BACKEND__COUNT) {
                fprintf(stderr, "--radio-type: unknown '%s' "
                        "(icom-civ|yaesu-cat|usrp-b210)\n", a + 13);
                return 1;
            }
            radio_backend = t;
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
    if (audio_device == NULL && !dry_run) {
        const char *backend_hint =
            (radio_backend == RADIO_BACKEND_ICOM_CIV) ? "icom" :
            (radio_backend == RADIO_BACKEND_YAESU_CAT) ? "yaesu" : NULL;
        int rc;
        if (audio_pick == 2) {
            rc = audio_find_radio_device(backend_hint, audio_device_buf,
                                         sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "error: --radio-audio: no matching ALSA card "
                        "found (rc=%d)\n", rc);
                return 1;
            }
            if (mod_input_override < 0) mod_input_override = RADIO_DATA_MOD_SRC_USB;
        } else {
            rc = audio_find_signalink_device(audio_device_buf,
                                             sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "error: --signalink-audio: SignaLink not "
                        "found (rc=%d). Pass --audio-device= or "
                        "--radio-audio.\n", rc);
                return 1;
            }
            if (mod_input_override < 0) mod_input_override = RADIO_DATA_MOD_SRC_ACC;
        }
        audio_device = audio_device_buf;
        fprintf(stderr, "tx_frame: auto-detected audio device %s\n", audio_device);
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
    opts.reed_solomon = use_rs;
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

    char effective_device[1024];
    if (radio_device != NULL) {
        snprintf(effective_device, sizeof effective_device, "%s", radio_device);
    } else if (radio_device_store_load(effective_device,
                                       sizeof effective_device) != 0) {
        snprintf(effective_device, sizeof effective_device, "/dev/ttyUSB1");
    }
    int effective_bps = radio_speed_bps;
    if (effective_bps <= 0) {
        if (radio_device_store_load_speed(&effective_bps) != 0) {
            effective_bps = (radio_backend == RADIO_BACKEND_YAESU_CAT)
                                ? 4800 : 115200;
        }
    }
    speed_t radio_speed;
    switch (effective_bps) {
        case 4800:   radio_speed = B4800;   break;
        case 9600:   radio_speed = B9600;   break;
        case 19200:  radio_speed = B19200;  break;
        case 38400:  radio_speed = B38400;  break;
        case 57600:  radio_speed = B57600;  break;
        case 115200: radio_speed = B115200; break;
        default:
            fprintf(stderr, "unsupported baud: %d\n", effective_bps);
            free(samples);
            return 1;
    }

    radio_t radio = {0};
    radio.device_filename = effective_device;
    radio.serial_speed = radio_speed;
    radio.nominal_downlink_frequency = freq_hz;
    radio.sub_park_frequency = RADIO_SUB_PARK_HZ;
    radio.tx_inhibit_cleared = allow_tx;
    if (radio_backend_select(&radio, radio_backend) != RADIO_OK) {
        free(samples);
        return 1;
    }
    int rc = radio_init(&radio);
    if (rc != RADIO_OK) {
        fprintf(stderr, "radio_init(%s) failed (rc=%d)\n", effective_device, rc);
        if (radio.connected) radio_disconnect(&radio);
        free(samples);
        return 1;
    }
    // radio_init leaves Main in plain FM. radio_uplink_prep tunes, sets
    // FM mode, picks the MOD input (default ACC = rear DATA jack), and
    // flips into DATA mode so the PCM we're about to stream actually
    // reaches the modulator. Without DATA mode + correct MOD input, a
    // stale front-panel setting gives an unmodulated carrier — CW on
    // the waterfall instead of GFSK.
    rc = radio_uplink_prep(&radio);
    if (rc != RADIO_OK) {
        fprintf(stderr, "warning: radio_uplink_prep failed (rc=%d); "
                "check MODE/DATA/DATA-MOD on the front panel.\n", rc);
    }
    if (mod_input_override >= 0) {
        rc = radio_set_data_mod_source(&radio, mod_input_override);
        if (rc != RADIO_OK && rc != RADIO_NOT_SUPPORTED) {
            fprintf(stderr, "warning: --mod-input override failed (rc=%d)\n", rc);
        }
    }
    if (uplink_mod_level >= 0) {
        int raw = (int)((uplink_mod_level * 255 + 50) / 100);
        rc = radio_set_usb_mod_level(&radio, raw);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: could not set USB MOD level (rc=%d)\n", rc);
        }
    }
    if (tx_power_pct >= 0) {
        if (tx_power_pct > 10 && !allow_high_power) {
            fprintf(stderr, "error: --tx-power=%d above 10%% safety threshold; "
                    "add --allow-high-power to override.\n", tx_power_pct);
            radio_disconnect(&radio);
            return 1;
        }
        int raw = (int)((tx_power_pct * 255 + 50) / 100);
        rc = radio_set_rf_power(&radio, raw);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: could not set RF power (rc=%d)\n", rc);
        }
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

    // Auto-generate a record path if recording is on and the user didn't
    // pass --record=<path>. Writes tx_frame_UT=YYYYMMDDTHHMMSS.sss.raw in
    // the current working directory so every frame we transmit leaves a
    // searchable breadcrumb.
    if (record_path == NULL && !no_record) {
        if (make_auto_record_path("tx_frame", auto_record_path,
                                  sizeof auto_record_path) != 0) {
            fprintf(stderr, "warning: could not build auto-record filename; "
                    "recording disabled for this run\n");
        } else {
            record_path = auto_record_path;
        }
    }

    // Start the recorder before opening playback so arecord gets a clean
    // device. Opening playback first on the PCM2901 seems to perturb the
    // capture stream (level glitches mid-recording); reversing the order
    // lets capture stabilize on its own then adapt to playback cleanly.
    if (record_path != NULL) {
        fprintf(stderr, "tx_frame: recording %s <- %s (warmup %d ms)\n",
                record_path, audio_device, record_warmup_ms);
        if (start_recorder(audio_device, record_path,
                           (unsigned)mp.samp_rate, record_warmup_ms) != 0) {
            radio_disconnect(&radio);
            free(samples);
            return 1;
        }
    }

    snd_pcm_t *pcm = NULL;
    rc = snd_pcm_open(&pcm, audio_device, SND_PCM_STREAM_PLAYBACK, 0);
    if (rc < 0) {
        fprintf(stderr, "snd_pcm_open(%s, PLAYBACK): %s\n", audio_device, snd_strerror(rc));
        stop_recorder();
        radio_disconnect(&radio);
        free(samples);
        return 1;
    }
    rc = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                            1, (unsigned)mp.samp_rate, 1, 500000);
    if (rc < 0) {
        fprintf(stderr, "snd_pcm_set_params: %s\n", snd_strerror(rc));
        snd_pcm_close(pcm);
        stop_recorder();
        radio_disconnect(&radio);
        free(samples);
        return 1;
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
