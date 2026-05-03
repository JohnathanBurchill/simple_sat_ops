/*

    Simple Satellite Operations  utils/rx_replay.c

    Offline equivalent of rx_live: same sliding-window AX100 decoder,
    same partial-RS rescue, same position-quantised dedup — but the
    samples come from a WAV or headerless S16_LE PCM file rather than
    ALSA. Use it to re-process the .raw / .wav companions rx_live
    leaves on disk after a pass, or to debug the decoder against a
    capture from rtl_fm / a different SDR pipeline.

    Differs from rx_decode (the single-frame offline decoder):
      - Walks the entire file at window/slide cadence, not just the
        first sync match.
      - Emits a UNCORRECTABLE-marked line for long bursts whose
        Golay header decoded but RS(255,223) couldn't correct the
        in-band errors (the descrambled bytes are recovered).
      - Dedups bursts seen across overlapping windows by absolute
        sample position; legitimate retransmissions of the same
        content at different times all emit.

    Copyright (C) 2026  Johnathan K Burchill

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
#include "decode_loop.h"
#include "hmac_keyfile.h"
#include "modem.h"
#include "wav_read.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

// Read a raw S16_LE file into a freshly-malloc'd int16 buffer. Returns
// 0 on success (caller frees *out_samples), -1 on error.
static int read_raw_pcm16(const char *path, int16_t **out_samples, size_t *out_n)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        fprintf(stderr, "rx_replay: open(%s): %s\n", path, strerror(errno));
        return -1;
    }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return -1; }
    long sz = ftell(f);
    if (sz < 0) { fclose(f); return -1; }
    if ((sz & 1) != 0) {
        fprintf(stderr, "rx_replay: %s has odd byte count (%ld); expected S16_LE\n",
                path, sz);
        fclose(f);
        return -1;
    }
    rewind(f);
    size_t n = (size_t)sz / 2u;
    int16_t *buf = (int16_t *)malloc(n * sizeof(int16_t));
    if (buf == NULL) { fclose(f); return -1; }
    if (fread(buf, sizeof(int16_t), n, f) != n) {
        fprintf(stderr, "rx_replay: read(%s): short read\n", path);
        free(buf); fclose(f); return -1;
    }
    fclose(f);
    *out_samples = buf;
    *out_n = n;
    return 0;
}

static void usage(FILE *dest, const char *name)
{
    fprintf(dest,
        "usage: %s <path> [options]\n"
        "\n"
        "Offline AX100 sliding-window decoder. Reads a WAV or headerless\n"
        "S16_LE raw PCM recording (e.g. rx_live's .wav / .raw companion)\n"
        "and applies the same window/slide/decode loop rx_live runs on\n"
        "live ALSA capture.\n"
        "\n"
        "Input:\n"
        "  --raw                    Treat <path> as headerless S16_LE PCM\n"
        "                           (auto-enabled when path ends in '.raw').\n"
        "  --rate=<hz>              Sample rate for --raw (default 48000).\n"
        "  --channels=<n>           Channels for --raw (default 2; ch 0 used).\n"
        "                           Pass --channels=1 for rtl_fm captures.\n"
        "\n"
        "Decoder (same defaults as rx_live):\n"
        "  --bit-rate=<bps>         Default 9600.\n"
        "  --window-s=<seconds>     Decoder window size (default 1.5).\n"
        "  --slide-s=<seconds>      Slide between decode attempts (default 0.5).\n"
        "  --sync-threshold=<0..8>  Max ASM bit errors (default 4).\n"
        "  --hmac                   Enable HMAC verification (off by default;\n"
        "                           AX100 downlink frames don't carry HMAC).\n"
        "  --keyfile=<path>         HMAC keyfile (default $HOME/%s).\n"
        "  --reed-solomon           RS(255,223) decode (DEFAULT).\n"
        "  --no-reed-solomon        Skip RS decode.\n"
        "  --no-dc-block            Skip the modem's DC-block IIR (use on\n"
        "                           radio digital taps with no DC offset).\n"
        "  --csp-crc32              Validate + strip a trailing CSP zlib\n"
        "                           CRC32 (off by default; opt-in only when\n"
        "                           the TX side is known to append one).\n"
        "  --no-partial-rs          Disable the partial-RS rescue. With it\n"
        "                           on (default) a frame whose Golay\n"
        "                           header decoded with 0 errors but whose\n"
        "                           RS(255,223) was uncorrectable still\n"
        "                           emits, marked rs=UNCORRECTABLE.\n"
        "\n"
        "Output:\n"
        "  --log=<path>             Append decode lines to <path> in\n"
        "                           addition to stdout. Re-opened per frame\n"
        "                           so log rotation works.\n"
        "  --quiet                  Skip stdout output (log-only mode).\n"
        "  --help                   Show this help.\n",
        name, HMAC_KEYFILE_DEFAULT_RELPATH);
}

int main(int argc, char **argv)
{
    const char *input_path = NULL;
    const char *log_path = NULL;
    const char *keyfile_path = NULL;
    int raw_mode = 0;
    int raw_mode_explicit = 0;
    int raw_rate = 48000;
    int raw_channels = 2;
    int bit_rate = 9600;
    double window_s = 1.5;
    double slide_s = 0.5;
    int sync_max_ham = 4;
    int use_hmac = 0;
    int use_rs = 1;
    int no_dc_block = 0;
    int csp_crc32 = 0;
    int allow_partial_rs = 1;
    int quiet = 0;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (strcmp(a, "--raw") == 0) { raw_mode = 1; raw_mode_explicit = 1; }
        else if (starts_with(a, "--rate="))      raw_rate = atoi(a + 7);
        else if (starts_with(a, "--channels="))  raw_channels = atoi(a + 11);
        else if (starts_with(a, "--bit-rate="))  bit_rate = atoi(a + 11);
        else if (starts_with(a, "--window-s=")) {
            window_s = atof(a + 11);
            if (window_s < 0.5) window_s = 0.5;
            if (window_s > 30.0) window_s = 30.0;
        } else if (starts_with(a, "--slide-s=")) {
            slide_s = atof(a + 10);
            if (slide_s < 0.05) slide_s = 0.05;
        } else if (starts_with(a, "--sync-threshold=")) {
            sync_max_ham = atoi(a + 17);
            if (sync_max_ham < 0 || sync_max_ham > 8) {
                fprintf(stderr, "rx_replay: --sync-threshold out of range [0,8]\n");
                return 1;
            }
        }
        else if (strcmp(a, "--hmac") == 0)             use_hmac = 1;
        else if (strcmp(a, "--no-hmac") == 0)          use_hmac = 0;
        else if (strcmp(a, "--reed-solomon") == 0)     use_rs = 1;
        else if (strcmp(a, "--no-reed-solomon") == 0)  use_rs = 0;
        else if (strcmp(a, "--no-dc-block") == 0)      no_dc_block = 1;
        else if (strcmp(a, "--csp-crc32") == 0)        csp_crc32 = 1;
        else if (strcmp(a, "--no-csp-crc32") == 0)     csp_crc32 = 0;
        else if (strcmp(a, "--no-partial-rs") == 0)    allow_partial_rs = 0;
        else if (strcmp(a, "--quiet") == 0)            quiet = 1;
        else if (starts_with(a, "--keyfile="))         keyfile_path = a + 10;
        else if (starts_with(a, "--log="))             log_path = a + 6;
        else if (a[0] == '-') {
            fprintf(stderr, "rx_replay: unknown option '%s'\n", a);
            usage(stderr, argv[0]);
            return 1;
        } else if (input_path == NULL) {
            input_path = a;
        } else {
            fprintf(stderr, "rx_replay: unexpected positional '%s'\n", a);
            return 1;
        }
    }
    if (input_path == NULL) {
        fprintf(stderr, "rx_replay: missing <path>\n");
        usage(stderr, argv[0]);
        return 1;
    }
    if (slide_s > window_s) slide_s = window_s;
    if (!raw_mode_explicit) {
        size_t plen = strlen(input_path);
        if (plen >= 4 && strcmp(input_path + plen - 4, ".raw") == 0) raw_mode = 1;
    }
    if (raw_channels < 1 || raw_channels > 8) {
        fprintf(stderr, "rx_replay: --channels out of range [1,8]\n");
        return 1;
    }

    // Load samples (interleaved if multi-channel) and reduce to ch 0.
    int16_t *samples = NULL;
    size_t n_samples = 0;
    int samp_rate = 0;
    int channels = 1;
    if (raw_mode) {
        if (read_raw_pcm16(input_path, &samples, &n_samples) != 0) return 1;
        samp_rate = raw_rate;
        channels = raw_channels;
    } else {
        if (wav_read_pcm16(input_path, &samples, &n_samples,
                           &samp_rate, &channels) != 0) {
            return 1;
        }
    }
    size_t n_frames = (channels > 1) ? (n_samples / (size_t)channels) : n_samples;
    if (channels > 1) {
        for (size_t f = 0; f < n_frames; ++f) {
            samples[f] = samples[f * (size_t)channels];
        }
    }

    if (samp_rate <= 0 || bit_rate <= 0 || (samp_rate % bit_rate) != 0) {
        fprintf(stderr, "rx_replay: samp_rate (%d) must be a multiple of "
                "bit_rate (%d)\n", samp_rate, bit_rate);
        free(samples);
        return 1;
    }
    int sps = samp_rate / bit_rate;

    // HMAC key.
    uint8_t hmac_key[128];
    ssize_t hmac_key_len = 0;
    if (use_hmac) {
        char default_path[512];
        const char *path = keyfile_path;
        if (path == NULL) {
            if (hmac_keyfile_default_path(default_path, sizeof default_path) != 0) {
                fprintf(stderr, "rx_replay: HOME unset; pass --keyfile=<path>\n");
                free(samples);
                return 1;
            }
            path = default_path;
        }
        hmac_key_len = hmac_keyfile_load(path, hmac_key, sizeof hmac_key);
        if (hmac_key_len < 0) { free(samples); return 1; }
    }

    modem_params_t mp;
    modem_params_defaults(&mp);
    mp.samp_rate = samp_rate;
    mp.bit_rate = bit_rate;
    mp.rx_disable_dc_block = no_dc_block;

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (use_hmac) {
        opts.hmac_key = hmac_key;
        opts.hmac_key_len = (size_t)hmac_key_len;
    }
    opts.reed_solomon = use_rs;

    size_t window_samples = (size_t)(window_s * (double)samp_rate);
    size_t slide_samples  = (size_t)(slide_s  * (double)samp_rate);
    if (slide_samples == 0) slide_samples = 1;
    if (slide_samples > window_samples) slide_samples = window_samples;
    size_t bits_cap = window_samples / (size_t)sps + 16;
    size_t bytes_cap = bits_cap / 8 + 16;
    uint8_t *bits_scratch  = (uint8_t *)malloc(bits_cap);
    uint8_t *bytes_scratch = (uint8_t *)malloc(bytes_cap);
    uint8_t packet[4100];
    if (bits_scratch == NULL || bytes_scratch == NULL) {
        free(bits_scratch); free(bytes_scratch); free(samples);
        return 1;
    }

    // Position-quantised dedup ring (mirrors rx_live).
    enum { DEDUP_RING_SZ = 64 };
    enum { DEDUP_QUANT_SAMPLES = 4800 };
    uint64_t recent_pos_quant[DEDUP_RING_SZ] = {0};
    int recent_idx = 0;
    int recent_count = 0;

    fprintf(stderr,
            "rx_replay: %s  rate=%d Hz  channels=%d  bit_rate=%d  "
            "window=%.2fs  slide=%.2fs  sync_thr=%d  rs=%s  hmac=%s  "
            "partial=%s  duration=%.3fs\n",
            input_path, samp_rate, channels, bit_rate,
            window_s, slide_s, sync_max_ham,
            use_rs ? "on" : "off",
            use_hmac ? "on" : "off",
            allow_partial_rs ? "on" : "off",
            (double)n_frames / (double)samp_rate);

    int n_emitted = 0;
    for (size_t window_start = 0;
         window_start + window_samples <= n_frames;
         window_start += slide_samples)
    {
        const int16_t *win = samples + window_start;
        size_t inner_min_offset = 0;
        for (;;) {
            ssize_t plen = -1;
            int golay_errs = 0, hmac_ok = -1;
            int rs_errs = -1, used_golay_len = -1;
            size_t sync_off_local = 0;
            if (!try_decode_window(win, window_samples, &mp, &opts,
                                   sync_max_ham, use_hmac,
                                   allow_partial_rs,
                                   inner_min_offset,
                                   bits_scratch, bits_cap,
                                   bytes_scratch, bytes_cap,
                                   packet, sizeof packet,
                                   &plen, &golay_errs, &hmac_ok,
                                   &rs_errs, &used_golay_len,
                                   &sync_off_local)) break;
            inner_min_offset = sync_off_local + 1;
            if (plen < 4 || (size_t)plen > sizeof packet) continue;

            // CSP zlib CRC32 trailer (opt-in via --csp-crc32; mirrors
            // rx_live so the same bytes produce the same output).
            int crc_status = -1;
            uint32_t crc_computed = 0, crc_le = 0, crc_be = 0;
            if (!use_hmac && csp_crc32 && plen >= 8) {
                crc_computed = csp_crc32_zlib(packet, (size_t)(plen - 4));
                crc_le = (uint32_t)packet[plen - 4]
                       | ((uint32_t)packet[plen - 3] << 8)
                       | ((uint32_t)packet[plen - 2] << 16)
                       | ((uint32_t)packet[plen - 1] << 24);
                crc_be = ((uint32_t)packet[plen - 4] << 24)
                       | ((uint32_t)packet[plen - 3] << 16)
                       | ((uint32_t)packet[plen - 2] <<  8)
                       |  (uint32_t)packet[plen - 1];
                if (crc_computed == crc_le || crc_computed == crc_be) {
                    crc_status = 1;
                    plen -= 4;
                } else {
                    crc_status = 0;
                }
            }

            uint64_t asm_abs_sample = (uint64_t)window_start
                + (uint64_t)sync_off_local * (uint64_t)sps
                + (uint64_t)(sps / 2);
            uint64_t pos_quant = asm_abs_sample / DEDUP_QUANT_SAMPLES;
            int seen = 0;
            int ring_n = recent_count < DEDUP_RING_SZ
                ? recent_count : DEDUP_RING_SZ;
            for (int r = 0; r < ring_n; r++) {
                if (recent_pos_quant[r] == pos_quant) { seen = 1; break; }
            }
            if (seen) continue;
            recent_pos_quant[recent_idx] = pos_quant;
            recent_idx = (recent_idx + 1) % DEDUP_RING_SZ;
            if (recent_count < DEDUP_RING_SZ) recent_count++;

            char ts[32];
            double t_sec = (double)asm_abs_sample / (double)samp_rate;
            snprintf(ts, sizeof ts, "t=%.3fs", t_sec);
            emit_frame(log_path, quiet, ts,
                       packet, (size_t)plen,
                       golay_errs, hmac_ok, use_hmac,
                       rs_errs, used_golay_len,
                       crc_status, crc_computed, crc_le, crc_be);
            ++n_emitted;
        }
    }

    fprintf(stderr, "rx_replay: %d frame(s) emitted.\n", n_emitted);
    free(bits_scratch);
    free(bytes_scratch);
    free(samples);
    return 0;
}
