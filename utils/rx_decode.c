/*

    Simple Satellite Operations  utils/rx_decode.c

    Offline AX100 frame decoder. Takes a WAV (or headerless S16_LE raw PCM
    from rtl_fm) captured over the air or produced synthetically by
    uplink_test, and recovers the CSP packet that was transmitted.

    Reverse of tx_frame:

        WAV/RAW samples
          -> modem_pcm16_to_bits  (DC-block, phase search, ASM detect)
          -> modem_bits_to_bytes  (MSB-first byte pack, ASM-aligned)
          -> ax100_unframe        (Golay24 len, descramble, HMAC verify)
          -> csp_v1_decode        (parse the 4-byte CSP header)
          -> print header + payload (hex + ASCII)

    The decoder has no RF-chain assumptions; the primary end-to-end test
    is uplink_test --out=<wav> | rx_decode <wav>, which should round-trip
    bit-for-bit because the WAV is literally what the TX modulator emits.

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
#include "hmac_keyfile.h"
#include "modem.h"
#include "wav_read.h"

#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static void print_hex_ascii(const uint8_t *buf, size_t len)
{
    // hex block
    fprintf(stdout, "  hex:   ");
    for (size_t i = 0; i < len; ++i) {
        fprintf(stdout, "%02x", buf[i]);
    }
    fputc('\n', stdout);
    // ascii block (. for non-printable)
    fprintf(stdout, "  ascii: ");
    for (size_t i = 0; i < len; ++i) {
        char c = (buf[i] >= 0x20 && buf[i] < 0x7F) ? (char)buf[i] : '.';
        fputc(c, stdout);
    }
    fputc('\n', stdout);
}

// Read a raw S16_LE file into a freshly-malloc'd int16 buffer. Returns 0
// on success (caller frees *out_samples), -1 on error.
static int read_raw_pcm16(const char *path, int16_t **out_samples, size_t *out_n)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        fprintf(stderr, "rx_decode: open(%s): %s\n", path, strerror(errno));
        return -1;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return -1;
    }
    long sz = ftell(f);
    if (sz < 0) {
        fclose(f);
        return -1;
    }
    if ((sz & 1) != 0) {
        fprintf(stderr, "rx_decode: %s has odd byte count (%ld); expected S16_LE\n",
                path, sz);
        fclose(f);
        return -1;
    }
    rewind(f);
    size_t n = (size_t)sz / 2u;
    int16_t *buf = (int16_t *)malloc(n * sizeof(int16_t));
    if (buf == NULL) {
        fclose(f);
        return -1;
    }
    if (fread(buf, sizeof(int16_t), n, f) != n) {
        fprintf(stderr, "rx_decode: read(%s): short read\n", path);
        free(buf);
        fclose(f);
        return -1;
    }
    fclose(f);
    *out_samples = buf;
    *out_n = n;
    return 0;
}

// Take interleaved stereo/multi-ch samples and in-place reduce to ch 0.
// Returns the new sample count.
static size_t extract_channel_zero(int16_t *samples, size_t n, int channels)
{
    if (channels <= 1) return n;
    size_t frames = n / (size_t)channels;
    for (size_t f = 0; f < frames; ++f) {
        samples[f] = samples[f * (size_t)channels];
    }
    return frames;
}

static void usage(FILE *out, const char *argv0)
{
    fprintf(out,
        "usage: %s <path> [options]\n"
        "\n"
        "Decodes an AX100 frame from a WAV or headerless S16_LE raw PCM\n"
        "recording and prints the CSP header and payload. Primary end-to-end\n"
        "test: `uplink_test --payload-ascii=HELLO --out=/tmp/h.wav && \\\n"
        "                  %s /tmp/h.wav`.\n"
        "\n"
        "Input:\n"
        "  --raw                      Treat <path> as headerless S16_LE PCM\n"
        "                             (output of `rtl_fm -M fm -s 48000`)\n"
        "  --rate=<hz>                Sample rate for --raw (default 48000)\n"
        "  --channels=<n>             Channels for --raw (default 1; ch 0 used)\n"
        "\n"
        "Framing / HMAC:\n"
        "  --keyfile=<path>           HMAC keyfile (default $HOME/%s)\n"
        "  --no-hmac                  Skip HMAC verification (matches\n"
        "                             `tx_frame --no-hmac`)\n"
        "\n"
        "Modem:\n"
        "  --bit-rate=<bps>           Default 9600\n"
        "  --invert                   Try inverted polarity first (both are\n"
        "                             always tried; this just reorders)\n"
        "  --sync-threshold=<0..8>    Max bit errors in the 32-bit ASM match\n"
        "                             (default 0 = strict)\n"
        "\n"
        "Output:\n"
        "  -v                         Verbose: pipeline-stage diagnostics\n"
        "  --hex-only                 Print only the decoded payload as hex\n"
        "                             (for scripting)\n"
        "  --help                     This message\n",
        argv0, argv0, HMAC_KEYFILE_DEFAULT_RELPATH);
}

int main(int argc, char **argv)
{
    const char *input_path = NULL;
    const char *keyfile_path = NULL;
    int raw_mode = 0;
    int raw_rate = 48000;
    int raw_channels = 1;
    int bit_rate = 9600;
    int invert = 0;
    int sync_max_ham = 0;
    int use_hmac = 1;
    int verbose = 0;
    int hex_only = 0;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else if (strcmp(a, "--raw") == 0) {
            raw_mode = 1;
        } else if (starts_with(a, "--rate=")) {
            raw_rate = atoi(a + 7);
        } else if (starts_with(a, "--channels=")) {
            raw_channels = atoi(a + 11);
        } else if (starts_with(a, "--keyfile=")) {
            keyfile_path = a + 10;
        } else if (strcmp(a, "--no-hmac") == 0) {
            use_hmac = 0;
        } else if (starts_with(a, "--bit-rate=")) {
            bit_rate = atoi(a + 11);
        } else if (strcmp(a, "--invert") == 0) {
            invert = 1;
        } else if (starts_with(a, "--sync-threshold=")) {
            sync_max_ham = atoi(a + 17);
            if (sync_max_ham < 0 || sync_max_ham > 8) {
                fprintf(stderr, "rx_decode: --sync-threshold out of range [0,8]\n");
                return 1;
            }
        } else if (strcmp(a, "-v") == 0 || strcmp(a, "--verbose") == 0) {
            verbose = 1;
        } else if (strcmp(a, "--hex-only") == 0) {
            hex_only = 1;
        } else if (a[0] == '-') {
            fprintf(stderr, "rx_decode: unknown option '%s'\n", a);
            usage(stderr, argv[0]);
            return 1;
        } else if (input_path == NULL) {
            input_path = a;
        } else {
            fprintf(stderr, "rx_decode: unexpected positional '%s'\n", a);
            usage(stderr, argv[0]);
            return 1;
        }
    }

    if (input_path == NULL) {
        fprintf(stderr, "rx_decode: missing <path>\n");
        usage(stderr, argv[0]);
        return 1;
    }
    if (raw_channels < 1 || raw_channels > 8) {
        fprintf(stderr, "rx_decode: --channels out of range [1,8]\n");
        return 1;
    }

    // Load samples.
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
    n_samples = extract_channel_zero(samples, n_samples, channels);

    if (verbose) {
        fprintf(stderr, "rx_decode: loaded %zu samples @ %d Hz (%d ch -> ch0)\n",
                n_samples, samp_rate, channels);
    }

    modem_params_t mp;
    modem_params_defaults(&mp);
    mp.samp_rate = samp_rate;
    mp.bit_rate = bit_rate;
    if (mp.samp_rate <= 0 || mp.bit_rate <= 0 ||
        mp.samp_rate % mp.bit_rate != 0) {
        fprintf(stderr, "rx_decode: samp_rate (%d) must be a multiple of bit_rate (%d)\n",
                mp.samp_rate, mp.bit_rate);
        free(samples);
        return 1;
    }

    size_t max_bits = n_samples / (size_t)(mp.samp_rate / mp.bit_rate);
    uint8_t *bits = (uint8_t *)malloc(max_bits);
    if (bits == NULL) {
        fprintf(stderr, "rx_decode: out of memory for %zu bits\n", max_bits);
        free(samples);
        return 1;
    }

    size_t n_bits = 0;
    size_t sync_off = 0;
    int polarity_used = -1;
    int rc = modem_pcm16_to_bits(samples, n_samples, &mp,
                                 invert, sync_max_ham,
                                 bits, &n_bits, &sync_off, &polarity_used);
    if (rc != 0) {
        fprintf(stderr, "rx_decode: no AX100 sync word (0x930B51DE) found "
                "(tried both polarities, sync-threshold=%d)\n",
                sync_max_ham);
        if (verbose) {
            // Dump the first 128 bits sliced at each phase so the operator
            // can eyeball whether a 0xAA (10101010) preamble pattern is
            // anywhere in the input. If yes, sync-threshold is too strict;
            // if no, the signal isn't there at all.
            int sps = mp.samp_rate / mp.bit_rate;
            fprintf(stderr, "rx_decode: first 128 bits at each phase (for debug):\n");
            for (int phase = 0; phase < sps; ++phase) {
                fprintf(stderr, "  phase=%d  ", phase);
                size_t mid = (size_t)phase + (size_t)sps / 2u;
                size_t printed = 0;
                for (size_t i = mid; i < n_samples && printed < 128; i += (size_t)sps) {
                    fputc((samples[i] > 0) ? '1' : '0', stderr);
                    ++printed;
                }
                fputc('\n', stderr);
            }
        }
        free(samples);
        free(bits);
        return 1;
    }
    free(samples);
    if (verbose) {
        fprintf(stderr, "rx_decode: ASM at bit offset %zu, %zu bits after ASM, polarity=%s\n",
                sync_off, n_bits, polarity_used ? "inverted" : "normal");
    }

    // Pack bits back into bytes for the framer.
    size_t max_bytes = (n_bits + 7) / 8;
    uint8_t *bytes = (uint8_t *)malloc(max_bytes);
    if (bytes == NULL) {
        fprintf(stderr, "rx_decode: out of memory for %zu bytes\n", max_bytes);
        free(bits);
        return 1;
    }
    size_t n_bytes = modem_bits_to_bytes(bits, n_bits, bytes);
    free(bits);

    // Load HMAC key if requested.
    uint8_t hmac_key[128];
    ssize_t hmac_key_len = 0;
    if (use_hmac) {
        char default_path[512];
        const char *path = keyfile_path;
        if (path == NULL) {
            if (hmac_keyfile_default_path(default_path, sizeof(default_path)) != 0) {
                fprintf(stderr, "rx_decode: HOME is unset; pass --keyfile=<path>\n");
                free(bytes);
                return 1;
            }
            path = default_path;
        }
        hmac_key_len = hmac_keyfile_load(path, hmac_key, sizeof(hmac_key));
        if (hmac_key_len < 0) {
            free(bytes);
            return 1;
        }
    }

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (use_hmac) {
        opts.hmac_key = hmac_key;
        opts.hmac_key_len = (size_t)hmac_key_len;
    }

    uint8_t packet[4100];
    int golay_errs = 0, hmac_ok = -1;
    ssize_t packet_len = ax100_unframe(bytes, n_bytes, &opts,
                                       packet, sizeof(packet),
                                       &golay_errs, &hmac_ok);
    free(bytes);
    if (packet_len < 0) {
        fprintf(stderr, "rx_decode: ax100_unframe failed (golay_errs=%d)\n",
                golay_errs);
        return 1;
    }

    if (verbose) {
        fprintf(stderr, "rx_decode: inner packet %zd bytes, golay errors=%d, hmac=%s\n",
                packet_len, golay_errs,
                hmac_ok == 1 ? "ok" : hmac_ok == 0 ? "MISMATCH" : "(not checked)");
    }

    if (packet_len < 4) {
        fprintf(stderr, "rx_decode: packet too short to contain CSP header (%zd)\n",
                packet_len);
        return 1;
    }

    csp_v1_header_t hdr;
    if (csp_v1_decode(packet, &hdr) != 0) {
        fprintf(stderr, "rx_decode: csp_v1_decode failed\n");
        return 1;
    }

    const uint8_t *payload = packet + 4;
    size_t payload_len = (size_t)packet_len - 4u;

    if (hex_only) {
        for (size_t i = 0; i < payload_len; ++i) {
            fprintf(stdout, "%02x", payload[i]);
        }
        fputc('\n', stdout);
        return (hmac_ok == 0) ? 2 : 0;
    }

    fprintf(stdout, "AX100: golay_errors=%d  hmac=%s\n",
            golay_errs,
            hmac_ok == 1 ? "ok"
            : hmac_ok == 0 ? "MISMATCH"
            : "(not checked)");
    fprintf(stdout, "CSP v1: src=%u dst=%u dport=%u sport=%u prio=%u flags=0x%02x\n",
            hdr.src, hdr.dst, hdr.dport, hdr.sport, hdr.prio, hdr.flags);
    fprintf(stdout, "payload (%zu bytes):\n", payload_len);
    if (payload_len > 0) {
        print_hex_ascii(payload, payload_len);
    }

    return (hmac_ok == 0) ? 2 : 0;
}
