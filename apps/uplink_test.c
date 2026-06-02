/*

    Simple Satellite Operations  uplink_test.c

    Standalone CLI that reproduces the CalgaryToSpace FrontierSat
    command-to-audio pipeline. Reads an ASCII-hex payload and HMAC key,
    composes a CSP v1 packet, frames it for the AX100 (HMAC + CCSDS
    scrambler + Golay length + sync word + preamble/postamble), optionally
    prints the framed bytes as hex for byte-compare against pycsplink,
    and optionally writes a 48 kHz mono 16-bit WAV file of the modulated
    baseband.

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

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(FILE *out, const char *argv0)
{
    fprintf(out,
        "usage: %s (--payload-hex=<HEX> | --payload-ascii=<STR>) [options]\n"
        "\n"
        "Composes a CSP v1 packet, AX100-frames it with HMAC, and writes\n"
        "either the framed byte sequence or a modulated 48 kHz/16-bit WAV.\n"
        "\n"
        "Required (exactly one):\n"
        "  --payload-hex=<HEX>      Payload as upper/lowercase hex\n"
        "  --payload-ascii=<STR>    Payload as literal ASCII bytes (no NUL)\n"
        "\n"
        "Options:\n"
        "  --keyfile=<path>         HMAC keyfile (default: $HOME/%s)\n"
        "  --no-hmac                Skip HMAC (invalid for live ops; test-only)\n"
        "  --reed-solomon           RS(255,223) encode (DEFAULT; matches pycsplink uplink)\n"
        "  --no-reed-solomon        Disable RS encode (for bit-compat with legacy frames)\n"
        "  --src=<0..31>            CSP source address (default 1)\n"
        "  --dst=<0..31>            CSP destination address (default 2)\n"
        "  --dport=<0..63>          CSP destination port (default 3)\n"
        "  --sport=<0..63>          CSP source port (default 4)\n"
        "  --prio=<0..3>            CSP priority, 2=norm (default 2)\n"
        "  --flags=<0..255>         CSP flags byte (default 0)\n"
        "  --print-frame            Print AX100-framed bytes as hex, one frame per line\n"
        "  --out=<file.wav>         Write modulated PCM as WAV (48 kHz mono 16-bit)\n"
        "  --bit-rate=<bps>         Bit rate (default 9600)\n"
        "  --samp-rate=<hz>         Sample rate (default 48000; must be integer multiple of bit-rate)\n"
        "  --gauss-bt=<float>       Gaussian BT (default 0.5; 0 disables filter)\n"
        "  --gauss-span=<symbols>   Gaussian filter span in symbols (default 4)\n"
        "  --gain-db=<float>        Output gain in dB (default 0)\n"
        "  --help                   This message\n",
        argv0, HMAC_KEYFILE_DEFAULT_RELPATH);
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Parses a hex string into bytes. Accepts upper+lower case here because
// this is operator input, not the keyfile (the keyfile's stricter).
// Returns length on success, -1 on malformed.
static ssize_t parse_payload_hex(const char *hex, uint8_t *out, size_t out_cap)
{
    size_t n = strlen(hex);
    if ((n & 1u) != 0) {
        fprintf(stderr, "payload hex: odd number of chars (%zu)\n", n);
        return -1;
    }
    size_t n_bytes = n / 2;
    if (n_bytes > out_cap) {
        fprintf(stderr, "payload hex: %zu bytes exceeds cap %zu\n",
                n_bytes, out_cap);
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

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "uplink_test")) return 0;
    const char *payload_hex = NULL;
    const char *payload_ascii = NULL;
    const char *keyfile_path = NULL;
    const char *out_wav = NULL;
    int use_hmac = 1;
    int use_rs = 1;  // default ON to match pycsplink uplink
    int print_frame = 0;

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
        else if (starts_with(a, "--payload-hex="))   payload_hex   = a + 14;
        else if (starts_with(a, "--payload-ascii=")) payload_ascii = a + 16;
        else if (starts_with(a, "--keyfile="))       keyfile_path  = a + 10;
        else if (strcmp(a, "--no-hmac") == 0)        use_hmac = 0;
        else if (strcmp(a, "--reed-solomon") == 0)    use_rs = 1;
        else if (strcmp(a, "--no-reed-solomon") == 0) use_rs = 0;
        else if (starts_with(a, "--src="))     csp_hdr.src   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dst="))     csp_hdr.dst   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dport="))   csp_hdr.dport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--sport="))   csp_hdr.sport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--prio="))    csp_hdr.prio  = (uint8_t)atoi(a + 7);
        else if (starts_with(a, "--flags="))   csp_hdr.flags = (uint8_t)atoi(a + 8);
        else if (strcmp(a, "--print-frame") == 0) print_frame = 1;
        else if (starts_with(a, "--out="))        out_wav = a + 6;
        else if (starts_with(a, "--bit-rate="))   mp.bit_rate = atoi(a + 11);
        else if (starts_with(a, "--samp-rate="))  mp.samp_rate = atoi(a + 12);
        else if (starts_with(a, "--gauss-bt="))   mp.gauss_bt = atof(a + 11);
        else if (starts_with(a, "--gauss-span=")) mp.gauss_symbol_span = atoi(a + 13);
        else if (starts_with(a, "--gain-db="))    mp.gain_db = atof(a + 10);
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
            fprintf(stderr, "payload ascii: %zu bytes exceeds cap %zu\n",
                    n, sizeof(payload));
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
    if (csp_len < 0) {
        fprintf(stderr, "csp_v1_encode failed\n");
        return 1;
    }

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (use_hmac) {
        opts.hmac_key = hmac_key;
        opts.hmac_key_len = (size_t)hmac_key_len;
    }
    opts.reed_solomon = use_rs;

    uint8_t frame[4200];
    ssize_t frame_len = ax100_frame(csp_packet, (size_t)csp_len, &opts,
                                    frame, sizeof(frame));
    if (frame_len < 0) {
        fprintf(stderr, "ax100_frame failed\n");
        return 1;
    }

    if (print_frame) {
        for (ssize_t i = 0; i < frame_len; ++i) {
            printf("%02X", frame[i]);
        }
        printf("\n");
    }

    if (out_wav != NULL) {
        int sps = mp.samp_rate / mp.bit_rate;
        size_t n_samples = (size_t)frame_len * 8u * (size_t)sps;
        int16_t *samples = (int16_t *)malloc(n_samples * sizeof(int16_t));
        if (samples == NULL) {
            fprintf(stderr, "out of memory for %zu samples\n", n_samples);
            return 1;
        }
        ssize_t n = modem_bytes_to_pcm16(frame, (size_t)frame_len, &mp,
                                         samples, n_samples);
        if (n < 0) {
            fprintf(stderr, "modem_bytes_to_pcm16 failed\n");
            free(samples);
            return 1;
        }
        if (pcm16_write_wav(out_wav, samples, (size_t)n, mp.samp_rate) != 0) {
            free(samples);
            return 1;
        }
        free(samples);
        fprintf(stderr,
                "wrote %s: %zd samples @ %d Hz (%.3f s), %zd frame bytes\n",
                out_wav, n, mp.samp_rate, (double)n / mp.samp_rate, frame_len);
    }

    return 0;
}
