/*

    Simple Satellite Operations  utils/rx_decode.c

    Offline AX100 frame decoder. Takes a WAV (or headerless S16_LE raw PCM
    from rtl_fm) captured over the air or produced synthetically by
    uplink_test, and recovers the CSP packet that was transmitted.

    Reverse of the uplink frame pipeline:

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
#include "beacon_cts1.h"
#include "csp.h"
#include "decode_loop.h"
#include "hmac_keyfile.h"
#include "modem.h"
#include "packet_db.h"
#include "wav_read.h"

#include <ctype.h>
#include <errno.h>
#include <time.h>
#include <math.h>
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
        "                             (output of `rtl_fm -M fm -s 48000` or\n"
        "                             rx_capture's .raw companion). Auto-\n"
        "                             enabled when <path> ends in '.raw'.\n"
        "  --rate=<hz>                Sample rate for --raw (default 48000)\n"
        "  --channels=<n>             Channels for --raw (default 2; ch 0\n"
        "                             used — matches rx_capture's stereo\n"
        "                             output. Pass --channels=1 for rtl_fm.)\n"
        "\n"
        "Framing / FEC / HMAC:\n"
        "  --reed-solomon             RS(255,223) decode (DEFAULT for uplink)\n"
        "  --no-reed-solomon          Disable RS decode. Use when talking to\n"
        "                             a TX that did not RS-encode (e.g.,\n"
        "                             downlink which uses CRC instead per\n"
        "                             pycsplink).\n"
        "  --hmac                     Enable HMAC verification. AX100\n"
        "                             downlink frames do NOT use HMAC, so\n"
        "                             this is OFF by default. Use when\n"
        "                             round-tripping uplink_test output.\n"
        "  --keyfile=<path>           HMAC keyfile (only relevant with\n"
        "                             --hmac; default $HOME/%s)\n"
        "\n"
        "Modem:\n"
        "  --bit-rate=<bps>           Default 9600\n"
        "  --invert                   Try inverted polarity first (both are\n"
        "                             always tried; this just reorders)\n"
        "  --sync-threshold=<0..8>    Max bit errors in the 32-bit ASM match\n"
        "                             (default 0 = strict; relax to 4-6 for\n"
        "                             real-RF captures where the sync word\n"
        "                             gets ~5 bit errors from ISI). When HMAC\n"
        "                             is enabled, rx_decode iterates through\n"
        "                             successive sync candidates until one\n"
        "                             HMAC-validates, so relaxing this is safe.\n"
        "\n"
        "Output:\n"
        "  -v                         Verbose: pipeline-stage diagnostics\n"
        "  --hex-only                 Print only the decoded payload as hex\n"
        "                             (for scripting)\n"
        "  --dump-bits[=<N>]          Print N bits at the detected ASM (default\n"
        "                             512). On success: bits from the modem's\n"
        "                             post-slicer stream starting at sync_off,\n"
        "                             with the expected ASM (0x930B51DE) shown\n"
        "                             above for visual Hamming inspection. On\n"
        "                             failure: 512 bits per phase from the\n"
        "                             diagnostic raw-slicer's best ASM-match\n"
        "                             window (was -v-only before).\n"
        "  --csp-crc32                Validate + strip a trailing CSP zlib\n"
        "                             CRC32. Off by default. AX100 frames\n"
        "                             in either direction don't necessarily\n"
        "                             carry one (uplink has HMAC; downlink\n"
        "                             depends on firmware). Enable only\n"
        "                             when you know the TX side appends a\n"
        "                             CRC; otherwise frames fail validation\n"
        "                             and the trailer stays in the payload.\n"
        "  --ref-hex=<hex>            Compare the decoded packet (or payload,\n"
        "                             auto-detected by length) against this\n"
        "                             reference and print byte positions\n"
        "                             that differ. Useful with the\n"
        "                             partial-RS rescue path to see where\n"
        "                             bit errors landed in a corrupted\n"
        "                             beacon. Whitespace and ':' in the hex\n"
        "                             string are ignored.\n"
        "  --no-partial-rs            Disable the partial-RS rescue. By\n"
        "                             default, when RS is on but the frame\n"
        "                             exceeds RS's 16-byte budget,\n"
        "                             rx_decode retries with RS off and\n"
        "                             reports the descrambled (uncorrected)\n"
        "                             bytes with rs=UNCORRECTABLE so the\n"
        "                             operator still sees the frame.\n"
        "  --force-beacon             Pad the decoded payload with zeros\n"
        "                             up to 130 bytes and print it as a\n"
        "                             beacon regardless of length or\n"
        "                             dispatch result. Last-ditch view of\n"
        "                             a heavily corrupted frame; a notice\n"
        "                             reports how many bytes were synthetic.\n"
        "  --quiet-headers            Hide the AX100 framing line, CSP\n"
        "                             header line, and payload hex/ascii\n"
        "                             dumps. Only the interpreted body\n"
        "                             (beacon/tcmd/log/bulk_file) and\n"
        "                             error states (HMAC/CRC mismatch,\n"
        "                             rs=UNCORRECTABLE) print. Default off\n"
        "                             — rx_decode is a forensic tool and\n"
        "                             ships with full headers. The same\n"
        "                             toggle is `--no-packet-headers` for\n"
        "                             consistency with rx_replay.\n"
        "  --packet-headers           Default; show every field. Kept as\n"
        "                             a no-op for scripts.\n"
        "  --db=<path>                Append decoded packets to a SQLite\n"
        "                             store. Default: $SSO_PACKET_DB or\n"
        "                             $HOME/.local/share/simple_sat_ops/\n"
        "                             packets.db. Rows are deduplicated\n"
        "                             on payload SHA1 + source-tool +\n"
        "                             source-run, so re-decoding the same\n"
        "                             capture is harmless.\n"
        "  --no-db                    Skip DB writes (the .log text\n"
        "                             output continues unchanged).\n"
        "  --source-run=<id>          Override the auto-generated\n"
        "                             per-launch run-id used for dedup.\n"
        "                             Useful when re-running the same\n"
        "                             input and you DO want a fresh row.\n"
        "  --no-dc-block              Skip the modem's DC-block IIR (alpha=0.995)\n"
        "                             on RX. Default-ON setting was added for\n"
        "                             rtl_fm's discriminator drift; for radio\n"
        "                             paths with no DC offset the HPF only adds\n"
        "                             baseline transients across burst\n"
        "                             boundaries that cost a few bits in the\n"
        "                             ASM detector. If the failure-path\n"
        "                             diagnostic shows ASM at HD=2-4 in the raw\n"
        "                             slicer but the modem reports 'no valid\n"
        "                             AX100 frame', this flag often closes the\n"
        "                             gap.\n"
        "  --help                     This message\n",
        argv0, argv0, HMAC_KEYFILE_DEFAULT_RELPATH);
}

int main(int argc, char **argv)
{
    const char *input_path = NULL;
    const char *keyfile_path = NULL;
    int raw_mode = 0;
    int raw_mode_explicit = 0;
    int raw_rate = 48000;
    int raw_channels = 2;
    int bit_rate = 9600;
    int invert = 0;
    int sync_max_ham = 0;
    int use_hmac = 0;  // AX100 downlink does not use HMAC; opt in with --hmac
    int use_rs = 1;  // default ON to match pycsplink uplink
    int csp_crc32 = 0;  // opt-in via --csp-crc32
    int verbose = 0;
    int hex_only = 0;
    int dump_bits = 0;  // 0 = disabled; >0 = bits to print on success/failure
    int no_dc_block = 0;
    const char *ref_hex_arg = NULL;
    uint8_t ref_buf[4100];
    size_t ref_buf_len = 0;
    int allow_partial_rs = 1;  // rescue path on by default; --no-partial-rs to disable
    int force_beacon = 0;
    // Forensic CLI: default to showing the raw header lines so single-frame
    // post-mortems aren't missing context. --quiet-headers flips it.
    int show_packet_headers = 1;
    const char *db_path = NULL;
    const char *source_run_override = NULL;
    int no_db = 0;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else if (strcmp(a, "--raw") == 0) {
            raw_mode = 1;
            raw_mode_explicit = 1;
        } else if (starts_with(a, "--rate=")) {
            raw_rate = atoi(a + 7);
        } else if (starts_with(a, "--channels=")) {
            raw_channels = atoi(a + 11);
        } else if (starts_with(a, "--keyfile=")) {
            keyfile_path = a + 10;
        } else if (strcmp(a, "--hmac") == 0) {
            use_hmac = 1;
        } else if (strcmp(a, "--no-hmac") == 0) {
            // Default is now --no-hmac; kept as a no-op so existing
            // scripts / docs don't break.
            use_hmac = 0;
        } else if (strcmp(a, "--reed-solomon") == 0) {
            use_rs = 1;
        } else if (strcmp(a, "--no-reed-solomon") == 0) {
            use_rs = 0;
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
        } else if (strcmp(a, "--dump-bits") == 0) {
            dump_bits = 512;
        } else if (starts_with(a, "--dump-bits=")) {
            dump_bits = atoi(a + 12);
            if (dump_bits <= 0) {
                fprintf(stderr, "rx_decode: --dump-bits must be > 0\n");
                return 1;
            }
        } else if (strcmp(a, "--no-dc-block") == 0) {
            no_dc_block = 1;
        } else if (strcmp(a, "--csp-crc32") == 0) {
            csp_crc32 = 1;
        } else if (strcmp(a, "--no-csp-crc32") == 0) {
            csp_crc32 = 0;
        } else if (starts_with(a, "--ref-hex=")) {
            ref_hex_arg = a + 10;
        } else if (strcmp(a, "--no-partial-rs") == 0) {
            allow_partial_rs = 0;
        } else if (strcmp(a, "--force-beacon") == 0) {
            force_beacon = 1;
        } else if (strcmp(a, "--packet-headers") == 0) {
            show_packet_headers = 1;
        } else if (strcmp(a, "--quiet-headers") == 0
                   || strcmp(a, "--no-packet-headers") == 0) {
            show_packet_headers = 0;
        } else if (starts_with(a, "--db=")) {
            db_path = a + 5;
        } else if (strcmp(a, "--no-db") == 0) {
            no_db = 1;
        } else if (starts_with(a, "--source-run=")) {
            source_run_override = a + 13;
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

    // Open the packet DB (or skip on --no-db) and plug it into emit_frame.
    // Note rx_decode doesn't actually call emit_frame (it has its own
    // duplicated output path), so this tap is currently inactive — but
    // wiring it now keeps rx_decode consistent with the other receivers
    // and primes for a future refactor that consolidates onto emit_frame.
    char db_run_id[24];
    packet_db_t *db = packet_db_setup(db_path, no_db,
                                      db_run_id, sizeof db_run_id);
    if (source_run_override != NULL) {
        snprintf(db_run_id, sizeof db_run_id, "%s", source_run_override);
    }
    decode_loop_set_packet_db(db, "rx_decode", db_run_id);

    if (ref_hex_arg != NULL) {
        // Tolerant hex parser: skip whitespace and ':' separators so
        // copy-pasted hex from beacon_gen output / ascii dumps Just Works.
        size_t n = 0;
        int high = -1;
        for (const char *p = ref_hex_arg; *p; ++p) {
            char c = *p;
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == ':') continue;
            int v = (c >= '0' && c <= '9') ? c - '0'
                  : (c >= 'a' && c <= 'f') ? c - 'a' + 10
                  : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
            if (v < 0) {
                fprintf(stderr, "rx_decode: --ref-hex: bad char '%c'\n", c);
                return 1;
            }
            if (high < 0) {
                high = v;
            } else {
                if (n >= sizeof ref_buf) {
                    fprintf(stderr, "rx_decode: --ref-hex too long (>%zu bytes)\n",
                            sizeof ref_buf);
                    return 1;
                }
                ref_buf[n++] = (uint8_t)((high << 4) | v);
                high = -1;
            }
        }
        if (high >= 0) {
            fprintf(stderr, "rx_decode: --ref-hex: odd number of hex digits\n");
            return 1;
        }
        if (n == 0) {
            fprintf(stderr, "rx_decode: --ref-hex: empty\n");
            return 1;
        }
        ref_buf_len = n;
    }
    // Auto-detect raw mode by file extension when --raw wasn't explicit.
    // .raw → headerless S16_LE PCM; anything else assumed to be a WAV
    // file (wav_read_pcm16 will reject mismatches with a clear error).
    if (!raw_mode_explicit) {
        size_t plen = strlen(input_path);
        if (plen >= 4 && strcmp(input_path + plen - 4, ".raw") == 0) {
            raw_mode = 1;
        }
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
    mp.rx_disable_dc_block = no_dc_block;
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

    // Load HMAC key now so we can use HMAC as a validation gate against
    // false-positive sync matches.
    uint8_t hmac_key[128];
    ssize_t hmac_key_len = 0;
    if (use_hmac) {
        char default_path[512];
        const char *path = keyfile_path;
        if (path == NULL) {
            if (hmac_keyfile_default_path(default_path, sizeof(default_path)) != 0) {
                fprintf(stderr, "rx_decode: HOME is unset; pass --keyfile=<path>\n");
                free(bits);
                free(samples);
                return 1;
            }
            path = default_path;
        }
        hmac_key_len = hmac_keyfile_load(path, hmac_key, sizeof(hmac_key));
        if (hmac_key_len < 0) {
            free(bits);
            free(samples);
            return 1;
        }
    }

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (use_hmac) {
        opts.hmac_key = hmac_key;
        opts.hmac_key_len = (size_t)hmac_key_len;
    }
    opts.reed_solomon = use_rs;

    // Preamble-anchored sync search: find the longest alternating run in
    // any phase/polarity, then start the sync-word scan just past its end.
    // The actual AX100 ASM is ALWAYS at (prefill * 8) bits after the
    // preamble starts, so anchoring the search there massively reduces
    // false positives compared to the full-file scan. We still do the
    // full scan as a fallback after exhausting preamble-anchored attempts.
    //
    // Must use the same DC-block as modem_pcm16_to_bits so the bit
    // streams align — otherwise altrun positions are off-grid.
    int sps_int = mp.samp_rate / mp.bit_rate;
    size_t best_altrun_len = 0;
    size_t best_altrun_start = 0;
    float *dc_blocked = (float *)malloc(n_samples * sizeof(float));
    uint8_t *pb_scratch = (uint8_t *)malloc(n_samples / (size_t)sps_int);
    if (dc_blocked != NULL && pb_scratch != NULL) {
        const float alpha = 0.995f;
        float prev_x = 0.0f, prev_y = 0.0f;
        for (size_t i = 0; i < n_samples; ++i) {
            float x = (float)samples[i];
            float y = x - prev_x + alpha * prev_y;
            dc_blocked[i] = y;
            prev_x = x; prev_y = y;
        }
        for (int pol = 0; pol < 2; ++pol) {
            for (int phase = 0; phase < sps_int; ++phase) {
                size_t mid = (size_t)phase + (size_t)sps_int / 2u;
                size_t nb = 0;
                for (size_t i = mid; i < n_samples; i += (size_t)sps_int) {
                    int v = (dc_blocked[i] > 0.0f) ? 1 : 0;
                    if (pol) v = !v;
                    pb_scratch[nb++] = (uint8_t)v;
                }
                size_t cur = 1, cur_start = 0, run_best = 1, run_start = 0;
                for (size_t i = 1; i < nb; ++i) {
                    if (pb_scratch[i] != pb_scratch[i-1]) { ++cur; }
                    else {
                        if (cur > run_best) { run_best = cur; run_start = cur_start; }
                        cur = 1; cur_start = i;
                    }
                }
                if (cur > run_best) { run_best = cur; run_start = cur_start; }
                if (run_best > best_altrun_len) {
                    best_altrun_len = run_best;
                    best_altrun_start = run_start;
                }
            }
        }
    }
    free(dc_blocked);
    free(pb_scratch);
    // Preamble ASM starts right at the end of the alt-run. We anchor
    // sync_search at one bit before that (to allow a single edge slip).
    int preamble_found = (best_altrun_len >= 64);  // 8 bytes of 0xAA = 64 bits
    size_t preamble_anchor = preamble_found
        ? best_altrun_start + best_altrun_len
        : 0;
    if (verbose && preamble_found) {
        fprintf(stderr, "rx_decode: preamble detected (altrun=%zu bits), "
                "anchoring sync scan at phase-bit %zu\n",
                best_altrun_len, preamble_anchor);
    }

    // Multi-hypothesis sync: find the Nth sync match, try to decode;
    // if HMAC fails (and HMAC is enabled as a gate), advance past it
    // and try again. Without this gate, the first random 32-bit
    // noise match in a ~10 s RX file gets accepted and produces
    // garbage payload + golay length.
    size_t n_bits = 0;
    size_t sync_off = 0;
    int polarity_used = -1;
    size_t min_offset = preamble_anchor > 4 ? preamble_anchor - 4 : 0;
    int attempts = 0;
    int decoded = 0;
    int preamble_fallback_done = 0;
    uint8_t packet[4100];
    ssize_t packet_len = -1;
    int golay_errs = 0, hmac_ok = -1;
    int rs_errs = -1, used_golay_len = -1;
    int rs_locs[32];
    int rc = 0;

    const int MAX_ATTEMPTS = 256;
    while (!decoded && attempts < MAX_ATTEMPTS) {
        rc = modem_pcm16_to_bits(samples, n_samples, &mp,
                                 invert, sync_max_ham, min_offset,
                                 bits, &n_bits, &sync_off, &polarity_used);
        if (rc != 0) {
            // No more matches at or past min_offset. If we were anchored
            // at the preamble and still haven't found a valid frame, fall
            // back to the full scan from bit 0.
            if (!preamble_fallback_done && preamble_anchor > 0) {
                preamble_fallback_done = 1;
                min_offset = 0;
                if (verbose) {
                    fprintf(stderr, "rx_decode: no match past preamble anchor, "
                            "falling back to full scan from bit 0\n");
                }
                continue;
            }
            break;
        }
        // If we moved past the anchored window without decoding, also
        // trigger the fallback. The anchor is ~30 bits around preamble_end
        // but the returned sync_off may drift as we advance min_offset.
        if (preamble_anchor > 0 && !preamble_fallback_done
            && sync_off > preamble_anchor + 32) {
            preamble_fallback_done = 1;
            min_offset = 0;
            if (verbose) {
                fprintf(stderr, "rx_decode: preamble-anchored window exhausted "
                        "(match drifted to bit %zu), falling back to full scan\n",
                        sync_off);
            }
            continue;
        }
        ++attempts;
        size_t max_bytes = (n_bits + 7) / 8;
        uint8_t *bytes = (uint8_t *)malloc(max_bytes);
        if (bytes == NULL) break;
        size_t n_bytes = modem_bits_to_bytes(bits, n_bits, bytes);
        packet_len = ax100_unframe(bytes, n_bytes, &opts,
                                   packet, sizeof(packet),
                                   &golay_errs, &hmac_ok,
                                   &rs_errs, &used_golay_len,
                                   rs_locs);
        // Partial-RS rescue: when RS is on, HMAC is off, and the Golay
        // length header decoded cleanly, retry with RS disabled to
        // recover the descrambled (uncorrected) bytes. Lets the operator
        // see corrupted-but-recognizable beacons when bit errors exceed
        // RS's 16-byte budget. Mirrors decode_loop's allow_partial_rs.
        if (packet_len < 0 && allow_partial_rs && use_rs && !use_hmac
            && golay_errs == 0) {
            ax100_opts_t partial = opts;
            partial.reed_solomon = 0;
            int p_golay = 0, p_hmac = -1, p_rs = -1, p_lensrc = -1;
            ssize_t pp = ax100_unframe(bytes, n_bytes, &partial,
                                       packet, sizeof(packet),
                                       &p_golay, &p_hmac,
                                       &p_rs, &p_lensrc,
                                       NULL);
            // pp is the descrambled byte count INCLUDING the original
            // 32-byte RS parity tail (RS isn't running to strip it).
            // Strip those 32 bytes here so the apparent packet length
            // matches what RS-on would have produced.
            if (pp > 32) {
                packet_len = pp - 32;
                golay_errs = p_golay;
                hmac_ok = -1;
                rs_errs = -2;  // UNCORRECTABLE marker
                used_golay_len = p_lensrc;
            }
        }
        free(bytes);
        if (packet_len < 0) {
            // Unframing failed outright (golay uncorrectable or length
            // overruns the buffer). Move past this sync and try next.
            if (verbose) {
                fprintf(stderr, "rx_decode: candidate #%d at bit %zu: unframe failed "
                        "(golay_errs=%d), trying next\n",
                        attempts, sync_off, golay_errs);
            }
            min_offset = sync_off + 1;
            continue;
        }
        // If HMAC is required and failed, this is likely a false match.
        if (use_hmac && hmac_ok == 0) {
            if (verbose) {
                fprintf(stderr, "rx_decode: candidate #%d at bit %zu: HMAC mismatch "
                        "(%zd-byte packet, golay_errs=%d), trying next\n",
                        attempts, sync_off, packet_len, golay_errs);
            }
            min_offset = sync_off + 1;
            continue;
        }
        decoded = 1;
    }

    if (!decoded) {
        fprintf(stderr, "rx_decode: no valid AX100 frame found "
                "(tried both polarities, sync-threshold=%d, %d candidate(s)%s)\n",
                sync_max_ham, attempts,
                use_hmac ? " HMAC-validated" : "");
        if (verbose || dump_bits > 0) {
            // For each phase, scan the full bit stream for:
            //   - longest alternating run (0xAA preamble detector)
            //   - closest 32-bit window match to the ASM (min Hamming distance)
            // Strong preamble + close sync = signal is there, relax threshold.
            // Weak preamble + far sync = signal isn't present at all.
            int sps = mp.samp_rate / mp.bit_rate;
            size_t n_phase_bits = n_samples / (size_t)sps;
            uint8_t *pb = (uint8_t *)malloc(n_phase_bits);
            if (pb != NULL) {
                const uint32_t ASM = 0x930B51DEu;
                fprintf(stderr, "rx_decode: per-phase diagnostic:\n");
                for (int phase = 0; phase < sps; ++phase) {
                    size_t mid = (size_t)phase + (size_t)sps / 2u;
                    size_t nb = 0;
                    for (size_t i = mid; i < n_samples; i += (size_t)sps) {
                        pb[nb++] = (uint8_t)((samples[i] > 0) ? 1 : 0);
                    }
                    // Longest alternating run.
                    size_t best_run = 1, cur_run = 1, best_run_start = 0, cur_run_start = 0;
                    for (size_t i = 1; i < nb; ++i) {
                        if (pb[i] != pb[i-1]) {
                            ++cur_run;
                        } else {
                            if (cur_run > best_run) { best_run = cur_run; best_run_start = cur_run_start; }
                            cur_run = 1;
                            cur_run_start = i;
                        }
                    }
                    if (cur_run > best_run) { best_run = cur_run; best_run_start = cur_run_start; }
                    // Closest 32-bit match to ASM.
                    int best_ham = 33;
                    size_t best_ham_pos = 0;
                    if (nb >= 32) {
                        uint32_t w = 0;
                        for (int i = 0; i < 32; ++i) w = (w << 1) | pb[i];
                        int h = __builtin_popcount(w ^ ASM);
                        if (h < best_ham) { best_ham = h; best_ham_pos = 0; }
                        for (size_t i = 32; i < nb; ++i) {
                            w = (w << 1) | pb[i];
                            h = __builtin_popcount(w ^ ASM);
                            if (h < best_ham) { best_ham = h; best_ham_pos = i - 31; }
                        }
                    }
                    fprintf(stderr, "  phase=%d  altrun=%zu@bit%zu  best_asm_hamming=%d@bit%zu\n",
                            phase, best_run, best_run_start, best_ham, best_ham_pos);
                }
                // Sliding RMS envelope in non-overlapping 100 ms windows.
                // Report the top-N loudest + the global median so the operator
                // can tell whether the signal is actually louder than the RX
                // noise floor. Peak sample also used to anchor the bit dump.
                size_t win_samples = (size_t)mp.samp_rate / 10u;  // 100 ms
                if (win_samples < 64) win_samples = 64;
                size_t n_windows = n_samples / win_samples;
                double *rms = (double *)malloc(n_windows * sizeof(double));
                if (rms != NULL && n_windows > 0) {
                    for (size_t w = 0; w < n_windows; ++w) {
                        double sum_sq = 0.0;
                        for (size_t j = 0; j < win_samples; ++j) {
                            double v = (double)samples[w * win_samples + j];
                            sum_sq += v * v;
                        }
                        rms[w] = sqrt(sum_sq / (double)win_samples);
                    }
                    double sorted[16];
                    size_t top_n = n_windows < 8 ? n_windows : 8;
                    for (size_t i = 0; i < top_n; ++i) sorted[i] = -1.0;
                    size_t top_idx[16] = {0};
                    for (size_t w = 0; w < n_windows; ++w) {
                        for (size_t i = 0; i < top_n; ++i) {
                            if (rms[w] > sorted[i]) {
                                for (size_t j = top_n - 1; j > i; --j) {
                                    sorted[j] = sorted[j-1];
                                    top_idx[j] = top_idx[j-1];
                                }
                                sorted[i] = rms[w];
                                top_idx[i] = w;
                                break;
                            }
                        }
                    }
                    // Median (approximate via partial sort of a copy).
                    double med = 0.0;
                    double *tmp = (double *)malloc(n_windows * sizeof(double));
                    if (tmp) {
                        memcpy(tmp, rms, n_windows * sizeof(double));
                        for (size_t i = 0; i < n_windows / 2 + 1; ++i) {
                            size_t mn = i;
                            for (size_t j = i + 1; j < n_windows; ++j)
                                if (tmp[j] < tmp[mn]) mn = j;
                            double t = tmp[i]; tmp[i] = tmp[mn]; tmp[mn] = t;
                        }
                        med = tmp[n_windows / 2];
                        free(tmp);
                    }
                    fprintf(stderr, "rx_decode: RMS envelope (%zu windows of %.0f ms, "
                            "median %.1f):\n",
                            n_windows, 1000.0 * (double)win_samples / mp.samp_rate, med);
                    for (size_t i = 0; i < top_n; ++i) {
                        double wt = (double)(top_idx[i] * win_samples
                                             + win_samples / 2) / mp.samp_rate;
                        fprintf(stderr, "  top-%zu: t=%.3fs  rms=%.1f  (x%.1f median)\n",
                                i + 1, wt, sorted[i],
                                med > 0 ? sorted[i] / med : 0.0);
                    }
                    free(rms);
                }
                // Anchor the bit dump at the global best_asm_hamming
                // position across all phases (rather than peak-RMS, which
                // is a bad detector under FM capture — a strong narrowband
                // signal can have LOWER RMS than wideband noise after FM
                // demodulation). The best_asm_hamming window is the closest
                // the bit stream ever gets to the ASM, so if there's a
                // burst at all that's where it is.
                int global_best_ham = 33;
                size_t global_best_ham_bit = 0;
                int global_best_phase = 0;
                for (int phase = 0; phase < sps; ++phase) {
                    size_t mid = (size_t)phase + (size_t)sps / 2u;
                    size_t nb = 0;
                    for (size_t i = mid; i < n_samples; i += (size_t)sps) {
                        pb[nb++] = (uint8_t)((samples[i] > 0) ? 1 : 0);
                    }
                    if (nb < 32) continue;
                    uint32_t w = 0;
                    for (int i = 0; i < 32; ++i) w = (w << 1) | pb[i];
                    const uint32_t ASM = 0x930B51DEu;
                    int h = __builtin_popcount(w ^ ASM);
                    if (h < global_best_ham) {
                        global_best_ham = h; global_best_ham_bit = 0;
                        global_best_phase = phase;
                    }
                    for (size_t i = 32; i < nb; ++i) {
                        w = (w << 1) | pb[i];
                        h = __builtin_popcount(w ^ ASM);
                        if (h < global_best_ham) {
                            global_best_ham = h;
                            global_best_ham_bit = i - 31;
                            global_best_phase = phase;
                        }
                    }
                }
                size_t peak_center_sample = (size_t)global_best_phase
                    + (size_t)sps / 2u
                    + global_best_ham_bit * (size_t)sps;
                if (peak_center_sample >= n_samples) peak_center_sample = n_samples - 1;
                double peak_t = (double)peak_center_sample / (double)mp.samp_rate;
                fprintf(stderr, "rx_decode: best-ASM-match window at t=%.3fs "
                        "(phase %d, ham=%d)\n",
                        peak_t, global_best_phase, global_best_ham);
                // Dump 512 bits starting ~20 ms before the peak so the preamble
                // (if present) is visible at the start of each line.
                size_t lead_samples = (size_t)mp.samp_rate / 50u;  // 20 ms
                size_t dump_start = peak_center_sample > lead_samples
                    ? peak_center_sample - lead_samples : 0;
                size_t fail_bits = (size_t)(dump_bits > 0 ? dump_bits : 512);
                fprintf(stderr, "rx_decode: %zu bits @ each phase starting %.3fs into file:\n",
                        fail_bits, (double)dump_start / (double)mp.samp_rate);
                for (int phase = 0; phase < sps; ++phase) {
                    fprintf(stderr, "  phase=%d  ", phase);
                    // Align start to this phase's slicing grid.
                    size_t mid = (size_t)phase + (size_t)sps / 2u;
                    size_t first = mid;
                    if (dump_start > mid) {
                        size_t skip = (dump_start - mid + (size_t)sps - 1) / (size_t)sps;
                        first = mid + skip * (size_t)sps;
                    }
                    size_t printed = 0;
                    for (size_t i = first; i < n_samples && printed < fail_bits; i += (size_t)sps) {
                        fputc((samples[i] > 0) ? '1' : '0', stderr);
                        ++printed;
                    }
                    fputc('\n', stderr);
                }
                free(pb);
            }
        }
        free(bits);
        free(samples);
        return 1;
    }
    if (dump_bits > 0) {
        // Modem-stream bit dump. modem_pcm16_to_bits aligns out_bits so
        // that bit 0 IS the ASM (despite sync_off being recorded as an
        // offset into the pre-alignment raw stream, which is just a
        // diagnostic). With normal polarity bits[0..31] match
        // 0x930B51DE; with inverted polarity all bits are flipped.
        const uint32_t ASM = 0x930B51DEu;
        fprintf(stderr,
                "rx_decode: bit dump (sync_off=%zu in raw stream, "
                "polarity=%s, %d-bit ASM | post-ASM data):\n",
                sync_off, polarity_used ? "inverted" : "normal", 32);
        fprintf(stderr, "  ASM expected: ");
        for (int b = 31; b >= 0; --b) {
            int bit = (int)((ASM >> b) & 1u);
            if (polarity_used) bit = !bit;
            fputc(bit ? '1' : '0', stderr);
            if (b > 0 && b % 8 == 0) fputc(' ', stderr);
        }
        fputc('\n', stderr);
        fprintf(stderr, "  bits @ bit 0: ");
        size_t want = (size_t)dump_bits;
        if (want > n_bits) want = n_bits;
        for (size_t k = 0; k < want; ++k) {
            fputc(bits[k] ? '1' : '0', stderr);
            // Space every 8 bits, double-space at the ASM/data boundary.
            if ((k + 1) % 8 == 0 && k + 1 < want) {
                fputc(' ', stderr);
                if (k + 1 == 32) fputc('|', stderr);
            }
        }
        fputc('\n', stderr);
    }

    free(bits);
    free(samples);

    if (verbose) {
        fprintf(stderr, "rx_decode: ASM at bit offset %zu, %zu bits after ASM, "
                "polarity=%s, candidate #%d%s\n",
                sync_off, n_bits, polarity_used ? "inverted" : "normal",
                attempts, use_hmac ? " (HMAC-validated)" : "");
        char rs_buf[32];
        if (rs_errs == -2)    snprintf(rs_buf, sizeof rs_buf, "UNCORRECTABLE");
        else if (rs_errs < 0) snprintf(rs_buf, sizeof rs_buf, "(off/failed)");
        else                  snprintf(rs_buf, sizeof rs_buf, "%d", rs_errs);
        if (use_hmac) {
            fprintf(stderr, "rx_decode: inner packet %zd bytes, golay errors=%d, "
                    "hmac=%s, rs_corrected=%s, len_source=%s\n",
                    packet_len, golay_errs,
                    hmac_ok == 1 ? "ok" : hmac_ok == 0 ? "MISMATCH" : "(not checked)",
                    rs_buf,
                    used_golay_len == 1 ? "golay-header"
                    : used_golay_len == 0 ? "brute-force" : "(n/a)");
        } else {
            fprintf(stderr, "rx_decode: inner packet %zd bytes, golay errors=%d, "
                    "rs_corrected=%s, len_source=%s\n",
                    packet_len, golay_errs, rs_buf,
                    used_golay_len == 1 ? "golay-header"
                    : used_golay_len == 0 ? "brute-force" : "(n/a)");
        }
    }

    if (packet_len < 4) {
        fprintf(stderr, "rx_decode: packet too short to contain CSP header (%zd)\n",
                packet_len);
        return 1;
    }

    // CSP v1 downlink: 4-byte zlib CRC trailer (libcsp CRC mode). Validate
    // and strip when HMAC is off; uplink frames have a 32-byte SHA-256
    // HMAC tag instead and that's the user's problem to interpret.
    int crc_ok = -1;
    if (!use_hmac && csp_crc32 && packet_len >= 8) {
        uint32_t computed = csp_crc32_zlib(packet, (size_t)(packet_len - 4));
        uint32_t le = (uint32_t)packet[packet_len - 4]
                    | ((uint32_t)packet[packet_len - 3] << 8)
                    | ((uint32_t)packet[packet_len - 2] << 16)
                    | ((uint32_t)packet[packet_len - 1] << 24);
        uint32_t be = ((uint32_t)packet[packet_len - 4] << 24)
                    | ((uint32_t)packet[packet_len - 3] << 16)
                    | ((uint32_t)packet[packet_len - 2] <<  8)
                    |  (uint32_t)packet[packet_len - 1];
        if (computed == le || computed == be) {
            crc_ok = 1;
            packet_len -= 4;
            // Print to stdout (not stderr / -v only) so the integrity
            // verdict is part of the regular decode output. Gated by
            // the headers toggle since success is just confirmation;
            // the MISMATCH branch below stays unconditional.
            if (show_packet_headers) {
                fprintf(stdout, "csp_crc32: ok (0x%08x, 4-byte trailer "
                        "stripped)\n", computed);
            }
        } else {
            crc_ok = 0;
            // Mismatch: keep the trailer in the payload so the operator
            // can inspect what was received. Print the verdict regardless.
            fprintf(stdout, "csp_crc32: MISMATCH "
                    "(computed=0x%08x, trailer LE=0x%08x BE=0x%08x; "
                    "trailer kept in payload)\n", computed, le, be);
        }
    }
    // crc_ok is consumed below by decode_loop_record_packet.

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

    char rs_summary[32];
    if (rs_errs == -2)    snprintf(rs_summary, sizeof rs_summary, "UNCORRECTABLE");
    else if (rs_errs < 0) snprintf(rs_summary, sizeof rs_summary, "off");
    else                  snprintf(rs_summary, sizeof rs_summary, "corrected=%d", rs_errs);
    const char *len_src =
        used_golay_len == 1 ? "golay-header"
        : used_golay_len == 0 ? "brute-forced" : "(n/a)";
    int hmac_bad = use_hmac && hmac_ok == 0;
    int rs_bad = rs_errs == -2;
    int show_ax100 = show_packet_headers || hmac_bad || rs_bad;
    if (show_ax100) {
        if (use_hmac) {
            fprintf(stdout, "AX100: golay_errors=%d  hmac=%s  rs=%s  len=%s\n",
                    golay_errs,
                    hmac_ok == 1 ? "ok"
                    : hmac_ok == 0 ? "MISMATCH"
                    : "(not checked)",
                    rs_summary, len_src);
        } else {
            fprintf(stdout, "AX100: golay_errors=%d  rs=%s  len=%s\n",
                    golay_errs, rs_summary, len_src);
        }
    }
    // Per-byte error positions from the RS solver. Sorted so the operator
    // can see at a glance whether errors cluster late in the block (clock
    // drift) or scatter (channel noise). Last 32 indices are RS parity.
    if (show_packet_headers && rs_errs > 0) {
        size_t on_wire_len = (size_t)packet_len + (use_hmac ? 4 : 0) + 32;
        int sorted[32];
        int n = rs_errs > 32 ? 32 : rs_errs;
        for (int i = 0; i < n; ++i) sorted[i] = rs_locs[i];
        for (int i = 1; i < n; ++i) {
            int v = sorted[i], j = i - 1;
            while (j >= 0 && sorted[j] > v) { sorted[j+1] = sorted[j]; --j; }
            sorted[j+1] = v;
        }
        fprintf(stdout, "rs_locs: corrected=%d of %zu on-wire bytes:",
                rs_errs, on_wire_len);
        for (int i = 0; i < n; ++i) fprintf(stdout, " %d", sorted[i]);
        fputc('\n', stdout);
    }
    // Reference comparison: byte positions where the decoded frame
    // differs from a known-good reference. Lets the operator see where
    // bit errors landed in a partial-RS-rescued (uncorrected) frame
    // when they have a clean copy of the same beacon to diff against.
    if (show_packet_headers && ref_buf_len > 0) {
        const uint8_t *cmp = NULL;
        size_t cmp_len = 0;
        const char *what = NULL;
        if (ref_buf_len == (size_t)packet_len) {
            cmp = packet; cmp_len = (size_t)packet_len; what = "packet";
        } else if (ref_buf_len == payload_len) {
            cmp = payload; cmp_len = payload_len; what = "payload";
        }
        if (cmp != NULL) {
            int diffs[256];
            int n_diff = 0;
            for (size_t i = 0; i < cmp_len; ++i) {
                if (cmp[i] != ref_buf[i]) {
                    if (n_diff < (int)(sizeof diffs / sizeof diffs[0])) {
                        diffs[n_diff] = (int)i;
                    }
                    ++n_diff;
                }
            }
            int shown = n_diff > (int)(sizeof diffs / sizeof diffs[0])
                ? (int)(sizeof diffs / sizeof diffs[0]) : n_diff;
            fprintf(stdout, "ref_diff: %d of %zu %s bytes differ:",
                    n_diff, cmp_len, what);
            for (int i = 0; i < shown; ++i) fprintf(stdout, " %d", diffs[i]);
            if (shown < n_diff) fprintf(stdout, " ...");
            fputc('\n', stdout);
        } else {
            fprintf(stdout, "ref_diff: skipped (ref %zu bytes, packet %zd, "
                    "payload %zu)\n",
                    ref_buf_len, packet_len, payload_len);
        }
    }
    if (show_packet_headers) {
        fprintf(stdout, "CSP v1: src=%u dst=%u dport=%u sport=%u prio=%u flags=0x%02x\n",
                hdr.src, hdr.dst, hdr.dport, hdr.sport, hdr.prio, hdr.flags);
    }
    if (force_beacon) {
        // Pad/truncate the payload to the 130-byte beacon size and print.
        // Lets the operator pry telemetry out of frames whose magic bytes
        // got mangled or whose Golay length header misdecoded.
        uint8_t buf[sizeof(COMMS_beacon_basic_packet_t)] = {0};
        size_t copy = payload_len < sizeof buf ? payload_len : sizeof buf;
        if (copy > 0) memcpy(buf, payload, copy);
        fprintf(stdout, "force_beacon: padded %zu->130 bytes "
                "(zero-fill from byte %zu)\n", payload_len, copy);
        beacon_print(stdout, NULL, buf, sizeof buf);
    } else {
        cts1_packet_print(stdout, NULL, payload, payload_len);
    }
    if (show_packet_headers) {
        fprintf(stdout, "payload (%zu bytes):\n", payload_len);
        if (payload_len > 0) {
            print_hex_ascii(payload, payload_len);
        }
    }

    // DB tap. rx_decode has its own duplicated output path (it doesn't
    // route through emit_frame), so call decode_loop_record_packet
    // explicitly. No-op when --no-db was passed or sqlite3 isn't built
    // in; the receiver's setter passed NULL into decode_loop in that
    // case.
    {
        char ts_iso[40];
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        struct tm utc;
        gmtime_r(&now.tv_sec, &utc);
        // Masks bound each field so gcc's -Wformat-truncation= can
        // prove no overrun (see decode_loop.c for the same pattern).
        int yr = (utc.tm_year + 1900) % 10000;
        int mo = (utc.tm_mon + 1) % 100;
        int da = utc.tm_mday % 100;
        int hh = utc.tm_hour % 100;
        int mm = utc.tm_min  % 100;
        int ss = utc.tm_sec  % 100;
        int ms = (int)((now.tv_nsec / 1000000) % 1000);
        snprintf(ts_iso, sizeof ts_iso,
                 "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                 yr, mo, da, hh, mm, ss, ms);
        decode_loop_record_packet(ts_iso, &hdr, /*csp_ok=*/1,
                                  payload, payload_len,
                                  golay_errs, hmac_ok,
                                  rs_errs, crc_ok);
    }

    return (hmac_ok == 0) ? 2 : 0;
}
