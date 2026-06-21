/*

    Simple Satellite Operations  utils/beacon_gen.c

    Synthesise a CTS1 OBC beacon recording: fill the firmware-canonical
    COMMS_beacon_basic_packet_t with deterministic plausible values,
    wrap with CSP+AX100, and write a 48 kHz mono S16_LE WAV that matches
    what an FM discriminator would emit during a perfect-signal downlink.

    Drop the WAV into rx_decode / rx_replay / b210_rx_tx to exercise
    the decode chain (and the new beacon parser in beacon_cts1.c)
    without waiting for a satellite pass.

    --repeats=N writes N beacons in one WAV with --gap-seconds=S
    silence between them; per-beacon, uptime_ms / eps_uptime_sec /
    unix_epoch_time_ms / total_beacon_count_since_boot advance so the
    decoded "session" is internally coherent.

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

#include "argparse.h"
#include "ax100.h"
#include "beacon_cts1.h"
#include "csp.h"
#include "modem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

typedef enum { PACKET_BEACON, PACKET_TCMD_RESPONSE } packet_kind_t;

// Parsed command-line configuration. parse_args() fills this; main() copies
// the fields out into working locals so the (large) synthesis body is
// unchanged. csp_hdr / mp are carried whole so their parse actions stay
// byte-identical (e.g. a->csp_hdr.src = (uint8_t)atoi(arg + 6)); main()
// seeds their non-literal defaults (modem_params_defaults + CTS1 constants)
// before parse_args runs.
typedef struct {
    const char *out_wav;
    int repeats;
    int gap_seconds;
    int use_rs;
    int print_frame;
    int print_struct;
    packet_kind_t kind;
    int tcmd_code;
    int tcmd_duration_ms;
    const char *tcmd_message;
    csp_v1_header_t csp_hdr;
    modem_params_t mp;
} beacon_gen_args_t;

// Option column width: the widest label below ("--type=tcmd-response") + a
// small margin. See src/cli/argparse.h for the parse_args convention.
#define OPTW 22

// Parse argv into *a (help == 0), or print one right-aligned help line per
// option and return (help != 0). Each option is one self-contained block whose
// test carries "|| help", so help mode falls through and prints them all.
static int parse_args(beacon_gen_args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (starts_with(arg, "--out=") || help) {
            if (help) parse_help_line(OPTW, "--out=<file.wav>", "output WAV path (required)");
            else a->out_wav = arg + 6;
            matched = 1;
        }
        if (strcmp(arg, "--type=beacon") == 0 || help) {
            if (help) parse_help_line(OPTW, "--type=beacon", "COMMS_beacon_basic_packet_t (default)");
            else a->kind = PACKET_BEACON;
            matched = 1;
        }
        if (strcmp(arg, "--type=tcmd-response") == 0 || help) {
            if (help) parse_help_line(OPTW, "--type=tcmd-response", "COMMS_tcmd_response_packet_t (single packet)");
            else a->kind = PACKET_TCMD_RESPONSE;
            matched = 1;
        }
        if (starts_with(arg, "--tcmd-code=") || help) {
            if (help) parse_help_line(OPTW, "--tcmd-code=<n>", "tcmd-response: response_code (default 0 = OK)");
            else a->tcmd_code = atoi(arg + 12);
            matched = 1;
        }
        if (starts_with(arg, "--tcmd-duration=") || help) {
            if (help) parse_help_line(OPTW, "--tcmd-duration=<n>", "tcmd-response: duration_ms (default 42)");
            else a->tcmd_duration_ms = atoi(arg + 16);
            matched = 1;
        }
        if (starts_with(arg, "--tcmd-message=") || help) {
            if (help) parse_help_line(OPTW, "--tcmd-message=<str>", "tcmd-response: body (default \"OK: telecommand executed\"; NUL appended)");
            else a->tcmd_message = arg + 15;
            matched = 1;
        }
        if (starts_with(arg, "--repeats=") || help) {
            if (help) parse_help_line(OPTW, "--repeats=<n>", "emit N packets in one WAV (default 1)");
            else a->repeats = atoi(arg + 10);
            matched = 1;
        }
        if (starts_with(arg, "--gap-seconds=") || help) {
            if (help) parse_help_line(OPTW, "--gap-seconds=<s>", "silence between/after packets (default 20)");
            else a->gap_seconds = atoi(arg + 14);
            matched = 1;
        }
        if (strcmp(arg, "--reed-solomon") == 0 || help) {
            if (help) parse_help_line(OPTW, "--reed-solomon", "RS(255,223) on (default)");
            else a->use_rs = 1;
            matched = 1;
        }
        if (strcmp(arg, "--no-reed-solomon") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-reed-solomon", "RS off (for negative-control testing)");
            else a->use_rs = 0;
            matched = 1;
        }
        if (starts_with(arg, "--src=") || help) {
            if (help) parse_help_line(OPTW, "--src=<0..31>", "CSP source (default 1, firmware sat)");
            else a->csp_hdr.src = (uint8_t)atoi(arg + 6);
            matched = 1;
        }
        if (starts_with(arg, "--dst=") || help) {
            if (help) parse_help_line(OPTW, "--dst=<0..31>", "CSP destination (default 10, firmware GS)");
            else a->csp_hdr.dst = (uint8_t)atoi(arg + 6);
            matched = 1;
        }
        if (starts_with(arg, "--dport=") || help) {
            if (help) parse_help_line(OPTW, "--dport=<0..63>", "CSP destination port (default 10)");
            else a->csp_hdr.dport = (uint8_t)atoi(arg + 8);
            matched = 1;
        }
        if (starts_with(arg, "--sport=") || help) {
            if (help) parse_help_line(OPTW, "--sport=<0..63>", "CSP source port (default 10)");
            else a->csp_hdr.sport = (uint8_t)atoi(arg + 8);
            matched = 1;
        }
        if (starts_with(arg, "--prio=") || help) {
            if (help) parse_help_line(OPTW, "--prio=<0..3>", "CSP priority (default 3, firmware value)");
            else a->csp_hdr.prio = (uint8_t)atoi(arg + 7);
            matched = 1;
        }
        if (starts_with(arg, "--flags=") || help) {
            if (help) parse_help_line(OPTW, "--flags=<0..255>", "CSP flags byte (default 0)");
            else a->csp_hdr.flags = (uint8_t)atoi(arg + 8);
            matched = 1;
        }
        if (starts_with(arg, "--bit-rate=") || help) {
            if (help) parse_help_line(OPTW, "--bit-rate=<bps>", "bit rate (default 9600)");
            else a->mp.bit_rate = atoi(arg + 11);
            matched = 1;
        }
        if (starts_with(arg, "--samp-rate=") || help) {
            if (help) parse_help_line(OPTW, "--samp-rate=<hz>", "sample rate (default 48000)");
            else a->mp.samp_rate = atoi(arg + 12);
            matched = 1;
        }
        if (starts_with(arg, "--gauss-bt=") || help) {
            if (help) parse_help_line(OPTW, "--gauss-bt=<float>", "Gaussian BT (default 0.5)");
            else a->mp.gauss_bt = atof(arg + 11);
            matched = 1;
        }
        if (starts_with(arg, "--gauss-span=") || help) {
            if (help) parse_help_line(OPTW, "--gauss-span=<n>", "Gaussian filter span in symbols (default 4)");
            else a->mp.gauss_symbol_span = atoi(arg + 13);
            matched = 1;
        }
        if (starts_with(arg, "--gain-db=") || help) {
            if (help) parse_help_line(OPTW, "--gain-db=<float>", "output gain dB (default 0)");
            else a->mp.gain_db = atof(arg + 10);
            matched = 1;
        }
        if (strcmp(arg, "--print-frame") == 0 || help) {
            if (help) parse_help_line(OPTW, "--print-frame", "hex dump the AX100 frame to stdout");
            else a->print_frame = 1;
            matched = 1;
        }
        if (strcmp(arg, "--print-struct") == 0 || help) {
            if (help) parse_help_line(OPTW, "--print-struct", "hex dump the wire payload to stderr");
            else a->print_struct = 1;
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "beacon_gen: unknown option: %s\n", arg);
            return PARSE_ERROR;
        }
    }
    return PARSE_OK;
}

// Fill the beacon struct with deterministic plausibles. n is the
// per-WAV beacon index (0..repeats-1) so multi-beacon outputs decode
// to a coherent "session" with advancing uptime + epoch + count.
static void fill_beacon(COMMS_beacon_basic_packet_t *b,
                        unsigned n, uint64_t epoch_ms_base)
{
    memset(b, 0, sizeof *b);

    b->packet_type = COMMS_PACKET_TYPE_BEACON_BASIC;
    memcpy(b->satellite_name, "CTS1", 4);

    b->active_rf_switch_antenna = 1;
    b->active_rf_switch_control_mode = 0;
    b->uptime_ms = 3600000u + n * 20000u;

    b->duration_since_last_uplink_ms = 3600000u;
    b->unix_epoch_time_ms = epoch_ms_base + (uint64_t)n * 20000u;
    b->last_time_sync_source_enum = 0;

    b->is_fs_mounted = 1;

    b->total_tcmd_queued_count = 0;
    b->pending_queued_tcmd_count = 0;

    b->total_beacon_count_since_boot = 100u + n;

    b->eps_mode_enum = 1;          // NOMINAL
    b->eps_reset_cause_enum = 0;
    b->eps_uptime_sec = 3600u + n * 20u;
    b->eps_error_code = 0;
    b->eps_battery_voltage_mV = 3000;
    b->eps_battery_percent = 70;
    b->eps_battery_temperature_0_cC = 500;     // 5.00 C
    b->eps_battery_temperature_1_cC = 500;
    b->eps_total_fault_count = 0;
    b->eps_enabled_channels_bitfield = 0xFF;
    b->eps_total_pcu_power_input_cW = 200;     // 2.00 W
    b->eps_total_pcu_power_output_cW = 180;    // 1.80 W
    b->eps_total_avg_pcu_power_input_cW = 200;
    b->eps_total_avg_pcu_power_output_cW = 180;

    b->obc_temperature_cC = 2500;              // 25.00 C

    b->reboot_reason = 0;
    b->cts1_operation_state = CTS1_OPERATION_STATE_NOMINAL_WITH_RADIO_TX;
    b->rbf_pin_state = 0;

    b->mpi_rx_mode_enum = 0;
    b->mpi_transceiver_state_enum = 0;
    b->mpi_last_reason_for_stopping_enum = 0;
    b->gnss_uart_interrupt_enabled = 0;
    b->gnss_rx_mode_enum = 0;

    // 38 chars + NUL fits in 42; remaining bytes left zero by memset.
    const char *msg = "Hello from CalgaryToSpace FrontierSat";
    size_t mlen = strlen(msg);
    if (mlen >= COMMS_BEACON_FRIENDLY_MESSAGE_SIZE) {
        mlen = COMMS_BEACON_FRIENDLY_MESSAGE_SIZE - 1;
    }
    memcpy(b->friendly_message, msg, mlen);

    memcpy(b->end_message, "END\0", 4);
}

// Single-packet TCMD response. Fills the header + copies `message` into
// the data field with a trailing NUL (firmware contract: strnlen+1 bytes
// transmitted on the last packet of a response). Returns wire-size:
// COMMS_TCMD_RESPONSE_HEADER_SIZE + strlen(message) + 1.
static size_t fill_tcmd_response(COMMS_tcmd_response_packet_t *t,
                                 uint64_t epoch_ms,
                                 uint8_t response_code,
                                 uint16_t duration_ms,
                                 const char *message)
{
    memset(t, 0, sizeof *t);
    t->packet_type = COMMS_PACKET_TYPE_TCMD_RESPONSE;
    t->ts_sent = epoch_ms;
    t->response_code = response_code;
    t->duration_ms = duration_ms;
    t->response_seq_num = 1;
    t->response_max_seq_num = 1;

    size_t mlen = strlen(message);
    // Reserve 1 byte for the trailing NUL.
    if (mlen >= COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET) {
        mlen = COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET - 1;
    }
    memcpy(t->data, message, mlen);
    t->data[mlen] = 0x00;

    return COMMS_TCMD_RESPONSE_HEADER_SIZE + mlen + 1;
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "beacon_gen")) return 0;

    beacon_gen_args_t cfg = {
        .repeats = 1,
        .gap_seconds = 20,
        .use_rs = 1,
        .kind = PACKET_BEACON,
        .tcmd_duration_ms = 42,
        .tcmd_message = "OK: telecommand executed",
        .csp_hdr = {
            .prio  = CTS1_BEACON_CSP_PRIO,
            .src   = CTS1_BEACON_CSP_SRC,
            .dst   = CTS1_BEACON_CSP_DST,
            .dport = CTS1_BEACON_CSP_DPORT,
            .sport = CTS1_BEACON_CSP_SPORT,
            .flags = CTS1_BEACON_CSP_FLAGS,
        },
    };
    // mp's defaults come from modem_params_defaults(), not literals.
    modem_params_defaults(&cfg.mp);

    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }

    if (cfg.out_wav == NULL) {
        fprintf(stderr, "beacon_gen: --out=<file.wav> is required\n");
        return 1;
    }
    if (cfg.repeats < 1 || cfg.repeats > 100000) {
        fprintf(stderr, "beacon_gen: --repeats must be in [1, 100000]\n");
        return 1;
    }
    if (cfg.gap_seconds < 0 || cfg.gap_seconds > 3600) {
        fprintf(stderr, "beacon_gen: --gap-seconds must be in [0, 3600]\n");
        return 1;
    }
    if (cfg.mp.samp_rate <= 0 || cfg.mp.bit_rate <= 0
        || (cfg.mp.samp_rate % cfg.mp.bit_rate) != 0) {
        fprintf(stderr,
                "beacon_gen: samp_rate (%d) must be a positive multiple of "
                "bit_rate (%d)\n", cfg.mp.samp_rate, cfg.mp.bit_rate);
        return 1;
    }
    // Upper-bound the rate too: with the repeats / gap caps above this keeps
    // total_cap (below) from overflowing into a tiny allocation. 100 MHz is
    // well above any rate this tool would realistically generate.
    if (cfg.mp.samp_rate > 100000000) {
        fprintf(stderr, "beacon_gen: --samp-rate too high (max 100 MHz)\n");
        return 1;
    }

    // Copy parsed config into the working locals the synthesis body uses.
    const char *out_wav = cfg.out_wav;
    int repeats = cfg.repeats;
    int gap_seconds = cfg.gap_seconds;
    int use_rs = cfg.use_rs;
    int print_frame = cfg.print_frame;
    int print_struct = cfg.print_struct;
    packet_kind_t kind = cfg.kind;
    int tcmd_code = cfg.tcmd_code;
    int tcmd_duration_ms = cfg.tcmd_duration_ms;
    const char *tcmd_message = cfg.tcmd_message;
    csp_v1_header_t csp_hdr = cfg.csp_hdr;
    modem_params_t mp = cfg.mp;

    int sps = mp.samp_rate / mp.bit_rate;

    // Worst-case AX100 frame size: prefill 32 + ASM 4 + Golay 3 + RS-padded
    // block 255 + tail 1 = 295. Round up for safety.
    const size_t frame_cap = 320;
    size_t per_beacon_samples = frame_cap * 8u * (size_t)sps;
    size_t per_gap_samples = (size_t)gap_seconds * (size_t)mp.samp_rate;
    // Emit one gap-of-silence after every beacon (not just between) so
    // rx_replay's sliding window has trailing context to fully capture
    // the last burst, and the on-disk WAV mirrors a continuous "every
    // 20 s" downlink rather than ending mid-frame.
    size_t total_cap = (size_t)repeats * (per_beacon_samples + per_gap_samples);
    // The input caps above keep this product well inside size_t; reject a
    // request past a fixed ceiling rather than attempt an absurd allocation.
    const size_t MAX_SAMPLES = 500000000;  // ~1 GB of int16
    if (total_cap == 0 || total_cap > MAX_SAMPLES) {
        fprintf(stderr, "beacon_gen: %zu samples requested (cap %zu) -- "
                "lower --repeats / --gap-seconds / --samp-rate\n",
                total_cap, MAX_SAMPLES);
        return 1;
    }

    int16_t *samples = (int16_t *)malloc(total_cap * sizeof(int16_t));
    if (samples == NULL) {
        fprintf(stderr, "beacon_gen: out of memory for %zu samples\n", total_cap);
        return 1;
    }

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    opts.hmac_key = NULL;
    opts.hmac_key_len = 0;
    opts.reed_solomon = use_rs;

    // Same wall-clock epoch for every beacon in this WAV; per-beacon
    // increments are added in fill_beacon.
    uint64_t epoch_ms_base = (uint64_t)time(NULL) * 1000ull;

    size_t total_n = 0;
    size_t total_frame_bytes = 0;
    for (int n = 0; n < repeats; ++n) {
        uint8_t payload[256];
        size_t payload_size = 0;

        if (kind == PACKET_BEACON) {
            COMMS_beacon_basic_packet_t b;
            fill_beacon(&b, (unsigned)n, epoch_ms_base);
            payload_size = sizeof b;
            memcpy(payload, &b, payload_size);
        } else {
            COMMS_tcmd_response_packet_t t;
            // Advance ts_sent by 1 s per repeat so multi-packet WAVs
            // decode to a coherent sequence of timestamps.
            uint64_t ts = epoch_ms_base + (uint64_t)n * 1000ull;
            payload_size = fill_tcmd_response(&t, ts,
                                              (uint8_t)tcmd_code,
                                              (uint16_t)tcmd_duration_ms,
                                              tcmd_message);
            memcpy(payload, &t, payload_size);
        }

        if (print_struct) {
            fprintf(stderr, "beacon_gen: payload[%d] (%zu bytes): ",
                    n, payload_size);
            for (size_t i = 0; i < payload_size; ++i) {
                fprintf(stderr, "%02x", payload[i]);
            }
            fputc('\n', stderr);
        }

        uint8_t csp_packet[256];
        ssize_t csp_len = csp_v1_encode(&csp_hdr,
                                        payload, payload_size,
                                        csp_packet, sizeof csp_packet);
        if (csp_len < 0) {
            fprintf(stderr, "beacon_gen: csp_v1_encode failed\n");
            free(samples);
            return 1;
        }

        uint8_t frame[512];
        ssize_t frame_len = ax100_frame(csp_packet, (size_t)csp_len,
                                        &opts, frame, sizeof frame);
        if (frame_len < 0) {
            fprintf(stderr, "beacon_gen: ax100_frame failed\n");
            free(samples);
            return 1;
        }
        total_frame_bytes += (size_t)frame_len;

        if (print_frame) {
            for (ssize_t i = 0; i < frame_len; ++i) printf("%02x", frame[i]);
            putchar('\n');
        }

        ssize_t n_samples = modem_bytes_to_pcm16(frame, (size_t)frame_len, &mp,
                                                 samples + total_n,
                                                 total_cap - total_n);
        if (n_samples < 0) {
            fprintf(stderr, "beacon_gen: modem_bytes_to_pcm16 failed\n");
            free(samples);
            return 1;
        }
        total_n += (size_t)n_samples;

        // One gap-of-silence after every beacon (including the last), so
        // a "20 s cadence" file looks like a continuous recording and
        // sliding-window decoders have trailing context past the last burst.
        if (per_gap_samples > 0) {
            if (total_n + per_gap_samples > total_cap) {
                fprintf(stderr, "beacon_gen: gap would overflow buffer\n");
                free(samples);
                return 1;
            }
            memset(samples + total_n, 0, per_gap_samples * sizeof(int16_t));
            total_n += per_gap_samples;
        }
    }

    if (pcm16_write_wav(out_wav, samples, total_n, mp.samp_rate) != 0) {
        free(samples);
        return 1;
    }
    free(samples);

    const char *kind_str = (kind == PACKET_TCMD_RESPONSE)
        ? "tcmd_response packet" : "beacon";
    fprintf(stderr,
            "beacon_gen: wrote %s: %d %s%s, %zu samples @ %d Hz "
            "(%.3f s), %zu frame bytes total\n",
            out_wav, repeats, kind_str, repeats == 1 ? "" : "s",
            total_n, mp.samp_rate,
            (double)total_n / mp.samp_rate, total_frame_bytes);

    return 0;
}
