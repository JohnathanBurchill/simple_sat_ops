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

#include "ax100.h"
#include "beacon_cts1.h"
#include "csp.h"
#include "modem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void usage(FILE *out, const char *argv0)
{
    fprintf(out,
        "usage: %s --out=<file.wav> [options]\n"
        "\n"
        "Fill a CTS1 firmware-canonical downlink packet with deterministic\n"
        "plausible values, frame for the AX100, and write a 48 kHz mono\n"
        "16-bit WAV that decodes round-trip via rx_decode / rx_replay /\n"
        "b210_rx_tx.\n"
        "\n"
        "Required:\n"
        "  --out=<file.wav>          Output WAV path\n"
        "\n"
        "Packet selection:\n"
        "  --type=beacon             COMMS_beacon_basic_packet_t (DEFAULT)\n"
        "  --type=tcmd-response      COMMS_tcmd_response_packet_t (single packet)\n"
        "\n"
        "Beacon options (--type=beacon):\n"
        "  (no per-field overrides in V1; values are deterministic plausibles)\n"
        "\n"
        "TCMD-response options (--type=tcmd-response):\n"
        "  --tcmd-code=N             response_code (default 0 = OK)\n"
        "  --tcmd-duration=N         duration_ms (default 42)\n"
        "  --tcmd-message=<STR>      response body (default \"OK: telecommand\n"
        "                            executed\"; trailing NUL added automatically)\n"
        "\n"
        "Common options:\n"
        "  --repeats=N               Emit N packets in one WAV (default 1)\n"
        "  --gap-seconds=S           Silence between/after packets (default 20,\n"
        "                            matches firmware beacon cadence)\n"
        "  --reed-solomon            RS(255,223) on (DEFAULT)\n"
        "  --no-reed-solomon         RS off (for negative-control testing)\n"
        "  --src=<0..31>             CSP source (default 1, firmware sat)\n"
        "  --dst=<0..31>             CSP destination (default 10, firmware GS)\n"
        "  --dport=<0..63>           CSP destination port (default 10)\n"
        "  --sport=<0..63>           CSP source port (default 10)\n"
        "  --prio=<0..3>             CSP priority (default 3, firmware value)\n"
        "  --flags=<0..255>          CSP flags byte (default 0)\n"
        "  --bit-rate=<bps>          Bit rate (default 9600)\n"
        "  --samp-rate=<hz>          Sample rate (default 48000)\n"
        "  --gauss-bt=<float>        Gaussian BT (default 0.5)\n"
        "  --gauss-span=<symbols>    Gaussian filter span (default 4)\n"
        "  --gain-db=<float>         Output gain dB (default 0)\n"
        "  --print-frame             Hex dump the AX100 frame to stdout\n"
        "  --print-struct            Hex dump the wire payload to stderr\n"
        "  --help                    This message\n",
        argv0);
}

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
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

typedef enum { PACKET_BEACON, PACKET_TCMD_RESPONSE } packet_kind_t;

int main(int argc, char **argv)
{
    const char *out_wav = NULL;
    int repeats = 1;
    int gap_seconds = 20;
    int use_rs = 1;
    int print_frame = 0;
    int print_struct = 0;
    packet_kind_t kind = PACKET_BEACON;
    int tcmd_code = 0;
    int tcmd_duration_ms = 42;
    const char *tcmd_message = "OK: telecommand executed";

    csp_v1_header_t csp_hdr = {
        .prio  = CTS1_BEACON_CSP_PRIO,
        .src   = CTS1_BEACON_CSP_SRC,
        .dst   = CTS1_BEACON_CSP_DST,
        .dport = CTS1_BEACON_CSP_DPORT,
        .sport = CTS1_BEACON_CSP_SPORT,
        .flags = CTS1_BEACON_CSP_FLAGS,
    };

    modem_params_t mp;
    modem_params_defaults(&mp);

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if      (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (starts_with(a, "--out="))            out_wav = a + 6;
        else if (starts_with(a, "--repeats="))        repeats = atoi(a + 10);
        else if (starts_with(a, "--gap-seconds="))    gap_seconds = atoi(a + 14);
        else if (strcmp(a, "--reed-solomon") == 0)    use_rs = 1;
        else if (strcmp(a, "--no-reed-solomon") == 0) use_rs = 0;
        else if (starts_with(a, "--src="))    csp_hdr.src   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dst="))    csp_hdr.dst   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dport="))  csp_hdr.dport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--sport="))  csp_hdr.sport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--prio="))   csp_hdr.prio  = (uint8_t)atoi(a + 7);
        else if (starts_with(a, "--flags="))  csp_hdr.flags = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--bit-rate="))   mp.bit_rate = atoi(a + 11);
        else if (starts_with(a, "--samp-rate="))  mp.samp_rate = atoi(a + 12);
        else if (starts_with(a, "--gauss-bt="))   mp.gauss_bt = atof(a + 11);
        else if (starts_with(a, "--gauss-span=")) mp.gauss_symbol_span = atoi(a + 13);
        else if (starts_with(a, "--gain-db="))    mp.gain_db = atof(a + 10);
        else if (strcmp(a, "--print-frame") == 0)  print_frame = 1;
        else if (strcmp(a, "--print-struct") == 0) print_struct = 1;
        else if (strcmp(a, "--type=beacon") == 0)         kind = PACKET_BEACON;
        else if (strcmp(a, "--type=tcmd-response") == 0)  kind = PACKET_TCMD_RESPONSE;
        else if (starts_with(a, "--tcmd-code="))     tcmd_code = atoi(a + 12);
        else if (starts_with(a, "--tcmd-duration=")) tcmd_duration_ms = atoi(a + 16);
        else if (starts_with(a, "--tcmd-message="))  tcmd_message = a + 15;
        else {
            fprintf(stderr, "beacon_gen: unknown option: %s\n", a);
            usage(stderr, argv[0]);
            return 1;
        }
    }

    if (out_wav == NULL) {
        fprintf(stderr, "beacon_gen: --out=<file.wav> is required\n");
        usage(stderr, argv[0]);
        return 1;
    }
    if (repeats < 1) {
        fprintf(stderr, "beacon_gen: --repeats must be >= 1\n");
        return 1;
    }
    if (gap_seconds < 0) {
        fprintf(stderr, "beacon_gen: --gap-seconds must be >= 0\n");
        return 1;
    }
    if (mp.samp_rate <= 0 || mp.bit_rate <= 0
        || (mp.samp_rate % mp.bit_rate) != 0) {
        fprintf(stderr,
                "beacon_gen: samp_rate (%d) must be a positive multiple of "
                "bit_rate (%d)\n", mp.samp_rate, mp.bit_rate);
        return 1;
    }

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
