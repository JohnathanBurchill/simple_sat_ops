/*

    Simple Satellite Operations  beacon_cts1.h

    Vendored copy of the CTS1 OBC beacon packet struct + the small set of
    constants needed to detect, parse, and pretty-print it on the ground.

    Source of truth:
      https://github.com/CalgaryToSpace/CTS-SAT-1-OBC-Firmware
      firmware/Core/Inc/comms_drivers/comms_tx.h          (struct + COMMS_packet_type)
      firmware/Core/Inc/rtos_tasks/rtos_bootup_operation_fsm_task.h
                                                          (CTS1_operation_state values)
      firmware/Core/Src/comms_drivers/ax100_tx.c          (CSP header values)

    Snapshot taken at firmware tag sat-1-rc3 (the flight image). Read with
    `git show sat-1-rc3:firmware/Core/Inc/comms_drivers/comms_tx.h`.
    Do NOT track the `main` branch — it has post-rc3 churn that diverges
    from what the satellite actually runs (e.g. BULK_FILE_DOWNLINK gained
    seq fields on main that the flight build does not transmit). Re-vendor
    only when a new RC tag is cut and replaces rc3 as the flight image.

    If the firmware adds or reorders fields, the _Static_assert below
    catches the size delta; subtler reorders (same size, different
    layout) will silently produce wrong values and need a manual resync.

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

#ifndef BEACON_CTS1_H
#define BEACON_CTS1_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// Packet-type byte (first byte after the CSP header). Values from
// COMMS_packet_type_enum_t in the firmware.
#define COMMS_PACKET_TYPE_BEACON_BASIC      0x01
#define COMMS_PACKET_TYPE_BEACON_PERIPHERAL 0x02
#define COMMS_PACKET_TYPE_LOG_MESSAGE       0x03
#define COMMS_PACKET_TYPE_TCMD_RESPONSE     0x04
#define COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK 0x10

#define COMMS_BEACON_FRIENDLY_MESSAGE_SIZE 42

// AX100_DOWNLINK_MAX_BYTES is 200 in the firmware (firmware/Core/Inc/
// comms_drivers/ax100_tx.h). Every non-beacon downlink packet is exactly
// this size on the wire; the data field is sized so the whole struct
// equals 200 bytes. The on-wire data length is shorter when the firmware
// only filled part of the buffer (the rest is uninitialised padding).
#define AX100_DOWNLINK_MAX_BYTES 200

// TCMD-response packet layout. Header is 1+8+1+2+1+1 = 14 bytes, so
// per-packet data tops out at 186. Long responses are split across
// multiple packets, indexed via response_seq_num (1..max) /
// response_max_seq_num.
#define COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET 186
#define COMMS_TCMD_RESPONSE_HEADER_SIZE 14

// LOG_MESSAGE: 1-byte packet_type + up to 199 bytes of ASCII text
// (NUL-terminated; firmware sends strnlen+1).
#define COMMS_LOG_MESSAGE_PACKET_MAX_DATA_BYTES_PER_PACKET 199

// BULK_FILE_DOWNLINK at sat-1-rc3: 1-byte packet_type + 4-byte file_offset
// + 195 bytes of file data. No seq_num / max_seq_num on the wire — the
// receiver reassembles by offset. (Post-rc3 main adds seq fields; we are
// pinned to rc3.)
#define COMMS_BULK_FILE_DOWNLINK_PACKET_HEADER_SIZE 5
#define COMMS_BULK_FILE_DOWNLINK_PACKET_MAX_DATA_BYTES_PER_PACKET 195

// CTS1_operation_state_enum_t values (declaration order = numeric value)
#define CTS1_OPERATION_STATE_BOOTED_AND_WAITING       0
#define CTS1_OPERATION_STATE_DEPLOYING                1
#define CTS1_OPERATION_STATE_NOMINAL_WITH_RADIO_TX    2
#define CTS1_OPERATION_STATE_NOMINAL_WITHOUT_RADIO_TX 3

// CSP header values used by the firmware AX100_downlink_bytes path
// (firmware/Core/Src/comms_drivers/ax100_tx.c).
#define CTS1_BEACON_CSP_PRIO  3
#define CTS1_BEACON_CSP_SRC   1
#define CTS1_BEACON_CSP_DST   10
#define CTS1_BEACON_CSP_DPORT 10
#define CTS1_BEACON_CSP_SPORT 10
#define CTS1_BEACON_CSP_FLAGS 0

#pragma pack(push, 1)

// Verbatim from firmware/Core/Inc/comms_drivers/comms_tx.h. Field
// comments preserved as-is so future resyncs are line-comparable.
typedef struct {
    uint8_t packet_type; // COMMS_packet_type_enum_t - Always COMMS_PACKET_TYPE_BEACON_BASIC for this packet

    char satellite_name[4]; // 4 bytes: "CTS1" :)

    uint8_t active_rf_switch_antenna; // Either 1 or 2.
    uint8_t active_rf_switch_control_mode; // Enum: COMMS_rf_switch_control_mode_enum_t
    uint32_t uptime_ms;

    uint32_t duration_since_last_uplink_ms;
    uint64_t unix_epoch_time_ms;
    uint8_t last_time_sync_source_enum; // Enum: TIME_sync_source_enum_t

    uint8_t is_fs_mounted;

    uint16_t total_tcmd_queued_count;
    uint16_t pending_queued_tcmd_count;

    uint32_t total_beacon_count_since_boot;

    uint8_t eps_mode_enum; // 0=startup, 1=nominal, 2=safety, 3=emergency_low_power
    uint8_t eps_reset_cause_enum; // 0=power_on, 1=watchdog, 2=commanded, 3=control_system_reset, 4=emergency_low_power
    uint32_t eps_uptime_sec;
    uint16_t eps_error_code;
    uint16_t eps_battery_voltage_mV;
    uint8_t eps_battery_percent;
    int16_t eps_battery_temperature_0_cC;
    int16_t eps_battery_temperature_1_cC;
    // Note: Third battery temperature sensor doesn't work on our model.
    int32_t eps_total_fault_count;
    uint32_t eps_enabled_channels_bitfield;
    int32_t eps_total_pcu_power_input_cW;
    int32_t eps_total_pcu_power_output_cW;
    int32_t eps_total_avg_pcu_power_input_cW;
    int32_t eps_total_avg_pcu_power_output_cW;

    int32_t obc_temperature_cC;

    uint8_t reboot_reason; // Enum: STM32_reset_cause_t

    uint8_t cts1_operation_state; // Enum: CTS1_operation_state_enum_t
    uint8_t rbf_pin_state; // Enum: OBC_rbf_state_enum_t

    uint8_t mpi_rx_mode_enum; // Enum: MPI_rx_mode_enum_t
    uint8_t mpi_transceiver_state_enum; // Enum: MPI_current_transceiver_state_enum_t

    uint8_t mpi_last_reason_for_stopping_enum; // Enum: MPI_reason_for_stopping_active_mode_enum_t

    uint8_t gnss_uart_interrupt_enabled;

    uint8_t gnss_rx_mode_enum; // Enum: GNSS_rx_mode_enum_t

    char friendly_message[COMMS_BEACON_FRIENDLY_MESSAGE_SIZE];

    char end_message[4]; // "END\0"

} COMMS_beacon_basic_packet_t;

#pragma pack(pop)

_Static_assert(sizeof(COMMS_beacon_basic_packet_t) == 130,
               "beacon struct must be 130 packed bytes; check pragma pack and field types");

#pragma pack(push, 1)

// Verbatim from firmware/Core/Inc/comms_drivers/comms_tx.h. The data
// member is the full 186-byte buffer; on the wire only the first
// (packet_size - 14) bytes are valid (the rest is uninitialized).
typedef struct {
    uint8_t packet_type; // COMMS_packet_type_enum_t - Always COMMS_PACKET_TYPE_TCMD_RESPONSE for this packet

    uint64_t ts_sent;        // 8 bytes
    uint8_t response_code;   // 1 byte
    uint16_t duration_ms;    // 2 bytes
    uint8_t response_seq_num; // 1 byte
    uint8_t response_max_seq_num;   // 1 byte

    uint8_t data[COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET];
} COMMS_tcmd_response_packet_t;

#pragma pack(pop)

_Static_assert(sizeof(COMMS_tcmd_response_packet_t) == 200,
               "tcmd_response struct must be 200 packed bytes (14 header + 186 data)");

#pragma pack(push, 1)

// Verbatim from sat-1-rc3 firmware/Core/Inc/comms_drivers/comms_tx.h.
// On-wire length is variable; the firmware emits packet_type + the
// log message (NUL-terminated, strnlen+1 bytes). The data field's full
// 199 bytes is the upper bound — most messages are far shorter.
typedef struct {
    uint8_t packet_type; // Always COMMS_PACKET_TYPE_LOG_MESSAGE for this packet
    uint8_t data[COMMS_LOG_MESSAGE_PACKET_MAX_DATA_BYTES_PER_PACKET];
} COMMS_log_message_packet_t;

#pragma pack(pop)

_Static_assert(sizeof(COMMS_log_message_packet_t) == 200,
               "log_message struct must be 200 packed bytes (1 header + 199 data)");

#pragma pack(push, 1)

// Verbatim from sat-1-rc3 firmware/Core/Inc/comms_drivers/comms_tx.h.
// rc3 packs only packet_type + offset + data. The seq_num / max_seq_num
// fields seen on main are NOT in the flight build — reassembly is
// offset-driven only.
typedef struct {
    uint8_t packet_type; // Always COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK
    uint32_t file_offset; // STM32 native = little-endian
    uint8_t data[COMMS_BULK_FILE_DOWNLINK_PACKET_MAX_DATA_BYTES_PER_PACKET];
} COMMS_bulk_file_downlink_packet_t;

#pragma pack(pop)

_Static_assert(sizeof(COMMS_bulk_file_downlink_packet_t) == 200,
               "bulk_file struct must be 200 packed bytes (5 header + 195 data)");

// Sniff test: returns 1 if the bytes look like a basic beacon packet.
// Three magic anchors (length, packet_type, "CTS1" name, "END\0" trailer)
// keep the false-positive rate on random bytes around 2^-72.
int beacon_is_basic(const uint8_t *payload, size_t len);

// Format the beacon as six lines of "[ts] beacon: ..." to fp.
// Caller has already verified beacon_is_basic; this function does no
// further validation (it memcpy's the bytes into a stack-allocated
// COMMS_beacon_basic_packet_t to handle any unaligned source pointer).
// ts is the timestamp string the caller picks — UTC wall clock for
// rx_live, file-relative "t=NN.NNNs" for rx_replay. Pass NULL for
// flat output (no "[ts] " prefix), e.g. when the caller already
// formats lines without timestamps.
void beacon_print(FILE *fp, const char *ts,
                  const uint8_t *payload, size_t len);

// One-line compact summary of the basic beacon for the operator's
// RX panel. Writes "CTS1 st=ARMED eps=NORMAL batt=8.12V/87%
// obc=15.2C up=1d2h cnt=12345 \"<msg>\"" or similar. Returns the
// number of bytes written (excluding the nul terminator), or 0 if
// the payload doesn't sniff as a basic beacon. Caller must have
// already accepted the dispatch packet (payload[0] == 0x01).
int beacon_basic_summary(const uint8_t *payload, size_t len,
                         char *out, size_t out_size);

// Sniff test for the TCMD-response packet. Strong anchors:
//   - len in [15, 200] (header + 1..186 data bytes)
//   - payload[0] == 0x04 (COMMS_PACKET_TYPE_TCMD_RESPONSE)
//   - 1 <= response_seq_num <= response_max_seq_num
//   - intermediate packets (seq < max) carry exactly 186 data bytes
//   - last packet (seq == max) ends with NUL (firmware sends strnlen+1)
int tcmd_response_is(const uint8_t *payload, size_t len);

// Format the tcmd response as two lines: header info + the data bytes
// rendered as a sanitised string. Trailing NUL on the last packet is
// stripped before display. ts semantics match beacon_print.
void tcmd_response_print(FILE *fp, const char *ts,
                         const uint8_t *payload, size_t len);

// One-line summary of the tcmd response — "[seq/max] code=N
// 'text...'" — for the RX panel. Returns bytes written or 0 if not
// a tcmd response.
int tcmd_response_summary(const uint8_t *payload, size_t len,
                          char *out, size_t out_size);

// Sniff for the LOG_MESSAGE packet. Anchors:
//   - 2 <= len <= 200 (1 header + at least 1 data byte)
//   - payload[0] == 0x03 (COMMS_PACKET_TYPE_LOG_MESSAGE), tolerant of
//     up to 3 bit errors so partial-RS rescues still route correctly.
//   - At least one printable byte in the first 8 of data (filters out
//     pure-binary noise that happens to land on packet_type 0x03).
int log_message_is(const uint8_t *payload, size_t len);

// Format the log message as one line: "[ts] log: \"<text>\"".
// NUL bytes at the tail are stripped; non-printable bytes inside become
// '.' so a single bad byte doesn't break terminal rendering.
void log_message_print(FILE *fp, const char *ts,
                       const uint8_t *payload, size_t len);

// One-line summary of the log message — "'<text>'" — for the RX
// panel. Returns bytes written or 0 if not a log message.
int log_message_summary(const uint8_t *payload, size_t len,
                        char *out, size_t out_size);

// Sniff for the BULK_FILE_DOWNLINK packet. Anchors:
//   - len >= 6 (header + at least 1 data byte)
//   - len <= 200 (AX100_DOWNLINK_MAX_BYTES)
//   - payload[0] == 0x10 (COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK)
// File data is arbitrary binary, so there's no further structural
// anchor — packet_type alone gates this one.
int bulk_file_is(const uint8_t *payload, size_t len);

// Format the bulk-file packet as one line:
//   "[ts] bulk_file: offset=0x<8 hex> bytes=<N> preview=<first 16 hex>"
// Operator reassembles the stream by offset; full payload is available
// via the `packetheaders on` toggle (which restores hex/ascii dumps).
void bulk_file_print(FILE *fp, const char *ts,
                     const uint8_t *payload, size_t len);

// Type-aware dispatcher. Tries the length-based beacon/tcmd_response
// sniffs first (those are robust to a corrupted packet_type byte
// because their lengths are unique enough), then falls back to a
// packet_type byte switch for log_message / bulk_file (whose lengths
// overlap). Prints nothing if no decoder matches; the caller's hex/
// ascii dump (gated by the headers toggle) still shows the bytes.
void cts1_packet_print(FILE *fp, const char *ts,
                       const uint8_t *payload, size_t len);

// Sanitise a byte buffer into a printable C string: strip trailing
// NULs, replace any other non-printable byte with '.', clip to outn-1.
// Reused by log_message_print and the rx_tui tcmd-response panel so
// the streaming text path and the curses panel render text the same
// way. Returns the length written to out (excluding the trailing NUL).
size_t cts1_sanitise_text(const uint8_t *data, size_t data_len,
                          char *out, size_t outn,
                          int *out_truncated);

#endif // BEACON_CTS1_H
