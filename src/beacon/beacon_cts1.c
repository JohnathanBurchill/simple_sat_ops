/*

    Simple Satellite Operations  beacon_cts1.c

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

#include "beacon_cts1.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// The EPS firmware reports INT*_MAX in a temperature field when the
// underlying sensor isn't returning a valid reading (on our model the
// first battery thermistor is the typical offender). Treating it as a
// real value gave operators a constant "327.67C" in the beacon line.
static const char *fmt_cC_i16(char *buf, size_t cap, int16_t v)
{
    if (v == INT16_MAX || v == INT16_MIN) snprintf(buf, cap, "n/a");
    else                                  snprintf(buf, cap, "%.2fC", v / 100.0);
    return buf;
}
static const char *fmt_cC_i32(char *buf, size_t cap, int32_t v)
{
    if (v == INT32_MAX || v == INT32_MIN) snprintf(buf, cap, "n/a");
    else                                  snprintf(buf, cap, "%.2fC", v / 100.0);
    return buf;
}

int beacon_is_basic(const uint8_t *payload, size_t len)
{
    // 130 is the bare struct size; 134 is the same with a 4-byte CSP
    // CRC32 trailer (firmware emits the trailer on downlink, and we
    // don't always strip it — --csp-crc32 is opt-in, and even with it
    // on, a corrupt trailer leaves the bytes in the payload). Both are
    // unambiguously beacon territory on this link. Magic bytes are not
    // checked here so corrupted-but-rescued beacons (where packet_type
    // / "CTS1" took bit-flips) still route to the beacon panel rather
    // than slipping into tcmd. beacon_print memcpys exactly the struct
    // size, so the trailing 4 bytes (when present) are harmless.
    if (payload == NULL) return 0;
    return (len == sizeof(COMMS_beacon_basic_packet_t)
         || len == sizeof(COMMS_beacon_basic_packet_t) + 4);
}

static const char *eps_mode_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "STARTUP";
        case 1: return "NOMINAL";
        case 2: return "SAFETY";
        case 3: return "EMERGENCY";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

static const char *cts1_state_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case CTS1_OPERATION_STATE_BOOTED_AND_WAITING:       return "BOOTING";
        case CTS1_OPERATION_STATE_DEPLOYING:                return "DEPLOYING";
        case CTS1_OPERATION_STATE_NOMINAL_WITH_RADIO_TX:    return "NOMINAL_TX";
        case CTS1_OPERATION_STATE_NOMINAL_WITHOUT_RADIO_TX: return "NOMINAL_NO_TX";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// STM32_reset_cause_t from sat-1-rc3:firmware/Core/Inc/stm32/stm32_reboot_reason.h.
// Declaration order = numeric value.
static const char *reboot_reason_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "UNKNOWN";
        case 1: return "LOW_POWER";
        case 2: return "WINDOW_WATCHDOG";
        case 3: return "INDEPENDENT_WATCHDOG";
        case 4: return "SOFTWARE";
        case 5: return "EXTERNAL_PIN";
        case 6: return "BROWNOUT";
        case 7: return "OPTION_BYTE_LOADER";
        case 8: return "FIREWALL";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// OBC_rbf_state_enum_t from sat-1-rc3:firmware/Core/Inc/obc_systems/external_led_and_rbf.h.
static const char *rbf_state_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "BENCH";
        case 1: return "FLYING";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// EPS reset-cause enum, from the comment on eps_reset_cause_enum in
// beacon_cts1.h: 0=power_on, 1=watchdog, 2=commanded,
// 3=control_system_reset, 4=emergency_low_power.
static const char *eps_reset_cause_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "POWER_ON";
        case 1: return "WATCHDOG";
        case 2: return "COMMANDED";
        case 3: return "CTRL_SYS_RESET";
        case 4: return "EMERGENCY_LOW_POWER";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// COMMS_rf_switch_control_mode_enum_t from sat-1-rc3:firmware/Core/Inc/
// comms_drivers/rf_antenna_switch.h. Note 255 = UNKNOWN sentinel used
// by the firmware string-parser for error handling.
static const char *rf_switch_mode_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0:   return "TOGGLE_PER_BEACON";
        case 1:   return "FORCE_ANT1";
        case 2:   return "FORCE_ANT2";
        case 3:   return "ADCS_NORMAL";
        case 4:   return "ADCS_FLIPPED";
        case 255: return "UNKNOWN";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// TIME_sync_source_enum_t from sat-1-rc3:firmware/Core/Inc/timekeeping/
// timekeeping.h. Telemetry-side context: GNSS_PPS is the most precise
// source (1 PPS edge); EPS_RTC is least (1-second resolution).
static const char *time_sync_source_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "NONE";
        case 1: return "GNSS_UART";
        case 2: return "GNSS_PPS";
        case 3: return "TCMD_ABS";
        case 4: return "TCMD_CORR";
        case 5: return "EPS_RTC";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// MPI_rx_mode_enum_t from sat-1-rc3:firmware/Core/Inc/mpi/mpi_types.h.
static const char *mpi_rx_mode_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "COMMAND";
        case 1: return "SENSING";
        case 2: return "NOT_LISTENING";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// MPI_transceiver_state_enum_t from same header.
static const char *mpi_transceiver_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "INACTIVE";
        case 1: return "MOSI";
        case 2: return "MISO";
        case 3: return "DUPLEX";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// MPI_reason_for_stopping_active_mode from sat-1-rc3:firmware/Core/Inc/
// mpi/mpi_command_handling.h. NOT_SET means the MPI hasn't been
// stopped since boot (or hasn't been active yet).
static const char *mpi_stop_reason_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "NOT_SET";
        case 1: return "TEMP_EXCEEDED";
        case 2: return "TELECOMMAND";
        case 3: return "MAX_TIME_EXCEEDED";
        case 4: return "SELF_CHECK_DONE";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// GNSS_rx_mode_enum_t from sat-1-rc3:firmware/Core/Inc/gnss_receiver/
// gnss_internal_drivers.h.
static const char *gnss_rx_mode_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "COMMAND";
        case 1: return "FIREHOSE";
        case 2: return "DISABLED";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// Format duration in milliseconds as HH:MM:SS.mmm. uint64 input handles
// the uint32 ms range (~49 days) without overflow at the * 1000 step.
static void fmt_ms_clock(uint64_t ms_in, char *out, size_t outn)
{
    uint64_t total_s = ms_in / 1000;
    uint64_t ms = ms_in % 1000;
    uint64_t s = total_s % 60;
    uint64_t m = (total_s / 60) % 60;
    uint64_t h = total_s / 3600;
    snprintf(out, outn, "%02llu:%02llu:%02llu.%03llu",
             (unsigned long long)h, (unsigned long long)m,
             (unsigned long long)s, (unsigned long long)ms);
}

// Format Unix epoch ms as ISO-8601 UTC at second precision. Sub-second
// is rarely useful in a downlink log and the uptime line already shows ms.
static void fmt_epoch_ms(uint64_t ms_in, char *out, size_t outn)
{
    time_t t = (time_t)(ms_in / 1000);
    struct tm tm;
    if (gmtime_r(&t, &tm) == NULL) {
        snprintf(out, outn, "?");
        return;
    }
    strftime(out, outn, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

void beacon_print(FILE *fp, const char *ts,
                  const uint8_t *payload, size_t len)
{
    (void)len;
    // memcpy into a stack-allocated struct so this works regardless of
    // payload alignment in the caller.
    COMMS_beacon_basic_packet_t b;
    memcpy(&b, payload, sizeof b);

    // ts == NULL -> flat output ("beacon: ...") for callers like rx_decode
    // that don't decorate lines with timestamps. ts != NULL -> "[ts] beacon: ..."
    // matching decode_loop's emit_frame style.
    char prefix[64];
    if (ts != NULL) snprintf(prefix, sizeof prefix, "[%s] ", ts);
    else prefix[0] = '\0';

    char eps_mode_buf[8], state_buf[8];
    char uptime_buf[24], since_uplink_buf[24], epoch_buf[32];
    fmt_ms_clock(b.uptime_ms, uptime_buf, sizeof uptime_buf);
    fmt_ms_clock(b.duration_since_last_uplink_ms,
                 since_uplink_buf, sizeof since_uplink_buf);
    fmt_epoch_ms(b.unix_epoch_time_ms, epoch_buf, sizeof epoch_buf);

    // satellite_name is exactly 4 bytes "CTS1" with no terminator.
    fprintf(fp,
            "%sbeacon: name=\"%.4s\" state=%s eps_mode=%s fs_mounted=%u count=%u\n",
            prefix, b.satellite_name,
            cts1_state_str(b.cts1_operation_state, state_buf, sizeof state_buf),
            eps_mode_str(b.eps_mode_enum, eps_mode_buf, sizeof eps_mode_buf),
            (unsigned)b.is_fs_mounted,
            (unsigned)b.total_beacon_count_since_boot);

    fprintf(fp,
            "%sbeacon: uptime=%s since_uplink=%s epoch=%s\n",
            prefix, uptime_buf, since_uplink_buf, epoch_buf);

    char t0[16], t1[16], obc[16];
    fprintf(fp,
            "%sbeacon: batt=%.3fV %u%% temps=%s/%s obc=%s\n",
            prefix,
            b.eps_battery_voltage_mV / 1000.0,
            (unsigned)b.eps_battery_percent,
            fmt_cC_i16(t0,  sizeof t0,  b.eps_battery_temperature_0_cC),
            fmt_cC_i16(t1,  sizeof t1,  b.eps_battery_temperature_1_cC),
            fmt_cC_i32(obc, sizeof obc, b.obc_temperature_cC));

    fprintf(fp,
            "%sbeacon: pcu in/out=%.2fW/%.2fW avg=%.2fW/%.2fW faults=%d channels=0x%x\n",
            prefix,
            b.eps_total_pcu_power_input_cW / 100.0,
            b.eps_total_pcu_power_output_cW / 100.0,
            b.eps_total_avg_pcu_power_input_cW / 100.0,
            b.eps_total_avg_pcu_power_output_cW / 100.0,
            (int)b.eps_total_fault_count,
            (unsigned)b.eps_enabled_channels_bitfield);

    char reboot_buf[8], rbf_buf[8], eps_reset_buf[8];
    fprintf(fp,
            "%sbeacon: tcmd queued=%u pending=%u reboot=%s eps_reset=%s rbf=%s antenna=%u\n",
            prefix,
            (unsigned)b.total_tcmd_queued_count,
            (unsigned)b.pending_queued_tcmd_count,
            reboot_reason_str(b.reboot_reason, reboot_buf, sizeof reboot_buf),
            eps_reset_cause_str(b.eps_reset_cause_enum,
                                eps_reset_buf, sizeof eps_reset_buf),
            rbf_state_str(b.rbf_pin_state, rbf_buf, sizeof rbf_buf),
            (unsigned)b.active_rf_switch_antenna);

    char rf_buf[8], time_buf[8], gnss_buf[8];
    fprintf(fp,
            "%sbeacon: rf_switch=%s time_sync=%s gnss=%s\n",
            prefix,
            rf_switch_mode_str(b.active_rf_switch_control_mode,
                               rf_buf, sizeof rf_buf),
            time_sync_source_str(b.last_time_sync_source_enum,
                                 time_buf, sizeof time_buf),
            gnss_rx_mode_str(b.gnss_rx_mode_enum, gnss_buf, sizeof gnss_buf));

    char mpi_rx_buf[8], mpi_tx_buf[8], mpi_stop_buf[8];
    fprintf(fp,
            "%sbeacon: mpi rx=%s tx=%s last_stop=%s\n",
            prefix,
            mpi_rx_mode_str(b.mpi_rx_mode_enum,
                            mpi_rx_buf, sizeof mpi_rx_buf),
            mpi_transceiver_str(b.mpi_transceiver_state_enum,
                                mpi_tx_buf, sizeof mpi_tx_buf),
            mpi_stop_reason_str(b.mpi_last_reason_for_stopping_enum,
                                mpi_stop_buf, sizeof mpi_stop_buf));

    // friendly_message may not be NUL-terminated within its 42 bytes;
    // strnlen-bounded copy into a 43-byte buffer guarantees printf safety.
    char msg[COMMS_BEACON_FRIENDLY_MESSAGE_SIZE + 1];
    size_t mlen = strnlen(b.friendly_message, COMMS_BEACON_FRIENDLY_MESSAGE_SIZE);
    memcpy(msg, b.friendly_message, mlen);
    msg[mlen] = '\0';
    fprintf(fp, "%sbeacon: msg=\"%s\"\n", prefix, msg);
}

int beacon_basic_summary(const uint8_t *payload, size_t len,
                         char *out, size_t out_size)
{
    if (!out || out_size == 0) return 0;
    out[0] = '\0';
    if (!beacon_is_basic(payload, len)) return 0;
    COMMS_beacon_basic_packet_t b;
    memcpy(&b, payload, sizeof b);
    // cts1_state_str / eps_mode_str return a literal for known enum
    // values and only fall through to the snprintf-into-buf path for
    // unknown values, so the call's return value is the authoritative
    // string. Capture it; using the buffer directly leaves it empty in
    // the common (known-enum) case.
    char state_buf[8], eps_buf[8], obc_buf[16], up_buf[24];
    const char *state_str = cts1_state_str(b.cts1_operation_state,
                                           state_buf, sizeof state_buf);
    const char *eps_str   = eps_mode_str  (b.eps_mode_enum,
                                           eps_buf,   sizeof eps_buf);
    fmt_cC_i32    (obc_buf, sizeof obc_buf, b.obc_temperature_cC);
    fmt_ms_clock  (b.uptime_ms,             up_buf,    sizeof up_buf);
    // Sanitised friendly_message — non-printables become '.' so a
    // single bad byte doesn't break terminal rendering.
    char msg[COMMS_BEACON_FRIENDLY_MESSAGE_SIZE + 1];
    cts1_sanitise_text((const uint8_t *) b.friendly_message,
                       COMMS_BEACON_FRIENDLY_MESSAGE_SIZE,
                       msg, sizeof msg, NULL);
    int n = snprintf(out, out_size,
        "%.4s st=%s eps=%s batt=%.2fV/%u%% obc=%s up=%s cnt=%u \"%s\"",
        b.satellite_name, state_str, eps_str,
        b.eps_battery_voltage_mV / 1000.0,
        (unsigned) b.eps_battery_percent,
        obc_buf, up_buf,
        (unsigned) b.total_beacon_count_since_boot,
        msg);
    if (n < 0) { out[0] = '\0'; return 0; }
    return (n < (int) out_size) ? n : (int) out_size - 1;
}

int tcmd_response_is(const uint8_t *payload, size_t len)
{
    if (payload == NULL) return 0;
    // 130 / 134 bytes are unambiguously beacon territory on this link
    // (134 = beacon + 4-byte CSP CRC32 trailer when --csp-crc32 isn't
    // active). Without this short-circuit, a noisy beacon whose
    // packet_type drifted from 0x01 -> 0x04 (just two bit-flips apart)
    // and whose bytes 12/13 are small naturals would slip past the
    // seq/max_seq gate and render in the tcmd column.
    if (len == sizeof(COMMS_beacon_basic_packet_t)
     || len == sizeof(COMMS_beacon_basic_packet_t) + 4) return 0;
    if (len < COMMS_TCMD_RESPONSE_HEADER_SIZE + 1) return 0;
    if (len > COMMS_TCMD_RESPONSE_HEADER_SIZE
              + COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET) return 0;
    // Require exact packet_type match. The earlier 3-bit tolerance was
    // safe when only 0x01/0x02/0x04 were defined, but log_message (0x03)
    // and bulk_file (0x10) both land within 3 bits of 0x04, so any
    // tolerance here misclassifies them as tcmd. Severely corrupted
    // tcmd packets won't decode cleanly via this path; the operator
    // can fall back to --packet-headers and inspect the raw bytes.
    if (payload[0] != COMMS_PACKET_TYPE_TCMD_RESPONSE) return 0;
    uint8_t seq = payload[12];
    uint8_t max_seq = payload[13];
    if (seq < 1 || max_seq < 1 || seq > max_seq) return 0;
    // Drop the strict equality (data_len == 186) and trailing-NUL checks —
    // both are tail-end byte tests that a single bit error trivially
    // breaks. The seq/max_seq range is enough to gate false positives.
    return 1;
}

void tcmd_response_print(FILE *fp, const char *ts,
                         const uint8_t *payload, size_t len)
{
    char prefix[64];
    if (ts != NULL) snprintf(prefix, sizeof prefix, "[%s] ", ts);
    else prefix[0] = '\0';

    // memcpy header into a struct so unaligned reads work portably.
    COMMS_tcmd_response_packet_t hdr;
    memcpy(&hdr, payload, COMMS_TCMD_RESPONSE_HEADER_SIZE);

    char ts_buf[32];
    fmt_epoch_ms(hdr.ts_sent, ts_buf, sizeof ts_buf);

    size_t data_len = len - COMMS_TCMD_RESPONSE_HEADER_SIZE;
    // Strip trailing NULs on the last packet for cleaner display; the raw
    // hex/ascii dump below still shows the wire-exact bytes for inspection.
    size_t print_len = data_len;
    if (hdr.response_seq_num == hdr.response_max_seq_num) {
        while (print_len > 0
               && payload[COMMS_TCMD_RESPONSE_HEADER_SIZE + print_len - 1] == 0x00) {
            print_len--;
        }
    }

    fprintf(fp,
            "%stcmd_response: code=%u%s duration=%ums seq=%u/%u ts_sent=%s\n",
            prefix,
            (unsigned)hdr.response_code,
            (hdr.response_code == 0) ? " (OK)" : "",
            (unsigned)hdr.duration_ms,
            (unsigned)hdr.response_seq_num,
            (unsigned)hdr.response_max_seq_num,
            ts_buf);

    fprintf(fp, "%stcmd_response: data (%zu bytes): \"", prefix, data_len);
    for (size_t i = 0; i < print_len; i++) {
        uint8_t b = payload[COMMS_TCMD_RESPONSE_HEADER_SIZE + i];
        char c = (b >= 0x20 && b < 0x7F) ? (char)b : '.';
        fputc(c, fp);
    }
    fprintf(fp, "\"\n");
}

int tcmd_response_summary(const uint8_t *payload, size_t len,
                          char *out, size_t out_size)
{
    if (!out || out_size == 0) return 0;
    out[0] = '\0';
    if (!tcmd_response_is(payload, len)) return 0;
    COMMS_tcmd_response_packet_t hdr;
    memcpy(&hdr, payload, COMMS_TCMD_RESPONSE_HEADER_SIZE);
    size_t data_len  = len - COMMS_TCMD_RESPONSE_HEADER_SIZE;
    size_t print_len = data_len;
    if (hdr.response_seq_num == hdr.response_max_seq_num) {
        while (print_len > 0
               && payload[COMMS_TCMD_RESPONSE_HEADER_SIZE + print_len - 1] == 0x00) {
            print_len--;
        }
    }
    char text[96];
    cts1_sanitise_text(payload + COMMS_TCMD_RESPONSE_HEADER_SIZE,
                       print_len, text, sizeof text, NULL);
    int n = snprintf(out, out_size,
                     "[%u/%u] code=%u%s '%s'",
                     (unsigned) hdr.response_seq_num,
                     (unsigned) hdr.response_max_seq_num,
                     (unsigned) hdr.response_code,
                     (hdr.response_code == 0) ? "(OK)" : "",
                     text);
    if (n < 0) { out[0] = '\0'; return 0; }
    return (n < (int) out_size) ? n : (int) out_size - 1;
}

size_t cts1_sanitise_text(const uint8_t *data, size_t data_len,
                          char *out, size_t outn,
                          int *out_truncated)
{
    if (outn == 0) {
        if (out_truncated) *out_truncated = data_len > 0;
        return 0;
    }
    while (data_len > 0 && data[data_len - 1] == 0x00) data_len--;
    size_t cap = outn - 1;
    size_t copy = data_len < cap ? data_len : cap;
    for (size_t i = 0; i < copy; i++) {
        uint8_t b = data[i];
        out[i] = (b >= 0x20 && b < 0x7F) ? (char)b : '.';
    }
    out[copy] = '\0';
    if (out_truncated) *out_truncated = (data_len > cap) ? 1 : 0;
    return copy;
}

int log_message_is(const uint8_t *payload, size_t len)
{
    if (payload == NULL) return 0;
    if (len < 2 || len > AX100_DOWNLINK_MAX_BYTES) return 0;
    if (payload[0] != COMMS_PACKET_TYPE_LOG_MESSAGE) return 0;
    // Beacon and tcmd_response have stronger anchors at their own lengths;
    // route those out first so a corrupted log packet at length 130/134
    // doesn't get reclassified as a log message.
    if (beacon_is_basic(payload, len)) return 0;
    // Require at least one printable byte in the first 8 data bytes —
    // filters out pure-binary frames that happen to start with 0x03.
    size_t scan = len - 1;
    if (scan > 8) scan = 8;
    int printable = 0;
    for (size_t i = 0; i < scan; i++) {
        uint8_t b = payload[1 + i];
        if (b >= 0x20 && b < 0x7F) { printable = 1; break; }
    }
    return printable;
}

void log_message_print(FILE *fp, const char *ts,
                       const uint8_t *payload, size_t len)
{
    char prefix[64];
    if (ts != NULL) snprintf(prefix, sizeof prefix, "[%s] ", ts);
    else prefix[0] = '\0';

    size_t data_len = (len > 0) ? len - 1 : 0;
    char text[COMMS_LOG_MESSAGE_PACKET_MAX_DATA_BYTES_PER_PACKET + 1];
    int truncated = 0;
    size_t shown = cts1_sanitise_text(payload + 1, data_len,
                                      text, sizeof text, &truncated);

    fprintf(fp, "%slog: \"%s\"%s (%zu bytes)\n",
            prefix, text, truncated ? "..." : "", shown);
}

int log_message_summary(const uint8_t *payload, size_t len,
                        char *out, size_t out_size)
{
    if (!out || out_size == 0) return 0;
    out[0] = '\0';
    if (!log_message_is(payload, len)) return 0;
    size_t data_len = (len > 0) ? len - 1 : 0;
    char text[COMMS_LOG_MESSAGE_PACKET_MAX_DATA_BYTES_PER_PACKET + 1];
    int truncated = 0;
    cts1_sanitise_text(payload + 1, data_len,
                       text, sizeof text, &truncated);
    int n = snprintf(out, out_size, "'%s'%s", text, truncated ? "..." : "");
    if (n < 0) { out[0] = '\0'; return 0; }
    return (n < (int) out_size) ? n : (int) out_size - 1;
}

int bulk_file_is(const uint8_t *payload, size_t len)
{
    if (payload == NULL) return 0;
    if (len < COMMS_BULK_FILE_DOWNLINK_PACKET_HEADER_SIZE + 1) return 0;
    if (len > AX100_DOWNLINK_MAX_BYTES) return 0;
    if (payload[0] != COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK) return 0;
    if (beacon_is_basic(payload, len)) return 0;
    return 1;
}

void bulk_file_print(FILE *fp, const char *ts,
                     const uint8_t *payload, size_t len)
{
    char prefix[64];
    if (ts != NULL) snprintf(prefix, sizeof prefix, "[%s] ", ts);
    else prefix[0] = '\0';

    // STM32 is little-endian; firmware writes the uint32_t directly via
    // packed struct, so the wire bytes are LSB-first.
    uint32_t offset = (uint32_t)payload[1]
                    | ((uint32_t)payload[2] << 8)
                    | ((uint32_t)payload[3] << 16)
                    | ((uint32_t)payload[4] << 24);
    size_t data_len = len - COMMS_BULK_FILE_DOWNLINK_PACKET_HEADER_SIZE;

    fprintf(fp, "%sbulk_file: offset=0x%08x bytes=%zu preview=",
            prefix, offset, data_len);
    size_t preview = data_len < 16 ? data_len : 16;
    for (size_t i = 0; i < preview; i++) {
        fprintf(fp, "%02x", payload[COMMS_BULK_FILE_DOWNLINK_PACKET_HEADER_SIZE + i]);
    }
    if (data_len > preview) fprintf(fp, "...");
    fputc('\n', fp);
}

void cts1_packet_print(FILE *fp, const char *ts,
                       const uint8_t *payload, size_t len)
{
    if (payload == NULL || len == 0) return;
    // Length-anchored sniffs first: beacon at 130/134 and tcmd_response
    // both survive a corrupted packet_type byte because their other
    // anchors (length, seq/max_seq) are strong enough.
    if (beacon_is_basic(payload, len)) {
        beacon_print(fp, ts, payload, len);
        return;
    }
    if (tcmd_response_is(payload, len)) {
        tcmd_response_print(fp, ts, payload, len);
        return;
    }
    // Fall through to packet_type for the two types whose lengths
    // overlap with each other and with arbitrary noise.
    if (log_message_is(payload, len)) {
        log_message_print(fp, ts, payload, len);
        return;
    }
    if (bulk_file_is(payload, len)) {
        bulk_file_print(fp, ts, payload, len);
        return;
    }
    // Unknown type — caller's hex/ascii dump (when packetheaders is on)
    // is the only output. Stay silent here.
}
