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
#include <math.h>
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

// The lines for the 130 bytes the basic and the extended beacon share.
// label is the word each line is tagged with -- "beacon" or
// "beacon_ext" -- so an extended beacon's block reads as its own rather
// than as a basic beacon with extra lines after it.
static void beacon_print_common(FILE *fp, const char *prefix,
                                const char *label,
                                const COMMS_beacon_basic_packet_t *bp)
{
    const COMMS_beacon_basic_packet_t b = *bp;
    char eps_mode_buf[8], state_buf[8];
    char uptime_buf[24], since_uplink_buf[24], epoch_buf[32];
    fmt_ms_clock(b.uptime_ms, uptime_buf, sizeof uptime_buf);
    fmt_ms_clock(b.duration_since_last_uplink_ms,
                 since_uplink_buf, sizeof since_uplink_buf);
    fmt_epoch_ms(b.unix_epoch_time_ms, epoch_buf, sizeof epoch_buf);

    // satellite_name is exactly 4 bytes "CTS1" with no terminator. On a
    // corrupted frame those bytes can be anything, so sanitise them the way
    // the message field below is: a raw control byte (e.g. 0x0E) sent to a
    // terminal flips it into line-drawing mode and garbles all that follows.
    char name_buf[5];
    cts1_sanitise_text((const uint8_t *) b.satellite_name, 4,
                       name_buf, sizeof name_buf, NULL);
    fprintf(fp,
            "%s%s: name=\"%s\" state=%s eps_mode=%s fs_mounted=%u count=%u\n",
            prefix, label, name_buf,
            cts1_state_str(b.cts1_operation_state, state_buf, sizeof state_buf),
            eps_mode_str(b.eps_mode_enum, eps_mode_buf, sizeof eps_mode_buf),
            (unsigned)b.is_fs_mounted,
            (unsigned)b.total_beacon_count_since_boot);

    fprintf(fp,
            "%s%s: uptime=%s since_uplink=%s epoch=%s\n",
            prefix, label, uptime_buf, since_uplink_buf, epoch_buf);

    char t0[16], t1[16], obc[16];
    fprintf(fp,
            "%s%s: batt=%.3fV %u%% temps=%s/%s obc=%s\n",
            prefix, label,
            b.eps_battery_voltage_mV / 1000.0,
            (unsigned)b.eps_battery_percent,
            fmt_cC_i16(t0,  sizeof t0,  b.eps_battery_temperature_0_cC),
            fmt_cC_i16(t1,  sizeof t1,  b.eps_battery_temperature_1_cC),
            fmt_cC_i32(obc, sizeof obc, b.obc_temperature_cC));

    fprintf(fp,
            "%s%s: pcu in/out=%.2fW/%.2fW avg=%.2fW/%.2fW faults=%d channels=0x%x\n",
            prefix, label,
            b.eps_total_pcu_power_input_cW / 100.0,
            b.eps_total_pcu_power_output_cW / 100.0,
            b.eps_total_avg_pcu_power_input_cW / 100.0,
            b.eps_total_avg_pcu_power_output_cW / 100.0,
            (int)b.eps_total_fault_count,
            (unsigned)b.eps_enabled_channels_bitfield);

    char reboot_buf[8], rbf_buf[8], eps_reset_buf[8];
    fprintf(fp,
            "%s%s: tcmd queued=%u pending=%u reboot=%s eps_reset=%s rbf=%s antenna=%u\n",
            prefix, label,
            (unsigned)b.total_tcmd_queued_count,
            (unsigned)b.pending_queued_tcmd_count,
            reboot_reason_str(b.reboot_reason, reboot_buf, sizeof reboot_buf),
            eps_reset_cause_str(b.eps_reset_cause_enum,
                                eps_reset_buf, sizeof eps_reset_buf),
            rbf_state_str(b.rbf_pin_state, rbf_buf, sizeof rbf_buf),
            (unsigned)b.active_rf_switch_antenna);

    char rf_buf[8], time_buf[8], gnss_buf[8];
    fprintf(fp,
            "%s%s: rf_switch=%s time_sync=%s gnss_rx_mode=%s\n",
            prefix, label,
            rf_switch_mode_str(b.active_rf_switch_control_mode,
                               rf_buf, sizeof rf_buf),
            time_sync_source_str(b.last_time_sync_source_enum,
                                 time_buf, sizeof time_buf),
            gnss_rx_mode_str(b.gnss_rx_mode_enum, gnss_buf, sizeof gnss_buf));

    char mpi_rx_buf[8], mpi_tx_buf[8], mpi_stop_buf[8];
    fprintf(fp,
            "%s%s: mpi rx=%s tx=%s last_stop=%s\n",
            prefix, label,
            mpi_rx_mode_str(b.mpi_rx_mode_enum,
                            mpi_rx_buf, sizeof mpi_rx_buf),
            mpi_transceiver_str(b.mpi_transceiver_state_enum,
                                mpi_tx_buf, sizeof mpi_tx_buf),
            mpi_stop_reason_str(b.mpi_last_reason_for_stopping_enum,
                                mpi_stop_buf, sizeof mpi_stop_buf));

    // friendly_message may not be NUL-terminated, and on a corrupted frame
    // can carry control bytes that would otherwise reach the terminal raw (a
    // stray 0x0E flips it into line-drawing mode). Sanitise non-printables to
    // '.' the way beacon_basic_summary already does.
    char msg[COMMS_BEACON_FRIENDLY_MESSAGE_SIZE + 1];
    cts1_sanitise_text((const uint8_t *) b.friendly_message,
                       COMMS_BEACON_FRIENDLY_MESSAGE_SIZE,
                       msg, sizeof msg, NULL);
    fprintf(fp, "%s%s: msg=\"%s\"\n", prefix, label, msg);
}

// Build the "[ts] " tag every line in a block carries. ts == NULL gives
// flat output for callers like rx_decode that don't decorate lines with
// timestamps; ts != NULL matches decode_loop's emit_frame style.
static void beacon_prefix(const char *ts, char *out, size_t outn)
{
    if (ts != NULL) snprintf(out, outn, "[%s] ", ts);
    else            out[0] = '\0';
}

void beacon_print(FILE *fp, const char *ts,
                  const uint8_t *payload, size_t len)
{
    (void)len;
    // memcpy into a stack-allocated struct so this works regardless of
    // payload alignment in the caller.
    COMMS_beacon_basic_packet_t b;
    memcpy(&b, payload, sizeof b);
    char prefix[64];
    beacon_prefix(ts, prefix, sizeof prefix);
    beacon_print_common(fp, prefix, "beacon", &b);
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
    char name[5];
    cts1_sanitise_text((const uint8_t *) b.satellite_name, 4,
                       name, sizeof name, NULL);
    int n = snprintf(out, out_size,
        "%s st=%s eps=%s batt=%.2fV/%u%% obc=%s up=%s cnt=%u \"%s\"",
        name, state_str, eps_str,
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
              + COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET
              + COMMS_CSP_CRC32_TRAILER_BYTES) return 0;
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
    // The firmware NUL-terminates the response string and can carry
    // trailing bytes past the terminator (framing/parity residue or
    // padding), so showing the full data_len appends garbage. Display
    // the message up to the first NUL; the raw hex/ascii dump below still
    // shows the wire-exact bytes for inspection. A mid-stream chunk of a
    // multi-packet response carries no NUL, so it still shows in full.
    size_t print_len = 0;
    while (print_len < data_len
           && payload[COMMS_TCMD_RESPONSE_HEADER_SIZE + print_len] != 0x00) {
        print_len++;
    }

    // Show both the humanized ts_sent and the raw unix-ms value. The
    // humanized form drops the milliseconds, so only the raw integer can
    // be matched exactly against the @tssent=<unix_ms> in the uplink
    // agenda log to correlate a response with the telecommand that sent it.
    fprintf(fp,
            "%stcmd_response: code=%u%s duration=%ums seq=%u/%u ts_sent=%s (%llu)\n",
            prefix,
            (unsigned)hdr.response_code,
            (hdr.response_code == 0) ? " (OK)" : "",
            (unsigned)hdr.duration_ms,
            (unsigned)hdr.response_seq_num,
            (unsigned)hdr.response_max_seq_num,
            ts_buf,
            (unsigned long long)hdr.ts_sent);

    fprintf(fp, "%stcmd_response: data (%zu bytes): \"", prefix, print_len);
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
    // Message is NUL-terminated; trailing buffer bytes past it are
    // garbage. Show up to the first NUL (see tcmd_response_print).
    size_t print_len = 0;
    while (print_len < data_len
           && payload[COMMS_TCMD_RESPONSE_HEADER_SIZE + print_len] != 0x00) {
        print_len++;
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
    if (len < 2
     || len > AX100_DOWNLINK_MAX_BYTES + COMMS_CSP_CRC32_TRAILER_BYTES) return 0;
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

void cts1_rx_panel_summary(const uint8_t *packet, size_t len,
                           uint8_t packet_type, int rs_errs,
                           char *out, size_t out_size)
{
    if (!out || out_size == 0) return;
    out[0] = '\0';
    if (rs_errs == -2) {
        // Uncorrectable Reed-Solomon block: the recovered bytes are garbage,
        // so parsing them into telemetry (battery voltage, log text, command
        // IDs) would mislead the operator. Mark it for every type instead of
        // decoding. CRC-mismatch frames (rs_errs >= 0) stay visible -- that is
        // the existing low-SNR policy, a different case.
        snprintf(out, out_size,
                 "%sRS-FAIL: telemetry hidden (uncorrectable frame)",
                 RX_RS_FAIL_TOKEN);
        return;
    }
    switch (packet_type) {
        case COMMS_PACKET_TYPE_BEACON_BASIC:
            beacon_basic_summary(packet, len, out, out_size);
            break;
        case COMMS_PACKET_TYPE_BEACON_EXTENDED:
            beacon_ext_summary(packet, len, out, out_size);
            break;
        case COMMS_PACKET_TYPE_TCMD_RESPONSE:
            tcmd_response_summary(packet, len, out, out_size);
            break;
        case COMMS_PACKET_TYPE_LOG_MESSAGE:
            log_message_summary(packet, len, out, out_size);
            break;
        default:
            // Peripheral / bulk / unknown: no one-line parser; the panel
            // falls back to a hex preview of the payload.
            break;
    }
}

int bulk_file_is(const uint8_t *payload, size_t len)
{
    if (payload == NULL) return 0;
    if (len < COMMS_BULK_FILE_DOWNLINK_PACKET_HEADER_SIZE + 1) return 0;
    // +trailer: a full 200-byte chunk plus the 4-byte CSP CRC32 trailer
    // is 204 on the wire; without the slack the whole file download is
    // rejected and lands in the "unknown" bucket.
    if (len > AX100_DOWNLINK_MAX_BYTES + COMMS_CSP_CRC32_TRAILER_BYTES) return 0;
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

// ---- the extended (blob) beacon -------------------------------------------

int beacon_is_extended(const uint8_t *payload, size_t len)
{
    if (payload == NULL) return 0;
    // 198 is the bare struct; 202 is the same with the 4-byte CSP CRC32
    // trailer left on (--csp-crc32 is opt-in, and a corrupt trailer stays
    // in the payload either way).
    if (len != sizeof(COMMS_beacon_extended_packet_t)
     && len != sizeof(COMMS_beacon_extended_packet_t)
              + COMMS_CSP_CRC32_TRAILER_BYTES) return 0;
    if (payload[0] != COMMS_PACKET_TYPE_BEACON_EXTENDED) return 0;
    // One of the two magic fields has to be intact. Requiring both would
    // throw away a beacon rescued with a single bad byte; requiring
    // neither would let 198 bytes of anything starting with 0x20 in.
    const int name_ok = (memcmp(payload + 1, "CTS1", 4) == 0);
    const size_t end_off = offsetof(COMMS_beacon_extended_packet_t, end_message);
    const int end_ok = (payload[end_off] == ' ' && payload[end_off + 1] == 'X');
    return name_ok || end_ok;
}

int beacon_magic_intact(const uint8_t *payload, size_t len)
{
    if (payload == NULL) return 0;
    // Both structs put satellite_name at byte 1 and end_message at the
    // same offset, so one test serves both kinds.
    const size_t end_off = offsetof(COMMS_beacon_basic_packet_t, end_message);
    const int is_ext   = beacon_is_extended(payload, len);
    const int is_basic = beacon_is_basic(payload, len);
    if (!is_ext && !is_basic) return 0;
    if (memcmp(payload + 1, "CTS1", 4) != 0) return 0;
    if (is_ext) {
        // " X" and then a digit: the blob version. beacon_ext_version
        // returns 0 for anything else.
        return beacon_ext_version(payload, len) > 0;
    }
    return memcmp(payload + end_off, "END", 3) == 0;
}

int beacon_ext_version(const uint8_t *payload, size_t len)
{
    if (!beacon_is_extended(payload, len)) return 0;
    const size_t end_off = offsetof(COMMS_beacon_extended_packet_t, end_message);
    if (payload[end_off] != ' ' || payload[end_off + 1] != 'X') return 0;
    const uint8_t d = payload[end_off + 2];
    if (d < '0' || d > '9') return 0;
    return d - '0';
}

void beacon_ext_adcs_state(const uint8_t p[6], beacon_ext_adcs_state_t *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof *out);
    if (p == NULL) return;

    // The blob zeroes the whole packet before filling it and leaves
    // these six bytes alone when the ADCS does not answer, so all-zero
    // means "not reported". A genuinely idle ADCS reports estimation
    // mode 0 with run mode 0 and no units enabled, which is the same six
    // zero bytes -- so this reads as not reported too. That is the safe
    // way round: it hides nothing an operator would act on (an ADCS
    // sitting in mode 0 with everything off), and it stops a failed
    // query from being drawn as a real attitude.
    if (p[0] == 0 && p[1] == 0 && p[2] == 0
     && p[3] == 0 && p[4] == 0 && p[5] == 0) return;
    out->reported = 1;

    out->estimation_mode = p[0] & 0x0F;
    out->control_mode    = (p[0] >> 4) & 0x0F;

    out->run_mode   = p[1] & 0x03;
    out->asgp4_mode = (p[1] >> 2) & 0x03;
    out->cubecontrol_signal_enabled = (p[1] >> 4) & 1;
    out->cubecontrol_motor_enabled  = (p[1] >> 5) & 1;
    out->cubesense1_enabled         = (p[1] >> 6) & 1;
    out->cubesense2_enabled         = (p[1] >> 7) & 1;

    out->cubewheel1_enabled      =  p[2]       & 1;
    out->cubewheel2_enabled      = (p[2] >> 1) & 1;
    out->cubewheel3_enabled      = (p[2] >> 2) & 1;
    out->cubestar_enabled        = (p[2] >> 3) & 1;
    out->gps_receiver_enabled    = (p[2] >> 4) & 1;
    out->gps_lna_power_enabled   = (p[2] >> 5) & 1;
    out->motor_driver_enabled    = (p[2] >> 6) & 1;
    out->sun_above_local_horizon = (p[2] >> 7) & 1;

    out->cubesense1_comm_error           =  p[3]       & 1;
    out->cubesense2_comm_error           = (p[3] >> 1) & 1;
    out->cubecontrol_signal_comm_error   = (p[3] >> 2) & 1;
    out->cubecontrol_motor_comm_error    = (p[3] >> 3) & 1;
    out->cubewheel1_comm_error           = (p[3] >> 4) & 1;
    out->cubewheel2_comm_error           = (p[3] >> 5) & 1;
    out->cubewheel3_comm_error           = (p[3] >> 6) & 1;
    out->cubestar_comm_error             = (p[3] >> 7) & 1;

    out->magnetometer_range_error          =  p[4]       & 1;
    out->cam1_sram_overcurrent_detected    = (p[4] >> 1) & 1;
    out->cam1_3v3_overcurrent_detected     = (p[4] >> 2) & 1;
    out->cam1_sensor_busy_error            = (p[4] >> 3) & 1;
    out->cam1_sensor_detection_error       = (p[4] >> 4) & 1;
    out->sun_sensor_range_error            = (p[4] >> 5) & 1;
    out->cam2_sram_overcurrent_detected    = (p[4] >> 6) & 1;
    out->cam2_3v3_overcurrent_detected     = (p[4] >> 7) & 1;

    out->cam2_sensor_busy_error          =  p[5]       & 1;
    out->cam2_sensor_detection_error     = (p[5] >> 1) & 1;
    out->nadir_sensor_range_error        = (p[5] >> 2) & 1;
    out->rate_sensor_range_error         = (p[5] >> 3) & 1;
    out->wheel_speed_range_error         = (p[5] >> 4) & 1;
    out->coarse_sun_sensor_error         = (p[5] >> 5) & 1;
    out->startracker_match_error         = (p[5] >> 6) & 1;
    out->startracker_overcurrent_detected = (p[5] >> 7) & 1;
}

int beacon_ext_attitude_is_valid(const uint8_t *payload, size_t len)
{
    if (!beacon_is_extended(payload, len)) return 0;
    COMMS_beacon_extended_packet_t b;
    memcpy(&b, payload, sizeof b);
    beacon_ext_adcs_state_t st;
    beacon_ext_adcs_state(b.adcs_current_state_1, &st);
    if (!st.reported) return 0;
    return (st.estimation_mode >= 3 && st.estimation_mode <= 6);
}

static const char *adcs_estimation_mode_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "NONE";
        case 1: return "MEMS_RATE";
        case 2: return "MAG_RATE";
        case 3: return "MAG_RATE_PITCH";
        case 4: return "MAG_SUN_TRIAD";
        case 5: return "FULL_EKF";
        case 6: return "GYRO_EKF";
        case 7: return "USER";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

static const char *adcs_control_mode_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0:  return "NONE";
        case 1:  return "DETUMBLE";
        case 2:  return "Y_THOMSON";
        case 3:  return "Y_WHEEL_ACQ";
        case 4:  return "Y_WHEEL_STEADY";
        case 5:  return "XYZ_WHEEL";
        case 6:  return "SUN_TRACK";
        case 7:  return "TARGET_TRACK";
        case 8:  return "VFAST_DETUMBLE";
        case 9:  return "FAST_DETUMBLE";
        case 10: return "USER_1";
        case 11: return "USER_2";
        case 12: return "STOP_WHEELS";
        case 13: return "USER_CODED";
        case 14: return "SUN_TRACK_1AX";
        case 15: return "TARGET_TRACK_1AX";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

static const char *adcs_run_mode_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "OFF";
        case 1: return "ENABLED";
        case 2: return "TRIGGERED";
        case 3: return "SIM";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

// The MPI temperature field carries three sentinels in place of a
// reading: -99 when the MPI was not active, -98 and -97 for errors.
static const char *fmt_mpi_temp(char *buf, size_t cap, int8_t v)
{
    if (v == -99)                  snprintf(buf, cap, "inactive");
    else if (v == -98 || v == -97) snprintf(buf, cap, "err%d", (int)v);
    else                           snprintf(buf, cap, "%dC", (int)v);
    return buf;
}

// The blob's own placeholder for a field whose EPS query failed. Same
// idea as fmt_cC_i16's INT16_MAX check, different sentinel: the blob
// pre-fills -9999 and only overwrites it when the EPS answers.
static const char *fmt_ext_i16(char *buf, size_t cap, int16_t v,
                               double scale, const char *unit)
{
    if (v == -9999) snprintf(buf, cap, "n/a");
    else            snprintf(buf, cap, "%.*f%s",
                             scale < 1.0 ? 2 : 0, v * scale, unit);
    return buf;
}

void beacon_ext_print(FILE *fp, const char *ts,
                      const uint8_t *payload, size_t len)
{
    (void)len;
    COMMS_beacon_extended_packet_t b;
    memcpy(&b, payload, sizeof b);
    char prefix[64];
    beacon_prefix(ts, prefix, sizeof prefix);

    // The shared 130 bytes, read exactly as a basic beacon. Safe to
    // alias: the two structs are asserted to agree up to end_message.
    COMMS_beacon_basic_packet_t basic;
    memcpy(&basic, payload, sizeof basic);
    beacon_print_common(fp, prefix, "beacon_ext", &basic);

    const int ver = beacon_ext_version(payload, len);
    char mpi_t[16];
    fprintf(fp,
            "%sbeacon_ext: blob=v%d osc=%uMHz obc_adc_batt=%.3fV "
            "mpi_last_temp=%s\n",
            prefix, ver,
            (unsigned)b.obc_active_oscillator_MHz,
            b.obc_adc_battery_voltage_mV / 1000.0,
            fmt_mpi_temp(mpi_t, sizeof mpi_t, b.mpi_last_temperature_C));

    // The four solar-panel conditioning channels, one line each, so a
    // channel that has stopped producing is easy to pick out.
    const int16_t chv[4] = { b.eps_pcu_ch0_volt_in_mppt_mV,
                             b.eps_pcu_ch1_volt_in_mppt_mV,
                             b.eps_pcu_ch2_volt_in_mppt_mV,
                             b.eps_pcu_ch3_volt_in_mppt_mV };
    const int16_t chi[4] = { b.eps_pcu_ch0_curr_in_mppt_mA,
                             b.eps_pcu_ch1_curr_in_mppt_mA,
                             b.eps_pcu_ch2_curr_in_mppt_mA,
                             b.eps_pcu_ch3_curr_in_mppt_mA };
    const int16_t cho[4] = { b.eps_pcu_ch0_curr_ou_mppt_mA,
                             b.eps_pcu_ch1_curr_ou_mppt_mA,
                             b.eps_pcu_ch2_curr_ou_mppt_mA,
                             b.eps_pcu_ch3_curr_ou_mppt_mA };
    for (int i = 0; i < 4; i++) {
        char v[16], ci[16], co[16];
        fprintf(fp, "%sbeacon_ext: solar ch%d in=%s cur_in=%s cur_out=%s\n",
                prefix, i,
                fmt_ext_i16(v,  sizeof v,  chv[i], 0.001, "V"),
                fmt_ext_i16(ci, sizeof ci, chi[i], 1.0,   "mA"),
                fmt_ext_i16(co, sizeof co, cho[i], 1.0,   "mA"));
    }

    // Battery pack status. The heater bit is the one worth calling out
    // by name: it explains a sudden draw with nothing else running.
    const uint16_t bp = b.eps_battery_pack_status_bitfield;
    char netp[16], distp[16];
    fprintf(fp,
            "%sbeacon_ext: batt_pack=0x%04x%s%s%s avg_net=%s avg_distributed=%s\n",
            prefix, (unsigned)bp,
            (bp == 0xFFFF) ? " (n/a)" : "",
            (bp != 0xFFFF && (bp & 0x1000)) ? " heater=on" : "",
            (bp != 0xFFFF && (bp & 0x8000)) ? " pack=enabled" : "",
            fmt_ext_i16(netp,  sizeof netp,  b.eps_total_avg_net_battery_power_cW,
                        0.01, "W"),
            fmt_ext_i16(distp, sizeof distp, b.eps_total_avg_power_distributed_cW,
                        0.01, "W"));

    beacon_ext_adcs_state_t st;
    beacon_ext_adcs_state(b.adcs_current_state_1, &st);
    if (!st.reported) {
        fprintf(fp, "%sbeacon_ext: adcs did not answer "
                    "(state, sun sensors, field and attitude all unset)\n",
                prefix);
        return;
    }

    char est[16], ctl[20], run[12];
    fprintf(fp,
            "%sbeacon_ext: adcs run=%s control=%s estimation=%s asgp4=%u "
            "sun_up=%d\n",
            prefix,
            adcs_run_mode_str(st.run_mode, run, sizeof run),
            adcs_control_mode_str(st.control_mode, ctl, sizeof ctl),
            adcs_estimation_mode_str(st.estimation_mode, est, sizeof est),
            (unsigned)st.asgp4_mode, st.sun_above_local_horizon);

    fprintf(fp,
            "%sbeacon_ext: adcs enabled: ctrl_sig=%d ctrl_motor=%d "
            "cs1=%d cs2=%d wheels=%d%d%d star=%d gps=%d lna=%d motor=%d\n",
            prefix,
            st.cubecontrol_signal_enabled, st.cubecontrol_motor_enabled,
            st.cubesense1_enabled, st.cubesense2_enabled,
            st.cubewheel1_enabled, st.cubewheel2_enabled, st.cubewheel3_enabled,
            st.cubestar_enabled, st.gps_receiver_enabled,
            st.gps_lna_power_enabled, st.motor_driver_enabled);

    // Faults, named, and only when there are some -- a clean ADCS
    // shouldn't cost a line saying so in the middle of a pass.
    {
        char faults[320];
        int fp2 = 0;
        #define FAULT(cond, name) \
            if (cond) fp2 += snprintf(faults + fp2, sizeof faults - fp2, \
                                      "%s%s", fp2 ? " " : "", name)
        FAULT(st.cubesense1_comm_error,         "cs1_comm");
        FAULT(st.cubesense2_comm_error,         "cs2_comm");
        FAULT(st.cubecontrol_signal_comm_error, "ctrl_sig_comm");
        FAULT(st.cubecontrol_motor_comm_error,  "ctrl_motor_comm");
        FAULT(st.cubewheel1_comm_error,         "wheel1_comm");
        FAULT(st.cubewheel2_comm_error,         "wheel2_comm");
        FAULT(st.cubewheel3_comm_error,         "wheel3_comm");
        FAULT(st.cubestar_comm_error,           "star_comm");
        FAULT(st.magnetometer_range_error,      "mag_range");
        FAULT(st.sun_sensor_range_error,        "sun_range");
        FAULT(st.nadir_sensor_range_error,      "nadir_range");
        FAULT(st.rate_sensor_range_error,       "rate_range");
        FAULT(st.wheel_speed_range_error,       "wheel_speed_range");
        FAULT(st.coarse_sun_sensor_error,       "css");
        FAULT(st.cam1_sram_overcurrent_detected, "cam1_sram_oc");
        FAULT(st.cam1_3v3_overcurrent_detected,  "cam1_3v3_oc");
        FAULT(st.cam1_sensor_busy_error,         "cam1_busy");
        FAULT(st.cam1_sensor_detection_error,    "cam1_detect");
        FAULT(st.cam2_sram_overcurrent_detected, "cam2_sram_oc");
        FAULT(st.cam2_3v3_overcurrent_detected,  "cam2_3v3_oc");
        FAULT(st.cam2_sensor_busy_error,         "cam2_busy");
        FAULT(st.cam2_sensor_detection_error,    "cam2_detect");
        FAULT(st.startracker_match_error,        "star_match");
        FAULT(st.startracker_overcurrent_detected, "star_oc");
        #undef FAULT
        if (fp2 > 0)
            fprintf(fp, "%sbeacon_ext: adcs faults: %s\n", prefix, faults);
    }

    fprintf(fp,
            "%sbeacon_ext: css 1-7,9=%u %u %u %u %u %u %u %u\n",
            prefix,
            (unsigned)b.adcs_raw_css_1, (unsigned)b.adcs_raw_css_2,
            (unsigned)b.adcs_raw_css_3, (unsigned)b.adcs_raw_css_4,
            (unsigned)b.adcs_raw_css_5, (unsigned)b.adcs_raw_css_6,
            (unsigned)b.adcs_raw_css_7, (unsigned)b.adcs_raw_css_9);

    // Field in microtesla (1 count = 10 nT) with its magnitude, and the
    // rates in degrees per second.
    const double bx = b.adcs_magnetic_field_x_T_en8 / 100.0;
    const double by = b.adcs_magnetic_field_y_T_en8 / 100.0;
    const double bz = b.adcs_magnetic_field_z_T_en8 / 100.0;
    fprintf(fp,
            "%sbeacon_ext: mag body=%.2f %.2f %.2f uT |B|=%.2f uT\n",
            prefix, bx, by, bz, sqrt(bx * bx + by * by + bz * bz));

    fprintf(fp,
            "%sbeacon_ext: rates mems_norm=%.2f est=%.2f %.2f %.2f deg/s\n",
            prefix,
            b.adcs_angular_rate_norm_cdeg_per_sec / 100.0,
            b.adcs_estimated_rate_x_cdeg_per_sec / 100.0,
            b.adcs_estimated_rate_y_cdeg_per_sec / 100.0,
            b.adcs_estimated_rate_z_cdeg_per_sec / 100.0);

    if (beacon_ext_attitude_is_valid(payload, len)) {
        fprintf(fp,
                "%sbeacon_ext: attitude roll=%.2f pitch=%.2f yaw=%.2f deg "
                "(body wrt orbit frame)\n",
                prefix,
                b.adcs_estimated_roll_angle_cdeg / 100.0,
                b.adcs_estimated_pitch_angle_cdeg / 100.0,
                b.adcs_estimated_yaw_angle_cdeg / 100.0);
    } else {
        fprintf(fp,
                "%sbeacon_ext: attitude not estimated in mode %s "
                "(angles only filled in modes 3-6)\n",
                prefix, adcs_estimation_mode_str(st.estimation_mode,
                                                 est, sizeof est));
    }
}

int beacon_ext_summary(const uint8_t *payload, size_t len,
                       char *out, size_t out_size)
{
    if (!out || out_size == 0) return 0;
    out[0] = '\0';
    if (!beacon_is_extended(payload, len)) return 0;
    COMMS_beacon_extended_packet_t b;
    memcpy(&b, payload, sizeof b);

    char state_buf[8], eps_buf[8], obc_buf[16], up_buf[24];
    const char *state_str = cts1_state_str(b.cts1_operation_state,
                                           state_buf, sizeof state_buf);
    const char *eps_str   = eps_mode_str  (b.eps_mode_enum,
                                           eps_buf,   sizeof eps_buf);
    fmt_cC_i32  (obc_buf, sizeof obc_buf, b.obc_temperature_cC);
    fmt_ms_clock(b.uptime_ms,             up_buf,    sizeof up_buf);
    char name[5];
    cts1_sanitise_text((const uint8_t *) b.satellite_name, 4,
                       name, sizeof name, NULL);

    // The attitude is what an extended beacon is for, so it goes in the
    // one line even when there is none to show -- what mode the ADCS was
    // estimating in is itself the answer to "why no attitude".
    beacon_ext_adcs_state_t st;
    beacon_ext_adcs_state(b.adcs_current_state_1, &st);
    char att[80];
    if (!st.reported) {
        snprintf(att, sizeof att, "adcs=silent");
    } else if (beacon_ext_attitude_is_valid(payload, len)) {
        char ctl[20];
        snprintf(att, sizeof att, "%s rpy=%.1f/%.1f/%.1f",
                 adcs_control_mode_str(st.control_mode, ctl, sizeof ctl),
                 b.adcs_estimated_roll_angle_cdeg / 100.0,
                 b.adcs_estimated_pitch_angle_cdeg / 100.0,
                 b.adcs_estimated_yaw_angle_cdeg / 100.0);
    } else {
        char ctl[20], est[16];
        snprintf(att, sizeof att, "%s est=%s",
                 adcs_control_mode_str(st.control_mode, ctl, sizeof ctl),
                 adcs_estimation_mode_str(st.estimation_mode,
                                          est, sizeof est));
    }

    int n = snprintf(out, out_size,
        "%s X%d st=%s eps=%s batt=%.2fV/%u%% obc=%s up=%s cnt=%u %s "
        "rate=%.2fdeg/s",
        name, beacon_ext_version(payload, len), state_str, eps_str,
        b.eps_battery_voltage_mV / 1000.0,
        (unsigned) b.eps_battery_percent,
        obc_buf, up_buf,
        (unsigned) b.total_beacon_count_since_boot,
        att,
        b.adcs_angular_rate_norm_cdeg_per_sec / 100.0);
    if (n < 0) { out[0] = '\0'; return 0; }
    return (n < (int) out_size) ? n : (int) out_size - 1;
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
    if (beacon_is_extended(payload, len)) {
        beacon_ext_print(fp, ts, payload, len);
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
