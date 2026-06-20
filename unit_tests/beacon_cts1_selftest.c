/*

    Simple Satellite Operations  unit_tests/beacon_cts1_selftest.c

    Coverage for src/beacon/beacon_cts1.{c,h}. The CTS1 beacon parser sits
    at the end of the receive chain — every byte offset, every enum
    mapping, and every length-anchored sniff has to stay aligned with the
    flight firmware (vendored from sat-1-rc3). A silent regression here
    shows up as wrong fields on the operator's panel during a pass, with
    no other diagnostic; build a packed-byte fixture and audit each
    parsed value.

    What's covered:
      - beacon_is_basic length gates (130, 134 accept; 0/129/131/200 reject;
        NULL safety).
      - beacon_basic_summary parses every load-bearing field at the right
        byte offset: satellite_name, operation_state, eps_mode_enum,
        battery voltage/percent, obc temperature, uptime, count, friendly
        message.
      - With and without the 4-byte CSP CRC32 trailer.
      - INT16_MAX sentinel renders "n/a" rather than 327.67C.
      - beacon_print writes the expected six-line block (captured via
        tmpfile to keep stdout clean).
      - tcmd_response_is short-circuits beacon-length payloads (the bit-
        flip-tolerance regression we hit at sat-1-rc3 bring-up).
      - log_message_is requires at least one printable byte in the first
        eight data bytes (filters pure-binary frames that happen to
        start with 0x03).
      - bulk_file_is gates on packet_type 0x10 and rejects beacon-shadow
        lengths.
      - cts1_sanitise_text: trailing NUL strip, non-printable → '.',
        truncation flag.

    Exit status: 0 = all tests passed, non-zero = failure.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include "beacon_cts1.h"
#include "tap.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Populate `buf` with a 130-byte basic-beacon fixture. Field values are
// chosen to be unambiguously identifiable in the formatted output (no
// accidental collisions with adjacent fields' values, and all enums in
// a range whose string mapping differs from the next-door value).
static void make_beacon_fixture(uint8_t *buf)
{
    COMMS_beacon_basic_packet_t b;
    memset(&b, 0, sizeof b);
    b.packet_type = COMMS_PACKET_TYPE_BEACON_BASIC;
    memcpy(b.satellite_name, "CTS1", 4);
    b.active_rf_switch_antenna = 1;
    b.active_rf_switch_control_mode = 0; // TOGGLE_PER_BEACON
    b.uptime_ms = 3661000;               // 01:01:01.000
    b.duration_since_last_uplink_ms = 60000;
    b.unix_epoch_time_ms = 1700000000000ULL;
    b.last_time_sync_source_enum = 2;    // GNSS_PPS
    b.is_fs_mounted = 1;
    b.total_tcmd_queued_count = 42;
    b.pending_queued_tcmd_count = 3;
    b.total_beacon_count_since_boot = 12345;
    b.eps_mode_enum = 1;                 // NOMINAL
    b.eps_reset_cause_enum = 0;          // POWER_ON
    b.eps_uptime_sec = 3661;
    b.eps_error_code = 0;
    b.eps_battery_voltage_mV = 8123;
    b.eps_battery_percent = 87;
    b.eps_battery_temperature_0_cC = 1234;
    b.eps_battery_temperature_1_cC = -567;
    b.eps_total_fault_count = 0;
    b.eps_enabled_channels_bitfield = 0x00ff;
    b.eps_total_pcu_power_input_cW = 250;
    b.eps_total_pcu_power_output_cW = 230;
    b.eps_total_avg_pcu_power_input_cW = 245;
    b.eps_total_avg_pcu_power_output_cW = 225;
    b.obc_temperature_cC = 1523;
    b.reboot_reason = 4;                 // SOFTWARE
    b.cts1_operation_state = CTS1_OPERATION_STATE_NOMINAL_WITH_RADIO_TX;
    b.rbf_pin_state = 1;                 // FLYING
    b.mpi_rx_mode_enum = 0;              // COMMAND
    b.mpi_transceiver_state_enum = 0;    // INACTIVE
    b.mpi_last_reason_for_stopping_enum = 0;
    b.gnss_uart_interrupt_enabled = 1;
    b.gnss_rx_mode_enum = 1;             // FIREHOSE
    memcpy(b.friendly_message, "hello world", 11);
    memcpy(b.end_message, "END", 4);     // 4 bytes incl. trailing NUL
    memcpy(buf, &b, sizeof b);
}

// ------------------------------------------------------------------
// 1. beacon_is_basic length gating.
// ------------------------------------------------------------------

static void test_beacon_is_basic_length_gates(void)
{
    uint8_t buf[200];
    make_beacon_fixture(buf);

    tap_ok(beacon_is_basic(buf, 130) == 1,
           "is_basic: 130-byte payload accepted");
    tap_ok(beacon_is_basic(buf, 134) == 1,
           "is_basic: 134-byte payload accepted (CSP CRC32 trailer)");
    tap_ok(beacon_is_basic(buf, 129) == 0,
           "is_basic: 129-byte payload rejected");
    tap_ok(beacon_is_basic(buf, 131) == 0,
           "is_basic: 131-byte payload rejected");
    tap_ok(beacon_is_basic(buf, 133) == 0,
           "is_basic: 133-byte payload rejected (one short of trailer)");
    tap_ok(beacon_is_basic(buf, 200) == 0,
           "is_basic: 200-byte payload rejected");
    tap_ok(beacon_is_basic(buf, 0) == 0,
           "is_basic: zero-length payload rejected");
    tap_ok(beacon_is_basic(NULL, 130) == 0,
           "is_basic: NULL payload pointer rejected");
}

// ------------------------------------------------------------------
// 2. beacon_basic_summary field-offset audit.
// ------------------------------------------------------------------

static void test_beacon_basic_summary_fields(void)
{
    uint8_t buf[130];
    make_beacon_fixture(buf);

    char out[256];
    int n = beacon_basic_summary(buf, sizeof buf, out, sizeof out);
    tap_okf(n > 0, "summary: returns positive length (%d)", n);

    // satellite_name at offset 1, exactly "CTS1" without NUL.
    tap_okf(strstr(out, "CTS1") != NULL,
            "summary: contains CTS1 name (%s)", out);
    // cts1_operation_state at offset 77 -> NOMINAL_TX (value 2).
    tap_ok(strstr(out, "st=NOMINAL_TX") != NULL,
           "summary: operation_state == NOMINAL_TX");
    // eps_mode_enum at offset 33 -> NOMINAL (value 1).
    tap_ok(strstr(out, "eps=NOMINAL") != NULL,
           "summary: eps_mode == NOMINAL");
    // eps_battery_voltage_mV at offset 41 -> 8123 mV -> 8.12V at %.2f.
    tap_ok(strstr(out, "batt=8.12V") != NULL,
           "summary: battery voltage 8.12V");
    // eps_battery_percent at offset 43 -> 87.
    tap_ok(strstr(out, "/87%") != NULL,
           "summary: battery percent 87");
    // obc_temperature_cC at offset 72 -> 1523 cC -> 15.23C at %.2f.
    tap_ok(strstr(out, "obc=15.23C") != NULL,
           "summary: obc temperature 15.23C");
    // uptime_ms at offset 7 -> 3661000 ms -> 01:01:01.000.
    tap_ok(strstr(out, "up=01:01:01.000") != NULL,
           "summary: uptime 01:01:01.000");
    // total_beacon_count_since_boot at offset 29 -> 12345.
    tap_ok(strstr(out, "cnt=12345") != NULL,
           "summary: beacon count 12345");
    // friendly_message at offset 84.
    tap_okf(strstr(out, "hello world") != NULL,
            "summary: friendly_message round-trips (%s)", out);
}

// ------------------------------------------------------------------
// 3. beacon_basic_summary with 4-byte CSP CRC32 trailer present.
// ------------------------------------------------------------------

static void test_beacon_basic_summary_with_trailer(void)
{
    uint8_t buf[134];
    make_beacon_fixture(buf);
    buf[130] = 0xde; buf[131] = 0xad; buf[132] = 0xbe; buf[133] = 0xef;

    char out[256];
    int n = beacon_basic_summary(buf, sizeof buf, out, sizeof out);
    tap_okf(n > 0,
            "summary+trailer: returns positive length (%d)", n);
    tap_ok(strstr(out, "CTS1") != NULL,
           "summary+trailer: name CTS1 still parses");
    tap_ok(strstr(out, "cnt=12345") != NULL,
           "summary+trailer: count 12345 still parses (trailer ignored)");
}

// ------------------------------------------------------------------
// 4. n/a temperature sentinel.
// ------------------------------------------------------------------

static void test_beacon_basic_summary_na_temperature(void)
{
    uint8_t buf[130];
    make_beacon_fixture(buf);
    // obc_temperature_cC is int32 at offset 72. INT32_MAX -> "n/a".
    int32_t na = INT32_MAX;
    memcpy(buf + 72, &na, sizeof na);

    char out[256];
    int n = beacon_basic_summary(buf, sizeof buf, out, sizeof out);
    tap_okf(n > 0, "n/a temp: summary still returns >0 (%d)", n);
    tap_ok(strstr(out, "obc=n/a") != NULL,
           "n/a temp: INT32_MAX renders as n/a");
    tap_ok(strstr(out, "327.67") == NULL,
           "n/a temp: no leaked 327.67 sentinel-as-real-value");
}

// ------------------------------------------------------------------
// 5. Length rejection in beacon_basic_summary.
// ------------------------------------------------------------------

static void test_beacon_basic_summary_rejects_wrong_length(void)
{
    uint8_t buf[200];
    make_beacon_fixture(buf);
    char out[256];

    tap_ok(beacon_basic_summary(buf, 129, out, sizeof out) == 0,
           "summary: 129-byte payload returns 0");
    tap_ok(out[0] == '\0',
           "summary: rejected output is empty string");

    tap_ok(beacon_basic_summary(buf, 200, out, sizeof out) == 0,
           "summary: 200-byte payload returns 0");

    tap_ok(beacon_basic_summary(NULL, 130, out, sizeof out) == 0,
           "summary: NULL payload returns 0");

    tap_ok(beacon_basic_summary(buf, 130, NULL, 0) == 0,
           "summary: NULL output buffer returns 0");
}

// ------------------------------------------------------------------
// 6. beacon_print writes the expected line block (captured via tmpfile).
// ------------------------------------------------------------------

static void test_beacon_print_block(void)
{
    uint8_t buf[130];
    make_beacon_fixture(buf);

    FILE *fp = tmpfile();
    if (!fp) { tap_bail("tmpfile"); return; }
    beacon_print(fp, "T+10.500s", buf, sizeof buf);
    long sz = ftell(fp);
    if (sz <= 0 || sz > 4096) { fclose(fp); tap_bail("ftell"); return; }
    rewind(fp);
    char *txt = (char *) malloc((size_t) sz + 1);
    if (!txt) { fclose(fp); tap_bail("oom"); return; }
    size_t got = fread(txt, 1, (size_t) sz, fp);
    txt[got] = '\0';
    fclose(fp);

    // ts prefix appears on every line.
    tap_ok(strstr(txt, "[T+10.500s] beacon: name=\"CTS1\"") != NULL,
           "print: name line uses [ts] prefix and quoted CTS1");
    tap_ok(strstr(txt, "state=NOMINAL_TX") != NULL,
           "print: state line");
    tap_ok(strstr(txt, "eps_mode=NOMINAL") != NULL,
           "print: eps_mode line");
    tap_ok(strstr(txt, "count=12345") != NULL,
           "print: count line");
    tap_ok(strstr(txt, "uptime=01:01:01.000") != NULL,
           "print: uptime line");
    tap_ok(strstr(txt, "batt=8.123V 87%") != NULL,
           "print: batt+percent line (full mV precision)");
    tap_ok(strstr(txt, "rf_switch=TOGGLE_PER_BEACON") != NULL,
           "print: rf_switch mode");
    tap_ok(strstr(txt, "time_sync=GNSS_PPS") != NULL,
           "print: time_sync source");
    tap_ok(strstr(txt, "gnss_rx_mode=FIREHOSE") != NULL,
           "print: gnss rx mode");
    tap_ok(strstr(txt, "reboot=SOFTWARE") != NULL,
           "print: reboot reason");
    tap_ok(strstr(txt, "rbf=FLYING") != NULL,
           "print: rbf state");
    tap_ok(strstr(txt, "msg=\"hello world\"") != NULL,
           "print: friendly_message");
    free(txt);
}

// ------------------------------------------------------------------
// 7. tcmd_response_is short-circuits beacon-shadow lengths.
// ------------------------------------------------------------------

static void test_tcmd_response_is(void)
{
    // 200-byte valid tcmd response: type=0x04, seq=1, max=2.
    uint8_t tcmd[200];
    memset(tcmd, 0, sizeof tcmd);
    tcmd[0]  = COMMS_PACKET_TYPE_TCMD_RESPONSE;
    tcmd[12] = 1;   // response_seq_num
    tcmd[13] = 2;   // response_max_seq_num
    tap_ok(tcmd_response_is(tcmd, sizeof tcmd) == 1,
           "tcmd: 200-byte type=0x04 seq=1/2 accepted");

    // Beacon-shadow lengths must short-circuit even with type=0x04.
    uint8_t shadow[134] = {0};
    shadow[0] = COMMS_PACKET_TYPE_TCMD_RESPONSE;
    shadow[12] = 1; shadow[13] = 1;
    tap_ok(tcmd_response_is(shadow, 130) == 0,
           "tcmd: 130-byte type=0x04 rejected (beacon territory)");
    tap_ok(tcmd_response_is(shadow, 134) == 0,
           "tcmd: 134-byte type=0x04 rejected (beacon + trailer)");

    // Bad seq numbering.
    uint8_t bad[200] = {0};
    bad[0] = COMMS_PACKET_TYPE_TCMD_RESPONSE;
    bad[12] = 0; bad[13] = 1;
    tap_ok(tcmd_response_is(bad, sizeof bad) == 0,
           "tcmd: seq=0 rejected");
    bad[12] = 3; bad[13] = 2;
    tap_ok(tcmd_response_is(bad, sizeof bad) == 0,
           "tcmd: seq > max rejected");

    // Wrong packet_type.
    uint8_t wt[100] = {0};
    wt[0] = 0x00; wt[12] = 1; wt[13] = 1;
    tap_ok(tcmd_response_is(wt, sizeof wt) == 0,
           "tcmd: type=0x00 rejected");

    // Too short.
    tap_ok(tcmd_response_is(tcmd, 14) == 0,
           "tcmd: header-only (14 bytes, no data) rejected");

    // NULL safety.
    tap_ok(tcmd_response_is(NULL, 200) == 0,
           "tcmd: NULL pointer rejected");
}

// ------------------------------------------------------------------
// 8. log_message_is printability gate.
// ------------------------------------------------------------------

static void test_log_message_is(void)
{
    uint8_t log[64];
    log[0] = COMMS_PACKET_TYPE_LOG_MESSAGE;
    memcpy(log + 1, "ok startup complete", 19);
    tap_ok(log_message_is(log, 1 + 19) == 1,
           "log: printable text accepted");

    // Pure-binary tail — no printable byte in first 8 data bytes.
    uint8_t bin[12] = {COMMS_PACKET_TYPE_LOG_MESSAGE,
                       0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a};
    tap_ok(log_message_is(bin, sizeof bin) == 0,
           "log: pure-binary first 8 bytes rejected");

    // Beacon-shadow length 130 with type=0x03 must NOT route to log.
    uint8_t shadow[130] = {0};
    shadow[0] = COMMS_PACKET_TYPE_LOG_MESSAGE;
    memcpy(shadow + 1, "looks-like-log", 14);
    tap_ok(log_message_is(shadow, sizeof shadow) == 0,
           "log: 130-byte payload rejected (beacon-shadow)");

    // Wrong type byte.
    uint8_t wt[16] = {0x99, 'h','e','l','l','o',0};
    tap_ok(log_message_is(wt, sizeof wt) == 0,
           "log: type=0x99 rejected");

    // Single-byte (header only, no data) — len < 2.
    uint8_t tiny[1] = {COMMS_PACKET_TYPE_LOG_MESSAGE};
    tap_ok(log_message_is(tiny, sizeof tiny) == 0,
           "log: 1-byte payload rejected (no data)");

    tap_ok(log_message_is(NULL, 20) == 0,
           "log: NULL pointer rejected");
}

// ------------------------------------------------------------------
// 9. bulk_file_is.
// ------------------------------------------------------------------

static void test_bulk_file_is(void)
{
    uint8_t pkt[200];
    memset(pkt, 0, sizeof pkt);
    pkt[0] = COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK;
    pkt[1] = 0x00; pkt[2] = 0x10; pkt[3] = 0x00; pkt[4] = 0x00; // offset = 0x1000
    pkt[5] = 0x42;                                              // 1 byte of data
    tap_ok(bulk_file_is(pkt, sizeof pkt) == 1,
           "bulk: 200-byte type=0x10 accepted");
    tap_ok(bulk_file_is(pkt, 6) == 1,
           "bulk: minimum-length (5 header + 1 data) accepted");
    tap_ok(bulk_file_is(pkt, 5) == 0,
           "bulk: header-only (5 bytes, no data) rejected");

    // 130-byte beacon-shadow with type=0x10 must NOT route to bulk.
    uint8_t shadow[130] = {0};
    shadow[0] = COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK;
    tap_ok(bulk_file_is(shadow, sizeof shadow) == 0,
           "bulk: 130-byte type=0x10 rejected (beacon-shadow)");

    // Wrong type byte.
    pkt[0] = 0x11;
    tap_ok(bulk_file_is(pkt, sizeof pkt) == 0,
           "bulk: type=0x11 rejected");

    tap_ok(bulk_file_is(NULL, 100) == 0,
           "bulk: NULL pointer rejected");
}

// ------------------------------------------------------------------
// 10. cts1_sanitise_text.
// ------------------------------------------------------------------

static void test_cts1_sanitise_text(void)
{
    char out[32];
    int truncated;

    // Trailing NULs stripped.
    uint8_t a[8] = {'h','i',0,0,0,0,0,0};
    size_t na = cts1_sanitise_text(a, sizeof a, out, sizeof out, &truncated);
    tap_okf(na == 2 && strcmp(out, "hi") == 0,
            "sanitise: trailing NULs stripped (len=%zu out='%s')", na, out);
    tap_ok(truncated == 0, "sanitise: not truncated");

    // Non-printable bytes replaced with '.'.
    uint8_t b[5] = {'a', 0x01, 'b', 0x7f, 'c'};
    size_t nb = cts1_sanitise_text(b, sizeof b, out, sizeof out, &truncated);
    tap_okf(nb == 5 && strcmp(out, "a.b.c") == 0,
            "sanitise: non-printable -> '.' (out='%s')", out);

    // Truncation flag.
    uint8_t c[40];
    memset(c, 'X', sizeof c);
    size_t nc = cts1_sanitise_text(c, sizeof c, out, sizeof out, &truncated);
    tap_okf(nc == sizeof out - 1 && truncated == 1,
            "sanitise: truncation flag set (copied=%zu)", nc);

    // outn == 0 path.
    char tiny[1] = {'!'};
    size_t nz = cts1_sanitise_text(c, sizeof c, tiny, 0, &truncated);
    tap_ok(nz == 0 && truncated == 1,
           "sanitise: outn=0 returns 0 and flags truncated");
    tap_ok(tiny[0] == '!',
           "sanitise: outn=0 does not write to out");
}

// ------------------------------------------------------------------
// 11. cts1_packet_print dispatcher routing.
// ------------------------------------------------------------------

static void test_cts1_packet_print_dispatch(void)
{
    // Build the four kinds of payload and check that the dispatcher
    // emits the corresponding "beacon:" / "tcmd_response:" / "log:" /
    // "bulk_file:" prefix on its single tmpfile sink. Length-anchored
    // sniffs win first (beacon at 130, tcmd_response by type+seq),
    // then the packet_type byte routes log vs bulk.
    struct {
        const char *what;
        size_t      len;
        const char *expect;
        void      (*fill)(uint8_t *buf, size_t len);
    } cases[4];

    // Beacon — 130 bytes, length-anchored.
    cases[0].what   = "beacon";
    cases[0].len    = 130;
    cases[0].expect = "beacon: name=\"CTS1\"";
    // tcmd_response — 200 bytes, length distinguishes from beacon.
    cases[1].what   = "tcmd_response";
    cases[1].len    = 200;
    cases[1].expect = "tcmd_response:";
    // log_message — 50 bytes (not 130/200).
    cases[2].what   = "log_message";
    cases[2].len    = 50;
    cases[2].expect = "log: ";
    // bulk_file — 100 bytes (not 130/200).
    cases[3].what   = "bulk_file";
    cases[3].len    = 100;
    cases[3].expect = "bulk_file:";

    for (int i = 0; i < 4; ++i) {
        uint8_t buf[200];
        memset(buf, 0, sizeof buf);
        switch (i) {
            case 0: make_beacon_fixture(buf); break;
            case 1:
                buf[0]  = COMMS_PACKET_TYPE_TCMD_RESPONSE;
                buf[12] = 1; buf[13] = 1;
                memcpy(buf + 14, "ok", 3);
                break;
            case 2:
                buf[0] = COMMS_PACKET_TYPE_LOG_MESSAGE;
                memcpy(buf + 1, "info: ready", 11);
                break;
            case 3:
                buf[0] = COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK;
                buf[1] = 0x00; buf[2] = 0x00; buf[3] = 0x00; buf[4] = 0x00;
                memset(buf + 5, 0xa5, cases[3].len - 5);
                break;
        }

        FILE *fp = tmpfile();
        if (!fp) { tap_bail("tmpfile"); return; }
        cts1_packet_print(fp, NULL, buf, cases[i].len);
        long sz = ftell(fp);
        rewind(fp);
        if (sz <= 0 || sz > 4096) {
            fclose(fp);
            tap_okf(0, "dispatch: %s produced no output", cases[i].what);
            continue;
        }
        char txt[4096];
        size_t got = fread(txt, 1, (size_t) sz, fp);
        txt[got] = '\0';
        fclose(fp);
        tap_okf(strstr(txt, cases[i].expect) != NULL,
                "dispatch: %s -> '%s'", cases[i].what, cases[i].expect);
    }
}

int main(void)
{
    test_beacon_is_basic_length_gates();
    test_beacon_basic_summary_fields();
    test_beacon_basic_summary_with_trailer();
    test_beacon_basic_summary_na_temperature();
    test_beacon_basic_summary_rejects_wrong_length();
    test_beacon_print_block();
    test_tcmd_response_is();
    test_log_message_is();
    test_bulk_file_is();
    test_cts1_sanitise_text();
    test_cts1_packet_print_dispatch();
    return tap_done();
}
