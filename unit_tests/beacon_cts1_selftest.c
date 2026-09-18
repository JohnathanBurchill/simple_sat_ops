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
      - The extended (blob) beacon, against one real packet off the air
        whose every field was read out by hand first: the sniff and its
        two magic anchors, the blob version, the six-byte ADCS state
        unpacked bit by bit, which estimation modes have a real
        attitude, every field in the printed block, the blob's own
        -9999 and MPI-temperature sentinels, the fault names, and the
        one-line panel summary.

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

// ------------------------------------------------------------------
// 12. cts1_rx_panel_summary: uncorrectable-RS frames are marked, not
//     parsed (issue #37). The same bytes that yield real telemetry at
//     rs_errs >= 0 must yield only the RS-FAIL marker at rs_errs == -2.
// ------------------------------------------------------------------

static void test_cts1_rx_panel_summary_rs_fail(void)
{
    uint8_t buf[130];
    make_beacon_fixture(buf);
    // Matches rx_session's RX_LAST_SUMMARY_MAX (the real on-panel buffer).
    char out[160];
    size_t tok = strlen(RX_RS_FAIL_TOKEN);

    // rs_errs == 0 (corrected, frame valid): real beacon telemetry, and it
    // must NOT carry the RS-FAIL marker. This is the baseline the bug fix
    // must not regress -- valid beacons keep showing.
    cts1_rx_panel_summary(buf, sizeof buf, COMMS_PACKET_TYPE_BEACON_BASIC,
                          /*rs_errs=*/0, out, sizeof out);
    tap_okf(strncmp(out, RX_RS_FAIL_TOKEN, tok) != 0,
            "rs_panel: rs_errs=0 beacon is not marked RS-FAIL (%s)", out);
    tap_ok(strstr(out, "batt=8.12V") != NULL,
           "rs_panel: rs_errs=0 beacon shows real telemetry");

    // rs_errs == -2 (uncorrectable): the SAME bytes must now be suppressed.
    cts1_rx_panel_summary(buf, sizeof buf, COMMS_PACKET_TYPE_BEACON_BASIC,
                          /*rs_errs=*/-2, out, sizeof out);
    tap_okf(strncmp(out, RX_RS_FAIL_TOKEN, tok) == 0,
            "rs_panel: rs_errs=-2 beacon carries the RS-FAIL marker (%s)", out);
    tap_ok(strstr(out, "batt=") == NULL,
           "rs_panel: rs_errs=-2 beacon hides the telemetry (no batt=)");
    tap_ok(strncmp(out + tok, "RS-FAIL", 7) == 0,
           "rs_panel: stripped display text begins 'RS-FAIL'");

    // rs_errs == -1 (RS off / not applied): NOT suppressed -- only the
    // uncorrectable sentinel hides data.
    cts1_rx_panel_summary(buf, sizeof buf, COMMS_PACKET_TYPE_BEACON_BASIC,
                          /*rs_errs=*/-1, out, sizeof out);
    tap_ok(strncmp(out, RX_RS_FAIL_TOKEN, tok) != 0,
           "rs_panel: rs_errs=-1 (RS off) is not marked RS-FAIL");

    // The marker covers every type, including ones with no parser. A bulk
    // / unknown type at rs_errs == -2 still gets the marker (not an empty
    // string the panel would hex-dump as if it were real bytes).
    cts1_rx_panel_summary(buf, sizeof buf, COMMS_PACKET_TYPE_TCMD_RESPONSE,
                          /*rs_errs=*/-2, out, sizeof out);
    tap_ok(strncmp(out, RX_RS_FAIL_TOKEN, tok) == 0,
           "rs_panel: rs_errs=-2 tcmd type also marked RS-FAIL");
    cts1_rx_panel_summary(buf, sizeof buf, COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK,
                          /*rs_errs=*/-2, out, sizeof out);
    tap_ok(strncmp(out, RX_RS_FAIL_TOKEN, tok) == 0,
           "rs_panel: rs_errs=-2 bulk type also marked RS-FAIL");

    // A parser-less type with a good frame yields an empty summary (the
    // panel hex-dumps it) -- and must NOT be marked. Guards against a
    // mutation that bakes the marker unconditionally.
    cts1_rx_panel_summary(buf, sizeof buf, COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK,
                          /*rs_errs=*/0, out, sizeof out);
    tap_ok(out[0] == '\0',
           "rs_panel: rs_errs=0 bulk type yields empty summary (hex fallback)");
}

// ---- the extended (blob) beacon --------------------------------------
//
// One real extended beacon, off the air and out of the operational
// packet database (row 638383, received 2026-09-18T01:28:34Z, a v4
// blob beacon with no Reed-Solomon errors). Every expected value below
// was read out of these bytes by hand -- a shell script walking the
// field offsets with its own little-endian reader -- before the decoder
// under test was pointed at them, so the two are independent. Several
// of them also cross-check each other: the estimated angular rates
// (bytes 186..191) are about a tenth of a degree per second, which is
// the rate at which the attitude angles (bytes 192..197) were observed
// to be changing across this pass's run of beacons, and the field
// magnitude comes to 44 microtesla, which is what low Earth orbit
// gives. A layout shifted by even two bytes breaks all of that at once.
static const char EXT_BEACON_HEX[] =
    "204354533102009EB34D046E5F980166FA21B2A00100000501D7002900730D0000"
    "0102701701000000F13D60A902BA02C60E0000A30000000D000000340000001000"
    "00003600000046050000060201020002000248656C6C6F2066726F6D2043616C67"
    "617279546F53706163652046726F6E7469657253617400000000002058340019463"
    "D0D86221D001300EF28F0FF0300E42506000600E425000006000080BA0004014531"
    "8700000005047B0D050C080BDB0403F0BAFB3A00F7FFFCFF0000B11A7F0A460E";

// Read a hex string into bytes. Returns how many were written.
static size_t unhex(const char *h, uint8_t *out, size_t cap)
{
    size_t n = 0;
    while (h[0] != '\0' && h[1] != '\0' && n < cap) {
        unsigned v = 0;
        if (sscanf(h, "%2x", &v) != 1) break;
        out[n++] = (uint8_t) v;
        h += 2;
    }
    return n;
}

static size_t ext_fixture(uint8_t *buf, size_t cap)
{
    return unhex(EXT_BEACON_HEX, buf, cap);
}

static void test_ext_fixture_is_intact(void)
{
    uint8_t buf[256];
    size_t n = ext_fixture(buf, sizeof buf);
    // If the hex literal above ever gets mangled, say so once here
    // rather than as twenty confusing field failures below.
    tap_okf(n == sizeof(COMMS_beacon_extended_packet_t),
            "ext fixture: decodes to %zu bytes (want %zu)",
            n, sizeof(COMMS_beacon_extended_packet_t));
}

static void test_ext_is_and_version(void)
{
    uint8_t buf[256];
    size_t n = ext_fixture(buf, sizeof buf);

    tap_ok(beacon_is_extended(buf, n) == 1,
           "ext sniff: the real beacon is recognised");
    tap_ok(beacon_ext_version(buf, n) == 4,
           "ext sniff: its end_message says blob v4");

    // With the CSP CRC32 trailer left on, which is how a good half of
    // the stored ones arrived.
    uint8_t withcrc[256];
    memcpy(withcrc, buf, n);
    memcpy(withcrc + n, "\xde\xad\xbe\xef", 4);
    tap_ok(beacon_is_extended(withcrc, n + 4) == 1,
           "ext sniff: accepted with the 4-byte CSP CRC32 trailer on");
    tap_ok(beacon_ext_version(withcrc, n + 4) == 4,
           "ext sniff: and the version still reads through the trailer");

    // Lengths either side, and the basic beacon's own length.
    tap_ok(beacon_is_extended(buf, n - 1) == 0,
           "ext sniff: 197 bytes rejected");
    tap_ok(beacon_is_extended(buf, n + 1) == 0,
           "ext sniff: 199 bytes rejected");
    tap_ok(beacon_is_extended(buf, 130) == 0,
           "ext sniff: a basic beacon's length is not an extended beacon");
    tap_ok(beacon_is_extended(NULL, n) == 0, "ext sniff: NULL is safe");

    // The packet-type byte is required. 0x01 is the basic beacon's, and
    // an extended beacon whose type byte took a hit is not worth
    // guessing at when its length already collides with nothing else.
    uint8_t bad[256];
    memcpy(bad, buf, n);
    bad[0] = 0x01;
    tap_ok(beacon_is_extended(bad, n) == 0,
           "ext sniff: a wrong packet-type byte is rejected");

    // Either magic field alone is enough, but not neither. This is what
    // lets a beacon rescued with one bad byte still decode.
    memcpy(bad, buf, n);
    memcpy(bad + 1, "XXXX", 4);
    tap_ok(beacon_is_extended(bad, n) == 1,
           "ext sniff: a corrupted satellite_name still passes on the version tag");
    memcpy(bad, buf, n);
    bad[126] = 'Z';
    tap_ok(beacon_is_extended(bad, n) == 1,
           "ext sniff: a corrupted version tag still passes on the name");
    tap_ok(beacon_ext_version(bad, n) == 0,
           "ext sniff: but then the version reads as unknown, not as a digit");
    memcpy(bad, buf, n);
    memcpy(bad + 1, "XXXX", 4);
    bad[126] = 'Z';
    tap_ok(beacon_is_extended(bad, n) == 0,
           "ext sniff: both magic fields gone is rejected");

    // And the basic-beacon sniff must not claim it, or the dispatcher
    // would print 130 of its bytes and drop the rest.
    tap_ok(beacon_is_basic(buf, n) == 0,
           "ext sniff: the basic-beacon sniff does not claim it");
    tap_ok(tcmd_response_is(buf, n) == 0,
           "ext sniff: the tcmd_response sniff does not claim it");
    tap_ok(log_message_is(buf, n) == 0,
           "ext sniff: the log-message sniff does not claim it");
    tap_ok(bulk_file_is(buf, n) == 0,
           "ext sniff: the bulk-file sniff does not claim it");
}

static void test_magic_intact(void)
{
    uint8_t ext[256];
    const size_t n = ext_fixture(ext, sizeof ext);
    uint8_t basic[134];
    make_beacon_fixture(basic);

    tap_ok(beacon_magic_intact(ext, n) == 1,
           "magic: a real extended beacon passes");
    tap_ok(beacon_magic_intact(basic, 130) == 1,
           "magic: a basic beacon passes");
    tap_ok(beacon_magic_intact(basic, 134) == 1,
           "magic: and so does one with the CRC trailer on");

    // One byte of either magic field is enough to fail it. This is the
    // whole point of the function: it is stricter than the sniff, which
    // accepts these so a rescued beacon still reaches the panel.
    uint8_t t[256];
    memcpy(t, ext, n);
    t[2] = 'X';
    tap_ok(beacon_is_extended(t, n) == 1 && beacon_magic_intact(t, n) == 0,
           "magic: a broken satellite_name fails, where the sniff accepts");
    memcpy(t, ext, n);
    t[127] = 'Y';                       // the 'X' of " X4"
    tap_ok(beacon_is_extended(t, n) == 1 && beacon_magic_intact(t, n) == 0,
           "magic: a broken version tag fails, where the sniff accepts");
    memcpy(t, ext, n);
    t[128] = 'z';                       // the digit of " X4"
    tap_ok(beacon_magic_intact(t, n) == 0,
           "magic: a version tag whose digit is not a digit fails");

    memcpy(t, basic, 130);
    t[126] = 'e';                       // "END" -> "eND"
    tap_ok(beacon_is_basic(t, 130) == 1 && beacon_magic_intact(t, 130) == 0,
           "magic: a broken end_message fails on a basic beacon");
    memcpy(t, basic, 130);
    t[1] = 0x00;
    tap_ok(beacon_magic_intact(t, 130) == 0,
           "magic: a zeroed satellite_name fails");

    // Something that is not a beacon at all.
    uint8_t junk[130];
    memset(junk, 0, sizeof junk);
    tap_ok(beacon_magic_intact(junk, sizeof junk) == 0,
           "magic: 130 zero bytes fail");
    tap_ok(beacon_magic_intact(ext, 42) == 0,
           "magic: a length no beacon has fails");
    tap_ok(beacon_magic_intact(NULL, 130) == 0, "magic: NULL is safe");
}

static void test_ext_adcs_state(void)
{
    // The fixture's six ADCS state bytes are 45 31 87 00 00 00, read
    // out of the packet by hand. Unpacked by the firmware's own bit
    // layout that is: estimation mode 5 and control mode 4 from 0x45,
    // run mode 1 with both CubeControl units enabled from 0x31, three
    // wheels enabled and the Sun above the horizon from 0x87, and no
    // faults at all from the three zero bytes.
    const uint8_t packed[6] = { 0x45, 0x31, 0x87, 0x00, 0x00, 0x00 };
    beacon_ext_adcs_state_t st;
    beacon_ext_adcs_state(packed, &st);

    tap_ok(st.reported == 1, "adcs state: reported");
    tap_okf(st.estimation_mode == 5, "adcs state: estimation mode 5 (got %u)",
            (unsigned) st.estimation_mode);
    tap_okf(st.control_mode == 4, "adcs state: control mode 4 (got %u)",
            (unsigned) st.control_mode);
    tap_okf(st.run_mode == 1, "adcs state: run mode 1 (got %u)",
            (unsigned) st.run_mode);
    tap_okf(st.asgp4_mode == 0, "adcs state: asgp4 mode 0 (got %u)",
            (unsigned) st.asgp4_mode);
    tap_ok(st.cubecontrol_signal_enabled == 1 && st.cubecontrol_motor_enabled == 1,
           "adcs state: both CubeControl units enabled");
    tap_ok(st.cubesense1_enabled == 0 && st.cubesense2_enabled == 0,
           "adcs state: neither CubeSense enabled");
    tap_ok(st.cubewheel1_enabled == 1 && st.cubewheel2_enabled == 1
           && st.cubewheel3_enabled == 1,
           "adcs state: all three wheels enabled");
    tap_ok(st.cubestar_enabled == 0 && st.gps_receiver_enabled == 0
           && st.gps_lna_power_enabled == 0 && st.motor_driver_enabled == 0,
           "adcs state: star tracker, GPS and motor driver all off");
    tap_ok(st.sun_above_local_horizon == 1,
           "adcs state: the Sun was above the local horizon");
    tap_ok(st.cubesense1_comm_error == 0 && st.magnetometer_range_error == 0
           && st.startracker_overcurrent_detected == 0,
           "adcs state: no faults");

    // All six zero is how the blob leaves the field when the ADCS did
    // not answer, and must not read as a real mode-0 state.
    const uint8_t silent[6] = {0};
    beacon_ext_adcs_state(silent, &st);
    tap_ok(st.reported == 0, "adcs state: all-zero reads as not reported");

    // Each flag has to come out of its own bit. Walk one bit at a time
    // through the three fault bytes and check exactly one flag lands.
    // A copy-pasted shift (the likely mistake in forty near-identical
    // lines) shows up as two flags set, or none.
    const struct { int byte, bit; const char *name; } probes[] = {
        { 3, 0, "cubesense1_comm_error" },
        { 3, 7, "cubestar_comm_error" },
        { 4, 0, "magnetometer_range_error" },
        { 4, 5, "sun_sensor_range_error" },
        { 5, 2, "nadir_sensor_range_error" },
        { 5, 7, "startracker_overcurrent_detected" },
    };
    for (size_t i = 0; i < sizeof probes / sizeof probes[0]; i++) {
        uint8_t p[6] = {0};
        p[0] = 0x01;   // keep the packet from reading as "not reported"
        p[probes[i].byte] = (uint8_t) (1u << probes[i].bit);
        beacon_ext_adcs_state(p, &st);
        const int flags[] = {
            st.cubesense1_comm_error, st.cubesense2_comm_error,
            st.cubecontrol_signal_comm_error, st.cubecontrol_motor_comm_error,
            st.cubewheel1_comm_error, st.cubewheel2_comm_error,
            st.cubewheel3_comm_error, st.cubestar_comm_error,
            st.magnetometer_range_error,
            st.cam1_sram_overcurrent_detected, st.cam1_3v3_overcurrent_detected,
            st.cam1_sensor_busy_error, st.cam1_sensor_detection_error,
            st.sun_sensor_range_error,
            st.cam2_sram_overcurrent_detected, st.cam2_3v3_overcurrent_detected,
            st.cam2_sensor_busy_error, st.cam2_sensor_detection_error,
            st.nadir_sensor_range_error, st.rate_sensor_range_error,
            st.wheel_speed_range_error, st.coarse_sun_sensor_error,
            st.startracker_match_error, st.startracker_overcurrent_detected,
        };
        int set = 0;
        for (size_t k = 0; k < sizeof flags / sizeof flags[0]; k++)
            set += (flags[k] != 0);
        tap_okf(set == 1, "adcs state: byte %d bit %d sets exactly one fault "
                          "flag, %s (got %d set)",
                probes[i].byte, probes[i].bit, probes[i].name, set);
    }
}

static void test_ext_attitude_validity(void)
{
    uint8_t buf[256];
    size_t n = ext_fixture(buf, sizeof buf);
    const size_t st_off = offsetof(COMMS_beacon_extended_packet_t,
                                   adcs_current_state_1);

    // The fixture is in estimation mode 5, so its angles mean something.
    tap_ok(beacon_ext_attitude_is_valid(buf, n) == 1,
           "attitude validity: estimation mode 5 is valid");

    // The ADCS only fills the angles in modes 3 through 6. Walk every
    // mode and check the boundaries land where the firmware says.
    for (unsigned mode = 0; mode <= 7; mode++) {
        uint8_t t[256];
        memcpy(t, buf, n);
        t[st_off] = (uint8_t) ((t[st_off] & 0xF0) | mode);
        const int want = (mode >= 3 && mode <= 6);
        tap_okf(beacon_ext_attitude_is_valid(t, n) == want,
                "attitude validity: estimation mode %u is %s", mode,
                want ? "valid" : "not valid");
    }

    // A silent ADCS has no attitude either, whatever the angle bytes
    // happen to hold -- this is the case that would otherwise draw a
    // satellite pointing perfectly at nadir out of six zero bytes.
    uint8_t silent[256];
    memcpy(silent, buf, n);
    memset(silent + st_off, 0, 6);
    tap_ok(beacon_ext_attitude_is_valid(silent, n) == 0,
           "attitude validity: a silent ADCS has no attitude");
}

static void test_ext_print_fields(void)
{
    uint8_t buf[256];
    size_t n = ext_fixture(buf, sizeof buf);

    char out[4096] = {0};
    FILE *fp = fmemopen(out, sizeof out, "w");
    tap_ok(fp != NULL, "ext print: fmemopen for capture");
    if (fp == NULL) return;
    // Through the dispatcher, so the routing is covered too: an
    // extended beacon must not come out of cts1_packet_print as
    // anything else.
    cts1_packet_print(fp, NULL, buf, n);
    fclose(fp);

    // Every value here was read out of the bytes by hand. They span the
    // whole packet -- the shared basic-beacon part, the blob's own EPS
    // fields, the ADCS block, and the attitude on the very end -- so a
    // wrong offset anywhere shows up.
    const struct { const char *want; const char *what; } lines[] = {
        { "beacon_ext: name=\"CTS1\"",            "satellite name" },
        { "state=NOMINAL_TX",                     "operation state" },
        { "count=3443",                           "beacon count" },
        { "batt=15.857V 96%",                     "battery voltage and percent" },
        { "obc=13.50C",                           "OBC temperature" },
        { "uptime=20:03:21.118",                  "uptime" },
        { "epoch=2026-09-18T01:29:26Z",           "satellite clock" },
        { "tcmd queued=215 pending=41",           "telecommand counts" },
        { "msg=\"Hello from CalgaryToSpace FrontierSat\"", "friendly message" },
        { "blob=v4",                              "blob version" },
        { "osc=25MHz",                            "active oscillator" },
        { "obc_adc_batt=15.686V",                 "the OBC's own battery reading" },
        { "mpi_last_temp=13C",                    "last MPI temperature" },
        { "solar ch0 in=8.84V cur_in=29mA cur_out=19mA", "solar channel 0" },
        { "solar ch1 in=10.48V cur_in=-16mA cur_out=3mA",
          "solar channel 1, whose input current was negative" },
        { "solar ch3 in=9.70V cur_in=0mA cur_out=6mA", "solar channel 3" },
        { "batt_pack=0x8000",                     "battery pack status" },
        { "pack=enabled",                         "the pack-enabled bit, named" },
        { "avg_net=1.86W",                        "average net battery power" },
        { "avg_distributed=2.60W",                "average distributed power" },
        { "adcs run=ENABLED control=Y_WHEEL_STEADY estimation=FULL_EKF",
          "the ADCS modes, by name" },
        { "sun_up=1",                             "Sun above the horizon" },
        { "wheels=111",                           "three wheels enabled" },
        { "css 1-7,9=5 4 123 13 5 12 8 11",       "the eight coarse sun sensors" },
        { "mag body=12.43 -40.93 -10.94 uT",      "the magnetic field vector" },
        { "|B|=44.15 uT",                         "its magnitude" },
        { "rates mems_norm=0.58 est=-0.09 -0.04 0.00 deg/s", "the angular rates" },
        { "attitude roll=68.33 pitch=26.87 yaw=36.54 deg", "the attitude angles" },
    };
    for (size_t i = 0; i < sizeof lines / sizeof lines[0]; i++) {
        tap_okf(strstr(out, lines[i].want) != NULL,
                "ext print: %s -- \"%s\"", lines[i].what, lines[i].want);
    }

    // No faults in this one, so the faults line must be absent rather
    // than present and empty.
    tap_ok(strstr(out, "adcs faults:") == NULL,
           "ext print: a clean ADCS costs no faults line");

    // The dispatcher must not have printed it as a basic beacon: every
    // line is tagged beacon_ext, and none is tagged plain beacon.
    tap_ok(strstr(out, "\nbeacon: ") == NULL && strncmp(out, "beacon: ", 8) != 0,
           "ext print: no line is tagged as a basic beacon");
}

static void test_ext_print_faults_and_sentinels(void)
{
    uint8_t buf[256];
    size_t n = ext_fixture(buf, sizeof buf);
    const size_t st_off = offsetof(COMMS_beacon_extended_packet_t,
                                   adcs_current_state_1);

    // Set two faults in different bytes and check both are named.
    uint8_t t[256];
    memcpy(t, buf, n);
    t[st_off + 4] |= 0x01;   // magnetometer_range_error
    t[st_off + 5] |= 0x20;   // coarse_sun_sensor_error
    char out[4096] = {0};
    FILE *fp = fmemopen(out, sizeof out, "w");
    if (fp != NULL) {
        beacon_ext_print(fp, NULL, t, n);
        fclose(fp);
    }
    tap_ok(strstr(out, "adcs faults:") != NULL
           && strstr(out, "mag_range") != NULL
           && strstr(out, "css") != NULL,
           "ext print: two faults in different bytes are both named");

    // A silent ADCS says so and stops, rather than printing zeros as
    // sun sensors, a zero magnetic field and a level attitude.
    memcpy(t, buf, n);
    memset(t + st_off, 0, 6);
    memset(out, 0, sizeof out);
    fp = fmemopen(out, sizeof out, "w");
    if (fp != NULL) {
        beacon_ext_print(fp, NULL, t, n);
        fclose(fp);
    }
    tap_ok(strstr(out, "adcs did not answer") != NULL,
           "ext print: a silent ADCS is called out");
    tap_ok(strstr(out, "css 1-7,9=") == NULL
           && strstr(out, "mag body=") == NULL
           && strstr(out, "attitude roll=") == NULL,
           "ext print: and its zeroed sensor fields are not printed as readings");

    // The blob's own -9999 placeholder for an EPS query that failed has
    // to read as unavailable, not as -9.999 volts.
    memcpy(t, buf, n);
    const size_t ch0 = offsetof(COMMS_beacon_extended_packet_t,
                                eps_pcu_ch0_volt_in_mppt_mV);
    t[ch0] = 0xF1; t[ch0 + 1] = 0xD8;   // -9999 little-endian
    memset(out, 0, sizeof out);
    fp = fmemopen(out, sizeof out, "w");
    if (fp != NULL) {
        beacon_ext_print(fp, NULL, t, n);
        fclose(fp);
    }
    tap_ok(strstr(out, "solar ch0 in=n/a") != NULL,
           "ext print: the -9999 placeholder reads as n/a, not as a voltage");

    // The MPI temperature sentinels.
    const size_t mpi_off = offsetof(COMMS_beacon_extended_packet_t,
                                    mpi_last_temperature_C);
    const struct { uint8_t raw; const char *want; } mpi[] = {
        { 0x9D, "mpi_last_temp=inactive" },   // -99
        { 0x9E, "mpi_last_temp=err-98" },     // -98
        { 0xE7, "mpi_last_temp=-25C" },       // a real negative reading
    };
    for (size_t i = 0; i < sizeof mpi / sizeof mpi[0]; i++) {
        memcpy(t, buf, n);
        t[mpi_off] = mpi[i].raw;
        memset(out, 0, sizeof out);
        fp = fmemopen(out, sizeof out, "w");
        if (fp != NULL) {
            beacon_ext_print(fp, NULL, t, n);
            fclose(fp);
        }
        tap_okf(strstr(out, mpi[i].want) != NULL,
                "ext print: MPI temperature byte 0x%02x reads as \"%s\"",
                mpi[i].raw, mpi[i].want);
    }

    // The heater bit is the one battery-pack flag worth naming.
    memcpy(t, buf, n);
    const size_t bp = offsetof(COMMS_beacon_extended_packet_t,
                               eps_battery_pack_status_bitfield);
    t[bp] = 0x00; t[bp + 1] = 0x90;   // 0x9000: enabled, heater on
    memset(out, 0, sizeof out);
    fp = fmemopen(out, sizeof out, "w");
    if (fp != NULL) {
        beacon_ext_print(fp, NULL, t, n);
        fclose(fp);
    }
    tap_ok(strstr(out, "heater=on") != NULL,
           "ext print: the battery heater bit is named when it is set");
}

static void test_ext_summary(void)
{
    uint8_t buf[256];
    size_t n = ext_fixture(buf, sizeof buf);

    char out[256] = {0};
    int written = beacon_ext_summary(buf, n, out, sizeof out);
    tap_okf(written > 0 && written == (int) strlen(out),
            "ext summary: returns the length written (%d)", written);
    tap_ok(strstr(out, "CTS1 X4") != NULL,
           "ext summary: names the satellite and the blob version");
    tap_ok(strstr(out, "batt=15.86V/96%") != NULL,
           "ext summary: carries the battery state");
    tap_ok(strstr(out, "Y_WHEEL_STEADY rpy=68.3/26.9/36.5") != NULL,
           "ext summary: carries the control mode and the attitude");
    tap_ok(strstr(out, "rate=0.58deg/s") != NULL,
           "ext summary: carries the body rate");

    // In a mode with no attitude, the summary says which mode instead
    // of printing meaningless angles.
    uint8_t t[256];
    memcpy(t, buf, n);
    const size_t st_off = offsetof(COMMS_beacon_extended_packet_t,
                                   adcs_current_state_1);
    t[st_off] = (uint8_t) ((t[st_off] & 0xF0) | 1u);   // MEMS rate sensing
    beacon_ext_summary(t, n, out, sizeof out);
    tap_ok(strstr(out, "est=MEMS_RATE") != NULL && strstr(out, "rpy=") == NULL,
           "ext summary: no attitude means the mode is reported instead");

    memcpy(t, buf, n);
    memset(t + st_off, 0, 6);
    beacon_ext_summary(t, n, out, sizeof out);
    tap_ok(strstr(out, "adcs=silent") != NULL,
           "ext summary: a silent ADCS is called out");

    // Not an extended beacon: nothing written, zero returned.
    uint8_t basic[134];
    make_beacon_fixture(basic);
    out[0] = 'x';
    tap_ok(beacon_ext_summary(basic, 130, out, sizeof out) == 0 && out[0] == '\0',
           "ext summary: a basic beacon yields nothing");

    // And the panel dispatcher routes the type to this summary.
    cts1_rx_panel_summary(buf, n, COMMS_PACKET_TYPE_BEACON_EXTENDED,
                          /*rs_errs=*/0, out, sizeof out);
    tap_ok(strstr(out, "CTS1 X4") != NULL,
           "ext summary: the RX panel dispatcher routes 0x20 here");
    cts1_rx_panel_summary(buf, n, COMMS_PACKET_TYPE_BEACON_EXTENDED,
                          /*rs_errs=*/-2, out, sizeof out);
    tap_ok(strncmp(out, RX_RS_FAIL_TOKEN, strlen(RX_RS_FAIL_TOKEN)) == 0,
           "ext summary: an uncorrectable frame is marked, not parsed");
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
    test_cts1_rx_panel_summary_rs_fail();
    test_ext_fixture_is_intact();
    test_ext_is_and_version();
    test_magic_intact();
    test_ext_adcs_state();
    test_ext_attitude_validity();
    test_ext_print_fields();
    test_ext_print_faults_and_sentinels();
    test_ext_summary();
    return tap_done();
}
