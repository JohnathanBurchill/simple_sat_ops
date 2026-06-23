/*

    Simple Satellite Operations  unit_tests/sso_ipc_codec_selftest.c

    Coverage for src/ipc/sso_ipc_codec.c — the newline-JSON wire codec that
    every operator/viewer link rides on. The encoder emits a fixed set of
    keys (conditional on event type + which fields are set); the decoder
    fills whatever keys it sees and tolerates unknown ones for forward
    compatibility. A silent key-name typo or field-swap here would mis-render
    a viewer or drop a telecommand mirror, so this pins the round-trip.

    What's covered:
      - Every event type maps to a name and back (table integrity); unknown
        / NULL names decode to SSO_EVT_UNKNOWN.
      - encode -> decode round-trips the common HELLO fields incl. the ts.
      - A populated STATE event round-trips its scalars, flags, and the
        SGP4 prediction doubles (within the wire's %.6g precision).
      - The RX-panel mirror round-trips: per-type counts, a hex-encoded
        payload preview, and the activity ribbon incl. NEGATIVE peak dBFS
        (two's-complement int8 over hex — the easiest path to get wrong).
      - A TX event round-trips its ascii/payload, CSP header bytes, the
        safety-gate flags, and the not-sent reason.
      - sso_event_set_roster embeds an array that survives the round-trip,
        and overflows the roster buffer cleanly (-1).
      - Strings with quote / backslash / newline / tab escape and unescape
        back to the exact bytes.
      - Decoder robustness: NULL args, a line with no "t", garbage, and an
        unknown field alongside known ones.
      - Encoder rejects NULL args and a too-small buffer.
      - Decoder tolerates a line with or without the trailing newline.

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

#include "sso_ipc.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// Doubles cross the wire as %.6g, so compare with a relative tolerance.
// Wide enough to absorb 6-significant-figure rounding (e.g. a Julian date
// ~2.46e6), tight enough that a field-swap to a different value is caught.
static int deq(double a, double b)
{
    return fabs(a - b) <= 1e-6 + 1e-6 * fabs(b);
}

int main(void)
{
    char line[8192];

    // --- Event type table ---------------------------------------------
    for (int i = 0; i <= SSO_EVT_AUDIO; ++i) {
        sso_event_type_t t = (sso_event_type_t) i;
        const char *nm = sso_event_type_name(t);
        tap_okf(nm != NULL && sso_event_type_from_name(nm) == t,
                "type %d round-trips through its name \"%s\"", i, nm ? nm : "(null)");
    }
    tap_ok(sso_event_type_from_name("no-such-event") == SSO_EVT_UNKNOWN,
           "unknown name decodes to SSO_EVT_UNKNOWN");
    tap_ok(sso_event_type_from_name(NULL) == SSO_EVT_UNKNOWN,
           "NULL name decodes to SSO_EVT_UNKNOWN");

    // --- Minimal HELLO round-trip -------------------------------------
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_HELLO);
        snprintf(e.from, sizeof e.from, "viewer-1");
        snprintf(e.user, sizeof e.user, "alice");
        snprintf(e.role, sizeof e.role, "viewer");

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode HELLO returns 0");
        tap_ok(sso_event_decode(line, &d) == 0, "decode HELLO returns 0");
        tap_ok(d.type == SSO_EVT_HELLO, "HELLO type preserved");
        tap_ok(strcmp(d.from, "viewer-1") == 0, "HELLO from preserved");
        tap_ok(strcmp(d.user, "alice") == 0, "HELLO user preserved");
        tap_ok(strcmp(d.role, "viewer") == 0, "HELLO role preserved");
        tap_ok(e.ts[0] && strcmp(d.ts, e.ts) == 0, "ts string round-trips");
    }

    // --- Populated STATE round-trip -----------------------------------
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_STATE);
        e.has_state = 1;
        snprintf(e.satellite, sizeof e.satellite, "ISS (ZARYA)");
        snprintf(e.source, sizeof e.source, "tle-only");
        e.az = 123.5; e.el = -4.25;
        e.freq_hz = 436150000; e.doppler_hz = -1234.5;
        snprintf(e.rx_status, sizeof e.rx_status, "LOCK");
        snprintf(e.tx_status, sizeof e.tx_status, "idle");
        snprintf(e.tle_path, sizeof e.tle_path, "/tmp/amateur.tle");
        e.target_az = 200.75; e.target_el = 45.5;
        e.flip = 1; e.in_pass = 1; e.tracking = 1; e.has_rotator = 1;
        snprintf(e.idesg, sizeof e.idesg, "98067A");
        e.jul_utc = 2460000.5;
        e.max_el = 78.5; e.range_km = 1234.5; e.range_rate_kms = -3.25;
        e.lat_deg = 50.8688; e.lon_deg = -114.291; e.alt_km = 420.5;
        e.speed_kms = 7.66;

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode STATE returns 0");
        tap_ok(sso_event_decode(line, &d) == 0, "decode STATE returns 0");
        tap_ok(d.type == SSO_EVT_STATE, "STATE type preserved");
        tap_ok(d.has_state == 1, "STATE has_state inferred on decode");
        tap_ok(strcmp(d.satellite, "ISS (ZARYA)") == 0, "STATE satellite preserved");
        tap_ok(strcmp(d.source, "tle-only") == 0, "STATE source preserved");
        tap_ok(deq(d.az, 123.5) && deq(d.el, -4.25), "STATE az/el preserved");
        tap_ok(d.freq_hz == 436150000, "STATE freq_hz preserved");
        tap_ok(deq(d.doppler_hz, -1234.5), "STATE doppler preserved");
        tap_ok(strcmp(d.rx_status, "LOCK") == 0, "STATE rx_status preserved");
        tap_ok(strcmp(d.tle_path, "/tmp/amateur.tle") == 0, "STATE tle_path preserved");
        tap_ok(deq(d.target_az, 200.75) && deq(d.target_el, 45.5), "STATE target az/el preserved");
        tap_ok(d.flip && d.in_pass && d.tracking && d.has_rotator, "STATE flags preserved");
        tap_ok(strcmp(d.idesg, "98067A") == 0, "STATE idesg preserved");
        tap_ok(deq(d.jul_utc, 2460000.5), "STATE jul_utc preserved (wire precision)");
        tap_ok(deq(d.max_el, 78.5) && deq(d.range_km, 1234.5)
               && deq(d.range_rate_kms, -3.25), "STATE prediction doubles preserved");
        tap_ok(deq(d.lat_deg, 50.8688) && deq(d.lon_deg, -114.291)
               && deq(d.alt_km, 420.5) && deq(d.speed_kms, 7.66),
               "STATE observer doubles preserved");
    }

    // --- source field: present when set, omitted (not "source") when empty -
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_STATE);
        e.has_state = 1;
        snprintf(e.satellite, sizeof e.satellite, "FRONTIERSAT");
        snprintf(e.source, sizeof e.source, "operator");
        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0,
               "encode STATE (source=operator) returns 0");
        tap_ok(strstr(line, "\"source\":\"operator\"") != NULL,
               "operator source emitted on the wire");
        tap_ok(sso_event_decode(line, &d) == 0
               && strcmp(d.source, "operator") == 0,
               "operator source round-trips");

        // Empty source: the encoder must not emit a "source" key at all.
        sso_event_init(&e, SSO_EVT_STATE);
        e.has_state = 1;
        snprintf(e.satellite, sizeof e.satellite, "FRONTIERSAT");
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0,
               "encode STATE (no source) returns 0");
        tap_ok(strstr(line, "\"source\"") == NULL,
               "empty source omitted from the wire");
        tap_ok(sso_event_decode(line, &d) == 0 && d.source[0] == '\0',
               "decoded source empty when absent");
    }

    // --- RX-panel mirror round-trip -----------------------------------
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_STATE);
        e.has_state = 1;
        e.rx_have_session = 1; e.rx_rec_active = 1;
        e.rx_freq_hz = 436.15e6; e.rx_peak_dbfs = -42.5; e.rx_rms_dbfs = -60.25;
        e.rx_frames_total = 7; e.rx_frames_pcm = 3; e.rx_frames_vit = 2;
        snprintf(e.rx_last_frame_summary, sizeof e.rx_last_frame_summary, "beacon");
        e.rx_age_s = 1.5;
        e.rx_pt_count[0] = 5;
        e.rx_pt_payload_len[0] = 3;
        e.rx_pt_payload[0][0] = 0xDE; e.rx_pt_payload[0][1] = 0xAD; e.rx_pt_payload[0][2] = 0xBE;
        snprintf(e.rx_pt_summary[0], sizeof e.rx_pt_summary[0], "PT0 summary");
        e.rx_ribbon_n = 4;
        memcpy(e.rx_ribbon, ".-.-", 5);  // includes the nul
        e.rx_ribbon_peak[0] = -90; e.rx_ribbon_peak[1] = -30;
        e.rx_ribbon_peak[2] = 0;   e.rx_ribbon_peak[3] = 12;

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode STATE+rx returns 0");
        tap_ok(sso_event_decode(line, &d) == 0, "decode STATE+rx returns 0");
        tap_ok(d.rx_have_session && d.rx_rec_active, "rx session/record flags preserved");
        tap_ok(deq(d.rx_freq_hz, 436.15e6), "rx_freq_hz preserved");
        tap_ok(deq(d.rx_peak_dbfs, -42.5) && deq(d.rx_rms_dbfs, -60.25), "rx peak/rms preserved");
        tap_ok(d.rx_frames_total == 7 && d.rx_frames_pcm == 3 && d.rx_frames_vit == 2,
               "rx frame counters preserved");
        tap_ok(strcmp(d.rx_last_frame_summary, "beacon") == 0, "rx last-frame summary preserved");
        tap_ok(deq(d.rx_age_s, 1.5), "rx_age_s preserved");
        tap_ok(d.rx_pt_count[0] == 5 && d.rx_pt_payload_len[0] == 3, "rx_pt count/len preserved");
        tap_ok(d.rx_pt_payload[0][0] == 0xDE && d.rx_pt_payload[0][1] == 0xAD
               && d.rx_pt_payload[0][2] == 0xBE, "rx_pt payload bytes preserved (hex)");
        tap_ok(strcmp(d.rx_pt_summary[0], "PT0 summary") == 0, "rx_pt summary preserved");
        tap_ok(d.rx_ribbon_n == 4 && strcmp(d.rx_ribbon, ".-.-") == 0, "ribbon string preserved");
        tap_ok(d.rx_ribbon_peak[0] == -90 && d.rx_ribbon_peak[1] == -30
               && d.rx_ribbon_peak[2] == 0 && d.rx_ribbon_peak[3] == 12,
               "ribbon negative peak dBFS preserved (two's-complement hex)");
    }

    // --- TX event round-trip ------------------------------------------
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_TX_NOT_SENT);
        snprintf(e.ascii, sizeof e.ascii, "ascii:CTS1+ping");
        snprintf(e.tx_payload_kind, sizeof e.tx_payload_kind, "ascii");
        snprintf(e.tx_payload, sizeof e.tx_payload, "CTS1+ping");
        e.tx_csp_src = 1; e.tx_csp_dst = 2; e.tx_csp_dport = 3;
        e.tx_csp_sport = 4; e.tx_csp_prio = 2;
        e.tx_freq_hz = 436150000; e.tx_gain_db = 12.5;
        e.tx_allow_tx = 1; e.tx_allow_high_power = 1; e.tx_allow_hf_tx = 0;
        e.tx_repeat = 3; e.tx_gap_ms = 100;
        snprintf(e.tx_not_sent_reason, sizeof e.tx_not_sent_reason, "rejected: no HMAC");
        snprintf(e.tx_origin, sizeof e.tx_origin, "auto-cmd (file)");

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode TX returns 0");
        tap_ok(strstr(line, "\"tx_org\":\"auto-cmd (file)\"") != NULL,
               "TX origin emitted on the wire");
        tap_ok(sso_event_decode(line, &d) == 0, "decode TX returns 0");
        tap_ok(d.type == SSO_EVT_TX_NOT_SENT, "TX type preserved");
        tap_ok(strcmp(d.tx_origin, "auto-cmd (file)") == 0, "TX origin preserved");
        tap_ok(strcmp(d.ascii, "ascii:CTS1+ping") == 0, "TX ascii preserved");
        tap_ok(strcmp(d.tx_payload_kind, "ascii") == 0 && strcmp(d.tx_payload, "CTS1+ping") == 0,
               "TX payload kind/text preserved");
        tap_ok(d.tx_csp_src == 1 && d.tx_csp_dst == 2 && d.tx_csp_dport == 3
               && d.tx_csp_sport == 4 && d.tx_csp_prio == 2, "TX CSP header bytes preserved");
        tap_ok(d.tx_freq_hz == 436150000 && deq(d.tx_gain_db, 12.5), "TX freq/gain preserved");
        tap_ok(d.tx_allow_tx == 1 && d.tx_allow_high_power == 1 && d.tx_allow_hf_tx == 0,
               "TX safety-gate flags preserved");
        tap_ok(d.tx_repeat == 3 && d.tx_gap_ms == 100, "TX repeat/gap preserved");
        tap_ok(strcmp(d.tx_not_sent_reason, "rejected: no HMAC") == 0, "TX not-sent reason preserved");

        // Empty origin: the encoder must not emit a "tx_org" key at all
        // (older operators / previews leave it blank).
        e.tx_origin[0] = '\0';
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode TX (no origin) returns 0");
        tap_ok(strstr(line, "\"tx_org\"") == NULL, "empty TX origin omitted from the wire");
        tap_ok(sso_event_decode(line, &d) == 0 && d.tx_origin[0] == '\0',
               "decoded TX origin empty when absent");
    }

    // --- Roster embed / round-trip / overflow -------------------------
    {
        sso_roster_entry_t r[2];
        memset(r, 0, sizeof r);
        snprintf(r[0].user, sizeof r[0].user, "alice");
        snprintf(r[0].role, sizeof r[0].role, "operator");
        snprintf(r[0].since, sizeof r[0].since, "2026-06-14T00:00:00.000Z");
        snprintf(r[1].user, sizeof r[1].user, "bob");
        snprintf(r[1].role, sizeof r[1].role, "viewer");
        snprintf(r[1].since, sizeof r[1].since, "2026-06-14T00:01:00.000Z");

        sso_event_t e;
        sso_event_init(&e, SSO_EVT_STATE);
        e.has_state = 1;
        tap_ok(sso_event_set_roster(&e, r, 2) == 0, "set_roster returns 0");
        tap_ok(e.roster_json[0] == '[', "roster_json holds a JSON array");

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode STATE+roster returns 0");
        tap_ok(sso_event_decode(line, &d) == 0, "decode STATE+roster returns 0");
        tap_ok(strcmp(d.roster_json, e.roster_json) == 0, "roster_json round-trips verbatim");
        tap_ok(strstr(d.roster_json, "alice") && strstr(d.roster_json, "bob"),
               "roster entries present after round-trip");

        sso_roster_entry_t big[64];
        memset(big, 0, sizeof big);
        for (int i = 0; i < 64; ++i) {
            snprintf(big[i].user, sizeof big[i].user, "user-%02d", i);
            snprintf(big[i].role, sizeof big[i].role, "viewer");
        }
        tap_ok(sso_event_set_roster(&e, big, 64) == -1, "set_roster overflow rejected (-1)");
    }

    // --- String escaping ----------------------------------------------
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_STATE);
        e.has_state = 1;
        const char *tricky = "q=\" b=\\ nl=\n tab=\t end";
        snprintf(e.rx_status, sizeof e.rx_status, "%s", tricky);

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode escaped string returns 0");
        tap_ok(strchr(line, '\n') == strrchr(line, '\n'),
               "encoded line has exactly one newline (the embedded one is escaped)");
        tap_ok(sso_event_decode(line, &d) == 0, "decode escaped string returns 0");
        tap_ok(strcmp(d.rx_status, tricky) == 0,
               "quote/backslash/newline/tab round-trip to exact bytes");
    }

    // --- Live-audio relay events --------------------------------------
    // audio-ctl: viewer -> operator. enable is a bool that must survive even
    // when false (json_field_bool always emits it); quality is optional.
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_AUDIO_CTL);
        e.audio_enable  = 1;
        e.audio_quality = 0.35;

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode audio-ctl returns 0");
        tap_ok(strstr(line, "\"t\":\"audio-ctl\"") != NULL, "audio-ctl type name on the wire");
        tap_ok(strstr(line, "\"enable\":true") != NULL, "audio-ctl enable=true on the wire");
        tap_ok(sso_event_decode(line, &d) == 0, "decode audio-ctl returns 0");
        tap_ok(d.type == SSO_EVT_AUDIO_CTL, "audio-ctl type preserved");
        tap_ok(d.audio_enable == 1, "audio-ctl enable preserved");
        tap_ok(deq(d.audio_quality, 0.35), "audio-ctl quality preserved");

        sso_event_init(&e, SSO_EVT_AUDIO_CTL);
        e.audio_enable = 0;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode audio-ctl(disable) returns 0");
        tap_ok(strstr(line, "\"enable\":false") != NULL, "audio-ctl enable=false on the wire");
        tap_ok(sso_event_decode(line, &d) == 0 && d.audio_enable == 0,
               "audio-ctl disable preserved");
    }

    // audio-status: operator -> viewer. state + sr/ch on "on"; reason on the
    // failure states (reason rides the common field, emitted once near the top).
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_AUDIO_STATUS);
        snprintf(e.audio_state, sizeof e.audio_state, "on");
        e.audio_sr = 96000; e.audio_ch = 1;

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode audio-status returns 0");
        tap_ok(sso_event_decode(line, &d) == 0, "decode audio-status returns 0");
        tap_ok(d.type == SSO_EVT_AUDIO_STATUS, "audio-status type preserved");
        tap_ok(strcmp(d.audio_state, "on") == 0, "audio-status state preserved");
        tap_ok(d.audio_sr == 96000 && d.audio_ch == 1, "audio-status sr/ch preserved");

        sso_event_init(&e, SSO_EVT_AUDIO_STATUS);
        snprintf(e.audio_state, sizeof e.audio_state, "unavailable");
        snprintf(e.reason, sizeof e.reason, "no operator");
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode audio-status(unavail) returns 0");
        tap_ok(sso_event_decode(line, &d) == 0
               && strcmp(d.audio_state, "unavailable") == 0
               && strcmp(d.reason, "no operator") == 0,
               "audio-status unavailable + reason preserved");
    }

    // audio frame: operator -> viewer. seq MUST be emitted even when 0 (the
    // first frame), start/sr/ch ride only the start frame, data is base64.
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_AUDIO);
        e.audio_seq = 0; e.audio_start = 1; e.audio_sr = 96000; e.audio_ch = 1;
        snprintf(e.audio_b64, sizeof e.audio_b64, "T2dnUwACAAAA");  // "OggS"-prefixed

        sso_event_t d;
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode audio frame returns 0");
        tap_ok(strstr(line, "\"seq\":0") != NULL, "audio seq=0 emitted (not dropped as zero)");
        tap_ok(strstr(line, "\"start\":true") != NULL, "audio start emitted on the wire");
        tap_ok(strstr(line, "\"data\":\"T2dnUwACAAAA\"") != NULL, "audio base64 data on the wire");
        tap_ok(sso_event_decode(line, &d) == 0, "decode audio frame returns 0");
        tap_ok(d.type == SSO_EVT_AUDIO, "audio type preserved");
        tap_ok(d.audio_seq == 0 && d.audio_start == 1, "audio seq/start preserved");
        tap_ok(d.audio_sr == 96000 && d.audio_ch == 1, "audio start-frame sr/ch preserved");
        tap_ok(strcmp(d.audio_b64, "T2dnUwACAAAA") == 0, "audio base64 data round-trips");

        sso_event_init(&e, SSO_EVT_AUDIO);
        e.audio_seq = 42;
        snprintf(e.audio_b64, sizeof e.audio_b64, "QUJD");
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode audio frame(2) returns 0");
        tap_ok(strstr(line, "\"start\"") == NULL, "non-start audio frame omits start");
        tap_ok(sso_event_decode(line, &d) == 0 && d.audio_seq == 42
               && d.audio_start == 0 && strcmp(d.audio_b64, "QUJD") == 0,
               "non-start audio frame round-trips");
    }

    // --- Decoder robustness -------------------------------------------
    {
        sso_event_t d;
        tap_ok(sso_event_decode(NULL, &d) == -1, "decode(NULL line) rejected");
        tap_ok(sso_event_decode("{\"t\":\"hello\"}", NULL) == -1, "decode(NULL evt) rejected");
        tap_ok(sso_event_decode("{}", &d) == -1, "decode without \"t\" rejected");
        tap_ok(sso_event_decode("not json at all", &d) == -1, "decode garbage rejected");
        tap_ok(sso_event_decode("{\"t\":\"hello\",\"zzz\":123,\"from\":\"x\"}", &d) == 0,
               "decode tolerates an unknown field");
        tap_ok(d.type == SSO_EVT_HELLO && strcmp(d.from, "x") == 0,
               "known fields still parsed alongside the unknown one");
    }

    // --- Encoder argument / buffer guards -----------------------------
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_HELLO);
        char tiny[8];
        tap_ok(sso_event_encode(&e, tiny, sizeof tiny) == -1, "encode into too-small buffer rejected");
        tap_ok(sso_event_encode(NULL, line, sizeof line) == -1, "encode(NULL evt) rejected");
        tap_ok(sso_event_encode(&e, NULL, 100) == -1, "encode(NULL out) rejected");
    }

    // --- Trailing-newline tolerance -----------------------------------
    {
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_BYE);
        snprintf(e.from, sizeof e.from, "z");
        tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "encode BYE returns 0");

        sso_event_t d;
        tap_ok(sso_event_decode(line, &d) == 0 && d.type == SSO_EVT_BYE,
               "decode tolerates the trailing newline");

        char nonl[8192];
        snprintf(nonl, sizeof nonl, "%s", line);
        size_t L = strlen(nonl);
        if (L && nonl[L - 1] == '\n') nonl[L - 1] = '\0';
        tap_ok(sso_event_decode(nonl, &d) == 0 && d.type == SSO_EVT_BYE,
               "decode tolerates a missing trailing newline");
    }

    return tap_done();
}
