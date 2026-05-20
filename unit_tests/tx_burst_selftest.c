/*

    Simple Satellite Operations  unit_tests/tx_burst_selftest.c

    Covers the two policy behaviours simple_sat_ops promises about
    every uplink burst:

      a) HMAC signing is ON by default. The compose modal stages a key
         from --hmac-keyfile (or the shared/per-user fallback path),
         hands it to tx_burst_build_frame, and the AX100 layer appends
         a 4-byte HMAC trailer the satellite verifies. Passing
         hmac_key=NULL/0 explicitly disables it — useful for offline
         bench tests but the operator never gets that path silently.

      b) TX Doppler shift is ON by default. simple_sat_ops computes
         f_tx = carrier / (1 - range_rate_km_s / c) every tick and
         stages that into the burst request. enable=0 short-circuits
         to the bare nominal carrier (used when --no-doppler-correction
         is set or when SGP4 hasn't produced a valid range rate yet).

    Both behaviours are exercised against the helpers tx_burst.h
    exports so the test runs without UHD / ncurses / SGP4. The helpers
    are the same ones simple_sat_ops calls per tick.

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

#include "ax100.h"
#include "b210_rx_tx_core.h"
#include "csp.h"
#include "tap.h"
#include "tx_burst.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Recording stub for b210_rx_tx_core_burst. tx_burst.c calls this with
// a fully-built IQ buffer + target frequency at the end of the burst
// path — capturing it lets the integration tests below assert what
// simple_sat_ops would actually transmit, without dragging in libuhd
// or opening a real device.
//
// captured_iq is copied (not aliased) since tx_burst_run frees the
// caller's buffer before returning.
static struct {
    int      called;
    double   tx_freq_hz;
    double   tx_rate_hz;
    double   tx_gain_db;
    double   rx_resume_freq_hz;
    int16_t *iq;
    size_t   n_samps;
} g_capture;

static void capture_reset(void)
{
    free(g_capture.iq);
    memset(&g_capture, 0, sizeof g_capture);
}

int b210_rx_tx_core_burst(b210_rx_tx_core_t *core,
                          const b210_rx_tx_core_burst_params_t *p)
{
    (void) core;
    g_capture.called++;
    g_capture.tx_freq_hz        = p->tx_freq_hz;
    g_capture.tx_rate_hz        = p->tx_rate_hz;
    g_capture.tx_gain_db        = p->tx_gain_db;
    g_capture.rx_resume_freq_hz = p->rx_resume_freq_hz;
    g_capture.n_samps           = p->n_samps;
    free(g_capture.iq);
    g_capture.iq = malloc(p->n_samps * 2 * sizeof(int16_t));
    if (g_capture.iq) {
        memcpy(g_capture.iq, p->iq, p->n_samps * 2 * sizeof(int16_t));
    }
    // Return success so tx_burst_run reports TX_BURST_OK and the
    // calling integration test can assert on the captured params.
    return 0;
}

// FrontierSat simplex carrier, copied here to avoid pulling in
// src/beacon/frontiersat.h (which carries flight-firmware vendor
// definitions the selftest doesn't need).
#define TEST_CARRIER_HZ 436150000.0

// A representative range rate for a LEO bird at AOS/LOS. Magnitude
// around 6 km/s — comfortably inside the ±7 km/s envelope a 400 km
// circular orbit produces and well above the helper's underflow
// guard.
#define TEST_RANGE_RATE_RECEDING_KMS  6.0   // LOS-end of pass
#define TEST_RANGE_RATE_APPROACHING_KMS (-6.0)  // AOS-start of pass

// ----------------------------------------------------------------
//  a) HMAC default-on / disable
// ----------------------------------------------------------------

// Decoder helper: re-frame the wire bytes through ax100_unframe with
// the supplied HMAC opts and return whether the trailer validated.
static int decode_hmac_ok(const uint8_t *frame, size_t frame_len,
                          const uint8_t *key, size_t key_len)
{
    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    opts.reed_solomon = 1;     // tx_burst_build_frame pins RS=1
    if (key && key_len > 0) {
        opts.hmac_key = key;
        opts.hmac_key_len = key_len;
    }
    // tx_burst_build_frame emits the AX100 frame with the default
    // 32-byte 0xAA prefill at the front; the unframer expects to land
    // AT the ASM, so skip the prefill.
    if (frame_len <= 32u) return -1;
    const uint8_t *in = frame + 32;
    size_t in_len = frame_len - 32 - 1u;   // strip 1-byte tailfill

    uint8_t out[4096];
    int hmac_ok = -2;
    ssize_t r = ax100_unframe(in, in_len, &opts, out, sizeof out,
                              NULL, &hmac_ok, NULL, NULL, NULL);
    if (r < 0) return -1;
    return hmac_ok;
}

static void test_hmac_default_on(void)
{
    const csp_v1_header_t hdr = {
        .prio = 2, .src = 10, .dst = 1,
        .dport = 7, .sport = 16, .flags = 0,
    };
    const uint8_t payload[] = "CTS1+ping";
    // Operator-supplied HMAC key — simple_sat_ops loads this once at
    // startup from --hmac-keyfile (or the shared fallback path) and
    // hands the bytes through to every tx_burst_run call.
    const uint8_t key[] = "frontiersat-operator-key";

    uint8_t frame[4200];
    ssize_t n = tx_burst_build_frame(payload, sizeof payload - 1, &hdr,
                                      key, sizeof key - 1,
                                      frame, sizeof frame);
    tap_okf(n > 0, "default-on: tx_burst_build_frame with key returns %zd bytes", n);

    // The frame must verify against the SAME key (signed correctly).
    int ok = decode_hmac_ok(frame, (size_t) n, key, sizeof key - 1);
    tap_okf(ok == 1,
            "default-on: receiver with matching key sees hmac_ok=1 (got %d)",
            ok);

    // A wrong key must not validate — proves the signature is data-
    // dependent, not a constant trailer. ax100_unframe with RS+HMAC
    // enters its brute-force length search on validation failure and
    // ultimately returns -1 when no length-candidate's HMAC matches,
    // so accept either "decoder rejected" (-1) or "explicit hmac_ok=0"
    // — both signal "wrong key was caught".
    const uint8_t wrong_key[] = "not-the-operator-key";
    ok = decode_hmac_ok(frame, (size_t) n, wrong_key, sizeof wrong_key - 1);
    tap_okf(ok != 1,
            "default-on: wrong key never validates (got %d, want != 1)",
            ok);
}

static void test_hmac_explicitly_disabled(void)
{
    const csp_v1_header_t hdr = {
        .prio = 2, .src = 10, .dst = 1,
        .dport = 7, .sport = 16, .flags = 0,
    };
    const uint8_t payload[] = "CTS1+ping";

    // Pass key=NULL / len=0: tx_burst_build_frame must skip the HMAC
    // trailer. This path is what a future --no-hmac bench mode (or an
    // offline test scaffold) would take.
    uint8_t frame[4200];
    ssize_t n = tx_burst_build_frame(payload, sizeof payload - 1, &hdr,
                                      NULL, 0,
                                      frame, sizeof frame);
    tap_okf(n > 0,
            "disabled: tx_burst_build_frame with key=NULL returns %zd bytes",
            n);

    // The unframer with NO key reports hmac_ok = -1 ("not attempted").
    int ok = decode_hmac_ok(frame, (size_t) n, NULL, 0);
    tap_okf(ok == -1,
            "disabled: receiver with no key reports hmac_ok=-1 (got %d)",
            ok);

    // And with a key, the frame should NOT validate — no trailer was
    // appended, so whatever 4 bytes RS pulls out of the payload tail
    // won't match HMAC(key, payload).
    const uint8_t key[] = "frontiersat-operator-key";
    ok = decode_hmac_ok(frame, (size_t) n, key, sizeof key - 1);
    tap_okf(ok != 1,
            "disabled: unsigned frame doesn't accidentally validate (got %d)",
            ok);
}

static void test_hmac_frame_includes_trailer(void)
{
    // Same payload, same CSP header, same RS settings — the only thing
    // that changes between the two builds is the HMAC key. tx_burst
    // pins reed_solomon=1, which pads the inner block to 223 bytes
    // before encoding to 255 — so the on-wire size is the same in both
    // cases. We can still see the trailer presence indirectly via
    // payload content: the unsigned and signed frames must differ at
    // the bit level (HMAC trailer bytes change the pre-RS data).
    const csp_v1_header_t hdr = {
        .prio = 2, .src = 10, .dst = 1,
        .dport = 7, .sport = 16, .flags = 0,
    };
    const uint8_t payload[] = "hello";
    const uint8_t key[] = "k";

    uint8_t unsigned_frame[4200];
    uint8_t signed_frame[4200];
    ssize_t n_unsigned = tx_burst_build_frame(payload, sizeof payload - 1, &hdr,
                                               NULL, 0,
                                               unsigned_frame,
                                               sizeof unsigned_frame);
    ssize_t n_signed = tx_burst_build_frame(payload, sizeof payload - 1, &hdr,
                                             key, sizeof key - 1,
                                             signed_frame, sizeof signed_frame);
    tap_okf(n_unsigned > 0 && n_signed > 0,
            "trailer audit: both builds succeed (unsigned=%zd, signed=%zd)",
            n_unsigned, n_signed);

    // Frames must differ — HMAC trailer changes the bytes RS sees.
    int differ = (n_unsigned != n_signed)
              || memcmp(unsigned_frame, signed_frame,
                        (size_t)(n_unsigned < n_signed ? n_unsigned
                                                       : n_signed)) != 0;
    tap_okf(differ,
            "trailer audit: signed and unsigned frames have different bytes");
}

// ----------------------------------------------------------------
//  b) TX Doppler default-on / disable
// ----------------------------------------------------------------

static void test_doppler_disabled_returns_nominal(void)
{
    // enable=0 must return the bare carrier regardless of range rate
    // (this is the --no-doppler-correction path).
    long f0 = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ, 0.0, 0);
    tap_okf(f0 == (long) TEST_CARRIER_HZ,
            "disabled: zero range rate -> carrier (%ld vs %ld)",
            f0, (long) TEST_CARRIER_HZ);

    long f_los = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ,
                                          TEST_RANGE_RATE_RECEDING_KMS, 0);
    tap_okf(f_los == (long) TEST_CARRIER_HZ,
            "disabled: receding range rate ignored (%ld vs %ld)",
            f_los, (long) TEST_CARRIER_HZ);

    long f_aos = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ,
                                          TEST_RANGE_RATE_APPROACHING_KMS, 0);
    tap_okf(f_aos == (long) TEST_CARRIER_HZ,
            "disabled: approaching range rate ignored (%ld vs %ld)",
            f_aos, (long) TEST_CARRIER_HZ);
}

static void test_doppler_enabled_sign_convention(void)
{
    // enable=1 with rr=0 still gives the nominal — at TCA range rate
    // crosses through zero so the helper must not perturb the carrier.
    long f0 = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ, 0.0, 1);
    tap_okf(f0 == (long) TEST_CARRIER_HZ,
            "default-on: rr=0 -> carrier (%ld)", f0);

    // Receding (LOS end): satellite sees redshift, so the ground must
    // transmit HIGHER to compensate. Expect f_tx > nominal.
    long f_los = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ,
                                          TEST_RANGE_RATE_RECEDING_KMS, 1);
    tap_okf(f_los > (long) TEST_CARRIER_HZ,
            "default-on: receding (LOS) bumps TX HIGHER (%ld > %ld)",
            f_los, (long) TEST_CARRIER_HZ);

    // Approaching (AOS start): satellite sees blueshift, so the ground
    // must transmit LOWER. Expect f_tx < nominal.
    long f_aos = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ,
                                          TEST_RANGE_RATE_APPROACHING_KMS, 1);
    tap_okf(f_aos < (long) TEST_CARRIER_HZ,
            "default-on: approaching (AOS) bumps TX LOWER (%ld < %ld)",
            f_aos, (long) TEST_CARRIER_HZ);

    // Symmetry: equal-magnitude rr should produce equal-magnitude
    // offsets to within 1 Hz of rounding error (and a few Hz of the
    // small second-order term in 1/(1-x) vs 1+x).
    long delta_los = f_los - (long) TEST_CARRIER_HZ;
    long delta_aos = (long) TEST_CARRIER_HZ - f_aos;
    long diff = delta_los - delta_aos;
    if (diff < 0) diff = -diff;
    tap_okf(diff < 1000,
            "default-on: ±rr produces near-symmetric offsets (LOS+%ld vs AOS-%ld, diff %ld Hz)",
            delta_los, delta_aos, diff);
}

static void test_doppler_magnitude_matches_formula(void)
{
    // f_tx = carrier / (1 - rr/c)
    const double c = 299792.458;
    double rr = TEST_RANGE_RATE_RECEDING_KMS;
    long expected = (long)(TEST_CARRIER_HZ / (1.0 - rr / c) + 0.5);
    long got = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ, rr, 1);
    long err = got - expected;
    if (err < 0) err = -err;
    tap_okf(err == 0,
            "magnitude: 6 km/s -> %ld Hz (expected %ld)",
            got, expected);

    // At ±7 km/s the offset must be in the canonical "about 10 kHz"
    // range cited in the project memory (carrier 436.15 MHz).
    long f_at_7 = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ, 7.0, 1);
    long offset = f_at_7 - (long) TEST_CARRIER_HZ;
    tap_okf(offset > 9000 && offset < 11000,
            "magnitude: 7 km/s @ 436 MHz -> +%ld Hz (want ~10 kHz)",
            offset);
}

static void test_doppler_underflow_guard(void)
{
    // |rr| > c is unphysical but the helper must not divide by ~0.
    // For rr > c the factor (1 - rr/c) goes <=0; helper should fall
    // back to the nominal rather than emit a huge negative frequency.
    long f_super = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ,
                                            299793.0 /* rr > c */, 1);
    tap_okf(f_super == (long) TEST_CARRIER_HZ,
            "guard: rr just past c -> nominal (%ld)", f_super);
}

// ----------------------------------------------------------------
//  c) End-to-end: tx_burst_run with a recording b210 stub
// ----------------------------------------------------------------
//
// These tests drive the exact code path simple_sat_ops uses when the
// operator commits a burst from the compose modal. The capturing
// stub above records what would have hit the B210 — frequency,
// rate, gain, and the IQ buffer itself — so we can assert that:
//
//   * the staged tx_freq_hz reaches the burst call verbatim (proves
//     g_tx_freq_hz_doppler → tx_request.tx_freq_hz → burst_params is
//     intact);
//   * passing a non-NULL hmac_key produces materially different IQ
//     from passing NULL with the same payload (proves the HMAC
//     trailer is reaching the FM modulator's input bytes — same path
//     build_iq takes in production after the recent refactor).

static void make_request(tx_request_slot_t *req, const char *ascii)
{
    memset(req, 0, sizeof *req);
    size_t n = strlen(ascii);
    if (n > sizeof req->payload) n = sizeof req->payload;
    memcpy(req->payload, ascii, n);
    req->payload_len = n;
    req->is_hex = 0;
    req->csp_hdr = (csp_v1_header_t){
        .prio = 2, .src = 10, .dst = 1, .dport = 7, .sport = 16, .flags = 0,
    };
    req->repeat = 1;
    req->gap_ms = 200;
    req->tx_gain_db = 30.0;
}

static void test_run_doppler_freq_reaches_b210(void)
{
    // Stage a request at a Doppler-corrected frequency (what
    // simple_sat_ops would compute mid-pass). The b210 call must see
    // the same value — if main.c's wiring breaks, this test catches
    // it because the staged frequency would no longer survive the
    // request → run → burst hop.
    capture_reset();
    tx_request_slot_t req;
    make_request(&req, "CTS1+ping");
    long f_los = tx_burst_doppler_freq_hz(TEST_CARRIER_HZ,
                                          TEST_RANGE_RATE_RECEDING_KMS, 1);
    req.tx_freq_hz = f_los;

    const uint8_t key[] = "frontiersat-operator-key";
    char summary[200];
    tx_burst_result_t rc = tx_burst_run(/*core=*/(b210_rx_tx_core_t*)0x1,
                                         &req,
                                         /*rx_resume=*/TEST_CARRIER_HZ,
                                         key, sizeof key - 1,
                                         summary, sizeof summary);
    tap_okf(rc == TX_BURST_OK,
            "run: tx_burst_run returns OK with stubbed b210 (rc=%d)", (int) rc);
    tap_okf(g_capture.called == 1,
            "run: b210 burst called exactly once (got %d)",
            g_capture.called);
    tap_okf((long) g_capture.tx_freq_hz == f_los,
            "run: Doppler-corrected freq reaches b210 (%ld == %ld)",
            (long) g_capture.tx_freq_hz, f_los);
    tap_okf(g_capture.n_samps > 0,
            "run: IQ buffer non-empty (%zu samples)", g_capture.n_samps);
    capture_reset();
}

static void test_run_hmac_key_changes_iq(void)
{
    // Same payload, same Doppler frequency. Only the HMAC key
    // changes. The IQ bytes must differ — that's the only way HMAC
    // can actually be reaching the wire (build_iq → ax100_frame
    // appends a 4-byte trailer that changes the FM modulator's input
    // bits). If a future refactor silently drops the key, the two
    // captures will be byte-identical and this catches it.
    tx_request_slot_t req;
    make_request(&req, "AB");   // short payload so any byte diff stands out
    req.tx_freq_hz = (long) TEST_CARRIER_HZ;

    // Capture #1: WITH the operator's HMAC key.
    capture_reset();
    const uint8_t key[] = "frontiersat-operator-key";
    char summary[200];
    tx_burst_result_t rc1 = tx_burst_run((b210_rx_tx_core_t*)0x1, &req,
                                          TEST_CARRIER_HZ,
                                          key, sizeof key - 1,
                                          summary, sizeof summary);
    tap_okf(rc1 == TX_BURST_OK, "run: with-key burst OK");
    size_t n_with = g_capture.n_samps;
    int16_t *iq_with = g_capture.iq;
    g_capture.iq = NULL;        // hand ownership to the test scope
    capture_reset();

    // Capture #2: with NO HMAC key (key=NULL).
    tx_burst_result_t rc2 = tx_burst_run((b210_rx_tx_core_t*)0x1, &req,
                                          TEST_CARRIER_HZ,
                                          NULL, 0,
                                          summary, sizeof summary);
    tap_okf(rc2 == TX_BURST_OK, "run: no-key burst OK");
    size_t n_without = g_capture.n_samps;
    int16_t *iq_without = g_capture.iq;
    g_capture.iq = NULL;
    capture_reset();

    // RS strips the inner zero-padding (CCSDS encode-with-pad), so
    // adding a 4-byte HMAC trailer makes the on-wire frame 4 bytes
    // longer — which at 9600 baud, 480 kHz, sps=50 is exactly
    // 4 * 8 * 50 = 1600 more IQ samples. Anything else means HMAC
    // didn't reach the modulator (or the framing parameters drifted).
    long long delta = (long long) n_with - (long long) n_without;
    tap_okf(delta == 1600,
            "run: with-HMAC IQ is exactly 1600 samples (= 4 bytes @ 9600/480k) "
            "longer than without (delta=%lld)", delta);
    int differ = (n_with > 0 && n_without > 0)
              && (n_with != n_without
                  || memcmp(iq_with, iq_without,
                            n_with * 2 * sizeof(int16_t)) != 0);
    tap_okf(differ,
            "run: with-key vs without-key IQ samples DIFFER "
            "(proves HMAC trailer reaches the modulator)");

    free(iq_with);
    free(iq_without);
}

static void test_run_deterministic_with_same_key(void)
{
    // Two back-to-back runs with the same payload + same key must
    // produce byte-identical IQ. If they don't, something stateful
    // is creeping into the burst build path (uninitialised memory,
    // NCO carry-over, etc) — none of which the production path
    // should have.
    tx_request_slot_t req;
    make_request(&req, "deterministic");
    req.tx_freq_hz = (long) TEST_CARRIER_HZ;
    const uint8_t key[] = "k";
    char summary[200];

    capture_reset();
    tx_burst_run((b210_rx_tx_core_t*)0x1, &req, TEST_CARRIER_HZ,
                 key, sizeof key - 1, summary, sizeof summary);
    size_t n_a = g_capture.n_samps;
    int16_t *iq_a = g_capture.iq;
    g_capture.iq = NULL;
    capture_reset();

    tx_burst_run((b210_rx_tx_core_t*)0x1, &req, TEST_CARRIER_HZ,
                 key, sizeof key - 1, summary, sizeof summary);
    size_t n_b = g_capture.n_samps;
    int16_t *iq_b = g_capture.iq;
    g_capture.iq = NULL;
    capture_reset();

    int same = (n_a == n_b) && n_a > 0
            && memcmp(iq_a, iq_b, n_a * 2 * sizeof(int16_t)) == 0;
    tap_okf(same,
            "run: same payload + same key -> byte-identical IQ (%zu vs %zu samps)",
            n_a, n_b);

    free(iq_a);
    free(iq_b);
}

int main(void)
{
    // HMAC --- enabled by default in the simple_sat_ops TX path
    test_hmac_default_on();
    test_hmac_explicitly_disabled();
    test_hmac_frame_includes_trailer();

    // TX Doppler --- enabled by default in the simple_sat_ops TX path
    test_doppler_disabled_returns_nominal();
    test_doppler_enabled_sign_convention();
    test_doppler_magnitude_matches_formula();
    test_doppler_underflow_guard();

    // End-to-end: drive tx_burst_run (the same function main.c's
    // tick loop invokes) through a capturing b210 stub.
    test_run_doppler_freq_reaches_b210();
    test_run_hmac_key_changes_iq();
    test_run_deterministic_with_same_key();

    capture_reset();
    return tap_done();
}
