// tx_burst.c — see header. Body lifted from utils/b210_rx_tx.c
// (tx_build_iq + tx_fm_modulate + tx_apply_envelope_ramp + the daemon's
// daemon_service_tx_request wrapper) when the daemon was folded in.

#include "tx_burst.h"
#include "ax100.h"
#include "b210_rx_tx_core.h"
#include "csp.h"
#include "fm_mod.h"
#include "modem.h"

#ifdef SSO_WITH_SDR
// Only the in-loop request servicer below needs the operator session,
// the RX/TX session, and the TX-event emitter; gate them so the standalone
// tx_burst selftest (built without SSO_WITH_SDR) still links.
#include "state.h"
#include "rx_session.h"
#include "tx_compose.h"     // emit_tx_event_local
#include "sso_audit.h"
#include "sso_ipc.h"        // SSO_TX_TEXT_MAX, SSO_EVT_TX_*
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

ssize_t tx_burst_parse_hex(const char *hex, uint8_t *out, size_t cap)
{
    if (hex == NULL || out == NULL) return -1;
    size_t n = 0;
    int hi = -1;
    for (const char *p = hex; *p != '\0'; p++) {
        unsigned char c = (unsigned char) *p;
        if (c == ' ' || c == '\t' || c == ':') continue;
        int d = hex_digit((char) c);
        if (d < 0) return -1;
        if (hi < 0) hi = d;
        else {
            if (n >= cap) return -1;
            out[n++] = (uint8_t)((hi << 4) | d);
            hi = -1;
        }
    }
    if (hi >= 0) return -1;
    return (ssize_t) n;
}

long tx_burst_doppler_freq_hz(double nominal_carrier_hz,
                              double range_rate_km_s,
                              int enable)
{
    if (!enable) return (long) nominal_carrier_hz;
    const double c = 299792.458;
    double factor = 1.0 - range_rate_km_s / c;
    // factor can only be <=0 for unphysical range rates (|rr|>c). Treat
    // that and anything tiny-positive as "don't divide" — fall back to
    // the bare nominal rather than emit a wild frequency.
    if (factor < 1e-9) return (long) nominal_carrier_hz;
    return (long)(nominal_carrier_hz / factor + 0.5);
}

ssize_t tx_burst_build_frame(const uint8_t *payload, size_t payload_len,
                              const csp_v1_header_t *csp_hdr,
                              const uint8_t *hmac_key, size_t hmac_key_len,
                              uint8_t *out_frame, size_t out_cap)
{
    if (csp_hdr == NULL || out_frame == NULL) return -1;
    uint8_t csp_packet[4096];
    ssize_t csp_len = csp_v1_encode(csp_hdr, payload, payload_len,
                                    csp_packet, sizeof csp_packet);
    if (csp_len < 0) return -1;

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (hmac_key && hmac_key_len > 0) {
        opts.hmac_key = hmac_key; opts.hmac_key_len = hmac_key_len;
    }
    opts.reed_solomon = 1;
    return ax100_frame(csp_packet, (size_t) csp_len, &opts,
                       out_frame, out_cap);
}

static int build_iq(const uint8_t *payload, size_t payload_len,
                    const csp_v1_header_t *csp_hdr,
                    const uint8_t *hmac_key, size_t hmac_key_len,
                    int use_rs,
                    int bit_rate, int tx_rate_hz, double deviation_hz,
                    int repeat, int gap_ms,
                    int preroll_ms, int postroll_ms, double ramp_ms,
                    int16_t **out_iq, size_t *out_n)
{
    if (out_iq == NULL || out_n == NULL) return -1;
    *out_iq = NULL; *out_n = 0;

    // use_rs is pinned to 1 by every caller (tx_burst_run); the
    // build_frame helper bakes that in. If a future caller needs
    // RS off this is the place to fork.
    (void) use_rs;
    uint8_t frame[4200];
    ssize_t frame_len = tx_burst_build_frame(payload, payload_len, csp_hdr,
                                              hmac_key, hmac_key_len,
                                              frame, sizeof frame);
    if (frame_len < 0) return -1;

    modem_params_t mp;
    modem_params_defaults(&mp);
    mp.bit_rate  = bit_rate;
    mp.samp_rate = tx_rate_hz;
    if (mp.samp_rate <= 0 || mp.bit_rate <= 0
        || mp.samp_rate % mp.bit_rate != 0) return -1;
    int sps = mp.samp_rate / mp.bit_rate;
    size_t n_pcm = (size_t) frame_len * 8u * (size_t) sps;
    int16_t *pcm = malloc(n_pcm * sizeof(int16_t));
    if (pcm == NULL) return -1;
    if (modem_bytes_to_pcm16(frame, (size_t) frame_len, &mp, pcm, n_pcm) < 0) {
        free(pcm); return -1;
    }

    size_t preroll_bytes  = ((size_t) preroll_ms * (size_t) mp.samp_rate / 1000)
                            / (8 * (size_t) sps);
    size_t preroll_samps  = preroll_bytes * 8 * (size_t) sps;
    size_t postroll_samps = (size_t)((double) mp.samp_rate
                                      * (double) postroll_ms / 1000.0);
    size_t ramp_samps     = (size_t)((double) mp.samp_rate * ramp_ms / 1000.0);

    int16_t *preroll_pcm = NULL;
    if (preroll_samps > 0) {
        uint8_t *pre_bytes = malloc(preroll_bytes);
        if (pre_bytes == NULL) { free(pcm); return -1; }
        memset(pre_bytes, 0xAA, preroll_bytes);
        preroll_pcm = malloc(preroll_samps * sizeof(int16_t));
        if (preroll_pcm == NULL) { free(pre_bytes); free(pcm); return -1; }
        if (modem_bytes_to_pcm16(pre_bytes, preroll_bytes, &mp,
                                  preroll_pcm, preroll_samps) < 0) {
            free(pre_bytes); free(preroll_pcm); free(pcm);
            return -1;
        }
        free(pre_bytes);
    }

    if (repeat < 1) repeat = 1;
    if (gap_ms < 0) gap_ms = 0;
    size_t gap_samps  = (size_t)((double) mp.samp_rate
                                   * (double) gap_ms / 1000.0);
    size_t per_rep    = n_pcm + gap_samps;
    size_t n_iq_total = preroll_samps + per_rep * (size_t) repeat + postroll_samps;
    int16_t *iq = calloc(n_iq_total * 2, sizeof(int16_t));
    if (iq == NULL) { free(preroll_pcm); free(pcm); return -1; }

    // One phase accumulator for the whole concatenated waveform so the
    // preroll and each repeat stay phase-continuous (the gaps between
    // repeats are zeros, but the carrier phase still threads through).
    fm_mod_t fm;
    fm_mod_init(&fm);
    if (preroll_pcm) {
        fm_mod_block(&fm, preroll_pcm, preroll_samps, deviation_hz,
                     (double) mp.samp_rate, iq);
    }
    for (int r = 0; r < repeat; r++) {
        size_t off = preroll_samps + (size_t) r * per_rep;
        fm_mod_block(&fm, pcm, n_pcm, deviation_hz, (double) mp.samp_rate,
                     iq + off * 2);
        size_t burst_start = (r == 0) ? 0 : off;
        size_t burst_n     = (r == 0) ? (preroll_samps + n_pcm) : n_pcm;
        fm_apply_ramp(iq + burst_start * 2, burst_n, ramp_samps);
    }

    free(preroll_pcm); free(pcm);
    *out_iq = iq; *out_n = n_iq_total;
    return 0;
}

// Format the one-line "ascii:<text>" / "hex:<bytes>" description of a
// payload for the UI / tx.log / viewer fan-out. Exposed (not static) so
// the selftest can pin the formatting directly -- it has had two display
// bugs (a mid-string truncation and a one-past-the-end stale byte).
void tx_burst_summarize(const uint8_t *payload, size_t n, int is_hex,
                        char *out, size_t out_size)
{
    if (out_size == 0) return;
    if (!is_hex) {
        // cap is the visible length: "ascii:" (6) + the n payload bytes.
        // The precision below is cap - 6, so cap must be n + 6, not n + 7
        // -- payload is not NUL-terminated (a bare memcpy of payload_len
        // bytes), so an n+1 precision printed one stale byte past the
        // command. The clamp keeps the whole string inside out_size.
        size_t cap = n + 6;
        if (cap >= out_size) cap = out_size - 1;
        snprintf(out, out_size, "ascii:%.*s",
                 (int)(cap - 6), (const char *) payload);
        return;
    }
    char hexbuf[64];
    size_t cap_bytes = n > 16 ? 16 : n;
    size_t k = 0;
    for (size_t i = 0; i < cap_bytes && k + 3 < sizeof hexbuf; ++i) {
        k += (size_t) snprintf(hexbuf + k, sizeof hexbuf - k, "%02x",
                                (unsigned) payload[i]);
    }
    if (n > cap_bytes) snprintf(hexbuf + k, sizeof hexbuf - k, "...");
    snprintf(out, out_size, "hex:%s", hexbuf);
}

tx_burst_result_t tx_burst_run(b210_rx_tx_core_t *core,
                                const tx_request_slot_t *req,
                                double rx_resume_freq_hz,
                                const uint8_t *hmac_key, size_t hmac_key_len,
                                char *out_summary, size_t summary_n)
{
    if (out_summary && summary_n) out_summary[0] = '\0';
    if (req == NULL) return TX_BURST_FRAME_BUILD_FAILED;
    tx_burst_summarize(req->payload, req->payload_len, req->is_hex,
                       out_summary, summary_n);
    // Note the heritage of an expanded "SSO+..." pseudo-command on the same
    // summary that lands in tx.log / the viewer fan-out.
    if (req->sso_origin[0] && out_summary && summary_n) {
        size_t l = strlen(out_summary);
        if (l < summary_n)
            snprintf(out_summary + l, summary_n - l,
                     " (replaced '%s')", req->sso_origin);
    }
    if (core == NULL) return TX_BURST_NO_CORE;

    int bit_rate     = 9600;
    int tx_rate_hz   = 480000;
    double deviation = (double) bit_rate / 4.0;
    int repeat       = req->repeat > 0 ? req->repeat : 1;
    int gap_ms       = req->gap_ms > 0 ? req->gap_ms : 200;
    int preroll_ms   = req->preroll_ms > 0 ? req->preroll_ms : 200;
    int postroll_ms  = 50;
    double ramp_ms   = 1.0;
    double start_delay_s = 0.5;
    double tx_gain_db    = req->tx_gain_db > 0 ? req->tx_gain_db : 70.0;
    long   tx_freq_hz    = req->tx_freq_hz > 0 ? req->tx_freq_hz : 436150000L;

    int16_t *iq = NULL;
    size_t n_samps = 0;
    if (build_iq(req->payload, req->payload_len, &req->csp_hdr,
                 hmac_key, hmac_key_len, /*use_rs=*/1,
                 bit_rate, tx_rate_hz, deviation,
                 repeat, gap_ms, preroll_ms, postroll_ms, ramp_ms,
                 &iq, &n_samps) != 0) {
        return TX_BURST_FRAME_BUILD_FAILED;
    }

    b210_rx_tx_core_burst_params_t bp = {
        .iq                = iq,
        .n_samps           = n_samps,
        .tx_rate_hz        = (double) tx_rate_hz,
        .tx_freq_hz        = (double) tx_freq_hz,
        .tx_gain_db        = tx_gain_db,
        .start_delay_s     = start_delay_s,
        .rx_resume_freq_hz = rx_resume_freq_hz,
    };
    int rc = b210_rx_tx_core_burst(core, &bp);
    free(iq);
    return (rc == 0) ? TX_BURST_OK : TX_BURST_UHD_ERROR;
}

#ifdef SSO_WITH_SDR
// Service a pending TX request from the main loop. Three paths:
//
//   1. --tx-dry-run:    synthesize "ok" without touching the SDR, so
//                       auto-tcmd + compose still exercise all their UI
//                       state on a dev host.
//   2. rx_session up:   real burst -- submitted async to the worker, which
//                       pauses RX, transmits and resumes RX (~1 s). The main
//                       loop keeps running between submit and poll so the
//                       rotator, redraw, IPC and the next auto-tcmd tick
//                       aren't frozen by the burst. tx_request.pending stays
//                       set across the in-flight window so auto-tcmd will not
//                       queue a second burst on top.
//   3. neither:         reject so auto-tcmd can move on (started
//                       --without-b210 and without --tx-dry-run): just clear
//                       the pending slot rather than deadlocking.
//
// Emits the SENT / NOT_SENT event + the tx-result audit line and clears the
// pending slot once a path resolves; a still-in-flight burst returns with
// the slot still pending so the next tick polls again.
void tx_burst_service_request(state_t *state)
{
    if (state->tx.tx_request.pending) {
        char summary[SSO_TX_TEXT_MAX];
        const char *outcome = NULL;
        int  on_air = 0;
        int  finished = 0;        // emit the result + clear pending this tick
        if (state->tx.tx_dry_run) {
            snprintf(summary, sizeof summary, "%s",
                     state->tx.tx_request.summary);
            outcome = "dry-run";   // composed but deliberately not keyed
            finished = 1;
        } else if (state->tx.hmac_key_len == 0) {
            // CTS1 expects HMAC on every uplink. Without a valid
            // key the burst would go out unsigned and the satellite
            // would silently drop it. Refuse here so the operator
            // sees a clear error instead of letting it go out unsigned.
            snprintf(summary, sizeof summary, "%s",
                     state->tx.tx_request.summary);
            outcome = "rejected: no HMAC key (see banner)";
            finished = 1;
        } else if (state->sdr.rx_session != NULL && !rx_session_can_tx(state->sdr.rx_session)) {
            // RX-only backend (e.g. RTL-SDR): never reaches the air.
            // Backstop for a stale queued burst that slipped past the
            // compose / auto-tcmd gates.
            snprintf(summary, sizeof summary, "%s", state->tx.tx_request.summary);
            outcome = "rejected: RX-only SDR";
            finished = 1;
        } else if (state->sdr.rx_session != NULL) {
            if (!state->tx.tx_inflight) {
                if (rx_session_submit_burst(state->sdr.rx_session, &state->tx.tx_request,
                                             state->tx.hmac_key, state->tx.hmac_key_len) == 0) {
                    state->tx.tx_inflight = 1;
                    // Stay pending; we'll poll on subsequent ticks.
                } else {
                    // Worker refused (slot already busy or rxs error).
                    snprintf(summary, sizeof summary, "%s",
                             state->tx.tx_request.summary);
                    outcome = "rejected: rx_session busy";
                    finished = 1;
                }
            } else {
                rx_burst_result_t br;
                int done = rx_session_poll_burst(state->sdr.rx_session, &br,
                                                  summary, sizeof summary);
                if (done == 1) {
                    switch (br) {
                        case RX_BURST_OK:                 outcome = "ok"; on_air = 1; break;
                        case RX_BURST_NO_CORE:            outcome = "rejected: no B210"; break;
                        case RX_BURST_FRAME_BUILD_FAILED: outcome = "rejected: frame build"; break;
                        case RX_BURST_UHD_ERROR:          outcome = "uhd-err"; break;
                        // Only the sync submit path produces this; the async
                        // poll here never does, but handle it for exhaustiveness.
                        case RX_BURST_ABORTED:            outcome = "rejected: aborted"; break;
                    }
                    state->tx.tx_inflight = 0;
                    finished = 1;
                }
                // else: still in flight; fall through and let the
                // rest of the main loop run.
            }
        } else {
            snprintf(summary, sizeof summary, "%s",
                     state->tx.tx_request.summary);
            outcome = "rejected: no B210";
            finished = 1;
        }
        if (finished) {
            // A command that made it on the air gets a plain TX
            // record, nothing more: the ground station can confirm
            // it transmitted, but only the satellite can acknowledge,
            // and that arrives on the downlink, not here. Anything
            // that did NOT reach the air (rejected, dry-run, uhd-err)
            // gets a not-sent note carrying the reason.
            if (on_air) {
                emit_tx_event_local(state, SSO_EVT_TX_COMMAND_SENT, summary, NULL);
            } else {
                emit_tx_event_local(state, SSO_EVT_TX_NOT_SENT, summary, outcome);
            }
            // Audit: the result of every queued TX burst, so post-
            // incident review can see each tx-commit and whether it
            // reached the air (on_air=1 means the burst left the radio).
            {
                char det[512];
                snprintf(det, sizeof det,
                         "outcome=\"%.80s\" on_air=%d summary=\"%.300s\"",
                         outcome ? outcome : "?", on_air, summary);
                sso_audit_event("tx-result", det);
            }
            state->tx.tx_request.pending = 0;
        }
    }
}
#endif
