// tx_burst.c — see header. Body lifted from utils/b210_rx_tx.c
// (tx_build_iq + tx_fm_modulate + tx_apply_envelope_ramp + the daemon's
// daemon_service_tx_request wrapper) when the daemon was folded in.

#include "tx_burst.h"
#include "ax100.h"
#include "b210_rx_tx_core.h"
#include "csp.h"
#include "modem.h"

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

static void fm_modulate(const int16_t *pcm, size_t n_pcm,
                         double dev_hz, double fs, int16_t *iq_out)
{
    static double phi = 0.0;
    const double k = 2.0 * M_PI * dev_hz / fs;
    const double inv = 1.0 / 32767.0;
    for (size_t i = 0; i < n_pcm; i++) {
        double x = (double) pcm[i] * inv;
        phi += k * x;
        if (phi >  M_PI) phi -= 2.0 * M_PI;
        if (phi < -M_PI) phi += 2.0 * M_PI;
        iq_out[2 * i + 0] = (int16_t) lround(cos(phi) * 22937.0);
        iq_out[2 * i + 1] = (int16_t) lround(sin(phi) * 22937.0);
    }
}

static void apply_ramp(int16_t *iq, size_t n_samps, size_t ramp_n)
{
    if (ramp_n == 0) return;
    if (ramp_n > n_samps / 2) ramp_n = n_samps / 2;
    for (size_t i = 0; i < ramp_n; i++) {
        double env_in  = 0.5 * (1.0 - cos(M_PI * (double) i / (double) ramp_n));
        double env_out = 0.5 * (1.0 + cos(M_PI * (double) i / (double) ramp_n));
        iq[2 * i + 0] = (int16_t) lround((double) iq[2 * i + 0] * env_in);
        iq[2 * i + 1] = (int16_t) lround((double) iq[2 * i + 1] * env_in);
        size_t k = n_samps - ramp_n + i;
        iq[2 * k + 0] = (int16_t) lround((double) iq[2 * k + 0] * env_out);
        iq[2 * k + 1] = (int16_t) lround((double) iq[2 * k + 1] * env_out);
    }
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

    if (preroll_pcm) {
        fm_modulate(preroll_pcm, preroll_samps, deviation_hz,
                    (double) mp.samp_rate, iq);
    }
    for (int r = 0; r < repeat; r++) {
        size_t off = preroll_samps + (size_t) r * per_rep;
        fm_modulate(pcm, n_pcm, deviation_hz, (double) mp.samp_rate,
                    iq + off * 2);
        size_t burst_start = (r == 0) ? 0 : off;
        size_t burst_n     = (r == 0) ? (preroll_samps + n_pcm) : n_pcm;
        apply_ramp(iq + burst_start * 2, burst_n, ramp_samps);
    }

    free(preroll_pcm); free(pcm);
    *out_iq = iq; *out_n = n_iq_total;
    return 0;
}

static void summarize(const uint8_t *payload, size_t n, int is_hex,
                      char *out, size_t out_size)
{
    if (out_size == 0) return;
    if (!is_hex) {
        size_t cap = n + 7;
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
    summarize(req->payload, req->payload_len, req->is_hex,
               out_summary, summary_n);
    if (core == NULL) return TX_BURST_NO_CORE;

    int bit_rate     = 9600;
    int tx_rate_hz   = 480000;
    double deviation = (double) bit_rate / 4.0;
    int repeat       = req->repeat > 0 ? req->repeat : 1;
    int gap_ms       = req->gap_ms > 0 ? req->gap_ms : 200;
    int preroll_ms   = 100;
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
