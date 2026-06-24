/*

   Simple Satellite Operations  sdr_uhd.c

   UHD (Ettus B2xx and clones) backend for the SDR abstraction. Owns the
   USRP device, the RX streamer, and the half-duplex TX burst path
   (including the LO-leak mitigation that unmaps the TX subdev between
   bursts). Yields raw sc16 IQ at the coerced native rate via read_iq;
   the device-agnostic DSP chain in b210_rx_tx_core.c does the rest.

   This code was extracted verbatim from b210_rx_tx_core.c; the behavior
   is unchanged.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "sdr_backend.h"
#include "sdr_usb_detect.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include <uhd/usrp/subdev_spec.h>
#include <uhd/types/tune_request.h>
#include <uhd/types/metadata.h>
#include <uhd/error.h>

struct sdr_uhd {
    uhd_usrp_handle         dev;
    uhd_rx_streamer_handle  stream;
    uhd_rx_metadata_handle  md;

    // Lazy-built TX streamer. Cached for the daemon's lifetime so
    // repeated bursts during one pass don't pay the rebuild tax.
    uhd_tx_streamer_handle  tx_stream;
    size_t                  tx_max_per_buff;
    double                  tx_rate_cached;
    double                  tx_gain_cached;
    int                     tx_powered;       // TX subdev mapped ("A:A") vs unmapped ("")

    size_t                  max_iq_in;        // UHD's max_num_samps
    int16_t                *drain_buf;        // scratch for the pre-TX RX drain
    double                  actual_freq;      // last read-back RX LO
    int                     stream_running;

    // Consecutive recv / metadata error counters for the log-throttling in
    // read_iq. Per-handle (not function-local static) so two open devices
    // can't cross-contaminate each other's dead-link tally.
    unsigned long           recv_err_run;
    unsigned long           md_err_run;

    // Serializes access to the device handle (the UHD property tree) across
    // the RX worker and the TX-burst thread. The two streamers run
    // full-duplex -- RX recv and TX send touch their own streamer objects
    // and may overlap freely -- but the configuration calls (set_tx_freq /
    // _gain / _rate / _subdev, set_rx_freq / _gain, get_time_now) all reach
    // into the shared device handle, which UHD does not make thread-safe.
    // Held briefly around recv and around each config call, but deliberately
    // NOT across the TX send loop, so a burst never starves the RX transport.
    pthread_mutex_t         dev_mu;
};

static int log_uhd(uhd_error e, const char *what)
{
    if (e == UHD_ERROR_NONE) return 0;
    char errbuf[256] = {0};
    (void)uhd_get_last_error(errbuf, sizeof errbuf);
    fprintf(stderr, "sdr_uhd: %s: UHD error %d: %s\n",
            what, (int)e, errbuf[0] ? errbuf : "(no detail)");
    return 1;
}

static void uhd_close(sdr_backend_t *be);

// Resolve the UHD device-args string to pass to uhd_usrp_make. Precedence:
//   1. --uhd-args   verbatim (escape hatch).
//   2. --sdr-fpga   forces fpga=<path>.
//   3. serial->image map: read the USB serial via libusb (NOT
//      uhd_usrp_find, which segfaults on macOS) and, if it matches an
//      entry in the shared map /usr/local/share/sso/sdr_fpga_map, load
//      that bitstream. This is how a B210 clone (identical to a genuine
//      board except its serial) gets its non-stock FPGA automatically.
//   4. otherwise the base args, letting UHD pick the stock image.
// uhd_usrp_make fails gracefully when no device is present, so AUTO can
// still fall through to the RTL-SDR backend.
static void uhd_resolve_device_args(const sdr_open_params_t *p,
                                    char *out, size_t cap)
{
    if (p->uhd_args_override != NULL && p->uhd_args_override[0]) {
        snprintf(out, cap, "%.500s", p->uhd_args_override);
        fprintf(stderr, "sdr_uhd: using --uhd-args verbatim: %s\n", out);
        return;
    }

    const char *base = (p->device_args != NULL && p->device_args[0])
                       ? p->device_args : "type=b200";

    const char *fpga = NULL;
    char        mapimg[768] = {0};
    char        serial[128] = {0};
    int         have_serial = 0;

    // Note: this serial scan reads the FIRST B2xx on the bus and ignores
    // p->device_index — multi-B2xx selection is expected to go through
    // --uhd-args (serial=...), which is handled verbatim above and never
    // reaches here. The FPGA-map lookup is likewise skipped when --uhd-args
    // is set (early return above), so a pinned serial isn't second-guessed.
    if (p->fpga_image_path != NULL && p->fpga_image_path[0]) {
        fpga = p->fpga_image_path;   // explicit override wins
    } else if (sdr_usb_b2xx_serial(serial, sizeof serial) == 0) {
        have_serial = 1;
        fprintf(stderr, "sdr_uhd: USB serial %s\n", serial);
        if (sdr_fpga_for_serial(serial, mapimg, sizeof mapimg)) {
            fpga = mapimg;
            fprintf(stderr, "sdr_uhd: serial %s -> FPGA %s (sdr_fpga_map)\n",
                    serial, fpga);
        } else {
            char mp[512] = {0};
            (void)sdr_fpga_map_path(mp, sizeof mp);
            fprintf(stderr, "sdr_uhd: serial %s not in %s — using the stock "
                    "FPGA. Add a line there to auto-load a clone image.\n",
                    serial, mp);
        }
    }

    int want_serial = (have_serial && serial[0]
                       && strstr(base, "serial=") == NULL);
    int want_fpga   = (fpga != NULL && fpga[0]);

    // One bounded snprintf: precision-capped (200 + 8 + 127 + 6 + 512 +
    // the trailing literal) fits a 1024-byte out. num_recv_frames=512
    // enlarges the host-side RX transport ring: the B2xx buffers RX only
    // in USB frames (no big socket buffer like the networked USRPs), so a
    // bigger ring is the documented cure for transient overflows — it
    // gives the continuous RX stream slack across a concurrent TX burst.
    snprintf(out, cap, "%.200s%s%.127s%s%.512s,num_recv_frames=512",
             base,
             want_serial ? ",serial=" : "", want_serial ? serial : "",
             want_fpga   ? ",fpga="   : "", want_fpga   ? fpga   : "");
    if (want_fpga) {
        fprintf(stderr, "sdr_uhd: loading FPGA image %s\n", fpga);
    }
}

static int uhd_open(sdr_backend_t *be, const sdr_open_params_t *p, sdr_caps_t *caps)
{
    if (be == NULL || p == NULL || caps == NULL) return -1;

    struct sdr_uhd *u = (struct sdr_uhd *)calloc(1, sizeof *u);
    if (u == NULL) {
        fprintf(stderr, "sdr_uhd: out of memory\n");
        return -1;
    }
    be->priv = u;
    u->tx_gain_cached = -1.0;
    pthread_mutex_init(&u->dev_mu, NULL);

    char device_args[1024];
    uhd_resolve_device_args(p, device_args, sizeof device_args);
    const char *rx_antenna  = p->rx_antenna  ? p->rx_antenna  : "RX2";
    double bw_hz = p->bw_hz > 0.0 ? p->bw_hz : p->rate_hz;

    if (log_uhd(uhd_usrp_make(&u->dev, device_args), "uhd_usrp_make")) goto fail;

    // The B200 maps TX channel 0 ("A:A") by default. Track that so the
    // burst path can unmap it after a burst to power the TX LO down.
    u->tx_powered = 1;

    if (log_uhd(uhd_usrp_set_rx_rate(u->dev, p->rate_hz, 0), "set_rx_rate")) goto fail;
    // The actual (coerced) rate feeds the decimation math downstream, so don't
    // ignore the query: on failure fall back to the requested rate and warn,
    // rather than letting a stale/garbage rate through.
    double rate = p->rate_hz;
    if (uhd_usrp_get_rx_rate(u->dev, 0, &rate) != UHD_ERROR_NONE) {
        fprintf(stderr, "sdr_uhd: get_rx_rate failed; assuming the requested "
                        "%.0f S/s for the decimation math\n", p->rate_hz);
        rate = p->rate_hz;
    }
    if (p->rate_hz > 0.0 && fabs(rate - p->rate_hz) / p->rate_hz > 0.01) {
        fprintf(stderr, "sdr_uhd: requested %.0f S/s, got %.0f S/s "
                        "(device coerced)\n", p->rate_hz, rate);
    }

    if (log_uhd(uhd_usrp_set_rx_gain(u->dev, p->gain_db, 0, ""), "set_rx_gain")) goto fail;
    // AD9361 background tracking loops. log_uhd reports but doesn't abort
    // because not every UHD build wires these all the way down on every
    // revision.
    log_uhd(uhd_usrp_set_rx_dc_offset_enabled(u->dev,
                                              p->rx_dc_offset_track ? true : false, 0),
            "set_rx_dc_offset_enabled");
    log_uhd(uhd_usrp_set_rx_iq_balance_enabled(u->dev,
                                               p->rx_iq_balance_track ? true : false, 0),
            "set_rx_iq_balance_enabled");
    fprintf(stderr, "sdr_uhd: AD9361 tracking: dc_offset=%s, iq_balance=%s\n",
            p->rx_dc_offset_track  ? "on" : "off",
            p->rx_iq_balance_track ? "on" : "off");
    if (log_uhd(uhd_usrp_set_rx_bandwidth(u->dev, bw_hz, 0), "set_rx_bandwidth")) goto fail;
    if (log_uhd(uhd_usrp_set_rx_antenna(u->dev, rx_antenna, 0), "set_rx_antenna")) goto fail;

    {
        // mode_n=fractional forces the AD9361's fractional-N PLL so the LO
        // resolution is ~2 Hz; the fine offset is taken up by the DSP NCO
        // in the chain.
        char tune_args[] = "mode_n=fractional";
        uhd_tune_request_t req = {
            .target_freq     = p->freq_hz,
            .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
            .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
            .args            = tune_args,
        };
        uhd_tune_result_t res = {0};
        if (log_uhd(uhd_usrp_set_rx_freq(u->dev, &req, 0, &res), "set_rx_freq")) goto fail;
    }
    u->actual_freq = p->freq_hz;
    {
        double f = u->actual_freq;
        if (uhd_usrp_get_rx_freq(u->dev, 0, &f) == UHD_ERROR_NONE) u->actual_freq = f;
    }

    if (log_uhd(uhd_rx_streamer_make(&u->stream), "rx_streamer_make")) goto fail;
    {
        size_t channels[1] = { 0 };
        uhd_stream_args_t args = {
            .cpu_format   = "sc16",
            .otw_format   = "sc16",
            .args         = "",
            .channel_list = channels,
            .n_channels   = 1,
        };
        if (log_uhd(uhd_usrp_get_rx_stream(u->dev, &args, u->stream),
                    "get_rx_stream")) goto fail;
    }

    if (log_uhd(uhd_rx_streamer_max_num_samps(u->stream, &u->max_iq_in),
                "rx_max_num_samps")) goto fail;
    if (u->max_iq_in == 0) u->max_iq_in = 2040;

    u->drain_buf = (int16_t *)malloc(u->max_iq_in * 2 * sizeof(int16_t));
    if (u->drain_buf == NULL) {
        fprintf(stderr, "sdr_uhd: out of memory for drain buf\n");
        goto fail;
    }

    if (log_uhd(uhd_rx_metadata_make(&u->md), "rx_metadata_make")) goto fail;

    {
        uhd_stream_cmd_t cmd = {
            .stream_mode          = UHD_STREAM_MODE_START_CONTINUOUS,
            .num_samps            = 0,
            .stream_now           = true,
            .time_spec_full_secs  = 0,
            .time_spec_frac_secs  = 0.0,
        };
        if (log_uhd(uhd_rx_streamer_issue_stream_cmd(u->stream, &cmd),
                    "issue_start_stream")) goto fail;
        u->stream_running = 1;
    }

    caps->can_tx            = 1;
    caps->native_rate_hz    = rate;
    caps->tune_resolution_hz = 2.0;
    caps->has_hw_lo_offset  = 1;
    caps->sc16_native       = 1;
    caps->max_rx_pairs      = u->max_iq_in;
    {
        char mb[32] = {0};
        if (uhd_usrp_get_mboard_name(u->dev, 0, mb, sizeof mb) == UHD_ERROR_NONE
            && mb[0]) {
            snprintf(caps->name, sizeof caps->name, "USRP %.20s", mb);
        } else {
            snprintf(caps->name, sizeof caps->name, "USRP (UHD)");
        }
    }
    return 0;

fail:
    uhd_close(be);   // frees u + handles and clears be->priv
    return -1;
}

static void uhd_close(sdr_backend_t *be)
{
    if (be == NULL) return;
    struct sdr_uhd *u = (struct sdr_uhd *)be->priv;
    if (u == NULL) return;
    if (u->stream_running && u->stream != NULL) {
        uhd_stream_cmd_t cmd = {
            .stream_mode = UHD_STREAM_MODE_STOP_CONTINUOUS,
            .num_samps   = 0,
            .stream_now  = true,
        };
        (void)uhd_rx_streamer_issue_stream_cmd(u->stream, &cmd);
    }
    if (u->md        != NULL) uhd_rx_metadata_free(&u->md);
    if (u->stream    != NULL) uhd_rx_streamer_free(&u->stream);
    if (u->tx_stream != NULL) uhd_tx_streamer_free(&u->tx_stream);
    if (u->dev       != NULL) uhd_usrp_free(&u->dev);
    pthread_mutex_destroy(&u->dev_mu);
    free(u->drain_buf);
    free(u);
    be->priv = NULL;
}

// Consecutive recv errors after which we declare the RX link dead (with
// the 1.0 s recv timeout below, ~this many seconds of no data).
#define UHD_RX_DEAD_LINK_ERRORS 8

static ssize_t uhd_read_iq(sdr_backend_t *be, int16_t *out, size_t cap_pairs)
{
    struct sdr_uhd *u = (struct sdr_uhd *)be->priv;
    if (u == NULL || u->stream == NULL || out == NULL) return -1;

    // A TX burst leaves the USB transport spewing LIBUSB_TRANSFER_OVERFLOW
    // on every recv (see the deferred B210 TX/RX overflow note). These
    // arrive thousands per second, so log only the first of a run plus a
    // periodic count — u->recv_err_run / u->md_err_run reset on the first
    // clean recv, so each new storm logs its onset. Keeps the stderr log
    // readable instead of megabytes of identical lines. Per-handle so two
    // open devices keep separate tallies.

    void  *bufs[1] = { out };
    size_t n_recv = 0;
    // 1.0 s timeout: while streaming, recv returns as soon as a chunk is
    // ready (every few ms), so the timeout only bites when the device
    // has stopped delivering — i.e. it was unplugged or the transport
    // wedged. A shorter timeout means we notice a dead link in ~1 s.
    // Serialize against the TX thread's device-config calls. recv returns at
    // chunk cadence (a few ms) while streaming, so a config call waits at
    // most one chunk for the lock. The metadata read below touches u->md,
    // which only this RX thread uses, so it stays outside the lock.
    pthread_mutex_lock(&u->dev_mu);
    uhd_error e = uhd_rx_streamer_recv(u->stream, bufs, cap_pairs, &u->md,
                                       /*timeout=*/1.0,
                                       /*one_packet=*/false,
                                       &n_recv);
    pthread_mutex_unlock(&u->dev_mu);
    if (e != UHD_ERROR_NONE) {
        if (u->recv_err_run == 0 || (u->recv_err_run % 2000) == 0) {
            log_uhd(e, "rx_recv");
            if (u->recv_err_run > 0)
                fprintf(stderr, "sdr_uhd: rx_recv: (%lu errors so far)\n",
                        u->recv_err_run + 1);
        }
        u->recv_err_run++;
        // A long unbroken run of recv errors is a dead link (device
        // unplugged / transport gone), not a transient overflow — report
        // it FATAL (-1) so the worker parks and the UI can warn, rather
        // than spinning forever. A real overflow recovers within a few
        // recvs, so the counter resets long before this trips.
        if (u->recv_err_run >= UHD_RX_DEAD_LINK_ERRORS) {
            fprintf(stderr,
                    "sdr_uhd: rx_recv: link dead after %lu consecutive "
                    "errors — reporting device loss\n", u->recv_err_run);
            return -1;
        }
        return 0;
    }
    u->recv_err_run = 0;
    uhd_rx_metadata_error_code_t mderr = 0;
    if (uhd_rx_metadata_error_code(u->md, &mderr) == UHD_ERROR_NONE
        && mderr != UHD_RX_METADATA_ERROR_CODE_NONE) {
        if (u->md_err_run == 0 || (u->md_err_run % 2000) == 0) {
            char errbuf[128] = {0};
            (void)uhd_rx_metadata_strerror(u->md, errbuf, sizeof errbuf);
            fprintf(stderr, "sdr_uhd: RX metadata error %d: %s%s\n",
                    (int)mderr, errbuf[0] ? errbuf : "(no detail)",
                    u->md_err_run > 0 ? " (repeating)" : "");
        }
        u->md_err_run++;
        // Overflow / late-packet are non-fatal — the samples we DID get
        // are already in the buffer; demod them rather than dropping.
    } else {
        u->md_err_run = 0;
    }
    return (ssize_t)n_recv;
}

static int uhd_set_freq(sdr_backend_t *be, double freq_hz)
{
    struct sdr_uhd *u = (struct sdr_uhd *)be->priv;
    if (u == NULL) return -1;
    char tune_args[] = "mode_n=fractional";
    uhd_tune_request_t req = {
        .target_freq     = freq_hz,
        .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
        .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
        .args            = tune_args,
    };
    uhd_tune_result_t res = {0};
    // RX retune touches the shared device handle -- serialize against recv
    // and the TX thread's config calls.
    pthread_mutex_lock(&u->dev_mu);
    int rc = log_uhd(uhd_usrp_set_rx_freq(u->dev, &req, 0, &res), "set_rx_freq")
             ? -1 : 0;
    if (rc == 0) {
        double f = u->actual_freq;
        if (uhd_usrp_get_rx_freq(u->dev, 0, &f) == UHD_ERROR_NONE)
            u->actual_freq = f;
    }
    pthread_mutex_unlock(&u->dev_mu);
    return rc;
}

static double uhd_get_actual_freq(sdr_backend_t *be)
{
    struct sdr_uhd *u = (struct sdr_uhd *)be->priv;
    return u ? u->actual_freq : 0.0;
}

static int uhd_set_gain(sdr_backend_t *be, double gain_db)
{
    struct sdr_uhd *u = (struct sdr_uhd *)be->priv;
    if (u == NULL) return -1;
    pthread_mutex_lock(&u->dev_mu);
    int rc = log_uhd(uhd_usrp_set_rx_gain(u->dev, gain_db, 0, ""), "set_rx_gain")
             ? -1 : 0;
    pthread_mutex_unlock(&u->dev_mu);
    return rc;
}

// --- TX burst (half-duplex) ----------------------------------------

// Full-duplex model: the RX streamer is started once at open and runs
// continuously for the device's whole life. A TX burst NEVER stops or
// retunes RX — it only maps/tunes/sends/unmaps the independent TX chain
// (the AD9361 RX and TX synthesizers are separate in FDD). The RX worker
// keeps reaping the USB transport on its own thread throughout the burst,
// so the transport never starves. Zero emission between bursts is still
// guaranteed by unmapping the TX subdev (tx_power_down), which powers the
// TX chain/LO down — not by stopping RX.

// Map (power up) or unmap (power down) the AD9361 TX chain by setting
// the TX subdev spec. "A:A" maps TX channel 0; "" leaves zero TX
// channels mapped.
static int tx_set_subdev(struct sdr_uhd *u, const char *markup)
{
    uhd_subdev_spec_handle sp = NULL;
    if (log_uhd(uhd_subdev_spec_make(&sp, markup), "subdev_spec_make")) return -1;
    int rc = log_uhd(uhd_usrp_set_tx_subdev_spec(u->dev, sp, 0),
                     "set_tx_subdev_spec") ? -1 : 0;
    uhd_subdev_spec_free(&sp);
    return rc;
}

// Power the AD9361 TX chain down at the end of a burst. Unmapping the TX
// frontend (empty TX subdev spec) makes the B200 driver drop the whole
// TX chain so the LO stops feeding through to the TX/RX port — the hard
// requirement is nothing on the antenna while in RX.
static void tx_power_down(struct sdr_uhd *u)
{
    if (u == NULL || !u->tx_powered) return;
    if (u->tx_stream != NULL) uhd_tx_streamer_free(&u->tx_stream);
    u->tx_stream       = NULL;
    u->tx_max_per_buff = 0;
    u->tx_rate_cached  = 0.0;
    u->tx_gain_cached  = -1.0;
    if (tx_set_subdev(u, "") != 0) {
        fprintf(stderr, "sdr_uhd: TX power-down via empty subdev spec failed; "
                        "transmit carrier may persist in RX\n");
        return;
    }
    u->tx_powered = 0;
}

// Re-map the TX chain before a burst. Clears the cached streamer so
// tx_streamer_lazy_build rebuilds on the fresh chain.
static int tx_power_up(struct sdr_uhd *u)
{
    if (u == NULL) return -1;
    if (u->tx_powered) return 0;
    if (tx_set_subdev(u, "A:A") != 0) return -1;
    u->tx_powered      = 1;
    u->tx_stream       = NULL;
    u->tx_max_per_buff = 0;
    u->tx_rate_cached  = 0.0;
    u->tx_gain_cached  = -1.0;
    return 0;
}

// Build the TX streamer on first use, cache rate so subsequent bursts
// only rebuild when the rate actually changed.
static int tx_streamer_lazy_build(struct sdr_uhd *u, double tx_rate_hz)
{
    if (u->tx_stream != NULL && fabs(u->tx_rate_cached - tx_rate_hz) < 1.0) {
        return 0;
    }
    if (u->tx_stream != NULL) {
        uhd_tx_streamer_free(&u->tx_stream);
        u->tx_stream = NULL;
        u->tx_max_per_buff = 0;
    }
    if (log_uhd(uhd_usrp_set_tx_antenna(u->dev, "TX/RX", 0),
                "set_tx_antenna")) return -1;
    if (log_uhd(uhd_usrp_set_tx_rate(u->dev, tx_rate_hz, 0),
                "set_tx_rate")) return -1;
    u->tx_rate_cached = tx_rate_hz;
    if (log_uhd(uhd_tx_streamer_make(&u->tx_stream),
                "tx_streamer_make")) return -1;
    size_t channels[1] = { 0 };
    uhd_stream_args_t args = {
        .cpu_format   = "sc16",
        .otw_format   = "sc16",
        .args         = "",
        .channel_list = channels,
        .n_channels   = 1,
    };
    if (log_uhd(uhd_usrp_get_tx_stream(u->dev, &args, u->tx_stream),
                "get_tx_stream")) return -1;
    if (log_uhd(uhd_tx_streamer_max_num_samps(u->tx_stream,
                                              &u->tx_max_per_buff),
                "tx_max_num_samps")) return -1;
    if (u->tx_max_per_buff == 0) u->tx_max_per_buff = 1024;
    u->tx_gain_cached = -1.0;  // force a set on first burst
    return 0;
}

static int uhd_tx_burst(sdr_backend_t *be, const sdr_tx_burst_params_t *p)
{
    struct sdr_uhd *u = (struct sdr_uhd *)be->priv;
    if (u == NULL || p == NULL || p->iq == NULL || p->n_samps == 0) return -1;

    int rc = -1;
    int locked = 0;
    // RX is NOT paused — it streams continuously on the worker thread.
    // We only bring the independent TX chain up, transmit, and drop it.
    // Hold the device lock across the configuration calls below (subdev /
    // rate / gain / time / freq) so they can't collide with the RX worker's
    // recv; it is released before the send loop so the actual transmission
    // overlaps RX as full-duplex.
    pthread_mutex_lock(&u->dev_mu);
    locked = 1;
    if (tx_power_up(u) != 0) goto resume;
    if (tx_streamer_lazy_build(u, p->tx_rate_hz) != 0) goto resume;

    if (fabs(u->tx_gain_cached - p->tx_gain_db) > 0.05) {
        if (log_uhd(uhd_usrp_set_tx_gain(u->dev, p->tx_gain_db, 0, ""),
                    "set_tx_gain")) goto resume;
        u->tx_gain_cached = p->tx_gain_db;
    }
    // Do NOT reset the device clock here — RX is streaming and a clock
    // reset would jump its sample timestamps. Schedule the burst at the
    // current device time + start_delay, which still gives the FPGA TX
    // FIFO room to buffer before emission begins.
    double tx_start_s = p->start_delay_s;
    {
        int64_t now_full = 0;
        double  now_frac = 0.0;
        if (uhd_usrp_get_time_now(u->dev, 0, &now_full, &now_frac)
                == UHD_ERROR_NONE) {
            tx_start_s = (double) now_full + now_frac + p->start_delay_s;
        }
    }
    {
        uhd_tune_request_t req = {
            .target_freq     = p->tx_freq_hz,
            .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
            .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
            .args            = NULL,
        };
        uhd_tune_result_t res = {0};
        if (log_uhd(uhd_usrp_set_tx_freq(u->dev, &req, 0, &res),
                    "set_tx_freq")) goto resume;
    }

    // Configuration is done. Drop the device lock so the send loop overlaps
    // the RX worker's recv -- the two streamers are independent, and holding
    // the lock across the whole burst would starve the RX transport.
    pthread_mutex_unlock(&u->dev_mu);
    locked = 0;

    uhd_tx_metadata_handle md = NULL;
    if (log_uhd(uhd_tx_metadata_make(&md, false, 0, 0.0, true, false),
                "tx_metadata_make")) goto resume;

    size_t sent_total = 0;
    while (sent_total < p->n_samps) {
        size_t remaining  = p->n_samps - sent_total;
        size_t this_chunk = (remaining < u->tx_max_per_buff)
                          ? remaining : u->tx_max_per_buff;
        int is_first = (sent_total == 0);
        int is_last  = (this_chunk == remaining);
        uhd_tx_metadata_free(&md);
        if (log_uhd(uhd_tx_metadata_make(&md,
                          /*has_time_spec=*/is_first ? true : false,
                          /*full_secs=*/(int64_t) tx_start_s,
                          /*frac_secs=*/tx_start_s
                                        - (double)(int64_t) tx_start_s,
                          /*start_of_burst=*/is_first ? true : false,
                          /*end_of_burst=*/is_last ? true : false),
                    "tx_metadata_make (loop)")) {
            uhd_tx_metadata_free(&md);
            goto resume;
        }
        const void *bufs[1] = { p->iq + sent_total * 2 };
        size_t items_sent = 0;
        double timeout = p->start_delay_s + 1.0;
        if (timeout < 1.0) timeout = 1.0;
        uhd_error e = uhd_tx_streamer_send(u->tx_stream, bufs, this_chunk,
                                           &md, timeout, &items_sent);
        if (e != UHD_ERROR_NONE) {
            log_uhd(e, "tx_streamer_send");
            uhd_tx_metadata_free(&md);
            goto resume;
        }
        if (items_sent == 0) {
            fprintf(stderr, "sdr_uhd: TX accepted 0 samples — backpressure?\n");
            uhd_tx_metadata_free(&md);
            goto resume;
        }
        sent_total += items_sent;
    }
    if (md != NULL) uhd_tx_metadata_free(&md);

    // Let the FPGA FIFO drain — the host call returned when the FIFO
    // accepted the last sample, not when the antenna stopped emitting.
    {
        double on_air_s = (double) p->n_samps / p->tx_rate_hz;
        double drain_s  = p->start_delay_s + on_air_s + 0.05;
        if (drain_s > 0.0) {
            usleep((useconds_t) (drain_s * 1e6));
        }
    }
    rc = 0;

resume:
    // Power the TX chain down (empty TX subdev) so the transmit LO stops
    // feeding through to the TX/RX port — nothing on the antenna while in
    // RX. Done even if the TX leg failed. RX was never stopped, so there
    // is nothing to resume; it has been streaming throughout.
    //
    // tx_power_down touches the device handle, so it must run under the lock.
    // The send loop dropped it; a config-phase failure jumps here still
    // holding it. Re-acquire only if needed, then release once.
    if (!locked) {
        pthread_mutex_lock(&u->dev_mu);
        locked = 1;
    }
    tx_power_down(u);
    pthread_mutex_unlock(&u->dev_mu);
    return rc;
}

static const sdr_backend_ops_t uhd_ops = {
    .name            = "uhd",
    .open            = uhd_open,
    .close           = uhd_close,
    .read_iq         = uhd_read_iq,
    .set_freq        = uhd_set_freq,
    .get_actual_freq = uhd_get_actual_freq,
    .set_gain        = uhd_set_gain,
    .tx_burst        = uhd_tx_burst,
};

const sdr_backend_ops_t *sdr_backend_uhd_ops(void)
{
    return &uhd_ops;
}
