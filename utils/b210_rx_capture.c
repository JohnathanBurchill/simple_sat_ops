/*

   Simple Satellite Operations  utils/b210_rx_capture.c

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

// USRP B210 RX capture tool. Tunes the B210 to a center frequency,
// streams IQ for a configurable duration, and (optionally) writes a
// mono WAV file with the demodulated audio plus, optionally, the
// raw int16 IQ stream.
//
// Demodulation modes (--demod=):
//
//   fm  (default)  FM discriminator: inst_freq[k] = arg(z[k]*conj(z[k-1]))
//                  scaled to PCM. Output is suitable for direct use by
//                  rx_decode and any other 9600-packet AX100 decoder.
//                  Full-scale PCM corresponds to ±25 kHz instantaneous
//                  deviation, so a 9600 GFSK h=0.5 signal (±2.4 kHz dev)
//                  occupies ~10% of full scale — comfortably above the
//                  bit slicer's threshold without clipping.
//
//   usb            Real-part-of-IQ "USB demod". Only useful when the
//                  B210 LO is tuned outside the desired audio passband
//                  (e.g., ~1 kHz below a CW carrier, so the steady tone
//                  lands at +1 kHz in baseband and shows up as a +1 kHz
//                  audio tone). The legacy behaviour of this tool.
//
// This is the SDR sibling of utils/rx_capture.c (which captures from
// ALSA on the receiver-radio path). Mac-buildable — SGP4 is never
// needed; ALSA is optional (when present, --monitor pipes the demoded
// audio to a playback device in real time alongside the WAV write).
//
// Sample-rate notes:
//   The B210's lowest comfortable host-visible RX rate is ~200 kHz
//   (AD9361 baseband floor). UHD will round whatever you ask for to
//   the closest supported value; the actual rate is queried back and
//   used as the WAV header's sample rate, so the output always plays
//   at the right speed even if UHD coerced the request. For 9600
//   packet decoding, 240-250 kHz gives 25-26 samples per symbol — a
//   comfortable working point.

#include "modem.h"   // pcm16_write_wav
#include "frontiersat.h"   // FRONTIERSAT_CARRIER_HZ
#include "carrier_trim.h"
#include "sw_nco.h"

#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include <uhd/types/tune_request.h>
#include <uhd/types/metadata.h>
#include <uhd/error.h>

#ifdef WITH_ALSA
#include <alsa/asoundlib.h>
#endif

static volatile int g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static int cmp_double_asc(const void *a, const void *b)
{
    double da = *(const double *)a, db = *(const double *)b;
    return (da < db) ? -1 : (da > db) ? 1 : 0;
}

static int uhd_check(uhd_error e, const char *what)
{
    if (e == UHD_ERROR_NONE) return 0;
    char errbuf[256] = {0};
    (void)uhd_get_last_error(errbuf, sizeof errbuf);
    fprintf(stderr, "b210_rx_capture: %s: UHD error %d: %s\n",
            what, (int)e, errbuf[0] ? errbuf : "(no detail)");
    return 1;
}

static void usage(FILE *f, const char *argv0)
{
    fprintf(f,
        "usage: %s [options]\n"
        "\n"
        "Capture I/Q from a USRP B210 and write a mono FM-demoded WAV\n"
        "(default) or USB-demoded WAV (legacy real(IQ) mode), plus\n"
        "optionally the raw int16 IQ stream.\n"
        "\n"
        "For 9600 packet decoding (FrontierSat AX100): default --demod=fm,\n"
        "tune to the carrier, capture at 240-250 kHz so rx_decode gets\n"
        "25+ samples per symbol:\n"
        "    %s --freq-hz=436150000 --rate=240000 --gain-db=50 \\\n"
        "        --duration-s=30 --wav=/tmp/frame.wav\n"
        "    rx_decode /tmp/frame.wav --hmac --sync-threshold=4\n"
        "\n"
        "For SSB / CW listening, switch to --demod=usb and tune the LO\n"
        "~1 kHz below the carrier so the steady tone lands at +1 kHz:\n"
        "    %s --demod=usb --freq-hz=432324000 --rate=250000 \\\n"
        "        --gain-db=60 --duration-s=30 --wav=/tmp/cw.wav\n"
        "    sox /tmp/cw.wav --rate 25000 /tmp/cw_25k.wav\n"
        "    aplay /tmp/cw_25k.wav   # (or: afplay on Mac)\n"
        "\n"
        "Tuning:\n"
        "  --freq-hz=<hz>             B210 LO center freq (default %.0f)\n"
        "  --gain-db=<n>              B210 RX gain dB, 0..76 (default 50)\n"
        "  --rate=<sps>               Sample rate (default 250000; the B210\n"
        "                             AD9361 floor is around 200 kHz)\n"
        "  --bw-hz=<hz>               Analog filter bandwidth (default = rate)\n"
        "  --duration-s=<f>           Capture length in seconds (default 5,\n"
        "                             max 600 — buffered in RAM)\n"
        "  --rx-antenna=<name>        B210 RX antenna port (default \"RX2\" =\n"
        "                             RF A RX-only jack; alternative: \"TX/RX\"\n"
        "                             = RF A TX/RX jack. The B210 has two SMA\n"
        "                             jacks per channel; antenna on the wrong\n"
        "                             one means silence even with the LO and\n"
        "                             gain set correctly.)\n"
        "  --device-args=<str>        UHD device args (default \"type=b200\")\n"
        "\n"
        "Demodulation:\n"
        "  --demod=<fm|usb>           Default fm. fm = FM discriminator\n"
        "                             (inst_freq from phase diff between\n"
        "                             consecutive complex samples), suitable\n"
        "                             for direct rx_decode consumption. usb =\n"
        "                             real(IQ), the legacy SSB / CW listening\n"
        "                             mode.\n"
        "  --fm-fullscale-hz=<hz>     Peak |inst_freq| that maps to 32767\n"
        "                             PCM (default 25000). Smaller values\n"
        "                             increase visual amplitude on quiet FM\n"
        "                             signals at the cost of clipping on\n"
        "                             wider ones. Bit-slicer doesn't care\n"
        "                             since it only reads polarity.\n"
        "  --fm-squelch=<auto|N>      During silence, the FM discriminator's\n"
        "                             phase difference of two near-zero IQ\n"
        "                             vectors is essentially a uniform-random\n"
        "                             angle in [-pi, +pi], which scales to\n"
        "                             saturated white noise. Squelch zeroes\n"
        "                             the audio whenever sqrt(I^2 + Q^2) < N.\n"
        "                             Default 'auto' = 4 * median(|IQ|) over\n"
        "                             the capture (catches both 'mostly\n"
        "                             silent with brief signal' and the\n"
        "                             reverse). Pass 0 to disable.\n"
        "\n"
        "Output (at least one required):\n"
        "  --wav=<path>               Mono S16 WAV (demoded per --demod=).\n"
        "                             Header sample rate is the *actual*\n"
        "                             rate UHD honoured (queried via\n"
        "                             uhd_usrp_get_rx_rate). Plays at the\n"
        "                             right speed regardless of what UHD\n"
        "                             coerced --rate to.\n"
        "  --raw-iq=<path>            Interleaved int16 IQ (sc16 host\n"
        "                             format), no header. Independent of\n"
        "                             --demod=.\n"
        "\n"
        "Live monitor (Linux only — requires this binary be built with ALSA):\n"
        "  --monitor                  Pipe the demoded audio to an ALSA\n"
        "                             playback device in real time alongside\n"
        "                             the WAV write. Independent of --wav=\n"
        "                             so you can monitor without saving, but\n"
        "                             at least one of --wav= / --raw-iq= is\n"
        "                             still required.\n"
        "  --monitor-device=<name>    ALSA device for --monitor (default\n"
        "                             \"default\"; use e.g. \"plughw:0,0\" to\n"
        "                             bypass PulseAudio resampling on a\n"
        "                             specific card).\n"
        "  --monitor-squelch=<a|N|0>  Live FM squelch on the monitor audio\n"
        "                             only — does not touch the WAV bytes.\n"
        "                             Default 'auto' bootstraps a 4 * median\n"
        "                             |IQ| threshold from the first ~0.25 s\n"
        "                             of capture, then applies it for the\n"
        "                             rest of the run. Pass a positive number\n"
        "                             for a fixed |IQ| threshold (engages\n"
        "                             immediately, no bootstrap delay), or\n"
        "                             'off' / 0 to disable. The post-capture\n"
        "                             WAV is governed by --fm-squelch=, which\n"
        "                             is independent.\n"
        "\n"
        "Other:\n"
        "  --help                     This message.\n",
        argv0, argv0, argv0, FRONTIERSAT_CARRIER_HZ);
}

int main(int argc, char **argv)
{
    double freq_hz   = FRONTIERSAT_CARRIER_HZ;
    double gain_db   = 50.0;
    double rate      = 250000.0;
    double bw_hz     = -1.0;          // -1 → take rate
    double duration_s = 5.0;
    const char *device_args = "type=b200";
    const char *wav_path    = NULL;
    const char *raw_iq_path = NULL;
    const char *rx_antenna  = "RX2";
    enum { DEMOD_FM = 0, DEMOD_USB = 1 } demod = DEMOD_FM;
    double fm_fullscale_hz = 25000.0;
    double fm_squelch_mag = -1.0;       // -1 = auto
    int    fm_squelch_explicit = 0;     // 1 if user passed --fm-squelch=
    int         monitor             = 0;
    const char *monitor_device      = "default";
    enum { MONSQ_AUTO = 0, MONSQ_FIXED = 1, MONSQ_OFF = 2 }
                monitor_sq_mode     = MONSQ_AUTO;
    double      monitor_sq_user_mag = 0.0;   // for MONSQ_FIXED

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0)              { usage(stdout, argv[0]); return 0; }
        else if (starts_with(a, "--freq-hz="))      freq_hz     = atof(a + 10);
        else if (starts_with(a, "--gain-db="))      gain_db     = atof(a + 10);
        else if (starts_with(a, "--rate="))         rate        = atof(a + 7);
        else if (starts_with(a, "--bw-hz="))        bw_hz       = atof(a + 8);
        else if (starts_with(a, "--duration-s="))   duration_s  = atof(a + 13);
        else if (starts_with(a, "--device-args="))  device_args = a + 14;
        else if (starts_with(a, "--rx-antenna="))   rx_antenna  = a + 13;
        else if (starts_with(a, "--wav="))          wav_path    = a + 6;
        else if (starts_with(a, "--raw-iq="))       raw_iq_path = a + 9;
        else if (starts_with(a, "--demod=")) {
            const char *s = a + 8;
            if      (strcmp(s, "fm")  == 0) demod = DEMOD_FM;
            else if (strcmp(s, "usb") == 0) demod = DEMOD_USB;
            else { fprintf(stderr, "b210_rx_capture: --demod=%s must be fm|usb\n", s); return 1; }
        }
        else if (starts_with(a, "--fm-fullscale-hz=")) {
            fm_fullscale_hz = atof(a + 18);
            if (fm_fullscale_hz <= 0.0) {
                fprintf(stderr, "b210_rx_capture: --fm-fullscale-hz must be > 0\n");
                return 1;
            }
        }
        else if (starts_with(a, "--fm-squelch=")) {
            const char *s = a + 13;
            fm_squelch_explicit = 1;
            if (strcmp(s, "auto") == 0) {
                fm_squelch_mag = -1.0;
            } else {
                fm_squelch_mag = atof(s);
                if (fm_squelch_mag < 0.0) {
                    fprintf(stderr, "b210_rx_capture: --fm-squelch must be >= 0 or 'auto'\n");
                    return 1;
                }
            }
        }
        else if (strcmp(a, "--monitor") == 0)        monitor = 1;
        else if (starts_with(a, "--monitor-device=")) monitor_device = a + 17;
        else if (starts_with(a, "--monitor-squelch=")) {
            const char *s = a + 18;
            if (strcmp(s, "auto") == 0) {
                monitor_sq_mode = MONSQ_AUTO;
            } else if (strcmp(s, "off") == 0 || strcmp(s, "0") == 0) {
                monitor_sq_mode = MONSQ_OFF;
            } else {
                double v = atof(s);
                if (v <= 0.0) {
                    fprintf(stderr, "b210_rx_capture: --monitor-squelch must be "
                                    "'auto', 'off', or a positive number\n");
                    return 1;
                }
                monitor_sq_mode     = MONSQ_FIXED;
                monitor_sq_user_mag = v;
            }
        }
        else { fprintf(stderr, "b210_rx_capture: unknown arg '%s'\n", a);
               usage(stderr, argv[0]); return 1; }
    }

    if (wav_path == NULL && raw_iq_path == NULL) {
        fprintf(stderr, "b210_rx_capture: need --wav= or --raw-iq=\n");
        return 1;
    }
    if (bw_hz < 0) bw_hz = rate;
    if (duration_s <= 0 || duration_s > 600.0) {
        fprintf(stderr, "b210_rx_capture: --duration-s must be in (0, 600]\n");
        return 1;
    }

    // Pre-allocate the full IQ buffer. At 250 kS/s × 30 s × 4 B/sample
    // = 30 MB. Plenty in RAM. The 600 s cap above bounds this at
    // 600 MB worst case — if you need bigger, --raw-iq= straight to
    // disk is the fallback, but that would need a different code path.
    size_t n_request = (size_t)(duration_s * rate);
    int16_t *iq = (int16_t *)calloc(n_request * 2, sizeof(int16_t));
    if (iq == NULL) {
        fprintf(stderr, "b210_rx_capture: out of memory for %zu samples\n", n_request);
        return 1;
    }

    uhd_usrp_handle dev = NULL;
    if (uhd_check(uhd_usrp_make(&dev, device_args), "uhd_usrp_make")) {
        free(iq); return 1;
    }
    int rc = 0;

    if (uhd_check(uhd_usrp_set_rx_rate(dev, rate, 0), "set_rx_rate")) { rc = 1; goto done; }
    double actual_rate = rate;
    if (uhd_usrp_get_rx_rate(dev, 0, &actual_rate) != UHD_ERROR_NONE) actual_rate = rate;
    if (fabs(actual_rate - rate) / rate > 0.01) {
        fprintf(stderr, "b210_rx_capture: requested %.0f S/s, got %.0f S/s "
                        "(B210 coerced); using actual rate for output.\n",
                rate, actual_rate);
    }
    if (uhd_check(uhd_usrp_set_rx_gain(dev, gain_db, 0, ""), "set_rx_gain")) { rc = 1; goto done; }
    if (uhd_check(uhd_usrp_set_rx_bandwidth(dev, bw_hz, 0), "set_rx_bandwidth")) { rc = 1; goto done; }
    // Antenna defaults to "RX2" (the RF A RX-only SMA jack). UHD's own
    // default depends on firmware/version; without an explicit set, you
    // can end up listening through the wrong port and get pure silence.
    if (uhd_check(uhd_usrp_set_rx_antenna(dev, rx_antenna, 0), "set_rx_antenna")) { rc = 1; goto done; }
    char actual_antenna[32] = {0};
    if (uhd_usrp_get_rx_antenna(dev, 0, actual_antenna, sizeof actual_antenna) != UHD_ERROR_NONE) {
        snprintf(actual_antenna, sizeof actual_antenna, "%s", rx_antenna);
    }
    {
        uhd_tune_request_t req = {
            .target_freq     = freq_hz,
            .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
            .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
            .args = NULL,
        };
        uhd_tune_result_t res = {0};
        if (uhd_check(uhd_usrp_set_rx_freq(dev, &req, 0, &res), "set_rx_freq")) { rc = 1; goto done; }
    }

    double actual_freq = freq_hz;
    if (uhd_usrp_get_rx_freq(dev, 0, &actual_freq) != UHD_ERROR_NONE) actual_freq = freq_hz;
    fprintf(stderr,
            "b210_rx_capture: freq=%.6f MHz (req %.6f) rate=%.0f S/s "
            "gain=%.1f dB bw=%.0f Hz antenna=%s duration=%.2f s -> %zu samples\n",
            actual_freq / 1e6, freq_hz / 1e6, actual_rate, gain_db, bw_hz,
            actual_antenna, duration_s, n_request);

    // Pre-write NCO: pulls the carrier to DC by cancelling the UHD
    // tune residual + the persistent per-host carrier trim. Applied
    // to each recv chunk before the WAV write / ALSA monitor / IQ
    // file write, so all three see the centered signal.
    //
    // sw_nco_set_freq(f) shifts signals DOWN by f. The carrier sits
    // at baseband = (target − actual) + trim_hz, so set the NCO
    // frequency to that sum to drop it onto DC.
    double trim_hz = carrier_trim_load_hz();
    double tune_residual_hz = freq_hz - actual_freq;
    double centering_freq_hz = tune_residual_hz + trim_hz;
    sw_nco_t centering_nco;
    sw_nco_init(&centering_nco, actual_rate);
    sw_nco_set_freq(&centering_nco, centering_freq_hz);
    if (fabs(centering_freq_hz) >= 1.0) {
        fprintf(stderr,
                "b210_rx_capture: centering NCO = %.3f Hz "
                "(tune residual %.3f + carrier trim %.3f)\n",
                centering_freq_hz, tune_residual_hz, trim_hz);
    }

    uhd_rx_streamer_handle stream = NULL;
    if (uhd_check(uhd_rx_streamer_make(&stream), "rx_streamer_make")) { rc = 1; goto done; }
    {
        size_t channels[1] = { 0 };
        uhd_stream_args_t args = {
            .cpu_format   = "sc16",   // host: int16 IQ
            .otw_format   = "sc16",   // wire: int16 IQ
            .args         = "",
            .channel_list = channels,
            .n_channels   = 1,
        };
        if (uhd_check(uhd_usrp_get_rx_stream(dev, &args, stream),
                      "get_rx_stream")) { rc = 1; goto done_stream; }
    }

    size_t max_per = 0;
    if (uhd_check(uhd_rx_streamer_max_num_samps(stream, &max_per),
                  "rx_max_num_samps")) { rc = 1; goto done_stream; }
    if (max_per == 0) max_per = 2040;

    uhd_rx_metadata_handle md = NULL;
    if (uhd_check(uhd_rx_metadata_make(&md), "rx_metadata_make")) { rc = 1; goto done_stream; }

    // Tell the FPGA to start delivering samples continuously. We'll
    // stop streaming when we've collected enough or hit Ctrl-C.
    uhd_stream_cmd_t cmd = {
        .stream_mode = UHD_STREAM_MODE_START_CONTINUOUS,
        .num_samps = 0, .stream_now = true,
        .time_spec_full_secs = 0, .time_spec_frac_secs = 0.0,
    };
    if (uhd_check(uhd_rx_streamer_issue_stream_cmd(stream, &cmd),
                  "issue_start_stream")) { rc = 1; goto done_md; }

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);

#ifdef WITH_ALSA
    // Live monitor: open an ALSA playback stream so the operator hears
    // the demoded audio in real time as IQ arrives. Best-effort — on
    // open / set_params failure we warn and continue with the WAV-only
    // path. Mac builds (no WITH_ALSA) print a "not built with ALSA"
    // notice instead.
    snd_pcm_t   *alsa            = NULL;
    int16_t     *monitor_chunk   = NULL;
    double       prev_I_live     = 0.0;
    double       prev_Q_live     = 0.0;
    int          have_prev_live  = 0;
    const double k_scale_live    = actual_rate * 32767.0
                                    / (fm_fullscale_hz * 2.0 * M_PI);
    // Live monitor squelch threshold² — applied only to the audio sent
    // to ALSA, never to the iq[] buffer or the WAV write. 0 = ungated.
    // Filled in below: immediately for MONSQ_FIXED, after a short
    // bootstrap window for MONSQ_AUTO.
    double       live_sq_mag_sq  = 0.0;
    double      *boot_mags_sq    = NULL;
    size_t       boot_n          = 0;
    size_t       boot_target     = 0;
    if (monitor) {
        int aerr = snd_pcm_open(&alsa, monitor_device,
                                SND_PCM_STREAM_PLAYBACK, 0);
        if (aerr < 0) {
            fprintf(stderr, "b210_rx_capture: snd_pcm_open(%s): %s "
                            "— monitor disabled\n",
                    monitor_device, snd_strerror(aerr));
            alsa = NULL;
        } else {
            // 200 ms latency: forgiving of irregular UHD recv chunk
            // arrival without giving the operator an annoying delay.
            // soft_resample = 1 lets ALSA's plug layer convert from
            // actual_rate (e.g. 240 kHz) to whatever the playback HW
            // supports.
            unsigned int latency_us = 200000;
            aerr = snd_pcm_set_params(alsa,
                                      SND_PCM_FORMAT_S16_LE,
                                      SND_PCM_ACCESS_RW_INTERLEAVED,
                                      1,
                                      (unsigned int)actual_rate,
                                      1,
                                      latency_us);
            if (aerr < 0) {
                fprintf(stderr, "b210_rx_capture: snd_pcm_set_params"
                                "(%u Hz mono S16_LE): %s — monitor disabled\n",
                        (unsigned int)actual_rate, snd_strerror(aerr));
                snd_pcm_close(alsa);
                alsa = NULL;
            } else {
                monitor_chunk = (int16_t *)malloc(max_per * sizeof(int16_t));
                if (monitor_chunk == NULL) {
                    fprintf(stderr, "b210_rx_capture: out of memory for "
                                    "monitor buffer — monitor disabled\n");
                    snd_pcm_close(alsa);
                    alsa = NULL;
                } else {
                    // Set up monitor squelch (FM only; USB ignores).
                    char sq_buf[128];
                    const char *sq_desc = "n/a";
                    if (demod == DEMOD_FM) {
                        if (monitor_sq_mode == MONSQ_FIXED) {
                            live_sq_mag_sq = monitor_sq_user_mag
                                              * monitor_sq_user_mag;
                            snprintf(sq_buf, sizeof sq_buf,
                                     "fixed (|IQ| > %.0f)",
                                     monitor_sq_user_mag);
                            sq_desc = sq_buf;
                        } else if (monitor_sq_mode == MONSQ_AUTO) {
                            boot_target = (size_t)(0.25 * actual_rate);
                            if (boot_target < 1024) boot_target = 1024;
                            boot_mags_sq = (double *)malloc(
                                boot_target * sizeof(double));
                            if (boot_mags_sq == NULL) {
                                fprintf(stderr, "b210_rx_capture: out of "
                                                "memory for monitor squelch "
                                                "bootstrap — disabled\n");
                                monitor_sq_mode = MONSQ_OFF;
                                sq_desc = "off (alloc failed)";
                            } else {
                                snprintf(sq_buf, sizeof sq_buf,
                                         "auto (4 * median |IQ| over first "
                                         "%.2f s; ungated until then)",
                                         (double)boot_target / actual_rate);
                                sq_desc = sq_buf;
                            }
                        } else {
                            sq_desc = "off";
                        }
                    }
                    fprintf(stderr,
                            "b210_rx_capture: monitor on %s @ %u Hz mono "
                            "(demod=%s, monitor squelch=%s)\n",
                            monitor_device, (unsigned int)actual_rate,
                            demod == DEMOD_FM ? "fm" : "usb", sq_desc);
                }
            }
        }
    }
#else
    if (monitor) {
        fprintf(stderr, "b210_rx_capture: --monitor requires ALSA support; "
                        "this binary was built without it.\n");
    }
#endif

    size_t got = 0;
    while (got < n_request && !g_stop) {
        size_t want = n_request - got;
        if (want > max_per) want = max_per;
        void *bufs[1] = { iq + got * 2 };
        size_t n_recv = 0;
        uhd_error e = uhd_rx_streamer_recv(stream, bufs, want, &md,
                                           /*timeout=*/3.0,
                                           /*one_packet=*/false,
                                           &n_recv);
        if (e != UHD_ERROR_NONE) {
            uhd_check(e, "rx_recv");
            // overflow / timeout aren't fatal — log + keep going.
            continue;
        }
        uhd_rx_metadata_error_code_t mderr = 0;
        if (uhd_rx_metadata_error_code(md, &mderr) == UHD_ERROR_NONE
            && mderr != UHD_RX_METADATA_ERROR_CODE_NONE) {
            char errbuf[128] = {0};
            (void)uhd_rx_metadata_strerror(md, errbuf, sizeof errbuf);
            fprintf(stderr, "b210_rx_capture: RX metadata error %d: %s\n",
                    (int)mderr, errbuf[0] ? errbuf : "(no detail)");
        }
        // Centre the carrier on DC before any downstream consumer
        // (monitor, demod, IQ write) touches the chunk. NCO phase is
        // preserved across calls so chunk boundaries don't pop.
        if (n_recv > 0 && centering_freq_hz != 0.0) {
            sw_nco_apply(&centering_nco, iq + got * 2, n_recv);
        }
        got += n_recv;

#ifdef WITH_ALSA
        // Live monitor: incremental demod of the just-received chunk
        // straight to ALSA. State (prev_I_live / prev_Q_live) carries
        // the FM phase reference across chunk boundaries; without it
        // the very first sample of each chunk would discontinuity-pop.
        // Independent of the post-capture batch demod that drives the
        // WAV write — that one re-reads iq[] from scratch with the
        // 4×median squelch threshold.
        if (alsa != NULL && monitor_chunk != NULL && n_recv > 0) {
            size_t base = got - n_recv;
            size_t out_n = 0;
            // Auto-squelch bootstrap: collect |IQ|² over the first
            // ~boot_target samples (~0.25 s of capture), then qsort
            // for the median. After the bootstrap window completes,
            // live_sq_mag_sq becomes (4 * median)² and stays put for
            // the rest of the capture. Audio plays raw during the
            // bootstrap window — operator hears the noise floor for
            // about a quarter second before the gate engages.
            if (monitor_sq_mode == MONSQ_AUTO
                && live_sq_mag_sq == 0.0
                && boot_mags_sq != NULL) {
                for (size_t i = 0;
                     i < n_recv && boot_n < boot_target; i++) {
                    size_t idx = base + i;
                    double I = (double)iq[idx * 2 + 0];
                    double Q = (double)iq[idx * 2 + 1];
                    boot_mags_sq[boot_n++] = I * I + Q * Q;
                }
                if (boot_n >= boot_target) {
                    qsort(boot_mags_sq, boot_n, sizeof(double),
                          cmp_double_asc);
                    double median_sq  = boot_mags_sq[boot_n / 2];
                    double median_mag = sqrt(median_sq);
                    double thr        = 4.0 * median_mag;
                    live_sq_mag_sq    = thr * thr;
                    free(boot_mags_sq);
                    boot_mags_sq = NULL;
                    fprintf(stderr,
                            "b210_rx_capture: monitor squelch engaged "
                            "(|IQ| > %.0f, 4 * median over %.2f s)\n",
                            thr, (double)boot_n / actual_rate);
                }
            }
            if (demod == DEMOD_USB) {
                for (size_t kk = 0; kk < n_recv; kk++) {
                    monitor_chunk[kk] = iq[(base + kk) * 2 + 0];
                }
                out_n = n_recv;
            } else {
                size_t kk = 0;
                if (!have_prev_live) {
                    prev_I_live    = (double)iq[base * 2 + 0];
                    prev_Q_live    = (double)iq[base * 2 + 1];
                    have_prev_live = 1;
                    kk             = 1;
                }
                for (; kk < n_recv; kk++) {
                    size_t idx = base + kk;
                    double I   = (double)iq[idx * 2 + 0];
                    double Q   = (double)iq[idx * 2 + 1];
                    int16_t pcm = 0;
                    if (live_sq_mag_sq > 0.0) {
                        double mag_sq  = I * I + Q * Q;
                        double prev_sq = prev_I_live * prev_I_live
                                          + prev_Q_live * prev_Q_live;
                        double min_sq  = (mag_sq < prev_sq) ? mag_sq : prev_sq;
                        if (min_sq < live_sq_mag_sq) {
                            prev_I_live = I; prev_Q_live = Q;
                            monitor_chunk[out_n++] = 0;
                            continue;
                        }
                    }
                    double dphi  = atan2(Q * prev_I_live - I * prev_Q_live,
                                         I * prev_I_live + Q * prev_Q_live);
                    double pcm_d = dphi * k_scale_live;
                    if (pcm_d >  32767.0) pcm_d =  32767.0;
                    if (pcm_d < -32768.0) pcm_d = -32768.0;
                    pcm = (int16_t)lround(pcm_d);
                    prev_I_live = I; prev_Q_live = Q;
                    monitor_chunk[out_n++] = pcm;
                }
            }
            if (out_n > 0) {
                snd_pcm_sframes_t w = snd_pcm_writei(alsa, monitor_chunk, out_n);
                if (w < 0) {
                    int rerr = snd_pcm_recover(alsa, (int)w, /*silent=*/1);
                    if (rerr < 0) {
                        fprintf(stderr, "b210_rx_capture: snd_pcm_writei "
                                        "recover: %s\n", snd_strerror(rerr));
                    }
                }
            }
        }
#endif
    }

    cmd.stream_mode = UHD_STREAM_MODE_STOP_CONTINUOUS;
    cmd.stream_now  = true;
    (void)uhd_rx_streamer_issue_stream_cmd(stream, &cmd);

    fprintf(stderr,
            "b210_rx_capture: captured %zu samples (%.2f s%s)\n",
            got, (double)got / actual_rate,
            g_stop ? ", interrupted" : "");

    // Quick signal-presence stats so the operator can tell if RF is
    // actually reaching the front-end. If both peak and RMS are tiny
    // (< a few hundred), gain is too low or the antenna is on the
    // wrong jack — the WAV will show no waveform regardless of how
    // good the demod is. Also computes |IQ| stats used for the FM
    // squelch threshold below.
    int32_t peak_i = 0, peak_q = 0;
    double sum_sq = 0.0;
    double median_iq_mag = 0.0;
    for (size_t i = 0; i < got; i++) {
        int16_t I = iq[i * 2 + 0];
        int16_t Q = iq[i * 2 + 1];
        if (abs(I) > peak_i) peak_i = abs(I);
        if (abs(Q) > peak_q) peak_q = abs(Q);
        sum_sq += (double)I * I + (double)Q * Q;
    }
    double rms = got > 0 ? sqrt(sum_sq / (double)got) : 0.0;
    fprintf(stderr,
            "b210_rx_capture: IQ stats peak |I|=%d |Q|=%d rms=%.1f "
            "(out of 32767). %s\n",
            peak_i, peak_q, rms,
            (peak_i + peak_q < 100) ? "(near silent — check --rx-antenna or raise --gain-db)"
                                    : "(signal present)");

    // For auto-squelch: median |IQ| magnitude. Median (not mean) so
    // brief signal bursts in mostly-silent recordings don't drag the
    // floor up, and brief silences in mostly-keyed recordings don't
    // drag it down. Computed only when needed.
    if (wav_path != NULL && demod == DEMOD_FM && fm_squelch_mag < 0.0) {
        // Subsample to keep this O(N log N) bounded — full-file
        // qsort of millions of doubles is overkill for a noise-floor
        // estimate. Take every 64th sample (still ~110k samples for
        // a 30-s 240-kHz capture).
        size_t step = (got > 65536) ? 64 : 1;
        size_t n_sub = (got + step - 1) / step;
        double *mags = (double *)malloc(n_sub * sizeof(double));
        if (mags != NULL) {
            size_t j = 0;
            for (size_t i = 0; i < got; i += step) {
                double I = (double)iq[i * 2 + 0];
                double Q = (double)iq[i * 2 + 1];
                mags[j++] = sqrt(I * I + Q * Q);
            }
            // Quickselect via qsort + index. n_sub is small enough
            // (~110k) that qsort is fine.
            qsort(mags, j, sizeof(double), cmp_double_asc);
            median_iq_mag = (j > 0) ? mags[j / 2] : 0.0;
            free(mags);
        }
    }

    if (raw_iq_path != NULL) {
        FILE *fp = fopen(raw_iq_path, "wb");
        if (fp == NULL) { perror("fopen --raw-iq"); rc = 1; goto done_md; }
        size_t wrote = fwrite(iq, sizeof(int16_t) * 2, got, fp);
        fclose(fp);
        if (wrote != got) {
            fprintf(stderr, "b210_rx_capture: short write to %s\n", raw_iq_path);
            rc = 1;
        } else {
            fprintf(stderr, "b210_rx_capture: wrote %zu IQ samples to %s\n",
                    got, raw_iq_path);
        }
    }

    if (wav_path != NULL) {
        size_t n_audio = 0;
        int16_t *audio = NULL;

        if (demod == DEMOD_USB) {
            // Legacy "USB demod" via real(IQ): take every even-indexed
            // int16. Useful only when the LO is offset from the carrier
            // so the signal of interest sits in the audio passband.
            n_audio = got;
            audio = (int16_t *)malloc(n_audio * sizeof(int16_t));
            if (audio == NULL) {
                fprintf(stderr, "b210_rx_capture: out of memory for audio buffer\n");
                rc = 1; goto done_md;
            }
            for (size_t i = 0; i < got; i++) audio[i] = iq[i * 2 + 0];
        } else {
            // FM demod: inst_freq[k] = arg(z[k] * conj(z[k-1])) where
            // z = I + jQ. Computed as atan2(Q1*I0 - I1*Q0, I1*I0 + Q1*Q0)
            // — equivalent to the two-step (unwrap then diff) approach
            // but safe across the ±π wrap. Output is one fewer sample
            // than the input (k = 1..got-1).
            //
            // Scale: full-scale 32767 PCM corresponds to ±fm_fullscale_hz
            // of instantaneous deviation. inst_freq_Hz = dphi_rad *
            // actual_rate / (2π), so pcm = dphi * (actual_rate * 32767
            // / (fm_fullscale_hz * 2π)). For the default 25 kHz full-
            // scale, a 9600 GFSK h=0.5 signal (±2.4 kHz dev) lands at
            // ~10% of full scale.
            if (got < 2) {
                fprintf(stderr, "b210_rx_capture: not enough samples for FM demod\n");
                rc = 1; goto done_md;
            }
            n_audio = got - 1;
            audio = (int16_t *)malloc(n_audio * sizeof(int16_t));
            if (audio == NULL) {
                fprintf(stderr, "b210_rx_capture: out of memory for audio buffer\n");
                rc = 1; goto done_md;
            }
            // Resolve auto squelch: 4x median |IQ| over the capture.
            // Multiplier 4x is empirical — high enough to cleanly
            // gate the random-phase noise during silence, low enough
            // to preserve weak signals a few sigma above the floor.
            double sq_mag = fm_squelch_mag;
            if (sq_mag < 0.0) {
                sq_mag = 4.0 * median_iq_mag;
                fprintf(stderr,
                        "b210_rx_capture: auto FM squelch threshold = %.0f "
                        "(4 * median |IQ| = %.0f)\n",
                        sq_mag, median_iq_mag);
            } else if (fm_squelch_explicit && sq_mag == 0.0) {
                fprintf(stderr,
                        "b210_rx_capture: FM squelch disabled (--fm-squelch=0)\n");
            } else if (fm_squelch_explicit) {
                fprintf(stderr,
                        "b210_rx_capture: FM squelch threshold = %.0f (explicit)\n",
                        sq_mag);
            }
            const double sq_mag_sq = sq_mag * sq_mag;

            const double k_scale = actual_rate * 32767.0
                                   / (fm_fullscale_hz * 2.0 * M_PI);
            int peak_pcm = 0;
            int clipped = 0;
            size_t squelched = 0;
            for (size_t k = 1; k < got; k++) {
                double I0 = (double)iq[(k - 1) * 2 + 0];
                double Q0 = (double)iq[(k - 1) * 2 + 1];
                double I1 = (double)iq[k * 2 + 0];
                double Q1 = (double)iq[k * 2 + 1];
                // Squelch on the smaller of the two adjacent IQ
                // magnitudes — a single-sample dropout at the edge
                // of a burst would otherwise leak through. Compare
                // squared values so we don't pay for two sqrts.
                double mag1_sq = I1 * I1 + Q1 * Q1;
                double mag0_sq = I0 * I0 + Q0 * Q0;
                double min_sq = (mag0_sq < mag1_sq) ? mag0_sq : mag1_sq;
                int16_t pcm = 0;
                if (min_sq < sq_mag_sq) {
                    squelched++;
                } else {
                    double dphi = atan2(Q1 * I0 - I1 * Q0,
                                        I1 * I0 + Q1 * Q0);
                    double pcm_d = dphi * k_scale;
                    if (pcm_d >  32767.0) { pcm_d =  32767.0; clipped++; }
                    if (pcm_d < -32768.0) { pcm_d = -32768.0; clipped++; }
                    pcm = (int16_t)lround(pcm_d);
                    if (abs(pcm) > peak_pcm) peak_pcm = abs(pcm);
                }
                audio[k - 1] = pcm;
            }
            // PCM peak as a fraction of full scale, rounded to nearest %.
            int peak_pct = (peak_pcm * 100 + 16383) / 32767;
            // Inverse of k_scale: pcm-to-Hz at this rate.
            double peak_hz = (double)peak_pcm * fm_fullscale_hz / 32767.0;
            int squelched_pct = (int)((squelched * 100) / (n_audio > 0 ? n_audio : 1));
            fprintf(stderr,
                    "b210_rx_capture: FM demod peak |inst_freq| ~ %.0f Hz "
                    "(%d%% PCM full scale; %d clipped, %zu squelched/%zu = %d%%)\n",
                    peak_hz, peak_pct, clipped, squelched, n_audio, squelched_pct);
        }

        if (pcm16_write_wav(wav_path, audio, n_audio, (int)actual_rate) != 0) {
            fprintf(stderr, "b210_rx_capture: pcm16_write_wav(%s) failed\n", wav_path);
            rc = 1;
        } else {
            fprintf(stderr,
                    "b210_rx_capture: wrote %zu mono samples (%.2f s @ %.0f Hz, "
                    "demod=%s) to %s\n",
                    n_audio, (double)n_audio / actual_rate, actual_rate,
                    demod == DEMOD_FM ? "fm" : "usb", wav_path);
        }
        free(audio);
    }

done_md:
#ifdef WITH_ALSA
    if (alsa != NULL) {
        (void)snd_pcm_drain(alsa);
        snd_pcm_close(alsa);
    }
    free(monitor_chunk);
    free(boot_mags_sq);
#endif
    if (md != NULL)     uhd_rx_metadata_free(&md);
done_stream:
    if (stream != NULL) uhd_rx_streamer_free(&stream);
done:
    if (dev != NULL)    uhd_usrp_free(&dev);
    free(iq);
    return rc;
}
