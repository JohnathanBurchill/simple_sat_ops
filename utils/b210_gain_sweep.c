/*

   Simple Satellite Operations  utils/b210_gain_sweep.c

   Sweep the B210 RX gain across a range, dwell at each step, and
   report mean post-decim IQ power.  Run this to find the operating
   gain — the knee where extra dB of RX gain stops raising the noise
   floor 1:1 is the point above which more gain doesn't improve
   sensitivity.

   Uses the same b210_rx_tx_core wrapper that simple_sat_ops does,
   so the measurement conditions (rate, decim, FIR cutoff, antenna)
   match the operational receiver exactly when invoked with the
   defaults.  Each gain step opens the device fresh — UHD reset is
   ~1 s and avoids touching the core API just to add a runtime
   set-gain call.

   Optional per-step IQ dump (--iq-prefix=PATH) lets you FFT each
   gain offline to see whether discrete spurs (60/120/180 Hz lines
   near DC, LO leakage, etc.) scale with the noise floor (baseband-
   coupled — more gain helps) or stay fixed above it (RF-coupled —
   gain doesn't help).

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

#include "b210_rx_tx_core.h"
#include "carrier_trim.h"
#include "frontiersat.h"
#include "pdf_writer.h"
#include "sso_paths.h"

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MAX_GAIN_STEPS 256

typedef struct {
    double gain_db;
    double mean_sq;
    double mean_dbfs;
    int    peak_env;
    size_t n_samples;
} sweep_point_t;

// Pretty single-page summary: gain (x) vs mean(I²+Q²) in dBFS (y),
// connected polyline, faint slope-1 reference anchored at the lowest
// gain so the knee where the line peels off is obvious by eye.
static int write_knee_pdf(const char *path,
                          const sweep_point_t *pts, size_t n,
                          double freq_hz, double rate_hz,
                          unsigned decim, double dwell_s,
                          const char *antenna);

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static double monotonic_seconds(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Sweeps B210 RX gain, dwells, reports mean(I^2 + Q^2) per gain.\n"
        "\n"
        "  --freq=HZ            RX center frequency (default %.0f)\n"
        "  --rate=HZ            UHD input rate (default 480000)\n"
        "  --decim=N            IQ decimation factor (default 5 -> 96 kHz)\n"
        "  --decim-cutoff=HZ    decim FIR -6 dB cutoff (default 18000)\n"
        "  --antenna=NAME       RX antenna (default RX2)\n"
        "  --bw=HZ              analog filter bandwidth (default = rate)\n"
        "\n"
        "  --gain-start=DB      first gain to test (default 0)\n"
        "  --gain-end=DB        last gain to test (default 73)\n"
        "  --gain-step=DB       step size (default 5)\n"
        "  --dwell=S            measurement window per gain (default 5.0)\n"
        "  --settle=S           seconds to skip after open before\n"
        "                         starting the average (default 1.0)\n"
        "\n"
        "  --csv=PATH           append CSV: gain_db,n_samples,mean_sq,\n"
        "                                   mean_sq_dbfs,peak_env\n"
        "  --iq-prefix=PATH     also dump per-step IQ to\n"
        "                       PATH_<gain>dB.iq (raw int16 I,Q pairs)\n"
        "  --testing            auto-create a bench folder under\n"
        "                       <FrontierSat>/Testing/YYYYMMDD/HHMMLT_gain_sweep/\n"
        "                       and default --csv / --iq-prefix into it\n"
        "                       unless you also passed them explicitly.\n"
        "\n"
        "  --ad9361-dc-track=on|off   AD9361 background DC-offset tracking.\n"
        "                             Default ON, mirroring simple_sat_ops, so\n"
        "                             the noise-floor knee found here matches\n"
        "                             what the operator UI will see at the\n"
        "                             same gain. Pass off to characterise the\n"
        "                             raw ADC floor (the BBDC IIR otherwise\n"
        "                             notches a few bins around DC).\n"
        "  --ad9361-iq-track=on|off   AD9361 background IQ-balance tracking.\n"
        "                             Default OFF (the spike-comb culprit).\n"
        "                             Pass on for A/B comparison.\n"
        "\n"
        "Output: one row per gain on stdout — column \"mean_dBFS\" is\n"
        "10*log10(mean(I^2+Q^2) / 32767^2). Plot dBFS vs gain; the knee\n"
        "is the gain above which the line stops being slope ~1.\n",
        argv0, (double) FRONTIERSAT_CARRIER_HZ - 25000.0);
}

int main(int argc, char *argv[])
{
    double      freq_hz       = (double) FRONTIERSAT_CARRIER_HZ - 25000.0;
    double      rate_hz       = 480000.0;
    unsigned    decim         = 5u;
    double      decim_cutoff  = 18000.0;
    const char *rx_antenna    = "RX2";
    double      bw_hz         = -1.0;

    double      gain_start    = 0.0;
    double      gain_end      = 73.0;
    double      gain_step     = 5.0;
    double      dwell_s       = 5.0;
    double      settle_s      = 1.0;

    const char *csv_path      = NULL;
    const char *iq_prefix     = NULL;
    int         testing_mode  = 0;

    // Defaults mirror simple_sat_ops so a sweep finds the operator-
    // visible noise-floor knee directly. CLI flags below override.
    int         dc_track      = 1;
    int         iq_track      = 0;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0 || strcmp(a, "-h") == 0) {
            usage(argv[0]); return 0;
        } else if (starts_with(a, "--freq="))         freq_hz      = atof(a + 7);
        else if (starts_with(a, "--rate="))           rate_hz      = atof(a + 7);
        else if (starts_with(a, "--decim="))          decim        = (unsigned) atoi(a + 8);
        else if (starts_with(a, "--decim-cutoff="))   decim_cutoff = atof(a + 15);
        else if (starts_with(a, "--antenna="))        rx_antenna   = a + 10;
        else if (starts_with(a, "--bw="))             bw_hz        = atof(a + 5);
        else if (starts_with(a, "--gain-start="))     gain_start   = atof(a + 13);
        else if (starts_with(a, "--gain-end="))       gain_end     = atof(a + 11);
        else if (starts_with(a, "--gain-step="))      gain_step    = atof(a + 12);
        else if (starts_with(a, "--dwell="))          dwell_s      = atof(a + 8);
        else if (starts_with(a, "--settle="))         settle_s     = atof(a + 9);
        else if (starts_with(a, "--csv="))            csv_path     = a + 6;
        else if (starts_with(a, "--iq-prefix="))      iq_prefix    = a + 12;
        else if (strcmp(a, "--testing") == 0)         testing_mode = 1;
        else if (starts_with(a, "--ad9361-dc-track=")) {
            const char *v = a + 18;
            dc_track = (strcmp(v, "on") == 0
                        || strcmp(v, "true") == 0
                        || strcmp(v, "1") == 0) ? 1 : 0;
        }
        else if (starts_with(a, "--ad9361-iq-track=")) {
            const char *v = a + 18;
            iq_track = (strcmp(v, "on") == 0
                        || strcmp(v, "true") == 0
                        || strcmp(v, "1") == 0) ? 1 : 0;
        }
        else {
            fprintf(stderr, "unknown option: %s\n", a);
            usage(argv[0]); return 2;
        }
    }

    if (gain_step <= 0.0) { fprintf(stderr, "--gain-step must be > 0\n"); return 2; }
    if (gain_end < gain_start) { fprintf(stderr, "--gain-end < --gain-start\n"); return 2; }
    if (dwell_s <= 0.0) { fprintf(stderr, "--dwell must be > 0\n"); return 2; }

    // --testing: bench-mode pass folder under
    //   <FrontierSat>/Testing/YYYYMMDD/HHMMLT_gain_sweep/
    // Suffix distinguishes the gain sweep run from a simple_sat_ops
    // --testing pass that might start in the same minute.  CSV and IQ
    // outputs default into the folder; explicit --csv / --iq-prefix
    // still win.
    char testing_folder[512];
    char testing_csv[640];
    char testing_iq[640];
    if (testing_mode) {
        time_t now = time(NULL);
        struct tm now_local;
        localtime_r(&now, &now_local);
        int n = snprintf(testing_folder, sizeof testing_folder,
                         "%s/%04d%02d%02d/%02d%02dLT_gain_sweep",
                         sso_testing_dir(),
                         now_local.tm_year + 1900,
                         now_local.tm_mon + 1,
                         now_local.tm_mday,
                         now_local.tm_hour,
                         now_local.tm_min);
        if (n <= 0 || (size_t) n >= sizeof testing_folder) {
            fprintf(stderr, "b210_gain_sweep: --testing folder path too long\n");
            return 1;
        }
        if (sso_mkdir_p(testing_folder) != 0) {
            fprintf(stderr, "b210_gain_sweep: mkdir -p %s failed: %s\n",
                    testing_folder, strerror(errno));
            return 1;
        }
        fprintf(stderr, "b210_gain_sweep: --testing folder %s\n", testing_folder);
        if (csv_path == NULL) {
            snprintf(testing_csv, sizeof testing_csv,
                     "%s/sweep.csv", testing_folder);
            csv_path = testing_csv;
        }
        if (iq_prefix == NULL) {
            snprintf(testing_iq, sizeof testing_iq,
                     "%s/spur", testing_folder);
            iq_prefix = testing_iq;
        }
    }

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);

    // libuhd's default console log level is INFO, which means the
    // "[INFO] [B200] Detecting internal GPSDO..." stanza scrolls past
    // every time we open the device — fifteen-plus times during a
    // full sweep.  Set the threshold to error before the first
    // uhd_usrp_make so only real failures break into the operator's
    // terminal.  setenv with overwrite=0 leaves a pre-existing value
    // alone so the operator can still crank it back up to debug.
    setenv("UHD_LOG_CONSOLE_LEVEL", "error", 0);

    FILE *csv = NULL;
    if (csv_path != NULL) {
        csv = fopen(csv_path, "w");
        if (csv == NULL) { perror(csv_path); return 1; }
        fprintf(csv,
            "# b210_gain_sweep — mean post-decim IQ power vs RX gain\n"
            "# freq_hz=%.0f rate_hz=%.0f decim=%u cutoff_hz=%.0f antenna=%s\n"
            "# dwell_s=%.2f settle_s=%.2f\n"
            "gain_db,n_samples,mean_sq,mean_sq_dbfs,peak_env\n",
            freq_hz, rate_hz, decim, decim_cutoff, rx_antenna,
            dwell_s, settle_s);
    }

    printf("# b210_gain_sweep freq=%.0f Hz rate=%.0f Hz decim=%u "
           "post-decim=%.0f Hz antenna=%s\n",
           freq_hz, rate_hz, decim,
           rate_hz / (decim == 0u ? 1.0 : (double) decim),
           rx_antenna);
    printf("# dwell=%.1f s   settle=%.1f s\n", dwell_s, settle_s);
    printf("gain_dB   mean_dBFS  peak_env  n_samples\n");

    sweep_point_t pts[MAX_GAIN_STEPS];
    size_t        n_pts = 0u;

    // Loaded once — creates the trim file with 0 if it doesn't yet
    // exist, so every B210-using program agrees on the calibration.
    double trim_hz = carrier_trim_load_hz();

    int rc = 0;
    for (double g = gain_start; g <= gain_end + 1e-9 && !g_stop; g += gain_step) {

        b210_rx_tx_core_params_t p = {
            .freq_hz                = freq_hz,
            .rate_hz                = rate_hz,
            .gain_db                = g,
            .bw_hz                  = bw_hz,
            .fm_fullscale_hz        = 25000.0,
            .device_args            = "type=b200",
            .rx_antenna             = rx_antenna,
            .decim_factor           = decim,
            .decim_cutoff_hz        = decim_cutoff,
            .decim_taps             = 0u,
            // Default to the same AD9361 tracking config the
            // operator UI uses (dc on, iq off), so the knee we find
            // here is the knee they'll see. --ad9361-dc-track=off /
            // --ad9361-iq-track=on override for characterisation.
            .rx_dc_offset_track     = dc_track,
            .rx_iq_balance_track    = iq_track,
            .fm_lo_compensation_hz  = 0.0,
            .carrier_trim_hz        = trim_hz,
        };
        b210_rx_tx_core_t *core = NULL;
        if (b210_rx_tx_core_open(&p, &core) != 0) {
            fprintf(stderr, "gain=%.1f dB: open failed, skipping\n", g);
            rc = 1;
            continue;
        }

        size_t max_chunk = b210_rx_tx_core_max_chunk(core);
        if (max_chunk == 0) max_chunk = 4096;

        int16_t *pcm = (int16_t *) malloc(max_chunk * sizeof(int16_t));
        int16_t *iq  = (int16_t *) malloc(max_chunk * 2u * sizeof(int16_t));
        if (pcm == NULL || iq == NULL) {
            fprintf(stderr, "alloc failed\n");
            free(pcm); free(iq);
            b210_rx_tx_core_close(core);
            rc = 1;
            break;
        }

        FILE *iq_fp = NULL;
        if (iq_prefix != NULL) {
            // 512 - "_<gg.g>dB.iq" (12) - NUL = 499 path chars max.
            // %.499s bounds the prefix so GCC's -Wformat-truncation
            // proves the snprintf can't overflow.
            char path[512];
            snprintf(path, sizeof path, "%.499s_%05.1fdB.iq", iq_prefix, g);
            iq_fp = fopen(path, "wb");
            if (iq_fp == NULL) {
                fprintf(stderr, "%s: %s\n", path, strerror(errno));
            }
        }

        double t_open  = monotonic_seconds();
        double t_meas  = t_open + settle_s;
        double t_stop  = t_meas + dwell_s;

        double sum_sq   = 0.0;
        size_t n_samps  = 0u;
        int16_t peak    = 0;

        while (!g_stop) {
            double t = monotonic_seconds();
            if (t >= t_stop) break;

            size_t out_iq_pairs = 0u;
            // Use the raw IQ tap (carrier at +lo_offset baseband). The
            // sweep only cares about RX level, not carrier centering;
            // the decode-path tap would be redundant here.
            ssize_t r = b210_rx_tx_core_pump(core,
                                             pcm, max_chunk,
                                             iq,  max_chunk * 2u,
                                             &out_iq_pairs,
                                             NULL, 0u, NULL);
            if (r < 0) { fprintf(stderr, "gain=%.1f dB: pump fatal\n", g); break; }
            if (r == 0) continue;
            if (out_iq_pairs == 0u) continue;

            if (t < t_meas) continue;

            for (size_t k = 0u; k < out_iq_pairs; ++k) {
                int16_t I = iq[2u * k + 0u];
                int16_t Q = iq[2u * k + 1u];
                sum_sq += (double) I * (double) I + (double) Q * (double) Q;
                int16_t aI = (int16_t)(I < 0 ? -I : I);
                int16_t aQ = (int16_t)(Q < 0 ? -Q : Q);
                if (aI > peak) peak = aI;
                if (aQ > peak) peak = aQ;
            }
            n_samps += out_iq_pairs;
            if (iq_fp != NULL) {
                fwrite(iq, sizeof(int16_t), out_iq_pairs * 2u, iq_fp);
            }
        }

        if (iq_fp != NULL) fclose(iq_fp);
        free(pcm); free(iq);
        b210_rx_tx_core_close(core);

        if (n_samps == 0u) {
            fprintf(stderr, "gain=%.1f dB: no samples collected\n", g);
            if (csv != NULL) {
                fprintf(csv, "%.1f,0,nan,nan,0\n", g);
                fflush(csv);
            }
            continue;
        }

        double mean_sq = sum_sq / (double) n_samps;
        double dbfs    = 10.0 * log10(mean_sq / (32767.0 * 32767.0));
        printf("%6.1f   %8.2f   %7d  %9zu\n",
               g, dbfs, (int) peak, n_samps);
        fflush(stdout);

        if (csv != NULL) {
            fprintf(csv, "%.1f,%zu,%.6e,%.3f,%d\n",
                    g, n_samps, mean_sq, dbfs, (int) peak);
            fflush(csv);
        }
        if (n_pts < MAX_GAIN_STEPS) {
            pts[n_pts].gain_db   = g;
            pts[n_pts].mean_sq   = mean_sq;
            pts[n_pts].mean_dbfs = dbfs;
            pts[n_pts].peak_env  = (int) peak;
            pts[n_pts].n_samples = n_samps;
            ++n_pts;
        }
    }

    if (csv != NULL) fclose(csv);

    // PDF knee plot — emit under --testing automatically.  We don't
    // expose a --pdf flag yet because the only consumer right now is
    // the bench-mode summary; if a future caller wants the PDF
    // without a Testing folder we can add it without touching the
    // shared writer.
    if (testing_mode && n_pts >= 2u && !g_stop) {
        char pdf_path[640];
        snprintf(pdf_path, sizeof pdf_path,
                 "%s/knee_plot.pdf", testing_folder);
        if (write_knee_pdf(pdf_path, pts, n_pts,
                           freq_hz, rate_hz, decim, dwell_s,
                           rx_antenna) == 0) {
            fprintf(stderr, "b210_gain_sweep: wrote %s\n", pdf_path);
        } else {
            fprintf(stderr, "b210_gain_sweep: pdf write failed (%s)\n",
                    pdf_path);
        }
    }

    if (g_stop) fprintf(stderr, "interrupted\n");
    return rc;
}

static int write_knee_pdf(const char *path,
                          const sweep_point_t *pts, size_t n,
                          double freq_hz, double rate_hz,
                          unsigned decim, double dwell_s,
                          const char *antenna)
{
    if (path == NULL || pts == NULL || n < 2u) return -1;

    // Landscape letter, generous margins so the page reads nicely as
    // a one-page bench report.
    const float page_w = 792.0f;
    const float page_h = 612.0f;
    pdfw_t *w = pdfw_begin(path, page_w, page_h);
    if (w == NULL) return -1;

    const float MARGIN   = 48.0f;
    const float HEADER_H = 70.0f;
    const float FOOTER_H = 24.0f;

    pdfw_set_fill(w, (pdfw_rgb_t){250, 250, 252, 255});
    pdfw_rect_fill(w, MARGIN, MARGIN, page_w - 2.0f*MARGIN, HEADER_H);
    pdfw_set_stroke(w, PDFW_DGREY);
    pdfw_lw(w, 0.7f);
    pdfw_rect_stroke(w, MARGIN, MARGIN, page_w - 2.0f*MARGIN, HEADER_H);

    pdfw_set_fill(w, PDFW_BLACK);
    pdfw_text(w, MARGIN + 10, MARGIN + 6,
              "b210_gain_sweep -- noise floor vs RX gain", 14.0f, 0);

    char line1[256];
    snprintf(line1, sizeof line1,
             "freq=%.0f Hz   rate=%.0f Hz   decim=%u  ->  %.0f Hz post-decim   "
             "antenna=%s   dwell=%.1f s",
             freq_hz, rate_hz, decim,
             rate_hz / (decim == 0u ? 1.0 : (double) decim),
             antenna, dwell_s);
    pdfw_text(w, MARGIN + 10, MARGIN + 28, line1, 9.0f, 1);

    time_t now = time(NULL);
    struct tm tm_local;
    localtime_r(&now, &tm_local);
    char ts[64];
    strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S %Z", &tm_local);
    char line2[160];
    snprintf(line2, sizeof line2, "captured %s   %zu gain steps", ts, n);
    pdfw_text(w, MARGIN + 10, MARGIN + 44, line2, 9.0f, 1);

    // Plot area.
    const float pL = MARGIN + 60.0f;
    const float pR = page_w - MARGIN - 16.0f;
    const float pT = MARGIN + HEADER_H + 28.0f;
    const float pB = page_h - MARGIN - FOOTER_H - 30.0f;
    const float plot_w = pR - pL;
    const float plot_h = pB - pT;

    pdfw_set_stroke(w, (pdfw_rgb_t){100, 100, 110, 255});
    pdfw_lw(w, 0.8f);
    pdfw_rect_stroke(w, pL, pT, plot_w, plot_h);

    // Axis ranges.  X = gain (snap to 10 dB grid).  Y = dBFS, expand
    // to the nearest 5 dB on each side and pad by 5 dB so the slope-1
    // reference line has room.
    double x_min = pts[0].gain_db;
    double x_max = pts[n - 1u].gain_db;
    double y_min = pts[0].mean_dbfs;
    double y_max = pts[0].mean_dbfs;
    for (size_t i = 0u; i < n; ++i) {
        if (pts[i].mean_dbfs < y_min) y_min = pts[i].mean_dbfs;
        if (pts[i].mean_dbfs > y_max) y_max = pts[i].mean_dbfs;
    }
    x_min = floor(x_min / 10.0) * 10.0;
    x_max = ceil (x_max / 10.0) * 10.0;
    y_min = floor((y_min - 2.0) / 5.0) * 5.0;
    y_max = ceil ((y_max + 2.0) / 5.0) * 5.0;
    if (x_max <= x_min) x_max = x_min + 10.0;
    if (y_max <= y_min) y_max = y_min + 5.0;

    const double x_span = x_max - x_min;
    const double y_span = y_max - y_min;
    #define XPIX(GX) ((float)(pL + ((GX) - x_min) / x_span * plot_w))
    #define YPIX(GY) ((float)(pB - ((GY) - y_min) / y_span * plot_h))

    // X-axis grid + ticks every 10 dB.
    pdfw_lw(w, 0.3f);
    for (double gx = x_min; gx <= x_max + 1e-9; gx += 10.0) {
        float x = XPIX(gx);
        pdfw_set_stroke(w, PDFW_LGREY);
        pdfw_line(w, x, pT, x, pB);
        pdfw_set_stroke(w, PDFW_DGREY);
        pdfw_line(w, x, pB, x, pB + 4);
        char buf[16];
        snprintf(buf, sizeof buf, "%g", gx);
        float tw = pdfw_str_width(buf, 9.0f, 1);
        pdfw_set_fill(w, PDFW_BLACK);
        pdfw_text(w, x - 0.5f*tw, pB + 6, buf, 9.0f, 1);
    }
    // Y-axis grid + ticks every 5 dB.
    for (double gy = y_min; gy <= y_max + 1e-9; gy += 5.0) {
        float y = YPIX(gy);
        pdfw_set_stroke(w, PDFW_LGREY);
        pdfw_line(w, pL, y, pR, y);
        pdfw_set_stroke(w, PDFW_DGREY);
        pdfw_line(w, pL - 4, y, pL, y);
        char buf[16];
        snprintf(buf, sizeof buf, "%g", gy);
        float tw = pdfw_str_width(buf, 9.0f, 1);
        pdfw_set_fill(w, PDFW_BLACK);
        pdfw_text(w, pL - 6 - tw, y - 4, buf, 9.0f, 1);
    }

    // Slope-1 reference: a 1 dB/dB line anchored at the lowest gain
    // point.  Below the knee the data should sit right on top of it.
    pdfw_set_stroke(w, (pdfw_rgb_t){180, 180, 200, 255});
    pdfw_lw(w, 0.7f);
    {
        double ref_x0 = pts[0].gain_db;
        double ref_y0 = pts[0].mean_dbfs;
        double ref_x1 = x_max;
        double ref_y1 = ref_y0 + (ref_x1 - ref_x0);
        if (ref_y1 > y_max) {
            ref_x1 = ref_x0 + (y_max - ref_y0);
            ref_y1 = y_max;
        }
        pdfw_line(w, XPIX(ref_x0), YPIX(ref_y0),
                     XPIX(ref_x1), YPIX(ref_y1));
    }

    // Data polyline.
    pdfw_set_stroke(w, (pdfw_rgb_t){30, 120, 200, 255});
    pdfw_lw(w, 1.4f);
    for (size_t i = 1u; i < n; ++i) {
        pdfw_line(w,
            XPIX(pts[i - 1u].gain_db), YPIX(pts[i - 1u].mean_dbfs),
            XPIX(pts[i].gain_db),      YPIX(pts[i].mean_dbfs));
    }
    // Markers.
    pdfw_set_fill(w, (pdfw_rgb_t){30, 120, 200, 255});
    for (size_t i = 0u; i < n; ++i) {
        float x = XPIX(pts[i].gain_db);
        float y = YPIX(pts[i].mean_dbfs);
        pdfw_rect_fill(w, x - 2.0f, y - 2.0f, 4.0f, 4.0f);
    }

    // Axis labels.
    pdfw_set_fill(w, PDFW_BLACK);
    pdfw_text(w, 0.5f*(pL + pR) - pdfw_str_width("RX gain (dB)", 10.0f, 0)*0.5f,
              pB + 22, "RX gain (dB)", 10.0f, 0);
    // Y-axis label — drawn horizontally at the top-left of the plot
    // because the simple writer doesn't do rotated text.
    pdfw_text(w, MARGIN + 10, pT - 18,
              "mean(I^2 + Q^2)  [dBFS]", 10.0f, 0);

    // Legend.
    pdfw_set_stroke(w, (pdfw_rgb_t){180, 180, 200, 255});
    pdfw_lw(w, 0.7f);
    pdfw_line(w, pR - 200.0f, pT + 10.0f, pR - 180.0f, pT + 10.0f);
    pdfw_set_fill(w, PDFW_GREY);
    pdfw_text(w, pR - 175.0f, pT + 4.0f, "slope-1 reference", 8.5f, 0);
    pdfw_set_stroke(w, (pdfw_rgb_t){30, 120, 200, 255});
    pdfw_lw(w, 1.4f);
    pdfw_line(w, pR - 200.0f, pT + 26.0f, pR - 180.0f, pT + 26.0f);
    pdfw_set_fill(w, PDFW_BLACK);
    pdfw_text(w, pR - 175.0f, pT + 20.0f, "measured", 8.5f, 0);

    // Footer.
    pdfw_set_fill(w, PDFW_GREY);
    pdfw_text(w, MARGIN, page_h - MARGIN - 12,
              "Knee = lowest gain at which the curve peels off the "
              "slope-1 reference. Operate ~5 dB above it.",
              9.0f, 0);

    return pdfw_end(w);
}
