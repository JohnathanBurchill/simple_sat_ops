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
#include "frontiersat.h"
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
            .fm_lo_compensation_hz  = 0.0,
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
            char path[512];
            snprintf(path, sizeof path, "%s_%05.1fdB.iq", iq_prefix, g);
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
            ssize_t r = b210_rx_tx_core_pump(core,
                                             pcm, max_chunk,
                                             iq,  max_chunk * 2u,
                                             &out_iq_pairs);
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
    }

    if (csv != NULL) fclose(csv);
    if (g_stop) fprintf(stderr, "interrupted\n");
    return rc;
}
