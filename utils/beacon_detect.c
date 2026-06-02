/*

    Simple Satellite Operations  utils/beacon_detect.c

    Find AX100 beacons in a post-FM-demod WAV by detecting the moments
    where signal-band energy (default 4500–5100 Hz, the GMSK preamble
    tone at half the bit rate) rises relative to out-of-band noise
    (default 8000–22000 Hz). Real beacons land as a paired event:
    the FM discriminator captures the carrier, the noise floor drops
    across the upper band, and the tone in the data band stands out.
    The ratio is the discriminating quantity, not either band alone.

    Detections are filtered against a 20 s lattice (anchored at the
    strongest detection) so isolated impulsive RFI events that don't
    line up with the beacon cadence get dropped.

    Designed to be the C counterpart of the post-pass triage Python
    that produced ~/FrontierSat/RAO/Operations/.../beacons_v2/. WAV
    in, CSV out; optional --render-png shells out to ffmpeg per
    detection for ±0.5 s spectrograms.

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

#include "biquad.h"
#include "monitor_squelch.h"
#include "wav_read.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Run a 4th-order bandpass (two cascaded biquads, same coeffs) over
// the input PCM and write the squared output for RMS-window summation
// in the caller. Output buffer must hold n samples.
static void run_bp_squared(const int16_t *pcm, size_t n,
                           double f0, double bw_hz, double fs,
                           double *out_sq)
{
    biquad_t s1, s2;
    biquad_bpf(&s1, f0, bw_hz, fs);
    biquad_bpf(&s2, f0, bw_hz, fs);
    for (size_t i = 0; i < n; i++) {
        double y = biquad_step(&s1, (double)pcm[i]);
        y = biquad_step(&s2, y);
        out_sq[i] = y * y;
    }
}

// Average y² inside fixed windows. Returns the new envelope length
// (n / win_samples). Windowed RMS is sqrt(env[i]).
static size_t rms_envelope(const double *sq, size_t n,
                           size_t win_samples, double *env_out)
{
    size_t n_env = n / win_samples;
    for (size_t i = 0; i < n_env; i++) {
        double acc = 0.0;
        const double *p = sq + i * win_samples;
        for (size_t j = 0; j < win_samples; j++) acc += p[j];
        env_out[i] = acc / (double)win_samples;
    }
    return n_env;
}

// Helper for qsort — sorts doubles ascending.
static int dcmp(const void *a, const void *b)
{
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

// Median + median absolute deviation. Sorts a copy of the input. The
// caller's buffer is left untouched. Returns 0 on success, -1 on OOM.
static int median_mad(const double *v, size_t n, double *median, double *mad)
{
    if (n == 0) return -1;
    double *tmp = malloc(n * sizeof(double));
    if (tmp == NULL) return -1;
    memcpy(tmp, v, n * sizeof(double));
    qsort(tmp, n, sizeof(double), dcmp);
    double med = (n & 1) ? tmp[n / 2]
                         : 0.5 * (tmp[n / 2 - 1] + tmp[n / 2]);
    for (size_t i = 0; i < n; i++) tmp[i] = fabs(v[i] - med);
    qsort(tmp, n, sizeof(double), dcmp);
    double m = (n & 1) ? tmp[n / 2]
                       : 0.5 * (tmp[n / 2 - 1] + tmp[n / 2]);
    free(tmp);
    *median = med;
    *mad    = m;
    return 0;
}

// Find local maxima in v[] above threshold and at least min_distance
// indices apart. Greedy: each accepted peak suppresses neighbours
// within ±min_distance. Returns the number of peaks written to
// out_idx (capped at out_cap).
static size_t find_peaks(const double *v, size_t n,
                         double threshold, size_t min_distance,
                         size_t *out_idx, size_t out_cap)
{
    size_t out_n = 0;
    for (size_t i = 1; i + 1 < n; i++) {
        if (v[i] < threshold) continue;
        if (v[i] <= v[i - 1] || v[i] <= v[i + 1]) continue;
        // Check distance to last accepted peak.
        if (out_n > 0 && i - out_idx[out_n - 1] < min_distance) {
            // Keep the larger of the two.
            if (v[i] > v[out_idx[out_n - 1]]) {
                out_idx[out_n - 1] = i;
            }
            continue;
        }
        if (out_n >= out_cap) return out_n;
        out_idx[out_n++] = i;
    }
    return out_n;
}

// Keep only peaks that fall within tol_s of a period_s lattice phase-
// anchored at the strongest peak. Drops non-cadence false positives
// like impulsive RFI. Operates in place on out_idx; returns the new
// count. Pass period_s <= 0 to skip the filter entirely.
static size_t lattice_filter(size_t *idx, size_t n,
                             const double *score,
                             double cell_s,
                             double period_s, double tol_s)
{
    if (period_s <= 0.0 || n <= 1) return n;
    size_t imax = 0;
    for (size_t i = 1; i < n; i++) {
        if (score[idx[i]] > score[idx[imax]]) imax = i;
    }
    double anchor_t = (double)idx[imax] * cell_s;
    double phase    = fmod(anchor_t, period_s);
    if (phase < 0.0) phase += period_s;
    size_t out_n = 0;
    for (size_t i = 0; i < n; i++) {
        double t   = (double)idx[i] * cell_s;
        double off = fmod(t - phase, period_s);
        if (off < 0.0) off += period_s;
        double dist = off < period_s - off ? off : period_s - off;
        if (dist <= tol_s) idx[out_n++] = idx[i];
    }
    return out_n;
}

static void usage(FILE *dest, const char *prog)
{
    fprintf(dest,
        "usage: %s <wavfile> [options]\n"
        "\n"
        "Detect AX100-style beacons in an FM-demoded WAV by finding\n"
        "moments where (signal-band energy) / (out-of-band noise) spikes.\n"
        "Real beacons capture the FM discriminator, dropping out-of-band\n"
        "noise while raising the data-band tone — so the *ratio* moves,\n"
        "not either band alone.\n"
        "\n"
        "Detection bands:\n"
        "  --sig-lo=<hz>            Signal band lower edge (default 4500)\n"
        "  --sig-hi=<hz>            Signal band upper edge (default 5100)\n"
        "  --noise-lo=<hz>          Noise band lower edge (default 8000)\n"
        "  --noise-hi=<hz>          Noise band upper edge (default 22000)\n"
        "  --env-ms=<n>             RMS envelope window in ms (default 10)\n"
        "\n"
        "Detection:\n"
        "  --threshold-sigma=<f>    Threshold above ratio median in σ\n"
        "                           (MAD-based; default 2.5)\n"
        "  --min-spacing-s=<f>      Minimum spacing between peaks (default 15)\n"
        "  --lattice-period-s=<f>   Beacon cadence for lattice filter\n"
        "                           (default 20; 0 disables)\n"
        "  --lattice-tol-s=<f>      Tolerance around lattice (default 2.0)\n"
        "  --max-burst-s=<f>        Drop peaks inside an above-threshold\n"
        "                           run longer than this (default 1.5).\n"
        "                           Suppresses Doppler-swept carriers\n"
        "                           that hang in the signal band for\n"
        "                           seconds. 0 disables.\n"
        "\n"
        "Output:\n"
        "  --csv=<path>             CSV path (default stdout)\n"
        "  --render-png[=<dir>]     Optional: shell out to ffmpeg per\n"
        "                           detection for a ±<window-s>s\n"
        "                           showspectrumpic PNG. <dir> defaults\n"
        "                           to <wav_dir>/beacons. ffmpeg must\n"
        "                           be on PATH.\n"
        "  --window-s=<f>           Render half-window (default 0.5)\n"
        "  --gate-wav=<path>        Run the input through the same\n"
        "                           carrier-presence squelch b210_rx_tx\n"
        "                           uses live, write the gated audio to\n"
        "                           <path>. Use this to listen-test the\n"
        "                           squelch on a recorded WAV before a\n"
        "                           live pass — silence between beacons,\n"
        "                           ~0.5 s of audio per detected burst.\n"
        "  --gate-offset-db=<dB>    Auto-mode threshold offset above\n"
        "                           bootstrap mean (default 3.0)\n"
        "  --gate-fixed-db=<dB>     Use a fixed dB threshold instead of\n"
        "                           auto-bootstrap.\n"
        "  --gate-hold-s=<f>        Hold-open time after each crossing\n"
        "                           (default 0.5).\n"
        "  --quiet                  Suppress the human-readable summary\n"
        "  --help                   This message.\n",
        prog);
}

static int starts_with(const char *s, const char *p)
{
    return strncmp(s, p, strlen(p)) == 0;
}

// Minimal RIFF/WAVE writer for 16-bit mono PCM. Same on-disk format
// the modem and b210_rx_tx emit. Used only by --gate-wav.
static int write_wav_pcm16_mono(const char *path,
                                const int16_t *samples, size_t n,
                                int sample_rate)
{
    FILE *f = fopen(path, "wb");
    if (f == NULL) return -1;
    uint32_t data_bytes = (uint32_t)(n * sizeof(int16_t));
    uint32_t riff_bytes = 36 + data_bytes;
    uint16_t chans = 1;
    uint32_t sr    = (uint32_t)sample_rate;
    uint32_t br    = sr * chans * 2;
    uint16_t ba    = chans * 2;
    uint16_t bps   = 16;
    fwrite("RIFF", 1, 4, f);
    fwrite(&riff_bytes, 4, 1, f);
    fwrite("WAVEfmt ", 1, 8, f);
    uint32_t fmt_size = 16; uint16_t fmt = 1;
    fwrite(&fmt_size, 4, 1, f);
    fwrite(&fmt, 2, 1, f);
    fwrite(&chans, 2, 1, f);
    fwrite(&sr, 4, 1, f);
    fwrite(&br, 4, 1, f);
    fwrite(&ba, 2, 1, f);
    fwrite(&bps, 2, 1, f);
    fwrite("data", 1, 4, f);
    fwrite(&data_bytes, 4, 1, f);
    size_t w = fwrite(samples, sizeof(int16_t), n, f);
    fclose(f);
    return (w == n) ? 0 : -1;
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "beacon_detect")) return 0;
    const char *wav_path     = NULL;
    double sig_lo            = 4500.0;
    double sig_hi            = 5100.0;
    double noise_lo          = 8000.0;
    double noise_hi          = 22000.0;
    int    env_ms            = 10;
    double threshold_sigma   = 2.5;
    double min_spacing_s     = 15.0;
    double lattice_period_s  = 20.0;
    double lattice_tol_s     = 2.0;
    // Carrier-suppression: drop peaks that sit inside an above-threshold
    // run longer than max_burst_s. A real packet's ratio_db crosses
    // threshold for tens of ms to a couple of seconds; a Doppler-swept
    // carrier dwelling in the signal band can hold for many seconds and
    // would otherwise produce a string of "detections". Set 0 to
    // disable.
    double max_burst_s       = 1.5;
    const char *csv_path     = NULL;
    int    quiet             = 0;
    int    render_png        = 0;
    const char *render_dir   = NULL;
    double render_half_s     = 0.5;
    const char *gate_wav_path = NULL;
    double gate_offset_db    = 3.0;
    int    gate_fixed_set    = 0;
    double gate_fixed_db     = 0.0;
    double gate_hold_s       = 0.5;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (starts_with(a, "--sig-lo="))           sig_lo = atof(a + 9);
        else if (starts_with(a, "--sig-hi="))           sig_hi = atof(a + 9);
        else if (starts_with(a, "--noise-lo="))         noise_lo = atof(a + 11);
        else if (starts_with(a, "--noise-hi="))         noise_hi = atof(a + 11);
        else if (starts_with(a, "--env-ms="))           env_ms = atoi(a + 9);
        else if (starts_with(a, "--threshold-sigma="))  threshold_sigma = atof(a + 18);
        else if (starts_with(a, "--min-spacing-s="))    min_spacing_s = atof(a + 16);
        else if (starts_with(a, "--lattice-period-s=")) lattice_period_s = atof(a + 19);
        else if (starts_with(a, "--lattice-tol-s="))    lattice_tol_s = atof(a + 16);
        else if (starts_with(a, "--max-burst-s="))      max_burst_s = atof(a + 14);
        else if (starts_with(a, "--csv="))              csv_path = a + 6;
        else if (strcmp(a, "--quiet") == 0)             quiet = 1;
        else if (strcmp(a, "--render-png") == 0)        render_png = 1;
        else if (starts_with(a, "--render-png=")) {
            render_png = 1;
            render_dir = a + 13;
        }
        else if (starts_with(a, "--window-s="))         render_half_s = atof(a + 11);
        else if (starts_with(a, "--gate-wav="))         gate_wav_path = a + 11;
        else if (starts_with(a, "--gate-offset-db="))   gate_offset_db = atof(a + 17);
        else if (starts_with(a, "--gate-fixed-db=")) {
            gate_fixed_db  = atof(a + 16);
            gate_fixed_set = 1;
        }
        else if (starts_with(a, "--gate-hold-s="))      gate_hold_s = atof(a + 14);
        else if (a[0] == '-' && a[1] == '-') {
            fprintf(stderr, "beacon_detect: unknown flag '%s'\n", a);
            return EXIT_FAILURE;
        }
        else if (wav_path == NULL) wav_path = a;
        else {
            fprintf(stderr, "beacon_detect: unexpected positional arg '%s'\n", a);
            usage(stderr, argv[0]);
            return EXIT_FAILURE;
        }
    }
    if (wav_path == NULL) {
        usage(stderr, argv[0]);
        return EXIT_FAILURE;
    }

    int16_t *pcm = NULL; size_t n_pcm = 0; int rate = 0; int ch = 0;
    if (wav_read_pcm16(wav_path, &pcm, &n_pcm, &rate, &ch) != 0) {
        return EXIT_FAILURE;
    }
    if (ch != 1) {
        fprintf(stderr,
                "beacon_detect: WAV has %d channels — only mono is supported. "
                "Use a tool like sox to remix to mono first.\n", ch);
        free(pcm);
        return EXIT_FAILURE;
    }
    double fs = (double)rate;
    double duration_s = (double)n_pcm / fs;
    if (sig_hi <= sig_lo || noise_hi <= noise_lo
        || sig_hi >= 0.5 * fs || noise_hi >= 0.5 * fs) {
        fprintf(stderr,
                "beacon_detect: invalid bands (sig=[%g,%g] noise=[%g,%g] fs=%g)\n",
                sig_lo, sig_hi, noise_lo, noise_hi, fs);
        free(pcm);
        return EXIT_FAILURE;
    }
    if (env_ms < 1 || env_ms > 1000) {
        fprintf(stderr, "beacon_detect: --env-ms must be 1..1000\n");
        free(pcm);
        return EXIT_FAILURE;
    }

    if (!quiet) {
        fprintf(stderr,
                "beacon_detect: %s  rate=%d  ch=%d  duration=%.2f s\n"
                "  sig_band=[%.0f,%.0f] noise_band=[%.0f,%.0f]\n"
                "  env_ms=%d  threshold=%.2fσ  min_spacing=%.2fs  "
                "lattice=%.2fs±%.2fs\n",
                wav_path, rate, ch, duration_s,
                sig_lo, sig_hi, noise_lo, noise_hi,
                env_ms, threshold_sigma, min_spacing_s,
                lattice_period_s, lattice_tol_s);
    }

    double *sig_sq   = malloc(n_pcm * sizeof(double));
    double *noise_sq = malloc(n_pcm * sizeof(double));
    if (sig_sq == NULL || noise_sq == NULL) {
        fprintf(stderr, "beacon_detect: out of memory (need %zu MB scratch)\n",
                (n_pcm * sizeof(double) * 2) / (1024 * 1024));
        free(pcm); free(sig_sq); free(noise_sq);
        return EXIT_FAILURE;
    }

    double sig_f0   = 0.5 * (sig_lo + sig_hi);
    double sig_bw   = sig_hi - sig_lo;
    double noise_f0 = 0.5 * (noise_lo + noise_hi);
    double noise_bw = noise_hi - noise_lo;
    run_bp_squared(pcm, n_pcm, sig_f0,   sig_bw,   fs, sig_sq);
    run_bp_squared(pcm, n_pcm, noise_f0, noise_bw, fs, noise_sq);

    size_t win_samples = (size_t)((double)env_ms / 1000.0 * fs);
    if (win_samples < 1) win_samples = 1;
    size_t n_env = n_pcm / win_samples;
    double *sig_env   = malloc(n_env * sizeof(double));
    double *noise_env = malloc(n_env * sizeof(double));
    double *ratio_db  = malloc(n_env * sizeof(double));
    if (sig_env == NULL || noise_env == NULL || ratio_db == NULL) {
        fprintf(stderr, "beacon_detect: out of memory\n");
        free(pcm); free(sig_sq); free(noise_sq);
        free(sig_env); free(noise_env); free(ratio_db);
        return EXIT_FAILURE;
    }
    rms_envelope(sig_sq,   n_pcm, win_samples, sig_env);
    rms_envelope(noise_sq, n_pcm, win_samples, noise_env);
    free(sig_sq); free(noise_sq);

    for (size_t i = 0; i < n_env; i++) {
        double s = sqrt(sig_env[i]);
        double n = sqrt(noise_env[i]);
        if (n < 1e-3) n = 1e-3;
        double r = s / n;
        if (r < 1e-3) r = 1e-3;
        ratio_db[i] = 20.0 * log10(r);
    }

    double med, mad;
    if (median_mad(ratio_db, n_env, &med, &mad) != 0) {
        fprintf(stderr, "beacon_detect: median computation failed\n");
        free(pcm); free(sig_env); free(noise_env); free(ratio_db);
        return EXIT_FAILURE;
    }
    double threshold = med + threshold_sigma * 1.4826 * mad;
    double cell_s    = (double)win_samples / fs;
    size_t min_dist  = (size_t)(min_spacing_s / cell_s);
    if (min_dist < 1) min_dist = 1;

    if (!quiet) {
        fprintf(stderr,
                "  ratio_db: median=%+.2f  MAD=%.2f  threshold=%+.2f  "
                "(env cells=%zu  cell=%.3fs)\n",
                med, mad, threshold, n_env, cell_s);
    }

    size_t *peak_idx = malloc(n_env * sizeof(size_t));
    if (peak_idx == NULL) {
        fprintf(stderr, "beacon_detect: out of memory\n");
        free(pcm); free(sig_env); free(noise_env); free(ratio_db);
        return EXIT_FAILURE;
    }
    size_t n_peaks = find_peaks(ratio_db, n_env, threshold, min_dist,
                                peak_idx, n_env);
    if (!quiet) {
        fprintf(stderr, "  raw detections: %zu\n", n_peaks);
    }

    // Carrier-suppression filter. For each peak, walk left then right
    // and count consecutive cells with ratio_db > threshold. If the
    // run length is longer than max_burst_s, the peak is inside a
    // sustained carrier dwell rather than a brief packet — drop it.
    // Mirrors the carrier-lockout in monitor_squelch.
    if (max_burst_s > 0.0 && n_peaks > 0) {
        size_t max_burst_cells = (size_t)(max_burst_s / cell_s);
        if (max_burst_cells < 1) max_burst_cells = 1;
        size_t kept = 0;
        for (size_t i = 0; i < n_peaks; ++i) {
            size_t k = peak_idx[i];
            // Walk left while still above threshold.
            size_t run = 1;
            size_t j = k;
            while (j > 0 && ratio_db[j - 1] > threshold && run <= max_burst_cells + 1) {
                --j; ++run;
            }
            j = k;
            while (j + 1 < n_env && ratio_db[j + 1] > threshold && run <= max_burst_cells + 1) {
                ++j; ++run;
            }
            if (run <= max_burst_cells) {
                peak_idx[kept++] = k;
            }
        }
        if (!quiet) {
            fprintf(stderr,
                "  after max-burst filter (%.2fs): %zu (dropped %zu)\n",
                max_burst_s, kept, n_peaks - kept);
        }
        n_peaks = kept;
    }

    n_peaks = lattice_filter(peak_idx, n_peaks, ratio_db,
                             cell_s, lattice_period_s, lattice_tol_s);
    if (!quiet) {
        fprintf(stderr, "  after lattice filter: %zu\n", n_peaks);
    }

    FILE *csv = stdout;
    if (csv_path != NULL) {
        csv = fopen(csv_path, "w");
        if (csv == NULL) {
            fprintf(stderr, "beacon_detect: fopen(%s): %s\n",
                    csv_path, strerror(errno));
            free(pcm); free(sig_env); free(noise_env); free(ratio_db);
            free(peak_idx);
            return EXIT_FAILURE;
        }
    }
    fprintf(csv,
            "# beacon_detect on %s rate=%d duration_s=%.3f "
            "sig=[%g,%g] noise=[%g,%g] env_ms=%d "
            "threshold_sigma=%.2f lattice_period_s=%.2f lattice_tol_s=%.2f\n",
            wav_path, rate, duration_s,
            sig_lo, sig_hi, noise_lo, noise_hi, env_ms,
            threshold_sigma, lattice_period_s, lattice_tol_s);
    fprintf(csv, "idx,t_s,sig_rms,noise_rms,ratio_db\n");
    for (size_t i = 0; i < n_peaks; i++) {
        size_t k = peak_idx[i];
        double t = (double)k * cell_s;
        double s = sqrt(sig_env[k]);
        double n = sqrt(noise_env[k]);
        fprintf(csv, "%zu,%.3f,%.1f,%.1f,%+.2f\n",
                i, t, s, n, ratio_db[k]);
    }
    if (csv != stdout) fclose(csv);

    if (render_png && n_peaks > 0) {
        char default_dir[512];
        if (render_dir == NULL) {
            // <wav_dir>/beacons by default.
            const char *slash = strrchr(wav_path, '/');
            if (slash == NULL) {
                snprintf(default_dir, sizeof default_dir, "beacons");
            } else {
                size_t dirlen = (size_t)(slash - wav_path);
                if (dirlen >= sizeof default_dir - 16) dirlen = sizeof default_dir - 16;
                memcpy(default_dir, wav_path, dirlen);
                default_dir[dirlen] = '\0';
                strncat(default_dir, "/beacons", sizeof default_dir - strlen(default_dir) - 1);
            }
            render_dir = default_dir;
        }
        if (mkdir(render_dir, 0755) != 0 && errno != EEXIST) {
            fprintf(stderr, "beacon_detect: mkdir(%s): %s — skipping renders\n",
                    render_dir, strerror(errno));
        } else {
            for (size_t i = 0; i < n_peaks; i++) {
                size_t k = peak_idx[i];
                double t = (double)k * cell_s;
                double t0 = t - render_half_s;
                if (t0 < 0.0) t0 = 0.0;
                double dur = 2.0 * render_half_s;
                char cmd[1024];
                snprintf(cmd, sizeof cmd,
                         "ffmpeg -y -loglevel error -ss %.3f -t %.3f "
                         "-i \"%s\" -lavfi "
                         "\"showspectrumpic=s=1280x720:mode=combined:"
                         "color=intensity:legend=1\" "
                         "\"%s/beacon_%02zu_t%07.3fs.png\"",
                         t0, dur, wav_path, render_dir, i, t);
                int rc = system(cmd);
                if (!quiet) {
                    fprintf(stderr,
                            "  render #%02zu t=%.2fs → %s/beacon_%02zu_t%07.3fs.png  rc=%d\n",
                            i, t, render_dir, i, t, rc);
                }
            }
        }
    }

    if (!quiet) {
        fprintf(stderr, "beacon_detect: %zu beacons emitted to %s\n",
                n_peaks, csv_path ? csv_path : "(stdout)");
    }

    if (gate_wav_path != NULL) {
        if (!quiet) {
            fprintf(stderr, "beacon_detect: gating %s through monitor_squelch\n",
                    wav_path);
        }
        monitor_squelch_t sq;
        monitor_squelch_params_t mp = {
            .rate_hz        = fs,
            .auto_offset_db = gate_offset_db,
            .hold_s         = gate_hold_s,
            .init_mode      = gate_fixed_set ? MSQ_FIXED : MSQ_AUTO_BOOTSTRAPPING,
            .init_thresh_db = gate_fixed_db,
        };
        monitor_squelch_init(&sq, &mp);
        int16_t *gated = malloc(n_pcm * sizeof(int16_t));
        if (gated == NULL) {
            fprintf(stderr, "beacon_detect: out of memory for gate buffer\n");
            free(pcm); free(sig_env); free(noise_env); free(ratio_db); free(peak_idx);
            return EXIT_FAILURE;
        }
        monitor_squelch_process(&sq, pcm, gated, n_pcm);
        if (write_wav_pcm16_mono(gate_wav_path, gated, n_pcm, rate) != 0) {
            fprintf(stderr, "beacon_detect: write %s: %s\n",
                    gate_wav_path, strerror(errno));
            free(gated);
            free(pcm); free(sig_env); free(noise_env); free(ratio_db); free(peak_idx);
            return EXIT_FAILURE;
        }
        if (!quiet) {
            char sqdesc[128];
            monitor_squelch_status(&sq, sqdesc, sizeof sqdesc);
            // Count audible samples for a quick "duty cycle" hint.
            size_t audible = 0;
            for (size_t i = 0; i < n_pcm; i++) if (gated[i] != 0) audible++;
            fprintf(stderr,
                    "  squelch final state: %s\n"
                    "  audible samples: %zu of %zu (%.2f%% of pass)\n"
                    "  wrote %s\n",
                    sqdesc, audible, n_pcm,
                    100.0 * (double)audible / (double)n_pcm,
                    gate_wav_path);
        }
        free(gated);
    }

    free(pcm); free(sig_env); free(noise_env); free(ratio_db); free(peak_idx);
    return 0;
}
