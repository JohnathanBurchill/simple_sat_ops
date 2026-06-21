// utils/waterfall_core.c — shared FFT / dB / detrend / notch / zoom
// pipeline, viridis colormap, and axis-tick helpers.
//
// Pulled out of utils/gen_waterfall.c so decode_inspector can compute the
// spectrogram in-process (no subprocess, no /tmp PNG) and re-colour it
// on the GPU. gen_waterfall keeps doing its bitmap-axes + PNG/PDF
// rendering on top of wf_compute's float output.

#include "waterfall_core.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------------------------------------------------------------------
// Iterative radix-2 FFT (in-place).
// --------------------------------------------------------------------

int wf_is_pow2(unsigned n) { return n > 0 && (n & (n - 1)) == 0; }

static void fft_bit_reverse(float *re, float *im, unsigned n)
{
    unsigned j = 0;
    for (unsigned i = 1; i < n; ++i) {
        unsigned bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
}

void wf_fft_forward(float *re, float *im, unsigned n)
{
    fft_bit_reverse(re, im, n);
    for (unsigned len = 2; len <= n; len <<= 1) {
        double ang = -2.0 * M_PI / (double) len;
        double wr_step = cos(ang);
        double wi_step = sin(ang);
        unsigned half = len >> 1;
        for (unsigned i = 0; i < n; i += len) {
            double wr = 1.0, wi = 0.0;
            for (unsigned k = 0; k < half; ++k) {
                unsigned a = i + k;
                unsigned b = a + half;
                double tr = wr * re[b] - wi * im[b];
                double ti = wr * im[b] + wi * re[b];
                re[b] = (float)(re[a] - tr);
                im[b] = (float)(im[a] - ti);
                re[a] = (float)(re[a] + tr);
                im[a] = (float)(im[a] + ti);
                double nwr = wr * wr_step - wi * wi_step;
                double nwi = wr * wi_step + wi * wr_step;
                wr = nwr;
                wi = nwi;
            }
        }
    }
}

static float median_inplace(float *buf, int n)
{
    // Quickselect-style nth_element. Mutates buf.
    if (n <= 0) return 0.0f;
    int lo = 0, hi = n - 1, k = n / 2;
    while (lo < hi) {
        float pivot = buf[(lo + hi) / 2];
        int i = lo, j = hi;
        while (i <= j) {
            while (buf[i] < pivot) ++i;
            while (buf[j] > pivot) --j;
            if (i <= j) {
                float t = buf[i]; buf[i] = buf[j]; buf[j] = t;
                ++i; --j;
            }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else return buf[k];
    }
    return buf[k];
}

// --------------------------------------------------------------------
// Viridis colormap (matplotlib's 256-entry table).
// --------------------------------------------------------------------

const uint8_t WF_VIRIDIS[256][3] = {
    { 68,  1, 84},{ 68,  2, 86},{ 69,  4, 87},{ 69,  5, 89},{ 70,  7, 90},
    { 70,  8, 92},{ 70, 10, 93},{ 70, 11, 94},{ 71, 13, 96},{ 71, 14, 97},
    { 71, 16, 99},{ 71, 17,100},{ 71, 19,101},{ 72, 20,103},{ 72, 22,104},
    { 72, 23,105},{ 72, 24,106},{ 72, 26,108},{ 72, 27,109},{ 72, 28,110},
    { 72, 29,111},{ 72, 31,112},{ 72, 32,113},{ 72, 33,115},{ 72, 35,116},
    { 72, 36,117},{ 72, 37,118},{ 72, 38,119},{ 72, 40,120},{ 72, 41,121},
    { 71, 42,122},{ 71, 44,122},{ 71, 45,123},{ 71, 46,124},{ 71, 47,125},
    { 70, 48,126},{ 70, 50,126},{ 70, 51,127},{ 70, 52,128},{ 69, 53,129},
    { 69, 55,129},{ 69, 56,130},{ 68, 57,131},{ 68, 58,131},{ 68, 59,132},
    { 67, 61,132},{ 67, 62,133},{ 66, 63,133},{ 66, 64,134},{ 66, 65,134},
    { 65, 66,135},{ 65, 68,135},{ 64, 69,136},{ 64, 70,136},{ 63, 71,136},
    { 63, 72,137},{ 62, 73,137},{ 62, 74,137},{ 62, 76,138},{ 61, 77,138},
    { 61, 78,138},{ 60, 79,138},{ 60, 80,139},{ 59, 81,139},{ 59, 82,139},
    { 58, 83,139},{ 58, 84,140},{ 57, 85,140},{ 57, 86,140},{ 56, 88,140},
    { 56, 89,140},{ 55, 90,140},{ 55, 91,141},{ 54, 92,141},{ 54, 93,141},
    { 53, 94,141},{ 53, 95,141},{ 52, 96,141},{ 52, 97,141},{ 51, 98,141},
    { 51, 99,141},{ 50,100,142},{ 50,101,142},{ 49,102,142},{ 49,103,142},
    { 49,104,142},{ 48,105,142},{ 48,106,142},{ 47,107,142},{ 47,108,142},
    { 46,109,142},{ 46,110,142},{ 46,111,142},{ 45,112,142},{ 45,113,142},
    { 44,113,142},{ 44,114,142},{ 44,115,142},{ 43,116,142},{ 43,117,142},
    { 42,118,142},{ 42,119,142},{ 42,120,142},{ 41,121,142},{ 41,122,142},
    { 41,123,142},{ 40,124,142},{ 40,125,142},{ 39,126,142},{ 39,127,142},
    { 39,128,142},{ 38,129,142},{ 38,130,142},{ 38,130,142},{ 37,131,142},
    { 37,132,142},{ 37,133,142},{ 36,134,142},{ 36,135,142},{ 35,136,142},
    { 35,137,142},{ 35,138,141},{ 34,139,141},{ 34,140,141},{ 34,141,141},
    { 33,142,141},{ 33,143,141},{ 33,144,141},{ 33,145,140},{ 32,146,140},
    { 32,146,140},{ 32,147,140},{ 31,148,140},{ 31,149,139},{ 31,150,139},
    { 31,151,139},{ 31,152,139},{ 31,153,138},{ 31,154,138},{ 30,155,138},
    { 30,156,137},{ 30,157,137},{ 31,158,137},{ 31,159,136},{ 31,160,136},
    { 31,161,136},{ 31,161,135},{ 31,162,135},{ 32,163,134},{ 32,164,134},
    { 33,165,133},{ 33,166,133},{ 34,167,133},{ 34,168,132},{ 35,169,131},
    { 36,170,131},{ 37,171,130},{ 37,172,130},{ 38,173,129},{ 39,173,129},
    { 40,174,128},{ 41,175,127},{ 42,176,127},{ 44,177,126},{ 45,178,125},
    { 46,179,124},{ 47,180,124},{ 49,181,123},{ 50,182,122},{ 52,182,121},
    { 53,183,121},{ 55,184,120},{ 56,185,119},{ 58,186,118},{ 59,187,117},
    { 61,188,116},{ 63,188,115},{ 64,189,114},{ 66,190,113},{ 68,191,112},
    { 70,192,111},{ 72,193,110},{ 74,193,109},{ 76,194,108},{ 78,195,107},
    { 80,196,106},{ 82,197,105},{ 84,197,104},{ 86,198,103},{ 88,199,101},
    { 90,200,100},{ 92,200, 99},{ 94,201, 98},{ 96,202, 96},{ 99,203, 95},
    {101,203, 94},{103,204, 92},{105,205, 91},{108,205, 90},{110,206, 88},
    {112,207, 87},{115,208, 86},{117,208, 84},{119,209, 83},{122,209, 81},
    {124,210, 80},{127,211, 78},{129,211, 77},{132,212, 75},{134,213, 73},
    {137,213, 72},{139,214, 70},{142,214, 69},{144,215, 67},{147,215, 65},
    {149,216, 64},{152,216, 62},{155,217, 60},{157,217, 59},{160,218, 57},
    {162,218, 55},{165,219, 54},{168,219, 52},{170,220, 50},{173,220, 48},
    {176,221, 47},{178,221, 45},{181,222, 43},{184,222, 41},{186,222, 40},
    {189,223, 38},{192,223, 37},{194,223, 35},{197,224, 33},{200,224, 32},
    {202,225, 31},{205,225, 29},{208,225, 28},{210,226, 27},{213,226, 26},
    {216,226, 25},{218,227, 25},{221,227, 24},{223,227, 24},{226,228, 24},
    {229,228, 25},{231,228, 25},{234,229, 26},{236,229, 27},{239,229, 28},
    {241,229, 29},{244,230, 30},{246,230, 32},{248,230, 33},{251,231, 35},
    {253,231, 37}
};

// --------------------------------------------------------------------
// Auto-detect dB range via percentile sampling.
// --------------------------------------------------------------------

void wf_auto_db_range(const float *db, int w, int h,
                      float *out_lo, float *out_hi)
{
    if (out_lo == NULL || out_hi == NULL || db == NULL || w <= 0 || h <= 0) {
        if (out_lo) *out_lo = -100.0f;
        if (out_hi) *out_hi =    0.0f;
        return;
    }
    size_t n_cells = (size_t) w * (size_t) h;
    size_t sample_n = n_cells / 64;
    if (sample_n < 4096)   sample_n = 4096;
    if (sample_n > 200000) sample_n = 200000;
    if (sample_n > n_cells) sample_n = n_cells;

    float *sample = (float *) malloc(sample_n * sizeof(float));
    if (sample == NULL) {
        *out_lo = -100.0f;
        *out_hi =    0.0f;
        return;
    }
    uint32_t state = 0xC0FFEEu;
    for (size_t i = 0; i < sample_n; ++i) {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        sample[i] = db[(size_t) state % n_cells];
    }
    float p05 = median_inplace(sample, (int) sample_n);

    // Fresh draw for 99th percentile (median_inplace mutated `sample`).
    for (size_t i = 0; i < sample_n; ++i) {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        sample[i] = db[(size_t) state % n_cells];
    }
    size_t k99 = (size_t)((double) sample_n * 0.99);
    if (k99 >= sample_n) k99 = sample_n - 1;
    {
        int lo_i = 0, hi_i = (int) sample_n - 1;
        while (lo_i < hi_i) {
            float pivot = sample[(lo_i + hi_i) / 2];
            int i = lo_i, j = hi_i;
            while (i <= j) {
                while (sample[i] < pivot) ++i;
                while (sample[j] > pivot) --j;
                if (i <= j) {
                    float t = sample[i]; sample[i] = sample[j]; sample[j] = t;
                    ++i; --j;
                }
            }
            if ((int) k99 <= j) hi_i = j;
            else if ((int) k99 >= i) lo_i = i;
            else break;
        }
    }
    float p99 = sample[k99];
    free(sample);

    float lo = p05 - 1.0f;
    float hi = p99;
    if (hi < lo + 12.0f) hi = lo + 12.0f;
    *out_lo = lo;
    *out_hi = hi;
}

// --------------------------------------------------------------------
// wf_compute — the main FFT + dB + detrend + notch + zoom pipeline.
// --------------------------------------------------------------------

int wf_compute(const int16_t *iq, size_t n_pairs,
               wf_opts_t *opt,
               float **out_db, int *out_w, int *out_h)
{
    if (iq == NULL || opt == NULL || out_db == NULL ||
        out_w == NULL || out_h == NULL) {
        return -1;
    }
    int N = opt->fft_size;
    int H = opt->hop;
    if (!wf_is_pow2((unsigned) N) || N < 16 || H <= 0 || H > N) {
        fprintf(stderr, "waterfall_core: invalid fft/hop\n");
        return -1;
    }
    if (n_pairs < (size_t) N) {
        fprintf(stderr, "waterfall_core: only %zu IQ pairs, need >= %d\n",
                n_pairs, N);
        return -1;
    }

    // Hann window.
    float *win = (float *) malloc((size_t) N * sizeof(float));
    if (!win) return -1;
    for (int i = 0; i < N; ++i) {
        win[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * i / (N - 1)));
    }

    size_t n_frames = (n_pairs - (size_t) N) / (size_t) H + 1;
    if (n_frames < 1) { free(win); return -1; }

    // Linear-power spectrogram, before time-binning.
    float *spec = (float *) malloc(n_frames * (size_t) N * sizeof(float));
    if (!spec) { free(win); return -1; }
    float *re = (float *) malloc((size_t) N * sizeof(float));
    float *im = (float *) malloc((size_t) N * sizeof(float));
    if (!re || !im) { free(win); free(spec); free(re); free(im); return -1; }

    // Throttle progress writes so the UI thread isn't flooded with
    // pointer stores (every ~1 % of the FFT loop is plenty).
    size_t progress_step = (n_frames >= 100) ? n_frames / 100 : 1;
    for (size_t fi = 0; fi < n_frames; ++fi) {
        size_t base = fi * (size_t) H;
        for (int i = 0; i < N; ++i) {
            float w = win[i];
            float I = (float) iq[(base + (size_t) i) * 2 + 0] / 32768.0f;
            float Q = (float) iq[(base + (size_t) i) * 2 + 1] / 32768.0f;
            re[i] = I * w;
            im[i] = Q * w;
        }
        wf_fft_forward(re, im, (unsigned) N);
        // fftshift: output bin 0 = -Fs/2, bin N/2 = DC, bin N-1 = +Fs/2-bin.
        for (int k = 0; k < N; ++k) {
            int src = (k + N / 2) % N;
            float mag2 = re[src] * re[src] + im[src] * im[src];
            spec[fi * (size_t) N + (size_t) k] = mag2;
        }
        if (opt->progress_pct_out != NULL
            && (fi % progress_step) == 0)
        {
            *opt->progress_pct_out =
                (float)((double)(fi + 1) / (double) n_frames);
        }
    }
    if (opt->progress_pct_out != NULL) {
        *opt->progress_pct_out = 1.0f;
    }
    free(win); free(re); free(im);

    // Time-bin frames into output rows; one log at the very end per cell.
    int out_rows = opt->out_rows;
    if (out_rows <= 0) out_rows = 1080;
    if ((size_t) out_rows > n_frames) out_rows = (int) n_frames;
    float *binned = (float *) malloc((size_t) out_rows * (size_t) N * sizeof(float));
    if (!binned) { free(spec); return -1; }
    for (int r = 0; r < out_rows; ++r) {
        size_t a = (size_t) r * n_frames / (size_t) out_rows;
        size_t b = (size_t)(r + 1) * n_frames / (size_t) out_rows;
        if (b <= a) b = a + 1;
        if (b > n_frames) b = n_frames;
        size_t span = b - a;
        double inv = 1.0 / (double) span;
        for (int k = 0; k < N; ++k) {
            double sum = 0.0;
            for (size_t fi = a; fi < b; ++fi) {
                sum += spec[fi * (size_t) N + (size_t) k];
            }
            binned[(size_t) r * (size_t) N + (size_t) k] =
                (float)(10.0 * log10(sum * inv + 1e-20));
        }
    }
    free(spec);

    // Per-column detrend so the noise floor flattens out.
    float *col = (float *) malloc((size_t) out_rows * sizeof(float));
    float *bin_medians = (float *) malloc((size_t) N * sizeof(float));
    if (!col || !bin_medians) {
        free(col); free(bin_medians); free(binned); return -1;
    }
    double duration_s = (double) n_pairs / (double) opt->sample_rate;
    double row_dt_s   = (out_rows > 0) ? (duration_s / (double) out_rows) : 1.0;
    double hpf_alpha  = (opt->detrend_mode == 1 && opt->detrend_tau_s > 0.0)
                       ? exp(-row_dt_s / opt->detrend_tau_s) : 0.0;

    for (int k = 0; k < N; ++k) {
        for (int r = 0; r < out_rows; ++r) {
            col[r] = binned[(size_t) r * (size_t) N + (size_t) k];
        }
        bin_medians[k] = median_inplace(col, out_rows);

        if (opt->detrend_mode == 1) {
            // HPF: zero-phase 1-pole LPF (forward+backward), then subtract.
            for (int r = 0; r < out_rows; ++r) {
                col[r] = binned[(size_t) r * (size_t) N + (size_t) k];
            }
            double a = hpf_alpha;
            double lp = (double) col[0];
            for (int r = 0; r < out_rows; ++r) {
                lp = a * lp + (1.0 - a) * (double) col[r];
                col[r] = (float) lp;
            }
            lp = (double) col[out_rows - 1];
            for (int r = out_rows - 1; r >= 0; --r) {
                lp = a * lp + (1.0 - a) * (double) col[r];
                col[r] = (float) lp;
            }
            for (int r = 0; r < out_rows; ++r) {
                binned[(size_t) r * (size_t) N + (size_t) k] -= col[r];
            }
        } else if (opt->detrend_mode == 0) {
            float med = bin_medians[k];
            for (int r = 0; r < out_rows; ++r) {
                binned[(size_t) r * (size_t) N + (size_t) k] -= med;
            }
        }
        // detrend_mode == 2 (none): cells unchanged.
    }
    float floor_raw_db = median_inplace(bin_medians, N);
    float fft_scale_db = 20.0f * (float) log10((double) opt->fft_size / 2.0);
    if (opt->detrend_mode == 2) {
        opt->display_db_floor = -fft_scale_db;
    } else {
        opt->display_db_floor = floor_raw_db - fft_scale_db;
    }
    free(bin_medians);
    free(col);

    // DC notch (B210 LO bleed).
    int dc_k = N / 2;
    int notch = opt->dc_notch_bins > 0 ? opt->dc_notch_bins : 2;
    if (opt->dc_notch && dc_k - notch - 1 >= 0 && dc_k + notch + 1 < N) {
        for (int r = 0; r < out_rows; ++r) {
            float left  = binned[(size_t) r * (size_t) N + (size_t)(dc_k - notch - 1)];
            float right = binned[(size_t) r * (size_t) N + (size_t)(dc_k + notch + 1)];
            float fill  = 0.5f * (left + right);
            for (int off = -notch; off <= notch; ++off) {
                binned[(size_t) r * (size_t) N + (size_t)(dc_k + off)] = fill;
            }
        }
    }

    // Zoom: keep columns inside ±zoom_hz/2 around DC.
    int k_lo = 0;
    int k_hi = N - 1;
    double display_bw = (double) opt->sample_rate;
    if (opt->zoom_hz > 0.0 && opt->zoom_hz < (double) opt->sample_rate) {
        double half = opt->zoom_hz / 2.0;
        int half_bins = (int)(half / (double) opt->sample_rate * (double) N);
        if (half_bins < 1) half_bins = 1;
        if (half_bins > N / 2 - 1) half_bins = N / 2 - 1;
        k_lo = dc_k - half_bins;
        k_hi = dc_k + half_bins;
        display_bw = 2.0 * half_bins * (double) opt->sample_rate / (double) N;
    }
    opt->display_bw_hz = display_bw;
    int zoom_N = k_hi - k_lo + 1;
    if (zoom_N != N) {
        float *cropped = (float *) malloc((size_t) out_rows * (size_t) zoom_N
                                          * sizeof(float));
        if (!cropped) { free(binned); return -1; }
        for (int r = 0; r < out_rows; ++r) {
            memcpy(cropped + (size_t) r * (size_t) zoom_N,
                   binned + (size_t) r * (size_t) N + (size_t) k_lo,
                   (size_t) zoom_N * sizeof(float));
        }
        free(binned);
        binned = cropped;
        N = zoom_N;
    }

    // Auto-clip dB range to data percentiles for any endpoint the caller
    // didn't pin; user overrides land in absolute dBFS and are mapped
    // back into the median-subtracted space the grid actually carries.
    float lo = 0.0f, hi = 0.0f;
    int need_auto = !opt->db_min_user_set || !opt->db_max_user_set;
    if (need_auto) {
        wf_auto_db_range(binned, N, out_rows, &lo, &hi);
    }
    if (opt->db_min_user_set) {
        lo = opt->db_min - opt->display_db_floor - opt->power_offset_db;
    }
    if (opt->db_max_user_set) {
        hi = opt->db_max - opt->display_db_floor - opt->power_offset_db;
    }
    opt->display_db_lo = lo;
    opt->display_db_hi = hi;

    *out_db = binned;
    *out_w  = N;
    *out_h  = out_rows;
    return 0;
}

// --------------------------------------------------------------------
// Axis helpers (shared between gen_waterfall's PNG axes and
// decode_inspector's live raylib axes).
// --------------------------------------------------------------------

double wf_pick_tick_step(double range, int target_ticks)
{
    if (range <= 0.0 || target_ticks < 1) return 1.0;
    double raw = range / (double) target_ticks;
    double p = pow(10.0, floor(log10(raw)));
    double frac = raw / p;
    double mul;
    if      (frac < 1.5) mul = 1.0;
    else if (frac < 3.5) mul = 2.0;
    else if (frac < 7.0) mul = 5.0;
    else                 mul = 10.0;
    return p * mul;
}

double wf_pick_time_step(double range_s, int target_ticks)
{
    static const double steps[] = {
        1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600, 7200, 14400
    };
    int n = (int)(sizeof steps / sizeof steps[0]);
    if (range_s <= 0.0 || target_ticks < 1) return 1.0;
    double target = range_s / (double) target_ticks;
    double best = steps[0];
    double best_d = fabs(steps[0] - target);
    for (int i = 1; i < n; ++i) {
        double d = fabs(steps[i] - target);
        if (d < best_d) { best_d = d; best = steps[i]; }
    }
    return best;
}

int wf_fmt_freq(double hz, char *out, size_t out_cap)
{
    double abs_hz = (hz < 0.0) ? -hz : hz;
    const char *sgn = (hz < 0.0) ? "-" : "+";
    if (abs_hz >= 1.0e6 - 1.0) {
        return snprintf(out, out_cap, "%s%.2fMHz", sgn, abs_hz / 1.0e6);
    } else {
        return snprintf(out, out_cap, "%s%.1fkHz", sgn, abs_hz / 1.0e3);
    }
}

int wf_fmt_time(time_t base_utc, double sec, char *out, size_t out_cap)
{
    if (sec < 0.0) sec = 0.0;
    if (base_utc != 0) {
        time_t t = base_utc + (time_t)(sec + 0.5);
        struct tm lt;
        localtime_r(&t, &lt);
        return snprintf(out, out_cap, "%02d:%02d:%02d",
                        lt.tm_hour, lt.tm_min, lt.tm_sec);
    }
    int total = (int)(sec + 0.5);
    int h = total / 3600;
    int m = (total / 60) % 60;
    int s = total % 60;
    return snprintf(out, out_cap, "%02d:%02d:%02d", h, m, s);
}
