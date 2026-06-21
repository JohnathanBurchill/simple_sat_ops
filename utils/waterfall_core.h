#ifndef WATERFALL_CORE_H
#define WATERFALL_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// Knobs and outputs for the spectrogram pipeline. Caller fills the
// inputs; wf_compute fills display_bw_hz, display_db_floor,
// display_db_lo, display_db_hi.
typedef struct wf_opts {
    int    fft_size;
    int    hop;
    int    out_rows;
    float  db_min;
    float  db_max;
    int    db_min_user_set;
    int    db_max_user_set;
    // 0 = median (whole-pass per-column median subtraction)
    // 1 = hpf (zero-phase 1-pole HPF along time)
    // 2 = none
    int    detrend_mode;
    double detrend_tau_s;
    const char *marks_csv_path;
    const char *show_tm_csv_path;
    int    sample_rate;
    double center_hz;
    double zoom_hz;
    int    dc_notch;
    int    dc_notch_bins;
    double display_bw_hz;
    time_t start_utc;
    double start_utc_subsec;
    float  display_db_lo;
    float  display_db_hi;
    float  display_db_floor;
    float  power_offset_db;
    char   power_unit[8];
    // Optional progress hook. If non-NULL, wf_compute writes a 0..1
    // fraction to *progress_pct_out during the heavy FFT frame loop so
    // a UI thread can render a progress bar. Caller is responsible for
    // making the read side tolerant of cross-thread access (e.g.
    // `volatile float` or atomic).
    volatile float *progress_pct_out;
} wf_opts_t;

// 256-entry viridis colormap (R,G,B in 0..255). Used by gen_waterfall to
// bake pixels into a PNG and by decode_inspector to build a GPU lookup
// texture for live recolouring.
extern const uint8_t WF_VIRIDIS[256][3];

// Shared radix-2 FFT primitives (also used by live_waterfall). wf_is_pow2
// reports whether n is a power of two; wf_fft_forward does an in-place
// forward DFT on the length-n (power-of-two) re/im arrays.
int  wf_is_pow2(unsigned n);
void wf_fft_forward(float *re, float *im, unsigned n);

// Compute the spectrogram dB grid.
// - iq: interleaved int16 I,Q pairs (length 2 * n_pairs samples).
// - n_pairs: number of IQ pairs.
// - opt: pipeline knobs (mutated to record display_bw_hz, display_db_floor,
//   and the dB range the auto-clip / user-override settled on).
// - out_db: receives a freshly malloc'd float array of (*out_w * *out_h)
//   cells in row-major order. Row 0 = earliest time, column 0 = lowest
//   displayed frequency (after fftshift + zoom crop). Caller frees.
//   Values are in MEDIAN-SUBTRACTED dB space (each column's noise floor
//   sits near 0 dB); add display_db_floor + power_offset_db to map back
//   to absolute dBFS/dBm.
// Returns 0 on success, non-zero on failure.
int wf_compute(const int16_t *iq, size_t n_pairs,
               wf_opts_t *opt,
               float **out_db, int *out_w, int *out_h);

// Recompute auto-detected dB range (5th and 99th percentiles) from a
// previously-computed grid. Used by decode_inspector when the operator
// presses "R" to reset — no need to redo the FFT.
void wf_auto_db_range(const float *db, int w, int h,
                      float *out_lo, float *out_hi);

// Axis tick / label helpers, shared by gen_waterfall (which bakes them
// into a PNG) and decode_inspector (which draws them live via raylib).
double wf_pick_tick_step(double range, int target_ticks);
double wf_pick_time_step(double range_s, int target_ticks);
int    wf_fmt_freq(double hz, char *out, size_t out_cap);
int    wf_fmt_time(time_t base_utc, double sec, char *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif
