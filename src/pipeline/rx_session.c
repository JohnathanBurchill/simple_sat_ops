// rx_session.c — see header. Lifted from utils/b210_rx_tx.c (the
// daemon-mode receiver) when simple_sat_ops absorbed B210 ownership.

#include "rx_session.h"

#include "ax100.h"
#include "b210_rx_tx_core.h"
#include "beacon_cts1.h"
#include "csp.h"
#include "decode_loop.h"
#include "modem.h"
#include "modem_iq.h"
#include "packet_db.h"
#include "tx_burst.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

// Streaming WAV writer (mono int16). The header's data/RIFF sizes are
// patched at close; an interrupted run leaves a "0-length" but otherwise
// playable file that the receiving tools handle.
typedef struct {
    FILE  *fp;
    size_t n_samples;
    int    sample_rate;
} wav_w_t;

static int wav_w_open(wav_w_t *w, const char *path, int sample_rate)
{
    memset(w, 0, sizeof *w);
    w->fp = fopen(path, "wb");
    if (w->fp == NULL) return -1;
    uint32_t sr  = (uint32_t) sample_rate;
    uint32_t bps = sr * 2;
    uint8_t hdr[44] = {
        'R','I','F','F', 0,0,0,0, 'W','A','V','E',
        'f','m','t',' ', 16,0,0,0, 1,0, 1,0,
        (uint8_t)(sr      & 0xFF), (uint8_t)((sr >> 8)  & 0xFF),
        (uint8_t)((sr>>16)& 0xFF), (uint8_t)((sr >> 24) & 0xFF),
        (uint8_t)(bps     & 0xFF), (uint8_t)((bps >> 8) & 0xFF),
        (uint8_t)((bps>>16)& 0xFF),(uint8_t)((bps >> 24) & 0xFF),
        2,0, 16,0,
        'd','a','t','a', 0,0,0,0,
    };
    if (fwrite(hdr, 1, 44, w->fp) != 44) { fclose(w->fp); w->fp = NULL; return -1; }
    w->sample_rate = sample_rate;
    return 0;
}

static void wav_w_append(wav_w_t *w, const int16_t *s, size_t n)
{
    if (w->fp == NULL || n == 0) return;
    if (fwrite(s, sizeof(int16_t), n, w->fp) == n) {
        w->n_samples += n;
    }
}

static void wav_w_close(wav_w_t *w)
{
    if (w->fp == NULL) return;
    uint32_t data_sz = (uint32_t)(w->n_samples * 2u);
    uint32_t riff_sz = data_sz + 36u;
    if (fseek(w->fp, 4,  SEEK_SET) == 0) (void) fwrite(&riff_sz, 4, 1, w->fp);
    if (fseek(w->fp, 40, SEEK_SET) == 0) (void) fwrite(&data_sz, 4, 1, w->fp);
    fclose(w->fp);
    w->fp = NULL;
}

static void fmt_utc(char *buf, size_t cap)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm utc;
    gmtime_r(&tv.tv_sec, &utc);
    snprintf(buf, cap, "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
             (utc.tm_year + 1900) % 10000,
             (utc.tm_mon + 1) % 100, utc.tm_mday % 100,
             utc.tm_hour % 100, utc.tm_min % 100, utc.tm_sec % 100,
             (long)(tv.tv_usec / 1000));
}

static double monotonic_seconds(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

// Read the three lines (name + line1 + line2) of the requested
// satellite out of a TLE file. Same case-sensitive prefix convention as
// load_tle. Used to register the TLE in the packet DB at startup.
static int read_tle_lines(const char *path, const char *sat_prefix,
                          char *out_name,  size_t name_n,
                          char *out_line1, size_t line1_n,
                          char *out_line2, size_t line2_n)
{
    if (path == NULL || sat_prefix == NULL) return -1;
    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;
    char a[256] = {0}, b[256] = {0}, c[256] = {0};
    size_t plen = strlen(sat_prefix);
    int found = 0;
    while (fgets(a, sizeof a, f) != NULL) {
        size_t n = strlen(a);
        while (n > 0 && (a[n-1] == '\n' || a[n-1] == '\r')) a[--n] = '\0';
        if (a[0] == '1' || a[0] == '2') continue;
        if (strncmp(a, sat_prefix, plen) != 0) continue;
        if (fgets(b, sizeof b, f) == NULL) break;
        size_t nb = strlen(b);
        while (nb > 0 && (b[nb-1] == '\n' || b[nb-1] == '\r')) b[--nb] = '\0';
        if (fgets(c, sizeof c, f) == NULL) break;
        size_t nc = strlen(c);
        while (nc > 0 && (c[nc-1] == '\n' || c[nc-1] == '\r')) c[--nc] = '\0';
        if (b[0] != '1' || c[0] != '2') continue;
        snprintf(out_name,  name_n,  "%s", a);
        snprintf(out_line1, line1_n, "%s", b);
        snprintf(out_line2, line2_n, "%s", c);
        found = 1;
        break;
    }
    fclose(f);
    return found ? 0 : -1;
}

// Auto-name a WAV file under `pass_folder` (or cwd) using a shared UTC
// stamp. Returns 0 + filled *out, -1 on overflow.
static int auto_name_wav(const char *pass_folder, char *out, size_t cap)
{
    struct timeval tv;
    struct tm utc;
    if (gettimeofday(&tv, NULL) != 0 || gmtime_r(&tv.tv_sec, &utc) == NULL)
        return -1;
    int n;
    if (pass_folder && pass_folder[0]) {
        n = snprintf(out, cap,
                     "%s/simple_sat_ops_UT=%04d%02d%02dT%02d%02d%02d.%03ld.wav",
                     pass_folder,
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec,
                     (long)(tv.tv_usec / 1000));
    } else {
        n = snprintf(out, cap,
                     "simple_sat_ops_UT=%04d%02d%02dT%02d%02d%02d.%03ld.wav",
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec,
                     (long)(tv.tv_usec / 1000));
    }
    return (n > 0 && (size_t) n < cap) ? 0 : -1;
}

#define DEDUP_RING_SZ 64

struct rx_session {
    // Modem + AX100 options.
    modem_params_t mp;
    ax100_opts_t   opts;
    int            sync_max_ham;
    int            use_hmac;
    int            csp_crc32;
    int            force_beacon;

    // SDR LO offset (Hz) below the nominal carrier. Stored so the
    // snapshot can reconstruct the effective downlink frequency the
    // operator panel cares about (= LO + lo_offset + NCO offset).
    double         lo_offset_hz;

    // Cached sample-rate-dependent sizes.
    int      samp_rate;
    int      sps;
    size_t   window_samples;
    size_t   slide_samples;
    uint64_t dedup_quant;

    // Allocated scratch.
    int16_t *pcm_chunk;
    size_t   max_chunk;
    int16_t *iq_chunk;   // 2 * max_chunk int16: interleaved I,Q pairs
    int16_t *window;
    size_t   window_filled;
    // Shadow IQ window: interleaved I,Q pairs, same length (in pairs)
    // as `window` is in PCM samples. Filled in lockstep with `window`
    // so the IQ-domain demod sees the same time slice the PCM demod
    // does — that's what makes the A/B fair on the same RF.
    int16_t *iq_window;
    size_t   iq_window_filled;  // in PAIRS
    uint8_t *bits_scratch;
    size_t   bits_cap;
    uint8_t *bytes_scratch;
    size_t   bytes_cap;
    uint8_t  packet[4200];

    // Dedup ring (quantised ASM absolute sample index). frames_total +
    // emit_frame + per-type bookkeeping below all drive off the IQ
    // chain now — the IQ-domain demod is the live primary because the
    // FM-discriminator path gives up ~14 dB of avoidable SNR. The PCM
    // and Viterbi chains keep running in parallel, each with their own
    // dedup ring + counter, purely as A/B shadows the operator can use
    // to spot regressions.
    uint64_t recent_pos_quant[DEDUP_RING_SZ];
    int      recent_idx;
    int      recent_count;
    uint64_t total_window_samples;
    // PCM/FM-audio shadow counter. Same window, independent dedup so
    // any frame both chains catch ticks BOTH frames_total (the IQ
    // primary) and pcm_frames_total — that's the A signal.
    uint64_t pcm_recent_pos_quant[DEDUP_RING_SZ];
    int      pcm_recent_idx;
    int      pcm_recent_count;
    uint64_t pcm_frames_total;

    // Viterbi MSK-MLSE shadow counter. Same role as the PCM shadow —
    // count only, no DB write, no panel update — so an operator who
    // suspects the live chain is missing frames can compare against
    // the other two before believing a regression.
    uint64_t vit_recent_pos_quant[DEDUP_RING_SZ];
    int      vit_recent_idx;
    int      vit_recent_count;
    uint64_t vit_frames_total;

    // Output paths.
    wav_w_t  wav;
    char     wav_path[512];
    char     log_path[512];
    char     pass_folder[256];
    int      want_wav;

    // Raw IQ sidecar (interleaved int16 I,Q at samp_rate). Opens/closes
    // in lockstep with the WAV so each pass produces both a .wav and a
    // .iq with the same UTC stamp. The .iq feeds gen_waterfall to make
    // SatNOGS-style waterfalls; the WAV stays as the FM-demoded audio.
    FILE     *iq_fp;
    char      iq_path[512];
    uint64_t  iq_pairs_written;

    // Doppler-trajectory sidecar. Each row: monotonic ms since wav
    // opened, unix_time_ms, doppler_offset_hz. Lets offline tools
    // reverse the in-pump software Doppler correction if they want to
    // apply a different ephemeris.
    FILE     *doppler_fp;
    char      doppler_path[512];
    double    doppler_last_log_t;  // monotonic_seconds() at last write

    // Frame counters + last-decoded summary.
    uint64_t frames_total;
    unsigned frames_in_window;
    char     last_frame_ts[24];
    int      last_frame_len;
    // Per-type stats. Worker writes under mu so the main-thread
    // snapshot can read a coherent copy.
    uint64_t per_type_count[RX_PT_COUNT];
    int      per_type_last_len[RX_PT_COUNT];
    uint8_t  per_type_last_payload[RX_PT_COUNT][RX_LAST_PAYLOAD_MAX];
    char     per_type_last_summary[RX_PT_COUNT][RX_LAST_SUMMARY_MAX];
    // Monotonic time of the most-recent decoded frame; 0.0 means
    // "no frame yet".
    double   last_frame_monotonic_s;

    // packet_db handle (owned).
    packet_db_t *db;
    char         db_run_id[24];

    // --- Threading ---
    // The worker thread owns `core` and runs the UHD pump + decode +
    // wav writer + TX burst. Main thread interacts via the request
    // flags and the snapshot, both protected by `mu`.
    b210_rx_tx_core_t *core;
    pthread_t          thread;
    int                thread_started;
    pthread_mutex_t    mu;
    pthread_cond_t     cv;
    volatile int       stop_requested;

    // Requests from main thread (set under mu, picked up by worker).
    int     freq_req_pending;
    double  freq_req_hz;
    int     wav_start_req;
    int     wav_stop_req;

    // TX burst handoff (synchronous).
    int                burst_req_pending;
    int                burst_complete;
    tx_request_slot_t  burst_req;
    uint8_t            burst_hmac_key[128];
    size_t             burst_hmac_key_len;
    rx_burst_result_t  burst_result;
    char               burst_summary[160];

    // Snapshot (updated by worker, read by main under mu).
    uint64_t snap_frames_total;
    double   snap_peak;
    double   snap_rms_sq;
    double   snap_actual_freq_hz;
    // Broadband-burst snapshot from the iq_burst detector. snap_burst_
    // bright_bins is the FFT bins-above-floor count; the operator
    // ribbon uses this to distinguish CW carriers (low count) from
    // wideband packet bursts (high count).
    int      snap_burst_bright_bins;
    double   snap_burst_peak_excess_db;
    char     snap_last_frame_ts[24];
    int      snap_last_frame_len;
    int      snap_wav_active;
    double   snap_last_frame_monotonic_s;
    uint64_t snap_per_type_count[RX_PT_COUNT];
    int      snap_per_type_last_len[RX_PT_COUNT];
    uint8_t  snap_per_type_last_payload[RX_PT_COUNT][RX_LAST_PAYLOAD_MAX];
    char     snap_per_type_last_summary[RX_PT_COUNT][RX_LAST_SUMMARY_MAX];
    char     snap_wav_path[512];
    long     snap_wav_n_samples;
    char     snap_iq_path[512];
    long     snap_iq_pairs;
    uint64_t snap_pcm_frames_total;
    uint64_t snap_vit_frames_total;

    // lo_offset.csv sidecar: same lifecycle as doppler.csv (opens with
    // the WAV, closes with it). One row per change of lo_offset_hz so
    // offline reprocessing of the .iq can replay the exact LO history.
    FILE     *lo_offset_fp;
    char      lo_offset_path[512];
};

static void *rx_session_thread_fn(void *arg);

int rx_session_open(rx_session_t **out, const rx_session_params_t *p,
                    b210_rx_tx_core_t *core)
{
    if (out == NULL || p == NULL || core == NULL) return -1;
    *out = NULL;

    rx_session_t *rxs = calloc(1, sizeof(*rxs));
    if (rxs == NULL) return -1;

    modem_params_defaults(&rxs->mp);
    rxs->mp.bit_rate  = p->bit_rate > 0 ? p->bit_rate : 9600;
    double actual_rate = b210_rx_tx_core_actual_rate(core);
    rxs->samp_rate    = (int) actual_rate;
    rxs->mp.samp_rate = rxs->samp_rate;
    if (rxs->samp_rate <= 0 || (rxs->samp_rate % rxs->mp.bit_rate) != 0) {
        fprintf(stderr,
            "rx_session: post-decim rate %d S/s is not a multiple of "
            "bit_rate %d — RX disabled\n",
            rxs->samp_rate, rxs->mp.bit_rate);
        free(rxs);
        return -1;
    }
    rxs->sps          = rxs->samp_rate / rxs->mp.bit_rate;
    double window_s   = p->window_s > 0.0 ? p->window_s : 1.5;
    double slide_s    = p->slide_s  > 0.0 ? p->slide_s  : 0.5;
    if (slide_s > window_s) slide_s = window_s;
    rxs->window_samples = (size_t)(window_s * rxs->samp_rate);
    rxs->slide_samples  = (size_t)(slide_s  * rxs->samp_rate);
    if (rxs->slide_samples == 0) rxs->slide_samples = rxs->window_samples;

    ax100_opts_defaults(&rxs->opts);
    rxs->opts.reed_solomon = p->use_rs;
    if (p->use_hmac && p->hmac_key && p->hmac_key_len > 0) {
        rxs->opts.hmac_key     = p->hmac_key;
        rxs->opts.hmac_key_len = p->hmac_key_len;
    }
    rxs->sync_max_ham = p->sync_max_ham > 0 ? p->sync_max_ham : 4;
    rxs->use_hmac     = p->use_hmac;
    rxs->csp_crc32    = p->csp_crc32;
    rxs->force_beacon = p->force_beacon;

    decode_loop_set_show_headers(p->show_packet_headers);

    rxs->max_chunk = b210_rx_tx_core_max_chunk(core);
    if (rxs->max_chunk == 0) rxs->max_chunk = 2040;
    rxs->pcm_chunk     = malloc(rxs->max_chunk * sizeof(int16_t));
    rxs->iq_chunk      = malloc(rxs->max_chunk * 2 * sizeof(int16_t));
    rxs->window        = malloc(rxs->window_samples * sizeof(int16_t));
    rxs->iq_window     = malloc(rxs->window_samples * 2 * sizeof(int16_t));
    rxs->bits_cap      = rxs->window_samples + 8;
    rxs->bits_scratch  = malloc(rxs->bits_cap);
    rxs->bytes_cap     = rxs->bits_cap / 8 + 1;
    rxs->bytes_scratch = malloc(rxs->bytes_cap);
    if (!rxs->pcm_chunk || !rxs->iq_chunk || !rxs->window
        || !rxs->iq_window
        || !rxs->bits_scratch || !rxs->bytes_scratch) {
        rx_session_close(rxs);
        return -1;
    }

    rxs->dedup_quant = (uint64_t)(0.1 * (double) rxs->samp_rate);
    if (rxs->dedup_quant == 0) rxs->dedup_quant = 1;

    // packet_db registration + TLE id + session dir.
    rxs->db = packet_db_setup(p->db_path, p->no_db,
                              rxs->db_run_id, sizeof rxs->db_run_id);
    decode_loop_set_packet_db(rxs->db, "simple_sat_ops", rxs->db_run_id);
    if (rxs->db != NULL && p->tle_path && p->sat_name) {
        char tle_name[128] = {0}, tle_line1[128] = {0}, tle_line2[128] = {0};
        if (read_tle_lines(p->tle_path, p->sat_name,
                           tle_name,  sizeof tle_name,
                           tle_line1, sizeof tle_line1,
                           tle_line2, sizeof tle_line2) == 0) {
            long long tle_id = packet_db_register_tle(rxs->db, tle_name,
                                                       tle_line1, tle_line2);
            if (tle_id > 0) decode_loop_set_tle_id(tle_id);
        }
    }
    if (p->session_dir && p->session_dir[0]) {
        decode_loop_set_session_dir(p->session_dir);
    }

    // Defer the WAV open until the caller arms a pass.
    rxs->want_wav = p->want_wav;
    if (p->pass_folder && p->pass_folder[0]) {
        snprintf(rxs->pass_folder, sizeof rxs->pass_folder,
                 "%s", p->pass_folder);
    }

    // Threading: rx_session takes ownership of the core. The worker
    // pumps UHD continuously and services freq retunes / WAV
    // start-stops / TX bursts queued by the main thread.
    rxs->core = core;
    rxs->lo_offset_hz = p->lo_offset_hz;
    // lo_offset_hz is SIGNED: positive → LO above nominal, negative
    // → LO below. Hardware LO = nominal + lo_offset_hz, so to recover
    // the nominal carrier from the actual hardware tune we subtract.
    rxs->snap_actual_freq_hz = b210_rx_tx_core_actual_freq(core)
                             - rxs->lo_offset_hz;
    pthread_mutex_init(&rxs->mu, NULL);
    pthread_cond_init (&rxs->cv, NULL);
    if (pthread_create(&rxs->thread, NULL, rx_session_thread_fn, rxs) != 0) {
        fprintf(stderr, "rx_session: pthread_create failed\n");
        pthread_cond_destroy (&rxs->cv);
        pthread_mutex_destroy(&rxs->mu);
        rxs->core = NULL;
        rx_session_close(rxs);
        return -1;
    }
    rxs->thread_started = 1;

    *out = rxs;
    return 0;
}

// Worker-internal: open/close the WAV file. Called only from the
// thread, so no locking needed for the wav_w_t itself.
static void worker_wav_start(rx_session_t *rxs)
{
    if (!rxs->want_wav || rxs->wav.fp != NULL) return;
    if (auto_name_wav(rxs->pass_folder[0] ? rxs->pass_folder : NULL,
                      rxs->wav_path, sizeof rxs->wav_path) != 0
        || wav_w_open(&rxs->wav, rxs->wav_path, rxs->samp_rate) != 0) {
        // Can't write to stderr while ncurses owns the screen — it
        // corrupts the operator UI. The wav_path[0]='\0' below leaves
        // wav.fp NULL, so the next snapshot reports wav_active = 0 and
        // the operator's [REC] indicator stays dark, which is the
        // visible signal that the open failed.
        rxs->wav_path[0] = '\0';
        return;
    }
    // Open the IQ sidecar — same base name, .iq extension. Raw int16
    // I,Q pairs at samp_rate; gen_waterfall reads them straight. If the
    // open fails we keep the WAV open (so the audio path still works)
    // and just leave iq_fp NULL so subsequent pumps skip the write.
    // Both buffers are 512; ".iq" needs 3 bytes + nul, so cap the
    // path-portion at 508 so the suffix always fits and GCC can prove
    // the snprintf won't truncate.
    size_t wlen = strlen(rxs->wav_path);
    if (wlen >= 4 && strcmp(rxs->wav_path + wlen - 4, ".wav") == 0) {
        int base_len = (int)(wlen - 4);
        if (base_len > 508) base_len = 508;
        snprintf(rxs->iq_path, sizeof rxs->iq_path,
                 "%.*s.iq", base_len, rxs->wav_path);
    } else {
        snprintf(rxs->iq_path, sizeof rxs->iq_path,
                 "%.508s.iq", rxs->wav_path);
    }
    rxs->iq_fp = fopen(rxs->iq_path, "wb");
    rxs->iq_pairs_written = 0;
    if (rxs->iq_fp == NULL) rxs->iq_path[0] = '\0';

    // Doppler-trajectory sidecar (same base name + .doppler.csv).
    // Records the software-NCO offset over time so an offline tool can
    // undo our correction and try a different ephemeris.
    // ".doppler.csv" is 12 bytes; doppler_path is 512 → path body must
    // fit in 499 bytes so the suffix + NUL lands inside the buffer and
    // GCC's -Wformat-truncation is satisfied (clang doesn't currently
    // emit that warning; see TROUBLESHOOTING below for the lint helper).
    if (wlen >= 4 && strcmp(rxs->wav_path + wlen - 4, ".wav") == 0) {
        int base_len = (int)(wlen - 4);
        if (base_len > 499) base_len = 499;
        snprintf(rxs->doppler_path, sizeof rxs->doppler_path,
                 "%.*s.doppler.csv", base_len, rxs->wav_path);
    } else {
        snprintf(rxs->doppler_path, sizeof rxs->doppler_path,
                 "%.499s.doppler.csv", rxs->wav_path);
    }
    rxs->doppler_fp = fopen(rxs->doppler_path, "w");
    rxs->doppler_last_log_t = 0.0;
    if (rxs->doppler_fp != NULL) {
        fputs("# unix_time_ms,doppler_offset_hz\n", rxs->doppler_fp);
    } else {
        rxs->doppler_path[0] = '\0';
    }

    // lo_offset sidecar: one row per change of the operator's SDR LO
    // offset (typed as `:lo_offset <±kHz>` in the UI). Seeded with the
    // current value at file open so a reader can reconstruct the LO
    // history without needing the CLI flag context. Same lifecycle as
    // doppler.csv: opens with the WAV, closes when it does.
    // ".lo_offset.csv" is 14 bytes; lo_offset_path is 512 → path body
    // must fit in 497 bytes so the suffix + NUL lands inside the
    // buffer and gcc's -Wformat-truncation is satisfied (clang
    // doesn't currently emit that warning — same trap the doppler.csv
    // path notes above).
    if (wlen >= 4 && strcmp(rxs->wav_path + wlen - 4, ".wav") == 0) {
        int base_len = (int)(wlen - 4);
        if (base_len > 497) base_len = 497;
        snprintf(rxs->lo_offset_path, sizeof rxs->lo_offset_path,
                 "%.*s.lo_offset.csv", base_len, rxs->wav_path);
    } else {
        snprintf(rxs->lo_offset_path, sizeof rxs->lo_offset_path,
                 "%.497s.lo_offset.csv", rxs->wav_path);
    }
    rxs->lo_offset_fp = fopen(rxs->lo_offset_path, "w");
    if (rxs->lo_offset_fp != NULL) {
        fputs("# unix_time_ms,lo_offset_hz\n", rxs->lo_offset_fp);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        long long unix_ms = (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000;
        fprintf(rxs->lo_offset_fp, "%lld,%.6f\n", unix_ms, rxs->lo_offset_hz);
        fflush(rxs->lo_offset_fp);
    } else {
        rxs->lo_offset_path[0] = '\0';
    }
    // No stderr print on success either — operator sees [REC] in the
    // UI, and the WAV path is auto-named from the same UTC stamp as
    // every other artifact in the pass folder.
}

static void worker_wav_stop(rx_session_t *rxs)
{
    if (rxs->wav.fp != NULL) wav_w_close(&rxs->wav);
    if (rxs->iq_fp != NULL) {
        fclose(rxs->iq_fp);
        rxs->iq_fp = NULL;
    }
    if (rxs->doppler_fp != NULL) {
        fclose(rxs->doppler_fp);
        rxs->doppler_fp = NULL;
    }
    if (rxs->lo_offset_fp != NULL) {
        fclose(rxs->lo_offset_fp);
        rxs->lo_offset_fp = NULL;
    }
}

void rx_session_request_wav_start(rx_session_t *rxs)
{
    if (rxs == NULL || !rxs->want_wav) return;
    pthread_mutex_lock(&rxs->mu);
    rxs->wav_start_req = 1;
    rxs->wav_stop_req  = 0;
    pthread_cond_broadcast(&rxs->cv);
    pthread_mutex_unlock(&rxs->mu);
}

void rx_session_request_wav_stop(rx_session_t *rxs)
{
    if (rxs == NULL) return;
    pthread_mutex_lock(&rxs->mu);
    rxs->wav_stop_req  = 1;
    rxs->wav_start_req = 0;
    pthread_cond_broadcast(&rxs->cv);
    pthread_mutex_unlock(&rxs->mu);
}

int rx_session_wav_active(const rx_session_t *rxs)
{
    if (rxs == NULL) return 0;
    pthread_mutex_lock((pthread_mutex_t *) &rxs->mu);
    int v = rxs->snap_wav_active;
    pthread_mutex_unlock((pthread_mutex_t *) &rxs->mu);
    return v;
}

void rx_session_wav_snapshot(const rx_session_t *rxs,
                             char     *out_path, size_t path_cap,
                             long     *out_n_samples,
                             int      *out_sample_rate,
                             int      *out_active)
{
    if (out_path && path_cap)  out_path[0]       = '\0';
    if (out_n_samples)         *out_n_samples    = 0;
    if (out_sample_rate)       *out_sample_rate  = 0;
    if (out_active)            *out_active       = 0;
    if (rxs == NULL) return;
    pthread_mutex_lock((pthread_mutex_t *) &rxs->mu);
    if (out_path && path_cap) {
        snprintf(out_path, path_cap, "%s", rxs->snap_wav_path);
    }
    if (out_n_samples)    *out_n_samples   = rxs->snap_wav_n_samples;
    if (out_sample_rate)  *out_sample_rate = rxs->samp_rate;
    if (out_active)       *out_active      = rxs->snap_wav_active;
    pthread_mutex_unlock((pthread_mutex_t *) &rxs->mu);
}

void rx_session_iq_snapshot(const rx_session_t *rxs,
                            char *out_path, size_t path_cap,
                            long *out_pairs,
                            int  *out_sample_rate)
{
    if (out_path && path_cap)  out_path[0]       = '\0';
    if (out_pairs)             *out_pairs        = 0;
    if (out_sample_rate)       *out_sample_rate  = 0;
    if (rxs == NULL) return;
    pthread_mutex_lock((pthread_mutex_t *) &rxs->mu);
    if (out_path && path_cap) {
        snprintf(out_path, path_cap, "%s", rxs->snap_iq_path);
    }
    if (out_pairs)        *out_pairs       = rxs->snap_iq_pairs;
    if (out_sample_rate)  *out_sample_rate = rxs->samp_rate;
    pthread_mutex_unlock((pthread_mutex_t *) &rxs->mu);
}

uint64_t rx_session_viterbi_frames(const rx_session_t *rxs)
{
    if (!rxs) return 0;
    pthread_mutex_lock((pthread_mutex_t *) &rxs->mu);
    uint64_t v = rxs->snap_vit_frames_total;
    pthread_mutex_unlock((pthread_mutex_t *) &rxs->mu);
    return v;
}

uint64_t rx_session_pcm_frames(const rx_session_t *rxs)
{
    if (rxs == NULL) return 0;
    pthread_mutex_lock((pthread_mutex_t *) &rxs->mu);
    uint64_t v = rxs->snap_pcm_frames_total;
    pthread_mutex_unlock((pthread_mutex_t *) &rxs->mu);
    return v;
}

void rx_session_request_freq(rx_session_t *rxs, double freq_hz)
{
    if (rxs == NULL) return;
    pthread_mutex_lock(&rxs->mu);
    rxs->freq_req_pending = 1;
    rxs->freq_req_hz      = freq_hz;
    pthread_cond_broadcast(&rxs->cv);
    pthread_mutex_unlock(&rxs->mu);
}

void rx_session_set_doppler_offset(rx_session_t *rxs, double offset_hz)
{
    if (rxs == NULL || rxs->core == NULL) return;
    // Lock-free: the NCO is a single double on the core, and the pump
    // re-reads it once per chunk. No worker handoff needed.
    b210_rx_tx_core_set_doppler_offset(rxs->core, offset_hz);
}

void rx_session_set_lo_offset(rx_session_t *rxs,
                              double nominal_freq_hz,
                              double new_lo_offset_hz)
{
    if (rxs == NULL || rxs->core == NULL) return;
    // Two coordinated changes: retune the hardware LO to
    // (nominal + new_lo_offset) and update the FM-path compensation
    // NCO to match. Without the second step the discriminator would
    // see the carrier somewhere other than DC and clip the FSK upper
    // level (same bug we fixed at session-open time, but now for
    // runtime adjustments).
    //
    // The hardware retune is forwarded through the existing worker
    // handoff (freq_req_pending) so we don't touch the UHD streamer
    // from this thread. The fm_lo_nco update is lock-free — same
    // pattern as set_doppler_offset above.
    rxs->lo_offset_hz = new_lo_offset_hz;
    b210_rx_tx_core_set_fm_lo_compensation(rxs->core, new_lo_offset_hz);
    rx_session_request_freq(rxs, nominal_freq_hz + new_lo_offset_hz);

    // Append to the per-pass lo_offset.csv so offline reprocessing of
    // the .iq sidecar can replay the same LO history. mu protects the
    // file pointer against worker-side close-on-wav-stop. No-op when
    // the WAV isn't open (sidecar lifecycle matches the .iq we'd be
    // re-tuning against).
    pthread_mutex_lock(&rxs->mu);
    if (rxs->lo_offset_fp != NULL) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        long long unix_ms = (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000;
        fprintf(rxs->lo_offset_fp, "%lld,%.6f\n", unix_ms, new_lo_offset_hz);
        fflush(rxs->lo_offset_fp);
    }
    pthread_mutex_unlock(&rxs->mu);
}

double rx_session_get_doppler_offset(const rx_session_t *rxs)
{
    if (rxs == NULL || rxs->core == NULL) return 0.0;
    return b210_rx_tx_core_get_doppler_offset(rxs->core);
}

double rx_session_get_lo_freq_hz(const rx_session_t *rxs)
{
    if (rxs == NULL || rxs->core == NULL) return 0.0;
    return b210_rx_tx_core_actual_freq(rxs->core);
}

double rx_session_get_bandwidth_hz(const rx_session_t *rxs)
{
    if (rxs == NULL || rxs->core == NULL) return 0.0;
    return b210_rx_tx_core_actual_rate(rxs->core);
}

rx_burst_result_t rx_session_request_burst_sync(
    rx_session_t *rxs,
    const tx_request_slot_t *req,
    const uint8_t *hmac_key, size_t hmac_key_len,
    char *out_summary, size_t summary_n)
{
    if (out_summary && summary_n) out_summary[0] = '\0';
    if (rxs == NULL || req == NULL) return RX_BURST_NO_CORE;

    pthread_mutex_lock(&rxs->mu);
    rxs->burst_req            = *req;
    rxs->burst_hmac_key_len   = (hmac_key && hmac_key_len > 0
                                  && hmac_key_len <= sizeof rxs->burst_hmac_key)
                                  ? hmac_key_len : 0;
    if (rxs->burst_hmac_key_len > 0) {
        memcpy(rxs->burst_hmac_key, hmac_key, rxs->burst_hmac_key_len);
    }
    rxs->burst_complete    = 0;
    rxs->burst_req_pending = 1;
    pthread_cond_broadcast(&rxs->cv);
    while (!rxs->burst_complete && !rxs->stop_requested) {
        pthread_cond_wait(&rxs->cv, &rxs->mu);
    }
    rx_burst_result_t res = rxs->burst_result;
    if (out_summary && summary_n) {
        snprintf(out_summary, summary_n, "%s", rxs->burst_summary);
    }
    pthread_mutex_unlock(&rxs->mu);
    return res;
}

void rx_session_close(rx_session_t *rxs)
{
    if (rxs == NULL) return;
    if (rxs->thread_started) {
        pthread_mutex_lock(&rxs->mu);
        rxs->stop_requested = 1;
        pthread_cond_broadcast(&rxs->cv);
        pthread_mutex_unlock(&rxs->mu);
        pthread_join(rxs->thread, NULL);
        rxs->thread_started = 0;
        pthread_cond_destroy (&rxs->cv);
        pthread_mutex_destroy(&rxs->mu);
    }
    // Worker exited, so we own the wav/iq/core/db scratch outright.
    if (rxs->wav.fp) wav_w_close(&rxs->wav);
    if (rxs->iq_fp)  { fclose(rxs->iq_fp); rxs->iq_fp = NULL; }
    if (rxs->core)   b210_rx_tx_core_close(rxs->core);
    if (rxs->db)     packet_db_close(rxs->db);
    free(rxs->pcm_chunk);
    free(rxs->iq_chunk);
    free(rxs->window);
    free(rxs->iq_window);
    free(rxs->bits_scratch);
    free(rxs->bytes_scratch);
    free(rxs);
}

// PCM/FM-audio shadow chain: dedup + bump pcm_frames_total. No
// emit_frame, no DB insert, no per-type bookkeeping — the IQ chain
// owns those now (see try_decode_iq_at_window). Counted purely so the
// operator panel + IPC can show the A signal alongside the live IQ
// count.
static void try_decode_at_window(rx_session_t *rxs)
{
    size_t inner_min_offset = 0;
    for (;;) {
        ssize_t plen = -1;
        int golay_errs = 0, hmac_ok = -1;
        int rs_errs = -1, used_golay_len = -1;
        int rs_locs[32];
        size_t sync_off_local = 0;
        if (!try_decode_window(rxs->window, rxs->window_samples,
                               &rxs->mp, &rxs->opts,
                               rxs->sync_max_ham, rxs->use_hmac,
                               /*allow_partial_rs=*/1,
                               inner_min_offset,
                               rxs->bits_scratch, rxs->bits_cap,
                               rxs->bytes_scratch, rxs->bytes_cap,
                               rxs->packet, sizeof rxs->packet,
                               &plen, &golay_errs, &hmac_ok,
                               &rs_errs, &used_golay_len,
                               &sync_off_local, rs_locs)) {
            break;
        }
        inner_min_offset = sync_off_local + 1;
        if (plen < 4 || (size_t) plen > sizeof rxs->packet) continue;

        uint64_t window_start_abs =
            rxs->total_window_samples - (uint64_t) rxs->window_samples;
        uint64_t asm_abs_sample = window_start_abs
            + (uint64_t) sync_off_local * (uint64_t) rxs->sps
            + (uint64_t)(rxs->sps / 2);
        uint64_t pos_quant = asm_abs_sample / rxs->dedup_quant;
        int seen = 0;
        int ring_n = rxs->pcm_recent_count < DEDUP_RING_SZ
                   ? rxs->pcm_recent_count : DEDUP_RING_SZ;
        for (int r = 0; r < ring_n; r++) {
            if (rxs->pcm_recent_pos_quant[r] == pos_quant) { seen = 1; break; }
        }
        if (seen) continue;
        rxs->pcm_recent_pos_quant[rxs->pcm_recent_idx] = pos_quant;
        rxs->pcm_recent_idx = (rxs->pcm_recent_idx + 1) % DEDUP_RING_SZ;
        if (rxs->pcm_recent_count < DEDUP_RING_SZ) rxs->pcm_recent_count++;

        rxs->pcm_frames_total++;
    }
}

// IQ-domain decoder — the LIVE primary chain. Runs the IQ-slicer on
// post-decim IQ (~14 dB SNR-better than the FM-discriminator path),
// dedupes via the main `recent_pos_quant` ring, then emits to the DB
// and packet log, updates per-type bookkeeping + last-frame state,
// and bumps frames_total. The PCM and Viterbi chains are shadow
// counters — see try_decode_at_window / try_decode_viterbi_at_window.
static void try_decode_iq_at_window(rx_session_t *rxs)
{
    size_t inner_min_offset = 0;
    for (;;) {
        ssize_t plen = -1;
        int golay_errs = 0, hmac_ok = -1;
        int rs_errs = -1, used_golay_len = -1;
        int rs_locs[32];
        size_t sync_off_local = 0;
        if (!try_decode_window_iq(rxs->iq_window, rxs->window_samples,
                                  &rxs->mp, &rxs->opts,
                                  rxs->sync_max_ham, rxs->use_hmac,
                                  /*allow_partial_rs=*/1,
                                  inner_min_offset,
                                  rxs->bits_scratch, rxs->bits_cap,
                                  rxs->bytes_scratch, rxs->bytes_cap,
                                  rxs->packet, sizeof rxs->packet,
                                  &plen, &golay_errs, &hmac_ok,
                                  &rs_errs, &used_golay_len,
                                  &sync_off_local, rs_locs)) {
            break;
        }
        inner_min_offset = sync_off_local + 1;
        if (plen < 4 || (size_t) plen > sizeof rxs->packet) continue;

        int       crc_status   = -1;
        uint32_t  crc_computed = 0, crc_le = 0, crc_be = 0;
        if (!rxs->use_hmac && rxs->csp_crc32 && plen >= 8) {
            crc_computed = csp_crc32_zlib(rxs->packet, (size_t)(plen - 4));
            crc_le = (uint32_t) rxs->packet[plen - 4]
                   | ((uint32_t) rxs->packet[plen - 3] << 8)
                   | ((uint32_t) rxs->packet[plen - 2] << 16)
                   | ((uint32_t) rxs->packet[plen - 1] << 24);
            crc_be = ((uint32_t) rxs->packet[plen - 4] << 24)
                   | ((uint32_t) rxs->packet[plen - 3] << 16)
                   | ((uint32_t) rxs->packet[plen - 2] <<  8)
                   |  (uint32_t) rxs->packet[plen - 1];
            if (crc_computed == crc_le || crc_computed == crc_be) {
                crc_status = 1;
                plen -= 4;
            } else {
                crc_status = 0;
            }
        }

        // Dedup by quantised absolute ASM sample index. Uses the main
        // recent_pos_quant ring (shared with the live emit path) so
        // the same physical frame caught in two overlapping windows
        // only writes once.
        uint64_t window_start_abs =
            rxs->total_window_samples - (uint64_t) rxs->window_samples;
        uint64_t asm_abs_sample = window_start_abs
            + (uint64_t) sync_off_local * (uint64_t) rxs->sps
            + (uint64_t)(rxs->sps / 2);
        uint64_t pos_quant = asm_abs_sample / rxs->dedup_quant;
        int seen = 0;
        int ring_n = rxs->recent_count < DEDUP_RING_SZ
                   ? rxs->recent_count : DEDUP_RING_SZ;
        for (int r = 0; r < ring_n; r++) {
            if (rxs->recent_pos_quant[r] == pos_quant) { seen = 1; break; }
        }
        if (seen) continue;
        rxs->recent_pos_quant[rxs->recent_idx] = pos_quant;
        rxs->recent_idx = (rxs->recent_idx + 1) % DEDUP_RING_SZ;
        if (rxs->recent_count < DEDUP_RING_SZ) rxs->recent_count++;

        char ts[64];
        fmt_utc(ts, sizeof ts);
        emit_frame(rxs->log_path[0] ? rxs->log_path : NULL,
                   /*quiet=*/1, ts,
                   rxs->packet, (size_t) plen,
                   golay_errs, hmac_ok, rxs->use_hmac,
                   rs_errs, used_golay_len,
                   crc_status, crc_computed, crc_le, crc_be,
                   rs_locs,
                   NULL, 0,
                   rxs->force_beacon);
        rxs->frames_total++;
        if (rxs->frames_in_window < UINT_MAX) rxs->frames_in_window++;
        snprintf(rxs->last_frame_ts, sizeof rxs->last_frame_ts,
                 "%.*s", (int)(sizeof rxs->last_frame_ts - 1), ts);
        rxs->last_frame_len = (int) plen;

        // Per-type bookkeeping. FrontierSat tags packet_type in the
        // first byte of the CSP payload (after the 4-byte CSP header).
        rx_packet_type_slot_t slot = RX_PT_OTHER;
        if (plen >= 5) {
            uint8_t ptype = rxs->packet[4];
            switch (ptype) {
                case 0x01: slot = RX_PT_BEACON_BASIC; break;
                case 0x02: slot = RX_PT_BEACON_PERIPHERAL; break;
                case 0x03: slot = RX_PT_LOG_MESSAGE; break;
                case 0x04: slot = RX_PT_TCMD_RESPONSE; break;
                case 0x10: slot = RX_PT_BULK_FILE; break;
                default:   slot = RX_PT_OTHER; break;
            }
        }
        rxs->per_type_count[slot]++;
        int copy = (plen < RX_LAST_PAYLOAD_MAX) ? (int)plen : RX_LAST_PAYLOAD_MAX;
        rxs->per_type_last_len[slot] = copy;
        memcpy(rxs->per_type_last_payload[slot], rxs->packet, (size_t)copy);
        char *sum_out  = rxs->per_type_last_summary[slot];
        size_t sum_cap = sizeof rxs->per_type_last_summary[slot];
        sum_out[0] = '\0';
        switch (slot) {
            case RX_PT_BEACON_BASIC:
                beacon_basic_summary(rxs->packet, (size_t) plen,
                                     sum_out, sum_cap);
                break;
            case RX_PT_TCMD_RESPONSE:
                tcmd_response_summary(rxs->packet, (size_t) plen,
                                      sum_out, sum_cap);
                break;
            case RX_PT_LOG_MESSAGE:
                log_message_summary(rxs->packet, (size_t) plen,
                                    sum_out, sum_cap);
                break;
            default:
                break;
        }

        struct timespec mono;
        if (clock_gettime(CLOCK_MONOTONIC, &mono) == 0) {
            rxs->last_frame_monotonic_s =
                (double) mono.tv_sec + (double) mono.tv_nsec * 1e-9;
        }
    }
}

// Viterbi MLSE shadow chain. Counts only — no DB write, no panel
// update. Independent dedup ring so any frame all three chains catch
// shows up in PCM, IQ (live), AND Viterbi counters separately.
static void try_decode_viterbi_at_window(rx_session_t *rxs)
{
    size_t inner_min_offset = 0;
    for (;;) {
        ssize_t plen = -1;
        int golay_errs = 0, hmac_ok = -1;
        int rs_errs = -1, used_golay_len = -1;
        int rs_locs[32];
        size_t sync_off_local = 0;
        if (!try_decode_window_viterbi(rxs->iq_window, rxs->window_samples,
                                       &rxs->mp, &rxs->opts,
                                       rxs->sync_max_ham, rxs->use_hmac,
                                       /*allow_partial_rs=*/1,
                                       inner_min_offset,
                                       rxs->bits_scratch, rxs->bits_cap,
                                       rxs->bytes_scratch, rxs->bytes_cap,
                                       rxs->packet, sizeof rxs->packet,
                                       &plen, &golay_errs, &hmac_ok,
                                       &rs_errs, &used_golay_len,
                                       &sync_off_local, rs_locs)) {
            break;
        }
        inner_min_offset = sync_off_local + 1;
        if (plen < 4 || (size_t) plen > sizeof rxs->packet) continue;

        uint64_t window_start_abs =
            rxs->total_window_samples - (uint64_t) rxs->window_samples;
        uint64_t asm_abs_sample = window_start_abs
            + (uint64_t) sync_off_local * (uint64_t) rxs->sps
            + (uint64_t)(rxs->sps / 2);
        uint64_t pos_quant = asm_abs_sample / rxs->dedup_quant;
        int seen = 0;
        int ring_n = rxs->vit_recent_count < DEDUP_RING_SZ
                   ? rxs->vit_recent_count : DEDUP_RING_SZ;
        for (int r = 0; r < ring_n; r++) {
            if (rxs->vit_recent_pos_quant[r] == pos_quant) { seen = 1; break; }
        }
        if (seen) continue;
        rxs->vit_recent_pos_quant[rxs->vit_recent_idx] = pos_quant;
        rxs->vit_recent_idx = (rxs->vit_recent_idx + 1) % DEDUP_RING_SZ;
        if (rxs->vit_recent_count < DEDUP_RING_SZ) rxs->vit_recent_count++;

        rxs->vit_frames_total++;
    }
}

static int worker_pump_once(rx_session_t *rxs)
{
    // Pass the IQ tap buffer through so the core copies post-decim IQ
    // alongside the FM-demoded PCM. iq_pairs may be less than n on the
    // very first pump (the discriminator emits a leading zero for the
    // missing prev-sample) — that's fine; we write only what the core
    // actually delivered.
    size_t iq_pairs = 0;
    ssize_t n = b210_rx_tx_core_pump(rxs->core, rxs->pcm_chunk,
                                     rxs->max_chunk,
                                     rxs->iq_chunk, rxs->max_chunk * 2,
                                     &iq_pairs);
    if (n < 0) return -1;
    if (n == 0) return 0;
    if (rxs->wav.fp) wav_w_append(&rxs->wav, rxs->pcm_chunk, (size_t) n);
    if (rxs->iq_fp && iq_pairs > 0) {
        size_t want = iq_pairs * 2;  // int16 count
        if (fwrite(rxs->iq_chunk, sizeof(int16_t), want, rxs->iq_fp) == want) {
            rxs->iq_pairs_written += iq_pairs;
        }
    }
    // Doppler-trajectory sidecar: one line per ~1 s of recording. Lets
    // an offline tool reverse the in-pump NCO if it wants to try a
    // different ephemeris. Stamped with wall-clock UNIX ms (the values
    // we tag the WAV/.iq filenames with), monotonic-aligned by sample
    // count so the sidecar replays without time drift.
    if (rxs->doppler_fp && rxs->core != NULL) {
        double t_now_mono = monotonic_seconds();
        if (rxs->doppler_last_log_t == 0.0
            || (t_now_mono - rxs->doppler_last_log_t) >= 1.0) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            long long unix_ms =
                (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000;
            double offset = b210_rx_tx_core_get_doppler_offset(rxs->core);
            fprintf(rxs->doppler_fp, "%lld,%.6f\n", unix_ms, offset);
            fflush(rxs->doppler_fp);
            rxs->doppler_last_log_t = t_now_mono;
        }
    }

    size_t pairs_to_use = iq_pairs;
    if (pairs_to_use > (size_t) n) pairs_to_use = (size_t) n;
    for (ssize_t i = 0; i < n; i++) {
        rxs->window[rxs->window_filled++] = rxs->pcm_chunk[i];
        if ((size_t) i < pairs_to_use
            && rxs->iq_window_filled < rxs->window_samples) {
            rxs->iq_window[rxs->iq_window_filled * 2 + 0] =
                rxs->iq_chunk[i * 2 + 0];
            rxs->iq_window[rxs->iq_window_filled * 2 + 1] =
                rxs->iq_chunk[i * 2 + 1];
            rxs->iq_window_filled++;
        }
        rxs->total_window_samples++;
        if (rxs->window_filled < rxs->window_samples) continue;
        try_decode_at_window(rxs);
        if (rxs->iq_window_filled >= rxs->window_samples) {
            try_decode_iq_at_window(rxs);
            try_decode_viterbi_at_window(rxs);
        }
        memmove(rxs->window, rxs->window + rxs->slide_samples,
                (rxs->window_samples - rxs->slide_samples) * sizeof(int16_t));
        rxs->window_filled = rxs->window_samples - rxs->slide_samples;
        if (rxs->iq_window_filled >= rxs->window_samples) {
            memmove(rxs->iq_window,
                    rxs->iq_window + rxs->slide_samples * 2,
                    (rxs->window_samples - rxs->slide_samples)
                        * 2 * sizeof(int16_t));
            rxs->iq_window_filled = rxs->window_samples - rxs->slide_samples;
        }
    }
    return (int) n;
}

static void worker_update_snapshot(rx_session_t *rxs)
{
    double peak = 0.0, rms_sq = 0.0;
    b210_rx_tx_core_iq_levels(rxs->core, &peak, &rms_sq);
    int    burst_bins = 0;
    double burst_excess = 0.0;
    b210_rx_tx_core_burst_snapshot(rxs->core, &burst_bins, &burst_excess);
    // Effective downlink carrier as the operator thinks of it:
    //   hardware LO − lo_offset (recovers the nominal carrier)
    //              + software-Doppler NCO offset (tracks Doppler).
    // The first two sum to the nominal carrier (lo_offset_hz is the
    // SIGNED offset of the hardware LO from nominal, so subtracting it
    // gives nominal back); the third is the Doppler shift. Updates
    // smoothly every tick at Hz precision instead of stepping in kHz
    // like the old hardware-retune scheme.
    double freq = b210_rx_tx_core_actual_freq(rxs->core)
                - rxs->lo_offset_hz
                + b210_rx_tx_core_get_doppler_offset(rxs->core);
    int wav_active = (rxs->wav.fp != NULL);
    pthread_mutex_lock(&rxs->mu);
    rxs->snap_frames_total   = rxs->frames_total;
    rxs->snap_peak           = peak;
    rxs->snap_rms_sq         = rms_sq;
    rxs->snap_actual_freq_hz = freq;
    rxs->snap_burst_bright_bins    = burst_bins;
    rxs->snap_burst_peak_excess_db = burst_excess;
    rxs->snap_wav_active     = wav_active;
    rxs->snap_wav_n_samples  = (long) rxs->wav.n_samples;
    rxs->snap_iq_pairs       = (long) rxs->iq_pairs_written;
    rxs->snap_pcm_frames_total = rxs->pcm_frames_total;
    rxs->snap_vit_frames_total = rxs->vit_frames_total;
    // Persist the last-known paths even after a close so that the
    // end-of-pass renderer (which runs post-close) can still find them.
    if (rxs->wav_path[0]) {
        snprintf(rxs->snap_wav_path, sizeof rxs->snap_wav_path,
                 "%s", rxs->wav_path);
    }
    if (rxs->iq_path[0]) {
        snprintf(rxs->snap_iq_path, sizeof rxs->snap_iq_path,
                 "%s", rxs->iq_path);
    }
    snprintf(rxs->snap_last_frame_ts, sizeof rxs->snap_last_frame_ts,
             "%.*s", (int)(sizeof rxs->snap_last_frame_ts - 1),
             rxs->last_frame_ts);
    rxs->snap_last_frame_len = rxs->last_frame_len;
    rxs->snap_last_frame_monotonic_s = rxs->last_frame_monotonic_s;
    memcpy(rxs->snap_per_type_count, rxs->per_type_count,
           sizeof rxs->snap_per_type_count);
    memcpy(rxs->snap_per_type_last_len, rxs->per_type_last_len,
           sizeof rxs->snap_per_type_last_len);
    memcpy(rxs->snap_per_type_last_payload, rxs->per_type_last_payload,
           sizeof rxs->snap_per_type_last_payload);
    memcpy(rxs->snap_per_type_last_summary, rxs->per_type_last_summary,
           sizeof rxs->snap_per_type_last_summary);
    pthread_mutex_unlock(&rxs->mu);
}

static void *rx_session_thread_fn(void *arg)
{
    rx_session_t *rxs = arg;
    while (1) {
        // Take any pending requests off the queue.
        pthread_mutex_lock(&rxs->mu);
        int stop          = rxs->stop_requested;
        int freq_change   = rxs->freq_req_pending;
        double new_freq   = rxs->freq_req_hz;
        int do_wav_start  = rxs->wav_start_req;
        int do_wav_stop   = rxs->wav_stop_req;
        int do_burst      = rxs->burst_req_pending && !rxs->burst_complete;
        tx_request_slot_t burst_local;
        uint8_t           hmac_local[128];
        size_t            hmac_local_len = 0;
        if (do_burst) {
            burst_local    = rxs->burst_req;
            hmac_local_len = rxs->burst_hmac_key_len;
            if (hmac_local_len > 0)
                memcpy(hmac_local, rxs->burst_hmac_key, hmac_local_len);
        }
        rxs->freq_req_pending = 0;
        rxs->wav_start_req    = 0;
        rxs->wav_stop_req     = 0;
        pthread_mutex_unlock(&rxs->mu);

        if (stop) break;

        if (freq_change) {
            b210_rx_tx_core_set_freq(rxs->core, new_freq);
        }
        if (do_wav_start) worker_wav_start(rxs);
        if (do_wav_stop)  worker_wav_stop(rxs);

        if (do_burst) {
            char summary[160];
            tx_burst_result_t br = tx_burst_run(
                rxs->core, &burst_local,
                b210_rx_tx_core_actual_freq(rxs->core),
                hmac_local_len > 0 ? hmac_local : NULL, hmac_local_len,
                summary, sizeof summary);
            pthread_mutex_lock(&rxs->mu);
            rxs->burst_result = (rx_burst_result_t) br;
            snprintf(rxs->burst_summary, sizeof rxs->burst_summary,
                     "%s", summary);
            rxs->burst_complete    = 1;
            rxs->burst_req_pending = 0;
            pthread_cond_broadcast(&rxs->cv);
            pthread_mutex_unlock(&rxs->mu);
        }

        // Pump UHD. recv blocks ~chunk-cadence; that's our pacing.
        if (worker_pump_once(rxs) < 0) {
            fprintf(stderr, "rx_session: UHD pump fatal — worker exiting\n");
            break;
        }
        worker_update_snapshot(rxs);
    }
    return NULL;
}

void rx_session_snapshot(const rx_session_t *rxs,
                         uint64_t *out_frames_total,
                         double   *out_peak_dbfs,
                         double   *out_rms_dbfs,
                         double   *out_actual_freq_hz,
                         char     *out_last_summary, size_t summary_n)
{
    if (rxs == NULL) {
        if (out_frames_total)   *out_frames_total   = 0;
        if (out_peak_dbfs)      *out_peak_dbfs      = -90.0;
        if (out_rms_dbfs)       *out_rms_dbfs       = -90.0;
        if (out_actual_freq_hz) *out_actual_freq_hz = 0.0;
        if (out_last_summary && summary_n) out_last_summary[0] = '\0';
        return;
    }
    pthread_mutex_lock((pthread_mutex_t *) &rxs->mu);
    uint64_t frames = rxs->snap_frames_total;
    double   peak   = rxs->snap_peak;
    double   rms_sq = rxs->snap_rms_sq;
    double   freq   = rxs->snap_actual_freq_hz;
    char     ts[24]; int len;
    snprintf(ts, sizeof ts, "%s", rxs->snap_last_frame_ts);
    len = rxs->snap_last_frame_len;
    pthread_mutex_unlock((pthread_mutex_t *) &rxs->mu);

    if (out_frames_total) *out_frames_total = frames;
    if (out_actual_freq_hz) *out_actual_freq_hz = freq;
    if (out_peak_dbfs) {
        *out_peak_dbfs = (peak < 1.0) ? -90.0 : 20.0 * log10(peak / 32768.0);
    }
    if (out_rms_dbfs) {
        double rms = sqrt(rms_sq);
        *out_rms_dbfs = (rms < 1.0) ? -90.0 : 20.0 * log10(rms / 32768.0);
    }
    if (out_last_summary && summary_n) {
        if (ts[0]) snprintf(out_last_summary, summary_n,
                            "%s  %d bytes", ts, len);
        else       out_last_summary[0] = '\0';
    }
}

void rx_session_burst_snapshot(const rx_session_t *rxs,
                               int    *out_bright_bins,
                               double *out_peak_excess_db)
{
    if (rxs == NULL) {
        if (out_bright_bins)    *out_bright_bins    = 0;
        if (out_peak_excess_db) *out_peak_excess_db = 0.0;
        return;
    }
    pthread_mutex_lock((pthread_mutex_t *) &rxs->mu);
    int    bins   = rxs->snap_burst_bright_bins;
    double excess = rxs->snap_burst_peak_excess_db;
    pthread_mutex_unlock((pthread_mutex_t *) &rxs->mu);
    if (out_bright_bins)    *out_bright_bins    = bins;
    if (out_peak_excess_db) *out_peak_excess_db = excess;
}

void rx_session_update_observer(rx_session_t *rxs,
                                double az_deg, double el_deg,
                                double range_km, double range_rate_km_s,
                                double doppler_offset_hz)
{
    (void) rxs;
    decode_loop_set_observer(az_deg, el_deg, range_km, range_rate_km_s,
                              doppler_offset_hz);
}

void rx_session_stats_snapshot(const rx_session_t *rxs,
                               rx_packet_type_stats_t out_stats[],
                               double *out_seconds_since_last_frame)
{
    if (rxs == NULL) {
        if (out_stats) {
            memset(out_stats, 0,
                   sizeof(rx_packet_type_stats_t) * RX_PT_COUNT);
        }
        if (out_seconds_since_last_frame) {
            *out_seconds_since_last_frame = -1.0;
        }
        return;
    }
    pthread_mutex_lock((pthread_mutex_t *)&rxs->mu);
    double last_mono = rxs->snap_last_frame_monotonic_s;
    if (out_stats) {
        for (int i = 0; i < RX_PT_COUNT; ++i) {
            out_stats[i].count = rxs->snap_per_type_count[i];
            out_stats[i].last_payload_len = rxs->snap_per_type_last_len[i];
            memcpy(out_stats[i].last_payload,
                   rxs->snap_per_type_last_payload[i],
                   RX_LAST_PAYLOAD_MAX);
            memcpy(out_stats[i].last_summary,
                   rxs->snap_per_type_last_summary[i],
                   RX_LAST_SUMMARY_MAX);
        }
    }
    pthread_mutex_unlock((pthread_mutex_t *)&rxs->mu);
    if (out_seconds_since_last_frame) {
        if (last_mono == 0.0) {
            *out_seconds_since_last_frame = -1.0;
        } else {
            struct timespec now;
            if (clock_gettime(CLOCK_MONOTONIC, &now) == 0) {
                double now_s = (double) now.tv_sec
                             + (double) now.tv_nsec * 1e-9;
                *out_seconds_since_last_frame = now_s - last_mono;
            } else {
                *out_seconds_since_last_frame = -1.0;
            }
        }
    }
}

const char *rx_packet_type_label(rx_packet_type_slot_t slot)
{
    switch (slot) {
        case RX_PT_BEACON_BASIC:      return "beacon";
        case RX_PT_BEACON_PERIPHERAL: return "periph";
        case RX_PT_LOG_MESSAGE:       return "log";
        case RX_PT_TCMD_RESPONSE:     return "tcmd";
        case RX_PT_BULK_FILE:         return "bulk";
        case RX_PT_OTHER:             return "other";
        default:                      return "?";
    }
}
