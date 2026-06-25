/*

    Simple Satellite Operations  rx_session.c

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

// rx_session.c — see header. Lifted from utils/b210_rx_tx.c (the
// daemon-mode receiver) when simple_sat_ops absorbed B210 ownership.

#include "rx_session.h"

#include "agenda_line.h"
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
    // The RIFF/data chunk sizes are 32-bit, so a WAV can describe at most
    // ~4 GiB. At 96 kHz mono int16 that is ~6.2 hours of audio; a longer
    // pass overruns the header fields. Compute in 64-bit and clamp to the
    // maximum representable value rather than letting the size silently wrap
    // to a tiny number (which would make players truncate playback). The
    // captured audio is all on disk regardless; only the advertised size
    // saturates.
    uint64_t bytes = (uint64_t) w->n_samples * 2u;
    uint32_t data_sz = (bytes > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t) bytes;
    uint32_t riff_sz = (bytes + 36u > 0xFFFFFFFFu) ? 0xFFFFFFFFu
                                                   : (uint32_t)(bytes + 36u);
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
    // Raw IQ tap from the pump (carrier at +lo_offset baseband). This
    // is what gets written to the .iq sidecar.
    int16_t *iq_chunk;   // 2 * max_chunk int16: interleaved I,Q pairs
    // Decode-path IQ tap from the pump (carrier at DC). Fills the
    // sliding window the shadow IQ + Viterbi decoders consume so they
    // see the same shape the FM discriminator does.
    int16_t *iq_decode_chunk;
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

    // Live-audio tap. When enabled, the worker copies each pump's PCM into
    // this ring (under mu) alongside the WAV append; the operator main loop
    // drains it with rx_session_read_audio and feeds the per-subscriber Ogg
    // encoders. Off by default — zero cost when no viewer is listening.
    int16_t *audio_ring;
    size_t   audio_ring_cap;    // capacity in samples
    size_t   audio_ring_head;   // next write index
    size_t   audio_ring_count;  // samples currently buffered
    int      audio_tap_on;
    uint64_t audio_dropped;     // samples dropped on overflow (diagnostic)

    // --- Threading ---
    // The worker thread owns the RX side of `core`: it runs the UHD RX
    // pump + decode + wav writer continuously and never blocks on TX.
    // A separate tx_thread runs each TX burst (the B210 is full-duplex:
    // RX keeps streaming on the worker while TX bursts on its own port),
    // so a transmit never starves the RX USB transport. Main thread
    // interacts via the request flags and the snapshot, both under `mu`.
    b210_rx_tx_core_t *core;
    pthread_t          thread;
    int                thread_started;
    pthread_t          tx_thread;
    int                tx_thread_started;
    pthread_mutex_t    mu;
    pthread_cond_t     cv;
    volatile int       stop_requested;
    // Set by the worker when the RX pump returns a fatal error (device
    // unplugged / transport dead). The worker then parks and the main
    // thread surfaces a TUI warning; the session object stays alive so
    // the operator can still quit cleanly.
    volatile int       device_lost;

    // Requests from main thread (set under mu, picked up by worker).
    int     freq_req_pending;
    double  freq_req_hz;
    int     gain_req_pending;
    double  gain_req_db;
    int     wav_start_req;
    int     wav_stop_req;

    // TX burst handoff. burst_req_pending: queued, not yet picked up.
    // burst_in_flight: the tx_thread is transmitting. burst_complete:
    // finished, result not yet consumed.
    int                burst_req_pending;
    int                burst_in_flight;
    int                burst_complete;
    tx_request_slot_t  burst_req;
    uint8_t            burst_hmac_key[128];
    size_t             burst_hmac_key_len;
    rx_burst_result_t  burst_result;
    // 256 to match tx_request_slot_t.summary / the IPC SSO_TX_TEXT_MAX field:
    // an expanded "SSO+..." command plus its " (replaced 'SSO+...')" heritage
    // note can exceed 160.
    char               burst_summary[256];

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
    int64_t  snap_wav_n_samples;
    char     snap_iq_path[512];
    int64_t  snap_iq_pairs;
    uint64_t snap_pcm_frames_total;
    uint64_t snap_vit_frames_total;

    // lo_offset.csv sidecar: same lifecycle as doppler.csv (opens with
    // the WAV, closes with it). One row per change of lo_offset_hz so
    // offline reprocessing of the .iq can replay the exact LO history.
    FILE     *lo_offset_fp;
    char      lo_offset_path[512];

    // burst.csv sidecar: wideband-burst events. Same lifecycle as the
    // doppler/lo_offset CSVs. Lets the operator A/B the waterfall
    // against "what the detector thought looked like a packet" — a
    // foundation for gating the decoder on bursts later. State below
    // is a small debouncer so a brief mid-burst dropout doesn't split
    // one beacon into two rows.
    FILE     *burst_fp;
    char      burst_path[512];
    int       burst_in_progress;
    int       burst_bins_threshold;   // min bright_bins to start a burst
    int       burst_quiet_frames;     // consecutive frames < threshold
    int       burst_min_quiet;        // debounce: frames to declare "end"
    long long burst_start_unix_ms;
    int       burst_peak_bins;
    double    burst_peak_excess_db;
};

static void *rx_session_thread_fn(void *arg);
static void *rx_session_tx_thread_fn(void *arg);

int rx_session_open(rx_session_t **out, const rx_session_params_t *p,
                    b210_rx_tx_core_t *core)
{
    if (out == NULL || p == NULL || core == NULL) return -1;
    *out = NULL;

    rx_session_t *rxs = calloc(1, sizeof(*rxs));
    if (rxs == NULL) return -1;

    modem_params_defaults(&rxs->mp);
    rxs->mp.bit_rate  = p->bit_rate > 0 ? p->bit_rate : 9600;
    // Propagate the operator's no_dc_block choice to the PCM/FM-audio
    // demod (used by the shadow PCM chain). The IQ + Viterbi chains
    // don't have an HPF anyway — they read complex IQ directly. Default
    // 0 keeps the HPF on, matching rx_replay's default.
    rxs->mp.rx_disable_dc_block = p->no_dc_block ? 1 : 0;

    // Burst detector thresholds for the burst.csv writer. ~16 lit FFT
    // bins is "wideband packet" territory at our 96 kHz / 512-FFT
    // setup (bin width 187.5 Hz; a 12 kHz GFSK lights up ~64 bins).
    // burst_min_quiet=5 debounces a beacon that briefly drops below
    // threshold mid-packet. Both can be tuned with sat-side data once
    // we see how the log lines up with the waterfall.
    rxs->burst_bins_threshold = 16;
    rxs->burst_min_quiet      = 5;
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
    // The downlink carries no HMAC (the AX100 downlink frame is
    // authenticated by its CSP CRC32 trailer, not an HMAC), so the
    // decoder never installs an HMAC key — see opts.hmac_key left NULL.
    rxs->sync_max_ham = p->sync_max_ham > 0 ? p->sync_max_ham : 4;
    rxs->force_beacon = p->force_beacon;

    decode_loop_set_show_headers(p->show_packet_headers);

    rxs->max_chunk = b210_rx_tx_core_max_chunk(core);
    if (rxs->max_chunk == 0) rxs->max_chunk = 2040;
    rxs->pcm_chunk        = malloc(rxs->max_chunk * sizeof(int16_t));
    rxs->iq_chunk         = malloc(rxs->max_chunk * 2 * sizeof(int16_t));
    rxs->iq_decode_chunk  = malloc(rxs->max_chunk * 2 * sizeof(int16_t));
    rxs->window           = malloc(rxs->window_samples * sizeof(int16_t));
    rxs->iq_window        = malloc(rxs->window_samples * 2 * sizeof(int16_t));
    rxs->bits_cap         = rxs->window_samples + 8;
    rxs->bits_scratch     = malloc(rxs->bits_cap);
    rxs->bytes_cap        = rxs->bits_cap / 8 + 1;
    rxs->bytes_scratch    = malloc(rxs->bytes_cap);
    // Live-audio ring: ~2 s at the post-decim rate, enough slack between
    // the worker's pump cadence and the operator's audio-drain cadence.
    rxs->audio_ring_cap   = (size_t) rxs->samp_rate * 2;
    rxs->audio_ring       = malloc(rxs->audio_ring_cap * sizeof(int16_t));
    if (!rxs->pcm_chunk || !rxs->iq_chunk || !rxs->iq_decode_chunk
        || !rxs->window || !rxs->iq_window
        || !rxs->bits_scratch || !rxs->bytes_scratch || !rxs->audio_ring) {
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
    // pumps UHD RX continuously and services freq retunes / WAV
    // start-stops queued by the main thread; a separate tx_thread runs
    // TX bursts so a transmit never blocks the RX pump.
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

    if (pthread_create(&rxs->tx_thread, NULL, rx_session_tx_thread_fn, rxs) != 0) {
        fprintf(stderr, "rx_session: TX pthread_create failed\n");
        // Unwind the worker thread cleanly, then the mutex/cond.
        pthread_mutex_lock(&rxs->mu);
        rxs->stop_requested = 1;
        pthread_cond_broadcast(&rxs->cv);
        pthread_mutex_unlock(&rxs->mu);
        pthread_join(rxs->thread, NULL);
        rxs->thread_started = 0;
        pthread_cond_destroy (&rxs->cv);
        pthread_mutex_destroy(&rxs->mu);
        rxs->core = NULL;
        rx_session_close(rxs);
        return -1;
    }
    rxs->tx_thread_started = 1;

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

    // burst.csv sidecar. Same naming convention. Detector lives in
    // b210_rx_tx_core (iq_burst), we just persist the snapshots.
    if (wlen >= 4 && strcmp(rxs->wav_path + wlen - 4, ".wav") == 0) {
        int base_len = (int)(wlen - 4);
        if (base_len > 501) base_len = 501;
        snprintf(rxs->burst_path, sizeof rxs->burst_path,
                 "%.*s.burst.csv", base_len, rxs->wav_path);
    } else {
        snprintf(rxs->burst_path, sizeof rxs->burst_path,
                 "%.501s.burst.csv", rxs->wav_path);
    }
    rxs->burst_fp = fopen(rxs->burst_path, "w");
    if (rxs->burst_fp != NULL) {
        fputs("# event,unix_time_ms,bright_bins,peak_excess_db,"
              "duration_ms\n",
              rxs->burst_fp);
        fflush(rxs->burst_fp);
    } else {
        rxs->burst_path[0] = '\0';
    }
    rxs->burst_in_progress    = 0;
    rxs->burst_quiet_frames   = 0;
    rxs->burst_peak_bins      = 0;
    rxs->burst_peak_excess_db = 0.0;

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
    if (rxs->burst_fp != NULL) {
        // Close any in-flight burst as an "end" row so a clean exit
        // doesn't lose the last event. Duration is current_time - start.
        if (rxs->burst_in_progress) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            long long now_ms =
                (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000;
            fprintf(rxs->burst_fp,
                "burst_end,%lld,%d,%.2f,%lld\n",
                now_ms, rxs->burst_peak_bins,
                rxs->burst_peak_excess_db,
                now_ms - rxs->burst_start_unix_ms);
            fflush(rxs->burst_fp);
            rxs->burst_in_progress = 0;
        }
        fclose(rxs->burst_fp);
        rxs->burst_fp = NULL;
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
                             int64_t  *out_n_samples,
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
                            int64_t *out_pairs,
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

void rx_session_set_gain(rx_session_t *rxs, double gain_db)
{
    if (rxs == NULL) return;
    // Same worker-thread handoff as freq retunes — the UHD streamer
    // lives on the worker, so we queue the change and wake it.
    pthread_mutex_lock(&rxs->mu);
    rxs->gain_req_pending = 1;
    rxs->gain_req_db      = gain_db;
    pthread_cond_broadcast(&rxs->cv);
    pthread_mutex_unlock(&rxs->mu);
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
    //
    // lo_offset_hz is read by the worker in worker_update_snapshot, so
    // assign it under mu. Held only for the assignment: the core call and
    // request_freq below must run unlocked (request_freq takes mu itself).
    pthread_mutex_lock(&rxs->mu);
    rxs->lo_offset_hz = new_lo_offset_hz;
    pthread_mutex_unlock(&rxs->mu);
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

int rx_session_can_tx(const rx_session_t *rxs)
{
    if (rxs == NULL || rxs->core == NULL) return 0;
    return b210_rx_tx_core_can_tx(rxs->core);
}

// True once the RX pump has hit a fatal error (device unplugged /
// transport dead). The worker sets device_lost under mu, so read it under
// mu too -- one discipline for every concurrent access rather than leaning
// on volatile, which gives no memory-ordering guarantee. (The close-path
// read is post-join and stays lock-free.)
int rx_session_device_lost(const rx_session_t *rxs)
{
    if (rxs == NULL) return 0;
    pthread_mutex_lock((pthread_mutex_t *) &rxs->mu);
    int lost = rxs->device_lost;
    pthread_mutex_unlock((pthread_mutex_t *) &rxs->mu);
    return lost;
}

const char *rx_session_sdr_name(const rx_session_t *rxs)
{
    if (rxs == NULL || rxs->core == NULL) return "";
    return b210_rx_tx_core_sdr_name(rxs->core);
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
    rx_burst_result_t res;
    if (rxs->burst_complete) {
        res = rxs->burst_result;
        if (out_summary && summary_n) {
            snprintf(out_summary, summary_n, "%s", rxs->burst_summary);
        }
    } else {
        // Woke on stop_requested before the burst ran. burst_result holds a
        // stale/zero value, so report the abort explicitly.
        res = RX_BURST_ABORTED;
        if (out_summary && summary_n) {
            snprintf(out_summary, summary_n, "aborted: session stopping");
        }
    }
    pthread_mutex_unlock(&rxs->mu);
    return res;
}

int rx_session_submit_burst(
    rx_session_t *rxs,
    const tx_request_slot_t *req,
    const uint8_t *hmac_key, size_t hmac_key_len)
{
    if (rxs == NULL || req == NULL) return -1;
    pthread_mutex_lock(&rxs->mu);
    // Refuse to overwrite a submission that is queued (burst_req_pending),
    // transmitting (burst_in_flight), or finished-but-unconsumed
    // (burst_complete). The tx_thread clears pending when it starts and
    // sets in_flight; the operator's poll clears complete. Any of the
    // three means a result the caller must reap first.
    if (rxs->burst_req_pending || rxs->burst_in_flight || rxs->burst_complete) {
        pthread_mutex_unlock(&rxs->mu);
        return -1;
    }
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
    pthread_mutex_unlock(&rxs->mu);
    return 0;
}

int rx_session_poll_burst(
    rx_session_t *rxs,
    rx_burst_result_t *out_result,
    char *out_summary, size_t summary_n)
{
    if (out_summary && summary_n) out_summary[0] = '\0';
    if (rxs == NULL || out_result == NULL) return -1;
    pthread_mutex_lock(&rxs->mu);
    if (!rxs->burst_complete) {
        pthread_mutex_unlock(&rxs->mu);
        return 0;
    }
    *out_result = rxs->burst_result;
    if (out_summary && summary_n) {
        snprintf(out_summary, summary_n, "%s", rxs->burst_summary);
    }
    rxs->burst_complete = 0;   // consume; slot ready for next submission
    pthread_mutex_unlock(&rxs->mu);
    return 1;
}

void rx_session_close(rx_session_t *rxs)
{
    if (rxs == NULL) return;
    if (rxs->thread_started || rxs->tx_thread_started) {
        // Wake both the RX worker and the TX thread, then join both
        // before tearing down the lock they share. A TX burst in flight
        // finishes first (it doesn't poll stop_requested mid-burst), then
        // the tx_thread sees the stop and exits.
        pthread_mutex_lock(&rxs->mu);
        rxs->stop_requested = 1;
        pthread_cond_broadcast(&rxs->cv);
        pthread_mutex_unlock(&rxs->mu);
        if (rxs->thread_started) {
            pthread_join(rxs->thread, NULL);
            rxs->thread_started = 0;
        }
        if (rxs->tx_thread_started) {
            pthread_join(rxs->tx_thread, NULL);
            rxs->tx_thread_started = 0;
        }
        pthread_cond_destroy (&rxs->cv);
        pthread_mutex_destroy(&rxs->mu);
    }
    // Worker exited, so we own the wav/iq/core/db scratch outright.
    if (rxs->wav.fp) wav_w_close(&rxs->wav);
    if (rxs->iq_fp)  { fclose(rxs->iq_fp); rxs->iq_fp = NULL; }
    // After a device loss the SDR is gone, so closing it (stream stop +
    // device free) could fault on the dead device. Leak the handle — the
    // process is exiting anyway — rather than risk a crash on the way out.
    if (rxs->core && !rxs->device_lost) b210_rx_tx_core_close(rxs->core);
    if (rxs->db)     packet_db_close(rxs->db);
    free(rxs->pcm_chunk);
    free(rxs->iq_chunk);
    free(rxs->iq_decode_chunk);
    free(rxs->window);
    free(rxs->iq_window);
    free(rxs->bits_scratch);
    free(rxs->bytes_scratch);
    free(rxs->audio_ring);
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
                               rxs->sync_max_ham,
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
                                  rxs->sync_max_ham,
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
        // Always validate the AX100 downlink's CSP CRC32 trailer. A match
        // strips the 4 trailing bytes; a mismatch is recorded (crc_status=0)
        // but the frame is still kept, so low-SNR / partly-corrupted
        // telemetry stays visible rather than being silently dropped.
        if (plen >= 8) {
            crc_computed = csp_crc32c(rxs->packet, (size_t)(plen - 4));
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
                   golay_errs, hmac_ok,
                   rs_errs, used_golay_len,
                   crc_status, crc_computed, crc_le, crc_be,
                   rs_locs,
                   NULL, 0,
                   rxs->force_beacon);
        rxs->frames_total++;
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
                                       rxs->sync_max_ham,
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

// Push n PCM samples into the live-audio ring (caller is the worker; takes
// mu for the copy since the operator main loop drains under the same lock).
// On overflow the oldest samples are dropped — audio is best-effort and the
// downstream Vorbis decoder resyncs at the next page.
static void audio_ring_push(rx_session_t *rxs, const int16_t *pcm, size_t n)
{
    pthread_mutex_lock(&rxs->mu);
    if (rxs->audio_tap_on && rxs->audio_ring && rxs->audio_ring_cap > 0
        && n > 0) {
        size_t cap = rxs->audio_ring_cap;
        if (n > cap) {
            // A single chunk larger than the whole ring: keep only its tail.
            rxs->audio_dropped += (n - cap);
            pcm += (n - cap);
            n = cap;
        }
        for (size_t i = 0; i < n; ++i) {
            rxs->audio_ring[rxs->audio_ring_head] = pcm[i];
            rxs->audio_ring_head = (rxs->audio_ring_head + 1) % cap;
        }
        if (rxs->audio_ring_count + n > cap) {
            rxs->audio_dropped   += (rxs->audio_ring_count + n - cap);
            rxs->audio_ring_count = cap;  // head wrapped over the old tail
        } else {
            rxs->audio_ring_count += n;
        }
    }
    pthread_mutex_unlock(&rxs->mu);
}

void rx_session_set_audio_tap(rx_session_t *rxs, int on)
{
    if (rxs == NULL) return;
    pthread_mutex_lock(&rxs->mu);
    if (on && !rxs->audio_tap_on) {
        // Start fresh so a new listener doesn't inherit stale buffered audio.
        rxs->audio_ring_head  = 0;
        rxs->audio_ring_count = 0;
    }
    rxs->audio_tap_on = on ? 1 : 0;
    pthread_mutex_unlock(&rxs->mu);
}

size_t rx_session_read_audio(rx_session_t *rxs, int16_t *out, size_t max_samples)
{
    if (rxs == NULL || out == NULL || max_samples == 0) return 0;
    pthread_mutex_lock(&rxs->mu);
    size_t cap   = rxs->audio_ring_cap;
    size_t take  = rxs->audio_ring_count < max_samples
                 ? rxs->audio_ring_count : max_samples;
    if (cap > 0 && take > 0) {
        size_t tail = (rxs->audio_ring_head + cap - rxs->audio_ring_count) % cap;
        for (size_t i = 0; i < take; ++i) {
            out[i] = rxs->audio_ring[(tail + i) % cap];
        }
        rxs->audio_ring_count -= take;
    } else {
        take = 0;
    }
    pthread_mutex_unlock(&rxs->mu);
    return take;
}

static int worker_pump_once(rx_session_t *rxs)
{
    // Two IQ taps from the core: iq_chunk is raw post-Doppler IQ with
    // the carrier at +lo_offset baseband (gets written to the .iq
    // sidecar), iq_decode_chunk is the same stream after fm_lo_nco
    // brings the carrier to DC (feeds the shadow IQ + Viterbi decoders
    // below). iq_pairs may be less than n on the very first pump (the
    // discriminator emits a leading zero for the missing prev-sample) —
    // that's fine; we write only what the core actually delivered.
    size_t iq_pairs = 0;
    size_t iq_decode_pairs = 0;
    ssize_t n = b210_rx_tx_core_pump(rxs->core, rxs->pcm_chunk,
                                     rxs->max_chunk,
                                     rxs->iq_chunk,
                                     rxs->max_chunk * 2, &iq_pairs,
                                     rxs->iq_decode_chunk,
                                     rxs->max_chunk * 2,
                                     &iq_decode_pairs);
    if (n < 0) return -1;
    if (n == 0) return 0;
    if (rxs->wav.fp) wav_w_append(&rxs->wav, rxs->pcm_chunk, (size_t) n);
    // Live-audio relay: copy PCM into the ring when a viewer is listening.
    if (rxs->audio_tap_on) audio_ring_push(rxs, rxs->pcm_chunk, (size_t) n);
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

    // iq_window feeds the shadow IQ + Viterbi decoders, both of which
    // are calibrated for carrier-at-DC. Source bytes come from the
    // decode-path tap (post-fm_lo_nco), not the raw .iq tap.
    //
    // The core derives the PCM count (n) and iq_decode_pairs from the same
    // n_demod and clamps both identically, so iq_decode_pairs == n and the
    // PCM and IQ windows advance in lockstep -- the absolute-sample label
    // below (total_window_samples - window_samples) is valid for both. The
    // min() is a guard: if a future change ever makes iq_decode_pairs < n
    // the IQ window would lag the PCM one and that label would drift, so a
    // separate IQ-window sample counter would be needed then.
    size_t pairs_to_use = iq_decode_pairs;
    if (pairs_to_use > (size_t) n) pairs_to_use = (size_t) n;
    for (ssize_t i = 0; i < n; i++) {
        rxs->window[rxs->window_filled++] = rxs->pcm_chunk[i];
        if ((size_t) i < pairs_to_use
            && rxs->iq_window_filled < rxs->window_samples) {
            rxs->iq_window[rxs->iq_window_filled * 2 + 0] =
                rxs->iq_decode_chunk[i * 2 + 0];
            rxs->iq_window[rxs->iq_window_filled * 2 + 1] =
                rxs->iq_decode_chunk[i * 2 + 1];
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

// Coalesce raw iq_burst snapshots into burst-start / burst-end rows
// in the per-pass burst.csv. A burst opens when bright_bins crosses
// up through burst_bins_threshold and closes after burst_min_quiet
// consecutive snapshots below threshold (debounce, so a brief
// mid-packet noise dip doesn't split one beacon in two). Called once
// per pump iteration — sample cadence matches snapshot cadence.
static void burst_log_step(rx_session_t *rxs, int bins, double excess_db)
{
    if (rxs->burst_fp == NULL) return;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long now_ms = (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000;

    if (bins >= rxs->burst_bins_threshold) {
        if (!rxs->burst_in_progress) {
            rxs->burst_in_progress    = 1;
            rxs->burst_start_unix_ms  = now_ms;
            rxs->burst_peak_bins      = bins;
            rxs->burst_peak_excess_db = excess_db;
            rxs->burst_quiet_frames   = 0;
            fprintf(rxs->burst_fp,
                "burst_start,%lld,%d,%.2f,\n",
                now_ms, bins, excess_db);
            fflush(rxs->burst_fp);
        } else {
            if (bins > rxs->burst_peak_bins) rxs->burst_peak_bins = bins;
            if (excess_db > rxs->burst_peak_excess_db) {
                rxs->burst_peak_excess_db = excess_db;
            }
            rxs->burst_quiet_frames = 0;
        }
    } else if (rxs->burst_in_progress) {
        rxs->burst_quiet_frames++;
        if (rxs->burst_quiet_frames >= rxs->burst_min_quiet) {
            fprintf(rxs->burst_fp,
                "burst_end,%lld,%d,%.2f,%lld\n",
                now_ms, rxs->burst_peak_bins, rxs->burst_peak_excess_db,
                now_ms - rxs->burst_start_unix_ms);
            fflush(rxs->burst_fp);
            rxs->burst_in_progress    = 0;
            rxs->burst_quiet_frames   = 0;
            rxs->burst_peak_bins      = 0;
            rxs->burst_peak_excess_db = 0.0;
        }
    }
}

static void worker_update_snapshot(rx_session_t *rxs)
{
    double peak = 0.0, rms_sq = 0.0;
    b210_rx_tx_core_iq_levels(rxs->core, &peak, &rms_sq);
    int    burst_bins = 0;
    double burst_excess = 0.0;
    b210_rx_tx_core_burst_snapshot(rxs->core, &burst_bins, &burst_excess);
    burst_log_step(rxs, burst_bins, burst_excess);
    // Effective downlink carrier as the operator thinks of it:
    //   hardware LO − lo_offset (recovers the nominal carrier)
    //              + software-Doppler NCO offset (tracks Doppler).
    // The first two sum to the nominal carrier (lo_offset_hz is the
    // SIGNED offset of the hardware LO from nominal, so subtracting it
    // gives nominal back); the third is the Doppler shift. Updates
    // smoothly every tick at Hz precision instead of stepping in kHz
    // like the old hardware-retune scheme.
    double core_freq = b210_rx_tx_core_actual_freq(rxs->core);
    double doppler   = b210_rx_tx_core_get_doppler_offset(rxs->core);
    int wav_active = (rxs->wav.fp != NULL);
    pthread_mutex_lock(&rxs->mu);
    // lo_offset_hz is written by the main thread under mu; read it here
    // inside the lock and finish the carrier math.
    double freq = core_freq - rxs->lo_offset_hz + doppler;
    rxs->snap_frames_total   = rxs->frames_total;
    rxs->snap_peak           = peak;
    rxs->snap_rms_sq         = rms_sq;
    rxs->snap_actual_freq_hz = freq;
    rxs->snap_burst_bright_bins    = burst_bins;
    rxs->snap_burst_peak_excess_db = burst_excess;
    rxs->snap_wav_active     = wav_active;
    rxs->snap_wav_n_samples  = (int64_t) rxs->wav.n_samples;
    rxs->snap_iq_pairs       = (int64_t) rxs->iq_pairs_written;
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

// After a successful on-air burst, record the transmitted telecommand so
// a later tcmd_response can be tied back to the command (and arguments)
// that produced it. Only commands carrying an @tssent directive are
// recorded: that value is what the satellite echoes back in its response,
// so it is the join key. Manual-compose commands carry no @tssent and are
// left unrecorded; auto-telecommand agenda lines always carry one.
//
// Runs on the TX thread. The received-packet inserts run on the RX worker
// thread, but they use a different prepared statement, and SQLite's
// serialized threading mode guards the shared connection.
static void worker_record_sent_tcmd(rx_session_t *rxs,
                                     const tx_request_slot_t *req)
{
    if (rxs->db == NULL || req->is_hex) return;
    size_t n = req->payload_len;
    if (n == 0) return;
    char cmd[sizeof req->payload + 1];
    if (n >= sizeof req->payload) n = sizeof req->payload - 1;
    memcpy(cmd, req->payload, n);
    cmd[n] = '\0';

    long long ts_sent_ms = -1;
    if (!agenda_parse_directive_ms(cmd, "@tssent=", &ts_sent_ms)) return;
    long long tsexec_ms = -1;
    agenda_parse_directive_ms(cmd, "@tsexec=", &tsexec_ms);

    char ts_tx[40];
    fmt_utc(ts_tx, sizeof ts_tx);

    sent_tcmd_record_t rec = {0};
    rec.ts_sent_ms     = ts_sent_ms;
    rec.tsexec_ms      = tsexec_ms;   // -1 -> stored NULL
    rec.command_text   = cmd;
    rec.tx_freq_hz     = req->tx_freq_hz;
    rec.tx_gain_db     = req->tx_gain_db;
    rec.source_tool    = "simple_sat_ops";
    rec.source_run     = rxs->db_run_id;
    rec.ts_transmitted = ts_tx;
    packet_db_insert_sent_tcmd(rxs->db, &rec);
}

// Dedicated TX-burst thread. Sleeps until a burst is queued, runs it
// (tx_burst_run maps/tunes/sends/unmaps the TX chain) while the RX
// worker keeps streaming on its own thread, then publishes the result.
// Keeping TX off the RX worker is what lets RX run continuously across a
// transmit instead of starving the USB transport mid-burst.
static void *rx_session_tx_thread_fn(void *arg)
{
    rx_session_t *rxs = arg;
    for (;;) {
        pthread_mutex_lock(&rxs->mu);
        while (!rxs->burst_req_pending && !rxs->stop_requested) {
            pthread_cond_wait(&rxs->cv, &rxs->mu);
        }
        if (rxs->stop_requested) {
            pthread_mutex_unlock(&rxs->mu);
            break;
        }
        tx_request_slot_t req = rxs->burst_req;
        uint8_t           hmac_local[128];
        size_t            hmac_local_len = rxs->burst_hmac_key_len;
        if (hmac_local_len > 0)
            memcpy(hmac_local, rxs->burst_hmac_key, hmac_local_len);
        int lost = rxs->device_lost;
        rxs->burst_req_pending = 0;
        rxs->burst_in_flight   = 1;
        pthread_mutex_unlock(&rxs->mu);

        tx_burst_result_t br;
        char summary[256] = "";
        if (lost) {
            // Device is gone — don't touch it (would risk a second
            // fault). Refuse the burst cleanly.
            br = TX_BURST_NO_CORE;
            snprintf(summary, sizeof summary, "rejected: SDR disconnected");
        } else {
            // rx_resume_freq is unused now (RX is never paused), so pass
            // 0.0 rather than read the worker-owned actual_freq cross-thread.
            br = tx_burst_run(
                rxs->core, &req, /*rx_resume_freq_hz=*/0.0,
                hmac_local_len > 0 ? hmac_local : NULL, hmac_local_len,
                summary, sizeof summary);
        }

        if (br == TX_BURST_OK) worker_record_sent_tcmd(rxs, &req);

        pthread_mutex_lock(&rxs->mu);
        rxs->burst_result = (rx_burst_result_t) br;
        snprintf(rxs->burst_summary, sizeof rxs->burst_summary, "%s", summary);
        rxs->burst_in_flight = 0;
        rxs->burst_complete  = 1;
        pthread_cond_broadcast(&rxs->cv);
        pthread_mutex_unlock(&rxs->mu);
    }
    return NULL;
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
        int gain_change   = rxs->gain_req_pending;
        double new_gain   = rxs->gain_req_db;
        int do_wav_start  = rxs->wav_start_req;
        int do_wav_stop   = rxs->wav_stop_req;
        // TX bursts are handled by the tx_thread, NOT here — the worker
        // must never block on a transmit or the RX transport starves.
        rxs->freq_req_pending = 0;
        rxs->gain_req_pending = 0;
        rxs->wav_start_req    = 0;
        rxs->wav_stop_req     = 0;
        pthread_mutex_unlock(&rxs->mu);

        if (stop) break;

        if (freq_change) {
            b210_rx_tx_core_set_freq(rxs->core, new_freq);
        }
        if (gain_change) {
            b210_rx_tx_core_set_gain(rxs->core, new_gain);
        }
        if (do_wav_start) worker_wav_start(rxs);
        if (do_wav_stop)  worker_wav_stop(rxs);

        // Pump UHD. recv blocks ~chunk-cadence; that's our pacing.
        if (worker_pump_once(rxs) < 0) {
            // Fatal RX error (device unplugged / transport dead). Flag it
            // so the main thread warns in the TUI, then stop pumping. The
            // session object stays alive — the UI keeps running and the
            // operator can quit cleanly.
            pthread_mutex_lock(&rxs->mu);
            rxs->device_lost = 1;
            pthread_mutex_unlock(&rxs->mu);
            fprintf(stderr, "rx_session: RX device lost — worker parked\n");
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
