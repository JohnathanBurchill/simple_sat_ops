// rx_session.c — see header. Lifted from utils/b210_rx_tx.c (the
// daemon-mode receiver) when simple_sat_ops absorbed B210 ownership.

#include "rx_session.h"

#include "ax100.h"
#include "b210_rx_tx_core.h"
#include "csp.h"
#include "decode_loop.h"
#include "modem.h"
#include "packet_db.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
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

    // Cached sample-rate-dependent sizes.
    int      samp_rate;
    int      sps;
    size_t   window_samples;
    size_t   slide_samples;
    uint64_t dedup_quant;

    // Allocated scratch.
    int16_t *pcm_chunk;
    size_t   max_chunk;
    int16_t *window;
    size_t   window_filled;
    uint8_t *bits_scratch;
    size_t   bits_cap;
    uint8_t *bytes_scratch;
    size_t   bytes_cap;
    uint8_t  packet[4200];

    // Dedup ring (quantised ASM absolute sample index).
    uint64_t recent_pos_quant[DEDUP_RING_SZ];
    int      recent_idx;
    int      recent_count;
    uint64_t total_window_samples;

    // Output paths.
    wav_w_t  wav;
    char     wav_path[512];
    char     log_path[512];

    // Frame counters + last-decoded summary.
    uint64_t frames_total;
    unsigned frames_in_window;
    char     last_frame_ts[24];
    int      last_frame_len;

    // packet_db handle (owned).
    packet_db_t *db;
    char         db_run_id[24];
};

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
    rxs->pcm_chunk   = malloc(rxs->max_chunk * sizeof(int16_t));
    rxs->window      = malloc(rxs->window_samples * sizeof(int16_t));
    rxs->bits_cap    = rxs->window_samples + 8;
    rxs->bits_scratch  = malloc(rxs->bits_cap);
    rxs->bytes_cap   = rxs->bits_cap / 8 + 1;
    rxs->bytes_scratch = malloc(rxs->bytes_cap);
    if (!rxs->pcm_chunk || !rxs->window
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

    // WAV in pass folder (or cwd).
    if (p->want_wav) {
        if (auto_name_wav(p->pass_folder,
                          rxs->wav_path, sizeof rxs->wav_path) != 0
            || wav_w_open(&rxs->wav, rxs->wav_path, rxs->samp_rate) != 0) {
            fprintf(stderr,
                "rx_session: WAV open failed (%s): %s — continuing without WAV\n",
                rxs->wav_path, strerror(errno));
            rxs->wav_path[0] = '\0';
        }
    }

    *out = rxs;
    return 0;
}

void rx_session_close(rx_session_t *rxs)
{
    if (rxs == NULL) return;
    if (rxs->wav.fp) wav_w_close(&rxs->wav);
    if (rxs->db) packet_db_close(rxs->db);
    free(rxs->pcm_chunk);
    free(rxs->window);
    free(rxs->bits_scratch);
    free(rxs->bytes_scratch);
    free(rxs);
}

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

        // Dedup by quantised absolute ASM sample index.
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
    }
}

int rx_session_pump(rx_session_t *rxs, b210_rx_tx_core_t *core,
                    double budget_s)
{
    if (rxs == NULL || core == NULL) return -1;
    double t_start = monotonic_seconds();
    for (;;) {
        ssize_t n = b210_rx_tx_core_pump(core, rxs->pcm_chunk, rxs->max_chunk);
        if (n < 0) return -1;
        if (n == 0) break;
        if (rxs->wav.fp) wav_w_append(&rxs->wav, rxs->pcm_chunk, (size_t) n);

        // Slide the decode window across the pump's PCM, calling
        // try_decode_at_window each time it fills.
        for (ssize_t i = 0; i < n; i++) {
            rxs->window[rxs->window_filled++] = rxs->pcm_chunk[i];
            rxs->total_window_samples++;
            if (rxs->window_filled < rxs->window_samples) continue;
            try_decode_at_window(rxs);
            memmove(rxs->window, rxs->window + rxs->slide_samples,
                    (rxs->window_samples - rxs->slide_samples)
                        * sizeof(int16_t));
            rxs->window_filled = rxs->window_samples - rxs->slide_samples;
        }
        if (budget_s > 0.0 && monotonic_seconds() - t_start >= budget_s) break;
    }
    return 0;
}

void rx_session_snapshot(const rx_session_t *rxs,
                         uint64_t *out_frames_total,
                         double   *out_peak_dbfs,
                         double   *out_rms_dbfs,
                         char     *out_last_summary, size_t summary_n)
{
    if (rxs == NULL) {
        if (out_frames_total) *out_frames_total = 0;
        if (out_peak_dbfs)    *out_peak_dbfs    = -90.0;
        if (out_rms_dbfs)     *out_rms_dbfs     = -90.0;
        if (out_last_summary && summary_n) out_last_summary[0] = '\0';
        return;
    }
    if (out_frames_total) *out_frames_total = rxs->frames_total;
    if (out_last_summary && summary_n) {
        if (rxs->last_frame_ts[0]) {
            snprintf(out_last_summary, summary_n,
                     "%s  %d bytes",
                     rxs->last_frame_ts, rxs->last_frame_len);
        } else {
            out_last_summary[0] = '\0';
        }
    }
    // Level meter — tap b210_rx_tx_core_iq_levels via a snapshot helper.
    // We don't have a core pointer here, so the caller fills those in.
    if (out_peak_dbfs) *out_peak_dbfs = -90.0;
    if (out_rms_dbfs)  *out_rms_dbfs  = -90.0;
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
