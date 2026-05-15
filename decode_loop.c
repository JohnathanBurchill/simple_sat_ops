/*

    Simple Satellite Operations  decode_loop.c

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

#include "decode_loop.h"

#include "beacon_cts1.h"
#include "csp.h"
#include "modem_iq.h"
#include "packet_db.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Process-global because emit_frame is called from rx_live, rx_replay,
// b210_rx_tx, and rx_decode. The alternative — a parameter on
// emit_frame — would touch every caller site for no real gain. The flag
// is read-mostly (set once at startup or via a REPL command) so the
// no-locking single-int approach is fine.
static int g_show_headers = 0;

// Optional packet-DB tap. NULL when no DB is configured (the default —
// rx_decode without --db, or any receiver run with --no-db). Strings
// are borrowed, not copied, so callers must keep them alive for the
// process lifetime — typically static run-id buffers in main().
static packet_db_t *g_packet_db = NULL;
static const char  *g_db_source_tool = NULL;
static const char  *g_db_source_run  = NULL;

// Observer-frame state pushed in by the receiver each time it changes
// (typically: once per Doppler tick from b210_rx_tx; rx_replay sets
// it per-packet during backfill). NaN means "not known". Other
// receivers never call the setter; they read NaN and the DB row gets
// NULL in those columns.
static double g_obs_az_deg            = (0.0 / 0.0);
static double g_obs_el_deg            = (0.0 / 0.0);
static double g_obs_range_km          = (0.0 / 0.0);
static double g_obs_range_rate_km_s   = (0.0 / 0.0);
static double g_obs_doppler_hz_offset = (0.0 / 0.0);
static long long g_obs_tle_id         = 0;
static const char *g_obs_session_dir  = NULL;
// Provenance of the audio under decode. Set once at startup by the
// receiver (e.g. rx_replay --capture-origin=satnogs). NULL means
// "not supplied"; the DB column stays NULL for those rows.
static const char *g_obs_capture_origin = NULL;

// Absolute-UTC anchor for "t=NN.NNNs" relative timestamps. NaN means
// no anchor; record_packet then falls back to wall-clock-now for
// ts_received (the legacy behaviour, kept so receivers that never
// resolve a UT base — e.g. rx_decode on a stripped WAV — still
// produce a sortable timestamp).
static double g_audio_anchor_unix     = (0.0 / 0.0);

void decode_loop_set_audio_clock_anchor(double unix_seconds)
{
    g_audio_anchor_unix = unix_seconds;
}

void decode_loop_set_packet_db(packet_db_t *db,
                               const char *source_tool,
                               const char *source_run)
{
    g_packet_db = db;
    g_db_source_tool = source_tool;
    g_db_source_run  = source_run;
}

void decode_loop_set_observer(double az_deg, double el_deg,
                              double range_km, double range_rate_km_s,
                              double doppler_hz_offset)
{
    g_obs_az_deg            = az_deg;
    g_obs_el_deg            = el_deg;
    g_obs_range_km          = range_km;
    g_obs_range_rate_km_s   = range_rate_km_s;
    g_obs_doppler_hz_offset = doppler_hz_offset;
}

void decode_loop_set_tle_id(long long tle_id)
{
    g_obs_tle_id = tle_id;
}

void decode_loop_set_session_dir(const char *path)
{
    // String is borrowed; caller keeps it alive (typically a static
    // buffer or argv pointer in the receiver's main()).
    g_obs_session_dir = path;
}

void decode_loop_set_capture_origin(const char *origin)
{
    // Borrowed pointer; caller must keep the string alive for the
    // process lifetime (typically argv or a static buffer).
    g_obs_capture_origin = origin;
}

void decode_loop_set_show_headers(int on)
{
    g_show_headers = on ? 1 : 0;
}

int decode_loop_show_headers(void)
{
    return g_show_headers;
}

static const char *skip_ws(const char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    return s;
}

int decode_loop_try_command(const char *cmd, char *status_buf, size_t cap)
{
    if (cap > 0 && status_buf != NULL) status_buf[0] = '\0';
    if (cmd == NULL) return 0;
    const char *p = skip_ws(cmd);
    // "packetheaders" or shorthand "ph"
    const char *rest = NULL;
    if (strncmp(p, "packetheaders", 13) == 0
        && (p[13] == ' ' || p[13] == '\t')) {
        rest = p + 13;
    } else if (strncmp(p, "ph", 2) == 0
               && (p[2] == ' ' || p[2] == '\t')) {
        rest = p + 2;
    } else {
        return 0;
    }
    rest = skip_ws(rest);
    if (strcmp(rest, "on") == 0) {
        decode_loop_set_show_headers(1);
        if (status_buf != NULL && cap > 0) {
            snprintf(status_buf, cap, "packetheaders: on");
        }
        return 1;
    }
    if (strcmp(rest, "off") == 0) {
        decode_loop_set_show_headers(0);
        if (status_buf != NULL && cap > 0) {
            snprintf(status_buf, cap, "packetheaders: off");
        }
        return 1;
    }
    if (status_buf != NULL && cap > 0) {
        snprintf(status_buf, cap,
                 "packetheaders: usage `packetheaders on|off`");
    }
    // Returned 1 because we recognised the verb — caller shouldn't
    // fall through and try its own parse; we already wrote a usage hint.
    return 1;
}

int try_decode_window(const int16_t *samples, size_t n_samples,
                      const modem_params_t *mp,
                      const ax100_opts_t *opts,
                      int sync_max_ham, int use_hmac,
                      int allow_partial_rs,
                      size_t min_offset_in,
                      uint8_t *bits_scratch, size_t bits_cap,
                      uint8_t *bytes_scratch, size_t bytes_cap,
                      uint8_t *packet, size_t packet_cap,
                      ssize_t *out_packet_len,
                      int *out_golay_errs, int *out_hmac_ok,
                      int *out_rs_errs, int *out_used_golay_len,
                      size_t *out_sync_off,
                      int *out_rs_locs)
{
    (void)bits_cap;  // sized by the caller; we honour bytes_cap explicitly
    // Was 32; bumped to 256 because the cross-phase best-Hamming search
    // can flounder through many low-HD noise candidates between real
    // frames in a long-packet window before finding the next real ASM.
    const int MAX_ATTEMPTS = 256;
    size_t min_offset = min_offset_in;
    int attempts = 0;

    while (attempts < MAX_ATTEMPTS) {
        size_t n_bits = 0, sync_off = 0;
        int polarity_used = -1;
        int rc = modem_pcm16_to_bits(samples, n_samples, mp,
                                     0, sync_max_ham, min_offset,
                                     bits_scratch, &n_bits,
                                     &sync_off, &polarity_used);
        if (rc != 0) break;
        ++attempts;
        if (n_bits == 0) {
            min_offset = sync_off + 1;
            continue;
        }
        size_t need_bytes = (n_bits + 7) / 8;
        if (need_bytes > bytes_cap) {
            // Too much data after sync — clamp by advancing past this match
            // rather than overflowing scratch.
            min_offset = sync_off + 1;
            continue;
        }
        size_t n_bytes = modem_bits_to_bytes(bits_scratch, n_bits,
                                             bytes_scratch);
        ssize_t plen = ax100_unframe(bytes_scratch, n_bytes, opts,
                                     packet, packet_cap,
                                     out_golay_errs, out_hmac_ok,
                                     out_rs_errs, out_used_golay_len,
                                     out_rs_locs);
        (void)polarity_used;
        if (plen < 0) {
            // Partial-decode rescue: RS(255,223) gives up at >16 byte
            // errors, but for long bursts on a marginal link the
            // descrambled bytes are still mostly readable. When the
            // Golay header decoded perfectly (errs==0) we trust the
            // sync was real; retry with RS disabled to recover the
            // uncorrected payload, mark it rs=-2 (UNCORRECTABLE), and
            // hand it up so the operator sees "Happy birthday..." with
            // a few flipped bytes instead of nothing at all. Off in
            // HMAC mode (no integrity gate would mean we'd emit
            // garbage at every false sync hit) and never overrides a
            // good RS decode.
            if (allow_partial_rs && !use_hmac && opts->reed_solomon
                && *out_golay_errs == 0) {
                ax100_opts_t partial_opts = *opts;
                partial_opts.reed_solomon = 0;
                int p_golay = 0, p_hmac = -1, p_rs = -1, p_lensrc = -1;
                ssize_t pp = ax100_unframe(bytes_scratch, n_bytes,
                                           &partial_opts,
                                           packet, packet_cap,
                                           &p_golay, &p_hmac,
                                           &p_rs, &p_lensrc,
                                           NULL);
                // Strip the trailing 32-byte RS parity (it's noise to
                // the operator and would push the apparent length up
                // by 32 vs. the as-transmitted packet). Anything <=32
                // would underflow; treat as no usable bytes.
                if (pp > 32) {
                    *out_packet_len = pp - 32;
                    *out_golay_errs = p_golay;
                    *out_hmac_ok = -1;
                    *out_rs_errs = -2;
                    *out_used_golay_len = p_lensrc;
                    if (out_sync_off) *out_sync_off = sync_off;
                    return 1;
                }
            }
            min_offset = sync_off + 1;
            continue;
        }
        if (use_hmac && *out_hmac_ok == 0) {
            min_offset = sync_off + 1;
            continue;
        }
        *out_packet_len = plen;
        if (out_sync_off) *out_sync_off = sync_off;
        return 1;
    }
    return 0;
}

int try_decode_window_iq(const int16_t *iq_pairs, size_t n_pairs,
                         const modem_params_t *mp,
                         const ax100_opts_t *opts,
                         int sync_max_ham, int use_hmac,
                         int allow_partial_rs,
                         size_t min_offset_in,
                         uint8_t *bits_scratch, size_t bits_cap,
                         uint8_t *bytes_scratch, size_t bytes_cap,
                         uint8_t *packet, size_t packet_cap,
                         ssize_t *out_packet_len,
                         int *out_golay_errs, int *out_hmac_ok,
                         int *out_rs_errs, int *out_used_golay_len,
                         size_t *out_sync_off,
                         int *out_rs_locs)
{
    (void) bits_cap;
    const int MAX_ATTEMPTS = 256;
    size_t min_offset = min_offset_in;
    int attempts = 0;

    while (attempts < MAX_ATTEMPTS) {
        size_t n_bits = 0, sync_off = 0;
        int polarity_used = -1;
        int rc = modem_iq_to_bits(iq_pairs, n_pairs, mp,
                                  0, sync_max_ham, min_offset,
                                  bits_scratch, &n_bits,
                                  &sync_off, &polarity_used);
        if (rc != 0) break;
        ++attempts;
        if (n_bits == 0) {
            min_offset = sync_off + 1;
            continue;
        }
        size_t need_bytes = (n_bits + 7) / 8;
        if (need_bytes > bytes_cap) {
            min_offset = sync_off + 1;
            continue;
        }
        size_t n_bytes = modem_bits_to_bytes(bits_scratch, n_bits,
                                             bytes_scratch);
        ssize_t plen = ax100_unframe(bytes_scratch, n_bytes, opts,
                                     packet, packet_cap,
                                     out_golay_errs, out_hmac_ok,
                                     out_rs_errs, out_used_golay_len,
                                     out_rs_locs);
        (void) polarity_used;
        if (plen < 0) {
            if (allow_partial_rs && !use_hmac && opts->reed_solomon
                && *out_golay_errs == 0) {
                ax100_opts_t partial_opts = *opts;
                partial_opts.reed_solomon = 0;
                int p_golay = 0, p_hmac = -1, p_rs = -1, p_lensrc = -1;
                ssize_t pp = ax100_unframe(bytes_scratch, n_bytes,
                                           &partial_opts,
                                           packet, packet_cap,
                                           &p_golay, &p_hmac,
                                           &p_rs, &p_lensrc,
                                           NULL);
                if (pp > 32) {
                    *out_packet_len = pp - 32;
                    *out_golay_errs = p_golay;
                    *out_hmac_ok = -1;
                    *out_rs_errs = -2;
                    *out_used_golay_len = p_lensrc;
                    if (out_sync_off) *out_sync_off = sync_off;
                    return 1;
                }
            }
            min_offset = sync_off + 1;
            continue;
        }
        if (use_hmac && *out_hmac_ok == 0) {
            min_offset = sync_off + 1;
            continue;
        }
        *out_packet_len = plen;
        if (out_sync_off) *out_sync_off = sync_off;
        return 1;
    }
    return 0;
}

void emit_frame(const char *log_path, int quiet, const char *ts,
                const uint8_t *packet, size_t packet_len,
                int golay_errs, int hmac_ok, int use_hmac,
                int rs_errs, int used_golay_len,
                int crc_status,
                uint32_t crc_computed, uint32_t crc_le, uint32_t crc_be,
                const int *rs_locs,
                const uint8_t *ref_buf, size_t ref_len,
                int force_beacon)
{
    char rs_buf[32];
    if (rs_errs == -2)      snprintf(rs_buf, sizeof rs_buf, "UNCORRECTABLE");
    else if (rs_errs < 0)   snprintf(rs_buf, sizeof rs_buf, "off");
    else                    snprintf(rs_buf, sizeof rs_buf, "corrected=%d", rs_errs);

    csp_v1_header_t hdr = {0};
    int csp_ok = (packet_len >= 4) && (csp_v1_decode(packet, &hdr) == 0);
    const uint8_t *payload = csp_ok ? packet + 4 : NULL;
    size_t payload_len = csp_ok ? packet_len - 4 : 0;

    FILE *streams[2] = { quiet ? NULL : stdout, NULL };
    FILE *log_fp = NULL;
    if (log_path != NULL) {
        log_fp = fopen(log_path, "a");
        if (log_fp != NULL) streams[1] = log_fp;
    }

    int show_headers = decode_loop_show_headers();
    int hmac_bad = use_hmac && hmac_ok == 0;
    int rs_bad = rs_errs == -2;
    // Always-show framing line when something went wrong — operator needs
    // the rs/hmac state even in terse mode. Otherwise gate on the toggle.
    int show_ax100 = show_headers || hmac_bad || rs_bad;

    const char *len_src =
        used_golay_len == 1 ? "golay-header"
        : used_golay_len == 0 ? "brute-forced" : "(n/a)";
    for (int s = 0; s < 2; s++) {
        FILE *fp = streams[s];
        if (fp == NULL) continue;
        if (show_ax100) {
            if (use_hmac) {
                fprintf(fp, "[%s] AX100: golay_errors=%d hmac=%s rs=%s len=%s "
                        "len_bytes=%zu\n",
                        ts, golay_errs,
                        hmac_ok == 1 ? "ok" : hmac_ok == 0 ? "MISMATCH" : "(off)",
                        rs_buf, len_src, packet_len);
            } else {
                fprintf(fp, "[%s] AX100: golay_errors=%d rs=%s len=%s "
                        "len_bytes=%zu\n",
                        ts, golay_errs, rs_buf, len_src, packet_len);
            }
        }
        // Per-byte error positions from the RS solver. Forensic detail —
        // gated on the toggle since terse-mode operators don't care.
        if (show_headers && rs_errs > 0 && rs_locs != NULL) {
            size_t on_wire_len = packet_len + (use_hmac ? 4 : 0) + 32;
            int sorted[32];
            int n = rs_errs > 32 ? 32 : rs_errs;
            for (int i = 0; i < n; ++i) sorted[i] = rs_locs[i];
            // Insertion sort — n <= 16 in practice, often < 8.
            for (int i = 1; i < n; ++i) {
                int v = sorted[i], j = i - 1;
                while (j >= 0 && sorted[j] > v) { sorted[j+1] = sorted[j]; --j; }
                sorted[j+1] = v;
            }
            fprintf(fp, "[%s] rs_locs: corrected=%d of %zu on-wire bytes:",
                    ts, rs_errs, on_wire_len);
            for (int i = 0; i < n; ++i) fprintf(fp, " %d", sorted[i]);
            fputc('\n', fp);
        }
        // Reference comparison: byte positions where the decoded frame
        // differs from a known-good reference. Useful when RS gave up
        // (rs_errs == -2) and we want to see where the bit errors
        // landed in the recovered (uncorrected) bytes.
        if (show_headers && ref_buf != NULL && ref_len > 0) {
            const uint8_t *cmp = NULL;
            size_t cmp_len = 0;
            const char *what = NULL;
            if (ref_len == packet_len) {
                cmp = packet; cmp_len = packet_len; what = "packet";
            } else if (csp_ok && ref_len == payload_len) {
                cmp = payload; cmp_len = payload_len; what = "payload";
            }
            if (cmp != NULL) {
                int diffs[256];
                int n_diff = 0;
                for (size_t i = 0; i < cmp_len; ++i) {
                    if (cmp[i] != ref_buf[i]) {
                        if (n_diff < (int)(sizeof diffs / sizeof diffs[0])) {
                            diffs[n_diff] = (int)i;
                        }
                        ++n_diff;
                    }
                }
                int shown = n_diff > (int)(sizeof diffs / sizeof diffs[0])
                    ? (int)(sizeof diffs / sizeof diffs[0]) : n_diff;
                fprintf(fp, "[%s] ref_diff: %d of %zu %s bytes differ:",
                        ts, n_diff, cmp_len, what);
                for (int i = 0; i < shown; ++i) fprintf(fp, " %d", diffs[i]);
                if (shown < n_diff) fprintf(fp, " ...");
                fputc('\n', fp);
            } else {
                fprintf(fp, "[%s] ref_diff: skipped (ref %zu bytes, "
                        "packet %zu, payload %zu)\n",
                        ts, ref_len, packet_len, payload_len);
            }
        }
        // CRC notice (only when --csp-crc32 was active for this frame).
        // Mismatch always prints — it's an error condition the operator
        // must see regardless of mode. The success line is gated.
        if (crc_status == 1 && show_headers) {
            fprintf(fp, "[%s] csp_crc32: ok (0x%08x, 4-byte trailer "
                    "stripped)\n", ts, crc_computed);
        } else if (crc_status == 0) {
            fprintf(fp, "[%s] csp_crc32: MISMATCH "
                    "(computed=0x%08x, trailer LE=0x%08x BE=0x%08x; "
                    "trailer kept in payload)\n",
                    ts, crc_computed, crc_le, crc_be);
        }
        if (csp_ok) {
            if (show_headers) {
                fprintf(fp, "[%s] CSP v1: src=%u dst=%u dport=%u sport=%u "
                        "prio=%u flags=0x%02x\n",
                        ts, hdr.src, hdr.dst, hdr.dport, hdr.sport, hdr.prio,
                        hdr.flags);
            }
            if (force_beacon) {
                // Build a 130-byte buffer: payload bytes first, zero-pad
                // the rest. Truncate if longer. Operator opt-in for
                // prying telemetry out of mangled frames.
                uint8_t buf[sizeof(COMMS_beacon_basic_packet_t)] = {0};
                size_t copy = payload_len < sizeof buf ? payload_len : sizeof buf;
                if (copy > 0) memcpy(buf, payload, copy);
                fprintf(fp,
                        "[%s] force_beacon: padded %zu->130 bytes "
                        "(zero-fill from byte %zu)\n",
                        ts, payload_len, copy);
                beacon_print(fp, ts, buf, sizeof buf);
            } else {
                cts1_packet_print(fp, ts, payload, payload_len);
            }
            if (show_headers) {
                fprintf(fp, "[%s] hex:   ", ts);
                for (size_t i = 0; i < payload_len; i++) {
                    fprintf(fp, "%02x", payload[i]);
                }
                fputc('\n', fp);
                fprintf(fp, "[%s] ascii: ", ts);
                for (size_t i = 0; i < payload_len; i++) {
                    char c = (payload[i] >= 0x20 && payload[i] < 0x7F)
                        ? (char)payload[i] : '.';
                    fputc(c, fp);
                }
                fputc('\n', fp);
            }
        }
        fflush(fp);
    }

    if (log_fp != NULL) fclose(log_fp);

    decode_loop_record_packet(ts, &hdr, csp_ok,
                              payload, payload_len,
                              golay_errs, hmac_ok, rs_errs, crc_status);
}

void decode_loop_record_packet(const char *ts,
                               const csp_v1_header_t *hdr, int csp_ok,
                               const uint8_t *payload, size_t payload_len,
                               int golay_errs, int hmac_ok,
                               int rs_errs, int crc_status)
{
    if (g_packet_db == NULL) return;
    if (!csp_ok || payload == NULL || payload_len == 0) return;

    int          ptype       = -1;
    const char  *ptype_name  = NULL;
    const char  *satellite   = NULL;
    if (beacon_is_basic(payload, payload_len)) {
        ptype = 0x01; ptype_name = "beacon"; satellite = "CTS1";
    } else if (tcmd_response_is(payload, payload_len)) {
        ptype = 0x04; ptype_name = "tcmd_response";
    } else if (log_message_is(payload, payload_len)) {
        ptype = 0x03; ptype_name = "log";
    } else if (bulk_file_is(payload, payload_len)) {
        ptype = 0x10; ptype_name = "bulk_file";
    }
    if (ptype_name == NULL) return;

    // Render the firmware-aware text into a stack buffer. 2 KiB is
    // more than enough for any of the four packet types (beacon's six
    // lines top out near 700 chars; tcmd_response adds ~200 for the
    // data preview).
    char summary_buf[2048];
    FILE *mem = fmemopen(summary_buf, sizeof summary_buf, "w");
    if (mem != NULL) {
        cts1_packet_print(mem, NULL, payload, payload_len);
        fflush(mem);
        long pos = ftell(mem);
        if (pos < 0) pos = 0;
        if ((size_t)pos >= sizeof summary_buf) pos = sizeof summary_buf - 1;
        summary_buf[pos] = '\0';
        fclose(mem);
    } else {
        summary_buf[0] = '\0';
    }

    // ts_received uses the ISO-8601 form when emit_frame's caller
    // produces one (rx_live / b210_rx_tx / rx_decode). rx_replay
    // passes a "t=NN.NNNs" relative offset; if an audio-clock anchor
    // is set, ts_received = (anchor + offset_s) so the column carries
    // the actual transmission UTC. Without an anchor, fall back to
    // wall-clock-now — sortable but loses the "when the satellite
    // really sent it" semantics.
    char ts_iso[40];
    const char *ts_for_db = ts;
    double offset_s = (0.0 / 0.0);  // NaN
    if (ts != NULL && strncmp(ts, "t=", 2) == 0) {
        char *endp = NULL;
        double v = strtod(ts + 2, &endp);
        if (endp != NULL && *endp == 's') {
            offset_s = v;
        }
        time_t epoch;
        long ms_long;
        if (!isnan(g_audio_anchor_unix) && !isnan(offset_s)) {
            double abs_t = g_audio_anchor_unix + offset_s;
            epoch = (time_t)floor(abs_t);
            double frac = abs_t - (double)epoch;
            ms_long = (long)(frac * 1000.0 + 0.5);
            // Round-up edge case: 999.5 ms rounds to 1000 → carry.
            if (ms_long >= 1000) { ms_long = 0; epoch += 1; }
        } else {
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);
            epoch = now.tv_sec;
            ms_long = (now.tv_nsec / 1000000) % 1000;
        }
        struct tm utc;
        gmtime_r(&epoch, &utc);
        // Mask each field into the range its format slot allows so
        // gcc -Wformat-truncation= can prove no overrun.
        int yr = (utc.tm_year + 1900) % 10000;
        int mo = (utc.tm_mon + 1) % 100;
        int da = utc.tm_mday % 100;
        int hh = utc.tm_hour % 100;
        int mm = utc.tm_min  % 100;
        int ss = utc.tm_sec  % 100;
        int ms = (int)(ms_long % 1000);
        snprintf(ts_iso, sizeof ts_iso,
                 "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                 yr, mo, da, hh, mm, ss, ms);
        ts_for_db = ts_iso;
    }

    packet_db_record_t rec = {
        .ts_received      = ts_for_db,
        .satellite        = satellite,
        .packet_type      = ptype,
        .packet_type_name = ptype_name,
        .csp_src          = hdr->src,
        .csp_dst          = hdr->dst,
        .csp_dport        = hdr->dport,
        .csp_sport        = hdr->sport,
        .csp_prio         = hdr->prio,
        .csp_flags        = hdr->flags,
        .csp_present      = 1,
        .payload          = payload,
        .payload_len      = payload_len,
        .golay_errs       = golay_errs,
        .rs_errs          = rs_errs,
        .hmac_ok          = hmac_ok,
        .crc_status       = crc_status,
        .source_tool      = g_db_source_tool,
        .source_run       = g_db_source_run,
        .audio_offset_s   = offset_s,
        .decoded_summary  = summary_buf[0] ? summary_buf : NULL,
        // Observer-frame state pulled from the setters. NaN sentinels
        // (the initial value when no setter has been called) map to
        // NULL in the DB.
        .az_deg            = g_obs_az_deg,
        .el_deg            = g_obs_el_deg,
        .range_km          = g_obs_range_km,
        .range_rate_km_s   = g_obs_range_rate_km_s,
        .doppler_hz_offset = g_obs_doppler_hz_offset,
        .tle_id            = g_obs_tle_id,
        .session_dir       = g_obs_session_dir,
        .capture_origin    = g_obs_capture_origin,
    };
    (void)packet_db_insert(g_packet_db, &rec);
}
