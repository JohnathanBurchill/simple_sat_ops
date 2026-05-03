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

#include "csp.h"

#include <stdio.h>
#include <string.h>

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
                      size_t *out_sync_off)
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
                                     out_rs_errs, out_used_golay_len);
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
                                           &p_rs, &p_lensrc);
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

void emit_frame(const char *log_path, int quiet, const char *ts,
                const uint8_t *packet, size_t packet_len,
                int golay_errs, int hmac_ok, int use_hmac,
                int rs_errs, int used_golay_len,
                int crc_status,
                uint32_t crc_computed, uint32_t crc_le, uint32_t crc_be)
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

    const char *len_src =
        used_golay_len == 1 ? "golay-header"
        : used_golay_len == 0 ? "brute-forced" : "(n/a)";
    for (int s = 0; s < 2; s++) {
        FILE *fp = streams[s];
        if (fp == NULL) continue;
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
        // CRC notice (only when --csp-crc32 was active for this frame).
        // Mismatch never drops the frame; the trailer stays in payload
        // so the operator can inspect what was received.
        if (crc_status == 1) {
            fprintf(fp, "[%s] csp_crc32: ok (0x%08x, 4-byte trailer "
                    "stripped)\n", ts, crc_computed);
        } else if (crc_status == 0) {
            fprintf(fp, "[%s] csp_crc32: MISMATCH "
                    "(computed=0x%08x, trailer LE=0x%08x BE=0x%08x; "
                    "trailer kept in payload)\n",
                    ts, crc_computed, crc_le, crc_be);
        }
        if (csp_ok) {
            fprintf(fp, "[%s] CSP v1: src=%u dst=%u dport=%u sport=%u "
                    "prio=%u flags=0x%02x\n",
                    ts, hdr.src, hdr.dst, hdr.dport, hdr.sport, hdr.prio,
                    hdr.flags);
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
        fflush(fp);
    }

    if (log_fp != NULL) fclose(log_fp);
}
