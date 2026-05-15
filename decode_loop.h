/*

    Simple Satellite Operations  decode_loop.h

    Shared multi-frame AX100 decode helpers used by rx_live (ALSA-fed
    real-time decoder) and rx_replay (offline file-fed equivalent).
    The decode loop is identical in both — only the source of audio
    samples differs — so the window-walking, partial-RS rescue, and
    output formatting live here once and rx_live / rx_replay only
    drive sample acquisition.

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

#ifndef DECODE_LOOP_H
#define DECODE_LOOP_H

#include "ax100.h"
#include "csp.h"
#include "modem.h"

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

// Multi-hypothesis sync search on a single window of mono int16 samples.
// Same logic as rx_decode.c's main decode loop — try sync, on failure
// advance min_offset past the bad match and try again. With HMAC enabled,
// HMAC mismatch is treated as a false-positive sync (advance + retry).
// Returns 1 on a successful decode (packet/packet_len/* filled), 0 if no
// frame found. The caller may loop on this function with min_offset_in
// advanced past the previously-returned sync_off to extract multiple
// frames per window (e.g. when bursts arrive faster than the slide
// interval); *out_sync_off receives the offset of the accepted sync
// match so the caller can compute the next min_offset_in.
//
// rs_errs sentinels (*out_rs_errs):
//   >= 0  RS-corrected; value is the count of corrected byte errors
//   -1    RS off / not exercised
//   -2    RS uncorrectable, descrambled-but-uncorrected bytes returned
//         via packet/packet_len so the operator can still see the data
//         (only happens when allow_partial_rs is on and HMAC is off).
//
// out_rs_locs (optional): if non-NULL, must have at least 32 slots. On a
// successful RS decode the first *out_rs_errs entries are byte offsets
// of corrected bytes relative to the start of the on-wire scrambled
// payload (last 32 are RS parity tail; lower indices are data). Lets
// callers print where in each frame the errors landed so the operator
// can spot timing-drift signatures (tail-clustered) vs uniform BER.
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
                      int *out_rs_locs);

// Same contract as try_decode_window, but consumes interleaved int16
// I,Q pairs from `iq_pairs` (length n_pairs) and runs the demod through
// modem_iq_to_bits instead of modem_pcm16_to_bits. Used by rx_session as
// a shadow decoder so the operator can A/B the IQ-domain chain against
// the FM-audio chain on the same live RF.
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
                         int *out_rs_locs);

// Same contract again, but FM-discriminates the IQ first and then
// runs the same DC-block + AGC + MF + M&M + slicer pipeline that
// modem_pcm16 uses on FM-demod audio. Modulation-index-agnostic
// (works on h=0.5 MSK and the h~2/3 FSK FrontierSat actually
// transmits); usually the best primary chain on real captures.
int try_decode_window_fsk(const int16_t *iq_pairs, size_t n_pairs,
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
                          int *out_rs_locs);

// Same contract again, but runs the MSK-MLSE Viterbi
// (modem_iq_viterbi_to_bits) on the IQ window. Used by rx_session as a
// second shadow chain so we can A/B Viterbi vs differential slicer vs
// FM-audio on live RF.
int try_decode_window_viterbi(const int16_t *iq_pairs, size_t n_pairs,
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
                              int *out_rs_locs);

// Append-only log line writer. Re-opens the log per frame so log
// rotation works (mv the log mid-run, next frame creates a fresh
// file). Mirrored to stdout if not quiet. ts is the timestamp string
// the caller picks — UTC wall clock for rx_live, file-relative
// "t=NN.NNNs" for rx_replay — always wrapped in [...] in the output.
//
// rs_locs (optional, NULL ok): pointer to the on-wire byte offsets of
// the bytes RS corrected. When non-NULL and rs_errs > 0, an extra line
//   `[ts] rs_locs: corrected=N of M on-wire bytes: a b c ...`
// is emitted (M = packet_len + (use_hmac?4:0) + 32). The list helps the
// operator distinguish tail-clustered errors (clock drift) from
// scattered errors (channel noise).
//
// ref_buf / ref_len (optional, NULL/0 ok): a known-good reference to
// compare the decoded bytes against. If ref_len matches packet_len the
// diff runs over the full packet (positions reported as packet bytes);
// if ref_len matches the payload length (packet_len - 4) the diff runs
// over just the payload (positions reported as payload bytes). On a
// length mismatch the line reports the mismatch and skips. Useful when
// RS gave up (rs_errs == -2 / partial-RS rescue) and the operator has
// a clean copy of the same frame to diff against.
//
// force_beacon (non-zero): bypass the dispatch and always print the
// payload as a CTS1 basic beacon. Bytes below 130 are zero-padded;
// bytes beyond 130 are truncated. Useful for prying telemetry out of
// frames whose magic bytes / length got mangled. Affects text output
// only (the TUI keeps its own length-based routing).
void emit_frame(const char *log_path, int quiet, const char *ts,
                const uint8_t *packet, size_t packet_len,
                int golay_errs, int hmac_ok, int use_hmac,
                int rs_errs, int used_golay_len,
                int crc_status,
                uint32_t crc_computed, uint32_t crc_le, uint32_t crc_be,
                const int *rs_locs,
                const uint8_t *ref_buf, size_t ref_len,
                int force_beacon);

// Headers toggle. When OFF (the default for live/replay tools), emit_frame
// hides the AX100 framing line, the CSP v1 header line, the rs_locs and
// ref_diff lines, the csp_crc32-OK confirmation, and the per-frame hex /
// ascii dumps. The interpreted body lines (beacon: / tcmd_response: /
// log: / bulk_file:) and error conditions (HMAC mismatch, csp_crc32
// mismatch, rs=UNCORRECTABLE) are always shown either way.
//
// rx_decode (offline forensic CLI) defaults this ON; rx_live, rx_replay,
// b210_rx_tx default it OFF and expose --packet-headers / `packetheaders
// on` to flip it. Flag is process-global because emit_frame is called
// from many sites and threading the bool through every call site would
// be churn for no benefit.
void decode_loop_set_show_headers(int on);
int  decode_loop_show_headers(void);

// Parse and apply a single REPL command. Returns 1 if recognised and
// handled, 0 if unknown. On success writes a one-line status into
// status_buf (truncated to cap) for the caller to surface via
// rx_tui_set_status / stderr / log. Recognised commands:
//   "packetheaders on"   / "packetheaders off"
//   "ph on"              / "ph off"
// Unknown input leaves status_buf empty and returns 0 so the caller can
// chain its own per-tool commands (sq, spectrum, force-beacon, ...).
int decode_loop_try_command(const char *cmd, char *status_buf, size_t cap);

// Forward decl so callers don't need to include packet_db.h here.
typedef struct packet_db packet_db_t;

// Plug a packet_db handle into emit_frame and decode_loop_record_packet.
// After this is called with a non-NULL `db`, every successfully decoded
// frame routed through either entry point produces one row in the DB
// (deduplicated by payload SHA-1 + source_tool + source_run, so
// rerunning the same capture is harmless). Pass db=NULL to disable
// (e.g. when --no-db was given). source_tool / source_run strings must
// remain valid for the rest of the process; they are not copied.
void decode_loop_set_packet_db(packet_db_t *db,
                               const char *source_tool,
                               const char *source_run);

// Record one decoded packet into the configured packet DB. No-op when
// no DB is set or when the payload doesn't match a recognised firmware
// packet type. Used by emit_frame internally and by rx_decode's
// duplicate output path; callers that already use emit_frame don't
// need to call this directly.
//
// ts: caller's timestamp string. ISO-8601 UTC is preferred and stored
// verbatim in ts_received. A "t=NN.NNNs" string from rx_replay's
// relative-clock mode is parsed into audio_offset_s, with ts_received
// getting wall-clock-now so the column stays sortable.
void decode_loop_record_packet(const char *ts,
                               const csp_v1_header_t *hdr, int csp_ok,
                               const uint8_t *payload, size_t payload_len,
                               int golay_errs, int hmac_ok,
                               int rs_errs, int crc_status);

// Observer-frame state for the next packets to be recorded. Pass NaN
// for any unknown component. Initial state (before the first call) is
// all-NaN, so receivers that don't track the satellite (rx_live,
// rx_decode, replay-without-TLE) leave the DB columns NULL.
void decode_loop_set_observer(double az_deg, double el_deg,
                              double range_km, double range_rate_km_s,
                              double doppler_hz_offset);

// Tag subsequent records with this TLE row id (returned by
// packet_db_register_tle). Pass 0 to clear.
void decode_loop_set_tle_id(long long tle_id);

// Tag subsequent records with this session directory (where the run's
// WAV / log / spectrogram live). Pointer is borrowed and must remain
// valid for the rest of the process. Pass NULL to clear.
void decode_loop_set_session_dir(const char *path);

// Tag subsequent records with the audio's provenance: "cts_ground"
// for our B210 captures, "satnogs" for a SatNOGS archive .ogg.
// Distinct from source_tool, which identifies the decoder. Pointer
// is borrowed; pass NULL to clear back to "unknown" (column NULL).
void decode_loop_set_capture_origin(const char *origin);

// Anchor for "t=NN.NNNs" relative timestamps. When set (Unix seconds,
// UTC), decode_loop_record_packet computes ts_received as
// (anchor + offset_s) so rows from a re-decoded WAV carry the actual
// transmission time rather than the wall-clock moment of re-decode.
// rx_replay calls this with start_utc parsed from --start-utc, the
// "UT=YYYYMMDDTHHMMSS.sss" stamp in the WAV filename, or the file
// mtime (in that order). Pass NaN to clear and revert to the
// wall-clock-now fallback.
void decode_loop_set_audio_clock_anchor(double unix_seconds);

#endif // DECODE_LOOP_H
