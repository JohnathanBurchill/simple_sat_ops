// rx_session.h — encapsulates the live B210 RX → decode pipeline so
// simple_sat_ops/main.c stays readable. Owns the WAV writer, the
// sliding-window AX100 decoder, the dedup ring, and the running
// frame-count + level-meter snapshot the operator UI renders.
//
// Lifted from utils/b210_rx_tx.c when the daemon was folded into the
// operator binary. The interface is deliberately narrow: open / pump /
// snapshot / update-observer / close.

#ifndef RX_SESSION_H
#define RX_SESSION_H

#include "tx_burst.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct b210_rx_tx_core;
typedef struct b210_rx_tx_core b210_rx_tx_core_t;

typedef struct rx_session rx_session_t;

// Mirrors tx_burst.h's tx_burst_result_t so callers don't need that
// header just to interpret the result.
typedef enum {
    RX_BURST_OK = 0,
    RX_BURST_NO_CORE,
    RX_BURST_FRAME_BUILD_FAILED,
    RX_BURST_UHD_ERROR,
} rx_burst_result_t;

typedef struct {
    int    bit_rate;            // default 9600
    double window_s;            // default 1.5
    double slide_s;             // default 0.5
    int    sync_max_ham;        // default 4
    int    use_hmac;
    int    use_rs;
    int    no_dc_block;         // 0 = DC block on (default)
    int    force_beacon;
    int    show_packet_headers;
    int    csp_crc32;
    const uint8_t *hmac_key;
    size_t         hmac_key_len;
    const char    *db_path;
    int            no_db;
    const char    *pass_folder; // base dir for WAV / log; NULL = cwd
    int            want_wav;    // 1 = honour rx_session_wav_start; 0 = stub
    const char    *tle_path;
    const char    *sat_name;
    const char    *session_dir; // for packet_db (typically pass_folder)
} rx_session_params_t;

// rx_session takes ownership of `core` and spawns a worker thread that
// owns the UHD device end-to-end. The main thread interacts via the
// request-* helpers below; it must not touch `core` directly afterwards.
int  rx_session_open(rx_session_t **out,
                     const rx_session_params_t *p,
                     b210_rx_tx_core_t *core);
void rx_session_close(rx_session_t *rxs);

// Async: queue a centre-freq retune. Picked up on the next worker
// iteration (sub-10 ms typically). Fire-and-forget.
void rx_session_request_freq(rx_session_t *rxs, double freq_hz);

// Async: ask the worker to start recording the post-FIR PCM to a
// fresh auto-named WAV under the configured pass folder. No-op if
// already recording.
void rx_session_request_wav_start(rx_session_t *rxs);

// Async: close the currently-recording WAV. No-op when not recording.
void rx_session_request_wav_stop(rx_session_t *rxs);

// 1 when a WAV is currently being written, 0 otherwise. (Snapshot.)
int  rx_session_wav_active(const rx_session_t *rxs);

// Snapshot the live WAV's path, written-sample count, and sample rate
// so an external worker can render a spectrogram off the bytes already
// on disk. out_active reflects whether the writer is currently open.
// The path remains valid (last-opened) after a wav_stop, which lets
// the end-of-pass renderer use it after rx_session_close.
//
// Any out-pointer may be NULL.
void rx_session_wav_snapshot(const rx_session_t *rxs,
                             char     *out_path, size_t path_cap,
                             long     *out_n_samples,
                             int      *out_sample_rate,
                             int      *out_active);

// Companion snapshot for the IQ sidecar (raw interleaved int16 I,Q at
// the post-decim sample rate). Pairs counts complex samples written.
// Same path-persistence semantics as wav_snapshot.
void rx_session_iq_snapshot(const rx_session_t *rxs,
                            char *out_path, size_t path_cap,
                            long *out_pairs,
                            int  *out_sample_rate);

// Frame count from the shadow IQ-domain demod (modem_iq.c). Runs in
// parallel with the primary PCM demod purely as a measurement. Use
// alongside rx_session_snapshot's frames_total to A/B the two chains.
uint64_t rx_session_iq_frames(const rx_session_t *rxs);

// Sync: hand a TX burst request to the worker, block until it pauses
// RX, transmits, and resumes RX. Returns the burst's outcome plus a
// short one-line summary suitable for the operator's TX log.
// hmac_key may be NULL.
rx_burst_result_t rx_session_request_burst_sync(
    rx_session_t *rxs,
    const tx_request_slot_t *req,
    const uint8_t *hmac_key, size_t hmac_key_len,
    char *out_summary, size_t summary_n);

// Snapshot for the UI. Any out-pointer may be NULL.
void rx_session_snapshot(const rx_session_t *rxs,
                         uint64_t *out_frames_total,
                         double   *out_peak_dbfs,
                         double   *out_rms_dbfs,
                         double   *out_actual_freq_hz,
                         char     *out_last_summary, size_t summary_n);

// Packet-type buckets that rx_session keeps live counters for. Matches
// the COMMS_PACKET_TYPE_* constants in beacon_cts1.h (low byte of the
// CSP payload). Anything we don't recognise lands in RX_PT_OTHER.
typedef enum {
    RX_PT_BEACON_BASIC = 0,
    RX_PT_BEACON_PERIPHERAL,
    RX_PT_LOG_MESSAGE,
    RX_PT_TCMD_RESPONSE,
    RX_PT_BULK_FILE,
    RX_PT_OTHER,
    RX_PT_COUNT,
} rx_packet_type_slot_t;

// One slot's worth of stats, returned by rx_session_stats_snapshot.
// last_payload holds the first last_payload_len bytes of the most
// recent packet that landed in this slot; the byte limit (64) is
// enough for a hex preview and short enough to keep the snapshot
// copy cheap. last_payload_len is the on-wire length stored, NOT
// the original frame length (so reading past last_payload_len is
// undefined).
#define RX_LAST_PAYLOAD_MAX 64
typedef struct {
    uint64_t count;
    int      last_payload_len;
    uint8_t  last_payload[RX_LAST_PAYLOAD_MAX];
} rx_packet_type_stats_t;

// Per-type stats snapshot + monotonic time of the last frame in any
// slot. out_seconds_since_last_frame is negative when no frame has
// arrived yet (or rxs is NULL). out_stats[] must point at an array
// of RX_PT_COUNT entries.
void rx_session_stats_snapshot(const rx_session_t *rxs,
                               rx_packet_type_stats_t out_stats[],
                               double *out_seconds_since_last_frame);
// Human-readable label for a packet-type slot ("beacon", "log", ...).
const char *rx_packet_type_label(rx_packet_type_slot_t slot);

// Tell the decoder where the satellite was at the last Doppler tick so
// per-frame DB rows carry geometry. Safe to call at any cadence.
void rx_session_update_observer(rx_session_t *rxs,
                                double az_deg, double el_deg,
                                double range_km, double range_rate_km_s,
                                double doppler_offset_hz);

#ifdef __cplusplus
}
#endif

#endif // RX_SESSION_H
