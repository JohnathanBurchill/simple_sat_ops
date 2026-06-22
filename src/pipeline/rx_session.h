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

// Values 0..3 mirror tx_burst.h's tx_burst_result_t so the worker can cast
// a tx_burst result straight across. RX_BURST_ABORTED is RX-only (never cast
// from a tx_burst result): the sync submit path returns it when the session
// is stopping before the burst ran, so the caller never reads a stale result.
typedef enum {
    RX_BURST_OK = 0,
    RX_BURST_NO_CORE,
    RX_BURST_FRAME_BUILD_FAILED,
    RX_BURST_UHD_ERROR,
    RX_BURST_ABORTED,
} rx_burst_result_t;

typedef struct {
    int    bit_rate;            // default 9600
    double window_s;            // default 1.5
    double slide_s;             // default 0.5
    int    sync_max_ham;        // default 4
    int    use_rs;
    int    no_dc_block;         // 0 = DC block on (default)
    int    force_beacon;
    int    show_packet_headers;
    const char    *db_path;
    int            no_db;
    const char    *pass_folder; // base dir for WAV / log; NULL = cwd
    int            want_wav;    // 1 = honour rx_session_wav_start; 0 = stub
    const char    *tle_path;
    const char    *sat_name;
    const char    *session_dir; // for packet_db (typically pass_folder)
    // LO offset (Hz) below the satellite's nominal carrier — what main
    // also stored in state->rx_lo_offset_hz. The hardware LO is already
    // tuned to (nominal - this) when the core is handed off; rx_session
    // needs the same value so the snapshot can reconstruct the effective
    // (Doppler-shifted) carrier frequency for the operator panel.
    double         lo_offset_hz;
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

// Update the hardware LO offset at runtime. Retunes the SDR LO to
// nominal_freq_hz + new_lo_offset_hz and updates the FM-path
// compensation NCO so the discriminator still sees the carrier at
// DC. Brief glitch in the IQ stream while the AD9361 PLL settles
// (~tens of ms); use for occasional manual nudges, NOT continuous
// tracking.
void rx_session_set_lo_offset(rx_session_t *rxs,
                              double nominal_freq_hz,
                              double new_lo_offset_hz);

// Set the software-Doppler NCO offset (Hz, relative to the LO). Lock-
// free — the NCO is single-double atomic on all targets we care about,
// and the pump reads it once per chunk. Phase continues across calls,
// so a smooth Doppler trajectory yields a phase-coherent baseband
// signal. Pass 0.0 to disable correction entirely.
void rx_session_set_doppler_offset(rx_session_t *rxs, double offset_hz);

// Change the AD9361 RX gain at runtime. Routed through the worker
// thread (same handoff pattern as freq retunes) so the UHD streamer
// isn't touched from the caller's thread. Brief noise discontinuity
// in the IQ stream while the gain table swaps stages, but the stream
// keeps running.
void rx_session_set_gain(rx_session_t *rxs, double gain_db);

// Latest NCO offset (Hz). Used by the operator UI to display the
// effective downlink frequency (= nominal + offset).
double rx_session_get_doppler_offset(const rx_session_t *rxs);

// Hardware SDR LO (Hz) — i.e. nominal_downlink_freq − lo_offset_hz.
// Useful for the operator panel to show where the SDR is actually
// listening alongside the Doppler-shifted carrier.
double rx_session_get_lo_freq_hz(const rx_session_t *rxs);

// Post-decimation sample rate (Hz) = captured bandwidth.
double rx_session_get_bandwidth_hz(const rx_session_t *rxs);

// 1 if the underlying SDR backend can transmit, 0 if it is RX-only
// (e.g. an RTL-SDR) or there is no session. The operator UI gates the
// TX compose / auto-telecommand paths on this. Reads a static
// capability set at open, so it is safe without the worker lock.
int rx_session_can_tx(const rx_session_t *rxs);

// True once the RX pump has hit a fatal error (device unplugged or the
// transport died). The session stays alive so the UI can warn and the
// operator can quit cleanly; the worker has parked.
int rx_session_device_lost(const rx_session_t *rxs);

// Friendly name of the active SDR ("USRP B210", "RTL-SDR ...") for the
// operator banner. Returns "" with no session. Static at open.
const char *rx_session_sdr_name(const rx_session_t *rxs);

// Async: ask the worker to start recording the post-FIR PCM to a
// fresh auto-named WAV under the configured pass folder. No-op if
// already recording.
void rx_session_request_wav_start(rx_session_t *rxs);

// Async: close the currently-recording WAV. No-op when not recording.
void rx_session_request_wav_stop(rx_session_t *rxs);

// 1 when a WAV is currently being written, 0 otherwise. (Snapshot.)
int  rx_session_wav_active(const rx_session_t *rxs);

// Live-audio tap for the viewer relay. With it on, the worker copies each
// pump's demodulated PCM into an internal ring; off (the default) it costs
// nothing. Toggling on discards any stale buffered audio. The PCM is mono
// int16 at rx_session_get_bandwidth_hz().
void   rx_session_set_audio_tap(rx_session_t *rxs, int on);

// Drain up to max_samples from the live-audio ring into out. Returns the
// number of samples copied (0 if the tap is off or nothing is buffered).
// Safe to call from the main thread; the worker fills the ring.
size_t rx_session_read_audio(rx_session_t *rxs, int16_t *out, size_t max_samples);

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

// Frame count from the shadow PCM/FM-audio demod (modem.c run on the
// post-FIR PCM stream). The IQ-domain chain is the live primary that
// writes the DB + drives the panel; PCM runs in parallel as a count-
// only A signal. rx_session_snapshot's frames_total is the IQ count.
uint64_t rx_session_pcm_frames(const rx_session_t *rxs);

// Frame count from the shadow MSK-MLSE Viterbi demod
// (modem_viterbi.c). Same role as rx_session_pcm_frames — a count-
// only B signal on the same IQ window the live IQ chain uses.
uint64_t rx_session_viterbi_frames(const rx_session_t *rxs);

// Sync: hand a TX burst request to the worker, block until it pauses
// RX, transmits, and resumes RX. Returns the burst's outcome plus a
// short one-line summary suitable for the operator's TX log.
// hmac_key may be NULL.
rx_burst_result_t rx_session_request_burst_sync(
    rx_session_t *rxs,
    const tx_request_slot_t *req,
    const uint8_t *hmac_key, size_t hmac_key_len,
    char *out_summary, size_t summary_n);

// Async split of the sync API. submit hands the request to the worker
// and returns immediately; poll is non-blocking and returns 1 when the
// burst has finished (filling out_result / out_summary), 0 while still
// in flight, -1 on argument error. submit returns 0 on accept, -1 if a
// prior submission is still pending. Lets the main loop keep servicing
// rotator / redraw / IPC / auto-tcmd while the worker runs the burst.
int rx_session_submit_burst(
    rx_session_t *rxs,
    const tx_request_slot_t *req,
    const uint8_t *hmac_key, size_t hmac_key_len);

int rx_session_poll_burst(
    rx_session_t *rxs,
    rx_burst_result_t *out_result,
    char *out_summary, size_t summary_n);

// Snapshot for the UI. Any out-pointer may be NULL.
// out_actual_freq_hz returns the EFFECTIVE downlink frequency, i.e.
// the SDR LO plus the current software-Doppler offset — the carrier
// the IQ stream is centred on after the in-pump NCO correction.
void rx_session_snapshot(const rx_session_t *rxs,
                         uint64_t *out_frames_total,
                         double   *out_peak_dbfs,
                         double   *out_rms_dbfs,
                         double   *out_actual_freq_hz,
                         char     *out_last_summary, size_t summary_n);

// Broadband-burst snapshot: distinguishes CW / Doppler-swept carrier
// (out_bright_bins ≈ 1-6) from a wideband packet (tens to hundreds).
// out_peak_excess_db is the brightest single bin's dB above its floor.
// Either pointer may be NULL.
void rx_session_burst_snapshot(const rx_session_t *rxs,
                               int    *out_bright_bins,
                               double *out_peak_excess_db);

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
#define RX_LAST_SUMMARY_MAX 160
typedef struct {
    uint64_t count;
    int      last_payload_len;
    uint8_t  last_payload[RX_LAST_PAYLOAD_MAX];
    // One-line decoded summary built by the rx_session worker when the
    // payload sniffs as a known FrontierSat packet type. Empty string
    // when no decode was possible — the operator-side renderer can
    // then fall back to a hex dump of last_payload[]. Saves the panel
    // from having to know the full beacon layout itself, and makes
    // the same string available to viewers over IPC.
    char     last_summary[RX_LAST_SUMMARY_MAX];
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
