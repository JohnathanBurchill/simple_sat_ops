// tx_burst.h — in-process TX burst handler. Owns the CSP+AX100 frame
// build, the FM modulator, the envelope ramp, and the call into
// b210_rx_tx_core_burst that pauses RX, transmits, and resumes RX.
//
// Lifted from utils/b210_rx_tx.c's daemon TX-request handler when the
// daemon was folded into simple_sat_ops.

#ifndef TX_BURST_H
#define TX_BURST_H

#include "csp.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct b210_rx_tx_core;
typedef struct b210_rx_tx_core b210_rx_tx_core_t;

typedef struct {
    int      pending;          // 1 when tx_compose_commit has stashed work
    uint8_t  payload[2048];
    size_t   payload_len;
    int      is_hex;
    csp_v1_header_t csp_hdr;
    long     tx_freq_hz;
    double   tx_gain_db;
    int      repeat;
    int      gap_ms;
    // Modulated 0xAA carrier prepended in front of the AX100 frame, ms.
    // Acts as receiver bit-clock training and lets the B210 TX FIFO
    // buffer the burst before the real preamble. 0 or negative selects
    // the tx_burst_run default.
    int      preroll_ms;
    int      allow_high_power;
    int      allow_hf_tx;
    // One-line "ascii:..." / "auto[i/n j/m]: <cmd>" description shown in
    // the UI, logged to tx.log, and mirrored to viewers. Holds a full RF
    // telecommand (215 chars) plus its short label; kept >= the IPC
    // sso_event_t.ascii field (SSO_TX_TEXT_MAX) it is copied into.
    char     summary[256];
} tx_request_slot_t;

typedef enum {
    TX_BURST_OK = 0,
    TX_BURST_NO_CORE,
    TX_BURST_FRAME_BUILD_FAILED,
    TX_BURST_UHD_ERROR,
} tx_burst_result_t;

// Build CSP+AX100+IQ, then call b210_rx_tx_core_burst (which pauses RX,
// transmits, resumes RX at rx_resume_freq_hz). On success returns
// TX_BURST_OK and fills `out_summary` with a one-line "ascii:..." /
// "hex:..." description suitable for the UI / IPC fan-out.
tx_burst_result_t tx_burst_run(b210_rx_tx_core_t *core,
                                const tx_request_slot_t *req,
                                double rx_resume_freq_hz,
                                const uint8_t *hmac_key, size_t hmac_key_len,
                                char *out_summary, size_t summary_n);

// Parse a tolerant hex string ('a:b' / whitespace ignored). Returns
// byte count on success, -1 on bad input.
ssize_t tx_burst_parse_hex(const char *hex, uint8_t *out, size_t cap);

// Compute the Doppler-corrected TX carrier (Hz) for a flying satellite.
// `nominal_carrier_hz` is the satellite's published RX frequency (e.g.
// FRONTIERSAT_CARRIER_HZ = 436.150 MHz). `range_rate_km_s` is the
// classical range rate: positive means receding (LOS end of pass).
// To make the satellite hear the nominal carrier, the ground must
// transmit at carrier / (1 - rr/c) — higher when receding, lower
// when approaching.
//
// enable=0 short-circuits to (long) nominal_carrier_hz so callers can
// pin the "Doppler-disabled" behaviour with the same helper.
//
// Returns the bare nominal when the computed factor would underflow
// (defensive: never returns a negative or zero frequency).
long tx_burst_doppler_freq_hz(double nominal_carrier_hz,
                              double range_rate_km_s,
                              int enable);

// Build the on-wire AX100 frame bytes (CSP-encoded payload + AX100
// framing including optional HMAC trailer and RS parity) the burst is
// about to transmit. Suitable for unit tests that want to inspect what
// goes on the wire without keying a B210. Mirrors the framing path
// inside tx_burst_run exactly (use_rs=1, default ax100 opts).
//
// hmac_key=NULL / hmac_key_len=0 disables HMAC signing — useful for
// confirming the default-on behaviour by toggling the key alone.
//
// Returns frame length on success, -1 on error.
ssize_t tx_burst_build_frame(const uint8_t *payload, size_t payload_len,
                              const csp_v1_header_t *csp_hdr,
                              const uint8_t *hmac_key, size_t hmac_key_len,
                              uint8_t *out_frame, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif // TX_BURST_H
