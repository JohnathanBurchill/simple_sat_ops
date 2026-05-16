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
    int      allow_high_power;
    int      allow_hf_tx;
    char     summary[160];
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

#ifdef __cplusplus
}
#endif

#endif // TX_BURST_H
