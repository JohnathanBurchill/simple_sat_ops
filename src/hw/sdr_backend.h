/*

   Simple Satellite Operations  sdr_backend.h

   Pluggable SDR backend abstraction. The device-agnostic RX chain
   (FIR decimation, Doppler NCO, FM discriminator, level meter, burst
   detector) lives in b210_rx_tx_core.c; the device-specific I/O lives
   behind this vtable. Mirrors the radio_backend_ops_t pattern used for
   the CAT/CI-V radios.

   A backend supplies raw IQ at its native sample rate via read_iq and
   the usual tune/gain controls; the chain does the rest. A backend with
   tx_burst == NULL is RX-only (e.g. RTL-SDR) and the rest of the program
   gates transmit accordingly.

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

#ifndef SDR_BACKEND_H
#define SDR_BACKEND_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

// Which backend to open. AUTO probes the compiled-in backends in a
// fixed order (UHD first, then RTL-SDR) and uses the first that opens.
typedef enum sdr_backend_type {
    SDR_TYPE_AUTO = 0,
    SDR_TYPE_UHD,
    SDR_TYPE_RTLSDR,
    SDR_TYPE__COUNT,   // sentinel: number of enum values; keep last
} sdr_backend_type_t;

// Device capabilities, filled by the backend's open(). Read-only to the
// rest of the program.
typedef struct sdr_caps {
    int    can_tx;             // 0 => RX-only (tx_burst is NULL)
    double native_rate_hz;     // coerced RX sample rate (pre-decimation)
    double tune_resolution_hz; // informational: LO step the device can hit
    int    has_hw_lo_offset;   // 1 if the LO can be parked off the carrier
    int    sc16_native;        // 1 if read_iq yields native int16 (UHD); 0 if converted (RTL uint8)
    size_t max_rx_pairs;       // upper bound on IQ pairs returned by one read_iq
    char   name[32];           // friendly device name for banner/audit
} sdr_caps_t;

// Open parameters. Superset of what any one backend needs; a backend
// ignores fields that don't apply to it.
typedef struct sdr_open_params {
    double      freq_hz;            // initial RX center freq
    double      rate_hz;            // requested native sample rate (coerced)
    double      target_post_decim_hz; // desired rate after the chain's decimator;
                                    // a backend with a fixed/quantized native
                                    // rate (RTL-SDR) picks a valid native that
                                    // is an integer multiple of this so the
                                    // chain hits the target. 0 => use rate_hz.
    double      gain_db;            // RX gain
    double      bw_hz;              // analog filter bw; <=0 => use rate_hz
    const char *rx_antenna;         // NULL => backend default
    const char *device_args;        // NULL => backend default
    int         rx_dc_offset_track; // AD9361 tracking loops (UHD only)
    int         rx_iq_balance_track;
    const char *uhd_args_override;  // UHD: raw device-args passthrough (NULL => none)
    const char *fpga_image_path;    // UHD: force fpga=<path> (NULL => none)
    int         device_index;       // RTL-SDR: dongle index (0 = first)
} sdr_open_params_t;

// Optional per-burst timing breakdown (seconds), filled by the backend's
// tx_burst when sdr_tx_burst_params.timing is non-NULL. Shows where a burst's
// wall-clock goes so the operator can read the real floor that bounds the
// auto-tcmd inter-send interval, rather than guess at it:
//   config_s    device lock + rate/gain/freq set
//   push_s      host send loop (samples handed to the FIFO)
//   drain_s     wait for the scheduled burst to finish emitting
//               (the start-delay lead plus the on-air frame)
//   powerdown_s TX chain teardown
//   total_s     the whole backend tx_burst call
typedef struct sdr_tx_burst_timing {
    double config_s;
    double push_s;
    double drain_s;
    double powerdown_s;
    double total_s;
} sdr_tx_burst_timing_t;

// One TX burst, half-duplex: the backend pauses its own RX, transmits,
// and resumes RX. Same contract as the old b210_rx_tx_core_burst.
typedef struct sdr_tx_burst_params {
    const int16_t *iq;            // pre-built sc16 interleaved, n_samps pairs
    size_t         n_samps;
    double         tx_rate_hz;
    double         tx_freq_hz;
    double         tx_gain_db;
    double         start_delay_s;
    double         rx_resume_freq_hz; // RX center freq to restore after TX
    // NULL => don't measure. Non-NULL => the backend fills it (see above).
    sdr_tx_burst_timing_t *timing;
} sdr_tx_burst_params_t;

typedef struct sdr_backend sdr_backend_t;

// The vtable. open/close/read_iq/set_freq/get_actual_freq/set_gain are
// required; tx_burst is NULL for RX-only backends.
typedef struct sdr_backend_ops {
    const char *name;  // backend id, e.g. "uhd", "rtlsdr"
    int     (*open)(sdr_backend_t *be, const sdr_open_params_t *p, sdr_caps_t *caps);
    void    (*close)(sdr_backend_t *be);
    // Blocking read with an internal timeout. Writes up to cap_pairs
    // interleaved int16 I,Q into out (native rate, sc16 scale). Returns
    // >0 pairs read, 0 transient (overflow/timeout — keep looping),
    // <0 fatal. The buffer is caller-owned.
    ssize_t (*read_iq)(sdr_backend_t *be, int16_t *out, size_t cap_pairs);
    int     (*set_freq)(sdr_backend_t *be, double freq_hz);
    double  (*get_actual_freq)(sdr_backend_t *be);
    int     (*set_gain)(sdr_backend_t *be, double gain_db);
    int     (*tx_burst)(sdr_backend_t *be, const sdr_tx_burst_params_t *p); // NULL => RX-only
} sdr_backend_ops_t;

// The handle. Not opaque so backends can reach priv/caps directly; the
// rest of the program goes through the dispatch wrappers below.
struct sdr_backend {
    const sdr_backend_ops_t *ops;
    sdr_caps_t               caps;
    void                    *priv;  // backend-private state
};

// Per-backend ops getters. Each returns a pointer to a const static ops
// struct, or NULL if that backend was not compiled in.
const sdr_backend_ops_t *sdr_backend_uhd_ops(void);
const sdr_backend_ops_t *sdr_backend_rtlsdr_ops(void);

// Parse a CLI string ("uhd", "rtlsdr", "auto") to a type. Unknown
// strings return -1.
int sdr_backend_type_from_string(const char *s, sdr_backend_type_t *out);

// Open a backend of the given type. On success *out is a live handle and
// 0 is returned; on failure *out is NULL and -1 is returned (stderr has
// detail). AUTO tries each compiled-in backend in order.
int sdr_backend_open(sdr_backend_type_t type,
                     const sdr_open_params_t *p,
                     sdr_backend_t **out);

// Close + free a handle (NULL-safe).
void sdr_backend_close(sdr_backend_t *be);

// Dispatch wrappers (NULL-safe; forward to the active backend's op).
ssize_t sdr_backend_read_iq(sdr_backend_t *be, int16_t *out, size_t cap_pairs);
int     sdr_backend_set_freq(sdr_backend_t *be, double freq_hz);
double  sdr_backend_get_actual_freq(sdr_backend_t *be);
int     sdr_backend_set_gain(sdr_backend_t *be, double gain_db);
// Returns -1 (and logs) when the active backend is RX-only.
int     sdr_backend_tx_burst(sdr_backend_t *be, const sdr_tx_burst_params_t *p);
const sdr_caps_t *sdr_backend_caps(const sdr_backend_t *be);
// 1 iff the active backend can transmit (tx_burst present and caps.can_tx).
int     sdr_backend_can_tx(const sdr_backend_t *be);

#endif // SDR_BACKEND_H
