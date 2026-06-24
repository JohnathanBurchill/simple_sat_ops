/*

   Simple Satellite Operations  src/sdr_state.h

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

// SDR backend + live RX session slice of state_t — see state.h.

#ifndef SDR_STATE_H
#define SDR_STATE_H

#include "sdr_backend.h"

// The RX session owns the SDR core + worker thread; sdr_t holds only an
// opaque handle. Forward-declared here so this header stays free of
// rx_session.h (which next_in_queue, a state.h consumer, never links).
typedef struct rx_session rx_session_t;

typedef struct sdr {
    // SDR backend selection + the live RX session. simple_sat_ops is the
    // single process that opens the SDR; without_b210 (--without-b210, or a
    // non-WITH_USRP_B210 build) leaves rx_session NULL and the loop falls
    // through cleanly. sdr_type defaults to AUTO (probe UHD, then RTL-SDR);
    // sdr_device is a backend-specific selector (RTL index; for UHD prefer
    // uhd_args); uhd_args is a verbatim UHD device-args passthrough;
    // sdr_fpga forces an FPGA image for a B2xx clone.
    int                without_b210;
    int                no_audio;   // --no-audio: refuse viewer live-audio requests
    // --always-record: start the WAV / IQ / sidecar recording as soon as
    // rx_session opens and keep it open until shutdown, ignoring the usual
    // per-pass elevation gate. For bench characterisation runs.
    int                always_record;
    sdr_backend_type_t sdr_type;
    char               sdr_device[128];
    char               uhd_args[256];
    char               sdr_fpga[512];
    rx_session_t      *rx_session;

    // SDR LO offset (Hz) below the nominal downlink carrier. Without this,
    // the corrected signal sits at DC after software Doppler tracking and
    // gets eaten by the B210's DC blocker whenever Doppler crosses zero
    // (i.e. exactly at TCA, the worst possible time). Offsetting the LO
    // by ~25 kHz parks the signal in a clean part of the captured 96 kHz
    // baseband for the whole pass. Configurable via --lo-offset=<kHz>.
    double rx_lo_offset_hz;

    // AD9361 RX gain (dB) — fed to b210_rx_tx_core at session open.
    // Configurable via --rx-gain=<dB>; runtime adjustment is via the
    // :gain colon command in the operator UI.
    double rx_gain_db;

    // AD9361 background tracking loops. Default off — these add a
    // ~51 Hz comb of impulsive spikes to the captured IQ at mid gain
    // settings (see b210_rx_tx_core.h). Configurable via
    // --ad9361-dc-track=on|off and --ad9361-iq-track=on|off so the
    // operator can A/B against the default-on UHD baseline.
    int rx_dc_offset_track;
    int rx_iq_balance_track;
} sdr_t;

#endif // SDR_STATE_H
