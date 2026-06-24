/*

    Simple Satellite Operations  state.h

    Copyright (C) 2025  Johnathan K Burchill

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

#ifndef STATE_H
#define STATE_H

#include "rot_state.h"
#include "sdr_state.h"
#include "track_state.h"
#include "sso_ipc.h"
#include "telemetry.h"
#include "trsw_state.h"
#include "tx_state.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>

// Signal ribbon: 1 Hz timeline of "I am alive" marks rendered as a vertical
// strip on the right side of the screen. Each char is one second; newest at
// the bottom. Plain ASCII so it works on minimal TTYs without UTF-8 fonts.
#define RIBBON_LEN 60

// Spectrogram render job. The `:spectrum N` command snapshots the last N
// seconds of the live WAV (which the rx_session worker is still appending
// to), copies them into a temporary WAV, and shells out to ffmpeg's
// showspectrumpic on its own pthread so the main loop keeps ticking.
// Single slot — only one render at a time.
typedef struct spectrum_job {
    pthread_t       thr;
    int             active;          // 1 once the thread has been launched
    volatile int    done;            // worker sets to 1 just before return
    // Source — pick one. When iq_in[0] is non-empty the worker renders
    // a SatNOGS-style waterfall via gen_waterfall(1) on the IQ slice;
    // otherwise it falls back to the FM-demod WAV slice through ffmpeg.
    char            wav_in[512];
    int             sample_rate;
    long            start_sample;
    long            n_samples;
    char            iq_in[512];
    int             iq_sample_rate;
    long            iq_start_pair;
    long            iq_pairs;
    char            png_out[640];
    char            status_msg[1024];
} spectrum_job_t;

// Latest broadcast snapshot, kept so a newly-connecting viewer gets state
// in its WELCOME response without waiting up to 500 ms for the next
// periodic STATE broadcast. ipc_broadcast_state fills it after each send;
// ipc_on_event reads it to seed a WELCOME.
typedef struct last_state {
    int    valid;
    char   sat[64];
    double az;
    double el;
    long   freq_hz;
    double doppler;
    char   tle[256];
    double tgt_az;
    double tgt_el;
    int    flip;
    int    in_pass;
    int    tracking;
    int    has_rot;
    double jul;
    char   idesg[9];
    double epoch_min;
    double min_visible;
    double min_above_0;
    double min_above_30;
    double max_el;
    double pred_az;
    double pred_el;
    double alt_km;
    double lat_deg;
    double lon_deg;
    double speed_kms;
    double range_km;
    double rrate_kms;
} last_state_t;

#define SCAN_MAX_TARGETS 512

// --scan-sky: drive the rotator through a grid of (az, el) targets spaced
// for roughly equal solid angle, dwelling at each while writing per-target
// arrival timestamps to a CSV. Owned by the operator loop.
typedef struct { double az_deg; double el_deg; } scan_target_t;
typedef struct scan_sky
{
    int           mode;            // CLI: --scan-sky rebinds T to a sky scan
    double        step_deg;        // elevation ring spacing (deg)
    scan_target_t targets[SCAN_MAX_TARGETS];
    int           n_targets;
    int           active;
    int           idx;
    // Set to t_now when the rotator's motion-flag clears at a target; the
    // dwell expires SCAN_DWELL_S later. 0 means "haven't arrived yet".
    double        dwell_start_s;
    FILE         *csv_fp;
    char          csv_path[640];
} scan_sky_t;

#define CMD_BUF_SIZE 128
#define CMD_HISTORY_SIZE 64

// Bottom-of-screen ":" command line (vi-style). While active, every key
// is routed through the command handler instead of the main key switch.
typedef struct cmdline
{
    int  active;
    char buf[CMD_BUF_SIZE];
    int  len;
    int  cursor;          // 0..len; insert position
    char status[160];
    // Preview debounce: dirty is set on every edit; the main loop
    // broadcasts a preview event once the buffer has been idle long enough.
    int  dirty;
    long last_edit_ns;
    // History: Up/Down cycle previously executed commands. The line being
    // edited is stashed on the first Up so Down can return to it.
    char history[CMD_HISTORY_SIZE][CMD_BUF_SIZE];
    int  history_count;   // entries in use (capped at SIZE)
    int  hist_pos;        // 0..count; ==count -> editing line
    char hist_saved[CMD_BUF_SIZE];  // editing line stash
} cmdline_t;

#define MAX_TLE_LINE_LENGTH 128
#define TRACKING_PREP_TIME_MINUTES 5.0

// SDR-only state. The CAT-radio + ALSA-audio fields are gone; the B210
// is now driven externally by b210_rx_tx / tx_frame_sdr, which talk
// to simple_sat_ops over the sso_ipc socket for operator coordination.
// What remains here is the satellite tracker, the rotator, and the
// nominal/Doppler-shifted frequency outputs computed for display + IPC
// broadcast.
typedef struct state
{
    // General
    int n_options;
    int running;
    int verbose_level;

    // Run mode + one-shot CLI flags (set once in apply_args / main).
    int control_mode;        // --control: this process is the operator
    int viewer_mode;         // bare invocation found a running operator
    int viewer_stream;       // --viewer-stream: headless JSON stream to stdout
    int self_test;           // --self-test: print a report and exit
    int run_live_waterfall;  // --live-waterfall
    int testing_mode;        // --testing

    const char       *operator_user;   // Unix user running the operator
    sso_ipc_server_t *ipc;             // IPC fan-out server (operator side)
    last_state_t      last_state;      // last broadcast snapshot (WELCOME seed)
    char   pass_folder[256];           // this pass's output folder; "" until set
    char   low_disk_msg[80];           // non-empty -> low-disk warning to show
    double low_disk_last_t;            // last low-disk probe (monotonic s)

    // Orbit prediction + Doppler frequencies + tracked-satellite identity.
    // See track_state.h.
    track_t track;

    // SDR backend selection + the live RX session + RX config. See sdr_state.h.
    sdr_t sdr;

    // Signal ribbon (1 Hz RX peak-dBFS timeline). peak/bright are parallel
    // circular buffers; ribbon_last_t gates the sampler; ribbon_push_count
    // drives the crawling 20 s tick mark.
    double ribbon_peak[RIBBON_LEN];
    int    ribbon_bright[RIBBON_LEN];
    int    ribbon_count;        // valid samples (caps at RIBBON_LEN)
    int    ribbon_head;         // next write index (circular)
    double ribbon_last_t;
    long   ribbon_push_count;   // total pushes since startup

    // Spectrogram render job (:spectrum N). Single slot.
    spectrum_job_t spec_job;

    // Antenna rotator + pursuit planner. See rot_state.h.
    rot_t rot;

    // T/R antenna switch (USB-CDC). See trsw_state.h.
    trsw_t trsw;

    // Telemetry overlay (still rendered alongside the prediction).
    telemetry_t telemetry;

    // Sky-scan grid + per-target CSV logging (--scan-sky).
    scan_sky_t scan;

    // Bottom-of-screen ":" command line.
    cmdline_t cmd;

    // Uplink: HMAC key, TX-compose + auto-telecommand modals, the burst
    // request slot, and the TX log. See tx_state.h.
    tx_t tx;
} state_t;


#endif // STATE_H
