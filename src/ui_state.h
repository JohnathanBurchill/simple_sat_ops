/*

   Simple Satellite Operations  src/ui_state.h

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

// Operator-UI slice of state_t: the signal ribbon, the spectrogram render
// job, and the live-waterfall flag (ui_t), plus the presentation types for
// the bottom command line (cmdline_t) and the sky-scan grid (scan_sky_t),
// which remain top-level members of state_t. See state.h.

#ifndef UI_STATE_H
#define UI_STATE_H

#include <pthread.h>
#include <stdio.h>

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

// Loose operator-UI fields: the signal ribbon, the single spectrogram
// render slot, and the --live-waterfall opt-in.
typedef struct ui {
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

    int run_live_waterfall;  // --live-waterfall

    // Keyboard lock. Starts unlocked (main sets it to 1); 'K' toggles it.
    // The keybindings dispatcher only fires non-KB_ALWAYS keys while set.
    int keyboard_unlocked;

    // Set when the terminal is resized (KEY_RESIZE). The main loop repaints
    // the whole screen on the next tick -- rather than waiting up to one
    // redraw period -- so the panels pick up the new width/height and the
    // old layout's stale cells get wiped right away. See ui/input.c.
    int need_full_redraw;
} ui_t;

#endif // UI_STATE_H
