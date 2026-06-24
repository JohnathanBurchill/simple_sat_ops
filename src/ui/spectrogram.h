/*

   Simple Satellite Operations  ui/spectrogram.h

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

// Spectrogram / waterfall rendering for the operator UI. The `:spectrum N`
// command snapshots the last N seconds of the live capture and shells out to
// gen_waterfall (IQ) or ffmpeg's showspectrumpic (FM-demod WAV) on its own
// pthread so the main loop keeps ticking. Single slot — the job lives on
// state_t.spec_job.

#ifndef UI_SPECTROGRAM_H
#define UI_SPECTROGRAM_H

#include "ui_state.h"    // ui_t

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Render a full IQ recording with gen_waterfall — SatNOGS-style viridis
// waterfall, no ffmpeg dependency. Blocks until gen_waterfall exits.
// Returns 0 on success, -1 on fork/exec failure or non-zero exit.
int generate_full_iq_waterfall(const char *iq_path, int rate_hz,
                               char *png_out, size_t png_cap);

// Render a finished WAV directly (no slicing) via ffmpeg. Blocks until ffmpeg
// exits. Returns 0 on success, -1 on failure. Used at end-of-pass on the
// final closed WAV.
int generate_full_spectrogram(const char *wav_path, char *png_out, size_t png_cap);

// pthread entry for the `:spectrum N` render job. arg is a spectrum_job_t*
// (state.ui.spec_job); renders the IQ slice when present, else the WAV slice,
// and sets j->done before returning.
void *spectrum_worker(void *arg);

// Reap a finished spectrum job so the slot is free for the next request.
// Called from cmd_dispatch (operator retry) and the shutdown path.
void spectrum_job_reap(ui_t *ui);

#ifdef __cplusplus
}
#endif

#endif // UI_SPECTROGRAM_H
