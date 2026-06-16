/*

   Simple Satellite Operations  ui/spectrogram.c

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

#include "spectrogram.h"
#include "state.h"

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

// Caveat: the WAV is FM-demoded mono PCM, not IQ. That puts a hard
// ceiling on how SatNOGS-like these spectrograms can look — see the
// commentary in b210_rx_tx_core.c (line ~303) about the FM-discriminator
// noise floor dropping on carrier capture. For a SatNOGS-style waterfall
// against a flat thermal floor we'd need to tap the IQ stream before
// the discriminator; that's a separate feature.

// Render a full IQ recording with gen_waterfall — SatNOGS-style
// viridis waterfall, no ffmpeg dependency, signals pop against a
// flat median-subtracted noise floor. Returns 0 on success, -1 on
// fork/exec failure or non-zero gen_waterfall exit.
int generate_full_iq_waterfall(const char *iq_path, int rate_hz,
                               char *png_out, size_t png_cap)
{
    if (iq_path == NULL || rate_hz <= 0) return -1;
    size_t len = strlen(iq_path);
    char png[640];
    int n;
    if (len >= 3 && strcmp(iq_path + len - 3, ".iq") == 0) {
        n = snprintf(png, sizeof png, "%.*s_waterfall.png",
                     (int)(len - 3), iq_path);
    } else {
        n = snprintf(png, sizeof png, "%s_waterfall.png", iq_path);
    }
    if (n <= 0 || (size_t) n >= sizeof png) return -1;
    char rate_buf[16];
    snprintf(rate_buf, sizeof rate_buf, "%d", rate_hz);

    // Detrend defaults: the operator's IQ stream is Doppler-corrected,
    // so the target carrier sits at a fixed baseband bin for the whole
    // pass. With the column-median or long-tau HPF, that fixed-bin
    // signal gets subtracted out of itself; a short HPF time constant
    // (~ a couple of seconds) preserves anything modulated faster than
    // ~10 Hz (the 9600 baud envelope, beacons, packet bursts) while
    // still flattening the slow drift inside the pass.
    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        char *args[] = {
            "gen_waterfall",
            (char *) iq_path, rate_buf, png,
            (char *) "--detrend=hpf",
            (char *) "--detrend-tau-s=2",
            NULL,
        };
        execvp("gen_waterfall", args);
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return -1;
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        if (png_out && png_cap) snprintf(png_out, png_cap, "%s", png);
        return 0;
    }
    return -1;
}

// Render a finished WAV directly (no slicing) via ffmpeg. Blocks until
// ffmpeg exits. Returns 0 on success, -1 on fork/exec/exit failure.
// Used at end-of-pass on the final closed WAV.
int generate_full_spectrogram(const char *wav_path, char *png_out, size_t png_cap)
{
    if (wav_path == NULL) return -1;
    size_t len = strlen(wav_path);
    char png[640];
    int n;
    if (len >= 4 && strcmp(wav_path + len - 4, ".wav") == 0) {
        n = snprintf(png, sizeof png, "%.*s.png", (int)(len - 4), wav_path);
    } else {
        n = snprintf(png, sizeof png, "%s.png", wav_path);
    }
    if (n <= 0 || (size_t) n >= sizeof png) return -1;

    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        char *args[] = {
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-i", (char *) wav_path,
            "-lavfi",
            "showspectrumpic=s=1920x1080:mode=combined:color=viridis:scale=log:legend=1:saturation=1.5",
            png, NULL,
        };
        execvp("ffmpeg", args);
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return -1;
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        if (png_out && png_cap) snprintf(png_out, png_cap, "%s", png);
        return 0;
    }
    return -1;
}

// IQ-slice branch of spectrum_worker. Snapshots `j->iq_pairs` pairs
// starting at `j->iq_start_pair` from the live .iq file, then shells
// out gen_waterfall on the slice. Returns 0 on success, -1 on failure
// (the worker fills j->status_msg in either case).
static int spectrum_worker_iq(spectrum_job_t *j)
{
    char tmp_iq[700];
    snprintf(tmp_iq, sizeof tmp_iq, "%s.tmp.iq", j->png_out);

    FILE *fin = fopen(j->iq_in, "rb");
    if (fin == NULL) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: open %s failed: %s", j->iq_in, strerror(errno));
        return -1;
    }
    if (fseek(fin, j->iq_start_pair * 4, SEEK_SET) != 0) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: iq seek failed: %s", strerror(errno));
        fclose(fin); return -1;
    }
    FILE *fout = fopen(tmp_iq, "wb");
    if (fout == NULL) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: open %s failed: %s", tmp_iq, strerror(errno));
        fclose(fin); return -1;
    }
    int16_t buf[4096];     // 1024 pairs per read
    long remaining = j->iq_pairs;
    while (remaining > 0) {
        long want_pairs = remaining > (long)(sizeof buf / 4)
                        ? (long)(sizeof buf / 4) : remaining;
        size_t int16_count = (size_t)(want_pairs * 2);
        size_t got = fread(buf, sizeof(int16_t), int16_count, fin);
        if (got == 0) break;
        fwrite(buf, sizeof(int16_t), got, fout);
        remaining -= (long)(got / 2);
    }
    fclose(fin); fclose(fout);

    char rate_buf[16];
    snprintf(rate_buf, sizeof rate_buf, "%d", j->iq_sample_rate);
    pid_t pid = fork();
    int rc = -1;
    if (pid == 0) {
        // See generate_full_iq_waterfall for the detrend rationale —
        // the slice path renders the same Doppler-corrected IQ data
        // and benefits from the same short-tau HPF.
        char *args[] = {
            "gen_waterfall",
            tmp_iq, rate_buf, j->png_out,
            (char *) "--detrend=hpf",
            (char *) "--detrend-tau-s=2",
            NULL,
        };
        execvp("gen_waterfall", args);
        _exit(127);
    } else if (pid > 0) {
        int status;
        if (waitpid(pid, &status, 0) >= 0 && WIFEXITED(status))
            rc = WEXITSTATUS(status);
    }
    unlink(tmp_iq);
    if (rc == 0) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: wrote %s (%.1fs IQ)",
                 j->png_out,
                 (double) j->iq_pairs / (double) j->iq_sample_rate);
        return 0;
    }
    snprintf(j->status_msg, sizeof j->status_msg,
             "spectrum: gen_waterfall failed (rc=%d) for %s", rc, j->png_out);
    return -1;
}

void *spectrum_worker(void *arg)
{
    spectrum_job_t *j = (spectrum_job_t *) arg;
    // Prefer the IQ path — gives a real SatNOGS-style waterfall. The
    // WAV fallback below stays in place for runs where the IQ sidecar
    // didn't open (e.g., disk full mid-pass).
    if (j->iq_in[0] && j->iq_pairs > 0 && j->iq_sample_rate > 0) {
        (void) spectrum_worker_iq(j);
        j->done = 1;
        return NULL;
    }
    char tmp_wav[700];
    snprintf(tmp_wav, sizeof tmp_wav, "%s.tmp.wav", j->png_out);

    FILE *fin = fopen(j->wav_in, "rb");
    if (fin == NULL) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: open %s failed: %s", j->wav_in, strerror(errno));
        j->done = 1;
        return NULL;
    }
    if (fseek(fin, 44 + j->start_sample * 2, SEEK_SET) != 0) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: seek failed: %s", strerror(errno));
        fclose(fin); j->done = 1; return NULL;
    }
    FILE *fout = fopen(tmp_wav, "wb");
    if (fout == NULL) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: open %s failed: %s", tmp_wav, strerror(errno));
        fclose(fin); j->done = 1; return NULL;
    }
    uint32_t sr   = (uint32_t) j->sample_rate;
    uint32_t bps  = sr * 2;
    uint32_t bcnt = (uint32_t)(j->n_samples * 2);
    uint32_t fsz  = bcnt + 36;
    uint8_t hdr[44] = {
        'R','I','F','F',
        (uint8_t) fsz,(uint8_t)(fsz>>8),(uint8_t)(fsz>>16),(uint8_t)(fsz>>24),
        'W','A','V','E', 'f','m','t',' ', 16,0,0,0, 1,0, 1,0,
        (uint8_t) sr, (uint8_t)(sr>>8), (uint8_t)(sr>>16), (uint8_t)(sr>>24),
        (uint8_t) bps,(uint8_t)(bps>>8),(uint8_t)(bps>>16),(uint8_t)(bps>>24),
        2,0, 16,0,
        'd','a','t','a',
        (uint8_t) bcnt,(uint8_t)(bcnt>>8),(uint8_t)(bcnt>>16),(uint8_t)(bcnt>>24),
    };
    if (fwrite(hdr, 1, 44, fout) != 44) {
        snprintf(j->status_msg, sizeof j->status_msg, "spectrum: header write failed");
        fclose(fin); fclose(fout); unlink(tmp_wav); j->done = 1; return NULL;
    }
    int16_t buf[4096];
    long remaining = j->n_samples;
    while (remaining > 0) {
        size_t want = remaining > (long)(sizeof buf / sizeof buf[0])
                    ? sizeof buf / sizeof buf[0] : (size_t) remaining;
        size_t got = fread(buf, sizeof buf[0], want, fin);
        if (got == 0) break;
        fwrite(buf, sizeof buf[0], got, fout);
        remaining -= (long) got;
    }
    fclose(fin); fclose(fout);

    pid_t pid = fork();
    int rc = -1;
    if (pid == 0) {
        char *args[] = {
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-i", tmp_wav,
            "-lavfi",
            "showspectrumpic=s=1920x1080:mode=combined:color=viridis:scale=log:legend=1:saturation=1.5",
            j->png_out, NULL,
        };
        execvp("ffmpeg", args);
        _exit(127);
    } else if (pid > 0) {
        int status;
        if (waitpid(pid, &status, 0) >= 0 && WIFEXITED(status))
            rc = WEXITSTATUS(status);
    }
    unlink(tmp_wav);

    if (rc == 0) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: wrote %s (%.1fs)",
                 j->png_out, (double) j->n_samples / (double) j->sample_rate);
    } else {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: ffmpeg failed (rc=%d) for %s", rc, j->png_out);
    }
    j->done = 1;
    return NULL;
}

void spectrum_job_reap(state_t *state)
{
    if (state->spec_job.active && state->spec_job.done) {
        pthread_join(state->spec_job.thr, NULL);
        state->spec_job.active = 0;
    }
}
