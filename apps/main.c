/*

   Simple Satellite Operations  main.c

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

#define _GNU_SOURCE

#include "antenna_rotator.h"
#include "antenna_rotator_async.h"
#include "rotator_calibrate.h"
#include "pursuit.h"
#include "tr_switch.h"
#include "state.h"
#include "prediction.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_operator.h"
#include "sso_ipc_paths.h"
#include "sso_paths.h"
#include "tle_csv.h"
#include "frontiersat.h"
#include "hmac_keyfile.h"
#include "agenda_line.h"
#include "tcmd_lint.h"
#include "sso_pseudo.h"
#include "sso_version.h"
#include "argparse.h"

#ifdef SSO_WITH_SDR
#include "b210_rx_tx_core.h"
#include "carrier_trim.h"
#include "rx_session.h"
#include "tx_burst.h"
#endif

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <locale.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <ncurses.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// Carrier defaults. FrontierSat is simplex on 436.150 MHz — uplink
// and downlink share the same frequency — so both default to the
// same value. Overridable with --uplink-freq-mhz= / --downlink-freq-mhz=
// for any future split-band bird.
#define UPLINK_FREQ_MHZ   436.150000
#define DOWNLINK_FREQ_MHZ 436.150000

// Doppler retune threshold (Hz). Frequencies update on the display
// when the residual drifts past this. 200 Hz keeps the residual offset
// well inside the 9600 GFSK clean-eye band (~±3 kHz) even at the peak
// ~100 Hz/s slew near TCA.
#define DOPPLER_SHIFT_RESOLUTION_KHZ 0.2

// Antenna rotator max angle from target before a new SET is issued.
#define MAX_DELTA_AZIMUTH_DEGREES 1.0
#define MAX_DELTA_ELEVATION_DEGREES 1.0

// How close a STATUS azimuth must be to the just-commanded home waypoint to
// be treated as the controller's post-SET target echo -- which it reports
// for a couple of seconds before its feedback shows real motion -- rather
// than a real position reading. Used to gate the two-step home's final leg.
#define HOME_ECHO_TOLERANCE_DEG 2.0

#define WARN_DAYS_SINCE_EPOCH 1.0
#define MAX_MINUTES_TO_PREDICT ((7 * 1440))

#define UPDATE_INTERVAL_MICROSEC 500000

#define SSO_IPC_MAX_CLIENTS_FOR_ROSTER 16

// --- Operator-mode IPC bookkeeping ---------------------------------
// Set by apply_args when --control is passed. When set, main() opens
// the sso_ipc server on /run/sso/simple_sat_ops.sock and fans out a
// state event on every UI tick. Other operator-aware tools
// (b210_rx_tx --control, tx_frame_sdr) verify the operator's Unix
// user matches their own via this socket.
static int g_control_mode = 0;
static int g_viewer_mode = 0;  // bare invocation found a running operator
static sso_ipc_server_t *g_ipc = NULL;
static const char *g_operator_user = NULL;

// --self-test: after CLI parse + HMAC keyfile load, print the resolved
// configuration to stdout and exit 0 — BEFORE opening the IPC socket,
// the rotator, the B210, or loading the TLE. Useful for confirming
// "did my command line do what I think?" without keying any hardware
// or claiming any shared resource. Skips the no-arg viewer-probe in
// apply_args too (which is itself a side effect).
static int g_self_test = 0;

// Refuse to fully start when the --tc-file agenda has telecommand lint
// errors. --ignore-at-your-peril-all-tc-errors clears this and lets a
// known-bad agenda through anyway.
static int g_ignore_tc_errors = 0;

// TX dry-run: record the command as not-sent (reason "dry-run")
// instead of pushing the burst through rx_session. Lets the operator
// exercise the auto-tcmd
// state machine + the TX compose modal on a dev host with no B210 (or
// with --without-b210 to skip the device). The allow-tx safety
// checkbox still has to be ticked to enter RUNNING — dry-run is about
// hardware presence, not about the operator's intent to transmit.
static int   g_tx_dry_run         = 0;

// Live raylib waterfall viewer. Off by default; --live-waterfall on
// the command line opts in. When recording starts, fork+exec the
// live_waterfall binary with the active .iq path. Track the child
// PID so we can SIGTERM it on shutdown. The iq-path scratch is only
// referenced inside the WITH_USRP_B210 launch block, so tag it
// unused — the cleanup path at the bottom of main() does use the
// pid, so that one stays unannotated.
static int   g_run_live_waterfall = 0;

// --always-record: start the WAV / IQ / sidecar recording as soon
// as rx_session opens and keep it open until shutdown, ignoring the
// usual per-pass elevation gate. Intended for bench characterisation
// runs (noise floor vs. antenna orientation, no-antenna baseline,
// gain stability over hours) where the operator wants continuous
// capture without a satellite pass to drive AOS / LOS.
static int   g_always_record      = 0;

// --testing: bench / characterisation runs that aren't tied to a
// pass. Pass folder lands under <root>/Testing/yyyymmdd/hhmmLT/ using
// the CURRENT local time, not a predicted AOS — keeps test captures
// out of the operational Operations/ tree and skips the "no AOS in
// next N minutes" abort in setup_pass_folder.
static int   g_testing_mode       = 0;

// --scan-sky: rebinds T to "scan the sky" — drive the rotator through
// a grid of (az, el) targets spaced for roughly equal solid angle,
// dwelling at each for SCAN_DWELL_S seconds, while writing per-target
// arrival timestamps to a CSV in the pass folder. Intended for noise-
// floor / antenna-pattern characterisation runs where the operator
// wants the same orientation sweep every time. 's' stops mid-scan.
static int    g_scan_sky_mode     = 0;
static double g_scan_step_deg     = 15.0;  // elevation ring spacing (default)
#define SCAN_DWELL_S 5.0
#define SCAN_MAX_TARGETS 512
typedef struct { double az_deg; double el_deg; } scan_target_t;
static scan_target_t g_scan_targets[SCAN_MAX_TARGETS];
static int    g_scan_n_targets    = 0;
static int    g_scan_active       = 0;
static int    g_scan_idx          = 0;
// Set to t_now when the rotator's motion-flag clears at a target;
// the dwell expires SCAN_DWELL_S later. 0 means "haven't arrived yet".
static double g_scan_dwell_start_s = 0.0;
static FILE  *g_scan_csv_fp       = NULL;
static char   g_scan_csv_path[640] = "";
static pid_t g_live_waterfall_pid = -1;
__attribute__((unused))
static char  g_live_waterfall_iq[512] = "";
// Write end of the pipe whose read end is dup2'd to the viewer's
// stdin. Colon commands like :wf_zoom_khz write line-based commands
// here so the running viewer can adjust without a relaunch. -1 when
// no viewer is alive.
static int   g_live_waterfall_stdin_fd = -1;

// HMAC keyfile selection. Path is resolved at startup (CLI override
// or hmac_keyfile_default_path); the file is loaded into g_hmac_key
// once so (a) the operator banner can show "(N bytes ok)" / "(missing)"
// / "(bad)" and (b) every TX burst signs the AX100 frame with these
// bytes. The CTS1 flight firmware expects HMAC on every uplink — if
// the keyfile is missing or won't parse, TX is REFUSED rather than
// sending unsigned frames the satellite would silently drop.
typedef enum {
    HMAC_DISPLAY_UNSET   = 0,
    HMAC_DISPLAY_OK      = 1,
    HMAC_DISPLAY_MISSING = 2,
    HMAC_DISPLAY_BAD     = 3,
} hmac_display_status_t;
static char                  g_hmac_keyfile_path[512] = "";
static hmac_display_status_t g_hmac_display_status    = HMAC_DISPLAY_UNSET;
static uint8_t               g_hmac_key[64];
static size_t                g_hmac_key_len           = 0;

// Signal ribbon: 60-second 1 Hz rolling window of RX peak dBFS rendered
// as a UTF-8 block-character strip in the RX panel. Oldest sample on
// the left, newest on the right. Cheap fixed-size; the sampler is
// gated by monotonic seconds in the main loop.
// Signal ribbon: 1 Hz timeline of "I am alive" marks rendered as a
// vertical strip on the right side of the screen. Each char represents
// one second; the most recent sample sits at the bottom of the strip
// and older samples sit above. Plain ASCII ('.' / '-') so the display
// works on minimal SSH sessions / TTYs without UTF-8 fonts.
//
// Tick semantics: every 20 seconds in absolute push-time gets a bold
// '-' instead of '.'. Because the tick is keyed to absolute push count
// (not to a fixed visual position), the tick crawls upward by one row
// per second — the eye reads the timeline progressing even when the
// signal is flat.
#define RIBBON_LEN 60
static double g_ribbon_peak[RIBBON_LEN];
// Parallel bright-bin samples from iq_burst, indexed in lock-step
// with g_ribbon_peak. Lets the renderer pick a different character
// for broadband bursts (many bright bins) vs narrowband / carrier
// (few bright bins) at the same peak-dBFS level.
static int    g_ribbon_bright[RIBBON_LEN];
static int    g_ribbon_count       = 0;  // number of valid samples (caps at RIBBON_LEN)
static int    g_ribbon_head        = 0;  // next write index (circular)
// Only written by ribbon_push inside #ifdef WITH_USRP_B210; without
// B210 the variable + helper exist but no one calls them.
__attribute__((unused)) static double g_ribbon_last_t      = 0.0;
static long   g_ribbon_push_count  = 0;  // total pushes since startup; drives ticks

__attribute__((unused))
static void ribbon_push(double peak_dbfs, int bright_bins)
{
    g_ribbon_peak[g_ribbon_head]   = peak_dbfs;
    g_ribbon_bright[g_ribbon_head] = bright_bins;
    g_ribbon_head = (g_ribbon_head + 1) % RIBBON_LEN;
    if (g_ribbon_count < RIBBON_LEN) g_ribbon_count++;
    g_ribbon_push_count++;
}

// Low-disk warning. statvfs on the pass folder filesystem; rendered in
// the RX panel when free space dips below LOW_DISK_BYTES. Re-checked
// every LOW_DISK_PERIOD_S seconds so we don't statvfs on every redraw.
#define LOW_DISK_BYTES   ((uint64_t)10 * 1000 * 1000 * 1000)
#define LOW_DISK_PERIOD_S 30.0
static char   g_low_disk_msg[80] = "";
static double g_low_disk_last_t  = 0.0;
// Tentative declaration so low_disk_refresh can reference g_pass_folder
// without reordering the whole file; the definition with an initialiser
// lives further down.
static char   g_pass_folder[256];

// Returns bytes available to a non-privileged user on the filesystem
// hosting `path`. Returns (uint64_t) -1 on error or when path is empty.
static uint64_t free_disk_bytes(const char *path)
{
    if (path == NULL || path[0] == '\0') return (uint64_t) -1;
    struct statvfs s;
    if (statvfs(path, &s) != 0) return (uint64_t) -1;
    return (uint64_t) s.f_bavail * (uint64_t) s.f_frsize;
}

// Refresh g_low_disk_msg if the period has elapsed. Empty message
// means "above threshold" — render_rx_panel skips the row in that case.
static void low_disk_refresh(double t_now)
{
    if ((t_now - g_low_disk_last_t) < LOW_DISK_PERIOD_S
        && g_low_disk_last_t != 0.0) return;
    g_low_disk_last_t = t_now;
    const char *probe = g_pass_folder[0] ? g_pass_folder : ".";
    uint64_t avail = free_disk_bytes(probe);
    if (avail == (uint64_t) -1) {
        g_low_disk_msg[0] = '\0';
        return;
    }
    if (avail >= LOW_DISK_BYTES) {
        g_low_disk_msg[0] = '\0';
        return;
    }
    double gb = (double) avail / 1.0e9;
    // Cap path width so the message fits in the 80-byte buffer: prefix
    // is ~26 chars ("LOW DISK: 12.34 GB free at "), leaving 50 for the
    // path. GCC -Wformat-truncation would otherwise flag the unbounded
    // %s against a 255-byte g_pass_folder.
    snprintf(g_low_disk_msg, sizeof g_low_disk_msg,
             "LOW DISK: %.2f GB free at %.50s", gb, probe);
}

// Spectrogram render job. The `:spectrum N` REPL command snapshots the
// last N seconds of the live WAV (which the rx_session worker is still
// appending to), copies them into a temporary WAV, and shells out to
// ffmpeg's showspectrumpic. Runs on its own pthread so the main loop
// keeps ticking. Single slot — only one render at a time.
//
// Caveat: the WAV is FM-demoded mono PCM, not IQ. That puts a hard
// ceiling on how SatNOGS-like these spectrograms can look — see the
// commentary in b210_rx_tx_core.c (line ~303) about the FM-discriminator
// noise floor dropping on carrier capture. For a SatNOGS-style waterfall
// against a flat thermal floor we'd need to tap the IQ stream before
// the discriminator; that's a separate feature.
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
    long             iq_start_pair;
    long             iq_pairs;
    char            png_out[640];
    char            status_msg[1024];
} spectrum_job_t;

static spectrum_job_t g_spec_job;

// Render a full IQ recording with gen_waterfall — SatNOGS-style
// viridis waterfall, no ffmpeg dependency, signals pop against a
// flat median-subtracted noise floor. Returns 0 on success, -1 on
// fork/exec failure or non-zero gen_waterfall exit.
__attribute__((unused))
static int generate_full_iq_waterfall(const char *iq_path, int rate_hz,
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
__attribute__((unused))
static int generate_full_spectrogram(const char *wav_path, char *png_out, size_t png_cap)
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

__attribute__((unused))
static void *spectrum_worker(void *arg)
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

// Reap a finished spectrum job so the slot is free for the next request.
// Called both from cmd_dispatch (so the operator can retry) and from the
// shutdown path (so we don't leak the worker thread).
static void spectrum_job_reap(void)
{
    if (g_spec_job.active && g_spec_job.done) {
        pthread_join(g_spec_job.thr, NULL);
        g_spec_job.active = 0;
    }
}

// Command line: vi-style ":" prompt at the bottom of the screen for
// runtime actions. While g_cmd_active, every key is routed through the
// command handler instead of the main key switch.
#define CMD_BUF_SIZE 128
static int  g_cmd_active = 0;
static char g_cmd_buf[CMD_BUF_SIZE];
static int  g_cmd_len = 0;
static int  g_cmd_cursor = 0;            // 0..g_cmd_len; insert position
static char g_cmd_status[160] = "";
// cmd-preview debounce state. cmd_dirty is set every time the buffer is
// edited (or :  is entered fresh); the main loop broadcasts a preview
// event once the buffer has been idle for cmd_debounce_ns. Mirrors how
// the TX compose modal debounces its tx-preview events.
static int    g_cmd_dirty        = 0;
static long   g_cmd_last_edit_ns = 0;
static long   g_cmd_debounce_ns  = 150000000L;  // 150 ms

// Command history: Up/Down cycle previously executed commands,
// shell-style. The line being edited is stashed on the first Up so Down
// can return to it.
#define CMD_HISTORY_SIZE 64
static char g_cmd_history[CMD_HISTORY_SIZE][CMD_BUF_SIZE];
static int  g_cmd_history_count = 0;   // entries in use (capped at SIZE)
static int  g_cmd_hist_pos      = 0;   // 0..count; ==count -> editing line
static char g_cmd_hist_saved[CMD_BUF_SIZE] = "";  // editing line stash

static long cmd_now_ns(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (long) ts.tv_sec * 1000000000L + (long) ts.tv_nsec;
}

static void cmd_enter(void)
{
    g_cmd_active = 1;
    g_cmd_buf[0] = '\0';
    g_cmd_len = 0;
    g_cmd_cursor = 0;
    // Start a fresh history walk at the (empty) editing line.
    g_cmd_hist_pos = g_cmd_history_count;
    // Force an immediate preview broadcast so viewers see the ":" prompt
    // appear the moment the operator opens it.
    g_cmd_dirty = 1;
    g_cmd_last_edit_ns = 0;
}

static void cmd_set_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_cmd_status, sizeof g_cmd_status, fmt, ap);
    va_end(ap);
}

// --- Command history -----------------------------------------------
// Append a just-executed line. Skips blanks and immediate duplicates so
// holding Enter or repeating a command doesn't bloat the ring.
static void cmd_history_push(const char *line)
{
    if (line == NULL || line[0] == '\0') return;
    if (g_cmd_history_count > 0
        && strcmp(g_cmd_history[g_cmd_history_count - 1], line) == 0) {
        g_cmd_hist_pos = g_cmd_history_count;
        return;
    }
    if (g_cmd_history_count < CMD_HISTORY_SIZE) {
        snprintf(g_cmd_history[g_cmd_history_count], CMD_BUF_SIZE, "%s", line);
        g_cmd_history_count++;
    } else {
        memmove(&g_cmd_history[0], &g_cmd_history[1],
                sizeof g_cmd_history[0] * (CMD_HISTORY_SIZE - 1));
        snprintf(g_cmd_history[CMD_HISTORY_SIZE - 1], CMD_BUF_SIZE, "%s", line);
    }
    g_cmd_hist_pos = g_cmd_history_count;
}

// Replace the live buffer and park the cursor at the end. Marks the
// buffer dirty so the next tick re-broadcasts the preview to viewers.
static void cmd_buf_set(const char *s)
{
    snprintf(g_cmd_buf, sizeof g_cmd_buf, "%s", s);
    g_cmd_len = (int) strlen(g_cmd_buf);
    g_cmd_cursor = g_cmd_len;
    g_cmd_dirty = 1;
    g_cmd_last_edit_ns = cmd_now_ns();
}

static void cmd_history_prev(void)   // Up
{
    if (g_cmd_history_count == 0) return;
    if (g_cmd_hist_pos == g_cmd_history_count) {
        snprintf(g_cmd_hist_saved, sizeof g_cmd_hist_saved, "%s", g_cmd_buf);
    }
    if (g_cmd_hist_pos > 0) g_cmd_hist_pos--;
    cmd_buf_set(g_cmd_history[g_cmd_hist_pos]);
}

static void cmd_history_next(void)   // Down
{
    if (g_cmd_history_count == 0) return;
    if (g_cmd_hist_pos >= g_cmd_history_count) return;
    g_cmd_hist_pos++;
    cmd_buf_set(g_cmd_hist_pos == g_cmd_history_count
                    ? g_cmd_hist_saved
                    : g_cmd_history[g_cmd_hist_pos]);
}

// --- Path expansion + tab completion -------------------------------

// Expand a leading ~ and any $VAR / ${VAR} references in `in` into
// `out`. ~ maps to $HOME at the very start only; unknown variables
// expand to empty. Returns 0 on success, -1 on overflow (out emptied).
// Lets a typed `$TLES/.../tle.tle` reach the filesystem at dispatch and
// during completion.
static int cmd_expand(const char *in, char *out, size_t cap)
{
    size_t o = 0;
#define CMD_EXPAND_PUT(c) \
    do { if (o + 1 >= cap) { out[0] = '\0'; return -1; } out[o++] = (c); } while (0)
    for (size_t i = 0; in[i] != '\0'; ) {
        if (i == 0 && in[0] == '~' && (in[1] == '/' || in[1] == '\0')) {
            const char *home = getenv("HOME");
            if (home) for (const char *h = home; *h; ++h) CMD_EXPAND_PUT(*h);
            i++;
            continue;
        }
        if (in[i] == '$') {
            int    braced = (in[i + 1] == '{');
            size_t s = i + 1 + (braced ? 1 : 0);
            size_t e = s;
            while (in[e] && (isalnum((unsigned char) in[e]) || in[e] == '_')) e++;
            if (e > s) {
                char   name[64];
                size_t nl = e - s;
                if (nl >= sizeof name) nl = sizeof name - 1;
                memcpy(name, in + s, nl);
                name[nl] = '\0';
                const char *val = getenv(name);
                if (val) for (const char *v = val; *v; ++v) CMD_EXPAND_PUT(*v);
                i = e;
                if (braced && in[i] == '}') i++;
                continue;
            }
            // Lone $ / ${ with no name -> emit literally.
        }
        CMD_EXPAND_PUT(in[i]);
        i++;
    }
    out[o] = '\0';
#undef CMD_EXPAND_PUT
    return 0;
}

// Insert a string at the cursor, shifting the tail right. Stops at the
// buffer cap. Marks the line dirty for the preview broadcast.
static void cmd_insert_str(const char *s)
{
    for (const char *p = s; *p; ++p) {
        if (g_cmd_len >= (int) sizeof g_cmd_buf - 1) break;
        memmove(&g_cmd_buf[g_cmd_cursor + 1], &g_cmd_buf[g_cmd_cursor],
                (size_t)(g_cmd_len - g_cmd_cursor + 1));  // include nul
        g_cmd_buf[g_cmd_cursor] = *p;
        g_cmd_len++;
        g_cmd_cursor++;
    }
    g_cmd_dirty = 1;
    g_cmd_last_edit_ns = cmd_now_ns();
}

// Command names, for first-token completion.
static const char *const g_cmd_names[] = {
    "help", "tx", "auto", "track", "stop", "home", "retarget",
    "freq", "rs", "spectrum", "lo_offset", "lo_bandwidth", "gain", "quit",
};

// Tab completion at the cursor. The first token completes against
// command names; any later token completes a filesystem path. Only the
// trailing path component is completed -- a $VAR / ~ prefix stays
// literal in the buffer and is expanded just to pick the directory to
// scan, so `:retarget $TLES/20260529/<TAB>` keeps `$TLES` and fills in
// the file. Single match -> full completion (with a trailing `/` for a
// directory); multiple -> extend to the longest common prefix.
static void cmd_tab_complete(void)
{
    if (!g_cmd_active) return;

    int start = g_cmd_cursor;
    while (start > 0 && g_cmd_buf[start - 1] != ' '
                     && g_cmd_buf[start - 1] != '\t') {
        start--;
    }
    int tok_len = g_cmd_cursor - start;
    char token[CMD_BUF_SIZE];
    if (tok_len < 0 || tok_len >= (int) sizeof token) return;
    memcpy(token, g_cmd_buf + start, (size_t) tok_len);
    token[tok_len] = '\0';

    // First token -> command-name completion.
    if (start == 0) {
        int    n = 0;
        size_t common = 0;
        char   lcp[32] = "";
        const char *only = NULL;
        for (size_t i = 0; i < sizeof g_cmd_names / sizeof g_cmd_names[0]; ++i) {
            if (strncmp(g_cmd_names[i], token, (size_t) tok_len) != 0) continue;
            n++;
            if (n == 1) {
                only = g_cmd_names[i];
                snprintf(lcp, sizeof lcp, "%s", g_cmd_names[i]);
                common = strlen(lcp);
            } else {
                size_t k = 0;
                while (k < common && g_cmd_names[i][k] == lcp[k]) k++;
                common = k;
                lcp[common] = '\0';
            }
        }
        if (n == 1) {
            cmd_insert_str(only + tok_len);
            cmd_insert_str(" ");
        } else if (n > 1 && common > (size_t) tok_len) {
            // lcp is already truncated to the common prefix.
            cmd_insert_str(lcp + tok_len);
        }
        return;
    }

    // Argument token -> path completion. Split into the literal dir
    // prefix (through the last '/') and the partial basename.
    const char *slash = strrchr(token, '/');
    char        dir_lit[CMD_BUF_SIZE];
    const char *base;
    if (slash) {
        size_t dl = (size_t)(slash - token) + 1;  // keep the '/'
        if (dl >= sizeof dir_lit) return;
        memcpy(dir_lit, token, dl);
        dir_lit[dl] = '\0';
        base = slash + 1;
    } else {
        dir_lit[0] = '\0';
        base = token;
    }
    size_t base_len = strlen(base);

    char scan_dir[1024];
    if (dir_lit[0] == '\0') {
        snprintf(scan_dir, sizeof scan_dir, ".");
    } else if (cmd_expand(dir_lit, scan_dir, sizeof scan_dir) != 0
               || scan_dir[0] == '\0') {
        return;
    }

    DIR *d = opendir(scan_dir);
    if (d == NULL) return;
    int    n = 0;
    size_t common = 0;
    // Sized to hold any directory entry name (gcc-15 treats d_name as up
    // to 1023 bytes for its truncation analysis).
    char   lcp[1024]  = "";
    char   only[1024] = "";
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        const char *nm = de->d_name;
        // Hide dotfiles (incl. . and ..) unless the user typed a dot.
        if (nm[0] == '.' && base[0] != '.') continue;
        if (strncmp(nm, base, base_len) != 0) continue;
        n++;
        if (n == 1) {
            snprintf(only, sizeof only, "%s", nm);
            snprintf(lcp,  sizeof lcp,  "%s", nm);
            common = strlen(lcp);
        } else {
            size_t k = 0;
            while (k < common && nm[k] == lcp[k]) k++;
            common = k;
            lcp[common] = '\0';
        }
    }
    closedir(d);
    if (n == 0) return;

    if (n == 1) {
        cmd_insert_str(only + base_len);
        // Append '/' when the single match is a directory.
        char full[2048];
        size_t sl = strlen(scan_dir);
        if (sl > 0 && scan_dir[sl - 1] == '/') {
            snprintf(full, sizeof full, "%s%s", scan_dir, only);
        } else {
            snprintf(full, sizeof full, "%s/%s", scan_dir, only);
        }
        struct stat st;
        if (stat(full, &st) == 0 && S_ISDIR(st.st_mode)) {
            cmd_insert_str("/");
        }
    } else if (common > base_len) {
        // lcp is already truncated to the common prefix of all matches.
        cmd_insert_str(lcp + base_len);
    }
}

// Forward decls so cmd_dispatch can call the existing action helpers,
// which live further down in the file.
static void tx_compose_open(void);
static void auto_tcmd_open(void);
void start_tracking(state_t *state);
void stop_tracking(state_t *state);
int  point_to_stationary_target(state_t *state, double azimuth, double elevation);
static void scan_sky_start(state_t *state);
static void scan_sky_stop(state_t *state, const char *reason);
static void scan_sky_tick(state_t *state, double t_now);
// g_rx_session is referenced here in cmd_dispatch but its definition
// sits with the rest of the B210 globals further down. Forward-declare
// it so the compiler doesn't reject the references; the symbol resolves
// at link time to the static definition below.
#ifdef SSO_WITH_SDR
static rx_session_t *g_rx_session;
#endif
// Forward-decl the auto-tcmd file path for the same reason — its
// definition lives next to the rest of the modal state further down,
// but cmd_dispatch needs to check it.
static char g_auto_tcmd_file_path[512];
// Forward-decl the auto-tcmd progress snapshot: the STATE / WELCOME
// builders (above the modal code) embed it so viewers can mirror the
// run's <sent>/<total>. Defined with the rest of the modal helpers.
static int auto_tcmd_progress(int *sent, int *total, const char **label);

// :retarget <file> swaps the tracked satellite mid-pass. retarget_to_tle
// reads the first satellite from the file and re-points the live
// ephemeris at it. These results let cmd_dispatch report what happened.
enum {
    RETARGET_OK        =  0,   // swapped; antenna re-aims on the next tick
    RETARGET_SAME      =  1,   // same file as the current target; no-op
    RETARGET_BAD_ARG   = -1,   // no path given
    RETARGET_READ_ERR  = -2,   // could not read a TLE from the file
    RETARGET_BAD_TLE   = -3,   // elements present but failed validation
};
static int retarget_to_tle(state_t *state, const char *path);
// File path of the satellite currently being tracked, so a repeat
// :retarget on the same file is a no-op. Seeded from the startup TLE
// path; updated on every successful retarget.
static char g_target_tle_path[1024];
// Stable backing store for satellite_ephem.name after a retarget (the
// startup name points at argv / an apply_args buffer).
static char g_target_name[64];

// Dispatch the typed command. state may be touched by tracking-related
// commands; nothing else needs it. Returns nothing -- result lands in
// g_cmd_status for the next redraw.
static void cmd_dispatch(state_t *state)
{
    char buf[CMD_BUF_SIZE];
    snprintf(buf, sizeof buf, "%s", g_cmd_buf);
    // Trim leading whitespace; an empty command is a no-op.
    char *p = buf;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '\0') { cmd_set_status(""); return; }

    char *save = NULL;
    char *cmd  = strtok_r(p, " \t", &save);
    char *arg1 = strtok_r(NULL, " \t", &save);

    if (cmd == NULL) {
        cmd_set_status("");
        return;
    }
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        cmd_set_status("commands: help tx track stop home quit "
                       "retarget <tle-file> "
                       "freq <MHz> lo_offset <±kHz> lo_bandwidth <kHz> "
                       "gain <dB> rs on|off spectrum <sec>");
    } else if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "q") == 0
               || strcmp(cmd, "exit") == 0) {
        state->running = 0;
        cmd_set_status("quitting");
    } else if (strcmp(cmd, "tx") == 0) {
        // Defer the modal until after we leave command-mode so the
        // bottom prompt doesn't bleed under the modal box.
        cmd_set_status("opening TX compose...");
        g_cmd_active = 0;
        tx_compose_open();
    } else if (strcmp(cmd, "auto") == 0) {
        if (g_auto_tcmd_file_path[0] == '\0') {
            cmd_set_status("auto: no --tc-file=<path> given on the cmdline");
        } else {
            cmd_set_status("opening auto-tcmd...");
            g_cmd_active = 0;
            auto_tcmd_open();
        }
    } else if (strcmp(cmd, "track") == 0) {
        start_tracking(state);
        cmd_set_status("tracking on");
        sso_audit_event("track-on",
            state->prediction.satellite_ephem.tle.sat_name[0]
                ? state->prediction.satellite_ephem.tle.sat_name : "");
    } else if (strcmp(cmd, "stop") == 0) {
        stop_tracking(state);
        cmd_set_status("tracking stopped");
        sso_audit_event("track-off", "");
    } else if (strcmp(cmd, "home") == 0) {
        stop_tracking(state);
        point_to_stationary_target(state, 0.0, 0.0);
        cmd_set_status("home: az=0 el=0");
        sso_audit_event("rotator-home", "az=0 el=0");
    } else if (strcmp(cmd, "retarget") == 0) {
        char expanded[1024];
        if (arg1 == NULL) {
            cmd_set_status("retarget: usage `retarget <tle-file>` "
                           "(first satellite in the file is used)");
        } else if (cmd_expand(arg1, expanded, sizeof expanded) != 0) {
            cmd_set_status("retarget: path too long after expansion");
        } else {
            int rc = retarget_to_tle(state, expanded);
            const char *name =
                state->prediction.satellite_ephem.tle.sat_name;
            double mins =
                state->prediction.predicted_minutes_until_visible;
            switch (rc) {
            case RETARGET_OK:
                if (mins > 0.0) {
                    cmd_set_status("retarget -> %s (AOS in %.1f min)",
                                   name, mins);
                } else {
                    cmd_set_status("retarget -> %s (in pass, %.0fs elapsed)",
                                   name, -mins * 60.0);
                }
                sso_audit_event("retarget", name);
                break;
            case RETARGET_SAME:
                cmd_set_status("retarget: already on %s (same file)", arg1);
                break;
            case RETARGET_READ_ERR:
                cmd_set_status("retarget: cannot read a TLE from '%s'", arg1);
                break;
            case RETARGET_BAD_TLE:
                cmd_set_status("retarget: '%s' has invalid TLE elements",
                               arg1);
                break;
            default:
                cmd_set_status("retarget: bad argument");
                break;
            }
        }
    } else if (strcmp(cmd, "freq") == 0) {
        if (arg1 == NULL) {
            cmd_set_status("freq: missing argument (MHz)");
        } else {
#ifdef SSO_WITH_SDR
            double v = atof(arg1);
            double hz = (v < 1e6) ? v * 1e6 : v;   // accept MHz or Hz
            if (hz < 1e6 || hz > 6e9) {
                cmd_set_status("freq: %g out of [1 MHz, 6 GHz]", hz);
            } else if (g_rx_session == NULL) {
                cmd_set_status("freq: no RX session");
            } else {
                rx_session_request_freq(g_rx_session, hz);
                cmd_set_status("freq -> %.6f MHz", hz / 1e6);
            }
#else
            cmd_set_status("freq: this build has no USRP support");
#endif
        }
    } else if (strcmp(cmd, "rs") == 0) {
        // Reed-Solomon toggle isn't a runtime knob yet -- rx_session
        // sets reed_solomon at open() time. Flag this clearly instead
        // of silently no-op'ing.
        if (arg1 == NULL) {
            cmd_set_status("rs: usage: rs on|off (not yet runtime-toggleable)");
        } else {
            cmd_set_status("rs %s: NOT YET WIRED -- rx_session_open params only",
                           arg1);
        }
    } else if (strcmp(cmd, "spectrum") == 0 || strcmp(cmd, "spec") == 0) {
#ifdef SSO_WITH_SDR
        if (arg1 == NULL) {
            cmd_set_status("spectrum: usage `spectrum <seconds>` (1..600)");
        } else if (g_rx_session == NULL) {
            cmd_set_status("spectrum: no RX session");
        } else {
            double duration_s = atof(arg1);
            if (duration_s <= 0.0) {
                cmd_set_status("spectrum: invalid duration '%s'", arg1);
            } else {
                if (duration_s > 600.0) duration_s = 600.0;
                if (duration_s < 1.0)   duration_s = 1.0;

                spectrum_job_reap();
                if (g_spec_job.active) {
                    cmd_set_status("spectrum: a render is already in progress");
                } else {
                    char wav_path[512];
                    long n_samples = 0;
                    int  sample_rate = 0;
                    int  wav_active = 0;
                    rx_session_wav_snapshot(g_rx_session,
                                            wav_path, sizeof wav_path,
                                            &n_samples, &sample_rate, &wav_active);
                    if (wav_path[0] == '\0' || sample_rate <= 0) {
                        cmd_set_status("spectrum: no WAV (recording not started yet)");
                    } else {
                        long want  = (long)(duration_s * (double) sample_rate);
                        long start = n_samples - want;
                        if (start < 0) { start = 0; want = n_samples; }
                        if (want <= 0) {
                            cmd_set_status("spectrum: no samples captured yet");
                        } else {
                            // Filename: strip .wav and append a local-time stamp range.
                            char base[512];
                            size_t plen = strlen(wav_path);
                            if (plen >= 4 && strcmp(wav_path + plen - 4, ".wav") == 0) {
                                snprintf(base, sizeof base, "%.*s",
                                         (int)(plen - 4), wav_path);
                            } else {
                                snprintf(base, sizeof base, "%s", wav_path);
                            }
                            time_t t_end = time(NULL);
                            time_t t_start = t_end - (time_t) llround(duration_s);
                            struct tm lt_start, lt_end;
                            localtime_r(&t_start, &lt_start);
                            localtime_r(&t_end,   &lt_end);
                            char ts_start[32], ts_end[32];
                            strftime(ts_start, sizeof ts_start, "%Y-%m-%d_%H-%M-%S", &lt_start);
                            strftime(ts_end,   sizeof ts_end,   "%H-%M-%S",          &lt_end);

                            memset(&g_spec_job, 0, sizeof g_spec_job);
                            snprintf(g_spec_job.wav_in, sizeof g_spec_job.wav_in,
                                     "%s", wav_path);
                            snprintf(g_spec_job.png_out, sizeof g_spec_job.png_out,
                                     "%.480s_LOCAL_%s_to_%s.png",
                                     base, ts_start, ts_end);
                            g_spec_job.sample_rate  = sample_rate;
                            g_spec_job.start_sample = start;
                            g_spec_job.n_samples    = want;

                            // Pair the WAV slice with an IQ slice if the
                            // sidecar exists — worker prefers IQ and only
                            // falls back to the WAV+ffmpeg path when iq_in
                            // is empty.
                            char iq_path[512] = "";
                            long iq_pairs = 0;
                            int  iq_rate  = 0;
                            rx_session_iq_snapshot(g_rx_session,
                                                   iq_path, sizeof iq_path,
                                                   &iq_pairs, &iq_rate);
                            if (iq_path[0] && iq_pairs > 0 && iq_rate > 0) {
                                long want_p  = (long)(duration_s * (double) iq_rate);
                                long start_p = iq_pairs - want_p;
                                if (start_p < 0) { start_p = 0; want_p = iq_pairs; }
                                if (want_p > 0) {
                                    snprintf(g_spec_job.iq_in,
                                             sizeof g_spec_job.iq_in, "%s", iq_path);
                                    g_spec_job.iq_sample_rate = iq_rate;
                                    g_spec_job.iq_start_pair  = start_p;
                                    g_spec_job.iq_pairs       = want_p;
                                }
                            }

                            if (pthread_create(&g_spec_job.thr, NULL,
                                               spectrum_worker, &g_spec_job) != 0) {
                                cmd_set_status("spectrum: pthread_create failed: %s",
                                               strerror(errno));
                            } else {
                                g_spec_job.active = 1;
                                cmd_set_status("spectrum: rendering %.1fs (%s) -> %s",
                                               (double) want / (double) sample_rate,
                                               g_spec_job.iq_in[0] ? "iq" : "wav",
                                               g_spec_job.png_out);
                            }
                        }
                    }
                }
            }
        }
#else
        cmd_set_status("spectrum: this build has no USRP support");
#endif
    } else if (strcmp(cmd, "lo_offset") == 0) {
        // Move the hardware LO mid-pass to dodge a baseband artifact.
        // SIGNED kHz: positive → LO above nominal (signal at NEGATIVE
        // baseband); negative → LO below (signal at POSITIVE baseband).
        // Brief PLL-settle glitch — decode skips a few frames. Comfort
        // range ~±5..±40 kHz; clipped here to ±45 kHz so the worst
        // case still keeps a 3 kHz margin to the post-decim band edge.
        if (arg1 == NULL) {
            cmd_set_status("lo_offset: usage `lo_offset <signed_kHz>` "
                           "(comfort range ±5..±40)");
        } else {
#ifdef SSO_WITH_SDR
            double khz = atof(arg1);
            if (khz < -45.0 || khz > 45.0) {
                cmd_set_status("lo_offset: %g kHz out of [-45, +45]", khz);
            } else if (g_rx_session == NULL) {
                cmd_set_status("lo_offset: no RX session");
            } else {
                double new_offset_hz = khz * 1000.0;
                state->rx_lo_offset_hz = new_offset_hz;
                rx_session_set_lo_offset(g_rx_session,
                                         state->nominal_downlink_frequency_hz,
                                         new_offset_hz);
                cmd_set_status("lo_offset -> %+.1f kHz (PLL glitching, "
                               "decode resumes shortly)", khz);
            }
#else
            cmd_set_status("lo_offset: this build has no USRP support");
#endif
        }
    } else if (strcmp(cmd, "lo_bandwidth") == 0) {
        // Adjust the live raylib waterfall's visible bandwidth at
        // runtime. The viewer reads line-based commands from its
        // stdin (we wired a pipe at fork time); send
        // "bandwidth N\n". Name mirrors :lo_offset so both LO-
        // relative knobs live in the same command family.
        if (arg1 == NULL) {
            cmd_set_status("lo_bandwidth: usage `lo_bandwidth <N>` (kHz)");
        } else if (g_live_waterfall_stdin_fd < 0) {
            cmd_set_status("lo_bandwidth: no live viewer running "
                           "(launch with --live-waterfall)");
        } else {
            double n = atof(arg1);
            if (n <= 0.0 || n > 1000.0) {
                cmd_set_status("lo_bandwidth: %g out of (0, 1000] kHz", n);
            } else {
                char line[64];
                int  ln = snprintf(line, sizeof line, "bandwidth %g\n", n);
                ssize_t w = (ln > 0) ? write(g_live_waterfall_stdin_fd,
                                             line, (size_t) ln) : -1;
                if (w == ln) {
                    cmd_set_status("lo_bandwidth: -> %g kHz", n);
                } else {
                    cmd_set_status("lo_bandwidth: write failed: %s",
                                   strerror(errno));
                }
            }
        }
    } else if (strcmp(cmd, "gain") == 0) {
        // Change the AD9361 RX gain mid-pass. The current operating
        // point lives in state->rx_gain_db; routed through the
        // worker thread so we don't touch the UHD streamer from this
        // thread (same handoff as :lo_offset).
        if (arg1 == NULL) {
            cmd_set_status("gain: usage `gain <dB>` (range 0-76; current %.1f)",
                           state->rx_gain_db);
        } else {
#ifdef SSO_WITH_SDR
            double g = atof(arg1);
            if (g < 0.0 || g > 76.0) {
                cmd_set_status("gain: %g dB out of [0, 76]", g);
            } else if (g_rx_session == NULL) {
                cmd_set_status("gain: no RX session");
            } else {
                state->rx_gain_db = g;
                rx_session_set_gain(g_rx_session, g);
                cmd_set_status("gain -> %.1f dB", g);
            }
#else
            cmd_set_status("gain: this build has no USRP support");
#endif
        }
    } else {
        cmd_set_status("unknown command '%s' (try :help)", cmd);
    }
}

// Mirror the operator's ":" prompt to viewers. cmd-preview carries the
// live buffer (debounced in the main loop); cmd-executed carries the
// dispatched command + the resulting status string. Both helpers no-op
// when g_ipc isn't open (e.g., --no-control).
static void cmd_broadcast_preview(void)
{
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_CMD_PREVIEW);
    snprintf(evt.from, sizeof evt.from, "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(evt.cmd_text, sizeof evt.cmd_text, "%s", g_cmd_buf);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }
}

static void cmd_broadcast_executed(const char *executed_cmd)
{
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_CMD_EXECUTED);
    snprintf(evt.from, sizeof evt.from, "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(evt.cmd_text,   sizeof evt.cmd_text,   "%s",
             executed_cmd ? executed_cmd : "");
    snprintf(evt.cmd_status, sizeof evt.cmd_status, "%s", g_cmd_status);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }
}

// Apply a single editing action to the cmd buffer. Factored so both
// the keypad-translated codepath (KEY_LEFT, KEY_RIGHT, KEY_DC, etc.)
// and the manual escape-sequence fallback (`\x1b[D` and friends) feed
// the same logic.
typedef enum {
    CMD_ACTION_NONE = 0,
    CMD_ACTION_LEFT,
    CMD_ACTION_RIGHT,
    CMD_ACTION_HOME,
    CMD_ACTION_END,
    CMD_ACTION_BACKSPACE,
    CMD_ACTION_DEL,
    CMD_ACTION_HIST_PREV,
    CMD_ACTION_HIST_NEXT,
} cmd_action_t;

static int cmd_apply_action(cmd_action_t a)
{
    switch (a) {
        case CMD_ACTION_LEFT:
            if (g_cmd_cursor > 0) g_cmd_cursor--;
            return 1;
        case CMD_ACTION_RIGHT:
            if (g_cmd_cursor < g_cmd_len) g_cmd_cursor++;
            return 1;
        case CMD_ACTION_HOME:
            g_cmd_cursor = 0;
            return 1;
        case CMD_ACTION_END:
            g_cmd_cursor = g_cmd_len;
            return 1;
        case CMD_ACTION_BACKSPACE:
            if (g_cmd_cursor > 0) {
                memmove(&g_cmd_buf[g_cmd_cursor - 1],
                        &g_cmd_buf[g_cmd_cursor],
                        (size_t)(g_cmd_len - g_cmd_cursor + 1));
                g_cmd_len--;
                g_cmd_cursor--;
                g_cmd_dirty = 1;
                g_cmd_last_edit_ns = cmd_now_ns();
            }
            return 1;
        case CMD_ACTION_DEL:
            if (g_cmd_cursor < g_cmd_len) {
                memmove(&g_cmd_buf[g_cmd_cursor],
                        &g_cmd_buf[g_cmd_cursor + 1],
                        (size_t)(g_cmd_len - g_cmd_cursor));
                g_cmd_len--;
                g_cmd_dirty = 1;
                g_cmd_last_edit_ns = cmd_now_ns();
            }
            return 1;
        case CMD_ACTION_HIST_PREV:
            cmd_history_prev();
            return 1;
        case CMD_ACTION_HIST_NEXT:
            cmd_history_next();
            return 1;
        default:
            return 0;
    }
}

// When keypad/terminfo doesn't translate arrow keys (some minimal
// $TERM values, or a tmux/screen pane that strips function-key info)
// the raw escape sequence arrives byte-by-byte instead of as a single
// KEY_*. The first byte is Esc (27); if we treat that as cancel the
// rest of the sequence falls into the main keyboard switch and gets
// silently dropped. Peek for follow-on bytes; only fall through to
// the Esc-as-cancel path when nothing else is queued.
static cmd_action_t cmd_drain_csi(void)
{
    // After Esc we expect '['; otherwise it's some other sequence we
    // don't recognise and we just swallow the lookahead.
    int b1 = getch();
    if (b1 == ERR) return CMD_ACTION_NONE;
    if (b1 != '[') return CMD_ACTION_NONE;
    int b2 = getch();
    if (b2 == ERR) return CMD_ACTION_NONE;
    if (b2 == 'D') return CMD_ACTION_LEFT;
    if (b2 == 'C') return CMD_ACTION_RIGHT;
    if (b2 == 'A') return CMD_ACTION_HIST_PREV;
    if (b2 == 'B') return CMD_ACTION_HIST_NEXT;
    if (b2 == 'H') return CMD_ACTION_HOME;
    if (b2 == 'F') return CMD_ACTION_END;
    // VT-style sequences: ESC [ <digits> ~ . We only care about a few.
    if (b2 >= '0' && b2 <= '9') {
        int b3 = getch();
        if (b3 == '~') {
            switch (b2) {
                case '1': return CMD_ACTION_HOME;   // some terminals
                case '3': return CMD_ACTION_DEL;
                case '4': return CMD_ACTION_END;    // some terminals
                case '7': return CMD_ACTION_HOME;
                case '8': return CMD_ACTION_END;
                default:  return CMD_ACTION_NONE;
            }
        }
    }
    return CMD_ACTION_NONE;
}

// Returns 1 if key was consumed by the command line, 0 to fall through.
// Supports left/right cursor movement, mid-line insert + delete, Home/
// End jumps, and Enter from any cursor position. The viewer's cmd_text
// wire field carries the buffer verbatim; cursor position stays local.
static int cmd_handle_key(int key, state_t *state)
{
    if (!g_cmd_active) return 0;
    if (key == ERR) return 1;
    if (key == 27 /* Esc OR start of a CSI sequence */) {
        cmd_action_t a = cmd_drain_csi();
        if (a != CMD_ACTION_NONE) {
            cmd_apply_action(a);
            return 1;
        }
        // Truly bare Esc — cancel.
        g_cmd_active = 0;
        g_cmd_status[0] = '\0';
        g_cmd_buf[0] = '\0';
        g_cmd_len = 0;
        g_cmd_cursor = 0;
        cmd_broadcast_executed("");
        g_cmd_dirty = 0;
        return 1;
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        char executed[CMD_BUF_SIZE];
        snprintf(executed, sizeof executed, "%s", g_cmd_buf);
        g_cmd_active = 0;
        cmd_history_push(executed);
        cmd_dispatch(state);
        // Audit: one line per `:` command the operator pressed Enter on,
        // with the post-dispatch status so a reviewer sees both the
        // request and the immediate result (e.g. "freq 437.5" /
        // "freq -> 437.500000 MHz").
        if (executed[0]) {
            char det[480];
            snprintf(det, sizeof det,
                     "input=\"%.100s\" result=\"%.150s\"",
                     executed, g_cmd_status);
            sso_audit_event("cmd", det);
        }
        // After dispatch, g_cmd_status holds the result string. Mirror
        // both to viewers so they see exactly what the operator sees.
        cmd_broadcast_executed(executed);
        g_cmd_dirty = 0;
        return 1;
    }
    if (key == '\t')      { cmd_tab_complete(); return 1; }
    if (key == KEY_UP)    { cmd_history_prev(); return 1; }
    if (key == KEY_DOWN)  { cmd_history_next(); return 1; }
    if (key == KEY_LEFT)  return cmd_apply_action(CMD_ACTION_LEFT);
    if (key == KEY_RIGHT) return cmd_apply_action(CMD_ACTION_RIGHT);
    if (key == KEY_HOME || key == 1 /* Ctrl-A */)
        return cmd_apply_action(CMD_ACTION_HOME);
    if (key == KEY_END  || key == 5 /* Ctrl-E */)
        return cmd_apply_action(CMD_ACTION_END);
    if (key == KEY_BACKSPACE || key == 127 || key == 8)
        return cmd_apply_action(CMD_ACTION_BACKSPACE);
    if (key == KEY_DC || key == 4 /* Ctrl-D */)
        return cmd_apply_action(CMD_ACTION_DEL);
    if (key >= 32 && key < 127 && g_cmd_len < (int) sizeof g_cmd_buf - 1) {
        // Insert at cursor: shift the tail right by one, drop char in.
        memmove(&g_cmd_buf[g_cmd_cursor + 1],
                &g_cmd_buf[g_cmd_cursor],
                (size_t)(g_cmd_len - g_cmd_cursor + 1));  // include nul
        g_cmd_buf[g_cmd_cursor] = (char) key;
        g_cmd_len++;
        g_cmd_cursor++;
        g_cmd_dirty = 1;
        g_cmd_last_edit_ns = cmd_now_ns();
    }
    return 1;
}

// Render the command prompt (or last-result status) on the bottom row.
// Cursor is drawn as a reverse-video block on the char at g_cmd_cursor
// — or on a trailing space when the cursor is at end-of-line. The
// surrounding text is plain so cursor position is unambiguous.
static void cmd_render(void)
{
    int row = LINES - 1;
    if (g_cmd_active) {
        move(row, 0);
        addch(':');
        for (int i = 0; i < g_cmd_len; ++i) {
            if (i == g_cmd_cursor) {
                addch(((unsigned char) g_cmd_buf[i]) | A_REVERSE);
            } else {
                addch((unsigned char) g_cmd_buf[i]);
            }
        }
        if (g_cmd_cursor == g_cmd_len) {
            addch(' ' | A_REVERSE);
        }
        clrtoeol();
        // Park the hardware cursor on the same cell the A_REVERSE
        // block highlights. The layered refresh below will curs_set(1)
        // when an editable context is active so the operator sees a
        // visible blinking cursor on top of the inverse block.
        move(row, 1 + g_cmd_cursor);
        return;
    } else if (g_cmd_status[0]) {
        mvprintw(row, 0, "%s", g_cmd_status);
    } else {
        move(row, 0);
    }
    clrtoeol();
}
// /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/ for the upcoming
// pass — created in main() once the AOS prediction is in, then
// broadcast on every STATE event so b210_rx_tx and tx_frame_sdr
// can drop their captures/logs in the same spot. Empty until set.
static char g_pass_folder[256] = "";
// SIGUSR1 sets this — used by the force-claim takeover path to nudge
// the operator-mode loop into a graceful exit. (Full in-place
// demotion is a follow-up; for now SIGUSR1 = quit.)
static volatile sig_atomic_t g_yield_requested = 0;
static void on_sigusr1(int sig) {
    (void) sig;
    g_yield_requested = 1;
}

// Loop pacing helper. With the B210 attached, the main loop runs at
// UHD-chunk cadence (~120 Hz at 240 kHz / 2040-sample chunks); slow-
// cadence work (IPC broadcast, ncurses redraw) is timestamp-gated so
// it stays at its historical 2 Hz / 10 Hz rates.
static double monotonic_seconds(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

// B210 ownership lives here now — simple_sat_ops is the single process
// that opens the SDR. --without-b210 (or a non-WITH_USRP_B210 build)
// leaves g_rx_session NULL and the loop falls through cleanly.
static int  g_without_b210 = 0;
#ifdef SSO_WITH_SDR
// SDR backend selection. g_sdr_type defaults to AUTO (probe UHD, then
// RTL-SDR). g_sdr_device is a backend-specific selector (RTL index;
// for UHD prefer --uhd-args). g_uhd_args is a verbatim UHD device-args
// passthrough; g_sdr_fpga forces an FPGA image for a B2xx clone.
static sdr_backend_type_t g_sdr_type   = SDR_TYPE_AUTO;
static char g_sdr_device[128] = "";
static char g_uhd_args[256]   = "";
static char g_sdr_fpga[512]   = "";
#endif
// Async wrapper around antenna_rotator's serial I/O. NULL if no rotator
// (--without-rotator, or antenna_rotator_init failed). Spawned right
// after antenna_rotator_init succeeds; joined on shutdown. While set, no
// other thread touches state.antenna_rotator's serial FD.
static antenna_rotator_async_t *g_rot_async = NULL;
// --calibrate-rotator: when 1, simple_sat_ops runs the calibration
// routine after opening the rotator and exits without entering the
// ncurses UI. --confirm-rotator-calibrate is the safety interlock so
// the physical motion is always deliberate.
static int g_calibrate_rotator = 0;
static int g_confirm_rotator_calibrate = 0;
// Rotator slew rates loaded from disk at startup (deg/s). Either both
// > 0 (calibration present, pursuit planner can run) or both 0
// (calibration absent, the track loop falls back to today's "aim
// where sat is now" behavior).
static double g_pursuit_az_dps = 0.0;
static double g_pursuit_el_dps = 0.0;
// --without-rotator-pursuit disables the planner without removing the
// calibration files. Useful for A/B comparisons on the bench.
static int    g_without_rotator_pursuit = 0;

// Pre-sampled mech-frame satellite trajectory backing the planner's
// sat_sample_fn_t callback. Sampled once at plan-build time so the
// planner's iterations see a consistent, order-independent function
// without us ever mutating state.prediction.satellite_ephem on the
// side. Owned alongside g_pursuit_plan; both live for the duration of
// the current tracking session and are freed together at LOS / 's' /
// shutdown.
typedef struct {
    double *t_jul;
    double *az_unwrapped;
    double *el;
    size_t  n;
} pursuit_track_t;
static pursuit_plan_t  g_pursuit_plan  = {0};
static pursuit_track_t g_pursuit_track = {0};
#ifdef SSO_WITH_SDR
// rx_session owns the b210 core + the worker thread. main.c only
// keeps a local handle long enough to open the device and hand it
// over.
static rx_session_t      *g_rx_session  = NULL;
static tx_request_slot_t  g_tx_request  = {0};
// Set when a burst has been submitted to the rx_session worker but not
// yet polled to completion. Gates submit-vs-poll in the main loop so the
// loop stays responsive (rotator / redraw / IPC / auto-tcmd ticks) while
// the worker pauses RX, transmits, and resumes RX.
static int                g_tx_inflight = 0;
// Doppler-corrected uplink carrier (Hz). Refreshed every main-loop
// tick from range_rate_km_s. The compose-preview, modal commit, and
// auto-tcmd staging all snapshot this so the burst is keyed at the
// frequency the satellite actually hears the nominal carrier on.
// Falls back to the bare nominal when Doppler correction is off or
// before SGP4 has produced a valid range_rate.
static long               g_tx_freq_hz_doppler =
                              (long) FRONTIERSAT_CARRIER_HZ;

// Current UTC in milliseconds -- the queue-time clock for expanding an
// "SSO+..." pseudo-command (see sso_pseudo.h). Captured fresh per send so each
// transmission carries a current time.
static long long sso_now_utc_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000LL;
}
// The @tssent dedup key stamped into every SSO+ expansion this session: the
// startup UTC truncated to the minute, pinned once in main(). One fixed value
// per process makes the flight firmware run an SSO+ time-sync once per pass
// (correct across an hour/midnight boundary within the pass); a later relaunch
// in a new minute runs it again. See sso_pseudo.h.
static long long g_sso_pass_tssent_ms = 0;
#endif

// CLI gate: --no-tx blocks the compose modal from actually committing
// a burst. Typing + preview broadcast still work so the operator can
// rehearse / get advice from viewers without keying the PA.
static int g_no_tx = 0;

// Modulated 0xAA carrier prepended in front of every TX burst, ms.
// Stamped into g_tx_request.preroll_ms at slot-build time and read by
// auto_tcmd_burst_seconds for the on-air progress estimate. Default
// matches the tx_burst_run fallback (200 ms). Override with
// --tx-preroll-ms=<n>.
static int g_tx_preroll_ms = 200;

// TX log ring buffer — last few PREVIEW/SENT/NOT_SENT events for display.
// Shared by operator and viewer renderers. ascii matches the upstream
// sso_event_t.ascii field (SSO_TX_TEXT_MAX) so the panel renders the
// entire command text — up to a full RF telecommand — instead of a
// truncated preview.
typedef struct {
    sso_event_type_t kind;     // PREVIEW | TX_COMMAND_SENT | TX_NOT_SENT
    char             ts[16];   // HH:MM:SS
    char             ascii[SSO_TX_TEXT_MAX];
    char             tx_not_sent_reason[24];
} tx_log_entry_t;
#define TX_LOG_SIZE 8
static tx_log_entry_t g_tx_log[TX_LOG_SIZE];
static size_t         g_tx_log_count = 0;

// Persistent on-disk TX log. JSONL — one encoded sso_event_t per line.
// Opened lazily on the first event after g_pass_folder is set, kept
// open for the rest of the process, fflushed after every line so a
// crash mid-pass doesn't lose the last command sent. Captures every
// preview / commit / not-sent — the same events that drive the in-memory
// ring above and the viewer-side mirror.
static FILE *g_tx_log_fp = NULL;
static char  g_tx_log_path[512] = "";

// Pull "HH:MM:SS" out of an event's ISO ts ("2026-05-14T13:22:01.450Z").
// Falls back to local clock if the event ts is empty/garbled.
static void tx_log_ts_from_event(const sso_event_t *evt,
                                 char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    if (evt && evt->ts[0]) {
        const char *t = strchr(evt->ts, 'T');
        if (t && strlen(t) >= 9) {
            size_t n = 8;
            if (n >= out_size) n = out_size - 1;
            memcpy(out, t + 1, n);
            out[n] = '\0';
            return;
        }
    }
    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    snprintf(out, out_size, "%02d:%02d:%02d",
             lt.tm_hour, lt.tm_min, lt.tm_sec);
}

// Append one event to <pass_folder>/tx.log as a JSON line. Opens the
// file lazily; no-op when g_pass_folder isn't set yet (so events that
// arrive before pass-folder bring-up land in the in-memory ring but
// aren't dropped silently — they just don't reach disk until the
// folder exists). fflush after every write so a SIGKILL mid-pass
// preserves the last command sent.
static void tx_log_file_append(const sso_event_t *evt)
{
    if (!evt) return;
    if (g_tx_log_fp == NULL) {
        if (g_pass_folder[0] == '\0') return;
        char path[512];
        snprintf(path, sizeof path, "%.500s/tx.log", g_pass_folder);
        FILE *fp = fopen(path, "a");
        if (!fp) return;
        snprintf(g_tx_log_path, sizeof g_tx_log_path, "%s", path);
        g_tx_log_fp = fp;
    }
    char buf[2048];
    if (sso_event_encode(evt, buf, sizeof buf) != 0) return;
    // sso_event_encode already terminates with "}\n" — don't add another.
    fputs(buf, g_tx_log_fp);
    fflush(g_tx_log_fp);
}

// Push an event into the ring. PREVIEW events overwrite a trailing
// PREVIEW entry (live cursor-style update). SENT promotes a trailing
// PREVIEW to SENT, or appends a fresh entry. NOT_SENT appends with the
// status string filled in for rendering.
static void tx_log_push(const sso_event_t *evt)
{
    if (!evt) return;
    if (evt->type != SSO_EVT_TX_COMMAND_PREVIEW
     && evt->type != SSO_EVT_TX_COMMAND_SENT
     && evt->type != SSO_EVT_TX_NOT_SENT) return;

    tx_log_file_append(evt);

    tx_log_entry_t entry;
    memset(&entry, 0, sizeof entry);
    entry.kind = evt->type;
    tx_log_ts_from_event(evt, entry.ts, sizeof entry.ts);
    snprintf(entry.ascii, sizeof entry.ascii, "%s", evt->ascii);
    snprintf(entry.tx_not_sent_reason, sizeof entry.tx_not_sent_reason, "%s",
             evt->tx_not_sent_reason);

    if (evt->type == SSO_EVT_TX_COMMAND_PREVIEW
        && g_tx_log_count > 0
        && g_tx_log[g_tx_log_count - 1].kind == SSO_EVT_TX_COMMAND_PREVIEW) {
        g_tx_log[g_tx_log_count - 1] = entry;
        return;
    }
    if (evt->type == SSO_EVT_TX_COMMAND_SENT
        && g_tx_log_count > 0
        && g_tx_log[g_tx_log_count - 1].kind == SSO_EVT_TX_COMMAND_PREVIEW) {
        // Promote the trailing draft in-place.
        g_tx_log[g_tx_log_count - 1] = entry;
        return;
    }
    if (g_tx_log_count < TX_LOG_SIZE) {
        g_tx_log[g_tx_log_count++] = entry;
    } else {
        memmove(&g_tx_log[0], &g_tx_log[1],
                sizeof(g_tx_log[0]) * (TX_LOG_SIZE - 1));
        g_tx_log[TX_LOG_SIZE - 1] = entry;
    }
}

// Render the TX log at rows [start_row .. start_row + (TX_LOG_SIZE+1)).
// Caller picks the column. Title line + one row per entry. Newest at
// the bottom; PREVIEW lines render with A_BOLD, SENT/NOT_SENT with A_DIM.
static void render_tx_log_panel(int start_row, int col)
{
    int row = start_row;
    // Cap width so clrtoeol-equivalent padding doesn't wipe the
    // vertical ribbon (rendered at col COLS-2). Leave a 1-col gap.
    int safe_w = COLS - col - 3;
    if (safe_w < 8)   safe_w = 8;
    if (safe_w > 200) safe_w = 200;

    mvprintw(row++, col, "%-*.*s", safe_w, safe_w, "TX log");

    for (size_t i = 0; i < g_tx_log_count; ++i) {
        const tx_log_entry_t *e = &g_tx_log[i];
        const char *tag = "sent>  ";
        int attr = A_DIM;
        if (e->kind == SSO_EVT_TX_COMMAND_PREVIEW) {
            tag = "draft> ";
            attr = A_BOLD;
        } else if (e->kind == SSO_EVT_TX_NOT_SENT) {
            tag = "notsent>";
            attr = A_DIM;
        }
        // Render the FULL payload — let safe_w (the column width)
        // be the only truncation. Hard-coded 40/60-char caps here
        // chopped the operator's command text mid-string for any
        // payload longer than that, even when the panel had room.
        // Holds the timestamp, tag, the full ascii command, and the
        // not-sent reason. Sized off SSO_TX_TEXT_MAX so it tracks the
        // command field — a fixed 256 here truncated the line once ascii
        // widened from 160 to 256.
        char line[SSO_TX_TEXT_MAX + 64];
        if (e->kind == SSO_EVT_TX_NOT_SENT && e->tx_not_sent_reason[0]) {
            snprintf(line, sizeof line, "%s  %s %s  [%s]",
                     e->ts, tag, e->ascii, e->tx_not_sent_reason);
        } else {
            snprintf(line, sizeof line, "%s  %s %s",
                     e->ts, tag, e->ascii);
        }
        attron(attr);
        mvprintw(row++, col, "%-*.*s", safe_w, safe_w, line);
        attroff(attr);
    }
}

// Latest broadcast snapshot, kept so a newly-connecting viewer gets
// state in its WELCOME response without having to wait up to 500 ms
// for the next periodic STATE broadcast.
static int    g_last_state_valid     = 0;
static char   g_last_state_sat[64]   = "";
static double g_last_state_az        = 0.0;
static double g_last_state_el        = 0.0;
static long   g_last_state_freq_hz   = 0;
static double g_last_state_doppler   = 0.0;
static char   g_last_state_tle[256]  = "";
static double g_last_state_tgt_az    = 0.0;
static double g_last_state_tgt_el    = 0.0;
static int    g_last_state_flip      = 0;
static int    g_last_state_in_pass   = 0;
static int    g_last_state_tracking  = 0;
static int    g_last_state_has_rot   = 0;
static double g_last_state_jul       = 0.0;
static char   g_last_state_idesg[9]  = "";
static double g_last_state_epoch_min   = 0.0;
static double g_last_state_min_visible = 0.0;
static double g_last_state_min_above_0 = 0.0;
static double g_last_state_min_above_30 = 0.0;
static double g_last_state_max_el      = 0.0;
static double g_last_state_pred_az     = 0.0;
static double g_last_state_pred_el     = 0.0;
static double g_last_state_alt_km      = 0.0;
static double g_last_state_lat_deg     = 0.0;
static double g_last_state_lon_deg     = 0.0;
static double g_last_state_speed_kms   = 0.0;
static double g_last_state_range_km    = 0.0;
static double g_last_state_rrate_kms   = 0.0;

// Snapshot the operator's live RX panel data into a self-contained
// struct so the same renderer can be driven by either the operator
// (reading rx_session + g_ribbon_*) or the viewer (filling it from a
// STATE event). RX_PT_COUNT comes from rx_session.h. ribbon is a
// nul-terminated string of glyph-index chars (' ' or '0'..'7'); empty
// when no samples have arrived yet.
//
// RX_PT_COUNT is gated by WITH_USRP_B210 (it lives in rx_session.h).
// On builds without B210 we still want the viewer to draw the panel
// from broadcast data, so define a fallback so the struct compiles
// everywhere.
#ifdef SSO_WITH_SDR
#define RX_PANEL_PT_COUNT RX_PT_COUNT
#define RX_PANEL_PAYLOAD_MAX RX_LAST_PAYLOAD_MAX
#define RX_PANEL_SUMMARY_MAX RX_LAST_SUMMARY_MAX
#else
#define RX_PANEL_PT_COUNT 6
#define RX_PANEL_PAYLOAD_MAX 64
#define RX_PANEL_SUMMARY_MAX 160
#endif

// Labels for the six RX packet-type slots, in the same order as the
// RX_PT_* enum (rx_session.h). Defined here so the viewer build —
// which doesn't link rx_session.c — can render the panel without
// pulling in the WITH_USRP_B210 codepath.
static const char *rx_panel_pt_label(int slot)
{
    static const char *labels[RX_PANEL_PT_COUNT] = {
        "beacon", "periph", "log", "tcmd", "bulk", "other",
    };
    if (slot < 0 || slot >= RX_PANEL_PT_COUNT) return "?";
    return labels[slot];
}

typedef struct {
    int        have_session;
    int        rec_active;
    char       sdr_name[32];       // active SDR (operator-side; "" for viewers)
    int        can_tx;             // 0 => RX-only backend (operator-side)
    double     rx_freq_hz;         // effective Doppler-shifted carrier
    double     rx_lo_hz;           // hardware SDR LO (without the
                                   // intentional LO offset added back)
    double     rx_bandwidth_hz;    // post-decim sample rate (BW = ±half)
    double     peak_dbfs;
    double     rms_dbfs;
    uint64_t   frames_total;
    char       last_frame_summary[80]; // "<ts>  N bytes" or empty
    double     age_s;                  // <0 = no frame yet
    uint64_t   pt_count[RX_PANEL_PT_COUNT];
    int        pt_payload_len[RX_PANEL_PT_COUNT];
    uint8_t    pt_payload[RX_PANEL_PT_COUNT][RX_PANEL_PAYLOAD_MAX];
    // One-line decoded summary built by rx_session when the payload
    // sniffs as a known FrontierSat packet type. Empty = no decode
    // available; render falls back to the hex preview above.
    char       pt_summary[RX_PANEL_PT_COUNT][RX_PANEL_SUMMARY_MAX];
    int        ribbon_n;
    char       ribbon[RIBBON_LEN + 1];
    // Parallel array: peak dBFS for the i-th second back. Clamped into
    // int8 (dBFS is naturally -90..0, well inside int8's range).
    int8_t     ribbon_peak[RIBBON_LEN];
    // frames_total above is the live IQ-domain chain (the one that
    // writes the DB + drives the per-type panel). The two fields
    // below are the shadow counts from the PCM/FM-audio and Viterbi
    // MLSE chains running in parallel on the same window — pure A/B
    // diagnostics so the operator can spot a chain regression.
    uint64_t   frames_pcm;
    uint64_t   frames_vit;
    // Optional warning row (e.g., low-disk). Empty when no warning.
    char       warning[80];
} rx_panel_data_t;

// Operator-side collector. Reads the live rx_session + g_ribbon globals
// into the struct. On non-B210 builds, only have_session=0 is filled.
static void rx_panel_collect_local(rx_panel_data_t *d)
{
    memset(d, 0, sizeof *d);
#ifdef SSO_WITH_SDR
    d->have_session = (g_rx_session != NULL);
    if (!d->have_session) return;
    d->rec_active = rx_session_wav_active(g_rx_session);
    snprintf(d->sdr_name, sizeof d->sdr_name, "%.31s",
             rx_session_sdr_name(g_rx_session));
    d->can_tx = rx_session_can_tx(g_rx_session);
    char last[sizeof d->last_frame_summary] = "";
    rx_session_snapshot(g_rx_session,
                        &d->frames_total,
                        &d->peak_dbfs,
                        &d->rms_dbfs,
                        &d->rx_freq_hz,
                        last, sizeof last);
    snprintf(d->last_frame_summary, sizeof d->last_frame_summary,
             "%s", last);
    // Hardware LO and captured bandwidth so the panel can show the
    // operator where the SDR is actually listening alongside the
    // Doppler-shifted carrier line.
    d->rx_lo_hz        = rx_session_get_lo_freq_hz(g_rx_session);
    d->rx_bandwidth_hz = rx_session_get_bandwidth_hz(g_rx_session);
    d->frames_pcm = rx_session_pcm_frames(g_rx_session);
    d->frames_vit = rx_session_viterbi_frames(g_rx_session);
    rx_packet_type_stats_t pts[RX_PT_COUNT];
    rx_session_stats_snapshot(g_rx_session, pts, &d->age_s);
    for (int s = 0; s < RX_PT_COUNT; ++s) {
        d->pt_count[s]       = pts[s].count;
        d->pt_payload_len[s] = pts[s].last_payload_len;
        int copy = pts[s].last_payload_len;
        if (copy < 0) copy = 0;
        if (copy > RX_PANEL_PAYLOAD_MAX) copy = RX_PANEL_PAYLOAD_MAX;
        memcpy(d->pt_payload[s], pts[s].last_payload, (size_t) copy);
        snprintf(d->pt_summary[s], sizeof d->pt_summary[s],
                 "%.*s", (int)(sizeof d->pt_summary[s] - 1),
                 pts[s].last_summary);
    }
    // Wire format: ribbon[0] is the newest sample, ribbon[ribbon_n-1]
    // is the oldest. Each char is '.' for a normal second or '-' for
    // a 20 s tick. Tick test uses absolute push count (P) rather than
    // a fixed visual position so the tick row visibly walks upward at
    // 1 row/s; without that the strip looked frozen once it filled.
    // ribbon_peak[i] is the peak dBFS that was pushed at that same
    // second, clamped into int8 — rendered alongside the marker so
    // the operator sees the signal level on every row.
    int n = g_ribbon_count;
    if (n > (int) sizeof d->ribbon - 1) n = (int) sizeof d->ribbon - 1;
    long P = g_ribbon_push_count;
    // Bright-bin thresholds for ribbon character selection. With
    // iq_burst's 10 dB / N=512 FFT, stationary noise lights ~30 bins
    // and a CW carrier adds ~5-6 more. A wideband packet burst lights
    // 80+. Pick BRIGHT_HI well above the noise floor so '#' fires
    // only on real broadband events.
    const int BRIGHT_HI = 80;   // broadband — '#'
    for (int i = 0; i < n; ++i) {
        long abs_t = P - (long) i;
        int idx = (g_ribbon_head - 1 - i + RIBBON_LEN) % RIBBON_LEN;
        int bright = g_ribbon_bright[idx];
        if (bright >= BRIGHT_HI)             d->ribbon[i] = '#';
        else if (abs_t > 0 && (abs_t % 20) == 0) d->ribbon[i] = '_';
        else                                 d->ribbon[i] = '.';
        double dbfs = g_ribbon_peak[idx];
        long lr = lround(dbfs);
        if (lr > 127)  lr = 127;
        if (lr < -127) lr = -127;
        d->ribbon_peak[i] = (int8_t) lr;
    }
    d->ribbon[n] = '\0';
    d->ribbon_n  = n;
#endif
    // Warning row is filled regardless of the B210 build — even without
    // an SDR the operator could be running short on disk for logs.
    snprintf(d->warning, sizeof d->warning, "%s", g_low_disk_msg);
#ifdef SSO_WITH_SDR
    // A lost SDR outranks the low-disk notice — show it instead so the
    // operator sees immediately that RX has stopped.
    if (d->have_session && rx_session_device_lost(g_rx_session)) {
        snprintf(d->warning, sizeof d->warning,
                 "SDR DISCONNECTED - RX stopped (restart to resume RX)");
    }
#endif
}

// Defined further down (with the status panel); used here for the
// "last frame T+" age too.
static void format_duration_compact(double seconds, char *out, size_t n);

// Render the RX panel from a snapshot. Compiles even without UHD —
// the viewer feeds this from broadcast STATE so it can draw what the
// operator sees.
static void render_rx_panel(const rx_panel_data_t *d,
                            int *print_row, int print_col)
{
    if (print_row == NULL || d == NULL) return;
    int row = *print_row;
    int col = print_col;
    if (d->sdr_name[0]) {
        // Operator side: show the detected SDR + RX-only flag.
        mvprintw(row++, col, "%15s   %s%s%s", "SDR",
                 d->sdr_name,
                 d->can_tx ? "" : "  (RX-only)",
                 d->rec_active ? "  [REC]" : "");
    } else {
        mvprintw(row++, col, "%15s   %s%s", "SDR",
                 d->have_session ? "active" : "(offline)",
                 d->rec_active ? "  [REC]" : "");
    }
    clrtoeol();
    if (d->warning[0]) {
        // Red attribute pair 1 was initialised in init_window; fall
        // back gracefully if colors aren't available.
        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(row++, col, "%15s   %s", "WARNING", d->warning);
        attroff(COLOR_PAIR(1) | A_BOLD);
        clrtoeol();
    }
    if (!d->have_session) {
        *print_row = row;
        return;
    }
    mvprintw(row++, col, "%15s   %.6f MHz", "RX freq", d->rx_freq_hz / 1e6);
    clrtoeol();
    // Show the hardware SDR window underneath the carrier so the
    // operator can see where the B210 is actually listening: LO ± half
    // the post-decimation bandwidth. The LO sits deliberately off the
    // nominal carrier to keep the signal away from DC after software
    // Doppler tracking — see state.rx_lo_offset_hz.
    if (d->rx_lo_hz > 0.0 && d->rx_bandwidth_hz > 0.0) {
        double half_bw_khz = d->rx_bandwidth_hz / 2000.0;
        mvprintw(row++, col, "%15s   %.6f MHz \xc2\xb1%.0f kHz", "LO",
                 d->rx_lo_hz / 1e6, half_bw_khz);
        clrtoeol();
    }
    mvprintw(row++, col, "%15s   peak %+5.1f  rms %+5.1f dBFS",
             "level", d->peak_dbfs, d->rms_dbfs);
    clrtoeol();
    // frames_total = live IQ chain (DB-writer). pcm/vit shown as
    // shadow A/B counts so a chain regression is obvious at a glance.
    mvprintw(row++, col, "%15s   %llu iq (live)  pcm=%llu vit=%llu",
             "frames",
             (unsigned long long) d->frames_total,
             (unsigned long long) d->frames_pcm,
             (unsigned long long) d->frames_vit);
    clrtoeol();
    if (d->last_frame_summary[0]) {
        mvprintw(row++, col, "%15s   %s", "last frame", d->last_frame_summary);
        clrtoeol();
    }
    if (d->age_s >= 0.0) {
        char ago[32];
        format_duration_compact(d->age_s, ago, sizeof ago);
        mvprintw(row++, col, "%15s   %s ago", "last frame T+", ago);
        clrtoeol();
    }
    char by_type[160] = {0};
    size_t bt_len = 0;
    for (int s = 0; s < RX_PANEL_PT_COUNT; ++s) {
        int n = snprintf(by_type + bt_len, sizeof by_type - bt_len,
                         "%s%s=%llu",
                         (bt_len ? "  " : ""),
                         rx_panel_pt_label(s),
                         (unsigned long long) d->pt_count[s]);
        if (n < 0 || (size_t) n >= sizeof by_type - bt_len) break;
        bt_len += (size_t) n;
    }
    mvprintw(row++, col, "%15s   %s", "by type", by_type);
    clrtoeol();
    for (int s = 0; s < RX_PANEL_PT_COUNT; ++s) {
        if (d->pt_count[s] == 0 || d->pt_payload_len[s] <= 0) continue;
        // Prefer the decoded summary when rx_session was able to
        // produce one (basic beacon / tcmd response / log message);
        // fall back to a hex preview for unknown / unparsed types.
        if (d->pt_summary[s][0]) {
            mvprintw(row++, col, "%15s   %s",
                     rx_panel_pt_label(s), d->pt_summary[s]);
            clrtoeol();
            continue;
        }
        char hexbuf[3 * 24 + 16];
        size_t hex_len = 0;
        int n_show = d->pt_payload_len[s];
        if (n_show > 24) n_show = 24;
        for (int b = 0; b < n_show; ++b) {
            int w = snprintf(hexbuf + hex_len, sizeof hexbuf - hex_len,
                             "%s%02X",
                             (b ? " " : ""),
                             d->pt_payload[s][b]);
            if (w < 0 || (size_t) w >= sizeof hexbuf - hex_len) break;
            hex_len += (size_t) w;
        }
        mvprintw(row++, col, "%15s   %s%s", rx_panel_pt_label(s), hexbuf,
                 d->pt_payload_len[s] > n_show ? " ..." : "");
        clrtoeol();
    }
    // No in-panel ribbon: the timeline lives in its own vertical strip
    // on the right of the screen (render_ribbon_vertical), so the panel
    // body never wraps onto a second line.
    *print_row = row;
}

// Vertical ribbon strip. Bottom row (`bot_row`) holds the newest
// sample; older samples climb upward toward `top_row`. Each row is
// a graded peak-above-noise indicator (dots packed leftward from
// col-1) followed by the timeline marker ('.' or bold '-') at `col`.
// One dot per RIBBON_DB_PER_DOT dB above the rolling noise floor,
// capped at RIBBON_DOTS_MAX dots — bursts stand out as clusters of
// dots without numeric noise. Noise floor is the median of the
// available ribbon peaks: robust to a few seconds of burst activity
// per minute, no protocol change needed (peaks are already on the
// wire as int8 dBFS).
static void render_ribbon_vertical(const rx_panel_data_t *d,
                                   int top_row, int bot_row, int col)
{
    if (d == NULL || bot_row <= top_row) return;
    if (col < 4) return;

    const int RIBBON_DB_PER_DOT = 10;
    const int RIBBON_DOTS_MAX   = 4;

    // Median-of-peaks noise estimate. With RIBBON_LEN=60 and bursts
    // lasting a second or two, the median sits firmly in the noise
    // floor even during a busy pass.
    int noise_dbfs = -60;
    if (d->ribbon_n > 0) {
        int8_t buf[RIBBON_LEN];
        int n = d->ribbon_n < RIBBON_LEN ? d->ribbon_n : RIBBON_LEN;
        memcpy(buf, d->ribbon_peak, (size_t) n);
        for (int a = 1; a < n; ++a) {
            int8_t v = buf[a];
            int b = a;
            while (b > 0 && buf[b - 1] > v) { buf[b] = buf[b - 1]; --b; }
            buf[b] = v;
        }
        noise_dbfs = buf[n / 2];
    }

    int max_rows = bot_row - top_row + 1;
    for (int i = 0; i < max_rows; ++i) {
        int row = bot_row - i;
        for (int dx = 1; dx <= RIBBON_DOTS_MAX; ++dx) {
            mvaddch(row, col - dx, ' ');
        }
        if (i < d->ribbon_n) {
            int over = (int) d->ribbon_peak[i] - noise_dbfs;
            int dots = over / RIBBON_DB_PER_DOT;
            if (dots < 0)                dots = 0;
            if (dots > RIBBON_DOTS_MAX)  dots = RIBBON_DOTS_MAX;
            for (int k = 1; k <= dots; ++k) {
                mvaddch(row, col - k, '.');
            }
            char c = d->ribbon[i];
            if (c == '\0') c = ' ';
            // Bold both the 20-s tick AND the broadband-burst marker
            // so they stand out against the quiet '.' background.
            if (c == '_' || c == '#') {
                attron(A_BOLD);
                mvaddch(row, col, c);
                attroff(A_BOLD);
            } else {
                mvaddch(row, col, c);
            }
        } else {
            mvaddch(row, col, ' ');
        }
    }
}

// Snapshot the operator's RX panel into the wire-side fields of an
// event. Called for both STATE broadcasts and WELCOME replies so a
// newly-connecting viewer sees the same panel state everyone else does.
static void ipc_fill_rx_panel(sso_event_t *evt)
{
    rx_panel_data_t d;
    rx_panel_collect_local(&d);
    evt->rx_have_session = d.have_session;
    // Warning row is operator-wide (e.g. low disk), not gated on the
    // SDR — fill it before the have_session early-return.
    snprintf(evt->rx_warning, sizeof evt->rx_warning, "%s", d.warning);
    if (!d.have_session) return;
    evt->rx_rec_active   = d.rec_active;
    evt->rx_freq_hz      = d.rx_freq_hz;
    evt->rx_peak_dbfs    = d.peak_dbfs;
    evt->rx_rms_dbfs     = d.rms_dbfs;
    evt->rx_frames_total = (long) d.frames_total;
    evt->rx_frames_pcm   = (long) d.frames_pcm;
    evt->rx_frames_vit   = (long) d.frames_vit;
    snprintf(evt->rx_last_frame_summary,
             sizeof evt->rx_last_frame_summary, "%s", d.last_frame_summary);
    evt->rx_age_s = d.age_s;
    int slots = RX_PANEL_PT_COUNT < SSO_RX_PT_SLOTS
              ? RX_PANEL_PT_COUNT : SSO_RX_PT_SLOTS;
    for (int s = 0; s < slots; ++s) {
        evt->rx_pt_count[s]       = (long) d.pt_count[s];
        int pl = d.pt_payload_len[s];
        if (pl < 0) pl = 0;
        int wire_pl = pl;
        if (wire_pl > SSO_RX_PT_PAYLOAD_MAX) wire_pl = SSO_RX_PT_PAYLOAD_MAX;
        evt->rx_pt_payload_len[s] = pl;
        memcpy(evt->rx_pt_payload[s], d.pt_payload[s], (size_t) wire_pl);
        snprintf(evt->rx_pt_summary[s], sizeof evt->rx_pt_summary[s],
                 "%.*s", (int)(sizeof evt->rx_pt_summary[s] - 1),
                 d.pt_summary[s]);
    }
    int rn = d.ribbon_n;
    if (rn > SSO_RIBBON_MAX) rn = SSO_RIBBON_MAX;
    evt->rx_ribbon_n = rn;
    memcpy(evt->rx_ribbon, d.ribbon, (size_t) rn);
    evt->rx_ribbon[rn] = '\0';
    memcpy(evt->rx_ribbon_peak, d.ribbon_peak,
           (size_t) rn * sizeof evt->rx_ribbon_peak[0]);
}

static void ipc_broadcast_state(state_t *s,
                                  double az, double el,
                                  double downlink_freq,
                                  double doppler_delta_dl,
                                  double jul_utc) {
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_STATE);
    snprintf(evt.from, sizeof(evt.from), "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(evt.operator_user, sizeof(evt.operator_user), "%s",
             g_operator_user ? g_operator_user : "?");
    evt.has_state = 1;
    if (s->prediction.satellite_ephem.name) {
        snprintf(evt.satellite, sizeof(evt.satellite), "%s",
                 s->prediction.satellite_ephem.name);
    }
    evt.az = az;
    evt.el = el;
    evt.freq_hz = (long) downlink_freq;
    evt.doppler_hz = doppler_delta_dl;
    if (g_pass_folder[0]) {
        snprintf(evt.pass_folder, sizeof(evt.pass_folder), "%s",
                 g_pass_folder);
    }
    if (s->prediction.tles_filename) {
        snprintf(evt.tle_path, sizeof(evt.tle_path), "%s",
                 s->prediction.tles_filename);
    }
    evt.target_az = s->antenna_rotator.target_azimuth;
    evt.target_el = s->antenna_rotator.target_elevation;
    evt.flip      = s->antenna_rotator.flip_mode_pass;
    evt.in_pass   = s->in_pass;
    evt.tracking  = s->antenna_rotator.tracking;
    evt.has_rotator = s->have_antenna_rotator;
    evt.jul_utc   = jul_utc;

    // Prediction snapshot — viewer renders these verbatim.
    snprintf(evt.idesg, sizeof evt.idesg, "%s",
             s->prediction.satellite_ephem.tle.idesg);
    evt.epoch_min      = s->prediction.minutes_since_epoch;
    evt.min_visible    = s->prediction.predicted_minutes_until_visible;
    evt.min_above_0    = s->prediction.predicted_minutes_above_0_degrees;
    evt.min_above_30   = s->prediction.predicted_minutes_above_30_degrees;
    evt.max_el         = s->prediction.predicted_max_elevation;
    evt.pred_az        = s->prediction.satellite_ephem.azimuth;
    evt.pred_el        = s->prediction.satellite_ephem.elevation;
    evt.alt_km         = s->prediction.satellite_ephem.altitude_km;
    evt.lat_deg        = s->prediction.satellite_ephem.latitude;
    evt.lon_deg        = s->prediction.satellite_ephem.longitude;
    evt.speed_kms      = s->prediction.satellite_ephem.speed_km_s;
    evt.range_km       = s->prediction.satellite_ephem.range_km;
    evt.range_rate_kms = s->prediction.satellite_ephem.range_rate_km_s;

    // Auto-TCMD progress so viewers can follow the run without the modal.
    {
        int at_sent = 0, at_total = 0;
        const char *at_label = NULL;
        if (auto_tcmd_progress(&at_sent, &at_total, &at_label)) {
            evt.auto_tcmd_on    = 1;
            evt.auto_tcmd_sent  = at_sent;
            evt.auto_tcmd_total = at_total;
            snprintf(evt.auto_tcmd_state, sizeof evt.auto_tcmd_state,
                     "%s", at_label);
        }
    }
    sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
    size_t n = 0;
    if (n < sizeof(entries) / sizeof(entries[0])) {
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 g_operator_user ? g_operator_user : "?");
        snprintf(entries[n].role, sizeof(entries[n].role), "operator");
        entries[n].since[0] = '\0';
        n++;
    }
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char user[64], role[16], since[40];
    while (n < sizeof(entries) / sizeof(entries[0])
           && sso_ipc_server_next_client(g_ipc, &it, &cid,
                                          user, sizeof(user),
                                          role, sizeof(role),
                                          since, sizeof(since)) == 0) {
        if (!user[0]) continue;
        snprintf(entries[n].user, sizeof(entries[n].user), "%s", user);
        snprintf(entries[n].role, sizeof(entries[n].role), "%s",
                 role[0] ? role : "viewer");
        snprintf(entries[n].since, sizeof(entries[n].since), "%s", since);
        n++;
    }
    sso_event_set_roster(&evt, entries, n);
    ipc_fill_rx_panel(&evt);
    char buf[4096];
    if (sso_event_encode(&evt, buf, sizeof(buf)) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }

    // Cache for WELCOME replies so a viewer doesn't have to wait for
    // the next periodic broadcast to see anything.
    snprintf(g_last_state_sat, sizeof g_last_state_sat, "%s", evt.satellite);
    snprintf(g_last_state_tle, sizeof g_last_state_tle, "%s", evt.tle_path);
    g_last_state_az      = evt.az;
    g_last_state_el      = evt.el;
    g_last_state_freq_hz = evt.freq_hz;
    g_last_state_doppler = evt.doppler_hz;
    g_last_state_tgt_az  = evt.target_az;
    g_last_state_tgt_el  = evt.target_el;
    g_last_state_flip    = evt.flip;
    g_last_state_in_pass = evt.in_pass;
    g_last_state_tracking= evt.tracking;
    g_last_state_has_rot = evt.has_rotator;
    g_last_state_jul     = evt.jul_utc;
    snprintf(g_last_state_idesg, sizeof g_last_state_idesg, "%s", evt.idesg);
    g_last_state_epoch_min    = evt.epoch_min;
    g_last_state_min_visible  = evt.min_visible;
    g_last_state_min_above_0  = evt.min_above_0;
    g_last_state_min_above_30 = evt.min_above_30;
    g_last_state_max_el       = evt.max_el;
    g_last_state_pred_az      = evt.pred_az;
    g_last_state_pred_el      = evt.pred_el;
    g_last_state_alt_km       = evt.alt_km;
    g_last_state_lat_deg      = evt.lat_deg;
    g_last_state_lon_deg      = evt.lon_deg;
    g_last_state_speed_kms    = evt.speed_kms;
    g_last_state_range_km     = evt.range_km;
    g_last_state_rrate_kms    = evt.range_rate_kms;
    g_last_state_valid   = 1;
}

static void ipc_on_event(sso_ipc_server_t *srv, sso_client_id_t id,
                         const sso_event_t *evt, void *user) {
    (void) user;
    if (evt->type != SSO_EVT_HELLO) return;
    sso_event_t welcome;
    sso_event_init(&welcome, SSO_EVT_WELCOME);
    snprintf(welcome.from, sizeof(welcome.from), "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(welcome.operator_user, sizeof(welcome.operator_user), "%s",
             g_operator_user ? g_operator_user : "?");
    if (g_pass_folder[0]) {
        snprintf(welcome.pass_folder, sizeof(welcome.pass_folder), "%s",
                 g_pass_folder);
    }
    if (g_last_state_valid) {
        welcome.has_state   = 1;
        snprintf(welcome.satellite, sizeof welcome.satellite,
                 "%s", g_last_state_sat);
        snprintf(welcome.tle_path, sizeof welcome.tle_path,
                 "%s", g_last_state_tle);
        welcome.az          = g_last_state_az;
        welcome.el          = g_last_state_el;
        welcome.freq_hz     = g_last_state_freq_hz;
        welcome.doppler_hz  = g_last_state_doppler;
        welcome.target_az   = g_last_state_tgt_az;
        welcome.target_el   = g_last_state_tgt_el;
        welcome.flip        = g_last_state_flip;
        welcome.in_pass     = g_last_state_in_pass;
        welcome.tracking    = g_last_state_tracking;
        welcome.has_rotator = g_last_state_has_rot;
        welcome.jul_utc     = g_last_state_jul;
        snprintf(welcome.idesg, sizeof welcome.idesg, "%s", g_last_state_idesg);
        welcome.epoch_min      = g_last_state_epoch_min;
        welcome.min_visible    = g_last_state_min_visible;
        welcome.min_above_0    = g_last_state_min_above_0;
        welcome.min_above_30   = g_last_state_min_above_30;
        welcome.max_el         = g_last_state_max_el;
        welcome.pred_az        = g_last_state_pred_az;
        welcome.pred_el        = g_last_state_pred_el;
        welcome.alt_km         = g_last_state_alt_km;
        welcome.lat_deg        = g_last_state_lat_deg;
        welcome.lon_deg        = g_last_state_lon_deg;
        welcome.speed_kms      = g_last_state_speed_kms;
        welcome.range_km       = g_last_state_range_km;
        welcome.range_rate_kms = g_last_state_rrate_kms;
        // Auto-TCMD progress reads the live modal state (like
        // ipc_fill_rx_panel below) — no g_last_state_* cache needed.
        {
            int at_sent = 0, at_total = 0;
            const char *at_label = NULL;
            if (auto_tcmd_progress(&at_sent, &at_total, &at_label)) {
                welcome.auto_tcmd_on    = 1;
                welcome.auto_tcmd_sent  = at_sent;
                welcome.auto_tcmd_total = at_total;
                snprintf(welcome.auto_tcmd_state,
                         sizeof welcome.auto_tcmd_state, "%s", at_label);
            }
        }
        // Roster — operator first, then the existing clients we know
        // of. The newly-connecting client is already in the slot table
        // (slot_dispatch_line ran first) but its role isn't populated
        // until HELLO is processed; that's why we iterate via
        // sso_ipc_server_next_client, which surfaces what HELLO set.
        sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
        size_t n = 0;
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 g_operator_user ? g_operator_user : "?");
        snprintf(entries[n].role, sizeof(entries[n].role), "operator");
        entries[n].since[0] = '\0';
        n++;
        sso_ipc_iter_t it = {0};
        sso_client_id_t cid;
        char ruser[64], rrole[16], rsince[40];
        while (n < sizeof(entries) / sizeof(entries[0])
               && sso_ipc_server_next_client(srv, &it, &cid,
                                              ruser, sizeof(ruser),
                                              rrole, sizeof(rrole),
                                              rsince, sizeof(rsince)) == 0) {
            if (!ruser[0]) continue;
            snprintf(entries[n].user,  sizeof(entries[n].user),  "%s", ruser);
            snprintf(entries[n].role,  sizeof(entries[n].role),  "%s",
                     rrole[0] ? rrole : "viewer");
            snprintf(entries[n].since, sizeof(entries[n].since), "%s", rsince);
            n++;
        }
        sso_event_set_roster(&welcome, entries, n);
        ipc_fill_rx_panel(&welcome);
    }
    char buf[4096];
    if (sso_event_encode(&welcome, buf, sizeof(buf)) == 0) {
        sso_ipc_server_send(srv, id, buf);
    }
}

// --- TX compose modal (operator side) -----------------------------
//
// Opened with `t` from the operator's main UI when the keyboard is
// unlocked. As the operator edits a field the modal broadcasts a
// debounced SSO_EVT_TX_COMMAND_PREVIEW so viewers see the draft and
// can call out typos. On Enter the modal stashes the parsed request in
// g_tx_request; the main loop submits it to rx_session, which runs the
// burst on its dedicated TX thread (no IPC round-trip, since the B210
// now lives in this process; RX keeps streaming on the worker thread
// throughout). ACK + COMMAND_SENT events are published locally for
// viewer fan-out.

// Compose modal is intentionally minimal: a single payload line
// (always ASCII, prefilled with "CTS1+"), a TX-power-in-dB field,
// and the --allow-tx checkbox. CSP src/dst/dport/sport/prio, freq,
// repeat/gap, and the secondary allow-flags are hard-coded to the
// FrontierSat defaults inside tx_compose_fill_event.
typedef enum {
    TXF_PAYLOAD = 0,
    TXF_POWER,
    TXF_ALLOW_TX,
    TXF_COUNT,
} tx_field_t;

typedef struct {
    // Big enough to type a full RF telecommand. tx_field_insert caps the
    // payload at TCMD_RF_MAX_LEN (215) chars — the over-the-air limit —
    // so the extra room here is headroom, not a typeable length.
    char payload[256];
    char power[12];           // TX power in dB
    int  allow_tx;
    tx_field_t focus;
    int  preview_dirty;
    struct timespec last_edit;
    char status_msg[160];
    // Per-field text cursor (only meaningful for the text fields —
    // payload, power). 0..strlen(buf). Bumped/clamped by every edit
    // helper below.
    int  cursors[TXF_COUNT];
    // Payload-history navigation state. history_idx == -1 means
    // "editing the current draft" (the live payload buffer); 0..N-1
    // points at g_tx_history[i] (newest at 0). When stepping into
    // history we stash the live draft into history_saved_edit so
    // DOWN can restore it.
    int  history_idx;
    char history_saved_edit[256];
} tx_compose_t;

// Survives Esc / commit so the operator can reopen and pick up the
// previous typed string. First open seeds it with "CTS1+" — the OBC's
// CTS1 telecommand prefix.
static char g_tx_last_payload[256] = "CTS1+";
static char g_tx_last_power[12]    = "80.0";
// Same idea for the --allow-tx checkbox: operators commonly send a
// series of commands during a pass and would rather not re-arm the
// safety gate between every one. Survives Esc + commit; cleared by
// process exit. Per-session, intentionally not persisted on disk.
static int  g_tx_last_allow_tx     = 0;

// Payload-only history ring (newest at index 0). Push happens on a
// successful commit; Esc-cancelled drafts don't enter history.
#define TX_HISTORY_MAX 32
static char g_tx_history[TX_HISTORY_MAX][256];
static int  g_tx_history_count = 0;

// Non-blocking modal state. The TX compose modal used to run a
// dedicated event loop inside run_tx_compose(), which froze the
// main loop's antenna control, screen redraws, and viewer broadcast
// for as long as the operator had the modal open. State now lives at
// file scope; the main loop ticks the modal alongside everything
// else, so tracking + rotator commands + IPC fanout keep flowing
// during composition. The modal window is drawn on top via a layered
// refresh helper.
static int           g_tx_compose_active        = 0;
static WINDOW       *g_tx_compose_win           = NULL;
static tx_compose_t  g_tx_compose_state;
static long          g_tx_compose_last_edit_ns  = 0;
static const long    g_tx_compose_debounce_ns   = 200000000L;

// --- Auto-TCMD modal ----------------------------------------------
//
// Drives a file of ASCII telecommands through the TX path automatically.
// Loaded via --tc-file=<path>; opened with 'A' (or `:auto`) from the
// operator UI. Each non-blank, non-comment line in the file is one
// CTS1+ telecommand. The operator picks TX power, how many times each
// command should be sent, and the inter-send delay. Once started the
// modal's tick runs alongside the main loop — non-blocking, like the
// TX compose modal — and queues one g_tx_request per shot, advancing
// through the file. Stops automatically when the satellite drops
// below the horizon (LOS) so an unattended run can't keep TXing after
// the pass. Every send goes through emit_tx_event_local, so the
// existing tx.log + viewer fanout capture all of them.
typedef enum {
    AUTO_F_POWER = 0,
    AUTO_F_REPEATS,
    AUTO_F_DELAY,
    AUTO_F_ALLOW_TX,
    AUTO_F_COUNT,
} auto_tcmd_field_t;

typedef enum {
    AUTO_STATE_SETUP = 0,
    AUTO_STATE_RUNNING,
    AUTO_STATE_STOPPED,    // user stopped, file not exhausted
    AUTO_STATE_DONE,       // file exhausted
    AUTO_STATE_PASS_OVER,  // LOS hit while running
} auto_tcmd_state_t;

typedef struct {
    // Commands loaded from --tc-file. commands[i] is one CTS1+ line,
    // trimmed of leading/trailing whitespace; comment lines (#...) and
    // blank lines are dropped at load.
    char **commands;
    int    n_commands;
    char   file_path[256];

    // Editable fields (text-edit semantics shared with TX compose).
    char power[12];
    char repeats[8];
    char delay_s[12];
    int  allow_tx;
    auto_tcmd_field_t focus;
    int               cursors[AUTO_F_COUNT];

    // Run state.
    auto_tcmd_state_t state;
    int    cmd_idx;        // index into commands[]
    int    repeat_idx;     // how many sends of commands[cmd_idx] so far
    int    repeats_total;  // parsed from repeats at start
    double delay_s_val;    // parsed from delay_s at start
    long   next_send_ns;
    long   start_ns;       // wall-clock at run start, for elapsed TX time
    int    sends_total;    // running tally — every queued burst
    // On-air seconds accumulated and total, computed from each
    // command's payload length using the AX100/9600/preroll/postroll
    // math in tx_burst.c. Drives the Progress "TX:" sub-line.
    double tx_seconds_spent;
    double tx_seconds_total;
    char   last_sent[SSO_TX_TEXT_MAX];   // full command text, not clipped
    char   status_msg[160];
} auto_tcmd_t;

static int          g_auto_tcmd_active           = 0;
static WINDOW      *g_auto_tcmd_win              = NULL;
static auto_tcmd_t  g_auto_tcmd;
// Path captured from --tc-file. The modal reads this lazily so the
// CLI can be parsed before all the modal infrastructure is up.
static char         g_auto_tcmd_file_path[512]   = "";

// Wall-clock seconds one auto-tcmd send occupies, end to end. Mirrors
// the framing and the fixed timing in tx_burst.c's build_iq / tx_burst_run:
//
//   frame_bytes = prefill(32) + ASM(4) + Golay(3)
//                 + csp_hdr(4) + payload + hmac(4) + rs_parity(32)
//                 + tailfill(1)
//               = 80 + payload
//
//   burst_s     = start_delay(0.5)            // UHD timed-start lead
//                 + g_tx_preroll_ms/1000       // modulated 0xAA carrier
//                 + frame_bytes * 8 / bit_rate // the frame itself
//                 + postroll(0.050)
//
// The start lead matters: tx_burst_run schedules the burst 0.5 s ahead
// and blocks until it completes, so each send is inhibited for the whole
// span -- leaving it out is most of the per-burst underestimate. auto-tcmd
// sends one burst per shot (repeat=1); the repeat count and inter-send
// delay are folded in by the caller, so this stays a per-send quantum.
static double auto_tcmd_burst_seconds(size_t payload_len) {
    const double start_delay_s = 0.500;   // tx_burst.c start_delay_s
    const double preroll_s     = (double) g_tx_preroll_ms * 1e-3;
    const double postroll_s    = 0.050;   // tx_burst.c postroll_ms
    const double bit_rate      = 9600.0;
    size_t frame_bytes = 80 + payload_len;
    return start_delay_s + preroll_s
         + ((double)(frame_bytes * 8) / bit_rate) + postroll_s;
}

// "Xm Ys" formatter for the Progress line. Caller's buffer needs ~16
// chars to be safe across reasonable durations.
static void fmt_minsec(double seconds, char *out, size_t cap) {
    if (seconds < 0.0) seconds = 0.0;
    long total = (long)(seconds + 0.5);
    long m = total / 60;
    long s = total % 60;
    snprintf(out, cap, "%ldm %lds", m, s);
}

// Trim leading and trailing whitespace in place; returns the (possibly
// advanced) start pointer. Used by the auto-tcmd file loader so the
// stored commands are clean for the wire.
static char *str_trim_inplace(char *s) {
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t'
                     || s[n - 1] == '\r' || s[n - 1] == '\n')) {
        s[--n] = '\0';
    }
    return s;
}

// Truncate an inline trailing comment from a command line in place. The
// rule (a '#' is a comment only when preceded by whitespace) is shared
// with agenda_check via agenda_find_inline_comment(); a '#' that is part
// of the command text is left intact -- a wrong telecommand is worse than
// an unstripped one. Whole-line comments (a leading '#') are handled by
// the caller before this is reached.
static void strip_inline_comment(char *s) {
    size_t cmd_len;
    agenda_find_inline_comment(s, &cmd_len);
    s[cmd_len] = '\0';
}

// Read commands from path; one per line. Whole-line comments (#...) and
// blank lines after trim are dropped, and an inline trailing comment
// (whitespace + #...) is stripped from each command. Returns 0 on
// success; allocates and stores in *out_commands / *out_n on success.
// Caller owns the allocation.
static int auto_tcmd_load_file(const char *path,
                               char ***out_commands, int *out_n)
{
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;
    int cap = 16, n = 0;
    char **arr = (char **) malloc((size_t) cap * sizeof(char *));
    if (!arr) { fclose(fp); return -1; }
    char line[512];
    while (fgets(line, sizeof line, fp)) {
        char *t = str_trim_inplace(line);
        if (t[0] == '\0' || t[0] == '#') continue;
        strip_inline_comment(t);
        if (t[0] == '\0') continue;
        if (n == cap) {
            int new_cap = cap * 2;
            char **new_arr = (char **) realloc(arr,
                (size_t) new_cap * sizeof(char *));
            if (!new_arr) {
                for (int i = 0; i < n; ++i) free(arr[i]);
                free(arr); fclose(fp); return -1;
            }
            arr = new_arr; cap = new_cap;
        }
        arr[n] = strdup(t);
        if (!arr[n]) {
            for (int i = 0; i < n; ++i) free(arr[i]);
            free(arr); fclose(fp); return -1;
        }
        n++;
    }
    fclose(fp);
    *out_commands = arr;
    *out_n        = n;
    return 0;
}

static void auto_tcmd_free_commands(char **commands, int n) {
    if (!commands) return;
    for (int i = 0; i < n; ++i) free(commands[i]);
    free(commands);
}

static void tx_history_push(const char *payload) {
    if (payload == NULL || payload[0] == '\0') return;
    if (g_tx_history_count > 0
        && strcmp(g_tx_history[0], payload) == 0) {
        return;  // suppress trivial duplicates of the most-recent entry
    }
    int keep = g_tx_history_count < TX_HISTORY_MAX - 1
             ? g_tx_history_count : TX_HISTORY_MAX - 1;
    for (int i = keep; i > 0; --i) {
        memcpy(g_tx_history[i], g_tx_history[i - 1], sizeof g_tx_history[0]);
    }
    snprintf(g_tx_history[0], sizeof g_tx_history[0], "%s", payload);
    if (g_tx_history_count < TX_HISTORY_MAX) g_tx_history_count++;
}

static void tx_compose_init(tx_compose_t *c) {
    memset(c, 0, sizeof *c);
    snprintf(c->payload, sizeof c->payload, "%s", g_tx_last_payload);
    snprintf(c->power,   sizeof c->power,   "%s", g_tx_last_power);
    c->allow_tx                = g_tx_last_allow_tx;
    c->cursors[TXF_PAYLOAD]    = (int) strlen(c->payload);
    c->cursors[TXF_POWER]      = (int) strlen(c->power);
    c->history_idx             = -1;
    snprintf(c->status_msg, sizeof c->status_msg,
             "edit; viewers see drafts ~200 ms after you stop typing");
}

static void tx_compose_remember(const tx_compose_t *c) {
    snprintf(g_tx_last_payload, sizeof g_tx_last_payload, "%s", c->payload);
    snprintf(g_tx_last_power,   sizeof g_tx_last_power,   "%s", c->power);
    g_tx_last_allow_tx = c->allow_tx;
}

static void tx_compose_summary(const tx_compose_t *c, char *out, size_t out_size) {
    if (out_size == 0) return;
    snprintf(out, out_size, "%s", c->payload[0] ? c->payload : "(empty)");
}

static void tx_compose_fill_event(const tx_compose_t *c, sso_event_t *evt) {
    snprintf(evt->tx_payload_kind, sizeof evt->tx_payload_kind, "ascii");
    snprintf(evt->tx_payload, sizeof evt->tx_payload, "%s", c->payload);
    // CSP defaults match cts_send -> FrontierSat OBC (CTS1 cmd handler).
    evt->tx_csp_src   = 10;
    evt->tx_csp_dst   = 1;
    evt->tx_csp_dport = 7;
    evt->tx_csp_sport = 16;
    evt->tx_csp_prio  = 2;
#ifdef SSO_WITH_SDR
    evt->tx_freq_hz   = g_tx_freq_hz_doppler;
#else
    evt->tx_freq_hz   = (long) FRONTIERSAT_CARRIER_HZ;
#endif
    evt->tx_gain_db   = atof(c->power);
    evt->tx_repeat    = 1;
    evt->tx_gap_ms    = 200;
    evt->tx_allow_tx         = c->allow_tx;
    evt->tx_allow_high_power = 0;
    evt->tx_allow_hf_tx      = 0;
    char summary[SSO_TX_TEXT_MAX];
    tx_compose_summary(c, summary, sizeof summary);
    snprintf(evt->ascii, sizeof evt->ascii, "%s", summary);
    snprintf(evt->from, sizeof evt->from, "%s",
             g_operator_user ? g_operator_user : "?");
}

static int tx_field_is_text(tx_field_t f) { return f == TXF_PAYLOAD; }
static int tx_field_is_decimal(tx_field_t f) { return f == TXF_POWER; }
static int tx_field_is_toggle(tx_field_t f) { return f == TXF_ALLOW_TX; }

static char *tx_field_buf(tx_compose_t *c, tx_field_t f, size_t *cap) {
    switch (f) {
        case TXF_PAYLOAD: *cap = sizeof c->payload; return c->payload;
        case TXF_POWER:   *cap = sizeof c->power;   return c->power;
        default:          *cap = 0; return NULL;
    }
}

// Clamp the per-field cursor into [0, strlen(buf)]. Called after any
// op that might leave the cursor past the end (focus change, history
// recall) so subsequent insert/delete don't run off the buffer.
static void tx_field_clamp_cursor(tx_compose_t *c, tx_field_t f) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, f, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (c->cursors[f] < 0) c->cursors[f] = 0;
    if (c->cursors[f] > n) c->cursors[f] = n;
}

static void tx_field_insert(tx_compose_t *c, int ch) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    // The payload is transmitted verbatim, so cap typing at the over-the-
    // air limit (TCMD_RF_MAX_LEN chars) even though the buffer is larger:
    // a longer telecommand can't be framed for the radio.
    if (c->focus == TXF_PAYLOAD && cap > (size_t) TCMD_RF_MAX_LEN + 1)
        cap = (size_t) TCMD_RF_MAX_LEN + 1;
    int n = (int) strlen(buf);
    if (n + 1 >= (int) cap) {
        if (c->focus == TXF_PAYLOAD)
            snprintf(c->status_msg, sizeof c->status_msg,
                     "at the %d-char RF uplink limit", TCMD_RF_MAX_LEN);
        return;
    }
    int accept = 0;
    if (tx_field_is_text(c->focus)) {
        accept = (ch >= 32 && ch < 127);
    } else if (tx_field_is_decimal(c->focus)) {
        accept = (ch >= '0' && ch <= '9') || ch == '.' || ch == '-';
    }
    if (!accept) return;
    int cur = c->cursors[c->focus];
    if (cur < 0) cur = 0;
    if (cur > n) cur = n;
    // Shift the tail (including the existing nul) right by one.
    memmove(buf + cur + 1, buf + cur, (size_t)(n - cur + 1));
    buf[cur] = (char) ch;
    c->cursors[c->focus] = cur + 1;
    c->preview_dirty = 1;
    // Any edit cancels history navigation — the operator is now off
    // the recalled string, the next UP should walk history fresh.
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_backspace(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = c->cursors[c->focus];
    if (cur <= 0 || n == 0) return;
    memmove(buf + cur - 1, buf + cur, (size_t)(n - cur + 1));
    c->cursors[c->focus] = cur - 1;
    c->preview_dirty = 1;
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_delete(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = c->cursors[c->focus];
    if (cur >= n) return;
    memmove(buf + cur, buf + cur + 1, (size_t)(n - cur));
    c->preview_dirty = 1;
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_kill_to_end(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = c->cursors[c->focus];
    if (cur >= n) return;
    buf[cur] = '\0';
    c->preview_dirty = 1;
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_left(tx_compose_t *c) {
    size_t cap = 0;
    if (!tx_field_buf(c, c->focus, &cap)) return;
    if (c->cursors[c->focus] > 0) c->cursors[c->focus]--;
}

static void tx_field_right(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (c->cursors[c->focus] < n) c->cursors[c->focus]++;
}

static void tx_field_home(tx_compose_t *c) {
    size_t cap = 0;
    if (!tx_field_buf(c, c->focus, &cap)) return;
    c->cursors[c->focus] = 0;
}

static void tx_field_end(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    c->cursors[c->focus] = (int) strlen(buf);
}

// direction = -1 (UP, older) or +1 (DOWN, newer). No-op when focus
// is not on the payload field, when history is empty, or at the edge.
static void tx_history_recall(tx_compose_t *c, int direction) {
    if (c->focus != TXF_PAYLOAD) return;
    if (g_tx_history_count == 0) return;
    int step = (direction < 0) ? +1 : -1;
    int new_idx = c->history_idx + step;
    if (new_idx < -1) return;
    if (new_idx >= g_tx_history_count) return;
    if (c->history_idx == -1 && new_idx >= 0) {
        snprintf(c->history_saved_edit, sizeof c->history_saved_edit,
                 "%s", c->payload);
    }
    if (new_idx == -1) {
        snprintf(c->payload, sizeof c->payload, "%s",
                 c->history_saved_edit);
    } else {
        snprintf(c->payload, sizeof c->payload, "%s",
                 g_tx_history[new_idx]);
    }
    c->cursors[TXF_PAYLOAD] = (int) strlen(c->payload);
    c->history_idx          = new_idx;
    c->preview_dirty        = 1;
}

// Tiny CSI fallback parser for terminals where ncurses' keypad mode
// doesn't translate arrow / nav keys into KEY_* (notably some tmux
// configurations). Same idea as cmd_drain_csi in the `:` prompt.
// Returns a KEY_* code on success, or -1 if the lookahead isn't a
// CSI we recognise.
static int tx_drain_csi(WINDOW *w) {
    int b1 = wgetch(w);
    if (b1 == ERR || b1 != '[') return -1;
    int b2 = wgetch(w);
    if (b2 == ERR) return -1;
    switch (b2) {
        case 'A': return KEY_UP;
        case 'B': return KEY_DOWN;
        case 'C': return KEY_RIGHT;
        case 'D': return KEY_LEFT;
        case 'H': return KEY_HOME;
        case 'F': return KEY_END;
        default: break;
    }
    if (b2 >= '0' && b2 <= '9') {
        int b3 = wgetch(w);
        if (b3 == '~') {
            switch (b2) {
                case '1': return KEY_HOME;
                case '3': return KEY_DC;
                case '4': return KEY_END;
                case '7': return KEY_HOME;
                case '8': return KEY_END;
                default: break;
            }
        }
    }
    return -1;
}

static void tx_field_toggle(tx_compose_t *c) {
    if (c->focus == TXF_ALLOW_TX) {
        c->allow_tx = !c->allow_tx;
        c->preview_dirty = 1;
    }
}

// Single-line redraw helper: prints a label + value, applies A_REVERSE
// on the value when this field is focused. Used for the allow-tx
// checkbox where there's no cursor.
static void tx_draw_field(WINDOW *w, int row, int col, int focused,
                          const char *label, const char *value) {
    mvwprintw(w, row, col, "%s", label);
    if (focused) wattron(w, A_REVERSE);
    wprintw(w, "%s", value);
    if (focused) wattroff(w, A_REVERSE);
    wclrtoeol(w);
}

// Cursor-aware text-field renderer for payload + power. The value is
// drawn cell-by-cell across value_w columns; the cursor cell (if
// focused) is inverted, the remainder normal, and any space past the
// value is filled with plain spaces. When the cursor sits past the
// visible window the viewport scrolls so it stays at the right edge.
static void tx_draw_text_field(WINDOW *w, int row, int col,
                               const char *label, const char *value,
                               int value_w, int focused, int cursor)
{
    mvwprintw(w, row, col, "%s", label);
    int x = col + (int) strlen(label);
    int n = (int) strlen(value);
    int start = 0;
    if (focused && cursor > value_w - 1) start = cursor - value_w + 1;
    for (int i = 0; i < value_w; ++i) {
        int idx = start + i;
        char ch = (idx < n) ? value[idx] : ' ';
        chtype out = (chtype)(unsigned char) ch;
        if (focused && idx == cursor) {
            wattron(w, A_REVERSE);
            mvwaddch(w, row, x + i, out);
            wattroff(w, A_REVERSE);
        } else {
            mvwaddch(w, row, x + i, out);
        }
    }
    wclrtoeol(w);
}

// Plain ASCII modal box drawer. ncurses' built-in box() uses ACS
// line-drawing glyphs, which fall back to raw vt100 alphabet on
// terminals where the charset map fails — that's where the "q" /
// "x" / "l m k j" letter soup came from. We can't safely upgrade to
// Unicode line-drawing here either: the project links against
// narrow ncurses (libncurses), so writing UTF-8 bytes through
// mvwprintw makes ncurses count each byte as its own screen cell,
// which mangles every subsequent write on the same row. Plain
// ASCII '+'/'-'/'|' is uglier but bulletproof on every locale and
// every terminal we plausibly run under, including dumb SSH
// sessions. Switching to libncursesw + add_wch is the only way to
// get nice line glyphs; left as a future change.
static void draw_box(WINDOW *w) {
    int h = getmaxy(w), wd = getmaxx(w);
    if (h < 2 || wd < 2) return;
    mvwaddch(w, 0, 0, '+');
    for (int x = 1; x < wd - 1; ++x) mvwaddch(w, 0, x, '-');
    mvwaddch(w, 0, wd - 1, '+');
    for (int y = 1; y < h - 1; ++y) {
        mvwaddch(w, y, 0, '|');
        mvwaddch(w, y, wd - 1, '|');
    }
    mvwaddch(w, h - 1, 0, '+');
    for (int x = 1; x < wd - 1; ++x) mvwaddch(w, h - 1, x, '-');
    mvwaddch(w, h - 1, wd - 1, '+');
}

static void tx_compose_draw(WINDOW *w, const tx_compose_t *c) {
    werase(w);
    draw_box(w);
    int width = getmaxx(w);
    int payload_w = width - 14;
    if (payload_w < 32) payload_w = 32;
    if (payload_w > (int) sizeof c->payload - 1)
        payload_w = (int) sizeof c->payload - 1;

    mvwprintw(w, 0, 2, " TX compose (operator: %s)%s ",
              g_operator_user ? g_operator_user : "?",
              g_no_tx ? "  [--no-tx]" : "");
#ifdef SSO_WITH_SDR
    mvwprintw(w, 1, 2,
              "B210: %s",
              g_rx_session ? "in-process (this binary)" : "(offline)");
#else
    mvwprintw(w, 1, 2, "B210: (this build has no UHD)");
#endif
    wclrtoeol(w);

    char buf[256];
    // Payload spans most of the modal width so a long telecommand
    // doesn't scroll off the right edge.
    tx_draw_text_field(w, 3, 2, "Payload  ",
                       c->payload, payload_w,
                       c->focus == TXF_PAYLOAD,
                       c->cursors[TXF_PAYLOAD]);

    tx_draw_text_field(w, 5, 2, "TX power ",
                       c->power, 8,
                       c->focus == TXF_POWER,
                       c->cursors[TXF_POWER]);
    mvwprintw(w, 5, 24, "dB  (B210 TX gain; 0..89.75)");
    wclrtoeol(w);

    snprintf(buf, sizeof buf, "[%c]", c->allow_tx ? 'x' : ' ');
    tx_draw_field(w, 7, 2, c->focus == TXF_ALLOW_TX,
                  "", buf);
    mvwprintw(w, 7, 7, "allow-tx  (required to key the PA)");
    wclrtoeol(w);

    char summary[SSO_TX_TEXT_MAX];
    tx_compose_summary(c, summary, sizeof summary);
    mvwprintw(w, 9, 2,  "Preview: %.*s",
              width - 12, summary);
    wclrtoeol(w);
    mvwprintw(w, 10, 2, "Status:  %.*s",
              width - 12, c->status_msg);
    wclrtoeol(w);

    mvwprintw(w, 12, 2,
              "Tab focus  Space toggle  Up/Down history  ^A/^E home/end  ^K kill  Enter send  Esc cancel");
    wclrtoeol(w);

    // Park the hardware cursor over the focused field's text cell so
    // the main loop's curs_set(1) lands a visible blinking cursor on
    // top of the A_REVERSE block. Toggle field has no cursor.
    if (c->focus == TXF_PAYLOAD) {
        int cur = c->cursors[TXF_PAYLOAD];
        int vis = (cur > payload_w - 1) ? (payload_w - 1) : cur;
        wmove(w, 3, 2 + 9 + vis);  // "Payload  " is 9 chars
    } else if (c->focus == TXF_POWER) {
        int cur = c->cursors[TXF_POWER];
        int vis = (cur > 7) ? 7 : cur;
        wmove(w, 5, 2 + 9 + vis);  // "TX power " is 9 chars
    }
    wrefresh(w);
}

static int tx_compose_validate(const tx_compose_t *c, char *err, size_t err_size) {
    if (!c->payload[0]) {
        snprintf(err, err_size, "empty payload");
        return -1;
    }
    if (strlen(c->payload) > (size_t) TCMD_RF_MAX_LEN) {
        snprintf(err, err_size,
                 "payload %zu chars exceeds the %d-char RF uplink limit",
                 strlen(c->payload), TCMD_RF_MAX_LEN);
        return -1;
    }
    if (!c->allow_tx) {
        snprintf(err, err_size, "--allow-tx is off; tick it before commit");
        return -1;
    }
    double db = atof(c->power);
    if (db < 0.0 || db > 89.75) {
        snprintf(err, err_size,
                 "TX power %.1f dB out of B210 range 0..89.75", db);
        return -1;
    }
    return 0;
}

static void tx_compose_broadcast_preview(const tx_compose_t *c) {
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_TX_COMMAND_PREVIEW);
    tx_compose_fill_event(c, &evt);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }
    // Mirror into our own ring buffer so the operator's TX log shows
    // the same draft line viewers are seeing.
    tx_log_push(&evt);
}

static int tx_compose_commit(const tx_compose_t *c, char *err, size_t err_size) {
#ifdef SSO_WITH_SDR
    if (g_no_tx) {
        snprintf(err, err_size,
                 "TX disabled by --no-tx (preview still goes to viewers)");
        return -1;
    }
    if (g_rx_session != NULL && !rx_session_can_tx(g_rx_session)) {
        snprintf(err, err_size,
                 "TX not supported by this SDR (RX-only backend)");
        return -1;
    }
    if (g_tx_request.pending) {
        snprintf(err, err_size, "previous burst still in flight");
        return -1;
    }
    // Expand a simple_sat_ops-directed "SSO+..." pseudo-command into the
    // concrete telecommand to transmit, with the clock captured now so the
    // embedded timestamp is current. A normal command passes through verbatim.
    sso_pseudo_ctx_t pc = { .now_ms    = sso_now_utc_ms(),
                            .tssent_ms = g_sso_pass_tssent_ms };
    char wire[512];
    char sso_err[160];
    sso_pseudo_status_t pst =
        sso_pseudo_expand(c->payload, &pc, wire, sizeof wire, sso_err, sizeof sso_err);
    if (pst != SSO_PSEUDO_OK && pst != SSO_PSEUDO_NOT_PSEUDO) {
        snprintf(err, err_size, "SSO+ expansion failed: %s", sso_err);
        return -1;
    }
    size_t n = strlen(wire);
    if (n == 0) {
        snprintf(err, err_size, "empty payload");
        return -1;
    }
    if (n > sizeof g_tx_request.payload) n = sizeof g_tx_request.payload;
    memcpy(g_tx_request.payload, wire, n);
    g_tx_request.payload_len  = n;
    g_tx_request.is_hex       = 0;  // always ascii in the simplified modal
    g_tx_request.csp_hdr      = (csp_v1_header_t){
        .prio  = 2, .src = 10, .dst = 1, .dport = 7, .sport = 16, .flags = 0,
    };
    g_tx_request.tx_freq_hz       = g_tx_freq_hz_doppler;
    g_tx_request.tx_gain_db       = atof(c->power);
    g_tx_request.repeat           = 1;
    g_tx_request.gap_ms           = 200;
    g_tx_request.preroll_ms       = g_tx_preroll_ms;
    g_tx_request.allow_high_power = 0;
    g_tx_request.allow_hf_tx      = 0;
    if (pst == SSO_PSEUDO_OK) {
        // Heritage: stash the SSO+ origin so the on-air summary notes it, and
        // bake the same note into the queue-time summary the dry-run /
        // rejected paths show.
        snprintf(g_tx_request.sso_origin, sizeof g_tx_request.sso_origin,
                 "%s", c->payload);
        snprintf(g_tx_request.summary, sizeof g_tx_request.summary,
                 "ascii:%.150s (replaced '%.80s')", wire, c->payload);
    } else {
        g_tx_request.sso_origin[0] = '\0';
        tx_compose_summary(c, g_tx_request.summary, sizeof g_tx_request.summary);
    }
    g_tx_request.pending = 1;
    {
        // Audit: TX commit — the moment the operator pressed Enter in
        // the compose modal and the burst was queued for the main loop
        // to actually transmit. The matching tx-command-sent (or
        // tx-not-sent) event lands when the burst returns (see
        // emit_tx_event_local site below).
        char det[512];
        snprintf(det, sizeof det,
                 "len=%zu freq_hz=%ld gain_db=%.1f payload=\"%.255s\"",
                 g_tx_request.payload_len,
                 (long) g_tx_request.tx_freq_hz,
                 g_tx_request.tx_gain_db,
                 c->payload);
        sso_audit_event("tx-commit", det);
    }
    return 0;
#else
    (void) c;
    snprintf(err, err_size, "this build has no B210 support");
    return -1;
#endif
}

#ifdef SSO_WITH_SDR
// Emit a TX event locally: push into the operator's own TX log and
// broadcast to viewers via the IPC server.
static void emit_tx_event_local(sso_event_type_t type,
                                 const char *summary,
                                 const char *ack_status)
{
    sso_event_t evt;
    sso_event_init(&evt, type);
    snprintf(evt.from, sizeof evt.from, "%s",
             g_operator_user ? g_operator_user : "?");
    if (summary && summary[0]) {
        snprintf(evt.ascii, sizeof evt.ascii, "%s", summary);
    }
    if (ack_status && ack_status[0]) {
        snprintf(evt.tx_not_sent_reason, sizeof evt.tx_not_sent_reason, "%s", ack_status);
    }
    tx_log_push(&evt);
    if (g_ipc) {
        char buf[2048];
        if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
            sso_ipc_server_broadcast(g_ipc, buf);
        }
    }
}

#endif

static long ts_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long) ts.tv_sec * 1000000000L + (long) ts.tv_nsec;
}

// Open the modal — allocate the window, seed the compose state, draw
// once, and flip g_tx_compose_active so the main loop starts ticking
// it. Idempotent: re-opening while already active is a no-op.
static void tx_compose_open(void) {
    if (!g_ipc) return;
    if (g_tx_compose_active) return;
    if (g_auto_tcmd_active) return;  // one modal at a time
    int h = 14, ww = 120;
    if (h > LINES) h = LINES;
    if (ww > COLS) ww = COLS;
    if (ww < 60)  ww = (COLS < 60) ? COLS : 60;
    g_tx_compose_win = newwin(h, ww, (LINES - h) / 2, (COLS - ww) / 2);
    if (!g_tx_compose_win) return;
    keypad(g_tx_compose_win, TRUE);
    nodelay(g_tx_compose_win, TRUE);
    tx_compose_init(&g_tx_compose_state);
#ifdef SSO_WITH_SDR
    // RX-only SDR (e.g. RTL-SDR): the burst can never reach the air, so
    // keep the allow-tx gate forced off. Compose + preview still work
    // (commit refuses with a clear message).
    if (g_rx_session != NULL && !rx_session_can_tx(g_rx_session)) {
        g_tx_compose_state.allow_tx = 0;
    }
#endif
    tx_compose_draw(g_tx_compose_win, &g_tx_compose_state);
    g_tx_compose_last_edit_ns = ts_now_ns();
    g_tx_compose_active = 1;
}

// Tear the modal down. Touchwin + refresh paints stdscr's cells back
// into the area the modal occupied so the operator's normal panels
// become visible again.
static void tx_compose_close(void) {
    if (g_tx_compose_win) {
        delwin(g_tx_compose_win);
        g_tx_compose_win = NULL;
    }
    g_tx_compose_active = 0;
    touchwin(stdscr);
    refresh();
}

// Consume one key (from stdscr's getch, which the main loop is doing).
// Returns 1 to keep the modal open, 0 when the operator's Enter or
// Esc closed it — the caller invokes tx_compose_close() in that case.
static int tx_compose_handle_key(int key) {
    if (!g_tx_compose_active) return 0;
    if (key == ERR) return 1;
    tx_compose_t *c = &g_tx_compose_state;
    WINDOW *w = g_tx_compose_win;
    int changed = 1;
    // Esc may be a bare cancel OR the start of a CSI sequence (arrow
    // keys, Home/End, Delete) when keypad mode can't translate them.
    if (key == 27) {
        int translated = tx_drain_csi(w);
        if (translated >= 0) {
            key = translated;
        } else {
            tx_compose_remember(c);
            return 0;
        }
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        char err[120];
        if (tx_compose_validate(c, err, sizeof err) != 0) {
            snprintf(c->status_msg, sizeof c->status_msg,
                     "rejected: %.*s",
                     (int)(sizeof c->status_msg - 16), err);
        } else if (tx_compose_commit(c, err, sizeof err) != 0) {
            snprintf(c->status_msg, sizeof c->status_msg,
                     "commit failed: %.*s",
                     (int)(sizeof c->status_msg - 20), err);
        } else {
            tx_history_push(c->payload);
            tx_compose_remember(c);
            return 0;
        }
    } else if (key == '\t') {
        c->focus = (tx_field_t) ((c->focus + 1) % TXF_COUNT);
        tx_field_clamp_cursor(c, c->focus);
    } else if (key == KEY_BTAB) {
        c->focus = (tx_field_t) ((c->focus + TXF_COUNT - 1) % TXF_COUNT);
        tx_field_clamp_cursor(c, c->focus);
    } else if (key == KEY_BACKSPACE || key == 127 || key == 8) {
        tx_field_backspace(c);
    } else if (key == KEY_DC || key == 4 /* Ctrl-D */) {
        tx_field_delete(c);
    } else if (key == 11 /* Ctrl-K */) {
        tx_field_kill_to_end(c);
    } else if (key == KEY_LEFT) {
        tx_field_left(c);
    } else if (key == KEY_RIGHT) {
        tx_field_right(c);
    } else if (key == KEY_HOME || key == 1 /* Ctrl-A */) {
        tx_field_home(c);
    } else if (key == KEY_END  || key == 5 /* Ctrl-E */) {
        tx_field_end(c);
    } else if (key == KEY_UP) {
        tx_history_recall(c, -1);
    } else if (key == KEY_DOWN) {
        tx_history_recall(c, +1);
    } else if (key == ' ' && tx_field_is_toggle(c->focus)) {
        tx_field_toggle(c);
    } else if (key >= 32 && key < 127) {
        tx_field_insert(c, key);
    } else {
        changed = 0;
    }
    if (changed) {
        g_tx_compose_last_edit_ns = ts_now_ns();
        tx_compose_draw(w, c);
    }
    return 1;
}

// Per-tick housekeeping. Pumps the debounced preview broadcast and
// re-renders if the broadcast fired (so the mirror line refreshes).
// Called every main-loop iteration when active.
static void tx_compose_pump(void) {
    if (!g_tx_compose_active) return;
    tx_compose_t *c = &g_tx_compose_state;
    if (c->preview_dirty
        && (ts_now_ns() - g_tx_compose_last_edit_ns) >= g_tx_compose_debounce_ns) {
        tx_compose_broadcast_preview(c);
        c->preview_dirty = 0;
        tx_compose_draw(g_tx_compose_win, c);
    }
}

// --- Auto-TCMD modal helpers --------------------------------------

static const char *auto_tcmd_state_label(auto_tcmd_state_t s) {
    switch (s) {
        case AUTO_STATE_SETUP:     return "idle";
        case AUTO_STATE_RUNNING:   return "running";
        case AUTO_STATE_STOPPED:   return "stopped";
        case AUTO_STATE_DONE:      return "done";
        case AUTO_STATE_PASS_OVER: return "pass-over";
    }
    return "?";
}

// Snapshot of the auto-tcmd run for the viewer broadcast: sends queued
// so far vs. the run's planned total (commands × repeats), plus the
// run-state label. Returns 1 when there is a run to report — the modal
// is open and Enter has started it (running, or finished and still on
// screen so viewers see the final tally). Returns 0 in setup or when
// the modal is closed, which drops the fields from the wire entirely.
static int auto_tcmd_progress(int *sent, int *total, const char **label) {
    const auto_tcmd_t *a = &g_auto_tcmd;
    if (!g_auto_tcmd_active || a->state == AUTO_STATE_SETUP) return 0;
    *sent  = a->sends_total;
    *total = a->n_commands * a->repeats_total;
    *label = auto_tcmd_state_label(a->state);
    return 1;
}

static int auto_field_is_text(auto_tcmd_field_t f) {
    return f == AUTO_F_POWER || f == AUTO_F_REPEATS || f == AUTO_F_DELAY;
}
static int auto_field_is_toggle(auto_tcmd_field_t f) {
    return f == AUTO_F_ALLOW_TX;
}

static char *auto_field_buf(auto_tcmd_t *a, auto_tcmd_field_t f, size_t *cap) {
    switch (f) {
        case AUTO_F_POWER:   *cap = sizeof a->power;   return a->power;
        case AUTO_F_REPEATS: *cap = sizeof a->repeats; return a->repeats;
        case AUTO_F_DELAY:   *cap = sizeof a->delay_s; return a->delay_s;
        default:             *cap = 0; return NULL;
    }
}

static int auto_field_char_ok(auto_tcmd_field_t f, int ch) {
    if (f == AUTO_F_POWER || f == AUTO_F_DELAY) {
        return (ch >= '0' && ch <= '9') || ch == '.' || ch == '-';
    }
    if (f == AUTO_F_REPEATS) {
        return (ch >= '0' && ch <= '9');
    }
    return 0;
}

static void auto_field_clamp_cursor(auto_tcmd_t *a, auto_tcmd_field_t f) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, f, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (a->cursors[f] < 0) a->cursors[f] = 0;
    if (a->cursors[f] > n) a->cursors[f] = n;
}

static void auto_field_insert(auto_tcmd_t *a, int ch) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (n + 1 >= (int) cap) return;
    if (!auto_field_char_ok(a->focus, ch)) return;
    int cur = a->cursors[a->focus];
    if (cur < 0) cur = 0;
    if (cur > n) cur = n;
    memmove(buf + cur + 1, buf + cur, (size_t)(n - cur + 1));
    buf[cur] = (char) ch;
    a->cursors[a->focus] = cur + 1;
}
static void auto_field_backspace(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = a->cursors[a->focus];
    if (cur <= 0 || n == 0) return;
    memmove(buf + cur - 1, buf + cur, (size_t)(n - cur + 1));
    a->cursors[a->focus] = cur - 1;
}
static void auto_field_delete(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = a->cursors[a->focus];
    if (cur >= n) return;
    memmove(buf + cur, buf + cur + 1, (size_t)(n - cur));
}
static void auto_field_kill_to_end(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = a->cursors[a->focus];
    if (cur >= n) return;
    buf[cur] = '\0';
}
static void auto_field_left(auto_tcmd_t *a) {
    size_t cap = 0;
    if (!auto_field_buf(a, a->focus, &cap)) return;
    if (a->cursors[a->focus] > 0) a->cursors[a->focus]--;
}
static void auto_field_right(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (a->cursors[a->focus] < n) a->cursors[a->focus]++;
}
static void auto_field_home(auto_tcmd_t *a) {
    size_t cap = 0;
    if (!auto_field_buf(a, a->focus, &cap)) return;
    a->cursors[a->focus] = 0;
}
static void auto_field_end(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    a->cursors[a->focus] = (int) strlen(buf);
}
static void auto_field_toggle(auto_tcmd_t *a) {
    if (a->focus == AUTO_F_ALLOW_TX) a->allow_tx = !a->allow_tx;
}

// Render helper — single inverse-cursor text-field cell, same shape as
// the TX compose renderer.
static void auto_draw_text_field(WINDOW *w, int row, int col,
                                 const char *label, const char *value,
                                 int value_w, int focused, int cursor)
{
    mvwprintw(w, row, col, "%s", label);
    int x = col + (int) strlen(label);
    int n = (int) strlen(value);
    int start = 0;
    if (focused && cursor > value_w - 1) start = cursor - value_w + 1;
    for (int i = 0; i < value_w; ++i) {
        int idx = start + i;
        char ch = (idx < n) ? value[idx] : ' ';
        chtype out = (chtype)(unsigned char) ch;
        if (focused && idx == cursor) {
            wattron(w, A_REVERSE);
            mvwaddch(w, row, x + i, out);
            wattroff(w, A_REVERSE);
        } else {
            mvwaddch(w, row, x + i, out);
        }
    }
    wclrtoeol(w);
}

static void auto_tcmd_draw(void) {
    if (!g_auto_tcmd_active || !g_auto_tcmd_win) return;
    WINDOW *w = g_auto_tcmd_win;
    auto_tcmd_t *a = &g_auto_tcmd;
    werase(w);
    draw_box(w);
    int width = getmaxx(w);

    mvwprintw(w, 0, 2, " Auto-TCMD (operator: %s)%s ",
              g_operator_user ? g_operator_user : "?",
              g_no_tx ? "  [--no-tx]" : "");

    mvwprintw(w, 1, 2, "File:    %.*s  (%d commands)",
              width - 28, a->file_path[0] ? a->file_path : "(none)",
              a->n_commands);
    wclrtoeol(w);

    int running_ro = (a->state == AUTO_STATE_RUNNING);

    auto_draw_text_field(w, 3, 2, "TX power ",
                         a->power, 8,
                         !running_ro && a->focus == AUTO_F_POWER,
                         a->cursors[AUTO_F_POWER]);
    mvwprintw(w, 3, 24, "dB  (B210 TX gain; 0..89.75)");
    wclrtoeol(w);

    auto_draw_text_field(w, 4, 2, "Repeats  ",
                         a->repeats, 6,
                         !running_ro && a->focus == AUTO_F_REPEATS,
                         a->cursors[AUTO_F_REPEATS]);
    mvwprintw(w, 4, 24, "per command (TCM1 N×, then TCM2 N×, ...)");
    wclrtoeol(w);

    auto_draw_text_field(w, 5, 2, "Delay    ",
                         a->delay_s, 8,
                         !running_ro && a->focus == AUTO_F_DELAY,
                         a->cursors[AUTO_F_DELAY]);
    mvwprintw(w, 5, 24, "s between every send");
    wclrtoeol(w);

    char tg[8];
    snprintf(tg, sizeof tg, "[%c]", a->allow_tx ? 'x' : ' ');
    mvwprintw(w, 7, 2, "%s", "");
    if (!running_ro && a->focus == AUTO_F_ALLOW_TX) wattron(w, A_REVERSE);
    mvwprintw(w, 7, 2, "%s", tg);
    if (!running_ro && a->focus == AUTO_F_ALLOW_TX) wattroff(w, A_REVERSE);
    mvwprintw(w, 7, 7, "allow-tx  (required to key the PA)");
    wclrtoeol(w);

    mvwprintw(w, 9, 2, "State:    %s", auto_tcmd_state_label(a->state));
    wclrtoeol(w);
    if (a->n_commands > 0) {
        int rt = a->repeats_total > 0 ? a->repeats_total : 0;
        char tx_spent[16], tx_total[16];
        fmt_minsec(a->tx_seconds_spent, tx_spent, sizeof tx_spent);
        fmt_minsec(a->tx_seconds_total, tx_total, sizeof tx_total);
        mvwprintw(w, 10, 2,
                  "Progress: cmd %d/%d   send %d/%d   total sent: %d   "
                  "(elapsed %s / ~%s)",
                  a->cmd_idx + (a->state == AUTO_STATE_RUNNING ? 1 : 0),
                  a->n_commands,
                  a->repeat_idx, rt,
                  a->sends_total,
                  tx_spent, tx_total);
    } else {
        mvwprintw(w, 10, 2, "Progress: (no commands loaded)");
    }
    wclrtoeol(w);
    mvwprintw(w, 11, 2, "Last sent: %.*s",
              width - 14, a->last_sent[0] ? a->last_sent : "-");
    wclrtoeol(w);
    mvwprintw(w, 12, 2, "Status:    %.*s",
              width - 14, a->status_msg[0] ? a->status_msg : "-");
    wclrtoeol(w);

    if (a->state == AUTO_STATE_RUNNING) {
        mvwprintw(w, 14, 2,
                  "Running — s stops   Esc closes (and stops)");
    } else {
        mvwprintw(w, 14, 2,
                  "Tab focus  Space toggle  Enter start  Esc cancel");
    }
    wclrtoeol(w);

    // Park the hardware cursor on the focused text field's cell. The
    // toggle field has no cursor; running mode is read-only so we
    // also skip cursor placement there.
    if (a->state != AUTO_STATE_RUNNING) {
        if (a->focus == AUTO_F_POWER) {
            int cur = a->cursors[AUTO_F_POWER];
            int vis = (cur > 7) ? 7 : cur;
            wmove(w, 3, 2 + 9 + vis);   // "TX power " is 9 chars
        } else if (a->focus == AUTO_F_REPEATS) {
            int cur = a->cursors[AUTO_F_REPEATS];
            int vis = (cur > 5) ? 5 : cur;
            wmove(w, 4, 2 + 9 + vis);   // "Repeats  " is 9 chars
        } else if (a->focus == AUTO_F_DELAY) {
            int cur = a->cursors[AUTO_F_DELAY];
            int vis = (cur > 7) ? 7 : cur;
            wmove(w, 5, 2 + 9 + vis);   // "Delay    " is 9 chars
        }
    }
    wrefresh(w);
}

// Open the modal. Refuses if the TX compose modal is already up — at
// most one modal owns the screen at a time. Lazily loads the file if
// --tc-file was passed and we haven't loaded yet.
static void auto_tcmd_open(void) {
    if (!g_ipc) return;
    if (g_tx_compose_active) return;
    if (g_auto_tcmd_active) return;
    if (g_auto_tcmd_file_path[0] == '\0') return;

    // (Re)load on open — file may have been edited since last open.
    if (g_auto_tcmd.commands) {
        auto_tcmd_free_commands(g_auto_tcmd.commands, g_auto_tcmd.n_commands);
        g_auto_tcmd.commands = NULL;
        g_auto_tcmd.n_commands = 0;
    }
    char **cmds = NULL;
    int    nc   = 0;
    if (auto_tcmd_load_file(g_auto_tcmd_file_path, &cmds, &nc) != 0) {
        return;  // silent — operator will notice via the absent modal
    }

    memset(&g_auto_tcmd, 0, sizeof g_auto_tcmd);
    g_auto_tcmd.commands   = cmds;
    g_auto_tcmd.n_commands = nc;
    snprintf(g_auto_tcmd.file_path, sizeof g_auto_tcmd.file_path,
             "%.*s", (int)(sizeof g_auto_tcmd.file_path - 1),
             g_auto_tcmd_file_path);
    snprintf(g_auto_tcmd.power,   sizeof g_auto_tcmd.power,   "80.0");
    snprintf(g_auto_tcmd.repeats, sizeof g_auto_tcmd.repeats, "3");
    snprintf(g_auto_tcmd.delay_s, sizeof g_auto_tcmd.delay_s, "2.0");
    g_auto_tcmd.allow_tx = 0;
    g_auto_tcmd.focus    = AUTO_F_POWER;
    g_auto_tcmd.cursors[AUTO_F_POWER]   = (int) strlen(g_auto_tcmd.power);
    g_auto_tcmd.cursors[AUTO_F_REPEATS] = (int) strlen(g_auto_tcmd.repeats);
    g_auto_tcmd.cursors[AUTO_F_DELAY]   = (int) strlen(g_auto_tcmd.delay_s);
    g_auto_tcmd.state    = AUTO_STATE_SETUP;
    snprintf(g_auto_tcmd.status_msg, sizeof g_auto_tcmd.status_msg,
             "loaded %d command(s). Set fields, then Enter to start.",
             nc);

    int h = 17, ww = 110;
    if (h > LINES) h = LINES;
    if (ww > COLS) ww = COLS;
    if (ww < 60)  ww = (COLS < 60) ? COLS : 60;
    g_auto_tcmd_win = newwin(h, ww, (LINES - h) / 2, (COLS - ww) / 2);
    if (!g_auto_tcmd_win) {
        auto_tcmd_free_commands(g_auto_tcmd.commands, g_auto_tcmd.n_commands);
        g_auto_tcmd.commands = NULL;
        g_auto_tcmd.n_commands = 0;
        return;
    }
    keypad(g_auto_tcmd_win, TRUE);
    nodelay(g_auto_tcmd_win, TRUE);
    g_auto_tcmd_active = 1;
    auto_tcmd_draw();
}

static void auto_tcmd_close(void) {
    if (g_auto_tcmd_win) {
        delwin(g_auto_tcmd_win);
        g_auto_tcmd_win = NULL;
    }
    g_auto_tcmd_active = 0;
    if (g_auto_tcmd.commands) {
        auto_tcmd_free_commands(g_auto_tcmd.commands, g_auto_tcmd.n_commands);
        g_auto_tcmd.commands = NULL;
        g_auto_tcmd.n_commands = 0;
    }
    touchwin(stdscr);
    refresh();
}

// Validate the setup fields and move to RUNNING. Returns 0 on success,
// fills status_msg + returns -1 on failure.
static int auto_tcmd_start(void) {
    auto_tcmd_t *a = &g_auto_tcmd;
    if (a->n_commands == 0) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: file has no commands");
        return -1;
    }
    if (!a->allow_tx) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: allow-tx is off");
        return -1;
    }
    double power = atof(a->power);
    if (power < 0.0 || power > 89.75) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: TX power %.1f dB out of B210 range 0..89.75",
                 power);
        return -1;
    }
    int repeats = atoi(a->repeats);
    if (repeats < 1) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: repeats must be >= 1");
        return -1;
    }
    double delay = atof(a->delay_s);
    if (delay < 0.0) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: delay must be >= 0");
        return -1;
    }
    a->repeats_total = repeats;
    a->delay_s_val   = delay;
    a->cmd_idx       = 0;
    a->repeat_idx    = 0;
    a->sends_total   = 0;
    a->tx_seconds_spent = 0.0;
    // Wall-clock estimate for the whole run. auto_tcmd_tick spaces sends
    // by max(delay, burst): it waits `delay` measured from the start of
    // each send AND for that send's burst to clear, so the delay and the
    // burst overlap rather than add. Every command is sent `repeats`
    // times; only the final send has no trailing delay.
    a->tx_seconds_total = 0.0;
    double last_burst = 0.0;
    for (int i = 0; i < a->n_commands; ++i) {
        double burst = auto_tcmd_burst_seconds(strlen(a->commands[i]));
        double slot  = (burst > delay) ? burst : delay;
        a->tx_seconds_total += slot * (double) repeats;
        last_burst = burst;
    }
    if (a->n_commands > 0 && delay > last_burst)
        a->tx_seconds_total -= (delay - last_burst);
    a->start_ns      = ts_now_ns();
    a->next_send_ns  = a->start_ns;  // first send fires immediately
    a->state         = AUTO_STATE_RUNNING;
    snprintf(a->status_msg, sizeof a->status_msg,
             "running: %d cmds × %d repeats, %.2f s delay",
             a->n_commands, repeats, delay);
    {
        char det[256];
        snprintf(det, sizeof det,
                 "n_commands=%d repeats=%d delay_s=%.2f "
                 "allow_tx=%d power=%.100s file=\"%.100s\"",
                 a->n_commands, repeats, delay, a->allow_tx,
                 a->power, a->file_path);
        sso_audit_event("auto-tcmd-start", det);
    }
    return 0;
}

// Pause / cancel without closing the modal so the operator can see the
// final progress numbers.
static void auto_tcmd_stop(const char *reason) {
    auto_tcmd_t *a = &g_auto_tcmd;
    if (a->state != AUTO_STATE_RUNNING) return;
    a->state = AUTO_STATE_STOPPED;
    snprintf(a->status_msg, sizeof a->status_msg, "stopped: %s",
             reason ? reason : "user");
    {
        char det[128];
        snprintf(det, sizeof det,
                 "reason=\"%.100s\" sends_total=%d",
                 reason ? reason : "user",
                 a->sends_total);
        sso_audit_event("auto-tcmd-stop", det);
    }
}

static int auto_tcmd_handle_key(int key) {
    if (!g_auto_tcmd_active) return 0;
    if (key == ERR) return 1;
    auto_tcmd_t *a = &g_auto_tcmd;
    int changed = 1;
    // Esc-as-CSI same fallback the TX modal uses.
    if (key == 27) {
        int translated = tx_drain_csi(g_auto_tcmd_win);
        if (translated >= 0) {
            key = translated;
        } else {
            return 0;  // Esc closes (and stops via close path below)
        }
    }
    if (a->state == AUTO_STATE_RUNNING) {
        // Run mode: only stop / close commands are honoured. Field
        // edits are blocked so an operator can't change power mid-run.
        if (key == 's' || key == 'S') {
            auto_tcmd_stop("user");
            auto_tcmd_draw();
            return 1;
        }
        return 1;
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        if (auto_tcmd_start() == 0) {
            auto_tcmd_draw();
        } else {
            auto_tcmd_draw();
        }
        return 1;
    } else if (key == '\t') {
        a->focus = (auto_tcmd_field_t) ((a->focus + 1) % AUTO_F_COUNT);
        auto_field_clamp_cursor(a, a->focus);
    } else if (key == KEY_BTAB) {
        a->focus = (auto_tcmd_field_t) ((a->focus + AUTO_F_COUNT - 1)
                                         % AUTO_F_COUNT);
        auto_field_clamp_cursor(a, a->focus);
    } else if (key == KEY_BACKSPACE || key == 127 || key == 8) {
        auto_field_backspace(a);
    } else if (key == KEY_DC || key == 4) {
        auto_field_delete(a);
    } else if (key == 11) {
        auto_field_kill_to_end(a);
    } else if (key == KEY_LEFT) {
        auto_field_left(a);
    } else if (key == KEY_RIGHT) {
        auto_field_right(a);
    } else if (key == KEY_HOME || key == 1) {
        auto_field_home(a);
    } else if (key == KEY_END || key == 5) {
        auto_field_end(a);
    } else if (key == ' ' && auto_field_is_toggle(a->focus)) {
        auto_field_toggle(a);
    } else if (key >= 32 && key < 127) {
        auto_field_insert(a, key);
    } else {
        changed = 0;
    }
    if (changed) auto_tcmd_draw();
    return 1;
}

// Per-tick burst driver. When running, queues one g_tx_request when
// (a) the previous burst has cleared, and (b) the inter-send delay
// has elapsed. Stops automatically on LOS so an unattended run won't
// keep TXing after the pass. emit_tx_event_local fires from the main
// loop's burst-handler the same way it does for the manual TX
// compose path, so tx.log + viewer fanout capture every shot.
static void auto_tcmd_tick(state_t *state) {
    if (!g_auto_tcmd_active) return;
    auto_tcmd_t *a = &g_auto_tcmd;
    if (a->state != AUTO_STATE_RUNNING) return;

    // Elapsed wall-clock since the run started, capped at the estimate,
    // so the Progress line reads elapsed/total (inter-send delays and the
    // burst start lead included) rather than on-air seconds only. Frozen
    // automatically once the state leaves RUNNING -- this returns early then.
    {
        double elapsed = (double) (ts_now_ns() - a->start_ns) * 1e-9;
        if (elapsed < 0.0) elapsed = 0.0;
        if (elapsed > a->tx_seconds_total) elapsed = a->tx_seconds_total;
        a->tx_seconds_spent = elapsed;
    }

    // LOS guard. We consider the pass over once the elevation has
    // gone negative AND the predictor has rolled the next pass into
    // the future. Sitting on a freshly-loaded prediction during AOS
    // ambiguity (elevation < 0 but next pass not yet predicted) is
    // not enough to abort; the running flag stays so a momentary
    // numerical wobble can't kill an active session.
    double el = state->prediction.satellite_ephem.elevation;
    if (el < 0.0
        && state->prediction.predicted_minutes_until_visible > 0.5) {
        a->state = AUTO_STATE_PASS_OVER;
        snprintf(a->status_msg, sizeof a->status_msg,
                 "stopped: pass over (elevation %.1f deg)", el);
        auto_tcmd_draw();
        return;
    }

    if (a->cmd_idx >= a->n_commands) {
        a->state = AUTO_STATE_DONE;
        snprintf(a->status_msg, sizeof a->status_msg,
                 "done: sent all %d command(s)", a->n_commands);
        auto_tcmd_draw();
        return;
    }

#ifdef SSO_WITH_SDR
    long now = ts_now_ns();
    if (now < a->next_send_ns) return;
    if (g_tx_request.pending)  return;  // prior burst still inflight

    const char *raw = a->commands[a->cmd_idx];
    // Expand a simple_sat_ops-directed "SSO+..." line into the concrete
    // telecommand, clock captured now so each send (and each repeat) carries a
    // fresh time. A normal line passes through verbatim. Startup lint already
    // vetted SSO+ lines, so a failure here is defensive: skip the whole command
    // rather than key a half-built payload.
    sso_pseudo_ctx_t pc = { .now_ms    = sso_now_utc_ms(),
                            .tssent_ms = g_sso_pass_tssent_ms };
    char wire[512];
    char sso_err[160];
    sso_pseudo_status_t pst =
        sso_pseudo_expand(raw, &pc, wire, sizeof wire, sso_err, sizeof sso_err);
    if (pst != SSO_PSEUDO_OK && pst != SSO_PSEUDO_NOT_PSEUDO) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "skipped SSO+ line %d: %.120s", a->cmd_idx + 1, sso_err);
        a->cmd_idx++;
        a->repeat_idx   = 0;
        a->next_send_ns = now + (long)(a->delay_s_val * 1e9);
        auto_tcmd_draw();
        return;
    }
    size_t n = strlen(wire);
    if (n > sizeof g_tx_request.payload) n = sizeof g_tx_request.payload;
    memcpy(g_tx_request.payload, wire, n);
    g_tx_request.payload_len  = n;
    g_tx_request.is_hex       = 0;
    g_tx_request.csp_hdr      = (csp_v1_header_t){
        .prio  = 2, .src = 10, .dst = 1, .dport = 7, .sport = 16, .flags = 0,
    };
    g_tx_request.tx_freq_hz       = g_tx_freq_hz_doppler;
    g_tx_request.tx_gain_db       = atof(a->power);
    g_tx_request.repeat           = 1;
    g_tx_request.gap_ms           = 200;
    g_tx_request.preroll_ms       = g_tx_preroll_ms;
    // No g_tx_request.allow_tx field — the TX-inhibit gate is enforced
    // at auto_tcmd_start time (refuses to enter RUNNING unless allow_tx
    // is ticked), same way tx_compose_validate handles it before commit.
    g_tx_request.allow_high_power = 0;
    g_tx_request.allow_hf_tx      = 0;
    if (pst == SSO_PSEUDO_OK)
        snprintf(g_tx_request.sso_origin, sizeof g_tx_request.sso_origin, "%s", raw);
    else
        g_tx_request.sso_origin[0] = '\0';
    {
        int m = snprintf(g_tx_request.summary, sizeof g_tx_request.summary,
                         "auto[%d/%d %d/%d]: %.190s",
                         a->cmd_idx + 1, a->n_commands,
                         a->repeat_idx + 1, a->repeats_total,
                         wire);
        if (pst == SSO_PSEUDO_OK && m > 0 && (size_t) m < sizeof g_tx_request.summary)
            snprintf(g_tx_request.summary + m, sizeof g_tx_request.summary - m,
                     " (replaced '%s')", raw);
    }
    g_tx_request.pending = 1;
    snprintf(a->last_sent, sizeof a->last_sent, "%.255s", wire);
    a->sends_total++;

    a->repeat_idx++;
    if (a->repeat_idx >= a->repeats_total) {
        a->cmd_idx++;
        a->repeat_idx = 0;
    }

    a->next_send_ns = now + (long)(a->delay_s_val * 1e9);
    auto_tcmd_draw();
#else
    (void) state;
    a->state = AUTO_STATE_STOPPED;
    snprintf(a->status_msg, sizeof a->status_msg,
             "stopped: this build has no SDR support");
    auto_tcmd_draw();
#endif
}

// --- Forward decls -------------------------------------------------

void start_tracking(state_t *state);
void stop_tracking(state_t *state);
int  point_to_stationary_target(state_t *state, double azimuth, double elevation);
static void scan_sky_start(state_t *state);
static void scan_sky_stop(state_t *state, const char *reason);
static void scan_sky_tick(state_t *state, double t_now);
void update_doppler_shifted_frequencies(state_t *state, double uplink_freq, double downlink_freq);
static int apply_args(state_t *state, int argc, char **argv, double jul_utc, int help);

// --- TLE auto-discovery helpers (used by --control with no
//     positional satellite name) ------------------------------------

// Recursively scan `dir` for *.tle files, return the path with the
// newest mtime via out_path. Returns 0 on success, -1 if dir is
// unreadable or no .tle file exists in the tree. Caller-allocated
// buffer must be at least PATH_MAX-ish.
static int find_newest_tle_recursive(const char *dir,
                                     char *out_path, size_t out_cap,
                                     time_t *out_mtime)
{
    DIR *d = opendir(dir);
    if (d == NULL) return -1;
    int found = 0;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char child[1024];
        int n = snprintf(child, sizeof child, "%s/%s", dir, de->d_name);
        if (n < 0 || (size_t)n >= sizeof child) continue;
        struct stat st;
        if (stat(child, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            char nested[1024];
            time_t nested_mtime = 0;
            if (find_newest_tle_recursive(child, nested, sizeof nested,
                                          &nested_mtime) == 0) {
                if (!found || nested_mtime > *out_mtime) {
                    snprintf(out_path, out_cap, "%s", nested);
                    *out_mtime = nested_mtime;
                    found = 1;
                }
            }
        } else if (S_ISREG(st.st_mode)) {
            size_t nlen = strlen(de->d_name);
            if (nlen < 4) continue;
            if (strcmp(de->d_name + nlen - 4, ".tle") != 0) continue;
            if (!found || st.st_mtime > *out_mtime) {
                snprintf(out_path, out_cap, "%s", child);
                *out_mtime = st.st_mtime;
                found = 1;
            }
        }
    }
    closedir(d);
    return found ? 0 : -1;
}

// Pull the satellite name out of a 3-line TLE — the first non-blank
// line that doesn't begin with "1 " or "2 ". Trims trailing
// whitespace. Returns 0 on success, -1 if the file is unreadable or
// has no name line.
static int read_tle_name(const char *tle_path,
                         char *out_name, size_t out_cap)
{
    FILE *f = fopen(tle_path, "r");
    if (f == NULL) return -1;
    char line[256];
    int rc = -1;
    while (fgets(line, sizeof line, f) != NULL) {
        size_t n = strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'
                      || line[n - 1] == ' '  || line[n - 1] == '\t')) {
            line[--n] = '\0';
        }
        if (n == 0) continue;
        if ((line[0] == '1' || line[0] == '2') && line[1] == ' ') continue;
        snprintf(out_name, out_cap, "%s", line);
        rc = 0;
        break;
    }
    fclose(f);
    return rc;
}

// mkdir -p for the parent of `path`. Used before copy_file to make
// sure ~/.local/state/simple_sat_ops/ exists.
static int mkdir_p_for_file(const char *path)
{
    char buf[1024];
    snprintf(buf, sizeof buf, "%s", path);
    for (char *p = buf + 1; *p; p++) {
        if (*p != '/') continue;
        *p = '\0';
        if (mkdir(buf, 0755) != 0 && errno != EEXIST) {
            *p = '/';
            return -1;
        }
        *p = '/';
    }
    return 0;
}

// Plain byte-copy. Returns 0 on success, -1 on any I/O error.
static int copy_file(const char *src, const char *dst)
{
    FILE *in = fopen(src, "rb");
    if (in == NULL) return -1;
    if (mkdir_p_for_file(dst) != 0) { fclose(in); return -1; }
    FILE *out = fopen(dst, "wb");
    if (out == NULL) { fclose(in); return -1; }
    char buf[4096];
    size_t n;
    int rc = 0;
    while ((n = fread(buf, 1, sizeof buf, in)) > 0) {
        if (fwrite(buf, 1, n, out) != n) { rc = -1; break; }
    }
    if (ferror(in)) rc = -1;
    fclose(in);
    if (fclose(out) != 0) rc = -1;
    return rc;
}

// --- Pass folder for the upcoming pass -----------------------------

// Julian Date -> Unix epoch seconds. Reference: JD 2440587.5 is
// 1970-01-01 00:00:00 UTC, which is what time_t counts from.
static time_t jul_to_unix(double jd)
{
    return (time_t)((jd - 2440587.5) * 86400.0 + 0.5);
}

// Refresh /FrontierSat/Operations/current so it points at `target`.
// Atomic-ish: unlink the old link, symlink the new one. If the
// symlink call fails we log and carry on — the pass folder itself is
// still created and broadcast over IPC, the symlink is just a
// convenience.
static void update_operations_current_symlink(const char *target)
{
    const char *link = sso_operations_current_symlink();
    if (link == NULL || link[0] == '\0') return;
    // Make sure /FrontierSat/Operations/ exists for the symlink slot.
    sso_mkdir_p_for_file(link);
    unlink(link);
    if (symlink(target, link) != 0) {
        fprintf(stderr,
                "simple_sat_ops: symlink %s -> %s failed: %s "
                "(non-fatal; pass folder still set)\n",
                link, target, strerror(errno));
    }
}

// Compute a fresh AOS prediction off `state`'s current position and
// Scan a YYYYMMDD parent dir for an existing HHMMLT folder whose
// HHMM is within `window_minutes` of (target_hh, target_mm).
// Restarting simple_sat_ops near a pass time shifts the predicted
// AOS by a minute or two between launches, which would otherwise
// spawn a fresh 1115LT/ alongside the 1114LT/ the operator was
// already writing to. With this lookup the second start reuses
// the existing folder. Returns 0 + fills out_path on a hit; -1 on
// miss (or any I/O failure — caller should fall back to creating
// a fresh folder).
static int find_nearby_pass_folder(const char *parent_dir,
                                   int target_hh, int target_mm,
                                   int window_minutes,
                                   char *out_path, size_t out_path_cap)
{
    DIR *d = opendir(parent_dir);
    if (d == NULL) return -1;
    int best_diff = INT_MAX;
    char best_name[16] = "";
    int target_mod = target_hh * 60 + target_mm;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        // HHMMLT name pattern: exactly 6 chars, [0-9]{4} then "LT".
        const char *n = de->d_name;
        if (strlen(n) != 6) continue;
        if (!isdigit((unsigned char) n[0]) || !isdigit((unsigned char) n[1])
         || !isdigit((unsigned char) n[2]) || !isdigit((unsigned char) n[3])
         || n[4] != 'L' || n[5] != 'T') continue;
        int hh = (n[0] - '0') * 10 + (n[1] - '0');
        int mm = (n[2] - '0') * 10 + (n[3] - '0');
        if (hh >= 24 || mm >= 60) continue;
        int mod = hh * 60 + mm;
        int diff = abs(mod - target_mod);
        if (diff > 12 * 60) diff = 24 * 60 - diff;   // wrap across midnight
        if (diff <= window_minutes && diff < best_diff) {
            best_diff = diff;
            // Filter above guarantees strlen(n) == 6, but gcc can't
            // prove that — pin the width to keep -Wformat-truncation
            // satisfied with our 16-byte best_name.
            snprintf(best_name, sizeof best_name, "%.6s", n);
        }
    }
    closedir(d);
    if (best_name[0] == '\0') return -1;
    int n = snprintf(out_path, out_path_cap, "%s/%s", parent_dir, best_name);
    return (n > 0 && (size_t) n < out_path_cap) ? 0 : -1;
}

// build /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/. Stashes the
// result in g_pass_folder so ipc_broadcast_state can publish it on
// every tick.
static void setup_pass_folder(state_t *state, double jul_utc_now)
{
    // Handoff case: --pass-folder seeded g_pass_folder before we got
    // here. Honour it — make sure the dir exists, refresh the
    // "current" symlink, and skip AOS-discovery entirely.
    if (g_pass_folder[0]) {
        if (sso_mkdir_p(g_pass_folder) != 0) {
            fprintf(stderr,
                "simple_sat_ops: mkdir -p %s failed: %s\n",
                g_pass_folder, strerror(errno));
        }
        update_operations_current_symlink(g_pass_folder);
        fprintf(stderr, "simple_sat_ops: using inherited pass folder %s\n",
                g_pass_folder);
        {
            char det[600];
            snprintf(det, sizeof det,
                     "mode=inherited path=\"%.500s\"", g_pass_folder);
            sso_audit_event("pass-folder", det);
        }
        return;
    }
    // --testing: bench run, not a pass. Land the folder under the
    // sibling Testing/ tree using the CURRENT local time so we don't
    // need a TLE / prediction at all.
    if (g_testing_mode) {
        time_t now = time(NULL);
        struct tm now_local;
        localtime_r(&now, &now_local);
        char folder[256];
        int n = snprintf(folder, sizeof folder,
                         "%s/%04d%02d%02d/%02d%02dLT",
                         sso_testing_dir(),
                         now_local.tm_year + 1900,
                         now_local.tm_mon + 1,
                         now_local.tm_mday,
                         now_local.tm_hour,
                         now_local.tm_min);
        if (n <= 0 || (size_t) n >= sizeof folder) {
            fprintf(stderr,
                "simple_sat_ops: --testing folder path too long; skipping\n");
            return;
        }
        if (sso_mkdir_p(folder) != 0) {
            fprintf(stderr,
                "simple_sat_ops: --testing: mkdir -p %s failed: %s\n",
                folder, strerror(errno));
            return;
        }
        snprintf(g_pass_folder, sizeof g_pass_folder, "%s", folder);
        // Skip update_operations_current_symlink — keep the
        // Operations/current pointer aimed at real passes, not bench
        // runs (avoids confusing operators who scrub recent activity
        // by looking at the symlink).
        fprintf(stderr,
            "simple_sat_ops: --testing folder %s\n", g_pass_folder);
        {
            char det[600];
            snprintf(det, sizeof det,
                     "mode=testing path=\"%.500s\"", g_pass_folder);
            sso_audit_event("pass-folder", det);
        }
        return;
    }
    minutes_until_visible(&state->prediction, jul_utc_now,
                          jul_utc_now + MAX_MINUTES_TO_PREDICT / 1440.0,
                          1.0);
    // minutes_until_visible sets predicted_minutes_until_visible and
    // uses -9999.0 as the "no AOS in this window" sentinel. Positive
    // values are minutes until AOS; negatives are minutes since AOS
    // when we started mid-pass — either way (now + N) lands on the
    // current pass's AOS, which is what we want for the folder name.
    double minutes = state->prediction.predicted_minutes_until_visible;
    if (minutes <= -9000.0) {
        fprintf(stderr,
                "simple_sat_ops: no AOS in the next %d minutes — "
                "pass folder not created\n",
                MAX_MINUTES_TO_PREDICT);
        return;
    }
    double aos_jul = jul_utc_now + minutes / 1440.0;
    time_t aos = jul_to_unix(aos_jul);
    struct tm aos_local;
    localtime_r(&aos, &aos_local);
    char parent_dir[256];
    int pn = snprintf(parent_dir, sizeof parent_dir,
                      "%s/%04d%02d%02d",
                      sso_operations_dir(),
                      aos_local.tm_year + 1900,
                      aos_local.tm_mon + 1,
                      aos_local.tm_mday);
    if (pn <= 0 || (size_t) pn >= sizeof parent_dir) {
        fprintf(stderr,
                "simple_sat_ops: pass folder parent path too long; skipping\n");
        return;
    }
    char folder[256];
    // Look for an existing HHMMLT folder for THIS pass within ±10
    // minutes of the predicted AOS — re-runs of simple_sat_ops near
    // a pass time can drift the prediction by a minute or two, and
    // we want to keep recording into the same folder.
    if (find_nearby_pass_folder(parent_dir,
                                aos_local.tm_hour, aos_local.tm_min,
                                10, folder, sizeof folder) == 0) {
        fprintf(stderr,
                "simple_sat_ops: reusing pass folder %s "
                "(predicted AOS %02d:%02dLT within 10 min)\n",
                folder, aos_local.tm_hour, aos_local.tm_min);
    } else {
        int n = snprintf(folder, sizeof folder,
                         "%s/%02d%02dLT",
                         parent_dir,
                         aos_local.tm_hour, aos_local.tm_min);
        if (n <= 0 || (size_t) n >= sizeof folder) {
            fprintf(stderr,
                "simple_sat_ops: pass folder name too long; skipping\n");
            return;
        }
    }
    if (sso_mkdir_p(folder) != 0) {
        fprintf(stderr,
                "simple_sat_ops: mkdir -p %s failed: %s\n",
                folder, strerror(errno));
        return;
    }

    // Pin the TLE the operator just loaded into the pass folder so
    // any post-pass analysis (rx_replay, packet_browser session-dir
    // lookups, whoever shows up later) can find the exact ephemeris
    // that was being tracked, even if active.tle gets rewritten on
    // the next --control startup.
    if (state->prediction.tles_filename
        && state->prediction.tles_filename[0]) {
        const char *src = state->prediction.tles_filename;
        const char *base = strrchr(src, '/');
        base = (base != NULL) ? base + 1 : src;
        char dst[512];
        int rc = snprintf(dst, sizeof dst, "%s/%s", folder, base);
        if (rc > 0 && (size_t)rc < sizeof dst) {
            if (copy_file(src, dst) != 0) {
                fprintf(stderr,
                        "simple_sat_ops: warning: TLE copy %s -> %s "
                        "failed: %s\n",
                        src, dst, strerror(errno));
            } else {
                fprintf(stderr, "simple_sat_ops: pinned TLE %s\n", dst);
            }
        }
    }

    snprintf(g_pass_folder, sizeof g_pass_folder, "%s", folder);
    update_operations_current_symlink(folder);
    fprintf(stderr, "simple_sat_ops: pass folder %s\n", folder);
    {
        char det[600];
        snprintf(det, sizeof det,
                 "mode=aos path=\"%.500s\"", g_pass_folder);
        sso_audit_event("pass-folder", det);
    }
}

// Sample the upcoming pass on a local prediction_t copy and render a
// polar az/el plot to pass_folder/az_el_plot.png via gnuplot. Two
// traces: the satellite's sky position and the rotator boom's beam
// direction (which match for non-flip passes and diverge near apex on
// flip passes -- a visual sanity check of the flip mapping). The
// raw TSV and the gnuplot script are left in the pass folder so the
// operator can rerun or tweak the plot offline.
static void generate_pass_plot(state_t *state, const char *pass_folder,
                               double jul_utc_now)
{
    if (!pass_folder || !pass_folder[0]) {
        return;
    }

    // Work on a local copy: update_pass_predictions / update_satellite_position
    // both mutate the prediction's satellite_ephem and aggregate fields.
    prediction_t pred = state->prediction;

    // Defensive: handoff case (setup_pass_folder used inherited
    // g_pass_folder) leaves predicted_minutes_until_visible stale.
    // Re-run the search so aos_jul below is well defined.
    minutes_until_visible(&pred, jul_utc_now,
                          jul_utc_now + 180.0 / 1440.0, 1.0);
    if (pred.predicted_minutes_until_visible <= -9000.0) {
        fprintf(stderr,
                "simple_sat_ops: no AOS in the next 3 h — skipping plot\n");
        return;
    }
    double aos_jul = jul_utc_now + pred.predicted_minutes_until_visible / 1440.0;

    // Populate predicted_max_elevation / predicted_ascension_azimuth /
    // predicted_pass_duration_minutes on the local copy.
    update_pass_predictions(&pred, aos_jul, 0.1);

    int flip_mode = 0;
    double aos_az = pred.predicted_ascension_azimuth;
    double los_az = pred.predicted_descent_azimuth;
    double aos_jul_pred = pred.predicted_ascension_jul_utc;
    double los_jul_pred = pred.predicted_descent_jul_utc;
    double pass_jul = los_jul_pred - aos_jul_pred;
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION > 90
        && pred.predicted_max_elevation
               >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD) {
        flip_mode = 1;
    }

    char dat_path[512];
    int n = snprintf(dat_path, sizeof dat_path, "%s/pass_plot.dat",
                     pass_folder);
    if (n <= 0 || (size_t)n >= sizeof dat_path) return;
    FILE *f = fopen(dat_path, "w");
    if (!f) {
        fprintf(stderr, "simple_sat_ops: fopen %s: %s\n",
                dat_path, strerror(errno));
        return;
    }
    fprintf(f, "# t_min\tsat_az\tsat_el\tbeam_az\tbeam_el\n");

    // 10 s cadence over the visible portion. predicted_pass_duration_minutes
    // includes the -5..0 deg pre-AOS / post-LOS buffer, so step a little
    // wider and let the el-filter below drop the wings.
    const double step_min = 10.0 / 60.0;
    const double duration_min = pred.predicted_pass_duration_minutes > 0
        ? pred.predicted_pass_duration_minutes
        : 15.0;
    const int n_steps = (int)(duration_min / step_min) + 1;

    int wrote = 0;
    for (int i = 0; i <= n_steps; ++i) {
        double t_min = i * step_min;
        double sample_jul = aos_jul + t_min / 1440.0;
        update_satellite_position(&pred, sample_jul);
        double sat_az = pred.satellite_ephem.azimuth;
        double sat_el = pred.satellite_ephem.elevation;
        if (sat_el < 0.0) continue;

        double mech_az = sat_az;
        double mech_el = sat_el;
        int half;
        double progress = 0.0;
        if (pass_jul > 0.0) {
            progress = (sample_jul - aos_jul_pred) / pass_jul;
        }
        antenna_rotator_to_mech_coords(flip_mode, aos_az, los_az,
                                       progress,
                                       sat_az, sat_el,
                                       &mech_az, &mech_el, &half);

        // Convert mech back to where the boom's beam actually points
        // on the sky. mech_el > 90 deg means back-pointing through the
        // rotator: equivalent sky direction is (mech_az + 180, 180 -
        // mech_el).
        double beam_az = mech_az;
        double beam_el = mech_el;
        if (beam_el > 90.0) {
            beam_az = fmod(beam_az + 180.0, 360.0);
            if (beam_az < 0.0) beam_az += 360.0;
            beam_el = 180.0 - beam_el;
        }

        fprintf(f, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
                t_min, sat_az, sat_el, beam_az, beam_el);
        ++wrote;
    }
    fclose(f);

    if (wrote == 0) {
        fprintf(stderr,
                "simple_sat_ops: no visible samples in predicted pass — "
                "skipping plot\n");
        return;
    }

    char gp_path[512];
    n = snprintf(gp_path, sizeof gp_path, "%s/pass_plot.gp", pass_folder);
    if (n <= 0 || (size_t)n >= sizeof gp_path) return;
    FILE *gp = fopen(gp_path, "w");
    if (!gp) {
        fprintf(stderr, "simple_sat_ops: fopen %s: %s\n",
                gp_path, strerror(errno));
        return;
    }

    const char *sat_name =
        (state->prediction.satellite_ephem.name
         && state->prediction.satellite_ephem.name[0])
            ? state->prediction.satellite_ephem.name : "satellite";

    // Mirror scripts/plot_sky_pass.sh's polar style so both plots read
    // the same way (N up, E clockwise, zenith at centre). Solid darker
    // grid + elevation-valued rtics so the reticules are readable.
    fprintf(gp,
        "set terminal pngcairo size 900,900 enhanced font 'Helvetica,11'\n"
        "set output '%s/az_el_plot.png'\n"
        "set polar\n"
        "set angles degrees\n"
        "set theta top clockwise\n"
        "set size square\n"
        // Explicit margins so the W cardinal label isn't clipped on the
        // left and the legend has room on the right.
        "set lmargin 8\n"
        "set rmargin 14\n"
        "set tmargin 4\n"
        "set bmargin 4\n"
        "set grid polar 30 lt 1 lw 0.6 lc rgb '#666666'\n"
        "unset border\n"
        "unset xtics\n"
        "unset ytics\n"
        "set rrange [0:90]\n"
        // Label the rings with the elevation they correspond to (r = 90
        // - el): inner ring = 60 deg el, outer ring = 0 deg el (horizon);
        // centre is zenith (90 deg el) and the title says so.
        "set rtics ('60' 30, '30' 60, '0' 90) "
              "font 'Helvetica,9' textcolor rgb '#333333'\n"
        "set ttics ('N' 0, 'E' 90, 'S' 180, 'W' 270) "
              "font 'Helvetica,11,Bold'\n"
        "set key outside right top\n"
        "set title \"%s  max_el=%.1f deg  flip=%s\\n"
                  "(zenith = centre; ring labels = elevation deg)\" noenhanced\n"
        "plot \\\n"
        "  '%s/pass_plot.dat' using 2:(90-$3) "
            "with lines lw 2 lc rgb '#1f77b4' title 'satellite', \\\n"
        "  '' using 4:(90-$5) "
            "with lines lw 2 lc rgb '#d62728' title 'antenna beam'\n",
        pass_folder,
        sat_name,
        pred.predicted_max_elevation,
        flip_mode ? "yes" : "no",
        pass_folder);
    fclose(gp);

    char cmd[1100];
    n = snprintf(cmd, sizeof cmd, "gnuplot '%s' 2>&1", gp_path);
    if (n <= 0 || (size_t)n >= sizeof cmd) return;
    int rc = system(cmd);
    if (rc != 0) {
        fprintf(stderr,
                "simple_sat_ops: gnuplot failed (rc=%d). Install gnuplot "
                "or run manually:  gnuplot '%s'\n",
                rc, gp_path);
    } else {
        fprintf(stderr,
                "simple_sat_ops: pass plot %s/az_el_plot.png "
                "(%d samples, %s)\n",
                pass_folder, wrote,
                flip_mode ? "flip" : "no flip");
    }
}


// --- ncurses ------------------------------------------------------

// While the ncurses TUI owns the screen, any stray write to stderr (a
// UHD/libusb error, a backend diagnostic, a library warning) lands on
// top of the panels and corrupts the display until the next full
// redraw. We therefore point fd 2 at a log file for the lifetime of
// the TUI: nothing reaches the terminal, but every message is still
// captured for post-pass debugging (the LIBUSB_TRANSFER_OVERFLOW flood
// during TX, for one). Restored on teardown so final console messages
// print normally. dup2 on the fd (not the FILE*) catches direct fd-2
// writes from C libraries too, not just our fprintf(stderr, ...).
static int   g_saved_stderr_fd = -1;
// Path of the redirected log and its size at grab time, so on quit we
// can tell the operator whether anything was logged THIS run (the file
// is opened append, so we compare against the starting size, not zero).
// Empty path => no real log file (e.g. a viewer with no pass folder).
static char  g_stderr_log_path[320] = "";
static off_t g_stderr_log_start_size = 0;

static void tui_grab_stderr(void)
{
    if (g_saved_stderr_fd != -1) return;   // already redirected

    char path[320];
    if (g_pass_folder[0]) {
        snprintf(path, sizeof path, "%.300s/sso_stderr.log", g_pass_folder);
    } else {
        // No pass folder (e.g. a viewer): we still must not corrupt the
        // screen, so swallow stderr rather than leave it on the tty.
        snprintf(path, sizeof path, "/dev/null");
    }

    int log_fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (log_fd < 0) return;                // leave stderr as-is on failure

    // Remember the log path + its current size so tui_report_errors can
    // tell whether this run appended anything. Only for a real file; a
    // /dev/null sink leaves the path empty and is never reported on.
    g_stderr_log_path[0]    = '\0';
    g_stderr_log_start_size = 0;
    if (g_pass_folder[0]) {
        struct stat st;
        if (fstat(log_fd, &st) == 0) g_stderr_log_start_size = st.st_size;
        snprintf(g_stderr_log_path, sizeof g_stderr_log_path, "%s", path);
    }

    fflush(stderr);
    g_saved_stderr_fd = dup(STDERR_FILENO);
    if (g_saved_stderr_fd < 0) { close(log_fd); g_saved_stderr_fd = -1; return; }
    dup2(log_fd, STDERR_FILENO);
    close(log_fd);
    // Unbuffered so log lines land promptly even on an abnormal exit.
    setvbuf(stderr, NULL, _IONBF, 0);
}

static void tui_release_stderr(void)
{
    if (g_saved_stderr_fd == -1) return;
    fflush(stderr);
    dup2(g_saved_stderr_fd, STDERR_FILENO);
    close(g_saved_stderr_fd);
    g_saved_stderr_fd = -1;
}

// One-line closing status: did anything hit the redirected stderr log
// this run? Call once, last, after the TUI has been torn down and stderr
// restored. Silent when there was no real log file (e.g. a viewer).
static void tui_report_errors(void)
{
    if (g_stderr_log_path[0] == '\0') return;
    struct stat st;
    if (stat(g_stderr_log_path, &st) == 0
        && st.st_size > g_stderr_log_start_size) {
        printf("Errors logged in %s\n", g_stderr_log_path);
    } else {
        printf("No errors reported\n");
    }
    fflush(stdout);
}

// --- Crash / quit signal safety net -------------------------------
//
// A streaming SDR yanked off USB makes UHD throw from a C++ destructor
// deep inside recv (on our worker thread), which the C API can't turn
// into an error return — it goes std::terminate -> abort -> SIGABRT. We
// can't recover from that, but we can refuse to leave the operator with
// a cryptic "Abort trap: 6" and a terminal stuck in ncurses raw mode.
// The handler restores the screen, prints one clear line to the real
// terminal, and re-raises so the process still dies with the original
// signal (and a core dump if enabled). SIGINT/SIGTERM instead ask the
// main loop to quit cleanly via its normal teardown.
static volatile sig_atomic_t g_signal_quit = 0;
// Claimed (test-and-set) by the first thread to enter the crash handler.
// On device loss TWO threads abort at once (our RX worker and a UHD
// internal thread); only one may run the terminal-restore + message.
static volatile char g_crash_claimed = 0;
// Terminal modes captured before ncurses took over, so the crash handler
// can restore the tty without relying on a (thread-racy) endwin().
static struct termios g_saved_termios;
static int            g_have_saved_termios = 0;

// write() a string literal — async-signal-safe (sizeof-1 drops the NUL).
#define CRASH_WRITE(fd, s) do { ssize_t w_ = write((fd), (s), sizeof(s) - 1); (void) w_; } while (0)

static void graceful_quit_handler(int sig)
{
    (void) sig;
    g_signal_quit = 1;   // main loop notices and runs its normal teardown
}

static void crash_handler(int sig)
{
    // On device loss two threads abort almost simultaneously. The first
    // claims the handler and does the cleanup; any other thread must NOT
    // _exit here — that single fast syscall would terminate the process
    // before the first thread finished restoring the terminal and
    // printing the message (the bug: "waterfall down, terminal garbled,
    // no message"). Instead, wait: the claiming thread re-raises and
    // brings the whole process down within microseconds. Bounded so a
    // wedged cleanup can't hang forever.
    if (__atomic_test_and_set(&g_crash_claimed, __ATOMIC_SEQ_CST)) {
        for (int i = 0; i < 50; i++) {
            struct timespec ts = { 0, 10 * 1000 * 1000 };   // 10 ms
            nanosleep(&ts, NULL);
        }
        _exit(128 + sig);
    }

    // Kill the spawned live-waterfall window so it doesn't orphan when we
    // die — the normal teardown that usually does this won't run. kill()
    // is async-signal-safe.
    if (g_live_waterfall_pid > 0) {
        kill(g_live_waterfall_pid, SIGKILL);
    }

    // Restore the terminal DETERMINISTICALLY. The fault is usually on the
    // RX worker thread, where ncurses endwin() races the main thread's
    // drawing and can leave the tty half-restored ("needs reset"). So we
    // skip endwin() and instead restore the saved termios + emit the
    // raw escapes to leave the alt-screen, show the cursor, and reset
    // attributes. tcsetattr + write are plain syscalls, safe from here.
    if (g_have_saved_termios) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_termios);
    }
    CRASH_WRITE(STDOUT_FILENO, "\033[?1049l\033[?25h\033[0m\r\n");

    // stderr is redirected to the pass-folder log during the TUI; aim the
    // message at the real terminal so the operator actually sees it.
    int fd = (g_saved_stderr_fd >= 0) ? g_saved_stderr_fd : STDERR_FILENO;
    CRASH_WRITE(fd, "\n*** simple_sat_ops: fatal error");
    switch (sig) {
        case SIGABRT: CRASH_WRITE(fd, " (SIGABRT - a USB/SDR device was likely disconnected)"); break;
        case SIGSEGV: CRASH_WRITE(fd, " (SIGSEGV)"); break;
        case SIGBUS:  CRASH_WRITE(fd, " (SIGBUS)");  break;
        default:      CRASH_WRITE(fd, "");           break;
    }
    CRASH_WRITE(fd,
        ".\nThe terminal has been restored. Any detail is in the pass-folder\n"
        "log (sso_stderr.log). Reconnect the device and restart.\n");

    // Re-raise with the default disposition so the process terminates
    // with the original signal (preserving core-dump behaviour).
    signal(sig, SIG_DFL);
    raise(sig);
}

static void install_signal_handlers(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = crash_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGABRT, &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGBUS,  &sa, NULL);

    struct sigaction sq;
    memset(&sq, 0, sizeof sq);
    sq.sa_handler = graceful_quit_handler;
    sigemptyset(&sq.sa_mask);
    sigaction(SIGINT,  &sq, NULL);
    sigaction(SIGTERM, &sq, NULL);
}

void init_window(void)
{
    // setlocale BEFORE initscr so ncurses knows the terminal can render
    // its alternate-character-set line glyphs (and UTF-8 elsewhere).
    // Without this, box() and friends emit the ACS fallback letters
    // (q for horizontal, x for vertical, lkjm for corners) instead of
    // line-drawing characters.
    setlocale(LC_ALL, "");

    initscr(); cbreak(); noecho();
    nonl();
    timeout(0);
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);
    // ncurses defaults ESCDELAY to 1000 ms — fine for distinguishing
    // bare Esc from the leading byte of a function-key sequence, but
    // makes Esc-to-cancel and arrow-key composition feel sluggish.
    // 25 ms is the conventional snappy value; any real escape sequence
    // arrives in a few ms so this isn't tight.
    set_escdelay(25);
    start_color();
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    curs_set(0);

    // Now that ncurses owns the screen, divert stderr to the pass-folder
    // log so backend/library errors never paint over the panels.
    tui_grab_stderr();
}

// --- Reports -------------------------------------------------------

// Pure-render predictions panel — operator runs the SGP4 search
// upstream, viewer fills state.prediction from broadcast. No SGP4
// calls inside, no current-time reads, so both sides paint the same
// thing for the same input state.
static void render_predictions_panel(state_t *state, double jul_utc,
                                     int *print_row, int print_col)
{
    if (print_row == NULL) return;
    (void) jul_utc;
    int row = *print_row;
    int col = print_col;

    struct tm utc;
    UTC_Calendar_Now(&utc, NULL);
    char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    mvprintw(row++, col, "%15s   %d %s %04d %02d:%02d:%02d UTC", "date",
             utc.tm_mday, months[utc.tm_mon - 1], utc.tm_year,
             utc.tm_hour, utc.tm_min, utc.tm_sec);
    clrtoeol();

    row++;
    mvprintw(row++, col, "%15s   %s (%s)", "satellite",
             state->prediction.satellite_ephem.tle.sat_name,
             state->prediction.satellite_ephem.tle.idesg);
    clrtoeol();
    if (state->prediction.minutes_since_epoch / 1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attron(COLOR_PAIR(2));
    }
    mvprintw(row++, col, "%15s   %0.1f days", "epoch age",
             state->prediction.minutes_since_epoch / 1440.0);
    if (state->prediction.minutes_since_epoch / 1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attroff(COLOR_PAIR(2));
    }
    clrtoeol();

    if (state->in_pass) {
        mvprintw(row++, col, "%15s   %s", "status", "** IN PASS **");
        if (state->antenna_rotator.tracking) {
            printw(" (TRACKING)");
        } else {
            attron(COLOR_PAIR(1));
            printw(" (NOT tracking)");
            attroff(COLOR_PAIR(1));
        }
    } else {
        mvprintw(row++, col, "%15s   %s", "status", "** NOT in pass **");
    }
    clrtoeol();

    if (state->prediction.predicted_minutes_until_visible > 0) {
        double minutes_until = state->prediction.predicted_minutes_until_visible;
        time_t aos_t = time(NULL) + (time_t)(minutes_until * 60.0);
        struct tm aos_local;
        localtime_r(&aos_t, &aos_local);
        char aos_hhmm[8];
        snprintf(aos_hhmm, sizeof aos_hhmm, "%02d:%02d",
                 aos_local.tm_hour, aos_local.tm_min);
        if (minutes_until < 1) {
            mvprintw(row++, col, "%15s   %s ", "next pass", aos_hhmm);
            attron(COLOR_PAIR(2));
            printw("(in %.0fs)", floor(minutes_until * 60.0));
            attroff(COLOR_PAIR(2));
        } else if (minutes_until < 10) {
            mvprintw(row++, col, "%15s   %s (in %.1fm)", "next pass",
                     aos_hhmm, minutes_until);
        } else {
            mvprintw(row++, col, "%15s   %s (in %.0fm)", "next pass",
                     aos_hhmm, minutes_until);
        }
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   ", "elapsed time");
        attron(COLOR_PAIR(3));
        if (fabs(state->prediction.predicted_minutes_until_visible) < 1) {
            printw("%.0f seconds",
                   floor(-state->prediction.predicted_minutes_until_visible * 60.0));
        } else {
            printw("%.1f minutes",
                   -state->prediction.predicted_minutes_until_visible);
        }
        attroff(COLOR_PAIR(3));
        clrtoeol();
    }
    mvprintw(row++, col, "%15s   %.1f minutes", "duration",
             state->prediction.predicted_minutes_above_0_degrees);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f minutes", "el>30",
             state->prediction.predicted_minutes_above_30_degrees);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg", "max elevation",
             state->prediction.predicted_max_elevation);
    clrtoeol();

    *print_row = row;
}

// SGP4 work that report_predictions used to do inline. Operator calls
// this each tick so its state->prediction.predicted_* fields are
// fresh before render + broadcast.
static void compute_predictions(state_t *state, double jul_utc)
{
    minutes_until_visible(&state->prediction, jul_utc,
                          jul_utc + MAX_MINUTES_TO_PREDICT / 1440.0, 1.0);
    if (fabs(state->prediction.predicted_minutes_until_visible) < 1) {
        minutes_until_visible(&state->prediction, jul_utc,
                              jul_utc + 2.0 / 1440.0, 1. / 120.0);
    } else if (fabs(state->prediction.predicted_minutes_until_visible) < 10) {
        minutes_until_visible(&state->prediction, jul_utc,
                              jul_utc + 20.0 / 1440.0, 0.1);
    }
    if (state->prediction.predicted_minutes_until_visible > 0) {
        update_pass_predictions(&state->prediction,
            jul_utc + state->prediction.predicted_minutes_until_visible / 1440.0,
            0.1);
    } else if (state->prediction.predicted_max_elevation == -180.0) {
        // Started mid-pass: walk back to AOS so update_pass_predictions
        // captures the true max elevation rather than just the remainder.
        update_pass_predictions(&state->prediction,
            jul_utc + state->prediction.predicted_minutes_until_visible / 1440.0,
            0.1);
    }
}

void report_predictions(state_t *state, double jul_utc, int *print_row, int print_col)
{
    compute_predictions(state, jul_utc);
    render_predictions_panel(state, jul_utc, print_row, print_col);
}

// Render the operator/carrier/rotator status block. Caller supplies
// the values so this function works for both the operator (who reads
// the rotator from hardware) and the viewer (who pulls them from the
// IPC broadcast).
typedef struct {
    int    control_mode;     // 1 = operator process; 0 = viewer process
    const  char *operator_user;
    const  char *viewers;    // comma-separated viewer names, or "(none)"
    double carrier_hz;
    int    have_rotator;     // 1 -> render az/el block; 0 -> "not initialized"
    double current_az;
    double current_el;
    double target_az;
    double target_el;
    int    flip;
    // HMAC keyfile display. Only the operator process fills these; the
    // viewer leaves status == HMAC_DISPLAY_UNSET so the row is skipped.
    const char           *hmac_path;
    hmac_display_status_t hmac_status;
    ssize_t               hmac_bytes;
    // T/R antenna switch. Operator-only: the viewer leaves tr_show = 0
    // so the block is skipped (the switch status isn't broadcast).
    int         tr_show;
    int         tr_connected;
    int         tr_stale;
    const char *tr_device;
    const char *tr_state;        // "RX" / "TX" / "?"
    const char *tr_mode;         // "AUTO" / "FORCE_TX" / ...
    double      tr_last_tx_ago_s; // NAN or +inf -> placeholder
} status_panel_t;

// Format a duration (seconds) as a compact "Dd Hh Mm Ss" string, emitting
// only the parts that are needed: "2s", "1h 12s", "3d 4h", etc. Leading
// zero units are dropped, and an interior zero unit is skipped (1h 0m 12s
// -> "1h 12s"). A duration that rounds to zero renders as "0s". Seconds
// are rounded to the nearest whole second.
static void format_duration_compact(double seconds, char *out, size_t n)
{
    if (n == 0) return;
    if (seconds < 0) seconds = 0;
    long total = (long) (seconds + 0.5);
    long days  =  total / 86400;
    long hours = (total % 86400) / 3600;
    long mins  = (total % 3600) / 60;
    long secs  =  total % 60;

    size_t off = 0;
    if (days > 0) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldd",
                                 off ? " " : "", days);
    }
    if (hours > 0 && off < n) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldh",
                                 off ? " " : "", hours);
    }
    if (mins > 0 && off < n) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldm",
                                 off ? " " : "", mins);
    }
    // Show seconds when nonzero, or when nothing else was emitted (so a
    // sub-second / zero duration still prints "0s").
    if ((secs > 0 || off == 0) && off < n) {
        snprintf(out + off, n - off, "%s%lds", off ? " " : "", secs);
    }
}

static void render_status_panel(const status_panel_t *p,
                                int *print_row, int print_col)
{
    if (print_row == NULL) return;
    int row = *print_row;
    int col = print_col;

    mvprintw(0, 0, "%-15s %s   viewers: %s",
             "OPERATOR",
             p->operator_user ? p->operator_user : "?",
             p->viewers && p->viewers[0] ? p->viewers : "(none)");
    clrtoeol();

    // HMAC keyfile selection. Path-only display — no key bytes ever
    // reach the UI. The operator uses this to verify they're pointed
    // at the right key for this pass (operations vs dev, shared vs
    // per-user fallback). Viewer process skips this line.
    if (p->control_mode == 1) {
        const char *path = (p->hmac_path && p->hmac_path[0])
                           ? p->hmac_path : "(unresolved)";
        const char *tag;
        char tag_buf[40];
        switch (p->hmac_status) {
        case HMAC_DISPLAY_OK:
            snprintf(tag_buf, sizeof tag_buf, "(%zd bytes ok)",
                     p->hmac_bytes);
            tag = tag_buf;
            break;
        case HMAC_DISPLAY_MISSING: tag = "(MISSING)";      break;
        case HMAC_DISPLAY_BAD:     tag = "(BAD — see log)"; break;
        case HMAC_DISPLAY_UNSET:
        default:                   tag = "(unset)";        break;
        }
        mvprintw(1, 0, "%-15s %s  %s", "HMAC keyfile", path, tag);
        clrtoeol();
    }

    // (CARRIER row removed — the same Doppler-shifted carrier is shown
    // on the RX panel's "RX freq" line, alongside the LO ± BW row that
    // tells the operator where the SDR is actually listening.)
    if (p->have_rotator) {
        double az_display = p->current_az;
        if (az_display < 0) az_display += 360.0;
        double target_az_display = p->target_az;
        if (target_az_display < 0) target_az_display += 360.0;
        const char *flip_tag = p->flip ? " (flip)" : "";
        mvprintw(row++, col, "%15s   %.1f deg%s", "target azimuth",
                 target_az_display, flip_tag);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "azimuth", az_display);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg%s", "target elevation",
                 p->target_el, flip_tag);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "elevation", p->current_el);
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   %s",
                 "antenna rotator", "* not initialized *");
        clrtoeol();
    }

    // T/R antenna switch block (operator-only).
    if (p->tr_show) {
        row++;
        if (p->tr_connected) {
            const char *rfs  = (p->tr_state && p->tr_state[0]) ? p->tr_state : "?";
            const char *mode = (p->tr_mode  && p->tr_mode[0])  ? p->tr_mode  : "?";
            mvprintw(row++, col, "%15s   connected (%s)",
                     "T/R switch", p->tr_device ? p->tr_device : "?");
            clrtoeol();

            // RF state in red while transmitting or if the link's gone
            // stale (the shown state may be many seconds old).
            int rfs_warn = (strcmp(rfs, "TX") == 0) || p->tr_stale;
            if (rfs_warn) attron(COLOR_PAIR(1));
            mvprintw(row++, col, "%15s   %s%s",
                     "RF state", rfs, p->tr_stale ? "  (stale)" : "");
            if (rfs_warn) attroff(COLOR_PAIR(1));
            clrtoeol();

            // Mode in yellow when it isn't AUTO.
            int mode_warn = (strcmp(mode, "AUTO") != 0);
            if (mode_warn) attron(COLOR_PAIR(2));
            mvprintw(row++, col, "%15s   %s", "mode", mode);
            if (mode_warn) attroff(COLOR_PAIR(2));
            clrtoeol();

            // NAN (field absent / unparseable) and +inf ("never") both
            // render as the placeholder.
            if (isnan(p->tr_last_tx_ago_s) || isinf(p->tr_last_tx_ago_s)) {
                mvprintw(row++, col, "%15s   --", "last TX");
            } else {
                char ago[32];
                format_duration_compact(p->tr_last_tx_ago_s, ago, sizeof ago);
                mvprintw(row++, col, "%15s   %s ago", "last TX", ago);
            }
            clrtoeol();
        } else {
            mvprintw(row++, col, "%15s   %s",
                     "T/R switch", "* not connected *");
            clrtoeol();
        }
    }

    *print_row = row;
}

// Build a comma-separated list of currently-connected viewer/external
// clients. Skips anonymous (no-name) slots and is bounded by the
// caller's buffer.
static void operator_viewers_list(char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    if (!g_ipc) return;
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char user[64], role[16], since[40];
    size_t written = 0;
    while (sso_ipc_server_next_client(g_ipc, &it, &cid,
                                       user, sizeof user,
                                       role, sizeof role,
                                       since, sizeof since) == 0) {
        if (!user[0]) continue;
        size_t nlen = strlen(user);
        size_t need = nlen + (written > 0 ? 1 : 0);
        if (written + need + 1 >= out_size) break;
        if (written > 0) out[written++] = ',';
        memcpy(out + written, user, nlen);
        written += nlen;
        out[written] = '\0';
    }
}

// Range-check, write target_* bookkeeping, and submit the wire-level SET
// to the rotator worker. Replaces the previous direct
// antenna_rotator_set_unwrapped() call; the target / wrap bookkeeping
// stays on the main thread, only the serial I/O moves to the worker.
static int main_rotator_submit_set(state_t *state,
                                    double az_unwrapped, double elevation)
{
    if (az_unwrapped < ANTENNA_ROTATOR_MINIMUM_AZIMUTH
        || az_unwrapped > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
        return ANTENNA_ROTATOR_AZIMUTH_LIMIT;
    }
    if (elevation < ANTENNA_ROTATOR_MINIMUM_ELEVATION
        || elevation > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
        return ANTENNA_ROTATOR_ELEVATION_LIMIT;
    }
    state->antenna_rotator.target_azimuth_unwrapped = az_unwrapped;
    state->antenna_rotator.target_azimuth           = az_unwrapped;
    state->antenna_rotator.target_elevation         = elevation;
    state->antenna_rotator.unwrapped_target_valid   = 1;
    if (g_rot_async != NULL) {
        antenna_rotator_async_submit_set(g_rot_async, az_unwrapped, elevation);
    }
    return ANTENNA_ROTATOR_OK;
}

// Mirror of antenna_rotator_increase_azimuth() but routed through the
// async worker via main_rotator_submit_set(). Used by the `[` / `]`
// (5 deg) and `{` / `}` (1 deg, shifted) hotkeys.
static int main_rotator_increase_azimuth(state_t *state, double delta)
{
    double base = state->antenna_rotator.unwrapped_target_valid
        ? state->antenna_rotator.target_azimuth_unwrapped
        : state->antenna_rotator.target_azimuth;
    return main_rotator_submit_set(state, base + delta,
                                    state->antenna_rotator.target_elevation);
}

// Same idea but stepping elevation. Used by the `,` / `.` (5 deg) and
// `<` / `>` (1 deg, shifted) hotkeys. The azimuth target is held — only
// the wire-level SET goes out, on the worker.
static int main_rotator_increase_elevation(state_t *state, double delta)
{
    double az = state->antenna_rotator.unwrapped_target_valid
        ? state->antenna_rotator.target_azimuth_unwrapped
        : state->antenna_rotator.target_azimuth;
    double new_el = state->antenna_rotator.target_elevation + delta;
    return main_rotator_submit_set(state, az, new_el);
}

// --- Pursuit planner integration ----------------------------------
//
// At AOS we pre-sample the satellite trajectory in unwrapped mech
// coords into g_pursuit_track, then ask the planner (src/orbit/
// pursuit.c) for a rate-feasible whole-pass antenna trajectory. Each
// track-loop tick the loop reads the next waypoint via
// pursuit_aim_at() and submits it through the existing
// main_rotator_submit_set() path — the playback is just "aim at the
// next waypoint", and the worker's constant-rate slew interpolates
// the segment for us. The planner runs once per pass; mid-pass the
// only work is the O(log N) waypoint lookup.

// Free the pre-sampled trajectory arrays. Idempotent.
static void pursuit_track_free(pursuit_track_t *trk)
{
    if (trk == NULL) return;
    free(trk->t_jul);
    free(trk->az_unwrapped);
    free(trk->el);
    trk->t_jul        = NULL;
    trk->az_unwrapped = NULL;
    trk->el           = NULL;
    trk->n            = 0;
}

// pursuit_sat_sample_fn_t backing the planner. Linear-interpolates the
// pre-sampled track at `jul`. Saturates at the endpoints so iterations
// that wander a fraction of a second beyond AOS / LOS still produce a
// sensible answer.
static int pursuit_track_lookup(double jul, double *az, double *el, void *ctx)
{
    const pursuit_track_t *trk = (const pursuit_track_t *) ctx;
    if (trk == NULL || trk->n == 0) return -1;
    if (jul <= trk->t_jul[0]) {
        if (az) *az = trk->az_unwrapped[0];
        if (el) *el = trk->el[0];
        return 0;
    }
    if (jul >= trk->t_jul[trk->n - 1]) {
        if (az) *az = trk->az_unwrapped[trk->n - 1];
        if (el) *el = trk->el[trk->n - 1];
        return 0;
    }
    size_t lo = 0, hi = trk->n - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (trk->t_jul[mid] <= jul) lo = mid;
        else hi = mid;
    }
    double frac = (jul - trk->t_jul[lo])
                / (trk->t_jul[hi] - trk->t_jul[lo]);
    if (az) *az = trk->az_unwrapped[lo]
                + (trk->az_unwrapped[hi] - trk->az_unwrapped[lo]) * frac;
    if (el) *el = trk->el[lo]
                + (trk->el[hi] - trk->el[lo]) * frac;
    return 0;
}

// Sample the live prediction's satellite at 1 s intervals across the
// pass window, run it through the existing flip mech-coord mapping (so
// flip-mode passes get back-hemisphere mech_el up to 180), accumulate
// unwrapped azimuth in time order. We work on a memcpy of
// state->prediction so the live satellite_ephem.azimuth/elevation
// displayed in the UI is not perturbed. Returns 0 on success.
static int pursuit_track_build(const state_t *state,
                                double jul_aos, double jul_los,
                                int flip,
                                double aos_az, double los_az,
                                double aos_jul, double los_jul,
                                double a0_unwrapped,
                                pursuit_track_t *out)
{
    if (out == NULL) return -1;
    pursuit_track_free(out);
    double dt_days = 1.0 / 86400.0;     // 1 s sampling
    if (jul_los <= jul_aos) return -1;
    size_t n = (size_t) floor((jul_los - jul_aos) / dt_days) + 1;
    if (n < 2)    n = 2;
    if (n > 4096) n = 4096;             // sanity cap; ~68 min pass
    out->t_jul        = calloc(n, sizeof *out->t_jul);
    out->az_unwrapped = calloc(n, sizeof *out->az_unwrapped);
    out->el           = calloc(n, sizeof *out->el);
    if (out->t_jul == NULL || out->az_unwrapped == NULL
        || out->el == NULL) {
        pursuit_track_free(out);
        return -1;
    }
    out->n = n;

    prediction_t scratch;
    memcpy(&scratch, &state->prediction, sizeof scratch);

    double prev = a0_unwrapped;
    double span = jul_los - jul_aos;
    for (size_t i = 0; i < n; ++i) {
        double frac = (n == 1) ? 0.0 : (double) i / (double) (n - 1);
        double t = jul_aos + frac * span;
        out->t_jul[i] = t;
        update_satellite_position(&scratch, t);
        double sat_az = scratch.satellite_ephem.azimuth;
        double sat_el = scratch.satellite_ephem.elevation;
        double mech_az = sat_az;
        double mech_el = sat_el;
        if (flip) {
            double progress = 0.0;
            double pass_jul = los_jul - aos_jul;
            if (pass_jul > 0.0) progress = (t - aos_jul) / pass_jul;
            int half = 0;
            antenna_rotator_to_mech_coords(1, aos_az, los_az, progress,
                                            sat_az, sat_el,
                                            &mech_az, &mech_el, &half);
        }
        prev = antenna_rotator_accumulate_unwrapped(prev, mech_az);
        out->az_unwrapped[i] = prev;
        out->el[i] = mech_el;
    }
    return 0;
}

// Free the current plan + its sampled trajectory. Idempotent.
static void main_pursuit_clear_plan(void)
{
    pursuit_plan_free(&g_pursuit_plan);
    pursuit_track_free(&g_pursuit_track);
}

// Build (or rebuild) the whole-pass plan from the current prediction.
// `jul_now` is used to clamp the plan start to the current time if
// we're already past AOS (mid-pass re-enter). Quietly does nothing
// when pursuit is disabled or prerequisites are missing — the caller
// just keeps the existing aim-where-sat-is-now logic.
static void main_pursuit_build_plan(state_t *state, double jul_now)
{
    main_pursuit_clear_plan();
    if (g_without_rotator_pursuit)              return;
    if (g_pursuit_az_dps <= 0.0)                return;
    if (g_pursuit_el_dps <= 0.0)                return;
    if (!state->have_antenna_rotator)           return;
    if (!state->antenna_rotator.unwrapped_target_valid) return;

    double aos = state->prediction.predicted_ascension_jul_utc;
    double los = state->prediction.predicted_descent_jul_utc;
    if (aos <= 0.0 || los <= aos)               return;
    if (jul_now > aos) aos = jul_now;
    if (los - aos < 5.0 / 86400.0)              return;  // <5 s left

    int    flip    = state->antenna_rotator.flip_mode_pass;
    double aos_az  = state->antenna_rotator.flip_aos_az;
    double los_az  = state->antenna_rotator.flip_los_az;
    double aos_jul = state->antenna_rotator.flip_aos_jul;
    double los_jul = state->antenna_rotator.flip_los_jul;
    double a0      = state->antenna_rotator.target_azimuth_unwrapped;
    double e0      = state->antenna_rotator.target_elevation;

    if (pursuit_track_build(state, aos, los,
                             flip, aos_az, los_az, aos_jul, los_jul,
                             a0, &g_pursuit_track) != 0) {
        fprintf(stderr, "pursuit: track sampling failed; "
                        "falling back to aim-where-sat-is-now\n");
        main_pursuit_clear_plan();
        return;
    }

    pursuit_config_t cfg;
    pursuit_config_defaults(&cfg);
    cfg.jul_aos      = aos;
    cfg.jul_los      = los;
    cfg.r_az_dps     = g_pursuit_az_dps;
    cfg.r_el_dps     = g_pursuit_el_dps;
    cfg.a0_unwrapped = a0;
    cfg.e0           = e0;

    if (pursuit_plan_build(&cfg, pursuit_track_lookup, &g_pursuit_track,
                            &g_pursuit_plan) != 0) {
        fprintf(stderr, "pursuit: plan build failed; "
                        "falling back to aim-where-sat-is-now\n");
        main_pursuit_clear_plan();
        return;
    }
    // Sanity bound: a plan with > 30 deg max error is suspect; the
    // calibration may be wildly off. Discard and let the track loop
    // fall back rather than driving the antenna to bogus targets.
    if (g_pursuit_plan.max_error_deg > 30.0) {
        fprintf(stderr,
                "pursuit: plan max_err=%.1f deg > 30; disabled, "
                "falling back to aim-where-sat-is-now\n",
                g_pursuit_plan.max_error_deg);
        main_pursuit_clear_plan();
        return;
    }
    fprintf(stderr,
            "pursuit: plan built %zu waypoints, "
            "max_err=%.2f mean_err=%.2f deg, %d iter\n",
            g_pursuit_plan.n_waypoints,
            g_pursuit_plan.max_error_deg, g_pursuit_plan.mean_error_deg,
            g_pursuit_plan.iterations_used);
}

// Two paths point at the same TLE file? Canonicalise both with
// realpath() so /a/./tle and /a/tle compare equal; fall back to a
// plain string compare if either path can't be resolved (e.g. a
// relative path whose file was just removed). NULL/empty never match.
static int retarget_same_file(const char *a, const char *b)
{
    if (a == NULL || b == NULL || a[0] == '\0' || b[0] == '\0') return 0;
    char *ra = realpath(a, NULL);
    char *rb = realpath(b, NULL);
    int same = strcmp(ra ? ra : a, rb ? rb : b) == 0;
    free(ra);
    free(rb);
    return same;
}

// Read the FIRST satellite from a 3-line TLE file: its name line and the
// two element lines, packed into the 139-byte buffer sgp4sdp4 wants
// (two 69-char lines joined, NUL-padded). Stops at the first complete
// record. Returns 0 on success, -1 if the file can't be opened or a
// full name+line1+line2 triple isn't present.
static int retarget_read_first_tle(const char *path,
                                   char *name, size_t name_cap,
                                   char tle_out[139])
{
    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;
    char line[256];
    char l1[256] = {0};
    char l2[256] = {0};
    int  have_name = 0;
    name[0] = '\0';
    while (fgets(line, sizeof line, f) != NULL) {
        size_t n = strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'
                      || line[n - 1] == ' '  || line[n - 1] == '\t')) {
            line[--n] = '\0';
        }
        if (n == 0) continue;
        int is_elem = ((line[0] == '1' || line[0] == '2') && line[1] == ' ');
        if (!have_name && !is_elem) {
            snprintf(name, name_cap, "%s", line);
            have_name = 1;
            continue;
        }
        if (line[0] == '1' && line[1] == ' ' && l1[0] == '\0') {
            snprintf(l1, sizeof l1, "%s", line);
            continue;
        }
        if (line[0] == '2' && line[1] == ' ' && l2[0] == '\0') {
            snprintf(l2, sizeof l2, "%s", line);
            break;  // first complete record -> done
        }
    }
    fclose(f);
    if (!have_name || l1[0] == '\0' || l2[0] == '\0') return -1;
    size_t a = strlen(l1);
    size_t b = strlen(l2);
    if (a > 69) a = 69;
    if (b > 69) b = 69;
    memset(tle_out, 0, 139);
    memcpy(tle_out, l1, a);
    memcpy(tle_out + 69, l2, b);
    return 0;
}

// Swap the tracked satellite mid-pass to the first one in `path`.
//
// The new target's elements replace the live ephemeris; the SGP4/SDP4
// selection (global flag state) is re-picked for it. Pass geometry is
// recomputed from scratch so the display, flip decision and pursuit
// plan all reflect the new object. The antenna is NOT homed first: we
// clear the flip latch + per-pass tracking flag so the track loop
// re-derives and aims straight at the new sat's current sky position,
// and we keep target_azimuth_unwrapped so the azimuth accumulator picks
// the co-terminal nearest where the antenna already is (short slew, no
// trip through 0,0).
//
// A repeat :retarget on the same file is a no-op (RETARGET_SAME);
// different files swap even when they name the same satellite. Returns
// one of the RETARGET_* codes.
static int retarget_to_tle(state_t *state, const char *path)
{
    if (path == NULL || path[0] == '\0') return RETARGET_BAD_ARG;
    if (retarget_same_file(path, g_target_tle_path)) return RETARGET_SAME;

    char name[64];
    char tle[139];
    if (retarget_read_first_tle(path, name, sizeof name, tle) != 0) {
        return RETARGET_READ_ERR;
    }
    if (!Good_Elements(tle)) return RETARGET_BAD_TLE;

    // Commit the new elements. select_ephemeris rewrites the TLE in
    // place, so it must be called exactly once on these freshly
    // converted elements -- clear the global flags first so the
    // SGP4/SDP4 choice is made fresh for this object.
    snprintf(g_target_name, sizeof g_target_name, "%s", name);
    Convert_Satellite_Data(tle, &state->prediction.satellite_ephem.tle);
    snprintf(state->prediction.satellite_ephem.tle.sat_name,
             sizeof state->prediction.satellite_ephem.tle.sat_name,
             "%s", name);
    state->prediction.satellite_ephem.name = g_target_name;
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state->prediction.satellite_ephem.tle);

    // Recompute pass geometry for the new target. Reset max-elevation to
    // the sentinel so compute_predictions walks back to AOS when we're
    // already mid-pass (otherwise it would only see the pass remainder).
    state->prediction.predicted_max_elevation = -180.0;
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_now = Julian_Date(&utc, &tv);
    update_satellite_position(&state->prediction, jul_now);
    compute_predictions(state, jul_now);

    // Re-aim without homing: drop the old plan and clear the per-pass
    // latches. An active track (satellite_tracking set) re-derives the
    // flip decision, rebuilds the pursuit plan, and slews to the new
    // target on the next tick. If we weren't tracking, only the target
    // changes -- nothing moves until the operator presses T.
    main_pursuit_clear_plan();
    state->antenna_rotator.tracking           = 0;
    state->antenna_rotator.flip_mode_pass     = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half          = 0;

    snprintf(g_target_tle_path, sizeof g_target_tle_path, "%s", path);
    return RETARGET_OK;
}

// Pull az/el from the async snapshot and write them through to az/el AND
// target_* on state. Used after a STOP / on tracking start when targets
// should reflect the physical position. Returns 0 on success, -1 if no
// good status has landed yet (or it's gone stale).
static int main_rotator_refresh_targets_from_snapshot(state_t *state)
{
    if (g_rot_async == NULL) return -1;
    double az = 0.0, el = 0.0;
    int    ok = 0, stale_ms = 0;
    antenna_rotator_async_snapshot(g_rot_async, &az, &el, &ok, &stale_ms, NULL);
    if (!ok || stale_ms > 1500) return -1;
    state->antenna_rotator.azimuth                  = az;
    state->antenna_rotator.elevation                = el;
    state->antenna_rotator.target_azimuth           = az;
    state->antenna_rotator.target_elevation         = el;
    state->antenna_rotator.target_azimuth_unwrapped = az;
    state->antenna_rotator.unwrapped_target_valid   = 1;
    return 0;
}

void report_status(state_t *state, int *print_row, int print_col)
{
    if (print_row == NULL) return;

    static char viewers[256];
    operator_viewers_list(viewers, sizeof viewers);

    status_panel_t p;
    memset(&p, 0, sizeof p);
    p.control_mode  = g_control_mode;
    p.operator_user = g_operator_user;
    p.viewers       = viewers[0] ? viewers : "(none)";

    double display_dl_hz = state->doppler_downlink_frequency_hz;
    if (display_dl_hz == 0.0) display_dl_hz = state->nominal_downlink_frequency_hz;
    p.carrier_hz = display_dl_hz;

    p.hmac_path   = g_hmac_keyfile_path;
    p.hmac_status = g_hmac_display_status;
    p.hmac_bytes  = (ssize_t) g_hmac_key_len;

    p.have_rotator = state->have_antenna_rotator;
    if (state->have_antenna_rotator) {
        // The serial roundtrip moved to a worker thread (see
        // src/hw/antenna_rotator_async.c); we just read the latest
        // snapshot here. No more 5-10 ms per redraw on the main loop,
        // and no 500 ms VTIME hang if the cable is unplugged.
        double azimuth = state->antenna_rotator.azimuth;
        double elevation = state->antenna_rotator.elevation;
        int    rot_ok = 0;
        int    rot_stale_ms = 0;
        if (g_rot_async != NULL) {
            antenna_rotator_async_snapshot(g_rot_async,
                                            &azimuth, &elevation,
                                            &rot_ok, &rot_stale_ms, NULL);
            // Cache the snapshot back into state so other code (the
            // antenna_is_moving heuristic, IPC broadcast, etc.) reads a
            // single consistent value across the tick.
            state->antenna_rotator.azimuth   = azimuth;
            state->antenna_rotator.elevation = elevation;
        }
        p.current_az = azimuth;
        p.current_el = elevation;
        p.target_az  = state->antenna_rotator.target_azimuth;
        p.target_el  = state->antenna_rotator.target_elevation;
        p.flip       = state->antenna_rotator.flip_mode_pass;
    }

    // T/R switch block — operator-only (this process owns the serial
    // link). The viewer never sets tr_show, so its panel skips it.
    p.tr_show = 1;
    p.tr_connected = state->have_tr_switch;
    if (state->have_tr_switch) {
        p.tr_device        = state->tr_switch.device_filename;
        p.tr_state         = state->tr_switch.state_str;
        p.tr_mode          = state->tr_switch.mode_str;
        p.tr_last_tx_ago_s = state->tr_switch.last_tx_ago_s;
        p.tr_stale         = tr_switch_is_stale(&state->tr_switch,
                                                monotonic_seconds());
    }

    render_status_panel(&p, print_row, print_col);
}

void report_position(state_t *state, int *print_row, int print_col)
{
    if (print_row == NULL) {
        return;
    }
    int row = *print_row;
    int col = print_col;

    mvprintw(row++, col, "%15s   %.2f deg", "azimuth",
             state->prediction.satellite_ephem.azimuth);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f deg", "elevation",
             state->prediction.satellite_ephem.elevation);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km", "altitude",
             state->prediction.satellite_ephem.altitude_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg N", "latitude",
             state->prediction.satellite_ephem.latitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg E", "longitude",
             state->prediction.satellite_ephem.longitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "speed",
             state->prediction.satellite_ephem.speed_km_s);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f km", "range",
             state->prediction.satellite_ephem.range_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "range rate",
             state->prediction.satellite_ephem.range_rate_km_s);
    clrtoeol();

    *print_row = row;
}

// --- Viewer mode --------------------------------------------------
//
// Read-only mirror of the operator instance. The viewer does NOT run
// SGP4 and does NOT load a TLE — it just deposits every broadcast
// field into a state_t and calls the same render helpers the operator
// uses, so the two displays are byte-identical except for the help text.

static int    g_viewer_event_pending      = 0;
static int    g_viewer_has_state          = 0;
static char   g_viewer_operator[64]       = "";
static char   g_viewer_roster_json[1024]  = "";
static time_t g_viewer_last_event         = 0;
static int    g_viewer_running            = 1;
// Mirror of the operator's ":" prompt state. cmd_active = 1 between
// the first cmd-preview after :  and the cmd-executed that closes it.
// cmd_buf and cmd_status track g_cmd_buf / g_cmd_status verbatim so
// the viewer's bottom row matches the operator's exactly. Sized to the
// wire field (sso_event_t.cmd_text) so snprintf can't truncate.
static int    g_viewer_cmd_active         = 0;
static char   g_viewer_cmd_buf[160]       = "";
static char   g_viewer_cmd_status[160]    = "";
// Mirror of the operator's auto-tcmd run. auto_on = 1 while the
// operator has a run to show; the render line is "<sent>/<total>
// sent (<state>)" and disappears when the operator closes the modal
// (the fields drop off the wire, decode zeroes them, we stash that).
static int  g_viewer_auto_on        = 0;
static int  g_viewer_auto_sent      = 0;
static int  g_viewer_auto_total     = 0;
static char g_viewer_auto_state[12] = "";
// Mirror of the operator's RX panel. Filled from STATE / WELCOME events;
// render_rx_panel reads it directly during viewer_render.
static rx_panel_data_t g_viewer_rx_panel;
// state_t whose fields the viewer mirrors from the broadcast each tick.
static state_t g_viewer_state;
static double  g_viewer_carrier_hz        = 0.0;
static double  g_viewer_jul_utc           = 0.0;
static int     g_viewer_has_rotator       = 0;
static char    g_viewer_tle_path[256]     = "";
static char    g_viewer_pass_folder[256]  = "";
// Take-control confirmation. Press 'c' once to arm, 'y' within
// CONFIRM_WINDOW_S seconds to commit. Anything else cancels.
#define VIEWER_CONFIRM_WINDOW_S 5
static time_t  g_viewer_confirm_until     = 0;

static void viewer_on_event(sso_ipc_client_t *cli, const sso_event_t *evt,
                            void *user)
{
    (void) cli;
    (void) user;
    if (evt->type == SSO_EVT_TX_COMMAND_PREVIEW
     || evt->type == SSO_EVT_TX_COMMAND_SENT
     || evt->type == SSO_EVT_TX_NOT_SENT) {
        tx_log_push(evt);
        g_viewer_last_event = time(NULL);
        g_viewer_event_pending = 1;
        return;
    }
    if (evt->type == SSO_EVT_CMD_PREVIEW) {
        g_viewer_cmd_active = 1;
        snprintf(g_viewer_cmd_buf, sizeof g_viewer_cmd_buf,
                 "%s", evt->cmd_text);
        g_viewer_last_event = time(NULL);
        g_viewer_event_pending = 1;
        return;
    }
    if (evt->type == SSO_EVT_CMD_EXECUTED) {
        // Empty cmd_text + empty cmd_status = Esc/cancel; clear the row.
        // Otherwise show the executed-command result string just like the
        // operator does after cmd_dispatch returns.
        g_viewer_cmd_active = 0;
        g_viewer_cmd_buf[0] = '\0';
        snprintf(g_viewer_cmd_status, sizeof g_viewer_cmd_status,
                 "%s", evt->cmd_status);
        g_viewer_last_event = time(NULL);
        g_viewer_event_pending = 1;
        return;
    }
    if (evt->type != SSO_EVT_STATE && evt->type != SSO_EVT_WELCOME) {
        return;
    }
    g_viewer_last_event = time(NULL);
    if (evt->operator_user[0]) {
        snprintf(g_viewer_operator, sizeof g_viewer_operator, "%s",
                 evt->operator_user);
    }
    if (evt->roster_json[0]) {
        snprintf(g_viewer_roster_json, sizeof g_viewer_roster_json, "%s",
                 evt->roster_json);
    }
    if (!evt->has_state) return;

    state_t *s = &g_viewer_state;
    snprintf(s->prediction.satellite_ephem.tle.sat_name,
             sizeof s->prediction.satellite_ephem.tle.sat_name, "%s",
             evt->satellite);
    snprintf(s->prediction.satellite_ephem.tle.idesg,
             sizeof s->prediction.satellite_ephem.tle.idesg, "%s",
             evt->idesg);
    s->prediction.minutes_since_epoch              = evt->epoch_min;
    s->prediction.predicted_minutes_until_visible  = evt->min_visible;
    s->prediction.predicted_minutes_above_0_degrees  = evt->min_above_0;
    s->prediction.predicted_minutes_above_30_degrees = evt->min_above_30;
    s->prediction.predicted_max_elevation          = evt->max_el;
    s->prediction.satellite_ephem.azimuth          = evt->pred_az;
    s->prediction.satellite_ephem.elevation        = evt->pred_el;
    s->prediction.satellite_ephem.altitude_km      = evt->alt_km;
    s->prediction.satellite_ephem.latitude         = evt->lat_deg;
    s->prediction.satellite_ephem.longitude        = evt->lon_deg;
    s->prediction.satellite_ephem.speed_km_s       = evt->speed_kms;
    s->prediction.satellite_ephem.range_km         = evt->range_km;
    s->prediction.satellite_ephem.range_rate_km_s  = evt->range_rate_kms;
    s->in_pass                                     = evt->in_pass;
    s->antenna_rotator.tracking                    = evt->tracking;
    s->antenna_rotator.azimuth                     = evt->az;
    s->antenna_rotator.elevation                   = evt->el;
    s->antenna_rotator.target_azimuth              = evt->target_az;
    s->antenna_rotator.target_elevation            = evt->target_el;
    s->antenna_rotator.flip_mode_pass              = evt->flip;

    g_viewer_has_rotator = evt->has_rotator;
    g_viewer_jul_utc     = evt->jul_utc;
    g_viewer_carrier_hz  = (evt->doppler_hz != 0.0)
        ? (double)evt->freq_hz + evt->doppler_hz
        : (double)evt->freq_hz;
    if (evt->tle_path[0]) {
        snprintf(g_viewer_tle_path, sizeof g_viewer_tle_path,
                 "%s", evt->tle_path);
    }
    if (evt->pass_folder[0]) {
        snprintf(g_viewer_pass_folder, sizeof g_viewer_pass_folder,
                 "%s", evt->pass_folder);
    }

    // Auto-tcmd progress — stashed unconditionally so a broadcast
    // without the fields (run over, modal closed) clears the line.
    g_viewer_auto_on    = evt->auto_tcmd_on;
    g_viewer_auto_sent  = evt->auto_tcmd_sent;
    g_viewer_auto_total = evt->auto_tcmd_total;
    snprintf(g_viewer_auto_state, sizeof g_viewer_auto_state,
             "%s", evt->auto_tcmd_state);

    // Mirror the operator's RX panel from the broadcast. Wipe to zero
    // first so a slot that the operator hasn't decoded in this run
    // doesn't carry stale state from a previous event.
    memset(&g_viewer_rx_panel, 0, sizeof g_viewer_rx_panel);
    g_viewer_rx_panel.have_session  = evt->rx_have_session;
    snprintf(g_viewer_rx_panel.warning, sizeof g_viewer_rx_panel.warning,
             "%s", evt->rx_warning);
    if (evt->rx_have_session) {
        g_viewer_rx_panel.rec_active     = evt->rx_rec_active;
        g_viewer_rx_panel.rx_freq_hz     = evt->rx_freq_hz;
        g_viewer_rx_panel.peak_dbfs      = evt->rx_peak_dbfs;
        g_viewer_rx_panel.rms_dbfs       = evt->rx_rms_dbfs;
        g_viewer_rx_panel.frames_total   = (uint64_t) evt->rx_frames_total;
        g_viewer_rx_panel.frames_pcm     = (uint64_t) evt->rx_frames_pcm;
        g_viewer_rx_panel.frames_vit     = (uint64_t) evt->rx_frames_vit;
        snprintf(g_viewer_rx_panel.last_frame_summary,
                 sizeof g_viewer_rx_panel.last_frame_summary,
                 "%s", evt->rx_last_frame_summary);
        g_viewer_rx_panel.age_s = evt->rx_age_s;
        int slots = RX_PANEL_PT_COUNT < SSO_RX_PT_SLOTS
                  ? RX_PANEL_PT_COUNT : SSO_RX_PT_SLOTS;
        for (int s = 0; s < slots; ++s) {
            g_viewer_rx_panel.pt_count[s] = (uint64_t) evt->rx_pt_count[s];
            int pl = evt->rx_pt_payload_len[s];
            if (pl < 0) pl = 0;
            int copy = pl;
            if (copy > RX_PANEL_PAYLOAD_MAX) copy = RX_PANEL_PAYLOAD_MAX;
            g_viewer_rx_panel.pt_payload_len[s] = pl;
            memcpy(g_viewer_rx_panel.pt_payload[s],
                   evt->rx_pt_payload[s], (size_t) copy);
            snprintf(g_viewer_rx_panel.pt_summary[s],
                     sizeof g_viewer_rx_panel.pt_summary[s],
                     "%.*s",
                     (int)(sizeof g_viewer_rx_panel.pt_summary[s] - 1),
                     evt->rx_pt_summary[s]);
        }
        int rn = evt->rx_ribbon_n;
        if (rn > RIBBON_LEN) rn = RIBBON_LEN;
        g_viewer_rx_panel.ribbon_n = rn;
        memcpy(g_viewer_rx_panel.ribbon, evt->rx_ribbon, (size_t) rn);
        g_viewer_rx_panel.ribbon[rn] = '\0';
        memcpy(g_viewer_rx_panel.ribbon_peak, evt->rx_ribbon_peak,
               (size_t) rn * sizeof g_viewer_rx_panel.ribbon_peak[0]);
    }

    g_viewer_has_state   = 1;
    g_viewer_event_pending = 1;
}

// Format the roster array into "alice,bob,carol" for the header bar,
// skipping the operator (already shown separately) and any entry whose
// user is empty. The roster JSON is built by sso_event_set_roster with
// the schema [{"user":"...","role":"...","since":"..."},...], so we
// can scan for "user":"..." and "role":"..." pairs.
static void viewer_roster_users(char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    const char *p = g_viewer_roster_json;
    size_t written = 0;
    while ((p = strstr(p, "\"user\":\"")) != NULL) {
        p += 8;
        const char *uend = strchr(p, '"');
        if (!uend) break;
        char name[64];
        size_t nlen = (size_t)(uend - p);
        if (nlen >= sizeof name) nlen = sizeof name - 1;
        memcpy(name, p, nlen);
        name[nlen] = '\0';
        const char *q = strstr(uend, "\"role\":\"");
        const char *next = strstr(uend, "\"user\":\"");
        // role must belong to this object (before the next "user")
        int is_op = 0;
        if (q && (!next || q < next)) {
            q += 8;
            const char *rend = strchr(q, '"');
            if (rend && (size_t)(rend - q) == 8
                && memcmp(q, "operator", 8) == 0) {
                is_op = 1;
            }
        }
        p = uend;
        if (is_op || nlen == 0) continue;
        size_t need = nlen + (written > 0 ? 1 : 0);
        if (written + need + 1 >= out_size) break;
        if (written > 0) out[written++] = ',';
        memcpy(out + written, name, nlen);
        written += nlen;
        out[written] = '\0';
    }
}

// Read the operator's PID from /run/sso/simple_sat_ops.pid. Returns 0
// and sets *out_pid on success; -1 on missing/unreadable file.
static int read_operator_pid(pid_t *out_pid)
{
    char path[256];
    if (sso_ipc_pid_path(path, sizeof path, "simple_sat_ops") != 0)
        return -1;
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int pid = 0;
    int got = fscanf(f, "%d", &pid);
    fclose(f);
    if (got != 1 || pid <= 0) return -1;
    *out_pid = (pid_t) pid;
    return 0;
}

// Hand-off: send SIGUSR1 to the running operator, wait until its
// socket file goes away (= the process has finished cleanup), then
// re-exec this process as `simple_sat_ops --control` with the
// inherited TLE + pass folder so the new operator picks up where the
// old one left off. Does not return on success.
static void viewer_take_control(sso_ipc_client_t *cli, const char *argv0)
{
    if (!g_viewer_tle_path[0]) {
        fprintf(stderr,
            "simple_sat_ops viewer: no TLE path from operator yet — "
            "wait for a state broadcast and try again.\n");
        return;
    }
    pid_t op_pid = 0;
    if (read_operator_pid(&op_pid) != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: couldn't read operator pid file.\n");
        return;
    }
    if (kill(op_pid, SIGUSR1) != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: kill(%d, SIGUSR1) failed: %s\n",
            (int) op_pid, strerror(errno));
        return;
    }

    // Wait for the old operator to clean up and remove its socket.
    char sock[256];
    if (sso_ipc_socket_path(sock, sizeof sock, "simple_sat_ops") != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: socket path lookup failed.\n");
        return;
    }
    int waited_ms = 0;
    for (;;) {
        struct stat st;
        if (stat(sock, &st) != 0 && errno == ENOENT) break;
        if (waited_ms >= 5000) {
            fprintf(stderr,
                "simple_sat_ops viewer: timed out waiting for operator "
                "to release the socket (%s)\n", sock);
            return;
        }
        usleep(100000);
        waited_ms += 100;
    }

    // Close our viewer IPC + curses cleanly before exec'ing.
    sso_ipc_client_close(cli);
    endwin();
    tui_release_stderr();

    // Re-exec self as --control with inherited TLE and pass folder.
    // Filename args use the space form so the spawned child sees the
    // same TAB-completable spelling the user types.
    char self_path[1024];
    ssize_t n = readlink("/proc/self/exe", self_path, sizeof self_path - 1);
    const char *exe = (n > 0) ? (self_path[n] = '\0', self_path) : argv0;
    char *new_argv[8];
    int ai = 0;
    new_argv[ai++] = (char *) exe;
    new_argv[ai++] = "--control";
    new_argv[ai++] = "--tle";
    new_argv[ai++] = g_viewer_tle_path;
    if (g_viewer_pass_folder[0]) {
        new_argv[ai++] = "--pass-folder";
        new_argv[ai++] = g_viewer_pass_folder;
    }
    new_argv[ai] = NULL;
    fprintf(stderr,
        "simple_sat_ops: taking control with --tle %s%s%s\n",
        g_viewer_tle_path,
        g_viewer_pass_folder[0] ? "  --pass-folder " : "",
        g_viewer_pass_folder[0] ? g_viewer_pass_folder : "");
    execv(exe, new_argv);
    // If we got here exec failed — best to bail loudly.
    fprintf(stderr,
        "simple_sat_ops viewer: execv %s failed: %s\n",
        exe, strerror(errno));
    exit(EXIT_FAILURE);
}

static void viewer_render(int connected)
{
    int cols = COLS;
    erase();

    if (!g_viewer_has_state) {
        mvprintw(2, 2, "(waiting for state from the operator...)");
    } else {
        int row = 1, col = 1;
        render_predictions_panel(&g_viewer_state, g_viewer_jul_utc,
                                 &row, col);

        char viewers[160];
        viewer_roster_users(viewers, sizeof viewers);
        int srow = row + 1;
        status_panel_t sp;
        memset(&sp, 0, sizeof sp);
        sp.control_mode  = 0;
        sp.operator_user = g_viewer_operator;
        sp.viewers       = viewers[0] ? viewers : "(none)";
        sp.carrier_hz    = g_viewer_carrier_hz;
        sp.have_rotator  = g_viewer_has_rotator;
        sp.current_az    = g_viewer_state.antenna_rotator.azimuth;
        sp.current_el    = g_viewer_state.antenna_rotator.elevation;
        sp.target_az     = g_viewer_state.antenna_rotator.target_azimuth;
        sp.target_el     = g_viewer_state.antenna_rotator.target_elevation;
        sp.flip          = g_viewer_state.antenna_rotator.flip_mode_pass;
        render_status_panel(&sp, &srow, col);

        // Auto-tcmd run progress, mirrored from the operator's modal.
        // Red while the run is live (the PA is being keyed on a timer),
        // matching the T/R panel's red-while-transmitting convention.
        if (g_viewer_auto_on) {
            srow++;
            int at_running = strcmp(g_viewer_auto_state, "running") == 0;
            if (at_running) attron(COLOR_PAIR(1));
            mvprintw(srow++, col, "%15s   %d/%d sent (%s)",
                     "auto-tcmd",
                     g_viewer_auto_sent, g_viewer_auto_total,
                     g_viewer_auto_state[0] ? g_viewer_auto_state : "?");
            if (at_running) attroff(COLOR_PAIR(1));
            clrtoeol();
        }

        int prow = 5;
        report_position(&g_viewer_state, &prow, 50);
        // RX panel directly below position (matches the operator's layout).
        prow++;
        render_rx_panel(&g_viewer_rx_panel, &prow, 50);

        // Vertical ribbon on the right edge, same placement as the
        // operator. The wire delivers the same '.'/'-' chars the
        // operator's collector built so both screens crawl in sync.
        int ribbon_col = COLS - 2;
        int ribbon_top = 1;
        int ribbon_bot = LINES - 2;
        if (ribbon_col >= 64 && ribbon_bot > ribbon_top) {
            render_ribbon_vertical(&g_viewer_rx_panel,
                                   ribbon_top, ribbon_bot, ribbon_col);
        }

        int tx_log_row = LINES - TX_LOG_SIZE - 2;
        if (tx_log_row >= 17) {
            render_tx_log_panel(tx_log_row, 1);
        }
    }

    time_t now = time(NULL);
    long stale_s = g_viewer_last_event > 0
        ? (long)(now - g_viewer_last_event)
        : -1;
    const char *status = !connected ? "DISCONNECTED"
                                    : (stale_s < 0 ? "WAITING"
                                                   : (stale_s > 5 ? "STALE"
                                                                  : "LIVE"));

    // Bottom row priority: take-control confirm > cmd mirror > footer.
    // The cmd mirror reproduces exactly what the operator sees on its
    // own LINES-1; the footer (connection status + viewer-only shortcuts)
    // is the fallback when neither is active. The mirror does not invert
    // the row — matches the operator's plain ":" prompt rendering.
    int show_confirm = (g_viewer_confirm_until > 0
                        && now < g_viewer_confirm_until);
    int show_mirror  = !show_confirm
        && (g_viewer_cmd_active || g_viewer_cmd_status[0]);

    move(LINES - 1, 0);
    clrtoeol();
    if (show_mirror) {
        if (g_viewer_cmd_active) {
            mvprintw(LINES - 1, 0, ":%s", g_viewer_cmd_buf);
            addch(' ' | A_REVERSE);
        } else {
            mvprintw(LINES - 1, 0, "%s", g_viewer_cmd_status);
        }
    } else {
        attron(A_REVERSE);
        char foot[200];
        if (show_confirm) {
            snprintf(foot, sizeof foot,
                " %s     Take control from %s? y/N ",
                status,
                g_viewer_operator[0] ? g_viewer_operator : "?");
        } else {
            snprintf(foot, sizeof foot,
                " %s     c : take control   q : quit ", status);
        }
        int flen = (int)strlen(foot);
        if (flen > cols) flen = cols;
        mvaddnstr(LINES - 1, 0, foot, flen);
        for (int i = flen; i < cols; i++) mvaddch(LINES - 1, i, ' ');
        attroff(A_REVERSE);
    }

    refresh();
}

static int run_viewer(const char *argv0)
{
    sso_ipc_client_t *cli = sso_ipc_client_connect("simple_sat_ops");
    if (cli == NULL) {
        fprintf(stderr,
                "simple_sat_ops viewer: connect failed: %s\n",
                strerror(errno));
        return EXIT_FAILURE;
    }
    sso_ipc_client_on_event(cli, viewer_on_event, NULL);

    // Viewer doesn't run SGP4 or load a TLE — it deposits every
    // displayed value into g_viewer_state from the broadcast and uses
    // the same render helpers the operator does. zero-init is enough.
    memset(&g_viewer_state, 0, sizeof g_viewer_state);

    sso_event_t hello;
    sso_event_init(&hello, SSO_EVT_HELLO);
    snprintf(hello.role, sizeof hello.role, "viewer");
    const char *me = getenv("USER");
    if (!me || !me[0]) me = sso_unix_user();
    snprintf(hello.user, sizeof hello.user, "%s", me ? me : "?");
    char buf[1024];
    if (sso_event_encode(&hello, buf, sizeof buf) == 0) {
        sso_ipc_client_send(cli, buf);
    }

    init_window();
    int last_connected = -1;
    time_t last_render = 0;
    viewer_render(sso_ipc_client_is_connected(cli));
    last_render = time(NULL);
    int confirm_was_armed = 0;
    while (g_viewer_running) {
        int rc = sso_ipc_client_step(cli, 200);
        if (rc < 0) break;
        int connected = sso_ipc_client_is_connected(cli);
        time_t now = time(NULL);
        int confirm_armed = (g_viewer_confirm_until > 0
                             && now < g_viewer_confirm_until);
        if (!confirm_armed && confirm_was_armed) {
            // Window just expired — re-render to drop the confirm footer.
            g_viewer_event_pending = 1;
        }
        confirm_was_armed = confirm_armed;
        if (g_viewer_event_pending
            || connected != last_connected
            || (now - last_render) >= 5) {
            viewer_render(connected);
            g_viewer_event_pending = 0;
            last_connected = connected;
            last_render = now;
        }
        int key = getch();
        if (key == ERR) continue;
        if (confirm_armed) {
            if (key == 'y' || key == 'Y') {
                // Commits: viewer_take_control re-execs on success and
                // doesn't return; if it returns, the operator wasn't
                // reachable and we just stay as a viewer.
                viewer_take_control(cli, argv0);
                g_viewer_confirm_until = 0;
                g_viewer_event_pending = 1;
            } else {
                // Anything else cancels the confirm window.
                g_viewer_confirm_until = 0;
                g_viewer_event_pending = 1;
            }
            continue;
        }
        if (key == 'q' || key == 'Q' || key == 27 /* Esc */) {
            g_viewer_running = 0;
        } else if (key == 'c' || key == 'C') {
            g_viewer_confirm_until = now + VIEWER_CONFIRM_WINDOW_S;
            g_viewer_event_pending = 1;
        }
    }

    endwin();
    tui_release_stderr();
    sso_ipc_client_close(cli);
    return 0;
}

// --- --self-test report -------------------------------------------
//
// Prints the resolved configuration after CLI parse + HMAC keyfile
// load, in a stable key: value layout so test harnesses can grep it.
// Every line is "key: value" with no surrounding quoting; values are
// short enough to fit on one line. The "self-test:" header line is
// the contract — downstream scripts can use it as a sentinel.

static const char *hmac_status_str(hmac_display_status_t s)
{
    switch (s) {
        case HMAC_DISPLAY_OK:      return "ok";
        case HMAC_DISPLAY_MISSING: return "missing";
        case HMAC_DISPLAY_BAD:     return "bad";
        case HMAC_DISPLAY_UNSET:   /* fall through */
        default:                   return "unset";
    }
}

static const char *baud_str(int speed_const)
{
    // antenna_rotator stores the serial speed as the POSIX termios
    // constant (B600 etc), not the integer baud rate. Map the ones
    // the rotator actually uses; "?" everything else so a change to
    // antenna_rotator.c shows up in the report instead of crashing
    // it.
    switch (speed_const) {
        case B600:    return "600";
        case B1200:   return "1200";
        case B2400:   return "2400";
        case B4800:   return "4800";
        case B9600:   return "9600";
        case B19200:  return "19200";
        case B38400:  return "38400";
        case B57600:  return "57600";
        case B115200: return "115200";
        default:      return "?";
    }
}

static void self_test_report(const state_t *state, FILE *out, int argc, char **argv)
{
    fprintf(out, "self-test: simple_sat_ops configuration snapshot\n");
    fprintf(out, "version: %s\n", sso_version_string());

    // Echo the command line so the report is self-describing — the
    // reader can see at a glance which flags produced this snapshot.
    fprintf(out, "argv:");
    for (int i = 1; i < argc; ++i) {
        fprintf(out, " %s", argv[i]);
    }
    fprintf(out, "\n");

    // Mode. apply_args has already set g_control_mode / g_viewer_mode
    // (the latter only via the auto-probe path, which --self-test
    // skips). Standalone is the default.
    const char *mode = g_control_mode ? "operator (--control)"
                     : g_viewer_mode  ? "viewer (auto-detected)"
                                       : "standalone";
    fprintf(out, "mode: %s\n", mode);

#ifdef SSO_WITH_SDR
    int sdr_compiled = 1;
#else
    int sdr_compiled = 0;
#endif
#ifdef WITH_USRP_B210
    int uhd_compiled = 1;
#else
    int uhd_compiled = 0;
#endif
#ifdef WITH_RTL_SDR
    int rtl_compiled = 1;
#else
    int rtl_compiled = 0;
#endif
    fprintf(out, "build: sdr=%s (uhd=%s, rtl-sdr=%s)\n",
            sdr_compiled ? "on" : "off",
            uhd_compiled ? "on" : "off",
            rtl_compiled ? "on" : "off");

    fprintf(out, "tle: %s\n",
            state->prediction.tles_filename
                ? state->prediction.tles_filename
                : "(auto-discover at startup)");

    // HMAC --- the operator's banner-and-sign state. CTS1 firmware
    // expects every uplink to be HMAC-signed; the dispatcher refuses
    // to key the PA if g_hmac_key_len == 0, so this line is the
    // single most-important pre-flight check.
    fprintf(out,
            "hmac: %s (path=%s, status=%s, bytes=%zu)\n",
            g_hmac_key_len > 0 ? "enabled (default)" : "DISABLED",
            g_hmac_keyfile_path[0] ? g_hmac_keyfile_path : "(unresolved)",
            hmac_status_str(g_hmac_display_status),
            g_hmac_key_len);

    // Doppler --- both the display correction and the TX-side burst
    // staging key off state->doppler_correction_enabled. On by
    // default; --no-doppler-correction clears it. Report RX and TX
    // separately so the operator can see where the correction is
    // applied: RX is software (sw_nco on post-decim IQ, no hardware
    // LO retune mid-pass — the threshold-driven retune was removed
    // because it caused phase resets in the coherent demod), TX is
    // hardware (b210_rx_tx_core_burst tunes the B210 LO to the
    // Doppler-corrected frequency for every burst).
    fprintf(out, "doppler-correction: %s\n",
            state->doppler_correction_enabled ? "enabled (default)"
                                              : "DISABLED (--no-doppler-correction)");
    fprintf(out, "doppler-rx: %s (software sw_nco on post-decim IQ; hardware LO fixed)\n",
            state->doppler_correction_enabled ? "enabled" : "disabled");
    fprintf(out, "doppler-tx: %s (hardware SDR LO retune per burst, f=carrier/(1-rr/c))\n",
            (!sdr_compiled || g_without_b210)
                ? "n/a (no SDR)"
                : (state->doppler_correction_enabled ? "enabled" : "disabled"));
    fprintf(out, "uplink-nominal-mhz: %.6f\n",
            state->nominal_uplink_frequency_hz / 1e6);
    fprintf(out, "downlink-nominal-mhz: %.6f\n",
            state->nominal_downlink_frequency_hz / 1e6);
    fprintf(out, "rx-lo-offset-khz: %+.3f\n", state->rx_lo_offset_hz / 1000.0);

    // TX safety / staging gates the operator might have set.
    fprintf(out, "tx-no-tx: %s\n", g_no_tx ? "on (--no-tx)" : "off");
    fprintf(out, "tx-dry-run: %s\n", g_tx_dry_run ? "on (--tx-dry-run)" : "off");
    fprintf(out, "tx-auto-tcmd-file: %s\n",
            g_auto_tcmd_file_path[0] ? g_auto_tcmd_file_path : "(none)");

    // Hardware. The flags don't reflect "is it physically present" —
    // they reflect "does this run intend to talk to it". The actual
    // open happens after the self-test exit.
    fprintf(out, "rotator: %s (device=%s, baud=%s)\n",
            state->run_with_antenna_rotator ? "enabled"
                                            : "disabled (--without-rotator)",
            state->antenna_rotator.device_filename,
            baud_str(state->antenna_rotator.serial_speed));
    fprintf(out, "sdr: %s\n",
            (!sdr_compiled || g_without_b210)
                ? "disabled (--without-b210 or build-time)"
                : "enabled");

    fprintf(out, "live-waterfall: %s\n",
            g_run_live_waterfall ? "on (--live-waterfall)" : "off");

    fprintf(out, "pass-folder-seed: %s\n",
            g_pass_folder[0] ? g_pass_folder : "(auto)");

    // Observer location. apply_args stored these in radians on the
    // ephem struct — convert back to degrees for the report.
    fprintf(out, "observer-lat-deg: %.6f\n",
            state->prediction.observer_ephem.position_geodetic.lat * 180.0 / M_PI);
    fprintf(out, "observer-lon-deg: %.6f\n",
            state->prediction.observer_ephem.position_geodetic.lon * 180.0 / M_PI);
    fprintf(out, "observer-alt-m: %.1f\n",
            state->prediction.observer_ephem.position_geodetic.alt * 1000.0);

    fprintf(out, "self-test: ok\n");
    fflush(out);
}

// --- main ---------------------------------------------------------

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "simple_sat_ops")) return 0;
    state_t state = {0};
    state.prediction.predicted_max_elevation = -180.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);
    // --help / --help-full historically exited 2; preserve that. Every
    // parse-or-runtime failure inside apply_args collapses to exit 1
    // (the old code returned 1 for too-many-positionals / startup
    // errors and 3 for an unknown --option; both are now PARSE_ERROR ->
    // 1, each still printing its own distinct stderr message).
    switch (apply_args(&state, argc, argv, jul_utc, HELP_OFF)) {
        case PARSE_HELP:  return 2;
        case PARSE_ERROR: return 1;
    }

    // Bare invocation found a running operator — run as a read-only
    // viewer and skip the rest of the operator/standalone bring-up.
    if (g_viewer_mode) {
        return run_viewer(argv[0]);
    }

#ifdef SSO_WITH_SDR
    // Pin the SSO+ @tssent dedup key for this session: the startup UTC,
    // truncated to the minute. Constant for the life of the process so the
    // satellite runs an SSO+ time-sync once per pass. See sso_pseudo.h.
    g_sso_pass_tssent_ms = (sso_now_utc_ms() / 60000LL) * 60000LL;
#endif

    // Resolve + load the HMAC keyfile. The bytes feed every TX burst's
    // AX100 frame (CTS1 firmware expects HMAC on every uplink), AND
    // light the operator banner — "(N bytes ok)" means TX is armed,
    // "(MISSING)" / "(BAD)" means the next TX request will be refused
    // before keying the PA. If --hmac-keyfile= wasn't given, fall back
    // to hmac_keyfile_default_path (shared first, per-user second).
    if (g_hmac_keyfile_path[0] == '\0') {
        if (hmac_keyfile_default_path(g_hmac_keyfile_path,
                                      sizeof g_hmac_keyfile_path) != 0) {
            g_hmac_keyfile_path[0] = '\0';
            g_hmac_display_status  = HMAC_DISPLAY_MISSING;
        }
    }
    if (g_hmac_keyfile_path[0] != '\0') {
        struct stat st;
        if (stat(g_hmac_keyfile_path, &st) != 0) {
            g_hmac_display_status = HMAC_DISPLAY_MISSING;
        } else {
            ssize_t got = hmac_keyfile_load(g_hmac_keyfile_path,
                                            g_hmac_key,
                                            sizeof g_hmac_key);
            if (got > 0) {
                g_hmac_display_status = HMAC_DISPLAY_OK;
                g_hmac_key_len        = (size_t) got;
            } else {
                g_hmac_display_status = HMAC_DISPLAY_BAD;
                g_hmac_key_len        = 0;
                memset(g_hmac_key, 0, sizeof g_hmac_key);
            }
        }
    }

    // --self-test: configuration snapshot, then exit. Runs after CLI
    // parse + HMAC keyfile load so every TX-relevant policy is
    // resolved; runs BEFORE the IPC socket bind, the rotator open,
    // the B210 open, and load_tle, so the process makes no observable
    // changes to the rest of the system.
    if (g_self_test) {
        self_test_report(&state, stdout, argc, argv);
        return 0;
    }

    // Telecommand-agenda lint gate. When a --tc-file was given, lint it
    // against the firmware's telecommand set (names, argument counts,
    // CTS1+...! framing, length limits) BEFORE bringing up anything that
    // can key the PA. Lint errors mean a command would be rejected (or
    // worse, mis-parsed) by the satellite, so refuse to start unless the
    // operator explicitly accepts the risk. Warnings (e.g. a command not
    // meant for routine flight operation) are printed but do not block.
    if (g_auto_tcmd_file_path[0] != '\0') {
        int tc_warns = 0;
        int tc_errs = tcmd_lint_file(g_auto_tcmd_file_path, stderr, &tc_warns);
        if (tc_errs > 0 && !g_ignore_tc_errors) {
            fprintf(stderr,
                "simple_sat_ops: %d error%s detected in the --tc-file content (%s).\n"
                "Refusing to start. Fix the agenda, or re-run with\n"
                "--ignore-at-your-peril-all-tc-errors to bypass this check.\n",
                tc_errs, tc_errs == 1 ? "" : "s", g_auto_tcmd_file_path);
            return EXIT_FAILURE;
        }
        if (tc_errs > 0) {
            fprintf(stderr,
                "simple_sat_ops: %d telecommand error%s in %s -- proceeding anyway "
                "(--ignore-at-your-peril-all-tc-errors).\n",
                tc_errs, tc_errs == 1 ? "" : "s", g_auto_tcmd_file_path);
        } else if (tc_warns > 0) {
            fprintf(stderr,
                "simple_sat_ops: %d telecommand warning%s in %s (see above); proceeding.\n",
                tc_warns, tc_warns == 1 ? "" : "s", g_auto_tcmd_file_path);
        }
    }

    // Audit + operator IPC bring-up.
    g_operator_user = sso_unix_user();
    sso_audit_start("simple_sat_ops",
                    g_control_mode ? "operator" : "standalone");
    // Record the exact command line so post-incident review can tie
    // every operator action back to the flags the session was started
    // with (recording mode, --tx settings, TLE, etc.). One line, tab-
    // safe (sso_audit's sanitiser replaces tabs/newlines with spaces).
    {
        char argv_buf[1024];
        size_t off = 0;
        argv_buf[0] = '\0';
        for (int i = 0; i < argc && off + 2 < sizeof argv_buf; ++i) {
            int n = snprintf(argv_buf + off, sizeof argv_buf - off,
                             "%s%s", (i == 0) ? "" : " ", argv[i]);
            if (n <= 0) break;
            off += (size_t) n;
            if (off >= sizeof argv_buf) { off = sizeof argv_buf - 1; break; }
        }
        sso_audit_event("argv", argv_buf);
    }
    if (g_control_mode) {
        // Refuse if another simple_sat_ops --control is already
        // bound — two operators driving the same SDR / rotator is
        // exactly the failure mode the IPC server existed to avoid.
        // The probe connects as a transient viewer, reads the
        // operator's identity off the welcome reply, and disconnects.
        char existing_user[64]    = {0};
        char existing_folder[256] = {0};
        int op_status = sso_operator_verify("viewer",
                                             existing_folder,
                                             sizeof existing_folder,
                                             existing_user,
                                             sizeof existing_user);
        if (op_status == SSO_OP_OK || op_status == SSO_OP_MISMATCH) {
            pid_t op_pid = 0;
            const char *who = existing_user[0] ? existing_user : "?";
            if (read_operator_pid(&op_pid) == 0) {
                fprintf(stderr,
                    "simple_sat_ops: --control refused: operator already "
                    "running as user=%s pid=%d.\n"
                    "  To take over, run a viewer (no --control) and press\n"
                    "  'c' then 'y' to force-claim; the running operator\n"
                    "  will yield and your viewer will re-exec into --control.\n",
                    who, (int) op_pid);
            } else {
                fprintf(stderr,
                    "simple_sat_ops: --control refused: operator already "
                    "running as user=%s.\n", who);
            }
            char det[96];
            snprintf(det, sizeof det,
                     "existing_user=%s existing_pid=%d",
                     who, (int) op_pid);
            sso_audit_event("control-refused", det);
            return EXIT_FAILURE;
        }

        g_ipc = sso_ipc_server_open("simple_sat_ops");
        if (g_ipc == NULL) {
            // Probe said "no operator" yet bind still failed — most
            // likely a stale socket / pid file from a crashed
            // previous operator (or a vanishingly-rare race with
            // another --control starting at the same instant).
            // Either way, refuse so we don't quietly drive hardware
            // alongside something else.
            fprintf(stderr,
                "simple_sat_ops: --control: socket bind failed. If this is "
                "from a crashed previous operator, remove "
                "/run/sso/simple_sat_ops.{sock,pid} and retry.\n");
            sso_audit_event("ipc-bind-failed", "");
            return EXIT_FAILURE;
        }
        sso_ipc_server_on_event(g_ipc, ipc_on_event, NULL);
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = on_sigusr1;
        sigemptyset(&sa.sa_mask);
        sigaction(SIGUSR1, &sa, NULL);
        fprintf(stderr, "simple_sat_ops: operator=%s ipc=on\n",
                g_operator_user);
    }

    /* Parse TLE data */
    int tle_status = load_tle(&state.prediction);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.prediction.satellite_ephem.tle);

    // Seed the retarget guard with the startup TLE so a `:retarget` on
    // the same file is correctly a no-op.
    snprintf(g_target_tle_path, sizeof g_target_tle_path, "%s",
             state.prediction.tles_filename
                 ? state.prediction.tles_filename : "");

    // With a fresh TLE loaded, find the upcoming pass and stand up
    // /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/ for it before the
    // tracking loop opens ncurses. Only on --control — the
    // standalone-tracker / dev path leaves Operations/ alone.
    if (g_control_mode) {
        UTC_Calendar_Now(&utc, &tv);
        double jul_now = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_now);
        setup_pass_folder(&state, jul_now);
        if (g_pass_folder[0]) {
            generate_pass_plot(&state, g_pass_folder, jul_now);
        }
    }

    int antenna_rotator_result = 0;
    if (state.run_with_antenna_rotator) {
        state.antenna_rotator.is_required = 1;
        antenna_rotator_result = antenna_rotator_init(&state.antenna_rotator);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error initializing antenna rotator\n");
            return EXIT_FAILURE;
        }
        state.have_antenna_rotator = 1;
        // Spawn the async worker. From here on, every serial roundtrip
        // happens on the worker thread; the main loop only reads the
        // snapshot via main_rotator_refresh_targets_from_snapshot() and
        // posts SETs via main_rotator_submit_set().
        if (antenna_rotator_async_open(&g_rot_async,
                                        &state.antenna_rotator, 0.5) != 0) {
            fprintf(stderr, "Error spawning antenna rotator worker\n");
            return EXIT_FAILURE;
        }
        // Adopt whatever extended position the SPID is already at so the
        // unwrapped accumulator starts grounded in reality. We wait
        // briefly for the worker's first STATUS read; the timeout is
        // bounded so a missing controller doesn't hang startup.
        //
        // The seed snapshot also overwrites target_* with the current
        // physical position — fine when nobody asked for a specific park
        // position, but a problem when the operator passed
        // --rotator-target-azimuth / --rotator-target-elevation: those
        // user-specified targets would be silently clobbered before T
        // ever fired. Snapshot them and restore after seeding.
        double sav_az    = state.antenna_rotator.target_azimuth;
        double sav_el    = state.antenna_rotator.target_elevation;
        double sav_az_uw = state.antenna_rotator.target_azimuth_unwrapped;
        int    sav_uw_ok = state.antenna_rotator.unwrapped_target_valid;
        if (antenna_rotator_async_wait_first_status(g_rot_async, 1500) != 0
            || main_rotator_refresh_targets_from_snapshot(&state) != 0) {
            fprintf(stderr, "Warning: could not read SPID position; "
                            "check that the Rot2ProG is in 'A' mode\n");
        }
        if (state.antenna_rotator.fixed_target) {
            state.antenna_rotator.target_azimuth            = sav_az;
            state.antenna_rotator.target_elevation          = sav_el;
            state.antenna_rotator.target_azimuth_unwrapped  = sav_az_uw;
            state.antenna_rotator.unwrapped_target_valid    = sav_uw_ok;
        }
    }

    // --calibrate-rotator: drive the antenna across known arcs to
    // measure deg/s on each axis, save the result to disk, and exit
    // without entering the operator UI. Requires the safety interlock
    // so a stray flag in a script can't move hardware.
    if (g_calibrate_rotator) {
        if (!state.have_antenna_rotator) {
            fprintf(stderr, "--calibrate-rotator: no rotator open "
                            "(was --without-rotator passed?)\n");
            return EXIT_FAILURE;
        }
        if (!g_confirm_rotator_calibrate) {
            fprintf(stderr,
                    "--calibrate-rotator will physically move the antenna.\n"
                    "Confirm the mast area is clear, then re-run with\n"
                    "  --calibrate-rotator --confirm-rotator-calibrate\n");
            return EXIT_FAILURE;
        }
        double az_dps = 0.0, el_dps = 0.0;
        rotator_calibrate_result_t cres = rotator_calibrate_run(
            g_rot_async, &az_dps, &el_dps, stderr);
        fprintf(stderr, "calibrate: result = %s\n",
                rotator_calibrate_result_name(cres));
        if (cres == ROTATOR_CALIBRATE_OK) {
            fprintf(stderr,
                    "calibrate: saved rates az=%.3f deg/s el=%.3f deg/s\n",
                    az_dps, el_dps);
        }
        // Shutdown cleanly — the operator UI never started, but the
        // rotator FD and worker are open.
        if (g_rot_async != NULL) {
            antenna_rotator_async_close(g_rot_async);
            g_rot_async = NULL;
        }
        if (state.have_antenna_rotator) {
            antenna_rotator_disconnect(&state.antenna_rotator);
            state.have_antenna_rotator = 0;
        }
        return (cres == ROTATOR_CALIBRATE_OK) ? 0 : 1;
    }

    // Normal startup: load saved rotator rates from the calibration
    // file. Missing or malformed file -> pursuit planner stays
    // disabled (Phase 2 hooks this in front of the track loop; Phase
    // 1 just loads + warns so the bench can see the values).
    if (state.have_antenna_rotator) {
        if (pursuit_load_rotator_rates(&g_pursuit_az_dps,
                                        &g_pursuit_el_dps) == 0) {
            fprintf(stderr,
                    "pursuit: loaded slew rates az=%.3f deg/s el=%.3f deg/s\n",
                    g_pursuit_az_dps, g_pursuit_el_dps);
        } else {
            fprintf(stderr,
                    "pursuit: no calibration on disk; run "
                    "`simple_sat_ops --calibrate-rotator "
                    "--confirm-rotator-calibrate` to enable lead-aim\n");
        }
    }

    // T/R antenna switch — auto-probe before ncurses takes the screen,
    // so a "not connected" warning lands on the terminal. Absent or
    // inaccessible hardware is non-fatal; the UI panel reads "not
    // connected" and the program runs normally.
    if (state.run_with_tr_switch) {
        if (tr_switch_init(&state.tr_switch) == 0) {
            state.have_tr_switch = 1;
        } else {
            fprintf(stderr,
                    "T/R switch: could not open %s (skipping; "
                    "pass --without-tr-switch to silence)\n",
                    state.tr_switch.device_filename
                        ? state.tr_switch.device_filename : "?");
        }
    }

#ifdef SSO_WITH_SDR
    // Open the B210 once, here, before ncurses init — soft-fail on any
    // UHD error so a dev host without a device can still run the UI.
    // rx_session takes ownership of the core; we drop our local handle
    // afterwards so main never touches UHD off-thread.
    if (g_control_mode && !g_without_b210) {
        // B210 RX rate doubled from the original 240 kHz / sps=5 to
        // 480 kHz / sps=10 (after the integer-5 decimation FIR). That
        // gives the modem_fsk clock-recovery loop the same oversampling
        // headroom the gr-satellites / AIT chain has (sps=6 with PFB-
        // Gardner) and then some, which is worth ~1-2 dB at marginal
        // SNR on real captures. The post-decim signal still only
        // carries the FrontierSat ±10 kHz FSK, so the decim FIR
        // cutoff stays at 18 kHz — narrower than the new 48 kHz
        // Nyquist, so the filter rejects more noise than it did at
        // the old 24 kHz Nyquist. IQ files double in size; with the
        // sustained-write rate at 96 kHz·2·2 = 384 kB/s, a 10-minute
        // pass produces ~230 MB which the laptop SSD has no trouble
        // with.
        b210_rx_tx_core_params_t cp = {
            // Tune the SDR LO off the nominal carrier so the corrected
            // signal lands well off DC. rx_lo_offset_hz is SIGNED:
            // positive → LO above nominal (signal at negative baseband),
            // negative → LO below (signal at positive baseband). Default
            // -25 kHz keeps existing pipelines unchanged; operator can
            // shift to dodge fixed-pattern noise.
            .freq_hz         = state.nominal_downlink_frequency_hz
                             + state.rx_lo_offset_hz,
            .rate_hz         = 480000.0,
            .gain_db         = state.rx_gain_db,
            .bw_hz           = -1.0,
            .fm_fullscale_hz = 25000.0,
            .rx_antenna      = "RX2",
            // fir_decim budget:
            //   - operator LO offset clamped ±45 kHz (apps/main.c
            //     KEY_LO_OFFSET clamp);
            //   - Doppler swing ±10 kHz for a typical LEO pass;
            //   - FM envelope ±5 kHz around the carrier;
            //   - Nyquist after decim by 5 = ±48 kHz.
            // Cutoff 42 kHz with 256 taps gives ~6 kHz transition
            // before Nyquist and lets the carrier sit anywhere
            // inside the clamp without the LPF rolling off half the
            // beacon. The carrier-at-DC convention moved to the
            // decode-only buffer (see b210_rx_tx_core.c); the IQ
            // tap now carries the carrier at +lo_offset baseband.
            .decim_factor    = 5u,
            .decim_cutoff_hz = 42000.0,
            .decim_taps      = 256u,
            // FM-path LO compensation: the core's second NCO cancels
            // the operator's lo_offset, the UHD-reported tune residual
            // (target − actual, from the AD9361 PLL step), AND the
            // persistent per-host carrier trim (TCXO calibration). The
            // carrier lands at exactly DC for every downstream consumer
            // (.iq sidecar, live waterfall, shadow IQ decoder, FM
            // discriminator).
            .rx_dc_offset_track  = state.rx_dc_offset_track,
            .rx_iq_balance_track = state.rx_iq_balance_track,
            .fm_lo_compensation_hz = state.rx_lo_offset_hz,
            .carrier_trim_hz       = carrier_trim_load_hz(),
            // SDR backend selection: type (default auto), and the UHD
            // clone overrides. --sdr-device routes to the UHD device
            // args when given (e.g. "serial=..."); --uhd-args takes
            // precedence and is passed verbatim.
            .backend_type        = g_sdr_type,
            .device_args         = g_sdr_device[0] ? g_sdr_device : "type=b200",
            .uhd_args_override   = g_uhd_args[0] ? g_uhd_args : NULL,
            .fpga_image_path     = g_sdr_fpga[0] ? g_sdr_fpga : NULL,
            // RTL-SDR dongle index (UHD ignores it; for UHD use --uhd-args).
            .device_index        = g_sdr_device[0] ? atoi(g_sdr_device) : 0,
        };
        b210_rx_tx_core_t *core = NULL;
        if (b210_rx_tx_core_open(&cp, &core) != 0) {
            fprintf(stderr,
                "simple_sat_ops: B210 open failed — continuing without RF "
                "(rotator + UI only). Pass --without-b210 to silence.\n");
            sso_audit_event("b210-open-failed", "");
        } else {
            fprintf(stderr,
                "simple_sat_ops: SDR open at %.6f MHz (post-decim rate %.0f, "
                "tx=%s)\n",
                b210_rx_tx_core_actual_freq(core) / 1e6,
                b210_rx_tx_core_actual_rate(core),
                b210_rx_tx_core_can_tx(core) ? "yes" : "no (RX-only)");
            {
                char det[256];
                snprintf(det, sizeof det,
                    "freq_hz=%.0f rate_hz=%.0f lo_offset_hz=%.0f",
                    b210_rx_tx_core_actual_freq(core),
                    b210_rx_tx_core_actual_rate(core),
                    state.rx_lo_offset_hz);
                sso_audit_event("b210-open", det);
            }
            rx_session_params_t rxp = {
                .bit_rate          = 9600,
                .window_s          = 1.5,
                .slide_s           = 0.5,
                .sync_max_ham      = 4,
                .use_hmac          = 0,
                .use_rs            = 1,
                .force_beacon      = 0,
                .show_packet_headers = 0,
                .csp_crc32         = 0,
                .pass_folder       = g_pass_folder[0] ? g_pass_folder : NULL,
                .want_wav          = 1,
                .tle_path          = state.prediction.tles_filename,
                .sat_name          = state.prediction.satellite_ephem.tle.sat_name[0]
                                     ? state.prediction.satellite_ephem.tle.sat_name
                                     : NULL,
                .session_dir       = g_pass_folder[0] ? g_pass_folder : NULL,
                .lo_offset_hz      = state.rx_lo_offset_hz,
            };
            if (rx_session_open(&g_rx_session, &rxp, core) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: rx_session_open failed — closing B210\n");
                b210_rx_tx_core_close(core);
            }
            // rx_session_open succeeded → it owns `core` now.
            core = NULL;
            // --always-record: start WAV + .iq + sidecars right now,
            // before any pass logic gets a chance to gate them. The
            // per-pass start/stop block in the tracking loop checks
            // g_always_record and skips itself when this is on.
            if (g_always_record && g_rx_session) {
                rx_session_request_wav_start(g_rx_session);
                fprintf(stderr,
                    "simple_sat_ops: --always-record on — WAV/IQ "
                    "capture started, pass gating disabled\n");
                sso_audit_event("rec-start", "trigger=always-record");
            }
        }
    }
#endif

    /* Tracking loop */
    double jul_idle_start = 0;  // last-tracked timestamp

    // Capture the cooked terminal modes BEFORE ncurses switches the tty
    // to raw, so the crash handler can put it back deterministically.
    if (tcgetattr(STDIN_FILENO, &g_saved_termios) == 0) {
        g_have_saved_termios = 1;
    }

    init_window();

    // Catch a fatal device fault (or Ctrl-C) now that the screen is up,
    // so it restores the terminal instead of dumping a raw abort on it.
    install_signal_handlers();

    // int (not char) — getch returns KEY_* codes well above 127 for
    // arrow keys / function keys / KEY_BACKSPACE etc. A signed char
    // would silently truncate those into bogus low-byte values, which
    // is what made KEY_LEFT (260) look like Ctrl-D (4) in the modal
    // handlers and broke field editing inside the auto-tcmd modal.
    int key = ERR;
    int row = 0;
    int col = 2;
    state.running = 1;

    double current_uplink_frequency = state.nominal_uplink_frequency_hz;
    double current_downlink_frequency = state.nominal_downlink_frequency_hz;
    double doppler_delta_uplink = 0.0;
    double doppler_delta_downlink = 0.0;
    double doppler_max_delta = DOPPLER_SHIFT_RESOLUTION_KHZ * 1000.0;
    (void) doppler_delta_uplink;  // tracked for display symmetry / future IPC
    (void) doppler_max_delta;     // threshold for any future on-display retune

    double delta_az = 0.0;
    double delta_el = 0.0;

    state.antenna_rotator.antenna_should_be_controlled =
        state.run_with_antenna_rotator && state.have_antenna_rotator;
    state.antenna_rotator.antenna_is_under_control =
        state.antenna_rotator.antenna_should_be_controlled;

    int keyboard_unlocked = 1;
    int keyboard_info_row = 20;

    mvprintw(keyboard_info_row++, 3, "%s", "T  - Track satellite");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "s  - Stop antenna immediately");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "r  - Reset to az=0 el=0");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "[/]- Jog azimuth -5/+5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "{/}- Jog azimuth -1/+1 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", ",/.- Jog elevation -5/+5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "</>- Jog elevation -1/+1 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "t  - Compose TX command");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "A  - Auto-TCMD (needs --tc-file=)");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "K  - Lock/unlock keyboard");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "q  - Quit");
    clrtoeol();

    double current_az = 0;
    double current_el = 0;
    double last_az = 0;
    double last_el = 0;

    // Slow-cadence work is timestamp-gated so the fast UHD-pump loop
    // doesn't spam viewers or burn CPU on ncurses redraws.
    //
    // Two redraw gates: the "slow" one drives the predictions / status
    // / RX panel / TX log rows and is the one that costs CPU — most
    // notably report_status, which does a blocking read() against the
    // rotator serial port (antenna_rotator.c:142). Keep it at 2 Hz so
    // the rotator isn't hammered. The "fast" path is cmd_render only
    // — runs every loop tick while the operator is typing in the ":"
    // prompt so each keystroke echoes immediately.
    double t_last_ipc_broadcast = 0.0;
    double t_last_redraw        = 0.0;
    const double IPC_BROADCAST_PERIOD_S = 0.5;   // 2 Hz
    const double REDRAW_PERIOD_S        = 0.5;   // 2 Hz

    // Per-pass WAV recording: arm 1 min before AOS, hold open through
    // the pass, close 1 min after LOS. Multiple passes during one
    // simple_sat_ops run each get their own file under the pass folder.
    // All three are consumed only inside #ifdef WITH_USRP_B210; the
    // attribute keeps gcc-15 quiet on the non-B210 dev build.
    __attribute__((unused)) const double RECORDING_PREROLL_S  = 60.0;
    __attribute__((unused)) const double RECORDING_POSTROLL_S = 60.0;
    __attribute__((unused)) double t_recording_close_at = 0.0;

    while (state.running) {
        // Ctrl-C / SIGTERM: leave the loop and run the normal teardown
        // (endwin, rotator home, device close) instead of dying raw.
        if (g_signal_quit) { state.running = 0; break; }
        double t_now = monotonic_seconds();
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_utc);

        // Drain whatever the T/R switch emitted since the last tick.
        // Non-blocking; the firmware beats every ~2.5 s so most ticks
        // read zero bytes.
        if (state.have_tr_switch) {
            tr_switch_pump(&state.tr_switch, t_now);
        }

        /* Calculate Doppler shift for the display + IPC publishing */
        if (state.doppler_correction_enabled) {
            update_doppler_shifted_frequencies(&state,
                                                state.nominal_uplink_frequency_hz,
                                                state.nominal_downlink_frequency_hz);
            doppler_delta_uplink   = fabs(state.doppler_uplink_frequency_hz
                                           - current_uplink_frequency);
            doppler_delta_downlink = fabs(state.doppler_downlink_frequency_hz
                                           - current_downlink_frequency);
            current_uplink_frequency   = state.doppler_uplink_frequency_hz;
            current_downlink_frequency = state.doppler_downlink_frequency_hz;
        }

#ifdef SSO_WITH_SDR
        // TX-side Doppler: transmit at the frequency that places the
        // nominal carrier at the satellite. Sign: range_rate_km_s > 0
        // when the satellite is receding (LOS end of a pass), so the
        // ground must transmit higher to compensate for redshift at
        // the moving receiver. Mirror of the RX-side correction,
        // applied to FRONTIERSAT_CARRIER_HZ (the actual TX carrier) —
        // state.doppler_uplink_frequency_hz is computed from the 2 m
        // amateur nominal and would give the wrong absolute frequency
        // here. Off when doppler_correction_enabled is false (e.g.
        // bench loopback) so RX and TX share one constant carrier.
        g_tx_freq_hz_doppler = tx_burst_doppler_freq_hz(
            FRONTIERSAT_CARRIER_HZ,
            state.prediction.satellite_ephem.range_rate_km_s,
            state.doppler_correction_enabled);
#endif

#ifdef SSO_WITH_SDR
        // Auto-record per pass: open the WAV 1 min before AOS (or as
        // soon as we're above the horizon, in case simple_sat_ops
        // started mid-pass), keep it open while the satellite is up,
        // close 1 min after LOS. Each pass gets its own auto-named
        // file in the pass folder. Note: this deliberately keys off the
        // satellite geometry (elevation + time-until-AOS) rather than
        // state.in_pass — the latter flips several minutes before AOS
        // (tracking_prep_time_minutes) so the rotator can pre-position,
        // which is far too early to start the WAV.
        //
        // --always-record disables this gate entirely: recording was
        // started once at rx_session_open and stays open until shutdown.
        if (g_rx_session && !g_always_record) {
            double sec_to_aos =
                state.prediction.predicted_minutes_until_visible * 60.0;
            int visible   = (state.prediction.satellite_ephem.elevation > 0.0);
            int in_preroll = (sec_to_aos > 0.0
                              && sec_to_aos <= RECORDING_PREROLL_S);
            int active = rx_session_wav_active(g_rx_session);
            if (!active && (visible || in_preroll)) {
                rx_session_request_wav_start(g_rx_session);
                t_recording_close_at = 0.0;
                char det[64];
                snprintf(det, sizeof det,
                    "trigger=%s sec_to_aos=%.1f el=%.1f",
                    visible ? "elevation" : "preroll",
                    sec_to_aos,
                    state.prediction.satellite_ephem.elevation);
                sso_audit_event("rec-start", det);
            } else if (active) {
                if (visible) {
                    t_recording_close_at = 0.0;  // cancel any pending close
                } else if (t_recording_close_at == 0.0) {
                    t_recording_close_at = t_now + RECORDING_POSTROLL_S;
                } else if (t_now >= t_recording_close_at) {
                    rx_session_request_wav_stop(g_rx_session);
                    t_recording_close_at = 0.0;
                    sso_audit_event("rec-stop", "trigger=postroll-expired");
                }
            }
        }
#endif

        current_az = state.antenna_rotator.azimuth;
        current_el = state.antenna_rotator.elevation;
        if (state.antenna_rotator.antenna_is_moving) {
            if (fabs(current_az - last_az) == 0
                && fabs(current_el - last_el) == 0) {
                state.antenna_rotator.antenna_is_moving = 0;
            }
            last_az = current_az;
            last_el = current_el;
        }

        // Drive the second leg of a two-step home. The first leg drops a mid
        // waypoint to start the antenna unwinding; the final 'go to target'
        // must wait until the antenna has unwound far enough that the
        // controller's SHORT path to the target runs the SAME way as the
        // unwind -- i.e. it has reached the 0..180 zone on the unwinding side.
        // Until then the short path is the opposite (winding) way, so issuing
        // the target now sends it back around and it winds up (330 -> 360).
        //
        // Complication: after a SET the controller's STATUS reports the
        // just-commanded target (the mid waypoint) for a couple of seconds
        // before its feedback shows real motion. So a reading that still
        // equals the commanded mid is that echo, not the real position --
        // ignore it. The first reading that DIFFERS is the antenna's true
        // position; act on that. Mid-slew the real position is far from the
        // mid waypoint, so there's no echo-vs-arrival ambiguity. (Unwinds
        // past a full turn, prev > 360, would need more than one waypoint;
        // a single pass winds < 360, so one mid waypoint suffices.)
        if (state.antenna_rotator.homing_in_progress
            && state.have_antenna_rotator) {
            double final_az  = state.antenna_rotator.home_pending_final_az;
            double mid_az    = state.antenna_rotator.target_azimuth_unwrapped;
            double from_mid  = fabs(antenna_rotator_wrap_to_pm180(current_az - mid_az));
            double unwind    = final_az - mid_az;   // sign = unwind direction
            double remaining = antenna_rotator_wrap_to_pm180(final_az - current_az);
            int in_zone = (remaining == 0.0)
                       || ((remaining > 0.0) == (unwind > 0.0));
            // from_mid > tol => the reading is real feedback, not the post-SET
            // target echo. The two-step always starts out of the unwind zone
            // (|prev| > 180), so the stale pre-SET reading can't fire early.
            if (from_mid > HOME_ECHO_TOLERANCE_DEG && in_zone) {
                int rc = main_rotator_submit_set(&state, final_az, 0.0);
                if (rc == ANTENNA_ROTATOR_OK) {
                    state.antenna_rotator.antenna_is_moving = 1;
                }
                state.antenna_rotator.homing_in_progress = 0;
                state.antenna_rotator.home_pending_final_az = 0.0;
                char det[96];
                snprintf(det, sizeof det, "leg2 fired at az=%.1f -> %.1f",
                         current_az, final_az);
                sso_audit_event("home", det);
            }
        }
        // --scan-sky: drives a sky grid one target at a time, dwelling
        // SCAN_DWELL_S at each. Bypasses the satellite_tracking +
        // pass-timing gate below entirely, so the operator can scan
        // regardless of TLE / pass state. 's' stops mid-scan.
        if (g_scan_active) {
            scan_sky_tick(&state, t_now);
        }
        if (state.satellite_tracking
            && state.prediction.predicted_minutes_until_visible
                   < state.antenna_rotator.tracking_prep_time_minutes) {
            if (!state.in_pass) {
                state.in_pass = 1;
            }
            if (state.antenna_rotator.antenna_should_be_controlled
                && !state.antenna_rotator.tracking) {
                if (!state.antenna_rotator.fixed_target
                    && !state.antenna_rotator.flip_decision_made) {
                    state.antenna_rotator.flip_mode_pass = 0;
                    state.antenna_rotator.flip_half = 0;
                    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION > 90
                        && state.prediction.predicted_max_elevation
                               >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD) {
                        state.antenna_rotator.flip_mode_pass = 1;
                        // Prefer the prediction-derived AOS azimuth (the
                        // satellite_ephem.azimuth here may be a few deg
                        // off as we are still pre-AOS); fall back to the
                        // live position if the pass walk didn't capture
                        // an ascension sample.
                        double aos_az_pred =
                            state.prediction.predicted_ascension_azimuth;
                        state.antenna_rotator.flip_aos_az =
                            (aos_az_pred != 0.0)
                                ? aos_az_pred
                                : state.prediction.satellite_ephem.azimuth;
                        state.antenna_rotator.flip_los_az =
                            state.prediction.predicted_descent_azimuth;
                        state.antenna_rotator.flip_aos_jul =
                            state.prediction.predicted_ascension_jul_utc;
                        state.antenna_rotator.flip_los_jul =
                            state.prediction.predicted_descent_jul_utc;
                    }
                    state.antenna_rotator.flip_decision_made = 1;
                    state.antenna_rotator.tracking = 1;
                    // Pre-sample the trajectory and ask the planner
                    // for a rate-feasible whole-pass aim sequence. On
                    // any failure (no calibration, planner unhappy,
                    // --without-rotator-pursuit) the helper leaves
                    // g_pursuit_plan zero and the track loop below
                    // falls back to today's aim-where-sat-is-now path.
                    main_pursuit_build_plan(&state, jul_utc);
                }
            }

            if (state.antenna_rotator.tracking
                && state.antenna_rotator.antenna_is_under_control) {
                if (!state.antenna_rotator.unwrapped_target_valid) {
                    if (main_rotator_refresh_targets_from_snapshot(&state)
                        != 0) {
                        state.antenna_rotator.tracking = 0;
                        main_pursuit_clear_plan();
                    }
                } else if (!state.antenna_rotator.antenna_is_moving) {
                    double next_az = 0.0, next_el = 0.0;
                    double prev_unwrapped =
                        state.antenna_rotator.target_azimuth_unwrapped;
                    int    used_pursuit = 0;
                    if (g_pursuit_plan.waypoints != NULL
                        && pursuit_aim_at(&g_pursuit_plan, jul_utc,
                                          &next_az, &next_el) == 0) {
                        used_pursuit = 1;
                    }
                    if (!used_pursuit) {
                        double pred_az =
                            state.prediction.satellite_ephem.azimuth;
                        double pred_el =
                            state.prediction.satellite_ephem.elevation;
                        double mech_az = pred_az;
                        double mech_el = pred_el;
                        int half = 0;
                        // AOS->LOS progress: drives the boom-meridian
                        // lerp in flip mode. Clamped to [0, 1] inside
                        // the function. Ignored for non-flip passes.
                        double progress = 0.0;
                        double pass_jul =
                            state.antenna_rotator.flip_los_jul
                            - state.antenna_rotator.flip_aos_jul;
                        if (pass_jul > 0.0) {
                            progress = (jul_utc
                                        - state.antenna_rotator.flip_aos_jul)
                                       / pass_jul;
                        }
                        antenna_rotator_to_mech_coords(
                            state.antenna_rotator.flip_mode_pass,
                            state.antenna_rotator.flip_aos_az,
                            state.antenna_rotator.flip_los_az,
                            progress,
                            pred_az, pred_el,
                            &mech_az, &mech_el, &half);
                        if (state.antenna_rotator.flip_mode_pass
                            && half != state.antenna_rotator.flip_half) {
                            state.antenna_rotator.target_azimuth_unwrapped =
                                mech_az;
                            state.antenna_rotator.flip_half = half;
                            prev_unwrapped =
                                state.antenna_rotator.target_azimuth_unwrapped;
                        }
                        next_az = antenna_rotator_accumulate_unwrapped(
                            prev_unwrapped, mech_az);
                        next_el = mech_el;
                    }
                    if (next_el < ANTENNA_ROTATOR_MINIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MINIMUM_ELEVATION;
                    } else if (next_el > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                    }
                    delta_az = next_az - prev_unwrapped;
                    // With a plan in play the elevation is part of the
                    // trajectory; respect it even below the horizon.
                    // The pre-pursuit fallback keeps the existing
                    // "only chase el while the sat is visible" rule.
                    if (used_pursuit
                        || state.antenna_rotator.flip_mode_pass
                        || state.prediction.satellite_ephem.elevation >= 0) {
                        delta_el = next_el
                                   - state.antenna_rotator.target_elevation;
                    } else {
                        delta_el = 0.0;
                    }

                    if (fabs(delta_az) >= MAX_DELTA_AZIMUTH_DEGREES
                        || fabs(delta_el) >= MAX_DELTA_ELEVATION_DEGREES) {
                        if (next_az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH
                            || next_az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                            state.antenna_rotator.tracking = 0;
                            main_pursuit_clear_plan();
                        } else {
                            int rc = main_rotator_submit_set(
                                &state, next_az, next_el);
                            if (rc != ANTENNA_ROTATOR_OK) {
                                fprintf(stderr,
                                        "Error setting antenna rotator position\n");
                            } else {
                                state.antenna_rotator.antenna_is_moving = 1;
                            }
                        }
                    }
                }
            }

            jul_idle_start = 0;
        } else {
            if (state.in_pass) {
                state.in_pass = 0;
                jul_idle_start = jul_utc;
            }
            if (state.antenna_rotator.tracking) {
                state.antenna_rotator.tracking = 0;
                state.antenna_rotator.flip_mode_pass = 0;
                state.antenna_rotator.flip_decision_made = 0;
                state.antenna_rotator.flip_half = 0;
                // Released the pass; tear down the planner so the
                // memory comes back and so the next pass / mid-pass
                // 'T' rebuilds against fresh state.
                main_pursuit_clear_plan();
            }
        }
        (void) jul_idle_start;  // reserved for any future idle-window behavior

        int redraw_due = (t_now - t_last_redraw) >= REDRAW_PERIOD_S;
        if (redraw_due) {
            row = 1;
            col = 1;
            report_predictions(&state, jul_utc, &row, col);

            row++;
            report_status(&state, &row, col);
            row = 5;
            col = 50;
            report_position(&state, &row, col);
            row++;
            // Refresh the low-disk warning lazily — statvfs every 30 s
            // is plenty given how slowly disk fills.
            low_disk_refresh(t_now);
            rx_panel_data_t rxd;
            rx_panel_collect_local(&rxd);
            render_rx_panel(&rxd, &row, 50);

            clrtoeol();

            // Vertical ribbon on the right edge — bottom = newest, with
            // a bold '-' tick crawling up one row per second so the
            // timeline is visibly alive even when the signal is flat.
            int ribbon_col = COLS - 2;
            int ribbon_top = 1;
            int ribbon_bot = LINES - 2;
            if (ribbon_col >= 64 && ribbon_bot > ribbon_top) {
                render_ribbon_vertical(&rxd, ribbon_top, ribbon_bot, ribbon_col);
            }

            // TX log lives below the keyboard info / antenna status if
            // the terminal is tall enough to host it without colliding.
            int tx_log_row = LINES - TX_LOG_SIZE - 2;
            if (tx_log_row >= keyboard_info_row + 4) {
                render_tx_log_panel(tx_log_row, 1);
            }
        }

        key = getch();
        if (g_tx_compose_active) {
            if (!tx_compose_handle_key(key)) {
                tx_compose_close();
            }
        } else if (g_auto_tcmd_active) {
            if (!auto_tcmd_handle_key(key)) {
                auto_tcmd_close();
            }
        } else if (g_cmd_active) {
            cmd_handle_key(key, &state);
        } else if (key == 'K') {
            keyboard_unlocked = !keyboard_unlocked;
        } else if (keyboard_unlocked) {
            switch (key) {
                case ':':
                    cmd_enter();
                    break;
                case 'q':
                    state.running = 0;
                    break;
                case 'T':
                    if (g_scan_sky_mode) {
                        scan_sky_start(&state);
                    } else {
                        start_tracking(&state);
                        if (state.antenna_rotator.fixed_target) {
                            char det[128];
                            snprintf(det, sizeof det,
                                "mode=fixed-target az=%.1f el=%.1f",
                                state.antenna_rotator.target_azimuth,
                                state.antenna_rotator.target_elevation);
                            sso_audit_event("track-on", det);
                        } else {
                            sso_audit_event("track-on",
                                state.prediction.satellite_ephem.tle.sat_name[0]
                                    ? state.prediction.satellite_ephem.tle.sat_name : "");
                        }
                    }
                    break;
                case 's':
                    if (g_scan_active) {
                        scan_sky_stop(&state, "user");
                    }
                    stop_tracking(&state);
                    break;
                case 'r':
                    stop_tracking(&state);
                    point_to_stationary_target(&state, 0.0, 0.0);
                    break;
                case '[':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ']':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '{':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, -1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '}':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, 1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ',':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '.':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '<':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, -1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '>':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, 1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case 't':
                    tx_compose_open();
                    break;
                case 'A':
                    auto_tcmd_open();
                    break;
                default:
                    break;
            }
        }

        if (redraw_due) {
            // Width-padded prints (not clrtoeol) so we don't wipe the
            // signal ribbon that paints over the right edge of these rows.
            mvprintw(keyboard_info_row, 3, "%s : %-8s", "Keyboard",
                     keyboard_unlocked ? "unlocked" : "LOCKED");
            mvprintw(keyboard_info_row + 2, 0, "%-18s",
                     state.antenna_rotator.antenna_is_moving
                         ? "Antenna moving"
                         : "Antenna stationary");
            t_last_redraw = t_now;
        }

        // Pump the modal's debounced preview broadcast before the
        // screen flush so the mirror line is current when we paint.
        tx_compose_pump();
        // Drive the auto-tcmd burst loop. Queues g_tx_request when
        // it's time for the next send; the existing main-loop burst
        // handler below transmits and emits the SENT/NOT_SENT events.
        auto_tcmd_tick(&state);

        // Bottom-row prompt + screen flush. When the operator is typing
        // in the ":" prompt we want this every tick (~50 Hz) so each
        // keystroke echoes immediately. Otherwise piggyback on the slow
        // redraw so the row picks up any post-command status string.
        // When a modal is open we force-redraw it on top of stdscr by
        // touchwin'ing every modal cell as dirty and wrefresh'ing the
        // window after stdscr's own refresh. doupdate's incremental
        // diff is otherwise free to skip "unchanged" modal cells, which
        // is what was letting panel updates (e.g. the antenna status
        // row) bleed through and overwrite the modal.
        if (redraw_due || g_cmd_active || g_tx_compose_active
            || g_auto_tcmd_active) {
            cmd_render();
            refresh();
            int show_hw_cursor = 0;
            if (g_tx_compose_active && g_tx_compose_win) {
                touchwin(g_tx_compose_win);
                wrefresh(g_tx_compose_win);
                tx_field_t f = g_tx_compose_state.focus;
                show_hw_cursor = (f == TXF_PAYLOAD || f == TXF_POWER);
            } else if (g_auto_tcmd_active && g_auto_tcmd_win) {
                touchwin(g_auto_tcmd_win);
                wrefresh(g_auto_tcmd_win);
                show_hw_cursor = (g_auto_tcmd.state != AUTO_STATE_RUNNING)
                              && auto_field_is_text(g_auto_tcmd.focus);
            } else if (g_cmd_active) {
                show_hw_cursor = 1;
            }
            curs_set(show_hw_cursor ? 1 : 0);
        }

#ifdef SSO_WITH_SDR
        // Signal ribbon sampler: push one peak-dBFS reading per second
        // so the ribbon in the RX panel rolls left in real time. Also
        // grab the iq_burst bright-bin count so the renderer can pick
        // a character that distinguishes broadband packets from a CW
        // carrier at the same peak level.
        if (g_rx_session && (t_now - g_ribbon_last_t) >= 1.0) {
            double peak = -90.0;
            rx_session_snapshot(g_rx_session, NULL, &peak, NULL,
                                NULL, NULL, 0);
            int burst_bins = 0;
            rx_session_burst_snapshot(g_rx_session, &burst_bins, NULL);
            ribbon_push(peak, burst_bins);
            g_ribbon_last_t = t_now;

            // Live waterfall: launch the raylib viewer the first time
            // a recording's .iq path appears, OR if the pass switched
            // to a new path. We poll once per second on the same
            // cadence as the ribbon — cheap, and a single second of
            // lag at viewer-launch is invisible to the operator.
            if (g_run_live_waterfall) {
                char iq_path[512] = "";
                int  iq_rate      = 0;
                rx_session_iq_snapshot(g_rx_session,
                                       iq_path, sizeof iq_path,
                                       NULL, &iq_rate);
                if (iq_path[0]
                    && strcmp(iq_path, g_live_waterfall_iq) != 0) {
                    // Tear down a viewer pointed at a stale path.
                    if (g_live_waterfall_pid > 0) {
                        kill(g_live_waterfall_pid, SIGTERM);
                        waitpid(g_live_waterfall_pid, NULL, 0);
                        g_live_waterfall_pid = -1;
                    }
                    if (g_live_waterfall_stdin_fd >= 0) {
                        close(g_live_waterfall_stdin_fd);
                        g_live_waterfall_stdin_fd = -1;
                    }
                    snprintf(g_live_waterfall_iq,
                             sizeof g_live_waterfall_iq, "%s", iq_path);
                    char rate_arg[32];
                    snprintf(rate_arg, sizeof rate_arg,
                             "--rate=%d",
                             iq_rate > 0 ? iq_rate : 96000);
                    // pipe()+dup2 so the parent can shove
                    // line-based commands (e.g. "zoom 60\n") at the
                    // viewer's stdin.
                    int pfd[2] = {-1, -1};
                    if (pipe(pfd) != 0) { pfd[0] = pfd[1] = -1; }
                    pid_t pid = fork();
                    if (pid == 0) {
                        if (pfd[0] >= 0) {
                            close(pfd[1]);
                            dup2(pfd[0], STDIN_FILENO);
                            close(pfd[0]);
                        }
                        char *args[] = {
                            (char *) "live_waterfall",
                            (char *) g_live_waterfall_iq,
                            rate_arg,
                            NULL
                        };
                        execvp("live_waterfall", args);
                        _exit(127);
                    } else if (pid > 0) {
                        g_live_waterfall_pid = pid;
                        if (pfd[0] >= 0) close(pfd[0]);
                        g_live_waterfall_stdin_fd = pfd[1];
                    } else {
                        if (pfd[0] >= 0) close(pfd[0]);
                        if (pfd[1] >= 0) close(pfd[1]);
                    }
                }
                // Reap a viewer that the operator closed via its
                // window — non-blocking so the main loop never stalls.
                if (g_live_waterfall_pid > 0) {
                    int status;
                    pid_t r = waitpid(g_live_waterfall_pid,
                                      &status, WNOHANG);
                    if (r == g_live_waterfall_pid) {
                        g_live_waterfall_pid = -1;
                        g_live_waterfall_iq[0] = '\0';
                        if (g_live_waterfall_stdin_fd >= 0) {
                            close(g_live_waterfall_stdin_fd);
                            g_live_waterfall_stdin_fd = -1;
                        }
                    }
                }
            }
        }
        // Software Doppler tracking: the SDR LO stays fixed at the
        // nominal carrier (set once at session open) and we apply the
        // Doppler correction inside the IQ pump as a complex multiply.
        // No PLL glitches, sub-Hz resolution, and the displayed RX freq
        // updates smoothly. The threshold-driven hardware retune that
        // lived here previously fired every 1–10 seconds during a
        // pass and caused brief phase resets in the coherent demod
        // chain; this loop runs every tick at full precision.
        if (g_rx_session && state.doppler_correction_enabled) {
            double offset = state.doppler_downlink_frequency_hz
                          - state.nominal_downlink_frequency_hz;
            rx_session_set_doppler_offset(g_rx_session, offset);
        }
        if (g_rx_session) {
            double doppler_offset =
                state.doppler_downlink_frequency_hz
                - state.nominal_downlink_frequency_hz;
            rx_session_update_observer(g_rx_session,
                state.antenna_rotator.target_azimuth,
                state.antenna_rotator.target_elevation,
                state.prediction.satellite_ephem.range_km,
                state.prediction.satellite_ephem.range_rate_km_s,
                doppler_offset);
        }

        // Service a pending TX request. Three paths:
        //
        //   1. --tx-dry-run:    synthesize "ok" without touching the
        //                       SDR. Auto-tcmd + compose still exercise
        //                       all their UI state on dev hosts.
        //   2. g_rx_session up: real burst — submitted async to the
        //                       worker, which pauses RX, transmits and
        //                       resumes RX (~1 s). The main loop keeps
        //                       running between submit and poll so the
        //                       rotator, redraw, IPC and the next auto-
        //                       tcmd tick aren't frozen by the burst.
        //                       g_tx_request.pending stays set across
        //                       the in-flight window, so auto-tcmd will
        //                       not queue a second burst on top.
        //   3. neither:         reject so auto-tcmd can move on. The
        //                       operator must have started simple_sat_ops
        //                       --without-b210 without also passing
        //                       --tx-dry-run; just clear the pending
        //                       slot rather than deadlocking.
        if (g_tx_request.pending) {
            char summary[SSO_TX_TEXT_MAX];
            const char *outcome = NULL;
            int  on_air = 0;
            int  finished = 0;        // emit the result + clear pending this tick
            if (g_tx_dry_run) {
                snprintf(summary, sizeof summary, "%s",
                         g_tx_request.summary);
                outcome = "dry-run";   // composed but deliberately not keyed
                finished = 1;
            } else if (g_hmac_key_len == 0) {
                // CTS1 expects HMAC on every uplink. Without a valid
                // key the burst would go out unsigned and the satellite
                // would silently drop it. Refuse here so the operator
                // sees a clear error instead of letting it go out unsigned.
                snprintf(summary, sizeof summary, "%s",
                         g_tx_request.summary);
                outcome = "rejected: no HMAC key (see banner)";
                finished = 1;
            } else if (g_rx_session != NULL && !rx_session_can_tx(g_rx_session)) {
                // RX-only backend (e.g. RTL-SDR): never reaches the air.
                // Backstop for a stale queued burst that slipped past the
                // compose / auto-tcmd gates.
                snprintf(summary, sizeof summary, "%s", g_tx_request.summary);
                outcome = "rejected: RX-only SDR";
                finished = 1;
            } else if (g_rx_session != NULL) {
                if (!g_tx_inflight) {
                    if (rx_session_submit_burst(g_rx_session, &g_tx_request,
                                                 g_hmac_key, g_hmac_key_len) == 0) {
                        g_tx_inflight = 1;
                        // Stay pending; we'll poll on subsequent ticks.
                    } else {
                        // Worker refused (slot already busy or rxs error).
                        snprintf(summary, sizeof summary, "%s",
                                 g_tx_request.summary);
                        outcome = "rejected: rx_session busy";
                        finished = 1;
                    }
                } else {
                    rx_burst_result_t br;
                    int done = rx_session_poll_burst(g_rx_session, &br,
                                                      summary, sizeof summary);
                    if (done == 1) {
                        switch (br) {
                            case RX_BURST_OK:                 outcome = "ok"; on_air = 1; break;
                            case RX_BURST_NO_CORE:            outcome = "rejected: no B210"; break;
                            case RX_BURST_FRAME_BUILD_FAILED: outcome = "rejected: frame build"; break;
                            case RX_BURST_UHD_ERROR:          outcome = "uhd-err"; break;
                        }
                        g_tx_inflight = 0;
                        finished = 1;
                    }
                    // else: still in flight; fall through and let the
                    // rest of the main loop run.
                }
            } else {
                snprintf(summary, sizeof summary, "%s",
                         g_tx_request.summary);
                outcome = "rejected: no B210";
                finished = 1;
            }
            if (finished) {
                // A command that made it on the air gets a plain TX
                // record, nothing more: the ground station can confirm
                // it transmitted, but only the satellite can acknowledge,
                // and that arrives on the downlink, not here. Anything
                // that did NOT reach the air (rejected, dry-run, uhd-err)
                // gets a not-sent note carrying the reason.
                if (on_air) {
                    emit_tx_event_local(SSO_EVT_TX_COMMAND_SENT, summary, NULL);
                } else {
                    emit_tx_event_local(SSO_EVT_TX_NOT_SENT, summary, outcome);
                }
                // Audit: the result of every queued TX burst, so post-
                // incident review can see each tx-commit and whether it
                // reached the air (on_air=1 means the burst left the radio).
                {
                    char det[512];
                    snprintf(det, sizeof det,
                             "outcome=\"%.80s\" on_air=%d summary=\"%.300s\"",
                             outcome ? outcome : "?", on_air, summary);
                    sso_audit_event("tx-result", det);
                }
                g_tx_request.pending = 0;
            }
        }
#endif

        // --- IPC: serve clients, fan out state, honour SIGUSR1 yield ---
        // Always service the socket (cheap; accepts new viewers) but
        // throttle STATE broadcasts to 2 Hz so viewers don't get
        // hammered when the loop is running at UHD-chunk cadence.
        if (g_ipc) {
            sso_ipc_server_step(g_ipc, 0);
            if ((t_now - t_last_ipc_broadcast) >= IPC_BROADCAST_PERIOD_S) {
                ipc_broadcast_state(&state, current_az, current_el,
                                     current_downlink_frequency,
                                     doppler_delta_downlink,
                                     jul_utc);
                t_last_ipc_broadcast = t_now;
            }
            // Debounced cmd-preview broadcast: viewers see the operator's
            // ":" prompt as it's typed, lagging by g_cmd_debounce_ns so we
            // don't fire a packet per keystroke.
            if (g_cmd_active && g_cmd_dirty
                && (cmd_now_ns() - g_cmd_last_edit_ns) >= g_cmd_debounce_ns) {
                cmd_broadcast_preview();
                g_cmd_dirty = 0;
            }
        }
        if (g_yield_requested) {
            sso_audit_event("yield-requested",
                            "SIGUSR1 (--force takeover) — exiting");
            state.running = 0;
        }

        // Surface a finished spectrum render so the operator sees the
        // outcome (PNG path or ffmpeg error) in the command-line status.
        // The reap only joins the worker thread; status_msg is left
        // alone, so reading it after reap is safe.
        if (g_spec_job.active && g_spec_job.done) {
            if (g_spec_job.status_msg[0]) {
                cmd_set_status("%s", g_spec_job.status_msg);
            }
            spectrum_job_reap();
        }

        if (state.running) {
            // The B210 worker thread pumps UHD on its own pthread now,
            // so the main loop doesn't pace itself off the radio. Sleep
            // at the historical 2 Hz so rotator-STATUS polls don't ramp
            // up unexpectedly; redraw/IPC gates do their own throttling.
            // Exception: while the operator is typing in the ":" prompt,
            // drop to 20 ms so getch() echoes each keystroke promptly
            // (the 500 ms tick was capping input at ~2 chars/sec).
            usleep((g_cmd_active || g_tx_compose_active || g_auto_tcmd_active)
                   ? 20000 : UPDATE_INTERVAL_MICROSEC);
        }
    }

    endwin();
    tui_release_stderr();
    if (state.have_tr_switch) {
        tr_switch_disconnect(&state.tr_switch);
        state.have_tr_switch = 0;
    }
    // Join the rotator worker before closing the serial FD — otherwise a
    // mid-read in the worker would see EBADF and corrupt the snapshot.
    if (g_rot_async != NULL) {
        antenna_rotator_async_close(g_rot_async);
        g_rot_async = NULL;
    }
    if (state.have_antenna_rotator) {
        antenna_rotator_disconnect(&state.antenna_rotator);
        state.have_antenna_rotator = 0;
    }
    // Free any plan that survived (mid-pass exit / crash on a key
    // before the LOS branch had a chance to clear it).
    main_pursuit_clear_plan();
    if (g_ipc) {
        sso_ipc_server_close(g_ipc);
        g_ipc = NULL;
    }
    // Politely terminate the live raylib waterfall if we spawned one.
    // 5 s timeout via WNOHANG polling so the operator doesn't wait on
    // a hung viewer at shutdown.
    if (g_live_waterfall_pid > 0) {
        kill(g_live_waterfall_pid, SIGTERM);
        for (int t = 0; t < 50; ++t) {
            int status;
            pid_t r = waitpid(g_live_waterfall_pid, &status, WNOHANG);
            if (r == g_live_waterfall_pid) {
                g_live_waterfall_pid = -1;
                break;
            }
            usleep(100000);
        }
        if (g_live_waterfall_pid > 0) {
            kill(g_live_waterfall_pid, SIGKILL);
            waitpid(g_live_waterfall_pid, NULL, 0);
            g_live_waterfall_pid = -1;
        }
        if (g_live_waterfall_stdin_fd >= 0) {
            close(g_live_waterfall_stdin_fd);
            g_live_waterfall_stdin_fd = -1;
        }
    }
#ifdef SSO_WITH_SDR
    char final_wav_path[512] = "";
    char final_iq_path[512]  = "";
    int  final_iq_rate       = 0;
    if (g_rx_session) {
        // Snapshot both sidecar paths before close so the full-pass
        // renderers can find the closed files on disk. Both paths
        // persist across wav_stop in rx_session.
        rx_session_wav_snapshot(g_rx_session,
                                final_wav_path, sizeof final_wav_path,
                                NULL, NULL, NULL);
        rx_session_iq_snapshot(g_rx_session,
                               final_iq_path, sizeof final_iq_path,
                               NULL, &final_iq_rate);
        // The worker owns the WAV writer and the B210 core. Closing
        // the session signals the worker to stop, joins it, then
        // tears down both. Any open WAV / .iq gets its header patched
        // (WAV) or its trailer flushed (IQ).
        rx_session_request_wav_stop(g_rx_session);
        rx_session_close(g_rx_session);
        g_rx_session = NULL;
    }

    // Any in-flight `:spectrum N` worker is touching the same WAV / IQ
    // — let it finish before we hand the file to the full-pass render.
    if (g_spec_job.active) {
        pthread_join(g_spec_job.thr, NULL);
        g_spec_job.active = 0;
        if (g_spec_job.status_msg[0]) {
            fprintf(stderr, "simple_sat_ops: %s\n", g_spec_job.status_msg);
        }
    }

    // Full-pass renderer. Prefer the IQ → gen_waterfall path because it
    // gives SatNOGS-style waterfalls (real complex FFT, median-subtracted
    // floor, viridis). Fall back to the FM-demod WAV via ffmpeg when the
    // IQ sidecar isn't on disk (e.g., disk full, mid-pass shutdown).
    int did_iq = 0;
    if (final_iq_path[0] && final_iq_rate > 0) {
        struct stat st;
        if (stat(final_iq_path, &st) == 0 && st.st_size > 0) {
            char png[640];
            if (generate_full_iq_waterfall(final_iq_path, final_iq_rate,
                                            png, sizeof png) == 0) {
                fprintf(stderr, "simple_sat_ops: waterfall -> %s\n", png);
                did_iq = 1;
            } else {
                fprintf(stderr,
                    "simple_sat_ops: gen_waterfall failed for %s "
                    "(gen_waterfall on PATH?)\n", final_iq_path);
            }
        }
    }
    if (!did_iq && final_wav_path[0]) {
        struct stat st;
        if (stat(final_wav_path, &st) == 0 && st.st_size > 44) {
            char png[640];
            if (generate_full_spectrogram(final_wav_path, png, sizeof png) == 0) {
                fprintf(stderr, "simple_sat_ops: pass spectrogram -> %s\n", png);
            } else {
                fprintf(stderr,
                    "simple_sat_ops: ffmpeg spectrogram failed for %s "
                    "(ffmpeg on PATH?)\n", final_wav_path);
            }
        }
    }
#endif

    if (state.prediction.auto_sat) {
        free_passes();
    }

    // Final line: tell the operator whether anything landed in the
    // redirected stderr log during the pass.
    tui_report_errors();
    return 0;
}

// --- apply_args ---------------------------------------------------

// One self-contained block per option, each testing "... || help" so
// that in help mode (help != HELP_OFF) every block prints its one-line
// help and falls through to the next. In parse mode only the matching
// block runs its body and writes its result straight into *state (there
// is no separate config struct; apply_args has always filled state).
// See src/cli/argparse.h for the convention.
//
// Option column width: widest label below is
// "--ignore-at-your-peril-all-tc-errors" (36) + a small margin.
#define OPTW 38

static int apply_args(state_t *state, int argc, char **argv, double jul_utc, int help)
{
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;
    double min_altitude_km = 0.0;
    double max_altitude_km = 1000.0;
    double min_minutes_away = 1.0;
    double max_minutes_away = 90.0;
    double min_elevation = 0.0;
    double max_elevation = 90.0;
    int with_constellations = 0;
    char *positional = NULL;

    if (!help) {
        state->antenna_rotator.tracking_prep_time_minutes = TRACKING_PREP_TIME_MINUTES;
        state->satellite_tracking = 0;

        state->nominal_uplink_frequency_hz = UPLINK_FREQ_MHZ * 1e6;
        state->nominal_downlink_frequency_hz = DOWNLINK_FREQ_MHZ * 1e6;
        state->doppler_uplink_frequency_hz = state->nominal_uplink_frequency_hz;
        state->doppler_downlink_frequency_hz = state->nominal_downlink_frequency_hz;
        state->doppler_correction_enabled = 1;
        // SIGNED LO offset from the nominal carrier. Positive → LO ABOVE
        // nominal (signal lands at negative baseband). Negative → LO
        // BELOW nominal (signal at positive baseband). Default -25 kHz
        // puts the corrected signal at +25 kHz baseband, away from the
        // B210's DC null. Operator can shift it to dodge fixed-pattern
        // spurs — comfortable range is roughly ±5..±35 kHz: at least
        // 5 kHz to clear DC, and at most ~35 kHz so the ±10 kHz Doppler
        // swing stays inside the 48 kHz post-decim half-band.
        state->rx_lo_offset_hz = -25000.0;
        state->rx_gain_db      = 30.0;
        // AD9361 background tracking. The visible ~51 Hz comb of impulsive
        // spikes at mid-range gain is from the IQ-balance loop (discrete
        // phase-rotation steps applied to the captured IQ); the DC-offset
        // loop is a slow continuous IIR notch that DOESN'T produce
        // spikes but DOES suppress the AD9361's static ADC DC bias.
        // Turn IQ tracking off by default (kills the spikes), leave DC
        // tracking on (otherwise the static DC bias rotates into a strong
        // +lo_offset_hz sinusoid via fm_lo_nco on the decode path, which
        // dominates the IQ time series).
        state->rx_dc_offset_track  = 1;
        state->rx_iq_balance_track = 0;

        state->run_with_antenna_rotator = 1;
        state->antenna_rotator.device_filename = "/dev/ttyUSB0";
        state->antenna_rotator.serial_speed = B600;
        state->antenna_rotator.fixed_target = 0;

        // T/R antenna switch: auto-probe /dev/ttyACM0. Failure is a
        // one-line warning, not an error.
        state->run_with_tr_switch = 1;
        state->have_tr_switch     = 0;
        state->tr_switch.device_filename = "/dev/ttyACM0";
        state->tr_switch.serial_speed    = B115200;
    }

    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // Positional first so <satellite_id> lists above the options.
        // A token that is not "--"-prefixed counts as the positional.
        // The actual pointer is resolved by the discovery scan AFTER the
        // loop (which re-walks argv exactly as the pre-conversion code
        // did, including its quirk that a space-form option value can be
        // grabbed as the positional); here we only print the help line
        // and mark the token matched so a bare extra positional falls
        // through to the post-loop n_positional > 1 check rather than the
        // unknown-token branch.
        if (strncmp("--", arg, 2) != 0 || help) {
            if (help) parse_help_line(OPTW, "<satellite_id>",
                "name prefix in the TLE, or `next` to auto-pick the next pass");
            matched = 1;
        }
        if (strcmp("--help", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "short help (this message)");
            else { apply_args(state, argc, argv, jul_utc, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp("--help-full", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--help-full", "detailed help with keyboard layout");
            else { apply_args(state, argc, argv, jul_utc, HELP_FULL); return PARSE_HELP; }
            matched = 1;
        }
        if (strncmp("--verbose=", arg, 10) == 0 || help) {
            if (help) parse_help_line(OPTW, "--verbose=<level>", "verbosity integer");
            else {
                state->n_options++;
                if (strlen(arg) < 11) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->verbose_level = atoi(arg + 10);
            }
            matched = 1;
        }
        if (strcmp("--with-rotator", arg) == 0
                || strcmp("--with-hardware", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--with-rotator",
                "drive the rotator (default; --with-hardware synonym, no-op)");
            else {
                // Rotator is on by default now. These flags survive as
                // silent no-ops so existing scripts and muscle memory
                // keep working.
                state->n_options++;
                state->run_with_antenna_rotator = 1;
            }
            matched = 1;
        }
        if (strcmp("--without-rotator", arg) == 0
                || strcmp("--without-hardware", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-rotator",
                "skip the SPID Rot2Prog (--without-hardware synonym)");
            else {
                state->n_options++;
                state->run_with_antenna_rotator = 0;
            }
            matched = 1;
        }
        if (strcmp("--calibrate-rotator", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--calibrate-rotator",
                "measure rotator slew rates then exit (needs --confirm-rotator-calibrate)");
            else { state->n_options++; g_calibrate_rotator = 1; }
            matched = 1;
        }
        if (strcmp("--confirm-rotator-calibrate", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--confirm-rotator-calibrate",
                "safety interlock for --calibrate-rotator (antenna moves)");
            else { state->n_options++; g_confirm_rotator_calibrate = 1; }
            matched = 1;
        }
        if (strcmp("--without-rotator-pursuit", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-rotator-pursuit",
                "disable the pursuit / lead-aim planner even if calibrated");
            else { state->n_options++; g_without_rotator_pursuit = 1; }
            matched = 1;
        }
        if (strcmp("--without-tr-switch", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-tr-switch",
                "skip the T/R switch probe entirely");
            else { state->n_options++; state->run_with_tr_switch = 0; }
            matched = 1;
        }
        if (strncmp("--tr-switch-device=", arg, 19) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tr-switch-device=<path>",
                "UHF T/R antenna switch tty (default /dev/ttyACM0)");
            else {
                state->n_options++;
                if (strlen(arg) < 20) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->tr_switch.device_filename = arg + 19;
            }
            matched = 1;
        }
        if (strcmp("--without-b210", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-b210",
                "skip the USRP B210 (UI + rotator only)");
            else { state->n_options++; g_without_b210 = 1; }
            matched = 1;
        }
#ifdef SSO_WITH_SDR
        if (strncmp("--sdr-type=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-type=uhd|rtlsdr|auto",
                "SDR backend (default auto; RTL-SDR is RX-only)");
            else {
                state->n_options++;
                if (sdr_backend_type_from_string(arg + 11, &g_sdr_type) != 0) {
                    fprintf(stderr, "--sdr-type: unknown '%s' "
                            "(want uhd | rtlsdr | auto)\n", arg + 11);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--sdr-device=", arg, 13) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-device=<sel>",
                "backend device selector (RTL-SDR index; UHD use --uhd-args)");
            else {
                state->n_options++;
                snprintf(g_sdr_device, sizeof g_sdr_device, "%s", arg + 13);
            }
            matched = 1;
        }
        if (strncmp("--uhd-args=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--uhd-args=<args>",
                "UHD device-args verbatim; overrides detection");
            else {
                state->n_options++;
                snprintf(g_uhd_args, sizeof g_uhd_args, "%s", arg + 11);
            }
            matched = 1;
        }
        if (strncmp("--sdr-fpga=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-fpga=<path>",
                "force a UHD FPGA image (B2xx clone with non-stock bitstream)");
            else {
                state->n_options++;
                snprintf(g_sdr_fpga, sizeof g_sdr_fpga, "%s", arg + 11);
            }
            matched = 1;
        }
#endif
        if (strcmp("--no-tx", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-tx",
                "open the B210 for RX but block the TX compose modal from keying the PA");
            else { state->n_options++; g_no_tx = 1; }
            matched = 1;
        }
        if (strcmp("--live-waterfall", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--live-waterfall",
                "auto-launch the raylib live_waterfall viewer when recording starts");
            else { state->n_options++; g_run_live_waterfall = 1; }
            matched = 1;
        }
        if (strcmp("--always-record", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--always-record",
                "record from B210 open until shutdown (skip per-pass start/stop)");
            else { state->n_options++; g_always_record = 1; }
            matched = 1;
        }
        if (strcmp("--testing", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--testing",
                "bench mode: pass folder under Testing/ at current local time, no TLE");
            else { state->n_options++; g_testing_mode = 1; }
            matched = 1;
        }
        if (strcmp("--scan-sky", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--scan-sky",
                "rebind T to walk the rotator through a sky grid, dwelling 5 s each");
            else { state->n_options++; g_scan_sky_mode = 1; }
            matched = 1;
        }
        if (strncmp("--scan-step=", arg, 12) == 0 || help) {
            if (help) parse_help_line(OPTW, "--scan-step=<deg>",
                "elevation ring spacing for --scan-sky (default 15, clamped [1,45])");
            else {
                state->n_options++;
                g_scan_step_deg = atof(arg + 12);
                if (g_scan_step_deg < 1.0)  g_scan_step_deg = 1.0;
                if (g_scan_step_deg > 45.0) g_scan_step_deg = 45.0;
            }
            matched = 1;
        }
        if (strcmp("--tx-dry-run", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tx-dry-run",
                "record every TX burst as not-sent instead of routing it through the SDR");
            else { state->n_options++; g_tx_dry_run = 1; }
            matched = 1;
        }
        if (strncmp("--tx-preroll-ms=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tx-preroll-ms=<n>",
                "modulated 0xAA carrier before each TX burst (default 200, [0,5000])");
            else {
                state->n_options++;
                int v = atoi(arg + 16);
                if (v < 0)    v = 0;
                if (v > 5000) v = 5000;
                g_tx_preroll_ms = v;
            }
            matched = 1;
        }
        // Filename args use the space form (--foo <path>) so bash
        // tab-completion works. The old --foo=<path> form is rejected
        // with a one-line hint pointing at the new spelling.
        if (strcmp("--tc-file", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tc-file <path>",
                "load ASCII telecommands (one per line; 'A' / `:auto` in the UI to send)");
            else {
                // arg is argv[t + 1]; its value is the next token,
                // argv[t + 2]. Consume it and step t past it.
                if (t + 2 >= argc) {
                    fprintf(stderr, "--tc-file: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                snprintf(g_auto_tcmd_file_path, sizeof g_auto_tcmd_file_path,
                         "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--tc-file=", arg, 10) == 0) {
            fprintf(stderr,
                "--tc-file=<path> is no longer accepted; "
                "use `--tc-file <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--ignore-at-your-peril-all-tc-errors", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ignore-at-your-peril-all-tc-errors",
                "start even if the --tc-file agenda has telecommand lint errors");
            else { state->n_options++; g_ignore_tc_errors = 1; }
            matched = 1;
        }
        if (strcmp("--hmac-keyfile", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--hmac-keyfile <path>",
                "HMAC key file shown on the operator banner (default shared, then user)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--hmac-keyfile: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                snprintf(g_hmac_keyfile_path, sizeof g_hmac_keyfile_path,
                         "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--hmac-keyfile=", arg, 15) == 0) {
            fprintf(stderr,
                "--hmac-keyfile=<path> is no longer accepted; "
                "use `--hmac-keyfile <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--tle", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tle <path>",
                "path to a TLE file (default $HOME/.local/state/simple_sat_ops/active.tle)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--tle: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                state->prediction.tles_filename = tle_path_resolve(argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--tle=", arg, 6) == 0) {
            fprintf(stderr,
                "--tle=<path> is no longer accepted; "
                "use `--tle <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--pass-folder", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--pass-folder <path>",
                "pre-seed the pass folder (handoff: inherit a previous operator's folder)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--pass-folder: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                // Pre-seed g_pass_folder; setup_pass_folder() then skips
                // its AOS-driven auto-discovery and uses the inherited
                // path (handoff case: new operator picks up the previous
                // operator's pass folder).
                snprintf(g_pass_folder, sizeof g_pass_folder, "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--pass-folder=", arg, 14) == 0) {
            fprintf(stderr,
                "--pass-folder=<path> is no longer accepted; "
                "use `--pass-folder <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--rotator-device", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-device <path>",
                "SPID Rot2Prog tty (default /dev/ttyUSB0)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--rotator-device: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                state->antenna_rotator.device_filename = argv[t + 2];
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--rotator-device=", arg, 17) == 0) {
            fprintf(stderr,
                "--rotator-device=<path> is no longer accepted; "
                "use `--rotator-device <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strncmp("--uplink-freq-mhz=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--uplink-freq-mhz=<mhz>",
                "uplink nominal carrier, MHz (informational)");
            else {
                state->n_options++;
                if (strlen(arg) < 19) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->nominal_uplink_frequency_hz = atof(arg + 18) * 1e6;
            }
            matched = 1;
        }
        if (strncmp("--downlink-freq-mhz=", arg, 20) == 0 || help) {
            if (help) parse_help_line(OPTW, "--downlink-freq-mhz=<mhz>",
                "downlink / simplex carrier nominal, MHz (informational)");
            else {
                state->n_options++;
                if (strlen(arg) < 21) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->nominal_downlink_frequency_hz = atof(arg + 20) * 1e6;
            }
            matched = 1;
        }
        if (strcmp("--no-doppler-correction", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-doppler-correction",
                "display nominal freqs without Doppler");
            else { state->n_options++; state->doppler_correction_enabled = 0; }
            matched = 1;
        }
        if (strncmp("--lo-offset=", arg, 12) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lo-offset=<kHz>",
                "park the SDR LO this far off the nominal carrier (signed, default -25)");
            else {
                state->n_options++;
                // Argument is kHz so an integer is easy to type; we store Hz.
                state->rx_lo_offset_hz = atof(arg + 12) * 1000.0;
            }
            matched = 1;
        }
        if (strncmp("--rx-gain=", arg, 10) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rx-gain=<dB>",
                "AD9361 RX gain at session open, dB (default 30, range [0,76])");
            else {
                state->n_options++;
                double g = atof(arg + 10);
                // AD9361 RX gain range is 0-76 dB; UHD coerces values outside
                // this and prints a warning, but we clip here so the value in
                // state matches what the hardware will use.
                if (g < 0.0)       g = 0.0;
                else if (g > 76.0) g = 76.0;
                state->rx_gain_db = g;
            }
            matched = 1;
        }
        if (strncmp("--ad9361-dc-track=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ad9361-dc-track=on|off",
                "AD9361 background DC-offset tracking (default on)");
            else {
                // on|off|true|false|1|0
                const char *v = arg + 18;
                state->n_options++;
                state->rx_dc_offset_track =
                    (strcmp(v, "on")   == 0
                     || strcmp(v, "true") == 0
                     || strcmp(v, "1")  == 0) ? 1 : 0;
            }
            matched = 1;
        }
        if (strncmp("--ad9361-iq-track=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ad9361-iq-track=on|off",
                "AD9361 background IQ-balance tracking (default off; ~51 Hz spike comb)");
            else {
                const char *v = arg + 18;
                state->n_options++;
                state->rx_iq_balance_track =
                    (strcmp(v, "on")   == 0
                     || strcmp(v, "true") == 0
                     || strcmp(v, "1")  == 0) ? 1 : 0;
            }
            matched = 1;
        }
        if (strncmp("--rotator-target-elevation=", arg, 27) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-target-elevation=<deg>",
                "park on a fixed elevation");
            else {
                state->n_options++;
                if (strlen(arg) < 28) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->antenna_rotator.target_elevation = atof(arg + 27);
                if (state->antenna_rotator.target_elevation < 0.0) {
                    state->antenna_rotator.target_elevation = 0.0;
                } else if (state->antenna_rotator.target_elevation
                           > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                    state->antenna_rotator.target_elevation =
                        ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                }
                state->antenna_rotator.fixed_target = 1;
            }
            matched = 1;
        }
        if (strncmp("--rotator-target-azimuth=", arg, 25) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-target-azimuth=<deg>",
                "park on a fixed azimuth");
            else {
                state->n_options++;
                if (strlen(arg) < 26) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                double az = atof(arg + 25);
                if (az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH) {
                    az = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
                } else if (az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                    az = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
                }
                state->antenna_rotator.target_azimuth = az;
                state->antenna_rotator.target_azimuth_unwrapped = az;
                state->antenna_rotator.unwrapped_target_valid = 1;
                state->antenna_rotator.fixed_target = 1;
            }
            matched = 1;
        }
        if (strncmp("--lat=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lat=<deg>", "geodetic latitude (default RAO Priddis)");
            else {
                state->n_options++;
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                site_latitude = atof(arg + 6);
            }
            matched = 1;
        }
        if (strncmp("--lon=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lon=<deg>", "geodetic longitude, east positive");
            else {
                state->n_options++;
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                site_longitude = atof(arg + 6);
            }
            matched = 1;
        }
        if (strncmp("--alt=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--alt=<m>", "altitude above ellipsoid, metres");
            else {
                state->n_options++;
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                site_altitude = atof(arg + 6);
            }
            matched = 1;
        }
        if (strcmp("--include-constellations", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--include-constellations",
                "include Starlink/OneWeb-style swarms in the `next` pass filter");
            else { state->n_options++; with_constellations = 1; }
            matched = 1;
        }
        if (strncmp("--min-altitude-km=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-altitude-km=<km>",
                "minimum orbital altitude (default 0)");
            else {
                state->n_options++;
                if (strlen(arg) < 19) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                min_altitude_km = atof(arg + 18);
            }
            matched = 1;
        }
        if (strncmp("--max-altitude-km=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-altitude-km=<km>",
                "maximum orbital altitude (default 1000)");
            else {
                state->n_options++;
                if (strlen(arg) < 19) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                max_altitude_km = atof(arg + 18);
            }
            matched = 1;
        }
        if (strncmp("--min-elevation=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-elevation=<deg>",
                "minimum peak elevation (default 0)");
            else {
                state->n_options++;
                if (strlen(arg) < 17) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                min_elevation = atof(arg + 16);
            }
            matched = 1;
        }
        if (strncmp("--min-minutes=", arg, 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-minutes=<n>",
                "minimum minutes until AOS (default 1)");
            else {
                state->n_options++;
                if (strlen(arg) < 15) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                min_minutes_away = atof(arg + 14);
            }
            matched = 1;
        }
        if (strncmp("--max-minutes=", arg, 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-minutes=<n>",
                "maximum minutes until AOS (default 90)");
            else {
                state->n_options++;
                if (strlen(arg) < 15) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                max_minutes_away = atof(arg + 14);
            }
            matched = 1;
        }
        if (strcmp("--control", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--control",
                "open the sso_ipc server (operator mode)");
            else { state->n_options++; g_control_mode = 1; }
            matched = 1;
        }
        if (strcmp("--self-test", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--self-test",
                "print the settings simple_sat_ops would run with, then exit 0");
            else { state->n_options++; g_self_test = 1; }
            matched = 1;
        }

        // Unknown token. Only "--"-prefixed tokens are an error here
        // (the old code's final `strncmp("--", ...) == 0` branch);
        // a bare extra positional falls through and is caught by the
        // n_positional > 1 check after the loop.
        if (!matched && !help) {
            if (strncmp("--", arg, 2) == 0) {
                fprintf(stderr, "Unable to parse option '%s'\n", arg);
                return PARSE_ERROR;
            }
        }
    }

    // Full-help epilog: the keyboard layout + examples, printed once
    // after all the option lines (only for --help-full).
    if (help >= HELP_FULL) {
        printf(
            "\n"
            "KEYBOARD (unlocked by default, press K to toggle lock state)\n"
            "\n"
            "  K         Toggle keyboard lock\n"
            "  T         Start tracking the current satellite\n"
            "  s         Stop tracking\n"
            "  r         Reset rotator to az=0, el=0\n"
            "  [ / ]     Nudge antenna azimuth -5 / +5 deg\n"
            "  { / }     Nudge antenna azimuth -1 / +1 deg (fine)\n"
            "  , / .     Nudge antenna elevation -5 / +5 deg\n"
            "  < / >     Nudge antenna elevation -1 / +1 deg (fine)\n"
            "  q         Quit\n"
            "\n"
            "EXAMPLES\n"
            "\n"
            "  # Auto-pick next visible pass above 10 deg (rotator on by default)\n"
            "  simple_sat_ops next --min-elevation=10 --min-minutes=10 --max-minutes=45\n"
            "\n"
            "  # Dry-run prediction on a dev host (no rotator hardware)\n"
            "  simple_sat_ops 'ISS (ZARYA)' --without-rotator\n"
            "\n"
            "  # Operator coordination (broadcasts state to viewers over sso_ipc)\n"
            "  simple_sat_ops next --control\n");
    }
    if (help) return PARSE_OK;

    int n_positional = argc - state->n_options - 1;  // -1 for argv[0]
    if (n_positional > 1) {
        fprintf(stderr,
            "simple_sat_ops: too many positional arguments "
            "(expected at most one <satellite_id>)\n");
        return PARSE_ERROR;
    }

    // Find the (single) positional, if any. Existing convention is
    // "positional at argv[1]" but loop is robust to options-before /
    // options-after orderings.
    for (int i = 1; i < argc; i++) {
        if (strncmp("--", argv[i], 2) == 0) continue;
        positional = argv[i];
        break;
    }

    // Any invocation without --control: the standalone tracker is being
    // phased out in favour of the operator+viewer split, so there is no
    // longer a "track this on my own" path. Probe for the running
    // operator and either attach as a viewer or bail with a hint.
    // This holds whether or not a satellite name was given on the
    // command line - a viewer mirrors whatever the operator is tracking,
    // so any positional is ignored here. --self-test skips the probe (a
    // side effect) so the config dump runs cleanly with no live operator.
    if (!g_control_mode && !g_self_test) {
        sso_ipc_client_t *probe = sso_ipc_client_connect("simple_sat_ops");
        if (probe == NULL) {
            fprintf(stderr,
                "operator not found: try `simple_sat_ops --control` "
                "to operate FrontierSat\n");
            return PARSE_ERROR;
        }
        sso_ipc_client_close(probe);
        // Operator is up — main() will dispatch into run_viewer()
        // instead of the standalone-tracker path.
        g_viewer_mode = 1;
        return PARSE_OK;
    }

    state->prediction.observer_ephem.position_geodetic.lat =
        site_latitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.lon =
        site_longitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.alt =
        site_altitude / 1000.0;

    // --control with no positional: auto-discover the newest TLE
    // under /FrontierSat/TLEs/ and load it directly. setup_pass_folder
    // pins this source file (under its original tle-YYYYMMDD.tle name)
    // into the pass folder once AOS is known.
    if (n_positional == 0 && g_control_mode) {
        if (state->prediction.tles_filename == NULL) {
            const char *tles_root = sso_tles_dir();
            static char src_tle[1024];
            time_t src_mtime = 0;
            if (find_newest_tle_recursive(tles_root, src_tle, sizeof src_tle,
                                          &src_mtime) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: --control wants a TLE under %s, "
                    "but no *.tle was found there. Drop one in "
                    "(or pass --tle=<path>).\n", tles_root);
                return PARSE_ERROR;
            }
            fprintf(stderr, "simple_sat_ops: using TLE %s\n", src_tle);
            state->prediction.tles_filename = tle_path_resolve(src_tle);
        }
        static char sat_name[64];
        if (read_tle_name(state->prediction.tles_filename,
                          sat_name, sizeof sat_name) != 0) {
            fprintf(stderr,
                "simple_sat_ops: %s has no name line (2-line TLE?); "
                "pass the satellite name explicitly\n",
                state->prediction.tles_filename);
            return PARSE_ERROR;
        }
        state->prediction.satellite_ephem.name = sat_name;
        fprintf(stderr, "simple_sat_ops: tracking '%s'\n", sat_name);
    } else {
        if (state->prediction.tles_filename == NULL) {
            static char default_tle[1024];
            if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
                fprintf(stderr,
                    "HOME unset or path too long; pass --tle=<path>\n");
                return PARSE_ERROR;
            }
            state->prediction.tles_filename = tle_path_resolve(default_tle);
        }
        state->prediction.satellite_ephem.name = positional;
    }

    // --self-test exits before TLE/pass-search anyway, and the bare
    // form (no positional, no --control) leaves .name == NULL; skip
    // the auto-pass search rather than feeding NULL to strcmp.
    if (state->prediction.satellite_ephem.name != NULL
        && strcmp(state->prediction.satellite_ephem.name, "next") == 0) {
        state->prediction.auto_sat = 1;
        criteria_t criteria = {
            .min_altitude_km = min_altitude_km,
            .max_altitude_km = max_altitude_km,
            .min_minutes = min_minutes_away,
            .max_minutes = max_minutes_away,
            .min_elevation = min_elevation,
            .max_elevation = max_elevation,
            .regex = NULL,
            .regex_ignore_case = 0,
            .with_constellations = with_constellations,
        };
        prediction_t prediction_tmp = {0};
        prediction_tmp.tles_filename = state->prediction.tles_filename;
        prediction_tmp.observer_ephem.position_geodetic.lat =
            state->prediction.observer_ephem.position_geodetic.lat;
        prediction_tmp.observer_ephem.position_geodetic.lon =
            state->prediction.observer_ephem.position_geodetic.lon;
        prediction_tmp.observer_ephem.position_geodetic.alt =
            state->prediction.observer_ephem.position_geodetic.alt;
        find_passes(&prediction_tmp, jul_utc, 0.5, &criteria, NULL, NULL, 0, 0);
        const size_t n = number_of_passes();
        if (n == 0) {
            fprintf(stderr, "Unable to automatically find next in queue.\n");
            return PARSE_ERROR;
        }

        const pass_t *p = get_pass(0);
        state->prediction.satellite_ephem.name = strdup(p->name);
        printf("Satellite: %s\n", state->prediction.satellite_ephem.name);
    }

    return PARSE_OK;
}

// --- Tracking helpers ---------------------------------------------

void start_tracking(state_t *state)
{
    int antenna_rotator_result = 0;

    state->satellite_tracking = 1;
    state->doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control =
        state->antenna_rotator.antenna_should_be_controlled;
    // Clear the flip latch so the next tracking-enable re-decides for
    // the upcoming pass.
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;
    if (state->antenna_rotator.fixed_target) {
        antenna_rotator_result = main_rotator_submit_set(state,
            state->antenna_rotator.target_azimuth_unwrapped,
            state->antenna_rotator.target_elevation);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error setting antenna rotator position\n");
        } else {
            state->antenna_rotator.antenna_is_moving = 1;
        }
    }
}

void stop_tracking(state_t *state)
{
    state->satellite_tracking = 0;
    state->doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control = 0;
    if (state->run_with_antenna_rotator && g_rot_async != NULL) {
        antenna_rotator_async_submit_stop(g_rot_async);
        antenna_rotator_async_kick_status(g_rot_async);
        // Wait briefly for the next OK STATUS so target_* reflect the
        // position the antenna actually stopped at (not where the
        // satellite was). Bounded — the operator's 's' / 'r' keystroke
        // shouldn't hang if the controller is unresponsive.
        if (antenna_rotator_async_wait_next_good_status(g_rot_async, 750) == 0) {
            (void) main_rotator_refresh_targets_from_snapshot(state);
        }
    }
    state->antenna_rotator.antenna_is_moving = 0;
    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;
}

int point_to_stationary_target(state_t *state, double azimuth, double elevation)
{
    state->satellite_tracking = 0;
    state->antenna_rotator.antenna_is_under_control = 0;
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;

    // Re-sync the unwrap accumulator to the antenna's *physical* position
    // before computing the home move. target_azimuth_unwrapped can drift
    // from reality across a pass -- e.g. it can hold a negative co-terminal
    // that the wrapped status display still renders as a positive angle --
    // which made "home" drive the long way around instead of unwinding.
    // Seeding prev from the live status makes r/home always move from where
    // the antenna actually is. If no fresh status is available, fall back to
    // the last known target, and only fail if we have neither.
    if (main_rotator_refresh_targets_from_snapshot(state) != 0
        && !state->antenna_rotator.unwrapped_target_valid) {
        return ANTENNA_ROTATOR_BAD_RESPONSE;
    }

    double prev = state->antenna_rotator.target_azimuth_unwrapped;
    double final_az = antenna_rotator_home_unwrapped_target(prev, azimuth);
    double delta = final_az - prev;

    // Already at the target (azimuth and elevation both within the rotator's
    // deadband)? Do nothing instead of issuing a zero-length SET -- on this
    // controller a SET kicks off a couple of seconds of target-echo on STATUS
    // and a spurious "moving" state, for no actual move. delta is the raw
    // rotation the unwind needs, so a wound antenna (|delta| large) still
    // moves; only a genuine no-op is skipped.
    if (fabs(delta) <= MAX_DELTA_AZIMUTH_DEGREES
        && fabs(elevation - state->antenna_rotator.elevation)
               <= MAX_DELTA_ELEVATION_DEGREES) {
        state->antenna_rotator.target_azimuth_unwrapped = final_az;
        state->antenna_rotator.target_azimuth           = final_az;
        state->antenna_rotator.target_elevation         = elevation;
        state->antenna_rotator.unwrapped_target_valid   = 1;
        state->antenna_rotator.homing_in_progress       = 0;
        state->antenna_rotator.home_pending_final_az    = 0.0;
        cmd_set_status("home: already at %.1f, %.1f deg -- no move",
                       prev, state->antenna_rotator.elevation);
        sso_audit_event("home", "already at target -- no move");
        return ANTENNA_ROTATOR_OK;
    }

    // Trace the home decision (audit log + a brief on-screen line) so the
    // unwind can be confirmed at the rotator: |delta| > 180 takes the
    // two-step unwind (mid waypoint, then the final leg once it's reached
    // the 0..180 zone -- see the loop), otherwise a direct move.
    {
        char det[128];
        snprintf(det, sizeof det, "from az=%.1f to %.1f delta=%+.1f (%s)",
                 prev, final_az, delta,
                 (fabs(delta) > 180.0) ? "two-step unwind" : "direct");
        sso_audit_event("home", det);
        cmd_set_status("home: %.1f -> %.1f, %+.1f deg %s (%s)",
                       prev, final_az, delta, delta < 0.0 ? "CCW" : "CW",
                       (fabs(delta) > 180.0) ? "unwind" : "direct");
    }

    if (fabs(delta) > 180.0) {
        // Two-step home: halfway waypoint first to disambiguate the
        // direction of rotation; the main loop drives the second leg
        // once the antenna has stopped at the intermediate.
        double mid = prev + delta / 2.0;
        if (mid < ANTENNA_ROTATOR_MINIMUM_AZIMUTH)
            mid = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
        if (mid > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH)
            mid = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
        state->antenna_rotator.home_pending_final_az = final_az;
        state->antenna_rotator.homing_in_progress = 1;
        int rc = main_rotator_submit_set(state, mid, elevation);
        if (rc == ANTENNA_ROTATOR_OK) {
            state->antenna_rotator.antenna_is_moving = 1;
        }
        return rc;
    }

    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    int rc = main_rotator_submit_set(state, final_az, elevation);
    if (rc == ANTENNA_ROTATOR_OK) {
        state->antenna_rotator.antenna_is_moving = 1;
    }
    return rc;
}

// -------------------------------------------------------------------
// --scan-sky helpers
// -------------------------------------------------------------------
//
// scan_build_targets fills g_scan_targets with a roughly equal-solid-
// angle grid covering the sky above the horizon. Elevation rings are
// spaced del_deg apart; at each ring, the azimuth count is round(
// 360/del_deg * cos(el)) so high-elevation rings (which subtend less
// solid angle on the sphere) carry proportionally fewer az samples.
// Ring direction alternates so consecutive targets across rings sit
// close in azimuth (snake pattern, minimises rotator transit).
//
// Azimuths are emitted in the range (-180, 180), with the exact
// boundary skipped so the rotator never tries to drive past its
// mechanical wrap limits (range is [-179, 539]; we stay well inside).
// The first target is forced to (0, 0) so every run starts from a
// known reference.

static int scan_build_targets(double del_deg)
{
    if (del_deg < 1.0) del_deg = 1.0;
    if (del_deg > 45.0) del_deg = 45.0;
    int n = 0;
    // Force the starting target.
    g_scan_targets[n].az_deg = 0.0;
    g_scan_targets[n].el_deg = 0.0;
    ++n;
    int direction = 1;
    int el_steps  = (int) round(90.0 / del_deg);
    for (int eli = 0; eli <= el_steps && n < SCAN_MAX_TARGETS; ++eli) {
        double el = (double) eli * del_deg;
        if (el > 90.0) el = 90.0;
        int n_az;
        if (el >= 90.0 - 0.001) {
            n_az = 1;
        } else {
            double cos_el = cos(el * M_PI / 180.0);
            n_az = (int) round(360.0 / del_deg * cos_el);
            if (n_az < 1) n_az = 1;
        }
        for (int i = 0; i < n_az && n < SCAN_MAX_TARGETS; ++i) {
            int idx = (direction > 0) ? i : (n_az - 1 - i);
            double az;
            if (n_az == 1) {
                az = 0.0;
            } else {
                az = (double) idx * 360.0 / (double) n_az;
                if (az >= 180.0) az -= 360.0;
            }
            // Stay strictly inside the rotator's [-179, +179] safe window.
            if (az <= -179.0) continue;
            if (az >= 179.5)  az = 179.0;
            // First sample on ring 0 is (0,0) — we already emitted it
            // as the forced starting target; skip the dup.
            if (eli == 0 && fabs(az) < 0.001 && n > 0
                && g_scan_targets[0].az_deg == 0.0
                && g_scan_targets[0].el_deg == 0.0
                && n == 1) {
                continue;
            }
            g_scan_targets[n].az_deg = az;
            g_scan_targets[n].el_deg = el;
            ++n;
        }
        direction = -direction;
    }
    g_scan_n_targets = n;
    return n;
}

static void scan_csv_open(void)
{
    if (g_scan_csv_fp != NULL) return;
    struct timeval tv;
    struct tm utc;
    if (gettimeofday(&tv, NULL) != 0 || gmtime_r(&tv.tv_sec, &utc) == NULL) {
        return;
    }
    const char *dir = g_pass_folder[0] ? g_pass_folder : ".";
    int n = snprintf(g_scan_csv_path, sizeof g_scan_csv_path,
                     "%s/scan_sky_UT=%04d%02d%02dT%02d%02d%02dZ.csv",
                     dir,
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec);
    if (n <= 0 || (size_t) n >= sizeof g_scan_csv_path) {
        g_scan_csv_path[0] = '\0';
        return;
    }
    g_scan_csv_fp = fopen(g_scan_csv_path, "w");
    if (g_scan_csv_fp == NULL) {
        g_scan_csv_path[0] = '\0';
        return;
    }
    fputs("# scan-sky log\n"
          "# unix_time_ms,target_az_deg,target_el_deg,"
          "actual_az_deg,actual_el_deg,event\n",
          g_scan_csv_fp);
    fflush(g_scan_csv_fp);
}

static void scan_csv_log(double tgt_az, double tgt_el,
                          double act_az, double act_el,
                          const char *event)
{
    if (g_scan_csv_fp == NULL) return;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long u_ms = (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000;
    fprintf(g_scan_csv_fp, "%lld,%.3f,%.3f,%.3f,%.3f,%s\n",
            u_ms, tgt_az, tgt_el, act_az, act_el,
            (event && event[0]) ? event : "");
    fflush(g_scan_csv_fp);
}

static void scan_csv_close(void)
{
    if (g_scan_csv_fp != NULL) {
        fclose(g_scan_csv_fp);
        g_scan_csv_fp = NULL;
    }
}

static void scan_sky_start(state_t *state)
{
    if (g_scan_active) return;
    if (g_scan_n_targets == 0) scan_build_targets(g_scan_step_deg);
    if (g_scan_n_targets == 0) return;
    scan_csv_open();
    g_scan_active        = 1;
    g_scan_idx           = 0;
    g_scan_dwell_start_s = 0.0;
    // Make sure no concurrent satellite-tracking logic competes for
    // the rotator.
    state->satellite_tracking         = 0;
    state->antenna_rotator.tracking   = 0;
    state->antenna_rotator.flip_mode_pass     = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half  = 0;
    // Command the first target via point_to_stationary_target so the
    // two-step homing handles wraparound shortest-path correctly.
    point_to_stationary_target(state,
                                g_scan_targets[0].az_deg,
                                g_scan_targets[0].el_deg);
    {
        char det[160];
        snprintf(det, sizeof det,
            "n_targets=%d step_deg=%.1f dwell_s=%.1f csv=\"%.100s\"",
            g_scan_n_targets, g_scan_step_deg, SCAN_DWELL_S,
            g_scan_csv_path[0] ? g_scan_csv_path : "(none)");
        sso_audit_event("scan-sky-start", det);
    }
}

static void scan_sky_stop(state_t *state, const char *reason)
{
    if (!g_scan_active) return;
    scan_csv_log(NAN, NAN,
                 state->antenna_rotator.azimuth,
                 state->antenna_rotator.elevation,
                 reason ? reason : "stop");
    scan_csv_close();
    int done_idx = g_scan_idx;
    int total    = g_scan_n_targets;
    g_scan_active        = 0;
    g_scan_idx           = 0;
    g_scan_dwell_start_s = 0.0;
    {
        char det[160];
        snprintf(det, sizeof det,
            "reason=\"%.60s\" completed=%d/%d",
            reason ? reason : "?", done_idx, total);
        sso_audit_event("scan-sky-stop", det);
    }
}

// Drive the scan state machine. Called once per tick of the main loop
// while g_scan_active is 1.
static void scan_sky_tick(state_t *state, double t_now)
{
    if (!g_scan_active) return;
    if (g_scan_idx >= g_scan_n_targets) {
        scan_sky_stop(state, "complete");
        return;
    }
    // Wait for the rotator to settle before dwelling.
    if (state->antenna_rotator.antenna_is_moving) return;
    if (g_scan_dwell_start_s <= 0.0) {
        g_scan_dwell_start_s = t_now;
        const scan_target_t *t = &g_scan_targets[g_scan_idx];
        scan_csv_log(t->az_deg, t->el_deg,
                     state->antenna_rotator.azimuth,
                     state->antenna_rotator.elevation,
                     "arrived");
        return;
    }
    if (t_now - g_scan_dwell_start_s < SCAN_DWELL_S) return;
    // Dwell expired — advance to the next target.
    ++g_scan_idx;
    g_scan_dwell_start_s = 0.0;
    if (g_scan_idx >= g_scan_n_targets) {
        scan_sky_stop(state, "complete");
        return;
    }
    const scan_target_t *t = &g_scan_targets[g_scan_idx];
    point_to_stationary_target(state, t->az_deg, t->el_deg);
}

void update_doppler_shifted_frequencies(state_t *state,
                                          double uplink_freq,
                                          double downlink_freq)
{
    double doppler_factor = 1.0
        - state->prediction.satellite_ephem.range_rate_km_s / 299792.458;
    state->doppler_uplink_frequency_hz   = uplink_freq   * doppler_factor;
    state->doppler_downlink_frequency_hz = downlink_freq * doppler_factor;
}
