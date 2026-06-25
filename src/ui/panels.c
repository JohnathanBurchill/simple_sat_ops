/*

   Simple Satellite Operations  ui/panels.c

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

#include "panels.h"
#include "state.h"

#include "antenna_rotator_async.h"
#include "beacon_cts1.h"     // RX_RS_FAIL_TOKEN -- the uncorrectable-RS marker
#include "duration_fmt.h"
#include "pass_session.h"
#include "prediction.h"
#include "sso_ipc.h"
#include "sso_time.h"

#include <math.h>
#include <ncurses.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <time.h>

#ifdef SSO_WITH_SDR
#include "rx_session.h"
#endif

// Low-disk warning thresholds (free_disk_bytes / low_disk_refresh).
#define LOW_DISK_BYTES   ((uint64_t)10 * 1000 * 1000 * 1000)
#define LOW_DISK_PERIOD_S 30.0

// Warn in the predictions panel when the TLE epoch is older than this.
#define WARN_DAYS_SINCE_EPOCH 1.0

//
// Ribbon tick semantics: every 20 seconds in absolute push-time gets a
// bold '-' instead of '.'. Because the tick is keyed to absolute push
// count (not a fixed visual position), it crawls upward by one row per
// second — the eye reads the timeline progressing even when the signal
// is flat.

void ribbon_push(ui_t *ui, double peak_dbfs, int bright_bins)
{
    ui->ribbon_peak[ui->ribbon_head]   = peak_dbfs;
    ui->ribbon_bright[ui->ribbon_head] = bright_bins;
    ui->ribbon_head = (ui->ribbon_head + 1) % RIBBON_LEN;
    if (ui->ribbon_count < RIBBON_LEN) ui->ribbon_count++;
    ui->ribbon_push_count++;
}

// Low-disk warning + the pass output folder now live on state_t
// (state.op.low_disk_msg / state.op.low_disk_last_t / state.op.pass_folder).

// Returns bytes available to a non-privileged user on the filesystem
// hosting `path`. Returns (uint64_t) -1 on error or when path is empty.
static uint64_t free_disk_bytes(const char *path)
{
    if (path == NULL || path[0] == '\0') return (uint64_t) -1;
    struct statvfs s;
    if (statvfs(path, &s) != 0) return (uint64_t) -1;
    return (uint64_t) s.f_bavail * (uint64_t) s.f_frsize;
}

// Refresh state->op.low_disk_msg if the period has elapsed. Empty message
// means "above threshold" — render_rx_panel skips the row in that case.
void low_disk_refresh(op_t *op, double t_now)
{
    if ((t_now - op->low_disk_last_t) < LOW_DISK_PERIOD_S
        && op->low_disk_last_t != 0.0) return;
    op->low_disk_last_t = t_now;
    const char *probe = op->pass_folder[0] ? op->pass_folder : ".";
    uint64_t avail = free_disk_bytes(probe);
    if (avail == (uint64_t) -1) {
        op->low_disk_msg[0] = '\0';
        return;
    }
    if (avail >= LOW_DISK_BYTES) {
        op->low_disk_msg[0] = '\0';
        return;
    }
    double gb = (double) avail / 1.0e9;
    // Cap path width so the message fits in the 80-byte buffer: prefix
    // is ~26 chars ("LOW DISK: 12.34 GB free at "), leaving 50 for the
    // path. GCC -Wformat-truncation would otherwise flag the unbounded
    // %s against a 255-byte op->pass_folder.
    snprintf(op->low_disk_msg, sizeof op->low_disk_msg,
             "LOW DISK: %.2f GB free at %.50s", gb, probe);
}

// Render the TX log at rows [start_row .. start_row + (TX_LOG_SIZE+1)).
// Caller picks the column. Title line + one row per entry. Newest at
// the bottom; PREVIEW lines render with A_BOLD, SENT/NOT_SENT with A_DIM.
void render_tx_log_panel(const tx_t *tx, int start_row, int col)
{
    int row = start_row;
    // Cap width so clrtoeol-equivalent padding doesn't wipe the
    // vertical ribbon (rendered at col COLS-2). Leave a 1-col gap.
    int safe_w = COLS - col - 3;
    if (safe_w < 8)   safe_w = 8;
    if (safe_w > 200) safe_w = 200;

    mvprintw(row++, col, "%-*.*s", safe_w, safe_w, "TX log");

    for (size_t i = 0; i < tx->tx_log_count; ++i) {
        const tx_log_entry_t *e = &tx->tx_log[i];
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
        // Origin tag ("auto-cmd (file)" / "manual send") prefixes the
        // command on sent / not-sent rows; previews carry no source.
        char src[20] = "";
        if (e->source[0]) snprintf(src, sizeof src, "[%s] ", e->source);
        char line[SSO_TX_TEXT_MAX + 96];
        if (e->kind == SSO_EVT_TX_NOT_SENT && e->tx_not_sent_reason[0]) {
            snprintf(line, sizeof line, "%s  %s %s%s  [%s]",
                     e->ts, tag, src, e->ascii, e->tx_not_sent_reason);
        } else {
            snprintf(line, sizeof line, "%s  %s %s%s",
                     e->ts, tag, src, e->ascii);
        }
        attron(attr);
        mvprintw(row++, col, "%-*.*s", safe_w, safe_w, line);
        attroff(attr);
    }
}

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

// Operator-side collector. Reads the live rx_session + state.ribbon_*
// fields into the struct. On non-B210 builds, only have_session=0 is filled.
void rx_panel_collect_local(state_t *state, rx_panel_data_t *d)
{
    memset(d, 0, sizeof *d);
#ifdef SSO_WITH_SDR
    d->have_session = (state->sdr.rx_session != NULL);
    if (!d->have_session) return;
    d->rec_active = rx_session_wav_active(state->sdr.rx_session);
    snprintf(d->sdr_name, sizeof d->sdr_name, "%.31s",
             rx_session_sdr_name(state->sdr.rx_session));
    d->can_tx = rx_session_can_tx(state->sdr.rx_session);
    char last[sizeof d->last_frame_summary] = "";
    rx_session_snapshot(state->sdr.rx_session,
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
    d->rx_lo_hz        = rx_session_get_lo_freq_hz(state->sdr.rx_session);
    d->rx_bandwidth_hz = rx_session_get_bandwidth_hz(state->sdr.rx_session);
    d->frames_pcm = rx_session_pcm_frames(state->sdr.rx_session);
    d->frames_vit = rx_session_viterbi_frames(state->sdr.rx_session);
    rx_packet_type_stats_t pts[RX_PT_COUNT];
    rx_session_stats_snapshot(state->sdr.rx_session, pts, &d->age_s);
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
    int n = state->ui.ribbon_count;
    if (n > (int) sizeof d->ribbon - 1) n = (int) sizeof d->ribbon - 1;
    long P = state->ui.ribbon_push_count;
    // Bright-bin thresholds for ribbon character selection. With
    // iq_burst's 10 dB / N=512 FFT, stationary noise lights ~30 bins
    // and a CW carrier adds ~5-6 more. A wideband packet burst lights
    // 80+. Pick BRIGHT_HI well above the noise floor so '#' fires
    // only on real broadband events.
    const int BRIGHT_HI = 80;   // broadband — '#'
    for (int i = 0; i < n; ++i) {
        long abs_t = P - (long) i;
        int idx = (state->ui.ribbon_head - 1 - i + RIBBON_LEN) % RIBBON_LEN;
        int bright = state->ui.ribbon_bright[idx];
        if (bright >= BRIGHT_HI)             d->ribbon[i] = '#';
        else if (abs_t > 0 && (abs_t % 20) == 0) d->ribbon[i] = '_';
        else                                 d->ribbon[i] = '.';
        double dbfs = state->ui.ribbon_peak[idx];
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
    snprintf(d->warning, sizeof d->warning, "%s", state->op.low_disk_msg);
#ifdef SSO_WITH_SDR
    // A lost SDR outranks the low-disk notice — show it instead so the
    // operator sees immediately that RX has stopped.
    if (d->have_session && rx_session_device_lost(state->sdr.rx_session)) {
        snprintf(d->warning, sizeof d->warning,
                 "SDR DISCONNECTED - RX stopped (restart to resume RX)");
    }
#endif
}

// Render the RX panel from a snapshot. Compiles even without UHD —
// the viewer feeds this from broadcast STATE so it can draw what the
// operator sees.
void render_rx_panel(const rx_panel_data_t *d,
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
    // Doppler tracking — see state.sdr.rx_lo_offset_hz.
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
            // An uncorrectable-RS frame carries the RS-FAIL marker instead
            // of telemetry. Strip the lead token and draw the message in
            // bold red so the operator can't mistake it for a valid decode.
            size_t tok = strlen(RX_RS_FAIL_TOKEN);
            if (strncmp(d->pt_summary[s], RX_RS_FAIL_TOKEN, tok) == 0) {
                attron(COLOR_PAIR(1) | A_BOLD);
                mvprintw(row++, col, "%15s   %s",
                         rx_panel_pt_label(s), d->pt_summary[s] + tok);
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else {
                mvprintw(row++, col, "%15s   %s",
                         rx_panel_pt_label(s), d->pt_summary[s]);
            }
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
void render_ribbon_vertical(const rx_panel_data_t *d,
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

// Pure-render predictions panel — operator runs the SGP4 search
// upstream, viewer fills state.track.prediction from broadcast. No SGP4
// calls inside, no current-time reads, so both sides paint the same
// thing for the same input state.
void render_predictions_panel(state_t *state, double jul_utc,
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
             state->track.prediction.satellite_ephem.tle.sat_name,
             state->track.prediction.satellite_ephem.tle.idesg);
    clrtoeol();
    if (state->track.prediction.minutes_since_epoch / 1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attron(COLOR_PAIR(2));
    }
    char epoch_age[40];
    format_age_compact(state->track.prediction.minutes_since_epoch * 60.0,
                       epoch_age, sizeof epoch_age);
    mvprintw(row++, col, "%15s   %s", "epoch age", epoch_age);
    if (state->track.prediction.minutes_since_epoch / 1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attroff(COLOR_PAIR(2));
    }
    clrtoeol();

    // "status" is the satellite's true geometry: is it above the local
    // horizon right now? This used to key off state->track.in_pass, but that is
    // really a rotator pre-position flag (set only when tracking is armed
    // AND within tracking_prep_time_minutes of AOS), so it read "NOT in
    // pass" with the satellite plainly overhead whenever tracking wasn't
    // armed -- and disagreed with the viewer, which keys off elevation. The
    // rotator's own state now lives on its own line below.
    double live_el = state->track.prediction.satellite_ephem.elevation;
    if (live_el > 0.0) {
        attron(COLOR_PAIR(3));
        mvprintw(row++, col, "%15s   ** SATELLITE UP **",
                 "status");
        attroff(COLOR_PAIR(3));
    } else {
        mvprintw(row++, col, "%15s   below horizon",
                 "status");
    }
    clrtoeol();

    // Rotator state, decoupled from the satellite geometry above -- an
    // explicit statement of whether the antenna is actually following the
    // satellite, so the operator doesn't have to infer it from the raw
    // az/el numbers. Only shown when a rotator is attached.
    if (state->rot.have_antenna_rotator) {
        char rot[96];
        int rot_color = 0;
        if (state->rot.antenna_rotator.homing_in_progress) {
            snprintf(rot, sizeof rot, "resetting");
            rot_color = 2;
        } else if (state->rot.antenna_rotator.tracking) {
            snprintf(rot, sizeof rot, "TRACKING satellite");
            rot_color = 3;
        } else if (state->track.satellite_tracking) {
            snprintf(rot, sizeof rot, "armed (will track at AOS)");
            rot_color = 2;
        } else if (state->rot.antenna_rotator.fixed_target) {
            snprintf(rot, sizeof rot, "fixed target (not tracking)");
        } else {
            snprintf(rot, sizeof rot, "not tracking");
        }
        if (rot_color) attron(COLOR_PAIR(rot_color));
        mvprintw(row++, col, "%15s   %s", "rotator", rot);
        if (rot_color) attroff(COLOR_PAIR(rot_color));
        clrtoeol();
    }

    if (state->track.prediction.predicted_minutes_until_visible > 0) {
        double minutes_until = state->track.prediction.predicted_minutes_until_visible;
        time_t aos_t = time(NULL) + (time_t)(minutes_until * 60.0);
        struct tm aos_local;
        localtime_r(&aos_t, &aos_local);
        char aos_hhmm[8];
        snprintf(aos_hhmm, sizeof aos_hhmm, "%02d:%02d",
                 aos_local.tm_hour, aos_local.tm_min);
        char until[32];
        format_duration_compact(minutes_until * 60.0, until, sizeof until);
        if (minutes_until < 1) {
            mvprintw(row++, col, "%15s   %s ", "next pass", aos_hhmm);
            attron(COLOR_PAIR(2));
            printw("(in %s)", until);
            attroff(COLOR_PAIR(2));
        } else {
            mvprintw(row++, col, "%15s   %s (in %s)", "next pass",
                     aos_hhmm, until);
        }
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   ", "elapsed time");
        attron(COLOR_PAIR(3));
        if (fabs(state->track.prediction.predicted_minutes_until_visible) < 1) {
            printw("%.0f seconds",
                   floor(-state->track.prediction.predicted_minutes_until_visible * 60.0));
        } else {
            printw("%.1f minutes",
                   -state->track.prediction.predicted_minutes_until_visible);
        }
        attroff(COLOR_PAIR(3));
        clrtoeol();
    }
    mvprintw(row++, col, "%15s   %.1f minutes", "duration",
             state->track.prediction.predicted_minutes_above_0_degrees);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f minutes", "el>30",
             state->track.prediction.predicted_minutes_above_30_degrees);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg", "max elevation",
             state->track.prediction.predicted_max_elevation);
    clrtoeol();

    *print_row = row;
}

// SGP4 work that report_predictions used to do inline. Operator calls
// this each tick so its state->track.prediction.predicted_* fields are
// fresh before render + broadcast.
void compute_predictions(track_t *track, double jul_utc)
{
    minutes_until_visible(&track->prediction, jul_utc,
                          jul_utc + MAX_MINUTES_TO_PREDICT / 1440.0, 1.0);
    if (fabs(track->prediction.predicted_minutes_until_visible) < 1) {
        minutes_until_visible(&track->prediction, jul_utc,
                              jul_utc + 2.0 / 1440.0, 1. / 120.0);
    } else if (fabs(track->prediction.predicted_minutes_until_visible) < 10) {
        minutes_until_visible(&track->prediction, jul_utc,
                              jul_utc + 20.0 / 1440.0, 0.1);
    }
    if (track->prediction.predicted_minutes_until_visible > 0) {
        update_pass_predictions(&track->prediction,
            jul_utc + track->prediction.predicted_minutes_until_visible / 1440.0,
            0.1);
    } else if (track->prediction.predicted_max_elevation == -180.0) {
        // Started mid-pass: walk back to AOS so update_pass_predictions
        // captures the true max elevation rather than just the remainder.
        update_pass_predictions(&track->prediction,
            jul_utc + track->prediction.predicted_minutes_until_visible / 1440.0,
            0.1);
    }
}

void report_predictions(state_t *state, double jul_utc, int *print_row, int print_col)
{
    compute_predictions(&state->track, jul_utc);
    render_predictions_panel(state, jul_utc, print_row, print_col);
}

void render_status_panel(const status_panel_t *p,
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
        case HMAC_DISPLAY_BAD:     tag = "(BAD - see --self-test)"; break;
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
static void operator_viewers_list(state_t *state, char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    if (!state->op.ipc) return;
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char user[64], role[16], since[40];
    size_t written = 0;
    while (sso_ipc_server_next_client(state->op.ipc, &it, &cid,
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

void report_status(state_t *state, int *print_row, int print_col)
{
    if (print_row == NULL) return;

    static char viewers[256];
    operator_viewers_list(state, viewers, sizeof viewers);

    status_panel_t p;
    memset(&p, 0, sizeof p);
    p.control_mode  = state->app.control_mode;
    p.operator_user = state->op.operator_user;
    p.viewers       = viewers[0] ? viewers : "(none)";

    double display_dl_hz = state->track.doppler_downlink_frequency_hz;
    if (display_dl_hz == 0.0) display_dl_hz = state->track.nominal_downlink_frequency_hz;
    p.carrier_hz = display_dl_hz;

    p.hmac_path   = state->tx.hmac_keyfile_path;
    p.hmac_status = state->tx.hmac_display_status;
    p.hmac_bytes  = (ssize_t) state->tx.hmac_key_len;

    p.have_rotator = state->rot.have_antenna_rotator;
    if (state->rot.have_antenna_rotator) {
        // The serial roundtrip moved to a worker thread (see
        // src/hw/antenna_rotator_async.c); we just read the latest
        // snapshot here. No more 5-10 ms per redraw on the main loop,
        // and no 500 ms VTIME hang if the cable is unplugged.
        double azimuth = state->rot.antenna_rotator.azimuth;
        double elevation = state->rot.antenna_rotator.elevation;
        int    rot_ok = 0;
        int    rot_stale_ms = 0;
        if (state->rot.rot_async != NULL) {
            antenna_rotator_async_snapshot(state->rot.rot_async,
                                            &azimuth, &elevation,
                                            &rot_ok, &rot_stale_ms, NULL);
            // Cache the snapshot back into state so other code (the
            // antenna_is_moving heuristic, IPC broadcast, etc.) reads a
            // single consistent value across the tick.
            state->rot.antenna_rotator.azimuth   = azimuth;
            state->rot.antenna_rotator.elevation = elevation;
        }
        p.current_az = azimuth;
        p.current_el = elevation;
        p.target_az  = state->rot.antenna_rotator.target_azimuth;
        p.target_el  = state->rot.antenna_rotator.target_elevation;
        p.flip       = state->rot.antenna_rotator.flip_mode_pass;
    }

    // T/R switch block — operator-only (this process owns the serial
    // link). The viewer never sets tr_show, so its panel skips it.
    p.tr_show = 1;
    p.tr_connected = state->trsw.have_tr_switch;
    if (state->trsw.have_tr_switch) {
        p.tr_device        = state->trsw.tr_switch.device_filename;
        p.tr_state         = state->trsw.tr_switch.state_str;
        p.tr_mode          = state->trsw.tr_switch.mode_str;
        p.tr_last_tx_ago_s = state->trsw.tr_switch.last_tx_ago_s;
        p.tr_stale         = tr_switch_is_stale(&state->trsw.tr_switch,
                                                monotonic_seconds());
    }

    render_status_panel(&p, print_row, print_col);
}

void report_position(track_t *track, int *print_row, int print_col)
{
    if (print_row == NULL) {
        return;
    }
    int row = *print_row;
    int col = print_col;

    mvprintw(row++, col, "%15s   %.2f deg", "azimuth",
             track->prediction.satellite_ephem.azimuth);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f deg", "elevation",
             track->prediction.satellite_ephem.elevation);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km", "altitude",
             track->prediction.satellite_ephem.altitude_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg N", "latitude",
             track->prediction.satellite_ephem.latitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg E", "longitude",
             track->prediction.satellite_ephem.longitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "speed",
             track->prediction.satellite_ephem.speed_km_s);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f km", "range",
             track->prediction.satellite_ephem.range_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "range rate",
             track->prediction.satellite_ephem.range_rate_km_s);
    clrtoeol();

    *print_row = row;
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
void draw_box(WINDOW *w) {
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

// Tiny CSI fallback parser for terminals where ncurses' keypad mode
// doesn't translate arrow / nav keys into KEY_* (notably some tmux
// configurations). Same idea as cmd_drain_csi in the `:` prompt.
// Returns a KEY_* code on success, or -1 if the lookahead isn't a
// CSI we recognise.
int tx_drain_csi(WINDOW *w) {
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


// Paint the operator's whole-screen layout for one redraw tick: the
// predictions / status / position columns, the lazy low-disk refresh, the
// RX panel, the right-edge signal ribbon, and (if the terminal is tall
// enough below the keyboard-info rows) the TX log. keyboard_info_row is the
// first keyboard-help row, used to decide whether the TX log fits.
void render_operator_screen(state_t *state, double jul_utc, double t_now,
                            int keyboard_info_row)
{
    int row = 1;
    int col = 1;
    report_predictions(state, jul_utc, &row, col);

    row++;
    report_status(state, &row, col);
    row = 5;
    col = 50;
    report_position(&state->track, &row, col);
    row++;
    // Refresh the low-disk warning lazily -- statvfs every 30 s is plenty
    // given how slowly disk fills.
    low_disk_refresh(&state->op, t_now);
    rx_panel_data_t rxd;
    rx_panel_collect_local(state, &rxd);
    render_rx_panel(&rxd, &row, 50);

    clrtoeol();

    // Vertical ribbon on the right edge -- bottom = newest, with a bold '-'
    // tick crawling up one row per second so the timeline is visibly alive
    // even when the signal is flat.
    int ribbon_col = COLS - 2;
    int ribbon_top = 1;
    int ribbon_bot = LINES - 2;
    if (ribbon_col >= 64 && ribbon_bot > ribbon_top) {
        render_ribbon_vertical(&rxd, ribbon_top, ribbon_bot, ribbon_col);
    }

    // TX log lives below the keyboard info / antenna status if the terminal
    // is tall enough to host it without colliding.
    int tx_log_row = LINES - TX_LOG_SIZE - 2;
    if (tx_log_row >= keyboard_info_row + 4) {
        render_tx_log_panel(&state->tx, tx_log_row, 1);
    }
}
