/*

    Simple Satellite Operations  rx_tui.c

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

#include "rx_tui.h"

#include <stdio.h>
#include <string.h>

#ifdef RX_TUI_AVAILABLE

#include "beacon_cts1.h"
#include "csp.h"

#include <ncurses.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

// ---- module state -----------------------------------------------------

static int  g_init = 0;
static int  g_have_color = 0;
static char g_header[256] = "rx_tui";
// Optional second title-bar row. Hidden (row collapsed) when empty —
// rx_live / rx_replay never call rx_tui_set_status so they keep their
// existing layout. b210_rx_live drives this with the live Doppler
// frequency so the operator always sees what the SDR is tuned to.
static char g_status[256] = "";

// Optional REPL input row + command handler. Active only when a
// caller has registered a handler via rx_tui_set_command_handler.
// The line editor lives entirely inside rx_tui — callers just see a
// fully-typed-and-Enter'd command via the callback.
typedef rx_tui_command_fn rx_tui_command_fn_t;
static rx_tui_command_fn_t g_cmd_fn  = NULL;
static void               *g_cmd_ctx = NULL;

#define RX_TUI_CMDLINE_MAX  255
#define RX_TUI_HIST_RING    64

static char  g_cmdline[RX_TUI_CMDLINE_MAX + 1] = "";
static size_t g_cmd_len    = 0;
static size_t g_cmd_cursor = 0;
// Buffer the operator was editing when they pressed Up — restored on
// Down past the newest history entry so the typed-but-unsubmitted
// line isn't lost.
static char  g_cmd_saved[RX_TUI_CMDLINE_MAX + 1] = "";

static char  g_hist[RX_TUI_HIST_RING][RX_TUI_CMDLINE_MAX + 1];
static int   g_hist_count    = 0;   // entries in ring (≤ RING)
static int   g_hist_head     = 0;   // index of OLDEST entry (when full)
static int   g_hist_view_idx = -1;  // -1 = editing fresh; else 0..count-1

static char  g_hist_path[300] = "";

static const char *hist_at(int view_idx)
{
    if (view_idx < 0 || view_idx >= g_hist_count) return NULL;
    int slot = (g_hist_head + view_idx) % RX_TUI_HIST_RING;
    return g_hist[slot];
}

// Append a command to the in-memory ring (de-duplicating against the
// most recent entry) and to the persistent file if set.
static void hist_append(const char *cmd, int persist)
{
    if (cmd == NULL || cmd[0] == '\0') return;
    if (g_hist_count > 0) {
        const char *newest = hist_at(g_hist_count - 1);
        if (newest != NULL && strcmp(newest, cmd) == 0) return;
    }
    int slot;
    if (g_hist_count < RX_TUI_HIST_RING) {
        slot = (g_hist_head + g_hist_count) % RX_TUI_HIST_RING;
        g_hist_count++;
    } else {
        slot = g_hist_head;
        g_hist_head = (g_hist_head + 1) % RX_TUI_HIST_RING;
    }
    snprintf(g_hist[slot], sizeof g_hist[slot], "%s", cmd);
    if (persist && g_hist_path[0] != '\0') {
        FILE *fp = fopen(g_hist_path, "a");
        if (fp != NULL) {
            fprintf(fp, "%s\n", cmd);
            fclose(fp);
        }
    }
}

static void hist_load_from_disk(void)
{
    if (g_hist_path[0] == '\0') return;
    FILE *fp = fopen(g_hist_path, "r");
    if (fp == NULL) return;
    char line[RX_TUI_CMDLINE_MAX + 2];
    while (fgets(line, sizeof line, fp) != NULL) {
        size_t l = strlen(line);
        while (l > 0 && (line[l - 1] == '\n' || line[l - 1] == '\r')) {
            line[--l] = '\0';
        }
        if (l > 0) hist_append(line, /*persist=*/0);
    }
    fclose(fp);
}
// volatile sig_atomic_t so callers may flip this from a signal handler
// via rx_tui_request_quit() without UB.
static volatile sig_atomic_t g_quit_requested = 0;

// Latest beacon (memcpy'd into a stack-aligned struct on observe).
static int g_have_beacon = 0;
static COMMS_beacon_basic_packet_t g_beacon;
static char g_beacon_ts[40];

// Per-frame monotonic clock so the title-bar "age" value keeps ticking
// even between observed frames.
static struct timespec g_last_frame_clock;
static int g_have_any_frame = 0;

// TCMD ring buffer. Newest at head, older indices walk backward.
typedef struct {
    char     ts[40];
    uint8_t  code;
    uint16_t duration_ms;
    uint8_t  seq;
    uint8_t  max_seq;
    size_t   data_len;
    char     msg[160];   // already-sanitised, NUL-terminated
    int      truncated;  // 1 if data was longer than msg[]
} tcmd_entry_t;

enum { TCMD_RING_SZ = 64 };
static tcmd_entry_t g_tcmd_ring[TCMD_RING_SZ];
static int g_tcmd_head = 0;       // index of newest entry
static int g_tcmd_count = 0;

// OTHER ring buffer — frames the dispatch couldn't classify as a CTS1
// beacon or a tcmd response. Captures bytes on the wire that are
// otherwise invisible to the operator (length-mismatched beacons,
// log-message packets, bulk-file packets, garbage frames whose CSP
// header decoded but payload doesn't fit either schema).
typedef struct {
    char     ts[40];
    size_t   payload_len;
    int      csp_ok;
    uint8_t  csp_src, csp_dst, csp_dport, csp_sport;
    int      rs_errs;     // -2 UNCORRECTABLE, -1 off, >=0 corrected count
    int      crc_status;  // -1 not checked, 0 mismatch, 1 ok
    char     preview[160]; // sanitised, NUL-terminated
    int      truncated;
} other_entry_t;

enum { OTHER_RING_SZ = 64 };
static other_entry_t g_other_ring[OTHER_RING_SZ];
static int g_other_head = 0;
static int g_other_count = 0;

// Frame counters.
static struct {
    uint64_t total;
    uint64_t beacon;
    uint64_t tcmd;
    uint64_t other;
    uint64_t hmac_mismatch;
    uint64_t rs_uncorrectable;
    uint64_t crc_mismatch;
} g_counters;

// Color pairs (only used when has_colors()).
enum {
    PAIR_TITLE = 1,
    PAIR_LABEL = 2,
    PAIR_OK    = 3,
    PAIR_WARN  = 4,
    PAIR_ALERT = 5,
};

// ---- helpers ----------------------------------------------------------

static const char *state_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case CTS1_OPERATION_STATE_BOOTED_AND_WAITING:       return "BOOTING";
        case CTS1_OPERATION_STATE_DEPLOYING:                return "DEPLOYING";
        case CTS1_OPERATION_STATE_NOMINAL_WITH_RADIO_TX:    return "NOMINAL_TX";
        case CTS1_OPERATION_STATE_NOMINAL_WITHOUT_RADIO_TX: return "NOMINAL_NO_TX";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

static const char *eps_mode_str(uint8_t v, char *buf, size_t bufn)
{
    switch (v) {
        case 0: return "STARTUP";
        case 1: return "NOMINAL";
        case 2: return "SAFETY";
        case 3: return "EMERGENCY";
    }
    snprintf(buf, bufn, "%u", v);
    return buf;
}

static void fmt_ms_clock(uint64_t ms_in, char *out, size_t outn)
{
    uint64_t total_s = ms_in / 1000;
    uint64_t ms = ms_in % 1000;
    uint64_t s = total_s % 60;
    uint64_t m = (total_s / 60) % 60;
    uint64_t h = total_s / 3600;
    snprintf(out, outn, "%02llu:%02llu:%02llu.%03llu",
             (unsigned long long)h, (unsigned long long)m,
             (unsigned long long)s, (unsigned long long)ms);
}

static void fmt_epoch_ms(uint64_t ms_in, char *out, size_t outn)
{
    time_t t = (time_t)(ms_in / 1000);
    struct tm tm;
    if (gmtime_r(&t, &tm) == NULL) { snprintf(out, outn, "?"); return; }
    strftime(out, outn, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

// Pad-or-truncate write at (row, col) honouring a right edge.
static void put_line(int row, int col, int right_edge, const char *s)
{
    if (row < 0 || col < 0 || col >= right_edge) return;
    int max = right_edge - col;
    mvaddnstr(row, col, s, max);
}

static void put_label(int row, int col, int right_edge, const char *s)
{
    if (g_have_color) attron(COLOR_PAIR(PAIR_LABEL) | A_BOLD);
    else              attron(A_BOLD);
    put_line(row, col, right_edge, s);
    if (g_have_color) attroff(COLOR_PAIR(PAIR_LABEL) | A_BOLD);
    else              attroff(A_BOLD);
}

// Battery-voltage colour band. Thresholds chosen to flag mid-discharge
// before the EPS itself escalates: < 2.7V red (cell knee), 2.7..2.9V
// yellow (degrading), >= 2.9V green. Per-cell numbers since the firmware
// reports the pack voltage divided by series count.
static int batt_voltage_pair(double v)
{
    if (v < 2.7) return PAIR_ALERT;
    if (v < 2.9) return PAIR_WARN;
    return PAIR_OK;
}

static int batt_percent_pair(unsigned p)
{
    if (p < 30) return PAIR_ALERT;
    if (p < 50) return PAIR_WARN;
    return PAIR_OK;
}

// Battery / OBC temperature thresholds. Spacecraft Li-ion cells like to
// see -10..+40 C; below -10 or above 50 is an alert.
static int temp_c_pair(double c)
{
    if (c < -10.0 || c > 50.0) return PAIR_ALERT;
    if (c <   0.0 || c > 40.0) return PAIR_WARN;
    return PAIR_OK;
}

static int faults_pair(int n)
{
    if (n > 0) return PAIR_ALERT;
    return PAIR_OK;
}

static int code_pair(uint8_t code)
{
    return (code == 0) ? PAIR_OK : PAIR_ALERT;
}

static void color_on(int pair)
{
    if (g_have_color) attron(COLOR_PAIR(pair));
    else if (pair == PAIR_ALERT) attron(A_BOLD);
}
static void color_off(int pair)
{
    if (g_have_color) attroff(COLOR_PAIR(pair));
    else if (pair == PAIR_ALERT) attroff(A_BOLD);
}

// ---- panels -----------------------------------------------------------

// Returns the next row to draw on. Caller passes (top, left, height, width)
// of the panel rectangle; we bound everything inside it.
static int draw_beacon_panel(int top, int left, int rows, int cols)
{
    int right = left + cols;
    int row   = top;
    int max   = top + rows;

    put_label(row, left, right, "BEACON");
    row++;

    if (!g_have_beacon) {
        if (row < max) put_line(row + 1, left + 2, right, "(no beacon received yet)");
        return max;
    }

    char buf[300];
    char state_buf[16], eps_buf[16];
    const char *state = state_str(g_beacon.cts1_operation_state,
                                  state_buf, sizeof state_buf);
    const char *eps_m = eps_mode_str(g_beacon.eps_mode_enum,
                                     eps_buf, sizeof eps_buf);

    if (row < max) {
        snprintf(buf, sizeof buf,
                 "  name=%.4s  state=%s  eps_mode=%s  count=%u  mounted=%u",
                 g_beacon.satellite_name, state, eps_m,
                 (unsigned)g_beacon.total_beacon_count_since_boot,
                 (unsigned)g_beacon.is_fs_mounted);
        put_line(row++, left, right, buf);
    }
    if (row < max) {
        snprintf(buf, sizeof buf, "  rx_at=%s", g_beacon_ts);
        put_line(row++, left, right, buf);
    }

    if (row < max) put_label(row++, left + 2, right, "Power");
    if (row < max) {
        double v = g_beacon.eps_battery_voltage_mV / 1000.0;
        unsigned pct = (unsigned)g_beacon.eps_battery_percent;
        // Voltage and percent each get their own colour band; render
        // the whole line plain then re-overlay the coloured tokens.
        snprintf(buf, sizeof buf,
                 "    batt=%.3fV %u%%  in/out=%.2fW/%.2fW  avg=%.2fW/%.2fW",
                 v, pct,
                 g_beacon.eps_total_pcu_power_input_cW / 100.0,
                 g_beacon.eps_total_pcu_power_output_cW / 100.0,
                 g_beacon.eps_total_avg_pcu_power_input_cW / 100.0,
                 g_beacon.eps_total_avg_pcu_power_output_cW / 100.0);
        put_line(row, left, right, buf);
        // Re-paint the "X.XXXV YY%" segment with thresholded colour.
        char color_buf[40];
        int color_col = left + 4 + 5;  // after "    batt="
        snprintf(color_buf, sizeof color_buf, "%.3fV", v);
        int vp = batt_voltage_pair(v);
        color_on(vp);
        put_line(row, color_col, right, color_buf);
        color_off(vp);
        int pct_col = color_col + (int)strlen(color_buf) + 1;
        snprintf(color_buf, sizeof color_buf, "%u%%", pct);
        int pp = batt_percent_pair(pct);
        color_on(pp);
        put_line(row, pct_col, right, color_buf);
        color_off(pp);
        row++;
    }
    if (row < max) {
        int faults = (int)g_beacon.eps_total_fault_count;
        snprintf(buf, sizeof buf, "    faults=%d  channels=0x%x",
                 faults, (unsigned)g_beacon.eps_enabled_channels_bitfield);
        put_line(row, left, right, buf);
        // Recolour the "faults=N" token if non-zero.
        int fp = faults_pair(faults);
        if (fp != PAIR_OK) {
            char fb[32];
            snprintf(fb, sizeof fb, "faults=%d", faults);
            color_on(fp);
            put_line(row, left + 4, right, fb);
            color_off(fp);
        }
        row++;
    }

    if (row < max) put_label(row++, left + 2, right, "Thermal");
    if (row < max) {
        double t0 = g_beacon.eps_battery_temperature_0_cC / 100.0;
        double t1 = g_beacon.eps_battery_temperature_1_cC / 100.0;
        double tobc = g_beacon.obc_temperature_cC / 100.0;
        snprintf(buf, sizeof buf,
                 "    bat=%.2fC/%.2fC  obc=%.2fC", t0, t1, tobc);
        put_line(row, left, right, buf);
        // Recolour each temp token if out of band.
        int p0 = temp_c_pair(t0), p1 = temp_c_pair(t1), pobc = temp_c_pair(tobc);
        if (p0 != PAIR_OK) {
            char tb[16]; snprintf(tb, sizeof tb, "%.2fC", t0);
            color_on(p0);  put_line(row, left + 4 + 4, right, tb);  color_off(p0);
        }
        if (p1 != PAIR_OK) {
            // "    bat=X.XXC/" — find offset of second number
            char tb0[16]; snprintf(tb0, sizeof tb0, "%.2fC", t0);
            int col1 = left + 4 + 4 + (int)strlen(tb0) + 1;
            char tb1[16]; snprintf(tb1, sizeof tb1, "%.2fC", t1);
            color_on(p1);  put_line(row, col1, right, tb1);  color_off(p1);
        }
        if (pobc != PAIR_OK) {
            char tbobc[16]; snprintf(tbobc, sizeof tbobc, "%.2fC", tobc);
            // Crude: search the rendered buffer for "obc=" + value
            const char *p = strstr(buf, "obc=");
            if (p != NULL) {
                int col = left + (int)(p - buf) + 4;
                color_on(pobc); put_line(row, col, right, tbobc); color_off(pobc);
            }
        }
        row++;
    }

    if (row < max) put_label(row++, left + 2, right, "Time");
    if (row < max) {
        char up[24], su[24];
        fmt_ms_clock(g_beacon.uptime_ms, up, sizeof up);
        fmt_ms_clock(g_beacon.duration_since_last_uplink_ms, su, sizeof su);
        snprintf(buf, sizeof buf,
                 "    uptime=%s  since_uplink=%s", up, su);
        put_line(row++, left, right, buf);
    }
    if (row < max) {
        char ep[32];
        fmt_epoch_ms(g_beacon.unix_epoch_time_ms, ep, sizeof ep);
        snprintf(buf, sizeof buf, "    epoch=%s", ep);
        put_line(row++, left, right, buf);
    }

    if (row < max) put_label(row++, left + 2, right, "Antenna / TCMD");
    if (row < max) {
        snprintf(buf, sizeof buf,
                 "    antenna=%u  rbf=%u  reboot_reason=%u  tcmd q=%u/p=%u",
                 (unsigned)g_beacon.active_rf_switch_antenna,
                 (unsigned)g_beacon.rbf_pin_state,
                 (unsigned)g_beacon.reboot_reason,
                 (unsigned)g_beacon.total_tcmd_queued_count,
                 (unsigned)g_beacon.pending_queued_tcmd_count);
        put_line(row++, left, right, buf);
    }

    if (row < max) put_label(row++, left + 2, right, "Message");
    if (row < max) {
        char msg[COMMS_BEACON_FRIENDLY_MESSAGE_SIZE + 1];
        size_t mlen = strnlen(g_beacon.friendly_message,
                              COMMS_BEACON_FRIENDLY_MESSAGE_SIZE);
        memcpy(msg, g_beacon.friendly_message, mlen);
        msg[mlen] = '\0';
        snprintf(buf, sizeof buf, "    \"%s\"", msg);
        put_line(row++, left, right, buf);
    }

    return max;
}

static int draw_tcmd_panel(int top, int left, int rows, int cols)
{
    int right = left + cols;
    int row   = top;
    int max   = top + rows;

    char hdr[64];
    snprintf(hdr, sizeof hdr, "TCMD RESPONSES (%d)", g_tcmd_count);
    put_label(row++, left, right, hdr);

    if (g_tcmd_count == 0) {
        if (row < max) put_line(row + 1, left + 2, right, "(none yet)");
        return max;
    }

    // Each entry occupies up to two rows: header line + indented body.
    // Stop when the next entry won't fit fully — partial entries look
    // confusing.
    for (int i = 0; i < g_tcmd_count && row + 1 < max; i++) {
        int idx = (g_tcmd_head - i + TCMD_RING_SZ) % TCMD_RING_SZ;
        const tcmd_entry_t *e = &g_tcmd_ring[idx];
        char line[300];
        const char *ok = (e->code == 0) ? " (OK)" : "";
        snprintf(line, sizeof line,
                 "  %s  code=%u%s  dur=%ums  seq=%u/%u",
                 e->ts, (unsigned)e->code, ok,
                 (unsigned)e->duration_ms,
                 (unsigned)e->seq, (unsigned)e->max_seq);
        put_line(row, left, right, line);
        // Recolour the "code=N" token.
        int cp = code_pair(e->code);
        if (cp != PAIR_OK) {
            // Locate "code=" in the rendered line so we don't have to
            // count the variable-width ts.
            const char *p = strstr(line, "code=");
            if (p != NULL) {
                int col = left + (int)(p - line);
                char cb[24];
                snprintf(cb, sizeof cb, "code=%u", (unsigned)e->code);
                color_on(cp);
                put_line(row, col, right, cb);
                color_off(cp);
            }
        }
        row++;
        if (row >= max) break;
        snprintf(line, sizeof line, "      \"%s\"%s",
                 e->msg, e->truncated ? " ..." : "");
        put_line(row++, left, right, line);
    }

    return max;
}

static int draw_other_panel(int top, int left, int rows, int cols)
{
    int right = left + cols;
    int row   = top;
    int max   = top + rows;

    char hdr[64];
    snprintf(hdr, sizeof hdr, "OTHER FRAMES (%d)", g_other_count);
    put_label(row++, left, right, hdr);

    if (g_other_count == 0) {
        if (row < max) put_line(row + 1, left + 2, right, "(none yet)");
        return max;
    }

    for (int i = 0; i < g_other_count && row + 1 < max; i++) {
        int idx = (g_other_head - i + OTHER_RING_SZ) % OTHER_RING_SZ;
        const other_entry_t *e = &g_other_ring[idx];
        char line[300];
        const char *rs_str = (e->rs_errs == -2) ? "UNCORR"
                           : (e->rs_errs == -1) ? "off" : "ok";
        if (e->csp_ok) {
            snprintf(line, sizeof line,
                     "  %s  len=%zu  csp=%u->%u port=%u/%u  rs=%s",
                     e->ts, e->payload_len,
                     (unsigned)e->csp_src, (unsigned)e->csp_dst,
                     (unsigned)e->csp_dport, (unsigned)e->csp_sport,
                     rs_str);
        } else {
            snprintf(line, sizeof line,
                     "  %s  csp decode failed  rs=%s",
                     e->ts, rs_str);
        }
        put_line(row++, left, right, line);
        if (row >= max) break;
        snprintf(line, sizeof line, "      \"%s\"%s",
                 e->preview, e->truncated ? " ..." : "");
        put_line(row++, left, right, line);
    }
    return max;
}

// ---- top-level render -------------------------------------------------

static void render(void)
{
    erase();
    int rows = LINES, cols = COLS;
    if (rows < 6 || cols < 40) {
        mvprintw(0, 0, "rx_tui: terminal too small (%dx%d)", cols, rows);
        refresh();
        return;
    }

    // ---- title bar
    if (g_have_color) attron(COLOR_PAIR(PAIR_TITLE));
    else              attron(A_REVERSE);
    mvhline(0, 0, ' ', cols);
    char title_left[400];
    snprintf(title_left, sizeof title_left, " %s", g_header);

    char title_right[80];
    if (g_have_any_frame) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long age_s = (long)(now.tv_sec - g_last_frame_clock.tv_sec);
        if (age_s < 0) age_s = 0;
        snprintf(title_right, sizeof title_right,
                 " frames=%llu  last_rx=%lds ago ",
                 (unsigned long long)g_counters.total, age_s);
    } else {
        snprintf(title_right, sizeof title_right,
                 " frames=0  last_rx=never ");
    }
    int rlen = (int)strlen(title_right);
    if (rlen > cols) rlen = cols;
    int left_max = cols - rlen;
    if (left_max < 0) left_max = 0;
    mvaddnstr(0, 0, title_left, left_max);
    mvaddstr(0, cols - rlen, title_right);
    if (g_have_color) attroff(COLOR_PAIR(PAIR_TITLE));
    else              attroff(A_REVERSE);

    // ---- optional status row (under the title bar)
    int title_rows = 1;
    if (g_status[0] != '\0') {
        if (g_have_color) attron(COLOR_PAIR(PAIR_TITLE));
        else              attron(A_REVERSE);
        mvhline(1, 0, ' ', cols);
        char status_line[300];
        snprintf(status_line, sizeof status_line, " %s", g_status);
        mvaddnstr(1, 0, status_line, cols);
        if (g_have_color) attroff(COLOR_PAIR(PAIR_TITLE));
        else              attroff(A_REVERSE);
        title_rows = 2;
    }

    // ---- main split (subtract one extra row when the REPL input is on)
    int main_top  = title_rows;
    int input_row = (g_cmd_fn != NULL) ? rows - 2 : -1;
    int main_rows = rows - title_rows - 1 - (g_cmd_fn != NULL ? 1 : 0);
    int side_by_side = (cols >= 100);
    if (side_by_side) {
        // beacon | (tcmd over others). The right column is split
        // horizontally; tcmd above, others below, separator between.
        int beacon_cols = cols / 2 - 1;
        int right_left  = beacon_cols + 1;
        int right_cols  = cols - right_left;
        draw_beacon_panel(main_top, 0, main_rows, beacon_cols);
        // vertical separator
        for (int r = main_top; r < main_top + main_rows; r++) {
            mvaddch(r, beacon_cols, ACS_VLINE);
        }
        int tcmd_rows  = main_rows / 2;
        int sep_row    = main_top + tcmd_rows;
        int other_top  = sep_row + 1;
        int other_rows = main_top + main_rows - other_top;
        draw_tcmd_panel(main_top, right_left, tcmd_rows, right_cols);
        if (other_rows > 0 && sep_row < main_top + main_rows) {
            mvhline(sep_row, right_left, ACS_HLINE, right_cols);
            draw_other_panel(other_top, right_left, other_rows, right_cols);
        }
    } else {
        // Stacked: beacon takes BEACON_PANEL_ROWS, then tcmd and others
        // split the remaining rows in half (separator between each).
        // The beacon panel needs 15 rows to show every subsystem block
        // (BEACON header through the friendly_message body); below that
        // the friendly_message is the first thing to disappear.
        const int BEACON_PANEL_ROWS = 15;
        int beacon_rows = BEACON_PANEL_ROWS;
        // Reserve at least 4 rows each for tcmd and others (header +
        // one entry's two lines), plus 2 rows for the two separators.
        const int MIN_BOTTOM_ROWS = 4 + 4 + 2;
        if (beacon_rows > main_rows - MIN_BOTTOM_ROWS) {
            beacon_rows = main_rows - MIN_BOTTOM_ROWS;
        }
        if (beacon_rows < 4) beacon_rows = 4;
        if (beacon_rows > main_rows) beacon_rows = main_rows;
        draw_beacon_panel(main_top, 0, beacon_rows, cols);
        int sep1_row   = main_top + beacon_rows;
        int tcmd_top   = sep1_row + 1;
        int remaining  = main_top + main_rows - tcmd_top;
        if (remaining > 0 && sep1_row < main_top + main_rows) {
            mvhline(sep1_row, 0, ACS_HLINE, cols);
            int tcmd_rows = remaining / 2;
            int sep2_row  = tcmd_top + tcmd_rows;
            int other_top = sep2_row + 1;
            int other_rows = main_top + main_rows - other_top;
            draw_tcmd_panel(tcmd_top, 0, tcmd_rows, cols);
            if (other_rows > 0 && sep2_row < main_top + main_rows) {
                mvhline(sep2_row, 0, ACS_HLINE, cols);
                draw_other_panel(other_top, 0, other_rows, cols);
            }
        }
    }

    // ---- REPL input row (above the footer, when active)
    if (input_row >= 0) {
        move(input_row, 0);
        clrtoeol();
        mvaddstr(input_row, 0, "> ");
        // Truncate the rendered line to the screen width.
        int max_chars = cols - 2;
        if (max_chars < 0) max_chars = 0;
        mvaddnstr(input_row, 2, g_cmdline, max_chars);
    }

    // ---- footer
    if (g_have_color) attron(COLOR_PAIR(PAIR_TITLE));
    else              attron(A_REVERSE);
    mvhline(rows - 1, 0, ' ', cols);
    char foot[300];
    snprintf(foot, sizeof foot,
             " beacons=%llu  tcmd=%llu  other=%llu  uncorr=%llu  hmac_mismatch=%llu  crc_mismatch=%llu",
             (unsigned long long)g_counters.beacon,
             (unsigned long long)g_counters.tcmd,
             (unsigned long long)g_counters.other,
             (unsigned long long)g_counters.rs_uncorrectable,
             (unsigned long long)g_counters.hmac_mismatch,
             (unsigned long long)g_counters.crc_mismatch);
    const char *kb = (g_cmd_fn != NULL) ? "  [^C] quit " : "  [q] quit ";
    int klen = (int)strlen(kb);
    int foot_max = cols - klen;
    if (foot_max < 0) foot_max = 0;
    mvaddnstr(rows - 1, 0, foot, foot_max);
    if (klen <= cols) mvaddstr(rows - 1, cols - klen, kb);
    if (g_have_color) attroff(COLOR_PAIR(PAIR_TITLE));
    else              attroff(A_REVERSE);

    // Park the cursor on the REPL input column so backspace / left
    // arrow look right; otherwise hide it (panels don't need a
    // visible cursor and ncurses leaves it wherever last printed).
    if (input_row >= 0) {
        int col = 2 + (int)g_cmd_cursor;
        if (col >= cols) col = cols - 1;
        move(input_row, col);
    }

    refresh();
}

// ---- public API -------------------------------------------------------

int rx_tui_init(void)
{
    if (g_init) return 0;
    if (initscr() == NULL) return -2;
    cbreak();
    noecho();
    nonl();
    curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(PAIR_TITLE, COLOR_WHITE,  COLOR_BLUE);
        init_pair(PAIR_LABEL, COLOR_CYAN,   -1);
        init_pair(PAIR_OK,    COLOR_GREEN,  -1);
        init_pair(PAIR_WARN,  COLOR_YELLOW, -1);
        init_pair(PAIR_ALERT, COLOR_RED,    -1);
        g_have_color = 1;
    }
    g_init = 1;
    render();
    return 0;
}

void rx_tui_close(void)
{
    if (!g_init) return;
    endwin();
    g_init = 0;
}

void rx_tui_set_header(const char *header)
{
    if (header == NULL) {
        g_header[0] = '\0';
    } else {
        snprintf(g_header, sizeof g_header, "%s", header);
    }
    if (g_init) render();
}

void rx_tui_set_status(const char *status)
{
    if (status == NULL) {
        g_status[0] = '\0';
    } else {
        snprintf(g_status, sizeof g_status, "%s", status);
    }
    if (g_init) render();
}

void rx_tui_set_command_handler(rx_tui_command_fn fn, void *ctx)
{
    g_cmd_fn  = fn;
    g_cmd_ctx = ctx;
    if (g_init) {
        // Show the cursor only while the REPL is active. Without this
        // the line editor is invisible and the operator has to count
        // backspaces.
        curs_set(fn != NULL ? 1 : 0);
        render();
    }
}

void rx_tui_set_history_path(const char *path)
{
    if (path == NULL) {
        g_hist_path[0] = '\0';
        return;
    }
    snprintf(g_hist_path, sizeof g_hist_path, "%s", path);
    hist_load_from_disk();
}

void rx_tui_observe_frame(const char *ts,
                          const uint8_t *packet, size_t packet_len,
                          int golay_errs, int hmac_ok, int use_hmac,
                          int rs_errs,
                          int crc_status)
{
    (void)golay_errs;
    if (!g_init) return;

    g_counters.total++;
    clock_gettime(CLOCK_MONOTONIC, &g_last_frame_clock);
    g_have_any_frame = 1;

    if (use_hmac && hmac_ok == 0) g_counters.hmac_mismatch++;
    if (rs_errs == -2) g_counters.rs_uncorrectable++;
    if (crc_status == 0) g_counters.crc_mismatch++;

    csp_v1_header_t hdr = {0};
    int csp_ok = (packet_len >= 4) && (csp_v1_decode(packet, &hdr) == 0);
    const uint8_t *payload = csp_ok ? packet + 4 : NULL;
    size_t payload_len = csp_ok ? packet_len - 4 : 0;

    if (csp_ok && beacon_is_basic(payload, payload_len)) {
        memcpy(&g_beacon, payload, sizeof g_beacon);
        snprintf(g_beacon_ts, sizeof g_beacon_ts, "%s", ts ? ts : "?");
        g_have_beacon = 1;
        g_counters.beacon++;
    } else if (csp_ok && tcmd_response_is(payload, payload_len)) {
        COMMS_tcmd_response_packet_t r;
        memcpy(&r, payload, COMMS_TCMD_RESPONSE_HEADER_SIZE);
        g_tcmd_head = (g_tcmd_head + 1) % TCMD_RING_SZ;
        if (g_tcmd_count < TCMD_RING_SZ) g_tcmd_count++;
        tcmd_entry_t *e = &g_tcmd_ring[g_tcmd_head];
        snprintf(e->ts, sizeof e->ts, "%s", ts ? ts : "?");
        e->code = r.response_code;
        e->duration_ms = r.duration_ms;
        e->seq = r.response_seq_num;
        e->max_seq = r.response_max_seq_num;
        size_t data_len = payload_len - COMMS_TCMD_RESPONSE_HEADER_SIZE;
        e->data_len = data_len;
        cts1_sanitise_text(payload + COMMS_TCMD_RESPONSE_HEADER_SIZE,
                           data_len, e->msg, sizeof e->msg,
                           &e->truncated);
        g_counters.tcmd++;
    } else {
        g_counters.other++;
        g_other_head = (g_other_head + 1) % OTHER_RING_SZ;
        if (g_other_count < OTHER_RING_SZ) g_other_count++;
        other_entry_t *e = &g_other_ring[g_other_head];
        snprintf(e->ts, sizeof e->ts, "%s", ts ? ts : "?");
        e->payload_len = payload_len;
        e->csp_ok = csp_ok;
        e->csp_src = hdr.src;
        e->csp_dst = hdr.dst;
        e->csp_dport = hdr.dport;
        e->csp_sport = hdr.sport;
        e->rs_errs = rs_errs;
        e->crc_status = crc_status;
        // Show whatever payload bytes were recovered. When CSP decode
        // failed we still preview the first few bytes of the packet so
        // the operator has something to spot patterns by.
        const uint8_t *prev_src = csp_ok ? payload : packet;
        size_t prev_len = csp_ok ? payload_len : packet_len;
        cts1_sanitise_text(prev_src, prev_len,
                           e->preview, sizeof e->preview,
                           &e->truncated);
    }

    render();
}

// Handle one keystroke against the REPL line editor. Returns 1 if a
// command was submitted (caller may want to re-render); 0 otherwise.
static int repl_handle_key(int ch)
{
    if (ch == '\n' || ch == '\r' || ch == KEY_ENTER) {
        if (g_cmd_len > 0) {
            // Submit a copy so the handler may free / overwrite without
            // racing with our local state cleanup below.
            char buf[RX_TUI_CMDLINE_MAX + 1];
            snprintf(buf, sizeof buf, "%s", g_cmdline);
            hist_append(buf, /*persist=*/1);
            if (g_cmd_fn != NULL) g_cmd_fn(buf, g_cmd_ctx);
        }
        g_cmdline[0]    = '\0';
        g_cmd_len       = 0;
        g_cmd_cursor    = 0;
        g_hist_view_idx = -1;
        return 1;
    }
    if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {
        if (g_cmd_cursor > 0) {
            memmove(g_cmdline + g_cmd_cursor - 1,
                    g_cmdline + g_cmd_cursor,
                    g_cmd_len - g_cmd_cursor + 1);
            g_cmd_cursor--;
            g_cmd_len--;
        }
        return 0;
    }
    if (ch == KEY_DC) {
        if (g_cmd_cursor < g_cmd_len) {
            memmove(g_cmdline + g_cmd_cursor,
                    g_cmdline + g_cmd_cursor + 1,
                    g_cmd_len - g_cmd_cursor);
            g_cmd_len--;
        }
        return 0;
    }
    if (ch == KEY_LEFT)  { if (g_cmd_cursor > 0)         g_cmd_cursor--; return 0; }
    if (ch == KEY_RIGHT) { if (g_cmd_cursor < g_cmd_len) g_cmd_cursor++; return 0; }
    if (ch == KEY_HOME)  { g_cmd_cursor = 0;          return 0; }
    if (ch == KEY_END)   { g_cmd_cursor = g_cmd_len;  return 0; }
    if (ch == KEY_UP) {
        if (g_hist_count <= 0) return 0;
        if (g_hist_view_idx == -1) {
            snprintf(g_cmd_saved, sizeof g_cmd_saved, "%s", g_cmdline);
            g_hist_view_idx = g_hist_count - 1;
        } else if (g_hist_view_idx > 0) {
            g_hist_view_idx--;
        }
        const char *h = hist_at(g_hist_view_idx);
        if (h != NULL) {
            snprintf(g_cmdline, sizeof g_cmdline, "%s", h);
            g_cmd_len    = strlen(g_cmdline);
            g_cmd_cursor = g_cmd_len;
        }
        return 0;
    }
    if (ch == KEY_DOWN) {
        if (g_hist_view_idx < 0) return 0;
        if (g_hist_view_idx < g_hist_count - 1) {
            g_hist_view_idx++;
            const char *h = hist_at(g_hist_view_idx);
            if (h != NULL) {
                snprintf(g_cmdline, sizeof g_cmdline, "%s", h);
                g_cmd_len    = strlen(g_cmdline);
                g_cmd_cursor = g_cmd_len;
            }
        } else {
            g_hist_view_idx = -1;
            snprintf(g_cmdline, sizeof g_cmdline, "%s", g_cmd_saved);
            g_cmd_len    = strlen(g_cmdline);
            g_cmd_cursor = g_cmd_len;
        }
        return 0;
    }
    if (ch == 27) {
        // Esc clears the line and exits any history browse.
        g_cmdline[0]    = '\0';
        g_cmd_len       = 0;
        g_cmd_cursor    = 0;
        g_hist_view_idx = -1;
        return 0;
    }
    if (ch >= 32 && ch < 127 && g_cmd_len < RX_TUI_CMDLINE_MAX) {
        memmove(g_cmdline + g_cmd_cursor + 1,
                g_cmdline + g_cmd_cursor,
                g_cmd_len - g_cmd_cursor + 1);
        g_cmdline[g_cmd_cursor] = (char)ch;
        g_cmd_cursor++;
        g_cmd_len++;
    }
    return 0;
}

int rx_tui_tick(void)
{
    if (!g_init) return 0;
    int ch;
    while ((ch = getch()) != ERR) {
        if (g_cmd_fn != NULL) {
            if (ch == KEY_RESIZE) continue;
            (void)repl_handle_key(ch);
        } else {
            if (ch == 'q' || ch == 'Q') {
                g_quit_requested = 1;
            } else if (ch == KEY_RESIZE) {
                // ncurses already resized; just redraw on next tick.
            }
        }
    }
    // Cheap redraw so the "last_rx age" keeps incrementing without an
    // observed frame.
    render();
    return g_quit_requested ? 1 : 0;
}

void rx_tui_hold_until_quit(void)
{
    if (!g_init) return;
    // Blocking input with a 1 s timeout so the title-bar age refreshes
    // once a second. The wait also exits on SIGINT/SIGTERM if the
    // caller wired its handler through rx_tui_request_quit().
    timeout(1000);
    while (!g_quit_requested) {
        int ch = getch();
        if (ch == 'q' || ch == 'Q') break;
        render();
    }
    nodelay(stdscr, TRUE);
}

void rx_tui_request_quit(void)
{
    g_quit_requested = 1;
}

#else  // !RX_TUI_AVAILABLE -------------------------------------------------

int rx_tui_init(void) { return -1; }
void rx_tui_close(void) {}
void rx_tui_set_header(const char *header) { (void)header; }
void rx_tui_set_status(const char *status) { (void)status; }
void rx_tui_set_command_handler(rx_tui_command_fn fn, void *ctx) { (void)fn; (void)ctx; }
void rx_tui_set_history_path(const char *path) { (void)path; }
void rx_tui_observe_frame(const char *ts, const uint8_t *packet,
                          size_t packet_len, int golay_errs, int hmac_ok,
                          int use_hmac, int rs_errs, int crc_status)
{
    (void)ts; (void)packet; (void)packet_len; (void)golay_errs;
    (void)hmac_ok; (void)use_hmac; (void)rs_errs; (void)crc_status;
}
int  rx_tui_tick(void) { return 0; }
void rx_tui_hold_until_quit(void) {}
void rx_tui_request_quit(void) {}

#endif
