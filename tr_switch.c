/*

   Simple Satellite Operations  tr_switch.c

   See tr_switch.h for the on-wire contract this driver implements.

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

#include "tr_switch.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static int tr_switch_write_char(tr_switch_t *s, char c)
{
    if (s == NULL || !s->connected) return -1;
    ssize_t n = write(s->fd, &c, 1);
    return (n == 1) ? 0 : -1;
}

// Pull the substring after `needle=` from `line` up to the next space
// or end-of-line, copy into `out` (truncating to cap-1 bytes), and
// return a pointer to the value (inside line) or NULL when the key is
// absent. The needle should include the trailing '='.
static const char *find_kv(const char *line, const char *needle,
                           char *out, size_t cap)
{
    if (out == NULL || cap == 0) return NULL;
    out[0] = '\0';
    const char *p = strstr(line, needle);
    if (p == NULL) return NULL;
    p += strlen(needle);
    size_t i = 0;
    while (p[i] != '\0' && p[i] != ' ' && p[i] != '\r' && p[i] != '\n'
           && i + 1 < cap) {
        out[i] = p[i];
        ++i;
    }
    out[i] = '\0';
    return p;
}

// Parse a complete line. Heartbeat lines look like
//   [12345] [ INFO ] Heartbeat: state=RX mode=AUTO
// with an optional `last_tx_ago_s=<sec>.<ms>` (or `=never`) field
// appended by a future firmware release. Anything else (boot banner,
// log-level ack, watchdog notice, debug "TX Sensed!" etc.) is
// silently dropped.
static void parse_line(tr_switch_t *s, const char *line, double t_now)
{
    if (s == NULL || line == NULL) return;
    const char *hb = strstr(line, "Heartbeat:");
    if (hb == NULL) return;

    char state_buf[TR_SWITCH_STATE_MAX] = {0};
    char mode_buf [TR_SWITCH_MODE_MAX]  = {0};
    if (find_kv(hb, "state=", state_buf, sizeof state_buf) == NULL) return;
    if (find_kv(hb, "mode=",  mode_buf,  sizeof mode_buf)  == NULL) return;

    snprintf(s->state_str, sizeof s->state_str, "%s", state_buf);
    snprintf(s->mode_str,  sizeof s->mode_str,  "%s", mode_buf);

    // Optional last_tx_ago_s field. Tolerate three cases: absent,
    // "never", and a numeric "<s>.<ms>" or "<s>" string.
    char age_buf[32] = {0};
    if (find_kv(hb, "last_tx_ago_s=", age_buf, sizeof age_buf) != NULL) {
        if (strcmp(age_buf, "never") == 0) {
            // Use +inf so the UI can render "never" as "—" without
            // colliding with a real, very-large age value.
            s->last_tx_ago_s = INFINITY;
        } else {
            // sscanf returns 1 on a successful parse; on failure the
            // field is left as whatever it was before. Mark it NAN
            // so the UI shows "—" rather than a stale number.
            double v = NAN;
            if (sscanf(age_buf, "%lf", &v) == 1) {
                s->last_tx_ago_s = v;
            } else {
                s->last_tx_ago_s = NAN;
            }
        }
    }
    // If the field is absent the old value (NAN at init) is preserved.

    s->t_last_heartbeat = t_now;
    s->heartbeat_count++;
}

int tr_switch_init(tr_switch_t *s)
{
    if (s == NULL || s->device_filename == NULL) return -1;
    s->connected        = 0;
    s->fd               = -1;
    s->rx_len           = 0;
    s->state_str[0]     = '\0';
    s->mode_str [0]     = '\0';
    s->last_tx_ago_s    = NAN;
    s->t_last_byte      = 0.0;
    s->t_last_heartbeat = 0.0;
    s->heartbeat_count  = 0;

    // O_NONBLOCK keeps open() from hanging on a CDC modem that hasn't
    // asserted DCD yet, and keeps the subsequent reads non-blocking
    // even though the FD inherits the flag.
    int fd = open(s->device_filename, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        // Caller logs; we stay silent so the warning isn't duplicated.
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }
    cfsetospeed(&tty, s->serial_speed ? s->serial_speed : B115200);
    cfsetispeed(&tty, s->serial_speed ? s->serial_speed : B115200);

    // Raw 8N1, no flow control. USB-CDC ignores most of this but the
    // raw-mode bits matter so we don't process backspace / signal
    // characters.
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                     | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);

    s->fd        = fd;
    s->tty       = tty;
    s->connected = 1;

    // Wake the firmware: switch to LOG_INFO so heartbeats start
    // flowing, then take serial-override AUTO so the TX detector
    // drives K1/K2 regardless of the physical slide switch.
    tr_switch_set_log_level  (s, 1);
    tr_switch_set_serial_mode(s, 'a');
    return 0;
}

void tr_switch_disconnect(tr_switch_t *s)
{
    if (s == NULL) return;
    if (s->connected) {
        // Best-effort: hand the firmware back to its slide switch
        // before we drop the link.
        tr_switch_set_serial_mode(s, 's');
        close(s->fd);
    }
    s->connected = 0;
    s->fd        = -1;
}

void tr_switch_pump(tr_switch_t *s, double t_now)
{
    if (s == NULL || !s->connected) return;
    // Drain whatever is available. The FD is non-blocking so a read
    // returning -1/EAGAIN just means "no bytes for now".
    for (;;) {
        char chunk[128];
        ssize_t n = read(s->fd, chunk, sizeof chunk);
        if (n < 0) {
            // EAGAIN/EWOULDBLOCK = no bytes; anything else = drop the
            // link and let the operator restart sso.
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                tr_switch_disconnect(s);
            }
            return;
        }
        if (n == 0) return;
        s->t_last_byte = t_now;
        // Append into rx_buf, splitting on '\n'. Overflow strategy:
        // when a single line exceeds the buffer, drop the trailing
        // chunk and keep enough head to find the eventual '\n' — the
        // firmware never emits long lines, so this is defensive.
        for (ssize_t i = 0; i < n; ++i) {
            char c = chunk[i];
            if (c == '\r') continue;
            if (c == '\n') {
                s->rx_buf[s->rx_len] = '\0';
                parse_line(s, s->rx_buf, t_now);
                s->rx_len = 0;
                continue;
            }
            if (s->rx_len + 1 >= sizeof s->rx_buf) {
                // Reset and look for the next newline.
                s->rx_len = 0;
                continue;
            }
            s->rx_buf[s->rx_len++] = c;
        }
    }
}

int tr_switch_set_log_level(tr_switch_t *s, int level)
{
    if (level < 0 || level > 3) return -1;
    return tr_switch_write_char(s, (char)('0' + level));
}

int tr_switch_set_serial_mode(tr_switch_t *s, char serial_mode)
{
    if (serial_mode != 't' && serial_mode != 'r'
        && serial_mode != 'a' && serial_mode != 's') return -1;
    return tr_switch_write_char(s, serial_mode);
}

int tr_switch_is_stale(const tr_switch_t *s, double t_now)
{
    if (s == NULL || !s->connected) return 0;
    if (s->t_last_heartbeat <= 0.0) {
        // Allow a grace window after open: only flag stale once a
        // full stale-window has elapsed without ever seeing a beat.
        return (t_now - s->t_last_byte) > TR_SWITCH_STALE_AFTER_S
            && s->t_last_byte > 0.0;
    }
    return (t_now - s->t_last_heartbeat) > TR_SWITCH_STALE_AFTER_S;
}
