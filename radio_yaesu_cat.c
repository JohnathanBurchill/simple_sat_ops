/*

   Simple Satellite Operations  radio_yaesu_cat.c

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

// Yaesu FT-991A (CAT ASCII protocol) backend. Targets 9600-bps GFSK uplink
// via the radio's analog FM modulator with switchable-input chain — the
// path the IC-9700's FPGA TX filter blocks. CAT commands are short ASCII
// strings terminated by ';'.
//
// FT-991A bring-up checklist (radio side). All menu numbers verified
// against "FT-991A CAT Operation Reference Manual" 1711-D, page 7-8:
//
//   Menu 031 CAT RATE  = 4800     P2 0:4800 1:9600 2:19200 3:38400.
//                                  Must match --radio-serial-speed=.
//   Menu 032 CAT TOT   = 10 ms    P2 0:10ms 1:100ms 2:1000ms 3:3000ms.
//                                  Must be >0 ms or replies are dropped.
//   Menu 033 CAT RTS   = DISABLE  P2 0:DISABLE 1:ENABLE.
//                                  macOS's CP2105 driver doesn't reliably
//                                  raise RTS; leave the radio off.
//   Menu 070 DATA IN   = REAR     0:MIC 1:REAR. Set by set_data_mod_source.
//   Menu 072 DATA PORT = USB|DATA 1:DATA jack 2:USB. Same.
//   Menu 079 FM PKT MODE = 9600   P2 0:1200 1:9600. Set on the front panel
//                                  before any 9600-bps work; not driven
//                                  here because it only matters when the
//                                  radio is in FM packet mode (MD04;).
//                                  Operating mode for our uplink is
//                                  DATA-FM (MD0A;) which uses Menu 070-072.
//
// The FT-991A's USB bridge (Silicon Labs CP2105) is a dual-port chip;
// either virtual COM port carries CAT, so first-time bring-up may need
// trying both /dev/cu.usbserial-XXXa and -XXXb (or /dev/ttyUSB0/1 on
// Linux). --store-device persists the working one.
//
// CAT command reference for the operations we use:
//
//   FA<9 digits Hz>;     VFO A frequency, set or read (FA;)
//   MD0<n>;              Operating mode for VFO A (n is a hex char):
//                          1=LSB, 2=USB, 3=CW, 4=FM, 5=AM, 6=RTTY-LSB,
//                          7=CW-R, 8=DATA-LSB, 9=RTTY-USB, A=FM-N,
//                          B=DATA-FM, C=DATA-USB, D=AM-N, E=C4FM
//                        (FT-991A 1.10+ firmware uses these mappings;
//                        older firmware omits some of the data modes.)
//   PC<nnn>;             RF power, 5..100 W
//   TX1; / RX;           PTT keyed via mic / unkey
//   EX076<DAKY|MIC>;     Menu 076 = DATA MOD source
//   EX078<nnn>;          Menu 078 = USB MOD level (0..100)
//   EX079<2>;            Menu 079 = PKT RATE (1=1200, 2=9600). Set on init.
//
// FT-991A has no equivalent of the IC-9700's satellite mode, sub VFO,
// FIL1/2/3 or 0x27 waterfall stream — those ops are NULL in the vtable.

#include "radio.h"
#include "radio_backend.h"
#include "qol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>
#include <errno.h>

// FT-991A operating-mode characters (used in MD0<n>;), per the
// FT-991A CAT Operation Reference Manual page 11:
//   1 LSB   2 USB   3 CW-U   4 FM      5 AM       6 RTTY-LSB
//   7 CW-L  8 DATA-LSB       9 RTTY-USB
//   A DATA-FM   B FM-N   C DATA-USB   D AM-N   E C4FM
//
// Note 'B' is FM-N (Narrow FM, ~5 kHz channel spacing), NOT DATA-FM.
// DATA-FM is 'A'. An earlier version of this file had them swapped,
// which silently put DATA-FM uplinks in narrow-FM with a much smaller
// modulator passband.
#define YAESU_MODE_LSB       '1'
#define YAESU_MODE_USB       '2'
#define YAESU_MODE_CW        '3'
#define YAESU_MODE_FM        '4'
#define YAESU_MODE_AM        '5'
#define YAESU_MODE_DATA_FM   'A'
#define YAESU_MODE_FM_N      'B'
#define YAESU_MODE_DATA_USB  'C'

static void yaesu_cat_connect(radio_t *radio);
static void yaesu_cat_disconnect(radio_t *radio);

// Print the wire trace for one CAT exchange. Default is the ASCII form
// (these are text commands, after all). --debug-output flips on the
// hex-bytes view useful when chasing parity / encoding issues.
static void yaesu_trace(const radio_t *r, const char *label,
                        const char *bytes, size_t len)
{
    if (r->debug_wire) {
        printcmd(label, (const unsigned char *)bytes, (int)len);
    } else {
        fprintf(stderr, "%s %.*s\n", label, (int)len, bytes);
    }
}

// Send an ASCII CAT command (no terminator added — caller includes the ';').
// If reply is non-NULL, read until the next ';' into reply (size reply_cap)
// and NUL-terminate. Returns RADIO_OK on success, RADIO_BAD_RESPONSE on I/O
// error or timeout.
static int yaesu_send(radio_t *radio, const char *cmd, char *reply, size_t reply_cap)
{
    if (!radio->connected) {
        return RADIO_BAD_RESPONSE;
    }
    yaesu_trace(radio, "CAT command:", cmd, strlen(cmd));

    // Drop any stale bytes the radio sent us (e.g. an unread reply from a
    // prior command, or auto-info chatter). TCIFLUSH only — TCIOFLUSH
    // would also discard our pending OUTPUT, truncating commands mid-byte
    // when the next caller's tcflush fires before the UART has drained.
    tcflush(radio->fd, TCIFLUSH);
    size_t cmd_len = strlen(cmd);
    ssize_t n = write(radio->fd, cmd, cmd_len);
    if (n != (ssize_t)cmd_len) {
        return RADIO_BAD_RESPONSE;
    }
    // Block until the bytes have actually left the UART. Without this a
    // subsequent operation can flush them mid-flight; at 38400 baud a
    // 12-byte FA<freq>; takes ~3 ms which is well within typical
    // user-space scheduling jitter.
    tcdrain(radio->fd);

    if (reply == NULL || reply_cap == 0) {
        return RADIO_OK;
    }

    // Read until ';' or buffer full or VTIME expiry.
    size_t off = 0;
    int idle_ticks = 0;
    while (off + 1 < reply_cap) {
        char ch = 0;
        ssize_t r = read(radio->fd, &ch, 1);
        if (r < 0) {
            return RADIO_BAD_RESPONSE;
        }
        if (r == 0) {
            // VTIME expiry; allow a few in case of slow USB.
            if (++idle_ticks > 20) {
                break;
            }
            continue;
        }
        idle_ticks = 0;
        reply[off++] = ch;
        if (ch == ';') break;
    }
    reply[off] = '\0';
    yaesu_trace(radio, "CAT reply:  ", reply, off);
    if (off == 0) {
        return RADIO_BAD_RESPONSE;
    }
    return RADIO_OK;
}

static int yaesu_cat_init(radio_t *radio)
{
    yaesu_cat_connect(radio);
    if (!radio->connected) {
        fprintf(stderr, "Error opening Yaesu CAT serial port. Is the FT-991A "
                "USB cable plugged in and powered?\n");
        return RADIO_OPEN;
    }

    // Identify the radio (ID;) so a wrong-radio mistake is loud.
    // FT-991A returns "ID0670;". This is a passive query; init() is
    // intentionally read-only on the radio side. Frequency / mode
    // configuration belongs in radio_uplink_prep() (or explicit
    // radio_set_frequency / radio_set_mode calls) so tools like
    // radio_ctl 'identify' don't retune the rig as a side effect.
    char idbuf[32] = {0};
    if (yaesu_send(radio, "ID;", idbuf, sizeof idbuf) != RADIO_OK) {
        fprintf(stderr, "warning: no reply from Yaesu CAT (ID;); continuing.\n");
    } else if (strncmp(idbuf, "ID0670;", 7) != 0) {
        fprintf(stderr, "warning: ID; reply '%s' is not FT-991A's ID0670; "
                "continuing anyway.\n", idbuf);
    }
    radio->transceiver_id = 0x0670;

    // CAT PTT-off bytes for the SIGINT handler — one async-signal-safe
    // write() releases TX. "TX0;" matches the documented form on FT-991A.
    static const uint8_t ptt_off[] = {'T', 'X', '0', ';'};
    memcpy(radio->ptt_off_raw, ptt_off, sizeof ptt_off);
    radio->ptt_off_raw_len = (uint8_t)sizeof ptt_off;

    return RADIO_OK;
}

static void yaesu_cat_connect(radio_t *radio)
{
    radio->connected = 0;
    radio->fd = open(radio->device_filename, O_RDWR | O_NOCTTY | O_SYNC);
    if (radio->fd == -1) {
        perror("Error opening serial port");
        return;
    }

    memset(&radio->tty, 0, sizeof(radio->tty));
    if (tcgetattr(radio->fd, &radio->tty) != 0) {
        perror("Error getting serial port attributes");
        close(radio->fd);
        return;
    }

    cfsetospeed(&radio->tty, radio->serial_speed);
    cfsetispeed(&radio->tty, radio->serial_speed);

    radio->tty.c_cflag = (radio->tty.c_cflag & ~CSIZE) | CS8;
    radio->tty.c_iflag &= ~IGNPAR;
    radio->tty.c_lflag = 0;
    radio->tty.c_oflag = 0;
    radio->tty.c_cc[VMIN] = 0;
    radio->tty.c_cc[VTIME] = 1;  // 100 ms read timeout per byte

    radio->tty.c_cflag |= (CLOCAL | CREAD);
    radio->tty.c_cflag &= ~(PARENB | PARODD);
    radio->tty.c_cflag &= ~CSTOPB;
    radio->tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(radio->fd, TCSANOW, &radio->tty) != 0) {
        perror("Error setting serial port attributes");
        close(radio->fd);
        return;
    }
    tcflush(radio->fd, TCIOFLUSH);

    radio->connected = 1;
}

static void yaesu_cat_disconnect(radio_t *radio)
{
    if (radio->connected) {
        // Give the radio a moment to commit the most-recent fire-and-
        // forget command (e.g. FA<freq>;) before close() drops DTR and
        // potentially resets CAT state. tcdrain() has already ensured
        // the bytes left the UART; this covers the radio's own
        // microcontroller processing latency. ~80 ms covers the
        // observed FT-991A worst case with margin.
        usleep(80 * 1000);
        close(radio->fd);
        radio->connected = 0;
    }
}

static double yaesu_cat_get_frequency(radio_t *radio)
{
    char rep[32] = {0};
    if (yaesu_send(radio, "FA;", rep, sizeof rep) != RADIO_OK) {
        return -1.0;
    }
    // "FA<9 digits>;" => 12 chars min
    if (strncmp(rep, "FA", 2) != 0 || strlen(rep) < 12) {
        return -1.0;
    }
    char digits[16] = {0};
    strncpy(digits, rep + 2, 9);
    return atof(digits);
}

static int yaesu_cat_set_frequency(radio_t *radio, double frequency)
{
    char fa[24];
    snprintf(fa, sizeof fa, "FA%09llu;", (unsigned long long)llround(frequency));
    if (yaesu_send(radio, fa, NULL, 0) != RADIO_OK) {
        return RADIO_SET_FREQUENCY;
    }
    return RADIO_OK;
}

// Translate the project's RADIO_MODE_* (IC-9700-shaped) into the FT-991A's
// MD0<n>; one-character mode index. Filter is ignored; FT-991A NAR/WIDE is
// mode-coupled and not addressed here.
static int yaesu_cat_set_mode(radio_t *radio, int mode, int filter)
{
    (void)filter;
    char ch;
    switch (mode) {
        case RADIO_MODE_LSB: ch = YAESU_MODE_LSB; break;
        case RADIO_MODE_USB: ch = YAESU_MODE_USB; break;
        case RADIO_MODE_CW:  ch = YAESU_MODE_CW;  break;
        case RADIO_MODE_FM:  ch = YAESU_MODE_FM;  break;
        case RADIO_MODE_AM:  ch = YAESU_MODE_AM;  break;
        default:
            fprintf(stderr, "yaesu_cat: mode 0x%02X not mapped; using FM\n", mode);
            ch = YAESU_MODE_FM;
            break;
    }
    char cmd[8];
    snprintf(cmd, sizeof cmd, "MD0%c;", ch);
    return yaesu_send(radio, cmd, NULL, 0);
}

// On the FT-991A there is no separate "DATA flag"; switching from FM to
// DATA-FM is a mode change. So set_data_mode(on=1, ...) → MD0B; (DATA-FM)
// and set_data_mode(on=0, ...) → MD04; (FM). Filter byte is ignored.
static int yaesu_cat_set_data_mode(radio_t *radio, int on, int filter)
{
    (void)filter;
    char cmd[8];
    snprintf(cmd, sizeof cmd, "MD0%c;", on ? YAESU_MODE_DATA_FM : YAESU_MODE_FM);
    return yaesu_send(radio, cmd, NULL, 0);
}

// FT-991A DATA-mode modulator routing (from the CAT manual menu list,
// page 8). All EX<menu><value>; values are pure digits.
//   Menu 070 DATA IN SELECT   : 0=MIC,  1=REAR
//   Menu 072 DATA PORT SELECT : 1=DATA jack, 2=USB CODEC (only when 070=REAR)
// We emit both so the routing is unambiguous regardless of any prior
// front-panel state. Caller's enum is IC-9700-shaped:
//   USB / MIC_USB → REAR + USB CODEC
//   ACC           → REAR + DATA jack
//   MIC / MIC_ACC → MIC (jack on the front of the radio)
static int yaesu_cat_set_data_mod_source(radio_t *radio, int source)
{
    const char *in_sel = NULL;     // Menu 070
    const char *port_sel = NULL;   // Menu 072 (only when REAR)
    switch (source) {
        case RADIO_DATA_MOD_SRC_MIC:
        case RADIO_DATA_MOD_SRC_MIC_ACC:
            in_sel = "EX0700;";
            break;
        case RADIO_DATA_MOD_SRC_USB:
        case RADIO_DATA_MOD_SRC_MIC_USB:
            in_sel = "EX0701;";
            port_sel = "EX0722;";
            break;
        case RADIO_DATA_MOD_SRC_ACC:
            in_sel = "EX0701;";
            port_sel = "EX0721;";
            break;
        default:
            fprintf(stderr, "yaesu_cat: DATA MOD source 0x%02X unknown; using REAR/USB\n", source);
            in_sel = "EX0701;";
            port_sel = "EX0722;";
            break;
    }
    int rc = yaesu_send(radio, in_sel, NULL, 0);
    if (rc != RADIO_OK) return rc;
    if (port_sel != NULL) {
        rc = yaesu_send(radio, port_sel, NULL, 0);
    }
    return rc;
}

// Menu 078 = USB MOD level, 0..100. The project passes 0..255; rescale.
static int yaesu_cat_set_usb_mod_level(radio_t *radio, int level_0_to_255)
{
    if (level_0_to_255 < 0)   level_0_to_255 = 0;
    if (level_0_to_255 > 255) level_0_to_255 = 255;
    int pct = (level_0_to_255 * 100 + 127) / 255;
    char cmd[16];
    snprintf(cmd, sizeof cmd, "EX078%03d;", pct);
    return yaesu_send(radio, cmd, NULL, 0);
}

// PC<nnn>; sets RF power 5..100 W. Project passes 0..255 ↔ 0..100% of band
// max; on UHF max is 50 W, but PC<nnn>; uses absolute watts. Compromise:
// scale 0..255 → 5..50 W on UHF for safety. The TX-inhibit and
// --allow-high-power gates upstream of this still apply.
static int yaesu_cat_set_rf_power(radio_t *radio, int level_0_to_255)
{
    if (level_0_to_255 < 0)   level_0_to_255 = 0;
    if (level_0_to_255 > 255) level_0_to_255 = 255;
    // UHF cap on FT-991A is 50 W. Map 0..255 -> 5..50 W. Anything below 5
    // is the radio's own minimum.
    int watts = 5 + (int)(((double)level_0_to_255 / 255.0) * 45.0 + 0.5);
    if (watts < 5)   watts = 5;
    if (watts > 100) watts = 100;
    char cmd[16];
    snprintf(cmd, sizeof cmd, "PC%03d;", watts);
    return yaesu_send(radio, cmd, NULL, 0);
}

static int yaesu_cat_ptt(radio_t *radio, int on)
{
    // TX1; = key via CAT (RADIO TX OFF / CAT TX ON in the manual's
    // wording). TX0; = release. The FT-991A CAT command list does not
    // include the legacy "RX;" alias, so use TX0; per the manual.
    return yaesu_send(radio, on ? "TX1;" : "TX0;", NULL, 0);
}

static const radio_backend_ops_t yaesu_cat_ops = {
    .name                  = "yaesu-cat",
    .init                  = yaesu_cat_init,
    .disconnect            = yaesu_cat_disconnect,
    .command               = NULL,
    .set_vfo               = NULL,  // FT-991A always operates on VFO A here
    .get_frequency         = yaesu_cat_get_frequency,
    .set_frequency         = yaesu_cat_set_frequency,
    .get_satellite_mode    = NULL,
    .set_satellite_mode    = NULL,
    .set_mode              = yaesu_cat_set_mode,
    .set_data_mode         = yaesu_cat_set_data_mode,
    .set_data_mod_source   = yaesu_cat_set_data_mod_source,
    .set_usb_mod_level     = yaesu_cat_set_usb_mod_level,
    .set_moni_level        = NULL,  // future Menu 009 helper if ever needed
    .set_rf_power          = yaesu_cat_set_rf_power,
    .ptt                   = yaesu_cat_ptt,
    .get_band_selection    = NULL,
    .set_band_selection    = NULL,
    .toggle_waterfall      = NULL,
};

const radio_backend_ops_t *radio_backend_yaesu_cat_ops(void)
{
    return &yaesu_cat_ops;
}
