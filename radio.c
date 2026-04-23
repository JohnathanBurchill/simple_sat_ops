/*

   Simple Satellite Operations  radio.c

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

#include "radio.h"
#include "qol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>

#include <err.h>

// Initialise the IC-9700 for simplex UHF ownership.
//
// simple_sat_ops assumes it is the sole user of this radio, so we start
// from a known-good simplex state every time:
//   - satellite mode OFF  (no split Main/Sub tracking pair)
//   - Sub parked on a benign VHF freq so same-band-on-both collisions are
//     impossible when Main tunes UHF
//   - Main selected, VFO A, FM / FIL1, tuned to radio->nominal_downlink_frequency
//
// If sub_park_frequency is 0, RADIO_SUB_PARK_HZ is used.
int radio_init(radio_t *radio)
{
    int rc = RADIO_OK;

    radio_connect(radio);
    if (!radio->connected) {
        fprintf(stderr, "Error opening radio. Is it plugged into USB and powered?\n");
        return RADIO_OPEN;
    }

    // Safety: disable any scope/waterfall streaming so CI-V replies are
    // not interleaved with scope frames.
    uint8_t zero[1] = {0};
    rc = radio_command(radio, 0x27, 0x10, -1, zero, 1, NULL, 0);
    if (rc != RADIO_OK) {
        fprintf(stderr, "Unable to disable scope output\n");
        return rc;
    }

    uint64_t id = 0;
    rc = radio_command(radio, 0x19, 0x00, -1, NULL, 0, &id, 0);
    if (rc != RADIO_OK) {
        fprintf(stderr, "Unexpected reply from radio while getting transceiver ID\n");
        return rc;
    }
    radio->transceiver_id = id;

    rc = radio_set_satellite_mode(radio, 0);
    if (rc != RADIO_OK) {
        fprintf(stderr, "Error disabling satellite mode\n");
        return rc;
    }

    // Park Sub on VHF, out of the way of Main's UHF carrier. Best-effort:
    // the 9700 NGs 0x07 0xD1 if Dualwatch is off, which we don't want to
    // block init on. Worst case the operator parks Sub manually. Skip the
    // rest of the Sub block if we can't enter it; do NOT fall through into
    // set_mode / set_frequency because those would land on Main.
    double sub_park = radio->sub_park_frequency > 0.0
                      ? radio->sub_park_frequency
                      : RADIO_SUB_PARK_HZ;
    rc = radio_set_vfo(radio, VFOSub);
    if (rc != RADIO_OK) {
        fprintf(stderr,
                "Warning: could not select Sub band (Dualwatch off?); "
                "skipping Sub-park. Main VFO will still be configured.\n");
    } else {
        rc = radio_set_vfo(radio, VFOA);
        if (rc != RADIO_OK) { fprintf(stderr, "Error selecting VFO A on Sub\n"); return rc; }
        rc = radio_set_mode(radio, RADIO_MODE_FM, RADIO_FILTER_FIL1);
        if (rc != RADIO_OK) { fprintf(stderr, "Error setting Sub mode\n"); return rc; }
        rc = radio_set_frequency(radio, sub_park);
        if (rc != RADIO_OK) { fprintf(stderr, "Error parking Sub frequency\n"); return rc; }
    }

    // Main: the only VFO simple_sat_ops uses on-air.
    rc = radio_set_vfo(radio, VFOMain);
    if (rc != RADIO_OK) { fprintf(stderr, "Error selecting Main band\n"); return rc; }
    rc = radio_set_vfo(radio, VFOA);
    if (rc != RADIO_OK) { fprintf(stderr, "Error selecting VFO A on Main\n"); return rc; }
    rc = radio_set_mode(radio, RADIO_MODE_FM, RADIO_FILTER_FIL1);
    if (rc != RADIO_OK) { fprintf(stderr, "Error setting Main mode\n"); return rc; }
    rc = radio_set_frequency(radio, radio->nominal_downlink_frequency);
    if (rc != RADIO_OK) { fprintf(stderr, "Error setting Main (carrier) frequency\n"); return rc; }

    double f = radio_get_frequency(radio);
    if (f < 0) {
        fprintf(stderr, "Error reading Main frequency\n");
        return RADIO_GET_FREQUENCY;
    }
    radio->vfo_main_actual_frequency = f;

    return RADIO_OK;
}

void radio_connect(radio_t *radio)
{
    radio->connected = 0;
    // radio->fd = open(radio->device_filename, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    radio->fd = open(radio->device_filename, O_RDWR | O_NOCTTY | O_SYNC);
    if (radio->fd == -1) {
        perror("Error opening serial port");
        return;
    }

    // fcntl(radio->fd, F_SETFL, O_NONBLOCK);

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
    // Do not block for bytes
    radio->tty.c_cc[VMIN] = 0;
    // Wait up to 0.1 s
    radio->tty.c_cc[VTIME] = 1;

    radio->tty.c_cflag |= (CLOCAL | CREAD);
    radio->tty.c_cflag &= ~(PARENB | PARODD);
    radio->tty.c_cflag &= ~CSTOPB;
    radio->tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(radio->fd, TCSANOW, &radio->tty) != 0) {
        perror("Error setting serial port attributes");
        close(radio->fd);
        return;
    }
    // Flush buffers
    tcflush(radio->fd, TCIOFLUSH);

    radio->connected = 1;

    return;
}

void radio_disconnect(radio_t *radio)
{
    if (radio->connected) {
        close(radio->fd);
        radio->connected = 0;
    }

    return;
}

int radio_command(radio_t *radio, uint8_t cmd, int16_t subcmd, int16_t subsubcmd, uint8_t *send_data, int len, uint64_t *received_value, uint8_t reverse_value)
{
    // Refuse I/O on an unconnected radio: fd may be zero-initialized (= stdin),
    // which would cause read() to block the UI thread waiting for keyboard input.
    if (!radio->connected) {
        return RADIO_BAD_RESPONSE;
    }
    uint8_t telemetry[RADIO_MAX_COMMAND_LEN] = {0};
    size_t offset = 0;
    telemetry[offset++] = 0xFE;
    telemetry[offset++] = 0xFE;
    telemetry[offset++] = 0xA2;
    telemetry[offset++] = 0xE0;
    telemetry[offset++] = cmd;
    if (subcmd >= 0) {
        telemetry[offset++] = (uint8_t)subcmd;
    }
    if (subsubcmd >= 0) {
        telemetry[offset++] = (uint8_t)subsubcmd;
    }
    if (send_data != NULL && len > 0) {
        // Binary coded decimal 0 to 9
        if(reverse_value) {
            for (int i = len-1; i >= 0; --i) {
                telemetry[offset++] = send_data[i];
            }
        } else {
            for (int i = 0; i < len; ++i) {
                telemetry[offset++] = send_data[i];
            }
        }
    }
    telemetry[offset++] = 0xFD;
    printcmd("Radio command:", telemetry, offset);

    // Clear the transmit and receive buffers 
    // (if this command has priority?)
    tcflush(radio->fd, TCIOFLUSH);
    // Send the command
    ssize_t bytes_sent = write(radio->fd, telemetry, offset); 
    if (bytes_sent != offset) {
        return RADIO_BAD_RESPONSE;
    }
    // TODO set a timeout for response
    radio->result[0] = '\0';
    ssize_t bytes_received = -1;
    int remaining_buffer = RADIO_MAX_COMMAND_RESULT_LEN;
    offset = 0;
    while (bytes_received != 0) {
        bytes_received = read(radio->fd, radio->result + offset, remaining_buffer);
        if (bytes_received == -1) {
            return RADIO_BAD_RESPONSE;
        }
        offset += bytes_received;
        remaining_buffer -= bytes_received;
        if (remaining_buffer <= 0) {
            break;
        }
    }
    radio->result_len = offset;
    printcmd("Radio response", radio->result, radio->result_len);

    if (radio->result_len < 6 || ((uint32_t*)radio->result)[0] != 0xA2E0FEFE) {
        return RADIO_BAD_RESPONSE;
    }

    uint8_t cmd_response = radio->result[4];
    if (cmd_response == 0xFA) {
        return RADIO_NG;
    } else if (cmd_response == 0xFB) {
        return RADIO_OK;
    }

    uint8_t subcmd_response = 0;
    uint8_t subsubcmd_response = 0;
    // start of data
    offset = 5;
    if (subcmd >= 0) {
       if (radio->result_len < 7) {
           return RADIO_BAD_RESPONSE;
       } else {
           subcmd_response = radio->result[5];
           if (subcmd_response != subcmd) {
               return RADIO_BAD_RESPONSE;
           }
           offset++;
       }
    }
    if (subsubcmd >= 0) {
       if (radio->result_len < 8) {
           return RADIO_BAD_RESPONSE;
       } else {
           subsubcmd_response = radio->result[6];
           if (subsubcmd_response != subsubcmd) {
               return RADIO_BAD_RESPONSE;
           }
           // TODO check if this should be += 1 for some commands?
           offset += 2;
       }
    }

    if (received_value != NULL) { 
        // printf("offset: %lu\n", offset);
        int32_t index = reverse_value ? radio->result_len - 2 : offset;
        int8_t increment = reverse_value ? -1 : 1;
        uint8_t byte = radio->result[index];
        index += increment;
        // Debugging
        // fprintf(stderr, "Response:"); 
        // Get value
        int64_t val = 0;
        while (index >= offset-1 && index != radio->result_len) {
            // fprintf(stderr, " %02X", byte);
            val *= 10;
            val += byte >> 4;
            val *= 10;
            val += (byte & 0b1111);
            byte = radio->result[index];
            index += increment;
        }
        // fprintf(stderr, "\n");
        *received_value = val;
    }

    return RADIO_OK;
}

int radio_set_vfo(radio_t *radio, int vfo)
{
    int radio_result = 0;
    radio_result = radio_command(radio, 0x07, vfo, -1, NULL, 0, NULL, 0);
    if (radio_result != RADIO_OK) {
        return radio_result;
    }
    return RADIO_OK;
}

double radio_get_frequency(radio_t *radio)
{
    // Read the operating frequency
    uint64_t frequency = 0;
    int radio_result = radio_command(radio, 0x03, -1, -1, NULL, 0, &frequency, 1);
    if (radio_result != RADIO_OK) {
        return -1.0;
    }

    return (double)frequency;
}

int radio_set_frequency(radio_t *radio, double frequency)
{
    uint64_t new_frequency = (uint64_t)round(frequency);
    uint8_t new_freq[5] = {0};
    // BCD
    for (int i = 0; i < 5; ++i) {
        new_freq[i] |= ((new_frequency % 10) & 0b1111);
        new_frequency /= 10;
        new_freq[i] |= (((new_frequency % 10) & 0b1111) << 4);
        new_frequency /= 10;
    }
    int radio_result = radio_command(radio, 0x05, -1, -1, new_freq, 5, NULL, 0);
    if (radio_result != RADIO_OK) {
        return RADIO_SET_FREQUENCY;
    }
    return RADIO_OK;
}

int radio_get_satellite_mode(radio_t *radio)
{
    int radio_result = 0;
    uint64_t value = 0;
    radio_result = radio_command(radio, 0x16, 0x5A, -1, NULL, 0, &value, 0);
    if (radio_result != RADIO_OK) {
        return -1;
    }
    return (int)value;
}

int radio_set_satellite_mode(radio_t *radio, int sat_mode)
{
    int radio_result = 0;
    uint8_t data[1];
    data[0] = sat_mode;
    radio_result = radio_command(radio, 0x16, 0x5A, -1, data, 1, NULL, 0);
    if (radio_result != RADIO_OK) {
        return radio_result;
    }
    radio->satellite_mode = sat_mode;
    return RADIO_OK;
}

int radio_set_mode(radio_t *radio, int mode, int filter)
{
    int radio_result = 0;
    uint8_t data[2] = {0};
    data[0] = mode;
    data[1] = filter;
    radio_result = radio_command(radio, 0x06, -1, -1, data, 2, NULL, 0);
    if (radio_result != RADIO_OK) {
        return radio_result;
    }
    return RADIO_OK;
}

// CI-V `1A 06 <on> <filter>`: data-mode flag + filter for the currently
// selected operating mode. Filter byte must be 0x00 when disabling.
// The `filter` argument picks which IF RX bandwidth (FIL1/2/3) is active;
// it does NOT affect TX audio bandwidth on the IC-9700.
// What DATA mode actually does on the 9700: bypasses pre-emphasis, mic AGC,
// and speech compressor so the MOD source (USB in our uplink flow) reaches
// the FM modulator with a linear amplitude response. It does NOT widen
// the TX audio passband beyond the voice-band rolloff (~300 Hz–2.9 kHz)
// that is hardwired into the FM TX chain. This was empirically verified by
// the 3 kHz rolloff in the TX spectrum during 9600 bps testing — the
// preamble fundamental at 4.8 kHz gets clobbered. For this rig, 2400 bps
// (1.2 kHz fundamental) is the practical ceiling through the USB/FM-DATA
// path. 9600 bps requires a radio with a genuine direct-FSK modulator
// input (Kenwood TM-D710, TS-2000) or a software-defined radio.
int radio_set_data_mode(radio_t *radio, int on, int filter)
{
    uint8_t data[2];
    data[0] = on ? 0x01 : 0x00;
    data[1] = on ? (uint8_t)filter : 0x00;
    return radio_command(radio, 0x1A, 0x06, -1, data, 2, NULL, 0);
}

// CI-V `1A 05 01 16 <src>`: menu item 0x0116 "DATA MOD" (SET > Connectors >
// MOD Input > DATA MOD). Selects which audio source feeds the modulator
// while DATA mode is on.
int radio_set_data_mod_source(radio_t *radio, int source)
{
    uint8_t data[3];
    data[0] = 0x01;
    data[1] = 0x16;
    data[2] = (uint8_t)source;
    return radio_command(radio, 0x1A, 0x05, -1, data, 3, NULL, 0);
}

// CI-V `1A 05 01 13 <BCD4>`: menu item 0x0113 "USB MOD Level". Value is a
// 2-byte packed-BCD percentage 0000..0255, carrying 0..100% in steps of
// ~0.4 percent. Tune empirically to meet the AX100's expected TX deviation.
int radio_set_usb_mod_level(radio_t *radio, int level_0_to_255)
{
    if (level_0_to_255 < 0)   level_0_to_255 = 0;
    if (level_0_to_255 > 255) level_0_to_255 = 255;
    int hundreds = level_0_to_255 / 100;
    int ones     = level_0_to_255 % 100;
    uint8_t data[4];
    data[0] = 0x01;
    data[1] = 0x13;
    data[2] = (uint8_t)(((hundreds / 10) << 4) | (hundreds % 10));
    data[3] = (uint8_t)(((ones / 10)     << 4) | (ones     % 10));
    return radio_command(radio, 0x1A, 0x05, -1, data, 4, NULL, 0);
}

// CI-V `14 15 <BCD4>`: Monitor audio (MONI) level. Value is a 2-byte
// packed-BCD 0000..0255 carrying 0..100% of monitor gain. MONI itself
// is a front-panel toggle (no CI-V on/off), so setting a level only
// has audible effect once MONI is on. Useful for bringing up --record=
// loopback. Subcommand 0x07 (which this used to be) is Inner PBT — the
// radio returns NG to 14 07 outside SSB, which was the source of the
// "could not set MONI level (rc=2)" warnings.
int radio_set_moni_level(radio_t *radio, int level_0_to_255)
{
    if (level_0_to_255 < 0)   level_0_to_255 = 0;
    if (level_0_to_255 > 255) level_0_to_255 = 255;
    int hundreds = level_0_to_255 / 100;
    int ones     = level_0_to_255 % 100;
    uint8_t data[2];
    data[0] = (uint8_t)(((hundreds / 10) << 4) | (hundreds % 10));
    data[1] = (uint8_t)(((ones / 10)     << 4) | (ones     % 10));
    return radio_command(radio, 0x14, 0x15, -1, data, 2, NULL, 0);
}

// Convenience: put the IC-9700 in the full configuration needed for an
// AX100 uplink. FM operating mode + DATA on + Data Mod source = USB.
// Does not touch USB MOD Level — call radio_set_usb_mod_level() separately
// when you know the target deviation for your station.
int radio_uplink_prep(radio_t *radio)
{
    int rc = radio_set_mode(radio, RADIO_MODE_FM, RADIO_FILTER_FIL1);
    if (rc != RADIO_OK) return rc;
    rc = radio_set_data_mode(radio, 1, RADIO_FILTER_FIL1);
    if (rc != RADIO_OK) return rc;
    rc = radio_set_data_mod_source(radio, RADIO_DATA_MOD_SRC_USB);
    if (rc != RADIO_OK) return rc;
    return RADIO_OK;
}

int radio_ptt(radio_t *radio, int on)
{
    uint8_t data[1];
    data[0] = on ? 1 : 0;
    return radio_command(radio, 0x1C, 0x00, -1, data, 1, NULL, 0);
}


// Band: 0 main, 1 sub
int radio_get_band_selection(radio_t *radio, int band)
{
    int radio_result = 0;
    uint64_t value = 0;
    radio_result = radio_command(radio, 0x07, 0xD2, band, NULL, 0, &value, 0);
    if (radio_result != RADIO_OK) {
        return -1;
    }
    return (int)value;
}

int radio_set_band_selection(radio_t *radio, int band)
{
    int radio_result = 0;
    uint8_t data[1];
    data[0] = 1;
    radio_result = radio_command(radio, 0x16, 0x5A, band, data, 1, NULL, 0);
    if (radio_result != RADIO_OK) {
        return radio_result;
    }
    return RADIO_OK;
}

int radio_toggle_waterfall(radio_t *radio)
{
    int radio_result = 0;
    uint8_t data[1];
    uint8_t enabled = !radio->waterfall_enabled;
    uint64_t scope_status = 0;
    radio_result = radio_command(radio, 0x27, 0x10, -1, NULL, 0, &scope_status, 0);
    if (radio_result != RADIO_OK) {
        fprintf(stderr, "Unable to get scope status\n");
        return radio_result;
    }
    if (scope_status != enabled) {
        data[0] = enabled;
        radio_result = radio_command(radio, 0x27, 0x10, -1, data, 1, NULL, 0);
        if (radio_result != RADIO_OK) {
            fprintf(stderr, "Unable to set scope status\n");
            return radio_result;
        }
    }
    // Set scope to main VFO
    data[0] = 0;
    radio_result = radio_command(radio, 0x27, 0x12, -1, data, 1, NULL, 0);
    if (radio_result != RADIO_OK) {
        fprintf(stderr, "Unable to set scope to Main VFO, %d\n", radio_result);
        return radio_result;
    }
    data[0] = enabled;
    radio_result = radio_command(radio, 0x27, 0x11, -1, data, 1, NULL, 0);
    if (radio_result != RADIO_OK) {
        fprintf(stderr, "Unable to set waveform data status\n");
        return radio_result;
    }
    radio->waterfall_enabled = enabled;
    return RADIO_OK;
}

