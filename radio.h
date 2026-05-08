/*

   Simple Satellite Operations  radio.h

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

#ifndef RADIO_H
#define RADIO_H

#include <termios.h>
#include <stdint.h>

#include "radio_backend.h"

#define RADIO_MAX_DEVICE_FILENAME_LEN 1024
#define RADIO_MAX_COMMAND_LEN 128
#define RADIO_MAX_COMMAND_RESULT_LEN 1024

#define FRONTIERSAT_CARRIER_HZ 436150000.0

/* simple_sat_ops owns this radio. Sub is never used on-air; park it on an
   unused VHF freq so Main (UHF) never collides with Sub on the same band. */
#define RADIO_SUB_PARK_HZ 145150000.0

enum RADIO_STATUS {
    RADIO_OK = 0,
    RADIO_BAD_RESPONSE,
    RADIO_NG,
    RADIO_DATA,
    RADIO_ERROR,
    RADIO_OPEN,
    RADIO_SET_FREQUENCY,
    RADIO_GET_FREQUENCY,
    RADIO_NOT_SUPPORTED,    // backend doesn't expose this op
    RADIO_TX_INHIBITED,     // PTT-on attempted before --allow-tx was given
    RADIO_NOT_IMPLEMENTED,  // backend op exists but isn't wired up yet
};

enum RADIO_VFO {
    VFOA = 0,
    VFOB = 1,
    VFOMain = 0xD0,
    VFOSub = 0xD1,
};

enum RADIO_MODE {
    RADIO_MODE_LSB = 0x00,
    RADIO_MODE_USB = 0x01,
    RADIO_MODE_AM = 0x02,
    RADIO_MODE_CW = 0x03,
    RADIO_MODE_RTTY = 0x04,
    RADIO_MODE_FM = 0x05,
    RADIO_MODE_CW_R = 0x07,
    RADIO_MODE_RTTY_R = 0x08,
    RADIO_MODE_DV = 0x17,
    RADIO_MODE_DD = 0x22,
};

enum RADIO_FILTER {
    RADIO_FILTER_FIL1 = 0x01,
    RADIO_FILTER_FIL2 = 0x02,
    RADIO_FILTER_FIL3 = 0x03,
};

// CI-V `1A 05 01 16 <src>` — SET > Connectors > MOD Input > DATA MOD
// (IC-9700-shaped numbering; backends translate to their own concept.)
enum RADIO_DATA_MOD_SRC {
    RADIO_DATA_MOD_SRC_MIC     = 0x00,
    RADIO_DATA_MOD_SRC_ACC     = 0x01,
    RADIO_DATA_MOD_SRC_MIC_ACC = 0x02,
    RADIO_DATA_MOD_SRC_USB     = 0x03,
    RADIO_DATA_MOD_SRC_MIC_USB = 0x04,
    RADIO_DATA_MOD_SRC_LAN     = 0x05,
};

typedef struct radio
{
    char *device_filename;
    speed_t serial_speed;
    uint8_t connected;
    int fd;
    struct termios tty;
    uint8_t *cmd;
    uint8_t result[RADIO_MAX_COMMAND_RESULT_LEN];
    uint32_t result_len;
    uint32_t transceiver_id;
    uint8_t is_required;
    uint8_t satellite_mode;
    uint8_t operating_mode;
    int satellite_uplink_mode;
    int satellite_downlink_mode;
    double satellite_uplink_frequency;
    double satellite_downlink_frequency;
    double reference_downlink_frequency;
    double nominal_uplink_frequency;
    double nominal_downlink_frequency;
    double sub_park_frequency;
    double doppler_uplink_frequency;
    double doppler_downlink_frequency;
    double vfo_main_actual_frequency;
    double vfo_sub_actual_frequency;
    int doppler_correction_enabled;
    int waterfall_enabled;
    int mode;
    int filter;

    // Backend dispatch. Wired by radio_backend_select() before radio_init().
    // Defaults to icom-civ if untouched (set by radio_init when ops==NULL).
    const radio_backend_ops_t *ops;
    void *backend_state;  // backend-private; opaque to public callers

    // PTT-on is rejected while this is 0 (the default). Callers set it via
    // their own --allow-tx flag handling. PTT-off is always passed through.
    int tx_inhibit_cleared;

    // Raw "release PTT now" wire bytes, populated by the backend during
    // init(). Callable from a SIGINT handler via a single async-signal-safe
    // write(fd, ...) — radio_ptt(0) goes through the dispatcher and is not
    // signal-safe (fprintf, malloc, etc). Length 0 means the backend has
    // no out-of-band PTT-release path (e.g. B210 has no PTT to release).
    uint8_t ptt_off_raw[16];
    uint8_t ptt_off_raw_len;

    // When set, backends with ASCII wire protocols (Yaesu CAT) print
    // wire activity as raw hex bytes rather than the human-readable
    // ASCII string. IC-9700 / CI-V is binary so its trace is always hex.
    int debug_wire;
} radio_t;

int radio_init(radio_t *radio);
void radio_connect(radio_t *radio);
void radio_disconnect(radio_t *radio);
int radio_command(radio_t *radio, uint8_t cmd, int16_t subcmd, int16_t subsubcmd, uint8_t *data, int len, uint64_t *return_value, uint8_t reverse_value);
int radio_set_vfo(radio_t *radio, int vfo);
double radio_get_frequency(radio_t *radio);
int radio_set_frequency(radio_t *radio, double frequency);
int radio_get_satellite_mode(radio_t *radio);
int radio_set_satellite_mode(radio_t *radio, int sat_mode);
int radio_set_mode(radio_t *radio, int mode, int filter);
int radio_set_data_mode(radio_t *radio, int on, int filter);
int radio_set_data_mod_source(radio_t *radio, int source);
// Prep the radio for clean data RX. Yaesu drives the full FT-991A
// downlink-prep checklist: auto-info off, wide IF, all DSPs off
// (NB / NR / auto-notch / manual-notch / contour), AGC FAST, RF
// front-end at max sensitivity (preamp on, attenuator off, RF gain
// max), IF shift centred, repeater shift / CTCSS cleared, and
// Menu 079 = 9600 so the rear DATA-OUT carries the wide pre-de-
// emphasis discriminator output. Issued before rx_capture / rx_live
// to keep 9600-baud bit transitions undistorted.
int radio_set_rx_clean(radio_t *radio);
int radio_set_usb_mod_level(radio_t *radio, int level_0_to_255);
int radio_set_moni_level(radio_t *radio, int level_0_to_255);
int radio_set_rf_power(radio_t *radio, int level_0_to_255);
int radio_set_rf_power_watts(radio_t *radio, int watts);
// Carrier squelch level, 0..100. radio_get_squelch returns the level on
// success or -1 on error / unsupported.
int radio_set_squelch(radio_t *radio, int level_0_to_100);
int radio_get_squelch(radio_t *radio);
// Send raw ASCII CAT bytes (caller includes the protocol terminator) and
// optionally read any reply. Returns RADIO_OK including the no-reply
// case; reply[0] == '\0' means "command sent, nothing came back".
int radio_cat_send(radio_t *radio, const char *cmd, char *reply, int reply_cap);
int radio_uplink_prep(radio_t *radio);
int radio_ptt(radio_t *radio, int on);
int radio_power(radio_t *radio, int on);
int radio_get_band_selection(radio_t *radio, int band);
int radio_set_band_selection(radio_t *radio, int band);
int radio_toggle_waterfall(radio_t *radio);


#endif // !RADIO_H
