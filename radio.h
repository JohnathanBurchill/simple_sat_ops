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

#define RADIO_MAX_DEVICE_FILENAME_LEN 1024
#define RADIO_MAX_COMMAND_LEN 128
#define RADIO_MAX_COMMAND_RESULT_LEN 1024

enum RADIO_STATUS {
    RADIO_OK = 0,
    RADIO_BAD_RESPONSE,
    RADIO_NG,
    RADIO_DATA,
    RADIO_ERROR,
    RADIO_OPEN,
    RADIO_SET_FREQUENCY,
    RADIO_GET_FREQUENCY,
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
    double doppler_uplink_frequency;
    double doppler_downlink_frequency;
    double vfo_main_actual_frequency;
    double vfo_sub_actual_frequency;
    int doppler_correction_enabled;
    int waterfall_enabled;
    int mode;
    int filter;
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
int radio_get_band_selection(radio_t *radio, int band);
int radio_set_band_selection(radio_t *radio, int band);
int radio_toggle_waterfall(radio_t *radio);


#endif // !RADIO_H


