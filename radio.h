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

typedef struct radio 
{
    char *device_filename;
    uint32_t serial_speed;
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
    double nominal_uplink_frequency;
    double nominal_downlink_frequency;
    double doppler_uplink_frequency;
    double doppler_downlink_frequency;
    double vfo_main_actual_frequency;
    double vfo_sub_actual_frequency;
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
int radio_get_band_selection(radio_t *radio, int band);
int radio_set_band_selection(radio_t *radio, int band);


#endif // !RADIO_H


