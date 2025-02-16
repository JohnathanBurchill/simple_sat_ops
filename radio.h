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
} radio_t;

void radio_connect(radio_t *radio);
void radio_disconnect(radio_t *radio);
int radio_send_command(radio_t *radio, uint8_t cmd, int16_t subcmd, int16_t subsubcmd, uint8_t *data, int len, uint64_t *return_value, uint8_t reverse_value);
int radio_set_satellite_mode(radio_t *radio, int sat_mode);


#endif // !RADIO_H


