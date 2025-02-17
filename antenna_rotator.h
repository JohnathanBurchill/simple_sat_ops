#ifndef ANTENNA_ROTATOR_H
#define ANTENNA_ROTATOR_H

#include <termios.h>
#include <stdint.h>

#define AR_CMD_LEN 13
#define AR_RESPONSE_LEN 12

enum ANTENNA_ROTATOR_STATUS {
    ANTENNA_ROTATOR_OK = 0,
    ANTENNA_ROTATOR_BAD_RESPONSE,
    ANTENNA_ROTATOR_ERROR,
    ANTENNA_ROTATOR_OPEN,
    ANTENNA_ROTATOR_ARGS,
};

typedef enum {
    ANTENNA_ROTATOR_STOP = 0x0F,
    ANTENNA_ROTATOR_STATUS = 0x1F,
    ANTENNA_ROTATOR_SET = 0x2F,
} antenna_rotator_command_t;

typedef struct antenna_rotator 
{
    char *device_filename;
    uint32_t serial_speed;
    uint8_t connected;
    int fd;
    struct termios tty;
    int is_required;
    double target_azimuth;
    double target_elevation;
    double azimuth;
    double elevation;
} antenna_rotator_t;

int antenna_rotator_init(antenna_rotator_t *antenna_rotator);
void antenna_rotator_connect(antenna_rotator_t *antenna_rotator);
int antenna_rotator_command(antenna_rotator_t *antenna_rotator, antenna_rotator_command_t cmd, double *azimuth, double *elevation);

#endif // ANTENNA_ROTATOR_H
