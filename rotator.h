#ifndef ROTATOR_H
#define ROTATOR_H

#include <termios.h>
#include <stdint.h>

typedef struct antenna_rotator 
{
    char *device_filename;
    uint32_t serial_speed;
    uint8_t connected;
    int fd;
    struct termios tty;
} antenna_rotator_t;


int antenna_rotator_get_position(antenna_rotator_t *antenna_rotator, double *azimuth_degrees, double *elevation_degrees);
int antenna_rotator_set_position(antenna_rotator_t *antenna_rotator, double azimuth_degrees, double elevation_degrees);


#endif // !ROTATOR_H
