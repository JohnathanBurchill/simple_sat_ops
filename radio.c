#include "radio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <err.h>

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
    // Read at least 1 byte
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
    }

    return;
}

int radio_send_command(radio_t *radio, uint8_t cmd, int16_t subcmd, int16_t subsubcmd, uint8_t *data, int len, uint32_t *return_value, uint8_t reverse_value)
{
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
    if (data != NULL && len > 0) {
        // Binary coded decimal 0 to 9
        for (int i = 0; i < len; i+=2) {
            telemetry[offset++] = data[i] << 4 | data[i+1]; 
        }
    }
    telemetry[offset++] = 0xFD;
    // For debugging
    fprintf(stderr, "Command:");
    for (int i = 0; i < offset; ++i) {
        fprintf(stderr, " %02X", telemetry[i]);
    }
    fprintf(stderr, "\n");

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
    // Debugging
    fprintf(stderr, "Radio response:");
    for (int i = 0; i < radio->result_len; ++i) {
        fprintf(stderr, " %x", radio->result[i]);
    }
    fprintf(stderr, "\n");

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

    if (return_value != NULL) { 
        // printf("offset: %lu\n", offset);
        int32_t index = reverse_value ? radio->result_len - 2 : offset;
        int8_t increment = reverse_value ? -1 : 1;
        uint8_t byte = radio->result[index];
        index += increment;
        // Debugging
        // fprintf(stderr, "Response:"); 
        // Get value
        int32_t value = 0;
        while (index >= offset-1 && index != radio->result_len) {
            // fprintf(stderr, " %02X", byte);
            value *= 10;
            value += byte >> 4;
            value *= 10;
            value += (byte & 0b1111);
            byte = radio->result[index];
            index += increment;
        }
        // fprintf(stderr, "\n");
        *return_value = value;
    }

    return RADIO_OK;
}

