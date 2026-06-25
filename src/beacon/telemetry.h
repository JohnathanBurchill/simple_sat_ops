/*

    Simple Satellite Operations  telemetry.h

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

#ifndef TELEMETRY_H
#define TELEMETRY_H

enum TELEMETRY_RETURN_CODES {
    TELEMETRY_OK = 0,
};

typedef struct telemetry {
    int decode_enabled;
    int encode_enabled;
    int decoding;
    int encoding;
} telemetry_t;


int init_telemetry_decoder(telemetry_t *state);
int telemetry_decoder_cleanup(telemetry_t *state);
int decode_live_audio_to_telemetry(telemetry_t *state);
int decode_file_audio_to_telemetry(telemetry_t *state, char *filename);
int encode_telecommand_bytes_to_audio(telemetry_t *state);

#endif // TELEMETRY_H
