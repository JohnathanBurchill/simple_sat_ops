/*

    Simple Satellite Operations  telemetry.c

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

#include "telemetry.h"

int init_telemetry_decoder(telemetry_t *state)
{
    state->decoding = 1;
    return TELEMETRY_OK;
}

int telemetry_decoder_cleanup(telemetry_t *state)
{
    state->decoding = 0;
    return TELEMETRY_OK;
}

// Stubs — keep the signatures so the header doesn't need to track
// which of these are wired up yet. (void) casts silence -Wunused-parameter
// without disturbing the prototype.
int decode_live_audio_to_telemetry(telemetry_t *state)
{
    (void) state;
    return TELEMETRY_OK;
}

int decode_audio_file_to_telemetry(telemetry_t *state, char *filename)
{
    (void) state;
    (void) filename;
    return TELEMETRY_OK;
}

int encode_telecommand_bytes_to_audio(telemetry_t *state)
{
    (void) state;
    return TELEMETRY_OK;
}


