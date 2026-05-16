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

int decode_live_audio_to_telemetry(telemetry_t *state)
{
    return TELEMETRY_OK;
}

int decode_audio_file_to_telemetry(telemetry_t *state, char *filename)
{
    return TELEMETRY_OK;
}

int encode_telecommand_bytes_to_audio(telemetry_t *state)
{
    return TELEMETRY_OK;
}


