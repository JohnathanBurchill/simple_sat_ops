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


