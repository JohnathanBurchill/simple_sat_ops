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
