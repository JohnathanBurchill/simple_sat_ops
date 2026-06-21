/*

    Simple Satellite Operations  src/audio/miniaudio_impl.c

    The single translation unit that compiles the vendored miniaudio
    library (external/miniaudio/miniaudio.h, public domain / MIT-0). Every
    other file includes miniaudio.h for declarations only; the actual
    implementation lives here behind MINIAUDIO_IMPLEMENTATION.

    We only use the raw playback/capture device API and its PCM ring
    buffer (see src/audio/audio_io.c), so the decoder/encoder/waveform-
    generator subsystems are switched off to keep the build small.

    No GPL header: this file is a thin shim around a third-party
    public-domain library; the simple_sat_ops code that uses it
    (audio_io.c and the ham_* tools) carries the project's GPL notice.
*/

#define MINIAUDIO_IMPLEMENTATION
#define MA_NO_DECODING
#define MA_NO_ENCODING
#define MA_NO_GENERATION

#include "miniaudio.h"
