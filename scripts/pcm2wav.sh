#!/usr/bin/env bash

PCMFILE=$1

BASE=$(basename ${PCMFILE} .raw)

WAVFILELEFT="${BASE}_L.wav"
WAVFILERIGHT="${BASE}_R.wav"

echo "Converting ${PCMFILE} to ${WAVFILELEFT}"

ffmpeg -f s16le -ar 48000 -ch_layout stereo -i ${PCMFILE} -af "pan=mono|c0=c0" ${WAVFILELEFT}
ffmpeg -f s16le -ar 48000 -ch_layout stereo -i ${PCMFILE} -af "pan=mono|c0=c1" ${WAVFILERIGHT}

