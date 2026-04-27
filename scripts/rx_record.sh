#!/usr/bin/env bash
set -uo pipefail

usage() {
    echo "usage: $(basename "$0") [freq_hz] [outdir]" >&2
    echo "  defaults: freq=436150000, outdir=/tmp" >&2
    echo "  press Ctrl-C to stop; .raw and .wav paths are printed on exit" >&2
    exit 2
}

case "${1:-}" in -h|--help) usage ;; esac

FREQ="${1:-436150000}"
OUTDIR="${2:-/tmp}"
PPM=16
GAIN=40
SAMP=48000

mkdir -p "$OUTDIR"
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
RAW="${OUTDIR}/rx_${STAMP}.raw"
WAV="${OUTDIR}/rx_${STAMP}.wav"

echo "rx_record: f=${FREQ} Hz, fs=${SAMP} Hz, gain=${GAIN}, ppm=${PPM}" >&2
echo "rx_record: writing ${RAW}" >&2
echo "rx_record: Ctrl-C to stop." >&2

# Catch SIGINT in this shell so we keep going past rtl_fm's exit and
# still do the WAV conversion. rtl_fm itself receives the same SIGINT
# from the tty's process group and exits cleanly with code 130.
trap : INT
rtl_fm -M fm -f "$FREQ" -s "$SAMP" -g "$GAIN" -p "$PPM" "$RAW"
trap - INT

if [ ! -s "$RAW" ]; then
    echo "rx_record: no samples captured" >&2
    exit 1
fi

ffmpeg -hide_banner -loglevel error -y \
    -f s16le -ar "$SAMP" -ac 1 -i "$RAW" "$WAV"

echo "$RAW"
echo "$WAV"
