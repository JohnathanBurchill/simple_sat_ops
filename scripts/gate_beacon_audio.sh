#!/usr/bin/env bash
# gate_beacon_audio.sh
#
# Offline post-pass scrubber: take a long wideband WAV (typically a
# capture from rx_capture / b210_rx_capture) and produce a copy that's
# silent everywhere the satellite wasn't transmitting. Same idea as
# the live --monitor squelch in simple_sat_ops, but as a one-shot ffmpeg
# pipeline so an existing recording can be cleaned up without re-running
# the receiver.
#
# How: split the audio in parallel into a detector path that bandpasses
# around 4800 Hz (Nyquist of the 9600 baud AX100 GMSK symbol rate) with
# 100 Hz bandwidth and boosts +40 dB. That tap carries strong tonal
# energy whenever the satellite is keying and almost none in FM noise
# or silence. The detector then drives a sidechain gate on the main
# path — open above threshold, crushed by ~80 dB below.
#
# Usage:
#   gate_beacon_audio.sh <input.wav> [output.wav] [--duration <seconds>]
#
# Defaults: output = <input-stem>_gated.wav, duration = entire file.

set -euo pipefail

# Detector and gate parameters. Tuned by the original ffmpeg one-liner
# this script grew out of; keep them named so they're easy to retune
# without diving into the filter graph.
DET_FREQ=4800       # Hz — half the 9600 baud GMSK symbol rate
DET_BW=100          # Hz — narrow enough to reject FM noise floor
DET_GAIN_DB=40      # dB — boost detector tap so threshold is meaningful
THRESHOLD=0.25      # gate opens when detector envelope crosses this
RATIO=9000          # essentially hard-gate (very high reduction below)
RANGE=0.0001        # ~ -80 dB floor when gate is closed
ATTACK_MS=15        # fast enough to catch burst heads
RELEASE_MS=80       # short enough to silence inter-burst gaps

usage() {
  echo "usage: $(basename "$0") <input.wav> [output.wav] [--duration <seconds>]" >&2
  exit 2
}

INPUT=""
OUTPUT=""
DURATION=""

while [ $# -gt 0 ]; do
  case "$1" in
    --duration)
      [ $# -ge 2 ] || usage
      DURATION="$2"
      shift 2
      ;;
    --duration=*) DURATION="${1#*=}"; shift ;;
    -h|--help) usage ;;
    --) shift; break ;;
    -*) echo "unknown option: $1" >&2; usage ;;
    *)
      if [ -z "$INPUT" ]; then INPUT="$1"
      elif [ -z "$OUTPUT" ]; then OUTPUT="$1"
      else echo "unexpected positional: $1" >&2; usage
      fi
      shift
      ;;
  esac
done

[ -n "$INPUT" ] || usage
[ -r "$INPUT" ] || { echo "cannot read $INPUT" >&2; exit 1; }

if [ -z "$OUTPUT" ]; then
  OUTPUT="${INPUT%.*}_gated.wav"
fi

# `-t` as an input option (before -i) lets ffmpeg's demuxer stop early
# rather than reading the whole file just to throw the tail away. Empty
# means process the entire input.
DURATION_FLAGS=()
if [ -n "$DURATION" ]; then
  DURATION_FLAGS=(-t "$DURATION")
fi

ffmpeg -hide_banner -y \
  "${DURATION_FLAGS[@]}" -i "$INPUT" \
  -filter_complex "
    [0:a]asplit=2[main][det];
    [det]bandpass=f=${DET_FREQ}:w=${DET_BW},
         bandpass=f=${DET_FREQ}:w=${DET_BW},
         bandpass=f=${DET_FREQ}:w=${DET_BW},
         bandpass=f=${DET_FREQ}:w=${DET_BW},
         volume=${DET_GAIN_DB}dB[detf];
    [main][detf]sidechaingate=threshold=${THRESHOLD}:ratio=${RATIO}:range=${RANGE}:attack=${ATTACK_MS}:release=${RELEASE_MS}
  " \
  -c:a pcm_s16le "$OUTPUT"
