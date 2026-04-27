#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "usage: $(basename "$0") <antenna|rig> [outdir]" >&2
    exit 2
}

[ $# -ge 1 ] || usage

case "$1" in
    antenna) DEV=/dev/video0 ;;
    rig)     DEV=/dev/video2 ;;
    *)       usage ;;
esac

OUTDIR="${2:-/tmp}"
mkdir -p "$OUTDIR"

STAMP=$(date -u +%Y%m%dT%H%M%SZ)
OUT="${OUTDIR}/${1}_${STAMP}.jpg"

ffmpeg -hide_banner -loglevel error -y \
    -f v4l2 -input_format mjpeg -video_size 1920x1080 -i "$DEV" \
    -ss 0:00:01 -frames:v 1 -q:v 2 "$OUT" 2>/dev/null \
|| ffmpeg -hide_banner -loglevel error -y \
    -f v4l2 -i "$DEV" \
    -ss 0:00:01 -frames:v 1 -q:v 2 "$OUT"

echo "$OUT"
