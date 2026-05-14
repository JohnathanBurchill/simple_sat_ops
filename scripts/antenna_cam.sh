#!/usr/bin/env bash
# antenna_cam.sh — live view of the rao antenna webcam, in a small
# always-on-top corner window. Run from your laptop, NOT on rao.
#
# How it works:
#   ssh user@rao 'ffmpeg /dev/videoN -> mjpeg passthrough' \
#     | mpv (or ffplay) -
# The remote ffmpeg doesn't transcode — it just packs the camera's
# native MJPEG into a stream over the SSH pipe. mpv renders with low
# latency and lets us pin the window to the top-right of the screen.
#
# Overrides (env vars or positional args):
#   REMOTE   ssh target              (default: va6)
#   DEV      v4l2 device on rao      (default: /dev/video0)
#   SIZE     ffmpeg -video_size      (default: 640x480)
#   TITLE    window title            (default: antenna-cam)
#
# Examples:
#   ./antenna_cam.sh
#   REMOTE=johnathan@rao DEV=/dev/video1 ./antenna_cam.sh
#   ./antenna_cam.sh va6rao@rao /dev/video2 1280x720

set -euo pipefail

REMOTE="${1:-${REMOTE:-rao}}"
DEV="${2:-${DEV:-/dev/video0}}"
SIZE="${3:-${SIZE:-640x480}}"
TITLE="${TITLE:-antenna-cam}"

# Pick a local viewer. mpv gets first pick because --geometry lets us
# anchor to the top-right corner; ffplay is the fallback.
if command -v mpv >/dev/null 2>&1; then
    viewer=(mpv
            --title="$TITLE"
            --ontop
            --geometry=480x360+100%+0
            --no-audio
            --force-window=immediate
            --video-sync=desync
            --untimed
            --cache=no
            --demuxer-readahead-secs=0
            --no-osc --no-osd-bar --no-input-default-bindings
            --demuxer=lavf --demuxer-lavf-format=mjpeg
            --no-correct-pts --container-fps-override=25
            -)
elif command -v ffplay >/dev/null 2>&1; then
    viewer=(ffplay
            -loglevel warning
            -window_title "$TITLE"
            -x 480 -y 360
            -fflags nobuffer -flags low_delay -framedrop
            -an
            -f mjpeg -i -)
else
    echo "antenna_cam: install mpv or ffmpeg locally (brew install mpv)" >&2
    exit 1
fi

# -T: NO pseudo-tty. ssh stdout stays in raw mode so MJPEG bytes go
#     through binary-clean. (-tt cooks the channel and mangles \n
#     bytes inside JPEG payloads, which breaks the decoder.)
# -e none: disable the SSH escape char so a literal '~' byte in the
#     stream doesn't get interpreted.
# Remote ffmpeg exits on SIGPIPE when the local viewer closes, so we
# don't need a tty just for teardown.
ssh -T -e none \
    -o ServerAliveInterval=30 \
    "$REMOTE" "ffmpeg -nostdin -loglevel error \
        -f v4l2 -input_format mjpeg -video_size $SIZE -i $DEV \
        -c copy -f mjpeg pipe:1" \
  | "${viewer[@]}"
