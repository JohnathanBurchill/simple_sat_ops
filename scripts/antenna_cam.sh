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
# Options:
#   --with-timelapse=N   while the live view runs, branch the MJPEG
#                        stream into a local ffmpeg that encodes a
#                        Discord-sized (h264 480p, ~500 kbps) timelapse
#                        MP4 sped up by N. Output written when the
#                        viewer exits; path printed to stderr.
#   --timelapse-out=PATH override the timelapse output path
#                        (default /tmp/antenna_cam_<UTC>_Nx.mp4).
#
# Overrides (env vars or positional args after the flags):
#   REMOTE   ssh target              (default: rao)
#   DEV      v4l2 device on rao      (default: /dev/video0)
#   SIZE     ffmpeg -video_size      (default: 640x480)
#   TITLE    window title            (default: antenna-cam)
#
# Examples:
#   ./antenna_cam.sh
#   ./antenna_cam.sh --with-timelapse=30
#   REMOTE=johnathan@rao DEV=/dev/video1 ./antenna_cam.sh
#   ./antenna_cam.sh va6rao@rao /dev/video2 1280x720

set -euo pipefail

SPEEDUP=0
TIMELAPSE_OUT=""
POSITIONAL=()
while [ $# -gt 0 ]; do
    case "$1" in
        --with-timelapse=*) SPEEDUP="${1#*=}"; shift ;;
        --with-timelapse)   SPEEDUP="${2:?--with-timelapse needs a speedup}"; shift 2 ;;
        --timelapse-out=*)  TIMELAPSE_OUT="${1#*=}"; shift ;;
        --timelapse-out)    TIMELAPSE_OUT="${2:?--timelapse-out needs a path}"; shift 2 ;;
        -h|--help)
            awk '/^#!/{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0" >&2
            exit 0 ;;
        --) shift; POSITIONAL+=("$@"); break ;;
        *)  POSITIONAL+=("$1"); shift ;;
    esac
done
set -- "${POSITIONAL[@]+"${POSITIONAL[@]}"}"

if [ "$SPEEDUP" -lt 0 ] 2>/dev/null; then
    echo "antenna_cam: --with-timelapse needs a positive integer speedup" >&2
    exit 2
fi

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

# If --with-timelapse=N is set, spin up a local ffmpeg in the
# background reading MJPEG from a FIFO, encoding to MP4 with PTS
# divided by N and resampled to 30 fps. The main SSH pipeline tees
# into that FIFO. When the viewer exits, tee closes the FIFO,
# ffmpeg reads EOF, finalises the MP4 (faststart moov), and the
# trap waits for it before printing the file size.
FIFO=""
FFMPEG_PID=""
if [ "$SPEEDUP" -gt 0 ]; then
    if ! command -v ffmpeg >/dev/null 2>&1; then
        echo "antenna_cam: --with-timelapse needs local ffmpeg on PATH" >&2
        exit 1
    fi
    if [ -z "$TIMELAPSE_OUT" ]; then
        TIMELAPSE_OUT="/tmp/antenna_cam_$(date -u +%Y%m%dT%H%M%SZ)_${SPEEDUP}x.mp4"
    fi
    FIFO=$(mktemp -u "${TMPDIR:-/tmp}/antenna_cam_fifo.XXXXXX")
    mkfifo "$FIFO"
    # 480p, 30 fps, libx264 CRF 26: under 1 Mbps for a low-motion
    # antenna scene, so a 30-minute live capture at 30x speedup
    # (60 s of video) lands well inside Discord's 25 MB free-tier
    # cap. -pix_fmt yuv420p keeps it playable everywhere.
    ffmpeg -nostdin -loglevel warning \
        -f mjpeg -i "$FIFO" \
        -vf "setpts=PTS/${SPEEDUP},scale=480:-2" -r 30 \
        -an -c:v libx264 -preset veryfast -crf 26 \
        -pix_fmt yuv420p -movflags +faststart \
        -y "$TIMELAPSE_OUT" &
    FFMPEG_PID=$!
    echo "antenna_cam: recording ${SPEEDUP}x timelapse -> $TIMELAPSE_OUT" >&2
fi

cleanup() {
    if [ -n "$FFMPEG_PID" ]; then
        wait "$FFMPEG_PID" 2>/dev/null || true
    fi
    if [ -n "$FIFO" ] && [ -e "$FIFO" ]; then
        rm -f "$FIFO"
    fi
    if [ -n "$TIMELAPSE_OUT" ] && [ -f "$TIMELAPSE_OUT" ]; then
        size=$(du -h "$TIMELAPSE_OUT" | cut -f1)
        echo "antenna_cam: timelapse $TIMELAPSE_OUT ($size)" >&2
    fi
}
trap cleanup EXIT

# -T: NO pseudo-tty. ssh stdout stays in raw mode so MJPEG bytes go
#     through binary-clean. (-tt cooks the channel and mangles \n
#     bytes inside JPEG payloads, which breaks the decoder.)
# -e none: disable the SSH escape char so a literal '~' byte in the
#     stream doesn't get interpreted.
# 'exec ffmpeg ...': replace the remote shell so ffmpeg IS the
#     session leader and gets SIGHUP directly when this side
#     disconnects. Without exec, a parent bash sits between SSH and
#     ffmpeg, ignores SIGHUP, and ffmpeg keeps /dev/video0 open
#     after we exit — the next run hits "Device or resource busy".
ssh_cmd=(ssh -T -e none
         -o ServerAliveInterval=30
         "$REMOTE" "exec ffmpeg -nostdin -loglevel error \
             -f v4l2 -input_format mjpeg -video_size $SIZE -i $DEV \
             -c copy -f mjpeg pipe:1")

if [ -n "$FIFO" ]; then
    "${ssh_cmd[@]}" | tee "$FIFO" | "${viewer[@]}"
else
    "${ssh_cmd[@]}" | "${viewer[@]}"
fi
