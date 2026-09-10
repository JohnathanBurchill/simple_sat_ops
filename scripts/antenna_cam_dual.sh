#!/bin/bash

echo Starting

# ssh "$REMOTE" "ffmpeg \
#   -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video0 \
#   -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video2 \
#   -filter_complex \"\
#     [0:v]scale=640:480[v0]; \
#     [1:v]scale=640:480[v1]; \
#     [v0][v1]hstack[out]\" \
#   -map \"[out]\" -c:v libx264 -preset ultrafast -tune zerolatency -f mpegts -" | ffplay -

#!/bin/bash
# antenna_cam_dual.sh — continuous dual-camera view over SSH, bash-forced remote shell
#
# Overrides:
#   REMOTE   ssh target   (default: rao)

REMOTE="${REMOTE:-rao}"

CLEANED_UP=0
cleanup() {
  [ "$CLEANED_UP" = 1 ] && return
  CLEANED_UP=1
  echo "Cleaning up remote ffmpeg..." >&2
  ssh -T "$REMOTE" "pkill -TERM -f 'ffmpeg.*video0|ffmpeg.*video2'" 2>/dev/null
  exit 0
}
trap cleanup INT TERM EXIT

echo "Clearing any stale camera processes..."
ssh -T "$REMOTE" "pkill -9 ffmpeg 2>/dev/null; sleep 0.5"

# Pick a local viewer. mpv handles a live mpegts pipe on stdin far more
# reliably than ffplay, which tends to probe the stream and then exit
# immediately instead of opening a window. ffplay stays as a fallback.
if command -v mpv >/dev/null 2>&1; then
  viewer=(mpv
          --title="antenna-cam-dual"
          --no-audio
          --force-window=immediate
          --profile=low-latency
          --cache=no
          --demuxer-readahead-secs=0
          --no-osc --no-osd-bar
          -)
elif command -v ffplay >/dev/null 2>&1; then
  viewer=(ffplay
          -loglevel warning
          -window_title "antenna-cam-dual"
          -fflags nobuffer -flags low_delay -framedrop
          -an
          -f mpegts -i -)
else
  echo "antenna_cam_dual: install mpv or ffmpeg locally" >&2
  exit 1
fi

echo "Starting stream..."
ssh -T -e none -o ServerAliveInterval=30 "$REMOTE" bash -s <<'REMOTE_SCRIPT' | "${viewer[@]}"
exec /usr/bin/ffmpeg -nostdin \
  -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video0 \
  -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video2 \
  -filter_complex "[0:v]scale=320:240[v0];[1:v]scale=320:240[v1];[v0][v1]hstack[out]" \
  -map "[out]" -c:v libx264 -preset ultrafast -tune zerolatency -f mpegts -
REMOTE_SCRIPT
