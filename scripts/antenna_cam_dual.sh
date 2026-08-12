#!/bin/bash

echo Starting

# ssh rao "ffmpeg \
#   -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video0 \
#   -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video2 \
#   -filter_complex \"\
#     [0:v]scale=640:480[v0]; \
#     [1:v]scale=640:480[v1]; \
#     [v0][v1]hstack[out]\" \
#   -map \"[out]\" -c:v libx264 -preset ultrafast -tune zerolatency -f mpegts -" | ffplay -

#!/bin/bash
# antenna_cam_dual.sh — continuous dual-camera view over SSH, bash-forced remote shell

CLEANED_UP=0
cleanup() {
  [ "$CLEANED_UP" = 1 ] && return
  CLEANED_UP=1
  echo "Cleaning up remote ffmpeg..." >&2
  ssh rao bash -c "pkill -TERM -f 'ffmpeg.*video0|ffmpeg.*video2'" 2>/dev/null
  exit 0
}
trap cleanup INT TERM EXIT

echo "Clearing any stale camera processes..."
ssh rao bash -c "pkill -9 ffmpeg 2>/dev/null; sleep 0.5"

echo "Starting stream..."
ssh rao bash -s <<'REMOTE_SCRIPT' | ffplay -fflags nobuffer -flags low_delay -f mpegts -i -
/usr/bin/ffmpeg \
  -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video0 \
  -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video2 \
  -filter_complex "[0:v]scale=320:240[v0];[1:v]scale=320:240[v1];[v0][v1]hstack[out]" \
  -map "[out]" -c:v libx264 -preset ultrafast -tune zerolatency -f mpegts -
REMOTE_SCRIPT
