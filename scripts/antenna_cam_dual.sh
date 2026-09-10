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
# drawtext needs a real font file. Ubuntu server images don't always
# ship fonts-dejavu-core, so probe the usual paths and fall back to
# whatever fontconfig can find.
FONT=""
for f in /usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf \
         /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf \
         /usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf \
         /usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf; do
  [ -f "$f" ] && { FONT="fontfile=$f"; break; }
done
[ -z "$FONT" ] && FONT="font=monospace"

# %{gmtime} is re-expanded per frame, so the burnt-in clock is the
# capture time on the remote host, in UTC. Drawn last, after hstack,
# so one stamp spans the combined frame.
#
# Escaping is three-deep here: remote bash's double quotes, then the
# filtergraph parser, then drawtext's own %{} expansion -- hence the
# \\\: before each colon in the time format. Note also that drawtext
# splits %{} arguments on whitespace, so the strftime format itself
# must contain no spaces (ISO-8601 'T' separator, label outside).
exec /usr/bin/ffmpeg -nostdin \
  -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video0 \
  -f v4l2 -input_format mjpeg -framerate 5 -video_size 320x240 -i /dev/video2 \
  -filter_complex "[0:v]scale=320:240[v0];[1:v]scale=320:240[v1];[v0][v1]hstack[st];\
[st]drawtext=$FONT:text='%{gmtime\:%Y-%m-%dT%H\\\\\:%M\\\\\:%S} UTC':\
fontcolor=white:fontsize=14:box=1:boxcolor=black@0.55:boxborderw=5:\
x=8:y=h-th-8[out]" \
  -map "[out]" -c:v libx264 -preset ultrafast -tune zerolatency -f mpegts -
REMOTE_SCRIPT
