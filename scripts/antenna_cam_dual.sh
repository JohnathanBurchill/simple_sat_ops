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
#   REMOTE   ssh target                  (default: rao)
#   SIZE     per-camera capture size     (default: 640x480)
#   FPS      capture/encode framerate    (default: 15)
#   BITRATE  encoder cap, bits/sec        (default: 4000000)
#   VERBOSE  1 = unfiltered ffmpeg/mpv logs (default: 0)
#
# SIZE is per camera; the two are stacked side by side, so the window
# ends up twice that wide (the 640x480 default gives a 1280x480 view).
# The camera has to actually support the requested mode -- check with
#   ssh $REMOTE v4l2-ctl -d /dev/video0 --list-formats-ext
# Note that pixel count drives remote encoder load, so a big SIZE and a
# high FPS together can push the remote off realtime; watch for the
# picture falling behind and back one of them off.
#
# On latency: at N fps each frame is inherently up to 1/N s stale
# before it is even encoded, so 5 fps cost ~200 ms per frame before
# anything else. Raising FPS is the single biggest win; the rest of
# the delay is encoder, mux and player buffering, all pinned open
# below. Drop FPS back to 5 if the link or the remote CPU can't keep
# up (watch the "speed=" figure -- it must stay at 1x).
#
# Temporal (inter-frame) compression is already on: libx264 codes P
# frames against previous frames by default, which is why a mostly
# static antenna scene costs so few bits. What -tune zerolatency
# deliberately gives up is B-frames -- they reference *future* frames,
# so they compress better but force the encoder to hold frames back,
# which is latency. The -maxrate/-bufsize cap below is the other half:
# a small bufsize bounds how much encoded data can queue in the SSH
# pipe, so a burst of motion can't push the view seconds behind.

REMOTE="${REMOTE:-rao}"
SIZE="${SIZE:-640x480}"
FPS="${FPS:-15}"
BITRATE="${BITRATE:-4000000}"
VERBOSE="${VERBOSE:-0}"

# The cameras emit JPEGs with APP markers ffmpeg's mjpeg decoder doesn't
# recognise. It logs one line per frame per camera and decodes them fine
# anyway -- at 15 fps that is 30 lines/sec of pure noise burying any real
# error. It can't be fixed at the source and isn't selectable by
# -loglevel (the decoder logs it at error level), so it gets filtered
# here by text. VERBOSE=1 turns the filtering off.
BENIGN='unable to decode APP fields'

if ! printf '%s' "$SIZE" | grep -Eq '^[0-9]+x[0-9]+$'; then
  echo "antenna_cam_dual: SIZE must look like WIDTHxHEIGHT (got '$SIZE')" >&2
  exit 2
fi

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

if [ "$VERBOSE" = 1 ]; then
  RLOGLEVEL=info
  BENIGN='$^'   # matches nothing, so grep -v passes everything through
  mpv_quiet=()
else
  RLOGLEVEL=error
  mpv_quiet=(--term-status-msg= --msg-level=cplayer=warn)
fi

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
          --demuxer-lavf-o=fflags=+nobuffer
          --demuxer-lavf-probe-info=nostreams
          --demuxer-lavf-analyzeduration=0.1
          --vd-lavc-threads=1
          --video-latency-hacks=yes
          --untimed
          --no-osc --no-osd-bar
          "${mpv_quiet[@]}"
          -)
elif command -v ffplay >/dev/null 2>&1; then
  viewer=(ffplay
          -loglevel warning
          -window_title "antenna-cam-dual"
          -fflags nobuffer -flags low_delay -framedrop
          -probesize 32 -analyzeduration 0 -sync ext
          -an
          -f mpegts -i -)
else
  echo "antenna_cam_dual: install mpv or ffmpeg locally" >&2
  exit 1
fi

echo "Starting stream..."
ssh -T -e none -o ServerAliveInterval=30 "$REMOTE" \
    SIZE="$SIZE" FPS="$FPS" BITRATE="$BITRATE" RLOGLEVEL="$RLOGLEVEL" \
    bash -s 2> >(grep --line-buffered -v "$BENIGN" >&2) <<'REMOTE_SCRIPT' | "${viewer[@]}"
# drawtext needs a real font file. Ubuntu server images don't always
# ship fonts-dejavu-core, so probe the usual paths and fall back to
# whatever fontconfig can find.
RLOGLEVEL="${RLOGLEVEL:-error}"

# scale= wants W:H, -video_size wants WxH. Font and padding scale with
# frame height so the stamp stays the same relative size at any SIZE.
SCALE="${SIZE/x/:}"
HEIGHT="${SIZE#*x}"
FONTSIZE=$((HEIGHT / 16))
[ "$FONTSIZE" -lt 12 ] && FONTSIZE=12
PAD=$((FONTSIZE / 2))

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
exec /usr/bin/ffmpeg -nostdin -hide_banner -nostats -loglevel $RLOGLEVEL \
  -fflags nobuffer -f v4l2 -input_format mjpeg -framerate $FPS -video_size $SIZE -i /dev/video0 \
  -fflags nobuffer -f v4l2 -input_format mjpeg -framerate $FPS -video_size $SIZE -i /dev/video2 \
  -filter_complex "[0:v]scale=$SCALE[v0];[1:v]scale=$SCALE[v1];[v0][v1]hstack[st];\
[st]drawtext=$FONT:text='%{gmtime\:%Y-%m-%dT%H\\\\\:%M\\\\\:%S} UTC':\
fontcolor=white:fontsize=$FONTSIZE:box=1:boxcolor=black@0.55:boxborderw=$PAD:\
x=$PAD:y=h-th-$PAD[out]" \
  -map "[out]" -c:v libx264 -preset ultrafast -tune zerolatency \
  -x264-params "bframes=0:rc-lookahead=0:sync-lookahead=0:sliced-threads=1" \
  -g $((FPS * 2)) -fps_mode cfr -r $FPS \
  -maxrate $BITRATE -bufsize $((BITRATE / 4)) \
  -muxdelay 0 -muxpreload 0 -flush_packets 1 \
  -f mpegts -
REMOTE_SCRIPT
