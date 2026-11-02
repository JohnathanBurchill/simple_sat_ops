#!/usr/bin/env bash
# antenna_cam_dual.sh — both rao antenna webcams side by side in one
# window, with the capture time burnt into the frame. Run from your
# laptop, NOT on rao.
#
# How it works:
#   ssh user@rao 'ffmpeg DEV1 + DEV2 -> hstack -> drawtext -> h264 mpegts' \
#     | mpv (or ffplay) -
# Unlike antenna_cam.sh this one transcodes on the ground station: the
# two MJPEG streams are decoded, stacked, stamped and re-encoded with
# libx264 before they cross the SSH pipe. Keep an eye on rao's CPU if
# you run it while a pass is being decoded.
#
# Overrides (env vars):
#   REMOTE   ssh target                  (default: rao)
#   DEV1     left camera, v4l2 device    (default: /dev/video0)
#   DEV2     right camera, v4l2 device   (default: /dev/video2)
#   SIZE     per-camera capture size     (default: 640x480)
#   FPS      capture/encode framerate    (default: 15)
#   BITRATE  encoder cap, bits/sec       (default: 4000000)
#   VERBOSE  1 = unfiltered ffmpeg/mpv logs plus the encoder's
#            speed= line                 (default: 0)
#
#   ANTENNA_CAM_ALLOW_REMOTE=1  skip the "you are in an ssh session"
#                        refusal (only if you have a forwarded display)
#
# /dev/videoN numbering can change when a camera is unplugged and
# plugged back in; the names under /dev/v4l/by-id/ on rao don't, so
# they make better DEV1/DEV2 values:
#   ssh rao ls -l /dev/v4l/by-id/
#
# SIZE is per camera; the two are stacked side by side, so the window
# ends up twice that wide (the 640x480 default gives a 1280x480 view).
# The camera has to actually support the requested mode -- check with
#   ssh rao v4l2-ctl -d /dev/video0 --list-formats-ext
# Note that pixel count drives remote encoder load, so a big SIZE and a
# high FPS together can push the remote off realtime; watch for the
# picture falling behind and back one of them off.
#
# On latency: at N fps each frame is inherently up to 1/N s stale
# before it is even encoded, so 5 fps cost ~200 ms per frame before
# anything else. Raising FPS is the single biggest win; the rest of
# the delay is encoder, mux and player buffering, all pinned open
# below. Drop FPS back to 5 if the link or the remote CPU can't keep
# up (run with VERBOSE=1 and watch the "speed=" figure -- it must stay
# at 1x).
#
# Temporal (inter-frame) compression is already on: libx264 codes P
# frames against previous frames by default, which is why a mostly
# static antenna scene costs so few bits. What -tune zerolatency
# deliberately gives up is B-frames -- they reference *future* frames,
# so they compress better but force the encoder to hold frames back,
# which is latency. The -maxrate/-bufsize cap below is the other half:
# a small bufsize bounds how much encoded data can queue in the SSH
# pipe, so a burst of motion can't push the view seconds behind.
#
# Examples:
#   ./antenna_cam_dual.sh
#   FPS=5 ./antenna_cam_dual.sh
#   DEV2=/dev/v4l/by-id/usb-XXXX-video-index0 ./antenna_cam_dual.sh

set -euo pipefail

case "${1:-}" in
    -h|--help)
        awk '/^#!/{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0" >&2
        exit 0 ;;
esac

REMOTE="${REMOTE:-rao}"
DEV1="${DEV1:-/dev/video0}"
DEV2="${DEV2:-/dev/video2}"
SIZE="${SIZE:-640x480}"
FPS="${FPS:-15}"
BITRATE="${BITRATE:-4000000}"
VERBOSE="${VERBOSE:-0}"

if ! printf '%s' "$SIZE" | grep -Eq '^[0-9]+x[0-9]+$'; then
    echo "antenna_cam_dual: SIZE must look like WIDTHxHEIGHT (got '$SIZE')" >&2
    exit 2
fi
for v in FPS BITRATE; do
    if ! printf '%s' "${!v}" | grep -Eq '^[0-9]+$'; then
        echo "antenna_cam_dual: $v must be a whole number (got '${!v}')" >&2
        exit 2
    fi
done

# This script belongs on the operator's laptop. It opens a video window
# HERE and pulls the camera stream FROM the ground station over ssh, so
# running it inside an ssh session on the ground station asks that host to
# ssh back to itself and to draw a window on a machine with no screen. The
# usual symptom is a bare "Hostname rao not found" -- the `rao` alias lives
# in the laptop's ~/.ssh/config and doesn't exist on the ground station --
# which says nothing about what actually went wrong. Say it plainly
# instead. Set ANTENNA_CAM_ALLOW_REMOTE=1 if you really are forwarding a
# display and know what you're doing. (Issue #63.)
if [ -z "${ANTENNA_CAM_ALLOW_REMOTE:-}" ] \
   && { [ -n "${SSH_CONNECTION:-}" ] || [ -n "${SSH_CLIENT:-}" ] \
        || [ -n "${SSH_TTY:-}" ]; }; then
    echo "antenna_cam_dual: this looks like an ssh session on $(hostname)." >&2
    echo "antenna_cam_dual: run this on your own laptop, not on the ground" >&2
    echo "antenna_cam_dual:   station -- it opens the viewer window locally and" >&2
    echo "antenna_cam_dual:   ssh'es to '$REMOTE' itself to fetch the stream." >&2
    echo "antenna_cam_dual: log out of the ssh session and run it again there." >&2
    echo "antenna_cam_dual: (set ANTENNA_CAM_ALLOW_REMOTE=1 to override, e.g." >&2
    echo "antenna_cam_dual:   with a forwarded display.)" >&2
    exit 2
fi

# The cameras emit JPEGs with APP markers ffmpeg's mjpeg decoder doesn't
# recognise. It logs one line per frame per camera and decodes them fine
# anyway -- at 15 fps that is 30 lines/sec of pure noise burying any real
# error. It can't be fixed at the source and isn't selectable by
# -loglevel (the decoder logs it at error level), so it gets filtered
# here by text. VERBOSE=1 turns the filtering off.
BENIGN='unable to decode APP fields'
if [ "$VERBOSE" = 1 ]; then
    BENIGN='$^'   # matches nothing, so grep -v passes everything through
    mpv_quiet=()
else
    mpv_quiet=(--term-status-msg= --msg-level=cplayer=warn)
fi

# Pick a local viewer before touching the remote end, so a laptop with
# neither installed fails here and never opens the cameras. mpv handles
# a live mpegts pipe on stdin far more reliably than ffplay, which tends
# to probe the stream and then exit immediately instead of opening a
# window. ffplay stays as a fallback.
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
            ${mpv_quiet[@]+"${mpv_quiet[@]}"}
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
    echo "antenna_cam_dual: install mpv or ffmpeg locally (brew install mpv)" >&2
    exit 1
fi

# The remote side is a small bash script fed over ssh's stdin. Its first
# line carries our settings, each shell-quoted with printf %q so an odd
# device path can't be misread by the remote shell; the rest is the
# quoted heredoc below, which the local shell leaves untouched.
remote_script() {
    printf 'DEV1=%q DEV2=%q SIZE=%q FPS=%q BITRATE=%q VERBOSE=%q\n' \
        "$DEV1" "$DEV2" "$SIZE" "$FPS" "$BITRATE" "$VERBOSE"
    cat <<'REMOTE_SCRIPT'
if [ "$VERBOSE" = 1 ]; then
    LOGLEVEL=info
    STATS=
else
    LOGLEVEL=error
    STATS=-nostats
fi

# scale= wants W:H, -video_size wants WxH. Font and padding scale with
# frame height so the stamp stays the same relative size at any SIZE.
SCALE="${SIZE/x/:}"
HEIGHT="${SIZE#*x}"
FONTSIZE=$((HEIGHT / 16))
[ "$FONTSIZE" -lt 12 ] && FONTSIZE=12
PAD=$((FONTSIZE / 2))

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
#
# 'exec': ffmpeg replaces this shell, so when the laptop side goes away
# it is ffmpeg itself that sees the closed pipe and exits, releasing the
# cameras. No cleanup kill is needed (see antenna_cam.sh for the history).
exec ffmpeg -nostdin -hide_banner $STATS -loglevel $LOGLEVEL \
  -fflags nobuffer -f v4l2 -input_format mjpeg -framerate $FPS -video_size $SIZE -i "$DEV1" \
  -fflags nobuffer -f v4l2 -input_format mjpeg -framerate $FPS -video_size $SIZE -i "$DEV2" \
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
}

# -T: NO pseudo-tty, so the mpegts bytes come through binary-clean.
# -e none: disable the SSH escape char so a literal '~' byte in the
#     stream doesn't get interpreted.
# Nothing is killed on rao, before or after: if a camera is busy ffmpeg
# says "Device or resource busy" and that is the right answer -- the
# station is shared, and someone else may be watching.
echo "antenna_cam_dual: starting stream from $REMOTE ($DEV1 + $DEV2, $SIZE @ ${FPS} fps)" >&2
remote_script \
    | ssh -T -e none -o ServerAliveInterval=30 "$REMOTE" bash -s \
          2> >(grep --line-buffered -v "$BENIGN" >&2) \
    | "${viewer[@]}"
