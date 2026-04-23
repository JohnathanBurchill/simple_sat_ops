#!/usr/bin/env bash

# tx_rx_test.sh — end-to-end transmit / record / decode test driven from
# the Mac side. SSHes to the remote rao box, runs rtl_fm in parallel
# with tx_frame, pulls both recordings back, converts to WAV, renders
# spectrograms, and invokes rx_decode on each.
#
# Usage:
#   scripts/tx_rx_test.sh [--payload=<str>] [--remote=<user@host>]
#                         [--remote-dir=<dir>] [--duration=<sec>]
#                         [--uplink-mod-level=<0..100>]
#                         [--moni-level=<0..100>]
#                         [--bit-rate=<bps>]   (passed to tx_frame and
#                                              rx_decode; must divide 48000)
#                         [--no-reed-solomon]  (match pycsplink downlink
#                                              or legacy non-RS frames —
#                                              default is RS on, matching
#                                              pycsplink uplink)
#                         [--play]
#
# Prerequisites on Mac (local):   ssh, scp, ffmpeg, ffplay
# Prerequisites on remote (rao):  rtl_fm, tx_frame, rx_decode
#
# Outputs (in the CWD on the Mac):
#   rtl_UT=<ts>.raw        over-the-air SDR capture
#   tx_frame_UT=<ts>.raw   9700 MONI loopback
#   <basename>.wav         playable WAV for each raw
#   <basename>_spec.png    spectrogram of each raw
#   <basename>_decode.txt  rx_decode output for each raw

set -euo pipefail

PAYLOAD="HELLO"
REMOTE="va6rao@rao"
# Relative path (resolves to the remote user's home) — avoids the scp
# "No such file or directory" you get when scp is passed a literal
# "$HOME/..." (scp/sftp does not perform shell variable expansion on
# remote paths). Pass an absolute path via --remote-dir= if preferred.
REMOTE_DIR="rxtest"
DURATION=10
UPLINK_MOD_LEVEL=50
MONI_LEVEL=80
BIT_RATE=9600
PLAY=0
RS_FLAG=""

for arg in "$@"; do
    case "$arg" in
        --payload=*)           PAYLOAD="${arg#*=}" ;;
        --remote=*)            REMOTE="${arg#*=}" ;;
        --remote-dir=*)        REMOTE_DIR="${arg#*=}" ;;
        --duration=*)          DURATION="${arg#*=}" ;;
        --uplink-mod-level=*)  UPLINK_MOD_LEVEL="${arg#*=}" ;;
        --moni-level=*)        MONI_LEVEL="${arg#*=}" ;;
        --bit-rate=*)          BIT_RATE="${arg#*=}" ;;
        --no-reed-solomon)     RS_FLAG="--no-reed-solomon" ;;
        --reed-solomon)        RS_FLAG="--reed-solomon" ;;
        --play)                PLAY=1 ;;
        --help|-h)
            sed -n '3,20p' "$0"
            exit 0
            ;;
        *)
            echo "unknown option: $arg" >&2
            exit 1
            ;;
    esac
done

for bin in ssh scp ffmpeg; do
    command -v "$bin" >/dev/null 2>&1 || { echo "missing $bin on PATH" >&2; exit 1; }
done

echo "== remote run: $REMOTE : $REMOTE_DIR =="

# Run the whole TX/RX sequence remotely in one shell so rtl_fm's job
# control is local to that shell and the wait works cleanly.
# The remote script:
#   1) mkdir / cd into $REMOTE_DIR
#   2) fork rtl_fm for $DURATION seconds into a timestamped file
#   3) sleep 1 to let the tuner settle
#   4) run tx_frame (auto-records its own MONI loopback)
#   5) wait for rtl_fm to finish
#   6) echo the two filenames so we can pick them up
ssh "$REMOTE" bash <<REMOTE_EOF
set -euo pipefail
# Non-interactive ssh shells don't source .bashrc / .profile, so
# \$HOME/bin isn't on PATH by default even though tx_frame / rx_decode
# are installed there.
export PATH="\$HOME/bin:\$PATH"
mkdir -p $REMOTE_DIR
cd $REMOTE_DIR
RTL_FILE="rtl_UT=\$(date -u '+%Y%m%dT%H%M%S').raw"
timeout $DURATION rtl_fm -M fm -f 436150000 -s 48000 -g 40 -p 0 "\$RTL_FILE" >/dev/null 2>&1 &
RTL_PID=\$!
sleep 1
echo ">> running tx_frame (bit-rate=$BIT_RATE)" >&2
tx_frame --payload-ascii='$PAYLOAD' --bit-rate=$BIT_RATE --uplink-mod-level=$UPLINK_MOD_LEVEL --moni-level=$MONI_LEVEL $RS_FLAG
wait \$RTL_PID 2>/dev/null || true
# Newest tx_frame and rtl recordings:
TX_FILE=\$(ls -t tx_frame_UT=*.raw | head -n1)
echo "RTL=\$RTL_FILE"
echo "TX=\$TX_FILE"
REMOTE_EOF

# Grab the two filenames from the remote tail echoes. Re-run a tiny
# remote listing since heredoc output is noisy to parse.
read -r RTL_FILE TX_FILE < <(ssh "$REMOTE" "cd $REMOTE_DIR && ls -t rtl_UT=*.raw tx_frame_UT=*.raw 2>/dev/null | awk '/^rtl_/{rtl=\$0} /^tx_frame_/{tx=\$0} END{print rtl, tx}'")

if [[ -z "${RTL_FILE:-}" || -z "${TX_FILE:-}" ]]; then
    echo "could not locate RTL and/or tx_frame recordings on remote" >&2
    exit 1
fi

echo "== remote files: $RTL_FILE  $TX_FILE =="

echo "== scp back =="
scp "$REMOTE:$REMOTE_DIR/$RTL_FILE" ./
scp "$REMOTE:$REMOTE_DIR/$TX_FILE" ./

process_one() {
    local raw="$1" ; local channels="$2"
    local base="${raw%.raw}"
    echo "-- $raw --"
    ffmpeg -y -loglevel error -f s16le -ar 48000 -ac "$channels" -i "$raw" "${base}.wav"
    ffmpeg -y -loglevel error -f s16le -ar 48000 -ac "$channels" -i "$raw" \
        -lavfi "showspectrumpic=s=1600x300:scale=log" "${base}_spec.png"
    echo ">> rx_decode ${raw} (bit-rate=$BIT_RATE)"
    if ssh "$REMOTE" "export PATH=\$HOME/bin:\$PATH && cd $REMOTE_DIR && rx_decode --raw --bit-rate=$BIT_RATE $RS_FLAG -v '$raw'" \
        > "${base}_decode.txt" 2>&1
    then
        echo "   decode OK (see ${base}_decode.txt)"
    else
        echo "   decode FAILED (diag in ${base}_decode.txt)"
    fi
    head -12 "${base}_decode.txt" | sed 's/^/     /'
}

# MONI files are mono; tx_frame's --record pipeline uses -c 1 (see
# utils/tx_frame.c start_recorder). rtl_fm -M fm is also mono.
process_one "$RTL_FILE" 1
process_one "$TX_FILE"  1

if [[ "$PLAY" -eq 1 ]]; then
    echo "== playback (Ctrl-C to skip to next) =="
    ffplay -autoexit "${RTL_FILE%.raw}.wav" || true
    ffplay -autoexit "${TX_FILE%.raw}.wav"  || true
fi

echo
echo "done. Artifacts in $(pwd):"
ls -lh "${RTL_FILE%.raw}".{raw,wav,_spec.png,_decode.txt} 2>/dev/null || true
ls -lh "${TX_FILE%.raw}".{raw,wav,_spec.png,_decode.txt} 2>/dev/null || true
