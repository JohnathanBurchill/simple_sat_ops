#!/usr/bin/env bash
# Stream the most-recently-modified .iq from the operations tree on
# the remote ground station into a local file, for use with
# `decode_inspector LOCAL_PATH --live`.
#
# Find runs over SSH (no install needed on the remote). Tail follows
# the file as the live capture grows; the resulting local file stays
# in lock-step with what's on the remote disk.
#
# Usage:
#   scripts/stream_latest_iq.sh [LOCAL_PATH]
#
# Defaults:
#   LOCAL_PATH    ./live.iq
#
# Environment:
#   HOST          remote ssh target           (default: va6raogndstn)
#   REMOTE_ROOT   where to look for .iq files (default: /FrontierSat/Operations)
#
# Example:
#   scripts/stream_latest_iq.sh
#   decode_inspector live.iq --live --live-window-s=60

set -uo pipefail

case "${1:-}" in -h|--help)
    sed -n '2,/^[^#]/p' "$0" | sed -E 's/^# ?//; /^$/d' >&2
    exit 2
    ;;
esac

HOST="${HOST:-rao}"
REMOTE_ROOT="${REMOTE_ROOT:-/FrontierSat/Operations}"
LOCAL_PATH="${1:-live.iq}"

# Find the latest .iq under REMOTE_ROOT (mtime descending). -printf is
# a GNU find feature so this assumes the remote is Linux (RAO is). The
# tab separator keeps paths-with-spaces intact through cut.
REMOTE_PATH=$(ssh "$HOST" \
    "find '$REMOTE_ROOT' -name '*.iq' -type f -printf '%T@\\t%p\\n' 2>/dev/null \
        | sort -nr | head -n1 | cut -f2-")

if [ -z "$REMOTE_PATH" ]; then
    echo "stream_latest_iq: no .iq under $REMOTE_ROOT on $HOST" >&2
    exit 1
fi

# Make sure the local parent exists.
LOCAL_DIR=$(dirname -- "$LOCAL_PATH")
if [ -n "$LOCAL_DIR" ] && [ ! -d "$LOCAL_DIR" ]; then
    mkdir -p -- "$LOCAL_DIR"
fi

echo "stream_latest_iq: $HOST:$REMOTE_PATH" >&2
echo "stream_latest_iq: -> $LOCAL_PATH (Ctrl-C to stop)" >&2

# tail -f -c +0 starts from byte 0 and follows growth. Throughput at
# 96 kSPS int16 IQ pairs is ~384 KB/s — nothing over SSH.
exec ssh "$HOST" "tail -f -c +0 -- '$REMOTE_PATH'" > "$LOCAL_PATH"
