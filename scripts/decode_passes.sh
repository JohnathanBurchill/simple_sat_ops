#!/usr/bin/env bash
# decode_passes.sh
#
# Walk a directory tree, find every .wav, run rx_replay on the 48 kHz mono
# captures, and summarise what was decoded. Beacons are printed as readable
# telemetry. Anything that isn't a beacon (TCMD responses, or CSP frames
# that don't match either format) is called out so post-uplink replies and
# oddities don't hide in a long batch.
#
# Run from the FrontierSat folder on the server, or pass --root.
#
# Usage:
#   decode_passes.sh [--root <dir>] [--sync-threshold N] [--rx-replay <path>]
# Defaults: root=., sync-threshold=4.

set -uo pipefail

# Decoded payloads can contain arbitrary bytes (corrupted beacon friendly_message
# fields land here as non-UTF-8). awk's default locale chokes on those with a
# "multibyte conversion failure" and silently drops the rest of the input,
# which made per-file frame counts come back as 0 even when beacons decoded.
export LC_ALL=C

ROOT="."
SYNC_THR=4
RX_REPLAY=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --root)             ROOT="$2"; shift 2 ;;
        --sync-threshold)   SYNC_THR="$2"; shift 2 ;;
        --rx-replay)        RX_REPLAY="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,/^# Defaults/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "unknown arg: $1" >&2
            exit 2 ;;
    esac
done

# Locate rx_replay: prefer ~/bin (the install location per CLAUDE.md), fall
# back to PATH, then the in-tree build directory.
if [[ -z "$RX_REPLAY" ]]; then
    for cand in \
        "$HOME/bin/rx_replay" \
        "$(command -v rx_replay 2>/dev/null || true)" \
        "$HOME/src/simple_sat_ops/build/rx_replay"
    do
        if [[ -n "$cand" && -x "$cand" ]]; then
            RX_REPLAY="$cand"
            break
        fi
    done
fi
if [[ -z "$RX_REPLAY" || ! -x "$RX_REPLAY" ]]; then
    echo "error: rx_replay not found. Install it or pass --rx-replay <path>." >&2
    exit 2
fi

if [[ ! -d "$ROOT" ]]; then
    echo "error: root directory not found: $ROOT" >&2
    exit 2
fi

# Validate WAV format. rx_replay expects 48 kHz mono s16, which is what
# b210_rx_live writes by default. Returns "OK <rate>" if usable, or
# "SKIP <reason>" otherwise.
wav_check() {
    python3 - "$1" <<'PY' 2>/dev/null || echo "SKIP not_wav"
import sys, wave
try:
    w = wave.open(sys.argv[1])
    fr, nc, sw = w.getframerate(), w.getnchannels(), w.getsampwidth()
    if fr == 48000 and nc == 1 and sw == 2:
        print(f"OK {fr}")
    else:
        print(f"SKIP rate={fr}Hz channels={nc} sample_bytes={sw}")
except Exception:
    print("SKIP not_wav")
PY
}

t_files=0
t_skip_open=0
t_skip_fmt=0
t_decoded=0
t_frames=0
t_beacons=0
t_tcmd=0
t_other=0

while IFS= read -r -d '' wav; do
    t_files=$((t_files + 1))
    chk="$(wav_check "$wav")"
    case "$chk" in
        "OK "*)
            ;;
        "SKIP not_wav")
            echo "[skip] $wav  (not a readable WAV)"
            t_skip_open=$((t_skip_open + 1))
            continue
            ;;
        SKIP*)
            echo "[skip] $wav  (${chk#SKIP })"
            t_skip_fmt=$((t_skip_fmt + 1))
            continue
            ;;
    esac

    out="$("$RX_REPLAY" "$wav" --sync-threshold="$SYNC_THR" 2>&1 || true)"

    frames=$(printf '%s\n' "$out" | awk '/^rx_replay: [0-9]+ frame\(s\) emitted/{print $2; exit}')
    [[ -z "$frames" ]] && frames=0

    beacon_count=$(printf '%s\n' "$out" | grep -c '^\[t=[^]]*\] beacon: name=' || true)
    tcmd_count=$(printf '%s\n' "$out"   | grep -c '^\[t=[^]]*\] tcmd_response: code=' || true)
    other_count=$((frames - beacon_count - tcmd_count))
    [[ "$other_count" -lt 0 ]] && other_count=0

    t_decoded=$((t_decoded + 1))
    t_frames=$((t_frames + frames))
    t_beacons=$((t_beacons + beacon_count))
    t_tcmd=$((t_tcmd + tcmd_count))
    t_other=$((t_other + other_count))

    echo
    echo "=== $wav"
    echo "    frames=${frames}  beacons=${beacon_count}  tcmd_responses=${tcmd_count}  other=${other_count}"
    if [[ "$frames" -gt 0 ]]; then
        # Decoded telemetry, verbatim from rx_replay.
        printf '%s\n' "$out" | grep -E '^\[t=[^]]*\] (beacon|tcmd_response):'
    fi
    if [[ "$tcmd_count" -gt 0 ]]; then
        echo "    !! $tcmd_count TCMD response packet(s) — post-uplink reply"
    fi
    if [[ "$other_count" -gt 0 ]]; then
        echo "    !! $other_count CSP-OK frame(s) that aren't beacons or TCMD responses"
        # Surface the AX100/CSP/hex/ascii lines for those so the operator
        # can spot what actually landed.
        printf '%s\n' "$out" | grep -E '^\[t=[^]]*\] (AX100|CSP v1|hex|ascii):'
    fi
done < <(find "$ROOT" -type f -iname '*.wav' -print0 | sort -z)

echo
echo "=== summary"
printf "    %-26s %d\n" "files seen:"             "$t_files"
printf "    %-26s %d\n" "skipped (not WAV):"      "$t_skip_open"
printf "    %-26s %d\n" "skipped (wrong format):" "$t_skip_fmt"
printf "    %-26s %d\n" "decoded:"                "$t_decoded"
printf "    %-26s %d\n" "frames total:"           "$t_frames"
printf "    %-26s %d\n" "  beacons:"              "$t_beacons"
printf "    %-26s %d\n" "  TCMD responses:"       "$t_tcmd"
printf "    %-26s %d\n" "  other (flagged):"      "$t_other"
