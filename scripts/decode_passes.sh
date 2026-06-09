#!/usr/bin/env bash
# decode_passes.sh
#
# Walk a directory tree, find every .wav and .ogg, run rx_replay on the
# 48 kHz mono captures (resampling .ogg via ffmpeg first), and summarise
# what was decoded. Beacons are printed as readable telemetry. Anything
# that isn't a beacon (TCMD responses, or CSP frames that don't match
# either format) is called out so post-uplink replies and oddities
# don't hide in a long batch.
#
# Origin tagging — each DB row carries a capture_origin column so
# cross-site decodes (our B210 vs SatNOGS) stay visible side-by-side:
#   - filenames matching satnogs_*.ogg get --capture-origin=satnogs
#   - everything else gets --capture-origin=cts_ground
#
# Run from the FrontierSat folder on the server, or pass --root.
#
# Incremental by default: after rx_replay returns, a sibling
# `<audio>.decoded` marker is touched. Subsequent runs skip files whose
# marker is at least as new as the audio, so a tree of N files stays an
# O(new-arrivals) operation instead of O(N). Pass --force-redecode to
# ignore markers (e.g., after changing rx_replay flags). Deleting a
# marker forces just that one file to be re-decoded.
#
# Usage:
#   decode_passes.sh [--root <dir>] [--db <path>] [--sync-threshold N]
#                    [--rx-replay <path>] [--force-redecode]
#
# --db <path> sends every decoded packet to that SQLite file (it exports
# SSO_PACKET_DB, which rx_replay honours first) instead of the default
# <root>/packet_db.sqlite. That's how you build a fresh DB off to the
# side and swap it in only after checking it looks right.
#
# Defaults: root=., sync-threshold=4, skip already-decoded files.

set -uo pipefail

# Decoded payloads can contain arbitrary bytes (corrupted beacon friendly_message
# fields land here as non-UTF-8). awk's default locale chokes on those with a
# "multibyte conversion failure" and silently drops the rest of the input,
# which made per-file frame counts come back as 0 even when beacons decoded.
export LC_ALL=C

# FrontierSat shared data root: /FrontierSat on the ground machine,
# $HOME/FrontierSat on dev hosts. Defaults --root to it so cron-driven
# decodes don't need an absolute path. Override with --root or the
# FRONTIERSAT_ROOT env var.
: "${FRONTIERSAT_ROOT:=$([[ -d /FrontierSat ]] && echo /FrontierSat || echo "$HOME/FrontierSat")}"
ROOT="$FRONTIERSAT_ROOT"
SYNC_THR=4
RX_REPLAY=""
SKIP_DECODED=1
DB_PATH=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --root)             ROOT="$2"; shift 2 ;;
        --db)               DB_PATH="$2"; shift 2 ;;
        --db=*)             DB_PATH="${1#*=}"; shift ;;
        --sync-threshold)   SYNC_THR="$2"; shift 2 ;;
        --rx-replay)        RX_REPLAY="$2"; shift 2 ;;
        --force-redecode)   SKIP_DECODED=0; shift ;;
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

# --db routes every rx_replay to one packet DB by exporting SSO_PACKET_DB,
# which rx_replay's default-path logic checks first. Exporting (not just
# setting) is what makes the child rx_replay processes see it. Without --db
# this is left alone: an SSO_PACKET_DB already in the environment still wins,
# otherwise rx_replay falls back to $FRONTIERSAT_ROOT/packet_db.sqlite. That
# fallback keys off the shared tree ROOT, NOT --root -- so a bare run that
# scans, say, satnogs_archive still writes to the one main packet DB, never a
# DB under the scanned directory.
if [[ -n "$DB_PATH" ]]; then
    export SSO_PACKET_DB="$DB_PATH"
fi
echo "rx_replay:  $RX_REPLAY"
echo "root:       $ROOT"
echo "packet DB:  ${SSO_PACKET_DB:-$FRONTIERSAT_ROOT/packet_db.sqlite (rx_replay default)}"

# Pre-flight: refuse to run if that packet DB can't be written. Otherwise a
# permissions problem is SILENT -- rx_replay drops the inserts but still exits
# 0, decode_passes touches each <audio>.decoded marker below, and the passes
# are then skipped on every future run even after the DB is fixed, quietly
# losing them. Fail loud and early instead. The writer needs write access to
# the .sqlite file, its -wal/-shm sidecars, AND the containing directory (WAL
# creates the sidecars there), so we check the directory, the file, and any
# existing WAL sidecars are writable.
db_target="${SSO_PACKET_DB:-$FRONTIERSAT_ROOT/packet_db.sqlite}"
db_dir="$(dirname "$db_target")"
preflight_fail() {
    echo "error: packet DB is not writable: $db_target" >&2
    echo "       ($1)" >&2
    echo "       Refusing to decode -- a silent write failure would mark every" >&2
    echo "       pass '.decoded' without storing it, and they would be skipped" >&2
    echo "       even after the DB is fixed. Give the writer (e.g. the cron" >&2
    echo "       user) write access to the .sqlite file, its -wal/-shm" >&2
    echo "       sidecars, and the directory $db_dir, then re-run." >&2
    exit 3
}
[[ -d "$db_dir" && -w "$db_dir" && -x "$db_dir" ]] || preflight_fail "directory not writable"
if [[ -e "$db_target" ]]; then
    [[ -w "$db_target" ]] || preflight_fail "file not writable"
    # In WAL mode the writer also updates these sidecars; a stale root- or
    # other-user-owned -shm/-wal blocks every other writer, so if they exist
    # they must be writable too. (-w uses access(2): it honours ACLs and is
    # evaluated as the user actually running this, e.g. the cron user.)
    for _sx in "$db_target-wal" "$db_target-shm"; do
        [[ ! -e "$_sx" || -w "$_sx" ]] || preflight_fail "WAL sidecar not writable: $_sx"
    done
fi

# ffmpeg is only required when an .ogg shows up in the tree; check
# lazily so a pure-WAV run on a host without ffmpeg still works.
have_ffmpeg=""
if command -v ffmpeg >/dev/null 2>&1; then
    have_ffmpeg=1
fi

# Derive capture_origin from filename. SatNOGS archive downloads
# follow `satnogs_<obs-id>_*` (case-insensitive); anything else is
# treated as our local B210 capture. Returns the value to pass to
# rx_replay --capture-origin=.
origin_for_filename() {
    local base
    base="$(basename "$1")"
    shopt -s nocasematch
    if [[ "$base" == satnogs_* ]]; then
        shopt -u nocasematch
        echo "satnogs"
    else
        shopt -u nocasematch
        echo "cts_ground"
    fi
}

# Convert an .ogg to a temp 48 kHz mono S16LE WAV that rx_replay can
# eat. Echoes the temp path to stdout; caller is responsible for rm.
# Returns non-zero on ffmpeg failure.
ogg_to_wav() {
    local in="$1"
    local out
    out="$(mktemp -t decode_passes_XXXXXX.wav)" || return 1
    if ! ffmpeg -loglevel error -y -i "$in" \
                -ar 48000 -ac 1 -f wav -acodec pcm_s16le \
                "$out" </dev/null; then
        rm -f "$out"
        return 1
    fi
    printf '%s' "$out"
}

# Pull a UTC ISO-8601 timestamp out of a SatNOGS-style filename, e.g.
# satnogs_<obs-id>_..._YYYY-MM-DDTHH-MM-SS.ogg → YYYY-MM-DDTHH:MM:SSZ.
# (Hyphens in the time portion are SatNOGS's filesystem-safe choice.)
# Echoes the ISO form on hit; empty on miss.
satnogs_start_utc() {
    local base
    base="$(basename "$1")"
    if [[ "$base" =~ ([0-9]{4}-[0-9]{2}-[0-9]{2})T([0-9]{2})-([0-9]{2})-([0-9]{2}) ]]; then
        printf '%sT%s:%s:%sZ' \
            "${BASH_REMATCH[1]}" \
            "${BASH_REMATCH[2]}" \
            "${BASH_REMATCH[3]}" \
            "${BASH_REMATCH[4]}"
    fi
}

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
t_skip_done=0
t_decoded=0
t_frames=0
t_beacons=0
t_tcmd=0
t_other=0

while IFS= read -r -d '' src; do
    t_files=$((t_files + 1))
    # Compute the .iq sidecar path up front. We prefer it for decode
    # when present, and we also factor it into the skip-marker check so
    # an existing .decoded marker doesn't shadow a fresher .iq drop.
    iq_candidate=""
    if [[ "${src,,}" == *.wav ]]; then
        iq_candidate="${src%.wav}.iq"
        [[ -r "$iq_candidate" ]] || iq_candidate=""
    fi
    if [[ "$SKIP_DECODED" -eq 1 ]]; then
        marker="${src}.decoded"
        if [[ -f "$marker" && ! "$src" -nt "$marker" ]]; then
            # Marker is at least as new as the audio — usually skip.
            # Exception: a sibling .iq newer than the marker indicates
            # the IQ path now has data the marker run didn't cover, so
            # force a re-decode.
            if [[ -n "$iq_candidate" && "$iq_candidate" -nt "$marker" ]]; then
                :  # fall through to re-decode
            else
                t_skip_done=$((t_skip_done + 1))
                continue
            fi
        fi
    fi
    origin="$(origin_for_filename "$src")"
    decode_path="$src"
    cleanup_path=""
    iq_mode=0

    case "${src,,}" in
        *.ogg)
            if [[ -z "$have_ffmpeg" ]]; then
                echo "[skip] $src  (ffmpeg not on PATH; install to decode .ogg)"
                t_skip_open=$((t_skip_open + 1))
                continue
            fi
            tmp_wav="$(ogg_to_wav "$src")"
            if [[ -z "$tmp_wav" ]]; then
                echo "[skip] $src  (ffmpeg conversion failed)"
                t_skip_open=$((t_skip_open + 1))
                continue
            fi
            decode_path="$tmp_wav"
            cleanup_path="$tmp_wav"
            ;;
        *.wav)
            # Prefer the .iq sidecar when simple_sat_ops dropped one
            # next to the WAV — the IQ-domain Viterbi decoder pulls
            # noticeably more frames out of low-SNR passes. .iq files
            # don't exist for SatNOGS .ogg or rtl_fm WAV captures, so
            # this falls back to the WAV when no .iq is present.
            if [[ -n "$iq_candidate" ]]; then
                decode_path="$iq_candidate"
                iq_mode=1
            else
                chk="$(wav_check "$src")"
                case "$chk" in
                    "OK "*)
                        ;;
                    "SKIP not_wav")
                        echo "[skip] $src  (not a readable WAV)"
                        t_skip_open=$((t_skip_open + 1))
                        continue
                        ;;
                    SKIP*)
                        echo "[skip] $src  (${chk#SKIP })"
                        t_skip_fmt=$((t_skip_fmt + 1))
                        continue
                        ;;
                esac
            fi
            ;;
        *)
            chk="$(wav_check "$src")"
            case "$chk" in
                "OK "*)
                    ;;
                "SKIP not_wav")
                    echo "[skip] $src  (not a readable WAV)"
                    t_skip_open=$((t_skip_open + 1))
                    continue
                    ;;
                SKIP*)
                    echo "[skip] $src  (${chk#SKIP })"
                    t_skip_fmt=$((t_skip_fmt + 1))
                    continue
                    ;;
            esac
            ;;
    esac

    # Build the rx_replay command. The temp WAV from an .ogg conversion
    # loses whatever timestamp lived in the source filename, so for the
    # SatNOGS path we extract it here and feed --start-utc; otherwise
    # rx_replay's UT=...filename / mtime fallbacks already cover the
    # cts_ground case.
    rx_args=( --sync-threshold="$SYNC_THR"
              --capture-origin="$origin"
              --session-dir="$(dirname "$src")" )
    if [[ "$origin" == "satnogs" ]]; then
        sat_utc="$(satnogs_start_utc "$src")"
        if [[ -n "$sat_utc" ]]; then
            rx_args+=( --start-utc="$sat_utc" )
        fi
        # Geometry should be relative to the recording SatNOGS station,
        # not RAO. satnogs_pull writes the obs detail JSON next to the
        # audio, which carries station_lat/lng/alt from the SatNOGS
        # API. If those fields aren't present we use --no-observer
        # rather than silently letting rx_replay fall back to RAO.
        obs_dir="$(dirname "$src")"
        obs_id="$(basename "$obs_dir")"
        meta="$obs_dir/satnogs_$obs_id.meta.json"
        sat_lat=""; sat_lng=""; sat_alt=""
        if [[ -r "$meta" ]]; then
            sat_lat="$(jq -r '.station_lat // empty' "$meta" 2>/dev/null)"
            sat_lng="$(jq -r '.station_lng // empty' "$meta" 2>/dev/null)"
            sat_alt="$(jq -r '.station_alt // empty' "$meta" 2>/dev/null)"
        fi
        if [[ -n "$sat_lat" && -n "$sat_lng" && -n "$sat_alt" ]]; then
            rx_args+=( --lat="$sat_lat" --lon="$sat_lng" --alt="$sat_alt" )
        else
            rx_args+=( --no-observer )
        fi
    fi
    out="$("$RX_REPLAY" "$decode_path" "${rx_args[@]}" 2>&1 || true)"
    [[ -n "$cleanup_path" ]] && rm -f "$cleanup_path"

    # Rename `src` back to the user-facing path the rest of this loop
    # already prints under, so the WAV-skip and decoded headers line
    # up regardless of the source format.
    wav="$src"

    frames=$(printf '%s\n' "$out" | awk -F: '/detected \(after position dedup\)/{gsub(/[^0-9]/,"",$2); print $2; exit}')
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

    chain_tag=""
    [[ "$iq_mode" -eq 1 ]] && chain_tag="  [iq+viterbi]"
    echo
    echo "=== $wav${chain_tag}"
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

    if [[ "$SKIP_DECODED" -eq 1 ]]; then
        # The marker's mtime is the skip-test key — its presence alone
        # doesn't guarantee freshness if the audio is later overwritten.
        touch "${src}.decoded"
    fi
done < <(find "$ROOT" -type f \( -iname '*.wav' -o -iname '*.ogg' \) -print0 | sort -z)

echo
echo "=== summary"
printf "    %-26s %d\n" "files seen:"             "$t_files"
printf "    %-26s %d\n" "skipped (already done):" "$t_skip_done"
printf "    %-26s %d\n" "skipped (not WAV):"      "$t_skip_open"
printf "    %-26s %d\n" "skipped (wrong format):" "$t_skip_fmt"
printf "    %-26s %d\n" "decoded:"                "$t_decoded"
printf "    %-26s %d\n" "frames total:"           "$t_frames"
printf "    %-26s %d\n" "  beacons:"              "$t_beacons"
printf "    %-26s %d\n" "  TCMD responses:"       "$t_tcmd"
printf "    %-26s %d\n" "  other (flagged):"      "$t_other"
