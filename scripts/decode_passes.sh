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
# Parallel by default — the per-file rx_replay decodes run across a pool
# of --jobs worker processes (default: one per CPU). This is the win for
# rebuilding the DB from a large tree of captures, and it speeds up the
# routine satnogs_pull --decode pass too. Each rx_replay is its own
# process writing the shared packet DB; that's safe because the DB is
# WAL-mode with a busy timeout and dedups inserts. Use --jobs 1 for the
# old strictly-serial behaviour. (Only the decode is parallelised; the
# satnogs *download* in satnogs_pull.sh stays serial and rate-limited.)
#
# Newest first — files are always handed to the workers in newest-mtime-
# first order, so a fresh capture is decoded before an old one. With
# parallel workers the per-file report blocks are collated back into that
# same newest-first order at the end (live one-line progress goes to
# stderr as each file finishes).
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
#                    [--jobs N] [--rx-replay <path>] [--force-redecode]
#                    [--forensics-report]
#
# --db <path> sends every decoded packet to that SQLite file (it exports
# SSO_PACKET_DB, which rx_replay honours first) instead of the default
# <root>/packet_db.sqlite. That's how you build a fresh DB off to the
# side and swap it in only after checking it looks right.
#
# --jobs N runs N rx_replay decodes at once (default: CPU count). During
# a live pass you may want a smaller number so the receiver isn't starved.
#
# --forensics-report runs every file through rx_replay --forensics-report
# (for research / decoder scoring across a corpus). The packet DB is never
# touched and no `.decoded` markers are read or written, so every file is
# re-processed each run. Output is grouped per file under a `=== <path>`
# header, each followed by that file's JSON frame lines on stdout. For a
# single flat JSON stream of one file, call rx_replay --forensics-report
# directly.
#
# Defaults: root=., sync-threshold=4, jobs=CPU count, skip already-decoded.

set -uo pipefail

# Decoded payloads can contain arbitrary bytes (corrupted beacon friendly_message
# fields land here as non-UTF-8). awk's default locale chokes on those with a
# "multibyte conversion failure" and silently drops the rest of the input,
# which made per-file frame counts come back as 0 even when beacons decoded.
export LC_ALL=C

# A literal tab, used as the field separator when sorting and when packing
# the "<index>\t<path>" work records handed to the parallel workers.
TAB="$(printf '\t')"

# --------------------------------------------------------------------------
# Per-file helpers. Defined up front so the internal --process-one worker
# re-entry (below) can call them without running any of the parent setup.
# --------------------------------------------------------------------------

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
# simple_sat_ops writes by default. Returns "OK <rate>" if usable, or
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

# Decide whether `src` should be skipped because it's already decoded.
# Mirrors the original incremental logic exactly: skip when SKIP_DECODED
# is on, a sibling `.decoded` marker exists and is at least as new as the
# audio, UNLESS a sibling .iq is newer than the marker (a fresh IQ drop
# the marker run didn't cover). Returns 0 to SKIP, 1 to process.
skip_marker_test() {
    local src="$1"
    [[ "$SKIP_DECODED" -eq 1 ]] || return 1
    local marker="${src}.decoded"
    [[ -f "$marker" && ! "$src" -nt "$marker" ]] || return 1
    local iqc=""
    if [[ "${src,,}" == *.wav ]]; then
        iqc="${src%.wav}.iq"
        [[ -r "$iqc" ]] || iqc=""
    fi
    if [[ -n "$iqc" && "$iqc" -nt "$marker" ]]; then
        return 1   # fresher .iq -> re-decode
    fi
    return 0       # skip
}

# List matching files under $ROOT, NUL-terminated, newest mtime first.
# GNU find has -printf (NUL-safe and fast); BSD/macOS find doesn't, so we
# fall back to `stat` over a -print0 list there (paths with embedded
# newlines aren't supported on that branch, but our capture files never
# contain them). Ties broken by path for a deterministic order.
list_files_newest_first() {
    if find "$ROOT" -maxdepth 0 -printf '' >/dev/null 2>&1; then
        find "$ROOT" -type f \( -iname '*.wav' -o -iname '*.ogg' \) \
                -printf '%T@\t%p\0' \
            | sort -z -t "$TAB" -k1,1rn -k2,2 \
            | cut -z -f2-
    else
        find "$ROOT" -type f \( -iname '*.wav' -o -iname '*.ogg' \) -print0 \
            | xargs -0 stat -f "%m${TAB}%N" 2>/dev/null \
            | sort -t "$TAB" -k1,1rn -k2,2 \
            | cut -f2- \
            | tr '\n' '\0'
    fi
}

# Default worker count: one per CPU, falling back to 4 if neither nproc
# (Linux) nor sysctl (macOS) is available.
default_jobs() {
    local n
    n="$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"
    [[ "$n" =~ ^[0-9]+$ && "$n" -ge 1 ]] || n=4
    echo "$n"
}

# Decode one file. Writes its human-readable report block to
# $SCRATCH/<idx>.out and a tab-separated stat line to $SCRATCH/<idx>.stat
# (status<TAB>frames<TAB>beacons<TAB>tcmd<TAB>other, status one of
# decoded|skip_open|skip_fmt), and emits a terse one-line progress note to
# stderr. The parent collates the .out blocks in index order and sums the
# .stat lines. Reads its configuration (RX_REPLAY, SYNC_THR, SKIP_DECODED,
# have_ffmpeg, SCRATCH, TOTAL) from the exported environment.
process_one_file() {
    local idx="$1" src="$2"
    local out="$SCRATCH/$idx.out"
    local stat="$SCRATCH/$idx.stat"
    local pos=$((10#$idx))
    : > "$out"

    # Compute the .iq sidecar path up front. We prefer it for decode when
    # present (the IQ-domain Viterbi decoder pulls more frames out of
    # low-SNR passes). It doesn't exist for SatNOGS .ogg or rtl_fm WAV.
    local iq_candidate=""
    if [[ "${src,,}" == *.wav ]]; then
        iq_candidate="${src%.wav}.iq"
        [[ -r "$iq_candidate" ]] || iq_candidate=""
    fi

    local origin decode_path cleanup_path iq_mode tmp_wav chk
    origin="$(origin_for_filename "$src")"
    decode_path="$src"
    cleanup_path=""
    iq_mode=0

    case "${src,,}" in
        *.ogg)
            if [[ -z "$have_ffmpeg" ]]; then
                printf '[skip] %s  (ffmpeg not on PATH; install to decode .ogg)\n' "$src" >> "$out"
                printf 'skip_open\t0\t0\t0\t0\n' > "$stat"
                printf '[%d/%d skip] %s  (no ffmpeg)\n' "$pos" "$TOTAL" "$src" >&2
                return 0
            fi
            tmp_wav="$(ogg_to_wav "$src")"
            if [[ -z "$tmp_wav" ]]; then
                printf '[skip] %s  (ffmpeg conversion failed)\n' "$src" >> "$out"
                printf 'skip_open\t0\t0\t0\t0\n' > "$stat"
                printf '[%d/%d skip] %s  (ffmpeg failed)\n' "$pos" "$TOTAL" "$src" >&2
                return 0
            fi
            decode_path="$tmp_wav"
            cleanup_path="$tmp_wav"
            ;;
        *.wav)
            # Prefer the .iq sidecar when simple_sat_ops dropped one next
            # to the WAV; fall back to the WAV when no .iq is present.
            if [[ -n "$iq_candidate" ]]; then
                decode_path="$iq_candidate"
                iq_mode=1
            else
                chk="$(wav_check "$src")"
                case "$chk" in
                    "OK "*)
                        ;;
                    "SKIP not_wav")
                        printf '[skip] %s  (not a readable WAV)\n' "$src" >> "$out"
                        printf 'skip_open\t0\t0\t0\t0\n' > "$stat"
                        printf '[%d/%d skip] %s  (not WAV)\n' "$pos" "$TOTAL" "$src" >&2
                        return 0
                        ;;
                    SKIP*)
                        printf '[skip] %s  (%s)\n' "$src" "${chk#SKIP }" >> "$out"
                        printf 'skip_fmt\t0\t0\t0\t0\n' > "$stat"
                        printf '[%d/%d skip] %s  (%s)\n' "$pos" "$TOTAL" "$src" "${chk#SKIP }" >&2
                        return 0
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
                    printf '[skip] %s  (not a readable WAV)\n' "$src" >> "$out"
                    printf 'skip_open\t0\t0\t0\t0\n' > "$stat"
                    printf '[%d/%d skip] %s  (not WAV)\n' "$pos" "$TOTAL" "$src" >&2
                    return 0
                    ;;
                SKIP*)
                    printf '[skip] %s  (%s)\n' "$src" "${chk#SKIP }" >> "$out"
                    printf 'skip_fmt\t0\t0\t0\t0\n' > "$stat"
                    printf '[%d/%d skip] %s  (%s)\n' "$pos" "$TOTAL" "$src" "${chk#SKIP }" >&2
                    return 0
                    ;;
            esac
            ;;
    esac

    # Build the rx_replay command. The temp WAV from an .ogg conversion
    # loses whatever timestamp lived in the source filename, so for the
    # SatNOGS path we extract it here and feed --start-utc; otherwise
    # rx_replay's UT=...filename / mtime fallbacks already cover the
    # cts_ground case.
    local rx_args
    rx_args=( --sync-threshold="$SYNC_THR"
              --capture-origin="$origin"
              --session-dir="$(dirname "$src")" )
    if [[ "$origin" == "satnogs" ]]; then
        local sat_utc obs_dir obs_id meta sat_lat sat_lng sat_alt
        sat_utc="$(satnogs_start_utc "$src")"
        if [[ -n "$sat_utc" ]]; then
            rx_args+=( --start-utc="$sat_utc" )
        fi
        # Geometry should be relative to the recording SatNOGS station,
        # not RAO. satnogs_pull writes the obs detail JSON next to the
        # audio, which carries station_lat/lng/alt from the SatNOGS API.
        # If those fields aren't present we use --no-observer rather than
        # silently letting rx_replay fall back to RAO.
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

    # Forensics report: hand the file to rx_replay --forensics-report and
    # stream its JSON frame lines. Capture stdout only — stderr carries the
    # decode summary and progress notes that would corrupt the JSON. Group
    # per file under a `=== <path>` header the parent collates newest-first,
    # exactly like the normal report blocks. No DB, no `.decoded` marker.
    if [[ "$FORENSICS_REPORT" -eq 1 ]]; then
        local fr_json fr_count
        fr_json="$("$RX_REPLAY" "$decode_path" "${rx_args[@]}" --forensics-report 2>/dev/null || true)"
        [[ -n "$cleanup_path" ]] && rm -f "$cleanup_path"
        fr_count=$(printf '%s\n' "$fr_json" | grep -c '^{' || true)
        {
            echo
            echo "=== $src"
            [[ "$fr_count" -gt 0 ]] && printf '%s\n' "$fr_json" | grep '^{'
        } >> "$out"
        printf 'decoded\t%d\t0\t0\t0\n' "$fr_count" > "$stat"
        printf '[%d/%d] %s  messages=%d\n' "$pos" "$TOTAL" "$src" "$fr_count" >&2
        return 0
    fi

    local out_text frames beacon_count tcmd_count other_count chain_tag wav
    out_text="$("$RX_REPLAY" "$decode_path" "${rx_args[@]}" 2>&1 || true)"
    [[ -n "$cleanup_path" ]] && rm -f "$cleanup_path"

    # The user-facing path is always the source file, regardless of which
    # decode target (temp WAV / .iq sidecar) we actually fed rx_replay.
    wav="$src"

    frames=$(printf '%s\n' "$out_text" | awk -F: '/detected \(after position dedup\)/{gsub(/[^0-9]/,"",$2); print $2; exit}')
    [[ -z "$frames" ]] && frames=0

    beacon_count=$(printf '%s\n' "$out_text" | grep -c '^\[t=[^]]*\] beacon: name=' || true)
    tcmd_count=$(printf '%s\n' "$out_text"   | grep -c '^\[t=[^]]*\] tcmd_response: code=' || true)
    other_count=$((frames - beacon_count - tcmd_count))
    [[ "$other_count" -lt 0 ]] && other_count=0

    chain_tag=""
    [[ "$iq_mode" -eq 1 ]] && chain_tag="  [iq+viterbi]"
    {
        echo
        echo "=== $wav${chain_tag}"
        echo "    frames=${frames}  beacons=${beacon_count}  tcmd_responses=${tcmd_count}  other=${other_count}"
        if [[ "$frames" -gt 0 ]]; then
            # Decoded telemetry, verbatim from rx_replay.
            printf '%s\n' "$out_text" | grep -E '^\[t=[^]]*\] (beacon|tcmd_response):'
        fi
        if [[ "$tcmd_count" -gt 0 ]]; then
            echo "    !! $tcmd_count TCMD response packet(s) — post-uplink reply"
        fi
        if [[ "$other_count" -gt 0 ]]; then
            echo "    !! $other_count CSP-OK frame(s) that aren't beacons or TCMD responses"
            # Surface the AX100/CSP/hex/ascii lines for those so the operator
            # can spot what actually landed.
            printf '%s\n' "$out_text" | grep -E '^\[t=[^]]*\] (AX100|CSP v1|hex|ascii):'
        fi
    } >> "$out"

    printf 'decoded\t%d\t%d\t%d\t%d\n' "$frames" "$beacon_count" "$tcmd_count" "$other_count" > "$stat"
    printf '[%d/%d] %s  frames=%d beacons=%d tcmd=%d other=%d\n' \
        "$pos" "$TOTAL" "$src" "$frames" "$beacon_count" "$tcmd_count" "$other_count" >&2

    if [[ "$SKIP_DECODED" -eq 1 ]]; then
        # The marker's mtime is the skip-test key — its presence alone
        # doesn't guarantee freshness if the audio is later overwritten.
        touch "${src}.decoded"
    fi
}

# --------------------------------------------------------------------------
# Internal worker re-entry. `decode_passes.sh --process-one "<idx>\t<path>"`
# decodes a single file; this is what the xargs pool below invokes per file.
# Short-circuits before any parent setup so the DB preflight, arg parse,
# etc. run exactly once (in the parent), not per file.
# --------------------------------------------------------------------------
if [[ "${1:-}" == "--process-one" ]]; then
    : "${SCRATCH:?--process-one is an internal worker mode invoked by decode_passes.sh itself}"
    record="${2:-}"
    idx="${record%%"$TAB"*}"
    src="${record#*"$TAB"}"
    process_one_file "$idx" "$src"
    exit 0
fi

# --------------------------------------------------------------------------
# Parent: parse args, validate, build the newest-first worklist, fan the
# decodes out across the worker pool, then collate the reports.
# --------------------------------------------------------------------------

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
JOBS=""
FORENSICS_REPORT=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --root)             ROOT="$2"; shift 2 ;;
        --db)               DB_PATH="$2"; shift 2 ;;
        --db=*)             DB_PATH="${1#*=}"; shift ;;
        --sync-threshold)   SYNC_THR="$2"; shift 2 ;;
        --jobs)             JOBS="$2"; shift 2 ;;
        --jobs=*)           JOBS="${1#*=}"; shift ;;
        --rx-replay)        RX_REPLAY="$2"; shift 2 ;;
        --force-redecode)   SKIP_DECODED=0; shift ;;
        --forensics-report) FORENSICS_REPORT=1; shift ;;
        -h|--help)
            sed -n '2,/^# Defaults/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "unknown arg: $1" >&2
            exit 2 ;;
    esac
done

# Resolve --jobs: default to one decode per CPU. Validate so a typo
# surfaces here rather than as a confusing xargs error.
[[ -z "$JOBS" ]] && JOBS="$(default_jobs)"
if ! [[ "$JOBS" =~ ^[0-9]+$ ]] || [[ "$JOBS" -lt 1 ]]; then
    echo "error: --jobs must be a positive integer (got '$JOBS')" >&2
    exit 2
fi

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

# --forensics-report is read-only: it never opens the packet DB and never
# reads or writes the `.decoded` markers. Force a full re-process (markers
# are a DB-decode bookkeeping concept that doesn't apply here) and warn if
# --db was given, since it's ignored.
if [[ "$FORENSICS_REPORT" -eq 1 ]]; then
    SKIP_DECODED=0
    [[ -n "$DB_PATH" ]] && \
        echo "note: --db ignored under --forensics-report (the DB is never touched)" >&2
fi

echo "rx_replay:  $RX_REPLAY"
echo "root:       $ROOT"
echo "jobs:       $JOBS  (newest-first)"

if [[ "$FORENSICS_REPORT" -eq 1 ]]; then
    echo "mode:       forensics report (JSON to stdout; packet DB not touched)"
else
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
fi

# ffmpeg is only required when an .ogg shows up in the tree; check
# lazily so a pure-WAV run on a host without ffmpeg still works.
have_ffmpeg=""
if command -v ffmpeg >/dev/null 2>&1; then
    have_ffmpeg=1
fi

# Absolute path to this script, used to re-invoke ourselves as a worker
# under xargs. Resolve a bare name via PATH and relative paths via cwd so
# the workers (which inherit this cwd) always find the right file.
SELF="$0"
case "$SELF" in
    */*) ;;
    *)   SELF="$(command -v "$SELF" 2>/dev/null || printf '%s' "$SELF")" ;;
esac
case "$SELF" in
    /*) ;;
    *)  SELF="$(cd "$(dirname "$SELF")" 2>/dev/null && pwd)/$(basename "$SELF")" ;;
esac

# Scratch dir for per-file report blocks and stat lines. Workers write into
# it; the parent reads it back after the pool drains and removes it on exit.
SCRATCH="$(mktemp -d -t decode_passes_run_XXXXXX)" \
    || { echo "error: could not create scratch dir" >&2; exit 3; }
trap 'rm -rf "$SCRATCH"' EXIT

# Things the worker re-entry needs from the environment.
export RX_REPLAY SYNC_THR SKIP_DECODED have_ffmpeg SCRATCH TAB FORENSICS_REPORT

# Build the worklist: walk newest-first, drop already-decoded files here in
# the parent (so the workers aren't spawned just to skip them — keeps an
# incremental run O(new arrivals)), and pack the survivors as NUL-terminated
# "<zero-padded-index>\t<path>" records. The zero-padded index preserves the
# newest-first order when the report blocks are collated by sorted filename.
t_files=0
t_skip_done=0
idx=0
worklist="$SCRATCH/worklist"
: > "$worklist"
while IFS= read -r -d '' src; do
    t_files=$((t_files + 1))
    if skip_marker_test "$src"; then
        t_skip_done=$((t_skip_done + 1))
        continue
    fi
    idx=$((idx + 1))
    printf '%08d\t%s\0' "$idx" "$src" >> "$worklist"
done < <(list_files_newest_first)

# TOTAL drives the "[k/N]" progress counter the workers print to stderr.
export TOTAL="$idx"

# Fan out: xargs runs up to $JOBS copies of `decode_passes.sh --process-one`
# at once, one file per copy (-n1 gives the best load balancing). Guarded on
# a non-empty worklist because GNU xargs would otherwise run the command once
# with no argument.
if [[ "$idx" -gt 0 ]]; then
    xargs -0 -P "$JOBS" -n1 "${BASH:-bash}" "$SELF" --process-one < "$worklist"
fi

# Collate the per-file report blocks in newest-first (index) order. Bash
# sorts glob results, and the zero-padded index makes that the right order.
shopt -s nullglob
for f in "$SCRATCH"/*.out; do
    cat "$f"
done

# Sum the per-file stat lines into the run totals.
agg="$(find "$SCRATCH" -name '*.stat' -exec cat {} + 2>/dev/null | awk -F'\t' '
    { fr += $2; be += $3; tc += $4; ot += $5
      if      ($1 == "decoded")   d++
      else if ($1 == "skip_open") so++
      else if ($1 == "skip_fmt")  sf++ }
    END { printf "%d %d %d %d %d %d %d", d+0, so+0, sf+0, fr+0, be+0, tc+0, ot+0 }')"
read -r t_decoded t_skip_open t_skip_fmt t_frames t_beacons t_tcmd t_other <<< "$agg"

echo
echo "=== summary"
printf "    %-26s %d\n" "files seen:"             "$t_files"
printf "    %-26s %d\n" "skipped (already done):" "$t_skip_done"
printf "    %-26s %d\n" "skipped (not WAV):"      "$t_skip_open"
printf "    %-26s %d\n" "skipped (wrong format):" "$t_skip_fmt"
printf "    %-26s %d\n" "decoded:"                "$t_decoded"
if [[ "$FORENSICS_REPORT" -eq 1 ]]; then
    # Forensics mode reports raw frames as JSON; the beacon/TCMD/other
    # split is a DB-recognition concept that isn't computed here.
    printf "    %-26s %d\n" "JSON messages total:"    "$t_frames"
else
    printf "    %-26s %d\n" "frames total:"           "$t_frames"
    printf "    %-26s %d\n" "  beacons:"              "$t_beacons"
    printf "    %-26s %d\n" "  TCMD responses:"       "$t_tcmd"
    printf "    %-26s %d\n" "  other (flagged):"      "$t_other"
fi
