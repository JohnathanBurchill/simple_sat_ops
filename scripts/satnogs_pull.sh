#!/usr/bin/env bash
# satnogs_pull.sh
#
# Pull observation audio for a given satellite from the SatNOGS network
# archive (https://network.satnogs.org/) and stash each pass in its own
# folder so decode_passes.sh can walk the tree.
#
# Designed for unattended cron use:
#   - Idempotent: tracks fetched observation IDs in `.fetched.txt` under
#     --out, so re-runs only download new passes.
#   - Locked: a flock (or mkdir fallback) prevents overlapping invocations.
#   - Atomic: audio writes to <file>.part first, mv'd into place on success.
#   - Filename layout matches decode_passes.sh's SatNOGS detection:
#       <out>/<obs-id>/satnogs_<obs-id>_<YYYY-MM-DDTHH-MM-SS>.ogg
#       <out>/<obs-id>/satnogs_<obs-id>.tle           (TLE used by SatNOGS)
#       <out>/<obs-id>/satnogs_<obs-id>.meta.json     (detail-endpoint JSON)
#
# Cron example (every 5 min for FrontierSat, with auto-decode):
#   */5 * * * * $HOME/bin/satnogs_pull.sh --decode \
#       >> $HOME/var/log/sso/satnogs_pull.log 2>&1
#
# Courtesy & rate limits
#   SatNOGS Network has no published hard limit; the project's standing
#   request is "be sensible — don't hammer." Each incremental run makes
#   ~3-5 API calls (one list page + a detail GET per new observation +
#   audio downloads from the CDN), so 5-minute polling for one satellite
#   stays well inside any reasonable budget. If running for many sats at
#   once, raise --rate-limit-ms or stagger the cron entries.
#
# Usage:
#   satnogs_pull.sh [--norad-id=<n>] [options]
#
# Options:
#   --norad-id=<n>          NORAD catalog ID (default 69015 — FrontierSat)
#   --out=<dir>             Output root (default $HOME/satnogs_archive)
#   --since=<spec>          24h | 30m | 7d | ISO-8601-Z. Default: pick up
#                           where the last run left off — uses the latest
#                           downloaded observation's start from
#                           <out>/.latest_start.<norad>.txt, capped at
#                           --lookback-cap so a stale or absent state
#                           file can't trigger a multi-year API walk.
#                           Falls back to --lookback-cap on first run.
#   --lookback-cap=<spec>   Floor on the resolved --since when it would
#                           otherwise be older. Default 24h. Use 0 to
#                           disable.
#   --until=<spec>          Same syntax as --since (default: now)
#   --status=<value>        Filter by SatNOGS observation status
#                           (good|bad|failed|future|unknown). Default
#                           is empty (any status); the payload check
#                           still skips obs without audio, so `future`
#                           and `failed`-without-recording fall away
#                           naturally. The SatNOGS API only takes a
#                           single status, not a CSV.
#   --max=<n>               Cap total new downloads per run (default 200)
#   --tle-dir=<dir>         Override per-observation TLE with the newest
#                           *.tle file in this directory (default
#                           $HOME/FrontierSat/TLEs). Falls back to the
#                           SatNOGS-shipped TLE when the directory has
#                           no .tle file. Best for incremental pulls of
#                           recent passes; for historical backfill use
#                           --no-local-tle so each obs keeps the TLE
#                           SatNOGS actually used at the time.
#   --no-local-tle          Always use the SatNOGS-shipped TLE.
#   --decode                On success, run decode_passes.sh against --out
#   --rate-limit-ms=<n>     Sleep between HTTP calls (default 250)
#   --user-agent=<str>      HTTP User-Agent (default "sso-satnogs-pull/1.0")
#   --decode-passes=<path>  Override decode_passes.sh location
#   --db=<path>             Pass SSO_PACKET_DB into the --decode invocation
#   -h | --help             This help

set -uo pipefail
export LC_ALL=C

# Default to FrontierSat — current best-guess NORAD ID. Override with
# --norad-id=<n> for other satellites.
NORAD_ID="69015"
# FrontierSat shared data root: /FrontierSat on the ground machine,
# $HOME/FrontierSat on dev hosts. Override with the FRONTIERSAT_ROOT
# env var or run /FrontierSat-setup first. See etc/tmpfiles.d/sso.conf
# and scripts/sso_setup_root.sh for the one-time root setup.
: "${FRONTIERSAT_ROOT:=$([[ -d /FrontierSat ]] && echo /FrontierSat || echo "$HOME/FrontierSat")}"
OUT="$FRONTIERSAT_ROOT/satnogs_archive"
SINCE_SPEC=""              # empty => resolve from state file or 24h fallback
UNTIL_SPEC=""
# Default to no status filter — FrontierSat uses a custom protocol that
# SatNOGS can't decode, so the auto-classifier leaves our obs at
# `unknown` indefinitely and `status=good` would silently drop them.
# The payload-empty check below skips passes that haven't uploaded
# audio yet, so `future` and `failed`-without-recording still fall away.
STATUS_FILTER=""
MAX_OBS=200
DECODE_AFTER=0
RATE_LIMIT_MS=250
USER_AGENT="sso-satnogs-pull/1.0"
DECODE_PASSES_BIN=""
DB_PATH=""
TLE_DIR="$FRONTIERSAT_ROOT/TLEs"
USE_LOCAL_TLE=1
# Maximum reach-back when --since is auto-resolved from the state file.
# Caps the API window even when the cursor is stale (e.g. on a fresh
# host or after the state file was wiped). Override with
# --lookback-cap=<spec>; 0 disables the cap (use cursor verbatim).
LOOKBACK_CAP_SPEC="24h"

usage() {
    sed -n '2,/^# Usage:/p' "$0" | sed 's/^# \{0,1\}//'
    sed -n '/^# Options:/,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --norad-id=*)       NORAD_ID="${1#--norad-id=}";;
        --out=*)            OUT="${1#--out=}";;
        --since=*)          SINCE_SPEC="${1#--since=}";;
        --until=*)          UNTIL_SPEC="${1#--until=}";;
        --status=*)         STATUS_FILTER="${1#--status=}";;
        --max=*)            MAX_OBS="${1#--max=}";;
        --decode)           DECODE_AFTER=1;;
        --rate-limit-ms=*)  RATE_LIMIT_MS="${1#--rate-limit-ms=}";;
        --user-agent=*)     USER_AGENT="${1#--user-agent=}";;
        --decode-passes=*)  DECODE_PASSES_BIN="${1#--decode-passes=}";;
        --db=*)             DB_PATH="${1#--db=}";;
        --tle-dir=*)        TLE_DIR="${1#--tle-dir=}";;
        --no-local-tle)     USE_LOCAL_TLE=0;;
        --lookback-cap=*)   LOOKBACK_CAP_SPEC="${1#--lookback-cap=}";;
        -h|--help)          usage; exit 0;;
        *)                  echo "unknown arg: $1" >&2; usage >&2; exit 2;;
    esac
    shift
done

if [[ -z "$NORAD_ID" ]]; then
    echo "error: --norad-id is required" >&2
    exit 2
fi
case "$NORAD_ID" in
    ''|*[!0-9]*) echo "error: --norad-id must be numeric" >&2; exit 2;;
esac

command -v curl >/dev/null 2>&1 \
    || { echo "error: curl not found on PATH" >&2; exit 2; }
command -v jq   >/dev/null 2>&1 \
    || { echo "error: jq not found on PATH (apt install jq / brew install jq)" >&2; exit 2; }

mkdir -p "$OUT"

# Convert a since/until spec into an ISO-8601 Z timestamp. Accepts
# duration-from-now (90s|30m|24h|7d) or any string with a `T`, which we
# trust and pass through. The two `date` invocations bridge GNU
# coreutils (-d) and BSD/macOS (-r).
to_iso_utc() {
    local spec="$1"
    if [[ "$spec" == *T* ]]; then
        printf '%s' "$spec"
        return 0
    fi
    local n unit secs
    n="${spec%[smhd]}"
    unit="${spec: -1}"
    case "$unit" in
        s) secs=$((n));;
        m) secs=$((n * 60));;
        h) secs=$((n * 3600));;
        d) secs=$((n * 86400));;
        *) return 1;;
    esac
    local now cutoff
    now="$(date -u +%s)"
    cutoff=$((now - secs))
    if date -u -d "@$cutoff" "+%Y-%m-%dT%H:%M:%SZ" 2>/dev/null; then
        return 0
    elif date -u -r "$cutoff" "+%Y-%m-%dT%H:%M:%SZ" 2>/dev/null; then
        return 0
    fi
    return 1
}

# Per-NORAD-ID state file: latest start of an observation we've actually
# downloaded. We use it as the default cursor so we only ask the API for
# observations newer than what we already have.
STATE_FILE="${OUT}/.latest_start.${NORAD_ID}.txt"

# Convert "YYYY-MM-DDTHH:MM:SSZ" to epoch (GNU and BSD date variants).
iso_to_epoch() {
    local iso="$1"
    date -u -d "$iso" +%s 2>/dev/null \
        || date -u -j -f "%Y-%m-%dT%H:%M:%SZ" "$iso" +%s 2>/dev/null \
        || return 1
}

epoch_to_iso() {
    local secs="$1"
    date -u -d "@$secs" "+%Y-%m-%dT%H:%M:%SZ" 2>/dev/null \
        || date -u -r "$secs" "+%Y-%m-%dT%H:%M:%SZ" 2>/dev/null \
        || return 1
}

# Translate a duration spec (90s|30m|24h|7d) to seconds. Returns 0 if
# the spec is "0" (cap disabled) or empty. Errors out on garbage so a
# typo in --lookback-cap surfaces before the API call.
spec_to_seconds() {
    local spec="$1"
    if [[ -z "$spec" || "$spec" == "0" ]]; then echo 0; return 0; fi
    local n unit
    n="${spec%[smhd]}"
    unit="${spec: -1}"
    case "$spec" in
        ''|*[!0-9smhd]*) return 1;;
    esac
    case "$unit" in
        s) echo "$n";;
        m) echo $((n * 60));;
        h) echo $((n * 3600));;
        d) echo $((n * 86400));;
        *) return 1;;
    esac
}

# Default --since: pick up from the cursor in the state file, but
# clip to --lookback-cap so a stale / absent state file doesn't trigger
# a multi-year API walk. The cursor itself is held back past any obs
# that was still pending audio at the end of the previous run (see the
# EARLIEST_PENDING_START handling near the end of this script), which
# is what stops SatNOGS out-of-order uploads from being missed by the
# start>=since filter on the next tick.
if [[ -z "$SINCE_SPEC" ]]; then
    NOW_EPOCH="$(date -u +%s)"
    if ! LOOKBACK_S="$(spec_to_seconds "$LOOKBACK_CAP_SPEC")"; then
        echo "error: bad --lookback-cap='$LOOKBACK_CAP_SPEC'" >&2
        exit 2
    fi
    CAP_EPOCH=$((NOW_EPOCH - LOOKBACK_S))

    CURSOR_EPOCH=""
    if [[ -s "$STATE_FILE" ]]; then
        LATEST="$(head -n 1 "$STATE_FILE")"
        CURSOR_EPOCH="$(iso_to_epoch "$LATEST" 2>/dev/null || true)"
    fi

    if [[ -n "$CURSOR_EPOCH" ]]; then
        if [[ "$LOOKBACK_S" -gt 0 && "$CURSOR_EPOCH" -lt "$CAP_EPOCH" ]]; then
            SINCE_SPEC="$(epoch_to_iso "$CAP_EPOCH")"
        else
            SINCE_SPEC="$(epoch_to_iso "$CURSOR_EPOCH")"
        fi
    elif [[ "$LOOKBACK_S" -gt 0 ]]; then
        SINCE_SPEC="$(epoch_to_iso "$CAP_EPOCH")"
    else
        SINCE_SPEC="24h"
    fi
fi

SINCE_ISO="$(to_iso_utc "$SINCE_SPEC")" || {
    echo "error: bad --since='$SINCE_SPEC'" >&2; exit 2; }
UNTIL_ISO=""
if [[ -n "$UNTIL_SPEC" ]]; then
    UNTIL_ISO="$(to_iso_utc "$UNTIL_SPEC")" || {
        echo "error: bad --until='$UNTIL_SPEC'" >&2; exit 2; }
fi

# Newest local TLE — used to override the SatNOGS-shipped per-obs TLE
# when --tle-dir has at least one .tle. Walks the whole tree so the
# operator's typical layout (~/FrontierSat/TLEs/YYYYMMDD/tle-*.tle)
# works without a flag. Mtime-newest wins. Empty means fall back to
# whatever SatNOGS shipped for the observation.
LOCAL_TLE=""
if [[ "$USE_LOCAL_TLE" -eq 1 && -d "$TLE_DIR" ]]; then
    # BSD stat (macOS) first, then GNU find -printf fallback. The order
    # matters because stat's `-f` flag has different meanings on each
    # platform, so we just try both and take the first that yields a hit.
    LOCAL_TLE="$(find "$TLE_DIR" -type f -name '*.tle' \
                    -exec stat -f '%m %N' {} \; 2>/dev/null \
                | sort -nr | head -n 1 | cut -d ' ' -f 2-)"
    if [[ -z "$LOCAL_TLE" ]]; then
        LOCAL_TLE="$(find "$TLE_DIR" -type f -name '*.tle' \
                        -printf '%T@ %p\n' 2>/dev/null \
                    | sort -nr | head -n 1 | cut -d ' ' -f 2-)"
    fi
fi
[[ -n "$LOCAL_TLE" ]] && echo "satnogs_pull: using local TLE $LOCAL_TLE"

# Build the comma-separated status filter into a jq selector and a URL
# query fragment. If STATUS_FILTER is empty, accept everything.
STATUS_QUERY=""
if [[ -n "$STATUS_FILTER" ]]; then
    STATUS_QUERY="status=$STATUS_FILTER"
fi

# Lock — prefer flock (cleanly self-releases on process exit). Fall back
# to a mkdir-based directory lock for hosts that don't ship flock (e.g.
# macOS). A single EXIT trap cleans up both the lock dir and any temp
# files (set later); chaining via $CLEANUP_CMDS avoids the
# trap-overwrite footgun where a second `trap ... EXIT` would silently
# replace the first.
CLEANUP_CMDS=""
add_cleanup() { CLEANUP_CMDS="$CLEANUP_CMDS"$'\n'"$1"; }
do_cleanup() { eval "$CLEANUP_CMDS"; }
trap do_cleanup EXIT

if command -v flock >/dev/null 2>&1; then
    LOCK_FILE="$OUT/.lock"
    exec 9>"$LOCK_FILE"
    if ! flock -n 9; then
        echo "satnogs_pull: another instance holds $LOCK_FILE; exiting"
        exit 0
    fi
else
    LOCK_DIR="$OUT/.lock.d"
    if ! mkdir "$LOCK_DIR" 2>/dev/null; then
        echo "satnogs_pull: another instance holds $LOCK_DIR; exiting"
        exit 0
    fi
    add_cleanup "rmdir \"$LOCK_DIR\" 2>/dev/null || true"
fi

FETCHED_FILE="$OUT/.fetched.txt"
touch "$FETCHED_FILE"
declare -A FETCHED
while IFS= read -r line; do
    [[ -n "$line" ]] && FETCHED["$line"]=1
done < "$FETCHED_FILE"

API_BASE="https://network.satnogs.org/api/observations/"
QUERY="norad_cat_id=${NORAD_ID}&start=${SINCE_ISO}"
[[ -n "$UNTIL_ISO" ]]   && QUERY="${QUERY}&end=${UNTIL_ISO}"
[[ -n "$STATUS_QUERY" ]] && QUERY="${QUERY}&${STATUS_QUERY}"

echo "satnogs_pull: norad=${NORAD_ID}  since=${SINCE_ISO}${UNTIL_ISO:+  until=${UNTIL_ISO}}  status=${STATUS_FILTER:-any}  out=${OUT}"

curl_api() {
    curl --silent --show-error --fail -L \
         --connect-timeout 10 --max-time 60 \
         --retry 3 --retry-delay 2 \
         -H "User-Agent: ${USER_AGENT}" \
         -H "Accept: application/json" \
         "$@"
}

curl_audio() {
    curl --silent --show-error --fail -L \
         --connect-timeout 10 --max-time 300 \
         --retry 3 --retry-delay 2 \
         -H "User-Agent: ${USER_AGENT}" \
         "$@"
}

polite_sleep() {
    if [[ "$RATE_LIMIT_MS" -gt 0 ]]; then
        local secs
        secs="$(awk -v ms="$RATE_LIMIT_MS" 'BEGIN { printf "%.3f", ms/1000 }')"
        sleep "$secs"
    fi
}

# Capture both the response body and the Link header so we can follow
# rel=next for pagination. Returns 0 on success, prints body to stdout,
# next URL to fd 4 (caller redirects).
PAGE_HDR="$(mktemp -t satnogs_pull_hdr_XXXXXX)"
add_cleanup "rm -f \"$PAGE_HDR\""

list_page() {
    local url="$1"
    : > "$PAGE_HDR"
    curl_api -D "$PAGE_HDR" "$url"
}

next_url_from_headers() {
    # SatNOGS returns: `link: <https://.../?cursor=...>; rel="next"`
    # Other rels (prev, first) may also be present.
    awk -v IGNORECASE=1 '
        /^link:/ {
            line = $0
            # Strip the leading "link:" key.
            sub(/^[Ll]ink:[[:space:]]*/, "", line)
            n = split(line, parts, ",")
            for (i = 1; i <= n; i++) {
                if (match(parts[i], /rel="?next"?/)) {
                    if (match(parts[i], /<[^>]+>/)) {
                        url = substr(parts[i], RSTART + 1, RLENGTH - 2)
                        print url
                        exit
                    }
                }
            }
        }
    ' "$PAGE_HDR"
}

URL="${API_BASE}?${QUERY}"
COUNT_FETCHED=0
COUNT_SKIPPED=0
COUNT_FAILED=0
COUNT_SEEN=0
NEWEST_START=""
# Earliest already-happened obs (start < now) we saw without an audio
# payload yet. Used at the end to clamp the cursor so the next run's
# start>=since query still includes it, even if SatNOGS uploaded a
# newer pass's audio first.
EARLIEST_PENDING_START=""
NOW_ISO_FOR_PENDING="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
if [[ -s "$STATE_FILE" ]]; then
    # Carry the existing cursor forward so a run that downloads nothing
    # still seeds the same state file value (no regressions on disk).
    NEWEST_START="$(head -n 1 "$STATE_FILE")"
fi

while [[ -n "$URL" && "$COUNT_FETCHED" -lt "$MAX_OBS" ]]; do
    echo "satnogs_pull: GET $URL"
    if ! JSON="$(list_page "$URL")"; then
        echo "satnogs_pull: API list request failed" >&2
        break
    fi

    # Both old (wrapped) and new (raw array) shapes — accept either.
    if echo "$JSON" | jq -e 'type == "array"' >/dev/null 2>&1; then
        RESULTS="$JSON"
    else
        RESULTS="$(echo "$JSON" | jq -c '.results // []')"
    fi
    NEXT="$(next_url_from_headers)"

    N="$(echo "$RESULTS" | jq 'length')"
    if [[ "$N" -eq 0 ]]; then
        URL="$NEXT"
        continue
    fi

    for ((i = 0; i < N; i++)); do
        if [[ "$COUNT_FETCHED" -ge "$MAX_OBS" ]]; then
            break 2
        fi

        OBS_LIST="$(echo "$RESULTS" | jq -c ".[$i]")"
        OBS_ID="$(echo "$OBS_LIST" | jq -r '.id // empty')"
        [[ -z "$OBS_ID" ]] && continue
        COUNT_SEEN=$((COUNT_SEEN + 1))

        if [[ -n "${FETCHED[$OBS_ID]:-}" ]]; then
            COUNT_SKIPPED=$((COUNT_SKIPPED + 1))
            continue
        fi

        # The list endpoint's `payload` field carries only the directory
        # URL (e.g. .../<obs-id>/), and other fields trickle in piecemeal
        # over time. The detail endpoint returns the canonical audio URL
        # plus the TLE block. One extra GET per new obs is cheap and
        # avoids guessing at file names.
        if ! OBS="$(curl_api "${API_BASE}${OBS_ID}/?format=json")"; then
            echo "    !! detail fetch failed for obs $OBS_ID" >&2
            COUNT_FAILED=$((COUNT_FAILED + 1))
            polite_sleep
            continue
        fi

        STATUS="$(echo "$OBS"     | jq -r '.status // ""')"
        PAYLOAD="$(echo "$OBS"    | jq -r '.payload // ""')"
        START="$(echo "$OBS"      | jq -r '.start // ""')"
        STATION="$(echo "$OBS"    | jq -r '.ground_station // ""')"
        STATION_NAME="$(echo "$OBS" | jq -r '.station_name // ""')"

        if [[ -z "$PAYLOAD" || "$PAYLOAD" == "null" || "$PAYLOAD" == */ ]]; then
            # SatNOGS knows about the observation but hasn't uploaded
            # the audio yet — typical for the first ~30 min after a
            # pass ends. DON'T pin the id in .fetched.txt; the next
            # cron tick re-checks the detail endpoint and grabs the
            # audio once it's there. For low-volume sats like
            # FrontierSat the extra detail GET per tick is negligible.
            # Only track obs whose pass should already be over; future
            # passes shouldn't hold the cursor back for hours/days.
            if [[ -n "$START" && "$START" < "$NOW_ISO_FOR_PENDING" ]]; then
                if [[ -z "$EARLIEST_PENDING_START" || "$START" < "$EARLIEST_PENDING_START" ]]; then
                    EARLIEST_PENDING_START="$START"
                fi
            fi
            COUNT_SKIPPED=$((COUNT_SKIPPED + 1))
            polite_sleep
            continue
        fi

        # Filename-safe start: replace HH:MM:SS with HH-MM-SS and drop
        # any trailing fractional / Z so decode_passes.sh's regex picks
        # the timestamp up.
        FN_START="$(printf '%s' "$START" \
            | sed -E 's/T([0-9]{2}):([0-9]{2}):([0-9]{2}).*/T\1-\2-\3/')"
        if [[ -z "$FN_START" || "$FN_START" == "$START" ]]; then
            FN_START="$(date -u +%Y-%m-%dT%H-%M-%S)"
        fi

        EXT_RAW="${PAYLOAD##*.}"
        EXT="${EXT_RAW,,}"
        case "$EXT" in
            ogg|wav|opus|flac) ;;
            *)                 EXT="ogg";;
        esac

        OBS_DIR="${OUT}/${OBS_ID}"
        AUDIO_FILE="${OBS_DIR}/satnogs_${OBS_ID}_${FN_START}.${EXT}"
        TLE_FILE="${OBS_DIR}/satnogs_${OBS_ID}.tle"
        META_FILE="${OBS_DIR}/satnogs_${OBS_ID}.meta.json"

        if [[ -f "$AUDIO_FILE" ]]; then
            echo "$OBS_ID" >> "$FETCHED_FILE"
            FETCHED["$OBS_ID"]=1
            COUNT_SKIPPED=$((COUNT_SKIPPED + 1))
            polite_sleep
            continue
        fi

        mkdir -p "$OBS_DIR"
        echo "$OBS" | jq '.' > "$META_FILE"

        # Prefer the local (operator-trusted) TLE if --tle-dir produced
        # one; that's the right call for incremental pulls of recent
        # passes. Fall back to the SatNOGS-shipped per-obs TLE for
        # historical / --no-local-tle runs.
        if [[ -n "$LOCAL_TLE" && -f "$LOCAL_TLE" ]]; then
            cp -- "$LOCAL_TLE" "$TLE_FILE"
        else
            TLE0="$(echo "$OBS" | jq -r '.tle0 // empty')"
            TLE1="$(echo "$OBS" | jq -r '.tle1 // empty')"
            TLE2="$(echo "$OBS" | jq -r '.tle2 // empty')"
            if [[ -z "$TLE1" || -z "$TLE2" ]]; then
                TLE_BLOCK="$(echo "$OBS" | jq -r '.tle // empty')"
                if [[ -n "$TLE_BLOCK" ]]; then
                    TLE0="$(echo "$TLE_BLOCK" | sed -n '1p')"
                    TLE1="$(echo "$TLE_BLOCK" | sed -n '2p')"
                    TLE2="$(echo "$TLE_BLOCK" | sed -n '3p')"
                fi
            fi
            if [[ -n "$TLE1" && -n "$TLE2" ]]; then
                {
                    if [[ -n "$TLE0" ]]; then
                        printf '%s\n' "$TLE0"
                    else
                        printf 'NORAD_%s\n' "$NORAD_ID"
                    fi
                    printf '%s\n' "$TLE1"
                    printf '%s\n' "$TLE2"
                } > "$TLE_FILE"
            fi
        fi

        echo "satnogs_pull: GET $PAYLOAD"
        TMP="${AUDIO_FILE}.part"
        if curl_audio -o "$TMP" "$PAYLOAD"; then
            mv -f "$TMP" "$AUDIO_FILE"
            echo "$OBS_ID" >> "$FETCHED_FILE"
            FETCHED["$OBS_ID"]=1
            COUNT_FETCHED=$((COUNT_FETCHED + 1))
            # ISO-8601 sorts lexicographically, so plain string > works
            # for tracking the newest downloaded start.
            if [[ -z "$NEWEST_START" || "$START" > "$NEWEST_START" ]]; then
                NEWEST_START="$START"
            fi
            echo "    -> $AUDIO_FILE  status=${STATUS}${STATION_NAME:+ station=\"$STATION_NAME\"}"
        else
            rm -f "$TMP"
            COUNT_FAILED=$((COUNT_FAILED + 1))
            echo "    !! audio download failed for obs $OBS_ID" >&2
        fi

        polite_sleep
    done

    URL="$NEXT"
done

if [[ -n "$EARLIEST_PENDING_START" ]]; then
    # SatNOGS sometimes uploads a newer pass's audio before an older
    # one's. Without this clamp the cursor would walk past the older
    # obs after we successfully fetched the newer one, and the next
    # start>=since query would miss it forever.
    if [[ -z "$NEWEST_START" || "$EARLIEST_PENDING_START" < "$NEWEST_START" ]]; then
        NEWEST_START="$EARLIEST_PENDING_START"
    fi
fi

if [[ -n "$NEWEST_START" ]]; then
    printf '%s\n' "$NEWEST_START" > "$STATE_FILE"
fi

echo "satnogs_pull: done.  seen=${COUNT_SEEN}  fetched=${COUNT_FETCHED}  skipped=${COUNT_SKIPPED}  failed=${COUNT_FAILED}${NEWEST_START:+  cursor=${NEWEST_START}}  out=${OUT}"

if [[ "$DECODE_AFTER" -eq 1 && "$COUNT_FETCHED" -gt 0 ]]; then
    if [[ -z "$DECODE_PASSES_BIN" ]]; then
        for cand in \
            "$HOME/bin/decode_passes.sh" \
            "$(dirname "$0")/decode_passes.sh" \
            "$(command -v decode_passes.sh 2>/dev/null || true)"
        do
            if [[ -n "$cand" && -x "$cand" ]]; then
                DECODE_PASSES_BIN="$cand"
                break
            fi
        done
    fi
    if [[ -z "$DECODE_PASSES_BIN" || ! -x "$DECODE_PASSES_BIN" ]]; then
        echo "satnogs_pull: --decode requested but decode_passes.sh not found" >&2
        exit 1
    fi
    echo "satnogs_pull: invoking $DECODE_PASSES_BIN --root $OUT"
    if [[ -n "$DB_PATH" ]]; then
        SSO_PACKET_DB="$DB_PATH" "$DECODE_PASSES_BIN" --root "$OUT"
    else
        "$DECODE_PASSES_BIN" --root "$OUT"
    fi
fi
