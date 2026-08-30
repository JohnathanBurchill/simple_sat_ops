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
# Cron example (every 4 min for FrontierSat, with auto-decode):
#   */4 * * * * $HOME/bin/satnogs_pull.sh --decode \
#       >> $HOME/var/log/sso/satnogs_pull.log 2>&1
#
# Courtesy & rate limits
#   The SatNOGS Network API throttles the observations endpoint at 60
#   requests/hour anonymous, 240/hour with an API token (see the
#   libre.space thread "API and throttling: anonymous vs with API
#   token", topic 12091). A token raises only the rate ceiling —
#   observation data is public either way — so pass one with
#   --api-token / $SATNOGS_API_TOKEN when polling often.
#
#   Only the listing is throttled. The endpoint installs its rate limit
#   on the `list` action alone, so a request for one observation is not
#   counted, and neither is the audio, which comes from object storage
#   on another host. What this script spends, therefore, is one request
#   per page of 25 observations in the window it asks for.
#
#   In steady state that is a single request per run: 4-minute polling
#   is 15 requests an hour for one satellite, comfortably inside even
#   the anonymous ceiling. The cost only grows when the window does —
#   a busy satellite draws upwards of 50 SatNOGS observations an hour
#   worldwide, so each hour the window reaches back is another two
#   pages, and a catch-up run over a full day is around 50 requests.
#   For many satellites at once, raise --rate-limit-ms, stagger the cron
#   entries, or use a token.
#
#   That budget holds only while the cursor keeps advancing, and two
#   bugs used to stop it — between them they got the ground station's
#   IP blocked at the SatNOGS edge in August 2026, after three weeks at
#   700-900 listings/hour against a 240/hour ceiling:
#
#     - An observation whose audio never arrived pinned the cursor to
#       its start time forever, so every 4-minute tick re-listed the
#       whole 24-hour window: about 50 pages a tick, 15 ticks an hour.
#       --pending-max-age now writes such an obs off (default 2h).
#     - curl --retry treats a 429 as retryable, so each throttled
#       request became four. curl_api no longer retries at all; the
#       next cron tick is the retry.
#
#   The `list reqs (last hr)` row in the run summary is the early
#   warning. In steady state it should read in the low tens. If it
#   climbs toward the ceiling, the cursor is stuck — check
#   <out>/.latest_start.<norad>.txt against the current time before it
#   turns into another block.
#
#   Each run ends with a `=== summary` table whose API rows report the
#   accesses made this run (audio downloads excluded) and the accesses in
#   the trailing 60 minutes against the ceiling — the same rolling hour the
#   SatNOGS throttle counts, so a backfill burst ages out instead of
#   poisoning the figure. The per-archive tally lives in
#   <out>/.api_stats.txt; delete it to reset.
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
#   --pending-max-age=<spec> How long an observation whose audio hasn't
#                           been uploaded yet may hold the cursor back.
#                           Default 2h. Past that the cursor moves on
#                           and the obs is written off, so a recording
#                           that never arrives can't pin every future
#                           run to the same window. Use 0 to wait
#                           indefinitely.
#   --until=<spec>          Same syntax as --since (default: now)
#   --status=<value>        Filter by SatNOGS observation status
#                           (good|bad|failed|future|unknown). Default
#                           is empty (any status); the payload check
#                           still skips obs without audio, so `future`
#                           and `failed`-without-recording fall away
#                           naturally. The SatNOGS API only takes a
#                           single status, not a CSV.
#   --max=<n>               Cap total new downloads per run (default 200)
#   --cache-day=<YYYY-MM-DD> List one UTC day into
#                           <out>/.daycache/<norad>/<day>.tsv and
#                           download nothing. This is what
#                           satnogs_browser calls to fill its day view;
#                           a day already listed costs no further
#                           requests, however often it is browsed. The
#                           cursor is neither read nor written, so
#                           listing an old day cannot disturb the cron
#                           job's position.
#   --obs-ids=<n,n,...>     Download exactly these observations and
#                           nothing else. The API's observation_id
#                           filter takes a list, so a marked set costs
#                           one listing page per 25 ids no matter how
#                           far apart their passes are. Also leaves the
#                           cursor alone.
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
#   --api-token=<str>       SatNOGS API token, sent as an Authorization
#                           header on API requests (never on audio
#                           downloads). Raises the throttle ceiling from
#                           60 to 240 requests/hour. Defaults to
#                           $SATNOGS_API_TOKEN.
#   --decode-passes=<path>  Override decode_passes.sh location
#   --db=<path>             Pass SSO_PACKET_DB into the --decode invocation
#   --jobs=<n>              Parallel decode workers for --decode (passed
#                           through to decode_passes.sh --jobs; default
#                           there is one per CPU). The download itself is
#                           always serial and rate-limited.
#   -h | --help             This help

set -uo pipefail
export LC_ALL=C

# Timestamped logging. The script is built for unattended cron use with
# stdout/stderr appended to a log file, so every lifecycle line carries a
# UTC timestamp (matching the SatNOGS API and the rest of this script) to
# mark when each run — and each step within it — happened.
now_ts() { date -u +%Y-%m-%dT%H:%M:%SZ; }
log()    { printf '%s satnogs_pull: %s\n' "$(now_ts)" "$*"; }
logerr() { printf '%s satnogs_pull: %s\n' "$(now_ts)" "$*" >&2; }

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
UNTIL_SPEC=""              # empty => now (excludes future scheduled passes)
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
# Optional SatNOGS API token. Sent only to the API host, never to the
# audio/object-storage host. Lifts the throttle from 60 to 240 req/hour.
API_TOKEN="${SATNOGS_API_TOKEN:-}"
DECODE_PASSES_BIN=""
DB_PATH=""
JOBS_SPEC=""
TLE_DIR="$FRONTIERSAT_ROOT/TLEs"
USE_LOCAL_TLE=1
# Maximum reach-back when --since is auto-resolved from the state file.
# Caps the API window even when the cursor is stale (e.g. on a fresh
# host or after the state file was wiped). Override with
# --lookback-cap=<spec>; 0 disables the cap (use cursor verbatim).
LOOKBACK_CAP_SPEC="24h"
# How long an observation with no audio yet may hold the cursor back.
# SatNOGS usually uploads within ~30 min of a pass ending, but an obs
# whose recording never arrives (a station that dropped it, a `failed`
# obs with nothing to upload) would otherwise pin the cursor forever
# and make every tick re-walk the whole window. Past this age we stop
# waiting and let the cursor move on. Two hours leaves plenty of room
# for a late upload while keeping the worst-case window small: on a
# satellite drawing 50 observations an hour, a cursor pinned two hours
# back is five listing pages a tick, where six hours would be thirteen.
# Override with --pending-max-age; 0 waits indefinitely.
PENDING_MAX_AGE_SPEC="2h"
# Browse-cache mode: list one UTC day into a tab-separated file under
# <out>/.daycache/<norad>/ and download nothing. satnogs_browser reads
# those files rather than the network, so a day it has already listed
# is free to revisit.
CACHE_DAY=""
# Download exactly these observation ids, ignoring the time window
# entirely. This is how the browser fetches a marked set.
OBS_IDS=""

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
        --api-token=*)      API_TOKEN="${1#--api-token=}";;
        --decode-passes=*)  DECODE_PASSES_BIN="${1#--decode-passes=}";;
        --db=*)             DB_PATH="${1#--db=}";;
        --jobs=*)           JOBS_SPEC="${1#--jobs=}";;
        --tle-dir=*)        TLE_DIR="${1#--tle-dir=}";;
        --no-local-tle)     USE_LOCAL_TLE=0;;
        --lookback-cap=*)   LOOKBACK_CAP_SPEC="${1#--lookback-cap=}";;
        --pending-max-age=*) PENDING_MAX_AGE_SPEC="${1#--pending-max-age=}";;
        --cache-day=*)      CACHE_DAY="${1#--cache-day=}";;
        --obs-ids=*)        OBS_IDS="${1#--obs-ids=}";;
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

# Mark the start of every run up front — before lock contention or the
# --since resolution can short-circuit — so the cron log always shows
# when (and whether) a tick fired.
log "start (norad ${NORAD_ID}, pid $$)"

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

# Per-archive API-access tally: one "<epoch> <calls>" record per run.
# Records older than an hour age out each run, so the surviving sum is the
# request count in the trailing 60 minutes — exactly the rolling hour the
# SatNOGS throttle counts. Not keyed by NORAD ID: the throttle is per
# client, not per satellite, and every run sharing this archive serialises
# on the lock, so one shared file reflects the real rate even when several
# satellites pull into the same --out. Delete it to reset.
STATS_FILE="${OUT}/.api_stats.txt"

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

# The two browser modes each pin the query themselves, so neither one
# consults the cursor. --cache-day bounds the window to one UTC day;
# --obs-ids replaces the window with an explicit list.
if [[ -n "$CACHE_DAY" && -n "$OBS_IDS" ]]; then
    echo "error: --cache-day and --obs-ids do different things; pick one" >&2
    exit 2
fi
if [[ -n "$CACHE_DAY" ]]; then
    if ! DAY_START_EPOCH="$(iso_to_epoch "${CACHE_DAY}T00:00:00Z")"; then
        echo "error: --cache-day wants YYYY-MM-DD, got '$CACHE_DAY'" >&2
        exit 2
    fi
    SINCE_SPEC="$(epoch_to_iso "$DAY_START_EPOCH")"
    UNTIL_SPEC="$(epoch_to_iso $((DAY_START_EPOCH + 86400)))"
fi
if [[ -n "$OBS_IDS" ]]; then
    OBS_IDS="${OBS_IDS//[[:space:]]/}"
    case "$OBS_IDS" in
        *[!0-9,]* | ,* | *, | *,,*)
            echo "error: --obs-ids wants a comma-separated list of numbers, got '$OBS_IDS'" >&2
            exit 2;;
    esac
    # Unused by the query, which the id list fully determines, but the
    # resolution below still wants something parseable.
    SINCE_SPEC="1970-01-01T00:00:00Z"
fi

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

# Default the upper bound to now. SatNOGS returns observations
# newest-first and schedules passes days ahead, so an unbounded query
# walks every future scheduled pass first — each one an extra (slow)
# list page plus a wasted detail GET on every run — before reaching the
# completed passes that actually have audio. Bounding end=now drops the
# future passes server-side. Pass --until explicitly to widen it.
if [[ -z "$UNTIL_SPEC" ]]; then
    UNTIL_SPEC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
fi
UNTIL_ISO="$(to_iso_utc "$UNTIL_SPEC")" || {
    echo "error: bad --until='$UNTIL_SPEC'" >&2; exit 2; }

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
[[ -n "$LOCAL_TLE" ]] && log "using local TLE $LOCAL_TLE"

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
        log "another instance holds $LOCK_FILE; exiting"
        exit 0
    fi
else
    LOCK_DIR="$OUT/.lock.d"
    if ! mkdir "$LOCK_DIR" 2>/dev/null; then
        log "another instance holds $LOCK_DIR; exiting"
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
if [[ -n "$OBS_IDS" ]]; then
    # observation_id takes a list, so 25 marked observations come back
    # in one page whether their passes are minutes or months apart.
    QUERY="observation_id=${OBS_IDS}"
else
    QUERY="norad_cat_id=${NORAD_ID}&start=${SINCE_ISO}"
    [[ -n "$UNTIL_ISO" ]]    && QUERY="${QUERY}&end=${UNTIL_ISO}"
    [[ -n "$STATUS_QUERY" ]] && QUERY="${QUERY}&${STATUS_QUERY}"
fi

log "norad=${NORAD_ID}  since=${SINCE_ISO}${UNTIL_ISO:+  until=${UNTIL_ISO}}  status=${STATUS_FILTER:-any}  out=${OUT}"

# API requests carry the token when one is set. curl strips the
# Authorization header on a cross-host redirect, so -L can't leak it to
# the object-storage host; audio downloads (curl_audio) never get it.
# No --retry here, deliberately. curl treats a 429 as retryable, so a
# throttled request would become four, pushing us further over the
# ceiling exactly when we need to back off. The cron tick is the retry:
# a failed list just ends the run and the next one picks it up. Audio
# downloads keep their retries — they go to object storage, which the
# API throttle doesn't count.
curl_api() {
    local auth=()
    [[ -n "$API_TOKEN" ]] && auth=(-H "Authorization: Token ${API_TOKEN}")
    curl --silent --show-error --fail -L \
         --connect-timeout 10 --max-time 60 \
         -H "User-Agent: ${USER_AGENT}" \
         -H "Accept: application/json" \
         ${auth[@]+"${auth[@]}"} \
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

# In --cache-day mode the day is written to a temporary file and moved
# into place only once the whole walk succeeds, so a run interrupted
# part way leaves whatever cache was there before rather than half a
# day that looks complete.
CACHE_FILE=""
CACHE_TMP=""
WALK_OK=1
if [[ -n "$CACHE_DAY" ]]; then
    CACHE_DIR="${OUT}/.daycache/${NORAD_ID}"
    mkdir -p "$CACHE_DIR"
    CACHE_FILE="${CACHE_DIR}/${CACHE_DAY}.tsv"
    CACHE_TMP="$(mktemp -t satnogs_daycache_XXXXXX)"
    add_cleanup "rm -f \"$CACHE_TMP\""
fi

URL="${API_BASE}?${QUERY}"
COUNT_FETCHED=0
COUNT_SKIPPED=0
COUNT_FAILED=0
COUNT_SEEN=0
# List-page requests made this run. These are the only requests SatNOGS
# throttles: the observations endpoint applies its rate limit to the
# `list` action alone, so nothing else this script does is counted --
# not the audio, which comes from object storage on another host, and
# not any per-observation lookup, which this script no longer makes.
API_CALLS=0
NEWEST_START=""
# Earliest already-happened obs (start < now) we saw without an audio
# payload yet. Used at the end to clamp the cursor so the next run's
# start>=since query still includes it, even if SatNOGS uploaded a
# newer pass's audio first.
EARLIEST_PENDING_START=""
NOW_ISO_FOR_PENDING="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
# Oldest start time a pending (no-audio-yet) observation may have and
# still hold the cursor back. Empty means no floor — wait forever.
PENDING_FLOOR_ISO=""
if ! PENDING_MAX_AGE_S="$(spec_to_seconds "$PENDING_MAX_AGE_SPEC")"; then
    echo "error: bad --pending-max-age='$PENDING_MAX_AGE_SPEC'" >&2
    exit 2
fi
if [[ "$PENDING_MAX_AGE_S" -gt 0 ]]; then
    PENDING_FLOOR_ISO="$(epoch_to_iso $(( $(date -u +%s) - PENDING_MAX_AGE_S )))"
fi
if [[ -s "$STATE_FILE" ]]; then
    # Carry the existing cursor forward so a run that downloads nothing
    # still seeds the same state file value (no regressions on disk).
    NEWEST_START="$(head -n 1 "$STATE_FILE")"
fi

while [[ -n "$URL" && "$COUNT_FETCHED" -lt "$MAX_OBS" ]]; do
    log "GET $URL"
    API_CALLS=$((API_CALLS + 1))
    if ! JSON="$(list_page "$URL")"; then
        logerr "API list request failed"
        WALK_OK=0
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

    if [[ -n "$CACHE_DAY" ]]; then
        # One tab-separated line per observation. An empty payload
        # column means SatNOGS holds no audio for that pass, which is
        # the whole question the browser's day view asks -- the listing
        # answers it, so nothing downstream has to ask again.
        echo "$RESULTS" | jq -r '.[] | [
                (.id | tostring),
                (.start // ""),
                (.end // ""),
                (.status // ""),
                (.waterfall_status // ""),
                (.ground_station // "" | tostring),
                (.station_name // ""),
                (.max_altitude // "" | tostring),
                (.payload // "")
            ] | @tsv' >> "$CACHE_TMP"
        COUNT_SEEN=$((COUNT_SEEN + N))
        URL="$NEXT"
        polite_sleep
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

        # Everything we need is already in the list record: the full
        # audio URL in `payload`, the status, the station name and the
        # three TLE lines. The list used to hand back only a directory
        # URL, which is why this once fetched the detail endpoint for
        # every observation -- one wasted request per new pass, on a
        # listing that had already answered the question.
        OBS="$OBS_LIST"

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
                if [[ -n "$PENDING_FLOOR_ISO" && "$START" < "$PENDING_FLOOR_ISO" ]]; then
                    # Waited long enough. The recording isn't coming,
                    # so stop letting this obs hold the cursor back —
                    # otherwise every future run re-walks the window
                    # from here and burns through the API throttle.
                    echo "    .. giving up on obs $OBS_ID (start $START, still no audio)" >&2
                elif [[ -z "$EARLIEST_PENDING_START" || "$START" < "$EARLIEST_PENDING_START" ]]; then
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

        log "GET $PAYLOAD"
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

if [[ -n "$CACHE_DAY" ]]; then
    if [[ "$WALK_OK" -eq 1 ]]; then
        # Sorted by start time so the browser renders in pass order
        # without sorting, and moved into place in a single step.
        SORTED="${CACHE_TMP}.sorted"
        add_cleanup "rm -f \"$SORTED\""
        sort -t "$(printf '\t')" -k2,2 "$CACHE_TMP" > "$SORTED"
        mv -f "$SORTED" "$CACHE_FILE"
        log "cached $COUNT_SEEN observations for $CACHE_DAY in $CACHE_FILE"
    else
        logerr "listing for $CACHE_DAY did not complete; cache left as it was"
    fi
fi

# The cursor belongs to the polling run. Browsing an old day or
# fetching a hand-picked set must not drag it backwards or shove it
# forwards, so neither browser mode writes it.
if [[ -n "$NEWEST_START" && -z "$CACHE_DAY" && -z "$OBS_IDS" ]]; then
    printf '%s\n' "$NEWEST_START" > "$STATE_FILE"
fi

# Roll this run's API accesses into the per-archive tally and total the
# accesses in the trailing 60 minutes — the same rolling hour the SatNOGS
# throttle counts (60/hour anonymous, 240 with a token). Keeping a window
# of per-run records (rather than a cumulative mean) means a one-off
# backfill burst ages out after an hour instead of inflating the figure
# for the rest of the day. Safe to do here unlocked: the flock/mkdir lock
# above serialises every run on this archive.
NOW_EPOCH_STATS="$(date -u +%s)"
API_WINDOW_START=$((NOW_EPOCH_STATS - 3600))
API_LAST_HOUR=0
STATS_TMP="$(mktemp -t satnogs_pull_stats_XXXXXX)"
if [[ -s "$STATS_FILE" ]]; then
    # Keep only records inside the trailing hour; drop stale or malformed
    # lines (this also retires the old cumulative-mean file format).
    while read -r ts calls _; do
        case "$ts"    in ''|*[!0-9]*) continue;; esac
        case "$calls" in ''|*[!0-9]*) continue;; esac
        if [[ "$ts" -ge "$API_WINDOW_START" ]]; then
            printf '%s %s\n' "$ts" "$calls" >> "$STATS_TMP"
            API_LAST_HOUR=$((API_LAST_HOUR + calls))
        fi
    done < "$STATS_FILE"
fi
printf '%s %s\n' "$NOW_EPOCH_STATS" "$API_CALLS" >> "$STATS_TMP"
API_LAST_HOUR=$((API_LAST_HOUR + API_CALLS))
mv -f "$STATS_TMP" "$STATS_FILE"

# Throttle ceiling for the trailing-hour figure: a token lifts it 60 -> 240.
if [[ -n "$API_TOKEN" ]]; then API_CEILING=240; else API_CEILING=60; fi

# End-of-run summary table. The API rows sit in their own labelled lines
# so the request budget reads clearly instead of buried in a stats line.
log "done (norad ${NORAD_ID})"
{
    echo "=== summary"
    printf '    %-22s %s\n' "observations seen:"  "$COUNT_SEEN"
    printf '    %-22s %s\n' "downloaded:"         "$COUNT_FETCHED"
    printf '    %-22s %s\n' "skipped:"            "$COUNT_SKIPPED"
    printf '    %-22s %s\n' "failed:"             "$COUNT_FAILED"
    printf '    %-22s %s\n' "list requests (run):" "$API_CALLS"
    printf '    %-22s %s\n' "list reqs (last hr):" "${API_LAST_HOUR} / ${API_CEILING}"
    # Only the polling run has a cursor; the browser modes deliberately
    # leave it alone, so printing one there would misdescribe what the
    # run just did.
    if [[ -n "$NEWEST_START" && -z "$CACHE_DAY" && -z "$OBS_IDS" ]]; then
        printf '    %-22s %s\n' "cursor:" "$NEWEST_START"
    fi
    printf '    %-22s %s\n' "archive:"            "$OUT"
}

if [[ "$DECODE_AFTER" -eq 1 ]]; then
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
        logerr "--decode requested but decode_passes.sh not found"
        exit 1
    fi
    DECODE_ARGS=( --root "$OUT" )
    [[ -n "$JOBS_SPEC" ]] && DECODE_ARGS+=( --jobs "$JOBS_SPEC" )
    log "invoking $DECODE_PASSES_BIN ${DECODE_ARGS[*]}"
    if [[ -n "$DB_PATH" ]]; then
        SSO_PACKET_DB="$DB_PATH" "$DECODE_PASSES_BIN" "${DECODE_ARGS[@]}"
    else
        "$DECODE_PASSES_BIN" "${DECODE_ARGS[@]}"
    fi
fi
