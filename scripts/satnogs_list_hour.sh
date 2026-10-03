#!/usr/bin/env bash
# satnogs_list_hour.sh
#
# List the SatNOGS observations of one satellite that start in one UTC
# hour, into <out>/.hourcache/<norad>/<YYYY-MM-DDTHH>.tsv, and download
# nothing. This is what satnogs_passes calls to fill its grid. It is a
# script of its own so the cron job's satnogs_pull.sh is left alone.
#
# It shares that script's archive all the same, because the request
# budget is shared: it takes the same lock, so the two never run at
# once, and it writes each request to the same .api_stats.txt tally, so
# the cron run and every browser see one count of what this address has
# spent against the SatNOGS throttle. When the lock is held it exits 0
# without listing; the caller sees the cache unchanged and tries later.
#
# The listing is written to a temporary file and moved into place only
# once every page is in, so a run that fails part way leaves the hour's
# previous listing (or none) rather than half an hour that looks whole.
#
# The query reaches 20 minutes past the hour. The API's end filter keeps
# only observations that have ENDED by then, so a pass running over the
# top of the hour would otherwise be in neither hour's listing; the
# extra rows are dropped again, keeping only passes that start inside.
#
# Columns, tab-separated, the same as satnogs_pull.sh --cache-day:
#   id, start, end, status, waterfall status, station id, station name,
#   max elevation, audio URL (empty if none), demodulated frame count
#
# Usage:
#   satnogs_list_hour.sh --hour=<YYYY-MM-DDTHH> [options]
#
# Options:
#   --hour=<YYYY-MM-DDTHH>  The UTC hour to list (required)
#   --norad-id=<n>          NORAD catalog ID (default 69015 — FrontierSat)
#   --out=<dir>             Archive root (default: the FrontierSat data
#                           root's satnogs_archive, as satnogs_pull.sh)
#   --api-token=<str>       SatNOGS Network API token; falls back to
#                           $SATNOGS_API_TOKEN, then <out>/.api_token
#   --rate-limit-ms=<n>     Sleep between pages (default 250)
#   -h | --help             This help

set -uo pipefail
export LC_ALL=C

NORAD_ID="69015"
: "${FRONTIERSAT_ROOT:=$([[ -d /FrontierSat ]] && echo /FrontierSat || echo "$HOME/FrontierSat")}"
OUT="$FRONTIERSAT_ROOT/satnogs_archive"
HOUR=""
API_TOKEN="${SATNOGS_API_TOKEN:-}"
RATE_LIMIT_MS=250
USER_AGENT="sso-satnogs-pull/1.0"

usage() {
    sed -n '2,/^# Usage:/p' "$0" | sed 's/^# \{0,1\}//'
    sed -n '/^# Usage:/,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --hour=*)          HOUR="${1#--hour=}";;
        --norad-id=*)      NORAD_ID="${1#--norad-id=}";;
        --out=*)           OUT="${1#--out=}";;
        --api-token=*)     API_TOKEN="${1#--api-token=}";;
        --rate-limit-ms=*) RATE_LIMIT_MS="${1#--rate-limit-ms=}";;
        -h|--help)         usage; exit 0;;
        *)                 echo "error: unknown argument $1" >&2; exit 2;;
    esac
    shift
done

case "$NORAD_ID" in
    ''|*[!0-9]*) echo "error: --norad-id must be numeric" >&2; exit 2;;
esac
if [[ ! "$HOUR" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}$ ]]; then
    echo "error: --hour wants YYYY-MM-DDTHH, got '$HOUR'" >&2
    exit 2
fi
if [[ ! -d "$OUT" ]]; then
    echo "error: no archive directory at $OUT" >&2
    exit 2
fi

# GNU date first, then BSD.
iso_to_epoch() {
    date -u -d "$1" +%s 2>/dev/null \
        || date -u -j -f "%Y-%m-%dT%H:%M:%SZ" "$1" +%s 2>/dev/null
}
epoch_to_iso() {
    date -u -d "@$1" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
        || date -u -r "$1" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null
}

if ! T0="$(iso_to_epoch "${HOUR}:00:00Z")"; then
    echo "error: --hour '$HOUR' is not a time" >&2
    exit 2
fi
SINCE="$(epoch_to_iso "$T0")"
UNTIL="$(epoch_to_iso $((T0 + 3600 + 1200)))"
BEFORE="$(epoch_to_iso $((T0 + 3600)))"

if [[ -z "$API_TOKEN" && -r "$OUT/.api_token" ]]; then
    API_TOKEN="$(head -n 1 "$OUT/.api_token" | tr -d '[:space:]')"
fi

# The same lock satnogs_pull.sh takes, so this never runs alongside it:
# that script prunes the tally at the end of its run, and an append made
# while it does would be lost.
CLEANUP=()
cleanup() {
    local f
    for f in ${CLEANUP[@]+"${CLEANUP[@]}"}; do rm -rf "$f"; done
}
trap cleanup EXIT
# Bash skips the EXIT trap when a signal kills it, and this script is
# killed in the ordinary run of things: satnogs_passes stops a listing
# with SIGTERM when it quits, and a write to a closed pipe is SIGPIPE.
# Without these the lock would outlive the run -- which on a host
# without flock is a directory that holds the cron job off until
# someone removes it by hand.
trap 'exit 1' HUP INT TERM PIPE

if command -v flock >/dev/null 2>&1; then
    exec 9>"$OUT/.lock"
    if ! flock -n 9; then
        echo "the archive lock is held (the cron run?); not listing $HOUR"
        exit 0
    fi
else
    if ! mkdir "$OUT/.lock.d" 2>/dev/null; then
        echo "the archive lock is held (the cron run?); not listing $HOUR"
        exit 0
    fi
    CLEANUP+=("$OUT/.lock.d")
fi

STATS_FILE="$OUT/.api_stats.txt"
CACHE_DIR="$OUT/.hourcache/$NORAD_ID"
mkdir -p "$CACHE_DIR"
# Beside the cache rather than in the system temp directory, so the
# final move is a rename on one file system.
TMP="$(mktemp "$CACHE_DIR/.${HOUR}.XXXXXX")"
HDR="$(mktemp "$CACHE_DIR/.${HOUR}.hdr.XXXXXX")"
CLEANUP+=("$TMP" "$HDR")

AUTH=()
[[ -n "$API_TOKEN" ]] && AUTH=(-H "Authorization: Token ${API_TOKEN}")

URL="https://network.satnogs.org/api/observations/?norad_cat_id=${NORAD_ID}&start=${SINCE}&end=${UNTIL}"
PAGES=0
while [[ -n "$URL" ]]; do
    echo "GET $URL"
    # Counted as it is made: a request spent is spent, even if this one
    # fails or the run is killed straight after.
    printf '%s 1 list\n' "$(date -u +%s)" >> "$STATS_FILE"
    PAGES=$((PAGES + 1))
    # No --retry: curl retries a 429, which would turn one throttled
    # request into four. The caller's next attempt is the retry.
    if ! JSON="$(curl --silent --show-error --fail -L \
                      --connect-timeout 10 --max-time 60 \
                      -H "User-Agent: ${USER_AGENT}" \
                      -H "Accept: application/json" \
                      ${AUTH[@]+"${AUTH[@]}"} \
                      -D "$HDR" "$URL" 2>&1)"; then
        echo "error: listing $HOUR failed: $JSON" >&2
        exit 1
    fi
    if ! echo "$JSON" | jq -r --arg before "$BEFORE" '
            (if type == "array" then . else (.results // []) end)
            | .[]
            | select((.start // "") < $before)
            | [ (.id | tostring),
                (.start // ""),
                (.end // ""),
                (.status // ""),
                (.waterfall_status // ""),
                (.ground_station // "" | tostring),
                (.station_name // ""),
                (.max_altitude // "" | tostring),
                (.payload // ""),
                (.demoddata | length | tostring) ]
            | @tsv' >> "$TMP"; then
        echo "error: listing $HOUR returned something that is not JSON" >&2
        exit 1
    fi
    # Pagination is a cursor in the Link header: <url>; rel="next".
    URL="$(tr -d '\r' < "$HDR" \
           | sed -n 's/^[Ll]ink:.*<\([^>]*\)>; *rel="\{0,1\}next"\{0,1\}.*/\1/p' \
           | head -n 1)"
    if [[ -n "$URL" && "$RATE_LIMIT_MS" -gt 0 ]]; then
        sleep "$(awk -v ms="$RATE_LIMIT_MS" 'BEGIN { printf "%.3f", ms / 1000 }')"
    fi
done

sort -t "$(printf '\t')" -k2,2 "$TMP" > "$TMP.sorted" && CLEANUP+=("$TMP.sorted")
mv -f "$TMP.sorted" "$CACHE_DIR/$HOUR.tsv"
echo "listed $(wc -l < "$CACHE_DIR/$HOUR.tsv" | tr -d ' ') observations for $HOUR in $PAGES request(s)"
