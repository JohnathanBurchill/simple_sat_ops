#!/usr/bin/env bash
#
# decode_regression.sh — smoketest the FSK + AX100 decode chain against
# the small known-good fixture snippets in test/decode_regression/. Run
# before and after any refactor that touches modem_fsk / modem.c /
# decode_loop / ax100; each recording must still decode to the same
# recognized-frame counts.
#
# We pin the integer counts from rx_replay's "decode summary" block
# (frames detected, valid CSP headers, Reed-Solomon corrected/uncorrectable,
# and the recognized-by-type breakdown) rather than a hash of the raw
# demodulated bytes. At the low SNR of these clips the beacon payloads are
# Reed-Solomon-uncorrectable, so the exact bytes wobble with sub-bit
# floating-point differences between machines; the counts do not.
#
# Usage:
#   scripts/decode_regression.sh                   compare against expected
#   scripts/decode_regression.sh --update          write current results
#                                                  back into the expected
#                                                  file (use after a known-
#                                                  good baseline run).
#
# Exits 0 on match, 1 on mismatch / missing fixture.

set -euo pipefail
export LC_ALL=C

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXPECTED="$REPO_ROOT/scripts/decode_regression.expected"
RX_REPLAY="$REPO_ROOT/build/rx_replay"
FIXTURES_DIR="$REPO_ROOT/test/decode_regression"
MODE="${1:-check}"

if [[ ! -x "$RX_REPLAY" ]]; then
    echo "decode_regression: $RX_REPLAY not built — run 'make -C build' first" >&2
    exit 1
fi

# label | type | filename inside test/decode_regression
# Type chooses the rx_replay command line:
#   iq   = headerless I,Q pairs (int16 interleaved) at 96 kHz
#   wav  = mono 48 kHz WAV (FM-audio path, what SatNOGS dumps look like)
FIXTURES=(
    "iq_localpass | iq  | iq_snippet.iq"
    "wav_satnogs  | wav | audio_snippet.wav"
)

# Run rx_replay on one fixture and emit a single result line of the
# stable counts from its "decode summary" block:
#   <label> detected=<N> csp_ok=<N> rs=<corr>/<uncorr> beacon=<N> tcmd=<N> log=<N> bulk=<N>
# A "?" in any field means the summary line was missing — rx_replay either
# crashed or changed its output format, both of which should fail the test.
# rx_replay writes a sidecar .iq.burst.csv next to the input, so we copy the
# snippet to a tmpdir first to keep the repo clean.
run_one() {
    local label="$1" kind="$2" filename="$3"
    local src="$FIXTURES_DIR/$filename"

    if [[ ! -f "$src" ]]; then
        printf "%-14s MISSING_FIXTURE (%s)\n" "$label" "$src"
        return
    fi

    local tmpdir
    tmpdir="$(mktemp -d)"
    cp "$src" "$tmpdir/$filename"
    local path="$tmpdir/$filename"

    local args=("--no-db")
    case "$kind" in
        iq)  args+=("--rate=96000") ;;
        wav) args+=("--channels=1") ;;
        *)   printf "%-14s UNKNOWN_KIND_%s\n" "$label" "$kind"
             rm -rf "$tmpdir"; return ;;
    esac

    local out
    out="$("$RX_REPLAY" "$path" "${args[@]}" 2>&1)" || true

    rm -rf "$tmpdir"

    # Pull the stable integer counts out of rx_replay's decode summary.
    # No pipes into head (SIGPIPE + pipefail would abort under set -e):
    # each pattern matches one summary line; trim to the first match.
    local detected csp_ok rs beacon tcmd logc bulk
    detected=$(sed -n 's/.*detected (after position dedup): *\([0-9]*\).*/\1/p' <<<"$out")
    csp_ok=$(sed -n 's/.*valid CSP header *: *\([0-9]*\).*/\1/p' <<<"$out")
    rs=$(sed -n 's@.*RS corrected / uncorrectable *: *\([0-9]*\) / \([0-9]*\).*@\1/\2@p' <<<"$out")
    detected=${detected%%$'\n'*}; csp_ok=${csp_ok%%$'\n'*}; rs=${rs%%$'\n'*}
    read -r beacon tcmd logc bulk < <(sed -n \
        's/.*recognized by type *: *beacon \([0-9]*\), tcmd_response \([0-9]*\), log \([0-9]*\), bulk_file \([0-9]*\).*/\1 \2 \3 \4/p' \
        <<<"$out") || true

    printf "%-14s detected=%s csp_ok=%s rs=%s beacon=%s tcmd=%s log=%s bulk=%s\n" \
        "$label" "${detected:-?}" "${csp_ok:-?}" "${rs:-?}" \
        "${beacon:-?}" "${tcmd:-?}" "${logc:-?}" "${bulk:-?}"
}

CURRENT="$(mktemp)"
trap 'rm -f "$CURRENT"' EXIT
for f in "${FIXTURES[@]}"; do
    # Strip whitespace around each "|" separator
    label="${f%%|*}";    rest="${f#*|}"
    kind="${rest%%|*}";  filename="${rest#*|}"
    label="${label// /}"
    kind="${kind// /}"
    filename="${filename// /}"
    run_one "$label" "$kind" "$filename"
done > "$CURRENT"

echo "--- current ---"
cat "$CURRENT"

if [[ "$MODE" == "--update" ]]; then
    cp "$CURRENT" "$EXPECTED"
    echo
    echo "decode_regression: wrote new baseline to $EXPECTED"
    exit 0
fi

if [[ ! -f "$EXPECTED" ]]; then
    echo
    echo "decode_regression: no expected file at $EXPECTED — run with --update to create it." >&2
    exit 1
fi

echo "--- expected ---"
cat "$EXPECTED"
echo

if diff -u "$EXPECTED" "$CURRENT"; then
    echo "decode_regression: MATCH"
    exit 0
else
    echo "decode_regression: MISMATCH" >&2
    exit 1
fi
