#!/usr/bin/env bash
#
# decode_regression.sh — pin the FSK + AX100 decode chain against the
# small fixture snippets in test/decode_regression/. Run before and
# after any refactor that touches modem_fsk / modem.c / decode_loop /
# ax100; the decoded payload bytes must hash identically.
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

# Run rx_replay on one fixture and emit a single result line
#   <label> frames=<N> sha256=<first-16-hex-of-sha256>
# We hash the decoded "[t=...]" payload lines + the "N frame(s)" footer.
# rx_replay writes a sidecar .iq.burst.csv next to the input — we copy
# the snippet to a tmpdir first so the repo stays clean.
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
    out=$("$RX_REPLAY" "$path" "${args[@]}" 2>&1 \
        | grep -E '^(\[t=|rx_replay: [0-9]+ frame)' \
        | sort)

    rm -rf "$tmpdir"

    local frames
    frames=$(printf '%s\n' "$out" | grep -oE 'rx_replay: [0-9]+ frame' \
        | grep -oE '[0-9]+' | head -1)
    frames="${frames:-0}"

    local hash
    hash=$(printf '%s\n' "$out" | shasum -a 256 | cut -c1-16)

    printf "%-14s frames=%s sha256=%s\n" "$label" "$frames" "$hash"
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
