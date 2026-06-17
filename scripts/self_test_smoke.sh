#!/usr/bin/env bash
#
# self_test_smoke.sh — run `simple_sat_ops --self-test` across the mode
# matrix and confirm the dry run comes up clean in each. Guards the
# parse -> TLE-load path that a segfault slipped through once: --control
# auto-reads the satellite name from the TLE, but the bare --self-test and
# --viewer-stream modes took a different branch that left the name unset
# and fed NULL into the TLE search. The daily `--self-test --control ...`
# run never exercised those branches, so this covers the modes that don't
# get hit by hand.
#
# Run after any change to apps/cli_args.c, the TLE/name resolution, or the
# --self-test path. Each case must exit 0 with "self-test: ok" (or, for the
# bad-TLE case, exit non-zero WITHOUT a crash). A crash (signal) is always
# a failure, never a pass.
#
# Usage:
#   scripts/self_test_smoke.sh                 use ./build/simple_sat_ops
#   scripts/self_test_smoke.sh <path-to-bin>   use a specific binary
#
# Exits 0 if every case passes, 1 otherwise.

set -uo pipefail
export LC_ALL=C

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SSO="${1:-$REPO_ROOT/build/simple_sat_ops}"
TLE="$REPO_ROOT/unit_tests/fixtures/selftest.tle"
MISSING="$REPO_ROOT/unit_tests/fixtures/does-not-exist.tle"

if [[ ! -x "$SSO" ]]; then
    echo "self_test_smoke: $SSO not built — run 'make -C build simple_sat_ops' first" >&2
    exit 1
fi
if [[ ! -f "$TLE" ]]; then
    echo "self_test_smoke: fixture $TLE missing" >&2
    exit 1
fi

fails=0

# A fatal-signal kill shows up in $? as 128+signum. Match only the signals
# that mean "crash" (SIGILL/TRAP/ABRT/FPE/BUS/SEGV) — a plain exit(255), as
# a -1 return becomes, also lands above 128 but is a clean error, not a crash.
is_crash() {
    case "$1" in
        132|133|134|136|138|139) return 0 ;;
        *) return 1 ;;
    esac
}

# run_case <label> <expect: ok|error> <args...>
# ok    -> exit 0 and output contains "self-test: ok"
# error -> exit non-zero, but NOT a crash
# A crash (fatal signal) fails either way.
run_case() {
    local label="$1" expect="$2"; shift 2
    local out rc
    out="$("$SSO" "$@" 2>&1)"; rc=$?

    if is_crash "$rc"; then
        echo "FAIL  $label: crashed (exit $rc, signal $((rc - 128))) — args: $*"
        fails=$((fails + 1))
        return
    fi

    case "$expect" in
        ok)
            if (( rc == 0 )) && grep -q "self-test: ok" <<<"$out"; then
                echo "ok    $label"
            else
                echo "FAIL  $label: expected clean 'self-test: ok' (exit $rc) — args: $*"
                fails=$((fails + 1))
            fi
            ;;
        error)
            if (( rc != 0 )); then
                echo "ok    $label (clean error, exit $rc)"
            else
                echo "FAIL  $label: expected a non-zero exit — args: $*"
                fails=$((fails + 1))
            fi
            ;;
    esac
}

# The modes the daily `--self-test --control ...` run does NOT cover: bare
# self-test (no --control) and --viewer-stream both auto-read the name.
run_case "self-test, no name (auto-read)"      ok    --self-test --tle "$TLE"
run_case "self-test, explicit name"            ok    --self-test --tle "$TLE" "OSCAR 7 (AO-7)"
run_case "viewer-stream self-test (auto-read)" ok    --viewer-stream --self-test --tle "$TLE"
# A missing TLE must fail cleanly, not segfault.
run_case "self-test, missing TLE"              error --self-test --tle "$MISSING"

echo
if (( fails == 0 )); then
    echo "self_test_smoke: all cases passed"
    exit 0
fi
echo "self_test_smoke: $fails case(s) failed"
exit 1
