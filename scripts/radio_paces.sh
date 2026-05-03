#!/usr/bin/env bash
#
# radio_paces.sh — exercise the FT-991A's CAT command surface via radio_ctl.
#
# Runs a sequence of read/set/verify operations to confirm the radio
# accepts every command we know how to send. By default it does NOT key
# TX; pass --allow-tx (and --allow-high-power if needed) to include a
# brief 1-second key cycle into whatever load is on the V/U connector.
#
# Each test prints PASS / FAIL with the line of radio_ctl output that
# matters; a final summary tells you how many failed.

set -uo pipefail

ALLOW_TX=0
EXTRA_ARGS=()

usage() {
    cat <<EOF >&2
usage: $(basename "$0") [options]

  --allow-tx          Include the live 1 s PTT cycle (UHF, 5%% power)
  --allow-high-power  Permit set-power above 10%% (default 5%%, not used)
  --allow-hf-tx       Permit TX below 100 MHz (default UHF only)
  -h, --help          This message

Without --allow-tx the script does no on-air TX; it only exercises the
control / configuration commands.
EOF
}

for a in "$@"; do
    case "$a" in
        --allow-tx)         ALLOW_TX=1; EXTRA_ARGS+=("$a") ;;
        --allow-high-power) EXTRA_ARGS+=("$a") ;;
        --allow-hf-tx)      EXTRA_ARGS+=("$a") ;;
        -h|--help)          usage; exit 0 ;;
        *) echo "unknown option: $a" >&2; usage; exit 2 ;;
    esac
done

PASS=0
FAIL=0
FAILED=()

step() {
    local label="$1"; shift
    echo
    echo "── $label ──"
    echo "   radio_ctl $*"
    if radio_ctl "$@"; then
        PASS=$((PASS + 1))
    else
        FAIL=$((FAIL + 1))
        FAILED+=("$label")
    fi
}

# --- 1. Connectivity --------------------------------------------------------

step "Identify radio"               identify

# --- 2. Frequency control ---------------------------------------------------

step "Read current frequency"       get-freq
step "Tune 2 m (145.500 MHz)"       set-freq 145500000 --verify
step "Tune 70 cm (436.150 MHz)"     set-freq 436150000 --verify

# --- 3. Operating mode ------------------------------------------------------

step "Mode FM"                      set-mode fm
step "Mode USB"                     set-mode usb
step "Mode LSB"                     set-mode lsb
step "Mode AM"                      set-mode am
step "Mode CW"                      set-mode cw
step "Mode FM (restore)"            set-mode fm

# --- 4. DATA mode flag ------------------------------------------------------

step "DATA mode ON (DATA-FM)"       set-data-mode on
step "DATA mode OFF (FM)"           set-data-mode off

# --- 5. DATA modulator routing ----------------------------------------------

step "DATA MOD source USB"          set-data-mod-source usb
step "DATA MOD source ACC"          set-data-mod-source acc
step "DATA MOD source USB"          set-data-mod-source usb

# --- 6. RF power ------------------------------------------------------------

step "RF power 1%"                  set-power 1
step "RF power 5%"                  set-power 5

# --- 7. Convenience: full uplink-prep ---------------------------------------

step "Uplink prep (FM-DATA, USB)"   uplink-prep

# --- 8. Optional: live TX cycle into the V/U load ---------------------------

if [ "$ALLOW_TX" = 1 ]; then
    echo
    echo "== Live PTT cycle (1 s, 70 cm, 5% power) =="
    step "Tune 70 cm"               set-freq 436150000
    step "Mode FM"                  set-mode fm
    step "RF power 5%"              set-power 5
    step "PTT ON"                   "${EXTRA_ARGS[@]}" ptt on
    echo "   ... 1 second on the air ..."
    sleep 1
    step "PTT OFF"                  ptt off
fi

echo
echo "================================================================"
echo "  Total: $((PASS + FAIL))   Pass: $PASS   Fail: $FAIL"
if [ "$FAIL" -gt 0 ]; then
    printf '  Failed: %s\n' "${FAILED[*]}"
    exit 1
fi
