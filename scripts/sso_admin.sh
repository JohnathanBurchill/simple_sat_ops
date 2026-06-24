#!/usr/bin/env bash
#
# sso_admin.sh — guided operator provisioning for the FrontierSat
# ground machine.
#
# Three subcommands, all idempotent:
#
#   setup                       One-time root setup: install /etc/ files
#                                from /usr/local/share/sso/etc/, reload
#                                systemd-tmpfiles + udev.
#   add-operator [<name>]       Create a Unix account in sso-ops, drop
#                                an SSH public key into authorized_keys.
#   verify [<name>]             Smoke-test the account. As root with a
#                                username, runs checks via `su -`; as
#                                the operator themselves, runs as self.
#
# See scripts/sso_setup_root.sh for the underlying bootstrap (group +
# /FrontierSat + /run/sso + /var/log/sso) that `setup` ensures has run.

set -euo pipefail

# ---------- Constants ----------------------------------------------------

ETC_SRC="/usr/local/share/sso/etc"
ETC_FILES=(
    "tmpfiles.d/sso.conf"
    "udev/rules.d/99-sso-hardware.rules"
    "logrotate.d/sso"
)
ADMIN_LOG="/var/log/sso/admin.log"
RUNS_LOG="/var/log/sso/runs.log"
SETUP_BOOTSTRAP="/usr/local/bin/sso_setup_root.sh"

YES=0
DRY_RUN=0
KEY_FILE=""
KEY_TEXT=""
UPDATE_MODE=0
GLOBAL_REST=()

# ---------- Helpers ------------------------------------------------------

_color() {
    if [[ -t 2 ]]; then
        printf '\033[%sm%s\033[0m' "$1" "$2"
    else
        printf '%s' "$2"
    fi
}

log()  { printf '%s %s\n' "$(_color '1;34' '==>')" "$*" >&2; }
warn() { printf '%s %s\n' "$(_color '1;33' '!!')"  "$*" >&2; }
err()  { printf '%s %s\n' "$(_color '1;31' 'xx')"  "$*" >&2; exit 1; }

ask() {
    # Prompt to stderr, return user input on stdout.
    local prompt="$1"
    local v
    read -r -p "$prompt " v
    printf '%s' "$v"
}

confirm() {
    local prompt="$1"
    if (( YES )); then return 0; fi
    local v
    read -r -p "$prompt [y/N] " v
    [[ "${v,,}" == y* ]]
}

need_root() {
    if (( EUID != 0 )); then
        err "$1: must run as root (try: sudo $0 $1 ...)"
    fi
}

# Append one tab-separated event record to /var/log/sso/admin.log.
# Best-effort — if the file isn't writable to us, swallow the error.
audit() {
    local event="$1"
    local detail="${2:-}"
    local ts who
    ts="$(date -u +%FT%TZ)"
    who="${SUDO_USER:-${USER:-?}}"
    {
        printf '%s\t%s\tsso_admin\t%d\t%s\t%s\n' \
            "$ts" "$who" "$$" "$event" "$detail" >> "$ADMIN_LOG"
    } 2>/dev/null || true
}

# Wrap state-changing commands so --dry-run is honoured.
run() {
    if (( DRY_RUN )); then
        printf '%s %s\n' "$(_color '0;36' 'would run:')" "$*" >&2
        return 0
    fi
    "$@"
}

usage() {
    cat <<'EOF' >&2
usage: sso_admin.sh <subcommand> [options]

Subcommands:
  setup                          One-time root setup. Installs the three
                                  sysadmin examples into /etc/, reloads
                                  tmpfiles + udev.
  add-operator [<name>]          Create a Unix account in sso-ops with an
                                  SSH key. Prompts for missing values.
  verify [<name>]                Smoke-test the account. As root with a
                                  name, runs checks via 'su -'; as the
                                  operator, runs checks as self.

Global options:
  -y, --yes                      Don't prompt; assume yes for confirms.
      --dry-run                  Print actions without running (setup only).
  -h, --help                     This message.

add-operator options:
      --key-file <path>          Read SSH public key from a file.
      --key-text "<line>"        Use SSH public key as given on the CLI.
      --update                   Operate on an existing user (skip useradd).

Examples:
  sudo sso_admin.sh setup
  sudo sso_admin.sh add-operator alice --key-file /tmp/alice.pub
  sudo sso_admin.sh verify alice
  sso_admin.sh verify                 # operator self-check, no sudo
EOF
}

# ---------- setup --------------------------------------------------------

cmd_setup() {
    need_root setup
    audit start setup ""

    # 1. sso-ops group
    if ! getent group sso-ops >/dev/null 2>&1; then
        warn "sso-ops group not found."
        if [[ -x "$SETUP_BOOTSTRAP" ]]; then
            if confirm "Run $SETUP_BOOTSTRAP to bootstrap?"; then
                run "$SETUP_BOOTSTRAP"
            else
                err "aborted: sso-ops group is required"
            fi
        else
            err "missing bootstrap script ($SETUP_BOOTSTRAP); 'sudo make install' first"
        fi
    else
        log "sso-ops group present."
    fi

    # 2. etc/ files
    for rel in "${ETC_FILES[@]}"; do
        local src="$ETC_SRC/$rel"
        local dst="/etc/$rel"
        if [[ ! -f "$src" ]]; then
            warn "missing source: $src — was 'sudo make install' run from the multi-operator branch?"
            continue
        fi
        if [[ -f "$dst" ]] && cmp -s "$src" "$dst"; then
            log "/etc/$rel already current"
            continue
        fi
        if [[ -f "$dst" ]]; then
            warn "/etc/$rel differs from shipped example:"
            diff -u "$dst" "$src" >&2 || true
            if ! confirm "Overwrite /etc/$rel?"; then
                log "skipped /etc/$rel"
                continue
            fi
        fi
        run install -D -m 0644 "$src" "$dst"
        audit etc-installed "$rel"
        log "installed /etc/$rel"
    done

    # 3. Reload tmpfiles + udev
    if command -v systemd-tmpfiles >/dev/null 2>&1; then
        run systemd-tmpfiles --create /etc/tmpfiles.d/sso.conf
        audit tmpfiles-reloaded ""
        log "reloaded tmpfiles.d"
    else
        warn "systemd-tmpfiles not found — skipped tmpfiles reload"
    fi
    if command -v udevadm >/dev/null 2>&1; then
        run udevadm control --reload-rules
        run udevadm trigger
        audit udev-reloaded ""
        log "reloaded udev rules — unplug+replug devices so the new group ownership applies"
    else
        warn "udevadm not found — skipped udev reload"
    fi

    # 4. Sanity-check key directories
    for d in /run/sso /var/log/sso /FrontierSat; do
        if [[ ! -d "$d" ]]; then
            warn "$d does not exist — re-run $SETUP_BOOTSTRAP"
        fi
    done

    audit end setup ""
    log "setup ok"
}

# ---------- add-operator -------------------------------------------------

validate_name() {
    local name="$1"
    [[ -n "$name" ]] || err "username required"
    [[ "$name" =~ ^[a-z_][a-z0-9_-]{0,31}$ ]] \
        || err "invalid username: '$name' (must be POSIX-portable, ≤32 chars)"
    if getent passwd "$name" >/dev/null 2>&1; then
        local uid
        uid="$(id -u "$name")"
        if (( uid < 1000 )); then
            err "'$name' is a system account (uid $uid) — refusing"
        fi
    fi
}

# Read the operator's public key from --key-text, --key-file, or stdin
# (interactive paste). Validates with ssh-keygen -lf -. Returns the key
# on stdout (single line, no trailing newline).
read_key() {
    local key=""
    if [[ -n "$KEY_TEXT" ]]; then
        key="$KEY_TEXT"
    elif [[ -n "$KEY_FILE" ]]; then
        [[ -r "$KEY_FILE" ]] || err "key file unreadable: $KEY_FILE"
        key="$(head -n 1 "$KEY_FILE")"
    else
        log "Paste the operator's public key (one line), then Enter:"
        IFS= read -r key
    fi
    [[ -n "$key" ]] || err "no key provided"
    if ! printf '%s\n' "$key" | ssh-keygen -lf - >/dev/null 2>&1; then
        err "key did not parse — paste the .pub file contents verbatim"
    fi
    printf '%s' "$key"
}

cmd_add_operator() {
    need_root add-operator
    local name="${1:-}"
    if [[ -z "$name" ]]; then
        name="$(ask 'Operator username:')"
    fi
    validate_name "$name"

    local key
    key="$(read_key)"

    local user_exists=0
    if getent passwd "$name" >/dev/null 2>&1; then
        user_exists=1
        if (( ! UPDATE_MODE )); then
            confirm "User '$name' already exists; append key + ensure sso-ops membership?" \
                || err "aborted"
        fi
    fi

    audit start add-operator "user=$name update=$user_exists"

    if (( ! user_exists )); then
        run useradd -m -s /bin/bash -U "$name"
        audit user-created "$name"
        if (( ! DRY_RUN )); then
            log "created user $name (uid $(id -u "$name"))"
        else
            log "would create user $name"
        fi
    fi

    # Add to sso-ops if not already a member.
    if (( DRY_RUN )) || ! id -nG "$name" 2>/dev/null | tr ' ' '\n' | grep -qx sso-ops; then
        run gpasswd -a "$name" sso-ops >/dev/null
        audit group-added "$name->sso-ops"
        log "added $name to sso-ops"
    else
        log "$name already in sso-ops"
    fi

    # Resolve $HOME for the target user. In --dry-run mode useradd
    # didn't actually run, so we have to assume the conventional path.
    local home
    if getent passwd "$name" >/dev/null 2>&1; then
        home="$(getent passwd "$name" | awk -F: '{print $6}')"
    else
        home="/home/$name"
    fi
    local ssh_dir="$home/.ssh"
    local auth="$ssh_dir/authorized_keys"

    run install -d -o "$name" -g "$name" -m 700 "$ssh_dir"

    if (( ! DRY_RUN )) && [[ -f "$auth" ]] && grep -qxF "$key" "$auth"; then
        log "key already authorized — nothing to append"
    else
        if (( DRY_RUN )); then
            log "would append key to $auth"
        else
            # umask 077 so that if this append CREATES authorized_keys it is
            # born 600, not world-readable at the default umask until the
            # chmod below lands (loose perms also make sshd ignore the file).
            ( umask 077; printf '%s\n' "$key" >> "$auth" )
            chown "$name:$name" "$auth"
            chmod 600 "$auth"
        fi
        audit key-appended "$name"
        log "appended SSH key to $auth"
    fi

    audit end add-operator "user=$name"

    cat >&2 <<EOF

next steps
  1. $name SSHes in fresh — supplementary group membership only takes
     effect at a new login.
  2. Verify the account:
       sudo sso_admin.sh verify $name
     or, from $name's own shell:
       sso_admin.sh verify
EOF
}

# ---------- verify -------------------------------------------------------

# Self-test that runs as the calling user. When invoked via 'sudo
# sso_admin.sh verify <name>', the parent shells `su - <name>` and
# re-enters this function inside that subshell.
run_checks_as_self() {
    local fails=0
    local d tool probe_path probe_line ts

    # 1. id contains sso-ops
    if id -nG | tr ' ' '\n' | grep -qx sso-ops; then
        echo "PASS id includes sso-ops"
    else
        echo "FAIL id does not include sso-ops (try logging out and back in)"
        fails=$(( fails + 1 ))
    fi

    # 2. Stat-able shared directories
    for d in /FrontierSat /run/sso /var/log/sso; do
        if [[ -d "$d" ]]; then
            echo "PASS $d exists"
        else
            echo "FAIL $d missing — run 'sudo sso_admin.sh setup'"
            fails=$(( fails + 1 ))
        fi
    done

    # 3. Write-probe in /FrontierSat/TLEs
    probe_path="/FrontierSat/TLEs/.write-probe-$$"
    if ( umask 002; : > "$probe_path" ) 2>/dev/null; then
        rm -f "$probe_path"
        echo "PASS /FrontierSat/TLEs writable to $USER"
    else
        echo "FAIL /FrontierSat/TLEs not writable to $USER (check setgid + sso-ops)"
        fails=$(( fails + 1 ))
    fi

    # 4. Tools on PATH
    for tool in simple_sat_ops tx_frame_sdr; do
        if command -v "$tool" >/dev/null 2>&1; then
            echo "PASS $tool on PATH ($(command -v "$tool"))"
        else
            echo "FAIL $tool not on PATH"
            fails=$(( fails + 1 ))
        fi
    done

    # 5. Audit log write probe. Curly-group + 2>/dev/null so a missing
    # /var/log/sso/ at probe time doesn't leak its ENOENT to the
    # terminal — bash sets up redirects before the inner command runs,
    # and the group-level 2>/dev/null covers that setup too.
    ts="$(date -u +%FT%TZ)"
    probe_line="$(printf '%s\t%s\tsso_admin\t%d\tverify-probe\t' "$ts" "$USER" "$$")"
    if { printf '%s\n' "$probe_line" >> /var/log/sso/runs.log; } 2>/dev/null; then
        echo "PASS /var/log/sso/runs.log writable"
    else
        local fallback="$HOME/.local/share/simple_sat_ops/runs.log"
        mkdir -p "$(dirname "$fallback")" 2>/dev/null || true
        if { printf '%s\n' "$probe_line" >> "$fallback"; } 2>/dev/null; then
            echo "PASS audit-log fallback ($fallback) writable"
        else
            echo "FAIL audit log not writable in either location"
            fails=$(( fails + 1 ))
        fi
    fi

    echo
    if (( fails == 0 )); then
        echo "verify ok ($USER)"
        return 0
    fi
    echo "verify failed: $fails check(s)"
    return 1
}

cmd_verify() {
    local name="${1:-}"
    if [[ -z "$name" ]] || [[ "$name" == "$USER" ]]; then
        run_checks_as_self
        return $?
    fi
    need_root verify
    if ! getent passwd "$name" >/dev/null 2>&1; then
        err "user '$name' does not exist"
    fi
    if (( ! YES )); then
        confirm "About to 'su - $name' and run checks. Continue?" || err "aborted"
    fi
    audit start verify "user=$name"
    local rc=0
    # Forward the checks into the target user's login shell. _color is
    # redefined to plain-text so output stays clean if the su pty isn't
    # a tty.
    #
    # SECURITY: this -c body runs in the target user's (often root's) shell.
    # It MUST stay free of shell-variable interpolation -- never splice $name,
    # $key, or any caller-controlled value into this string, or it becomes a
    # privilege-escalation injection. It is variable-free today; if a check
    # ever needs data, pass it via the environment or stdin, not by expanding
    # it into this command.
    su - "$name" -c "$(declare -f run_checks_as_self)
_color() { printf '%s' \"\$2\"; }
run_checks_as_self" || rc=$?
    audit end verify "user=$name rc=$rc"
    return $rc
}

# ---------- Argument dispatch -------------------------------------------

# Strip global flags out of a flat arg list. Anything left lands in
# GLOBAL_REST for the subcommand to consume.
parse_global_flags() {
    local -a rest=()
    while (( $# )); do
        case "$1" in
            -y|--yes)   YES=1; shift ;;
            --dry-run)  DRY_RUN=1; shift ;;
            -h|--help)  usage; exit 0 ;;
            --)         shift; rest+=("$@"); break ;;
            *)          rest+=("$1"); shift ;;
        esac
    done
    GLOBAL_REST=("${rest[@]+"${rest[@]}"}")
}

parse_add_operator_flags() {
    local -a rest=()
    while (( $# )); do
        case "$1" in
            --key-file)   KEY_FILE="$2"; shift 2 ;;
            --key-text)   KEY_TEXT="$2"; shift 2 ;;
            --update)     UPDATE_MODE=1; shift ;;
            -y|--yes)     YES=1; shift ;;
            --dry-run)    DRY_RUN=1; shift ;;
            -h|--help)    usage; exit 0 ;;
            --)           shift; rest+=("$@"); break ;;
            *)            rest+=("$1"); shift ;;
        esac
    done
    GLOBAL_REST=("${rest[@]+"${rest[@]}"}")
}

main() {
    if (( $# < 1 )); then
        usage
        exit 2
    fi
    local sub="$1"
    shift
    case "$sub" in
        setup)
            parse_global_flags "$@"
            cmd_setup
            ;;
        add-operator)
            parse_add_operator_flags "$@"
            cmd_add_operator "${GLOBAL_REST[@]+"${GLOBAL_REST[@]}"}"
            ;;
        verify)
            parse_global_flags "$@"
            cmd_verify "${GLOBAL_REST[@]+"${GLOBAL_REST[@]}"}"
            ;;
        -h|--help)
            usage
            ;;
        *)
            err "unknown subcommand: $sub (try -h)"
            ;;
    esac
}

main "$@"
