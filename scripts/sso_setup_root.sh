#!/usr/bin/env bash
#
# sso_setup_root.sh — one-time root setup for the multi-operator
# ground machine.
#
# Run as root on the ground computer. Creates the `sso-ops` group,
# the /FrontierSat/ data tree, the /run/sso/ socket directory, and
# /var/log/sso/ for the audit log. Idempotent — safe to re-run after
# upgrades or to fix a botched setup.
#
# After this script runs, add each operator to the sso-ops group:
#   usermod -a -G sso-ops alice
# and ship the example tmpfiles.d / udev / logrotate snippets into
# /etc/ so /run/sso and device permissions survive reboots:
#   install -m 0644 etc/tmpfiles.d/sso.conf /etc/tmpfiles.d/
#   install -m 0644 etc/udev/rules.d/99-sso-hardware.rules /etc/udev/rules.d/
#   install -m 0644 etc/logrotate.d/sso /etc/logrotate.d/
#   systemd-tmpfiles --create /etc/tmpfiles.d/sso.conf
#   udevadm control --reload-rules && udevadm trigger

set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
    echo "$(basename "$0"): must run as root" >&2
    exit 1
fi

# 1. Group.
if ! getent group sso-ops >/dev/null; then
    groupadd --system sso-ops
    echo "created group: sso-ops"
fi

# 2. Shared data root.
install -d -o root -g sso-ops -m 2775 /FrontierSat
install -d -o root -g sso-ops -m 2775 /FrontierSat/TLEs
install -d -o root -g sso-ops -m 2775 /FrontierSat/Operations
install -d -o root -g sso-ops -m 2775 /FrontierSat/satnogs_archive
install -d -o root -g sso-ops -m 2775 /FrontierSat/captures
echo "ensured: /FrontierSat/{TLEs,Operations,satnogs_archive,captures} (2775 root:sso-ops)"

# 3. Runtime socket directory (recreated at boot via tmpfiles.d).
install -d -o root -g sso-ops -m 2770 /run/sso
echo "ensured: /run/sso (2770 root:sso-ops)"

# 4. Audit log directory.
install -d -o root -g sso-ops -m 2770 /var/log/sso
if [[ ! -f /var/log/sso/runs.log ]]; then
    install -o root -g sso-ops -m 0664 /dev/null /var/log/sso/runs.log
fi
echo "ensured: /var/log/sso/runs.log (0664 root:sso-ops)"

echo
echo "Setup complete. To finish:"
echo "  1. usermod -a -G sso-ops <each operator's account>"
echo "  2. Have operators log out + back in so the new group takes effect."
echo "  3. Install the example /etc files (tmpfiles.d, udev, logrotate)."
echo "  4. systemd-tmpfiles --create /etc/tmpfiles.d/sso.conf"
echo "  5. udevadm control --reload-rules && udevadm trigger"
