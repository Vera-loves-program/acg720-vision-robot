#!/bin/sh
# Install/check permission only. This script NEVER invokes poweroff itself.
set -eu
if [ "$(id -un)" != orangepi ]; then
    echo 'Run this script as orangepi (without prefixing the whole script with sudo).' >&2
    exit 1
fi
test -x /usr/bin/systemctl
case "${1:---check}" in
    --check)
        sudo -n -l /usr/bin/systemctl poweroff
        ;;
    --install)
        task_rule_file=$(mktemp)
        trap 'rm -f -- "$task_rule_file"' EXIT HUP INT TERM
        printf '%s\n' 'orangepi ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff' > "$task_rule_file"
        sudo visudo -cf "$task_rule_file"
        sudo install -o root -g root -m 0440 "$task_rule_file" /etc/sudoers.d/acg720-poweroff
        sudo -n -l /usr/bin/systemctl poweroff
        echo 'Installed poweroff-only permission. No shutdown was requested.'
        ;;
    *) echo 'Usage: sh tools/install_orangepi_poweroff.sh --install|--check' >&2; exit 2 ;;
esac
