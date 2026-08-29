#!/usr/bin/env bash
# Restart the ROS stack when this machine's network address changes.
#
# WHY THIS HAS TO EXIST
#
# DDS binds its interfaces when a node starts. Change the machine's address
# afterwards - move to another Wi-Fi, join a hotspot, plug in a cable - and every
# running node keeps advertising the address it had at start-up. Nothing errors.
# `ros2 node list` comes back empty, subscriber counts drop to zero, and the
# topics are all still there, so everything that is easy to check says the system
# is fine.
#
# It has already happened here for real: the board was linked to the agent and
# publishing telemetry that `ros2 topic echo` could read, while the web UI sat on
# data seven minutes old. The fix is a restart, and there is no reason a person
# should have to know that.
#
# WHY `ip monitor` RATHER THAN POLLING
#
# Address changes arrive as events, so the stack is back within a second or two
# instead of up to a polling interval. It also needs no NetworkManager, so this
# works the same on a machine using systemd-networkd, dhcpcd or anything else.
#
# WHAT IT DELIBERATELY DOES NOT DO
#
# It restarts on a change of ADDRESS, not on every network event. Wi-Fi generates
# a lot of noise - renewals, IPv6 privacy addresses coming and going, link
# flaps - and restarting the stack on each one would be worse than the problem,
# since a restart drops the micro-ROS link and the robot briefly loses its
# heartbeat.
#
#   ./netwatch.sh                    # watch, restart gps_localize on change
#   ./netwatch.sh --dry-run          # report changes without restarting
set -uo pipefail

UNIT="gps_localize"
DEBOUNCE=3
DRY_RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --unit) UNIT="$2"; shift 2 ;;
        --debounce) DEBOUNCE="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) sed -n '2,34p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# The address other machines would reach us on. Not the interface list: a
# machine can hold several addresses at once and only one of them is the route
# out, which is the one DDS will have bound.
primary_ip() {
    ip -4 route get 1.1.1.1 2>/dev/null \
        | awk '{for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit }}'
}

restart_stack() {
    local from="$1" to="$2"
    echo "[netwatch] address changed: ${from:-none} -> ${to:-none}"
    if [ "$DRY_RUN" = 1 ]; then
        echo "[netwatch] --dry-run, not restarting $UNIT"
        return
    fi
    if ! systemctl is-enabled --quiet "$UNIT" 2>/dev/null \
       && ! systemctl is-active --quiet "$UNIT" 2>/dev/null; then
        echo "[netwatch] $UNIT is not installed here, nothing to restart"
        return
    fi
    # Only restart once the address has settled. Joining a network produces a
    # burst of events and the first address is often not the final one.
    sleep "$DEBOUNCE"
    local settled
    settled="$(primary_ip)"
    if [ "$settled" != "$to" ]; then
        echo "[netwatch] still settling (${to} -> ${settled}), waiting for the next event"
        return
    fi
    echo "[netwatch] restarting $UNIT so DDS rebinds to $settled"
    systemctl restart "$UNIT" \
        && echo "[netwatch] $UNIT restarted" \
        || echo "[netwatch] could not restart $UNIT" >&2
}

current="$(primary_ip)"
echo "[netwatch] watching for address changes; currently ${current:-none}"

# `ip monitor` streams for as long as it runs, so this loop only turns over when
# something actually happened on an interface.
ip -4 monitor address 2>/dev/null | while read -r _; do
    latest="$(primary_ip)"
    if [ "$latest" != "$current" ]; then
        was="$current"
        current="$latest"
        restart_stack "$was" "$latest"
        # Re-read: the restart above may have taken longer than another change.
        current="$(primary_ip)"
    fi
done
