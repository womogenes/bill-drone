#!/bin/sh
# Cold-restart a board by switching its laptop USB port off for a few seconds, which a reset cannot replace (a camera sensor that stops answering its probe needs its power removed): tools/usb_power_cycle.sh /dev/ttyUSB1 [seconds]. It restarts the drone and its camera while they run from USB alone; a connected battery keeps them powered. Needs tools/usb-power.rules installed once.
set -e
tty=$(basename "$1")
switch=$(readlink -f "/sys/class/tty/$tty/device/../../port")/disable
[ -w "$switch" ] || { echo "$switch is not writable: install tools/usb-power.rules" >&2; exit 1; }
echo 1 > "$switch"
sleep "${2:-5}"
echo 0 > "$switch"
echo "power restored to $(dirname "$switch")"
