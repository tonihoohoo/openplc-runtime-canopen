#!/bin/bash
# Creates a simulated GPIO chip (gpio-sim) with the lines lio-in, lio-out and lio-on for the
# linux_io blocks test, and writes ready=true and sysfs=<the chip's sysfs directory> to
# $GITHUB_OUTPUT. A kernel without gpio-sim gives ready=false and a notice, not a failure.
set -uo pipefail
out="${GITHUB_OUTPUT:-/dev/stdout}"
skip() {
    echo "::notice::GPIO simulator not available ($1); the Linux I/O blocks test is skipped"
    echo "ready=false" >> "$out"
    exit 0
}
if ! sudo modprobe gpio-sim 2>/dev/null; then
    pkg="linux-modules-extra-$(uname -r)"
    sudo apt-get install -y --no-install-recommends "${pkg}" >/dev/null 2>&1 || true
    sudo modprobe gpio-sim 2>/dev/null || skip "no gpio-sim module for kernel $(uname -r)"
fi
cfg=/sys/kernel/config
if [ ! -d "$cfg/gpio-sim" ]; then
    sudo mount -t configfs none "$cfg" 2>/dev/null || true
fi
[ -d "$cfg/gpio-sim" ] || skip "no gpio-sim directory in configfs"
chip="$cfg/gpio-sim/lio"
sudo mkdir -p "$chip/gpio-bank0" || skip "cannot create a simulated chip"
echo 4 | sudo tee "$chip/gpio-bank0/num_lines" >/dev/null
i=0
for name in lio-in lio-out lio-on; do
    sudo mkdir -p "$chip/gpio-bank0/line$i"
    echo "$name" | sudo tee "$chip/gpio-bank0/line$i/name" >/dev/null
    i=$((i + 1))
done
echo 1 | sudo tee "$chip/live" >/dev/null || skip "the simulated chip did not go live"
dev=$(cat "$chip/dev_name")
name=$(cat "$chip/gpio-bank0/chip_name")
sysfs="/sys/devices/platform/$dev/$name"
[ -d "$sysfs/sim_gpio0" ] || skip "no sysfs directory $sysfs/sim_gpio0"
ls -l "/dev/$name"
echo "ready=true" >> "$out"
echo "sysfs=$sysfs" >> "$out"
