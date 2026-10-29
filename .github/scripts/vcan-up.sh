#!/bin/bash
# Brings up the vcan link $1 (default vcan0) for the CANopen tests.
set -euo pipefail
iface="${1:-vcan0}"
if ! sudo modprobe vcan 2>/dev/null; then
    pkg="linux-modules-extra-$(uname -r)"
    echo "vcan module not loaded; installing ${pkg}"
    sudo apt-get update -q
    if ! sudo apt-get install -y --no-install-recommends "${pkg}"; then
        echo "::error::kernel $(uname -r) has no vcan module and no ${pkg} package"
        exit 1
    fi
    sudo modprobe vcan
fi
sudo ip link add dev "${iface}" type vcan
sudo ip link set "${iface}" up
ip -details link show "${iface}"
