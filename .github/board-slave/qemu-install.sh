#!/bin/bash
# Installs Espressif's QEMU (with the TWAI model) to ~/qemu unless it is there already.
set -euo pipefail
sudo apt-get install -y --no-install-recommends libsdl2-2.0-0 libslirp0 can-utils >/dev/null
if [ ! -x ~/qemu/bin/qemu-system-xtensa ]; then
    url=$(curl -fsSL https://raw.githubusercontent.com/espressif/esp-idf/master/tools/tools.json | python3 -c "
import json, sys
tools = json.load(sys.stdin)['tools']
t = next(t for t in tools if t['name'] == 'qemu-xtensa')
print(t['versions'][0]['linux-amd64']['url'])")
    echo "QEMU $url"
    mkdir -p ~/qemu-dl && curl -fsSL "$url" | tar xJ -C ~/qemu-dl
    rm -rf ~/qemu && mv ~/qemu-dl/qemu ~/qemu
fi
~/qemu/bin/qemu-system-xtensa --version
