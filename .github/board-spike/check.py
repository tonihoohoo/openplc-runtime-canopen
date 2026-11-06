# SPDX-License-Identifier: Apache-2.0
"""Spike check: boot-up, heartbeat period and RPDO echo from the board in QEMU on vcan."""

import socket
import struct
import sys
import time

FRAME = struct.Struct("=IB3x8s")


def main() -> int:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((sys.argv[1],))
    s.settimeout(0.2)
    beats: list[float] = []
    boot = False
    echoes = 0
    sent = 0
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline and len(beats) < 60:
        try:
            cob, dlc, data = FRAME.unpack(s.recv(16))
        except TimeoutError:
            continue
        if cob == 0x705 and dlc == 1 and data[0] == 0:
            boot = True
        elif cob == 0x705 and dlc == 1:
            beats.append(time.monotonic())
            if len(beats) > 5:
                payload = struct.pack("<I", sent)
                s.send(FRAME.pack(0x205, 4, payload.ljust(8, b"\0")))
                sent += 1
        elif cob == 0x185 and dlc == 4:
            echoes += 1
    gaps = [b - a for a, b in zip(beats, beats[1:])]
    print(f"boot-up seen: {boot}, heartbeats: {len(beats)}, echoes: {echoes}/{sent}")
    if gaps:
        print(f"heartbeat gap ms: min {min(gaps)*1000:.1f} max {max(gaps)*1000:.1f}")
    return 0 if len(beats) >= 20 and echoes >= sent - 1 and sent > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
