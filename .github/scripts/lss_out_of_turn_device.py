# SPDX-License-Identifier: Apache-2.0
"""An LSS device (CiA 305) on raw SocketCAN that answers a fastscan out of turn.

It has no node ID and takes part in an LSS fastscan, but once found (in the configuration state) it
keeps answering every fastscan, as some devices do. It answers switch state selective, inquire node
ID and configure node ID; a node ID given takes effect when the device is switched to waiting, after
which it takes no part in a search. Used to check that the master's search ends as incomplete with
the devices found so far.

    lss_out_of_turn_device.py --interface vcan0 --serial 0x1234 [--seconds 300]
"""

import argparse
import socket
import struct
import time

FRAME = struct.Struct("=IB3x8s")
MASTER = 0x7E5
DEVICE = 0x7E4


class Device:
    def __init__(self, address: tuple[int, int, int, int]) -> None:
        self.address = address
        self.node_id = 255
        self.pending = 255
        self.config = False
        self.lss_sub = 0
        self.selected = [0, 0, 0, 0]

    def answer(self, data: bytes) -> bytes | None:
        cs = data[0]
        value = int.from_bytes(data[1:5], "little")
        if cs == 0x04:
            self.config = data[1] == 1
            if not self.config and self.pending != 255:
                self.node_id = self.pending
            return None
        if 0x40 <= cs <= 0x43:
            self.selected[cs - 0x40] = value
            if cs == 0x43 and tuple(self.selected) == self.address:
                self.config = True
                return bytes([0x44])
            return None
        if cs == 0x5E:
            return bytes([0x5E, self.node_id]) if self.config else None
        if cs == 0x11:
            if not self.config:
                return None
            ok = 1 <= data[1] <= 127 or data[1] == 255
            if ok:
                self.pending = data[1]
            return bytes([0x11, 0 if ok else 1])
        if cs == 0x51:
            if self.config:
                return bytes([0x4F])
            if self.node_id != 255:
                return None
            bit, sub, nxt = data[5], data[6], data[7]
            if bit == 0x80:
                self.lss_sub = 0
                return bytes([0x4F])
            if sub != self.lss_sub or (value ^ self.address[sub]) >> bit:
                return None
            if bit == 0:
                self.lss_sub = nxt
                if nxt < sub:
                    self.config = True
            return bytes([0x4F])
        return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--serial", type=lambda s: int(s, 0), required=True)
    parser.add_argument("--vendor", type=lambda s: int(s, 0), default=1)
    parser.add_argument("--product", type=lambda s: int(s, 0), default=2)
    parser.add_argument("--revision", type=lambda s: int(s, 0), default=3)
    parser.add_argument("--seconds", type=float, default=300)
    args = parser.parse_args()
    device = Device((args.vendor, args.product, args.revision, args.serial))
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack("=II", MASTER, 0x7FF))
    sock.bind((args.interface,))
    sock.settimeout(0.5)
    end = time.monotonic() + args.seconds
    print(f"LSS device {args.serial:#x} answering out of turn on {args.interface}", flush=True)
    while time.monotonic() < end:
        try:
            raw = sock.recv(FRAME.size)
        except TimeoutError:
            continue
        can_id, length, data = FRAME.unpack(raw)
        if can_id & 0x7FF != MASTER or length < 1:
            continue
        reply = device.answer(data[:length].ljust(8, b"\0"))
        if reply is not None:
            sock.send(FRAME.pack(DEVICE, 8, reply.ljust(8, b"\0")))
            if reply[0] == 0x11:
                print(f"node ID {device.pending} configured", flush=True)


if __name__ == "__main__":
    main()
