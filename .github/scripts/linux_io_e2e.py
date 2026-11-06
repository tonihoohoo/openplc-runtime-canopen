# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the linux_io blocks on simulated GPIO lines through a raw SocketCAN master.

Needs the editor's linux-io e2e project running in the runtime (slave node 5 on the interface) and a
gpio-sim chip with the lines lio-in (offset 0), lio-out (1) and lio-on (2), whose sysfs directory is
given. The level of lio-in comes in TPDO 1 (bit 0; bits 1-3 are the blocks' VALID outputs); RPDO 1
bit 0 drives lio-out (off on a PLC stop) and bit 1 lio-on (on on a PLC stop). 0x2100:02 holds the
loopback MTU read by READ_VALUE and 0x2100:03-05 the blocks' ERROR outputs. Runs as root (sysfs).

    linux_io_e2e.py --user ci --password secret --interface vcan0 --sim /sys/devices/platform/...
"""

import argparse
import json
import socket
import ssl
import struct
import sys
import threading
import time
import urllib.request
from collections.abc import Callable
from pathlib import Path

BASE = "https://127.0.0.1:8443/api"
CONTEXT = ssl.create_default_context()
CONTEXT.check_hostname = False
CONTEXT.verify_mode = ssl.CERT_NONE
NODE = 5
FRAME = struct.Struct("=IB3x8s")

Json = dict[str, object]
Frame = tuple[int, bytes]


def call(path: str, token: str | None = None, body: Json | None = None) -> Json:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method="POST" if data else "GET")
    req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(req, context=CONTEXT, timeout=10) as resp:
        answer: Json = json.loads(resp.read())
    return answer


def expect(what: str, got: object, wanted: object) -> None:
    if got != wanted:
        sys.exit(f"FAIL: {what}: {got!r}, expected {wanted!r}")
    print(f"ok: {what}")


class Bus:
    """A SocketCAN socket that keeps every received frame."""

    def __init__(self, interface: str) -> None:
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.settimeout(0.05)
        self.frames: list[Frame] = []
        self.lock = threading.Lock()
        self.stop = False
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self) -> None:
        while not self.stop:
            try:
                raw = self.sock.recv(FRAME.size)
            except TimeoutError:
                continue
            ident, dlc, data = FRAME.unpack(raw)
            with self.lock:
                self.frames.append((ident & 0x7FF, data[:dlc]))

    def send(self, ident: int, data: bytes) -> None:
        self.sock.send(FRAME.pack(ident, len(data), data.ljust(8, b"\0")))

    def mark(self) -> int:
        with self.lock:
            return len(self.frames)

    def wait(self, start: int, match: Callable[[Frame], bool], seconds: float) -> Frame | None:
        end = time.monotonic() + seconds
        while True:
            with self.lock:
                for frame in self.frames[start:]:
                    if match(frame):
                        return frame
            if time.monotonic() >= end:
                return None
            time.sleep(0.01)


def sdo_read(bus: Bus, index: int, sub: int) -> tuple[int, int]:
    """Returns (abort code, value); an abort code of 0 means success."""
    start = bus.mark()
    bus.send(0x600 + NODE, struct.pack("<BHB4x", 0x40, index, sub))
    frame = bus.wait(start, lambda f: f[0] == 0x580 + NODE, 2)
    if frame is None:
        sys.exit(f"FAIL: no SDO answer for 0x{index:04X}:{sub}")
    cmd, _, _, value = struct.unpack("<BHBI", frame[1])
    return (value, 0) if cmd == 0x80 else (0, value)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--sim", required=True, help="sysfs directory of the gpio-sim chip")
    args = parser.parse_args()
    sim = Path(args.sim)
    token = str(
        call("/login", body={"username": args.user, "password": args.password})["access_token"]
    )
    bus = Bus(args.interface)

    def line(offset: int) -> str:
        return (sim / f"sim_gpio{offset}" / "value").read_text().strip()

    def pull(offset: int, how: str) -> None:
        (sim / f"sim_gpio{offset}" / "pull").write_text(how)

    def wait_line(what: str, offset: int, wanted: str, seconds: float = 3) -> None:
        end = time.monotonic() + seconds
        got = line(offset)
        while got != wanted and time.monotonic() < end:
            time.sleep(0.02)
            got = line(offset)
        expect(what, got, wanted)

    def wait_object(
        what: str, index: int, sub: int, check: Callable[[int], bool], seconds: float = 10
    ) -> int:
        end = time.monotonic() + seconds
        abort, value = sdo_read(bus, index, sub)
        while (abort != 0 or not check(value)) and time.monotonic() < end:
            time.sleep(0.1)
            abort, value = sdo_read(bus, index, sub)
        if abort != 0 or not check(value):
            sys.exit(f"FAIL: {what}: abort 0x{abort:08X}, value {value}")
        print(f"ok: {what} ({value})")
        return value

    def tpdo_bit0(level: int) -> Callable[[Frame], bool]:
        return lambda f: f[0] == 0x180 + NODE and len(f[1]) >= 1 and (f[1][0] & 1) == level

    heartbeat = bus.wait(0, lambda f: f[0] == 0x700 + NODE, 30)
    if heartbeat is None:
        sys.exit("FAIL: no heartbeat from node 5")
    start = bus.mark()
    bus.send(0x000, bytes([0x01, NODE]))
    expect(
        "operational on the master's start",
        bus.wait(start, lambda f: f[0] == 0x700 + NODE and f[1] == b"\x05", 5) is not None,
        True,
    )

    wait_object("GPIO_IN has its line", 0x2100, 3, lambda v: v == 0)
    wait_object("GPIO_OUT has its lines", 0x2100, 4, lambda v: v == 0)
    wait_object("READ_VALUE reads without error", 0x2100, 5, lambda v: v == 0)
    mtu = int(Path("/sys/class/net/lo/mtu").read_text().strip())
    wait_object("READ_VALUE gives the loopback MTU", 0x2100, 2, lambda v: v == mtu)
    wait_object("all blocks VALID", 0x2100, 1, lambda v: (v & 0x0E) == 0x0E)

    start = bus.mark()
    pull(0, "pull-up")
    expect(
        "lio-in high reaches the master in TPDO 1",
        bus.wait(start, tpdo_bit0(1), 3) is not None,
        True,
    )
    start = bus.mark()
    pull(0, "pull-down")
    expect(
        "lio-in low reaches the master in TPDO 1",
        bus.wait(start, tpdo_bit0(0), 3) is not None,
        True,
    )

    bus.send(0x200 + NODE, bytes([0x01]))
    wait_line("RPDO bit 0 drives lio-out high", 1, "1")
    wait_line("lio-on stays low", 2, "0")
    bus.send(0x200 + NODE, bytes([0x02]))
    wait_line("RPDO bit 0 clear drives lio-out low", 1, "0")
    wait_line("RPDO bit 1 drives lio-on high", 2, "1")
    bus.send(0x200 + NODE, bytes([0x01]))
    wait_line("lio-out high before the stop", 1, "1")
    wait_line("lio-on low before the stop", 2, "0")

    call("/stop-plc", token)
    wait_line("lio-out goes off on a PLC stop (LIO_STOP_OFF)", 1, "0")
    wait_line("lio-on goes on on a PLC stop (LIO_STOP_ON)", 2, "1")

    time.sleep(3)  # past the 2 s release time
    call("/start-plc", token)
    wait_object("GPIO_OUT has its lines again after the start", 0x2100, 4, lambda v: v == 0, 20)
    bus.send(0x200 + NODE, bytes([0x00]))
    wait_line("lio-out follows the master again", 1, "0")
    wait_line("lio-on follows the master again", 2, "0")
    bus.send(0x200 + NODE, bytes([0x01]))
    wait_line("lio-out high again", 1, "1")
    bus.stop = True
    print("linux_io end-to-end test passed")


if __name__ == "__main__":
    main()
