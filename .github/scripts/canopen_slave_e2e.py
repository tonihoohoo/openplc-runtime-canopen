# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CANopen slave through a raw SocketCAN master.

Needs the slave e2e project running in the runtime: node 5 on the interface, a program that copies
0x2000:1 (from the master) to 0x2100:1 and the parameter 0x2000:3 to 0x2100:2, RPDO1 0x205 and an
event TPDO1 0x185, a heartbeat of 100 ms and a watch of master node 127. This script plays the
master: SDO reads and writes with range checks, NMT start, an RPDO in and the TPDO back, the master
heartbeat lost, the PLC stopped and started, and the slave in /api/canopen/status.

    canopen_slave_e2e.py --user ci --password secret --interface vcan0
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

BASE = "https://127.0.0.1:8443/api"
CONTEXT = ssl.create_default_context()
CONTEXT.check_hostname = False
CONTEXT.verify_mode = ssl.CERT_NONE
NODE = 5
MASTER = 127
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
    """Returns (value, abort code); an abort code of 0 means success."""
    start = bus.mark()
    bus.send(0x600 + NODE, struct.pack("<BHB4x", 0x40, index, sub))
    frame = bus.wait(start, lambda f: f[0] == 0x580 + NODE, 2)
    if frame is None:
        sys.exit(f"FAIL: no SDO answer for 0x{index:04X}:{sub}")
    cmd, _, _, value = struct.unpack("<BHBI", frame[1])
    return (0, value) if cmd == 0x80 else (value, 0)


def sdo_write(bus: Bus, index: int, sub: int, value: int, size: int) -> int:
    start = bus.mark()
    cmd = 0x23 | ((4 - size) << 2)
    bus.send(0x600 + NODE, struct.pack("<BHBI", cmd, index, sub, value))
    frame = bus.wait(start, lambda f: f[0] == 0x580 + NODE, 2)
    if frame is None:
        sys.exit(f"FAIL: no SDO answer for 0x{index:04X}:{sub}")
    answer, _, _, code = struct.unpack("<BHBI", frame[1])
    return code if answer == 0x80 else 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--interface", required=True)
    args = parser.parse_args()
    token = str(
        call("/login", body={"username": args.user, "password": args.password})["access_token"]
    )
    bus = Bus(args.interface)

    def slave() -> Json:
        slaves = call("/canopen/status", token).get("slaves")
        if isinstance(slaves, list) and slaves and isinstance(slaves[0], dict):
            found: Json = slaves[0]
            return found
        return {}

    def wait_status(what: str, check: Callable[[Json], bool], seconds: float = 20) -> None:
        end = time.monotonic() + seconds
        last: Json = {}
        while time.monotonic() < end:
            last = slave()
            if check(last):
                print(f"ok: {what}")
                return
            time.sleep(0.3)
        sys.exit(f"FAIL: {what}: {last}")

    heartbeat = bus.wait(0, lambda f: f[0] == 0x700 + NODE, 30)
    if heartbeat is None:
        sys.exit("FAIL: no heartbeat from node 5")
    expect("heartbeat in pre-operational", heartbeat[1], b"\x7f")
    wait_status(
        "status shows the slave pre-operational with the PLC running",
        lambda s: s.get("state") == "pre_operational" and s.get("plc_running") is True,
    )

    expect("vendor id", sdo_read(bus, 0x1018, 1), (0, 0x0000ABCD))
    expect("heartbeat time", sdo_read(bus, 0x1017, 0), (0, 100))
    expect("parameter default", sdo_read(bus, 0x2000, 3), (0, 1000))
    expect("parameter written", sdo_write(bus, 0x2000, 3, 1234, 2), 0)
    expect("parameter below its minimum", sdo_write(bus, 0x2000, 3, 50, 2), 0x06090032)
    expect("parameter above its maximum", sdo_write(bus, 0x2000, 3, 40000, 2), 0x06090031)
    expect("object to the master is read-only", sdo_write(bus, 0x2100, 1, 1, 2), 0x06010002)
    expect("store parameters is refused", sdo_write(bus, 0x1010, 1, 0x65766173, 4), 0x06020000)

    beating = threading.Event()
    beating.set()

    def beat() -> None:
        while beating.is_set():
            bus.send(0x700 + MASTER, b"\x05")
            time.sleep(0.1)

    beater = threading.Thread(target=beat, daemon=True)
    beater.start()

    start = bus.mark()
    bus.send(0x000, bytes([0x01, NODE]))
    expect(
        "heartbeat in operational",
        bus.wait(start, lambda f: f[0] == 0x700 + NODE and f[1] == b"\x05", 5) is not None,
        True,
    )
    start = bus.mark()
    bus.send(0x200 + NODE, struct.pack("<hB", 0x1234, 7))
    expect(
        "TPDO carries the RPDO value and the parameter back through the program",
        bus.wait(
            start,
            lambda f: f[0] == 0x180 + NODE and f[1][:4] == struct.pack("<hH", 0x1234, 1234),
            5,
        )
        is not None,
        True,
    )
    wait_status(
        "status shows the slave operational and the master seen",
        lambda s: s.get("state") == "operational" and s.get("master_seen") is True,
    )

    start = bus.mark()
    beating.clear()
    beater.join()
    emcy = bus.wait(start, lambda f: f[0] == 0x80 + NODE and f[1][:2] == b"\x30\x81", 5)
    expect("EMCY 8130h when the master heartbeat is lost", emcy is not None, True)
    expect(
        "pre-operational after the master is lost",
        bus.wait(start, lambda f: f[0] == 0x700 + NODE and f[1] == b"\x7f", 5) is not None,
        True,
    )

    start = bus.mark()
    call("/stop-plc", token)
    emcy = bus.wait(start, lambda f: f[0] == 0x80 + NODE and f[1][:2] == b"\x00\x62", 10)
    expect("EMCY 6200h when the PLC stops", emcy is not None, True)
    wait_status("status shows the PLC stopped", lambda s: s.get("plc_running") is False)
    expect(
        "the slave stays on the bus while the PLC is stopped", sdo_read(bus, 0x1017, 0), (0, 100)
    )

    start = bus.mark()
    call("/start-plc", token)
    emcy = bus.wait(start, lambda f: f[0] == 0x80 + NODE and f[1][:2] == b"\x00\x00", 20)
    expect("EMCY 0000h when the PLC starts again", emcy is not None, True)
    expect("parameter back to its default after the PLC start", sdo_read(bus, 0x2000, 3), (0, 1000))
    bus.stop = True
    print("slave e2e passed")


if __name__ == "__main__":
    main()
