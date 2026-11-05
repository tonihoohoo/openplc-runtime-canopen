# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the slave blocks of a PLC program through a raw SocketCAN master.

Needs the slave blocks e2e project running in the runtime: slave node 5 on the interface with
`on_plc_stop_state: pre_operational` and SDO objects from the master 0x2000:1 (Command, USINT) and
0x2000:2 (Alarm, UDINT), and to the master 0x2100:1 (the slave status block's STATE), 0x2100:2
(the write counter of 0x2000:2), 0x2100:3 (ERRORREGISTER), 0x2100:4 (the emergency block's ERROR)
and 0x2100:5 (the NMT block's ERROR). The program raises emergency 4210h with INFO = Alarm while
Alarm is not 0 (6200h when Alarm is 6200), and while Command is not 0 asks for pre-operational
(1), operational (2) or a reset node (3). This script plays the master.

    canopen_slave_blocks_e2e.py --user ci --password secret --interface vcan0
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
FRAME = struct.Struct("=IB3x8s")
OPERATIONAL = 5
PRE_OPERATIONAL = 3
WRONG_PARAMETER = 6

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

    def count(self, start: int, match: Callable[[Frame], bool]) -> int:
        with self.lock:
            return sum(1 for frame in self.frames[start:] if match(frame))


def sdo_read(bus: Bus, index: int, sub: int) -> tuple[int, int]:
    """Returns (abort code, value); an abort code of 0 means success."""
    start = bus.mark()
    bus.send(0x600 + NODE, struct.pack("<BHB4x", 0x40, index, sub))
    frame = bus.wait(start, lambda f: f[0] == 0x580 + NODE, 2)
    if frame is None:
        sys.exit(f"FAIL: no SDO answer for 0x{index:04X}:{sub}")
    cmd, _, _, value = struct.unpack("<BHBI", frame[1])
    return (value, 0) if cmd == 0x80 else (0, value)


def sdo_write(bus: Bus, index: int, sub: int, value: int, size: int) -> int:
    start = bus.mark()
    cmd = 0x23 | ((4 - size) << 2)
    bus.send(0x600 + NODE, struct.pack("<BHBI", cmd, index, sub, value))
    frame = bus.wait(start, lambda f: f[0] == 0x580 + NODE, 2)
    if frame is None:
        sys.exit(f"FAIL: no SDO answer for 0x{index:04X}:{sub}")
    answer, _, _, code = struct.unpack("<BHBI", frame[1])
    return code if answer == 0x80 else 0


def is_emcy(code: int) -> Callable[[Frame], bool]:
    return lambda f: f[0] == 0x80 + NODE and len(f[1]) == 8 and f[1][:2] == struct.pack("<H", code)


def is_heartbeat(state: int) -> Callable[[Frame], bool]:
    return lambda f: f[0] == 0x700 + NODE and f[1] == bytes([state])


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

    def wait_object(what: str, index: int, sub: int, wanted: int, seconds: float = 5) -> None:
        end = time.monotonic() + seconds
        got = sdo_read(bus, index, sub)
        while got != (0, wanted) and time.monotonic() < end:
            time.sleep(0.05)
            got = sdo_read(bus, index, sub)
        expect(what, got, (0, wanted))

    def write(what: str, index: int, sub: int, value: int, size: int) -> None:
        expect(what, sdo_write(bus, index, sub, value, size), 0)

    def active_codes(s: Json) -> list[object]:
        found = s.get("emcy_active")
        return list(found) if isinstance(found, list) else []

    def program_codes(s: Json) -> list[object]:
        found = s.get("emcy_program")
        return list(found) if isinstance(found, list) else []

    def program_emcy_event(s: Json, code: int) -> bool:
        events = s.get("events")
        return isinstance(events, list) and any(
            isinstance(e, dict)
            and e.get("kind") == "emcy"
            and e.get("code") == code
            and e.get("program") is True
            for e in events
        )

    heartbeat = bus.wait(0, lambda f: f[0] == 0x700 + NODE, 30)
    if heartbeat is None:
        sys.exit("FAIL: no heartbeat from node 5")
    expect("heartbeat in pre-operational", heartbeat[1], b"\x7f")
    wait_status(
        "status shows the slave with the PLC running and its PLC stop state",
        lambda s: s.get("plc_running") is True and s.get("on_plc_stop_state") == "pre_operational",
    )
    expect("0x1029 sub 2 holds pre-operational", sdo_read(bus, 0x1029, 2), (0, 0))
    wait_object("status block reports pre-operational", 0x2100, 1, PRE_OPERATIONAL)

    start = bus.mark()
    bus.send(0x000, bytes([0x01, NODE]))
    expect(
        "operational on the master's start",
        bus.wait(start, is_heartbeat(0x05), 5) is not None,
        True,
    )
    wait_object("status block reports operational", 0x2100, 1, OPERATIONAL)

    start = bus.mark()
    write("alarm 865 written", 0x2000, 2, 865, 4)
    emcy = bus.wait(start, is_emcy(0x4210), 5)
    expect("program EMCY 4210h sent", emcy is not None, True)
    if emcy is not None:
        expect("EMCY error register is generic and temperature", emcy[1][2], 0x09)
        expect("EMCY bytes 4 to 7 carry INFO", struct.unpack("<I", emcy[1][4:8])[0], 865)
    wait_object("status block reports the error register", 0x2100, 3, 0x09)
    wait_object("write counter counts the write", 0x2100, 2, 1)
    wait_status(
        "status lists 4210h as the program's and its EMCY event",
        lambda s: 0x4210 in program_codes(s)
        and 0x4210 in active_codes(s)
        and program_emcy_event(s, 0x4210),
    )

    start = bus.mark()
    write("the same alarm written again", 0x2000, 2, 865, 4)
    wait_object("write counter counts the same value again", 0x2100, 2, 2)
    expect("an unchanged alarm sends no EMCY", bus.count(start, is_emcy(0x4210)), 0)

    start = bus.mark()
    write("alarm cleared", 0x2000, 2, 0, 4)
    emcy = bus.wait(start, is_emcy(0x0000), 5)
    expect("EMCY 0000h when the alarm clears", emcy is not None, True)
    if emcy is not None:
        expect("error register clear after the alarm", emcy[1][2], 0x00)
    wait_object("status block reports no error", 0x2100, 3, 0)

    start = bus.mark()
    write("alarm with the PLC stop's code", 0x2000, 2, 6200, 4)
    wait_object("emergency block refuses 6200h", 0x2100, 4, WRONG_PARAMETER)
    expect("no EMCY 6200h from the program", bus.count(start, is_emcy(0x6200)), 0)
    write("alarm cleared again", 0x2000, 2, 0, 4)
    wait_object("emergency block error clears", 0x2100, 4, 0)

    start = bus.mark()
    write("command reset node", 0x2000, 1, 3, 1)
    wait_object("NMT block refuses a reset", 0x2100, 5, WRONG_PARAMETER)
    expect("no boot-up after the refused reset", bus.count(start, is_heartbeat(0x00)), 0)
    write("command cleared", 0x2000, 1, 0, 1)

    start = bus.mark()
    write("command pre-operational", 0x2000, 1, 1, 1)
    expect(
        "the program takes the slave to pre-operational",
        bus.wait(start, is_heartbeat(0x7F), 5) is not None,
        True,
    )
    wait_status(
        "status shows the program as the NMT source", lambda s: s.get("nmt_by") == "program"
    )
    wait_object("status block reports pre-operational again", 0x2100, 1, PRE_OPERATIONAL)
    write("command cleared after pre-operational", 0x2000, 1, 0, 1)
    start = bus.mark()
    write("command operational", 0x2000, 1, 2, 1)
    expect(
        "the program takes the slave back to operational",
        bus.wait(start, is_heartbeat(0x05), 5) is not None,
        True,
    )
    write("command cleared after operational", 0x2000, 1, 0, 1)

    start = bus.mark()
    write("alarm 900 written", 0x2000, 2, 900, 4)
    expect("program EMCY 4210h sent again", bus.wait(start, is_emcy(0x4210), 5) is not None, True)
    start = bus.mark()
    call("/stop-plc", token)
    expect("EMCY 6200h when the PLC stops", bus.wait(start, is_emcy(0x6200), 10) is not None, True)
    expect(
        "pre-operational on the PLC stop",
        bus.wait(start, is_heartbeat(0x7F), 5) is not None,
        True,
    )
    wait_status(
        "status shows the PLC stop as the NMT source and the program's alarm cleared",
        lambda s: s.get("plc_running") is False
        and s.get("nmt_by") == "plc_stop"
        and 0x6200 in active_codes(s)
        and 0x4210 not in active_codes(s)
        and not program_codes(s),
    )

    start = bus.mark()
    call("/start-plc", token)
    expect("EMCY 0000h when the PLC starts", bus.wait(start, is_emcy(0x0000), 20) is not None, True)
    wait_status(
        "the slave stays pre-operational after the PLC start",
        lambda s: s.get("plc_running") is True and s.get("state") == "pre_operational",
    )
    start = bus.mark()
    bus.send(0x000, bytes([0x01, NODE]))
    expect(
        "the master's start brings it back",
        bus.wait(start, is_heartbeat(0x05), 5) is not None,
        True,
    )
    bus.stop = True
    print("slave blocks e2e passed")


if __name__ == "__main__":
    main()
