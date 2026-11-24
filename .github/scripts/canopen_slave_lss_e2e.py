# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of LSS in the CANopen slave through a raw SocketCAN master.

Needs the slave LSS e2e project running in the runtime: the slave of the slave e2e project with LSS
on (project node 5) and the fixed serial number 23. This script plays a commissioning tool's LSS
master: the slave runs as node 5 and takes no part in a fastscan, is selected by its LSS address,
gets node ID 12, stores it and comes up as node 12 after a reset communication with its COB-IDs
following; a reset node keeps the stored ID, and the forget request sends it back to node 5.

    canopen_slave_lss_e2e.py --user ci --password secret --interface vcan0
"""

import argparse
import struct
import sys
import time
from collections.abc import Callable

from canopen_slave_e2e import Bus, Json, call, expect

LSS_MASTER = 0x7E5
LSS_SLAVE = 0x7E4
NODE = 12
ADDRESS = (0x0000ABCD, 1, 0x00010000, 23)


def lss(bus: Bus, data: bytes, answer: bool = True) -> bytes | None:
    start = bus.mark()
    bus.send(LSS_MASTER, data.ljust(8, b"\0"))
    if not answer:
        return None
    frame = bus.wait(start, lambda f: f[0] == LSS_SLAVE, 1)
    return frame[1] if frame else None


def fastscan(bus: Bus, ident: int, bit: int, sub: int, nxt: int) -> bool:
    start = bus.mark()
    bus.send(LSS_MASTER, struct.pack("<BIBBB", 0x51, ident, bit, sub, nxt))
    return bus.wait(start, lambda f: f[0] == LSS_SLAVE and f[1][0] == 0x4F, 0.1) is not None


def sdo_read(bus: Bus, node: int, index: int, sub: int) -> int:
    start = bus.mark()
    bus.send(0x600 + node, struct.pack("<BHB4x", 0x40, index, sub))
    frame = bus.wait(start, lambda f: f[0] == 0x580 + node, 2)
    if frame is None:
        sys.exit(f"FAIL: no SDO answer from node {node} for 0x{index:04X}:{sub}")
    cmd, _, _, value = struct.unpack("<BHBI", frame[1])
    if cmd == 0x80:
        sys.exit(f"FAIL: node {node} refused 0x{index:04X}:{sub} with 0x{value:08X}")
    return int(value)


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

    def lss_status(s: Json) -> Json:
        value = s.get("lss")
        return value if isinstance(value, dict) else {}

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

    wait_status(
        "status shows the slave as node 5 from the project",
        lambda s: (
            s.get("node_id") == 5
            and s.get("serial") == 23
            and lss_status(s).get("node_id_from") == "project"
        ),
    )
    expect("no fastscan answer from a configured slave", fastscan(bus, 0, 0x80, 0, 0), False)
    lss(bus, struct.pack("<BI", 0x40, ADDRESS[0]), answer=False)
    lss(bus, struct.pack("<BI", 0x41, ADDRESS[1]), answer=False)
    lss(bus, struct.pack("<BI", 0x42, ADDRESS[2]), answer=False)
    answer = lss(bus, struct.pack("<BI", 0x43, ADDRESS[3]))
    expect("switch selective by the LSS address", answer and answer[0], 0x44)
    answer = lss(bus, b"\x5e")
    expect("inquire node ID", answer and answer[1], 5)
    answer = lss(bus, bytes([0x11, NODE]))
    expect("configure node ID 12", answer and answer[1], 0)
    answer = lss(bus, b"\x17")
    expect("store configuration", answer and answer[1], 0)
    lss(bus, b"\x04\x00", answer=False)

    start = bus.mark()
    bus.send(0x000, bytes([0x82, 5]))
    expect(
        "boot-up as node 12 after a reset communication",
        bus.wait(start, lambda f: f[0] == 0x700 + NODE and f[1] == b"\x00", 5) is not None,
        True,
    )
    wait_status(
        "status shows node 12 from the stored values",
        lambda s: (
            s.get("node_id") == NODE
            and lss_status(s).get("node_id_from") == "stored"
            and lss_status(s).get("stored_node_id") == NODE
        ),
    )
    expect("serial number in 0x1018:4", sdo_read(bus, NODE, 0x1018, 4), 23)
    expect("EMCY COB-ID follows the node ID", sdo_read(bus, NODE, 0x1014, 0) & 0x7FF, 0x80 + NODE)
    expect(
        "TPDO 1 COB-ID follows the node ID", sdo_read(bus, NODE, 0x1800, 1) & 0x7FF, 0x180 + NODE
    )

    start = bus.mark()
    bus.send(0x000, bytes([0x81, NODE]))
    expect(
        "boot-up as node 12 again after a reset node",
        bus.wait(start, lambda f: f[0] == 0x700 + NODE and f[1] == b"\x00", 5) is not None,
        True,
    )
    wait_status(
        "status shows node 12 from the stored values",
        lambda s: s.get("node_id") == NODE and lss_status(s).get("node_id_from") == "stored",
    )

    answer_json = call("/canopen/request", token, {"op": "slave_lss_forget", "slave": "slave"})
    expect("forget request answered", answer_json.get("ok"), True)
    wait_status(
        "status shows node 5 from the project again",
        lambda s: s.get("node_id") == 5 and "stored_node_id" not in lss_status(s),
    )
    start = bus.mark()
    time.sleep(1)
    expect(
        "no heartbeat as node 12 after forget",
        bus.wait(start, lambda f: f[0] == 0x700 + NODE, 0),
        None,
    )
    bus.stop = True
    print("slave LSS e2e passed")


if __name__ == "__main__":
    main()
