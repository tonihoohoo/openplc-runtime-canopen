# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CANopen online tools through the runtime's web API.

Needs the e2e project running with the simulated node 2 on the bus. Reads and writes node 2's
objects, puts it in pre-operational and starts it again while the PLC runs. Then stops the PLC:
the network stays on the bus (the master's heartbeat goes on, node 2 is left pre-operational and
still answers SDO), service mode is refused on its interface and opens on the free one, and the
next PLC start sends no reset and no SDO to node 2.

    canopen_online_e2e.py --user ci --password secret --interface vcan0 --free-interface vcan1
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
from typing import Callable, Dict, List, Optional, Tuple

BASE = "https://127.0.0.1:8443/api"
CONTEXT = ssl.create_default_context()
CONTEXT.check_hostname = False
CONTEXT.verify_mode = ssl.CERT_NONE

Json = Dict[str, object]


def call(path: str, token: Optional[str] = None, body: Optional[Json] = None) -> Json:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method="POST" if data else "GET")
    req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(req, context=CONTEXT, timeout=10) as resp:
        answer: Json = json.loads(resp.read())
    return answer


def wait_for(what: str, check: Callable[[], bool], seconds: float = 30) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check():
            print(f"ok: {what}")
            return
        time.sleep(0.5)
    sys.exit(f"FAIL: {what}")


def expect(what: str, got: object, wanted: object) -> None:
    if got != wanted:
        sys.exit(f"FAIL: {what}: {got!r}, expected {wanted!r}")
    print(f"ok: {what}")


class Capture:
    """The CAN frames on an interface while it runs: (time, id, first data byte)."""

    def __init__(self, interface: str) -> None:
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.settimeout(0.2)
        self.frames: List[Tuple[float, int, int]] = []
        self.running = True
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self) -> None:
        while self.running:
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                continue
            can_id, length = struct.unpack("=IB3x", frame[:8])
            first = frame[8] if length else 0
            self.frames.append((time.monotonic(), can_id & socket.CAN_EFF_MASK, first))

    def stop(self) -> List[Tuple[float, int, int]]:
        self.running = False
        self.thread.join()
        self.sock.close()
        return self.frames


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--free-interface", required=True)
    args = parser.parse_args()
    token = str(
        call("/login", body={"username": args.user, "password": args.password})["access_token"]
    )

    def request(body: Json) -> Json:
        return call("/canopen/request", token, body)

    def job(body: Json) -> Json:
        started = request(body)
        if "job" not in started:
            sys.exit(f"FAIL: {body}: {started}")
        end = time.monotonic() + 10
        while time.monotonic() < end:
            result = request({"op": "job", "job": started["job"]})
            if result.get("state") != "pending":
                return result
            time.sleep(0.1)
        sys.exit(f"FAIL: {body} never finished")

    def network() -> Json:
        nets = call("/canopen/status", token).get("networks")
        if not isinstance(nets, list) or not nets:
            return {}
        return dict(nets[0])

    def node2() -> Json:
        nodes = network().get("nodes")
        if not isinstance(nodes, list):
            return {}
        for n in nodes:
            if n["node_id"] == 2:
                return dict(n)
        return {}

    def sdo(op: str, index: int, sub: int, data: str = "") -> Json:
        body: Json = {"op": op, "network": 0, "node": 2, "index": index, "subindex": sub}
        if data:
            body["data"] = data
        return job(body)

    def nmt(command: str) -> Json:
        return request({"op": "nmt", "network": 0, "node": 2, "command": command})

    wait_for("node 2 operational", lambda: node2().get("state") == "operational")
    expect(
        "read of the device name (segmented)",
        sdo("sdo_read", 0x1008, 0),
        {"state": "done", "data": b"Generic I/O module".hex()},
    )
    expect(
        "write refused while operational",
        sdo("sdo_write", 0x2101, 0, "1400"),
        {"state": "failed", "abort_code": 0x08000022},
    )
    expect("pre-operational", nmt("pre_operational"), {"ok": True})
    wait_for("node 2 manual", lambda: node2().get("state") == "manual")
    expect("write in pre-operational", sdo("sdo_write", 0x2101, 0, "1400"), {"state": "done"})
    expect("read back", sdo("sdo_read", 0x2101, 0), {"state": "done", "data": "1400"})
    expect("start", nmt("start"), {"ok": True})
    wait_for("node 2 operational again", lambda: node2().get("state") == "operational")
    expect(
        "unknown node refused",
        request({"op": "nmt", "network": 0, "node": 77, "command": "start"}),
        {"error": "unknown_node"},
    )

    service: Json = {
        "op": "service_open",
        "name": "canopen_1",
        "interface": args.interface,
        "bitrate": 0,
        "master_node_id": 127,
        "sdo_timeout_ms": 1000,
        "nodes": [2],
    }
    expect("service mode refused while the PLC runs", request(service), {"error": "plc_running"})
    capture = Capture(args.interface)
    call("/stop-plc", token)
    # A start sent while the PLC is still stopping is ignored, so wait for STOPPED.
    wait_for("PLC stopped", lambda: "STOPPED" in str(call("/status", token).get("status", "")))
    wait_for("the network stays on the bus", lambda: network().get("plc_running") is False)
    expect("CANopen still active", call("/canopen/status", token).get("active"), True)
    wait_for("node 2 left pre-operational", lambda: node2().get("state") == "stopped")
    losses_at_stop = node2().get("losses")
    stopped_at = time.monotonic()
    read = sdo("sdo_read", 0x1018, 1)
    expect("read of the vendor ID with the PLC stopped", read.get("state"), "done")
    expect(
        "service mode refused on the network's interface",
        request(service),
        {"error": "interface_in_use"},
    )
    free = dict(service, interface=args.free_interface, name="free")
    expect("service mode opens on a free interface", request(free), {"ok": True, "network": 1})
    expect("the network goes on", network().get("plc_running"), False)
    expect("service mode closes", request({"op": "service_close"}), {"ok": True})
    time.sleep(max(0.0, stopped_at + 2 - time.monotonic()))
    expect("node 2 still left pre-operational", node2().get("state"), "stopped")
    expect("node 2 not lost", node2().get("losses"), losses_at_stop)

    restarted = time.monotonic()
    call("/start-plc", token)
    wait_for(
        "node 2 operational after the PLC start", lambda: node2().get("state") == "operational"
    )
    expect("the PLC runs on the network", network().get("plc_running"), True)
    time.sleep(1)
    frames = capture.stop()
    heartbeats = [f for f in frames if stopped_at <= f[0] < restarted and f[1] == 0x77F]
    print(f"ok: {len(heartbeats)} master heartbeats while the PLC was stopped")
    if len(heartbeats) < 10:
        sys.exit("FAIL: the master's heartbeat did not go on while the PLC was stopped")
    after = [f for f in frames if f[0] >= restarted]
    resets = [f for f in after if f[1] == 0x000 and f[2] in (0x81, 0x82)]
    expect("no NMT reset at the PLC start", len(resets), 0)
    sdos = [f for f in after if f[1] == 0x602]
    expect("no SDO to node 2 at the PLC start", len(sdos), 0)
    starts = [f for f in after if f[1] == 0x000 and f[2] == 0x01]
    if not starts:
        sys.exit("FAIL: no NMT start at the PLC start")
    print("ok: NMT start at the PLC start")
    print("PASS")


if __name__ == "__main__":
    main()
