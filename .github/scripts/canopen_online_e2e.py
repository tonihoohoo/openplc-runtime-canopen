# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CANopen online tools through the runtime's web API.

Needs the e2e project running with the simulated node 2 on the bus. Reads and writes node 2's
objects, puts it in pre-operational and starts it again while the PLC runs; then stops the PLC,
opens service mode, reads the node, closes service mode and starts the PLC again.

    canopen_online_e2e.py --user ci --password secret --interface vcan0
"""

import argparse
import json
import ssl
import sys
import time
import urllib.request
from typing import Callable, Dict, Optional

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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--interface", required=True)
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

    def node2() -> Json:
        nets = call("/canopen/status", token).get("networks")
        if not isinstance(nets, list) or not nets:
            return {}
        for n in nets[0]["nodes"]:
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
    call("/stop-plc", token)
    # A start sent while the PLC is still stopping is ignored, so wait for STOPPED.
    wait_for("PLC stopped", lambda: "STOPPED" in str(call("/status", token).get("status", "")))
    wait_for("CANopen stopped", lambda: call("/canopen/status", token).get("active") is False)
    expect("service mode opens", request(service), {"ok": True, "network": 0})
    wait_for("node 2 in service mode", lambda: node2().get("state") == "service")
    read = sdo("sdo_read", 0x1000, 0)
    expect("read in service mode", read.get("state"), "done")
    expect("service mode closes", request({"op": "service_close"}), {"ok": True})
    wait_for("service mode closed", lambda: call("/canopen/status", token).get("active") is False)
    call("/start-plc", token)
    wait_for(
        "node 2 operational after the PLC start", lambda: node2().get("state") == "operational"
    )
    print("PASS")


if __name__ == "__main__":
    main()
