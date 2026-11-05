# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CANopen LSS tools through the runtime's web API.

Needs the e2e project running with the simulated node 2 and two simulated devices without a node
ID (serial numbers 0x5678 and 0x5679) on the bus. Finds the two devices while node 2 runs, gives
the first node ID 4 with store, then stops the PLC, opens service mode, sets the second device's
bit rate to 125 kbit/s, closes service mode and starts the PLC again.

    canopen_lss_e2e.py --user ci --password secret --interface vcan0
"""

import argparse
import sys
import time

from canopen_online_e2e import Json, call, expect, wait_for

FIRST = 0x5678
SECOND = 0x5679


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
        end = time.monotonic() + 60
        while time.monotonic() < end:
            result = request({"op": "job", "job": started["job"]})
            if result.get("state") != "pending":
                return result
            time.sleep(0.2)
        sys.exit(f"FAIL: {body} never finished")

    def node2_state() -> str:
        nets = call("/canopen/status", token).get("networks")
        if not isinstance(nets, list) or not nets:
            return ""
        for n in nets[0]["nodes"]:
            if n["node_id"] == 2:
                return str(n["state"])
        return ""

    wait_for("node 2 operational", lambda: node2_state() == "operational", 60)
    found = job({"op": "lss", "network": 0, "action": "find"})
    expect("find", found.get("state"), "done")
    raw = found.get("devices")
    devices: list[dict[str, int]] = raw if isinstance(raw, list) else []
    by_serial = {d["serial_number"]: d for d in devices}
    expect("devices without a node ID", sorted(by_serial), [FIRST, SECOND])
    expect("node 2 still operational", node2_state(), "operational")

    def address(serial: int) -> Json:
        return dict(by_serial[serial])

    expect(
        "node ID in use refused",
        job({"op": "lss", "network": 0, "action": "set_id", "node_id": 2, **address(FIRST)}),
        {"state": "failed", "reason": "node_in_use"},
    )
    expect(
        "node ID 4 given and stored",
        job(
            {
                "op": "lss",
                "network": 0,
                "action": "set_id",
                "node_id": 4,
                "store": True,
                **address(FIRST),
            }
        ),
        {"state": "done", "previous_id": 255, "stored": True},
    )
    bitrate: Json = {
        "op": "lss",
        "network": 0,
        "action": "set_bitrate",
        "bitrate": 125000,
        **address(SECOND),
    }
    expect(
        "bit rate refused while the PLC runs",
        job(bitrate),
        {"state": "failed", "reason": "needs_service_mode"},
    )

    call("/stop-plc", token)
    wait_for("PLC stopped", lambda: "STOPPED" in str(call("/status", token).get("status", "")))
    wait_for("CANopen stopped", lambda: call("/canopen/status", token).get("active") is False)
    service: Json = {
        "op": "service_open",
        "name": "canopen_1",
        "interface": args.interface,
        "bitrate": 0,
        "master_node_id": 127,
        "sdo_timeout_ms": 1000,
        "nodes": [2],
    }
    expect("service mode opens", request(service), {"ok": True, "network": 0})
    expect("bit rate set and stored", job(bitrate), {"state": "done", "stored": True})
    expect("service mode closes", request({"op": "service_close"}), {"ok": True})
    wait_for("service mode closed", lambda: call("/canopen/status", token).get("active") is False)
    call("/start-plc", token)
    wait_for("node 2 operational after the PLC start", lambda: node2_state() == "operational", 60)
    print("PASS")


if __name__ == "__main__":
    main()
