# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of device commissioning through the runtime's web API.

Needs the slave LSS e2e project in the runtime (its slave on vcan0) and two simulated LSS devices on
vcan1: node 10 (serial number 0x0A0A) and node 127 (serial number 0x7F7F). With the PLC stopped,
service mode for commissioning is refused on vcan0, naming the slave. On vcan1 a bus scan finds both
devices with their identity, node 127 gets node ID 20 over LSS with store and a reset node, and a
second scan finds nodes 10 and 20. Then service mode closes and the PLC starts again.

    canopen_commission_e2e.py --user ci --password secret --interface vcan1
"""

import argparse
import sys
import time

from canopen_online_e2e import Json, call, expect, wait_for

FIRST = 0x0A0A
SPARE = 0x7F7F


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--slave-interface", default="vcan0")
    args = parser.parse_args()
    token = str(
        call("/login", body={"username": args.user, "password": args.password})["access_token"]
    )

    def request(body: Json) -> Json:
        return call("/canopen/request", token, body)

    def job(body: Json, seconds: float = 30) -> Json:
        started = request(body)
        if "job" not in started:
            sys.exit(f"FAIL: {body}: {started}")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            result = request({"op": "job", "job": started["job"]})
            if result.get("state") != "pending":
                return result
            time.sleep(0.2)
        sys.exit(f"FAIL: {body} never finished")

    def open_on(interface: str) -> Json:
        return request(
            {
                "op": "service_open",
                "interface": interface,
                "bitrate": 0,
                "master_node_id": 127,
                "sdo_timeout_ms": 500,
                "commission": True,
            }
        )

    def scan(network: int) -> dict[int, Json]:
        started = time.monotonic()
        result = job({"op": "scan", "network": network})
        expect("scan done", result.get("state"), "done")
        if time.monotonic() - started > 6:
            sys.exit("FAIL: the scan took more than 6 seconds")
        raw = result.get("devices")
        devices = raw if isinstance(raw, list) else []
        return {int(d["node_id"]): d for d in devices if d.get("answered")}

    call("/stop-plc", token)
    wait_for("PLC stopped", lambda: "STOPPED" in str(call("/status", token).get("status", "")))

    expect(
        "commissioning refused on the slave's interface",
        open_on(args.slave_interface),
        {"error": "slave_on_interface", "name": "slave"},
    )
    opened = open_on(args.interface)
    expect("commissioning open", opened.get("ok"), True)
    network = int(str(opened.get("network")))
    wait_for(
        "the service bus runs",
        lambda: (
            "job"
            in request(
                {"op": "sdo_read", "network": network, "node": 10, "index": 0x1000, "subindex": 0}
            )
        ),
        20,
    )

    found = scan(network)
    expect("devices found", sorted(found), [10, 127])
    expect("node 10 serial number", found[10].get("serial_number"), FIRST)
    expect("node 127 serial number", found[127].get("serial_number"), SPARE)
    expect("node 127 vendor ID", found[127].get("vendor_id"), 1)
    expect("node 127 device type", found[127].get("device_type"), 0x191)

    address: Json = {
        "vendor_id": found[127]["vendor_id"],
        "product_code": found[127]["product_code"],
        "revision": found[127]["revision"],
        "serial_number": SPARE,
    }
    expect(
        "node ID 20 given and stored",
        job(
            {
                "op": "lss",
                "network": network,
                "action": "set_id",
                "node_id": 20,
                "store": True,
                **address,
            }
        ),
        {"state": "done", "previous_id": 127, "stored": True},
    )
    expect(
        "reset node sent",
        request({"op": "nmt", "network": network, "node": 127, "command": "reset_node"}),
        {"ok": True},
    )
    time.sleep(1)
    again = scan(network)
    expect("devices after the new node ID", sorted(again), [10, 20])
    read = job({"op": "sdo_read", "network": network, "node": 20, "index": 0x1018, "subindex": 4})
    expect("node 20 serial number", read.get("data"), SPARE.to_bytes(4, "little").hex())

    expect("commissioning closed", request({"op": "service_close"}), {"ok": True})
    call("/start-plc", token)
    wait_for("PLC running", lambda: "RUNNING" in str(call("/status", token).get("status", "")))
    print("PASS")


if __name__ == "__main__":
    main()
