# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of an LSS search that a device ends out of turn, through the runtime's web API.

Needs the e2e project running with the simulated node 2, a device that answers a fastscan out of
turn (lss_out_of_turn_device.py, serial 0x1234) and a simulated device without a node ID (serial
0x5679). The first search ends as incomplete with the out-of-turn device only; once that one has a
node ID, the next search finds the other.

    canopen_lss_out_of_turn_e2e.py --user ci --password secret
"""

import argparse
import sys
import time

from canopen_online_e2e import Json, call, expect, wait_for

OUT_OF_TURN = 0x1234
OTHER = 0x5679


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    args = parser.parse_args()
    token = str(
        call("/login", body={"username": args.user, "password": args.password})["access_token"]
    )

    def job(body: Json) -> Json:
        started = call("/canopen/request", token, body)
        if "job" not in started:
            sys.exit(f"FAIL: {body}: {started}")
        end = time.monotonic() + 60
        while time.monotonic() < end:
            result = call("/canopen/request", token, {"op": "job", "job": started["job"]})
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

    def serials(result: Json) -> list[int]:
        raw = result.get("devices")
        devices: list[dict[str, int]] = raw if isinstance(raw, list) else []
        return sorted(d["serial_number"] for d in devices)

    wait_for("node 2 operational", lambda: node2_state() == "operational", 60)
    first = job({"op": "lss", "network": 0, "action": "find"})
    expect("first search", (first.get("state"), first.get("reason")), ("failed", "incomplete"))
    expect("first search keeps the device found", serials(first), [OUT_OF_TURN])
    expect(
        "node ID 6 given to the out-of-turn device",
        job(
            {
                "op": "lss",
                "network": 0,
                "action": "set_id",
                "node_id": 6,
                "vendor_id": 1,
                "product_code": 2,
                "revision": 3,
                "serial_number": OUT_OF_TURN,
            }
        ),
        {"state": "done", "previous_id": 255, "stored": False},
    )
    second = job({"op": "lss", "network": 0, "action": "find"})
    expect("second search", second.get("state"), "done")
    expect("second search finds the other device", serials(second), [OTHER])
    expect("node 2 still operational", node2_state(), "operational")
    print("PASS")


if __name__ == "__main__":
    main()
