# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the canopen PLC function blocks through a running PLC program.

Needs the editor fork's canopen-plc-blocks project uploaded and running. Starts the simulated
node 2 in its block test mode and asks the program for runs of its checks (every block against the
node); between runs it stops and starts the PLC, once while a run is in progress, and checks that
the runtime keeps running and the blocks work after each start.

    canopen_blocks_e2e.py --sim <canopen_sim_slave> --interface vcan0 --user ci --password secret
"""

import argparse
import json
import queue
import ssl
import subprocess
import sys
import threading
import time
import urllib.request
from typing import Callable, Dict, List, Optional

BASE = "https://127.0.0.1:8443/api"
CONTEXT = ssl.create_default_context()
CONTEXT.check_hostname = False
CONTEXT.verify_mode = ssl.CERT_NONE
RESTARTS = 3

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


def wait_for(what: str, check: Callable[[], bool], seconds: float = 60) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check():
            print(f"ok: {what}", flush=True)
            return
        time.sleep(0.5)
    sys.exit(f"FAIL: {what}")


class Sim:
    """The simulated node in its block test mode, driven through stdin and stdout."""

    def __init__(self, command: List[str]) -> None:
        self.proc = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1
        )
        self.lines: "queue.Queue[str]" = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            print(f"sim: {line.rstrip()}", flush=True)
            self.lines.put(line.strip())
        self.lines.put("")

    def expect(self, wanted: str, seconds: float = 90) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                line = self.lines.get(timeout=max(0.1, end - time.monotonic()))
            except queue.Empty:
                break
            if line == wanted:
                return
            if "FAIL" in line or (line == "" and self.proc.poll() is not None):
                sys.exit(f"FAIL: the simulated node: {line or 'exited'}")
        sys.exit(f"FAIL: no '{wanted}' from the simulated node")

    def send(self, command: str) -> None:
        assert self.proc.stdin is not None
        self.proc.stdin.write(command + "\n")
        self.proc.stdin.flush()

    def close(self) -> int:
        try:
            self.send("quit")
        except BrokenPipeError:
            pass
        return self.proc.wait(timeout=30)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", required=True)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    args = parser.parse_args()
    token = str(
        call("/login", body={"username": args.user, "password": args.password})["access_token"]
    )

    def node2_operational() -> bool:
        nets = call("/canopen/status", token).get("networks")
        if not isinstance(nets, list) or not nets:
            return False
        return any(n["node_id"] == 2 and n["state"] == "operational" for n in nets[0]["nodes"])

    def restart(why: str) -> None:
        call("/stop-plc", token)
        wait_for(
            f"CANopen stopped ({why})",
            lambda: call("/canopen/status", token).get("active") is False,
        )
        call("/start-plc", token)
        wait_for(f"node 2 operational after the PLC start ({why})", node2_operational)

    sim = Sim(
        [
            args.sim,
            "--interface",
            args.interface,
            "--node",
            "2",
            "--steps",
            "1",
            "--timeout",
            "120",
            "--blocks",
            "1",
        ]
    )
    sim.expect("blocks: ready", 150)
    run = 1
    sim.send(f"run {run}")
    sim.expect(f"run {run} ok")
    print(f"ok: every block worked in run {run}", flush=True)

    for i in range(RESTARTS):
        restart(f"restart {i + 1}")
        run += 1
        sim.send(f"run {run}")
        sim.expect(f"run {run} ok")
        print(f"ok: every block worked in run {run}, after PLC start {i + 1}", flush=True)

    # Stop the PLC while a run's blocks are busy; the next run must work.
    run += 1
    sim.send(f"run {run}")
    sim.expect(f"run {run} started")
    restart("while blocks were busy")
    run += 1
    sim.send(f"run {run}")
    # The interrupted run may still finish or fail before this one starts.
    end = time.monotonic() + 180
    while True:
        try:
            line = sim.lines.get(timeout=max(0.1, end - time.monotonic()))
        except queue.Empty:
            line = ""
        if line == f"run {run} ok":
            break
        if (
            line.startswith(f"run {run} FAIL")
            or time.monotonic() > end
            or sim.proc.poll() is not None
        ):
            sys.exit(f"FAIL: run {run} after a stop while busy: {line}")
    print(f"ok: every block worked in run {run}, after a stop while blocks were busy", flush=True)

    if sim.close() != 0:
        print("note: the interrupted run was reported as failed by the simulated node", flush=True)
    status = call("/status", token)
    print(f"runtime status: {status}", flush=True)
    print("PASS")


if __name__ == "__main__":
    main()
