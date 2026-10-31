# CANopen

The `canopen` native plugin is a CANopen master (CiA 301/302) on a Linux SocketCAN interface. It
runs inside `plc_main`, exchanges PDOs with the nodes once per PLC cycle, and is built on
[Lely CANopen](https://gitlab.com/lely_industries/lely-core) (Apache-2.0), which is fetched at a
pinned commit and linked statically into the plugin.

```
Editor upload ──► webserver ──► conf/canopen.json ──► plc_main (canopen plugin) ◄── can0 ──► nodes
```

The OpenPLC Editor writes `conf/canopen.json` from the project's CANopen device: it reads each
node's EDS, lets the user map PDO entries to IEC locations, and resolves everything the master
needs into this one file. The runtime never reads an EDS. A project without a CANopen device
uploads no `conf/canopen.json`, and the plugin stays disabled.

## What the master does

- **Boot.** For every node, the NMT master checks the identity (0x1000, 0x1018 against the
  configured values; 0 skips a value), restores the node's defaults when `restore_defaults` is set,
  writes the configuration SDOs in order, and starts the node. A node that fails is booted again in
  the background with a growing delay (1 s up to 16 s); the other nodes are not held back.
- **Restore defaults.** With `restore_defaults` n (1 all parameters, 2 communication, 3
  application, 4-127 manufacturer specific), the master writes `load` to 0x1011 sub n after the
  identity check, resets the node (reset communication for 2, reset node otherwise) so the
  defaults take effect, waits for its boot-up, checks the identity again and then writes the
  configuration SDOs. This happens once each time the node is brought up (PLC start, or a reboot
  after the node was lost). Nothing is stored in the node: 0x1010 is never written.
- **No initialization.** A node with `configure: false` is not checked and gets no SDO at all; the
  master only starts it with NMT and supervises it. It must already use the PDO layout and timings
  of the configuration.
- **Missing nodes.** A node that has not booted is looked for again about every second, which also
  finds a node that was connected later or whose boot-up message the master missed, and a node
  that kept running while the master's reset never reached it (only its heartbeat arrives). A node not
  booted `boot_timeout_ms` after the start is reported once: as not answering (a warning) when it is
  mandatory, as absent (information) when it is `optional`. Mandatory and optional differ only in
  that report: no node, mandatory or not, holds back the others.
- **Exchange.** Each TPDO of a node is received into its `%I` locations; each RPDO of a node is
  sent from its `%Q` locations right after the PLC cycle that changed a value, and in full when the
  node comes up.
- **SYNC.** `none`, `period` (Lely produces SYNC every `period_us`), or `plc_cycle` (the master
  sends the newest outputs and then one SYNC after each PLC cycle).
- **Supervision.** With `heartbeat`, the master consumes the node's heartbeat; with `guarding`, the
  master guards the node with remote frames on 0x700 + node ID every `guard_time_ms` and expects an
  answer within `guard_time_ms` times `life_time_factor` (the node's own 0x100C and 0x100D are set
  by SDOs in `sdo`). Node guarding needs a CAN controller and driver that handle remote frames; use
  heartbeat when the node has it. A timeout, or a boot-up message from a running node, means the
  node is lost: its state reads 0, its inputs hold their last values (`on_loss: hold`) or go to 0
  (`on_loss: zero`), its outputs are no longer sent, and it is booted again
  (`on_node_error: restart`), or sent NMT stop and left alone until the next PLC start
  (`on_node_error: stop`); a new bus session in the same PLC run (after a lost link) stops it again
  when it boots.
- **Node state.** `state_location` (a `%IB`) reads the node's NMT state (CiA 301: 5 operational,
  127 pre-operational, 4 stopped) while it is up, and 0 while it is not.
- **Receive timeout.** A TPDO with `receive_timeout_ms` is late when no frame of it arrives for that
  long while the node is operational. The master logs it once, holds or zeroes its inputs as
  `on_loss` says, and keeps the node running; the inputs follow again with the next frame.
- **Emergencies.** The master consumes each node's EMCY messages on its `emcy.cob_id` (default
  0x80 + node ID), logs them (at most one line per node and second) and keeps the last ones in the
  status. With `emcy.enabled: false` the node's emergencies are ignored.
- **TIME.** With `time`, the master sends the TIME object (CiA 301 TIME_OF_DAY: milliseconds after
  midnight and days since 1 January 1984, UTC, from the PLC's clock) on `time.cob_id` every
  `time.period_ms` while the PLC runs.
- **PLC stop.** The runtime zeroes `%Q` and runs one more cycle, so the nodes get zero outputs;
  then the master sends `on_plc_stop` to all nodes (`pre_operational`, the default, or `stopped`,
  or `none`) and closes the interface.
- The master never writes 0x1010 (store parameters), and writes 0x1011 (restore defaults) only
  through `restore_defaults`; the parser refuses configuration SDOs to either.

## conf/canopen.json

```json
{
  "version": 3,
  "networks": [
    {
      "name": "CANopen 1",
      "interface": "can0",
      "bitrate": 500000,
      "master_node_id": 127,
      "heartbeat_producer_ms": 100,
      "sdo_timeout_ms": 1000,
      "priority": 90,
      "sync": { "mode": "plc_cycle" },
      "on_plc_stop": "pre_operational",
      "boot_timeout_ms": 2000,
      "on_node_error": "restart",
      "time": { "cob_id": 256, "period_ms": 1000 },
      "nodes": [
        {
          "node_id": 2,
          "name": "io1",
          "optional": false,
          "configure": true,
          "restore_defaults": 0,
          "emcy": { "enabled": true, "cob_id": 130 },
          "identity": { "device_type": 401, "vendor_id": 1, "product_code": 2, "revision": 0 },
          "supervision": { "type": "heartbeat", "consumer_ms": 300 },
          "on_loss": "hold",
          "state_location": "%IB10",
          "sdo": [{ "index": 4119, "subindex": 0, "data": "6400" }],
          "tpdos": [
            {
              "number": 1,
              "cob_id": 386,
              "receive_timeout_ms": 250,
              "entries": [
                { "index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB0" },
                { "index": 25601, "subindex": 1, "type": "INTEGER16", "location": "%IW0" }
              ]
            }
          ],
          "rpdos": [
            {
              "number": 1,
              "cob_id": 514,
              "entries": [
                { "index": 25088, "subindex": 1, "type": "UNSIGNED8", "location": "%QB0" }
              ]
            }
          ]
        }
      ]
    }
  ]
}
```

| Field | Meaning |
|---|---|
| `version` | Format version, 3. Older files are refused; build the project again with a matching editor. |
| `interface` | SocketCAN interface, unique per network. |
| `bitrate` | Bit/s, one of the CiA 301 rates (10k to 1M); 0 leaves the link as it is (vcan, or a link set up outside the runtime). |
| `master_node_id` | 1-127, default 127. |
| `heartbeat_producer_ms` | The master's own heartbeat, 0 for none. Nodes that watch it (0x1016, written as a configuration SDO) react when the runtime stops or dies. |
| `priority` | SCHED_FIFO priority of the network's bus thread while the master runs, 1-97, default 90. |
| `sdo_timeout_ms` | Timeout of each SDO the master sends to a node while it boots and configures it (identity check, restore, configuration), default 1000. |
| `sync.mode`, `sync.period_us` | `none`, `period` (with `period_us`), `plc_cycle`. SYNC uses COB-ID 0x080. |
| `on_plc_stop` | NMT command for all nodes on a PLC stop. |
| `boot_timeout_ms` | 100-60000, default 2000: a node not booted this long after the start is reported as not answering (mandatory) or absent (optional). |
| `on_node_error` | `restart` (default): boot a lost node again; `stop`: send it NMT stop and leave it until the next PLC start. |
| `time` | Absent or `null` for no TIME; else `cob_id` (default 256, 0x100) and `period_ms` (10-65535, default 1000). |
| `optional` | `true`: an absent node is not an error. Default `false`. |
| `configure` | `false`: no identity check, restore or configuration SDOs (`sdo` must be empty); the node is only started. Default `true`. |
| `restore_defaults` | 0x1011 sub-index to restore before the configuration, 1-127; 0 or absent for none. Not with `configure: false`. |
| `emcy` | `enabled` (default `true`) and `cob_id` (11-bit, default 0x80 + node ID) of the node's emergency messages. |
| `identity` | Expected 0x1000 and 0x1018 sub 1-3 values; a missing or 0 value is not checked. |
| `supervision` | `{"type": "none"}`, `{"type": "heartbeat", "consumer_ms": n}` or `{"type": "guarding", "guard_time_ms": n, "life_time_factor": n}` (1-65535 and 1-255). The node's own heartbeat producer (0x1017) or guard time and life time factor (0x100C, 0x100D) are set by SDOs in `sdo`. |
| `sdo` | Configuration SDO downloads, in order: `data` is the value's bytes in hexadecimal, least significant byte first, 1 to 1024 bytes. Typically the node's PDO communication and mapping parameters and its heartbeat producer time. |
| `tpdos` / `rpdos` | The node's TPDOs (received by the master, `%I` locations) and RPDOs (sent by the master, `%Q` locations): PDO number on the node, its 11-bit COB-ID, and the mapped entries in mapping order. |
| `receive_timeout_ms` | TPDOs only: report the PDO late after this many milliseconds without a frame, 1-65535; absent or 0 for no timeout. It is the master RPDO's event timer (0x1400 + n sub 5). |
| `type` | `BOOLEAN`, `INTEGER8/16/32/64`, `UNSIGNED8/16/32/64`, `REAL32`, `REAL64`. Its size must equal the location's (`BOOLEAN` `%IX/%QX`, 8 bits `B`, 16 `W`, 32 `D`, 64 `L`). |

The plugin refuses the whole file, with one log line per problem, when anything is wrong: an
unknown version, a duplicate interface or node ID, a node ID equal to the master's, a non-standard
bit rate, a reserved or duplicate COB-ID (PDOs, emergencies and TIME), a configuration SDO to
0x1010 or 0x1011, restore defaults or configuration SDOs on a node with `configure: false`, a PDO
over 64 bits, a type that does not fit its
location, a `%Q` location in a TPDO or a `%I` location in an RPDO, two entries on the same `%I`
location, or a location outside the runtime's I/O image. Two entries on the same `%Q` location are
a warning.

## Status

While a CANopen program runs, the plugin keeps a live status of every network and serves it on
the Unix socket `/run/runtime/canopen.socket` (mode 0600). A client sends one JSON request per line
and gets one JSON answer per line; `{"op": "status"}` returns the status. The socket is there while
the plugin is loaded; with no CANopen network running the status is `{"version": 1, "active":
false}`. The other requests are the [online tools](#online-tools). An unknown op is answered
`{"error": "unknown_op"}` and a malformed request `{"error": "bad_request"}`. The plugin needs
nothing else for it, so the socket works the same in a runtime without the web routes below.

The web server returns the same answer at `GET /api/canopen/status` (login required), or
`{"active": false}` when the plugin is not loaded, and lists `"features": ["status", "sdo", "nmt",
"service"]` in the `canopen` entry of `/api/capabilities`. The OpenPLC Editor shows the status on
the CANopen device only when `status` is listed, and its online tools only when the others are.

```json
{
  "version": 1, "active": true, "now": 1793300000000,
  "networks": [{
    "name": "CANopen 1", "interface": "can0",
    "bus": {"state": "error_active", "tx_errors": 0, "rx_errors": 0, "bus_off_count": 0,
            "rx_frames": 1520, "tx_frames": 1488, "rx_dropped": 0, "tx_dropped": 0,
            "load_percent": 3.1,
            "link_resets": 0},
    "all_mandatory_up": false, "all_optional_up": true,
    "nodes": [{
      "node_id": 3, "name": "io2", "optional": false, "state": "failed", "nmt": "pre_operational",
      "since": 1793299990000, "boot_attempts": 2, "losses": 0, "warning": false,
      "problem": {"kind": "identity", "field": "product_code", "expected": 66, "found": 67,
                  "boot_status": "M"},
      "emcy": [{"time": 1793299995000, "code": 8960, "register": 3, "data": "0000000000",
                "active": true}],
      "late_pdos": []
    }],
    "events": [{"time": 1793299990000, "node": 3, "kind": "boot_failed", "problem": {"kind": "identity"}}]
  }]
}
```

| Field | Meaning |
|---|---|
| `now`, `since`, `time` | Wall-clock milliseconds since the epoch. |
| `bus.state` | `error_active`, `error_warning`, `error_passive`, `bus_off`, `stopped`, `down` (interface down), `missing` (no such interface), `unknown`. A vcan link is `error_active`. |
| `bus.*` | Error counters, bus-off count, frame counts since the PLC started and an estimated bus load in percent (standard frames, without bit stuffing). Each is left out when the driver does not report it. `rx_dropped` and `tx_dropped` count dropped receive and transmit frames. `link_resets` counts the times the master reset a CAN link. |
| `all_mandatory_up`, `all_optional_up` | Every mandatory (optional) node is operational; `true` when there is none. |
| `nodes[].state` | `waiting` (master started, not heard yet), `silent` (mandatory, not booted `boot_timeout_ms` after the start), `absent` (optional, not booted by then), `booting` (also while it restores its defaults), `failed` (the last boot failed; retried in the background), `operational`, `lost` (retried in the background unless `held`), `stopped` (PLC stop), `manual` (stopped or pre-operational by a client's NMT command), `service` (service mode: nobody manages the node). |
| `nodes[].held` | Present and `true` when the node was lost with `on_node_error: stop` and stays stopped until the next PLC start. |
| `nodes[].nmt` | Last NMT state seen: `pre_operational` (also after a boot-up), `operational`, `stopped`, `unknown`. In service mode it follows the NMT commands sent and the nodes' heartbeats and boot-ups; a reset makes it `unknown` until the node is heard again. |
| `networks[].mode` | `service` while [service mode](#service-mode) is open; left out otherwise. |
| `nodes[].problem` | Why the node is not running, or `null`: `no_response`, `identity` (with `field`, `expected` and the `found` value read from the node), `sdo_abort` (a configuration SDO or the restore write refused, with `index`, `subindex` and `abort_code`), `heartbeat`, `guarding`, `reset`, `boot_error` (with the stack's `text`). `boot_status` is the CiA 302-2 boot error letter. |
| `nodes[].warning` | An operational node with an active emergency or a late PDO. |
| `nodes[].emcy` | The last 8 emergencies, newest first; an emergency with code 0 (error reset) marks them inactive. |
| `nodes[].late_pdos` | TPDOs with a receive timeout whose frames are late. |
| `events` | The last 64 events of the network, newest first: `boot_ok`, `boot_failed`, `silent`, `absent`, `lost`, `emcy`, `emcy_reset`, `sdo_abort`, `pdo_late`, `pdo_back`, `pdo_length`, `bus_state`, `bus_reset` (the master reset the link, with `outcome` `done`, `not_permitted` or `failed`, and `cause` `bus_off`, `bit_rate` or `tx_stalled`), `sdo_write` (a client's SDO write, with `index`, `subindex`, `ok` and the `abort_code` of a refusal), `nmt_command` (a client's NMT command, with `command`), `service` (service mode opened or closed, with `open`). |

Counters and lists start again when the PLC starts. The status thread never blocks the PLC scan
or the bus: each answer is built from a copy taken under a short lock.

## Online tools

The OpenPLC Editor reads and writes a node's objects (SDO) and sends it NMT commands while the PLC
runs, and in service mode while it is stopped. The requests go over the plugin's socket, or through
`POST /api/canopen/request` (login required), which forwards a JSON body with one of the ops below
and returns the plugin's answer unchanged, its refusals included; `{"active": false}` when the
plugin is not loaded, 400 for another op or a body that is not a JSON object, 502 when the plugin
gives no usable answer.

| Request | Answer |
|---|---|
| `{"op": "sdo_read", "network": 0, "node": 2, "index": 4120, "subindex": 1}` | `{"job": 41}` |
| `{"op": "sdo_write", "network": 0, "node": 2, "index": 8449, "subindex": 0, "data": "1400"}` | `{"job": 42}` |
| `{"op": "job", "job": 41}` | `{"state": "pending"}`, `{"state": "done", "data": "34120000"}` (a read), `{"state": "done"}` (a write or NMT), `{"state": "failed", "abort_code": 134217762}` (the node's SDO abort, or 0x05040000 for a timeout) or `{"state": "failed", "reason": "booting"}` |
| `{"op": "nmt", "network": 0, "node": 2, "command": "pre_operational"}` | `{"ok": true}` |
| `{"op": "service_open", "name": "CANopen 1", "interface": "can0", "bitrate": 250000, "master_node_id": 127, "sdo_timeout_ms": 1000, "nodes": [2, 3]}` | `{"ok": true, "network": 0}` |
| `{"op": "service_close"}` | `{"ok": true}` |

- `network` is the network's index in `canopen.json`; `node` must be one of its nodes (1 to 127).
  `data` is the value's bytes in hex, little-endian as on the bus, at most 64 KiB.
- An SDO request is a job: the answer comes at once with the job's id, and the client asks for its
  result with `job` until it is no longer `pending`. Results are kept 30 seconds. The bus thread
  serves one request at a time, between its own work, through the master's own SDO client, so a
  request never collides with the master's boot SDOs. A node that the master is booting is answered
  `reason: booting`; try again once it is up.
- Refusals: `bad_request`, `unknown_op`, `too_large`, `not_running` (no network runs),
  `unknown_network`, `unknown_node`, `queue_full` (16 requests waiting), `unknown_job`.
- `command` is `start`, `stop`, `pre_operational`, `reset_node` or `reset_communication`; `node` 0
  sends it to each node of the network in turn. A node that a client stops or puts in
  pre-operational is `manual`: its missing PDOs are not reported as late, its inputs follow
  `on_loss` (`zero` zeroes them), it is not booted again, and its heartbeat is still watched.
  `start` puts it back to `operational` (or boots it again when it was held or not up). A reset is
  not a loss: the master boots and configures the node again when its boot-up arrives.
- Each write and NMT command is logged and recorded as an `sdo_write` or `nmt_command` event.
- Saving to and restoring from a node's non-volatile memory is a plain write to 0x1010 ("save",
  73617665) or 0x1011 ("load", 6c6f6164). The master itself never writes 0x1010.

### Service mode

Service mode runs the online tools while the PLC is stopped, with no program uploaded. A client
opens it with `service_open` and the CAN settings to use; the plugin then runs a master on that
interface (under `name` in the status, `Service` when left out) that sends nothing of its own (no NMT reset, start or stop, no SYNC, no boot, no
configuration) and only serves the requests, with the NMT commands sent as plain frames. The nodes
listed are the ones requests may address, all in state `service`; their `nmt` follows the commands
sent and the heartbeats and boot-ups the plugin hears. It is refused with `plc_running`
while the PLC runs CANopen, and `busy` while it is already open. A PLC start closes it first, as do
`service_close` and five minutes without a request. Opening and closing it are logged and recorded
as `service` events.

## Installing a release

Releases of this fork are tagged `v<upstream version>-canopen.<N>` (for example `v4.3.1-canopen.1`)
and published as the image `ghcr.io/tonihoohoo/openplc-runtime` for `linux/arm64` and
`linux/amd64`, with `latest` on the newest release. Install with the Docker installer, pointed at
that image:

```bash
curl -fsSL https://raw.githubusercontent.com/tonihoohoo/openplc-runtime-canopen/canopen/scripts/install-docker.sh \
  | sudo RUNTIME_REPOSITORY=ghcr.io/tonihoohoo/openplc-runtime bash -s -- --runtime-version v4.3.1-canopen.1
```

The installer records the image repository on the device, so the OpenPLC Editor's
**Change version** dialog then lists this fork's releases and switches between them; picking an
earlier tag rolls back. The runtime container runs privileged on the host network, so it sees
`can0`. Leave out `--runtime-version` to install `latest`.

**Upload the project again after installing or changing version.** Every install, version change
or rollback creates a new runtime container, and the compiled program, `canopen.json` and the
plugin's entry in `plugins.conf` live inside the container, so the PLC starts empty with CANopen off.
The data directory keeps only the editor's project snapshot, which the runtime does not rebuild
from. This is upstream behaviour and affects every program, not only CANopen ones.

`/api/capabilities` reports `"canopen": {"configVersions": [...], "features": [...]}` when the
plugin is built, and the editor refuses to upload a CANopen project to a runtime without it or
without the project's config version.

Each release image keeps the plugin's licence and Lely's `LICENSE` and `NOTICE` in
`/workdir/core/src/drivers/plugins/native/canopen/build/licenses/`. How releases are made is in
[CANOPEN_RELEASES.md](CANOPEN_RELEASES.md).

## Timing

Like the EtherCAT relay, each network's bus thread runs at real-time priority while its master
runs: SCHED_FIFO at `priority` (default 90: above every IEC task, at most 49, and below the
runtime's dispatcher at 98) with 1 ns timer slack. Preparing the interface and waiting to retry run
at normal priority. Without permission for real-time scheduling (a container without `SYS_NICE`)
the master logs one warning and runs at normal priority.

The PLC side stays out of the way: `cycle_start` copies the newest inputs into `%I` through the
runtime's journal, and `cycle_end` reads `%Q` under the image lock, publishes it and wakes the bus
thread with one non-blocking `eventfd` write. Neither allocates, logs or waits for the bus thread.
The bus thread then sends the changed outputs at once, followed by SYNC with `plc_cycle`, so every
PLC cycle is followed by exactly one SYNC. A 10 ms timer remains for boot retries.

If the runtime stops or dies, the master's heartbeat stops. A node that consumes it (0x1016) goes to
the state its 0x1029 sets, pre-operational by default; what it then does with its outputs is set in
the device's own objects.

### Link recovery

When the master configures a CAN link (bit rate, or bringing it up), it turns on the controller's
bus-off restart (`restart-ms` 100). Some drivers, such as `gs_usb` for USB adapters, have no such
restart and refuse the setting; the master then configures the link without it and logs that once.

When a link is bus-off and the kernel does not restart it (the restart is off, the driver has
none, or the link is still bus-off 2 s after the restart delay), the master resets the link
itself: it takes the link down and up, and brings the nodes up again, without a PLC restart. The
bus state is read once a second, so a reset follows the bus-off within about a second. If the bus
goes bus-off again within 10 s of a reset, the next reset waits 1 s, then 2, 4, 8 and at most 10 s;
after 60 s without bus-off the next reset is immediate again.

The master resets the link in the same way, with the same back-off, in two more cases:

- **Wrong bit rate.** The configuration sets a bit rate and the link reports another one, for
  example after someone changed it by hand. The link comes back at the configured rate. With
  `bitrate: 0` the link's rate is left alone.
- **Stalled transmitter.** The link is error-active or error-warning, has sent no frame for 3 s and,
  in that time, either dropped transmit frames or the master's writes failed with none confirmed as
  sent. The second case catches an adapter that takes frames and never sends them, which the kernel
  does not count as dropped (`tx_dropped` stays 0). A quiet network writes and drops nothing, and an
  unplugged cable gives error-passive, so neither counts.

Each reset is logged, counted in the status (`link_resets`) and recorded as a `bus_reset` event
with its cause.

Resetting the link needs `CAP_NET_ADMIN`. Without it the master logs once the commands to reset it
by hand (`ip link set can0 down`, then `up`) and picks up the link again when someone does.

A real-time (PREEMPT_RT) kernel and CPU isolation are deployment choices, as for the runtime
itself; they lower the worst-case delays, not the typical ones.

## Running

- The interface must exist. Setting the bit rate and bringing the link up needs `CAP_NET_ADMIN`
  (the native install runs as root); without it, set the link up yourself
  (`ip link set can0 up type can bitrate 500000`) and give `bitrate: 0` or the same rate.
- In Docker the container needs the host's network namespace (`--network host`) to see `can0`, and
  `--cap-add NET_ADMIN` for the plugin to configure it.
- An image built by hand from this fork needs a semantic version in `RUNTIME_VERSION`
  (`docker build --build-arg RUNTIME_VERSION=v4.3.1-canopen.<commit> ...`): the editor refuses to
  upload to a runtime whose version it cannot compare. Keep such tags off the repository: a commit
  hash sorts above every numbered release of the same base.
- While the interface is missing or down the master logs it once and retries every second.
- Messages go to the runtime log with the `[CANOPEN]` prefix and the interface name.
  A failed boot or configuration SDO is reported once per node, and a node that has not answered
  once until it boots; the CANopen stack's own lines for each failed SDO or frame are logged at
  debug level only (link faults are reported by the bus state).
- Existing installs keep their `plugins.conf`; add the `canopen` line from `plugins_default.conf`
  to it, or remove it so that it is created again.

## Building and testing

`install.sh` builds the plugin with the other native plugins. Lely is built from source, which
needs `git`, `autoconf`, `automake` and `libtool` (the dependency step installs them). By hand:

```bash
cd core/src/drivers/plugins/native/canopen
cmake -B build -DCANOPEN_BUILD_TESTS=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

`-DLELY_PREFIX=<dir>` uses a Lely already installed there instead of fetching it; a Lely installed
in `lely-prefix/` next to `CMakeLists.txt` is used by default (the release workflow caches one there).

| Test | What it covers |
|---|---|
| `test_bus_off_recovery` | When the master resets a faulty link: bus-off with and without a kernel restart, a wrong bit rate or a stall, and the back-off across causes. |
| `test_can_link` | What the master does to the CAN link before opening it: bit rate, bus-off auto-restart (also on a driver that refuses it), bringing it up; the link statistics read from netlink. |
| `test_config` | Parser and validator of `conf/canopen.json`. |
| `test_entry_points` | The plugin's C entry points with an allocation failure injected at each point of `start_loop`: no exception leaves them and a failed start leaves nothing running. |
| `test_link_stall` | When a link counts as stalled: no frame sent for 3 s with drops or failing writes, never while quiet, error-passive or bus-off. |
| `test_master_dcf` | The master's own object dictionary (loaded into Lely), the concise DCF of the configuration SDOs, PDO bit packing. |
| `test_network` | The master against simulated generic I/O modules on Lely's in-process CAN bus: boot with identity check and configuration SDO, inputs, outputs, one SYNC right after each cycle's outputs with no allocation in the scan hooks, node state, loss and recovery, a node with the wrong identity, a refused configuration SDO, a node that never answers, a running node that missed the master's reset, emergencies, late PDOs, the status and its socket, the online tools (SDO reads and writes, a refused write,
pre-operational, save, start and reset of a node), service mode, the NMT command on stop, and the reset of a link that is bus-off, at a wrong bit rate or stalled (on a stood-in link state). With `CANOPEN_TEST_INTERFACE=vcan0` it runs on a SocketCAN interface instead. |
| `test_status` | The status record, its JSON answer and requests, link parsing and the bus monitor. |

`canopen_sim_slave --interface vcan0 --node 2` (built with the tests) is a simulated generic I/O
module: it sets its digital inputs to 1, 2, ... and waits each time until its digital outputs
follow, so it checks a PLC program that copies one to the other.

The CI workflow `.github/workflows/canopen.yml` runs these tests, the SocketCAN variant on a vcan
link, and an end-to-end check: it installs the runtime from source, builds the OpenPLC Editor fork
(the branch with this branch's name, else `canopen`), uploads its `e2e/fixtures/canopen-generic-io`
project with the editor's command line, and runs the simulated module against it on `vcan0`.

vcan needs the kernel's `vcan` module. On a runner whose kernel has no matching
`linux-modules-extra` package the vcan jobs fail at their first step, and the reason is in its log.
