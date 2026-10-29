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
  configured values; 0 skips a value), writes the configuration SDOs in order, and starts the node.
  A node that fails is booted again in the background with a growing delay (1 s up to 16 s); the
  other nodes are not held back.
- **Exchange.** Each TPDO of a node is received into its `%I` locations; each RPDO of a node is
  sent from its `%Q` locations when a value changes, and in full when the node comes up.
- **SYNC.** `none`, `period` (Lely produces SYNC every `period_us`), or `plc_cycle` (the master
  sends the newest outputs and then one SYNC after each PLC cycle).
- **Supervision.** With `heartbeat`, the master consumes the node's heartbeat. A timeout, or a
  boot-up message from a running node, means the node is lost: its state reads 0, its inputs hold
  their last values (`on_loss: hold`) or go to 0 (`on_loss: zero`), its outputs are no longer sent,
  and it is booted again.
- **Node state.** `state_location` (a `%IB`) reads the node's NMT state (CiA 301: 5 operational,
  127 pre-operational, 4 stopped) while it is up, and 0 while it is not.
- **PLC stop.** The runtime zeroes `%Q` and runs one more cycle, so the nodes get zero outputs;
  then the master sends `on_plc_stop` to all nodes (`pre_operational`, the default, or `stopped`,
  or `none`) and closes the interface.
- The master never writes 0x1010 (store parameters) or 0x1011 (restore defaults); the parser
  refuses such SDOs.

## conf/canopen.json

```json
{
  "version": 1,
  "networks": [
    {
      "name": "CANopen 1",
      "interface": "can0",
      "bitrate": 500000,
      "master_node_id": 127,
      "heartbeat_producer_ms": 0,
      "sdo_timeout_ms": 1000,
      "sync": { "mode": "plc_cycle" },
      "on_plc_stop": "pre_operational",
      "nodes": [
        {
          "node_id": 2,
          "name": "io1",
          "identity": { "device_type": 401, "vendor_id": 1, "product_code": 2, "revision": 0 },
          "supervision": { "type": "heartbeat", "consumer_ms": 300 },
          "on_loss": "hold",
          "state_location": "%IB10",
          "sdo": [{ "index": 4119, "subindex": 0, "data": "6400" }],
          "tpdos": [
            {
              "number": 1,
              "cob_id": 386,
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
| `version` | Format version, 1. |
| `interface` | SocketCAN interface, unique per network. |
| `bitrate` | Bit/s, one of the CiA 301 rates (10k to 1M); 0 leaves the link as it is (vcan, or a link set up outside the runtime). |
| `master_node_id` | 1-127, default 127. |
| `heartbeat_producer_ms` | The master's own heartbeat, 0 for none. |
| `sdo_timeout_ms` | Timeout of each configuration SDO, default 1000. |
| `sync.mode`, `sync.period_us` | `none`, `period` (with `period_us`), `plc_cycle`. SYNC uses COB-ID 0x080. |
| `on_plc_stop` | NMT command for all nodes on a PLC stop. |
| `identity` | Expected 0x1000 and 0x1018 sub 1-3 values; a missing or 0 value is not checked. |
| `supervision` | `{"type": "none"}` or `{"type": "heartbeat", "consumer_ms": n}`. The node's own heartbeat producer (0x1017) is set by an SDO in `sdo`. |
| `sdo` | Configuration SDO downloads, in order: `data` is the value's bytes in hexadecimal, least significant byte first, 1 to 1024 bytes. Typically the node's PDO communication and mapping parameters and its heartbeat producer time. |
| `tpdos` / `rpdos` | The node's TPDOs (received by the master, `%I` locations) and RPDOs (sent by the master, `%Q` locations): PDO number on the node, its 11-bit COB-ID, and the mapped entries in mapping order. |
| `type` | `BOOLEAN`, `INTEGER8/16/32/64`, `UNSIGNED8/16/32/64`, `REAL32`, `REAL64`. Its size must equal the location's (`BOOLEAN` `%IX/%QX`, 8 bits `B`, 16 `W`, 32 `D`, 64 `L`). |

The plugin refuses the whole file, with one log line per problem, when anything is wrong: an
unknown version, a duplicate interface or node ID, a node ID equal to the master's, a non-standard
bit rate, a reserved or duplicate COB-ID, a PDO over 64 bits, a type that does not fit its
location, a `%Q` location in a TPDO or a `%I` location in an RPDO, two entries on the same `%I`
location, or a location outside the runtime's I/O image. Two entries on the same `%Q` location are
a warning.

## Running

- The interface must exist. Setting the bit rate and bringing the link up needs `CAP_NET_ADMIN`
  (the native install runs as root); without it, set the link up yourself
  (`ip link set can0 up type can bitrate 500000`) and give `bitrate: 0` or the same rate.
- In Docker the container needs the host's network namespace (`--network host`) to see `can0`, and
  `--cap-add NET_ADMIN` for the plugin to configure it.
- An image built from this fork needs a semantic version in `RUNTIME_VERSION`
  (`docker build --build-arg RUNTIME_VERSION=v4.3.1-canopen.<commit> ...`): the editor refuses to
  upload to a runtime whose version it cannot compare.
- While the interface is missing or down the master logs it once and retries every second.
- Messages go to the runtime log with the `[CANOPEN]` prefix and the interface name.
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

`-DLELY_PREFIX=<dir>` uses a Lely already installed there instead of fetching it.

| Test | What it covers |
|---|---|
| `test_config` | Parser and validator of `conf/canopen.json`. |
| `test_master_dcf` | The master's own object dictionary (loaded into Lely), the concise DCF of the configuration SDOs, PDO bit packing. |
| `test_network` | The master against simulated generic I/O modules on Lely's in-process CAN bus: boot with identity check and configuration SDO, inputs, outputs, node state, loss and recovery, a node with the wrong identity, and the NMT command on stop. With `CANOPEN_TEST_INTERFACE=vcan0` it runs on a SocketCAN interface instead. |

`canopen_sim_slave --interface vcan0 --node 2` (built with the tests) is a simulated generic I/O
module: it sets its digital inputs to 1, 2, ... and waits each time until its digital outputs
follow, so it checks a PLC program that copies one to the other.

The CI workflow `.github/workflows/canopen.yml` runs these tests, the SocketCAN variant on a vcan
link, and an end-to-end check: it installs the runtime from source, builds the OpenPLC Editor fork
(the branch with this branch's name, else `canopen`), uploads its `e2e/fixtures/canopen-generic-io`
project with the editor's command line, and runs the simulated module against it on `vcan0`.

vcan needs the kernel's `vcan` module. On a runner whose kernel has no matching
`linux-modules-extra` package the vcan jobs fail at their first step, and the reason is in its log.
