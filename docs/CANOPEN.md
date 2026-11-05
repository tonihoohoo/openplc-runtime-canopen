# CANopen

The `canopen` native plugin is a CANopen master (CiA 301/302) on a Linux SocketCAN interface. It
runs inside `plc_main`, exchanges PDOs with the nodes once per PLC cycle, and is built on
[Lely CANopen](https://gitlab.com/lely_industries/lely-core) (Apache-2.0), which is fetched at a
pinned commit and linked statically into the plugin. The same plugin can also make the runtime a
[CANopen slave](#canopen-slave) (CiA 301) on another interface, built on
[CANopenNode](https://github.com/CANopenNode/CANopenNode) (Apache-2.0) at its newest release.

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
- **Start by the PLC program.** A node with `start: program` is booted and configured as any
  other, then left pre-operational: its status is `waiting_start`, its heartbeat or guarding is
  supervised and SDOs reach it, but no PDO is exchanged and it is not counted as lost. The PLC
  program (the `NMT` block) or a client starts it with NMT start; a node that started itself is
  sent NMT pre-operational. It waits again after every new boot (after a loss, a reset or the next
  PLC start). Other nodes are started by the plugin once booted (the master's 0x1F80 has bit 3
  set, so Lely itself starts none).
- **Nodes off.** A node with `enabled: false` is off at PLC start: it is out of the master's
  network list (0x1F81 bit 0), so it is not looked for, booted, supervised or reported, gets no
  PDOs, and NMT to every node skips it. The PLC program turns a node on or off while the PLC runs
  (`CANOPEN_NODE_ENABLE`): off sends a node that is up NMT stop and holds or zeroes its inputs
  as `on_loss` says, without a loss; on looks for the node again and boots it like a node that
  appears. The program's choice lasts until the PLC stops.
- **Exchange.** Each TPDO of a node is received into its `%I` locations; each RPDO of a node is
  sent from its `%Q` locations right after the PLC cycle that changed a value, and in full when the
  node comes up.
- **SYNC.** `none`, `period` (Lely produces SYNC every `period_us`), or `plc_cycle` (the master
  sends the newest outputs and then one SYNC after each PLC cycle).
- **Supervision.** With `heartbeat`, the master consumes the node's heartbeat; with `guarding`, the
  master guards the node with remote frames on 0x700 + node ID every `guard_time_ms` and expects an
  answer within `guard_time_ms` times `life_time_factor` (the node's own 0x100C and 0x100D are set
  by SDOs in `sdo`). Node guarding needs a CAN controller and driver that handle remote frames; use
  heartbeat when the node has it. A timeout, a boot-up message from a running node, or a running
  node that reports pre-operational or stopped without a command from the master or a client
  (another device's NMT command, for example), means the node is lost (a stopped node is sent NMT
  pre-operational before it is booted again, since it answers no SDO): its state reads 0, its inputs hold their last values (`on_loss: hold`) or go to 0
  (`on_loss: zero`), its outputs are no longer sent, and it is booted again
  (`on_node_error: restart`), or sent NMT stop and left alone until the next PLC start
  (`on_node_error: stop`); a new bus session in the same PLC run (after a lost link) stops it again
  when it boots.
  A lost link, or a bus session that fails with an error and is retried, takes every node down in
  the same way.
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
- **Node IDs over LSS.** A node with `lss` gets its node ID from the master before it boots; see
  [Node IDs over LSS](#node-ids-over-lss).

## Node IDs over LSS

A device that supports LSS (CiA 305, `LSS_Supported=1` in its device file) can get its node ID from
the master instead of from switches or its own memory. The node in the configuration names the
device by its LSS address, the four values of its 0x1018: vendor ID, product code, revision and
serial number (the serial number tells two identical devices apart). The revision may be left
out (0): the master then tries the identity's revision first and otherwise searches the revision
with an LSS slowscan, the other three values fixed.

- **At every PLC start**, before any node boots, the master looks for each enabled node with `lss`
  in turn: it switches the device with that address to LSS configuration, asks for its node ID
  and sets the node's ID only when it differs. Then it switches every device back to waiting, so a
  device that had no node ID (255) starts with the new one; a device that had another ID is sent
  NMT reset communication on its old ID, so that it boots with the new one. The boot then goes on
  as for any node.
- **Store.** With `store: true` the master also asks the device to store its node ID (LSS store
  configuration), so it keeps it after a power-off. It does so when it changed the ID, and once
  while the plugin is loaded when the device already had the right ID, since LSS cannot tell
  whether that ID is in the device's memory; it is not stored again at every PLC start. A refused
  store is logged and reported (`store_refused`); the node still boots with its ID until the next
  power-off. Without `store` the device loses the ID at power-off and gets it again at the next
  PLC start.
- **Not found.** When no device answers with the address, the master logs it, reports
  `not_found` and looks again before every boot attempt of the node, with the same growing pause
  (1 s up to 16 s); the other nodes are not held back. A node that is lost, or turned on by the
  program, gets the assignment again before it boots.
- **Refused.** A device that refuses the node ID is reported (`refused`) and tried again as above.

The master never changes a device's bit rate over LSS by itself (see [LSS tools](#lss-tools)), and only nodes with `lss` take part. Lely's LSS master is created only when at least one node
has `lss`.

## conf/canopen.json

```json
{
  "version": 10,
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
          "start": "master",
          "enabled": true,
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

The whole file is refused when any value is outside its range. The editor holds a copy of these
ranges (`runtime-ranges.test.ts` in its CANopen tests) and refuses the same values before a build;
change both together.

| Field | Meaning |
|---|---|
| `version` | Format version, 10. Older files are refused; build the project again with a matching editor. |
| `networks`, `devices` | The master networks and the [slave devices](#canopen-slave); either list may be empty or left out, not both. |
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
| `start` | Required. `master`: started once booted; `program`: left pre-operational until the PLC program or a client starts it. |
| `enabled` | Required. `false`: the node is off at PLC start until the PLC program turns it on. |
| `optional` | `true`: an absent node is not an error. Default `false`. |
| `configure` | `false`: no identity check, restore or configuration SDOs (`sdo` must be empty); the node is only started. Default `true`. |
| `restore_defaults` | 0x1011 sub-index to restore before the configuration, 1-127; 0 or absent for none. Not with `configure: false`. |
| `emcy` | `enabled` (default `true`) and `cob_id` (11-bit, default 0x80 + node ID) of the node's emergency messages. |
| `identity` | Expected 0x1000 and 0x1018 sub 1-3 values; a missing or 0 value is not checked. |
| `lss` | Absent for a node with a fixed node ID. Else the device's LSS address and options: `vendor_id`, `product_code` and `serial_number` (required, the serial number not 0), `revision` (0 or absent: any revision) and `store` (`true`: store the node ID in the device; default `false`). See [Node IDs over LSS](#node-ids-over-lss). |
| `supervision` | `{"type": "none"}`, `{"type": "heartbeat", "consumer_ms": n}` or `{"type": "guarding", "guard_time_ms": n, "life_time_factor": n}` (1-65535 and 1-255). The node's own heartbeat producer (0x1017) or guard time and life time factor (0x100C, 0x100D) are set by SDOs in `sdo`. |
| `sdo` | Configuration SDO downloads, in order: `data` is the value's bytes in hexadecimal, least significant byte first, 1 to 1024 bytes. Typically the node's PDO communication and mapping parameters and its heartbeat producer time. An item may give `on_abort`: `stop` (default: a refusal fails the node's boot), `continue` (go on with the next item) or `jump` with `jump_to`, the 0-based index of a later item of the same list. A refusal with `continue` or `jump` is logged, recorded as an `sdo_abort` event with its `policy`, and listed under the node's `sdo_skipped`. |
| `tpdos` / `rpdos` | The node's TPDOs (received by the master, `%I` locations) and RPDOs (sent by the master, `%Q` locations): PDO number on the node, its 11-bit COB-ID, and the mapped entries in mapping order. |
| `send_period_ms` | RPDOs only: send the PDO again when it has not been sent for this many milliseconds, 1-65535, also when no output changed; absent for none. |
| `location` | The entry's IEC location. Only a dummy entry (index 0x0001-0x0007, sub-index 0, of the dummy's type) leaves it out: its bits are skipped in a received PDO and sent as zeros. |
| `receive_timeout_ms` | TPDOs only: report the PDO late after this many milliseconds without a frame, 1-65535; absent or 0 for no timeout. It is the master RPDO's event timer (0x1400 + n sub 5). |
| `type` | `BOOLEAN`, `INTEGER8/16/32/64`, `UNSIGNED8/16/32/64`, `REAL32`, `REAL64`. Its size must equal the location's (`BOOLEAN` `%IX/%QX`, 8 bits `B`, 16 `W`, 32 `D`, 64 `L`). |

The plugin refuses the whole file, with one log line per problem, when anything is wrong: an
unknown version, a duplicate interface or node ID, a node ID equal to the master's, a non-standard
bit rate, a reserved or duplicate COB-ID (PDOs, emergencies and TIME), a configuration SDO to
0x1010 or 0x1011, restore defaults or configuration SDOs on a node with `configure: false`, a PDO
over 64 bits, a type that does not fit its
location, a `%Q` location in a TPDO or a `%I` location in an RPDO, two entries on the same `%I`
location, or a location outside the runtime's I/O image, an entry without a location that is not a
dummy, an unknown `on_abort`, a `jump_to` that is missing or not a later item, an `lss` without
vendor ID, product code or serial number, or two nodes of a network with the same LSS address. Two entries on the same `%Q` location are
a warning.

The runtime tells the plugin how long each table of its I/O image is (`%IX`, `%IB`, ... `%QL`), sized
from the `image.conf` the editor uploads and from the program's own located variables, and each
location is checked against the table of its own size letter.

## CANopen slave

A project's CANopen slave server makes the runtime a CANopen slave (CiA 301) on its own SocketCAN
interface: a master on that bus reads and writes the objects the project defines, and the values
pass to and from the PLC program through `%I` and `%Q` locations. The editor writes it as an item
of `devices` in `conf/canopen.json`, next to `networks` (which may be empty). The slave runs on
[CANopenNode](https://github.com/CANopenNode/CANopenNode) v4.1 with the plugin's own CAN driver and
an object dictionary built from the file when the PLC starts.

```json
{
  "version": 10,
  "networks": [],
  "devices": [{
    "name": "io", "interface": "can1", "bitrate": 250000, "node_id": 5, "priority": 90,
    "identity": {"device_name": "plc-node", "software_version": "1.0", "vendor_id": 0,
                 "product_code": 1, "revision": 65536},
    "heartbeat_ms": 100,
    "master_watch": {"node_id": 127, "time_ms": 300},
    "on_master_lost": "pre_operational",
    "on_plc_stop": "hold",
    "start_alone": false,
    "lss": null,
    "objects": [
      {"index": 8192, "subindex": 1, "name": "Setpoint", "type": "INTEGER16",
       "direction": "from_master", "exchange": "pdo", "location": "%IW0"},
      {"index": 8192, "subindex": 3, "name": "Limit", "type": "UNSIGNED16",
       "direction": "from_master", "exchange": "sdo", "location": "%IW2",
       "default": 1000, "min": 100, "max": 30000},
      {"index": 8192, "subindex": 4, "name": "Valves", "type": "UNSIGNED8",
       "direction": "from_master", "exchange": "pdo",
       "bits": ["%IX0.0", "%IX0.1", "%IX0.2", "%IX0.3", "%IX0.4", "%IX0.5", "%IX0.6", "%IX0.7"]},
      {"index": 8448, "subindex": 1, "name": "Actual", "type": "INTEGER16",
       "direction": "to_master", "exchange": "pdo", "location": "%QW0"},
      {"index": 8448, "subindex": 9, "name": "Firmware", "type": "VISIBLE_STRING",
       "direction": "to_master", "exchange": "constant", "value": "1.0.0"}
    ],
    "rpdos": [{"number": 1, "cob_id": 517, "transmission": 255,
               "entries": [{"index": 8192, "subindex": 1}]}],
    "tpdos": [{"number": 1, "cob_id": 389, "transmission": 255, "inhibit_100us": 0,
               "event_timer_ms": 0, "entries": [{"index": 8448, "subindex": 1}]}]
  }]
}
```

| Field | Meaning |
|---|---|
| `interface`, `bitrate`, `node_id` | The slave's SocketCAN interface (not one a master network or another slave uses), its bit rate as for a network, and its node ID, 1-127. |
| `priority` | SCHED_FIFO priority of the slave's thread, 1-97, default 90. |
| `identity` | 0x1008 device name, 0x100A software version, 0x1018 vendor ID, product code and revision. 0x1000 (device type) is 0: no device profile. |
| `heartbeat_ms` | The slave's heartbeat producer time (0x1017), 1-65535. |
| `master_watch` | `null` or `{node_id, time_ms}`: the heartbeat consumer (0x1016) for the master's heartbeat. |
| `on_master_lost` | What the slave does when the watched heartbeat is lost, written to 0x1029 sub 1: `pre_operational` (default), `no_change` or `stopped`. It also sends EMCY 8130h. |
| `on_plc_stop` | The values to the master while the PLC is stopped: `hold` (default, the last values) or `zero`. |
| `start_alone` | `true`: the slave goes operational by itself after its boot-up (NMT startup); default `false`, it waits for the master's NMT start. |
| `lss` | `null` (default, LSS off) or the [LSS](#lss-in-the-slave) settings `{enabled, wait_for_master, serial, bitrate_change}`. |
| `objects` | The application objects, index 0x2000-0x5FFF, sub-index 1-254, each with a `type` as for a PDO entry or `VISIBLE_STRING`, a `direction` (`from_master`: written by the master, read by the program from a `%I` location; `to_master`: written by the program to a `%Q` location, read by the master) and an `exchange`: `pdo` (may be mapped to a PDO), `sdo` (by SDO only) or `constant` (a fixed `value`, no location). A `VISIBLE_STRING` object (at most 255 characters) is always a constant. |
| `default`, `min`, `max` | The object's value at start (default 0) and, for objects from the master, the range a write must keep (SDO aborts 0609 0031h above and 0609 0032h below it). |
| `bits` | Instead of `location`: the object is an `UNSIGNED8` packed as 8 bits, and `bits` lists the 8 bit locations, bit 0 first (`%IX` from the master, `%QX` to the master). The master sees one byte; the program reads or writes each bit at its own location. Not with `location`, `min`, `max` or exchange `constant`. |
| `rpdos`, `tpdos` | Up to four of each: PDO `number` 1-4, `cob_id`, `transmission` (0-240, 254 or 255; default 255), `event_timer_ms`, `inhibit_100us` (TPDOs only) and the mapped `entries` in order, at most 8 bytes, each a `pdo` object of the right direction. Unused PDOs are present and invalid. |

Behaviour:

- The slave has the CiA 301 communication objects 0x1000-0x1029 (0x1001, 0x1003 with up to 8 errors,
  newest first and without the 0000h "error reset" messages, 0x1005-0x1007, 0x1014, 0x1016, 0x1017, 0x1018, 0x1029), one SDO server (0x1200), and the PDO
  parameters 0x1400-0x1403, 0x1600-0x1603, 0x1800-0x1803 and 0x1A00-0x1A03. A master may remap the
  PDOs by the CiA 301 procedure; an object that is not `pdo` cannot be mapped (abort 0604 0041h).
  Writes to 0x1010 (store) and 0x1011 (restore) are refused (0602 0000h): nothing is stored, and a
  master writes its settings again at every boot.
- Event-driven TPDOs (254, 255) are sent when a mapped value changes and on entering operational;
  synchronous ones on the SYNC. RPDOs are taken in operational only.
- The slave stays on the bus while the PLC is stopped. A stop sends EMCY 6200h (and zeroes the
  values to the master with `on_plc_stop: zero`); the next start sends EMCY 0000h and puts the
  objects from the master back to their defaults. NMT reset node restores every object, reset
  communication the communication objects.
- A new upload keeps a slave whose settings did not change running, without a new boot-up; a
  changed or removed slave leaves the bus and a changed one starts again with the new settings.
- A failed interface is opened again every second. Service mode is refused (`busy`) on an
  interface a slave uses.
- The slave watches its own link like a master network: error passive sends EMCY 8120h and bus-off
  EMCY 8140h (both reset with EMCY 0000h when the link is back). A bus-off the kernel does not
  restart is handled as for a network ([Link recovery](#link-recovery)): the plugin resets the link
  and the slave goes on without a new boot-up; a runtime without the right to reset links logs
  how to reset it by hand.
- The error register (0x1001) has its generic bit set whenever an error is active (CiA 301
  7.5.2.2), and the manufacturer bit for 6200h (PLC stopped).
- The status (below) lists each slave under `slaves`, with what the master did to it: the NMT
  state and who set it, the SDO writes and refusals, the PDOs in use with their frame counts, the
  current values of its objects and its active emergencies.

### LSS in the slave

With `lss.enabled` the slave is also a CiA 305 LSS slave, so an LSS master can find it and give it
its node ID and bit rate. The slave answers switch state global and selective, configure node ID,
configure bit timing, store configuration, the inquire services and, while it has no node ID and
is in the waiting state, LSS fastscan (once a master has selected it, it no longer takes part). Its LSS address is the vendor ID, product code and revision from `identity` and the
serial number in 0x1018 sub 4.

```json
"lss": {"enabled": true, "wait_for_master": true, "serial": 23, "bitrate_change": true}
```

| Field | Meaning |
|---|---|
| `enabled` | LSS on. Off (default), the slave ignores LSS frames and any stored LSS values. |
| `wait_for_master` | `true`: the slave starts with no node ID and stays silent apart from LSS until a master gives it one; `node_id` is then only shown in the editor. Default `false`: it starts with `node_id`. |
| `serial` | The serial number, 1-4294967295. Left out, the slave picks a random one the first time and keeps it in its stored file, so the LSS address stays the same across restarts and uploads. |
| `bitrate_change` | Default `true`: the master may set a new bit rate (CiA 301 table) and store it; it is used from the next start of the slave. `false`: the request is refused. Activate bit timing is ignored: the slave never switches its bit rate on the bus. |

- A node ID from configure node ID is taken when the master switches the slave back to waiting:
  the slave resets its communication and boots up with the new ID. The COB-IDs of the EMCY
  (0x1014), the SDO server and PDOs 1-4 follow the node ID when they are the CiA 301 defaults
  (0x80, 0x180, 0x200, ... plus the node ID); other COB-IDs stay as set.
- Store configuration writes the node ID and bit rate to
  `<data folder>/canopen/lss-<interface>.json` (the data folder is `OPENPLC_PERSISTENT_DATA_DIR`,
  else `/var/run/runtime` in the container and `/var/lib/openplc-runtime` on a host). Stored values
  win over the project's `node_id` and `bitrate` at every start, reset node and upload, until the
  editor's Forget (request `slave_lss_forget`) removes them; the slave then restarts with the
  project's values. Turning LSS off keeps the file but does not use it. A store that cannot be
  written is answered with error 2.
- The status shows `node_id` as `null` while the slave has no node ID, its `serial`, and an `lss`
  object: `state` (`waiting` or `configuration`), `serial_auto`, `node_id_from` (`none`,
  `project`, `stored`, or `lss` for one a master gave and did not store, also when it equals the
  project's), `bitrate_from` (`project` or `stored`), `bitrate`, the project's
  `project_node_id` and `project_bitrate` and the stored `stored_node_id` and `stored_bitrate`.

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
      "late_pdos": [],
      "sdo_skipped": []
    }],
    "events": [{"time": 1793299990000, "node": 3, "kind": "boot_failed", "problem": {"kind": "identity"}}]
  }],
  "slaves": [{
    "name": "io", "interface": "can1", "node_id": 5, "bus_open": true, "state": "operational",
    "plc_running": true, "master_node_id": 127, "master_seen": true, "last_emcy": 0,
    "rx_frames": 3201, "tx_frames": 3410,
    "bus": {"state": "error_active", "tx_errors": 0, "rx_errors": 0, "link_resets": 0},
    "since": 1793299992000, "nmt_by": "master", "master_last_seen": 1793299999950,
    "error_register": 0, "emcy_active": [], "error_history": [],
    "heartbeat_ms": 100, "master_watch_ms": 300, "on_master_lost": "pre_operational",
    "pdos": [{"dir": "rx", "pdo": 1, "cob_id": 517, "valid": true, "transmission": 255,
              "event_ms": 0, "mapping": [{"index": 8192, "subindex": 1, "bits": 16}],
              "frames": 812, "last": 1793299999900}],
    "values": [{"index": 8192, "subindex": 1, "data": "3412"}],
    "events": [{"time": 1793299992000, "kind": "nmt", "state": "operational", "by": "master"}]
  }]
}
```

| Field | Meaning |
|---|---|
| `now`, `since`, `time` | Wall-clock milliseconds since the epoch. |
| `bus.state` | `error_active`, `error_warning`, `error_passive`, `bus_off`, `stopped`, `down` (interface down), `missing` (no such interface), `unknown`. A vcan link is `error_active`. |
| `bus.*` | Error counters, bus-off count, frame counts since the PLC started and an estimated bus load in percent (standard frames, without bit stuffing). Each is left out when the driver does not report it. `rx_dropped` and `tx_dropped` count dropped receive and transmit frames. `link_resets` counts the times the master reset a CAN link. |
| `all_mandatory_up`, `all_optional_up` | Every mandatory (optional) node that is on is operational or waits for the program's start; `true` when there is none. |
| `nodes[].start`, `nodes[].enabled`, `nodes[].off_by` | `master` or `program`; whether the node is on; for a node that is off, `configuration` or `program`. |
| `nodes[].state` | `waiting` (master started, not heard yet), `silent` (mandatory, not booted `boot_timeout_ms` after the start), `absent` (optional, not booted by then), `booting` (also while it restores its defaults), `failed` (the last boot failed; retried in the background), `operational`, `lost` (retried in the background unless `held`), `stopped` (PLC stop), `manual` (stopped or pre-operational by a client's NMT command), `service` (service mode: nobody manages the node), `waiting_start` (configured, pre-operational until the program starts it), `disabled` (off). |
| `nodes[].held` | Present and `true` when the node was lost with `on_node_error: stop` and stays stopped until the next PLC start. |
| `nodes[].nmt` | Last NMT state seen: `pre_operational` (also after a boot-up), `operational`, `stopped`, `unknown`. In service mode it follows the NMT commands sent and the nodes' heartbeats and boot-ups; a reset makes it `unknown` until the node is heard again. |
| `networks[].mode` | `service` while [service mode](#service-mode) is open; left out otherwise. |
| `nodes[].problem` | Why the node is not running, or `null`: `no_response`, `identity` (with `field`, `expected` and the `found` value read from the node), `sdo_abort` (a configuration SDO or the restore write refused, with `index`, `subindex` and `abort_code`), `heartbeat`, `guarding`, `reset`, `state` (the node left operational by itself), `boot_error` (with the stack's `text`). `boot_status` is the CiA 302-2 boot error letter. |
| `nodes[].warning` | An operational node with an active emergency or a late PDO. |
| `nodes[].emcy` | The last 8 emergencies, newest first; an emergency with code 0 (error reset) marks them inactive. |
| `nodes[].late_pdos` | TPDOs with a receive timeout whose frames are late. |
| `nodes[].lss` | Only for a node with `lss`: `result` is `none` (not run yet), `given` (the device had another node ID or none and got the node's), `already` (it had the node's ID), `not_found`, `refused` or `store_refused`; then `previous_id` (the device's node ID before, 255 for none), `stored` (the ID was stored in the device this time) and `time`. |
| `nodes[].sdo_skipped` | Configuration SDOs the node refused in its current boot that had `on_abort` `continue` or `jump`, with `index`, `subindex` and `abort_code`. |
| `slaves[]` | One per [slave device](#canopen-slave): `state` is the slave's own NMT state (`boot_up`, `pre_operational`, `operational`, `stopped`) or `offline` while its interface is not open (with the reason in `problem`); `master_seen` whether the watched master heartbeat is there; `last_emcy` the code of the last emergency it sent. While no master network runs, the answer is `{"version": 1, "active": false, "now": ..., "slaves": [...]}`. |
| `slaves[].bus` | The slave's link, as `bus` of a network. |
| `slaves[].since`, `nmt_by` | When the slave entered its NMT state and why: `boot` (its boot-up), `master` (an NMT command), `self` (it left operational by itself, such as on the master's heartbeat lost) or `start_alone`. |
| `slaves[].master_last_seen` | When the watched master heartbeat was last received; left out before the first one. |
| `slaves[].error_register`, `emcy_active`, `error_history` | 0x1001, the emergency codes active now and 0x1003 (newest first, each entry with the code in its low 16 bits). |
| `slaves[].heartbeat_ms`, `master_watch_ms`, `on_master_lost` | The values in use of 0x1017, 0x1016 sub 1 (0 when not watched) and 0x1029 sub 1 (`pre_operational`, `no_change`, `stopped` or `other`), as the master may have written them. |
| `slaves[].pdos` | Every PDO with its parameters in use: `dir` `rx` or `tx`, `pdo` 1-4, `cob_id`, `valid`, `transmission`, `inhibit_100us` (TPDOs), `event_ms`, the `mapping` and the `frames` received or sent with the time of the `last` one. |
| `slaves[].values` | The current value of every object except strings, `data` as little-endian hex bytes. |
| `slaves[].events` | The last 50 events of the slave, newest first: `on_bus`, `nmt` (with `state` and `by`), `reset` (`what` `node` or `communication`), `sdo_write` (a master's SDO download, with `index`, `subindex`, `ok` and the `abort_code` of a refusal), `heartbeat` (the master's heartbeat, `seen` or lost), `emcy` (with `code` and `register`), `emcy_reset`, `plc` (`running`), `bus_state` and `bus_reset` (with `outcome`). |
| `events` | The last 64 events of the network, newest first: `boot_ok` (with `waits` true when the node waits for the PLC program's start), `boot_failed`, `silent`, `absent`, `lost`, `emcy`, `emcy_reset`, `sdo_abort` (with `policy` `stop`, `continue` or `jump`), `pdo_late`, `pdo_back`, `pdo_length`, `bus_state`, `bus_reset` (the master reset the link, with `outcome` `done`, `not_permitted` or `failed`, and `cause` `bus_off`, `bit_rate` or `tx_stalled`), `sdo_write` (a client's SDO write, with `index`, `subindex`, `ok` and the `abort_code` of a refusal), `nmt_command` (an NMT command, with `command` and `source` `client` or `program`), `service` (service mode opened or closed, with `open`), `node_on` and `node_off` (the PLC program turned the node on or off), `lss` (a node ID given over LSS or the device not found, with the node's `lss` fields; a device still not found is not repeated). |

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

### LSS tools

The `lss` request finds the devices on a network that have no node ID, gives one of them a node
ID, or sets one device's bit rate. It runs on a running network and in service mode, takes a few
seconds and is a job like an SDO request; `job` answers with the members below added.

| Request | Result (`job`) |
|---|---|
| `{"op": "lss", "network": 0, "action": "find"}` | `{"state": "done", "devices": [{"vendor_id": 1, "product_code": 2, "revision": 3, "serial_number": 22136}]}` |
| `{"op": "lss", "network": 0, "action": "set_id", "vendor_id": 1, "product_code": 2, "revision": 3, "serial_number": 22136, "node_id": 4, "store": true}` | `{"state": "done", "previous_id": 255, "stored": true}` |
| `{"op": "lss", "network": 0, "action": "set_bitrate", "vendor_id": 1, "product_code": 2, "revision": 3, "serial_number": 22136, "bitrate": 125000}` | `{"state": "done", "stored": true}` |

- **find** searches with LSS fastscan until no device answers, at most 64 devices. Some devices
  take part in a fastscan although they have a node ID, so each device found is then asked its
  node ID on its own and only those without one (255) are in `devices`. While the job is
  `pending`, `devices` lists the devices that have answered so far. The nodes already running
  are not disturbed.
- **set_id** selects the device by its LSS address, gives it the node ID (1 to 127) and, with
  `store`, stores it. `previous_id` is the ID it had (255: none). The device starts with the new ID
  when it is switched back to waiting at the end; a device that had another ID keeps using the old
  one until it is reset. A node ID that a running node of the network uses is refused.
- **set_bitrate** only runs in service mode, and always stores: the device uses the new bit rate
  after its next reset or power-off, so it is then no longer heard at the old one. `bitrate` is one
  of 1000000, 800000, 500000, 250000, 125000, 50000, 20000 or 10000.
- Every request ends with all devices in the LSS waiting state.
- Failed: `not_found` (no device has the address), `no_answer`, `refused` (the device refused the
  value), `store_refused` (set but not stored), `too_many` (find stopped after 64 devices),
  `incomplete` (find stopped because a device answered out of turn: some devices answer a
  fastscan while selected, which hides the devices after them). A failed find still lists the
  devices found; give them a node ID and search again to find the rest.
  Refused at once: `busy` (another LSS sequence, the start-up assignment included, is running),
  `needs_service_mode`, `node_in_use`, `bad_request`.
- Each request and its result are logged.

### Service mode

Service mode runs the online tools while the PLC is stopped, with no program uploaded. A client
opens it with `service_open` and the CAN settings to use; the plugin then runs a master on that
interface (under `name` in the status, `Service` when left out) that sends nothing of its own (no NMT reset, start or stop, no SYNC, no boot, no
configuration) and only serves the requests, with the NMT commands sent as plain frames. The nodes
listed are the ones requests may address, all in state `service`; their `nmt` follows the commands
sent and the heartbeats and boot-ups the plugin hears. It is refused with `plc_running`
while the PLC runs CANopen or is starting it, and `busy` while it is already open. A PLC start closes it first, as do
`service_close` and five minutes without a request. Opening and closing it are logged and recorded
as `service` events.

## PLC function blocks

The OpenPLC Editor's `canopen` block library (CiA 405 names: `SDO_READ4`, `SDO_WRITE4`,
`SDO_READ_DATA`, `SDO_WRITE_DATA`, `SDO_READ_STRING`, `SDO_WRITE_STRING`, `NMT`, `GET_STATE`,
`RECV_EMCY`, `RECV_EMCY_DEV`, `GET_LOCAL_NODE_ID`, `GET_CANOPEN_KERNEL_STATE`, and the library's
own `CANOPEN_NODE_ENABLE`, `CANOPEN_NODE_STATUS`, `CANOPEN_NETWORK_STATE`) lets a PLC program
read and write a node's objects, send NMT commands, turn nodes on and off, and read node and
network states and emergencies while it runs. The editor's CANopen documentation describes the blocks; this is what the plugin does for them.

- The plugin exports `canopen_plc_api(version)`, a table of C functions (`plc_api.h`, version 2;
  version 1 callers get the same table, which starts with version 1's functions). The blocks find
  the loaded plugin by its SONAME `libcanopen_plugin.so`. The plugin is linked with `-z nodelete`,
  so the runtime's reload at each PLC start never unmaps it, and the table's `flags` say so
  (`CANOPEN_PLC_FLAG_PERMANENT`): the blocks then look the table up once and keep it. Without that
  flag (an older plugin) they look it up on each call.
- Nothing a block calls waits, locks or allocates. An SDO or NMT request goes into one of 16 slots
  of its network; the network's bus thread takes the oldest, one transfer at a time, taking turns
  with the online tools' requests, which have their own queue. A request with a timeout of 0 uses
  the network's `sdo_timeout_ms`; transfers are expedited or segmented, up to 256 bytes.
- A request is refused at once with a CiA 405 kernel error when no network runs
  (`CANOPEN_KERNEL_NO_DRIVER`), the network or node is not configured (`NET_ID_UNKNOWN`,
  `NODE_ID_UNKNOWN`), the link is down (`BUS_OFF`, or `NO_DRIVER` for a missing interface) or the
  16 slots are taken (`REQUEST_ERROR`). A node being booted answers `REQUEST_ERROR`; an SDO abort
  is `OTHER_ERROR` with the abort code, no answer is `TIMEOUT` (abort code 0x05040000).
- A link reset or PLC stop ends every pending request with `BUS_OFF`. A request handle from an
  earlier PLC run is answered `NO_DRIVER` (cancelled by the restart), never with another request's
  result.
- `GET_STATE` reads the node's NMT state as the master tracks it (`NOT_AVAIL` while the node is not
  up), `GET_CANOPEN_KERNEL_STATE` the CAN controller state, and both answer in the same scan. Each
  network keeps its last 32 emergencies for `RECV_EMCY`; a reader that falls behind continues with
  the oldest and is told how many it missed.
- `CANOPEN_NODE_STATUS` reads the node's run state as the master runs it (not found, booting,
  failed, waiting for the program's start, operational, manual, lost, held, off, PLC stopped), its
  options, and for a failed, lost or held node the problem in words; `CANOPEN_NETWORK_STATE` the
  network's all-up flags and node counts. `CANOPEN_NODE_ENABLE` only records the wanted state;
  the bus thread applies it on its next tick (10 ms) and the node status shows it applied.
- NMT from a block is handled as from the online tools (a manual stop or pre-operational is not a
  loss). Block writes and NMT commands are logged and recorded as `sdo_write` and `nmt_command`
  events, at most once a second per node and object.

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
needs `git`, `autoconf`, `automake` and `libtool` (the dependency step installs them), and
CANopenNode is cloned at its release tag into the build directory. By hand:

```bash
cd core/src/drivers/plugins/native/canopen
cmake -B build -DCANOPEN_BUILD_TESTS=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

`-DLELY_PREFIX=<dir>` uses a Lely already installed there instead of fetching it; a Lely installed
in `lely-prefix/` next to `CMakeLists.txt` is used by default (the release workflow caches one there).
`-DCANOPENNODE_SOURCE_DIR=<dir>` uses a CANopenNode checkout instead of cloning it. When
CANopenNode cannot be fetched, the plugin is built without the slave (with a CMake warning) and
refuses a file with `devices`.

| Test | What it covers |
|---|---|
| `test_bus_off_recovery` | When the master resets a faulty link: bus-off with and without a kernel restart, a wrong bit rate or a stall, and the back-off across causes. |
| `test_can_link` | What the master does to the CAN link before opening it: bit rate, bus-off auto-restart (also on a driver that refuses it), bringing it up; the link statistics read from netlink. |
| `test_config` | Parser and validator of `conf/canopen.json`. |
| `test_entry_points` | The plugin's C entry points with an allocation failure injected at each point of `start_loop`: no exception leaves them and a failed start leaves nothing running. |
| `test_link_stall` | When a link counts as stalled: no frame sent for 3 s with drops or failing writes, never while quiet, error-passive or bus-off. |
| `test_lss` | Node IDs over LSS on Lely's in-process CAN bus against simulated LSS devices: a device without a node ID gets the node's and boots, one with the node's ID is left as it is, one with another ID gets it and is reset, a device not on the bus is reported and looked for again, store only when asked and once when the ID was already right, a revision found with a slowscan, the result in the status. With `CANOPEN_TEST_INTERFACE=vcan0` it runs on a SocketCAN interface instead. |
| `test_master_dcf` | The master's own object dictionary (loaded into Lely), the concise DCF of the configuration SDOs, PDO bit packing. |
| `test_network` | The master against simulated generic I/O modules on Lely's in-process CAN bus: boot with identity check and configuration SDO, inputs, outputs, one SYNC right after each cycle's outputs with no allocation in the scan hooks, node state, loss and recovery, a node with the wrong identity, a refused configuration SDO, a node that never answers, a running node that missed the master's reset, emergencies, late PDOs, the status and its socket, the online tools (SDO reads and writes, a refused write,
pre-operational, save, start and reset of a node), service mode, the function blocks' requests (reads of every size, a write with read-back, an abort, a timeout, NMT, emergencies), a node started by the program (waiting, started, waiting again after a reset) and a node turned on and off (booted, stopped, pulled without a loss, skipped by NMT to every node), the NMT command on stop, and the reset of a link that is bus-off, at a wrong bit rate or stalled (on a stood-in link state). With `CANOPEN_TEST_INTERFACE=vcan0` it runs on a SocketCAN interface instead. |
| `test_plc_api` | The built plugin loaded as the runtime loads it: the blocks' lookup by SONAME and its cost per call (it fails above 50 µs), the answers with no network and with a missing interface, old handles across PLC runs and reloads, a kept table that stays the same across reloads, and version 2's table serving version 1. |
| `test_plc_port` | The function blocks' request slots, cancel, the queue limit, a session end, stale handles, node and bus states, the emergency ring, and the node control (wanted states, run states, network counts), with no allocation on the scan's side. |
| `test_slave` | The slave on Lely's in-process CAN bus against a scripted master: boot-up and heartbeat, the identity objects, SDO reads and writes with range checks, read-only and refused objects, a string constant, NMT start, event and SYNC TPDOs, RPDOs into `%I`, a remap by the CiA 301 procedure, PLC stop and start (EMCY 6200h and 0000h, defaults back), the master's heartbeat lost (EMCY 8130h, pre-operational), reset node, the status fields (NMT source, SDO writes and refusals, values, PDO counts and remap, active emergencies and the error register), error passive and bus-off on a stood-in link (EMCY 8120h and 8140h, the link reset without a new boot-up), the plugin's own master booting and running the slave, and with LSS: silence while unconfigured, fastscan, configure, store, the stored values across restarts and reset node, reset communication taking a new node ID, forget, a refused bit rate, a failed store, and the stored file's checks. |
| `test_status` | The status record, its JSON answer and requests, link parsing and the bus monitor. |

`canopen_sim_slave --interface vcan0 --node 2` (built with the tests) is a simulated generic I/O
module: it sets its digital inputs to 1, 2, ... and waits each time until its digital outputs
follow, so it checks a PLC program that copies one to the other. With `--lss-serial 0x1001` it is
an LSS device with that serial number, and with `--node 255` it starts without a node ID and waits
for one over LSS; `--expect-lss-stores 1` checks the number of LSS store requests.

The CI workflow `.github/workflows/canopen.yml` runs these tests, the SocketCAN variant on a vcan
link, and an end-to-end check: it installs the runtime from source, builds the OpenPLC Editor fork
(the branch with this branch's name, else `canopen`; without minification, source maps or module
concatenation, which takes about a minute), uploads its `e2e/fixtures/canopen-generic-io`
project with the editor's command line, and runs the simulated module against it on `vcan0`.
It then uploads `e2e/fixtures/canopen-plc-blocks`, a program that calls every function block
against the simulated module, and `.github/scripts/canopen_blocks_e2e.py` asks it for runs of its
checks (`canopen_sim_slave --blocks 1` reads the commands), stopping and starting the PLC between
runs and once while blocks are busy. Last it uploads `e2e/fixtures/canopen-slave`, a project
that makes the runtime a slave on `vcan0`, and `.github/scripts/canopen_slave_e2e.py` plays its
master over raw SocketCAN: SDOs with range checks, NMT start, an RPDO through the program back in a
TPDO, the master's heartbeat lost, and a PLC stop and start, checking `slaves` in the status
(NMT source, SDO writes and refusals, values, PDO counts, active emergencies).
With `e2e/fixtures/canopen-lss` the simulated module starts without a node ID, and the check waits
until the master has given it node 2 over LSS, stored it once and booted it.
`e2e/fixtures/canopen-slave-lss` is the same slave with LSS on, waiting for the master, and
`.github/scripts/canopen_slave_lss_e2e.py` plays an LSS master: fastscan, node ID 12 configured
and stored, the boot-up as node 12 with its COB-IDs following, a reset node, and the forget request.

vcan needs the kernel's `vcan` module. On a runner whose kernel has no matching
`linux-modules-extra` package the vcan jobs fail at their first step, and the reason is in its log.
