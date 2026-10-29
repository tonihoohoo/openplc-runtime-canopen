// SPDX-License-Identifier: Apache-2.0
//
// config.h - conf/canopen.json: the data model, parser and validator of the CANopen master's
// configuration (docs/CANOPEN.md). The editor writes the file; the plugin loads it in
// start_loop() and refuses all of it when any check fails.

#ifndef CANOPEN_CONFIG_H
#define CANOPEN_CONFIG_H

#include <cstdint>
#include <string>
#include <vector>

#include "iec_location.h"

namespace canopen_plugin
{

constexpr int kConfigVersion = 3;

// One object mapped into a PDO, bound to one IEC location.
struct PdoEntry
{
    uint16_t index   = 0;
    uint8_t subindex = 0;
    CoType type      = CoType::UNSIGNED8;
    IecLocation location;
};

// A PDO as the master sees it: a node's TPDO is received by the master (inputs, %I), a node's
// RPDO is sent by the master (outputs, %Q).
struct PdoConfig
{
    unsigned number = 0; // PDO number on the node, 1-512
    uint32_t cob_id = 0; // 11-bit COB-ID
    // A node TPDO only: report the PDO late when no frame arrives for this long; 0 for never.
    unsigned receive_timeout_ms = 0;
    std::vector<PdoEntry> entries;
    unsigned bits() const;
};

// An SDO download sent to the node while it is configured, in list order.
struct SdoWrite
{
    uint16_t index   = 0;
    uint8_t subindex = 0;
    std::vector<uint8_t> data; // little-endian value bytes
};

enum class Supervision
{
    None,
    Heartbeat,
    Guarding
};

enum class OnLoss
{
    Hold,
    Zero
};

struct NodeConfig
{
    unsigned node_id = 0;
    std::string name;
    // Expected identity; 0 skips that check.
    uint32_t device_type  = 0;
    uint32_t vendor_id    = 0;
    uint32_t product_code = 0;
    uint32_t revision     = 0;
    // Optional: absent is not an error. Mandatory nodes are told apart only by the plugin.
    bool optional = false;
    // false: no identity check, restore or configuration SDOs; the node is only started.
    bool configure = true;
    // 0x1011 sub-index written with "load" before the configuration SDOs; 0 for none.
    unsigned restore_defaults      = 0;
    Supervision supervision        = Supervision::None;
    unsigned heartbeat_consumer_ms = 0;
    unsigned guard_time_ms         = 0; // node guarding
    unsigned life_time_factor      = 0;
    bool emcy_enabled              = true;
    uint32_t emcy_cob_id           = 0; // 11-bit; 0x80 + node ID by default
    OnLoss on_loss                 = OnLoss::Hold;
    bool has_state_location        = false;
    IecLocation state_location; // %IB: CiA 301 NMT state, 0 while not running
    std::vector<SdoWrite> sdo;
    std::vector<PdoConfig> tpdos; // received by the master
    std::vector<PdoConfig> rpdos; // sent by the master
    std::string label() const;    // "node 2 (io1)"
};

enum class SyncMode
{
    None,
    Period,
    PlcCycle
};

enum class OnNodeError
{
    Restart, // boot the node again in the background
    Stop     // send NMT stop and leave it until the next PLC start
};

enum class OnStop
{
    PreOperational,
    Stopped,
    None
};

// Real-time priority of a bus thread: like the EtherCAT relay, above every IEC task (at most 49)
// and below the runtime's dispatcher (98) and watchdog (99).
constexpr int kDefaultPriority = 90;
constexpr int kMaxPriority     = 97;

struct NetworkConfig
{
    std::string name;
    std::string interface;
    unsigned bitrate               = 0; // bit/s; 0 leaves the link as it is
    unsigned master_node_id        = 127;
    unsigned heartbeat_producer_ms = 0;
    SyncMode sync                  = SyncMode::None;
    unsigned sync_period_us        = 0;
    unsigned sdo_timeout_ms        = 1000;
    OnStop on_stop                 = OnStop::PreOperational;
    // A mandatory node not booted this long after the start is reported as not answering.
    unsigned boot_timeout_ms  = 2000;
    OnNodeError on_node_error = OnNodeError::Restart;
    bool time_producer        = false; // TIME (CiA 301) with the host's clock
    uint32_t time_cob_id      = 0x100;
    unsigned time_period_ms   = 1000;
    int priority              = kDefaultPriority; // SCHED_FIFO of the bus thread in a session
    std::vector<NodeConfig> nodes;
    std::string label() const; // "network \"CANopen 1\" (can0)"
};

struct Config
{
    std::vector<NetworkConfig> networks;
    std::vector<std::string> warnings;
};

// Parses and validates the JSON text. Returns false with one message per problem in `errors`;
// warnings go to cfg.warnings.
bool parse_config(const std::string &text, Config &cfg, std::vector<std::string> &errors);

// Reads the file and calls parse_config().
bool load_config(const std::string &path, Config &cfg, std::vector<std::string> &errors);

// Bit rates a CAN link can be set to (CiA 301 table).
bool is_standard_bitrate(unsigned bitrate);

} // namespace canopen_plugin

#endif // CANOPEN_CONFIG_H
