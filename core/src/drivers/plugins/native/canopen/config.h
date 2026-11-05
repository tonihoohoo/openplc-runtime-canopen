// SPDX-License-Identifier: Apache-2.0
//
// config.h - conf/canopen.json: the data model, parser and validator of the CANopen master's
// configuration (docs/CANOPEN.md). The editor writes the file; the plugin loads it in
// start_loop() and refuses all of it when any check fails.

#ifndef CANOPEN_CONFIG_H
#define CANOPEN_CONFIG_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "iec_location.h"

namespace canopen_plugin
{

constexpr int kConfigVersion = 11;

// One object mapped into a PDO, bound to one IEC location. A dummy entry (0x0001-0x0007) has no
// location: its bits are skipped on receive and sent as zeros.
struct PdoEntry
{
    uint16_t index   = 0;
    uint8_t subindex = 0;
    CoType type      = CoType::UNSIGNED8;
    bool dummy       = false;
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
    // A node RPDO only: send it again when it has not been sent for this long; 0 for never.
    unsigned send_period_ms = 0;
    std::vector<PdoEntry> entries;
    unsigned bits() const;
};

// What the boot does when the node refuses an SDO write.
enum class OnAbort
{
    Stop,     // fail the node's boot
    Continue, // go on with the next write
    Jump      // go on with the write at jump_to
};

// An SDO download sent to the node while it is configured, in list order.
struct SdoWrite
{
    uint16_t index   = 0;
    uint8_t subindex = 0;
    std::vector<uint8_t> data; // little-endian value bytes
    OnAbort on_abort = OnAbort::Stop;
    size_t jump_to   = 0; // 0-based index of a later write of the same list, for OnAbort::Jump
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

// A node whose device gets its node ID from the master over LSS (CiA 305), found by its LSS
// address: the four values of its object 0x1018. Revision 0 means any revision.
struct LssConfig
{
    uint32_t vendor_id     = 0;
    uint32_t product_code  = 0;
    uint32_t revision      = 0;
    uint32_t serial_number = 0;
    bool store             = false; // LSS 'store configuration' after giving the ID
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
    // true: after its boot the node stays pre-operational until the PLC program starts it.
    bool program_start = false;
    // false: the node is off at PLC start (not booted, supervised or reported) until turned on.
    bool enabled = true;
    // Node ID given over LSS before the boot.
    bool has_lss = false;
    LssConfig lss;
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

// ---- slave (the `devices` list) ----

enum class SlaveDirection
{
    FromMaster, // the master writes, the slave's PLC reads (%I)
    ToMaster    // the slave's PLC writes, the master reads (%Q)
};

enum class SlaveExchange
{
    Pdo,     // mappable into a PDO, also read or written by SDO
    Sdo,     // by SDO only
    Constant // a fixed value, no IEC location
};

// One application object of the slave (0x2000-0x5FFF).
struct SlaveObject
{
    uint16_t index   = 0;
    uint8_t subindex = 0;
    std::string name;
    CoType type        = CoType::UNSIGNED8;
    bool string        = false; // VISIBLE_STRING: constants only
    SlaveDirection dir = SlaveDirection::FromMaster;
    SlaveExchange exch = SlaveExchange::Pdo;
    IecLocation location; // not for constants or packed objects
    uint64_t value = 0;   // raw little-endian bits of the default or constant value
    std::string text;     // the value of a string constant
    // A packed UNSIGNED8: the %IX/%QX location of each bit, bit 0 first; empty otherwise.
    std::vector<IecLocation> bits;
    bool has_min = false;
    bool has_max = false;
    uint64_t min = 0; // raw bits, compared as the object's type
    uint64_t max = 0;
    unsigned bytes() const; // size in the object dictionary
    bool packed() const
    {
        return !bits.empty();
    }
};

struct SlavePdoEntry
{
    uint16_t index   = 0;
    uint8_t subindex = 0;
};

// A PDO as the slave sees it: an RPDO is received from the master, a TPDO sent to it.
struct SlavePdo
{
    unsigned number         = 0; // 1-4
    uint32_t cob_id         = 0; // 11-bit
    unsigned transmission   = 255;
    unsigned inhibit_100us  = 0; // TPDO only
    unsigned event_timer_ms = 0;
    std::vector<SlavePdoEntry> entries;
};

enum class OnMasterLost
{
    PreOperational,
    NoChange,
    Stopped
};

enum class SlaveOnStop
{
    Hold,
    Zero
};

// The slave's NMT state after a PLC stop (0x1029 sub 2).
enum class SlaveStopState
{
    PreOperational,
    NoChange,
    Stopped
};

constexpr unsigned kSlavePdos = 4;

// The slave's LSS (CiA 305) settings.
struct SlaveLss
{
    bool enabled         = false;
    bool wait_for_master = false; // start with no node ID until an LSS master gives one
    uint32_t serial      = 0;     // 0x1018 sub 4; 0 for the automatic serial
    bool bitrate_change  = true;  // an LSS master may configure the bit rate
};

struct DeviceConfig
{
    std::string name;
    std::string interface;
    unsigned bitrate = 0; // bit/s; 0 leaves the link as it is
    unsigned node_id = 0;
    int priority     = kDefaultPriority;
    std::string device_name;
    std::string software_version;
    uint32_t vendor_id               = 0;
    uint32_t product_code            = 0;
    uint32_t revision                = 0;
    unsigned heartbeat_ms            = 1000;
    unsigned master_node_id          = 0; // 0: the master's heartbeat is not watched
    unsigned master_watch_ms         = 0;
    OnMasterLost on_master_lost      = OnMasterLost::PreOperational;
    SlaveOnStop on_plc_stop          = SlaveOnStop::Hold;
    SlaveStopState on_plc_stop_state = SlaveStopState::NoChange;
    bool start_alone                 = false;
    SlaveLss lss;
    std::vector<SlaveObject> objects;
    std::vector<SlavePdo> rpdos;
    std::vector<SlavePdo> tpdos;
    // The entry's JSON text: a running slave with the same text is kept across a PLC restart.
    std::string source;
    std::string label() const; // "slave \"io\" (can1)"
    const SlaveObject *find(uint16_t index, uint8_t subindex) const;
};

struct Config
{
    std::vector<NetworkConfig> networks;
    std::vector<DeviceConfig> devices;
    std::vector<std::string> warnings;
};

// Whether this build has the CANopen slave (CANopenNode); without it a file with devices is
// refused.
bool slave_supported();

// Compares two raw values of `type` as that type: <0, 0 or >0.
int compare_raw(CoType type, uint64_t a, uint64_t b);

// Parses and validates the JSON text. Returns false with one message per problem in `errors`;
// warnings go to cfg.warnings.
bool parse_config(const std::string &text, Config &cfg, std::vector<std::string> &errors);

// Reads the file and calls parse_config().
bool load_config(const std::string &path, Config &cfg, std::vector<std::string> &errors);

// Bit rates a CAN link can be set to (CiA 301 table).
bool is_standard_bitrate(unsigned bitrate);

} // namespace canopen_plugin

#endif // CANOPEN_CONFIG_H
