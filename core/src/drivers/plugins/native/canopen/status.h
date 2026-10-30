// SPDX-License-Identifier: Apache-2.0
//
// status.h - the live status of one network for diagnostics: per node a state, the NMT state, the
// reason of the last failure, recent emergencies and late PDOs; per network the CAN controller
// state and recent events. Written only by the network's bus thread, read by the status server,
// never touched by the PLC scan. Guarded by a priority-inheritance mutex, since a bus thread may
// run at a real-time priority while the reader does not. Times are wall-clock milliseconds.

#ifndef CANOPEN_STATUS_H
#define CANOPEN_STATUS_H

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "config.h"
#include "pi_mutex.h"

namespace canopen_plugin
{

enum class NodeRun : uint8_t
{
    Waiting,     // the master started, the node has not reported back yet
    Silent,      // mandatory, not booted within the boot-up timeout
    Absent,      // optional, not booted within the boot-up timeout
    Booting,     // the boot (identity check, configuration SDOs) is running
    Failed,      // the last boot failed; retried in the background
    Operational, // up and started
    Lost,        // was up; retried in the background
    Stopped      // the PLC stopped
};

enum class NmtSeen : uint8_t
{
    Unknown,
    PreOperational,
    Operational,
    Stopped
};

enum class ProblemKind : uint8_t
{
    None,
    NoResponse, // the node did not answer the boot
    Identity,   // the device is not the configured one
    SdoAbort,   // a configuration SDO was refused
    Heartbeat,  // a running node missed its heartbeat
    Guarding,   // a running node did not answer node guarding
    Reset,      // a running node sent a boot-up message
    BootError   // any other boot status
};

enum class IdentityField : uint8_t
{
    DeviceType,
    VendorId,
    ProductCode,
    Revision,
    SerialNumber
};

struct Problem
{
    ProblemKind kind    = ProblemKind::None;
    IdentityField field = IdentityField::DeviceType; // Identity
    uint32_t expected   = 0;                         // Identity
    bool has_found      = false;                     // Identity: the read of the found value worked
    uint32_t found      = 0;
    uint16_t index      = 0; // SdoAbort
    uint8_t subindex    = 0; // SdoAbort
    uint32_t abort_code = 0; // SdoAbort
    char boot_status    = 0; // CiA 302-2 boot error status letter, when the boot reported one
    std::string text;        // BootError: the stack's description

    bool operator==(const Problem &o) const;
    bool operator!=(const Problem &o) const
    {
        return !(*this == o);
    }
};

enum class BusState : uint8_t
{
    Unknown,
    ErrorActive,
    ErrorWarning,
    ErrorPassive,
    BusOff,
    Stopped,
    Down,   // the interface exists but is down
    Missing // no such interface
};

struct BusStatus
{
    BusState state      = BusState::Unknown;
    bool has_counters   = false;
    unsigned tx_errors  = 0;
    unsigned rx_errors  = 0;
    bool has_bus_off    = false;
    uint32_t bus_off    = 0; // since the PLC started
    bool has_frames     = false;
    uint64_t rx_frames  = 0; // since the PLC started
    uint64_t tx_frames  = 0;
    uint64_t rx_dropped = 0;
    bool has_load       = false;
    double load_percent = 0;
    bool has_resets     = false; // a CAN link the master may reset
    uint32_t resets     = 0;     // link resets after bus-off since the PLC started
};

struct Emcy
{
    int64_t time    = 0;
    uint16_t code   = 0;
    uint8_t reg     = 0;
    uint8_t data[5] = {0, 0, 0, 0, 0};
    bool active     = true;
};

struct LatePdo
{
    unsigned pdo    = 0; // the node's TPDO number
    uint32_t cob_id = 0;
    int64_t since   = 0;
};

enum class EventKind : uint8_t
{
    BootOk,
    BootFailed,
    Silent,
    Absent,
    Lost,
    Emcy,
    EmcyReset,
    SdoAbort,
    PdoLate,
    PdoBack,
    PdoLength,
    BusChange,
    BusReset
};

enum class ResetOutcome : uint8_t
{
    Done,
    NotPermitted,
    Failed
};

struct Event
{
    int64_t time   = 0;
    unsigned node  = 0; // 0: the network
    EventKind kind = EventKind::BootOk;
    Problem problem;        // BootFailed, Lost, SdoAbort
    uint16_t code      = 0; // Emcy: error code; PdoLength: 0x8210 or 0x8220
    uint8_t reg        = 0; // Emcy
    unsigned pdo       = 0; // Pdo*
    uint32_t cob_id    = 0; // Pdo*
    BusState bus       = BusState::Unknown;
    ResetOutcome reset = ResetOutcome::Done; // BusReset
};

struct NodeStatus
{
    unsigned node_id = 0;
    std::string name;
    bool optional  = false;
    bool held      = false; // lost and not restarted until the next PLC start (on node error: stop)
    NodeRun state  = NodeRun::Waiting;
    NmtSeen nmt    = NmtSeen::Unknown;
    int64_t since  = 0;
    unsigned boots = 0; // finished boot attempts since the PLC started
    unsigned losses = 0;
    Problem problem;
    std::vector<Emcy> emcy; // newest first
    std::vector<LatePdo> late;

    // An operational node with an active emergency or a late PDO.
    bool warning() const;
};

struct NetworkSnapshot
{
    std::string name;
    std::string interface;
    BusStatus bus;
    std::vector<NodeStatus> nodes;
    std::deque<Event> events; // newest first

    // Every mandatory (optional) node is operational; true when there is none.
    bool all_up(bool optional) const;
};

// Wall-clock time in milliseconds since the epoch.
int64_t wall_ms();

class NetworkStatus
{
  public:
    static constexpr size_t kMaxEmcy   = 8;
    static constexpr size_t kMaxEvents = 64;

    explicit NetworkStatus(const NetworkConfig &cfg);

    // A new session: every node waits again unless the PLC stopped; counters and lists stay.
    // Keeps the nodes held by `held()`: they stay stopped until the next PLC start.
    void begin_session();
    void set_state(unsigned node, NodeRun state);
    void set_nmt(unsigned node, NmtSeen nmt);
    // Not booted within the boot-up timeout: silent when mandatory, absent when optional.
    void silent(unsigned node);
    // Lost and left stopped until the next PLC start.
    void held(unsigned node);
    bool is_held(unsigned node);
    void boot_ok(unsigned node);
    void boot_failed(unsigned node, const Problem &p);
    // The found value of an identity failure, read after the failure.
    void identity_found(unsigned node, IdentityField field, uint32_t value);
    void lost(unsigned node, const Problem &p);
    void sdo_abort(unsigned node, uint16_t index, uint8_t subindex, uint32_t code);
    void emcy(unsigned node, uint16_t code, uint8_t reg, const uint8_t data[5]);
    void pdo_late(unsigned node, unsigned pdo, uint32_t cob_id);
    void pdo_back(unsigned node, unsigned pdo);
    void pdo_length(unsigned node, unsigned pdo, uint32_t cob_id, uint16_t code);
    void clear_late(unsigned node);
    // Keeps the count of link resets.
    void set_bus(const BusStatus &bus);
    // The master reset the link after a bus-off, or tried to.
    void link_reset(ResetOutcome outcome);
    // Every node stopped (the PLC stopped).
    void stop_all();

    NetworkSnapshot snapshot() const;

  private:
    NodeStatus *find(unsigned node);
    void change(NodeStatus &n, NodeRun state);
    void push(Event e);

    mutable PiMutex mutex_;
    NetworkSnapshot data_;
};

// The status of every network as the JSON answer of the status socket (version 1).
std::string status_json(const std::vector<NetworkSnapshot> &networks, int64_t now);

const char *node_run_name(NodeRun s);
const char *bus_state_name(BusState s);

} // namespace canopen_plugin

#endif // CANOPEN_STATUS_H
