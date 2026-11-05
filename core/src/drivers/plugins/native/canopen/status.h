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

#include "client.h"
#include "config.h"
#include "pi_mutex.h"

namespace canopen_plugin
{

enum class NodeRun : uint8_t
{
    Waiting,      // the master started, the node has not reported back yet
    Silent,       // mandatory, not booted within the boot-up timeout
    Absent,       // optional, not booted within the boot-up timeout
    Booting,      // the boot (identity check, configuration SDOs) is running
    Failed,       // the last boot failed; retried in the background
    Operational,  // up and started
    Lost,         // was up; retried in the background
    Stopped,      // the PLC stopped
    Manual,       // stopped or pre-operational by a client's NMT command
    Service,      // service mode: the master does not run the node
    WaitingStart, // booted and configured, pre-operational until the PLC program starts it
    Disabled      // off: not booted, supervised or reported
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
    State,      // a running node left operational without a command
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
    uint64_t tx_dropped = 0;
    bool has_load       = false;
    double load_percent = 0;
    bool has_resets     = false; // a CAN link the master may reset
    uint32_t resets     = 0;     // link resets by the master since the PLC started
};

struct Emcy
{
    int64_t time    = 0;
    uint16_t code   = 0;
    uint8_t reg     = 0;
    uint8_t data[5] = {0, 0, 0, 0, 0};
    bool active     = true;
};

// A configuration SDO the node refused, with a policy that let the boot go on.
struct SkippedSdo
{
    uint16_t index      = 0;
    uint8_t subindex    = 0;
    uint32_t abort_code = 0;
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
    BusReset,
    SdoWrite,   // a client's SDO write
    NmtCommand, // an NMT command from a client or the PLC program
    Service,    // service mode opened or closed
    NodeOn,     // the PLC program turned a node on
    NodeOff,    // the PLC program turned a node off
    Lss         // the outcome of giving a node its ID over LSS
};

// The outcome of giving a node its node ID over LSS (CiA 305).
enum class LssOutcome : uint8_t
{
    None,
    Given,       // the device had another node ID or none, and got the node's
    Already,     // the device already had the node's ID
    NotFound,    // no device with the node's LSS address answered
    Refused,     // the device refused the node ID
    StoreRefused // the ID was given, but the device refused to store it
};

struct LssResult
{
    LssOutcome outcome   = LssOutcome::None;
    unsigned previous_id = 0; // the device's node ID before; 255: none
    bool stored          = false;
    int64_t time         = 0;
};

enum class ResetOutcome : uint8_t
{
    Done,
    NotPermitted,
    Failed
};

enum class ResetCause : uint8_t
{
    BusOff,
    BitRate,
    TxStalled
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
    ResetCause cause   = ResetCause::BusOff; // BusReset
    uint16_t index     = 0;                  // SdoWrite
    uint8_t subindex   = 0;                  // SdoWrite
    bool ok            = true;               // SdoWrite
    bool has_abort     = false;              // SdoWrite
    uint32_t abort     = 0;                  // SdoWrite
    NmtRequest command = NmtRequest::Start;  // NmtCommand
    bool by_program    = false;              // NmtCommand: sent by the PLC program
    bool waits         = false;              // BootOk: waits for the PLC program's start
    bool open          = false;              // Service
    OnAbort policy     = OnAbort::Stop;      // SdoAbort
    LssResult lss;                           // Lss
};

struct NodeStatus
{
    unsigned node_id = 0;
    std::string name;
    bool optional = false;
    bool held     = false; // lost and not restarted until the next PLC start (on node error: stop)
    bool program_start   = false; // started by the PLC program
    bool enabled         = true;  // on; false: off at PLC start or turned off by the program
    bool off_by_program  = false; // turned off by the PLC program, not by the configuration
    NodeRun state        = NodeRun::Waiting;
    NmtSeen nmt          = NmtSeen::Unknown;
    int64_t since        = 0;
    unsigned boots       = 0; // finished boot attempts since the PLC started
    unsigned losses      = 0;
    bool silent_reported = false; // the silent or absent event was sent; cleared when it boots
    Problem problem;
    std::vector<Emcy> emcy; // newest first
    std::vector<LatePdo> late;
    std::vector<SkippedSdo> skipped; // refused writes of the current boot
    bool has_lss = false;            // the node gets its ID over LSS
    LssResult lss;                   // the last LSS outcome

    // An operational node with an active emergency or a late PDO.
    bool warning() const;
};

struct NetworkSnapshot
{
    bool service = false; // service mode
    std::string name;
    std::string interface;
    BusStatus bus;
    std::vector<NodeStatus> nodes;
    std::deque<Event> events; // newest first

    // Every mandatory (optional) node that is on is operational or waits for the program's start;
    // true when there is none.
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
    // Not booted within the boot-up timeout: silent when mandatory, absent when optional. Returns
    // true the first time since the PLC start or the node's last boot, when the event is sent.
    bool silent(unsigned node);
    // Lost and left stopped until the next PLC start.
    void held(unsigned node);
    bool is_held(unsigned node);
    void boot_ok(unsigned node, bool waits = false);
    void boot_failed(unsigned node, const Problem &p);
    // The found value of an identity failure, read after the failure.
    void identity_found(unsigned node, IdentityField field, uint32_t value);
    void lost(unsigned node, const Problem &p);
    // A refused configuration SDO. With continue or jump the boot goes on and the write is listed
    // under the node's skipped writes until clear_skipped().
    void sdo_abort(unsigned node, uint16_t index, uint8_t subindex, uint32_t code,
                   OnAbort policy = OnAbort::Stop);
    // A new configuration of the node starts.
    void clear_skipped(unsigned node);
    void emcy(unsigned node, uint16_t code, uint8_t reg, const uint8_t data[5]);
    void pdo_late(unsigned node, unsigned pdo, uint32_t cob_id);
    void pdo_back(unsigned node, unsigned pdo);
    void pdo_length(unsigned node, unsigned pdo, uint32_t cob_id, uint16_t code);
    void clear_late(unsigned node);
    // Keeps the count of link resets.
    void set_bus(const BusStatus &bus);
    // The master reset the link, or tried to.
    void link_reset(ResetOutcome outcome, ResetCause cause = ResetCause::BusOff);
    // Every node stopped (the PLC stopped).
    void stop_all();
    // Stopped or pre-operational by a client's NMT command; the NMT state comes from the node.
    void manual(unsigned node);
    // A held node released by a client: no longer held, booting.
    void release(unsigned node);
    // A client's SDO write and its outcome; the data is not kept.
    void client_write(unsigned node, uint16_t index, uint8_t subindex, bool ok, bool has_abort,
                      uint32_t abort);
    // An NMT command from a client or the PLC program to a node, or to every node (0).
    void nmt_command(unsigned node, NmtRequest command, bool by_program = false);
    // Service mode: every node in state Service, and an event for the open or the close.
    void set_service(bool open);
    // Booted and configured, waiting for the PLC program's start.
    void waiting_start(unsigned node);
    // The node is turned on or off; `by_program` records an event, otherwise it is the
    // configuration's setting at PLC start.
    void set_enabled(unsigned node, bool on, bool by_program);
    // The outcome of giving the node its ID over LSS; recorded as an event when it changes.
    void lss_result(unsigned node, LssOutcome outcome, unsigned previous_id, bool stored);
    // Changes with every node's state or problem, for readers that copy only on a change.
    uint64_t generation() const;
    // The node's run state, hold and problem; false for an unknown node.
    bool node_brief(unsigned node, NodeRun &state, bool &held, Problem &problem) const;

    NetworkSnapshot snapshot() const;

  private:
    NodeStatus *find(unsigned node);
    void change(NodeStatus &n, NodeRun state);
    void push(Event e);

    mutable PiMutex mutex_;
    NetworkSnapshot data_;
    uint64_t gen_ = 0;
};

// What happened to a slave, for the status's events.
enum class SlaveEventKind : uint8_t
{
    OnBus,     // node on the bus after a start or a link reopen
    Nmt,       // NMT state change: state, by
    Reset,     // reset node or communication by the master: what
    SdoWrite,  // a master's SDO download: index, subindex, ok, abort_code
    Heartbeat, // the master's heartbeat: seen
    Emcy,      // EMCY sent: code, reg
    EmcyReset, // EMCY 0000h sent: reg
    Plc,       // PLC run or stop: running
    BusState,  // bus state change: bus_state
    BusReset   // link reset after a bus-off: outcome
};

// Who brought the slave to its NMT state.
enum class SlaveNmtBy : uint8_t
{
    Boot,       // after its start or a reset
    Master,     // an NMT command
    Self,       // its error behaviour (0x1029)
    StartAlone, // started without a master
    Program,    // the PLC program (CANOPEN_SLAVE_NMT)
    PlcStop     // the NMT state set for a PLC stop (0x1029 sub 2)
};

struct SlaveEvent
{
    int64_t time         = 0;
    SlaveEventKind kind  = SlaveEventKind::OnBus;
    uint8_t state        = 0; // Nmt: CiA 301 state
    SlaveNmtBy by        = SlaveNmtBy::Boot;
    bool node            = false; // Reset: node (true) or communication
    uint16_t index       = 0;
    uint8_t subindex     = 0;
    bool ok              = false; // SdoWrite: taken; Heartbeat: seen; Plc: running
    uint32_t abort_code  = 0;
    uint16_t code        = 0;
    uint8_t reg          = 0;
    bool program         = false; // Emcy: raised by the PLC program
    BusState bus_state   = BusState::Unknown;
    ResetOutcome outcome = ResetOutcome::Done;
};

// A slave PDO as it runs now.
struct SlavePdoStatus
{
    unsigned number        = 0; // 1-4
    bool tpdo              = false;
    uint32_t cob_id        = 0; // 0x1400/0x1800 sub 1, with the invalid bit
    uint8_t transmission   = 0;
    uint16_t inhibit_100us = 0;
    uint16_t event_ms      = 0;
    std::vector<uint32_t> mapping; // 0x1600/0x1A00 entries: index << 16 | subindex << 8 | bits
    uint64_t frames = 0;
    int64_t last    = 0; // wall time of the last frame, 0 for none
};

// A data object's value in the dictionary.
struct SlaveValue
{
    uint16_t index   = 0;
    uint8_t subindex = 0;
    uint8_t bytes    = 0;
    uint8_t data[8]{};
};

// The status of every network as the JSON answer of the status socket (version 1).
// One CANopen slave of this runtime, as the status reports it.
struct SlaveSnapshot
{
    std::string name;
    std::string interface;
    unsigned node_id = 0;
    bool bus_open    = false;    // the CAN interface is open
    std::string problem;         // why it is not, "" when it is
    uint8_t nmt             = 0; // CiA 301 state: 0 boot-up, 4 stopped, 5 operational, 127 pre-op
    bool plc_running        = false;
    unsigned master_node_id = 0;     // 0: not watched
    bool master_seen        = false; // the master's heartbeat is being received
    uint16_t last_emcy      = 0;     // error code of the last EMCY sent
    bool emcy_sent          = false;
    uint64_t rx_frames      = 0;
    uint64_t tx_frames      = 0;
    uint32_t serial         = 0; // 0x1018 sub 4
    // LSS (CiA 305), reported when `lss` is on.
    bool lss                 = false;
    bool serial_auto         = false;
    bool unconfigured        = false;     // waits for an LSS master to give a node ID
    bool lss_selected        = false;     // in LSS configuration state
    unsigned bitrate         = 0;         // bit/s the interface runs at; 0 when left as it was
    const char *node_from    = "project"; // project, stored, lss (set by a master, not stored)
    const char *rate_from    = "project"; // project, stored
    unsigned project_node_id = 0;
    unsigned project_bitrate = 0;
    unsigned stored_node_id  = 0; // 0: none stored
    unsigned stored_bitrate  = 0;
    // Details for the editor's Online tab; `detailed` is false until the slave is first on the bus.
    bool detailed = false;
    BusStatus bus;
    int64_t since          = 0; // wall time of the last NMT state change
    SlaveNmtBy nmt_by      = SlaveNmtBy::Boot;
    int64_t master_last    = 0; // wall time of the master's last heartbeat, 0 for none
    uint8_t error_register = 0;
    std::vector<uint16_t> emcy_active;   // error codes not yet reset, newest first
    std::vector<uint16_t> emcy_program;  // those of them the PLC program raised
    std::vector<uint32_t> error_history; // 0x1003, newest first
    uint16_t heartbeat_ms     = 0;       // 0x1017
    uint16_t master_watch_ms  = 0;       // 0x1016 sub 1
    uint8_t on_master_lost    = 0;       // 0x1029 sub 1
    uint8_t on_plc_stop_state = 1;       // 0x1029 sub 2
    std::vector<SlavePdoStatus> pdos;
    std::vector<SlaveValue> values;
    std::deque<SlaveEvent> events; // newest first
};

constexpr size_t kSlaveEvents = 50;

std::string status_json(const std::vector<NetworkSnapshot> &networks, int64_t now,
                        const std::vector<SlaveSnapshot> &slaves = {});
// The answer while no network runs: inactive, with the slaves that run.
std::string inactive_status_json(const std::vector<SlaveSnapshot> &slaves);

const char *node_run_name(NodeRun s);
// The node's problem in words for the PLC blocks ("" for none), cut to fit `size`.
void problem_text(const Problem &p, char *out, size_t size);
const char *bus_state_name(BusState s);

} // namespace canopen_plugin

#endif // CANOPEN_STATUS_H
