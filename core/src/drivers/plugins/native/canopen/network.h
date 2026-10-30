// SPDX-License-Identifier: Apache-2.0
//
// network.h - one CANopen master on one CAN interface, on Lely's BasicMaster. Runs only on the
// network's bus thread.
//
// Bring-up: Lely's NMT master boots each node (CiA 302-2): identity check against 0x1F84-0x1F87,
// then the 'update configuration' step, where this class restores the node's defaults when asked
// (0x1011, then an NMT reset and a second boot) and writes the node's configuration SDOs (one
// concise DCF), then NMT start. A node that is not initialized skips all of that and is only
// started. A node that fails is booted again in the background with a growing delay; a node that
// has not answered is looked for every second (an SDO read of 0x1000). Every node is optional to
// Lely, so none holds back the others; mandatory and optional differ only in how a missing node is
// reported after the network's boot-up timeout.
// Exchange: received PDOs are decoded into the process image. Each finished scan wakes the bus
// thread through the image's eventfd; the outputs of the newest scan are then written into the
// master's TPDO objects and sent when they change (and in full when a node comes up). With SYNC
// once per PLC cycle, the outputs go first and then SYNC.
// Node loss (heartbeat or node guarding timeout, or a boot-up message): the node's inputs hold or
// go to zero, its PDOs stop, its state reads 0, and it is booted again, or, with the network's
// on_node_error set to stop, sent NMT stop and left alone until the next PLC start.
// Client requests (the online tools): SDO jobs run one at a time through the master's own SDO
// client, so they queue behind the master's SDOs; Lely refuses them while it boots the node. NMT
// stop or pre-operational from a client is a manual state, not a loss: no boot retries, no late-PDO
// warnings, inputs held or zeroed, heartbeat still supervised. Start or a reset hands the node back
// to the master, which boots it again (also a held node).
// Service mode (PLC stopped): the master is never started, so it sends nothing of its own; client
// SDOs use a client SDO of their own per node and NMT commands are sent as plain frames.
// With a TIME producer, the TIME object carries the host's clock at the configured period.
// A received PDO with a receive timeout that is late has its inputs held or zeroed in the same
// way; the node keeps running. Everything seen is also recorded in the network's status.
//
// Derived in design from canworks ec7fd59d, plugin/src/canopen/network.h.

#ifndef CANOPEN_NETWORK_H
#define CANOPEN_NETWORK_H

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <lely/coapp/master.hpp>
#include <lely/coapp/sdo.hpp>
#include <lely/ev/task.h>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/timer.hpp>

#include "client.h"
#include "config.h"
#include "master_dcf.h"
#include "process_image.h"
#include "status.h"

namespace canopen_plugin
{

// Little-endian bit field access in PDO data. Exposed for tests.
uint64_t pdo_get_bits(const uint8_t *data, size_t len, unsigned offset, unsigned bits);
void pdo_set_bits(uint8_t *data, unsigned offset, unsigned bits, uint64_t value);

class Network : public lely::canopen::BasicMaster
{
  public:
    Network(ev_exec_t *exec, lely::io::Poll &poll, lely::io::TimerBase &timer,
            lely::io::TimerBase &tick, lely::io::CanChannelBase &chan, const std::string &dcf_path,
            const NetworkConfig &cfg, const MasterLayout &layout, ProcessImage &image,
            NetworkStatus &status, ClientQueue *client = nullptr, bool service = false);
    ~Network() override;

    // Resets the master, which boots every node, and starts the periodic work. In service mode it
    // only starts serving client requests.
    void Start();
    // Sends the configured NMT command to every node and stops sending PDOs and SYNC.
    void StopNodes();
    // Marks every node down (the session ends): state 0, inputs held or zeroed.
    void MarkAllDown();

    bool IsUp(unsigned node_id) const;
    // With SYNC once per PLC cycle: scans that shared a SYNC with a later scan.
    uint64_t MergedScans() const
    {
        return merged_scans_;
    }

    // Housekeeping (boot retries, looking for missing nodes, TIME); the exchange runs on each
    // scan's wake.
    static constexpr std::chrono::milliseconds kTick{10};
    static constexpr std::chrono::milliseconds kRetryMin{1000};
    static constexpr std::chrono::milliseconds kRetryMax{16000};
    // How often a node that has not answered is looked for.
    static constexpr std::chrono::milliseconds kPollEvery{1000};
    // Written to 0x1011 to restore defaults (CiA 301): ASCII "load", little-endian.
    static constexpr uint32_t kRestoreSignature = 0x64616F6C;

  protected:
    void OnBoot(uint8_t id, lely::canopen::NmtState st, char es,
                const std::string &what) noexcept override;
    void OnConfig(uint8_t id) noexcept override;
    void OnHeartbeat(uint8_t id, bool occurred) noexcept override;
    void OnNodeGuarding(uint8_t id, bool occurred) noexcept override;
    void OnState(uint8_t id, lely::canopen::NmtState st) noexcept override;
    void OnRpdo(int num, std::error_code ec, const void *p, std::size_t n) noexcept override;
    void OnEmcy(uint8_t id, uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;
    // Logged once per PDO instead of Lely's EMCY per frame.
    void OnRpdoError(int num, uint16_t eec, uint8_t er) noexcept override;

  private:
    using clock = std::chrono::steady_clock;

    struct NodeState
    {
        const NodeConfig *cfg = nullptr;
        size_t slot           = 0;
        bool up               = false;
        bool stopped          = false; // StopNodes ran
        bool heard            = false; // the node or a boot attempt has reported back
        bool silent_logged    = false;
        bool booting          = false; // a Lely boot of the node is running
        bool polling          = false; // a read of 0x1000 is running
        clock::time_point next_poll{};
        bool restored        = false; // the defaults were restored in this bring-up
        bool restore_pending = false; // waiting for the reset that follows the restore
        bool held            = false; // lost and stopped until the next PLC start
        bool has_abort       = false; // a configuration SDO was refused in this boot
        bool manual          = false; // stopped or pre-operational by a client
        Problem abort;
        uint8_t state      = 0;
        bool retry_pending = false;
        clock::time_point retry_at{};
        std::chrono::milliseconds backoff{kRetryMin};
        std::string last_problem;
        clock::time_point last_emcy{};
        std::vector<uint8_t> concise;
        std::vector<const MasterPdo *> in_pdos;
        std::vector<const MasterPdo *> out_pdos;
    };

    template <class F> void Defer(F &&f);

    void HandleBoot(uint8_t id, lely::canopen::NmtState st, char es, const std::string &what);
    void HandleConfig(uint8_t id);
    void HandleRestore(uint8_t id);
    void Poll(NodeState &node, uint8_t id);
    void SendTime();
    // Sends NMT stop to a node held by on_node_error stop, from the executor.
    void StopHeld(uint8_t id);
    void HandleLoss(uint8_t id, ProblemKind why);
    void HandleState(uint8_t id, lely::canopen::NmtState st);
    void HandleRpdo(int num, const uint8_t *data, std::size_t n);
    void HandleEmcy(uint8_t id, uint16_t eec, uint8_t er, const uint8_t msef[5]);
    void ReadFoundIdentity(uint8_t id, IdentityField field);
    Problem BootProblem(NodeState &node, char es, const std::string &what);
    void ZeroPdoInputs(const MasterPdo &m);
    void HandleRpdoError(int num, uint16_t eec);
    void SetUp(NodeState &node, bool up);
    void SetState(NodeState &node, uint8_t state);
    NodeState *Find(uint8_t id);
    void ScheduleRetry(NodeState &node);
    void ZeroInputs(const NodeState &node);
    void OnTick();
    void Exchange();
    void WatchScans();
    static void OnScanEvent(struct io_poll_watch *watch, int events);
    static void OnScanTask(ev_task *task);
    void WriteOutputs(bool all);
    void WriteObject(uint16_t index, CoType type, uint64_t value);
    void SendSync();
    void ServeClient();
    void ClientRead(const ClientJob &job);
    void ClientWrite(ClientJob &job);
    void ClientNmt(const ClientJob &job);
    void ManualNmt(uint8_t id, NmtRequest cmd);
    void SendNmt(NmtRequest cmd, uint8_t id);
    void FinishSdo(uint64_t job, std::error_code ec, std::vector<uint8_t> data);
    lely::canopen::Sdo &ServiceSdo(uint8_t id);

    ev_exec_t *exec_;
    lely::io::Poll &poll_;
    // The scan wake: a watch on the image's eventfd that posts a task to the bus thread's loop.
    struct ScanHook
    {
        struct io_poll_watch watch;
        ev_task task;
        Network *net;
        bool watched; // registered with the poll (each event needs a new watch() call)
        bool posted;
    };
    ScanHook scan_;
    lely::io::TimerBase &tick_;
    lely::io::TimerWait tick_wait_;
    const NetworkConfig &cfg_;
    const MasterLayout &layout_;
    ProcessImage &image_;
    NetworkStatus &status_;
    std::map<unsigned, NodeState> nodes_;
    std::vector<uint64_t> last_out_;
    std::vector<bool> out_valid_;
    std::vector<bool> resend_;         // per master TPDO: send in full at the next tick
    std::vector<uint16_t> rpdo_error_; // per master RPDO: the last error logged
    uint64_t synced_scans_ = 0;
    uint64_t merged_scans_ = 0;
    bool running_          = false;
    ClientQueue *client_;
    bool service_;
    bool client_busy_ = false; // a client SDO is running
    std::map<uint8_t, std::unique_ptr<lely::canopen::Sdo>> service_sdos_;
    clock::time_point started_{};
    clock::time_point next_time_{};
};

} // namespace canopen_plugin

#endif // CANOPEN_NETWORK_H
