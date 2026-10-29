// SPDX-License-Identifier: Apache-2.0
//
// network.h - one CANopen master on one CAN interface, on Lely's BasicMaster. Runs only on the
// network's bus thread.
//
// Bring-up: Lely's NMT master boots each node (CiA 302-2): identity check against 0x1F84-0x1F87,
// then the 'update configuration' step, where this class writes the node's configuration SDOs
// (one concise DCF), then NMT start. A node that fails is booted again in the background with a
// growing delay; the other nodes are not held back.
// Exchange: received PDOs are decoded into the process image; outputs from the newest scan are
// written into the master's TPDO objects and sent when they change (and in full when a node
// comes up). With SYNC once per PLC cycle, the outputs go first and then SYNC.
// Node loss (heartbeat timeout or a boot-up message): the node's inputs hold or go to zero, its
// PDOs stop, its state reads 0, and it is booted again.
//
// Derived in design from canworks ec7fd59d, plugin/src/canopen/network.h.

#ifndef CANOPEN_NETWORK_H
#define CANOPEN_NETWORK_H

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <lely/coapp/master.hpp>
#include <lely/io2/timer.hpp>

#include "config.h"
#include "master_dcf.h"
#include "process_image.h"

namespace canopen_plugin
{

// Little-endian bit field access in PDO data. Exposed for tests.
uint64_t pdo_get_bits(const uint8_t *data, size_t len, unsigned offset, unsigned bits);
void pdo_set_bits(uint8_t *data, unsigned offset, unsigned bits, uint64_t value);

class Network : public lely::canopen::BasicMaster
{
  public:
    Network(ev_exec_t *exec, lely::io::TimerBase &timer, lely::io::TimerBase &tick,
            lely::io::CanChannelBase &chan, const std::string &dcf_path, const NetworkConfig &cfg,
            const MasterLayout &layout, ProcessImage &image);
    ~Network() override;

    // Resets the master, which boots every node, and starts the periodic work.
    void Start();
    // Sends the configured NMT command to every node and stops sending PDOs and SYNC.
    void StopNodes();
    // Marks every node down (the session ends): state 0, inputs held or zeroed.
    void MarkAllDown();

    bool IsUp(unsigned node_id) const;

    static constexpr std::chrono::milliseconds kTick{1};
    static constexpr std::chrono::milliseconds kRetryMin{1000};
    static constexpr std::chrono::milliseconds kRetryMax{16000};

  protected:
    void OnBoot(uint8_t id, lely::canopen::NmtState st, char es,
                const std::string &what) noexcept override;
    void OnConfig(uint8_t id) noexcept override;
    void OnHeartbeat(uint8_t id, bool occurred) noexcept override;
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
        uint8_t state         = 0;
        bool retry_pending    = false;
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
    void HandleLoss(uint8_t id, const char *why);
    void HandleState(uint8_t id, lely::canopen::NmtState st);
    void HandleRpdo(int num, const uint8_t *data, std::size_t n);
    void HandleEmcy(uint8_t id, uint16_t eec, uint8_t er);
    void HandleRpdoError(int num, uint16_t eec);
    void SetUp(NodeState &node, bool up);
    void SetState(NodeState &node, uint8_t state);
    NodeState *Find(uint8_t id);
    void ScheduleRetry(NodeState &node);
    void ZeroInputs(const NodeState &node);
    void OnTick();
    void WriteOutputs(bool all);
    void WriteObject(uint16_t index, CoType type, uint64_t value);
    void SendSync();

    ev_exec_t *exec_;
    lely::io::TimerBase &tick_;
    lely::io::TimerWait tick_wait_;
    const NetworkConfig &cfg_;
    const MasterLayout &layout_;
    ProcessImage &image_;
    std::map<unsigned, NodeState> nodes_;
    std::vector<uint64_t> last_out_;
    std::vector<bool> out_valid_;
    std::vector<bool> resend_;         // per master TPDO: send in full at the next tick
    std::vector<uint16_t> rpdo_error_; // per master RPDO: the last error logged
    uint64_t synced_scans_ = 0;
    bool running_          = false;
};

} // namespace canopen_plugin

#endif // CANOPEN_NETWORK_H
