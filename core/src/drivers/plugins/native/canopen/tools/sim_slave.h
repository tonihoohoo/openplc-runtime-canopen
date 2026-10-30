// SPDX-License-Identifier: Apache-2.0
//
// sim_slave.h - a simulated generic I/O module (a CANopen slave on Lely's BasicSlave) for tests
// and the end-to-end check. Objects:
//   0x6000:01 UNSIGNED8  digital inputs   TPDO1 (0x180 + id), with 0x6401:01
//   0x6401:01 INTEGER16  analogue input   TPDO1, sent every 20 ms
//   0x6200:01 UNSIGNED8  digital outputs  RPDO1 (0x200 + id)
//   0x2100:00 UNSIGNED32 a parameter the master can write during configuration (read-only, so the
//                        write is refused, with parameter_read_only)
// It produces heartbeats at 0x1017 (default 50 ms; 0 to be guarded instead, with 0x100C and
// 0x100D) and emergencies on request (0x80 + id), can watch the master's heartbeat (0x1016:01,
// written by the master; 0x1029:01 sets the reaction, pre-operational by default), restores its
// defaults on "load" written to 0x1011 sub 1-3 (taken at the next reset) and runs on its own
// thread, on Lely's virtual bus or on a SocketCAN interface.

#ifndef CANOPEN_SIM_SLAVE_H
#define CANOPEN_SIM_SLAVE_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace lely
{
namespace io
{
class VirtualCanController;
}
} // namespace lely

namespace canopen_sim
{

struct SimSlaveSpec
{
    unsigned node_id         = 2;
    uint32_t device_type     = 0x00000191;
    uint32_t vendor_id       = 0x00000001;
    uint32_t product_code    = 0x00000002;
    uint32_t revision        = 0x00000003;
    unsigned heartbeat_ms    = 50;
    bool parameter_read_only = false;
    bool restore_refused     = false; // 0x1011 aborts every write
};

// The slave's DCF text.
std::string sim_slave_dcf(const SimSlaveSpec &spec);

class SimSlave
{
  public:
    // On the virtual bus when `vbus` is set, else on SocketCAN `interface`. `dcf_path` is where
    // the slave's DCF is written.
    SimSlave(const SimSlaveSpec &spec, std::string dcf_path,
             std::shared_ptr<lely::io::VirtualCanController> vbus, std::string interface = "");
    ~SimSlave();

    void start();
    void stop();

    void set_inputs(uint8_t digital, int16_t analogue);
    // Sends an emergency; clear_emcy() sends the error reset (code 0).
    void send_emcy(uint16_t code, uint8_t reg);
    void clear_emcy();
    // Stops or resumes TPDO 1 while the heartbeat goes on.
    void set_tpdo_enabled(bool on);
    uint8_t outputs() const
    {
        return outputs_.load();
    }
    uint32_t parameter() const
    {
        return parameter_.load();
    }
    // The last NMT command code received or applied by the node itself (0 before any), such as
    // 0x80 (enter pre-operational) when the master's heartbeat stops.
    uint8_t last_command() const
    {
        return nmt_state_.load();
    }
    // Restore requests (0x1011 written with "load") since start().
    unsigned restores() const
    {
        return restores_.load();
    }

  private:
    struct Impl;
    void thread_main();

    SimSlaveSpec spec_;
    std::string dcf_path_;
    std::shared_ptr<lely::io::VirtualCanController> vbus_;
    std::string interface_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> ready_{false};
    std::atomic<uint8_t> digital_{0};
    std::atomic<int16_t> analogue_{0};
    std::atomic<bool> inputs_dirty_{false};
    std::atomic<uint32_t> emcy_request_{0}; // bit 16: pending, bits 0-15 code
    std::atomic<uint8_t> emcy_reg_{0};
    std::atomic<bool> emcy_clear_{false};
    std::atomic<int> tpdo_request_{-1}; // -1 none, 0 off, 1 on
    std::atomic<uint8_t> outputs_{0};
    std::atomic<uint32_t> parameter_{0};
    std::atomic<uint8_t> nmt_state_{0};
    std::atomic<unsigned> restores_{0};
};

} // namespace canopen_sim

#endif // CANOPEN_SIM_SLAVE_H
