// SPDX-License-Identifier: Apache-2.0
//
// sim_slave.h - a simulated generic I/O module (a CANopen slave on Lely's BasicSlave) for tests
// and the end-to-end check. Objects:
//   0x6000:01 UNSIGNED8  digital inputs   TPDO1 (0x180 + id), with 0x6401:01
//   0x6401:01 INTEGER16  analogue input   TPDO1, sent every 20 ms
//   0x6200:01 UNSIGNED8  digital outputs  RPDO1 (0x200 + id)
//   0x2100:00 UNSIGNED32 a parameter the master can write during configuration
// It produces heartbeats at 0x1017 (default 50 ms) and runs on its own thread, on Lely's virtual
// bus or on a SocketCAN interface.

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
    unsigned node_id      = 2;
    uint32_t device_type  = 0x00000191;
    uint32_t vendor_id    = 0x00000001;
    uint32_t product_code = 0x00000002;
    uint32_t revision     = 0x00000003;
    unsigned heartbeat_ms = 50;
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
    uint8_t outputs() const
    {
        return outputs_.load();
    }
    uint32_t parameter() const
    {
        return parameter_.load();
    }
    // The last NMT command code received (0 before any).
    uint8_t last_command() const
    {
        return nmt_state_.load();
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
    std::atomic<uint8_t> outputs_{0};
    std::atomic<uint32_t> parameter_{0};
    std::atomic<uint8_t> nmt_state_{0};
};

} // namespace canopen_sim

#endif // CANOPEN_SIM_SLAVE_H
