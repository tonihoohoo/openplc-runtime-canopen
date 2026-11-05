// SPDX-License-Identifier: Apache-2.0
//
// slave.h - one CANopen slave of the runtime, on CANopenNode: its own thread receives the frames,
// runs the stack and sends, and exchanges values with the PLC scan through triple buffers, so the
// scan never waits for it. The slave stays on the bus while the PLC is stopped (heartbeat, NMT,
// SDO), sends EMCY 6200h on a PLC stop and 0000h when the PLC runs again, and is replaced only when
// a PLC start brings a different configuration.

#ifndef CANOPEN_SLAVE_H
#define CANOPEN_SLAVE_H

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "can_link.h"
#include "config.h"
#include "image_io.h"
#include "pi_mutex.h"
#include "plugin_types.h"
#include "status.h"

namespace lely
{
namespace io
{
class VirtualCanController;
}
} // namespace lely

namespace canopen_plugin
{

// The slave's way to the bus. read() returns 1 with a frame, 0 on timeout, -1 on an error.
class SlaveLink
{
  public:
    struct Frame
    {
        uint32_t id = 0; // 11-bit; kRtr for a remote frame
        uint8_t dlc = 0;
        uint8_t data[8]{};
    };
    static constexpr uint32_t kRtr = 0x8000;

    virtual ~SlaveLink() = default;
    // Opens the link; false with a reason (the slave retries).
    virtual bool open(std::string &problem)        = 0;
    virtual void close()                           = 0;
    virtual int read(Frame &frame, int timeout_ms) = 0;
    // 0 when the frame was queued.
    virtual int write(const Frame &frame) = 0;
    // The bit rate the next open() brings the link up at (0 keeps the link's own).
    virtual void set_bitrate(unsigned bitrate)
    {
        (void)bitrate;
    }
    // A reading of the CAN interface for the bus status; false when there is none (in-process bus).
    virtual bool info(LinkInfo &out)
    {
        (void)out;
        return false;
    }
    // Takes the interface down for a reset after a bus-off: 0 or a negative errno. The next
    // open() brings it up again.
    virtual int take_down()
    {
        return -EOPNOTSUPP;
    }
    // The driver refused a bus-off auto-restart when the link was set up.
    virtual bool restart_refused() const
    {
        return false;
    }
};

// A SocketCAN interface, brought up at `bitrate` (0 keeps the link's own) like a network's.
std::unique_ptr<SlaveLink> socket_slave_link(const std::string &interface, unsigned bitrate);
// Lely's in-process bus, for the tests.
std::unique_ptr<SlaveLink> virtual_slave_link(std::shared_ptr<lely::io::VirtualCanController> bus);

class Slave
{
  public:
    Slave(const DeviceConfig &dev, std::unique_ptr<SlaveLink> link);
    ~Slave();
    Slave(const Slave &)            = delete;
    Slave &operator=(const Slave &) = delete;

    // Builds the object dictionary and starts the thread. False with a reason.
    bool start(std::string &problem);
    // Ends the thread (the slave leaves the bus). Idempotent.
    void stop();

    const DeviceConfig &config() const
    {
        return dev_;
    }

    // ---- PLC scan (no allocation, blocking or logging) ----
    // Values from the master into %I.
    void copy_to_plc(const plugin_runtime_args_t &rt);
    // %Q into the values to the master.
    void copy_from_plc(const plugin_runtime_args_t &rt);

    // PLC start and stop (EMCY 0000h and 6200h, and the values to the master on a stop).
    void set_plc_running(bool running);

    // Forgets the node ID and bit rate an LSS master stored and restarts the slave with the
    // project's values (done by the slave thread). False when the slave's LSS is off.
    bool forget_lss();

    SlaveSnapshot snapshot() const;

  private:
    struct Impl;
    void thread_main();
    // The node ID and bit rate to start with (stored, else the project's); slave thread or start().
    void load_lss_values();
    bool store_lss(uint8_t node_id, uint16_t kbit);

    DeviceConfig dev_;
    std::unique_ptr<SlaveLink> link_;
    std::unique_ptr<Impl> impl_;
    TripleBuffer in_;  // values from the master, slave thread -> scan
    TripleBuffer out_; // values to the master, scan -> slave thread
    std::atomic<bool> plc_running_{false};
    std::atomic<bool> stop_{false};
    std::atomic<bool> forget_{false};
    // LSS: pending node ID (0xFF unconfigured) and bit rate (kbit/s) as CANopenNode sees them, the
    // serial number, the stored values and the bit rate the link runs at. Slave thread only after
    // start().
    uint8_t pending_node_  = 0;
    uint16_t pending_kbit_ = 0;
    uint32_t serial_       = 0;
    bool serial_auto_      = false;
    uint8_t start_node_     = 0xFF;    // the node ID this start began with
    const char *start_from_ = "project";
    unsigned link_bitrate_ = 0;
    unsigned stored_node_  = 0;
    unsigned stored_rate_  = 0;
    std::string store_path_;
    std::thread thread_;
    mutable PiMutex status_mutex_;
    SlaveSnapshot status_;
};

} // namespace canopen_plugin

#endif // CANOPEN_SLAVE_H
