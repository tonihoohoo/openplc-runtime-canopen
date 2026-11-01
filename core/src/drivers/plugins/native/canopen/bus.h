// SPDX-License-Identifier: Apache-2.0
//
// bus.h - the bus thread of one network: prepares the CAN link (bit rate, up), opens the SocketCAN
// interface, runs the network's master on a Lely event loop, and keeps retrying while the
// interface is missing, down or cannot be configured. A link that is bus-off and not restarted by
// the kernel, at another bit rate than configured, or stalled (nothing sent while frames are
// dropped or the master's writes fail), is reset (taken down and up) with a back-off, ending the
// session. A session runs at the network's SCHED_FIFO priority with 1 ns timer slack; preparing the
// link and waiting to retry run at SCHED_OTHER. With a virtual controller (tests) the master runs
// on Lely's in-process bus instead; `links` then stands in for the CAN interface's state, or none
// is read.
//
// Derived in design from canworks ec7fd59d, plugin/src/canopen/bus.h.

#ifndef CANOPEN_BUS_H
#define CANOPEN_BUS_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "bus_monitor.h"
#include "bus_off_recovery.h"
#include "can_link.h"
#include "client.h"
#include "config.h"
#include "link_stall.h"
#include "master_dcf.h"
#include "plc_port.h"
#include "process_image.h"
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

class Network;

// The signal Lely's poll uses to wake a bus thread. Not SIGUSR1, which the runtime uses for its
// own tasks: each poll replaces the signal's handler while it lives and puts the old one back.
int wake_signal();
// Installs a no-op handler for wake_signal(); call before a set of bus threads starts. Counted, so
// sets may overlap.
void prepare_wake_signal();
// Ignores wake_signal() again once every set that prepared it has ended (the handler is in this
// library, which the runtime unloads).
void release_wake_signal();

class Bus
{
  public:
    Bus(const NetworkConfig &net, const MasterLayout &layout, ProcessImage &image,
        NetworkStatus &status, std::string dcf_path,
        std::shared_ptr<lely::io::VirtualCanController> vbus = nullptr, LinkOps *links = nullptr,
        bool service = false);
    ~Bus();

    void start();
    // Sends the network's NMT command for a PLC stop, ends the session and joins. Idempotent.
    void stop();
    // Client requests to this network (the online tools); served while a session runs.
    ClientQueue &client()
    {
        return client_;
    }
    // Opens the network to the PLC's function blocks as network `index` (0-based); call before
    // start(). Not in service mode.
    void enable_plc(uint8_t index);
    // The blocks' side of the network, or null.
    PlcPort *plc()
    {
        return plc_.get();
    }
    // True while a session runs at the real-time priority.
    bool realtime() const
    {
        return realtime_.load();
    }

  private:
    static constexpr std::chrono::milliseconds kLoopSlice{100};
    static constexpr std::chrono::milliseconds kRetry{1000};
    static constexpr std::chrono::milliseconds kStopDrain{50};
    static constexpr int kShutdownSlices = 20;
    static constexpr int kMonitorSlices  = 10;                   // bus status once a second
    static constexpr std::chrono::milliseconds kResetPause{100}; // link down during a reset

    void thread_main();
    void enter_realtime();
    void leave_realtime();
    void run_session();
    // Reads the link; true if it is missing or down. Updates the bus status when `monitor`.
    bool link_lost(bool monitor);
    // The fault in a monitor reading of a CAN link, if any.
    LinkFault fault_of(BusState state, const LinkInfo &info);
    // Resets the link if a reading calls for it; true if the link was taken down.
    bool reset_if_faulty(BusState state, const LinkInfo &info);
    bool wait_for(std::chrono::milliseconds d); // false if stop was requested
    // Records the bus state in the status and for the blocks.
    void publish_bus(const BusStatus &bus);
    void set_sessions_running(bool running);

    const NetworkConfig &net_;
    const MasterLayout &layout_;
    ProcessImage &image_;
    NetworkStatus &status_;
    BusMonitor monitor_;
    std::string dcf_path_;
    std::shared_ptr<lely::io::VirtualCanController> vbus_;
    LinkOps &links_;
    bool service_; // service mode: the master runs no node, only client requests
    ClientQueue client_;
    std::unique_ptr<PlcPort> plc_;
    bool use_link_; // a CAN interface (real or stood in for) is read and prepared
    LinkSetup setup_;
    BusOffRecovery recovery_;
    LinkStall stall_;
    const Network *session_net_ = nullptr; // the running session's master, read for write failures
    unsigned session_rate_      = 0;       // the link's bit rate first read in this session
    bool resetting_             = false;   // the session ends for a link reset
    bool reset_forbidden_       = false;   // not permitted to reset; said once
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> realtime_{false};
    bool realtime_warned_ = false;
    bool restart_noted_   = false;
    std::mutex mutex_;
    std::condition_variable cv_;
};

} // namespace canopen_plugin

#endif // CANOPEN_BUS_H
