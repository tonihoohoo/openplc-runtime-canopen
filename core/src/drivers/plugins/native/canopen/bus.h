// SPDX-License-Identifier: Apache-2.0
//
// bus.h - the bus thread of one network: prepares the CAN link (bit rate, up), opens the SocketCAN
// interface, runs the network's master on a Lely event loop, and keeps retrying while the
// interface is missing, down or cannot be configured. A bus-off link the kernel does not restart is
// reset (taken down and up) with a back-off, ending the session. A session runs at the network's
// SCHED_FIFO priority with 1 ns timer slack; preparing the link and waiting to retry run at
// SCHED_OTHER. With a virtual controller (tests) the master runs on Lely's in-process bus instead;
// `links` then stands in for the CAN interface's state, or none is read.
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
#include "config.h"
#include "master_dcf.h"
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

// The signal Lely's poll uses to wake a bus thread. Not SIGUSR1, which the runtime uses for its
// own tasks: each poll replaces the signal's handler while it lives and puts the old one back.
int wake_signal();
// Installs a no-op handler for wake_signal(); call before the first bus thread starts.
void prepare_wake_signal();
// Ignores wake_signal() again; call after the last bus thread ended (the handler is in this
// library, which the runtime unloads).
void release_wake_signal();

class Bus
{
  public:
    Bus(const NetworkConfig &net, const MasterLayout &layout, ProcessImage &image,
        NetworkStatus &status, std::string dcf_path,
        std::shared_ptr<lely::io::VirtualCanController> vbus = nullptr, LinkOps *links = nullptr);
    ~Bus();

    void start();
    // Sends the network's NMT command for a PLC stop, ends the session and joins. Idempotent.
    void stop();
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
    // Resets the link if a bus-off reading calls for it; true if the link was taken down.
    bool reset_after_bus_off(BusState state, const LinkInfo &info);
    bool wait_for(std::chrono::milliseconds d); // false if stop was requested

    const NetworkConfig &net_;
    const MasterLayout &layout_;
    ProcessImage &image_;
    NetworkStatus &status_;
    BusMonitor monitor_;
    std::string dcf_path_;
    std::shared_ptr<lely::io::VirtualCanController> vbus_;
    LinkOps &links_;
    bool use_link_; // a CAN interface (real or stood in for) is read and prepared
    LinkSetup setup_;
    BusOffRecovery recovery_;
    bool resetting_       = false; // the session ends for a link reset
    bool reset_forbidden_ = false; // not permitted to reset; said once
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
