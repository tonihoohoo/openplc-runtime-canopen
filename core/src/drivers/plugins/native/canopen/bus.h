// SPDX-License-Identifier: Apache-2.0
//
// bus.h - the bus thread of one network: prepares the CAN link (bit rate, up), opens the SocketCAN
// interface, runs the network's master on a Lely event loop, and keeps retrying while the
// interface is missing, down or cannot be configured. With a virtual controller (tests) the master
// runs on Lely's in-process bus instead.
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

#include "config.h"
#include "master_dcf.h"
#include "process_image.h"

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
        std::string dcf_path, std::shared_ptr<lely::io::VirtualCanController> vbus = nullptr);
    ~Bus();

    void start();
    // Sends the network's NMT command for a PLC stop, ends the session and joins. Idempotent.
    void stop();

  private:
    static constexpr std::chrono::milliseconds kLoopSlice{100};
    static constexpr std::chrono::milliseconds kRetry{1000};
    static constexpr std::chrono::milliseconds kStopDrain{50};
    static constexpr int kShutdownSlices = 20;

    void thread_main();
    void run_session();
    bool link_lost() const;
    bool wait_for(std::chrono::milliseconds d); // false if stop was requested

    const NetworkConfig &net_;
    const MasterLayout &layout_;
    ProcessImage &image_;
    std::string dcf_path_;
    std::shared_ptr<lely::io::VirtualCanController> vbus_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
};

} // namespace canopen_plugin

#endif // CANOPEN_BUS_H
