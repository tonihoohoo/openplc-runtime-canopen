// SPDX-License-Identifier: Apache-2.0
//
// bus_off_recovery.h - when the master resets a faulty CAN link itself: a bus-off link the kernel
// does not restart (restart-ms off or refused by the driver, or still bus-off 2 s past the delay),
// a link at a wrong bit rate or a stalled one, with a back-off while a fault returns soon after
// each reset. No I/O; fed with link readings.

#ifndef CANOPEN_BUS_OFF_RECOVERY_H
#define CANOPEN_BUS_OFF_RECOVERY_H

#include <chrono>
#include <cstdint>

namespace canopen_plugin
{

// Why a link needs a reset.
enum class LinkFault : uint8_t
{
    None,
    BusOff,
    BitRate,  // at another bit rate than the configured one
    TxStalled // sends nothing while dropping frames
};

class BusOffRecovery
{
  public:
    using clock = std::chrono::steady_clock;

    static constexpr std::chrono::milliseconds kKernelGrace{2000};
    static constexpr std::chrono::milliseconds kFirstWait{1000};
    static constexpr std::chrono::milliseconds kMaxWait{10000};
    static constexpr std::chrono::milliseconds kSoon{10000};    // bus-off again this soon: back off
    static constexpr std::chrono::milliseconds kHealthy{60000}; // this long without: no wait

    // A reading of the link and its fault. `restart_ms` is the link's auto-restart delay (0: off,
    // -1: none); `restart_refused` says the driver has none. True when the link should be reset
    // now.
    bool reading(LinkFault fault, long restart_ms, bool restart_refused, clock::time_point now);
    // A reset was made (or tried) at `now`.
    void reset_done(clock::time_point now);
    // While a due reset is held back: when it may happen.
    clock::time_point next_reset() const
    {
        return last_reset_ + wait_;
    }
    // True once per fault when a due reset is held back by the back-off.
    bool held_back_news();
    std::chrono::milliseconds wait() const
    {
        return wait_;
    }

  private:
    bool faulty_ = false;
    clock::time_point since_{};
    bool have_reset_ = false;
    clock::time_point last_reset_{};
    std::chrono::milliseconds wait_{0};
    bool held_      = false;
    bool held_told_ = false;
};

} // namespace canopen_plugin

#endif // CANOPEN_BUS_OFF_RECOVERY_H
