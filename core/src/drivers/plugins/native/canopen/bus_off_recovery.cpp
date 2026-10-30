// SPDX-License-Identifier: Apache-2.0

#include "bus_off_recovery.h"

#include <algorithm>

namespace canopen_plugin
{

constexpr std::chrono::milliseconds BusOffRecovery::kKernelGrace;
constexpr std::chrono::milliseconds BusOffRecovery::kFirstWait;
constexpr std::chrono::milliseconds BusOffRecovery::kMaxWait;
constexpr std::chrono::milliseconds BusOffRecovery::kSoon;
constexpr std::chrono::milliseconds BusOffRecovery::kHealthy;

bool BusOffRecovery::reading(LinkFault fault, long restart_ms, bool restart_refused,
                             clock::time_point now)
{
    if (fault == LinkFault::None)
    {
        faulty_ = false;
        held_   = false;
        return false;
    }
    if (!faulty_)
    {
        faulty_    = true;
        since_     = now;
        held_      = false;
        held_told_ = false;
        if (have_reset_)
        {
            auto gap = now - last_reset_;
            if (gap < kSoon)
                wait_ = wait_.count() ? std::min(wait_ * 2, kMaxWait) : kFirstWait;
            else if (gap >= kHealthy)
                wait_ = std::chrono::milliseconds(0);
        }
    }
    // The kernel restarts a bus-off link only.
    if (fault == LinkFault::BusOff && !restart_refused && restart_ms > 0 &&
        now - since_ < std::chrono::milliseconds(restart_ms) + kKernelGrace)
        return false;
    if (have_reset_ && now < last_reset_ + wait_)
    {
        held_ = true;
        return false;
    }
    return true;
}

void BusOffRecovery::reset_done(clock::time_point now)
{
    have_reset_ = true;
    last_reset_ = now;
    faulty_     = false;
    held_       = false;
}

bool BusOffRecovery::held_back_news()
{
    if (!held_ || held_told_)
        return false;
    held_told_ = true;
    return true;
}

} // namespace canopen_plugin
