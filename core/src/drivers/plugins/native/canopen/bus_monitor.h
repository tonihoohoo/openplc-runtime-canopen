// SPDX-License-Identifier: Apache-2.0
//
// bus_monitor.h - the CAN bus part of a network's status: controller state, error counters,
// bus-off count, frame counts and an estimated bus load, from rtnetlink readings of the link.
// Logs each state change. Runs on the bus thread, once a second.
//
// Derived from canworks ec7fd59d, plugin/src/canopen/bus_monitor.{h,cpp}.

#ifndef CANOPEN_BUS_MONITOR_H
#define CANOPEN_BUS_MONITOR_H

#include <chrono>
#include <cstdint>
#include <string>

#include "can_link.h"
#include "status.h"

namespace canopen_plugin
{

class BusMonitor
{
  public:
    using clock = std::chrono::steady_clock;

    explicit BusMonitor(std::string interface);

    // A reading of the link; returns the bus status. Counters count from the first reading.
    BusStatus update(const LinkInfo &li, clock::time_point now);
    // The link cannot be used.
    BusStatus unusable(BusState state);
    // Lely's in-process bus (tests): error active, nothing counted.
    BusStatus simulated();

  private:
    void on_change(BusState from, BusState to);

    std::string interface_;
    BusState state_ = BusState::Unknown;
    bool have_base_ = false;
    LinkInfo base_;
    LinkInfo last_;
    clock::time_point last_at_{};
    uint32_t bus_off_     = 0;
    bool warned_counters_ = false;
};

// Bits on the wire for `frames` standard frames carrying `bytes` data bytes in all: 47 bits of
// frame and interframe space each plus the data, without stuffing.
double bus_load_percent(uint64_t frames, uint64_t bytes, double seconds, unsigned bitrate);

} // namespace canopen_plugin

#endif // CANOPEN_BUS_MONITOR_H
