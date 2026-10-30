// SPDX-License-Identifier: Apache-2.0
//
// link_stall.h - whether a CAN link stopped sending: up and error-active or error-warning, no frame
// sent for 3 s and transmit frames dropped in that time. A quiet link drops nothing, so it never
// counts. No I/O; fed with link readings.

#ifndef CANOPEN_LINK_STALL_H
#define CANOPEN_LINK_STALL_H

#include <chrono>
#include <cstdint>
#include <deque>

#include "status.h"

namespace canopen_plugin
{

class LinkStall
{
  public:
    using clock = std::chrono::steady_clock;

    static constexpr std::chrono::milliseconds kWindow{3000};

    // A reading of the link's state and its sent and dropped transmit frame counts. True while the
    // link counts as stalled.
    bool reading(BusState state, uint64_t tx_frames, uint64_t tx_dropped, clock::time_point now);
    // Forgets the readings (after a reset, or when the link reports no counts).
    void clear()
    {
        readings_.clear();
    }

  private:
    struct Reading
    {
        clock::time_point at;
        uint64_t tx_frames;
        uint64_t tx_dropped;
    };
    std::deque<Reading> readings_;
};

} // namespace canopen_plugin

#endif // CANOPEN_LINK_STALL_H
