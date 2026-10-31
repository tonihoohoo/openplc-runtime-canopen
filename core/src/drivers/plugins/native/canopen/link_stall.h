// SPDX-License-Identifier: Apache-2.0
//
// link_stall.h - whether a CAN link stopped sending: up and error-active or error-warning, no frame
// sent for 3 s and, in that time, transmit frames dropped or the master's writes failing without
// one being confirmed. A quiet link drops nothing and writes nothing, so it never counts. No I/O;
// fed with link readings.

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

    // No write failing: the value of `failing_since` for a master whose writes go out.
    static constexpr clock::time_point kNotFailing = clock::time_point::max();

    // A reading of the link's state, its sent and dropped transmit frame counts, and since when the
    // master's writes have failed with none confirmed (kNotFailing if not). True while the link
    // counts as stalled.
    bool reading(BusState state, uint64_t tx_frames, uint64_t tx_dropped,
                 clock::time_point failing_since, clock::time_point now);
    // True if the last stall reported included dropped frames.
    bool dropped() const
    {
        return dropped_;
    }
    // Forgets the readings (after a reset, or when the link reports no counts).
    void clear()
    {
        readings_.clear();
        dropped_ = false;
    }

  private:
    struct Reading
    {
        clock::time_point at;
        uint64_t tx_frames;
        uint64_t tx_dropped;
    };
    std::deque<Reading> readings_;
    bool dropped_ = false;
};

// Since when the master's CAN writes have failed with none confirmed as sent; fed with Lely's write
// results (an error, or the first confirmed write after errors) on the bus thread.
class WriteFailures
{
  public:
    using clock = LinkStall::clock;

    void result(bool ok, clock::time_point now)
    {
        if (ok)
            since_ = LinkStall::kNotFailing;
        else if (since_ == LinkStall::kNotFailing)
            since_ = now;
    }
    // LinkStall::kNotFailing while writes go out.
    clock::time_point since() const
    {
        return since_;
    }

  private:
    clock::time_point since_ = LinkStall::kNotFailing;
};

} // namespace canopen_plugin

#endif // CANOPEN_LINK_STALL_H
