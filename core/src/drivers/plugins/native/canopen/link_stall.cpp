// SPDX-License-Identifier: Apache-2.0

#include "link_stall.h"

namespace canopen_plugin
{

constexpr std::chrono::milliseconds LinkStall::kWindow;
constexpr LinkStall::clock::time_point LinkStall::kNotFailing;

bool LinkStall::reading(BusState state, uint64_t tx_frames, uint64_t tx_dropped,
                        clock::time_point failing_since, clock::time_point now)
{
    dropped_ = false;
    if (state != BusState::ErrorActive && state != BusState::ErrorWarning)
    {
        readings_.clear();
        return false;
    }
    if (!readings_.empty())
    {
        const Reading &last = readings_.back();
        // Counters went back (a re-created link) or a frame went out: start again.
        if (tx_frames != last.tx_frames || tx_dropped < last.tx_dropped)
            readings_.clear();
    }
    readings_.push_back({now, tx_frames, tx_dropped});
    // Keep one reading at least kWindow old, and none older than needed.
    while (readings_.size() > 2 && now - readings_[1].at >= kWindow)
        readings_.pop_front();
    const Reading &first = readings_.front();
    if (now - first.at < kWindow)
        return false;
    dropped_           = tx_dropped > first.tx_dropped;
    bool writes_failed = failing_since != kNotFailing && now - failing_since >= kWindow;
    return dropped_ || writes_failed;
}

} // namespace canopen_plugin
