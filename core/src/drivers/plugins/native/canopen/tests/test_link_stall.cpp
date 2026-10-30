// SPDX-License-Identifier: Apache-2.0
//
// When a CAN link counts as stalled: error-active or error-warning, no frame sent for 3 s and
// frames dropped in that time; never while quiet, error-passive or bus-off.

#include "check.h"
#include "link_stall.h"

using namespace canopen_plugin;
using std::chrono::milliseconds;

namespace
{

LinkStall::clock::time_point at(long ms)
{
    return LinkStall::clock::time_point(milliseconds(ms));
}

} // namespace

int main()
{
    // Nothing sent, drops rising: stalled once 3 s of readings agree.
    {
        LinkStall s;
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, at(0)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 5, at(1000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 9, at(2000)));
        CHECK(s.reading(BusState::ErrorActive, 10, 14, at(3000)));
    }

    // Error-warning counts as well.
    {
        LinkStall s;
        for (long t = 0; t < 3000; t += 1000)
            s.reading(BusState::ErrorWarning, 10, static_cast<uint64_t>(t), at(t));
        CHECK(s.reading(BusState::ErrorWarning, 10, 3000, at(3000)));
    }

    // Quiet: nothing sent, nothing dropped.
    {
        LinkStall s;
        for (long t = 0; t <= 10000; t += 1000)
            CHECK(!s.reading(BusState::ErrorActive, 10, 7, at(t)));
    }

    // Frames still go out: never stalled, whatever is dropped.
    {
        LinkStall s;
        for (long t = 0; t <= 10000; t += 1000)
            CHECK(!s.reading(BusState::ErrorActive, static_cast<uint64_t>(t),
                             static_cast<uint64_t>(t), at(t)));
    }

    // Error-passive or bus-off: not a stall, and the readings start again after it.
    {
        LinkStall s;
        s.reading(BusState::ErrorActive, 10, 0, at(0));
        s.reading(BusState::ErrorActive, 10, 1, at(1000));
        CHECK(!s.reading(BusState::ErrorPassive, 10, 2, at(2000)));
        CHECK(!s.reading(BusState::BusOff, 10, 3, at(3000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 4, at(4000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 5, at(6000)));
        CHECK(s.reading(BusState::ErrorActive, 10, 6, at(7000)));
    }

    // A frame sent restarts the window.
    {
        LinkStall s;
        s.reading(BusState::ErrorActive, 10, 0, at(0));
        s.reading(BusState::ErrorActive, 10, 1, at(1000));
        CHECK(!s.reading(BusState::ErrorActive, 11, 2, at(2000)));
        CHECK(!s.reading(BusState::ErrorActive, 11, 3, at(4000)));
        CHECK(s.reading(BusState::ErrorActive, 11, 4, at(5000)));
    }

    // Drops only long ago: the old readings fall out of the window.
    {
        LinkStall s;
        s.reading(BusState::ErrorActive, 10, 0, at(0));
        s.reading(BusState::ErrorActive, 10, 5, at(1000));
        for (long t = 2000; t <= 10000; t += 1000)
            s.reading(BusState::ErrorActive, 10, 5, at(t));
        CHECK(!s.reading(BusState::ErrorActive, 10, 5, at(11000)));
    }

    // Cleared: starts again.
    {
        LinkStall s;
        for (long t = 0; t < 3000; t += 1000)
            s.reading(BusState::ErrorActive, 10, static_cast<uint64_t>(t), at(t));
        s.clear();
        CHECK(!s.reading(BusState::ErrorActive, 10, 3000, at(3000)));
    }
    return TEST_RESULT();
}
