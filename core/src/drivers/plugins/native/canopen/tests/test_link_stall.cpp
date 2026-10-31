// SPDX-License-Identifier: Apache-2.0
//
// When a CAN link counts as stalled: error-active or error-warning, no frame sent for 3 s and
// frames dropped or the master's writes failing in that time; never while quiet, error-passive or
// bus-off.

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

const LinkStall::clock::time_point kNone = LinkStall::kNotFailing;

} // namespace

int main()
{
    // Nothing sent, drops rising: stalled once 3 s of readings agree.
    {
        LinkStall s;
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, kNone, at(0)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 5, kNone, at(1000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 9, kNone, at(2000)));
        CHECK(s.reading(BusState::ErrorActive, 10, 14, kNone, at(3000)));
        CHECK(s.dropped());
    }

    // Nothing sent, nothing dropped, writes failing all along: stalled after 3 s, without drops.
    {
        LinkStall s;
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, at(0), at(0)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, at(0), at(1000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, at(0), at(2000)));
        CHECK(s.reading(BusState::ErrorActive, 10, 0, at(0), at(3000)));
        CHECK(!s.dropped());
    }

    // Writes failing for less than the window: not yet.
    {
        LinkStall s;
        for (long t = 0; t < 3000; t += 1000)
            s.reading(BusState::ErrorActive, 10, 0, kNone, at(t));
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, at(1500), at(3000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, at(1500), at(4000)));
        CHECK(s.reading(BusState::ErrorActive, 10, 0, at(1500), at(4500)));
    }

    // Writes failing but frames still going out: not a stall.
    {
        LinkStall s;
        for (long t = 0; t <= 10000; t += 1000)
            CHECK(!s.reading(BusState::ErrorActive, static_cast<uint64_t>(t), 0, at(0), at(t)));
    }

    // Writes failing while error-passive (no other node acknowledging): not a stall.
    {
        LinkStall s;
        for (long t = 0; t <= 10000; t += 1000)
            CHECK(!s.reading(BusState::ErrorPassive, 10, 0, at(0), at(t)));
    }

    // A confirmed write ends the failure.
    {
        LinkStall s;
        s.reading(BusState::ErrorActive, 10, 0, at(0), at(0));
        s.reading(BusState::ErrorActive, 10, 0, at(0), at(1000));
        s.reading(BusState::ErrorActive, 10, 0, kNone, at(2000));
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, kNone, at(3000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 0, kNone, at(6000)));
    }

    // Error-warning counts as well.
    {
        LinkStall s;
        for (long t = 0; t < 3000; t += 1000)
            s.reading(BusState::ErrorWarning, 10, static_cast<uint64_t>(t), kNone, at(t));
        CHECK(s.reading(BusState::ErrorWarning, 10, 3000, kNone, at(3000)));
    }

    // Quiet: nothing sent, nothing dropped.
    {
        LinkStall s;
        for (long t = 0; t <= 10000; t += 1000)
            CHECK(!s.reading(BusState::ErrorActive, 10, 7, kNone, at(t)));
    }

    // Frames still go out: never stalled, whatever is dropped.
    {
        LinkStall s;
        for (long t = 0; t <= 10000; t += 1000)
            CHECK(!s.reading(BusState::ErrorActive, static_cast<uint64_t>(t),
                             static_cast<uint64_t>(t), kNone, at(t)));
    }

    // Error-passive or bus-off: not a stall, and the readings start again after it.
    {
        LinkStall s;
        s.reading(BusState::ErrorActive, 10, 0, kNone, at(0));
        s.reading(BusState::ErrorActive, 10, 1, kNone, at(1000));
        CHECK(!s.reading(BusState::ErrorPassive, 10, 2, kNone, at(2000)));
        CHECK(!s.reading(BusState::BusOff, 10, 3, kNone, at(3000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 4, kNone, at(4000)));
        CHECK(!s.reading(BusState::ErrorActive, 10, 5, kNone, at(6000)));
        CHECK(s.reading(BusState::ErrorActive, 10, 6, kNone, at(7000)));
    }

    // A frame sent restarts the window.
    {
        LinkStall s;
        s.reading(BusState::ErrorActive, 10, 0, kNone, at(0));
        s.reading(BusState::ErrorActive, 10, 1, kNone, at(1000));
        CHECK(!s.reading(BusState::ErrorActive, 11, 2, kNone, at(2000)));
        CHECK(!s.reading(BusState::ErrorActive, 11, 3, kNone, at(4000)));
        CHECK(s.reading(BusState::ErrorActive, 11, 4, kNone, at(5000)));
    }

    // Drops only long ago: the old readings fall out of the window.
    {
        LinkStall s;
        s.reading(BusState::ErrorActive, 10, 0, kNone, at(0));
        s.reading(BusState::ErrorActive, 10, 5, kNone, at(1000));
        for (long t = 2000; t <= 10000; t += 1000)
            s.reading(BusState::ErrorActive, 10, 5, kNone, at(t));
        CHECK(!s.reading(BusState::ErrorActive, 10, 5, kNone, at(11000)));
    }

    // Cleared: starts again.
    {
        LinkStall s;
        for (long t = 0; t < 3000; t += 1000)
            s.reading(BusState::ErrorActive, 10, static_cast<uint64_t>(t), kNone, at(t));
        s.clear();
        CHECK(!s.reading(BusState::ErrorActive, 10, 3000, kNone, at(3000)));
    }
    // Write failures: since the first error, until a confirmed write.
    {
        WriteFailures w;
        CHECK(w.since() == LinkStall::kNotFailing);
        w.result(false, at(100));
        w.result(false, at(900));
        CHECK(w.since() == at(100));
        w.result(true, at(1000));
        CHECK(w.since() == LinkStall::kNotFailing);
        w.result(false, at(2000));
        CHECK(w.since() == at(2000));
    }
    return TEST_RESULT();
}
