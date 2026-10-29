// SPDX-License-Identifier: Apache-2.0
//
// When the master resets a bus-off link: at once when the kernel does not restart it, after the
// kernel's own delay plus 2 s when it should, and with a back-off from 1 s doubling to 10 s while
// the bus goes bus-off again soon after each reset, back to no wait after 60 s without bus-off.

#include "bus_off_recovery.h"
#include "check.h"

using namespace canopen_plugin;
using std::chrono::milliseconds;

namespace
{

using tp = BusOffRecovery::clock::time_point;

tp at(long ms)
{
    return tp(milliseconds(ms));
}

// Bus-off from `start`, read once a second, until the policy resets; the reset time or -1.
long reset_at(BusOffRecovery &r, long start, long restart_ms = 0, bool refused = false)
{
    for (long t = start; t < start + 120000; t += 1000)
        if (r.reading(BusState::BusOff, restart_ms, refused, at(t)))
        {
            r.reset_done(at(t));
            return t;
        }
    return -1;
}

} // namespace

int main()
{
    // Healthy readings never reset.
    {
        BusOffRecovery r;
        CHECK(!r.reading(BusState::ErrorActive, 0, false, at(0)));
        CHECK(!r.reading(BusState::ErrorPassive, 0, false, at(1000)));
    }

    // No kernel restart: the first reset is at once.
    {
        BusOffRecovery r;
        CHECK(reset_at(r, 1000) == 1000);
    }

    // The driver refused restart-ms: at once, whatever the link reports.
    {
        BusOffRecovery r;
        CHECK(reset_at(r, 1000, 100, true) == 1000);
    }

    // Kernel restart on and working: never reset.
    {
        BusOffRecovery r;
        CHECK(!r.reading(BusState::BusOff, 100, false, at(0)));
        CHECK(!r.reading(BusState::BusOff, 100, false, at(1000)));
        CHECK(!r.reading(BusState::ErrorActive, 100, false, at(2000)));
    }

    // Kernel restart on but stuck: reset once restart-ms + 2 s have passed.
    {
        BusOffRecovery r;
        CHECK(reset_at(r, 0, 100) == 3000);
    }

    // Persistent fault: bus-off again 1 s after each reset; waits 1, 2, 4, 8, 10, 10 s.
    {
        BusOffRecovery r;
        long t = reset_at(r, 0);
        CHECK(t == 0);
        long expected[] = {1000, 2000, 4000, 8000, 10000, 10000};
        for (long wait : expected)
        {
            long back = t + 1000;
            long next = reset_at(r, back);
            CHECK(next - t == (wait > 1000 ? wait : 1000));
            CHECK(r.wait() == milliseconds(wait));
            t = next;
        }
    }

    // A held reset is told once per bus-off.
    {
        BusOffRecovery r;
        reset_at(r, 0);
        CHECK(!r.reading(BusState::BusOff, 0, false, at(500)));
        CHECK(r.held_back_news());
        CHECK(!r.held_back_news());
        CHECK(r.reading(BusState::BusOff, 0, false, at(1500)));
    }

    // Healthy for 60 s after a long back-off: the next reset is at once again.
    {
        BusOffRecovery r;
        long t = reset_at(r, 0);
        for (int i = 0; i < 5; ++i)
            t = reset_at(r, t + 1000);
        CHECK(r.wait() == milliseconds(10000));
        CHECK(!r.reading(BusState::ErrorActive, 0, false, at(t + 30000)));
        CHECK(reset_at(r, t + 60000) == t + 60000);
        CHECK(r.wait() == milliseconds(0));
    }

    // Between 10 and 60 s: the wait stays as it was.
    {
        BusOffRecovery r;
        long t = reset_at(r, 0);
        t      = reset_at(r, t + 1000);
        t      = reset_at(r, t + 1000);
        CHECK(r.wait() == milliseconds(2000));
        CHECK(reset_at(r, t + 20000) == t + 20000);
        CHECK(r.wait() == milliseconds(2000));
    }
    return TEST_RESULT();
}
