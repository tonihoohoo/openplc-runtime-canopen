// SPDX-License-Identifier: Apache-2.0
//
// What the master does to a CAN link before it opens it: the bit rate, the bus-off auto-restart and
// bringing the link up, also on drivers that refuse restart-ms. The netlink calls themselves need
// a real CAN interface.

#include <cerrno>

#include "can_link.h"
#include "check.h"
#include "fake_link.h"
#include "log.h"

using namespace canopen_plugin;
using canopen_test::FakeLink;

namespace
{

LinkInfo link(const char *kind, bool up, unsigned bitrate, long restart_ms)
{
    LinkInfo li;
    li.kind       = kind;
    li.up         = up;
    li.bitrate    = bitrate;
    li.restart_ms = restart_ms;
    return li;
}

int g_infos = 0;

void count_sink(LogLevel level, const char *)
{
    g_infos += level == LogLevel::Info;
}

void check_prepare()
{
    std::string problem;

    // Another bit rate on a driver with auto-restart: set with restart-ms, brought up.
    {
        FakeLink l;
        l.set(true, 250000, 0);
        LinkSetup setup;
        CHECK(prepare_link(l, "can0", 500000, problem, setup) == LinkResult::Ready);
        CHECK(l.info().up && l.info().bitrate == 500000 && l.info().restart_ms == kRestartMs);
        CHECK(!setup.restart_refused);
    }

    // The driver refuses restart-ms: the bit rate is set without it, once noted and remembered.
    {
        FakeLink l;
        l.no_restart_support();
        l.set(false, 250000, 0);
        LinkSetup setup;
        CHECK(prepare_link(l, "can0", 500000, problem, setup) == LinkResult::Ready);
        CHECK(l.info().up && l.info().bitrate == 500000 && l.info().restart_ms == 0);
        CHECK(setup.restart_refused);
        CHECK(l.set_calls() == 2);
        // Later bring-ups do not ask again.
        l.set(false, 125000, 0);
        CHECK(prepare_link(l, "can0", 500000, problem, setup) == LinkResult::Ready);
        CHECK(l.set_calls() == 3 && l.info().bitrate == 500000);
        l.set(false, 500000, 0);
        CHECK(prepare_link(l, "can0", 500000, problem, setup) == LinkResult::Ready);
        CHECK(l.set_calls() == 3 && l.info().up);
    }

    // Down at the right rate with restart off, refused: brought up anyway.
    {
        FakeLink l;
        l.no_restart_support();
        l.set(false, 500000, 0);
        LinkSetup setup;
        g_infos = 0;
        CHECK(prepare_link(l, "can0", 500000, problem, setup) == LinkResult::Ready);
        CHECK(l.info().up && setup.restart_refused);
        CHECK(g_infos == 2); // the refusal, then the link up
    }

    // Up from outside without restart: left alone and reported.
    {
        FakeLink l;
        LinkSetup setup;
        CHECK(prepare_link(l, "can0", 500000, problem, setup) == LinkResult::Ready);
        CHECK(setup.no_restart && l.set_calls() == 0);
    }

    // Not permitted.
    {
        FakeLink l;
        l.refuse_all();
        l.set(false, 250000, 0);
        LinkSetup setup;
        CHECK(prepare_link(l, "can0", 500000, problem, setup) == LinkResult::NotPermitted);
    }
}

} // namespace

int main()
{
    set_log_sink(count_sink);
    check_prepare();

    // Another bit rate: set it, with the restart delay, and bring the link up.
    LinkPlan p = plan_link(link("can", true, 250000, 0), 500000);
    CHECK(p.set_bitrate && !p.set_restart && p.bring_up && !p.no_restart);

    // A down link at the right rate with restart off: turn restart on, bring it up.
    p = plan_link(link("can", false, 500000, 0), 500000);
    CHECK(!p.set_bitrate && p.set_restart && p.bring_up);
    p = plan_link(link("can", false, 500000, 0), 0);
    CHECK(!p.set_bitrate && p.set_restart && p.bring_up);

    // A down link that already restarts: only bring it up.
    p = plan_link(link("can", false, 500000, 100), 500000);
    CHECK(!p.set_bitrate && !p.set_restart && p.bring_up);

    // A link that is up at the right rate is left alone; restart off is reported.
    p = plan_link(link("can", true, 500000, 0), 500000);
    CHECK(!p.set_bitrate && !p.set_restart && !p.bring_up && p.no_restart);
    p = plan_link(link("can", true, 500000, 200), 0);
    CHECK(!p.bring_up && !p.no_restart);

    // vcan has neither a bit rate nor restart: only brought up.
    p = plan_link(link("vcan", false, 0, -1), 500000);
    CHECK(!p.set_bitrate && !p.set_restart && p.bring_up);
    p = plan_link(link("vcan", true, 0, -1), 500000);
    CHECK(!p.bring_up && !p.no_restart);

    // A driver without auto-restart is never asked to turn it on.
    p = plan_link(link("can", false, 500000, 0), 500000, true);
    CHECK(!p.set_bitrate && !p.set_restart && p.bring_up);
    return TEST_RESULT();
}
