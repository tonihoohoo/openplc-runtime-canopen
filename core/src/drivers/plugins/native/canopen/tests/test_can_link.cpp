// SPDX-License-Identifier: Apache-2.0
//
// What the master does to a CAN link before it opens it: the bit rate, the bus-off auto-restart and
// bringing the link up. The netlink calls themselves need a real CAN interface.

#include "can_link.h"
#include "check.h"

using namespace canopen_plugin;

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

} // namespace

int main()
{
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
    return TEST_RESULT();
}
