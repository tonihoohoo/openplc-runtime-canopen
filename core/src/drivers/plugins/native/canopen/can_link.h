// SPDX-License-Identifier: Apache-2.0
//
// can_link.h - the SocketCAN link the master opens, read and set over rtnetlink (no iproute2):
// the configured bit rate and the link up. A vcan link has no bit rate and is only brought up.
// Changing the link needs CAP_NET_ADMIN. Reading also returns the controller state, error
// counters and frame statistics for the bus status. Some drivers have no bus-off auto-restart and
// refuse restart-ms; the master then resets the link itself after a bus-off.

#ifndef CANOPEN_CAN_LINK_H
#define CANOPEN_CAN_LINK_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace canopen_plugin
{

struct LinkInfo
{
    bool up = false;
    std::string kind; // IFLA_INFO_KIND: "can", "vcan", ...
    unsigned bitrate = 0;
    long restart_ms  = -1; // bus-off auto-restart delay; 0: off, -1: not reported (vcan)
    // Bus diagnostics, kind "can" only unless noted.
    bool has_can_state  = false; // IFLA_CAN_STATE present
    unsigned can_state  = 0;     // enum can_state: 0 error-active ... 3 bus-off, 4 stopped
    bool has_berr       = false; // IFLA_CAN_BERR_COUNTER present (the driver reports counters)
    unsigned tx_errors  = 0;
    unsigned rx_errors  = 0;
    bool has_xstats     = false; // IFLA_INFO_XSTATS (struct can_device_stats) present
    uint32_t bus_off    = 0;     // bus-off events since the link was created
    bool has_stats      = false; // IFLA_STATS64 present (every kind)
    uint64_t rx_frames  = 0;
    uint64_t tx_frames  = 0;
    uint64_t rx_bytes   = 0;
    uint64_t tx_bytes   = 0;
    uint64_t rx_dropped = 0; // dropped and overrun receive frames
};

// Parses one RTM_NEWLINK message (header included) into `out`. Returns false if it is not one.
// Exposed for tests.
bool parse_newlink(const void *msg, size_t len, LinkInfo &out);

// Each returns 0 or a negative errno (-ENODEV: no such interface, -EPERM: not permitted).
int link_get(const std::string &name, LinkInfo &out);
int link_set_up(const std::string &name, bool up);
// `bitrate` 0 keeps the link's bit rate; `restart_ms` -1 keeps its restart delay. The link must
// be down.
int link_set_bitrate(const std::string &name, unsigned bitrate, long restart_ms);

// The link calls, behind an interface so tests can stand in for a CAN interface.
class LinkOps
{
  public:
    virtual ~LinkOps()                                                                  = default;
    virtual int get(const std::string &name, LinkInfo &out)                             = 0;
    virtual int set_up(const std::string &name, bool up)                                = 0;
    virtual int set_bitrate(const std::string &name, unsigned bitrate, long restart_ms) = 0;
};
// The real calls above.
LinkOps &system_link_ops();

enum class LinkResult
{
    Ready,
    Missing,
    NotPermitted,
    Failed
};

// Bus-off auto-restart delay set whenever the master configures the link, unless refused.
constexpr long kRestartMs = 100;

// What prepare_link does to a link in state `li` for the configured `bitrate` (0: keep). With
// `restart_refused` the driver has no auto-restart, so restart-ms is never asked for.
struct LinkPlan
{
    bool set_bitrate = false; // take the link down and set the bit rate (with kRestartMs)
    bool set_restart = false; // set kRestartMs only (the link is down)
    bool bring_up    = false;
    bool no_restart  = false; // left up as it is, with auto-restart off
};
LinkPlan plan_link(const LinkInfo &li, unsigned bitrate, bool restart_refused = false);

struct LinkSetup
{
    bool no_restart      = false; // out: left up as it is, with auto-restart off
    bool restart_refused = false; // in/out: the driver refused restart-ms (kept per PLC start)
};

// Makes the link usable: sets `bitrate` (0 keeps the link's own) when it differs, turns on the
// bus-off auto-restart when it brings a CAN link up with it off, and brings the link up. When the
// driver refuses restart-ms it repeats the change without it and sets `setup.restart_refused`.
// Logs what it changes; `problem` says why it is not Ready. A link found already up is left as it
// is; `setup.no_restart` is then set when its auto-restart is off.
LinkResult prepare_link(LinkOps &ops, const std::string &name, unsigned bitrate,
                        std::string &problem, LinkSetup &setup);

} // namespace canopen_plugin

#endif // CANOPEN_CAN_LINK_H
