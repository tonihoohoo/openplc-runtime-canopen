// SPDX-License-Identifier: Apache-2.0
//
// can_link.h - the SocketCAN link the master opens, read and set over rtnetlink (no iproute2):
// the configured bit rate and the link up. A vcan link has no bit rate and is only brought up.
// Changing the link needs CAP_NET_ADMIN.

#ifndef CANOPEN_CAN_LINK_H
#define CANOPEN_CAN_LINK_H

#include <string>

namespace canopen_plugin
{

struct LinkInfo
{
    bool up = false;
    std::string kind; // IFLA_INFO_KIND: "can", "vcan", ...
    unsigned bitrate = 0;
    long restart_ms  = -1; // bus-off auto-restart delay; 0: off, -1: not reported (vcan)
};

// Each returns 0 or a negative errno (-ENODEV: no such interface, -EPERM: not permitted).
int link_get(const std::string &name, LinkInfo &out);
int link_set_up(const std::string &name, bool up);
// `bitrate` 0 keeps the link's bit rate; `restart_ms` -1 keeps its restart delay. The link must
// be down.
int link_set_bitrate(const std::string &name, unsigned bitrate, long restart_ms);

enum class LinkResult
{
    Ready,
    Missing,
    NotPermitted,
    Failed
};

// Bus-off auto-restart delay set whenever the master configures the link.
constexpr long kRestartMs = 100;

// Makes the link usable: sets `bitrate` (0 keeps the link's own) when it differs, turns on the
// bus-off auto-restart when it brings a CAN link up with it off, and brings the link up. Logs what
// it changes; `problem` says why it is not Ready. A link found already up is left as it is;
// `no_restart` is then set when its auto-restart is off.
LinkResult prepare_link(const std::string &name, unsigned bitrate, std::string &problem,
                        bool *no_restart = nullptr);

} // namespace canopen_plugin

#endif // CANOPEN_CAN_LINK_H
