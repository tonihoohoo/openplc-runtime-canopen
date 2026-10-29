// SPDX-License-Identifier: Apache-2.0
//
// Derived from canworks ec7fd59d, plugin/src/can/can_adapter.cpp (SocketCAN over rtnetlink).

#include "can_link.h"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <linux/can/netlink.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "log.h"

namespace canopen_plugin
{

namespace
{

// A request: header, ifinfomsg, then attributes appended in place.
struct NlRequest
{
    explicit NlRequest(uint16_t type, uint16_t flags)
    {
        std::memset(buf, 0, sizeof(buf));
        nh()->nlmsg_len   = NLMSG_LENGTH(sizeof(ifinfomsg));
        nh()->nlmsg_type  = type;
        nh()->nlmsg_flags = NLM_F_REQUEST | flags;
        ifi()->ifi_family = AF_UNSPEC;
    }
    nlmsghdr *nh()
    {
        return reinterpret_cast<nlmsghdr *>(buf);
    }
    ifinfomsg *ifi()
    {
        return static_cast<ifinfomsg *>(NLMSG_DATA(nh()));
    }

    rtattr *add(uint16_t type, const void *data, size_t len)
    {
        size_t at     = NLMSG_ALIGN(nh()->nlmsg_len);
        rtattr *rta   = reinterpret_cast<rtattr *>(buf + at);
        rta->rta_type = type;
        rta->rta_len  = RTA_LENGTH(len);
        if (len)
            std::memcpy(RTA_DATA(rta), data, len);
        nh()->nlmsg_len = at + RTA_ALIGN(rta->rta_len);
        return rta;
    }
    rtattr *begin_nest(uint16_t type)
    {
        return add(type, nullptr, 0);
    }
    void end_nest(rtattr *nest)
    {
        nest->rta_len =
            static_cast<unsigned short>(buf + nh()->nlmsg_len - reinterpret_cast<char *>(nest));
    }

    alignas(nlmsghdr) char buf[512];
};

class Socket
{
  public:
    Socket()
    {
        fd_ = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
        if (fd_ >= 0)
        {
            timeval tv{2, 0};
            setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        }
    }
    ~Socket()
    {
        if (fd_ >= 0)
            close(fd_);
    }
    int fd() const
    {
        return fd_;
    }

  private:
    int fd_;
};

void parse_linkinfo(const rtattr *linkinfo, LinkInfo &out)
{
    int len = RTA_PAYLOAD(linkinfo);
    for (const rtattr *a = static_cast<const rtattr *>(RTA_DATA(linkinfo)); RTA_OK(a, len);
         a               = RTA_NEXT(a, len))
    {
        if (a->rta_type == IFLA_INFO_KIND)
        {
            out.kind.assign(static_cast<const char *>(RTA_DATA(a)),
                            strnlen(static_cast<const char *>(RTA_DATA(a)), RTA_PAYLOAD(a)));
        }
        else if (a->rta_type == IFLA_INFO_DATA)
        {
            int dlen = RTA_PAYLOAD(a);
            for (const rtattr *d = static_cast<const rtattr *>(RTA_DATA(a)); RTA_OK(d, dlen);
                 d               = RTA_NEXT(d, dlen))
            {
                if (d->rta_type == IFLA_CAN_BITTIMING && RTA_PAYLOAD(d) >= sizeof(can_bittiming))
                {
                    can_bittiming bt;
                    std::memcpy(&bt, RTA_DATA(d), sizeof(bt));
                    out.bitrate = bt.bitrate;
                }
            }
        }
    }
}

void parse_ifinfo(const nlmsghdr *nh, LinkInfo &out)
{
    const ifinfomsg *ifi = static_cast<const ifinfomsg *>(NLMSG_DATA(nh));
    out.up               = (ifi->ifi_flags & IFF_UP) != 0;
    int alen             = IFLA_PAYLOAD(nh);
    for (const rtattr *a = IFLA_RTA(ifi); RTA_OK(a, alen); a = RTA_NEXT(a, alen))
        if (a->rta_type == IFLA_LINKINFO)
            parse_linkinfo(a, out);
}

// Sends `req` and reads the reply. With `info`, expects an RTM_NEWLINK and
// parses it; otherwise expects an ACK. Returns 0 or -errno.
int transact(NlRequest &req, LinkInfo *info)
{
    Socket s;
    if (s.fd() < 0)
        return -errno;
    sockaddr_nl addr{};
    addr.nl_family      = AF_NETLINK;
    static unsigned seq = 0;
    req.nh()->nlmsg_seq = ++seq;
    if (sendto(s.fd(), req.buf, req.nh()->nlmsg_len, 0, reinterpret_cast<sockaddr *>(&addr),
               sizeof(addr)) < 0)
        return -errno;
    alignas(nlmsghdr) char reply[16384];
    for (;;)
    {
        ssize_t n = recv(s.fd(), reply, sizeof(reply), 0);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            return -errno;
        }
        int len = static_cast<int>(n);
        for (nlmsghdr *nh = reinterpret_cast<nlmsghdr *>(reply); NLMSG_OK(nh, len);
             nh           = NLMSG_NEXT(nh, len))
        {
            if (nh->nlmsg_seq != req.nh()->nlmsg_seq)
                continue;
            if (nh->nlmsg_type == NLMSG_ERROR)
            {
                const nlmsgerr *err = static_cast<const nlmsgerr *>(NLMSG_DATA(nh));
                return err->error; // 0 = ACK
            }
            if (info && nh->nlmsg_type == RTM_NEWLINK)
            {
                parse_ifinfo(nh, *info);
                return 0;
            }
            if (nh->nlmsg_type == NLMSG_DONE)
                return -ENODEV;
        }
    }
}

} // namespace

int link_get(const std::string &name, LinkInfo &out)
{
    out = LinkInfo();
    if (name.size() >= IFNAMSIZ)
        return -ENODEV;
    NlRequest req(RTM_GETLINK, 0);
    req.add(IFLA_IFNAME, name.c_str(), name.size() + 1);
    return transact(req, &out);
}

int link_set_up(const std::string &name, bool up)
{
    unsigned index = if_nametoindex(name.c_str());
    if (!index)
        return -ENODEV;
    NlRequest req(RTM_NEWLINK, NLM_F_ACK);
    req.ifi()->ifi_index  = static_cast<int>(index);
    req.ifi()->ifi_change = IFF_UP;
    req.ifi()->ifi_flags  = up ? IFF_UP : 0;
    return transact(req, nullptr);
}

int link_set_bitrate(const std::string &name, unsigned bitrate, long restart_ms)
{
    unsigned index = if_nametoindex(name.c_str());
    if (!index)
        return -ENODEV;
    // What `ip link set <name> type can bitrate <rate> restart-ms <ms>` sends.
    NlRequest req(RTM_NEWLINK, NLM_F_ACK);
    req.ifi()->ifi_index = static_cast<int>(index);
    rtattr *linkinfo     = req.begin_nest(IFLA_LINKINFO);
    req.add(IFLA_INFO_KIND, "can", 3);
    rtattr *data = req.begin_nest(IFLA_INFO_DATA);
    can_bittiming bt{};
    bt.bitrate = bitrate;
    req.add(IFLA_CAN_BITTIMING, &bt, sizeof(bt));
    if (restart_ms >= 0)
    {
        uint32_t ms = static_cast<uint32_t>(restart_ms);
        req.add(IFLA_CAN_RESTART_MS, &ms, sizeof(ms));
    }
    req.end_nest(data);
    req.end_nest(linkinfo);
    return transact(req, nullptr);
}

LinkResult prepare_link(const std::string &name, unsigned bitrate, std::string &problem)
{
    LinkInfo li;
    int rc = link_get(name, li);
    if (rc == -ENODEV)
    {
        problem = "CAN interface " + name + " does not exist";
        return LinkResult::Missing;
    }
    if (rc < 0)
    {
        problem = "cannot read CAN interface " + name + ": " + std::strerror(-rc);
        return LinkResult::Failed;
    }
    bool change_rate = li.kind == "can" && bitrate && li.bitrate != bitrate;
    if (!change_rate && li.up)
        return LinkResult::Ready;
    if (change_rate)
    {
        if (li.up && (rc = link_set_up(name, false)) < 0)
            goto fail;
        if ((rc = link_set_bitrate(name, bitrate, kRestartMs)) < 0)
            goto fail;
        log_info("set CAN interface %s to %u bit/s", name.c_str(), bitrate);
    }
    if ((rc = link_set_up(name, true)) < 0)
        goto fail;
    log_info("brought CAN interface %s up", name.c_str());
    return LinkResult::Ready;
fail:
    if (rc == -EPERM)
    {
        problem = "not permitted to configure CAN interface " + name +
                  " (the runtime needs CAP_NET_ADMIN, or set the link up beforehand at the "
                  "configured bit rate)";
        return LinkResult::NotPermitted;
    }
    problem = "cannot configure CAN interface " + name + ": " + std::strerror(-rc);
    return LinkResult::Failed;
}

} // namespace canopen_plugin
