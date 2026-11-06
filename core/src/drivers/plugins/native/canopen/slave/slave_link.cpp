// SPDX-License-Identifier: Apache-2.0
//
// slave_link.cpp - the slave's ways to the bus: a SocketCAN interface and Lely's in-process bus
// for the tests.

#include "slave.h"

#include <cerrno>
#include <cstring>

#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/vcan.hpp>

#include "can_link.h"

namespace canopen_plugin
{

namespace
{

class SocketLink : public SlaveLink
{
  public:
    SocketLink(std::string interface, unsigned bitrate)
        : interface_(std::move(interface)), bitrate_(bitrate)
    {
    }
    ~SocketLink() override
    {
        close();
    }

    bool open(std::string &problem) override
    {
        close();
        if (prepare_link(system_link_ops(), interface_, bitrate_, problem, setup_) !=
            LinkResult::Ready)
            return false;
        fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
        if (fd_ < 0)
        {
            problem = std::string("cannot open a CAN socket: ") + std::strerror(errno);
            return false;
        }
        sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = static_cast<int>(if_nametoindex(interface_.c_str()));
        if (addr.can_ifindex == 0 ||
            bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
        {
            problem = "cannot bind to " + interface_ + ": " + std::strerror(errno);
            close();
            return false;
        }
        return true;
    }

    void close() override
    {
        if (fd_ >= 0)
            ::close(fd_);
        fd_ = -1;
    }

    int read(Frame &frame, int timeout_ms) override
    {
        pollfd p{fd_, POLLIN, 0};
        int rc = poll(&p, 1, timeout_ms);
        if (rc == 0)
            return 0;
        if (rc < 0)
            return errno == EINTR ? 0 : -1;
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
            return -1;
        can_frame f{};
        ssize_t n = ::read(fd_, &f, sizeof(f));
        if (n < 0)
            return errno == EAGAIN || errno == EINTR ? 0 : -1;
        if (n != static_cast<ssize_t>(sizeof(f)) || (f.can_id & (CAN_EFF_FLAG | CAN_ERR_FLAG)))
            return 0;
        frame.id  = (f.can_id & CAN_SFF_MASK) | ((f.can_id & CAN_RTR_FLAG) ? kRtr : 0);
        frame.dlc = f.can_dlc > 8 ? 8 : f.can_dlc;
        std::memcpy(frame.data, f.data, 8);
        return 1;
    }

    void set_bitrate(unsigned bitrate) override
    {
        bitrate_ = bitrate;
    }

    int write(const Frame &frame) override
    {
        can_frame f{};
        f.can_id  = (frame.id & CAN_SFF_MASK) | ((frame.id & kRtr) ? CAN_RTR_FLAG : 0);
        f.can_dlc = frame.dlc;
        std::memcpy(f.data, frame.data, 8);
        return ::write(fd_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f)) ? 0 : -1;
    }

    bool info(LinkInfo &out) override
    {
        return system_link_ops().get(interface_, out) == 0;
    }

    int take_down() override
    {
        return system_link_ops().set_up(interface_, false);
    }

    bool restart_refused() const override
    {
        return setup_.restart_refused;
    }

  private:
    std::string interface_;
    unsigned bitrate_;
    int fd_ = -1;
    LinkSetup setup_; // restart_refused is kept across reopens
};

class VirtualLink : public SlaveLink
{
  public:
    explicit VirtualLink(std::shared_ptr<lely::io::VirtualCanController> bus) : bus_(bus) {}

    bool open(std::string &) override
    {
        chan_.reset(new lely::io::VirtualCanChannel(ctx_, loop_.get_executor()));
        chan_->open(*bus_);
        return true;
    }
    void close() override
    {
        chan_.reset();
    }
    int read(Frame &frame, int timeout_ms) override
    {
        can_msg msg = CAN_MSG_INIT;
        std::error_code ec;
        if (chan_->read(&msg, nullptr, nullptr, timeout_ms, ec) != 1)
            return 0;
        if (msg.flags & CAN_FLAG_IDE)
            return 0;
        frame.id  = (msg.id & 0x7FF) | ((msg.flags & CAN_FLAG_RTR) ? kRtr : 0);
        frame.dlc = msg.len > 8 ? 8 : msg.len;
        std::memcpy(frame.data, msg.data, 8);
        return 1;
    }
    int write(const Frame &frame) override
    {
        can_msg msg = CAN_MSG_INIT;
        msg.id      = frame.id & 0x7FF;
        if (frame.id & kRtr)
            msg.flags |= CAN_FLAG_RTR;
        msg.len = frame.dlc;
        std::memcpy(msg.data, frame.data, 8);
        std::error_code ec;
        chan_->write(msg, 100, ec);
        return ec ? -1 : 0;
    }

  private:
    std::shared_ptr<lely::io::VirtualCanController> bus_;
    lely::io::Context ctx_;
    lely::ev::Loop loop_;
    std::unique_ptr<lely::io::VirtualCanChannel> chan_;
};

} // namespace

std::unique_ptr<SlaveLink> socket_slave_link(const std::string &interface, unsigned bitrate)
{
    return std::unique_ptr<SlaveLink>(new SocketLink(interface, bitrate));
}

std::unique_ptr<SlaveLink> virtual_slave_link(std::shared_ptr<lely::io::VirtualCanController> bus)
{
    return std::unique_ptr<SlaveLink>(new VirtualLink(bus));
}

} // namespace canopen_plugin
