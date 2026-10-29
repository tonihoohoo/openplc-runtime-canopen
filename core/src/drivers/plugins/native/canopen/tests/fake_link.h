// SPDX-License-Identifier: Apache-2.0
//
// fake_link.h - a CAN interface stood in for by memory, for prepare_link and the bus thread: its
// state, bit rate and restart-ms, the calls made to it, and drivers that refuse restart-ms or
// any change.

#ifndef CANOPEN_TEST_FAKE_LINK_H
#define CANOPEN_TEST_FAKE_LINK_H

#include <cerrno>
#include <mutex>
#include <string>

#include "can_link.h"

namespace canopen_test
{

// enum can_state values; its header clashes with Lely's.
constexpr unsigned kCanErrorActive = 0;
constexpr unsigned kCanBusOff      = 3;

class FakeLink : public canopen_plugin::LinkOps
{
  public:
    FakeLink()
    {
        info_.kind          = "can";
        info_.up            = true;
        info_.bitrate       = 500000;
        info_.restart_ms    = 0;
        info_.has_can_state = true;
        info_.can_state     = kCanErrorActive;
    }

    int get(const std::string &, canopen_plugin::LinkInfo &out) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        out = info_;
        return 0;
    }
    int set_up(const std::string &, bool up) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (refuse_all_)
            return -EPERM;
        if (!up)
            ++downs_;
        else if (!info_.up)
            ++ups_;
        info_.up = up;
        if (up)
            info_.can_state = kCanErrorActive; // a reset clears the bus-off
        return 0;
    }
    int set_bitrate(const std::string &, unsigned bitrate, long restart_ms) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++set_calls_;
        if (refuse_all_)
            return -EPERM;
        if (restart_ms >= 0 && no_restart_support_)
            return -EOPNOTSUPP;
        if (bitrate)
            info_.bitrate = bitrate;
        if (restart_ms >= 0)
            info_.restart_ms = restart_ms;
        return 0;
    }

    void set(bool up, unsigned bitrate, long restart_ms)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        info_.up         = up;
        info_.bitrate    = bitrate;
        info_.restart_ms = restart_ms;
    }
    void set_can_state(unsigned state)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        info_.can_state = state;
    }
    void no_restart_support()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        no_restart_support_ = true;
    }
    void refuse_all()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        refuse_all_ = true;
    }
    canopen_plugin::LinkInfo info()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_;
    }
    int downs()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return downs_;
    }
    int ups()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return ups_;
    }
    int set_calls()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return set_calls_;
    }

  private:
    std::mutex mutex_;
    canopen_plugin::LinkInfo info_;
    bool no_restart_support_ = false;
    bool refuse_all_         = false;
    int downs_               = 0;
    int ups_                 = 0;
    int set_calls_           = 0;
};

} // namespace canopen_test

#endif // CANOPEN_TEST_FAKE_LINK_H
