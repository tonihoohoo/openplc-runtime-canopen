// SPDX-License-Identifier: Apache-2.0
//
// Derived from canworks ec7fd59d, plugin/src/canopen/bus_monitor.cpp.

#include "bus_monitor.h"

#include <linux/can/netlink.h>

#include "log.h"

namespace canopen_plugin
{

namespace
{

BusState state_of(const LinkInfo &li)
{
    if (!li.up)
        return BusState::Down;
    if (li.kind != "can" || !li.has_can_state)
        return BusState::ErrorActive;
    switch (li.can_state)
    {
    case CAN_STATE_ERROR_ACTIVE:
        return BusState::ErrorActive;
    case CAN_STATE_ERROR_WARNING:
        return BusState::ErrorWarning;
    case CAN_STATE_ERROR_PASSIVE:
        return BusState::ErrorPassive;
    case CAN_STATE_BUS_OFF:
        return BusState::BusOff;
    default:
        return BusState::Stopped; // stopped, sleeping
    }
}

uint64_t delta(uint64_t now, uint64_t base)
{
    // A re-created link restarts its counters.
    return now >= base ? now - base : now;
}

} // namespace

double bus_load_percent(uint64_t frames, uint64_t bytes, double seconds, unsigned bitrate)
{
    if (seconds <= 0 || bitrate == 0)
        return 0;
    double bits = static_cast<double>(frames) * 47.0 + static_cast<double>(bytes) * 8.0;
    double load = bits / (seconds * bitrate) * 100.0;
    return load > 100.0 ? 100.0 : load;
}

BusMonitor::BusMonitor(std::string interface) : interface_(std::move(interface)) {}

BusStatus BusMonitor::update(const LinkInfo &li, clock::time_point now)
{
    BusStatus b;
    b.state      = state_of(li);
    bool is_can  = li.kind == "can";
    b.has_resets = is_can;
    if (!have_base_)
    {
        have_base_ = true;
        base_      = li;
        last_      = li;
        last_at_   = now;
    }
    if (is_can && li.has_berr)
    {
        b.has_counters = true;
        b.tx_errors    = li.tx_errors;
        b.rx_errors    = li.rx_errors;
    }
    else if (is_can && !warned_counters_)
    {
        warned_counters_ = true;
        log_info("CAN interface %s does not report error counters", interface_.c_str());
    }
    if (is_can && li.has_xstats)
    {
        uint32_t offs = static_cast<uint32_t>(delta(li.bus_off, base_.bus_off));
        if (offs > bus_off_ && b.state != BusState::BusOff)
            log_error("CAN interface %s went bus-off and recovered", interface_.c_str());
        bus_off_      = offs;
        b.has_bus_off = true;
        b.bus_off     = bus_off_;
    }
    if (li.has_stats)
    {
        b.has_frames = true;
        b.rx_frames  = delta(li.rx_frames, base_.rx_frames);
        b.tx_frames  = delta(li.tx_frames, base_.tx_frames);
        b.rx_dropped = delta(li.rx_dropped, base_.rx_dropped);
        double secs  = std::chrono::duration<double>(now - last_at_).count();
        if (is_can && li.bitrate && secs > 0)
        {
            uint64_t frames =
                delta(li.rx_frames, last_.rx_frames) + delta(li.tx_frames, last_.tx_frames);
            uint64_t bytes =
                delta(li.rx_bytes, last_.rx_bytes) + delta(li.tx_bytes, last_.tx_bytes);
            b.has_load     = true;
            b.load_percent = bus_load_percent(frames, bytes, secs, li.bitrate);
        }
    }
    last_    = li;
    last_at_ = now;
    on_change(state_, b.state);
    state_ = b.state;
    return b;
}

BusStatus BusMonitor::unusable(BusState state)
{
    state_     = state;
    have_base_ = false;
    BusStatus b;
    b.state = state;
    return b;
}

BusStatus BusMonitor::simulated()
{
    state_ = BusState::ErrorActive;
    BusStatus b;
    b.state = BusState::ErrorActive;
    return b;
}

void BusMonitor::on_change(BusState from, BusState to)
{
    if (from == to)
        return;
    // Becoming usable is logged by the bus thread.
    if (to == BusState::ErrorActive &&
        (from == BusState::Unknown || from == BusState::Down || from == BusState::Missing))
        return;
    const char *name = interface_.c_str();
    switch (to)
    {
    case BusState::ErrorActive:
        log_info("CAN interface %s is error-active again", name);
        break;
    case BusState::ErrorWarning:
    case BusState::ErrorPassive:
        log_warn("CAN interface %s is %s (error counters high; check wiring, termination and bit "
                 "rate)",
                 name, to == BusState::ErrorWarning ? "error-warning" : "error-passive");
        break;
    case BusState::BusOff:
        log_error("CAN interface %s is bus-off", name);
        break;
    case BusState::Stopped:
        log_warn("CAN interface %s is stopped", name);
        break;
    default:
        break;
    }
}

} // namespace canopen_plugin
