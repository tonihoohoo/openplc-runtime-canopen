// SPDX-License-Identifier: Apache-2.0

#include "bus.h"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <sys/prctl.h>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/io2/vcan.hpp>

#include "can_link.h"
#include "log.h"
#include "network.h"

namespace canopen_plugin
{

namespace
{
void wake_noop(int) {}
// The PLC's networks and service mode can overlap while one closes and the other starts.
std::mutex wake_mutex;
unsigned wake_users = 0;
} // namespace

int wake_signal()
{
    return SIGRTMIN + 4;
}

void prepare_wake_signal()
{
    std::lock_guard<std::mutex> lock(wake_mutex);
    if (wake_users++)
        return;
    struct sigaction sa = {};
    sa.sa_handler       = wake_noop;
    sigemptyset(&sa.sa_mask);
    sigaction(wake_signal(), &sa, nullptr);
}

void release_wake_signal()
{
    std::lock_guard<std::mutex> lock(wake_mutex);
    if (wake_users == 0 || --wake_users)
        return;
    struct sigaction sa = {};
    sa.sa_handler       = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sigaction(wake_signal(), &sa, nullptr);
}

constexpr std::chrono::milliseconds Bus::kLoopSlice;
constexpr std::chrono::milliseconds Bus::kRetry;
constexpr std::chrono::milliseconds Bus::kStopDrain;
constexpr int Bus::kShutdownSlices;
constexpr int Bus::kMonitorSlices;
constexpr std::chrono::milliseconds Bus::kResetPause;

Bus::Bus(const NetworkConfig &net, const MasterLayout &layout, ProcessImage &image,
         NetworkStatus &status, std::string dcf_path,
         std::shared_ptr<lely::io::VirtualCanController> vbus, LinkOps *links, bool service)
    : net_(net), layout_(layout), image_(image), status_(status), monitor_(net.interface),
      dcf_path_(std::move(dcf_path)), vbus_(std::move(vbus)),
      links_(links ? *links : system_link_ops()), service_(service), use_link_(!vbus_ || links)
{
}

Bus::~Bus()
{
    stop();
}

void Bus::start()
{
    if (thread_.joinable())
        return;
    stop_   = false;
    thread_ = std::thread([this] { thread_main(); });
}

void Bus::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable())
        thread_.join();
}

bool Bus::wait_for(std::chrono::milliseconds d)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, d, [this] { return stop_.load(); });
    return !stop_;
}

void Bus::enable_plc(uint8_t index)
{
    if (service_)
        return;
    std::vector<unsigned> ids;
    for (const NodeConfig &n : net_.nodes)
        ids.push_back(n.node_id);
    plc_.reset(new PlcPort(new_nonce(), index, static_cast<uint8_t>(net_.master_node_id),
                           ids.data(), ids.size()));
}

void Bus::publish_bus(const BusStatus &bus)
{
    status_.set_bus(bus);
    if (!plc_)
        return;
    uint8_t k = CANOPEN_PLC_NO_ERROR;
    switch (bus.state)
    {
    case BusState::ErrorPassive:
        k = CANOPEN_PLC_ERROR_PASSIVE;
        break;
    case BusState::BusOff:
        k = CANOPEN_PLC_BUS_OFF;
        break;
    case BusState::Stopped:
    case BusState::Down:
    case BusState::Missing:
        k = CANOPEN_PLC_NO_DRIVER;
        break;
    default:
        break;
    }
    plc_->set_kernel(k);
}

void Bus::set_sessions_running(bool running)
{
    client_.set_running(running);
    if (plc_)
        plc_->set_running(running);
}

bool Bus::link_lost(bool monitor)
{
    if (!use_link_)
        return false;
    LinkInfo info;
    int rc    = links_.get(net_.interface, info);
    bool lost = rc != 0 || !info.up;
    if (lost)
    {
        publish_bus(monitor_.unusable(rc == -ENODEV ? BusState::Missing : BusState::Down));
    }
    else if (monitor)
    {
        BusStatus b = monitor_.update(info, BusMonitor::clock::now());
        publish_bus(b);
        if (info.kind == "can" && reset_if_faulty(b.state, info))
            lost = true;
    }
    return lost;
}

namespace
{

const char *fault_text(LinkFault f)
{
    switch (f)
    {
    case LinkFault::BitRate:
        return "at a wrong bit rate";
    case LinkFault::TxStalled:
        return "not sending";
    default:
        return "bus-off";
    }
}

ResetCause cause_of(LinkFault f)
{
    switch (f)
    {
    case LinkFault::BitRate:
        return ResetCause::BitRate;
    case LinkFault::TxStalled:
        return ResetCause::TxStalled;
    default:
        return ResetCause::BusOff;
    }
}

} // namespace

LinkFault Bus::fault_of(BusState state, const LinkInfo &info)
{
    if (!session_rate_)
        session_rate_ = info.bitrate;
    auto failing = session_net_ ? session_net_->WritesFailingSince() : LinkStall::kNotFailing;
    bool stalled = info.has_stats && stall_.reading(state, info.tx_frames, info.tx_dropped, failing,
                                                    LinkStall::clock::now());
    if (!info.has_stats)
        stall_.clear();
    if (state == BusState::BusOff)
        return LinkFault::BusOff;
    // A rate the kernel rounds the configured one to stays as first read.
    if (net_.bitrate && info.bitrate && info.bitrate != net_.bitrate &&
        info.bitrate != session_rate_)
        return LinkFault::BitRate;
    if (stalled)
        return LinkFault::TxStalled;
    return LinkFault::None;
}

bool Bus::reset_if_faulty(BusState state, const LinkInfo &info)
{
    const LinkFault fault = fault_of(state, info);
    if (reset_forbidden_)
        return false;
    auto now = BusOffRecovery::clock::now();
    if (!recovery_.reading(fault, info.restart_ms, setup_.restart_refused, now))
    {
        if (recovery_.held_back_news())
        {
            auto ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(recovery_.next_reset() - now)
                    .count();
            log_warn("CAN interface %s is %s again; resetting the link in %lld s",
                     net_.interface.c_str(), fault_text(fault),
                     static_cast<long long>((ms + 999) / 1000));
        }
        return false;
    }
    recovery_.reset_done(now);
    const bool dropped = stall_.dropped();
    stall_.clear();
    const char *name = net_.interface.c_str();
    const char *what = fault_text(fault);
    ResetCause cause = cause_of(fault);
    int rc           = links_.set_up(net_.interface, false);
    if (rc == 0)
    {
        if (fault == LinkFault::BitRate)
            log_error("CAN interface %s is at %u bit/s, not the configured %u; resetting the link",
                      name, info.bitrate, net_.bitrate);
        else if (fault == LinkFault::TxStalled)
            log_error("CAN interface %s stopped sending%s; resetting the link", name,
                      dropped ? " and drops frames" : "");
        else
            log_error("CAN interface %s is bus-off; resetting the link", name);
        status_.link_reset(ResetOutcome::Done, cause);
        resetting_ = true;
        return true;
    }
    if (rc == -EPERM)
    {
        reset_forbidden_ = true;
        log_error("CAN interface %s is %s and the runtime is not permitted to reset it; reset "
                  "it by hand with \"ip link set %s down\" and then \"ip link set %s up\"",
                  name, what, name, name);
        status_.link_reset(ResetOutcome::NotPermitted, cause);
        return false;
    }
    log_error("CAN interface %s is %s and resetting the link failed: %s", name, what,
              std::strerror(-rc));
    status_.link_reset(ResetOutcome::Failed, cause);
    return false;
}

namespace
{
// The kernel's default timer slack, which SCHED_OTHER threads normally have.
constexpr unsigned long kDefaultSlackNs = 50000;
} // namespace

void Bus::enter_realtime()
{
    sched_param sp{};
    sp.sched_priority = net_.priority;
    int rc            = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0)
    {
        if (!realtime_warned_)
            log_warn("cannot use real-time priority %d (%s); the CANopen bus runs at normal "
                     "priority",
                     net_.priority, std::strerror(rc));
        realtime_warned_ = true;
        return;
    }
    // Default slack becomes wake-up jitter on every timer the session waits for.
    prctl(PR_SET_TIMERSLACK, 1UL, 0, 0, 0);
    realtime_ = true;
}

void Bus::leave_realtime()
{
    if (!realtime_)
        return;
    sched_param sp{};
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
    prctl(PR_SET_TIMERSLACK, kDefaultSlackNs, 0, 0, 0);
    realtime_ = false;
}

void Bus::thread_main()
{
    pthread_setname_np(pthread_self(), "canopen_bus");
    // The thread inherits whatever started the plugin; link setup runs at normal scheduling.
    {
        sched_param sp{};
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
        prctl(PR_SET_TIMERSLACK, kDefaultSlackNs, 0, 0, 0);
    }
    ScopedLogPrefix prefix(net_.interface + ": ");
    std::string last_problem;
    while (!stop_)
    {
        if (use_link_)
        {
            std::string problem;
            LinkResult r = prepare_link(links_, net_.interface, net_.bitrate, problem, setup_);
            if (setup_.no_restart && !restart_noted_)
            {
                log_info(
                    "automatic bus-off restart is off on CAN interface %s; should it go bus-off, "
                    "the master resets the link",
                    net_.interface.c_str());
                restart_noted_ = true;
            }
            if (r != LinkResult::Ready)
            {
                if (problem != last_problem)
                    log_error("%s; the nodes are not operational, retrying", problem.c_str());
                status_.begin_session();
                publish_bus(monitor_.unusable(r == LinkResult::Missing ? BusState::Missing
                                                                       : BusState::Down));
                last_problem = problem;
                if (!wait_for(kRetry))
                    break;
                continue;
            }
            if (!last_problem.empty())
                log_info("CAN interface %s is ready", net_.interface.c_str());
            last_problem.clear();
        }
        enter_realtime();
        run_session();
        leave_realtime();
        bool reset = resetting_;
        resetting_ = false;
        if (reset)
            last_problem = "link reset"; // "is ready" follows
        if (!wait_for(reset ? kResetPause : kRetry))
            break;
    }
}

void Bus::run_session()
{
    try
    {
        lely::io::IoGuard io_guard;
        lely::io::Context ctx;
        lely::io::Poll poll(ctx, wake_signal());
        lely::ev::Loop loop(poll.get_poll());
        auto exec = loop.get_executor();
        lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);
        lely::io::Timer tick(poll, exec, CLOCK_MONOTONIC);
        // Lely's channel classes have no common virtual destructor: own each as what it is.
        std::unique_ptr<lely::io::VirtualCanChannel> vchan;
        std::unique_ptr<lely::io::CanController> ctrl;
        std::unique_ptr<lely::io::CanChannel> chan;
        lely::io::CanChannelBase *base = nullptr;
        if (vbus_)
        {
            vchan.reset(new lely::io::VirtualCanChannel(ctx, exec));
            vchan->open(*vbus_);
            base = vchan.get();
        }
        else
        {
            ctrl.reset(new lely::io::CanController(net_.interface.c_str()));
            chan.reset(new lely::io::CanChannel(poll, exec));
            chan->open(*ctrl);
            base = chan.get();
        }

        Network net(exec, poll, timer, tick, *base, dcf_path_, net_, layout_, image_, status_,
                    &client_, service_, plc_.get());
        session_net_  = &net;
        session_rate_ = 0;
        stall_.clear();
        if (use_link_)
            link_lost(true);
        else
            publish_bus(monitor_.simulated());
        net.Start();
        set_sessions_running(true);
        if (!session_failure_.empty())
        {
            log_info("the CANopen session runs again");
            session_failure_.clear();
        }
        bool shut_down = false;
        bool lost      = false;
        int slices     = 0;
        int polls      = 0;
        while (true)
        {
            loop.run_for(kLoopSlice);
            if (loop.stopped())
                break;
            bool monitor = ++polls % kMonitorSlices == 0;
            if (!shut_down && (stop_ || link_lost(monitor)))
            {
                if (stop_)
                {
                    net.StopNodes();
                    loop.run_for(kStopDrain);
                }
                else
                {
                    lost = true;
                }
                set_sessions_running(false);
                // Cancels every pending operation, after which the loop drains and stops.
                ctx.shutdown();
                shut_down = true;
            }
            else if (shut_down && ++slices >= kShutdownSlices)
            {
                // Expected while a faulty link is reset: writes stuck in the adapter never finish.
                if (resetting_)
                    log_debug("the CANopen session did not end cleanly");
                else
                    log_warn("the CANopen session did not end cleanly");
                loop.stop();
                break;
            }
        }
        set_sessions_running(false);
        session_net_ = nullptr;
        net.MarkAllDown();
        if (lost && !resetting_)
            log_error("CAN interface %s went down; the nodes are not operational, retrying",
                      net_.interface.c_str());
    }
    catch (const std::exception &e)
    {
        session_net_ = nullptr;
        set_sessions_running(false);
        for (size_t slot = 0; slot < net_.nodes.size(); ++slot)
            image_.set_node_state(slot, 0);
        image_.commit();
        status_.begin_session();
        // Retried every second: the same failure is logged once.
        if (session_failure_ != e.what())
        {
            session_failure_ = e.what();
            log_error("CANopen session failed: %s; retrying", e.what());
        }
        else
            log_debug("CANopen session failed again: %s", e.what());
    }
}

} // namespace canopen_plugin
