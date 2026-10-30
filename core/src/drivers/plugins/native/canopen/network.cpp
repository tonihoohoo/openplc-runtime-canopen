// SPDX-License-Identifier: Apache-2.0

#include "network.h"

#include <algorithm>
#include <cstring>
#include <mutex>

#include <unistd.h>

#include <lely/can/msg.h>
#include <lely/coapp/sdo_error.hpp>
#include <lely/ev/exec.h>
#include <lely/ev/exec.hpp>
#include <lely/util/util.h>

#include "log.h"

// <lely/can/net.h> clashes with the C++ headers; only this is needed.
extern "C" int can_net_send(__can_net *net, const can_msg *msg);

namespace canopen_plugin
{

using lely::canopen::NmtCommand;
using lely::canopen::NmtState;

constexpr std::chrono::milliseconds Network::kTick;
constexpr std::chrono::milliseconds Network::kRetryMin;
constexpr std::chrono::milliseconds Network::kRetryMax;
constexpr std::chrono::milliseconds Network::kPollEvery;
constexpr uint32_t Network::kRestoreSignature;

uint64_t pdo_get_bits(const uint8_t *data, size_t len, unsigned offset, unsigned bits)
{
    uint64_t v = 0;
    for (unsigned i = 0; i < bits; ++i)
    {
        unsigned bit = offset + i;
        if (bit / 8 >= len)
            break;
        if (data[bit / 8] >> (bit % 8) & 1)
            v |= uint64_t{1} << i;
    }
    return v;
}

void pdo_set_bits(uint8_t *data, unsigned offset, unsigned bits, uint64_t value)
{
    for (unsigned i = 0; i < bits; ++i)
    {
        unsigned bit = offset + i;
        uint8_t mask = static_cast<uint8_t>(1u << (bit % 8));
        if (value >> i & 1)
            data[bit / 8] |= mask;
        else
            data[bit / 8] &= static_cast<uint8_t>(~mask);
    }
}

Network::Network(ev_exec_t *exec, lely::io::Poll &poll, lely::io::TimerBase &timer,
                 lely::io::TimerBase &tick, lely::io::CanChannelBase &chan,
                 const std::string &dcf_path, const NetworkConfig &cfg, const MasterLayout &layout,
                 ProcessImage &image, NetworkStatus &status, ClientQueue *client, bool service)
    : BasicMaster(exec, timer, chan, dcf_path, "", static_cast<uint8_t>(cfg.master_node_id)),
      exec_(exec), poll_(poll), tick_(tick), tick_wait_(exec,
                                                        [this](int, std::error_code ec)
                                                        {
                                                            if (!ec)
                                                                OnTick();
                                                        }),
      cfg_(cfg), layout_(layout), image_(image), status_(status), client_(client), service_(service)
{
    for (size_t slot = 0; slot < cfg_.nodes.size(); ++slot)
    {
        const NodeConfig &n = cfg_.nodes[slot];
        NodeState &s        = nodes_[n.node_id];
        s.cfg               = &n;
        s.slot              = slot;
        s.concise           = concise_dcf(n.sdo);
    }
    for (const MasterPdo &m : layout_.rpdos)
        nodes_[m.node_id].in_pdos.push_back(&m);
    for (const MasterPdo &m : layout_.tpdos)
        nodes_[m.node_id].out_pdos.push_back(&m);
    last_out_.assign(layout_.output_entries, 0);
    out_valid_.assign(layout_.output_entries, false);
    resend_.assign(layout_.tpdos.size(), false);
    rpdo_error_.assign(layout_.rpdos.size(), 0);
    scan_.watch   = IO_POLL_WATCH_INIT(&Network::OnScanEvent);
    scan_.task    = EV_TASK_INIT(exec, &Network::OnScanTask);
    scan_.net     = this;
    scan_.watched = false;
    scan_.posted  = false;
}

Network::~Network()
{
    tick_.cancel_wait(tick_wait_);
    if (scan_.watched)
        io_poll_watch(poll_, image_.wake_fd(), 0, &scan_.watch);
    if (scan_.posted)
        ev_exec_abort(exec_, &scan_.task);
}

void Network::Start()
{
    if (service_)
    {
        // Never reset: the master sends no boot-up, heartbeat or NMT command of its own.
        status_.begin_session();
        log_info("%s: service mode, %zu node(s)", cfg_.label().c_str(), nodes_.size());
        tick_.settime(kTick, kTick);
        tick_.submit_wait(tick_wait_);
        return;
    }
    running_      = true;
    synced_scans_ = image_.scans();
    started_      = clock::now();
    next_time_    = started_;
    for (auto &it : nodes_)
    {
        it.second.next_poll = started_ + kPollEvery; // after the boot-ups of the reset
        it.second.held      = status_.is_held(it.first);
    }
    status_.begin_session();
    // Lely's boot SDOs (identity check, configuration) otherwise wait 100 ms per attempt.
    SetTimeout(std::chrono::milliseconds(cfg_.sdo_timeout_ms));
    log_info("%s: starting, %zu node(s), %zu input and %zu output PDO(s)", cfg_.label().c_str(),
             nodes_.size(), layout_.rpdos.size(), layout_.tpdos.size());
    Reset();
    for (auto &it : nodes_)
        if (it.second.held)
            StopHeld(it.first);
    tick_.settime(kTick, kTick);
    tick_.submit_wait(tick_wait_);
    image_.drain_wake();
    WatchScans();
}

void Network::WatchScans()
{
    std::error_code ec;
    poll_.watch(image_.wake_fd(), lely::io::Event::IN, scan_.watch, ec);
    if (!ec)
        scan_.watched = true;
    else
        log_error("%s: cannot watch the PLC scans (%s); outputs follow the %lld ms tick",
                  cfg_.label().c_str(), ec.message().c_str(),
                  static_cast<long long>(kTick.count()));
}

// Called from the poll on the bus thread: hand the work to the loop without allocating.
void Network::OnScanEvent(struct io_poll_watch *watch, int events)
{
    (void)events;
    ScanHook *hook = structof(watch, ScanHook, watch);
    if (!hook->posted)
    {
        hook->posted = true;
        ev_exec_post(hook->task.exec, &hook->task);
    }
}

void Network::OnScanTask(ev_task *task)
{
    ScanHook *hook = structof(task, ScanHook, task);
    hook->posted   = false;
    Network *self  = hook->net;
    self->image_.drain_wake();
    self->WatchScans();
    self->Exchange();
}

void Network::StopNodes()
{
    if (service_)
        return;
    running_ = false;
    if (cfg_.sync == SyncMode::Period)
    {
        // Stop producing SYNC.
        std::error_code ec;
        Write<uint32_t>(0x1005, 0, 0x80, ec);
    }
    for (auto &it : nodes_)
        it.second.stopped = true;
    status_.stop_all();
    if (cfg_.on_stop == OnStop::None)
        return;
    const NmtCommand cmd =
        cfg_.on_stop == OnStop::Stopped ? NmtCommand::STOP : NmtCommand::ENTER_PREOP;
    // Addressed to each configured node: a broadcast would also reach devices this master does
    // not manage.
    for (const auto &it : nodes_)
    {
        try
        {
            Command(cmd, static_cast<uint8_t>(it.first));
        }
        catch (const std::exception &e)
        {
            log_warn("%s: cannot send the NMT command for PLC stop to node %u: %s",
                     cfg_.label().c_str(), it.first, e.what());
        }
    }
}

void Network::MarkAllDown()
{
    for (auto &it : nodes_)
        if (it.second.up)
            SetUp(it.second, false);
    image_.commit();
}

bool Network::IsUp(unsigned node_id) const
{
    auto it = nodes_.find(node_id);
    return it != nodes_.end() && it->second.up;
}

Network::NodeState *Network::Find(uint8_t id)
{
    auto it = nodes_.find(id);
    return it == nodes_.end() ? nullptr : &it->second;
}

// Lely invokes the On*() callbacks on the bus thread with the master's mutex held, and every
// BasicMaster call takes that mutex again. Callbacks that only change this class's state run
// at once, so a node is up before its first PDO; those that call into Lely post their work.
template <class F> void Network::Defer(F &&f)
{
    try
    {
        lely::ev::Executor(exec_).post(std::forward<F>(f));
    }
    catch (const std::exception &e)
    {
        log_error("cannot queue work on the bus thread: %s", e.what());
    }
}

void Network::OnBoot(uint8_t id, NmtState st, char es, const std::string &what) noexcept
{
    HandleBoot(id, st, es, what);
}

void Network::OnConfig(uint8_t id) noexcept
{
    NodeState *n = Find(id);
    if (n && n->cfg->restore_defaults && !n->restored)
    {
        Defer([this, id] { HandleRestore(id); });
        return;
    }
    if (!n || n->concise.size() <= 4)
    {
        ConfigResult(id, {});
        return;
    }
    Defer([this, id] { HandleConfig(id); });
}

void Network::OnHeartbeat(uint8_t id, bool occurred) noexcept
{
    if (occurred)
        HandleLoss(id, ProblemKind::Heartbeat);
}

void Network::OnNodeGuarding(uint8_t id, bool occurred) noexcept
{
    if (occurred)
        HandleLoss(id, ProblemKind::Guarding);
}

void Network::OnState(uint8_t id, NmtState st) noexcept
{
    // On a boot-up, drops the node's SDO client, which Lely's boot of the node needs.
    BasicMaster::OnState(id, st);
    HandleState(id, st);
}

void Network::OnRpdo(int num, std::error_code ec, const void *p, std::size_t n) noexcept
{
    if (!ec)
        HandleRpdo(num, static_cast<const uint8_t *>(p), n);
}

void Network::OnEmcy(uint8_t id, uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept
{
    uint8_t m[5];
    std::memcpy(m, msef, sizeof(m));
    Defer([this, id, eec, er, m] { HandleEmcy(id, eec, er, m); });
}

void Network::OnRpdoError(int num, uint16_t eec, uint8_t er) noexcept
{
    (void)er;
    Defer([this, num, eec] { HandleRpdoError(num, eec); });
}

void Network::HandleBoot(uint8_t id, NmtState st, char es, const std::string &what)
{
    (void)st;
    NodeState *n = Find(id);
    if (!n || n->stopped || n->held)
        return;
    n->booting = false;
    if (n->restore_pending)
    {
        // The boot ended so that the node can reset with its defaults; it boots again after.
        n->restore_pending = false;
        n->heard           = true;
        return;
    }
    if (!n->heard && (es == 'B' || es == 'K'))
    {
        // Looked for and not there (no answer, or no heartbeat): look again.
        n->next_poll = clock::now() + kPollEvery;
        return;
    }
    n->heard = true;
    if (es == 0 || es == 'L')
    {
        if (!n->last_problem.empty())
            log_info("%s is up again", n->cfg->label().c_str());
        else
            log_info("%s is up", n->cfg->label().c_str());
        n->last_problem.clear();
        n->backoff       = kRetryMin;
        n->retry_pending = false;
        n->has_abort     = false;
        status_.boot_ok(id);
        SetUp(*n, true);
        if (!n->cfg->configure)
        {
            // Lely boots such a node without starting it.
            Defer(
                [this, id]
                {
                    try
                    {
                        Command(NmtCommand::START, id);
                    }
                    catch (const std::exception &e)
                    {
                        log_warn("node %u: cannot send NMT start: %s", id, e.what());
                    }
                });
        }
        if (cfg_.on_node_error == OnNodeError::Stop)
        {
            // Out of the network list for Lely (bit 0 of 0x1F81), so that an error of the node
            // does not make Lely reset and boot it again; the plugin stops it instead.
            uint32_t value = slave_assignment(*n->cfg) & ~0x01u;
            Defer(
                [this, id, value]
                {
                    std::error_code ec;
                    Write<uint32_t>(0x1F81, id, value, ec);
                    if (ec)
                        log_warn("node %u: cannot keep the master from restarting it: %s", id,
                                 ec.message().c_str());
                });
        }
        return;
    }
    Problem p    = BootProblem(*n, es, what);
    n->has_abort = false;
    status_.boot_failed(id, p);
    if (p.kind == ProblemKind::Identity)
    {
        IdentityField field = p.field;
        Defer([this, id, field] { ReadFoundIdentity(id, field); });
    }
    std::string problem = std::string(1, es) + ": " + what;
    if (problem != n->last_problem)
    {
        log_warn("%s did not boot (error %s); retrying in the background", n->cfg->label().c_str(),
                 problem.c_str());
        n->last_problem = problem;
    }
    SetUp(*n, false);
    ScheduleRetry(*n);
}

void Network::HandleConfig(uint8_t id)
{
    NodeState *n = Find(id);
    if (!n)
        return;
    const uint8_t *begin = n->concise.data();
    const uint8_t *end   = begin + n->concise.size();
    auto con             = [this](uint8_t id, uint16_t idx, uint8_t subidx, std::error_code ec)
    {
        if (ec)
        {
            NodeState *n = Find(id);
            log_error("%s: configuration SDO to 0x%04X:%u failed: %s",
                      n ? n->cfg->label().c_str() : "node", idx, subidx, ec.message().c_str());
            // Not an abort from the node (a local error): CiA 301 "general error".
            uint32_t code = ec.category() == lely::canopen::SdoCategory()
                                ? static_cast<uint32_t>(ec.value())
                                : 0x08000000u;
            if (n)
            {
                n->has_abort        = true;
                n->abort            = Problem();
                n->abort.kind       = ProblemKind::SdoAbort;
                n->abort.index      = idx;
                n->abort.subindex   = subidx;
                n->abort.abort_code = code;
            }
            status_.sdo_abort(id, idx, subidx, code);
        }
        std::lock_guard<lely::util::BasicLockable> lock(*this);
        ConfigResult(id, ec);
    };
    std::error_code ec;
    try
    {
        SubmitWriteDcf(exec_, id, begin, end, con, std::chrono::milliseconds(cfg_.sdo_timeout_ms),
                       ec);
    }
    catch (const std::exception &e)
    {
        log_error("%s: cannot start the configuration SDOs: %s", n->cfg->label().c_str(), e.what());
        ec = std::make_error_code(std::errc::io_error);
    }
    if (ec)
    {
        std::lock_guard<lely::util::BasicLockable> lock(*this);
        ConfigResult(id, ec);
    }
}

void Network::HandleRestore(uint8_t id)
{
    NodeState *n = Find(id);
    if (!n)
        return;
    const uint8_t sub = static_cast<uint8_t>(n->cfg->restore_defaults);
    auto con          = [this, sub](uint8_t id, uint16_t idx, uint8_t subidx, std::error_code ec)
    {
        NodeState *n = Find(id);
        if (!n)
            return;
        if (ec)
        {
            log_error("%s: restoring the defaults (0x1011:%u) failed: %s", n->cfg->label().c_str(),
                      sub, ec.message().c_str());
            uint32_t code       = ec.category() == lely::canopen::SdoCategory()
                                      ? static_cast<uint32_t>(ec.value())
                                      : 0x08000000u;
            n->has_abort        = true;
            n->abort            = Problem();
            n->abort.kind       = ProblemKind::SdoAbort;
            n->abort.index      = idx;
            n->abort.subindex   = subidx;
            n->abort.abort_code = code;
            status_.sdo_abort(id, idx, subidx, code);
            std::lock_guard<lely::util::BasicLockable> lock(*this);
            ConfigResult(id, ec);
            return;
        }
        log_info("%s: defaults restored (0x1011:%u); resetting the node", n->cfg->label().c_str(),
                 sub);
        n->restored        = true;
        n->restore_pending = true;
        {
            // Ends this boot without a failure; the node boots again after its reset.
            std::lock_guard<lely::util::BasicLockable> lock(*this);
            ConfigResult(id, std::make_error_code(std::errc::operation_canceled));
        }
        try
        {
            // Communication objects take effect at a communication reset, the rest at a reset
            // of the node (CiA 301).
            Command(sub == 2 ? NmtCommand::RESET_COMM : NmtCommand::RESET_NODE, id);
        }
        catch (const std::exception &e)
        {
            log_warn("%s: cannot reset the node after restoring its defaults: %s",
                     n->cfg->label().c_str(), e.what());
        }
    };
    std::error_code ec;
    try
    {
        SubmitWrite(exec_, id, 0x1011, sub, uint32_t{kRestoreSignature}, con,
                    std::chrono::milliseconds(cfg_.sdo_timeout_ms), ec);
    }
    catch (const std::exception &e)
    {
        log_error("%s: cannot restore the defaults: %s", n->cfg->label().c_str(), e.what());
        ec = std::make_error_code(std::errc::io_error);
    }
    if (ec)
    {
        std::lock_guard<lely::util::BasicLockable> lock(*this);
        ConfigResult(id, ec);
    }
}

void Network::HandleLoss(uint8_t id, ProblemKind kind)
{
    NodeState *n = Find(id);
    if (!n || !n->up || n->stopped || n->held)
        return;
    n->manual       = false;
    const char *why = kind == ProblemKind::Heartbeat  ? "heartbeat timeout"
                      : kind == ProblemKind::Guarding ? "node guarding timeout"
                                                      : "node reset";
    const bool stop = cfg_.on_node_error == OnNodeError::Stop;
    log_warn("%s lost (%s); %s its inputs and %s", n->cfg->label().c_str(), why,
             n->cfg->on_loss == OnLoss::Zero ? "zeroing" : "holding",
             stop ? "stopping it until the next PLC start" : "booting it again");
    n->last_problem = why;
    n->restored     = false;
    Problem p;
    p.kind = kind;
    status_.lost(id, p);
    SetUp(*n, false);
    if (!stop)
    {
        ScheduleRetry(*n);
        return;
    }
    n->held = true;
    status_.held(id);
    StopHeld(id);
}

void Network::StopHeld(uint8_t id)
{
    Defer(
        [this, id]
        {
            try
            {
                Command(NmtCommand::STOP, id);
            }
            catch (const std::exception &e)
            {
                log_warn("node %u: cannot send NMT stop: %s", id, e.what());
            }
        });
}

void Network::HandleState(uint8_t id, NmtState st)
{
    NodeState *n = Find(id);
    if (!n)
        return;
    n->heard = true;
    switch (st)
    {
    case NmtState::PREOP:
        status_.set_nmt(id, NmtSeen::PreOperational);
        break;
    case NmtState::START:
        status_.set_nmt(id, NmtSeen::Operational);
        break;
    case NmtState::STOP:
        status_.set_nmt(id, NmtSeen::Stopped);
        break;
    default:
        status_.set_nmt(id, NmtSeen::Unknown);
        break;
    }
    if (n->held && st == NmtState::BOOTUP)
    {
        // A held node that restarted (power cycle, new session) is stopped again.
        StopHeld(id);
        return;
    }
    if (!n->up)
    {
        // A boot-up message starts Lely's boot of the node.
        if (st == NmtState::BOOTUP && !n->stopped && !n->held)
        {
            n->booting = true;
            status_.set_state(id, NodeRun::Booting);
        }
        return;
    }
    if (st == NmtState::BOOTUP)
    {
        HandleLoss(id, ProblemKind::Reset);
        return;
    }
    SetState(*n, static_cast<uint8_t>(st) & 0x7F);
    image_.commit();
}

void Network::HandleRpdo(int num, const uint8_t *data, std::size_t n)
{
    if (num < 1 || static_cast<size_t>(num) > layout_.rpdos.size())
        return;
    const MasterPdo &m = layout_.rpdos[static_cast<size_t>(num - 1)];
    // Only from a node that booted with this configuration.
    NodeState *node = Find(static_cast<uint8_t>(m.node_id));
    if (!node || !node->up)
        return;
    uint16_t &error = rpdo_error_[static_cast<size_t>(num - 1)];
    if (error == 0x8250)
    {
        error = 0;
        log_info("%s: TPDO %u (COB-ID 0x%03X) is received again", node->cfg->label().c_str(),
                 m.pdo->number, m.pdo->cob_id);
        status_.pdo_back(m.node_id, m.pdo->number);
    }
    unsigned offset = 0;
    for (size_t k = 0; k < m.pdo->entries.size(); ++k)
    {
        unsigned bits = co_type_bits(m.pdo->entries[k].type);
        image_.set_input(m.first_entry + k, pdo_get_bits(data, n, offset, bits));
        offset += bits;
    }
    image_.commit();
}

void Network::HandleRpdoError(int num, uint16_t eec)
{
    if (num < 1 || static_cast<size_t>(num) > layout_.rpdos.size())
        return;
    const MasterPdo &m = layout_.rpdos[static_cast<size_t>(num - 1)];
    uint16_t &last     = rpdo_error_[static_cast<size_t>(num - 1)];
    NodeState *node    = Find(static_cast<uint8_t>(m.node_id));
    // A deadline missed while the node is not running says nothing about the PDO.
    if (eec == 0x8250 && (!node || !node->up || node->manual))
        return;
    if (last == eec)
        return;
    last = eec;
    if (eec == 0x8250)
    {
        status_.pdo_late(m.node_id, m.pdo->number, m.pdo->cob_id);
        if (node->cfg->on_loss == OnLoss::Zero)
        {
            ZeroPdoInputs(m);
            image_.commit();
        }
    }
    else
    {
        status_.pdo_length(m.node_id, m.pdo->number, m.pdo->cob_id, eec);
    }
    const char *what = eec == 0x8210   ? "shorter than its mapping; it is ignored"
                       : eec == 0x8220 ? "longer than its mapping"
                                       : "not received in time";
    log_warn("%s: TPDO %u (COB-ID 0x%03X) is %s", node ? node->cfg->label().c_str() : "node",
             m.pdo->number, m.pdo->cob_id, what);
}

void Network::HandleEmcy(uint8_t id, uint16_t eec, uint8_t er, const uint8_t msef[5])
{
    NodeState *n = Find(id);
    if (!n)
        return;
    status_.emcy(id, eec, er, msef);
    auto now = clock::now();
    if (eec == 0)
    {
        log_info("%s: emergency cleared", n->cfg->label().c_str());
        return;
    }
    if (now - n->last_emcy < std::chrono::seconds(1))
        return;
    n->last_emcy = now;
    log_warn("%s: emergency 0x%04X, error register 0x%02X", n->cfg->label().c_str(), eec, er);
}

void Network::SetUp(NodeState &node, bool up)
{
    node.up = up;
    if (up)
    {
        SetState(node, static_cast<uint8_t>(NmtState::START));
        // Send the node's outputs in full.
        for (const MasterPdo *m : node.out_pdos)
            resend_[m->master_number - 1] = true;
    }
    else
    {
        SetState(node, 0);
        if (node.cfg->on_loss == OnLoss::Zero)
            ZeroInputs(node);
        // A PDO late before the loss is judged again once the node is back.
        for (const MasterPdo *m : node.in_pdos)
            if (rpdo_error_[m->master_number - 1] == 0x8250)
                rpdo_error_[m->master_number - 1] = 0;
    }
    image_.commit();
}

void Network::SetState(NodeState &node, uint8_t state)
{
    node.state = state;
    image_.set_node_state(node.slot, state);
}

void Network::ScheduleRetry(NodeState &node)
{
    node.retry_pending = true;
    node.retry_at      = clock::now() + node.backoff;
    node.backoff       = std::min(node.backoff * 2, kRetryMax);
}

void Network::ZeroInputs(const NodeState &node)
{
    for (const MasterPdo *m : node.in_pdos)
        ZeroPdoInputs(*m);
}

void Network::ZeroPdoInputs(const MasterPdo &m)
{
    for (size_t k = 0; k < m.pdo->entries.size(); ++k)
        image_.set_input(m.first_entry + k, 0);
}

Problem Network::BootProblem(NodeState &node, char es, const std::string &what)
{
    Problem p;
    p.boot_status = es;
    // CiA 302-2 boot error status.
    switch (es)
    {
    case 'B':
        p.kind = ProblemKind::NoResponse;
        return p;
    case 'C':
    case 'D':
    case 'M':
    case 'N':
    case 'O':
    {
        p.kind  = ProblemKind::Identity;
        p.field = es == 'C'   ? IdentityField::DeviceType
                  : es == 'D' ? IdentityField::VendorId
                  : es == 'M' ? IdentityField::ProductCode
                  : es == 'N' ? IdentityField::Revision
                              : IdentityField::SerialNumber;
        // The values the master checks (0x1F84-0x1F87) come from the configuration.
        const NodeConfig &c = *node.cfg;
        p.expected          = es == 'C'   ? c.device_type
                              : es == 'D' ? c.vendor_id
                              : es == 'M' ? c.product_code
                              : es == 'N' ? c.revision
                                          : 0;
        return p;
    }
    case 'J':
        if (node.has_abort)
        {
            Problem a     = node.abort;
            a.boot_status = es;
            return a;
        }
        break;
    default:
        break;
    }
    p.kind = ProblemKind::BootError;
    p.text = what;
    return p;
}

// Derived from canworks ec7fd59d, plugin/src/canopen/network.cpp (Network::ReportIdentity): the
// boot reports only that a value differs, so the value found is read once afterwards.
void Network::ReadFoundIdentity(uint8_t id, IdentityField field)
{
    NodeState *n = Find(id);
    if (!n || n->stopped)
        return;
    uint16_t index = field == IdentityField::DeviceType ? 0x1000 : 0x1018;
    uint8_t sub =
        static_cast<uint8_t>(field == IdentityField::DeviceType ? 0 : static_cast<unsigned>(field));
    try
    {
        SubmitRead<uint32_t>(
            exec_, id, index, sub,
            [this, field](uint8_t id, uint16_t, uint8_t, std::error_code ec, uint32_t value)
            {
                if (!ec)
                    status_.identity_found(id, field, value);
            });
    }
    catch (const std::exception &e)
    {
        log_debug("node %u: cannot read the identity found: %s", id, e.what());
    }
}

void Network::OnTick()
{
    tick_.submit_wait(tick_wait_);
    ServeClient();
    if (!running_)
        return;
    auto now = clock::now();
    for (auto &it : nodes_)
    {
        NodeState &n = it.second;
        if (!n.heard && !n.silent_logged &&
            now - started_ >= std::chrono::milliseconds(cfg_.boot_timeout_ms))
        {
            // Lely waits for a node that never answers without reporting it.
            n.silent_logged = true;
            status_.silent(it.first);
            if (n.cfg->optional)
                log_info("%s (optional) is absent; it is started when it answers",
                         n.cfg->label().c_str());
            else
                log_warn("%s has not answered since the master started; check that it is "
                         "powered, connected, set to node ID %u and to the bus bit rate",
                         n.cfg->label().c_str(), it.first);
        }
        if (n.up || n.stopped || n.held || n.booting || n.manual)
            continue;
        if (!n.retry_pending)
        {
            if (!n.heard && !n.polling && now >= n.next_poll)
                Poll(n, static_cast<uint8_t>(it.first));
            continue;
        }
        if (now < n.retry_at)
            continue;
        if (Boot(static_cast<uint8_t>(it.first)))
        {
            n.retry_pending = false;
            n.booting       = true;
            status_.set_state(it.first, NodeRun::Booting);
        }
        else
            n.retry_at = now + kRetryMin; // a boot is already running
    }
    if (cfg_.time_producer && now >= next_time_)
    {
        SendTime();
        next_time_ += std::chrono::milliseconds(cfg_.time_period_ms);
        if (next_time_ < now)
            next_time_ = now + std::chrono::milliseconds(cfg_.time_period_ms);
    }
    // A fallback only: each scan wakes the exchange itself.
    Exchange();
}

// Looks for a node that has not answered: a boot-up message missed while the master started (the
// node was already running, or connected later) leaves Lely waiting for nothing. Once asked, Lely
// keeps trying to boot an optional node about every second without reporting each attempt. A node
// that Lely starts without checking it (not initialized, no heartbeat to wait for) is first asked
// for 0x1000, so that an absent node is not taken as up.
void Network::Poll(NodeState &node, uint8_t id)
{
    if (node.cfg->configure || node.cfg->supervision == Supervision::Heartbeat)
    {
        if (Boot(id))
            node.booting = true;
        else
            node.next_poll = clock::now() + kPollEvery;
        return;
    }
    node.polling = true;
    try
    {
        SubmitRead<uint32_t>(
            exec_, id, 0x1000, 0,
            [this](uint8_t id, uint16_t, uint8_t, std::error_code ec, uint32_t)
            {
                NodeState *n = Find(id);
                if (!n)
                    return;
                n->polling   = false;
                n->next_poll = clock::now() + kPollEvery;
                if (ec || !running_ || n->up || n->booting || n->held || n->retry_pending)
                    return;
                if (Boot(id))
                    n->booting = true;
            },
            std::chrono::milliseconds(std::min<unsigned>(cfg_.sdo_timeout_ms, 500)));
    }
    catch (const std::exception &e)
    {
        node.polling   = false;
        node.next_poll = clock::now() + kPollEvery;
        log_debug("node %u: cannot look for it: %s", id, e.what());
    }
}

// TIME (CiA 301 TIME_OF_DAY): milliseconds after midnight and days since 1 January 1984, UTC.
void Network::SendTime()
{
    using namespace std::chrono;
    const int64_t ms  = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    const int64_t day = 86400000;
    const int64_t days = ms / day - 5113; // 1970-01-01 to 1984-01-01
    if (days < 0)
        return;
    const uint32_t tod = static_cast<uint32_t>(ms % day);
    can_msg msg        = CAN_MSG_INIT;
    msg.id             = cfg_.time_cob_id;
    msg.len            = 6;
    for (int i = 0; i < 4; ++i)
        msg.data[i] = static_cast<uint8_t>(tod >> (8 * i));
    msg.data[4] = static_cast<uint8_t>(days);
    msg.data[5] = static_cast<uint8_t>(days >> 8);
    std::lock_guard<lely::util::BasicLockable> lock(*this);
    can_net_send(net(), &msg);
}

void Network::Exchange()
{
    if (!running_)
        return;
    if (cfg_.sync == SyncMode::PlcCycle)
    {
        uint64_t scans = image_.scans();
        if (scans == synced_scans_)
            return;
        merged_scans_ += scans - synced_scans_ - 1;
        synced_scans_ = scans;
        WriteOutputs(false);
        SendSync();
        return;
    }
    WriteOutputs(false);
}

void Network::WriteOutputs(bool all)
{
    bool fresh          = false;
    const uint64_t *out = image_.latest_outputs(&fresh);
    if (!all && !fresh && std::find(resend_.begin(), resend_.end(), true) == resend_.end())
        return;
    for (const MasterPdo &m : layout_.tpdos)
    {
        size_t num            = m.master_number - 1;
        const NodeState &node = nodes_[m.node_id];
        if (!node.up)
            continue;
        bool send = all || resend_[num];
        for (size_t k = 0; k < m.pdo->entries.size(); ++k)
        {
            size_t i = m.first_entry + k;
            if (out_valid_[i] && last_out_[i] == out[i] && !resend_[num] && !all)
                continue;
            WriteObject(static_cast<uint16_t>(kOutputObjectBase + i), m.pdo->entries[k].type,
                        out[i]);
            last_out_[i]  = out[i];
            out_valid_[i] = true;
            send          = true;
        }
        resend_[num] = false;
        if (send)
            TpdoEvent(static_cast<int>(m.master_number));
    }
}

void Network::WriteObject(uint16_t index, CoType type, uint64_t v)
{
    std::error_code ec;
    switch (type)
    {
    case CoType::BOOLEAN:
        Write<bool>(index, 0, v != 0, ec);
        break;
    case CoType::INTEGER8:
        Write<int8_t>(index, 0, static_cast<int8_t>(v), ec);
        break;
    case CoType::INTEGER16:
        Write<int16_t>(index, 0, static_cast<int16_t>(v), ec);
        break;
    case CoType::INTEGER32:
        Write<int32_t>(index, 0, static_cast<int32_t>(v), ec);
        break;
    case CoType::INTEGER64:
        Write<int64_t>(index, 0, static_cast<int64_t>(v), ec);
        break;
    case CoType::UNSIGNED8:
        Write<uint8_t>(index, 0, static_cast<uint8_t>(v), ec);
        break;
    case CoType::UNSIGNED16:
        Write<uint16_t>(index, 0, static_cast<uint16_t>(v), ec);
        break;
    case CoType::UNSIGNED32:
        Write<uint32_t>(index, 0, static_cast<uint32_t>(v), ec);
        break;
    case CoType::UNSIGNED64:
        Write<uint64_t>(index, 0, v, ec);
        break;
    case CoType::REAL32:
    {
        uint32_t raw = static_cast<uint32_t>(v);
        float f;
        std::memcpy(&f, &raw, sizeof(f));
        Write<float>(index, 0, f, ec);
        break;
    }
    case CoType::REAL64:
    {
        double d;
        std::memcpy(&d, &v, sizeof(d));
        Write<double>(index, 0, d, ec);
        break;
    }
    }
}

void Network::SendSync()
{
    can_msg msg = CAN_MSG_INIT;
    msg.id      = 0x80;
    std::lock_guard<lely::util::BasicLockable> lock(*this);
    can_net_send(net(), &msg);
}

void Network::ServeClient()
{
    if (!client_ || client_busy_)
        return;
    ClientJob job;
    if (!client_->next(job))
        return;
    switch (job.kind)
    {
    case ClientJob::Kind::Read:
        ClientRead(job);
        break;
    case ClientJob::Kind::Write:
        ClientWrite(job);
        break;
    case ClientJob::Kind::Nmt:
        ClientNmt(job);
        break;
    }
}

lely::canopen::Sdo &Network::ServiceSdo(uint8_t id)
{
    std::unique_ptr<lely::canopen::Sdo> &sdo = service_sdos_[id];
    if (!sdo)
        sdo.reset(new lely::canopen::Sdo(net(), id));
    return *sdo;
}

void Network::FinishSdo(uint64_t job, std::error_code ec, std::vector<uint8_t> data)
{
    client_busy_ = false;
    JobResult r;
    if (!ec)
    {
        r.state = JobResult::State::Done;
        r.data  = std::move(data);
    }
    else
    {
        r.state = JobResult::State::Failed;
        if (ec.category() != lely::canopen::SdoCategory())
            r.reason = "error";
        else if (static_cast<uint32_t>(ec.value()) ==
                 static_cast<uint32_t>(lely::canopen::SdoErrc::NO_SDO))
            r.reason = "booting"; // Lely boots the node with its own client SDO
        else
        {
            r.has_abort  = true;
            r.abort_code = static_cast<uint32_t>(ec.value());
        }
    }
    client_->finish(job, std::move(r));
}

void Network::ClientRead(const ClientJob &job)
{
    const uint64_t jid = job.id;
    const auto timeout = std::chrono::milliseconds(cfg_.sdo_timeout_ms);
    auto con = [this, jid](uint8_t, uint16_t, uint8_t, std::error_code ec, std::vector<uint8_t> v)
    { FinishSdo(jid, ec, std::move(v)); };
    client_busy_ = true;
    std::error_code ec;
    try
    {
        if (service_)
        {
            std::lock_guard<lely::util::BasicLockable> lock(*this);
            ServiceSdo(static_cast<uint8_t>(job.node))
                .SubmitUpload<std::vector<uint8_t>>(exec_, job.index, job.subindex, con, false,
                                                    timeout);
        }
        else
            SubmitRead<std::vector<uint8_t>>(exec_, static_cast<uint8_t>(job.node), job.index,
                                             job.subindex, con, timeout, ec);
    }
    catch (const std::exception &e)
    {
        log_debug("node %u: client read failed: %s", job.node, e.what());
        ec = std::make_error_code(std::errc::io_error);
    }
    if (ec)
        FinishSdo(jid, ec, {});
}

void Network::ClientWrite(ClientJob &job)
{
    const uint64_t jid   = job.id;
    const unsigned node  = job.node;
    const uint16_t index = job.index;
    const uint8_t sub    = job.subindex;
    const auto timeout   = std::chrono::milliseconds(cfg_.sdo_timeout_ms);
    auto con = [this, jid, node, index, sub](uint8_t, uint16_t, uint8_t, std::error_code ec)
    {
        bool abort    = ec && ec.category() == lely::canopen::SdoCategory();
        uint32_t code = abort ? static_cast<uint32_t>(ec.value()) : 0;
        if (ec)
            log_warn("node %u: online write to 0x%04X:%u failed: %s", node, index, sub,
                     ec.message().c_str());
        else
            log_info("node %u: online write to 0x%04X:%u", node, index, sub);
        status_.client_write(node, index, sub, !ec, abort, code);
        FinishSdo(jid, ec, {});
    };
    client_busy_ = true;
    std::error_code ec;
    try
    {
        if (service_)
        {
            std::lock_guard<lely::util::BasicLockable> lock(*this);
            ServiceSdo(static_cast<uint8_t>(node))
                .SubmitDownload(exec_, index, sub, std::move(job.data), con, false, timeout);
        }
        else
            SubmitWrite(exec_, static_cast<uint8_t>(node), index, sub, std::move(job.data), con,
                        timeout, ec);
    }
    catch (const std::exception &e)
    {
        log_debug("node %u: client write failed: %s", node, e.what());
        ec = std::make_error_code(std::errc::io_error);
    }
    if (ec)
    {
        con(0, 0, 0, ec);
    }
}

void Network::SendNmt(NmtRequest cmd, uint8_t id)
{
    static const uint8_t cs[] = {1, 2, 0x80, 0x81, 0x82};
    can_msg msg               = CAN_MSG_INIT;
    msg.id                    = 0;
    msg.len                   = 2;
    msg.data[0]               = cs[static_cast<size_t>(cmd)];
    msg.data[1]               = id;
    std::lock_guard<lely::util::BasicLockable> lock(*this);
    can_net_send(net(), &msg);
}

void Network::ManualNmt(uint8_t id, NmtRequest cmd)
{
    if (service_)
    {
        SendNmt(cmd, id);
        return;
    }
    NodeState *n = Find(id);
    if (!n)
        return;
    static const NmtCommand lely_cmd[] = {NmtCommand::START, NmtCommand::STOP,
                                          NmtCommand::ENTER_PREOP, NmtCommand::RESET_NODE,
                                          NmtCommand::RESET_COMM};
    try
    {
        Command(lely_cmd[static_cast<size_t>(cmd)], id);
    }
    catch (const std::exception &e)
    {
        log_warn("%s: cannot send the NMT command: %s", n->cfg->label().c_str(), e.what());
        return;
    }
    // `wait`: the node's boot-up message starts the boot; the retry is only a fallback.
    auto boot_again = [this, n, id](std::chrono::milliseconds wait)
    {
        if (cfg_.on_node_error == OnNodeError::Stop)
        {
            // Back in the network list, which the boot needs; taken out again once it is up.
            std::error_code ec;
            Write<uint32_t>(0x1F81, id, slave_assignment(*n->cfg), ec);
        }
        n->manual        = false;
        n->retry_pending = true;
        n->retry_at      = clock::now() + wait;
        n->backoff       = kRetryMin;
    };
    switch (cmd)
    {
    case NmtRequest::Stop:
    case NmtRequest::PreOperational:
        n->manual = true;
        status_.manual(id);
        if (n->up && n->cfg->on_loss == OnLoss::Zero)
        {
            ZeroInputs(*n);
            image_.commit();
        }
        break;
    case NmtRequest::Start:
        if (n->held)
        {
            n->held = false;
            status_.release(id);
            boot_again(std::chrono::milliseconds(0));
        }
        else if (n->up)
        {
            n->manual = false;
            status_.set_state(id, NodeRun::Operational);
        }
        else
            boot_again(std::chrono::milliseconds(0));
        break;
    case NmtRequest::ResetNode:
    case NmtRequest::ResetCommunication:
        if (n->held)
        {
            n->held = false;
            status_.release(id);
        }
        // The boot-up that follows is expected: no loss, the master boots the node again.
        if (n->up)
            SetUp(*n, false);
        status_.set_state(id, NodeRun::Booting);
        boot_again(kRetryMin);
        break;
    }
}

void Network::ClientNmt(const ClientJob &job)
{
    if (job.node == 0)
    {
        if (service_)
        {
            for (const auto &it : nodes_)
                SendNmt(job.command, static_cast<uint8_t>(it.first));
        }
        else
            for (const auto &it : nodes_)
                ManualNmt(static_cast<uint8_t>(it.first), job.command);
    }
    else
        ManualNmt(static_cast<uint8_t>(job.node), job.command);
    log_info("%s: NMT %s to %s from a client", cfg_.label().c_str(), nmt_request_name(job.command),
             job.node ? ("node " + std::to_string(job.node)).c_str() : "every node");
    status_.nmt_command(job.node, job.command);
    JobResult r;
    r.state = JobResult::State::Done;
    client_->finish(job.id, std::move(r));
}

} // namespace canopen_plugin
