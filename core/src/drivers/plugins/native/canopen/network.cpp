// SPDX-License-Identifier: Apache-2.0

#include "network.h"

#include <algorithm>
#include <cstring>
#include <mutex>

#include <lely/can/msg.h>
#include <lely/ev/exec.hpp>

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

Network::Network(ev_exec_t *exec, lely::io::TimerBase &timer, lely::io::TimerBase &tick,
                 lely::io::CanChannelBase &chan, const std::string &dcf_path,
                 const NetworkConfig &cfg, const MasterLayout &layout, ProcessImage &image)
    : BasicMaster(exec, timer, chan, dcf_path, "", static_cast<uint8_t>(cfg.master_node_id)),
      exec_(exec), tick_(tick), tick_wait_(exec,
                                           [this](int, std::error_code ec)
                                           {
                                               if (!ec)
                                                   OnTick();
                                           }),
      cfg_(cfg), layout_(layout), image_(image)
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
}

Network::~Network()
{
    tick_.cancel_wait(tick_wait_);
}

void Network::Start()
{
    running_      = true;
    synced_scans_ = image_.scans();
    log_info("%s: starting, %zu node(s), %zu input and %zu output PDO(s)", cfg_.label().c_str(),
             nodes_.size(), layout_.rpdos.size(), layout_.tpdos.size());
    Reset();
    tick_.settime(kTick, kTick);
    tick_.submit_wait(tick_wait_);
}

void Network::StopNodes()
{
    running_ = false;
    if (cfg_.sync == SyncMode::Period)
    {
        // Stop producing SYNC.
        std::error_code ec;
        Write<uint32_t>(0x1005, 0, 0x80, ec);
    }
    for (auto &it : nodes_)
        it.second.stopped = true;
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
        HandleLoss(id, "heartbeat timeout");
}

void Network::OnState(uint8_t id, NmtState st) noexcept
{
    HandleState(id, st);
}

void Network::OnRpdo(int num, std::error_code ec, const void *p, std::size_t n) noexcept
{
    if (!ec)
        HandleRpdo(num, static_cast<const uint8_t *>(p), n);
}

void Network::OnEmcy(uint8_t id, uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept
{
    (void)msef;
    Defer([this, id, eec, er] { HandleEmcy(id, eec, er); });
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
    if (!n || n->stopped)
        return;
    if (es == 0 || es == 'L')
    {
        if (!n->last_problem.empty())
            log_info("%s is up again", n->cfg->label().c_str());
        else
            log_info("%s is up", n->cfg->label().c_str());
        n->last_problem.clear();
        n->backoff       = kRetryMin;
        n->retry_pending = false;
        SetUp(*n, true);
        return;
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

void Network::HandleLoss(uint8_t id, const char *why)
{
    NodeState *n = Find(id);
    if (!n || !n->up || n->stopped)
        return;
    log_warn("%s lost (%s); %s its inputs and booting it again", n->cfg->label().c_str(), why,
             n->cfg->on_loss == OnLoss::Zero ? "zeroing" : "holding");
    n->last_problem = why;
    SetUp(*n, false);
    ScheduleRetry(*n);
}

void Network::HandleState(uint8_t id, NmtState st)
{
    NodeState *n = Find(id);
    if (!n || !n->up)
        return;
    if (st == NmtState::BOOTUP)
    {
        HandleLoss(id, "node reset");
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
    if (last == eec)
        return;
    last             = eec;
    const char *what = eec == 0x8210   ? "shorter than its mapping; it is ignored"
                       : eec == 0x8220 ? "longer than its mapping"
                                       : "not received in time";
    NodeState *n     = Find(static_cast<uint8_t>(m.node_id));
    log_warn("%s: TPDO %u (COB-ID 0x%03X) is %s", n ? n->cfg->label().c_str() : "node",
             m.pdo->number, m.pdo->cob_id, what);
}

void Network::HandleEmcy(uint8_t id, uint16_t eec, uint8_t er)
{
    NodeState *n = Find(id);
    if (!n)
        return;
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
        for (size_t k = 0; k < m->pdo->entries.size(); ++k)
            image_.set_input(m->first_entry + k, 0);
}

void Network::OnTick()
{
    tick_.submit_wait(tick_wait_);
    if (!running_)
        return;
    auto now = clock::now();
    for (auto &it : nodes_)
    {
        NodeState &n = it.second;
        if (!n.retry_pending || n.up || n.stopped || now < n.retry_at)
            continue;
        if (Boot(static_cast<uint8_t>(it.first)))
            n.retry_pending = false;
        else
            n.retry_at = now + kRetryMin; // a boot is already running
    }
    if (cfg_.sync == SyncMode::PlcCycle)
    {
        uint64_t scans = image_.scans();
        if (scans == synced_scans_)
            return;
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

} // namespace canopen_plugin
