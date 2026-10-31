// SPDX-License-Identifier: Apache-2.0
//
// plc_port.cpp - see plc_port.h. A slot moves Free -> Filling (scan) -> Queued -> Running (bus
// thread) -> Ended -> Free (scan, after its poll). A cancel frees a queued slot at once and marks a
// running one Abandoned, which the bus thread frees when the transfer ends. Each claim bumps the
// slot's generation, which is part of the handle, so a stale handle never reads a newer request.

#include "plc_port.h"

#include <chrono>
#include <cstring>

namespace canopen_plugin
{

namespace
{
constexpr uint32_t kTokenSlotMask = 0xFFFFu;

void fail(canopen_plc_result &res, uint8_t error, uint8_t reason)
{
    res        = canopen_plc_result{};
    res.error  = error;
    res.reason = reason;
}

bool known_command(uint8_t cs)
{
    return cs == 1 || cs == 2 || cs == 0x80 || cs == 0x81 || cs == 0x82;
}
} // namespace

uint8_t device_state_of(uint8_t nmt)
{
    switch (nmt)
    {
    case 0:
        return CANOPEN_PLC_NOT_AVAIL;
    case 0x04:
        return CANOPEN_PLC_STOPPED;
    case 0x05:
        return CANOPEN_PLC_OPERATIONAL;
    case 0x7F:
        return CANOPEN_PLC_PRE_OPERATIONAL;
    default:
        return CANOPEN_PLC_UNKNOWN;
    }
}

uint32_t new_nonce()
{
    // The plugin may stay mapped across a reload (dlclose does not always unload it), so the
    // nonce is new for each port, not for each load: a counter mixed with the clock and an address.
    static std::atomic<uint32_t> counter{0};
    static int anchor;
    uint64_t t = static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    uint64_t a = reinterpret_cast<uintptr_t>(&anchor);
    uint32_t n = static_cast<uint32_t>(t ^ (t >> 32) ^ a ^ (a >> 32)) +
                 counter.fetch_add(0x9E3779B9u, std::memory_order_relaxed);
    return n ? n : 1u;
}

PlcPort::PlcPort(uint32_t nonce, uint8_t net, uint8_t master_node_id, const unsigned *nodes,
                 size_t count)
    : nonce_(nonce), net_(net), master_(master_node_id)
{
    for (auto &s : nmt_)
        s.store(0, std::memory_order_relaxed);
    for (size_t i = 0; i < count; ++i)
        if (nodes[i] >= 1 && nodes[i] <= 127)
            known_[nodes[i]] = true;
}

bool PlcPort::known(unsigned node) const
{
    return node >= 1 && node <= 127 && known_[node];
}

uint8_t PlcPort::device_state(unsigned node) const
{
    if (!known(node))
        return CANOPEN_PLC_UNKNOWN;
    return device_state_of(nmt_[node].load(std::memory_order_acquire));
}

uint8_t PlcPort::kernel_state() const
{
    return kernel_.load(std::memory_order_acquire);
}

uint64_t PlcPort::start(const canopen_plc_request &req, canopen_plc_result &res)
{
    res = canopen_plc_result{};
    if (req.kind == CANOPEN_PLC_NMT)
    {
        if (req.node != 0 && !known(req.node))
            return fail(res, CANOPEN_PLC_NODE_ID_UNKNOWN, CANOPEN_PLC_REASON_NONE), 0;
        if (!known_command(req.command))
            return fail(res, CANOPEN_PLC_WRONG_PARAMETER, CANOPEN_PLC_REASON_COMMAND), 0;
    }
    else if (req.kind == CANOPEN_PLC_READ || req.kind == CANOPEN_PLC_WRITE)
    {
        if (!known(req.node))
            return fail(res, CANOPEN_PLC_NODE_ID_UNKNOWN, CANOPEN_PLC_REASON_NONE), 0;
        if (req.kind == CANOPEN_PLC_WRITE &&
            (req.length == 0 || req.length > CANOPEN_PLC_MAX_DATA || !req.data))
            return fail(res, CANOPEN_PLC_WRONG_PARAMETER, CANOPEN_PLC_REASON_LENGTH), 0;
    }
    else
        return fail(res, CANOPEN_PLC_WRONG_PARAMETER, CANOPEN_PLC_REASON_NONE), 0;

    if (!running_.load(std::memory_order_acquire))
    {
        uint8_t k = kernel_state();
        return fail(res, k == CANOPEN_PLC_NO_ERROR ? static_cast<uint8_t>(CANOPEN_PLC_BUS_OFF) : k,
                    CANOPEN_PLC_REASON_LINK),
               0;
    }

    for (size_t i = 0; i < kSlots; ++i)
    {
        Slot &s           = slots_[i];
        uint32_t expected = Free;
        if (!s.state.compare_exchange_strong(expected, Filling, std::memory_order_acquire))
            continue;
        uint16_t gen = static_cast<uint16_t>(s.gen.load(std::memory_order_relaxed) + 1);
        if (gen == 0)
            gen = 1;
        s.gen.store(gen, std::memory_order_relaxed);
        s.req = req;
        if (req.kind == CANOPEN_PLC_WRITE)
        {
            std::memcpy(s.data, req.data, req.length);
            s.req.data = s.data;
        }
        else
            s.req.data = nullptr;
        s.res = canopen_plc_result{};
        s.seq = next_seq_.fetch_add(1, std::memory_order_relaxed);
        s.state.store(Queued, std::memory_order_release);
        return make_handle(nonce_, gen, net_, static_cast<uint8_t>(i));
    }
    return fail(res, CANOPEN_PLC_REQUEST_ERROR, CANOPEN_PLC_REASON_FULL), 0;
}

PlcPort::Slot *PlcPort::slot_of(uint64_t handle)
{
    if (static_cast<uint32_t>(handle >> 32) != nonce_)
        return nullptr;
    uint8_t slot = static_cast<uint8_t>(handle & 0xFFu);
    uint16_t gen = static_cast<uint16_t>((handle >> 16) & 0xFFFFu);
    if (network_of(handle) != net_ || slot >= kSlots)
        return nullptr;
    Slot &s = slots_[slot];
    if (s.gen.load(std::memory_order_relaxed) != gen)
        return nullptr;
    return &s;
}

int PlcPort::poll(uint64_t handle, canopen_plc_result &res, uint8_t *data, uint32_t cap)
{
    Slot *s = slot_of(handle);
    if (!s)
        return fail(res, CANOPEN_PLC_NO_DRIVER, CANOPEN_PLC_REASON_RESTART), 2;
    uint32_t st = s->state.load(std::memory_order_acquire);
    if (st == Queued || st == Running || st == Filling)
        return 0;
    if (st != Ended)
        return fail(res, CANOPEN_PLC_NO_DRIVER, CANOPEN_PLC_REASON_RESTART), 2;
    res = s->res;
    if (res.error == CANOPEN_PLC_NO_ERROR && data && cap)
    {
        uint32_t n = res.length < cap ? res.length : cap;
        if (n > CANOPEN_PLC_MAX_DATA)
            n = CANOPEN_PLC_MAX_DATA;
        std::memcpy(data, s->data, n);
    }
    s->state.store(Free, std::memory_order_release);
    return res.error == CANOPEN_PLC_NO_ERROR ? 1 : 2;
}

void PlcPort::cancel(uint64_t handle)
{
    Slot *s = slot_of(handle);
    if (!s)
        return;
    uint32_t st = Queued;
    if (s->state.compare_exchange_strong(st, Free, std::memory_order_acq_rel))
        return;
    st = Running;
    if (s->state.compare_exchange_strong(st, Abandoned, std::memory_order_acq_rel))
        return;
    st = Ended;
    s->state.compare_exchange_strong(st, Free, std::memory_order_acq_rel);
}

void PlcPort::set_running(bool running)
{
    running_.store(running, std::memory_order_release);
    if (running)
        return;
    for (size_t i = 0; i < kSlots; ++i)
    {
        Slot &s     = slots_[i];
        uint32_t st = Queued;
        // A queued request is taken as the bus thread would take it, then ended like a running one.
        s.state.compare_exchange_strong(st, Running, std::memory_order_acq_rel);
        uint32_t token = (static_cast<uint32_t>(s.gen.load(std::memory_order_relaxed)) << 16) |
                         static_cast<uint32_t>(i);
        finish(token, CANOPEN_PLC_BUS_OFF, CANOPEN_PLC_REASON_LINK, 0, nullptr, 0);
    }
    for (auto &n : nmt_)
        n.store(0, std::memory_order_release);
}

bool PlcPort::take(PlcJob &job)
{
    Slot *best = nullptr;
    size_t at  = 0;
    for (size_t i = 0; i < kSlots; ++i)
    {
        Slot &s = slots_[i];
        if (s.state.load(std::memory_order_acquire) != Queued)
            continue;
        if (!best || s.seq < best->seq)
        {
            best = &s;
            at   = i;
        }
    }
    if (!best)
        return false;
    uint32_t st = Queued;
    if (!best->state.compare_exchange_strong(st, Running, std::memory_order_acq_rel))
        return take(job); // cancelled meanwhile
    const canopen_plc_request &r = best->req;
    job.token = (static_cast<uint32_t>(best->gen.load(std::memory_order_relaxed)) << 16) |
                static_cast<uint32_t>(at);
    job.kind       = r.kind;
    job.node       = r.node;
    job.command    = r.command;
    job.index      = r.index;
    job.subindex   = r.subindex;
    job.timeout_ms = r.timeout_ms;
    job.length     = r.length;
    job.data       = r.kind == CANOPEN_PLC_WRITE ? best->data : nullptr;
    return true;
}

void PlcPort::finish(uint32_t token, uint8_t error, uint8_t reason, uint32_t abort_code,
                     const uint8_t *data, uint32_t length)
{
    size_t at = token & kTokenSlotMask;
    if (at >= kSlots)
        return;
    Slot &s = slots_[at];
    if (s.gen.load(std::memory_order_relaxed) != static_cast<uint16_t>(token >> 16))
        return;
    uint32_t st = s.state.load(std::memory_order_acquire);
    if (st == Abandoned)
    {
        s.state.compare_exchange_strong(st, Free, std::memory_order_acq_rel);
        return;
    }
    if (st != Running)
        return; // ended by set_running(false)
    s.res            = canopen_plc_result{};
    s.res.error      = error;
    s.res.reason     = reason;
    s.res.abort_code = abort_code;
    s.res.length     = length;
    if (error == CANOPEN_PLC_NO_ERROR && data && length)
        std::memcpy(s.data, data, length < CANOPEN_PLC_MAX_DATA ? length : CANOPEN_PLC_MAX_DATA);
    if (!s.state.compare_exchange_strong(st, Ended, std::memory_order_acq_rel) && st == Abandoned)
        s.state.store(Free, std::memory_order_release);
}

void PlcPort::set_nmt(unsigned node, uint8_t state)
{
    if (node >= 1 && node <= 127)
        nmt_[node].store(state, std::memory_order_release);
}

void PlcPort::set_kernel(uint8_t error)
{
    kernel_.store(error, std::memory_order_release);
}

void PlcPort::push_emcy(uint8_t node, uint16_t code, uint8_t reg, const uint8_t data[5])
{
    uint64_t pos = emcy_head_.load(std::memory_order_relaxed);
    EmcyEntry &e = emcy_[pos % kEmcy];
    e.seq.store(2 * pos + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    uint64_t head = static_cast<uint64_t>(node) | (static_cast<uint64_t>(reg) << 8) |
                    (static_cast<uint64_t>(code) << 16);
    for (unsigned b = 0; b < 4; ++b)
        head |= static_cast<uint64_t>(data[b]) << (32 + 8 * b);
    e.head.store(head, std::memory_order_relaxed);
    e.tail.store(data[4], std::memory_order_relaxed);
    e.seq.store(2 * pos + 2, std::memory_order_release);
    emcy_head_.store(pos + 1, std::memory_order_release);
}

uint64_t PlcPort::emcy_cursor() const
{
    return emcy_head_.load(std::memory_order_acquire);
}

int PlcPort::emcy_next(uint8_t node, uint64_t &cursor, canopen_plc_emcy &out) const
{
    uint32_t lost = 0;
    while (true)
    {
        uint64_t head = emcy_head_.load(std::memory_order_acquire);
        if (cursor > head)
            cursor = head; // a cursor of an earlier plugin load
        if (head - cursor > kEmcy)
        {
            lost += static_cast<uint32_t>(head - kEmcy - cursor);
            cursor = head - kEmcy;
        }
        if (cursor == head)
            return 0;
        const EmcyEntry &e = emcy_[cursor % kEmcy];
        uint64_t s1        = e.seq.load(std::memory_order_acquire);
        uint64_t v         = e.head.load(std::memory_order_relaxed);
        uint8_t t          = e.tail.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        uint64_t s2 = e.seq.load(std::memory_order_relaxed);
        if (s1 != 2 * cursor + 2 || s2 != s1)
        {
            // Overwritten while read: the writer is a lap ahead.
            ++lost;
            ++cursor;
            continue;
        }
        ++cursor;
        uint8_t from = static_cast<uint8_t>(v & 0xFF);
        if (node != 0 && from != node)
            continue;
        out                = canopen_plc_emcy{};
        out.node           = from;
        out.error_register = static_cast<uint8_t>((v >> 8) & 0xFF);
        out.error_code     = static_cast<uint16_t>((v >> 16) & 0xFFFF);
        for (unsigned b = 0; b < 4; ++b)
            out.data[b] = static_cast<uint8_t>((v >> (32 + 8 * b)) & 0xFF);
        out.data[4] = t;
        out.lost    = lost;
        return 1;
    }
}

} // namespace canopen_plugin
