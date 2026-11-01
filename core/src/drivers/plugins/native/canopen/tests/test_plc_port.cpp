// SPDX-License-Identifier: Apache-2.0
//
// The function blocks' side of a network without a bus: requests queued, taken in order,
// finished, polled and cancelled; the queue limit; a session end failing what is pending; handles
// of another slot generation or plugin load; node and bus states; the emergency ring with its
// per-reader position, node filter and loss count; and no allocation on the scan's side.

#include <cstdlib>
#include <cstring>
#include <new>
#include <thread>
#include <vector>

#include "check.h"
#include "plc_port.h"

thread_local long t_allocs = 0;

void *operator new(std::size_t n)
{
    ++t_allocs;
    if (void *p = std::malloc(n ? n : 1))
        return p;
    throw std::bad_alloc();
}
void operator delete(void *p) noexcept
{
    std::free(p);
}
void operator delete(void *p, std::size_t) noexcept
{
    std::free(p);
}

using namespace canopen_plugin;

namespace
{

const unsigned kNodes[] = {2, 5};

canopen_plc_request read_req(uint8_t node, uint16_t index, uint8_t sub)
{
    canopen_plc_request r{};
    r.network  = 1;
    r.node     = node;
    r.kind     = CANOPEN_PLC_READ;
    r.index    = index;
    r.subindex = sub;
    return r;
}

void check_round_trip()
{
    PlcPort port(0x1234, 0, 127, kNodes, 2);
    canopen_plc_result res{};
    // Not running: refused with the bus state.
    CHECK(port.start(read_req(2, 0x1018, 1), res) == 0);
    CHECK(res.error == CANOPEN_PLC_BUS_OFF && res.reason == CANOPEN_PLC_REASON_LINK);
    port.set_kernel(CANOPEN_PLC_NO_DRIVER);
    CHECK(port.start(read_req(2, 0x1018, 1), res) == 0 && res.error == CANOPEN_PLC_NO_DRIVER);
    port.set_kernel(CANOPEN_PLC_NO_ERROR);

    port.set_running(true);
    long before           = t_allocs;
    uint64_t h1           = port.start(read_req(2, 0x1018, 1), res);
    canopen_plc_request w = read_req(5, 0x2100, 0);
    w.kind                = CANOPEN_PLC_WRITE;
    const uint8_t bytes[] = {0x78, 0x56, 0x34, 0x12};
    w.data                = bytes;
    w.length              = 4;
    uint64_t h2           = port.start(w, res);
    CHECK(h1 != 0 && h2 != 0 && h1 != h2);
    CHECK((h1 >> 32) == 0x1234 && PlcPort::network_of(h1) == 0);
    uint8_t data[8] = {};
    CHECK(port.poll(h1, res, data, sizeof data) == 0);
    CHECK(t_allocs == before);

    // Taken in the order queued.
    PlcJob job;
    CHECK(port.take(job) && job.kind == CANOPEN_PLC_READ && job.node == 2 && job.index == 0x1018);
    const uint8_t vendor[] = {1, 0, 0, 0};
    port.finish(job.token, CANOPEN_PLC_NO_ERROR, 0, 0, vendor, 4);
    PlcJob job2;
    CHECK(port.take(job2) && job2.kind == CANOPEN_PLC_WRITE && job2.length == 4 &&
          std::memcmp(job2.data, bytes, 4) == 0);
    CHECK(!port.take(job));
    port.finish(job2.token, CANOPEN_PLC_OTHER_ERROR, 0, 0x06090030, nullptr, 0);

    before = t_allocs;
    CHECK(port.poll(h1, res, data, sizeof data) == 1);
    CHECK(res.length == 4 && data[0] == 1);
    CHECK(port.poll(h2, res, nullptr, 0) == 2);
    CHECK(res.error == CANOPEN_PLC_OTHER_ERROR && res.abort_code == 0x06090030);
    CHECK(t_allocs == before);
    // A finished request is forgotten; polling it again is a stale handle.
    CHECK(port.poll(h1, res, data, sizeof data) == 2 && res.reason == CANOPEN_PLC_REASON_RESTART);
}

void check_inputs()
{
    PlcPort port(1, 0, 127, kNodes, 2);
    port.set_running(true);
    canopen_plc_result res{};
    CHECK(port.start(read_req(3, 0x1000, 0), res) == 0 && res.error == CANOPEN_PLC_NODE_ID_UNKNOWN);
    canopen_plc_request w = read_req(2, 0x2100, 0);
    w.kind                = CANOPEN_PLC_WRITE;
    CHECK(port.start(w, res) == 0 && res.error == CANOPEN_PLC_WRONG_PARAMETER &&
          res.reason == CANOPEN_PLC_REASON_LENGTH);
    uint8_t big[CANOPEN_PLC_MAX_DATA + 1] = {};
    w.data                                = big;
    w.length                              = sizeof big;
    CHECK(port.start(w, res) == 0 && res.error == CANOPEN_PLC_WRONG_PARAMETER);
    canopen_plc_request nmt{};
    nmt.network = 1;
    nmt.kind    = CANOPEN_PLC_NMT;
    nmt.node    = 0;
    nmt.command = 3;
    CHECK(port.start(nmt, res) == 0 && res.reason == CANOPEN_PLC_REASON_COMMAND);
    nmt.command = 0x81;
    CHECK(port.start(nmt, res) != 0);
}

void check_queue_and_cancel()
{
    PlcPort port(7, 0, 127, kNodes, 2);
    port.set_running(true);
    canopen_plc_result res{};
    std::vector<uint64_t> handles;
    for (size_t i = 0; i < PlcPort::kSlots; ++i)
        handles.push_back(port.start(read_req(2, 0x1000, 0), res));
    CHECK(handles.back() != 0);
    CHECK(port.start(read_req(2, 0x1000, 0), res) == 0 && res.error == CANOPEN_PLC_REQUEST_ERROR &&
          res.reason == CANOPEN_PLC_REASON_FULL);

    // A queued request cancelled is gone; its slot is free again.
    port.cancel(handles[0]);
    CHECK(port.start(read_req(2, 0x1001, 0), res) != 0);
    // A running request cancelled: its end is discarded and the slot freed.
    PlcJob job;
    CHECK(port.take(job) && job.index == 0x1000);
    port.cancel(handles[1]);
    port.finish(job.token, CANOPEN_PLC_NO_ERROR, 0, 0, nullptr, 0);
    CHECK(port.poll(handles[1], res, nullptr, 0) == 2 && res.reason == CANOPEN_PLC_REASON_RESTART);
    CHECK(port.start(read_req(2, 0x1002, 0), res) != 0);

    // The session ends: every pending request fails, then new ones are refused.
    CHECK(port.take(job));
    port.set_running(false);
    CHECK(port.poll(handles[2], res, nullptr, 0) == 2 && res.error == CANOPEN_PLC_BUS_OFF &&
          res.reason == CANOPEN_PLC_REASON_LINK);
    // The transfer that was running ends later and changes nothing.
    port.finish(job.token, CANOPEN_PLC_NO_ERROR, 0, 0, nullptr, 0);
    CHECK(port.poll(handles[3], res, nullptr, 0) == 2 && res.error == CANOPEN_PLC_BUS_OFF);
    CHECK(port.start(read_req(2, 0x1000, 0), res) == 0);
}

void check_stale_handles()
{
    PlcPort old_load(100, 0, 127, kNodes, 2);
    old_load.set_running(true);
    canopen_plc_result res{};
    uint64_t h = old_load.start(read_req(2, 0x1000, 0), res);
    PlcPort new_load(200, 0, 127, kNodes, 2);
    new_load.set_running(true);
    uint64_t h2 = new_load.start(read_req(2, 0x1000, 0), res);
    // Same slot and generation, another load: never the new request's result.
    CHECK((h & 0xFFFFFFFFu) == (h2 & 0xFFFFFFFFu));
    CHECK(new_load.poll(h, res, nullptr, 0) == 2 && res.error == CANOPEN_PLC_NO_DRIVER &&
          res.reason == CANOPEN_PLC_REASON_RESTART);
    new_load.cancel(h); // ignored
    CHECK(new_load.poll(h2, res, nullptr, 0) == 0);
    // Another network's handle.
    PlcPort other(200, 1, 127, kNodes, 2);
    CHECK(other.poll(h2, res, nullptr, 0) == 2);
}

void check_states()
{
    PlcPort port(1, 0, 42, kNodes, 2);
    CHECK(port.master_node_id() == 42);
    CHECK(port.device_state(2) == CANOPEN_PLC_NOT_AVAIL);
    CHECK(port.device_state(3) == CANOPEN_PLC_UNKNOWN);
    port.set_nmt(2, 0x05);
    CHECK(port.device_state(2) == CANOPEN_PLC_OPERATIONAL);
    port.set_nmt(2, 0x04);
    CHECK(port.device_state(2) == CANOPEN_PLC_STOPPED);
    port.set_nmt(2, 0x7F);
    CHECK(port.device_state(2) == CANOPEN_PLC_PRE_OPERATIONAL);
    port.set_running(true);
    port.set_running(false);
    CHECK(port.device_state(2) == CANOPEN_PLC_NOT_AVAIL);
    port.set_kernel(CANOPEN_PLC_ERROR_PASSIVE);
    CHECK(port.kernel_state() == CANOPEN_PLC_ERROR_PASSIVE);
}

void check_emcy()
{
    PlcPort port(1, 0, 127, kNodes, 2);
    const uint8_t msef[5] = {1, 2, 3, 4, 5};
    port.push_emcy(2, 0x1000, 1, msef); // before the reader started: not seen
    uint64_t a = port.emcy_cursor();
    uint64_t b = a;
    port.push_emcy(2, 0x8130, 0x11, msef);
    port.push_emcy(5, 0x2310, 0x03, msef);
    port.push_emcy(2, 0x0000, 0x00, msef);
    canopen_plc_emcy e{};
    long before = t_allocs;
    CHECK(port.emcy_next(0, a, e) == 1 && e.node == 2 && e.error_code == 0x8130 &&
          e.error_register == 0x11 && e.data[4] == 5 && e.lost == 0);
    CHECK(port.emcy_next(0, a, e) == 1 && e.node == 5 && e.error_code == 0x2310);
    CHECK(port.emcy_next(0, a, e) == 1 && e.node == 2 && e.error_code == 0);
    CHECK(port.emcy_next(0, a, e) == 0);
    // A second reader of node 2 only sees everything again.
    CHECK(port.emcy_next(2, b, e) == 1 && e.error_code == 0x8130);
    CHECK(port.emcy_next(2, b, e) == 1 && e.error_code == 0);
    CHECK(port.emcy_next(2, b, e) == 0);
    CHECK(t_allocs == before);

    // A reader behind by more than the ring holds continues with the oldest and counts the loss.
    uint64_t c = port.emcy_cursor();
    for (unsigned i = 0; i < PlcPort::kEmcy + 5; ++i)
        port.push_emcy(5, static_cast<uint16_t>(0x5000 + i), 0, msef);
    CHECK(port.emcy_next(0, c, e) == 1 && e.error_code == 0x5005 && e.lost == 5);
    CHECK(port.emcy_next(0, c, e) == 1 && e.lost == 0);

    // A writer racing a reader: every message is read whole or counted as lost.
    PlcPort race(1, 0, 127, kNodes, 2);
    uint64_t r = race.emcy_cursor();
    std::thread writer(
        [&]
        {
            for (unsigned i = 1; i <= 20000; ++i)
            {
                uint8_t d[5] = {static_cast<uint8_t>(i), static_cast<uint8_t>(i),
                                static_cast<uint8_t>(i), static_cast<uint8_t>(i),
                                static_cast<uint8_t>(i)};
                race.push_emcy(2, static_cast<uint16_t>(i), static_cast<uint8_t>(i), d);
            }
        });
    unsigned seen = 0, lost = 0, torn = 0;
    while (seen + lost < 20000)
    {
        int got = race.emcy_next(0, r, e);
        if (got != 1)
            continue;
        ++seen;
        lost += e.lost;
        uint8_t v = static_cast<uint8_t>(e.error_code);
        torn += e.error_register != v || e.data[0] != v || e.data[4] != v;
    }
    writer.join();
    CHECK(torn == 0);
    CHECK(seen + lost == 20000);
}

} // namespace

int main()
{
    check_round_trip();
    check_inputs();
    check_queue_and_cancel();
    check_stale_handles();
    check_states();
    check_emcy();
    CHECK(new_nonce() != 0 && new_nonce() != new_nonce());
    return TEST_RESULT();
}
