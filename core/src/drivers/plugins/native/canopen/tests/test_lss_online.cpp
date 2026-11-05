// SPDX-License-Identifier: Apache-2.0
//
// The LSS find sequence against modelled devices on an ideal bus, with a simulated clock: devices
// without a node ID are found and one with a node ID is not; a device that answers a fastscan out
// of turn (in the configuration state) ends the search as incomplete, with the devices found so
// far and without a made-up address.

#include <cstdint>
#include <vector>

#include "check.h"
#include "lss_online.h"

using namespace canopen_plugin;

namespace
{

using Frame = LssOnline::Frame;

// One LSS slave as CiA 305 describes it; `out_of_turn` also answers every fastscan while in the
// configuration state.
struct Device
{
    uint32_t addr[4];
    unsigned node_id = 255;
    bool out_of_turn = false;
    bool config      = false;
    uint8_t lss_sub  = 0;
    uint32_t sel[4]  = {0, 0, 0, 0};

    bool answer(const Frame &f, Frame &a)
    {
        a                = Frame{};
        const uint32_t v = f[1] | f[2] << 8 | f[3] << 16 | static_cast<uint32_t>(f[4]) << 24;
        switch (f[0])
        {
        case 0x04:
            config = f[1] == 1;
            return false;
        case 0x40:
        case 0x41:
        case 0x42:
        case 0x43:
            sel[f[0] - 0x40] = v;
            if (f[0] == 0x43 && sel[0] == addr[0] && sel[1] == addr[1] && sel[2] == addr[2] &&
                sel[3] == addr[3])
            {
                config = true;
                a[0]   = 0x44;
                return true;
            }
            return false;
        case 0x5E:
            if (!config)
                return false;
            a[0] = 0x5E;
            a[1] = static_cast<uint8_t>(node_id);
            return true;
        case 0x51:
        {
            a[0] = 0x4F;
            if (config)
                return out_of_turn;
            if (node_id != 255)
                return false;
            const uint8_t bit = f[5], sub = f[6], next = f[7];
            if (bit == 0x80)
            {
                lss_sub = 0;
                return true;
            }
            if (sub != lss_sub || ((uint64_t{v} ^ addr[sub]) >> bit) != 0)
                return false;
            if (bit == 0)
            {
                lss_sub = next;
                if (next < sub)
                    config = true;
            }
            return true;
        }
        default:
            return false;
        }
    }
};

struct Result
{
    LssOnline::Outcome out;
    std::vector<LssAddress4> found;
};

Result find(std::vector<Device> &devices)
{
    std::vector<Frame> answers;
    LssOnline lss(
        [&](const Frame &f)
        {
            for (Device &d : devices)
            {
                Frame a;
                if (d.answer(f, a))
                    answers.push_back(a);
            }
        });
    Result r;
    bool done = false;
    auto now  = LssOnline::clock::time_point{};
    LssOnline::Request req;
    lss.start(
        req, now, [&](const LssAddress4 &a) { r.found.push_back(a); },
        [&](const LssOnline::Outcome &o)
        {
            r.out = o;
            done  = true;
        });
    for (int ms = 0; ms < 600000 && !done; ++ms)
    {
        now += std::chrono::milliseconds(1);
        std::vector<Frame> got;
        got.swap(answers);
        for (const Frame &a : got)
            lss.receive(a.data(), a.size(), now);
        lss.tick(now);
    }
    CHECK(done);
    for (const Device &d : devices)
        CHECK(!d.config);
    return r;
}

Device device(uint32_t serial, unsigned node_id = 255, bool out_of_turn = false)
{
    Device d{{1, 2, 3, serial}};
    d.node_id     = node_id;
    d.out_of_turn = out_of_turn;
    return d;
}

bool has(const std::vector<LssAddress4> &list, uint32_t serial)
{
    for (const LssAddress4 &a : list)
        if (a.vendor_id == 1 && a.product_code == 2 && a.revision == 3 && a.serial_number == serial)
            return true;
    return false;
}

} // namespace

int main()
{
    // Two devices without a node ID are found; one with a node ID takes no part.
    {
        std::vector<Device> bus = {device(0x5678), device(0x5679), device(0x1001, 2)};
        Result r                = find(bus);
        CHECK(r.out.ok && r.out.reason.empty());
        CHECK(r.out.devices.size() == 2 && has(r.out.devices, 0x5678) &&
              has(r.out.devices, 0x5679));
        CHECK(r.found.size() == 2);
    }

    // The device answering out of turn is found first: the search ends as incomplete with it,
    // and the other device stays to be found by the next search.
    {
        std::vector<Device> bus = {device(0x5678, 255, true), device(0x5679)};
        Result r                = find(bus);
        CHECK(!r.out.ok && r.out.reason == "incomplete");
        CHECK(r.out.devices.size() == 1 && has(r.out.devices, 0x5678));
        for (const LssAddress4 &a : r.found)
            CHECK(!(a == LssAddress4()));
    }

    // Found after a conforming device: both are in the result.
    {
        std::vector<Device> bus = {device(0x5678), device(0x5679, 255, true)};
        Result r                = find(bus);
        CHECK(!r.out.ok && r.out.reason == "incomplete");
        CHECK(r.out.devices.size() == 2 && has(r.out.devices, 0x5678) &&
              has(r.out.devices, 0x5679));
    }

    // Alone on the bus: found, and the search still ends as incomplete.
    {
        std::vector<Device> bus = {device(0x5678, 255, true)};
        Result r                = find(bus);
        CHECK(r.out.reason == "incomplete" && r.out.devices.size() == 1);
    }
    return TEST_RESULT();
}
