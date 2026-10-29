// SPDX-License-Identifier: Apache-2.0
//
// The master end to end on Lely's in-process virtual CAN bus (or on the SocketCAN interface named
// by CANOPEN_TEST_INTERFACE, such as a vcan link), against simulated generic I/O modules: boot with
// identity check and configuration SDOs (PDO remapping as the editor writes it), inputs into %I,
// outputs from %Q, node state, loss and recovery, a node with the wrong identity, a configured node
// that never answers, and the NMT command on stop, sent only to the configured nodes.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/sys/clock.hpp>
#include <lely/io2/vcan.hpp>

#include "bus.h"
#include "check.h"
#include "config.h"
#include "log.h"
#include "master_dcf.h"
#include "process_image.h"
#include "sim_slave.h"

using namespace canopen_plugin;
using canopen_sim::SimSlave;
using canopen_sim::SimSlaveSpec;

namespace
{

const char *kConfig = R"({
  "version": 1,
  "networks": [{
    "name": "test", "interface": "vcan-test", "bitrate": 0,
    "sync": {"mode": "plc_cycle"},
    "sdo_timeout_ms": 500,
    "nodes": [
      {"node_id": 2, "name": "io1",
       "identity": {"device_type": 401, "vendor_id": 1, "product_code": 2, "revision": 3},
       "supervision": {"type": "heartbeat", "consumer_ms": 300},
       "on_loss": "hold",
       "state_location": "%IB10",
       "sdo": [{"index": 8448, "subindex": 0, "data": "78563412"},
               {"index": 6144, "subindex": 1, "data": "82010080"},
               {"index": 6656, "subindex": 0, "data": "00"},
               {"index": 6656, "subindex": 1, "data": "08010060"},
               {"index": 6656, "subindex": 2, "data": "10010164"},
               {"index": 6656, "subindex": 0, "data": "02"},
               {"index": 6144, "subindex": 1, "data": "82010000"},
               {"index": 5120, "subindex": 1, "data": "02020080"},
               {"index": 5632, "subindex": 0, "data": "00"},
               {"index": 5632, "subindex": 1, "data": "08010062"},
               {"index": 5632, "subindex": 0, "data": "01"},
               {"index": 5120, "subindex": 1, "data": "02020000"}],
       "tpdos": [{"number": 1, "cob_id": 386, "entries": [
         {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB0"},
         {"index": 25601, "subindex": 1, "type": "INTEGER16", "location": "%IW0"}]}],
       "rpdos": [{"number": 1, "cob_id": 514, "entries": [
         {"index": 25088, "subindex": 1, "type": "UNSIGNED8", "location": "%QB0"}]}]},
      {"node_id": 3, "name": "wrong",
       "identity": {"vendor_id": 153},
       "state_location": "%IB11",
       "tpdos": [{"number": 1, "cob_id": 387, "entries": [
         {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB1"}]}]},
      {"node_id": 9, "name": "absent", "state_location": "%IB12"}
    ]
  }]
})";

constexpr int kSize = 32;
IEC_BYTE g_byte_in[kSize];
IEC_UINT g_int_in[kSize];
IEC_BYTE g_byte_out[kSize];
IEC_BYTE *g_byte_out_ptr[kSize];
IEC_BOOL *g_bool_out_ptr[kSize][8];
IEC_UINT *g_int_out_ptr[kSize];
IEC_UDINT *g_dint_out_ptr[kSize];
IEC_ULINT *g_lint_out_ptr[kSize];

int write_bool(int, int, int, int)
{
    return 0;
}
int write_byte(int, int index, int value)
{
    g_byte_in[index] = static_cast<IEC_BYTE>(value);
    return 0;
}
int write_int(int, int index, int value)
{
    g_int_in[index] = static_cast<IEC_UINT>(value);
    return 0;
}
int write_dint(int, int, unsigned)
{
    return 0;
}
int write_lint(int, int, unsigned long long)
{
    return 0;
}
void lock() {}
void unlock() {}

plugin_runtime_args_t fake_args()
{
    plugin_runtime_args_t rt;
    std::memset(&rt, 0, sizeof(rt));
    for (int i = 0; i < kSize; ++i)
        g_byte_out_ptr[i] = &g_byte_out[i];
    rt.bool_output        = g_bool_out_ptr;
    rt.byte_output        = g_byte_out_ptr;
    rt.int_output         = g_int_out_ptr;
    rt.dint_output        = g_dint_out_ptr;
    rt.lint_output        = g_lint_out_ptr;
    rt.image_lock         = lock;
    rt.image_unlock       = unlock;
    rt.journal_write_bool = write_bool;
    rt.journal_write_byte = write_byte;
    rt.journal_write_int  = write_int;
    rt.journal_write_dint = write_dint;
    rt.journal_write_lint = write_lint;
    rt.buffer_size        = kSize;
    return rt;
}

// Runs PLC scans every 2 ms until `done` or the timeout; true if `done`.
bool scan_until(ProcessImage &image, const plugin_runtime_args_t &rt, std::function<bool()> done,
                std::chrono::milliseconds timeout)
{
    auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end)
    {
        image.copy_to_plc(rt);
        if (done())
            return true;
        image.copy_from_plc(rt);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

std::mutex g_log_mutex;
std::vector<std::string> g_warnings;

void quiet_sink(LogLevel level, const char *msg)
{
    if (level == LogLevel::Debug)
        return;
    std::printf("%s\n", msg);
    if (level == LogLevel::Warn)
    {
        std::lock_guard<std::mutex> lock(g_log_mutex);
        g_warnings.push_back(msg);
    }
}

size_t warnings_with(const std::string &text)
{
    std::lock_guard<std::mutex> lock(g_log_mutex);
    size_t n = 0;
    for (const auto &w : g_warnings)
        n += w.find(text) != std::string::npos;
    return n;
}

struct Frame
{
    uint32_t id;
    uint8_t first;
    std::chrono::steady_clock::time_point at;
};

// Records the frames on the virtual bus while `run` is set.
void listen(std::shared_ptr<lely::io::VirtualCanController> vbus, std::atomic<bool> &run,
            std::vector<Frame> &frames)
{
    lely::io::Context ctx;
    lely::ev::Loop loop;
    lely::io::VirtualCanChannel chan(ctx, loop.get_executor());
    chan.open(*vbus);
    while (run)
    {
        can_msg msg = CAN_MSG_INIT;
        std::error_code ec;
        if (chan.read(&msg, nullptr, nullptr, 50, ec) == 1)
            frames.push_back(
                {msg.id, msg.len ? msg.data[0] : uint8_t{0}, std::chrono::steady_clock::now()});
    }
}

// One SYNC per scan, right after the scan's outputs, which leave without waiting for a tick.
void check_scan_exchange(std::shared_ptr<lely::io::VirtualCanController> vbus, ProcessImage &image,
                         const plugin_runtime_args_t &rt)
{
    constexpr int kScans = 100;
    std::atomic<bool> run{true};
    std::vector<Frame> frames;
    std::thread listener([&] { listen(vbus, run, frames); });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::vector<std::chrono::steady_clock::time_point> ends;
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < kScans; ++i)
    {
        next += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(next);
        image.copy_to_plc(rt);
        g_byte_out[0] = static_cast<IEC_BYTE>(0x80 + i);
        image.copy_from_plc(rt);
        ends.push_back(std::chrono::steady_clock::now());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    run = false;
    listener.join();

    int syncs   = 0;
    int ordered = 0;
    std::vector<double> latency_ms;
    int rpdo_value = -1;
    for (const Frame &f : frames)
    {
        if (f.at < ends.front())
            continue;
        if (f.id == 0x202)
        {
            rpdo_value = f.first;
            int scan   = f.first - 0x80;
            if (scan >= 0 && scan < kScans)
                latency_ms.push_back(
                    std::chrono::duration<double, std::milli>(f.at - ends[scan]).count());
        }
        else if (f.id == 0x80)
        {
            // The outputs of the scan this SYNC belongs to went first.
            if (rpdo_value == 0x80 + syncs)
                ++ordered;
            ++syncs;
        }
    }
    std::printf("scan exchange: %d SYNC, %d after their scan's outputs, %zu output frames\n", syncs,
                ordered, latency_ms.size());
    CHECK(syncs == kScans);
    CHECK(ordered == kScans);
    CHECK(latency_ms.size() == kScans);
    if (!latency_ms.empty())
    {
        std::sort(latency_ms.begin(), latency_ms.end());
        double median = latency_ms[latency_ms.size() / 2];
        std::printf("scan exchange: median output latency %.3f ms\n", median);
        // The housekeeping tick is 10 ms: a woken exchange is far below it.
        CHECK(median < 3.0);
    }
    g_byte_out[0] = 0x5A;
}

const char *kWatchConfig = R"({
  "version": 1,
  "networks": [{
    "name": "watch", "interface": "vcan-test", "bitrate": 0,
    "heartbeat_producer_ms": 50,
    "on_plc_stop": "none",
    "nodes": [
      {"node_id": 2, "name": "io1",
       "identity": {"device_type": 401, "vendor_id": 1, "product_code": 2, "revision": 3},
       "supervision": {"type": "heartbeat", "consumer_ms": 300},
       "state_location": "%IB10",
       "sdo": [{"index": 4118, "subindex": 1, "data": "C8007F00"}]}
    ]
  }]
})";

// A node watching the master's heartbeat goes pre-operational once the master is gone, even when
// the master sends no NMT command on stop.
void check_master_watch(std::shared_ptr<lely::io::VirtualCanController> vbus,
                        const std::string &wire, const std::string &dir,
                        const plugin_runtime_args_t &rt)
{
    std::string text = kWatchConfig;
    if (!wire.empty())
        text.replace(text.find("vcan-test"), 9, wire);
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(text, cfg, errors));
    if (!errors.empty())
        return;
    const NetworkConfig &net = cfg.networks[0];
    MasterLayout layout      = master_layout(net);
    ProcessImage image;
    CHECK(image.build(net, layout, kSize, errors));
    std::string dcf = dir + "/watch.dcf";
    {
        FILE *f       = std::fopen(dcf.c_str(), "w");
        std::string t = master_dcf(net, layout);
        std::fwrite(t.data(), 1, t.size(), f);
        std::fclose(f);
    }
    SimSlaveSpec spec;
    spec.node_id = 2;
    SimSlave io(spec, dir + "/watch_node2.dcf", vbus, wire);
    io.start();
    Bus bus(net, layout, image, dcf, vbus);
    bus.start();
    CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 5; }, std::chrono::seconds(10)));
    CHECK(io.last_command() == 1);
    bus.stop();
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (io.last_command() != 0x80 && std::chrono::steady_clock::now() < end)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    CHECK(io.last_command() == 0x80);
    io.stop();
}

} // namespace

int main()
{
    set_log_sink(quiet_sink);
    route_lely_diagnostics();
    prepare_wake_signal();
    const std::string dir = "/tmp/canopen_test_" + std::to_string(getpid());
    CHECK(system(("mkdir -p " + dir).c_str()) == 0);

    const char *iface = std::getenv("CANOPEN_TEST_INTERFACE");
    std::string text  = kConfig;
    if (iface && *iface)
        text.replace(text.find("vcan-test"), 9, iface);
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(text, cfg, errors));
    for (const auto &e : errors)
        std::fprintf(stderr, "%s\n", e.c_str());
    if (!errors.empty())
        return TEST_RESULT();
    const NetworkConfig &net = cfg.networks[0];
    MasterLayout layout      = master_layout(net);
    ProcessImage image;
    CHECK(image.build(net, layout, kSize, errors));
    std::string dcf = dir + "/master.dcf";
    {
        FILE *f          = std::fopen(dcf.c_str(), "w");
        std::string text = master_dcf(net, layout);
        std::fwrite(text.data(), 1, text.size(), f);
        std::fclose(f);
    }
    plugin_runtime_args_t rt = fake_args();

    std::shared_ptr<lely::io::VirtualCanController> vbus;
    std::string wire;
    if (iface && *iface)
        wire = iface;
    else
        vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    SimSlaveSpec spec2;
    spec2.node_id = 2;
    SimSlaveSpec spec3;
    spec3.node_id = 3;
    std::unique_ptr<SimSlave> io1(new SimSlave(spec2, dir + "/node2.dcf", vbus, wire));
    SimSlave wrong(spec3, dir + "/node3.dcf", vbus, wire);
    // On the bus but not in the configuration.
    SimSlaveSpec spec4;
    spec4.node_id = 4;
    SimSlave foreign(spec4, dir + "/node4.dcf", vbus, wire);
    io1->start();
    wrong.start();
    foreign.start();
    io1->set_inputs(0xA5, -1234);
    wrong.set_inputs(0x77, 0);

    Bus bus(net, layout, image, dcf, vbus);
    bus.start();

    // Boot: identity checked, parameter written, node started, state 5.
    CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 5; }, std::chrono::seconds(10)));
    CHECK(io1->parameter() == 0x12345678);
    CHECK(io1->last_command() == 1); // NMT start

    // Inputs.
    CHECK(scan_until(
        image, rt,
        [] { return g_byte_in[0] == 0xA5 && g_int_in[0] == static_cast<IEC_UINT>(-1234); },
        std::chrono::seconds(2)));

    // Outputs.
    g_byte_out[0] = 0x3C;
    auto *node    = io1.get();
    CHECK(
        scan_until(image, rt, [node] { return node->outputs() == 0x3C; }, std::chrono::seconds(2)));
    g_byte_out[0] = 0x5A;
    CHECK(
        scan_until(image, rt, [node] { return node->outputs() == 0x5A; }, std::chrono::seconds(2)));

    // A session runs at the real-time priority, or says once that it cannot.
    CHECK(bus.realtime() || warnings_with("cannot use real-time priority") == 1);

    if (vbus)
        check_scan_exchange(vbus, image, rt);

    // The node with the wrong identity never comes up; the other one is not held back.
    CHECK(g_byte_in[11] == 0);
    CHECK(g_byte_in[1] == 0);

    // Loss: heartbeat stops, state 0, inputs held.
    io1.reset();
    CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 0; }, std::chrono::seconds(3)));
    CHECK(g_byte_in[0] == 0xA5);

    // Recovery: the node is back, booted and configured again.
    io1.reset(new SimSlave(spec2, dir + "/node2.dcf", vbus, wire));
    io1->start();
    io1->set_inputs(0x0F, 100);
    CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 5; }, std::chrono::seconds(20)));
    CHECK(io1->parameter() == 0x12345678);
    CHECK(scan_until(
        image, rt, [] { return g_byte_in[0] == 0x0F && g_int_in[0] == 100; },
        std::chrono::seconds(2)));
    // The outputs are sent in full when the node comes up, though they did not change.
    node = io1.get();
    CHECK(
        scan_until(image, rt, [node] { return node->outputs() == 0x5A; }, std::chrono::seconds(2)));

    // The configured node that is not on the bus is reported once; the others are not.
    CHECK(scan_until(
        image, rt, [] { return warnings_with("node 9 (absent) has not answered") > 0; },
        std::chrono::seconds(10)));
    CHECK(warnings_with("has not answered") == 1);

    // PLC stop: the configured nodes go to PRE-OPERATIONAL, a node the master does not manage is
    // left alone.
    bus.stop();
    CHECK(io1->last_command() == 0x80);
    CHECK(wrong.last_command() == 0x80);
    CHECK(foreign.last_command() != 0x80);

    io1.reset();
    wrong.stop();
    foreign.stop();

    check_master_watch(vbus, wire, dir, rt);
    CHECK(system(("rm -rf " + dir).c_str()) == 0);
    return TEST_RESULT();
}
