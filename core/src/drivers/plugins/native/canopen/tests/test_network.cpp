// SPDX-License-Identifier: Apache-2.0
//
// The master end to end on Lely's in-process virtual CAN bus (or on the SocketCAN interface named
// by CANOPEN_TEST_INTERFACE, such as a vcan link), against simulated generic I/O modules: boot with
// identity check and configuration SDOs (PDO remapping as the editor writes it), inputs into %I,
// outputs from %Q, node state, loss and recovery, a node with the wrong identity, a configured node
// that never answers, a refused configuration SDO, emergencies, late PDOs with their inputs held
// and zeroed, the status record and its socket, the NMT command on stop, sent only to the
// configured nodes, and the reset of a bus-off link (on a stood-in link state). Then the node
// options: restore defaults before the configuration, a node that is not initialized, an optional
// node that is absent, node guarding, a node stopped after an error, and the TIME producer.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/sys/clock.hpp>
#include <lely/io2/vcan.hpp>

#include "bus.h"
#include "cJSON.h"
#include "check.h"
#include "config.h"
#include "fake_link.h"
#include "log.h"
#include "master_dcf.h"
#include "process_image.h"
#include "requests.h"
#include "sim_slave.h"
#include "status.h"
#include "status_server.h"

// Counts this thread's allocations, to show the scan hooks make none.
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
using canopen_sim::SimSlave;
using canopen_sim::SimSlaveSpec;
using canopen_test::FakeLink;

namespace
{

const char *kConfig = R"({
  "version": 3,
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
       "tpdos": [{"number": 1, "cob_id": 386, "receive_timeout_ms": 150, "entries": [
         {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB0"},
         {"index": 25601, "subindex": 1, "type": "INTEGER16", "location": "%IW0"}]}],
       "rpdos": [{"number": 1, "cob_id": 514, "entries": [
         {"index": 25088, "subindex": 1, "type": "UNSIGNED8", "location": "%QB0"}]}]},
      {"node_id": 3, "name": "wrong",
       "identity": {"vendor_id": 153},
       "state_location": "%IB11",
       "tpdos": [{"number": 1, "cob_id": 387, "entries": [
         {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB1"}]}]},
      {"node_id": 9, "name": "absent", "state_location": "%IB12"},
      {"node_id": 5, "name": "timed",
       "supervision": {"type": "heartbeat", "consumer_ms": 300},
       "on_loss": "zero",
       "tpdos": [{"number": 1, "cob_id": 389, "receive_timeout_ms": 150, "entries": [
         {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB2"}]}]},
      {"node_id": 6, "name": "refuses",
       "sdo": [{"index": 8448, "subindex": 0, "data": "01000000"}]}
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
std::vector<std::string> g_errors;

void quiet_sink(LogLevel level, const char *msg)
{
    if (level == LogLevel::Debug)
        return;
    std::printf("%s\n", msg);
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (level == LogLevel::Warn)
        g_warnings.push_back(msg);
    else if (level == LogLevel::Error)
        g_errors.push_back(msg);
}

size_t errors_with(const std::string &text)
{
    std::lock_guard<std::mutex> lock(g_log_mutex);
    size_t n = 0;
    for (const auto &w : g_errors)
        n += w.find(text) != std::string::npos;
    return n;
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
    ends.reserve(kScans);
    long scan_allocs = 0;
    auto next        = std::chrono::steady_clock::now();
    for (int i = 0; i < kScans; ++i)
    {
        next += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(next);
        long before = t_allocs;
        image.copy_to_plc(rt);
        g_byte_out[0] = static_cast<IEC_BYTE>(0x80 + i);
        image.copy_from_plc(rt);
        scan_allocs += t_allocs - before;
        ends.push_back(std::chrono::steady_clock::now());
    }
    CHECK(scan_allocs == 0);
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

NodeStatus node_status(const NetworkStatus &status, unsigned id)
{
    NetworkSnapshot s = status.snapshot();
    for (const NodeStatus &n : s.nodes)
        if (n.node_id == id)
            return n;
    return NodeStatus();
}

const char *kWatchConfig = R"({
  "version": 3,
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
    NetworkStatus status(net);
    Bus bus(net, layout, image, status, dcf, vbus);
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

const char *kOptionsConfig = R"({
  "version": 3,
  "networks": [{
    "name": "options", "interface": "vcan-test", "bitrate": 0,
    "on_node_error": "stop", "boot_timeout_ms": 1000, "sdo_timeout_ms": 500,
    "time": {"cob_id": 256, "period_ms": 100},
    "nodes": [
      {"node_id": 2, "name": "restored", "restore_defaults": 1,
       "supervision": {"type": "guarding", "guard_time_ms": 100, "life_time_factor": 3},
       "state_location": "%IB10",
       "sdo": [{"index": 8448, "subindex": 0, "data": "78563412"},
               {"index": 4119, "subindex": 0, "data": "0000"},
               {"index": 4108, "subindex": 0, "data": "6400"},
               {"index": 4109, "subindex": 0, "data": "03"}]},
      {"node_id": 3, "name": "as-is", "configure": false,
       "supervision": {"type": "heartbeat", "consumer_ms": 300},
       "state_location": "%IB11"},
      {"node_id": 4, "name": "maybe", "optional": true, "state_location": "%IB12"},
      {"node_id": 7, "name": "keeps", "restore_defaults": 3, "state_location": "%IB13"}
    ]
  }]
})";

// Restore defaults before the configuration, a node that is not initialized, an absent optional
// node, a refused restore, node guarding with the node stopped after the error, and TIME.
void check_node_options(std::shared_ptr<lely::io::VirtualCanController> vbus,
                        const std::string &wire, const std::string &dir,
                        const plugin_runtime_args_t &rt)
{
    std::string text = kOptionsConfig;
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
    std::string dcf = dir + "/options.dcf";
    {
        FILE *f       = std::fopen(dcf.c_str(), "w");
        std::string t = master_dcf(net, layout);
        std::fwrite(t.data(), 1, t.size(), f);
        std::fclose(f);
    }
    SimSlaveSpec spec2;
    spec2.node_id = 2;
    SimSlaveSpec spec3;
    spec3.node_id = 3;
    SimSlaveSpec spec7;
    spec7.node_id         = 7;
    spec7.restore_refused = true;
    std::unique_ptr<SimSlave> restored(new SimSlave(spec2, dir + "/opt_node2.dcf", vbus, wire));
    SimSlave as_is(spec3, dir + "/opt_node3.dcf", vbus, wire);
    SimSlave keeps(spec7, dir + "/opt_node7.dcf", vbus, wire);
    restored->start();
    as_is.start();
    keeps.start();
    std::memset(g_byte_in, 0, sizeof(g_byte_in));

    std::atomic<bool> run{true};
    std::vector<Frame> frames;
    std::thread listener;
    if (vbus)
        listener = std::thread([&] { listen(vbus, run, frames); });

    size_t warnings_before = warnings_with("node 4 (maybe)");
    NetworkStatus status(net);
    Bus bus(net, layout, image, status, dcf, vbus);
    bus.start();

    // Restored, then configured: the parameter written by the configuration survived the reset
    // that made the defaults take effect.
    CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 5; }, std::chrono::seconds(10)));
    CHECK(restored->restores() == 1);
    SimSlave *node2 = restored.get();
    CHECK(scan_until(
        image, rt, [node2] { return node2->parameter() == 0x12345678; }, std::chrono::seconds(1)));
    CHECK(restored->last_command() == 1);
    // Not initialized: started, nothing written.
    CHECK(scan_until(image, rt, [] { return g_byte_in[11] == 5; }, std::chrono::seconds(5)));
    CHECK(as_is.parameter() == 0 && as_is.restores() == 0 && as_is.last_command() == 1);
    // The refused restore fails the boot with the object and the abort code.
    CHECK(scan_until(
        image, rt,
        [&status]
        {
            Problem p = node_status(status, 7).problem;
            return node_status(status, 7).state == NodeRun::Failed &&
                   p.kind == ProblemKind::SdoAbort && p.index == 0x1011 && p.subindex == 3;
        },
        std::chrono::seconds(5)));
    CHECK(node_status(status, 7).problem.abort_code == 0x06010002);
    // The optional node is absent, without a warning.
    CHECK(scan_until(
        image, rt, [&status] { return node_status(status, 4).state == NodeRun::Absent; },
        std::chrono::seconds(3)));
    CHECK(warnings_with("node 4 (maybe)") == warnings_before);
    // Lely's error line for each failed boot attempt is logged at debug level only.
    CHECK(errors_with("lely: SDO abort code") == 0);
    {
        NetworkSnapshot snap = status.snapshot();
        CHECK(!snap.all_up(false) && !snap.all_up(true));
    }

    // Node guarding: the node goes away, is lost with the reason guarding, sent NMT stop and left
    // alone, also when it comes back.
    restored.reset();
    CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 0; }, std::chrono::seconds(2)));
    {
        NodeStatus n = node_status(status, 2);
        CHECK(n.state == NodeRun::Lost && n.problem.kind == ProblemKind::Guarding && n.held);
    }
    restored.reset(new SimSlave(spec2, dir + "/opt_node2.dcf", vbus, wire));
    restored->start();
    CHECK(!scan_until(image, rt, [] { return g_byte_in[10] != 0; }, std::chrono::seconds(3)));
    CHECK(restored->restores() == 0 && restored->parameter() == 0);
    CHECK(g_byte_in[11] == 5);

    // A new session in the same PLC run (as after a lost or reset link) keeps the node stopped.
    bus.stop();
    Bus again(net, layout, image, status, dcf, vbus);
    again.start();
    CHECK(scan_until(image, rt, [] { return g_byte_in[11] == 5; }, std::chrono::seconds(10)));
    CHECK(!scan_until(image, rt, [] { return g_byte_in[10] != 0; }, std::chrono::seconds(2)));
    CHECK(node_status(status, 2).held && restored->last_command() == 2);
    CHECK(restored->restores() == 0);

    again.stop();
    run = false;
    if (listener.joinable())
        listener.join();
    if (vbus)
    {
        // TIME every 100 ms: 6 bytes on 0x100.
        size_t time_frames = 0;
        for (const Frame &f : frames)
            time_frames += f.id == 0x100;
        std::printf("node options: %zu TIME frames\n", time_frames);
        CHECK(time_frames >= 20);
    }
    restored.reset();
    as_is.stop();
    keeps.stop();
}

size_t events_of(const NetworkStatus &status, unsigned id, EventKind kind)
{
    size_t n = 0;
    for (const Event &e : status.snapshot().events)
        n += e.node == id && e.kind == kind;
    return n;
}

// One request over the status socket at `path`, without the newline; "" on failure.
std::string ask(const std::string &path, const std::string &request)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
    if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
    {
        close(fd);
        return "";
    }
    std::string line = request + "\n";
    if (send(fd, line.data(), line.size(), 0) != static_cast<ssize_t>(line.size()))
    {
        close(fd);
        return "";
    }
    std::string answer;
    char buf[4096];
    while (answer.find('\n') == std::string::npos)
    {
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0)
            break;
        answer.append(buf, static_cast<size_t>(n));
    }
    close(fd);
    if (!answer.empty() && answer.back() == '\n')
        answer.pop_back();
    return answer;
}

uint32_t link_resets(const NetworkStatus &status)
{
    return status.snapshot().bus.resets;
}

size_t resets_of(const NetworkStatus &status, ResetOutcome outcome)
{
    size_t n = 0;
    for (const Event &e : status.snapshot().events)
        n += e.kind == EventKind::BusReset && e.reset == outcome;
    return n;
}

size_t resets_for(const NetworkStatus &status, ResetCause cause)
{
    size_t n = 0;
    for (const Event &e : status.snapshot().events)
        n += e.kind == EventKind::BusReset && e.reset == ResetOutcome::Done && e.cause == cause;
    return n;
}

// A bus-off link without auto-restart is reset by the master and the node comes back; without
// permission the master says so once and keeps the session.
void check_bus_off_reset(std::shared_ptr<lely::io::VirtualCanController> vbus,
                         const std::string &dir, const plugin_runtime_args_t &rt)
{
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(kWatchConfig, cfg, errors));
    if (!errors.empty())
        return;
    const NetworkConfig &net = cfg.networks[0];
    MasterLayout layout      = master_layout(net);
    ProcessImage image;
    CHECK(image.build(net, layout, kSize, errors));
    std::string dcf = dir + "/busoff.dcf";
    {
        FILE *f       = std::fopen(dcf.c_str(), "w");
        std::string t = master_dcf(net, layout);
        std::fwrite(t.data(), 1, t.size(), f);
        std::fclose(f);
    }
    SimSlaveSpec spec;
    spec.node_id = 2;
    SimSlave io(spec, dir + "/busoff_node2.dcf", vbus, "");
    io.start();

    {
        FakeLink link;
        NetworkStatus status(net);
        Bus bus(net, layout, image, status, dcf, vbus, &link);
        bus.start();
        CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 5; }, std::chrono::seconds(10)));
        unsigned boots = node_status(status, 2).boots;
        link.set_can_state(canopen_test::kCanBusOff);
        CHECK(scan_until(
            image, rt, [&link] { return link.downs() == 1 && link.ups() == 1; },
            std::chrono::seconds(3)));
        CHECK(errors_with("is bus-off; resetting the link") == 1);
        CHECK(link_resets(status) == 1 && resets_of(status, ResetOutcome::Done) == 1);
        CHECK(scan_until(
            image, rt,
            [&status, boots]
            {
                return node_status(status, 2).state == NodeRun::Operational &&
                       node_status(status, 2).boots > boots;
            },
            std::chrono::seconds(10)));
        CHECK(g_byte_in[10] == 5);
        bus.stop();
    }

    {
        FakeLink link;
        link.refuse_all();
        NetworkStatus status(net);
        Bus bus(net, layout, image, status, dcf, vbus, &link);
        bus.start();
        CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 5; }, std::chrono::seconds(10)));
        link.set_can_state(canopen_test::kCanBusOff);
        CHECK(scan_until(
            image, rt, [&status] { return resets_of(status, ResetOutcome::NotPermitted) == 1; },
            std::chrono::seconds(3)));
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        CHECK(errors_with("not permitted to reset it") == 1);
        CHECK(resets_of(status, ResetOutcome::NotPermitted) == 1 && link_resets(status) == 0);
        CHECK(link.downs() == 0);
        CHECK(node_status(status, 2).state == NodeRun::Operational);
        bus.stop();
    }
    io.stop();
}

// A link found at another bit rate than configured, or one that stopped sending, is reset by the
// master and the node comes back; with no bit rate configured the link's rate is left alone.
void check_link_fault_reset(std::shared_ptr<lely::io::VirtualCanController> vbus,
                            const std::string &dir, const plugin_runtime_args_t &rt)
{
    using Body = std::function<void(FakeLink &, NetworkStatus &, ProcessImage &)>;
    auto run   = [&](unsigned bitrate, const Body &body)
    {
        std::string text = kWatchConfig;
        text.replace(text.find("\"bitrate\": 0"), 12, "\"bitrate\": " + std::to_string(bitrate));
        Config cfg;
        std::vector<std::string> errors;
        CHECK(parse_config(text, cfg, errors));
        if (!errors.empty())
            return;
        const NetworkConfig &net = cfg.networks[0];
        MasterLayout layout      = master_layout(net);
        ProcessImage image;
        CHECK(image.build(net, layout, kSize, errors));
        std::string dcf = dir + "/linkfault.dcf";
        {
            FILE *f       = std::fopen(dcf.c_str(), "w");
            std::string t = master_dcf(net, layout);
            std::fwrite(t.data(), 1, t.size(), f);
            std::fclose(f);
        }
        SimSlaveSpec spec;
        spec.node_id = 2;
        SimSlave io(spec, dir + "/linkfault_node2.dcf", vbus, "");
        io.start();
        {
            FakeLink link;
            NetworkStatus status(net);
            Bus bus(net, layout, image, status, dcf, vbus, &link);
            bus.start();
            CHECK(
                scan_until(image, rt, [] { return g_byte_in[10] == 5; }, std::chrono::seconds(10)));
            body(link, status, image);
            bus.stop();
        }
        io.stop();
    };
    auto back_up = [](NetworkStatus &status, unsigned boots)
    {
        return [&status, boots]
        {
            return node_status(status, 2).state == NodeRun::Operational &&
                   node_status(status, 2).boots > boots;
        };
    };

    // Wrong bit rate: one reset, the link back at the configured rate, the node up again.
    run(500000,
        [&](FakeLink &link, NetworkStatus &status, ProcessImage &image)
        {
            unsigned boots = node_status(status, 2).boots;
            link.set(true, 125000, 0);
            CHECK(scan_until(
                image, rt, [&link] { return link.downs() == 1 && link.ups() == 1; },
                std::chrono::seconds(3)));
            CHECK(link.info().bitrate == 500000);
            CHECK(errors_with("is at 125000 bit/s, not the configured 500000") == 1);
            CHECK(resets_for(status, ResetCause::BitRate) == 1 && link_resets(status) == 1);
            CHECK(scan_until(image, rt, back_up(status, boots), std::chrono::seconds(10)));
        });

    // No bit rate configured: another rate is left alone.
    run(0,
        [&](FakeLink &link, NetworkStatus &status, ProcessImage &)
        {
            link.set(true, 125000, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
            CHECK(link.downs() == 0 && link_resets(status) == 0);
        });

    // Stalled: nothing sent while drops rise; one reset after 3 s, the node up again.
    run(0,
        [&](FakeLink &link, NetworkStatus &status, ProcessImage &image)
        {
            unsigned boots   = node_status(status, 2).boots;
            uint64_t dropped = 0;
            auto end         = std::chrono::steady_clock::now() + std::chrono::seconds(8);
            while (link.downs() == 0 && std::chrono::steady_clock::now() < end)
            {
                link.set_tx(100, ++dropped);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            link.set_tx(200, dropped);
            CHECK(link.downs() == 1);
            CHECK(errors_with("stopped sending and drops frames; resetting the link") == 1);
            CHECK(resets_for(status, ResetCause::TxStalled) == 1 && link_resets(status) == 1);
            CHECK(scan_until(image, rt, back_up(status, boots), std::chrono::seconds(10)));
        });

    // The link's sent count stands still but the master's writes are confirmed: no reset.
    run(0,
        [&](FakeLink &link, NetworkStatus &status, ProcessImage &)
        {
            link.set_tx(300, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(4500));
            CHECK(link.downs() == 0 && link_resets(status) == 0);
        });
}

} // namespace

// Service mode: the master sends nothing of its own, neither resets nor starts the node, and
// serves SDO and NMT requests to it.
void check_service_mode(std::shared_ptr<lely::io::VirtualCanController> vbus,
                        const std::string &wire, const std::string &dir,
                        const plugin_runtime_args_t &rt)
{
    SimSlaveSpec spec;
    spec.node_id = 2;
    SimSlave io(spec, dir + "/service_node2.dcf", vbus, wire);
    io.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ServiceRequest req;
    req.interface           = wire.empty() ? "vcan-test" : wire;
    req.sdo_timeout_ms      = 500;
    req.nodes               = {2, 3};
    const NetworkConfig net = service_network(req);
    MasterLayout layout     = master_layout(net);
    ProcessImage image;
    std::vector<std::string> errors;
    CHECK(image.build(net, layout, kSize, errors));
    std::string dcf = dir + "/service.dcf";
    {
        FILE *f       = std::fopen(dcf.c_str(), "w");
        std::string t = master_dcf(net, layout);
        std::fwrite(t.data(), 1, t.size(), f);
        std::fclose(f);
    }

    std::atomic<bool> run{true};
    std::vector<Frame> frames;
    std::thread listener;
    if (vbus)
        listener = std::thread([&] { listen(vbus, run, frames); });
    NetworkStatus status(net);
    status.set_service(true);
    Bus bus(net, layout, image, status, dcf, vbus, nullptr, true);
    bus.start();
    CHECK(scan_until(
        image, rt, [&status] { return node_status(status, 2).state == NodeRun::Service; },
        std::chrono::seconds(2)));

    auto finish = [&](ClientJob job)
    {
        std::string reason;
        JobResult r;
        uint64_t id = 0;
        auto end    = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!(id = bus.client().submit(job, reason)) && std::chrono::steady_clock::now() < end)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CHECK(id != 0);
        end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (std::chrono::steady_clock::now() < end && bus.client().result(id, r) &&
               r.state == JobResult::State::Pending)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return r;
    };
    ClientJob read;
    read.kind   = ClientJob::Kind::Read;
    read.node   = 2;
    read.index  = 0x1000;
    JobResult r = finish(read);
    CHECK(r.state == JobResult::State::Done && r.data.size() == 4);
    // An absent node: the SDO times out.
    read.node = 3;
    r         = finish(read);
    CHECK(r.state == JobResult::State::Failed && r.has_abort && r.abort_code == 0x05040000);

    ClientJob nmt;
    nmt.kind    = ClientJob::Kind::Nmt;
    nmt.node    = 2;
    nmt.command = NmtRequest::PreOperational;
    r           = finish(nmt);
    CHECK(r.state == JobResult::State::Done);
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (io.last_command() != 0x80 && std::chrono::steady_clock::now() < end)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    CHECK(io.last_command() == 0x80);
    CHECK(node_status(status, 2).state == NodeRun::Service);
    CHECK(node_status(status, 2).nmt == NmtSeen::PreOperational);

    nmt.command = NmtRequest::Start;
    r           = finish(nmt);
    CHECK(r.state == JobResult::State::Done);
    CHECK(node_status(status, 2).nmt == NmtSeen::Operational);

    // A reset: the boot-up reports pre-operational, and the node stays in service mode.
    nmt.command = NmtRequest::ResetNode;
    r           = finish(nmt);
    CHECK(r.state == JobResult::State::Done);
    CHECK(scan_until(
        image, rt,
        [&status]
        {
            NodeStatus s = node_status(status, 2);
            return s.nmt == NmtSeen::PreOperational && s.state == NodeRun::Service;
        },
        std::chrono::seconds(3)));
    CHECK(node_status(status, 2).state == NodeRun::Service);

    bus.stop();
    run = false;
    if (listener.joinable())
        listener.join();
    // The only NMT frames are the client's; no SYNC.
    size_t nmt_frames = 0, syncs = 0;
    for (const Frame &f : frames)
    {
        nmt_frames += f.id == 0;
        syncs += f.id == 0x80;
    }
    if (vbus)
        CHECK(nmt_frames == 3 && syncs == 0);
    io.stop();
}

// The socket's requests over one bus, as the plugin answers them while a PLC runs.
class BusRequests : public Requests
{
  public:
    BusRequests(NetworkStatus &status, Bus &bus, const NetworkConfig &net)
        : status_(status), bus_(bus), net_(net)
    {
    }
    std::string status() override
    {
        return status_json({status_.snapshot()}, wall_ms());
    }
    Lookup network(size_t net, unsigned node, ClientQueue *&queue) override
    {
        if (net != 0)
            return Lookup::UnknownNetwork;
        bool known = node == 0;
        for (const NodeConfig &n : net_.nodes)
            known = known || n.node_id == node;
        if (!known)
            return Lookup::UnknownNode;
        queue = &bus_.client();
        return Lookup::Ok;
    }
    bool job(uint64_t id, JobResult &out) override
    {
        return bus_.client().result(id, out);
    }
    std::string service_open(const ServiceRequest &) override
    {
        return "plc_running";
    }
    void service_close() override {}

  private:
    NetworkStatus &status_;
    Bus &bus_;
    const NetworkConfig &net_;
};

// Sends an SDO request and waits for its job to end, scanning meanwhile; the job's last answer.
std::string run_job(ProcessImage &image, const plugin_runtime_args_t &rt, const std::string &sock,
                    const std::string &request)
{
    std::string answer = ask(sock, request);
    size_t at          = answer.find("\"job\":");
    if (at == std::string::npos)
        return answer;
    const std::string poll =
        "{\"op\":\"job\",\"job\":" + answer.substr(at + 6, answer.find('}') - at - 6) + "}";
    scan_until(
        image, rt,
        [&]
        {
            answer = ask(sock, poll);
            return answer.find("pending") == std::string::npos;
        },
        std::chrono::seconds(5));
    return answer;
}

std::string sdo_request(const char *op, unsigned node, unsigned index, unsigned sub,
                        const std::string &data = "")
{
    std::string r = std::string("{\"op\":\"") + op +
                    "\",\"network\":0,\"node\":" + std::to_string(node) +
                    ",\"index\":" + std::to_string(index) + ",\"subindex\":" + std::to_string(sub);
    if (!data.empty())
        r += ",\"data\":\"" + data + "\"";
    return r + "}";
}

std::string nmt_request(unsigned node, const char *command)
{
    return "{\"op\":\"nmt\",\"network\":0,\"node\":" + std::to_string(node) + ",\"command\":\"" +
           command + "\"}";
}

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
    SimSlaveSpec spec5;
    spec5.node_id = 5;
    SimSlave timed(spec5, dir + "/node5.dcf", vbus, wire);
    SimSlaveSpec spec6;
    spec6.node_id             = 6;
    spec6.parameter_read_only = true;
    SimSlave refuses(spec6, dir + "/node6.dcf", vbus, wire);
    io1->start();
    wrong.start();
    foreign.start();
    timed.set_inputs(0x42, 0);
    io1->set_inputs(0xA5, -1234);
    wrong.set_inputs(0x77, 0);

    NetworkStatus status(net);
    Bus bus(net, layout, image, status, dcf, vbus);
    bus.start();
    const std::string sock = dir + "/status.socket";
    BusRequests requests(status, bus, net);
    StatusServer server(sock, [&requests](const std::string &request)
                        { return handle_request(request, requests); });
    std::string problem;
    CHECK(server.start(problem));

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
    // Nodes 5 and 6 join only now: their boots would load the bus thread during the timing check.
    timed.start();
    refuses.start();

    // The node with the wrong identity never comes up; the other one is not held back.
    CHECK(g_byte_in[11] == 0);
    CHECK(g_byte_in[1] == 0);

    // Status: node 2 operational; node 3 failed with the identity field, expected and found
    // values; node 6 failed with the refused SDO and its abort code.
    CHECK(node_status(status, 2).state == NodeRun::Operational);
    CHECK(node_status(status, 2).nmt == NmtSeen::Operational);
    CHECK(scan_until(
        image, rt,
        [&status]
        {
            Problem p = node_status(status, 3).problem;
            return node_status(status, 3).state == NodeRun::Failed &&
                   p.kind == ProblemKind::Identity && p.has_found;
        },
        std::chrono::seconds(5)));
    {
        Problem p = node_status(status, 3).problem;
        CHECK(p.field == IdentityField::VendorId && p.expected == 153 && p.found == 1);
    }
    CHECK(scan_until(
        image, rt,
        [&status]
        {
            return node_status(status, 6).state == NodeRun::Failed &&
                   node_status(status, 6).problem.kind == ProblemKind::SdoAbort;
        },
        std::chrono::seconds(5)));
    {
        Problem p = node_status(status, 6).problem;
        CHECK(p.index == 0x2100 && p.subindex == 0 && p.abort_code == 0x06010002);
        CHECK(events_of(status, 6, EventKind::SdoAbort) >= 1);
    }

    // Emergency and its reset.
    io1->send_emcy(0x2310, 0x03);
    CHECK(scan_until(
        image, rt, [&status] { return !node_status(status, 2).emcy.empty(); },
        std::chrono::seconds(2)));
    {
        NodeStatus n = node_status(status, 2);
        CHECK(n.emcy[0].code == 0x2310 && n.emcy[0].reg == 0x03 && n.emcy[0].active);
        CHECK(n.warning());
    }
    io1->clear_emcy();
    CHECK(scan_until(
        image, rt, [&status] { return !node_status(status, 2).warning(); },
        std::chrono::seconds(2)));
    CHECK(!node_status(status, 2).emcy[0].active);
    CHECK(events_of(status, 2, EventKind::EmcyReset) == 1);

    // Late PDOs: node 2 holds its inputs, node 5 zeroes them; both stay operational.
    CHECK(scan_until(image, rt, [] { return g_byte_in[2] == 0x42; }, std::chrono::seconds(5)));
    io1->set_tpdo_enabled(false);
    timed.set_tpdo_enabled(false);
    CHECK(scan_until(
        image, rt, [&status]
        { return !node_status(status, 2).late.empty() && !node_status(status, 5).late.empty(); },
        std::chrono::seconds(2)));
    CHECK(scan_until(image, rt, [] { return g_byte_in[2] == 0; }, std::chrono::seconds(1)));
    CHECK(g_byte_in[0] == 0xA5);
    CHECK(node_status(status, 2).state == NodeRun::Operational && node_status(status, 2).warning());
    CHECK(node_status(status, 5).state == NodeRun::Operational);
    CHECK(warnings_with("node 5 (timed): TPDO 1 (COB-ID 0x185) is not received in time") == 1);
    io1->set_tpdo_enabled(true);
    timed.set_tpdo_enabled(true);
    CHECK(scan_until(
        image, rt, [&status]
        { return node_status(status, 2).late.empty() && node_status(status, 5).late.empty(); },
        std::chrono::seconds(2)));
    CHECK(scan_until(image, rt, [] { return g_byte_in[2] == 0x42; }, std::chrono::seconds(1)));
    CHECK(events_of(status, 5, EventKind::PdoBack) == 1);

    // The status over the socket.
    {
        std::string answer = ask(sock, R"({"op":"status"})");
        cJSON *root        = cJSON_Parse(answer.c_str());
        CHECK(root != nullptr);
        if (root)
        {
            const cJSON *nets  = cJSON_GetObjectItem(root, "networks");
            const cJSON *nodes = cJSON_GetObjectItem(cJSON_GetArrayItem(nets, 0), "nodes");
            const cJSON *first = cJSON_GetArrayItem(nodes, 0);
            CHECK(cJSON_GetObjectItem(root, "version")->valueint == 1);
            CHECK(std::string(cJSON_GetObjectItem(first, "state")->valuestring) == "operational");
            const cJSON *bus_state = cJSON_GetObjectItem(
                cJSON_GetObjectItem(cJSON_GetArrayItem(nets, 0), "bus"), "state");
            CHECK(std::string(bus_state->valuestring) == "error_active");
            cJSON_Delete(root);
        }
        CHECK(ask(sock, R"({"op":"nmt"})") == R"({"error":"bad_request"})");
        CHECK(ask(sock, R"({"op":"teleport"})") == R"({"error":"unknown_op"})");
        CHECK(ask(sock, "nonsense").find("error") != std::string::npos);
    }

    // Online tools on node 2: reads (expedited and segmented), a write the node refuses while
    // operational, then the same write in pre-operational, a save and a reset.
    {
        std::string a = run_job(image, rt, sock, sdo_request("sdo_read", 2, 0x1018, 1));
        CHECK(a.find("\"state\":\"done\",\"data\":\"") != std::string::npos);
        CHECK(run_job(image, rt, sock, sdo_request("sdo_read", 2, 0x1008, 0)) ==
              "{\"state\":\"done\",\"data\":\"" +
                  to_hex({'G', 'e', 'n', 'e', 'r', 'i', 'c', ' ', 'I', '/', 'O', ' ', 'm', 'o', 'd',
                          'u', 'l', 'e'}) +
                  "\"}");
        CHECK(run_job(image, rt, sock, sdo_request("sdo_read", 2, 0x5555, 0)) ==
              R"({"state":"failed","abort_code":100794368})"); // 0x06020000
        CHECK(run_job(image, rt, sock, sdo_request("sdo_write", 2, 0x2101, 0, "1400")) ==
              R"({"state":"failed","abort_code":134217762})"); // 0x08000022
        CHECK(ask(sock, sdo_request("sdo_read", 7, 0x1000, 0)) == R"({"error":"unknown_node"})");
        CHECK(ask(sock, R"({"op":"sdo_read","network":0,"node":2,"index":1})") ==
              R"({"error":"bad_request"})");

        size_t late_events = events_of(status, 2, EventKind::PdoLate);
        CHECK(ask(sock, nmt_request(2, "pre_operational")) == R"({"ok":true})");
        CHECK(scan_until(
            image, rt, [&status] { return node_status(status, 2).state == NodeRun::Manual; },
            std::chrono::seconds(2)));
        CHECK(scan_until(
            image, rt, [&io1] { return io1->last_command() == 0x80; }, std::chrono::seconds(2)));
        // In pre-operational the node sends no PDOs; that is not reported as late or as a loss.
        scan_until(image, rt, [] { return false; }, std::chrono::milliseconds(400));
        CHECK(node_status(status, 2).state == NodeRun::Manual);
        CHECK(node_status(status, 2).losses == 0);
        CHECK(events_of(status, 2, EventKind::PdoLate) == late_events);
        CHECK(g_byte_in[0] == 0xA5);
        CHECK(events_of(status, 2, EventKind::NmtCommand) == 1);

        CHECK(run_job(image, rt, sock, sdo_request("sdo_write", 2, 0x2101, 0, "1400")) ==
              R"({"state":"done"})");
        CHECK(run_job(image, rt, sock, sdo_request("sdo_read", 2, 0x2101, 0)) ==
              R"({"state":"done","data":"1400"})");
        CHECK(run_job(image, rt, sock, sdo_request("sdo_write", 2, 0x1010, 1, "73617665")) ==
              R"({"state":"done"})");
        CHECK(io1->saves() == 1);
        CHECK(events_of(status, 2, EventKind::SdoWrite) == 3);

        CHECK(ask(sock, nmt_request(2, "start")) == R"({"ok":true})");
        CHECK(scan_until(
            image, rt, [&status] { return node_status(status, 2).state == NodeRun::Operational; },
            std::chrono::seconds(2)));
        CHECK(scan_until(
            image, rt, [&io1] { return io1->last_command() == 1; }, std::chrono::seconds(2)));

        // A reset node is configured and started again, without counting as a loss.
        CHECK(ask(sock, nmt_request(2, "reset_node")) == R"({"ok":true})");
        CHECK(scan_until(
            image, rt, [&status] { return node_status(status, 2).state == NodeRun::Booting; },
            std::chrono::seconds(2)));
        CHECK(scan_until(
            image, rt,
            [&status, &io1] {
                return node_status(status, 2).state == NodeRun::Operational &&
                       io1->last_command() == 1;
            },
            std::chrono::seconds(10)));
        CHECK(io1->parameter() == 0x12345678);
        CHECK(node_status(status, 2).losses == 0);
        // The reset put the node's parameter back to its default.
        CHECK(run_job(image, rt, sock, sdo_request("sdo_read", 2, 0x2101, 0)) ==
              R"({"state":"done","data":"0a00"})");
        // Its inputs, reset too, come in again.
        CHECK(scan_until(image, rt, [] { return g_byte_in[0] == 0; }, std::chrono::seconds(2)));
        io1->set_inputs(0xA5, -1234);
        CHECK(scan_until(image, rt, [] { return g_byte_in[0] == 0xA5; }, std::chrono::seconds(2)));
    }

    // Loss: heartbeat stops, state 0, inputs held.
    io1.reset();
    CHECK(scan_until(image, rt, [] { return g_byte_in[10] == 0; }, std::chrono::seconds(3)));
    CHECK(g_byte_in[0] == 0xA5);
    CHECK(node_status(status, 2).state == NodeRun::Lost);
    CHECK(node_status(status, 2).problem.kind == ProblemKind::Heartbeat);
    CHECK(node_status(status, 2).losses == 1);

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
    // Nodes 5 and 6 joined late, so on a slow machine they may have been reported too.
    CHECK(warnings_with("has not answered") ==
          1 + warnings_with("node 5 (timed) has not answered") +
              warnings_with("node 6 (refuses) has not answered"));
    CHECK(node_status(status, 9).state == NodeRun::Silent);
    CHECK(node_status(status, 2).state == NodeRun::Operational);

    // PLC stop: the configured nodes go to PRE-OPERATIONAL, a node the master does not manage is
    // left alone.
    server.stop();
    CHECK(access(sock.c_str(), F_OK) != 0);
    bus.stop();
    CHECK(node_status(status, 2).state == NodeRun::Stopped);
    CHECK(io1->last_command() == 0x80);
    CHECK(wrong.last_command() == 0x80);
    CHECK(foreign.last_command() != 0x80);

    io1.reset();
    wrong.stop();
    foreign.stop();
    timed.stop();
    refuses.stop();

    check_master_watch(vbus, wire, dir, rt);
    if (vbus)
    {
        check_bus_off_reset(vbus, dir, rt);
        check_link_fault_reset(vbus, dir, rt);
    }
    check_node_options(vbus, wire, dir, rt);
    check_service_mode(vbus, wire, dir, rt);
    CHECK(system(("rm -rf " + dir).c_str()) == 0);
    return TEST_RESULT();
}
