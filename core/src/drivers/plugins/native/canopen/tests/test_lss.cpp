// SPDX-License-Identifier: Apache-2.0
//
// Node IDs over LSS on Lely's in-process virtual CAN bus (or on the SocketCAN interface named by
// CANOPEN_TEST_INTERFACE), against simulated generic I/O modules that are LSS devices: a device
// without a node ID gets the node's and boots; one that already has it is left as it is; one with
// another ID gets the node's and is reset to boot with it; a device not on the bus is reported and
// looked for again; store only when asked, and once when the ID was already right; a revision
// that is not configured is found with a slowscan; the result in the status JSON.

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

#include <lely/io2/sys/clock.hpp>
#include <lely/io2/vcan.hpp>

#include "bus.h"
#include "check.h"
#include "config.h"
#include "log.h"
#include "master_dcf.h"
#include "process_image.h"
#include "sim_slave.h"
#include "status.h"

using namespace canopen_plugin;
using canopen_sim::SimSlave;
using canopen_sim::SimSlaveSpec;

namespace
{

constexpr int kSize = 16;
IEC_BYTE g_byte_in[kSize];
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
int write_int(int, int, int)
{
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

std::mutex g_log_mutex;
std::vector<std::string> g_log;

void sink(LogLevel level, const char *msg)
{
    if (level == LogLevel::Debug)
        return;
    std::printf("%s\n", msg);
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log.push_back(msg);
}

size_t logged(const std::string &text)
{
    std::lock_guard<std::mutex> lock(g_log_mutex);
    size_t n = 0;
    for (const auto &l : g_log)
        n += l.find(text) != std::string::npos;
    return n;
}

// Node 2 found by its LSS address (serial 0x1001), its state in %IB10 and its digital inputs in
// %IB0 (with its analogue input in %IW0); node 3 (serial 0x1003) only with `absent`.
std::string config_text(const std::string &wire, const std::string &lss2, bool absent)
{
    std::string t = R"({
  "version": 7,
  "networks": [{
    "name": "lss", "interface": ")" +
                    wire + R"(", "bitrate": 0,
    "sdo_timeout_ms": 300,
    "nodes": [
      {"node_id": 2, "start": "master", "enabled": true, "name": "io1",
       "identity": {"device_type": 401, "vendor_id": 1, "product_code": 2, "revision": 3},
       "lss": {)" + lss2 +
                    R"(},
       "supervision": {"type": "heartbeat", "consumer_ms": 300},
       "state_location": "%IB10",
       "tpdos": [{"number": 1, "cob_id": 386, "entries": [
         {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB0"},
         {"index": 25601, "subindex": 1, "type": "INTEGER16", "location": "%IW0"}]}]})";
    if (absent)
        t += R"(,
      {"node_id": 3, "start": "master", "enabled": true, "optional": true, "name": "missing",
       "lss": {"vendor_id": 1, "product_code": 2, "revision": 3, "serial_number": 4099},
       "state_location": "%IB11"})";
    t += R"(
    ]
  }]
})";
    return t;
}

struct Run
{
    Config cfg;
    MasterLayout layout;
    ProcessImage image;
    std::unique_ptr<NetworkStatus> status;
    std::unique_ptr<Bus> bus;
};

// Starts the master with `text`; nullptr when the configuration is refused.
std::unique_ptr<Run> start_master(const std::string &text, const std::string &dir,
                                  std::shared_ptr<lely::io::VirtualCanController> vbus)
{
    std::unique_ptr<Run> r(new Run);
    std::vector<std::string> errors;
    CHECK(parse_config(text, r->cfg, errors));
    for (const auto &e : errors)
        std::fprintf(stderr, "%s\n", e.c_str());
    if (!errors.empty())
        return nullptr;
    const NetworkConfig &net = r->cfg.networks[0];
    r->layout                = master_layout(net);
    CHECK(r->image.build(net, r->layout, ImageSizes::uniform(kSize), errors));
    const std::string dcf = dir + "/master.dcf";
    const std::string t   = master_dcf(net, r->layout);
    CHECK(t.find("LSS_Supported=1") != std::string::npos);
    FILE *f = std::fopen(dcf.c_str(), "w");
    std::fwrite(t.data(), 1, t.size(), f);
    std::fclose(f);
    std::memset(g_byte_in, 0, sizeof g_byte_in);
    r->status.reset(new NetworkStatus(net));
    r->bus.reset(new Bus(net, r->layout, r->image, *r->status, dcf, vbus));
    r->bus->start();
    return r;
}

bool scan_until(Run &r, const plugin_runtime_args_t &rt, std::function<bool()> done,
                std::chrono::milliseconds timeout)
{
    auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end)
    {
        r.image.copy_to_plc(rt);
        if (done())
            return true;
        r.image.copy_from_plc(rt);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

NodeStatus node_status(const Run &r, unsigned id)
{
    NetworkSnapshot s = r.status->snapshot();
    for (const NodeStatus &n : s.nodes)
        if (n.node_id == id)
            return n;
    return NodeStatus();
}

size_t lss_events(const Run &r, unsigned id)
{
    size_t n = 0;
    for (const Event &e : r.status->snapshot().events)
        n += e.node == id && e.kind == EventKind::Lss;
    return n;
}

SimSlaveSpec lss_spec(unsigned node_id, uint32_t serial)
{
    SimSlaveSpec s;
    s.node_id       = node_id;
    s.lss           = true;
    s.serial_number = serial;
    return s;
}

const std::string kAddr2 =
    R"("vendor_id": 1, "product_code": 2, "revision": 3, "serial_number": 4097)";

} // namespace

int main()
{
    set_log_sink(sink);
    route_lely_diagnostics();
    prepare_wake_signal();
    const std::string dir = "/tmp/canopen_lss_test_" + std::to_string(getpid());
    CHECK(system(("mkdir -p " + dir).c_str()) == 0);
    const char *iface = std::getenv("CANOPEN_TEST_INTERFACE");
    std::string wire;
    std::shared_ptr<lely::io::VirtualCanController> vbus;
    if (iface && *iface)
        wire = iface;
    else
        vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    const std::string name   = wire.empty() ? "vcan-test" : wire;
    plugin_runtime_args_t rt = fake_args();
    auto operational         = [](unsigned at) { return [at] { return g_byte_in[at] == 5; }; };

    // 1. A device without a node ID gets node 2's, boots and runs its PDOs. Store is off. The
    //    absent node 3 is reported, without holding node 2 back, and looked for again.
    {
        SimSlave dev(lss_spec(255, 0x1001), dir + "/dev1.dcf", vbus, wire);
        dev.set_inputs(0x21, 0);
        dev.start();
        auto r = start_master(config_text(name, kAddr2, true), dir, vbus);
        if (!r)
            return TEST_RESULT();
        CHECK(scan_until(*r, rt, operational(10), std::chrono::seconds(15)));
        CHECK(scan_until(*r, rt, [] { return g_byte_in[0] == 0x21; }, std::chrono::seconds(2)));
        CHECK(dev.node_id() == 2);
        CHECK(dev.lss_stores() == 0);
        NodeStatus n2 = node_status(*r, 2);
        CHECK(n2.has_lss && n2.lss.outcome == LssOutcome::Given);
        CHECK(n2.lss.previous_id == 255 && !n2.lss.stored);
        CHECK(logged("LSS gave the device node ID 2 (it had: none)") == 1);
        CHECK(scan_until(
            *r, rt, [&] { return node_status(*r, 3).lss.outcome == LssOutcome::NotFound; },
            std::chrono::seconds(5)));
        CHECK(logged("node 3 (missing): no device with") >= 1);
        // Looked for again with a growing pause; a repeat is not a new event.
        const size_t first = logged("node 3 (missing): no device with");
        scan_until(*r, rt, [] { return false; }, std::chrono::milliseconds(2500));
        CHECK(logged("node 3 (missing): no device with") > first);
        CHECK(lss_events(*r, 3) == 1);
        CHECK(g_byte_in[10] == 5);
        // The status JSON carries the result.
        const std::string json = status_json({r->status->snapshot()}, 1);
        CHECK(json.find("\"result\":\"given\"") != std::string::npos ||
              json.find("\"result\": \"given\"") != std::string::npos);
        r->bus->stop();
        dev.stop();
    }

    // 2. Store on: the ID is given and stored once; at the next PLC start the device already has
    //    it and nothing is stored again.
    {
        SimSlave dev(lss_spec(255, 0x1001), dir + "/dev2.dcf", vbus, wire);
        dev.start();
        {
            auto r =
                start_master(config_text(name, kAddr2 + ", \"store\": true", false), dir, vbus);
            if (!r)
                return TEST_RESULT();
            CHECK(scan_until(*r, rt, operational(10), std::chrono::seconds(15)));
            CHECK(dev.lss_stores() == 1);
            NodeStatus n2 = node_status(*r, 2);
            CHECK(n2.lss.outcome == LssOutcome::Given && n2.lss.stored);
            r->bus->stop();
        }
        {
            auto r =
                start_master(config_text(name, kAddr2 + ", \"store\": true", false), dir, vbus);
            if (!r)
                return TEST_RESULT();
            CHECK(scan_until(*r, rt, operational(10), std::chrono::seconds(15)));
            NodeStatus n2 = node_status(*r, 2);
            CHECK(n2.lss.outcome == LssOutcome::Already && !n2.lss.stored);
            CHECK(dev.lss_stores() == 1);
            r->bus->stop();
        }
        dev.stop();
    }

    // 3. A device that already has the node's ID and store on that was never stored since load:
    //    stored once, since LSS cannot tell whether the ID is in its memory.
    {
        SimSlave dev(lss_spec(2, 0x1005), dir + "/dev3.dcf", vbus, wire);
        dev.start();
        const std::string addr =
            R"("vendor_id": 1, "product_code": 2, "revision": 3, "serial_number": 4101, "store": true)";
        auto r = start_master(config_text(name, addr, false), dir, vbus);
        if (!r)
            return TEST_RESULT();
        CHECK(scan_until(*r, rt, operational(10), std::chrono::seconds(15)));
        NodeStatus n2 = node_status(*r, 2);
        CHECK(n2.lss.outcome == LssOutcome::Already && n2.lss.stored);
        CHECK(dev.lss_stores() == 1);
        r->bus->stop();
        dev.stop();
    }

    // 4. A device with another node ID (7) gets node 2's and is reset to boot with it. The
    //    revision is not configured and the device file's (3) is tried first.
    {
        SimSlave dev(lss_spec(7, 0x1007), dir + "/dev4.dcf", vbus, wire);
        dev.set_inputs(0x47, 0);
        dev.start();
        const std::string addr = R"("vendor_id": 1, "product_code": 2, "serial_number": 4103)";
        auto r                 = start_master(config_text(name, addr, false), dir, vbus);
        if (!r)
            return TEST_RESULT();
        CHECK(scan_until(*r, rt, operational(10), std::chrono::seconds(15)));
        CHECK(scan_until(*r, rt, [] { return g_byte_in[0] == 0x47; }, std::chrono::seconds(2)));
        CHECK(dev.node_id() == 2);
        NodeStatus n2 = node_status(*r, 2);
        CHECK(n2.lss.outcome == LssOutcome::Given && n2.lss.previous_id == 7);
        CHECK(dev.lss_stores() == 0);
        r->bus->stop();
        dev.stop();
    }

    // 5. Revision not configured and not the device file's: found with a slowscan over it.
    {
        SimSlaveSpec spec = lss_spec(255, 0x1009);
        spec.revision     = 0x00020005;
        SimSlave dev(spec, dir + "/dev5.dcf", vbus, wire);
        dev.start();
        std::string text =
            config_text(name, R"("vendor_id": 1, "product_code": 2, "serial_number": 4105)", false);
        // The identity check would refuse the other revision.
        text.replace(text.find(", \"revision\": 3}"), 16, "}");
        auto r = start_master(text, dir, vbus);
        if (!r)
            return TEST_RESULT();
        CHECK(scan_until(*r, rt, operational(10), std::chrono::seconds(30)));
        CHECK(dev.node_id() == 2);
        CHECK(node_status(*r, 2).lss.outcome == LssOutcome::Given);
        r->bus->stop();
        dev.stop();
    }

    CHECK(system(("rm -rf " + dir).c_str()) == 0);
    return TEST_RESULT();
}
