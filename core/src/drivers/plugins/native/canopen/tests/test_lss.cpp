// SPDX-License-Identifier: Apache-2.0
//
// A client's LSS requests and bus scans on Lely's in-process virtual CAN bus (or on the SocketCAN
// interface named by CANOPEN_TEST_INTERFACE), against simulated generic I/O modules that are LSS
// devices: find two devices without a node ID on a running network, give one a node ID with
// store, refuse an ID in use and a bit rate while the PLC runs; in service mode set and store a
// device's bit rate. In service mode for commissioning: SDO to a node ID no network lists, a bus
// scan that finds devices with their identity and the IDs only heard, a scan of an empty bus, and
// the refusal of a scan outside it. The master gives no node ID by itself.

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
#include "requests.h"
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

// Node 2 with its state in %IB10 and its digital inputs in %IB0 (with its analogue input in %IW0).
std::string config_text(const std::string &wire)
{
    return R"({
  "version": 15,
  "networks": [{
    "name": "lss", "interface": ")" +
           wire + R"(", "bitrate": 0,
    "sdo_timeout_ms": 300,
    "nodes": [
      {"node_id": 2, "start": "master", "enabled": true, "name": "io1",
       "identity": {"device_type": 401, "vendor_id": 1, "product_code": 2, "revision": 3},
       "supervision": {"type": "heartbeat", "consumer_ms": 300},
       "state_location": "%IB10",
       "tpdos": [{"number": 1, "cob_id": 386, "entries": [
         {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB0"},
         {"index": 25601, "subindex": 1, "type": "INTEGER16", "location": "%IW0"}]}]}
    ]
  }]
})";
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
    CHECK(t.find("LSS_Supported=0") != std::string::npos);
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

SimSlaveSpec lss_spec(unsigned node_id, uint32_t serial)
{
    SimSlaveSpec s;
    s.node_id       = node_id;
    s.lss           = true;
    s.serial_number = serial;
    return s;
}

// Submits a client job and waits for its end.
JobResult run_job(ClientQueue &q, ClientJob job, std::chrono::seconds timeout)
{
    std::string reason;
    uint64_t id = 0;
    auto end    = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!(id = q.submit(job, reason)) && std::chrono::steady_clock::now() < end)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    JobResult r;
    if (!id)
    {
        r.state  = JobResult::State::Failed;
        r.reason = reason;
        return r;
    }
    end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end && q.result(id, r) &&
           r.state == JobResult::State::Pending)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return r;
}

ClientJob lss_job(LssOnline::Action action, uint32_t serial = 0, unsigned node_id = 0,
                  bool store = false, unsigned bitrate = 0)
{
    ClientJob j;
    j.kind                     = ClientJob::Kind::Lss;
    j.lss.action               = action;
    j.lss.address.vendor_id    = 1;
    j.lss.address.product_code = 2;
    j.lss.address.revision     = 3;
    j.lss.address.serial_number = serial;
    j.lss.node_id              = node_id;
    j.lss.store                = store;
    j.lss.bitrate              = bitrate;
    return j;
}

// A service mode bus as the plugin opens it.
struct Service
{
    NetworkConfig net;
    MasterLayout layout;
    ProcessImage image;
    std::unique_ptr<NetworkStatus> status;
    std::unique_ptr<Bus> bus;

    Service(const std::string &name, const std::string &dir,
            std::shared_ptr<lely::io::VirtualCanController> vbus, bool commission)
    {
        ServiceRequest req;
        req.interface  = name;
        req.commission = commission;
        if (!commission)
            req.nodes = {2};
        net    = service_network(req);
        layout = master_layout(net);
        std::vector<std::string> errors;
        CHECK(image.build(net, layout, ImageSizes::uniform(kSize), errors));
        const std::string dcf = dir + "/service.dcf";
        FILE *f               = std::fopen(dcf.c_str(), "w");
        const std::string t   = master_dcf(net, layout);
        std::fwrite(t.data(), 1, t.size(), f);
        std::fclose(f);
        status.reset(new NetworkStatus(net));
        status->set_service(true);
        bus.reset(new Bus(net, layout, image, *status, dcf, vbus, nullptr, true));
        bus->start();
    }
    ~Service()
    {
        bus->stop();
    }
};

ClientJob scan_job()
{
    ClientJob j;
    j.kind = ClientJob::Kind::Scan;
    return j;
}

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

    // 1. A client finds the two devices without a node ID while node 2 runs, gives one node ID 4
    //    with store; node 2's ID and a bit rate while the PLC runs are refused.
    {
        SimSlave dev(lss_spec(2, 0x1001), dir + "/dev6.dcf", vbus, wire);
        SimSlave new1(lss_spec(255, 0x5678), dir + "/new1.dcf", vbus, wire);
        SimSlave new2(lss_spec(255, 0x5679), dir + "/new2.dcf", vbus, wire);
        dev.start();
        new1.start();
        new2.start();
        auto r = start_master(config_text(name), dir, vbus);
        if (!r)
            return TEST_RESULT();
        CHECK(scan_until(*r, rt, operational(10), std::chrono::seconds(15)));
        ClientQueue &q = r->bus->client();
        JobResult found;
        std::thread find([&] { found = run_job(q, lss_job(LssOnline::Action::Find),
                                                std::chrono::seconds(60)); });
        // Node 2 keeps running meanwhile.
        bool stayed = true;
        scan_until(*r, rt,
                   [&]
                   {
                       stayed = stayed && g_byte_in[10] == 5;
                       return found.state != JobResult::State::Pending;
                   },
                   std::chrono::seconds(60));
        find.join();
        CHECK(stayed);
        CHECK(found.state == JobResult::State::Done);
        CHECK(found.json.find("\"serial_number\":22136") != std::string::npos);
        CHECK(found.json.find("\"serial_number\":22137") != std::string::npos);
        CHECK(found.json.find("\"serial_number\":4097") == std::string::npos);
        CHECK(dev.node_id() == 2 && new1.node_id() == 255 && new2.node_id() == 255);

        JobResult used = run_job(q, lss_job(LssOnline::Action::SetId, 0x5678, 2),
                                 std::chrono::seconds(5));
        CHECK(used.state == JobResult::State::Failed && used.reason == "node_in_use");
        CHECK(new1.node_id() == 255);
        JobResult rate = run_job(q, lss_job(LssOnline::Action::SetBitrate, 0x5678, 0, true,
                                            250000),
                                 std::chrono::seconds(5));
        CHECK(rate.state == JobResult::State::Failed && rate.reason == "needs_plc_stopped");

        JobResult given = run_job(q, lss_job(LssOnline::Action::SetId, 0x5678, 4, true),
                                  std::chrono::seconds(5));
        CHECK(given.state == JobResult::State::Done);
        CHECK(given.json == "\"previous_id\":255,\"stored\":true");
        CHECK(scan_until(*r, rt, [&] { return new1.node_id() == 4; }, std::chrono::seconds(2)));
        CHECK(new1.lss_stores() == 1 && new2.node_id() == 255);
        // No device with that address.
        JobResult none = run_job(q, lss_job(LssOnline::Action::SetId, 0x9999, 5),
                                 std::chrono::seconds(5));
        CHECK(none.state == JobResult::State::Failed && none.reason == "not_found");
        CHECK(g_byte_in[10] == 5);
        r->bus->stop();
        new2.stop();
        new1.stop();
        dev.stop();
    }

    // 2. Service mode: a device's bit rate set and stored; its node ID given without store.
    {
        SimSlave dev(lss_spec(255, 0x5678), dir + "/dev7.dcf", vbus, wire);
        dev.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        Service svc(name, dir, vbus, false);
        Bus &bus       = *svc.bus;
        JobResult rate = run_job(bus.client(), lss_job(LssOnline::Action::SetBitrate, 0x5678, 0,
                                                       true, 250000),
                                 std::chrono::seconds(5));
        CHECK(rate.state == JobResult::State::Done && rate.json == "\"stored\":true");
        CHECK(dev.lss_stores() == 1 && dev.lss_bitrate() == 250000);
        JobResult given = run_job(bus.client(), lss_job(LssOnline::Action::SetId, 0x5678, 2),
                                  std::chrono::seconds(5));
        CHECK(given.state == JobResult::State::Done);
        CHECK(given.json == "\"previous_id\":255,\"stored\":false");
        auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (dev.node_id() != 2 && std::chrono::steady_clock::now() < end)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CHECK(dev.node_id() == 2 && dev.lss_stores() == 1);
        dev.stop();
    }

    // 3. Commissioning: a scan finds node 10 and node 127 with their identity, and a device
    //    without a node ID is not in it; SDO reaches node 127, which no network lists.
    {
        SimSlave a(lss_spec(10, 0x0A0A), dir + "/scan10.dcf", vbus, wire);
        SimSlave b(lss_spec(127, 0x7F7F), dir + "/scan127.dcf", vbus, wire);
        SimSlave none(lss_spec(255, 0x5555), dir + "/scan255.dcf", vbus, wire);
        a.start();
        b.start();
        none.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        Service svc(name, dir, vbus, true);
        ClientQueue &q = svc.bus->client();
        const auto t0  = std::chrono::steady_clock::now();
        JobResult r    = run_job(q, scan_job(), std::chrono::seconds(10));
        CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
        CHECK(r.state == JobResult::State::Done);
        CHECK(r.json.find("\"node_id\":10,\"answered\":true") != std::string::npos);
        CHECK(r.json.find("\"node_id\":127,\"answered\":true") != std::string::npos);
        CHECK(r.json.find("\"serial_number\":2570") != std::string::npos);
        CHECK(r.json.find("\"serial_number\":32639") != std::string::npos);
        CHECK(r.json.find("\"vendor_id\":1,\"product_code\":2,\"revision\":3") !=
              std::string::npos);
        CHECK(r.json.find("\"device_type\":401") != std::string::npos);
        CHECK(r.json.find("\"heartbeat\"") != std::string::npos);
        CHECK(r.json.find("21845") == std::string::npos);
        // Any node ID in commissioning.
        ClientJob read;
        read.kind     = ClientJob::Kind::Read;
        read.node     = 127;
        read.index    = 0x1018;
        read.subindex = 4;
        JobResult v   = run_job(q, read, std::chrono::seconds(5));
        CHECK(v.state == JobResult::State::Done && v.data.size() == 4 && v.data[0] == 0x7F &&
              v.data[1] == 0x7F);
        // The spare at 127 gets node ID 20 with store, and boots with it after a reset node.
        ClientJob give  = lss_job(LssOnline::Action::SetId, 0x7F7F, 20, true);
        JobResult given = run_job(q, give, std::chrono::seconds(5));
        CHECK(given.state == JobResult::State::Done);
        CHECK(given.json == "\"previous_id\":127,\"stored\":true");
        ClientJob reset;
        reset.kind    = ClientJob::Kind::Nmt;
        reset.node    = 127;
        reset.command = NmtRequest::ResetNode;
        CHECK(run_job(q, reset, std::chrono::seconds(5)).state == JobResult::State::Done);
        auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (b.node_id() != 20 && std::chrono::steady_clock::now() < end)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CHECK(b.node_id() == 20);
        JobResult again = run_job(q, scan_job(), std::chrono::seconds(10));
        CHECK(again.json.find("\"node_id\":20,\"answered\":true") != std::string::npos);
        CHECK(again.json.find("\"node_id\":127,") == std::string::npos);
        none.stop();
        b.stop();
        a.stop();
    }

    // 4. A scan of an empty bus ends within 5 seconds with nothing found.
    {
        Service svc(name, dir, vbus, true);
        const auto t0 = std::chrono::steady_clock::now();
        JobResult r   = run_job(svc.bus->client(), scan_job(), std::chrono::seconds(10));
        CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
        CHECK(r.state == JobResult::State::Done && r.json == "\"devices\":[]");
    }

    CHECK(system(("rm -rf " + dir).c_str()) == 0);
    return TEST_RESULT();
}
