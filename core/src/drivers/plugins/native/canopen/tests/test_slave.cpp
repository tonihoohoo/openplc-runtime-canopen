// SPDX-License-Identifier: Apache-2.0
//
// test_slave.cpp - the CANopen slave (slave/) on Lely's in-process bus: the frames a master sends
// (NMT, SDO, RPDO, SYNC, heartbeat) are written by hand and the slave's answers checked, with a
// fake PLC image on the scan side: boot-up and heartbeat, identity, SDO range checks and abort
// codes, a segmented string upload, PDOs both ways, SYNC TPDOs, remapping by the CiA 301 procedure,
// master loss and return, PLC stop and start, and reset node. Then our own master boots the slave
// from the configuration the slave's EDS would give and exchanges PDOs with it. Last, LSS: an LSS
// master on raw frames finds an unconfigured slave by fastscan, gives it a node ID and a bit rate
// and stores them; selective switch, inquire, a node ID change on a configured slave, store
// failure, bit rate refused, forget and LSS off. The PLC block side (plc_api.h
// version 3): status, local NMT, program emergencies, write counters and the NMT state on PLC
// stop.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
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
#include "lss_store.h"
#include "master_dcf.h"
#include "process_image.h"
#include "slave.h"
#include "status.h"

using namespace canopen_plugin;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

namespace
{

const char *const kConfig = R"({
  "version": 12,
  "networks": [],
  "devices": [{
    "name": "io", "interface": "vcan-slave", "bitrate": 0, "node_id": 5,
    "identity": {"device_name": "test-node", "hardware_version": "rev B", "vendor_id": 4660,
                 "product_code": 1, "revision": 65536},
    "heartbeat_ms": 100,
    "master_watch": {"node_id": 127, "time_ms": 300},
    "on_master_lost": "pre_operational",
    "on_plc_stop": "hold",
    "on_plc_stop_state": "none",
    "objects": [
      {"index": 8192, "subindex": 1, "name": "Setpoint", "type": "INTEGER16",
       "direction": "from_master", "exchange": "pdo", "location": "%IW0", "default": 0},
      {"index": 8192, "subindex": 2, "name": "Mode", "type": "UNSIGNED8",
       "direction": "from_master", "exchange": "pdo", "location": "%IB2", "default": 0},
      {"index": 8192, "subindex": 4, "name": "High limit", "type": "UNSIGNED16",
       "direction": "from_master", "exchange": "sdo", "location": "%IW2", "default": 1000,
       "min": 100, "max": 32767},
      {"index": 8448, "subindex": 1, "name": "Status", "type": "UNSIGNED16",
       "direction": "to_master", "exchange": "pdo", "location": "%QW0"},
      {"index": 8448, "subindex": 2, "name": "Actual", "type": "INTEGER16",
       "direction": "to_master", "exchange": "pdo", "location": "%QW1"},
      {"index": 8448, "subindex": 3, "name": "Counter", "type": "UNSIGNED8",
       "direction": "to_master", "exchange": "pdo", "location": "%QB0"},
      {"index": 8448, "subindex": 4, "name": "Run hours", "type": "UNSIGNED32",
       "direction": "to_master", "exchange": "sdo", "location": "%QD0"},
      {"index": 8448, "subindex": 5, "name": "Version", "type": "VISIBLE_STRING",
       "direction": "to_master", "exchange": "constant", "value": "1.4.0"}
    ],
    "rpdos": [{"number": 1, "cob_id": 517, "transmission": 255,
               "entries": [{"index": 8192, "subindex": 1}, {"index": 8192, "subindex": 2}]}],
    "tpdos": [{"number": 1, "cob_id": 389, "transmission": 255, "inhibit_100us": 0,
               "event_timer_ms": 0,
               "entries": [{"index": 8448, "subindex": 1}, {"index": 8448, "subindex": 2}]},
              {"number": 2, "cob_id": 645, "transmission": 1,
               "entries": [{"index": 8448, "subindex": 3}]}]
  }]
})";

// ---- fake PLC image ----
constexpr int kSize = 16;
IEC_BYTE g_byte_in[kSize];
IEC_BOOL g_bool_in[kSize][8];
IEC_BOOL g_bool_out[kSize][8];
IEC_UINT g_int_in[kSize];
IEC_UDINT g_dint_in[kSize];
IEC_BYTE g_byte_out[kSize];
IEC_UINT g_int_out[kSize];
IEC_UDINT g_dint_out[kSize];
IEC_BYTE *g_byte_out_ptr[kSize];
IEC_BOOL *g_bool_out_ptr[kSize][8];
IEC_UINT *g_int_out_ptr[kSize];
IEC_UDINT *g_dint_out_ptr[kSize];
IEC_ULINT *g_lint_out_ptr[kSize];

int write_bool(int, int index, int bit, int value)
{
    if (index >= 0 && index < kSize && bit >= 0 && bit < 8)
        g_bool_in[index][bit] = static_cast<IEC_BOOL>(value != 0);
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
int write_dint(int, int index, unsigned value)
{
    g_dint_in[index] = value;
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
    {
        g_byte_out_ptr[i] = &g_byte_out[i];
        g_int_out_ptr[i]  = &g_int_out[i];
        g_dint_out_ptr[i] = &g_dint_out[i];
        for (int b = 0; b < 8; ++b)
            g_bool_out_ptr[i][b] = &g_bool_out[i][b];
    }
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

std::atomic<int> g_master_seen_lines{0};

void quiet_sink(LogLevel, const char *msg)
{
    if (std::strstr(msg, "heartbeat of master node") && std::strstr(msg, " seen"))
        ++g_master_seen_lines;
    if (std::getenv("CANOPEN_TEST_VERBOSE"))
        std::fprintf(stderr, "%s\n", msg);
}

// ---- the bus as a master sees it ----
struct Frame
{
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    Clock::time_point at;
};

class Probe
{
  public:
    explicit Probe(std::shared_ptr<lely::io::VirtualCanController> bus) : bus_(bus)
    {
        worker_ = std::thread([this] { run(); });
    }
    ~Probe()
    {
        stop_ = true;
        worker_.join();
    }

    void send(uint32_t id, std::vector<uint8_t> data)
    {
        std::lock_guard<std::mutex> lock(out_mutex_);
        Frame f{id, static_cast<uint8_t>(data.size()), {}, Clock::now()};
        std::memcpy(f.data, data.data(), data.size());
        out_.push_back(f);
    }
    size_t mark()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return frames_.size();
    }
    // The first frame after `from` that matches, waiting up to `timeout`.
    bool wait(size_t from, std::function<bool(const Frame &)> match, milliseconds timeout,
              Frame *out = nullptr)
    {
        auto end = Clock::now() + timeout;
        for (;;)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (size_t i = from; i < frames_.size(); ++i)
                    if (match(frames_[i]))
                    {
                        if (out)
                            *out = frames_[i];
                        return true;
                    }
            }
            if (Clock::now() >= end)
                return false;
            std::this_thread::sleep_for(milliseconds(2));
        }
    }
    bool wait_id(size_t from, uint32_t id, milliseconds timeout, Frame *out = nullptr)
    {
        return wait(from, [id](const Frame &f) { return f.id == id; }, timeout, out);
    }
    size_t count(size_t from, uint32_t id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t n = 0;
        for (size_t i = from; i < frames_.size(); ++i)
            n += frames_[i].id == id;
        return n;
    }

  private:
    void run()
    {
        lely::io::Context ctx;
        lely::ev::Loop loop;
        lely::io::VirtualCanChannel chan(ctx, loop.get_executor());
        chan.open(*bus_);
        while (!stop_)
        {
            std::vector<Frame> out;
            {
                std::lock_guard<std::mutex> lock(out_mutex_);
                out.swap(out_);
            }
            std::error_code ec;
            for (const Frame &f : out)
            {
                can_msg msg = CAN_MSG_INIT;
                msg.id      = f.id;
                msg.len     = f.len;
                std::memcpy(msg.data, f.data, 8);
                chan.write(msg, 100, ec);
            }
            can_msg msg = CAN_MSG_INIT;
            if (chan.read(&msg, nullptr, nullptr, 1, ec) == 1)
            {
                Frame f{msg.id, msg.len, {}, Clock::now()};
                std::memcpy(f.data, msg.data, 8);
                std::lock_guard<std::mutex> lock(mutex_);
                frames_.push_back(f);
            }
        }
    }

    std::shared_ptr<lely::io::VirtualCanController> bus_;
    std::thread worker_;
    std::atomic<bool> stop_{false};
    std::mutex mutex_;
    std::vector<Frame> frames_;
    std::mutex out_mutex_;
    std::vector<Frame> out_;
};

constexpr uint32_t kNode  = 5;
constexpr uint32_t kSdoRx = 0x600 + kNode;
constexpr uint32_t kSdoTx = 0x580 + kNode;

uint32_t le32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// An expedited SDO upload: true with the value, false with the abort code in `value`.
bool sdo_read(Probe &p, uint16_t index, uint8_t sub, uint32_t &value, uint32_t node = kNode)
{
    size_t m = p.mark();
    p.send(0x600 + node,
           {0x40, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), sub, 0, 0, 0, 0});
    Frame f;
    if (!p.wait_id(m, 0x580 + node, milliseconds(500), &f))
    {
        value = 0xFFFFFFFF;
        return false;
    }
    value = le32(f.data + 4);
    return f.data[0] != 0x80;
}

// An expedited SDO download of `len` bytes: 0 or the abort code; 1 when nothing answered.
uint32_t sdo_write(Probe &p, uint16_t index, uint8_t sub, uint32_t value, int len)
{
    size_t m        = p.mark();
    uint8_t command = static_cast<uint8_t>(0x23 | ((4 - len) << 2));
    p.send(kSdoRx, {command, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), sub,
                    static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
                    static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)});
    Frame f;
    if (!p.wait_id(m, kSdoTx, milliseconds(500), &f))
        return 1;
    return f.data[0] == 0x60 ? 0 : le32(f.data + 4);
}

// A segmented SDO upload of a string.
bool sdo_read_string(Probe &p, uint16_t index, uint8_t sub, std::string &text)
{
    size_t m = p.mark();
    p.send(kSdoRx,
           {0x40, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), sub, 0, 0, 0, 0});
    Frame f;
    if (!p.wait_id(m, kSdoTx, milliseconds(500), &f))
        return false;
    if ((f.data[0] & 0xE0) != 0x40)
        return false;
    if (f.data[0] & 0x02) // expedited
    {
        int n = (f.data[0] & 0x01) ? 4 - ((f.data[0] >> 2) & 3) : 4;
        text.assign(reinterpret_cast<const char *>(f.data + 4), n);
        return true;
    }
    uint8_t toggle = 0;
    text.clear();
    for (int seg = 0; seg < 40; ++seg)
    {
        m = p.mark();
        p.send(kSdoRx, {static_cast<uint8_t>(0x60 | toggle), 0, 0, 0, 0, 0, 0, 0});
        if (!p.wait_id(m, kSdoTx, milliseconds(500), &f))
            return false;
        int n = 7 - ((f.data[0] >> 1) & 7);
        text.append(reinterpret_cast<const char *>(f.data + 1), n);
        if (f.data[0] & 0x01)
            return true;
        toggle ^= 0x10;
    }
    return false;
}

// Runs scans every 2 ms until `done` or the timeout.
bool scan_until(Slave &s, const plugin_runtime_args_t &rt, std::function<bool()> done,
                milliseconds timeout)
{
    auto end = Clock::now() + timeout;
    while (Clock::now() < end)
    {
        s.copy_to_plc(rt);
        if (done())
            return true;
        s.copy_from_plc(rt);
        std::this_thread::sleep_for(milliseconds(2));
    }
    return false;
}

// Waits for the slave's status (refreshed every 100 ms) to satisfy `ok`.
bool snapshot_until(Slave &s, std::function<bool(const SlaveSnapshot &)> ok, milliseconds timeout)
{
    auto end = Clock::now() + timeout;
    for (;;)
    {
        if (ok(s.snapshot()))
            return true;
        if (Clock::now() >= end)
            return false;
        std::this_thread::sleep_for(milliseconds(10));
    }
}

const SlaveEvent *find_event(const SlaveSnapshot &s, std::function<bool(const SlaveEvent &)> ok)
{
    for (const SlaveEvent &e : s.events)
        if (ok(e))
            return &e;
    return nullptr;
}

const SlaveValue *find_value(const SlaveSnapshot &s, uint16_t index, uint8_t sub)
{
    for (const SlaveValue &v : s.values)
        if (v.index == index && v.subindex == sub)
            return &v;
    return nullptr;
}

bool is_emcy(const Frame &f, uint16_t code)
{
    return f.id == 0x80 + kNode && f.len == 8 && (f.data[0] | (f.data[1] << 8)) == code;
}

void check_raw(const DeviceConfig &dev, const plugin_runtime_args_t &rt)
{
    auto vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    Probe p(vbus);
    size_t m0 = p.mark();
    Slave slave(dev, virtual_slave_link(vbus));
    std::string problem;
    CHECK(slave.start(problem));

    // Boot-up, then the heartbeat in pre-operational.
    Frame f;
    // LSS is off: no LSS answer.
    {
        size_t lm = p.mark();
        p.send(0x7E5, {0x04, 1, 0, 0, 0, 0, 0, 0});
        p.send(0x7E5, {0x5E, 0, 0, 0, 0, 0, 0, 0});
        CHECK(!p.wait_id(lm, 0x7E4, milliseconds(200)));
    }
    CHECK(p.wait(
        m0, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x00; }, milliseconds(1000)));
    CHECK(p.wait(
        m0, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x7F; }, milliseconds(1000)));
    // Defaults reach the PLC.
    CHECK(scan_until(slave, rt, [] { return g_int_in[2] == 1000; }, milliseconds(500)));

    // Identity and objects that do not exist.
    uint32_t v = 0;
    CHECK(sdo_read(p, 0x1018, 1, v) && v == 4660);
    CHECK(sdo_read(p, 0x1018, 3, v) && v == 65536);
    CHECK(sdo_read(p, 0x1000, 0, v) && v == 0);
    CHECK(!sdo_read(p, 0x1010, 1, v) && v == 0x06020000);
    CHECK(sdo_write(p, 0x1010, 1, 0x65766173, 4) == 0x06020000);
    CHECK(sdo_write(p, 0x1011, 1, 0x64616F6C, 4) == 0x06020000);
    std::string text;
    CHECK(sdo_read_string(p, 0x1008, 0, text) && text == "test-node");
    CHECK(sdo_read_string(p, 0x1009, 0, text) && text == "rev B");
    CHECK(sdo_read_string(p, 0x2100, 5, text) && text == "1.4.0");

    // Range checks and read-only objects.
    CHECK(sdo_write(p, 0x2000, 4, 1200, 2) == 0);
    CHECK(scan_until(slave, rt, [] { return g_int_in[2] == 1200; }, milliseconds(500)));
    CHECK(sdo_write(p, 0x2000, 4, 40000, 2) == 0x06090031);
    CHECK(sdo_write(p, 0x2000, 4, 50, 2) == 0x06090032);
    CHECK(sdo_read(p, 0x2000, 4, v) && v == 1200);
    CHECK(sdo_write(p, 0x2100, 1, 7, 2) == 0x06010002);
    CHECK(sdo_write(p, 0x2100, 5, 0x31, 1) == 0x06010002);
    // The writes in the status: events with the result, and the value now in the dictionary.
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            const SlaveValue *v = find_value(s, 0x2000, 4);
            return s.detailed && v && v->bytes == 2 && v->data[0] == 0xB0 && v->data[1] == 0x04;
        },
        milliseconds(500)));
    {
        SlaveSnapshot snap = slave.snapshot();
        CHECK(find_event(snap,
                         [](const SlaveEvent &e) {
                             return e.kind == SlaveEventKind::SdoWrite && e.index == 0x2000 &&
                                    e.subindex == 4 && e.ok;
                         }));
        CHECK(find_event(snap,
                         [](const SlaveEvent &e)
                         {
                             return e.kind == SlaveEventKind::SdoWrite && e.index == 0x2000 &&
                                    e.subindex == 4 && !e.ok && e.abort_code == 0x06090031;
                         }));
        CHECK(find_event(snap,
                         [](const SlaveEvent &e)
                         {
                             return e.kind == SlaveEventKind::SdoWrite && e.index == 0x1010 &&
                                    !e.ok && e.abort_code == 0x06020000;
                         }));
        CHECK(
            find_event(snap, [](const SlaveEvent &e) { return e.kind == SlaveEventKind::OnBus; }));
        CHECK(snap.nmt == 127 && snap.nmt_by == SlaveNmtBy::Boot);
        CHECK(snap.heartbeat_ms == 100 && snap.master_watch_ms == 300 && snap.on_master_lost == 0);
        CHECK(snap.pdos.size() == 32 && snap.pdos[0].cob_id == 0x205 && !snap.pdos[0].tpdo);
        CHECK(snap.pdos[16].tpdo && snap.pdos[16].cob_id == 0x185 &&
              snap.pdos[16].mapping.size() == 2);
        CHECK(snap.pdos[17].transmission == 1);
        CHECK(snap.bus.state == BusState::ErrorActive);
        CHECK(!find_value(snap, 0x2100, 5)); // the string constant is left out
        // The fields in the JSON answer.
        std::string json = inactive_status_json({snap});
        CHECK(json.find("\"nmt_by\":\"boot\"") != std::string::npos);
        CHECK(json.find("\"kind\":\"sdo_write\"") != std::string::npos);
        CHECK(json.find("\"data\":\"b004\"") != std::string::npos);
        CHECK(json.find("\"now\":") != std::string::npos);
    }

    // An SDO-only object to the master is read with the PLC's value.
    g_dint_out[0] = 123456;
    CHECK(scan_until(
        slave, rt, [&] { return sdo_read(p, 0x2100, 4, v) && v == 123456; }, milliseconds(1000)));

    // No PDO in pre-operational.
    size_t m = p.mark();
    p.send(0x205, {0x34, 0x12, 0x09});
    std::this_thread::sleep_for(milliseconds(50));
    slave.copy_to_plc(rt);
    CHECK(g_int_in[0] == 0);

    // Started by the master: TPDOs on change, RPDOs into %I.
    m            = p.mark();
    g_int_out[0] = 0x0011;
    g_int_out[1] = static_cast<IEC_UINT>(-5);
    p.send(0x000, {0x01, kNode});
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x05; }, milliseconds(1000)));
    CHECK(scan_until(
        slave, rt,
        [&]
        {
            return p.wait(
                m, [](const Frame &x)
                { return x.id == 0x185 && x.data[0] == 0x11 && x.data[2] == 0xFB; },
                milliseconds(0));
        },
        milliseconds(1000)));
    m            = p.mark();
    g_int_out[0] = 0x0022;
    CHECK(scan_until(
        slave, rt,
        [&]
        {
            return p.wait(
                m, [](const Frame &x) { return x.id == 0x185 && x.data[0] == 0x22; },
                milliseconds(0));
        },
        milliseconds(1000)));
    p.send(0x205, {0x34, 0x12, 0x09});
    CHECK(scan_until(
        slave, rt, [] { return g_int_in[0] == 0x1234 && g_byte_in[2] == 9; }, milliseconds(1000)));
    // Started by the master, and the PDOs counted.
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            return s.nmt == 5 && s.nmt_by == SlaveNmtBy::Master && s.since > 0 &&
                   s.pdos.size() == 32 && s.pdos[0].frames >= 1 && s.pdos[0].last > 0 &&
                   s.pdos[16].frames >= 2;
        },
        milliseconds(500)));
    CHECK(find_event(
        slave.snapshot(), [](const SlaveEvent &e)
        { return e.kind == SlaveEventKind::Nmt && e.state == 5 && e.by == SlaveNmtBy::Master; }));

    // A SYNC TPDO.
    g_byte_out[0] = 77;
    scan_until(slave, rt, [] { return false; }, milliseconds(20));
    m = p.mark();
    p.send(0x080, {});
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x285 && x.data[0] == 77; }, milliseconds(500)));

    // Remapping by the CiA 301 procedure, in pre-operational.
    p.send(0x000, {0x80, kNode});
    std::this_thread::sleep_for(milliseconds(30));
    CHECK(sdo_write(p, 0x1800, 1, 0x80000185, 4) == 0);
    CHECK(sdo_write(p, 0x1A00, 0, 0, 1) == 0);
    CHECK(sdo_write(p, 0x1A00, 1, 0x21000210, 4) == 0);
    CHECK(sdo_write(p, 0x1A00, 2, 0x21000110, 4) == 0);
    CHECK(sdo_write(p, 0x1A00, 3, 0x21000420, 4) == 0x06040041); // SDO-only object
    CHECK(sdo_write(p, 0x1A00, 0, 2, 1) == 0);
    CHECK(sdo_write(p, 0x1800, 1, 0x185, 4) == 0);
    m = p.mark();
    p.send(0x000, {0x01, kNode});
    g_int_out[1] = 0x0303;
    CHECK(scan_until(
        slave, rt,
        [&]
        {
            return p.wait(
                m, [](const Frame &x)
                { return x.id == 0x185 && x.data[0] == 0x03 && x.data[2] == 0x22; },
                milliseconds(0));
        },
        milliseconds(1000)));

    // The remap in the status.
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            return s.pdos.size() == 32 && s.pdos[16].mapping.size() == 2 &&
                   s.pdos[16].mapping[0] == 0x21000210 && s.pdos[16].mapping[1] == 0x21000110;
        },
        milliseconds(500)));

    // PLC stop and start.
    m = p.mark();
    slave.set_plc_running(false);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x6200); }, milliseconds(500)));
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            return !s.plc_running && s.emcy_active.size() == 1 && s.emcy_active[0] == 0x6200 &&
                   s.error_register == 0x81 && !s.error_history.empty() &&
                   (s.error_history[0] & 0xFFFF) == 0x6200;
        },
        milliseconds(500)));
    CHECK(find_event(slave.snapshot(),
                     [](const SlaveEvent &e) { return e.kind == SlaveEventKind::Plc && !e.ok; }));
    CHECK(find_event(slave.snapshot(), [](const SlaveEvent &e)
                     { return e.kind == SlaveEventKind::Emcy && e.code == 0x6200; }));
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x05; }, milliseconds(500)));
    m = p.mark();
    slave.set_plc_running(true);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x0000); }, milliseconds(500)));

    // Master watch: lost, then back.
    std::atomic<bool> beat{true};
    std::thread master(
        [&]
        {
            while (beat)
            {
                p.send(0x77F, {0x05});
                std::this_thread::sleep_for(milliseconds(50));
            }
        });
    std::this_thread::sleep_for(milliseconds(300));
    CHECK(slave.snapshot().master_seen);
    m    = p.mark();
    beat = false;
    master.join();
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x8130); }, milliseconds(1000)));
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x7F; }, milliseconds(1000)));
    CHECK(g_int_in[0] == 0x1234); // values from the master hold
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            return s.nmt == 127 && s.nmt_by == SlaveNmtBy::Self && s.master_last > 0 &&
                   std::find(s.emcy_active.begin(), s.emcy_active.end(), 0x8130) !=
                       s.emcy_active.end();
        },
        milliseconds(500)));
    CHECK(find_event(slave.snapshot(), [](const SlaveEvent &e)
                     { return e.kind == SlaveEventKind::Heartbeat && !e.ok; }));
    const int seen_before = g_master_seen_lines.load();
    m                     = p.mark();
    beat                  = true;
    std::thread again(
        [&]
        {
            while (beat)
            {
                p.send(0x77F, {0x05});
                std::this_thread::sleep_for(milliseconds(50));
            }
        });
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x0000); }, milliseconds(1000)));
    std::this_thread::sleep_for(milliseconds(200));
    CHECK(p.count(m, 0x705) > 0);
    // The error history holds the errors, not the 0000h "error reset" messages.
    {
        uint32_t n = 0;
        CHECK(sdo_read(p, 0x1003, 0, n) && n >= 2);
        uint32_t newest = 0;
        CHECK(sdo_read(p, 0x1003, 1, newest) && (newest & 0xFFFF) == 0x8130);
        for (uint8_t s = 1; s <= n; ++s)
        {
            uint32_t e = 0;
            CHECK(sdo_read(p, 0x1003, s, e) && (e & 0xFFFF) != 0);
        }
    }
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x7F; },
        milliseconds(300))); // stays pre-operational until started
    // A master that resets the slave's communication right after its return is logged once.
    m = p.mark();
    p.send(0x000, {0x82, 0x05});
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x00; }, milliseconds(500)));
    std::this_thread::sleep_for(milliseconds(300));
    CHECK(slave.snapshot().master_seen);
    CHECK(g_master_seen_lines.load() - seen_before == 1);
    CHECK(find_event(slave.snapshot(), [](const SlaveEvent &e)
                     { return e.kind == SlaveEventKind::Reset && !e.node; }));
    beat = false;
    again.join();

    // Status.
    SlaveSnapshot snap = slave.snapshot();
    CHECK(snap.bus_open && snap.node_id == kNode && snap.emcy_sent);

    // Stopped: no SDO answer.
    p.send(0x000, {0x02, kNode});
    std::this_thread::sleep_for(milliseconds(30));
    CHECK(sdo_write(p, 0x2000, 4, 2000, 2) == 1);

    // Reset node: boot-up and the defaults again.
    m = p.mark();
    p.send(0x000, {0x81, kNode});
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x00; }, milliseconds(1000)));
    CHECK(sdo_read(p, 0x2000, 4, v) && v == 1000);
    CHECK(sdo_read(p, 0x1A00, 0, v) && v == 2); // the default layout again
    CHECK(sdo_read(p, 0x1A00, 1, v) && v == 0x21000110);

    slave.stop();
    CHECK(!slave.snapshot().bus_open);
}

// A CAN link whose state the test sets: frames go over Lely's in-process bus.
class FakeCanLink : public SlaveLink
{
  public:
    explicit FakeCanLink(std::shared_ptr<lely::io::VirtualCanController> bus)
        : inner_(virtual_slave_link(bus))
    {
    }
    bool open(std::string &problem) override
    {
        ++opens;
        return inner_->open(problem);
    }
    void close() override
    {
        inner_->close();
    }
    int read(Frame &frame, int timeout_ms) override
    {
        return inner_->read(frame, timeout_ms);
    }
    int write(const Frame &frame) override
    {
        return inner_->write(frame);
    }
    bool info(LinkInfo &out) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        out = info_;
        return true;
    }
    int take_down() override
    {
        ++downs;
        set_state(0, 0, 0); // the reset clears the controller
        return 0;
    }
    void set_state(unsigned can_state, unsigned tx_errors, unsigned rx_errors)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        info_.up            = true;
        info_.kind          = "can";
        info_.restart_ms    = 0; // no auto-restart: the plugin resets the link
        info_.has_can_state = true;
        info_.can_state     = can_state;
        info_.has_berr      = true;
        info_.tx_errors     = tx_errors;
        info_.rx_errors     = rx_errors;
    }
    std::atomic<int> opens{0};
    std::atomic<int> downs{0};

  private:
    std::unique_ptr<SlaveLink> inner_;
    std::mutex mutex_;
    LinkInfo info_;
};

// Bus status, and a bus-off link without auto-restart reset by the plugin, as a network's.
void check_bus_off(const DeviceConfig &dev)
{
    auto vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    Probe p(vbus);
    std::unique_ptr<FakeCanLink> owned(new FakeCanLink(vbus));
    FakeCanLink *link = owned.get();
    link->set_state(0, 0, 0);
    size_t m = p.mark();
    Slave slave(dev, std::move(owned));
    std::string problem;
    CHECK(slave.start(problem));
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x7F; }, milliseconds(1000)));

    // Error passive: in the status and the events, and the stack's EMCY 8120h.
    m = p.mark();
    link->set_state(2, 130, 0);
    CHECK(snapshot_until(
        slave, [](const SlaveSnapshot &s)
        { return s.bus.state == BusState::ErrorPassive && s.bus.tx_errors == 130; },
        milliseconds(2000)));
    CHECK(find_event(
        slave.snapshot(), [](const SlaveEvent &e)
        { return e.kind == SlaveEventKind::BusState && e.bus_state == BusState::ErrorPassive; }));
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x8120); }, milliseconds(1000)));

    // Bus-off: the link is reset and reopened, the stack keeps its state.
    m               = p.mark();
    const int opens = link->opens.load();
    link->set_state(3, 256, 0);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x8140); }, milliseconds(2000)));
    CHECK(snapshot_until(
        slave, [&](const SlaveSnapshot &) { return link->downs.load() == 1; }, milliseconds(3000)));
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            return s.bus.state == BusState::ErrorActive && s.bus.has_resets && s.bus.resets == 1 &&
                   s.bus_open;
        },
        milliseconds(3000)));
    CHECK(link->opens.load() == opens + 1);
    CHECK(find_event(
        slave.snapshot(), [](const SlaveEvent &e)
        { return e.kind == SlaveEventKind::BusReset && e.outcome == ResetOutcome::Done; }));
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x0000); }, milliseconds(2000)));
    CHECK(p.count(m, 0x705) > 0); // heartbeat goes on
    CHECK(!p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x00; },
        milliseconds(0))); // no new boot-up: the stack was kept
    slave.stop();
}

// Our master boots the slave and exchanges PDOs with it.
const char *const kMasterConfig = R"({
  "version": 12,
  "networks": [{
    "name": "line", "interface": "vcan-master", "bitrate": 0, "master_node_id": 127,
    "heartbeat_producer_ms": 100,
    "nodes": [{
      "node_id": 5, "name": "io", "start": "master", "enabled": true,
      "identity": {"vendor_id": 4660, "product_code": 1, "revision": 65536},
      "supervision": {"type": "heartbeat", "consumer_ms": 400},
      "sdo": [{"index": 8192, "subindex": 4, "data": "b004"}],
      "tpdos": [{"number": 1, "cob_id": 389, "entries": [
        {"index": 8448, "subindex": 1, "type": "UNSIGNED16", "location": "%IW4"},
        {"index": 8448, "subindex": 2, "type": "INTEGER16", "location": "%IW5"}]}],
      "rpdos": [{"number": 1, "cob_id": 517, "entries": [
        {"index": 8192, "subindex": 1, "type": "INTEGER16", "location": "%QW4"},
        {"index": 8192, "subindex": 2, "type": "UNSIGNED8", "location": "%QB4"}]}]
    }]
  }]
})";

void check_with_master(const DeviceConfig &dev, const plugin_runtime_args_t &rt,
                       const std::string &dir)
{
    Config mcfg;
    std::vector<std::string> errors;
    CHECK(parse_config(kMasterConfig, mcfg, errors));
    for (const auto &e : errors)
        std::fprintf(stderr, "%s\n", e.c_str());
    if (!errors.empty())
        return;
    const NetworkConfig &net = mcfg.networks[0];
    MasterLayout layout      = master_layout(net);
    ProcessImage image;
    CHECK(image.build(net, layout, ImageSizes::uniform(kSize), errors));
    std::string dcf = dir + "/master.dcf";
    {
        FILE *f       = std::fopen(dcf.c_str(), "w");
        std::string t = master_dcf(net, layout);
        std::fwrite(t.data(), 1, t.size(), f);
        std::fclose(f);
    }
    auto vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    Slave slave(dev, virtual_slave_link(vbus));
    std::string problem;
    CHECK(slave.start(problem));
    NetworkStatus status(net);
    Bus bus(net, layout, image, status, dcf, vbus);
    bus.start();

    // Slave's PLC writes Status and Actual; the master's PLC writes Setpoint and Mode.
    g_int_out[0]  = 0x00AB;
    g_int_out[1]  = static_cast<IEC_UINT>(-42);
    g_int_out[4]  = 2500;
    g_byte_out[4] = 3;
    bool through  = false;
    auto end      = Clock::now() + milliseconds(5000);
    while (Clock::now() < end && !through)
    {
        slave.copy_to_plc(rt);
        image.copy_to_plc(rt);
        through = g_int_in[4] == 0x00AB && g_int_in[5] == static_cast<IEC_UINT>(-42) &&
                  g_int_in[0] == 2500 && g_byte_in[2] == 3;
        slave.copy_from_plc(rt);
        image.copy_from_plc(rt);
        std::this_thread::sleep_for(milliseconds(2));
    }
    CHECK(through);
    // The master's startup SDO set the parameter (0x04B0 = 1200).
    CHECK(g_int_in[2] == 1200);
    CHECK(slave.snapshot().nmt == 5);
    CHECK(slave.snapshot().master_seen);
    bus.stop();
    slave.stop();
}

// ---- LSS (CiA 305) on raw frames ----
constexpr uint32_t kLssMaster = 0x7E5;
constexpr uint32_t kLssSlave  = 0x7E4;

const char *const kLssConfig = R"({
  "version": 12,
  "networks": [],
  "devices": [{
    "name": "lss", "interface": "vcan-lss", "bitrate": 500000, "node_id": 5,
    "identity": {"device_name": "test-node", "vendor_id": 4660, "product_code": 1,
                 "revision": 65536},
    "heartbeat_ms": 100,
    "on_plc_stop_state": "none",
    "lss": {"enabled": true, "wait_for_master": true, "serial": 23},
    "objects": [
      {"index": 8448, "subindex": 1, "name": "Status", "type": "UNSIGNED16",
       "direction": "to_master", "exchange": "pdo", "location": "%QW0"}
    ],
    "rpdos": [],
    "tpdos": [{"number": 1, "cob_id": 389, "transmission": 255, "inhibit_100us": 0,
               "event_timer_ms": 0, "entries": [{"index": 8448, "subindex": 1}]}]
  }]
})";

// Sends an LSS request; with `answer`, waits for the slave's answer with the same command.
bool lss(Probe &p, std::vector<uint8_t> data, Frame *answer = nullptr,
         milliseconds timeout = milliseconds(200))
{
    data.resize(8, 0);
    const uint8_t cs = data[0];
    size_t m         = p.mark();
    p.send(kLssMaster, data);
    if (!answer)
        return true;
    return p.wait(
        m, [cs](const Frame &f) { return f.id == kLssSlave && (f.data[0] == cs || cs >= 0x40); },
        timeout, answer);
}

// One fastscan step: true when some slave answered.
bool fastscan(Probe &p, uint32_t id, uint8_t bit, uint8_t sub, uint8_t next)
{
    size_t m = p.mark();
    p.send(kLssMaster,
           {0x51, static_cast<uint8_t>(id), static_cast<uint8_t>(id >> 8),
            static_cast<uint8_t>(id >> 16), static_cast<uint8_t>(id >> 24), bit, sub, next});
    return p.wait(
        m, [](const Frame &f) { return f.id == kLssSlave && f.data[0] == 0x4F; }, milliseconds(30));
}

// The whole fastscan: the found LSS address, or false when no unconfigured slave answers.
bool fastscan_all(Probe &p, uint32_t (&address)[4])
{
    if (!fastscan(p, 0, 0x80, 0, 0))
        return false;
    for (uint8_t sub = 0; sub < 4; ++sub)
    {
        uint32_t id = 0;
        for (int bit = 31; bit >= 0; --bit)
            if (!fastscan(p, id, static_cast<uint8_t>(bit), sub, sub))
                id |= 1u << bit;
        address[sub] = id;
        if (!fastscan(p, id, 0, sub, static_cast<uint8_t>(sub < 3 ? sub + 1 : 0)))
            return false;
    }
    return true;
}

bool no_frame_in(Probe &p, uint32_t lo, uint32_t hi, milliseconds window)
{
    size_t m = p.mark();
    return !p.wait(m, [lo, hi](const Frame &f) { return f.id >= lo && f.id <= hi; }, window);
}

std::string read_file(const std::string &path)
{
    std::string text;
    if (FILE *f = std::fopen(path.c_str(), "r"))
    {
        char buf[256];
        size_t n = std::fread(buf, 1, sizeof(buf), f);
        text.assign(buf, n);
        std::fclose(f);
    }
    return text;
}

void check_lss_store(const std::string &dir)
{
    const std::string path = dir + "/sub/lss-x.json";
    LssStored in;
    in.serial  = 0xDEADBEEF;
    in.node_id = 12;
    in.bitrate = 250000;
    std::string problem;
    CHECK(lss_save(path, in, problem));
    LssStored out;
    CHECK(lss_load(path, out, problem));
    CHECK(out.serial == 0xDEADBEEF && out.node_id == 12 && out.bitrate == 250000);
    CHECK(!lss_load(dir + "/missing.json", out, problem) && problem.empty());
    FILE *f = std::fopen(path.c_str(), "w");
    std::fputs("{\"node_id\": 300}", f);
    std::fclose(f);
    CHECK(!lss_load(path, out, problem) && !problem.empty() && out.node_id == 0);
    CHECK(lss_random_serial() != 0);
    setenv("OPENPLC_PERSISTENT_DATA_DIR", dir.c_str(), 1);
    CHECK(lss_store_path("can1") == dir + "/canopen/lss-can1.json");
}

void check_lss(const plugin_runtime_args_t &rt, const std::string &dir)
{
    setenv("OPENPLC_PERSISTENT_DATA_DIR", dir.c_str(), 1);
    const std::string file = dir + "/canopen/lss-vcan-lss.json";
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(kLssConfig, cfg, errors));
    for (const auto &e : errors)
        std::fprintf(stderr, "%s\n", e.c_str());
    if (!errors.empty())
        return;
    const DeviceConfig &dev = cfg.devices[0];
    auto vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    Probe p(vbus);
    Frame f;
    uint32_t v = 0;
    {
        Slave slave(dev, virtual_slave_link(vbus));
        std::string problem;
        CHECK(slave.start(problem));
        // Unconfigured: silent apart from LSS.
        CHECK(no_frame_in(p, 0x080, 0x77F, milliseconds(300)));
        CHECK(slave.snapshot().unconfigured && slave.snapshot().serial == 23);
        // Fastscan finds it with its full address.
        uint32_t address[4] = {};
        CHECK(fastscan_all(p, address));
        CHECK(address[0] == 4660 && address[1] == 1 && address[2] == 65536 && address[3] == 23);
        // Selected by the fastscan, it is in configuration and no longer takes part.
        CHECK(!fastscan(p, 0, 0x80, 0, 0));
        CHECK(lss(p, {0x5E}, &f) && f.data[1] == 0xFF);
        // Node ID 12 and 250 kbit/s (table index 3), stored.
        CHECK(lss(p, {0x11, 12}, &f) && f.data[1] == 0);
        CHECK(lss(p, {0x11, 200}, &f) && f.data[1] == 1);
        CHECK(lss(p, {0x13, 0, 3}, &f) && f.data[1] == 0);
        CHECK(lss(p, {0x17}, &f) && f.data[1] == 0);
        const std::string text = read_file(file);
        CHECK(text.find("\"node_id\":12") != std::string::npos);
        CHECK(text.find("\"bitrate\":250000") != std::string::npos);
        // Back to waiting: the slave takes node ID 12 at once.
        size_t m = p.mark();
        lss(p, {0x04, 0});
        CHECK(p.wait(
            m, [](const Frame &x) { return x.id == 0x70C && x.data[0] == 0x00; },
            milliseconds(1000)));
        CHECK(sdo_read(p, 0x1018, 4, v, 12) && v == 23);
        CHECK(sdo_read(p, 0x1800, 1, v, 12) && (v & 0x7FF) == 0x18C);
        CHECK(sdo_read(p, 0x1014, 0, v, 12) && (v & 0x7FF) == 0x8C);
        CHECK(slave.snapshot().node_id == 12 && !slave.snapshot().unconfigured);
        // Its TPDO follows the new node ID.
        m = p.mark();
        p.send(0x000, {0x01, 12});
        g_int_out[0] = 0x1234;
        CHECK(scan_until(slave, rt, [&] { return p.count(m, 0x18C) > 0; }, milliseconds(1000)));
        slave.stop();
    }
    {
        // A restart uses the stored values; the link is asked for 250 kbit/s.
        Slave slave(dev, virtual_slave_link(vbus));
        std::string problem;
        size_t m = p.mark();
        CHECK(slave.start(problem));
        CHECK(p.wait(
            m, [](const Frame &x) { return x.id == 0x70C && x.data[0] == 0x00; },
            milliseconds(1000)));
        SlaveSnapshot s = slave.snapshot();
        CHECK(s.node_id == 12 && std::string(s.node_from) == "stored" && s.bitrate == 250000 &&
              std::string(s.rate_from) == "stored" && s.stored_node_id == 12);
        const std::string json = status_json({}, 0, {s});
        CHECK(json.find("\"node_id_from\":\"stored\"") != std::string::npos);
        // Selective switch, inquire, and a node ID change active after reset communication.
        CHECK(lss(p, {0x40, 0x34, 0x12}));
        CHECK(lss(p, {0x41, 1}));
        CHECK(lss(p, {0x42, 0, 0, 1}));
        CHECK(lss(p, {0x43, 23}, &f) && f.data[0] == 0x44);
        CHECK(lss(p, {0x5D}, &f) && le32(f.data + 1) == 23);
        CHECK(lss(p, {0x11, 13}, &f) && f.data[1] == 0);
        lss(p, {0x04, 0});
        m = p.mark();
        p.send(0x000, {0x82, 12});
        CHECK(p.wait(
            m, [](const Frame &x) { return x.id == 0x70D && x.data[0] == 0x00; },
            milliseconds(1000)));
        CHECK(std::string(slave.snapshot().node_from) == "lss");
        // Reset node: the unsaved 13 is gone, the stored 12 is back.
        m = p.mark();
        p.send(0x000, {0x81, 13});
        CHECK(p.wait(
            m, [](const Frame &x) { return x.id == 0x70C && x.data[0] == 0x00; },
            milliseconds(2000)));
        // Forget: back to the project, which waits for the master.
        CHECK(slave.forget_lss());
        CHECK(scan_until(
            slave, rt, [&] { return slave.snapshot().unconfigured; }, milliseconds(2000)));
        const std::string text = read_file(file);
        CHECK(text.find("node_id") == std::string::npos);
        CHECK(text.find("\"serial\":23") == std::string::npos);
        // The project's node ID 5 given by a master without a store still counts as from the master.
        CHECK(lss(p, {0x04, 1}));
        CHECK(lss(p, {0x11, 5}, &f) && f.data[1] == 0);
        m = p.mark();
        lss(p, {0x04, 0});
        CHECK(p.wait(
            m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x00; },
            milliseconds(1000)));
        CHECK(scan_until(
            slave, rt, [&] { return std::string(slave.snapshot().node_from) == "lss"; },
            milliseconds(1000)));
        slave.stop();
    }
    {
        // Bit rate change not allowed, and a store that cannot be written.
        const std::string blocked = dir + "/not-a-folder";
        FILE *f2                  = std::fopen(blocked.c_str(), "w");
        std::fclose(f2);
        setenv("OPENPLC_PERSISTENT_DATA_DIR", blocked.c_str(), 1);
        DeviceConfig fixed        = dev;
        fixed.lss.wait_for_master = false;
        fixed.lss.bitrate_change  = false;
        Slave slave(fixed, virtual_slave_link(vbus));
        std::string problem;
        size_t m = p.mark();
        CHECK(slave.start(problem));
        CHECK(p.wait(
            m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x00; },
            milliseconds(1000)));
        CHECK(lss(p, {0x04, 1}));
        CHECK(lss(p, {0x5E}, &f) && f.data[1] == 5);
        CHECK(lss(p, {0x13, 0, 3}, &f) && f.data[1] == 1);
        CHECK(lss(p, {0x17}, &f) && f.data[1] == 2);
        lss(p, {0x04, 0});
        CHECK(!fastscan(p, 0, 0x80, 0, 0)); // configured and waiting: no fastscan answer
        slave.stop();
        setenv("OPENPLC_PERSISTENT_DATA_DIR", dir.c_str(), 1);
    }
}

// The slave's side of the PLC blocks, as the scan calls it.
void check_program(const DeviceConfig &dev, const plugin_runtime_args_t &rt)
{
    auto vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    Probe p(vbus);
    size_t m = p.mark();
    Slave slave(dev, virtual_slave_link(vbus));
    std::string problem;
    CHECK(slave.start(problem));
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x7F; }, milliseconds(1000)));
    auto status_until = [&](std::function<bool(const canopen_plc_slave_status &)> ok)
    {
        auto end = Clock::now() + milliseconds(500);
        for (;;)
        {
            canopen_plc_slave_status st{};
            slave.plc_status(st);
            if (ok(st))
                return true;
            if (Clock::now() >= end)
                return false;
            std::this_thread::sleep_for(milliseconds(5));
        }
    };
    CHECK(status_until(
        [](const canopen_plc_slave_status &st)
        {
            return st.state == CANOPEN_PLC_PRE_OPERATIONAL && st.node_id == kNode &&
                   (st.flags & CANOPEN_PLC_SLAVE_ON_BUS) &&
                   (st.flags & CANOPEN_PLC_SLAVE_MASTER_WATCHED) &&
                   !(st.flags & CANOPEN_PLC_SLAVE_MASTER_SEEN) &&
                   st.bus_state == CANOPEN_PLC_BUS_ERROR_ACTIVE;
        }));

    // Local NMT: operational, pre-operational, refused resets.
    bool done = true;
    CHECK(slave.plc_nmt(0x81) == CANOPEN_PLC_WRONG_PARAMETER);
    CHECK(slave.plc_nmt(0x82) == CANOPEN_PLC_WRONG_PARAMETER);
    CHECK(slave.plc_nmt_done(0x01, done) == CANOPEN_PLC_NO_ERROR && !done);
    m = p.mark();
    CHECK(slave.plc_nmt(0x01) == CANOPEN_PLC_NO_ERROR);
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x05; }, milliseconds(500)));
    CHECK(status_until([](const canopen_plc_slave_status &st)
                       { return st.state == CANOPEN_PLC_OPERATIONAL; }));
    CHECK(slave.plc_nmt_done(0x01, done) == CANOPEN_PLC_NO_ERROR && done);
    const auto t0 = Clock::now();
    CHECK(slave.plc_nmt(0x80) == CANOPEN_PLC_NO_ERROR);
    CHECK(status_until([](const canopen_plc_slave_status &st)
                       { return st.state == CANOPEN_PLC_PRE_OPERATIONAL; }));
    CHECK(Clock::now() - t0 < milliseconds(50));
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            return s.nmt == 127 && s.nmt_by == SlaveNmtBy::Program &&
                   find_event(
                       s, [](const SlaveEvent &e)
                       { return e.kind == SlaveEventKind::Nmt && e.by == SlaveNmtBy::Program; });
        },
        milliseconds(500)));
    p.send(0x000, {0x01, kNode});
    CHECK(status_until([](const canopen_plc_slave_status &st)
                       { return st.state == CANOPEN_PLC_OPERATIONAL; }));

    // Program emergencies: a temperature alarm with its info, then a manufacturer one.
    uint32_t a = 0, b = 0;
    CHECK(slave.plc_emcy_claim(a) == CANOPEN_PLC_NO_ERROR);
    CHECK(slave.plc_emcy_claim(b) == CANOPEN_PLC_NO_ERROR && a != b);
    CHECK(slave.plc_emcy(a, true, 0x6200, 0) == CANOPEN_PLC_WRONG_PARAMETER);
    CHECK(slave.plc_emcy(a, true, 0x8130, 0) == CANOPEN_PLC_WRONG_PARAMETER);
    CHECK(slave.plc_emcy(a, true, 0x0000, 0) == CANOPEN_PLC_WRONG_PARAMETER);
    CHECK(slave.plc_emcy(0x12345678, true, 0x4210, 0) == CANOPEN_PLC_REQUEST_ERROR);
    Frame f;
    m = p.mark();
    CHECK(slave.plc_emcy(a, true, 0x4210, 865) == CANOPEN_PLC_NO_ERROR);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x4210); }, milliseconds(500), &f));
    CHECK(f.data[2] == 0x09 && f.data[3] == 0 && le32(f.data + 4) == 865);
    CHECK(slave.plc_emcy(a, true, 0x4210, 865) == CANOPEN_PLC_NO_ERROR); // unchanged: no frame
    std::this_thread::sleep_for(milliseconds(30));
    CHECK(p.count(m, 0x80 + kNode) == 1);
    m = p.mark();
    CHECK(slave.plc_emcy(a, true, 0x4210, 870) == CANOPEN_PLC_NO_ERROR); // new info: one EMCY
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x4210); }, milliseconds(500), &f));
    CHECK(le32(f.data + 4) == 870);
    std::this_thread::sleep_for(milliseconds(30));
    CHECK(p.count(m, 0x80 + kNode) == 1);
    m = p.mark();
    CHECK(slave.plc_emcy(b, true, 0xFF01, 0) == CANOPEN_PLC_NO_ERROR);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0xFF01); }, milliseconds(500), &f));
    CHECK(f.data[2] == 0x89);
    CHECK(status_until([](const canopen_plc_slave_status &st)
                       { return st.error_register == 0x89 && st.emcy_active == 2; }));
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            return s.emcy_program.size() == 2 &&
                   find_event(
                       s, [](const SlaveEvent &e)
                       { return e.kind == SlaveEventKind::Emcy && e.code == 0xFF01 && e.program; });
        },
        milliseconds(500)));
    m = p.mark();
    CHECK(slave.plc_emcy(a, false, 0, 0) == CANOPEN_PLC_NO_ERROR);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x0000); }, milliseconds(500), &f));
    CHECK(f.data[2] == 0x81);
    m = p.mark();
    CHECK(slave.plc_emcy(b, false, 0, 0) == CANOPEN_PLC_NO_ERROR);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x0000); }, milliseconds(500), &f));
    CHECK(f.data[2] == 0x00);

    // Write counters: SDO writes with the same value, RPDOs, unknown objects.
    uint32_t c0 = 0, c1 = 0;
    CHECK(slave.plc_written(0x2000, 9, c0) == CANOPEN_PLC_WRONG_PARAMETER);
    CHECK(slave.plc_written(0x2000, 4, c0) == CANOPEN_PLC_NO_ERROR);
    CHECK(sdo_write(p, 0x2000, 4, 500, 2) == 0);
    CHECK(sdo_write(p, 0x2000, 4, 500, 2) == 0);
    CHECK(sdo_write(p, 0x2000, 4, 50, 2) == 0x06090032); // refused writes do not count
    CHECK(slave.plc_written(0x2000, 4, c1) == CANOPEN_PLC_NO_ERROR && c1 == c0 + 2);
    CHECK(slave.plc_written(0x2000, 1, c0) == CANOPEN_PLC_NO_ERROR);
    m = p.mark();
    for (int i = 0; i < 3; ++i)
    {
        p.send(0x205, {0x10, 0x00, 0x01});
        std::this_thread::sleep_for(milliseconds(10));
    }
    std::this_thread::sleep_for(milliseconds(30));
    CHECK(slave.plc_written(0x2000, 1, c1) == CANOPEN_PLC_NO_ERROR && c1 == c0 + 3);
    CHECK(slave.plc_written(0x2100, 1, c1) == CANOPEN_PLC_NO_ERROR && c1 == 0); // to the master

    // PLC stop with an alarm active and 0x1029 sub 2 set to pre-operational by the master.
    uint32_t v = 0;
    CHECK(sdo_read(p, 0x1029, 0, v) && v == 2);
    CHECK(sdo_read(p, 0x1029, 2, v) && v == 1);
    CHECK(sdo_write(p, 0x1029, 2, 3, 1) == 0x06090030);
    CHECK(sdo_write(p, 0x1029, 2, 0, 1) == 0);
    CHECK(slave.plc_emcy(a, true, 0x4210, 1) == CANOPEN_PLC_NO_ERROR);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x4210); }, milliseconds(500)));
    std::this_thread::sleep_for(milliseconds(20));
    m = p.mark();
    slave.set_plc_running(false);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x6200); }, milliseconds(500)));
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x7F; }, milliseconds(500)));
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            return s.nmt == 127 && s.nmt_by == SlaveNmtBy::PlcStop && s.emcy_program.empty() &&
                   s.on_plc_stop_state == 0 && s.emcy_active.size() == 1 &&
                   s.emcy_active[0] == 0x6200;
        },
        milliseconds(500)));
    // A PLC start keeps the state; handles of the earlier run are refused.
    m = p.mark();
    slave.set_plc_running(true);
    CHECK(p.wait(m, [](const Frame &x) { return is_emcy(x, 0x0000); }, milliseconds(500)));
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x7F; }, milliseconds(300)));
    CHECK(slave.plc_emcy(a, true, 0x4210, 1) == CANOPEN_PLC_REQUEST_ERROR);
    uint32_t again = 0;
    CHECK(slave.plc_emcy_claim(again) == CANOPEN_PLC_NO_ERROR && again != a);
    // All slots in use.
    uint32_t h  = 0;
    int claimed = 1;
    while (slave.plc_emcy_claim(h) == CANOPEN_PLC_NO_ERROR)
        ++claimed;
    CHECK(claimed == static_cast<int>(CANOPEN_PLC_SLAVE_EMCY_SLOTS));
    (void)rt;

    slave.stop();
    canopen_plc_slave_status st{};
    slave.plc_status(st);
    CHECK(st.state == CANOPEN_PLC_NOT_AVAIL && !(st.flags & CANOPEN_PLC_SLAVE_ON_BUS));
}

// The PLC stop NMT state in the configuration.
void check_stop_state_config()
{
    Config cfg;
    std::vector<std::string> errors;
    std::string text        = kConfig;
    const std::string field = "\"on_plc_stop_state\": \"none\",";
    const size_t at         = text.find(field);
    CHECK(at != std::string::npos);
    std::string missing = text;
    missing.erase(at, field.size());
    CHECK(!parse_config(missing, cfg, errors));
    bool named = false;
    for (const auto &e : errors)
        named = named || e.find("missing \"on_plc_stop_state\"") != std::string::npos;
    CHECK(named);
    std::string wrong = text;
    wrong.replace(at, field.size(), "\"on_plc_stop_state\": \"reset\",");
    errors.clear();
    CHECK(!parse_config(wrong, cfg, errors));
    named = false;
    for (const auto &e : errors)
        named =
            named || e.find("is not one of none, pre_operational, stopped") != std::string::npos;
    CHECK(named);
    std::string stopped = text;
    stopped.replace(at, field.size(), "\"on_plc_stop_state\": \"stopped\",");
    Config ok;
    errors.clear();
    CHECK(parse_config(stopped, ok, errors) && ok.devices.size() == 1 &&
          ok.devices[0].on_plc_stop_state == SlaveStopState::Stopped);
}

} // namespace

const char *const kPackedConfig = R"({
  "version": 12,
  "networks": [],
  "devices": [{
    "name": "bits", "interface": "vcan-packed", "bitrate": 0, "node_id": 5,
    "identity": {"device_name": "bits", "vendor_id": 1, "product_code": 1, "revision": 1},
    "heartbeat_ms": 1000, "master_watch": null, "on_master_lost": "pre_operational",
    "on_plc_stop": "zero", "on_plc_stop_state": "none",
    "objects": [
      {"index": 8192, "subindex": 1, "name": "Valves", "type": "UNSIGNED8",
       "direction": "from_master", "exchange": "pdo", "default": 129,
       "bits": ["%IX0.0", "%IX0.1", "%IX0.2", "%IX0.3", "%IX0.4", "%IX0.5", "%IX0.6", "%IX0.7"]},
      {"index": 8448, "subindex": 1, "name": "Inputs", "type": "UNSIGNED8",
       "direction": "to_master", "exchange": "pdo",
       "bits": ["%QX1.0", "%QX1.1", "%QX1.2", "%QX1.3", "%QX1.4", "%QX1.5", "%QX1.6", "%QX1.7"]}
    ],
    "rpdos": [{"number": 1, "cob_id": 517, "entries": [{"index": 8192, "subindex": 1}]}],
    "tpdos": [{"number": 1, "cob_id": 389, "entries": [{"index": 8448, "subindex": 1}]}]
  }]
})";

std::string replaced(std::string text, const std::string &from, const std::string &to)
{
    size_t at = text.find(from);
    CHECK(at != std::string::npos);
    if (at != std::string::npos)
        text.replace(at, from.size(), to);
    return text;
}

bool packed_refused(const std::string &text, const std::string &part)
{
    Config cfg;
    std::vector<std::string> errors;
    bool ok = parse_config(text, cfg, errors);
    for (const auto &e : errors)
        if (e.find(part) != std::string::npos)
            return !ok;
    std::fprintf(stderr, "expected an error with \"%s\", got:\n", part.c_str());
    for (const auto &e : errors)
        std::fprintf(stderr, "  %s\n", e.c_str());
    return false;
}

void check_packed_config()
{
    const std::string base = kPackedConfig;
    const std::string bits = R"("%IX0.0", "%IX0.1", "%IX0.2", "%IX0.3", "%IX0.4", "%IX0.5", "%IX0.6", "%IX0.7")";
    CHECK(packed_refused(replaced(base, bits, R"("%IX0.0", "%IX0.1", "%IX0.2", "%IX0.3", "%IX0.4", "%IX0.5", "%IX0.6")"),
                         "\"bits\" must list 8 locations"));
    CHECK(packed_refused(replaced(base, "\"device_name\": \"bits\"",
                                  "\"device_name\": \"bits\", \"hardware_version\": \"" +
                                      std::string(300, 'h') + "\""),
                         "\"hardware_version\" is at most 255 characters"));
    CHECK(packed_refused(replaced(base, "\"device_name\": \"bits\"",
                                  "\"device_name\": \"bits\", \"hardware_version\": \"r\\u00e9v\""),
                         "\"hardware_version\" must be printable ASCII"));
    CHECK(packed_refused(replaced(base, "\"%IX0.7\"", "\"%QX2.0\""),
                         "bit location %QX2.0 must be a %IX location"));
    CHECK(packed_refused(replaced(base, "\"%IX0.7\"", "\"%IB3\""),
                         "bit location %IB3 must be a %IX location"));
    CHECK(packed_refused(replaced(base, "\"%IX0.7\"", "\"%IX0.0\""),
                         "bit location %IX0.0 is given twice"));
    CHECK(packed_refused(replaced(base, "\"name\": \"Valves\", \"type\": \"UNSIGNED8\"",
                                  "\"name\": \"Valves\", \"type\": \"UNSIGNED16\""),
                         "\"bits\" needs type UNSIGNED8, not UNSIGNED16"));
    CHECK(packed_refused(replaced(base, "\"default\": 129,", "\"default\": 129, \"location\": \"%IB0\","),
                         "an object with \"bits\" has no \"location\""));
    CHECK(packed_refused(replaced(base, "\"default\": 129,", "\"default\": 129, \"max\": 7,"),
                         "an object with \"bits\" has no \"min\" or \"max\""));
    CHECK(packed_refused(replaced(base, "\"exchange\": \"pdo\", \"default\": 129,",
                                  "\"exchange\": \"constant\", \"value\": 129,"),
                         "a constant has no \"bits\""));
    // A bit shared with another object.
    CHECK(packed_refused(
        replaced(base, "    ],\n    \"rpdos\"",
                 ",\n      {\"index\": 8192, \"subindex\": 2, \"name\": \"Run\", \"type\": \"UNSIGNED8\",\n"
                 "       \"direction\": \"from_master\", \"exchange\": \"sdo\",\n"
                 "       \"bits\": [\"%IX3.0\", \"%IX3.1\", \"%IX3.2\", \"%IX3.3\", \"%IX3.4\", \"%IX0.5\", \"%IX3.6\", \"%IX3.7\"]}\n"
                 "    ],\n    \"rpdos\""),
        "both use %IX0.5"));
}

void check_packed(const DeviceConfig &dev, const plugin_runtime_args_t &rt)
{
    std::memset(g_bool_in, 0, sizeof(g_bool_in));
    std::memset(g_bool_out, 0, sizeof(g_bool_out));
    auto vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    Probe p(vbus);
    size_t m0 = p.mark();
    Slave slave(dev, virtual_slave_link(vbus));
    std::string problem;
    CHECK(slave.start(problem));
    CHECK(p.wait(
        m0, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x00; }, milliseconds(1000)));

    // The default 0x81 reaches bits 0 and 7.
    CHECK(scan_until(
        slave, rt, [] { return g_bool_in[0][0] && g_bool_in[0][7] && !g_bool_in[0][1]; },
        milliseconds(500)));
    uint32_t v = 0;
    CHECK(sdo_read(p, 0x2000, 1, v) && v == 0x81);
    // No hardware version in the configuration: no 0x1009.
    CHECK(!sdo_read(p, 0x1009, 0, v) && v == 0x06020000);

    // Operational: RPDO 0x05 gives bits 0 and 2.
    p.send(0x000, {0x01, static_cast<uint8_t>(kNode)});
    CHECK(p.wait(
        m0, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x05; }, milliseconds(1000)));
    p.send(0x205, {0x05});
    CHECK(scan_until(
        slave, rt,
        []
        {
            for (int b = 0; b < 8; ++b)
                if (g_bool_in[0][b] != (b == 0 || b == 2))
                    return false;
            return true;
        },
        milliseconds(500)));
    CHECK(sdo_read(p, 0x2000, 1, v) && v == 0x05);

    // Bits 0, 1 and 3 of the program form 0x0B in TPDO 1 and by SDO.
    g_bool_out[1][0] = 1;
    g_bool_out[1][1] = 1;
    g_bool_out[1][3] = 1;
    size_t m = p.mark();
    Frame f;
    CHECK(scan_until(
        slave, rt,
        [&] { return p.wait(m, [](const Frame &x) { return x.id == 0x185 && x.data[0] == 0x0B; },
                            milliseconds(0), &f); },
        milliseconds(1000)));
    CHECK(f.len == 1);
    CHECK(sdo_read(p, 0x2100, 1, v) && v == 0x0B);

    // The byte in the status.
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            const SlaveValue *x = find_value(s, 0x2100, 1);
            return x && x->bytes == 1 && x->data[0] == 0x0B;
        },
        milliseconds(500)));

    // PLC stop with zero: the byte to the master reads 0.
    slave.set_plc_running(false);
    auto end = Clock::now() + milliseconds(1000);
    bool zero = false;
    while (!zero && Clock::now() < end)
    {
        zero = sdo_read(p, 0x2100, 1, v) && v == 0;
        std::this_thread::sleep_for(milliseconds(20));
    }
    CHECK(zero);
    slave.stop();
}

const char *const kMorePdosConfig = R"({
  "version": 12,
  "networks": [],
  "devices": [{
    "name": "many", "interface": "vcan-pdos", "bitrate": 0, "node_id": 5,
    "identity": {"device_name": "many", "vendor_id": 1, "product_code": 1, "revision": 1},
    "heartbeat_ms": 100, "master_watch": null, "on_master_lost": "pre_operational",
    "on_plc_stop": "hold", "on_plc_stop_state": "none",
    "objects": [
      {"index": 8192, "subindex": 1, "name": "Setpoint", "type": "INTEGER16",
       "direction": "from_master", "exchange": "pdo", "location": "%IW0", "default": 0},
      {"index": 8448, "subindex": 1, "name": "Counter", "type": "UNSIGNED8",
       "direction": "to_master", "exchange": "pdo", "location": "%QB0"},
      {"index": 8448, "subindex": 2, "name": "Fast", "type": "UNSIGNED16",
       "direction": "to_master", "exchange": "pdo", "location": "%QW0"}
    ],
    "rpdos": [{"number": 1, "cob_id": 517, "event_timer_ms": 200,
               "entries": [{"index": 8192, "subindex": 1}]}],
    "tpdos": [{"number": 5, "event_timer_ms": 50, "entries": [{"index": 8448, "subindex": 1}]},
              {"number": 6, "cob_id": 464, "event_timer_ms": 50,
               "entries": [{"index": 8448, "subindex": 2}]}]
  }]
})";

void check_more_pdos_config()
{
    const std::string base = kMorePdosConfig;
    CHECK(packed_refused(replaced(base, "\"number\": 6,", "\"number\": 17,"),
                         "\"number\" must be an integer from 1 to 16"));
    CHECK(packed_refused(replaced(base, "\"number\": 1, \"cob_id\": 517,", "\"number\": 1,"),
                         "RPDO 1: missing \"cob_id\""));
    CHECK(packed_refused(replaced(base, "\"number\": 5,", "\"number\": 5, \"cob_id\": 464,"),
                         "use the same COB-ID 0x1D0"));
    CHECK(packed_refused(replaced(base, "\"number\": 5,", "\"number\": 5, \"cob_id\": 0,"),
                         "COB-ID 0x000 is reserved for NMT"));
}

void check_more_pdos(const DeviceConfig &dev, const plugin_runtime_args_t &rt)
{
    std::memset(g_int_in, 0, sizeof(g_int_in));
    std::memset(g_byte_out, 0, sizeof(g_byte_out));
    std::memset(g_int_out, 0, sizeof(g_int_out));
    auto vbus = std::make_shared<lely::io::VirtualCanController>(lely::io::clock_monotonic);
    Probe p(vbus);
    size_t m0 = p.mark();
    Slave slave(dev, virtual_slave_link(vbus));
    std::string problem;
    CHECK(slave.start(problem));
    CHECK(p.wait(
        m0, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x00; }, milliseconds(1000)));

    // Sixteen PDOs each way; the unused ones and TPDO 5 without a COB-ID are off.
    uint32_t v = 0;
    CHECK(sdo_read(p, 0x140F, 1, v) && (v & 0x80000000u));
    CHECK(sdo_read(p, 0x180F, 1, v) && (v & 0x80000000u));
    CHECK(sdo_read(p, 0x1A0F, 0, v) && v == 0);
    CHECK(sdo_read(p, 0x1804, 1, v) && v == 0x80000000u);
    CHECK(sdo_read(p, 0x1A04, 0, v) && v == 1);
    CHECK(sdo_read(p, 0x1A04, 1, v) && v == 0x21000108u);
    CHECK(sdo_read(p, 0x1805, 1, v) && v == 0x1D0);
    uint32_t abort_code = 0;
    CHECK(!sdo_read(p, 0x1810, 1, abort_code) && abort_code == 0x06020000u);

    // Operational: the fixed TPDO 6 runs, TPDO 5 stays silent.
    g_byte_out[0] = 0x42;
    g_int_out[0]  = 0x1234;
    p.send(0x000, {0x01, static_cast<uint8_t>(kNode)});
    size_t m = p.mark();
    Frame f;
    CHECK(scan_until(
        slave, rt, [&] { return p.wait_id(m, 0x1D0, milliseconds(0), &f); }, milliseconds(1000)));
    CHECK(f.len == 2 && f.data[0] == 0x34 && f.data[1] == 0x12);
    CHECK(!scan_until(slave, rt, [] { return false; }, milliseconds(200)));
    for (uint32_t id = 0x181; id < 0x200; ++id)
        if (id != 0x1D0 && p.count(m, id) != 0)
        {
            std::fprintf(stderr, "unexpected frame 0x%X\n", static_cast<unsigned>(id));
            CHECK(false);
        }

    // A master gives TPDO 5 a COB-ID in pre-operational and starts it again.
    p.send(0x000, {0x80, static_cast<uint8_t>(kNode)});
    CHECK(p.wait(
        m, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x7F; }, milliseconds(1000)));
    CHECK(sdo_write(p, 0x1804, 1, 0x1C2, 4) == 0);
    p.send(0x000, {0x01, static_cast<uint8_t>(kNode)});
    m = p.mark();
    CHECK(scan_until(
        slave, rt, [&] { return p.wait_id(m, 0x1C2, milliseconds(0), &f); }, milliseconds(1000)));
    CHECK(f.len == 1 && f.data[0] == 0x42);
    CHECK(snapshot_until(
        slave,
        [](const SlaveSnapshot &s)
        {
            if (s.pdos.size() != 32)
                return false;
            const SlavePdoStatus &t5 = s.pdos[16 + 4];
            return t5.tpdo && t5.number == 5 && t5.cob_id == 0x1C2 && t5.frames > 0;
        },
        milliseconds(1000)));

    // RPDO 1 receive timeout: EMCY 8250h after the frames stop, value held, still operational.
    m = p.mark();
    p.send(0x205, {0xDC, 0x05});
    CHECK(scan_until(slave, rt, [] { return g_int_in[0] == 1500; }, milliseconds(500)));
    CHECK(scan_until(
        slave, rt, [&] { return p.wait(m, [](const Frame &x) { return is_emcy(x, 0x8250); },
                                       milliseconds(0)); },
        milliseconds(1000)));
    CHECK(g_int_in[0] == 1500);
    size_t after = p.mark();
    CHECK(p.wait(
        after, [](const Frame &x) { return x.id == 0x705 && x.data[0] == 0x05; },
        milliseconds(500)));
    p.send(0x205, {0xDD, 0x05});
    CHECK(scan_until(
        slave, rt, [&] { return p.wait(after, [](const Frame &x) { return is_emcy(x, 0x0000); },
                                       milliseconds(0)); },
        milliseconds(1000)));
    CHECK(scan_until(slave, rt, [] { return g_int_in[0] == 1501; }, milliseconds(500)));
    slave.stop();
}

int main()
{
    set_log_sink(quiet_sink);
    route_lely_diagnostics();
    prepare_wake_signal();
    const std::string dir = "/tmp/canopen_slave_test_" + std::to_string(getpid());
    CHECK(system(("mkdir -p " + dir).c_str()) == 0);
    setenv("OPENPLC_PERSISTENT_DATA_DIR", dir.c_str(), 1);

    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(kConfig, cfg, errors));
    for (const auto &e : errors)
        std::fprintf(stderr, "%s\n", e.c_str());
    if (!errors.empty())
        return TEST_RESULT();
    plugin_runtime_args_t rt = fake_args();
    check_raw(cfg.devices[0], rt);
    std::memset(g_int_in, 0, sizeof(g_int_in));
    std::memset(g_byte_in, 0, sizeof(g_byte_in));
    check_with_master(cfg.devices[0], rt, dir);
    check_bus_off(cfg.devices[0]);
    check_lss_store(dir);
    check_lss(rt, dir);
    check_program(cfg.devices[0], rt);
    check_stop_state_config();
    check_packed_config();
    {
        Config pcfg;
        std::vector<std::string> perrors;
        CHECK(parse_config(kPackedConfig, pcfg, perrors));
        for (const auto &e : perrors)
            std::fprintf(stderr, "%s\n", e.c_str());
        if (perrors.empty())
            check_packed(pcfg.devices[0], rt);
    }
    check_more_pdos_config();
    {
        Config mcfg;
        std::vector<std::string> merrors;
        CHECK(parse_config(kMorePdosConfig, mcfg, merrors));
        for (const auto &e : merrors)
            std::fprintf(stderr, "%s\n", e.c_str());
        if (merrors.empty())
            check_more_pdos(mcfg.devices[0], rt);
    }
    CHECK(system(("rm -rf " + dir).c_str()) == 0);
    return TEST_RESULT();
}
