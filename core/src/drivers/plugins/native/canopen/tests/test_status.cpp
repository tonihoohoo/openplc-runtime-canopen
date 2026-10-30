// SPDX-License-Identifier: Apache-2.0
//
// The status record without a bus: node states and failure reasons, emergencies and their limit,
// late PDOs, the event ring, the JSON answer, the status requests, rtnetlink link parsing and the
// bus monitor.

#include <chrono>
#include <cstring>
#include <linux/can/netlink.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <string>
#include <vector>

#include "bus_monitor.h"
#include "cJSON.h"
#include "can_link.h"
#include "check.h"
#include "config.h"
#include "requests.h"
#include "status.h"
#include "status_server.h"

using namespace canopen_plugin;

namespace
{

NetworkConfig two_nodes()
{
    NetworkConfig net;
    net.name      = "CANopen 1";
    net.interface = "can0";
    NodeConfig a;
    a.node_id = 2;
    a.name    = "io1";
    NodeConfig b;
    b.node_id = 3;
    b.name    = "io2";
    net.nodes = {a, b};
    return net;
}

const NodeStatus &node(const NetworkSnapshot &s, unsigned id)
{
    for (const NodeStatus &n : s.nodes)
        if (n.node_id == id)
            return n;
    return s.nodes.front();
}

void test_states_and_reasons()
{
    NetworkConfig net = two_nodes();
    NetworkStatus st(net);
    NetworkSnapshot s = st.snapshot();
    CHECK(node(s, 2).state == NodeRun::Waiting && !s.all_up(false) && s.all_up(true));

    st.boot_ok(2);
    Problem p;
    p.kind     = ProblemKind::Identity;
    p.field    = IdentityField::ProductCode;
    p.expected = 0x42;
    st.boot_failed(3, p);
    st.identity_found(3, IdentityField::ProductCode, 0x43);
    // A found value of another field is not taken.
    st.identity_found(3, IdentityField::VendorId, 7);
    s = st.snapshot();
    CHECK(node(s, 2).state == NodeRun::Operational && node(s, 2).boots == 1);
    CHECK(node(s, 3).state == NodeRun::Failed);
    CHECK(node(s, 3).problem.has_found && node(s, 3).problem.found == 0x43);
    CHECK(s.events.front().kind == EventKind::BootFailed && s.events.front().problem.found == 0x43);

    Problem hb;
    hb.kind = ProblemKind::Heartbeat;
    st.lost(2, hb);
    s = st.snapshot();
    CHECK(node(s, 2).state == NodeRun::Lost && node(s, 2).losses == 1);
    st.boot_ok(2);
    CHECK(st.snapshot().nodes[0].problem.kind == ProblemKind::None);

    st.stop_all();
    CHECK(node(st.snapshot(), 3).state == NodeRun::Stopped);
    st.begin_session();
    CHECK(node(st.snapshot(), 3).state == NodeRun::Waiting);
    CHECK(node(st.snapshot(), 2).losses == 1);
}

void test_emcy_and_late()
{
    NetworkConfig net = two_nodes();
    NetworkStatus st(net);
    st.boot_ok(2);
    const uint8_t data[5] = {1, 2, 3, 4, 5};
    for (unsigned i = 0; i < 10; ++i)
        st.emcy(2, static_cast<uint16_t>(0x1000 + i), 0x01, data);
    NetworkSnapshot s = st.snapshot();
    CHECK(node(s, 2).emcy.size() == NetworkStatus::kMaxEmcy);
    CHECK(node(s, 2).emcy.front().code == 0x1009 && node(s, 2).emcy.back().code == 0x1002);
    CHECK(node(s, 2).warning());
    st.emcy(2, 0, 0, data);
    s = st.snapshot();
    CHECK(!node(s, 2).warning());
    CHECK(s.events.front().kind == EventKind::EmcyReset);

    st.pdo_late(2, 1, 0x182);
    st.pdo_late(2, 1, 0x182);
    s = st.snapshot();
    CHECK(node(s, 2).late.size() == 1 && node(s, 2).warning());
    st.pdo_back(2, 1);
    CHECK(st.snapshot().nodes[0].late.empty());
    CHECK(st.snapshot().events.front().kind == EventKind::PdoBack);
}

void test_event_ring()
{
    NetworkConfig net = two_nodes();
    NetworkStatus st(net);
    for (unsigned i = 0; i < 100; ++i)
        st.sdo_abort(2, 0x2100, 0, i);
    NetworkSnapshot s = st.snapshot();
    CHECK(s.events.size() == NetworkStatus::kMaxEvents);
    CHECK(s.events.front().problem.abort_code == 99);
}

void test_json()
{
    NetworkConfig net = two_nodes();
    NetworkStatus st(net);
    st.boot_ok(2);
    Problem p;
    p.kind       = ProblemKind::SdoAbort;
    p.index      = 0x1800;
    p.subindex   = 5;
    p.abort_code = 0x06090030;
    st.boot_failed(3, p);
    BusStatus b;
    b.state        = BusState::ErrorPassive;
    b.has_counters = true;
    b.tx_errors    = 128;
    st.set_bus(b);
    std::string text = status_json({st.snapshot()}, 1234);
    cJSON *root      = cJSON_Parse(text.c_str());
    CHECK(root != nullptr);
    if (!root)
        return;
    CHECK(cJSON_GetObjectItem(root, "now")->valuedouble == 1234);
    const cJSON *n0   = cJSON_GetArrayItem(cJSON_GetObjectItem(root, "networks"), 0);
    const cJSON *bus  = cJSON_GetObjectItem(n0, "bus");
    const cJSON *node = cJSON_GetArrayItem(cJSON_GetObjectItem(n0, "nodes"), 1);
    const cJSON *prob = cJSON_GetObjectItem(node, "problem");
    CHECK(std::string(cJSON_GetObjectItem(bus, "state")->valuestring) == "error_passive");
    CHECK(cJSON_GetObjectItem(bus, "tx_errors")->valueint == 128);
    CHECK(cJSON_GetObjectItem(bus, "load_percent") == nullptr);
    CHECK(std::string(cJSON_GetObjectItem(node, "state")->valuestring) == "failed");
    CHECK(std::string(cJSON_GetObjectItem(prob, "kind")->valuestring) == "sdo_abort");
    CHECK(cJSON_GetObjectItem(prob, "abort_code")->valuedouble == 0x06090030);
    CHECK(cJSON_IsNull(
        cJSON_GetObjectItem(cJSON_GetArrayItem(cJSON_GetObjectItem(n0, "nodes"), 0), "problem")));
    CHECK(cJSON_IsFalse(cJSON_GetObjectItem(n0, "all_mandatory_up")));
    CHECK(cJSON_IsTrue(cJSON_GetObjectItem(n0, "all_optional_up")));
    const cJSON *ev = cJSON_GetArrayItem(cJSON_GetObjectItem(n0, "events"), 0);
    CHECK(std::string(cJSON_GetObjectItem(ev, "kind")->valuestring) == "bus_state");
    CHECK(cJSON_GetObjectItem(bus, "link_resets") == nullptr);
    cJSON_Delete(root);

    // Link resets: counted when done, kept across bus readings, each recorded with its outcome and
    // cause.
    b.state      = BusState::BusOff;
    b.has_resets = true;
    st.set_bus(b);
    st.link_reset(ResetOutcome::Done);
    st.link_reset(ResetOutcome::NotPermitted, ResetCause::BitRate);
    st.link_reset(ResetOutcome::Done, ResetCause::TxStalled);
    b.state      = BusState::ErrorActive;
    b.has_frames = true;
    b.tx_dropped = 42;
    st.set_bus(b);
    CHECK(st.snapshot().bus.resets == 2);
    text = status_json({st.snapshot()}, 1234);
    root = cJSON_Parse(text.c_str());
    CHECK(root != nullptr);
    if (!root)
        return;
    n0  = cJSON_GetArrayItem(cJSON_GetObjectItem(root, "networks"), 0);
    bus = cJSON_GetObjectItem(n0, "bus");
    CHECK(cJSON_GetObjectItem(bus, "link_resets")->valueint == 2);
    CHECK(cJSON_GetObjectItem(bus, "tx_dropped")->valueint == 42);
    const cJSON *events = cJSON_GetObjectItem(n0, "events");
    ev                  = cJSON_GetArrayItem(events, 1);
    CHECK(std::string(cJSON_GetObjectItem(ev, "kind")->valuestring) == "bus_reset");
    CHECK(std::string(cJSON_GetObjectItem(ev, "outcome")->valuestring) == "done");
    CHECK(std::string(cJSON_GetObjectItem(ev, "cause")->valuestring) == "tx_stalled");
    ev = cJSON_GetArrayItem(events, 2);
    CHECK(std::string(cJSON_GetObjectItem(ev, "outcome")->valuestring) == "not_permitted");
    CHECK(std::string(cJSON_GetObjectItem(ev, "cause")->valuestring) == "bit_rate");
    ev = cJSON_GetArrayItem(events, 3);
    CHECK(std::string(cJSON_GetObjectItem(ev, "outcome")->valuestring) == "done");
    CHECK(std::string(cJSON_GetObjectItem(ev, "cause")->valuestring) == "bus_off");
    cJSON_Delete(root);
}

void test_optional_and_held()
{
    NetworkConfig net     = two_nodes();
    net.nodes[1].optional = true;
    NetworkStatus st(net);
    st.silent(2);
    st.silent(3);
    NetworkSnapshot s = st.snapshot();
    CHECK(node(s, 2).state == NodeRun::Silent && node(s, 3).state == NodeRun::Absent);
    CHECK(s.events.front().kind == EventKind::Absent);
    st.boot_ok(2);
    s = st.snapshot();
    CHECK(s.all_up(false) && !s.all_up(true));

    Problem g;
    g.kind = ProblemKind::Guarding;
    st.lost(2, g);
    st.held(2);
    std::string text = status_json({st.snapshot()}, 1);
    CHECK(text.find("\"kind\":\"guarding\"") != std::string::npos);
    CHECK(text.find("\"held\":true") != std::string::npos);
    CHECK(text.find("\"state\":\"absent\"") != std::string::npos);
    CHECK(text.find("\"optional\":true") != std::string::npos);
    // A new session of the same PLC run keeps the hold and the lost state.
    st.begin_session();
    CHECK(node(st.snapshot(), 2).held && st.is_held(2));
    CHECK(node(st.snapshot(), 2).state == NodeRun::Lost);
}

struct FakeRequests : Requests
{
    ClientQueue queue;
    bool known_node = true;
    std::string refuse;
    ServiceRequest opened;
    int closes = 0;

    std::string status() override
    {
        return "{\"active\":true}";
    }
    Lookup network(size_t net, unsigned node, ClientQueue *&q) override
    {
        if (net != 0)
            return Lookup::UnknownNetwork;
        if (node && !known_node)
            return Lookup::UnknownNode;
        q = &queue;
        return Lookup::Ok;
    }
    bool job(uint64_t id, JobResult &out) override
    {
        return queue.result(id, out);
    }
    std::string service_open(const ServiceRequest &req) override
    {
        opened = req;
        return refuse;
    }
    void service_close() override
    {
        ++closes;
    }
};

uint64_t job_id(const std::string &answer)
{
    size_t at = answer.find("\"job\":");
    return at == std::string::npos ? 0 : std::stoull(answer.substr(at + 6));
}

void test_requests()
{
    FakeRequests t;
    CHECK(handle_request("{\"op\":\"status\"}", t) == "{\"active\":true}");
    CHECK(handle_request("{\"op\":\"teleport\"}", t) == "{\"error\":\"unknown_op\"}");
    CHECK(handle_request("[1]", t) == "{\"error\":\"bad_request\"}");
    CHECK(handle_request("{}", t) == "{\"error\":\"bad_request\"}");
    CHECK(handle_request("not json", t) == "{\"error\":\"bad_request\"}");

    // No session yet: refused.
    const std::string read = R"({"op":"sdo_read","network":0,"node":2,"index":4120,"subindex":2})";
    CHECK(handle_request(read, t) == "{\"error\":\"not_running\"}");
    t.queue.set_running(true);
    uint64_t id = job_id(handle_request(read, t));
    CHECK(id != 0);
    CHECK(handle_request("{\"op\":\"job\",\"job\":" + std::to_string(id) + "}", t) ==
          "{\"state\":\"pending\"}");
    ClientJob job;
    CHECK(t.queue.next(job) && job.id == id && job.kind == ClientJob::Kind::Read);
    CHECK(job.node == 2 && job.index == 0x1018 && job.subindex == 2);
    JobResult done;
    done.state = JobResult::State::Done;
    done.data  = {0x34, 0x12, 0x00, 0x00};
    t.queue.finish(id, done);
    CHECK(handle_request("{\"op\":\"job\",\"job\":" + std::to_string(id) + "}", t) ==
          "{\"state\":\"done\",\"data\":\"34120000\"}");
    CHECK(handle_request("{\"op\":\"job\",\"job\":999999}", t) == "{\"error\":\"unknown_job\"}");
    CHECK(handle_request("{\"op\":\"job\",\"job\":0}", t) == "{\"error\":\"bad_request\"}");

    // Writes: hex data, either case; bad hex, empty data and out-of-range fields are refused.
    id = job_id(handle_request(
        R"({"op":"sdo_write","network":0,"node":2,"index":8449,"subindex":0,"data":"0A0b"})", t));
    CHECK(id != 0 && t.queue.next(job) && job.kind == ClientJob::Kind::Write);
    CHECK(job.data.size() == 2 && job.data[0] == 0x0A && job.data[1] == 0x0B);
    JobResult aborted;
    aborted.state      = JobResult::State::Failed;
    aborted.has_abort  = true;
    aborted.abort_code = 0x08000022;
    t.queue.finish(id, aborted);
    CHECK(handle_request("{\"op\":\"job\",\"job\":" + std::to_string(id) + "}", t) ==
          "{\"state\":\"failed\",\"abort_code\":134217762}");
    for (const char *bad :
         {R"({"op":"sdo_write","network":0,"node":2,"index":1,"subindex":0,"data":"abc"})",
          R"({"op":"sdo_write","network":0,"node":2,"index":1,"subindex":0,"data":"zz"})",
          R"({"op":"sdo_write","network":0,"node":2,"index":1,"subindex":0,"data":""})",
          R"({"op":"sdo_write","network":0,"node":2,"index":1,"subindex":0})",
          R"({"op":"sdo_read","network":0,"node":0,"index":1,"subindex":0})",
          R"({"op":"sdo_read","network":0,"node":128,"index":1,"subindex":0})",
          R"({"op":"sdo_read","network":0,"node":2,"index":65536,"subindex":0})",
          R"({"op":"sdo_read","network":0,"node":2,"index":1,"subindex":256})",
          R"({"op":"sdo_read","network":0,"node":2.5,"index":1,"subindex":0})",
          R"({"op":"sdo_read","node":2,"index":1,"subindex":0})"})
        CHECK(handle_request(bad, t) == "{\"error\":\"bad_request\"}");
    std::string big(2 * (ClientQueue::kMaxData + 1), '0');
    CHECK(
        handle_request(R"({"op":"sdo_write","network":0,"node":2,"index":1,"subindex":0,"data":")" +
                           big + "\"}",
                       t) == "{\"error\":\"too_large\"}");
    CHECK(handle_request(R"({"op":"sdo_read","network":1,"node":2,"index":1,"subindex":0})", t) ==
          "{\"error\":\"unknown_network\"}");
    t.known_node = false;
    CHECK(handle_request(read, t) == "{\"error\":\"unknown_node\"}");
    t.known_node = true;

    // NMT: node 0 for all nodes, named commands only.
    CHECK(handle_request(R"({"op":"nmt","network":0,"node":0,"command":"pre_operational"})", t) ==
          "{\"ok\":true}");
    CHECK(t.queue.next(job) && job.kind == ClientJob::Kind::Nmt && job.node == 0 &&
          job.command == NmtRequest::PreOperational);
    CHECK(handle_request(R"({"op":"nmt","network":0,"node":2,"command":"jump"})", t) ==
          "{\"error\":\"bad_request\"}");
    CHECK(handle_request(R"({"op":"nmt"})", t) == "{\"error\":\"bad_request\"}");
    for (const char *name :
         {"start", "stop", "pre_operational", "reset_node", "reset_communication"})
    {
        NmtRequest r;
        CHECK(parse_nmt_request(name, r) && std::strcmp(nmt_request_name(r), name) == 0);
    }

    // The queue holds kMaxWaiting jobs; ending the session fails what is left.
    uint64_t first = 0;
    for (size_t i = 0; i < ClientQueue::kMaxWaiting; ++i)
    {
        uint64_t n = job_id(handle_request(read, t));
        CHECK(n != 0);
        if (!first)
            first = n;
    }
    CHECK(handle_request(read, t) == "{\"error\":\"queue_full\"}");
    t.queue.set_running(false);
    CHECK(handle_request("{\"op\":\"job\",\"job\":" + std::to_string(first) + "}", t) ==
          "{\"state\":\"failed\",\"reason\":\"not_running\"}");

    // Service mode.
    const std::string open =
        R"({"op":"service_open","interface":"can0","bitrate":250000,"master_node_id":127,"sdo_timeout_ms":500,"nodes":[2,5]})";
    CHECK(handle_request(open, t) == "{\"ok\":true,\"network\":0}");
    CHECK(t.opened.interface == "can0" && t.opened.bitrate == 250000 &&
          t.opened.sdo_timeout_ms == 500 && t.opened.nodes.size() == 2 && t.opened.nodes[1] == 5);
    t.refuse = "plc_running";
    CHECK(handle_request(open, t) == "{\"error\":\"plc_running\"}");
    for (
        const char *bad :
        {R"({"op":"service_open","interface":"can0","bitrate":123,"master_node_id":127,"sdo_timeout_ms":500,"nodes":[2]})",
         R"({"op":"service_open","interface":"a-very-long-name0","bitrate":0,"master_node_id":127,"sdo_timeout_ms":500,"nodes":[2]})",
         R"({"op":"service_open","interface":"can0","bitrate":0,"master_node_id":2,"sdo_timeout_ms":500,"nodes":[2]})",
         R"({"op":"service_open","interface":"can0","bitrate":0,"master_node_id":127,"sdo_timeout_ms":500,"nodes":[2,2]})",
         R"({"op":"service_open","interface":"can0","bitrate":0,"master_node_id":127,"sdo_timeout_ms":0,"nodes":[2]})",
         R"({"op":"service_open","interface":"","bitrate":0,"master_node_id":127,"sdo_timeout_ms":500,"nodes":[2]})"})
        CHECK(handle_request(bad, t) == "{\"error\":\"bad_request\"}");
    CHECK(handle_request("{\"op\":\"service_close\"}", t) == "{\"ok\":true}" && t.closes == 1);

    std::vector<uint8_t> bytes;
    CHECK(from_hex("00ff7F", bytes) && to_hex(bytes) == "00ff7f");
}

// Appends an attribute to the message in `buf` and returns it.
rtattr *put(char *buf, uint16_t type, const void *data, size_t len)
{
    nlmsghdr *nh  = reinterpret_cast<nlmsghdr *>(buf);
    rtattr *rta   = reinterpret_cast<rtattr *>(buf + NLMSG_ALIGN(nh->nlmsg_len));
    rta->rta_type = type;
    rta->rta_len  = static_cast<unsigned short>(RTA_LENGTH(len));
    if (len)
        std::memcpy(RTA_DATA(rta), data, len);
    nh->nlmsg_len = NLMSG_ALIGN(nh->nlmsg_len) + RTA_ALIGN(rta->rta_len);
    return rta;
}

void close_nest(char *buf, rtattr *nest)
{
    nlmsghdr *nh = reinterpret_cast<nlmsghdr *>(buf);
    nest->rta_len =
        static_cast<unsigned short>(buf + nh->nlmsg_len - reinterpret_cast<char *>(nest));
}

// An RTM_NEWLINK of a CAN link as the kernel sends it.
size_t can_newlink(char *buf, uint32_t state, uint16_t txerr, uint32_t bus_off, uint64_t rx)
{
    std::memset(buf, 0, 1024);
    nlmsghdr *nh   = reinterpret_cast<nlmsghdr *>(buf);
    nh->nlmsg_len  = NLMSG_LENGTH(sizeof(ifinfomsg));
    nh->nlmsg_type = RTM_NEWLINK;
    ifinfomsg *ifi = static_cast<ifinfomsg *>(NLMSG_DATA(nh));
    ifi->ifi_flags = IFF_UP;
    rtnl_link_stats64 stats{};
    stats.rx_packets = rx;
    stats.rx_bytes   = rx * 8;
    put(buf, IFLA_STATS64, &stats, sizeof(stats));
    rtattr *li = put(buf, IFLA_LINKINFO, nullptr, 0);
    put(buf, IFLA_INFO_KIND, "can", 4);
    rtattr *data = put(buf, IFLA_INFO_DATA, nullptr, 0);
    can_bittiming bt{};
    bt.bitrate = 500000;
    put(buf, IFLA_CAN_BITTIMING, &bt, sizeof(bt));
    put(buf, IFLA_CAN_STATE, &state, sizeof(state));
    can_berr_counter bc{txerr, 0};
    put(buf, IFLA_CAN_BERR_COUNTER, &bc, sizeof(bc));
    close_nest(buf, data);
    can_device_stats cs{};
    cs.bus_off = bus_off;
    put(buf, IFLA_INFO_XSTATS, &cs, sizeof(cs));
    close_nest(buf, li);
    return nh->nlmsg_len;
}

void test_link_parsing_and_monitor()
{
    alignas(nlmsghdr) char buf[1024];
    size_t len = can_newlink(buf, CAN_STATE_ERROR_ACTIVE, 3, 2, 1000);
    LinkInfo li;
    CHECK(parse_newlink(buf, len, li));
    CHECK(li.up && li.kind == "can" && li.bitrate == 500000);
    CHECK(li.has_can_state && li.can_state == CAN_STATE_ERROR_ACTIVE);
    CHECK(li.has_berr && li.tx_errors == 3);
    CHECK(li.has_xstats && li.bus_off == 2);
    CHECK(li.has_stats && li.rx_frames == 1000);

    BusMonitor m("can0");
    auto t0     = BusMonitor::clock::now();
    BusStatus b = m.update(li, t0);
    CHECK(b.state == BusState::ErrorActive && b.has_counters && b.tx_errors == 3);
    // Counted from the first reading.
    CHECK(b.bus_off == 0 && b.rx_frames == 0);

    // One second later: 1000 frames of 8 bytes at 500 kbit/s, a bus-off since.
    len = can_newlink(buf, CAN_STATE_ERROR_PASSIVE, 130, 3, 2000);
    CHECK(parse_newlink(buf, len, li));
    b = m.update(li, t0 + std::chrono::seconds(1));
    CHECK(b.state == BusState::ErrorPassive && b.tx_errors == 130);
    CHECK(b.bus_off == 1 && b.rx_frames == 1000);
    CHECK(b.has_load && b.load_percent > 21.0 && b.load_percent < 23.0);

    CHECK(m.unusable(BusState::Missing).state == BusState::Missing);
    LinkInfo vcan;
    vcan.up   = true;
    vcan.kind = "vcan";
    b         = m.update(vcan, t0 + std::chrono::seconds(2));
    CHECK(b.state == BusState::ErrorActive && !b.has_counters && !b.has_load);
}

} // namespace

int main()
{
    test_states_and_reasons();
    test_optional_and_held();
    test_emcy_and_late();
    test_event_ring();
    test_json();
    test_requests();
    test_link_parsing_and_monitor();
    return TEST_RESULT();
}
