// SPDX-License-Identifier: Apache-2.0
//
// Parser and validator of conf/canopen.json.

#include <string>
#include <vector>

#include "check.h"
#include "config.h"

using namespace canopen_plugin;

namespace
{

const char *kValid = R"({
  "version": 4,
  "networks": [{
    "name": "CANopen 1", "interface": "can0", "bitrate": 500000,
    "heartbeat_producer_ms": 100,
    "sync": {"mode": "plc_cycle"},
    "nodes": [{
      "node_id": 2, "name": "io1",
      "identity": {"device_type": 401, "vendor_id": 1, "product_code": 2, "revision": 3},
      "supervision": {"type": "heartbeat", "consumer_ms": 300},
      "on_loss": "zero",
      "state_location": "%IB10",
      "sdo": [{"index": 8448, "subindex": 0, "data": "78563412"}],
      "tpdos": [{"number": 1, "cob_id": 386, "entries": [
        {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB0"},
        {"index": 25601, "subindex": 1, "type": "INTEGER16", "location": "%IW0"}]}],
      "rpdos": [{"number": 1, "cob_id": 514, "entries": [
        {"index": 25088, "subindex": 1, "type": "UNSIGNED8", "location": "%QB0"}]}]
    }]
  }]
})";

std::string with(std::string text, const std::string &from, const std::string &to)
{
    size_t pos = text.find(from);
    if (pos == std::string::npos)
    {
        std::fprintf(stderr, "test bug: \"%s\" not in the base config\n", from.c_str());
        return "";
    }
    return text.replace(pos, from.size(), to);
}

bool contains(const std::vector<std::string> &list, const std::string &part)
{
    for (const auto &s : list)
        if (s.find(part) != std::string::npos)
            return true;
    return false;
}

// Parses `text`; true when it is refused with an error containing `part`.
bool refused_with(const std::string &text, const std::string &part)
{
    Config cfg;
    std::vector<std::string> errors;
    bool ok = parse_config(text, cfg, errors);
    if (ok || !contains(errors, part))
    {
        std::fprintf(stderr, "expected an error with \"%s\", got:\n", part.c_str());
        for (const auto &e : errors)
            std::fprintf(stderr, "  %s\n", e.c_str());
        return false;
    }
    return true;
}

void test_valid()
{
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(kValid, cfg, errors));
    CHECK(errors.empty());
    CHECK(cfg.warnings.empty());
    CHECK(cfg.networks.size() == 1);
    const NetworkConfig &net = cfg.networks[0];
    CHECK(net.interface == "can0");
    CHECK(net.bitrate == 500000);
    CHECK(net.master_node_id == 127);
    CHECK(net.sdo_timeout_ms == 1000);
    CHECK(net.heartbeat_producer_ms == 100);
    CHECK(net.priority == kDefaultPriority);
    CHECK(net.sync == SyncMode::PlcCycle);
    CHECK(net.on_stop == OnStop::PreOperational);
    CHECK(net.nodes.size() == 1);
    const NodeConfig &n = net.nodes[0];
    CHECK(n.node_id == 2);
    CHECK(n.vendor_id == 1 && n.product_code == 2 && n.revision == 3 && n.device_type == 401);
    CHECK(n.supervision == Supervision::Heartbeat);
    CHECK(n.heartbeat_consumer_ms == 300);
    CHECK(n.on_loss == OnLoss::Zero);
    CHECK(n.has_state_location && n.state_location.str() == "%IB10");
    CHECK(n.sdo.size() == 1);
    CHECK(n.sdo[0].index == 0x2100 && n.sdo[0].subindex == 0);
    CHECK((n.sdo[0].data == std::vector<uint8_t>{0x78, 0x56, 0x34, 0x12}));
    CHECK(n.tpdos.size() == 1 && n.tpdos[0].cob_id == 0x182);
    CHECK(n.tpdos[0].entries.size() == 2 && n.tpdos[0].bits() == 24);
    CHECK(n.tpdos[0].entries[1].type == CoType::INTEGER16);
    CHECK(n.rpdos.size() == 1 && n.rpdos[0].entries[0].location.str() == "%QB0");
    CHECK(n.tpdos[0].receive_timeout_ms == 0);
    // Defaults of the node startup and supervision options.
    CHECK(!n.optional && n.configure && n.restore_defaults == 0);
    CHECK(n.emcy_enabled && n.emcy_cob_id == 0x82);
    CHECK(net.boot_timeout_ms == 2000 && net.on_node_error == OnNodeError::Restart);
    CHECK(!net.time_producer);
}

void test_node_options()
{
    Config cfg;
    std::vector<std::string> errors;
    std::string text =
        with(with(kValid, "\"node_id\": 2, \"name\": \"io1\",",
                  "\"node_id\": 2, \"name\": \"io1\", \"optional\": true, "
                  "\"restore_defaults\": 2, \"emcy\": {\"enabled\": true, \"cob_id\": 242},"),
             "\"supervision\": {\"type\": \"heartbeat\", \"consumer_ms\": 300}",
             "\"supervision\": {\"type\": \"guarding\", \"guard_time_ms\": 100, "
             "\"life_time_factor\": 3}");
    text = with(text, "\"bitrate\": 500000,",
                "\"bitrate\": 500000, \"boot_timeout_ms\": 5000, \"on_node_error\": \"stop\", "
                "\"time\": {\"cob_id\": 256, \"period_ms\": 500},");
    CHECK(parse_config(text, cfg, errors));
    CHECK(errors.empty());
    const NetworkConfig &net = cfg.networks[0];
    const NodeConfig &n      = net.nodes[0];
    CHECK(n.optional && n.restore_defaults == 2 && n.emcy_cob_id == 0xF2);
    CHECK(n.supervision == Supervision::Guarding && n.guard_time_ms == 100 &&
          n.life_time_factor == 3);
    CHECK(net.boot_timeout_ms == 5000 && net.on_node_error == OnNodeError::Stop);
    CHECK(net.time_producer && net.time_cob_id == 0x100 && net.time_period_ms == 500);

    // Not initialized: no restore and no configuration SDOs.
    std::string no_init = with(kValid, "\"node_id\": 2, \"name\": \"io1\",",
                               "\"node_id\": 2, \"name\": \"io1\", \"configure\": false, "
                               "\"restore_defaults\": 1,");
    CHECK(refused_with(no_init, "restore defaults needs"));
    CHECK(refused_with(no_init, "has configuration SDOs but is not initialized"));
    CHECK(refused_with(with(kValid, "\"type\": \"heartbeat\", \"consumer_ms\": 300",
                            "\"type\": \"guarding\", \"guard_time_ms\": 0, "
                            "\"life_time_factor\": 3"),
                       "guard_time_ms"));
    CHECK(refused_with(with(kValid, "\"node_id\": 2, \"name\": \"io1\",",
                            "\"node_id\": 2, \"name\": \"io1\", \"emcy\": {\"cob_id\": 386},"),
                       "same COB-ID"));
    CHECK(refused_with(
        with(kValid, "\"bitrate\": 500000,", "\"bitrate\": 500000, \"on_node_error\": \"reboot\","),
        "on_node_error"));
    CHECK(refused_with(
        with(kValid, "\"bitrate\": 500000,", "\"bitrate\": 500000, \"time\": {\"cob_id\": 1793},"),
        "reserved"));
    CHECK(refused_with(with(kValid, "\"node_id\": 2, \"name\": \"io1\",",
                            "\"node_id\": 2, \"name\": \"io1\", \"optional\": 1,"),
                       "true or false"));
}

void test_receive_timeout()
{
    Config cfg;
    std::vector<std::string> errors;
    std::string text = with(kValid, "\"tpdos\": [{\"number\": 1,",
                            "\"tpdos\": [{\"number\": 1, \"receive_timeout_ms\": 250,");
    CHECK(parse_config(text, cfg, errors));
    CHECK(errors.empty());
    CHECK(cfg.networks[0].nodes[0].tpdos[0].receive_timeout_ms == 250);
}

void test_refusals()
{
    CHECK(refused_with("{", "not valid JSON"));
    CHECK(refused_with(with(kValid, "\"version\": 4", "\"version\": 3"),
                       "version 3 is not supported (this runtime reads version 4)"));
    CHECK(refused_with(with(kValid, "\"tpdos\": [{\"number\": 1,",
                            "\"tpdos\": [{\"number\": 1, \"receive_timeout_ms\": 70000,"),
                       "receive_timeout_ms"));
    CHECK(refused_with(with(kValid, "\"rpdos\": [{\"number\": 1,",
                            "\"rpdos\": [{\"number\": 1, \"receive_timeout_ms\": 100,"),
                       "only for a node's TPDOs"));
    CHECK(refused_with(with(kValid, "\"bitrate\": 500000", "\"bitrate\": 400000"), "bit rate"));
    CHECK(refused_with(with(kValid, "\"bitrate\": 500000", "\"bitrate\": 500000, \"priority\": 98"),
                       "\"priority\" must be an integer from 1 to 97"));
    CHECK(refused_with(with(kValid, "\"node_id\": 2", "\"node_id\": 127"), "master's node ID"));
    CHECK(refused_with(with(kValid, "\"index\": 8448", "\"index\": 4112"), "0x1010"));
    CHECK(refused_with(with(kValid, "\"index\": 8448", "\"index\": 4113"), "0x1011"));
    CHECK(refused_with(with(kValid, "\"data\": \"78563412\"", "\"data\": \"785\""), "hexadecimal"));
    CHECK(
        refused_with(with(kValid, "\"location\": \"%IB0\"", "\"location\": \"%QB4\""), "must be"));
    CHECK(refused_with(with(kValid, "\"location\": \"%IW0\"", "\"location\": \"%ID0\""),
                       "INTEGER16"));
    CHECK(refused_with(with(kValid, "\"type\": \"UNSIGNED8\"", "\"type\": \"VISIBLE_STRING\""),
                       "not supported"));
    CHECK(refused_with(with(kValid, "\"cob_id\": 386", "\"cob_id\": 128"), "reserved"));
    CHECK(refused_with(with(kValid, "\"cob_id\": 386", "\"cob_id\": 514"), "same COB-ID"));
    CHECK(refused_with(with(kValid, "\"location\": \"%IW0\"",
                            "\"location\": \"%IW0\"}, "
                            "{\"index\": 1, \"subindex\": 0, "
                            "\"type\": \"UNSIGNED64\", "
                            "\"location\": \"%IL0\""),
                       "64"));
    CHECK(refused_with(
        with(kValid, "\"state_location\": \"%IB10\"", "\"state_location\": \"%IB0\""), "%IB0"));
    CHECK(
        refused_with(with(kValid, "\"mode\": \"plc_cycle\"", "\"mode\": \"often\""), "sync mode"));

    // Every problem is reported, not only the first.
    Config cfg;
    std::vector<std::string> errors;
    std::string two = with(with(kValid, "\"bitrate\": 500000", "\"bitrate\": 400000"),
                           "\"cob_id\": 386", "\"cob_id\": 128");
    CHECK(!parse_config(two, cfg, errors));
    CHECK(contains(errors, "bit rate") && contains(errors, "reserved"));
}

void test_output_overlap_warns()
{
    std::string text = with(kValid, "\"location\": \"%QB0\"}]}]",
                            "\"location\": \"%QB0\"}]}, {\"number\": 2, \"cob_id\": 770, "
                            "\"entries\": [{\"index\": 25088, \"subindex\": 2, \"type\": "
                            "\"UNSIGNED8\", \"location\": \"%QB0\"}]}]");
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(text, cfg, errors));
    CHECK(!cfg.warnings.empty() && contains(cfg.warnings, "%QB0"));
}

void test_priority()
{
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(with(kValid, "\"bitrate\": 500000", "\"bitrate\": 500000, \"priority\": 80"),
                       cfg, errors));
    CHECK(cfg.networks.size() == 1 && cfg.networks[0].priority == 80);
}

// A node with three configuration SDOs; item 0 gets `policy`.
std::string with_policy(const std::string &policy)
{
    return with(kValid, "\"sdo\": [{\"index\": 8448, \"subindex\": 0, \"data\": \"78563412\"}]",
                "\"sdo\": [{\"index\": 8448, \"subindex\": 0, \"data\": \"78563412\"" + policy +
                    "}, {\"index\": 8448, \"subindex\": 1, \"data\": \"01\"}, "
                    "{\"index\": 8448, \"subindex\": 2, \"data\": \"02\"}]");
}

void test_sdo_policies()
{
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(with_policy(""), cfg, errors));
    CHECK(cfg.networks[0].nodes[0].sdo[0].on_abort == OnAbort::Stop);
    CHECK(parse_config(with_policy(", \"on_abort\": \"continue\""), cfg, errors));
    CHECK(cfg.networks[0].nodes[0].sdo[0].on_abort == OnAbort::Continue);
    CHECK(parse_config(with_policy(", \"on_abort\": \"jump\", \"jump_to\": 2"), cfg, errors));
    CHECK(cfg.networks[0].nodes[0].sdo[0].on_abort == OnAbort::Jump);
    CHECK(cfg.networks[0].nodes[0].sdo[0].jump_to == 2);
    CHECK(errors.empty());

    CHECK(refused_with(with_policy(", \"on_abort\": \"retry\""), "not one of stop, continue, jump"));
    CHECK(refused_with(with_policy(", \"on_abort\": \"jump\""), "jump_to"));
    CHECK(refused_with(with_policy(", \"on_abort\": \"jump\", \"jump_to\": 0"),
                       "node 2 (io1): SDO 1"));
    CHECK(refused_with(with_policy(", \"on_abort\": \"jump\", \"jump_to\": 3"), "later item"));
    CHECK(refused_with(with_policy(", \"jump_to\": 2"), "only for"));
}

void test_dummy_entries()
{
    const std::string gap = "\"location\": \"%QB0\"}, {\"index\": 5, \"subindex\": 0, "
                            "\"type\": \"UNSIGNED8\"}, {\"index\": 25088, \"subindex\": 2, "
                            "\"type\": \"UNSIGNED8\", \"location\": \"%QB1\"";
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(with(kValid, "\"location\": \"%QB0\"", gap), cfg, errors));
    CHECK(errors.empty());
    const PdoConfig &rpdo = cfg.networks[0].nodes[0].rpdos[0];
    CHECK(rpdo.entries.size() == 3 && rpdo.entries[1].dummy && !rpdo.entries[0].dummy);
    CHECK(rpdo.bits() == 24);

    CHECK(refused_with(with(kValid, ", \"location\": \"%QB0\"", ""),
                       "RPDO 1 entry 1 0x6200:1: has no \"location\""));
    CHECK(refused_with(with(kValid, "\"location\": \"%QB0\"",
                            "\"location\": \"%QB0\"}, {\"index\": 6, \"subindex\": 0, "
                            "\"type\": \"UNSIGNED8\""),
                       "is a UNSIGNED16"));
}

void test_send_period()
{
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(with(kValid, "\"rpdos\": [{\"number\": 1,",
                            "\"rpdos\": [{\"number\": 1, \"send_period_ms\": 50,"),
                       cfg, errors));
    CHECK(cfg.networks[0].nodes[0].rpdos[0].send_period_ms == 50);
    CHECK(cfg.networks[0].nodes[0].tpdos[0].send_period_ms == 0);
    CHECK(refused_with(with(kValid, "\"rpdos\": [{\"number\": 1,",
                            "\"rpdos\": [{\"number\": 1, \"send_period_ms\": 0,"),
                       "RPDO 1"));
    CHECK(refused_with(with(kValid, "\"tpdos\": [{\"number\": 1,",
                            "\"tpdos\": [{\"number\": 1, \"send_period_ms\": 10,"),
                       "only for a node's RPDOs"));
}

} // namespace

int main()
{
    test_valid();
    test_refusals();
    test_node_options();
    test_receive_timeout();
    test_output_overlap_warns();
    test_priority();
    test_sdo_policies();
    test_dummy_entries();
    test_send_period();
    CHECK(is_standard_bitrate(125000) && is_standard_bitrate(1000000));
    CHECK(!is_standard_bitrate(100000) && !is_standard_bitrate(0));
    return TEST_RESULT();
}
