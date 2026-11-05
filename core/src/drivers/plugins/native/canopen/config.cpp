// SPDX-License-Identifier: Apache-2.0

#include "config.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include "cJSON.h"

namespace canopen_plugin
{

namespace
{

constexpr unsigned kMaxNodeId    = 127;
constexpr unsigned kMaxPdoNumber = 512;
constexpr unsigned kMaxPdoBits   = 64;
constexpr size_t kMaxSdoBytes    = 1024;
// The master's own objects for mapped entries (master_dcf.h).
constexpr size_t kMaxMasterEntries = 4096;

std::string hex(unsigned v, int width)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%0*X", width, v);
    return buf;
}

std::string object_text(uint16_t index, uint8_t subindex)
{
    return hex(index, 4) + ":" + std::to_string(subindex);
}

class Parser
{
  public:
    Parser(Config &cfg, std::vector<std::string> &errors) : cfg_(cfg), errors_(errors) {}

    void error(const std::string &where, const std::string &what)
    {
        errors_.push_back(where.empty() ? what : where + ": " + what);
    }

    void warn(const std::string &where, const std::string &what)
    {
        cfg_.warnings.push_back(where.empty() ? what : where + ": " + what);
    }

    // An integer member in [lo, hi]. Missing: `def` when given, else an error.
    bool get_uint(const cJSON *obj, const char *key, const std::string &where, uint64_t lo,
                  uint64_t hi, uint64_t &out, const uint64_t *def = nullptr)
    {
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
        if (!v)
        {
            if (def)
            {
                out = *def;
                return true;
            }
            error(where, std::string("missing \"") + key + "\"");
            return false;
        }
        double d = v->valuedouble;
        if (!cJSON_IsNumber(v) || std::floor(d) != d || d < static_cast<double>(lo) ||
            d > static_cast<double>(hi))
        {
            error(where, std::string("\"") + key + "\" must be an integer from " +
                             std::to_string(lo) + " to " + std::to_string(hi));
            return false;
        }
        out = static_cast<uint64_t>(d);
        return true;
    }

    // A boolean member; missing keeps `out`.
    bool get_bool(const cJSON *obj, const char *key, const std::string &where, bool &out)
    {
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
        if (!v)
            return true;
        if (!cJSON_IsBool(v))
        {
            error(where, std::string("\"") + key + "\" must be true or false");
            return false;
        }
        out = cJSON_IsTrue(v);
        return true;
    }

    bool get_string(const cJSON *obj, const char *key, const std::string &where, std::string &out,
                    bool required)
    {
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
        if (!v)
        {
            if (required)
                error(where, std::string("missing \"") + key + "\"");
            return !required;
        }
        if (!cJSON_IsString(v))
        {
            error(where, std::string("\"") + key + "\" must be a string");
            return false;
        }
        out = v->valuestring;
        return true;
    }

    const cJSON *get_array(const cJSON *obj, const char *key, const std::string &where,
                           bool required)
    {
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
        if (!v)
        {
            if (required)
                error(where, std::string("missing \"") + key + "\"");
            return nullptr;
        }
        if (!cJSON_IsArray(v))
        {
            error(where, std::string("\"") + key + "\" must be an array");
            return nullptr;
        }
        return v;
    }

    bool parse(const cJSON *root);

  private:
    bool parse_network(const cJSON *j, size_t pos, NetworkConfig &net);
    bool parse_node(const cJSON *j, const NetworkConfig &net, NodeConfig &node);
    void parse_lss(const cJSON *j, const std::string &where, NodeConfig &node);
    bool parse_sdo(const cJSON *j, const std::string &where, SdoWrite &sdo);
    bool parse_pdo(const cJSON *j, const std::string &where, IecArea area, PdoConfig &pdo);
    void check_cob_ids(const NetworkConfig &net);
    void check_locations();
    bool parse_device(const cJSON *j, size_t pos, DeviceConfig &dev);
    void parse_slave_lss(const cJSON *j, const std::string &where, SlaveLss &lss);
    bool parse_object(const cJSON *j, const std::string &where, SlaveObject &obj);
    void parse_packed_bits(const cJSON *j, const cJSON *bits, const std::string &w, IecArea area,
                           SlaveObject &obj);
    bool parse_value(const cJSON *v, const std::string &where, const char *key, CoType type,
                     uint64_t &out);
    void parse_slave_pdos(const cJSON *j, const char *key, const std::string &where, bool tpdo,
                          DeviceConfig &dev);

    Config &cfg_;
    std::vector<std::string> &errors_;
};

bool parse_hex_bytes(const std::string &text, std::vector<uint8_t> &out)
{
    if (text.empty() || text.size() % 2)
        return false;
    out.clear();
    for (size_t i = 0; i < text.size(); i += 2)
    {
        unsigned v = 0;
        for (size_t k = i; k < i + 2; ++k)
        {
            char c = text[k];
            v <<= 4;
            if (c >= '0' && c <= '9')
                v |= c - '0';
            else if (c >= 'a' && c <= 'f')
                v |= c - 'a' + 10;
            else if (c >= 'A' && c <= 'F')
                v |= c - 'A' + 10;
            else
                return false;
        }
        out.push_back(static_cast<uint8_t>(v));
    }
    return true;
}

// COB-IDs a PDO must not use (CiA 301 predefined connection set and LSS).
const char *reserved_cob_id(uint32_t id)
{
    if (id == 0x000)
        return "NMT";
    if (id == 0x080)
        return "SYNC";
    if (id >= 0x081 && id <= 0x0FF)
        return "EMCY";
    if (id == 0x100)
        return "TIME";
    if (id >= 0x581 && id <= 0x5FF)
        return "SDO";
    if (id >= 0x601 && id <= 0x67F)
        return "SDO";
    if (id >= 0x701 && id <= 0x77F)
        return "heartbeat";
    if (id == 0x7E4 || id == 0x7E5)
        return "LSS";
    return nullptr;
}

bool Parser::parse(const cJSON *root)
{
    size_t failed = errors_.size();
    if (!cJSON_IsObject(root))
    {
        error("", "the configuration must be a JSON object");
        return false;
    }
    uint64_t version = 0;
    if (!get_uint(root, "version", "", 0, 0xFFFF, version))
        return false;
    if (version != kConfigVersion)
    {
        error("", "configuration format version " + std::to_string(version) +
                      " is not supported (this runtime reads version " +
                      std::to_string(kConfigVersion) + ")");
        return false;
    }
    const cJSON *nets = get_array(root, "networks", "", true);
    if (!nets)
        return false;
    size_t pos     = 0;
    const cJSON *j = nullptr;
    cJSON_ArrayForEach(j, nets)
    {
        NetworkConfig net;
        parse_network(j, ++pos, net);
        cfg_.networks.push_back(std::move(net));
    }
    const cJSON *devs = get_array(root, "devices", "", false);
    if (devs)
    {
        pos = 0;
        cJSON_ArrayForEach(j, devs)
        {
            DeviceConfig dev;
            parse_device(j, ++pos, dev);
            cfg_.devices.push_back(std::move(dev));
        }
    }
    if (cfg_.networks.empty() && cfg_.devices.empty())
        error("", "\"networks\" and \"devices\" are both empty");
    if (!cfg_.devices.empty() && !slave_supported())
        error("", "this runtime has no CANopen slave support; \"devices\" cannot be run");
    std::map<std::string, std::string> ifaces;
    for (const auto &net : cfg_.networks)
    {
        if (net.interface.empty())
            continue;
        auto it = ifaces.find(net.interface);
        if (it != ifaces.end())
            error(net.label(), "interface " + net.interface + " is also used by " + it->second);
        else
            ifaces[net.interface] = net.label();
        check_cob_ids(net);
    }
    for (const auto &dev : cfg_.devices)
    {
        if (dev.interface.empty())
            continue;
        auto it = ifaces.find(dev.interface);
        if (it != ifaces.end())
            error(dev.label(), "interface " + dev.interface + " is also used by " + it->second +
                                   "; a CANopen master and slave cannot share an interface");
        else
            ifaces[dev.interface] = dev.label();
    }
    check_locations();
    return errors_.size() == failed;
}

bool Parser::parse_network(const cJSON *j, size_t pos, NetworkConfig &net)
{
    std::string where = "network " + std::to_string(pos);
    if (!cJSON_IsObject(j))
    {
        error(where, "must be an object");
        return false;
    }
    get_string(j, "name", where, net.name, false);
    get_string(j, "interface", where, net.interface, true);
    if (!net.interface.empty() || !net.name.empty())
        where = net.label();
    if (net.interface.size() >= 16)
        error(where, "interface name \"" + net.interface + "\" is longer than 15 characters");

    uint64_t v          = 0;
    const uint64_t zero = 0;
    if (get_uint(j, "bitrate", where, 0, 1000000, v, &zero))
    {
        net.bitrate = static_cast<unsigned>(v);
        if (v && !is_standard_bitrate(net.bitrate))
            error(where, "bit rate " + std::to_string(v) +
                             " is not a CANopen bit rate (10, 20, 50, 125, 250, 500, 800 or "
                             "1000 kbit/s)");
    }
    const uint64_t def_master = 127;
    if (get_uint(j, "master_node_id", where, 1, kMaxNodeId, v, &def_master))
        net.master_node_id = static_cast<unsigned>(v);
    if (get_uint(j, "heartbeat_producer_ms", where, 0, 65535, v, &zero))
        net.heartbeat_producer_ms = static_cast<unsigned>(v);
    const uint64_t def_priority = kDefaultPriority;
    if (get_uint(j, "priority", where, 1, kMaxPriority, v, &def_priority))
        net.priority = static_cast<int>(v);
    const uint64_t def_sdo_timeout = 1000;
    if (get_uint(j, "sdo_timeout_ms", where, 10, 60000, v, &def_sdo_timeout))
        net.sdo_timeout_ms = static_cast<unsigned>(v);

    const uint64_t def_boot_timeout = 2000;
    if (get_uint(j, "boot_timeout_ms", where, 100, 60000, v, &def_boot_timeout))
        net.boot_timeout_ms = static_cast<unsigned>(v);
    std::string on_error;
    if (get_string(j, "on_node_error", where, on_error, false) && !on_error.empty())
    {
        if (on_error == "restart")
            net.on_node_error = OnNodeError::Restart;
        else if (on_error == "stop")
            net.on_node_error = OnNodeError::Stop;
        else
            error(where, "\"on_node_error\" \"" + on_error + "\" is not one of restart, stop");
    }
    const cJSON *time = cJSON_GetObjectItemCaseSensitive(j, "time");
    if (time && !cJSON_IsNull(time))
    {
        const std::string w    = where + ": time";
        const uint64_t def_cob = 0x100, def_period = 1000;
        if (!cJSON_IsObject(time))
            error(where, "\"time\" must be an object or null");
        else
        {
            net.time_producer = true;
            if (get_uint(time, "cob_id", w, 1, 0x7FF, v, &def_cob))
                net.time_cob_id = static_cast<uint32_t>(v);
            if (get_uint(time, "period_ms", w, 10, 65535, v, &def_period))
                net.time_period_ms = static_cast<unsigned>(v);
        }
    }

    const cJSON *sync = cJSON_GetObjectItemCaseSensitive(j, "sync");
    if (sync)
    {
        std::string mode;
        if (!cJSON_IsObject(sync))
            error(where, "\"sync\" must be an object");
        else if (get_string(sync, "mode", where + ": sync", mode, true))
        {
            if (mode == "none")
                net.sync = SyncMode::None;
            else if (mode == "plc_cycle")
                net.sync = SyncMode::PlcCycle;
            else if (mode == "period")
            {
                net.sync = SyncMode::Period;
                if (get_uint(sync, "period_us", where + ": sync", 1000, 10000000, v))
                    net.sync_period_us = static_cast<unsigned>(v);
            }
            else
                error(where, "sync mode \"" + mode + "\" is not one of none, period, plc_cycle");
        }
    }

    std::string on_stop;
    if (get_string(j, "on_plc_stop", where, on_stop, false) && !on_stop.empty())
    {
        if (on_stop == "pre_operational")
            net.on_stop = OnStop::PreOperational;
        else if (on_stop == "stopped")
            net.on_stop = OnStop::Stopped;
        else if (on_stop == "none")
            net.on_stop = OnStop::None;
        else
            error(where, "\"on_plc_stop\" \"" + on_stop +
                             "\" is not one of pre_operational, stopped, none");
    }

    const cJSON *nodes = get_array(j, "nodes", where, true);
    if (!nodes)
        return false;
    const cJSON *n = nullptr;
    std::map<unsigned, std::string> ids;
    cJSON_ArrayForEach(n, nodes)
    {
        NodeConfig node;
        if (!parse_node(n, net, node))
            continue;
        auto it = ids.find(node.node_id);
        if (it != ids.end())
            error(where, node.label() + " and " + it->second + " have the same node ID");
        else
            ids[node.node_id] = node.label();
        if (node.node_id == net.master_node_id)
            error(where, node.label() + " has the master's node ID");
        for (const NodeConfig &other : net.nodes)
            if (node.has_lss && other.has_lss && other.lss.vendor_id == node.lss.vendor_id &&
                other.lss.product_code == node.lss.product_code &&
                other.lss.revision == node.lss.revision &&
                other.lss.serial_number == node.lss.serial_number)
                error(where, node.label() + " and " + other.label() +
                                 " have the same LSS address (vendor ID, product code, revision "
                                 "and serial number)");
        net.nodes.push_back(std::move(node));
    }
    size_t in = 0, out = 0, master_rpdos = 0, master_tpdos = 0;
    for (const auto &node : net.nodes)
    {
        master_rpdos += node.tpdos.size();
        master_tpdos += node.rpdos.size();
        for (const auto &p : node.tpdos)
            in += p.entries.size();
        for (const auto &p : node.rpdos)
            out += p.entries.size();
    }
    if (master_rpdos > kMaxPdoNumber || master_tpdos > kMaxPdoNumber)
        error(where, "more than 512 PDOs in one direction");
    if (in > kMaxMasterEntries || out > kMaxMasterEntries)
        error(where, "more than 4096 mapped entries in one direction");
    return true;
}

bool Parser::parse_node(const cJSON *j, const NetworkConfig &net, NodeConfig &node)
{
    std::string where = net.label() + ": node";
    if (!cJSON_IsObject(j))
    {
        error(where, "must be an object");
        return false;
    }
    uint64_t v = 0;
    if (!get_uint(j, "node_id", where, 1, kMaxNodeId, v))
        return false;
    node.node_id = static_cast<unsigned>(v);
    get_string(j, "name", where, node.name, false);
    where            = net.label() + ": " + node.label();
    size_t failed    = errors_.size();
    node.emcy_cob_id = 0x80 + node.node_id;

    get_bool(j, "optional", where, node.optional);
    get_bool(j, "configure", where, node.configure);
    std::string start;
    if (get_string(j, "start", where, start, true))
    {
        if (start == "master")
            node.program_start = false;
        else if (start == "program")
            node.program_start = true;
        else
            error(where, "\"start\" \"" + start + "\" is not one of master, program");
    }
    if (!cJSON_GetObjectItemCaseSensitive(j, "enabled"))
        error(where, "missing \"enabled\"");
    else
        get_bool(j, "enabled", where, node.enabled);
    const uint64_t zero_restore = 0;
    if (get_uint(j, "restore_defaults", where, 0, 127, v, &zero_restore))
        node.restore_defaults = static_cast<unsigned>(v);
    if (node.restore_defaults && !node.configure)
        error(where, "restore defaults needs the node to be initialized (\"configure\": true)");

    const cJSON *emcy = cJSON_GetObjectItemCaseSensitive(j, "emcy");
    if (emcy)
    {
        const uint64_t def_cob = node.emcy_cob_id;
        if (!cJSON_IsObject(emcy))
            error(where, "\"emcy\" must be an object");
        else
        {
            get_bool(emcy, "enabled", where + ": emcy", node.emcy_enabled);
            if (get_uint(emcy, "cob_id", where + ": emcy", 1, 0x7FF, v, &def_cob))
                node.emcy_cob_id = static_cast<uint32_t>(v);
        }
    }

    const cJSON *ident = cJSON_GetObjectItemCaseSensitive(j, "identity");
    if (ident)
    {
        const uint64_t zero = 0;
        const std::string w = where + ": identity";
        if (!cJSON_IsObject(ident))
            error(where, "\"identity\" must be an object");
        else
        {
            if (get_uint(ident, "device_type", w, 0, 0xFFFFFFFF, v, &zero))
                node.device_type = static_cast<uint32_t>(v);
            if (get_uint(ident, "vendor_id", w, 0, 0xFFFFFFFF, v, &zero))
                node.vendor_id = static_cast<uint32_t>(v);
            if (get_uint(ident, "product_code", w, 0, 0xFFFFFFFF, v, &zero))
                node.product_code = static_cast<uint32_t>(v);
            if (get_uint(ident, "revision", w, 0, 0xFFFFFFFF, v, &zero))
                node.revision = static_cast<uint32_t>(v);
        }
    }

    if (const cJSON *lss = cJSON_GetObjectItemCaseSensitive(j, "lss"))
        parse_lss(lss, where, node);

    const cJSON *sup = cJSON_GetObjectItemCaseSensitive(j, "supervision");
    if (sup)
    {
        std::string type;
        if (!cJSON_IsObject(sup))
            error(where, "\"supervision\" must be an object");
        else if (get_string(sup, "type", where + ": supervision", type, true))
        {
            if (type == "none")
                node.supervision = Supervision::None;
            else if (type == "heartbeat")
            {
                node.supervision = Supervision::Heartbeat;
                if (get_uint(sup, "consumer_ms", where + ": supervision", 1, 65535, v))
                    node.heartbeat_consumer_ms = static_cast<unsigned>(v);
            }
            else if (type == "guarding")
            {
                node.supervision = Supervision::Guarding;
                if (get_uint(sup, "guard_time_ms", where + ": supervision", 1, 65535, v))
                    node.guard_time_ms = static_cast<unsigned>(v);
                if (get_uint(sup, "life_time_factor", where + ": supervision", 1, 255, v))
                    node.life_time_factor = static_cast<unsigned>(v);
            }
            else
                error(where,
                      "supervision type \"" + type + "\" is not one of none, heartbeat, guarding");
        }
    }

    std::string on_loss;
    if (get_string(j, "on_loss", where, on_loss, false) && !on_loss.empty())
    {
        if (on_loss == "hold")
            node.on_loss = OnLoss::Hold;
        else if (on_loss == "zero")
            node.on_loss = OnLoss::Zero;
        else
            error(where, "\"on_loss\" \"" + on_loss + "\" is not one of hold, zero");
    }

    std::string state;
    if (get_string(j, "state_location", where, state, false) && !state.empty())
    {
        std::string why;
        if (!parse_iec_location(state, node.state_location, why))
            error(where, "state location \"" + state + "\": " + why);
        else if (node.state_location.area != IecArea::Input ||
                 node.state_location.size != IecSize::B)
            error(where, "state location " + state + " must be a byte input (%IB)");
        else
            node.has_state_location = true;
    }

    if (const cJSON *sdos = get_array(j, "sdo", where, false))
    {
        size_t pos     = 0;
        const cJSON *s = nullptr;
        cJSON_ArrayForEach(s, sdos)
        {
            SdoWrite w;
            if (parse_sdo(s, where + ": SDO " + std::to_string(++pos), w))
                node.sdo.push_back(std::move(w));
            else
                node.sdo.push_back(SdoWrite()); // keeps jump_to indices aligned
        }
        for (size_t k = 0; k < node.sdo.size(); ++k)
        {
            const SdoWrite &w = node.sdo[k];
            if (w.on_abort == OnAbort::Jump && (w.jump_to <= k || w.jump_to >= node.sdo.size()))
                error(where + ": SDO " + std::to_string(k + 1),
                      "\"jump_to\" " + std::to_string(w.jump_to) +
                          " must be a later item of the node's list (" + std::to_string(k + 1) +
                          " to " + std::to_string(node.sdo.size() - 1) + ")");
        }
    }
    if (!node.configure && !node.sdo.empty())
        error(where, "has configuration SDOs but is not initialized (\"configure\": false)");

    const char *keys[2] = {"tpdos", "rpdos"};
    for (int dir = 0; dir < 2; ++dir)
    {
        const cJSON *pdos = get_array(j, keys[dir], where, false);
        if (!pdos)
            continue;
        std::vector<PdoConfig> &list = dir == 0 ? node.tpdos : node.rpdos;
        std::set<unsigned> numbers;
        const cJSON *p = nullptr;
        cJSON_ArrayForEach(p, pdos)
        {
            PdoConfig pdo;
            std::string w = where + (dir == 0 ? ": TPDO" : ": RPDO");
            if (!parse_pdo(p, w, dir == 0 ? IecArea::Input : IecArea::Output, pdo))
                continue;
            if (!numbers.insert(pdo.number).second)
                error(where, std::string(dir == 0 ? "TPDO " : "RPDO ") +
                                 std::to_string(pdo.number) + " is listed twice");
            list.push_back(std::move(pdo));
        }
    }
    return errors_.size() == failed;
}

void Parser::parse_lss(const cJSON *j, const std::string &where, NodeConfig &node)
{
    const std::string w = where + ": lss";
    if (!cJSON_IsObject(j))
    {
        error(where, "\"lss\" must be an object");
        return;
    }
    static const char *const kKeys[] = {"vendor_id", "product_code", "revision", "serial_number",
                                        "store"};
    const cJSON *item                = nullptr;
    cJSON_ArrayForEach(item, j)
    {
        bool known = false;
        for (const char *k : kKeys)
            known = known || std::strcmp(item->string, k) == 0;
        if (!known)
            error(w, std::string("unknown field \"") + item->string + "\"");
    }
    uint64_t v          = 0;
    const uint64_t zero = 0;
    if (get_uint(j, "vendor_id", w, 0, 0xFFFFFFFF, v))
        node.lss.vendor_id = static_cast<uint32_t>(v);
    if (get_uint(j, "product_code", w, 0, 0xFFFFFFFF, v))
        node.lss.product_code = static_cast<uint32_t>(v);
    if (get_uint(j, "revision", w, 0, 0xFFFFFFFF, v, &zero))
        node.lss.revision = static_cast<uint32_t>(v);
    if (get_uint(j, "serial_number", w, 0, 0xFFFFFFFF, v))
    {
        if (v == 0)
            error(w, "\"serial_number\" must not be 0");
        node.lss.serial_number = static_cast<uint32_t>(v);
    }
    get_bool(j, "store", w, node.lss.store);
    node.has_lss = true;
}

bool Parser::parse_sdo(const cJSON *j, const std::string &where, SdoWrite &sdo)
{
    if (!cJSON_IsObject(j))
    {
        error(where, "must be an object");
        return false;
    }
    uint64_t index = 0, sub = 0;
    std::string data;
    bool ok = get_uint(j, "index", where, 0, 0xFFFF, index) &
              get_uint(j, "subindex", where, 0, 0xFF, sub) &
              get_string(j, "data", where, data, true);
    if (!ok)
        return false;
    sdo.index     = static_cast<uint16_t>(index);
    sdo.subindex  = static_cast<uint8_t>(sub);
    std::string w = where + " " + object_text(sdo.index, sdo.subindex);
    if (sdo.index == 0x1010 || sdo.index == 0x1011)
    {
        error(w, sdo.index == 0x1010
                     ? "the master never writes 0x1010 (store parameters)"
                     : "0x1011 (restore defaults) is written only through the node's "
                       "\"restore_defaults\" setting");
        return false;
    }
    if (!parse_hex_bytes(data, sdo.data) || sdo.data.size() > kMaxSdoBytes)
    {
        error(w, "\"data\" must be 1 to 1024 bytes as hexadecimal digits, least significant "
                 "byte first");
        return false;
    }
    std::string policy;
    if (get_string(j, "on_abort", w, policy, false) && !policy.empty())
    {
        if (policy == "stop")
            sdo.on_abort = OnAbort::Stop;
        else if (policy == "continue")
            sdo.on_abort = OnAbort::Continue;
        else if (policy == "jump")
            sdo.on_abort = OnAbort::Jump;
        else
        {
            error(w, "\"on_abort\" \"" + policy + "\" is not one of stop, continue, jump");
            return false;
        }
    }
    const cJSON *jump = cJSON_GetObjectItemCaseSensitive(j, "jump_to");
    if (sdo.on_abort == OnAbort::Jump)
    {
        uint64_t to = 0;
        if (!get_uint(j, "jump_to", w, 0, 0xFFFF, to))
            return false;
        sdo.jump_to = static_cast<size_t>(to);
    }
    else if (jump)
    {
        error(w, "\"jump_to\" is only for \"on_abort\": \"jump\"");
        return false;
    }
    return true;
}

bool Parser::parse_pdo(const cJSON *j, const std::string &where, IecArea area, PdoConfig &pdo)
{
    if (!cJSON_IsObject(j))
    {
        error(where, "must be an object");
        return false;
    }
    uint64_t number = 0, cob = 0;
    if (!get_uint(j, "number", where, 1, kMaxPdoNumber, number))
        return false;
    pdo.number    = static_cast<unsigned>(number);
    std::string w = where + " " + std::to_string(pdo.number);
    size_t failed = errors_.size();
    if (get_uint(j, "cob_id", w, 1, 0x7FF, cob))
    {
        pdo.cob_id = static_cast<uint32_t>(cob);
        if (const char *what = reserved_cob_id(pdo.cob_id))
            error(w, "COB-ID " + hex(pdo.cob_id, 3) + " is reserved for " + what);
    }
    if (area == IecArea::Input)
    {
        static const uint64_t kNoTimeout = 0;
        uint64_t timeout                 = 0;
        if (get_uint(j, "receive_timeout_ms", w, 0, 65535, timeout, &kNoTimeout))
            pdo.receive_timeout_ms = static_cast<unsigned>(timeout);
    }
    else if (cJSON_GetObjectItemCaseSensitive(j, "receive_timeout_ms"))
    {
        error(w, "\"receive_timeout_ms\" is only for a node's TPDOs");
    }
    if (area == IecArea::Output)
    {
        uint64_t period = 0;
        if (cJSON_GetObjectItemCaseSensitive(j, "send_period_ms") &&
            get_uint(j, "send_period_ms", w, 1, 65535, period))
            pdo.send_period_ms = static_cast<unsigned>(period);
    }
    else if (cJSON_GetObjectItemCaseSensitive(j, "send_period_ms"))
    {
        error(w, "\"send_period_ms\" is only for a node's RPDOs");
    }
    const cJSON *entries = get_array(j, "entries", w, true);
    if (entries)
    {
        size_t pos     = 0;
        const cJSON *e = nullptr;
        cJSON_ArrayForEach(e, entries)
        {
            std::string ew = w + " entry " + std::to_string(++pos);
            if (!cJSON_IsObject(e))
            {
                error(ew, "must be an object");
                continue;
            }
            uint64_t index = 0, sub = 0;
            std::string type, loc;
            bool ok = get_uint(e, "index", ew, 1, 0xFFFF, index) &
                      get_uint(e, "subindex", ew, 0, 0xFF, sub) &
                      get_string(e, "type", ew, type, true);
            if (!ok)
                continue;
            PdoEntry entry;
            entry.index    = static_cast<uint16_t>(index);
            entry.subindex = static_cast<uint8_t>(sub);
            ew += " " + object_text(entry.index, entry.subindex);
            const bool has_location = cJSON_GetObjectItemCaseSensitive(e, "location") != nullptr;
            std::string why;
            if (!parse_co_type(type, entry.type))
                error(ew, "type \"" + type + "\" is not supported");
            else if (!has_location)
            {
                CoType dummy_type = CoType::UNSIGNED8;
                if (!dummy_object_type(entry.index, entry.subindex, dummy_type))
                    error(ew, "has no \"location\"; only a dummy object (0x0001-0x0007, "
                              "sub-index 0) may leave it out");
                else if (dummy_type != entry.type)
                    error(ew, std::string("dummy object ") + hex(entry.index, 4) + " is a " +
                                  co_type_name(dummy_type) + ", not a " + type);
                else
                {
                    entry.dummy = true;
                    pdo.entries.push_back(entry);
                }
            }
            else if (!get_string(e, "location", ew, loc, true))
                continue;
            else if (!parse_iec_location(loc, entry.location, why))
                error(ew, "location \"" + loc + "\": " + why);
            else if (entry.location.area != area)
                error(ew, std::string("location ") + loc + " must be " +
                              (area == IecArea::Input ? "an input (%I)" : "an output (%Q)"));
            else if (!co_type_fits(entry.type, entry.location.size))
                error(ew, std::string("a ") + co_type_name(entry.type) + " does not fit location " +
                              loc);
            else
                pdo.entries.push_back(entry);
        }
        if (pdo.bits() > kMaxPdoBits)
            error(w, "maps " + std::to_string(pdo.bits()) + " bits; a PDO holds at most 64");
    }
    return errors_.size() == failed;
}

void Parser::check_cob_ids(const NetworkConfig &net)
{
    std::map<uint32_t, std::string> used;
    auto use = [&](uint32_t cob, const std::string &what)
    {
        auto it = used.find(cob);
        if (it != used.end())
            error(net.label(), what + " and " + it->second + " use the same COB-ID " + hex(cob, 3));
        else
            used[cob] = what;
    };
    if (net.time_producer)
    {
        if (net.time_cob_id == 0x080 || (net.time_cob_id >= 0x581 && net.time_cob_id <= 0x67F) ||
            (net.time_cob_id >= 0x701 && net.time_cob_id <= 0x77F))
            error(net.label(), "TIME COB-ID " + hex(net.time_cob_id, 3) + " is reserved");
        use(net.time_cob_id, "TIME");
    }
    for (const auto &node : net.nodes)
        if (node.emcy_enabled)
            use(node.emcy_cob_id, node.label() + " emergency");
    for (const auto &node : net.nodes)
    {
        for (int dir = 0; dir < 2; ++dir)
        {
            for (const auto &p : dir == 0 ? node.tpdos : node.rpdos)
            {
                use(p.cob_id,
                    node.label() + (dir == 0 ? " TPDO " : " RPDO ") + std::to_string(p.number));
            }
        }
    }
}

void Parser::check_locations()
{
    struct Use
    {
        IecLocation loc;
        std::string what;
    };
    std::vector<Use> uses;
    for (const auto &net : cfg_.networks)
    {
        for (const auto &node : net.nodes)
        {
            std::string prefix = net.label() + ": " + node.label();
            if (node.has_state_location)
                uses.push_back({node.state_location, prefix + " state"});
            for (int dir = 0; dir < 2; ++dir)
                for (const auto &p : dir == 0 ? node.tpdos : node.rpdos)
                    for (const auto &e : p.entries)
                        if (!e.dummy)
                            uses.push_back({e.location, prefix + (dir == 0 ? " TPDO " : " RPDO ") +
                                                            std::to_string(p.number) + " " +
                                                            object_text(e.index, e.subindex)});
        }
    }
    for (const auto &dev : cfg_.devices)
        for (const auto &o : dev.objects)
        {
            if (o.exch == SlaveExchange::Constant)
                continue;
            const std::string what = dev.label() + " " + object_text(o.index, o.subindex);
            if (!o.packed())
                uses.push_back({o.location, what});
            for (size_t k = 0; k < o.bits.size(); ++k)
                uses.push_back({o.bits[k], what + " bit " + std::to_string(k)});
        }
    for (size_t a = 0; a < uses.size(); ++a)
    {
        for (size_t b = a + 1; b < uses.size(); ++b)
        {
            if (!uses[a].loc.overlaps(uses[b].loc))
                continue;
            std::string text =
                uses[a].what + " and " + uses[b].what + " both use " + uses[a].loc.str();
            if (uses[a].loc.area == IecArea::Input)
                error("", text);
            else
                warn("", text);
        }
    }
}

bool Parser::parse_value(const cJSON *v, const std::string &where, const char *key, CoType type,
                         uint64_t &out)
{
    if (type == CoType::BOOLEAN && cJSON_IsBool(v))
    {
        out = cJSON_IsTrue(v) ? 1 : 0;
        return true;
    }
    if (!cJSON_IsNumber(v))
    {
        error(where, std::string("\"") + key + "\" must be a number");
        return false;
    }
    const double d = v->valuedouble;
    auto bad       = [&](const char *what)
    {
        error(where, std::string("\"") + key + "\" " + what + " for a " + co_type_name(type));
        return false;
    };
    switch (type)
    {
    case CoType::REAL32:
    {
        float f = static_cast<float>(d);
        uint32_t bits;
        std::memcpy(&bits, &f, sizeof(bits));
        out = bits;
        return true;
    }
    case CoType::REAL64:
    {
        uint64_t bits;
        std::memcpy(&bits, &d, sizeof(bits));
        out = bits;
        return true;
    }
    default:
        break;
    }
    if (std::floor(d) != d)
        return bad("must be a whole number");
    const unsigned bits  = co_type_bits(type);
    const bool is_signed = type == CoType::INTEGER8 || type == CoType::INTEGER16 ||
                           type == CoType::INTEGER32 || type == CoType::INTEGER64;
    if (type == CoType::BOOLEAN)
    {
        if (d != 0 && d != 1)
            return bad("must be 0 or 1");
        out = static_cast<uint64_t>(d);
        return true;
    }
    if (is_signed)
    {
        const double lo = -std::ldexp(1.0, static_cast<int>(bits) - 1);
        const double hi = std::ldexp(1.0, static_cast<int>(bits) - 1) - 1;
        if (d < lo || d > hi)
            return bad("is out of range");
        const int64_t s = static_cast<int64_t>(d);
        out             = bits == 64 ? static_cast<uint64_t>(s)
                                     : (static_cast<uint64_t>(s) & ((uint64_t(1) << bits) - 1));
        return true;
    }
    const double hi = std::ldexp(1.0, static_cast<int>(bits)) - 1;
    if (d < 0 || d > hi)
        return bad("is out of range");
    out = static_cast<uint64_t>(d);
    return true;
}

bool Parser::parse_object(const cJSON *j, const std::string &where, SlaveObject &obj)
{
    if (!cJSON_IsObject(j))
    {
        error(where, "must be an object");
        return false;
    }
    uint64_t index = 0, sub = 0;
    std::string type, dir, exch;
    bool ok =
        get_uint(j, "index", where, 0x2000, 0x5FFF, index) &
        get_uint(j, "subindex", where, 1, 254, sub) & get_string(j, "type", where, type, true) &
        get_string(j, "direction", where, dir, true) & get_string(j, "exchange", where, exch, true);
    get_string(j, "name", where, obj.name, false);
    if (!ok)
        return false;
    obj.index     = static_cast<uint16_t>(index);
    obj.subindex  = static_cast<uint8_t>(sub);
    std::string w = where + " " + object_text(obj.index, obj.subindex);
    size_t failed = errors_.size();
    if (dir == "from_master")
        obj.dir = SlaveDirection::FromMaster;
    else if (dir == "to_master")
        obj.dir = SlaveDirection::ToMaster;
    else
        error(w, "\"direction\" \"" + dir + "\" is not one of from_master, to_master");
    if (exch == "pdo")
        obj.exch = SlaveExchange::Pdo;
    else if (exch == "sdo")
        obj.exch = SlaveExchange::Sdo;
    else if (exch == "constant")
        obj.exch = SlaveExchange::Constant;
    else
        error(w, "\"exchange\" \"" + exch + "\" is not one of pdo, sdo, constant");
    if (type == "VISIBLE_STRING")
        obj.string = true;
    else if (!parse_co_type(type, obj.type))
        error(w, "type \"" + type + "\" is not supported");
    if (errors_.size() != failed)
        return false;

    const cJSON *loc = cJSON_GetObjectItemCaseSensitive(j, "location");
    if (obj.exch == SlaveExchange::Constant)
    {
        if (loc)
            error(w, "a constant has no \"location\"");
        const cJSON *value = cJSON_GetObjectItemCaseSensitive(j, "value");
        if (!value)
            error(w, "a constant needs a \"value\"");
        else if (obj.string)
        {
            if (!cJSON_IsString(value))
                error(w, "\"value\" of a VISIBLE_STRING must be a string");
            else if (std::strlen(value->valuestring) > 255)
                error(w, "\"value\" is longer than 255 characters");
            else
                obj.text = value->valuestring;
        }
        else
            parse_value(value, w, "value", obj.type, obj.value);
        if (cJSON_GetObjectItemCaseSensitive(j, "min") ||
            cJSON_GetObjectItemCaseSensitive(j, "max"))
            error(w, "a constant has no \"min\" or \"max\"");
        if (cJSON_GetObjectItemCaseSensitive(j, "bits"))
            error(w, "a constant has no \"bits\"");
        return errors_.size() == failed;
    }
    if (obj.string)
    {
        error(w, "a VISIBLE_STRING object must be a constant");
        return false;
    }
    std::string text, why;
    const IecArea area = obj.dir == SlaveDirection::FromMaster ? IecArea::Input : IecArea::Output;
    if (const cJSON *bits = cJSON_GetObjectItemCaseSensitive(j, "bits"))
    {
        parse_packed_bits(j, bits, w, area, obj);
        return errors_.size() == failed;
    }
    if (!get_string(j, "location", w, text, true))
        return false;
    if (!parse_iec_location(text, obj.location, why))
        error(w, "location \"" + text + "\": " + why);
    else if (obj.location.area != area)
        error(w, std::string("location ") + text + " must be " +
                     (area == IecArea::Input ? "an input (%I) for an object from the master"
                                             : "an output (%Q) for an object to the master"));
    else if (!co_type_fits(obj.type, obj.location.size))
        error(w, std::string("a ") + co_type_name(obj.type) + " does not fit location " + text);
    if (const cJSON *def = cJSON_GetObjectItemCaseSensitive(j, "default"))
        parse_value(def, w, "default", obj.type, obj.value);
    const cJSON *mn = cJSON_GetObjectItemCaseSensitive(j, "min");
    const cJSON *mx = cJSON_GetObjectItemCaseSensitive(j, "max");
    if ((mn || mx) && obj.dir != SlaveDirection::FromMaster)
        error(w, "only an object from the master has \"min\" or \"max\"");
    else
    {
        if (mn)
            obj.has_min = parse_value(mn, w, "min", obj.type, obj.min);
        if (mx)
            obj.has_max = parse_value(mx, w, "max", obj.type, obj.max);
        if (obj.has_min && obj.has_max && compare_raw(obj.type, obj.min, obj.max) > 0)
            error(w, "\"min\" is above \"max\"");
        else if ((obj.has_min && compare_raw(obj.type, obj.value, obj.min) < 0) ||
                 (obj.has_max && compare_raw(obj.type, obj.value, obj.max) > 0))
            error(w, "the default is outside \"min\" to \"max\"");
    }
    return errors_.size() == failed;
}

void Parser::parse_packed_bits(const cJSON *j, const cJSON *bits, const std::string &w,
                               IecArea area, SlaveObject &obj)
{
    if (obj.type != CoType::UNSIGNED8)
        error(w, std::string("\"bits\" needs type UNSIGNED8, not ") + co_type_name(obj.type));
    if (cJSON_GetObjectItemCaseSensitive(j, "location"))
        error(w, "an object with \"bits\" has no \"location\"");
    if (cJSON_GetObjectItemCaseSensitive(j, "min") || cJSON_GetObjectItemCaseSensitive(j, "max"))
        error(w, "an object with \"bits\" has no \"min\" or \"max\"");
    if (!cJSON_IsArray(bits) || cJSON_GetArraySize(bits) != 8)
    {
        error(w, "\"bits\" must list 8 locations");
        return;
    }
    const char *want = area == IecArea::Input ? "%IX" : "%QX";
    const cJSON *b   = nullptr;
    cJSON_ArrayForEach(b, bits)
    {
        IecLocation loc;
        std::string why;
        if (!cJSON_IsString(b))
        {
            error(w, "\"bits\" must list 8 locations");
            return;
        }
        const std::string text = b->valuestring;
        if (!parse_iec_location(text, loc, why))
            error(w, "bit location \"" + text + "\": " + why);
        else if (loc.area != area || loc.size != IecSize::X)
            error(w, "bit location " + text + " must be a " + want + " location");
        else
        {
            for (const auto &other : obj.bits)
                if (other.overlaps(loc))
                    error(w, "bit location " + text + " is given twice");
            obj.bits.push_back(loc);
        }
    }
    if (obj.bits.size() != 8)
        obj.bits.clear();
    if (const cJSON *def = cJSON_GetObjectItemCaseSensitive(j, "default"))
        parse_value(def, w, "default", obj.type, obj.value);
}

void Parser::parse_slave_pdos(const cJSON *j, const char *key, const std::string &where, bool tpdo,
                              DeviceConfig &dev)
{
    const cJSON *list = get_array(j, key, where, false);
    if (!list)
        return;
    std::vector<SlavePdo> &out = tpdo ? dev.tpdos : dev.rpdos;
    const char *kind           = tpdo ? "TPDO" : "RPDO";
    const cJSON *p             = nullptr;
    std::set<const SlaveObject *> mapped;
    cJSON_ArrayForEach(p, list)
    {
        if (!cJSON_IsObject(p))
        {
            error(where, std::string(kind) + " must be an object");
            continue;
        }
        SlavePdo pdo;
        uint64_t v = 0;
        if (!get_uint(p, "number", where + " " + kind, 1, kSlavePdos, v))
            continue;
        pdo.number          = static_cast<unsigned>(v);
        const std::string w = where + " " + kind + " " + std::to_string(pdo.number);
        for (const auto &o : out)
            if (o.number == pdo.number)
                error(w, "is given twice");
        if (get_uint(p, "cob_id", w, 1, 0x7FF, v))
        {
            pdo.cob_id = static_cast<uint32_t>(v);
            if (const char *what = reserved_cob_id(pdo.cob_id))
                error(w, "COB-ID " + hex(pdo.cob_id, 3) + " is reserved for " + what);
        }
        const uint64_t def_tx = 255, zero = 0;
        if (get_uint(p, "transmission", w, 0, 255, v, &def_tx))
        {
            if (v > 240 && v < 254)
                error(w, "transmission type " + std::to_string(v) + " is reserved");
            else
                pdo.transmission = static_cast<unsigned>(v);
        }
        if (get_uint(p, "event_timer_ms", w, 0, 65535, v, &zero))
            pdo.event_timer_ms = static_cast<unsigned>(v);
        if (tpdo && get_uint(p, "inhibit_100us", w, 0, 65535, v, &zero))
            pdo.inhibit_100us = static_cast<unsigned>(v);
        else if (!tpdo && cJSON_GetObjectItemCaseSensitive(p, "inhibit_100us"))
            error(w, "\"inhibit_100us\" is only for a TPDO");
        const cJSON *entries = get_array(p, "entries", w, true);
        unsigned bits        = 0;
        const cJSON *e       = nullptr;
        cJSON_ArrayForEach(e, entries)
        {
            uint64_t index = 0, sub = 0;
            if (!cJSON_IsObject(e) || !(get_uint(e, "index", w, 1, 0xFFFF, index) &
                                        get_uint(e, "subindex", w, 0, 0xFF, sub)))
                continue;
            SlavePdoEntry entry{static_cast<uint16_t>(index), static_cast<uint8_t>(sub)};
            const std::string ew   = w + " " + object_text(entry.index, entry.subindex);
            const SlaveObject *obj = dev.find(entry.index, entry.subindex);
            const SlaveDirection want =
                tpdo ? SlaveDirection::ToMaster : SlaveDirection::FromMaster;
            if (!obj)
                error(ew, "is not one of the slave's objects");
            else if (obj->exch != SlaveExchange::Pdo)
                error(ew, "is not exchanged by PDO");
            else if (obj->dir != want)
                error(ew,
                      tpdo ? "is not an object to the master" : "is not an object from the master");
            else if (!mapped.insert(obj).second)
                error(ew, "is mapped twice");
            else
            {
                bits += co_type_bits(obj->type) < 8 ? 8 : co_type_bits(obj->type);
                pdo.entries.push_back(entry);
            }
        }
        if (bits > kMaxPdoBits)
            error(w, "maps " + std::to_string(bits / 8) + " bytes; a PDO holds at most 8");
        out.push_back(std::move(pdo));
    }
}

void Parser::parse_slave_lss(const cJSON *j, const std::string &where, SlaveLss &lss)
{
    const cJSON *o = cJSON_GetObjectItemCaseSensitive(j, "lss");
    if (!o || cJSON_IsNull(o))
        return;
    if (!cJSON_IsObject(o))
    {
        error(where, "\"lss\" must be an object or null");
        return;
    }
    const std::string w = where + ": lss";
    for (const cJSON *item = o->child; item; item = item->next)
    {
        const std::string key = item->string ? item->string : "";
        if (key != "enabled" && key != "wait_for_master" && key != "serial" &&
            key != "bitrate_change")
            error(w, "unknown field \"" + key + "\"");
    }
    get_bool(o, "enabled", w, lss.enabled);
    get_bool(o, "wait_for_master", w, lss.wait_for_master);
    get_bool(o, "bitrate_change", w, lss.bitrate_change);
    uint64_t v = 0;
    if (cJSON_GetObjectItemCaseSensitive(o, "serial") && get_uint(o, "serial", w, 1, 0xFFFFFFFF, v))
        lss.serial = static_cast<uint32_t>(v);
}

bool Parser::parse_device(const cJSON *j, size_t pos, DeviceConfig &dev)
{
    std::string where = "device " + std::to_string(pos);
    if (!cJSON_IsObject(j))
    {
        error(where, "must be an object");
        return false;
    }
    if (char *text = cJSON_PrintUnformatted(j))
    {
        dev.source = text;
        cJSON_free(text);
    }
    get_string(j, "name", where, dev.name, false);
    get_string(j, "interface", where, dev.interface, true);
    if (!dev.interface.empty() || !dev.name.empty())
        where = dev.label();
    if (dev.interface.size() >= 16)
        error(where, "interface name \"" + dev.interface + "\" is longer than 15 characters");
    uint64_t v          = 0;
    const uint64_t zero = 0;
    if (get_uint(j, "bitrate", where, 0, 1000000, v, &zero))
    {
        dev.bitrate = static_cast<unsigned>(v);
        if (v && !is_standard_bitrate(dev.bitrate))
            error(where, "bit rate " + std::to_string(v) + " is not a CANopen bit rate");
    }
    if (get_uint(j, "node_id", where, 1, kMaxNodeId, v))
        dev.node_id = static_cast<unsigned>(v);
    const uint64_t def_priority = kDefaultPriority;
    if (get_uint(j, "priority", where, 1, kMaxPriority, v, &def_priority))
        dev.priority = static_cast<int>(v);
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(j, "identity");
    if (!cJSON_IsObject(id))
        error(where, "missing \"identity\" object");
    else
    {
        const std::string w = where + ": identity";
        get_string(id, "device_name", w, dev.device_name, false);
        get_string(id, "software_version", w, dev.software_version, false);
        if (dev.device_name.size() > 255 || dev.software_version.size() > 255)
            error(w, "names are at most 255 characters");
        if (get_uint(id, "vendor_id", w, 0, 0xFFFFFFFF, v, &zero))
            dev.vendor_id = static_cast<uint32_t>(v);
        if (get_uint(id, "product_code", w, 0, 0xFFFFFFFF, v, &zero))
            dev.product_code = static_cast<uint32_t>(v);
        if (get_uint(id, "revision", w, 0, 0xFFFFFFFF, v, &zero))
            dev.revision = static_cast<uint32_t>(v);
    }
    if (get_uint(j, "heartbeat_ms", where, 1, 65535, v))
        dev.heartbeat_ms = static_cast<unsigned>(v);
    const cJSON *watch = cJSON_GetObjectItemCaseSensitive(j, "master_watch");
    if (watch && !cJSON_IsNull(watch))
    {
        const std::string w = where + ": master_watch";
        if (!cJSON_IsObject(watch))
            error(where, "\"master_watch\" must be an object or null");
        else
        {
            if (get_uint(watch, "node_id", w, 1, kMaxNodeId, v))
                dev.master_node_id = static_cast<unsigned>(v);
            if (get_uint(watch, "time_ms", w, 0, 65535, v))
                dev.master_watch_ms = static_cast<unsigned>(v);
            if (dev.master_node_id && dev.master_node_id == dev.node_id)
                error(w, "the master's node ID is the slave's own");
        }
    }
    std::string text;
    if (get_string(j, "on_master_lost", where, text, false) && !text.empty())
    {
        if (text == "pre_operational")
            dev.on_master_lost = OnMasterLost::PreOperational;
        else if (text == "no_change")
            dev.on_master_lost = OnMasterLost::NoChange;
        else if (text == "stopped")
            dev.on_master_lost = OnMasterLost::Stopped;
        else
            error(where, "\"on_master_lost\" \"" + text +
                             "\" is not one of pre_operational, no_change, stopped");
    }
    text.clear();
    if (get_string(j, "on_plc_stop", where, text, false) && !text.empty())
    {
        if (text == "hold")
            dev.on_plc_stop = SlaveOnStop::Hold;
        else if (text == "zero")
            dev.on_plc_stop = SlaveOnStop::Zero;
        else
            error(where, "\"on_plc_stop\" \"" + text + "\" is not one of hold, zero");
    }
    get_bool(j, "start_alone", where, dev.start_alone);
    parse_slave_lss(j, where, dev.lss);

    const cJSON *objects = get_array(j, "objects", where, true);
    const cJSON *o       = nullptr;
    size_t opos          = 0;
    cJSON_ArrayForEach(o, objects)
    {
        SlaveObject obj;
        if (!parse_object(o, where + ": object " + std::to_string(++opos), obj))
            continue;
        if (dev.find(obj.index, obj.subindex))
            error(where, object_text(obj.index, obj.subindex) + " is given twice");
        else
            dev.objects.push_back(std::move(obj));
    }
    parse_slave_pdos(j, "rpdos", where, false, dev);
    parse_slave_pdos(j, "tpdos", where, true, dev);
    std::map<uint32_t, std::string> cobs;
    for (int dir = 0; dir < 2; ++dir)
        for (const auto &p : dir ? dev.tpdos : dev.rpdos)
        {
            std::string what = std::string(dir ? "TPDO " : "RPDO ") + std::to_string(p.number);
            auto it          = cobs.find(p.cob_id);
            if (it != cobs.end())
                error(where,
                      what + " and " + it->second + " use the same COB-ID " + hex(p.cob_id, 3));
            else
                cobs[p.cob_id] = what;
        }
    return true;
}

} // namespace

unsigned PdoConfig::bits() const
{
    unsigned n = 0;
    for (const auto &e : entries)
        n += co_type_bits(e.type);
    return n;
}

unsigned SlaveObject::bytes() const
{
    if (string)
        return static_cast<unsigned>(text.size());
    const unsigned bits = co_type_bits(type);
    return bits < 8 ? 1 : bits / 8;
}

std::string DeviceConfig::label() const
{
    std::string text = "slave";
    if (!name.empty())
        text += " \"" + name + "\"";
    if (!interface.empty())
        text += " (" + interface + ")";
    return text;
}

const SlaveObject *DeviceConfig::find(uint16_t index, uint8_t subindex) const
{
    for (const auto &o : objects)
        if (o.index == index && o.subindex == subindex)
            return &o;
    return nullptr;
}

int compare_raw(CoType type, uint64_t a, uint64_t b)
{
    auto sign_extend = [](uint64_t v, unsigned bits)
    {
        if (bits >= 64)
            return static_cast<int64_t>(v);
        const uint64_t m = uint64_t(1) << (bits - 1);
        v &= (uint64_t(1) << bits) - 1;
        return static_cast<int64_t>((v ^ m) - m);
    };
    switch (type)
    {
    case CoType::INTEGER8:
    case CoType::INTEGER16:
    case CoType::INTEGER32:
    case CoType::INTEGER64:
    {
        const int64_t x = sign_extend(a, co_type_bits(type));
        const int64_t y = sign_extend(b, co_type_bits(type));
        return x < y ? -1 : (x > y ? 1 : 0);
    }
    case CoType::REAL32:
    {
        float x, y;
        uint32_t ua = static_cast<uint32_t>(a), ub = static_cast<uint32_t>(b);
        std::memcpy(&x, &ua, sizeof(x));
        std::memcpy(&y, &ub, sizeof(y));
        return x < y ? -1 : (x > y ? 1 : 0);
    }
    case CoType::REAL64:
    {
        double x, y;
        std::memcpy(&x, &a, sizeof(x));
        std::memcpy(&y, &b, sizeof(y));
        return x < y ? -1 : (x > y ? 1 : 0);
    }
    default:
        return a < b ? -1 : (a > b ? 1 : 0);
    }
}

std::string NodeConfig::label() const
{
    std::string s = "node " + std::to_string(node_id);
    if (!name.empty())
        s += " (" + name + ")";
    return s;
}

std::string NetworkConfig::label() const
{
    if (name.empty())
        return "network " + interface;
    return "network \"" + name + "\" (" + interface + ")";
}

bool is_standard_bitrate(unsigned bitrate)
{
    switch (bitrate)
    {
    case 10000:
    case 20000:
    case 50000:
    case 125000:
    case 250000:
    case 500000:
    case 800000:
    case 1000000:
        return true;
    default:
        return false;
    }
}

bool parse_config(const std::string &text, Config &cfg, std::vector<std::string> &errors)
{
    cfg         = Config();
    cJSON *root = cJSON_ParseWithLength(text.data(), text.size());
    if (!root)
    {
        errors.push_back("the configuration is not valid JSON");
        return false;
    }
    Parser p(cfg, errors);
    bool ok = p.parse(root);
    cJSON_Delete(root);
    return ok;
}

bool load_config(const std::string &path, Config &cfg, std::vector<std::string> &errors)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        errors.push_back("cannot read " + path);
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse_config(ss.str(), cfg, errors);
}

} // namespace canopen_plugin
