// SPDX-License-Identifier: Apache-2.0

#include "config.h"

#include <cmath>
#include <cstdio>
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
    bool parse_sdo(const cJSON *j, const std::string &where, SdoWrite &sdo);
    bool parse_pdo(const cJSON *j, const std::string &where, IecArea area, PdoConfig &pdo);
    void check_cob_ids(const NetworkConfig &net);
    void check_locations();

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
    if (cfg_.networks.empty())
        error("", "\"networks\" is empty");
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
    const uint64_t def_sdo_timeout = 1000;
    if (get_uint(j, "sdo_timeout_ms", where, 10, 60000, v, &def_sdo_timeout))
        net.sdo_timeout_ms = static_cast<unsigned>(v);

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
    where         = net.label() + ": " + node.label();
    size_t failed = errors_.size();

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
            else
                error(where, "supervision type \"" + type + "\" is not one of none, heartbeat");
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
        }
    }

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
        error(w, "the master never writes 0x1010 (store) or 0x1011 (restore defaults)");
        return false;
    }
    if (!parse_hex_bytes(data, sdo.data) || sdo.data.size() > kMaxSdoBytes)
    {
        error(w, "\"data\" must be 1 to 1024 bytes as hexadecimal digits, least significant "
                 "byte first");
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
                      get_string(e, "type", ew, type, true) &
                      get_string(e, "location", ew, loc, true);
            if (!ok)
                continue;
            PdoEntry entry;
            entry.index    = static_cast<uint16_t>(index);
            entry.subindex = static_cast<uint8_t>(sub);
            ew += " " + object_text(entry.index, entry.subindex);
            std::string why;
            if (!parse_co_type(type, entry.type))
                error(ew, "type \"" + type + "\" is not supported");
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
    for (const auto &node : net.nodes)
    {
        for (int dir = 0; dir < 2; ++dir)
        {
            for (const auto &p : dir == 0 ? node.tpdos : node.rpdos)
            {
                std::string what =
                    node.label() + (dir == 0 ? " TPDO " : " RPDO ") + std::to_string(p.number);
                auto it = used.find(p.cob_id);
                if (it != used.end())
                    error(net.label(),
                          what + " and " + it->second + " use the same COB-ID " + hex(p.cob_id, 3));
                else
                    used[p.cob_id] = what;
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
                        uses.push_back({e.location, prefix + (dir == 0 ? " TPDO " : " RPDO ") +
                                                        std::to_string(p.number) + " " +
                                                        object_text(e.index, e.subindex)});
        }
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

} // namespace

unsigned PdoConfig::bits() const
{
    unsigned n = 0;
    for (const auto &e : entries)
        n += co_type_bits(e.type);
    return n;
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
