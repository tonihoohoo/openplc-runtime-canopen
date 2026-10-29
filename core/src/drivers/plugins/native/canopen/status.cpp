// SPDX-License-Identifier: Apache-2.0

#include "status.h"

#include <chrono>
#include <cstdio>
#include <mutex>

#include "cJSON.h"

namespace canopen_plugin
{

constexpr size_t NetworkStatus::kMaxEmcy;
constexpr size_t NetworkStatus::kMaxEvents;

bool Problem::operator==(const Problem &o) const
{
    return kind == o.kind && field == o.field && expected == o.expected &&
           has_found == o.has_found && found == o.found && index == o.index &&
           subindex == o.subindex && abort_code == o.abort_code && boot_status == o.boot_status &&
           text == o.text;
}

bool NodeStatus::warning() const
{
    if (state != NodeRun::Operational)
        return false;
    if (!late.empty())
        return true;
    for (const Emcy &e : emcy)
        if (e.active)
            return true;
    return false;
}

bool NetworkSnapshot::all_nodes_up() const
{
    for (const NodeStatus &n : nodes)
        if (n.state != NodeRun::Operational)
            return false;
    return true;
}

int64_t wall_ms()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

NetworkStatus::NetworkStatus(const NetworkConfig &cfg)
{
    data_.name      = cfg.name;
    data_.interface = cfg.interface;
    int64_t now     = wall_ms();
    for (const NodeConfig &n : cfg.nodes)
    {
        NodeStatus s;
        s.node_id = n.node_id;
        s.name    = n.name;
        s.since   = now;
        data_.nodes.push_back(s);
    }
}

NodeStatus *NetworkStatus::find(unsigned node)
{
    for (NodeStatus &n : data_.nodes)
        if (n.node_id == node)
            return &n;
    return nullptr;
}

void NetworkStatus::change(NodeStatus &n, NodeRun state)
{
    if (n.state == state)
        return;
    n.state = state;
    n.since = wall_ms();
}

void NetworkStatus::push(Event e)
{
    e.time = wall_ms();
    data_.events.push_front(e);
    if (data_.events.size() > kMaxEvents)
        data_.events.pop_back();
}

void NetworkStatus::begin_session()
{
    std::lock_guard<PiMutex> lock(mutex_);
    for (NodeStatus &n : data_.nodes)
    {
        change(n, NodeRun::Waiting);
        n.nmt = NmtSeen::Unknown;
        n.late.clear();
    }
}

void NetworkStatus::set_state(unsigned node, NodeRun state)
{
    std::lock_guard<PiMutex> lock(mutex_);
    if (NodeStatus *n = find(node))
        change(*n, state);
}

void NetworkStatus::set_nmt(unsigned node, NmtSeen nmt)
{
    std::lock_guard<PiMutex> lock(mutex_);
    if (NodeStatus *n = find(node))
        n->nmt = nmt;
}

void NetworkStatus::silent(unsigned node)
{
    std::lock_guard<PiMutex> lock(mutex_);
    NodeStatus *n = find(node);
    if (!n)
        return;
    change(*n, NodeRun::Silent);
    Event e;
    e.node = node;
    e.kind = EventKind::Silent;
    push(e);
}

void NetworkStatus::boot_ok(unsigned node)
{
    std::lock_guard<PiMutex> lock(mutex_);
    NodeStatus *n = find(node);
    if (!n)
        return;
    ++n->boots;
    n->problem = Problem();
    change(*n, NodeRun::Operational);
    n->nmt = NmtSeen::Operational;
    Event e;
    e.node = node;
    e.kind = EventKind::BootOk;
    push(e);
}

void NetworkStatus::boot_failed(unsigned node, const Problem &p)
{
    std::lock_guard<PiMutex> lock(mutex_);
    NodeStatus *n = find(node);
    if (!n)
        return;
    ++n->boots;
    n->problem = p;
    change(*n, NodeRun::Failed);
    Event e;
    e.node    = node;
    e.kind    = EventKind::BootFailed;
    e.problem = p;
    push(e);
}

void NetworkStatus::identity_found(unsigned node, IdentityField field, uint32_t value)
{
    std::lock_guard<PiMutex> lock(mutex_);
    NodeStatus *n = find(node);
    if (!n || n->problem.kind != ProblemKind::Identity || n->problem.field != field)
        return;
    n->problem.has_found = true;
    n->problem.found     = value;
    // The failure's event gets the value too.
    for (Event &e : data_.events)
    {
        if (e.node == node && e.kind == EventKind::BootFailed)
        {
            if (e.problem.kind == ProblemKind::Identity && e.problem.field == field)
            {
                e.problem.has_found = true;
                e.problem.found     = value;
            }
            break;
        }
    }
}

void NetworkStatus::lost(unsigned node, const Problem &p)
{
    std::lock_guard<PiMutex> lock(mutex_);
    NodeStatus *n = find(node);
    if (!n)
        return;
    ++n->losses;
    n->problem = p;
    n->late.clear();
    change(*n, NodeRun::Lost);
    Event e;
    e.node    = node;
    e.kind    = EventKind::Lost;
    e.problem = p;
    push(e);
}

void NetworkStatus::sdo_abort(unsigned node, uint16_t index, uint8_t subindex, uint32_t code)
{
    std::lock_guard<PiMutex> lock(mutex_);
    Event e;
    e.node               = node;
    e.kind               = EventKind::SdoAbort;
    e.problem.kind       = ProblemKind::SdoAbort;
    e.problem.index      = index;
    e.problem.subindex   = subindex;
    e.problem.abort_code = code;
    push(e);
}

void NetworkStatus::emcy(unsigned node, uint16_t code, uint8_t reg, const uint8_t data[5])
{
    std::lock_guard<PiMutex> lock(mutex_);
    NodeStatus *n = find(node);
    if (!n)
        return;
    Event e;
    e.node = node;
    e.code = code;
    e.reg  = reg;
    if (code == 0)
    {
        for (Emcy &m : n->emcy)
            m.active = false;
        e.kind = EventKind::EmcyReset;
        push(e);
        return;
    }
    Emcy m;
    m.time = wall_ms();
    m.code = code;
    m.reg  = reg;
    for (int i = 0; i < 5; ++i)
        m.data[i] = data[i];
    n->emcy.insert(n->emcy.begin(), m);
    if (n->emcy.size() > kMaxEmcy)
        n->emcy.pop_back();
    e.kind = EventKind::Emcy;
    push(e);
}

void NetworkStatus::pdo_late(unsigned node, unsigned pdo, uint32_t cob_id)
{
    std::lock_guard<PiMutex> lock(mutex_);
    NodeStatus *n = find(node);
    if (!n)
        return;
    for (const LatePdo &l : n->late)
        if (l.pdo == pdo)
            return;
    LatePdo l;
    l.pdo    = pdo;
    l.cob_id = cob_id;
    l.since  = wall_ms();
    n->late.push_back(l);
    Event e;
    e.node   = node;
    e.kind   = EventKind::PdoLate;
    e.pdo    = pdo;
    e.cob_id = cob_id;
    push(e);
}

void NetworkStatus::pdo_back(unsigned node, unsigned pdo)
{
    std::lock_guard<PiMutex> lock(mutex_);
    NodeStatus *n = find(node);
    if (!n)
        return;
    for (auto it = n->late.begin(); it != n->late.end(); ++it)
    {
        if (it->pdo != pdo)
            continue;
        Event e;
        e.node   = node;
        e.kind   = EventKind::PdoBack;
        e.pdo    = pdo;
        e.cob_id = it->cob_id;
        n->late.erase(it);
        push(e);
        return;
    }
}

void NetworkStatus::pdo_length(unsigned node, unsigned pdo, uint32_t cob_id, uint16_t code)
{
    std::lock_guard<PiMutex> lock(mutex_);
    Event e;
    e.node   = node;
    e.kind   = EventKind::PdoLength;
    e.pdo    = pdo;
    e.cob_id = cob_id;
    e.code   = code;
    push(e);
}

void NetworkStatus::clear_late(unsigned node)
{
    std::lock_guard<PiMutex> lock(mutex_);
    if (NodeStatus *n = find(node))
        n->late.clear();
}

void NetworkStatus::set_bus(const BusStatus &bus)
{
    std::lock_guard<PiMutex> lock(mutex_);
    bool changed     = bus.state != data_.bus.state;
    uint32_t resets  = data_.bus.resets;
    data_.bus        = bus;
    data_.bus.resets = resets;
    if (!changed)
        return;
    Event e;
    e.kind = EventKind::BusChange;
    e.bus  = bus.state;
    push(e);
}

void NetworkStatus::link_reset(ResetOutcome outcome)
{
    std::lock_guard<PiMutex> lock(mutex_);
    if (outcome == ResetOutcome::Done)
        ++data_.bus.resets;
    Event e;
    e.kind  = EventKind::BusReset;
    e.reset = outcome;
    push(e);
}

void NetworkStatus::stop_all()
{
    std::lock_guard<PiMutex> lock(mutex_);
    for (NodeStatus &n : data_.nodes)
    {
        change(n, NodeRun::Stopped);
        n.late.clear();
    }
}

NetworkSnapshot NetworkStatus::snapshot() const
{
    std::lock_guard<PiMutex> lock(mutex_);
    return data_;
}

const char *node_run_name(NodeRun s)
{
    switch (s)
    {
    case NodeRun::Waiting:
        return "waiting";
    case NodeRun::Silent:
        return "silent";
    case NodeRun::Booting:
        return "booting";
    case NodeRun::Failed:
        return "failed";
    case NodeRun::Operational:
        return "operational";
    case NodeRun::Lost:
        return "lost";
    case NodeRun::Stopped:
        return "stopped";
    }
    return "waiting";
}

const char *bus_state_name(BusState s)
{
    switch (s)
    {
    case BusState::Unknown:
        return "unknown";
    case BusState::ErrorActive:
        return "error_active";
    case BusState::ErrorWarning:
        return "error_warning";
    case BusState::ErrorPassive:
        return "error_passive";
    case BusState::BusOff:
        return "bus_off";
    case BusState::Stopped:
        return "stopped";
    case BusState::Down:
        return "down";
    case BusState::Missing:
        return "missing";
    }
    return "unknown";
}

namespace
{

const char *nmt_name(NmtSeen s)
{
    switch (s)
    {
    case NmtSeen::Unknown:
        return "unknown";
    case NmtSeen::PreOperational:
        return "pre_operational";
    case NmtSeen::Operational:
        return "operational";
    case NmtSeen::Stopped:
        return "stopped";
    }
    return "unknown";
}

const char *field_name(IdentityField f)
{
    switch (f)
    {
    case IdentityField::DeviceType:
        return "device_type";
    case IdentityField::VendorId:
        return "vendor_id";
    case IdentityField::ProductCode:
        return "product_code";
    case IdentityField::Revision:
        return "revision";
    case IdentityField::SerialNumber:
        return "serial_number";
    }
    return "device_type";
}

const char *problem_name(ProblemKind k)
{
    switch (k)
    {
    case ProblemKind::None:
        return "none";
    case ProblemKind::NoResponse:
        return "no_response";
    case ProblemKind::Identity:
        return "identity";
    case ProblemKind::SdoAbort:
        return "sdo_abort";
    case ProblemKind::Heartbeat:
        return "heartbeat";
    case ProblemKind::Reset:
        return "reset";
    case ProblemKind::BootError:
        return "boot_error";
    }
    return "none";
}

const char *reset_outcome_name(ResetOutcome r)
{
    switch (r)
    {
    case ResetOutcome::Done:
        return "done";
    case ResetOutcome::NotPermitted:
        return "not_permitted";
    case ResetOutcome::Failed:
        return "failed";
    }
    return "failed";
}

const char *event_name(EventKind k)
{
    switch (k)
    {
    case EventKind::BootOk:
        return "boot_ok";
    case EventKind::BootFailed:
        return "boot_failed";
    case EventKind::Silent:
        return "silent";
    case EventKind::Lost:
        return "lost";
    case EventKind::Emcy:
        return "emcy";
    case EventKind::EmcyReset:
        return "emcy_reset";
    case EventKind::SdoAbort:
        return "sdo_abort";
    case EventKind::PdoLate:
        return "pdo_late";
    case EventKind::PdoBack:
        return "pdo_back";
    case EventKind::PdoLength:
        return "pdo_length";
    case EventKind::BusChange:
        return "bus_state";
    case EventKind::BusReset:
        return "bus_reset";
    }
    return "boot_ok";
}

void add_uint(cJSON *o, const char *key, uint64_t v)
{
    cJSON_AddNumberToObject(o, key, static_cast<double>(v));
}

cJSON *problem_json(const Problem &p)
{
    if (p.kind == ProblemKind::None)
        return cJSON_CreateNull();
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "kind", problem_name(p.kind));
    switch (p.kind)
    {
    case ProblemKind::Identity:
        cJSON_AddStringToObject(o, "field", field_name(p.field));
        add_uint(o, "expected", p.expected);
        if (p.has_found)
            add_uint(o, "found", p.found);
        break;
    case ProblemKind::SdoAbort:
        add_uint(o, "index", p.index);
        add_uint(o, "subindex", p.subindex);
        add_uint(o, "abort_code", p.abort_code);
        break;
    case ProblemKind::BootError:
        if (!p.text.empty())
            cJSON_AddStringToObject(o, "text", p.text.c_str());
        break;
    default:
        break;
    }
    if (p.boot_status)
    {
        char s[2] = {p.boot_status, 0};
        cJSON_AddStringToObject(o, "boot_status", s);
    }
    return o;
}

cJSON *bus_json(const BusStatus &b)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "state", bus_state_name(b.state));
    if (b.has_counters)
    {
        add_uint(o, "tx_errors", b.tx_errors);
        add_uint(o, "rx_errors", b.rx_errors);
    }
    if (b.has_bus_off)
        add_uint(o, "bus_off_count", b.bus_off);
    if (b.has_frames)
    {
        add_uint(o, "rx_frames", b.rx_frames);
        add_uint(o, "tx_frames", b.tx_frames);
        add_uint(o, "rx_dropped", b.rx_dropped);
    }
    if (b.has_load)
        cJSON_AddNumberToObject(o, "load_percent", static_cast<int>(b.load_percent * 10) / 10.0);
    if (b.has_resets || b.resets)
        add_uint(o, "link_resets", b.resets);
    return o;
}

cJSON *node_json(const NodeStatus &n)
{
    cJSON *o = cJSON_CreateObject();
    add_uint(o, "node_id", n.node_id);
    cJSON_AddStringToObject(o, "name", n.name.c_str());
    cJSON_AddStringToObject(o, "state", node_run_name(n.state));
    cJSON_AddStringToObject(o, "nmt", nmt_name(n.nmt));
    cJSON_AddNumberToObject(o, "since", static_cast<double>(n.since));
    add_uint(o, "boot_attempts", n.boots);
    add_uint(o, "losses", n.losses);
    cJSON_AddBoolToObject(o, "warning", n.warning());
    cJSON_AddItemToObject(o, "problem", problem_json(n.problem));
    cJSON *emcy = cJSON_AddArrayToObject(o, "emcy");
    for (const Emcy &m : n.emcy)
    {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddNumberToObject(e, "time", static_cast<double>(m.time));
        add_uint(e, "code", m.code);
        add_uint(e, "register", m.reg);
        char hex[11];
        std::snprintf(hex, sizeof(hex), "%02X%02X%02X%02X%02X", m.data[0], m.data[1], m.data[2],
                      m.data[3], m.data[4]);
        cJSON_AddStringToObject(e, "data", hex);
        cJSON_AddBoolToObject(e, "active", m.active);
        cJSON_AddItemToArray(emcy, e);
    }
    cJSON *late = cJSON_AddArrayToObject(o, "late_pdos");
    for (const LatePdo &l : n.late)
    {
        cJSON *e = cJSON_CreateObject();
        add_uint(e, "pdo", l.pdo);
        add_uint(e, "cob_id", l.cob_id);
        cJSON_AddNumberToObject(e, "since", static_cast<double>(l.since));
        cJSON_AddItemToArray(late, e);
    }
    return o;
}

cJSON *event_json(const Event &ev)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "time", static_cast<double>(ev.time));
    if (ev.node)
        add_uint(o, "node", ev.node);
    cJSON_AddStringToObject(o, "kind", event_name(ev.kind));
    switch (ev.kind)
    {
    case EventKind::BootFailed:
    case EventKind::Lost:
    case EventKind::SdoAbort:
        cJSON_AddItemToObject(o, "problem", problem_json(ev.problem));
        break;
    case EventKind::Emcy:
        add_uint(o, "code", ev.code);
        add_uint(o, "register", ev.reg);
        break;
    case EventKind::PdoLength:
        add_uint(o, "code", ev.code);
        add_uint(o, "pdo", ev.pdo);
        add_uint(o, "cob_id", ev.cob_id);
        break;
    case EventKind::PdoLate:
    case EventKind::PdoBack:
        add_uint(o, "pdo", ev.pdo);
        add_uint(o, "cob_id", ev.cob_id);
        break;
    case EventKind::BusChange:
        cJSON_AddStringToObject(o, "bus_state", bus_state_name(ev.bus));
        break;
    case EventKind::BusReset:
        cJSON_AddStringToObject(o, "outcome", reset_outcome_name(ev.reset));
        break;
    default:
        break;
    }
    return o;
}

} // namespace

std::string status_json(const std::vector<NetworkSnapshot> &networks, int64_t now)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "version", 1);
    cJSON_AddBoolToObject(root, "active", true);
    cJSON_AddNumberToObject(root, "now", static_cast<double>(now));
    cJSON *nets = cJSON_AddArrayToObject(root, "networks");
    for (const NetworkSnapshot &n : networks)
    {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "name", n.name.c_str());
        cJSON_AddStringToObject(o, "interface", n.interface.c_str());
        cJSON_AddItemToObject(o, "bus", bus_json(n.bus));
        cJSON_AddBoolToObject(o, "all_nodes_up", n.all_nodes_up());
        cJSON *nodes = cJSON_AddArrayToObject(o, "nodes");
        for (const NodeStatus &s : n.nodes)
            cJSON_AddItemToArray(nodes, node_json(s));
        cJSON *events = cJSON_AddArrayToObject(o, "events");
        for (const Event &e : n.events)
            cJSON_AddItemToArray(events, event_json(e));
        cJSON_AddItemToArray(nets, o);
    }
    char *text      = cJSON_PrintUnformatted(root);
    std::string out = text ? text : "{}";
    cJSON_free(text);
    cJSON_Delete(root);
    return out;
}

} // namespace canopen_plugin
