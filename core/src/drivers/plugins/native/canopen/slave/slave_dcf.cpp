// SPDX-License-Identifier: Apache-2.0

#include "slave_dcf.h"

#include <algorithm>
#include <cstdio>
#include <sstream>
#include <vector>

namespace canopen_plugin
{

namespace
{

constexpr uint32_t kCobInvalid = 0x80000000u;

std::string hex(uint32_t v)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%X", static_cast<unsigned>(v));
    return buf;
}

std::string idx_text(uint16_t index)
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%04X", static_cast<unsigned>(index));
    return buf;
}

class Dcf
{
  public:
    // A variable at sub-index 0 (object type VAR).
    void var(uint16_t index, const char *name, unsigned type, const char *access,
             const std::string &value, bool map = false)
    {
        list_.push_back(index);
        body_ << "[" << idx_text(index) << "]\n";
        entry(name, type, access, value, map);
    }
    // An ARRAY (0x8) or RECORD (0x9) with `subs` sub-objects including sub 0, which holds
    // `highest`.
    void rec(uint16_t index, const char *name, unsigned subs, unsigned highest, bool array = false,
             const char *sub0_access = "ro")
    {
        list_.push_back(index);
        body_ << "[" << idx_text(index) << "]\nParameterName=" << name
              << "\nObjectType=" << (array ? "0x8" : "0x9") << "\nSubNumber=" << subs << "\n\n";
        sub(index, 0, "Highest sub-index supported", 0x5, sub0_access, std::to_string(highest));
    }
    void sub(uint16_t index, unsigned s, const char *name, unsigned type, const char *access,
             const std::string &value, bool map = false)
    {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%X", s);
        body_ << "[" << idx_text(index) << "sub" << buf << "]\n";
        entry(name, type, access, value, map);
    }
    std::string text(const DeviceConfig &dev, unsigned node_id) const
    {
        std::ostringstream out;
        out << "[FileInfo]\nFileName=slave.dcf\nFileVersion=1\nFileRevision=0\nEDSVersion=4.0\n\n"
               "[DeviceInfo]\nVendorNumber="
            << hex(dev.vendor_id) << "\nProductNumber=" << hex(dev.product_code)
            << "\nRevisionNumber=" << hex(dev.revision)
            << "\nBaudRate_10=1\nBaudRate_20=1\nBaudRate_50=1\nBaudRate_125=1\nBaudRate_250=1\n"
               "BaudRate_500=1\nBaudRate_800=1\nBaudRate_1000=1\nSimpleBootUpMaster=0\n"
               "SimpleBootUpSlave=1\nGranularity=8\nNrOfRXPDO="
            << kSlavePdos << "\nNrOfTXPDO=" << kSlavePdos
            << "\nLSS_Supported=" << (dev.lss.enabled ? 1 : 0) << "\n\n";
        // Without a node ID (255) the section is left out: the device waits for LSS.
        if (node_id >= 1 && node_id <= 127)
            out << "[DeviceComissioning]\nNodeID=" << node_id << "\n\n";
        std::vector<uint16_t> sorted = list_;
        std::sort(sorted.begin(), sorted.end());
        std::ostringstream mand, opt, man;
        int nm = 0, no = 0, nf = 0;
        for (uint16_t i : sorted)
        {
            if (i == 0x1000 || i == 0x1001 || i == 0x1018)
                mand << ++nm << "=0x" << idx_text(i) << "\n";
            else if (i >= 0x2000 && i <= 0x5FFF)
                man << ++nf << "=0x" << idx_text(i) << "\n";
            else
                opt << ++no << "=0x" << idx_text(i) << "\n";
        }
        out << "[MandatoryObjects]\nSupportedObjects=" << nm << "\n" << mand.str() << "\n";
        out << "[OptionalObjects]\nSupportedObjects=" << no << "\n" << opt.str() << "\n";
        out << "[ManufacturerObjects]\nSupportedObjects=" << nf << "\n" << man.str() << "\n";
        // Every dummy object may be mapped (CiA 301).
        out << "[DummyUsage]\n";
        for (int i = 1; i <= 7; ++i)
            out << "Dummy000" << i << "=1\n";
        out << "\n" << body_.str();
        return out.str();
    }

  private:
    void entry(const char *name, unsigned type, const char *access, const std::string &value,
               bool map)
    {
        body_ << "ParameterName=" << name << "\nObjectType=0x7\nDataType=" << hex(type)
              << "\nAccessType=" << access << "\nDefaultValue=" << value
              << "\nPDOMapping=" << (map ? 1 : 0) << "\n\n";
    }

    std::vector<uint16_t> list_;
    std::ostringstream body_;
};

// A COB-ID that follows the node ID: $NODEID+base, keeping the flags in the high bits.
std::string node_relative(uint32_t value)
{
    return "$NODEID+" + hex(value);
}

void add_pdos(Dcf &d, const DeviceConfig &dev, bool tpdo)
{
    const std::vector<SlavePdo> &pdos = tpdo ? dev.tpdos : dev.rpdos;
    const uint16_t comm_base          = tpdo ? 0x1800 : 0x1400;
    const uint16_t map_base           = tpdo ? 0x1A00 : 0x1600;
    const uint32_t cob_base           = tpdo ? 0x180 : 0x200;
    for (unsigned n = 1; n <= kSlavePdos; ++n)
    {
        const SlavePdo *pdo = nullptr;
        for (const auto &p : pdos)
            if (p.number == n)
                pdo = &p;
        const uint16_t comm = static_cast<uint16_t>(comm_base + n - 1);
        const uint32_t base = cob_base + 0x100 * (n - 1);
        // PDO 1-4 on their pre-defined COB-ID follow the node ID in use; PDO 5-16 have none and
        // are off with no CAN ID until a master writes one.
        std::string cob;
        if (!pdo)
            cob = n <= 4 ? node_relative(kCobInvalid | base) : hex(kCobInvalid);
        else if (pdo->cob_id == 0)
            cob = hex(kCobInvalid);
        else if (n <= 4 && (pdo->cob_id & 0x7FF) == base + dev.node_id)
            cob = node_relative((pdo->cob_id & ~0x7FFu) | base);
        else
            cob = hex(pdo->cob_id);
        d.rec(comm, tpdo ? "TPDO communication parameter" : "RPDO communication parameter",
              tpdo ? 7 : 4, tpdo ? 6 : 5);
        d.sub(comm, 1, "COB-ID", 0x7, "rw", cob);
        d.sub(comm, 2, "Transmission type", 0x5, "rw",
              std::to_string(pdo ? pdo->transmission : 255));
        if (tpdo)
        {
            d.sub(comm, 3, "Inhibit time", 0x6, "rw", std::to_string(pdo ? pdo->inhibit_100us : 0));
            d.sub(comm, 4, "Reserved", 0x5, "rw", "0");
        }
        d.sub(comm, 5, "Event timer", 0x6, "rw", std::to_string(pdo ? pdo->event_timer_ms : 0));
        if (tpdo)
            d.sub(comm, 6, "SYNC start value", 0x5, "rw", "0");
    }
    for (unsigned n = 1; n <= kSlavePdos; ++n)
    {
        const SlavePdo *pdo = nullptr;
        for (const auto &p : pdos)
            if (p.number == n)
                pdo = &p;
        const uint16_t map = static_cast<uint16_t>(map_base + n - 1);
        const size_t count = pdo ? pdo->entries.size() : 0;
        d.rec(map, tpdo ? "TPDO mapping parameter" : "RPDO mapping parameter", 9,
              static_cast<unsigned>(count), false, "rw");
        for (unsigned s = 1; s <= 8; ++s)
        {
            uint32_t value = 0;
            if (pdo && s <= pdo->entries.size())
            {
                const SlavePdoEntry &e = pdo->entries[s - 1];
                const SlaveObject *o   = dev.find(e.index, e.subindex);
                const unsigned bits    = o ? o->bytes() * 8 : 0;
                value                  = (static_cast<uint32_t>(e.index) << 16) |
                        (static_cast<uint32_t>(e.subindex) << 8) | bits;
            }
            d.sub(map, s, "Mapped object", 0x7, "rw", hex(value));
        }
    }
}

const char *access_of(const SlaveObject &o)
{
    if (o.exch == SlaveExchange::Constant)
        return "const";
    return o.dir == SlaveDirection::FromMaster ? "rw" : "ro";
}

} // namespace

std::string slave_dcf(const DeviceConfig &dev, uint32_t serial, unsigned node_id)
{
    Dcf d;
    d.var(0x1000, "Device type", 0x7, "ro", "0");
    d.var(0x1001, "Error register", 0x5, "ro", "0", true);
    d.rec(0x1003, "Pre-defined error field", 9, 0, true, "rw");
    for (unsigned s = 1; s <= 8; ++s)
        d.sub(0x1003, s, "Standard error field", 0x7, "ro", "0");
    d.var(0x1005, "COB-ID SYNC", 0x7, "rw", "0x80");
    d.var(0x1006, "Communication cycle period", 0x7, "rw", "0");
    d.var(0x1007, "Synchronous window length", 0x7, "rw", "0");
    // Strings are set after loading, so any printable text survives the DCF syntax.
    if (!dev.device_name.empty())
        d.var(0x1008, "Manufacturer device name", 0x9, "const", "");
    if (!dev.hardware_version.empty())
        d.var(0x1009, "Manufacturer hardware version", 0x9, "const", "");
    if (!dev.software_version.empty())
        d.var(0x100A, "Manufacturer software version", 0x9, "const", "");
    d.var(0x1014, "COB-ID EMCY", 0x7, "rw", node_relative(0x80));
    d.rec(0x1016, "Consumer heartbeat time", 2, 1, true);
    const uint32_t watch = dev.master_node_id ? (static_cast<uint32_t>(dev.master_node_id) << 16) |
                                                    (dev.master_watch_ms & 0xFFFF)
                                              : 0;
    d.sub(0x1016, 1, "Consumer heartbeat time", 0x7, "rw", hex(watch));
    d.var(0x1017, "Producer heartbeat time", 0x6, "rw", std::to_string(dev.heartbeat_ms));
    d.rec(0x1018, "Identity object", 5, 4);
    d.sub(0x1018, 1, "Vendor-ID", 0x7, "ro", hex(dev.vendor_id));
    d.sub(0x1018, 2, "Product code", 0x7, "ro", hex(dev.product_code));
    d.sub(0x1018, 3, "Revision number", 0x7, "ro", hex(dev.revision));
    d.sub(0x1018, 4, "Serial number", 0x7, "ro", hex(serial));
    // 0x1029 sub 1 (CiA 301) and sub 2, the NMT state after a PLC stop, coded like sub 1.
    unsigned lost = 0;
    switch (dev.on_master_lost)
    {
    case OnMasterLost::PreOperational:
        lost = 0;
        break;
    case OnMasterLost::NoChange:
        lost = 1;
        break;
    case OnMasterLost::Stopped:
        lost = 2;
        break;
    }
    unsigned stop = 1;
    switch (dev.on_plc_stop_state)
    {
    case SlaveStopState::PreOperational:
        stop = 0;
        break;
    case SlaveStopState::NoChange:
        stop = 1;
        break;
    case SlaveStopState::Stopped:
        stop = 2;
        break;
    }
    d.rec(0x1029, "Error behaviour", 3, 2, true);
    d.sub(0x1029, 1, "Communication error", 0x5, "rw", std::to_string(lost));
    d.sub(0x1029, 2, "PLC stop", 0x5, "rw", std::to_string(stop));
    d.rec(0x1200, "SDO server parameter", 3, 2);
    d.sub(0x1200, 1, "COB-ID client to server", 0x7, "ro", node_relative(0x600));
    d.sub(0x1200, 2, "COB-ID server to client", 0x7, "ro", node_relative(0x580));
    add_pdos(d, dev, false);
    add_pdos(d, dev, true);
    // NMT startup (CiA 302-2): bit 2 keeps the slave in pre-operational until a master (or the
    // program) starts it; without it the slave starts by itself.
    d.var(0x1F80, "NMT startup", 0x7, "const", dev.start_alone ? "0" : "0x4");

    // Application objects, grouped by index in ascending order.
    std::vector<const SlaveObject *> objs;
    for (const auto &o : dev.objects)
        objs.push_back(&o);
    std::sort(objs.begin(), objs.end(), [](const SlaveObject *a, const SlaveObject *b)
              { return a->index != b->index ? a->index < b->index : a->subindex < b->subindex; });
    for (size_t i = 0; i < objs.size();)
    {
        const uint16_t index = objs[i]->index;
        size_t end           = i;
        while (end < objs.size() && objs[end]->index == index)
            ++end;
        // Names stay generic: a row name may hold characters the DCF syntax does not allow.
        d.rec(index, "Application object", static_cast<unsigned>(end - i + 1),
              objs[end - 1]->subindex);
        for (size_t k = i; k < end; ++k)
        {
            const SlaveObject &o = *objs[k];
            const unsigned type  = o.string ? 0x9u : static_cast<unsigned>(o.type);
            d.sub(index, o.subindex, "Application value", type, access_of(o), o.string ? "" : "0",
                  o.exch == SlaveExchange::Pdo);
        }
        i = end;
    }
    return d.text(dev, node_id);
}

} // namespace canopen_plugin
