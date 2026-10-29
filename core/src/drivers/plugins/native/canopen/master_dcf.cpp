// SPDX-License-Identifier: Apache-2.0

#include "master_dcf.h"

#include <cstdio>
#include <sstream>

namespace canopen_plugin
{

namespace
{

std::string hex(uint32_t v)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%X", v);
    return buf;
}

std::string idx_name(uint16_t index)
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%04X", index);
    return buf;
}

class DcfWriter
{
  public:
    void var(uint16_t index, const char *name, CoType type, const char *access, uint32_t value,
             bool pdo_mapping = false)
    {
        list(index);
        body_ << "[" << idx_name(index) << "]\n";
        entry(name, 0x7, type, access, value, pdo_mapping);
    }

    // An ARRAY or RECORD (object_type 0x8 or 0x9) with sub-index 0 holding `highest`.
    void begin(uint16_t index, const char *name, unsigned object_type, unsigned subs,
               unsigned highest)
    {
        list(index);
        body_ << "[" << idx_name(index) << "]\nParameterName=" << name
              << "\nObjectType=" << hex(object_type) << "\nSubNumber=" << subs + 1 << "\n\n";
        sub(index, 0, "Highest sub-index supported", CoType::UNSIGNED8, "ro", highest);
    }

    void sub(uint16_t index, unsigned subindex, const char *name, CoType type, const char *access,
             uint32_t value)
    {
        body_ << "[" << idx_name(index) << "sub" << std::hex << std::uppercase << subindex
              << std::dec << "]\n";
        entry(name, 0x7, type, access, value, false);
    }

    std::string text(unsigned node_id) const
    {
        std::ostringstream out;
        out << "[FileInfo]\nFileName=master.dcf\nFileVersion=1\nFileRevision=0\nEDSVersion=4.0\n"
               "Description=OpenPLC CANopen master\n\n"
               "[DeviceInfo]\nVendorName=OpenPLC\nProductName=CANopen master\n"
               "BaudRate_10=1\nBaudRate_20=1\nBaudRate_50=1\nBaudRate_125=1\nBaudRate_250=1\n"
               "BaudRate_500=1\nBaudRate_800=1\nBaudRate_1000=1\nSimpleBootUpMaster=1\n"
               "SimpleBootUpSlave=0\nGranularity=8\nNrOfRXPDO="
            << rpdos_ << "\nNrOfTXPDO=" << tpdos_ << "\nLSS_Supported=0\n\n";
        out << "[DeviceComissioning]\nNodeID=" << node_id << "\n\n";
        section(out, "MandatoryObjects", mandatory_);
        section(out, "OptionalObjects", optional_);
        section(out, "ManufacturerObjects", manufacturer_);
        out << body_.str();
        return out.str();
    }

    unsigned rpdos_ = 0;
    unsigned tpdos_ = 0;

  private:
    void entry(const char *name, unsigned object_type, CoType type, const char *access,
               uint32_t value, bool pdo_mapping)
    {
        body_ << "ParameterName=" << name << "\nObjectType=" << hex(object_type)
              << "\nDataType=" << hex(static_cast<uint16_t>(type)) << "\nAccessType=" << access
              << "\nDefaultValue=" << hex(value) << "\nPDOMapping=" << (pdo_mapping ? 1 : 0)
              << "\n\n";
    }

    void list(uint16_t index)
    {
        if (index == 0x1000 || index == 0x1001 || index == 0x1018)
            mandatory_.push_back(index);
        else if (index >= 0x2000 && index < 0x6000)
            manufacturer_.push_back(index);
        else
            optional_.push_back(index);
    }

    static void section(std::ostringstream &out, const char *name,
                        const std::vector<uint16_t> &list)
    {
        out << "[" << name << "]\nSupportedObjects=" << list.size() << "\n";
        for (size_t i = 0; i < list.size(); ++i)
            out << i + 1 << "=" << hex(list[i]) << "\n";
        out << "\n";
    }

    std::vector<uint16_t> mandatory_, optional_, manufacturer_;
    std::ostringstream body_;
};

void mirror_pdos(DcfWriter &w, const std::vector<MasterPdo> &pdos, bool receive)
{
    uint16_t comm = receive ? 0x1400 : 0x1800;
    uint16_t map  = receive ? 0x1600 : 0x1A00;
    uint16_t base = receive ? kInputObjectBase : kOutputObjectBase;
    for (const MasterPdo &m : pdos)
    {
        uint16_t n = static_cast<uint16_t>(m.master_number - 1);
        w.begin(comm + n, receive ? "RPDO communication parameter" : "TPDO communication parameter",
                0x9, receive ? 2 : 3, receive ? 2 : 3);
        w.sub(comm + n, 1, "COB-ID", CoType::UNSIGNED32, "rw", m.pdo->cob_id);
        w.sub(comm + n, 2, "Transmission type", CoType::UNSIGNED8, "rw", 0xFF);
        if (!receive)
            w.sub(comm + n, 3, "Inhibit time", CoType::UNSIGNED16, "rw", 0);
        const auto &entries = m.pdo->entries;
        w.begin(map + n, receive ? "RPDO mapping parameter" : "TPDO mapping parameter", 0x9,
                static_cast<unsigned>(entries.size()), static_cast<unsigned>(entries.size()));
        for (size_t k = 0; k < entries.size(); ++k)
        {
            uint32_t object = base + static_cast<uint32_t>(m.first_entry + k);
            w.sub(map + n, static_cast<unsigned>(k + 1), "Mapped object", CoType::UNSIGNED32, "rw",
                  object << 16 | co_type_bits(entries[k].type));
        }
    }
}

} // namespace

MasterLayout master_layout(const NetworkConfig &net)
{
    MasterLayout l;
    for (const auto &node : net.nodes)
    {
        for (const auto &p : node.tpdos)
        {
            l.rpdos.push_back(
                {static_cast<unsigned>(l.rpdos.size() + 1), node.node_id, &p, l.input_entries});
            l.input_entries += p.entries.size();
        }
        for (const auto &p : node.rpdos)
        {
            l.tpdos.push_back(
                {static_cast<unsigned>(l.tpdos.size() + 1), node.node_id, &p, l.output_entries});
            l.output_entries += p.entries.size();
        }
    }
    return l;
}

std::string master_dcf(const NetworkConfig &net, const MasterLayout &layout)
{
    DcfWriter w;
    w.rpdos_ = static_cast<unsigned>(layout.rpdos.size());
    w.tpdos_ = static_cast<unsigned>(layout.tpdos.size());
    w.var(0x1000, "Device type", CoType::UNSIGNED32, "ro", 0);
    w.var(0x1001, "Error register", CoType::UNSIGNED8, "ro", 0);
    // SYNC: produced by Lely at the period, or sent by the plugin once per PLC cycle.
    uint32_t sync_cob = 0x80;
    if (net.sync == SyncMode::Period)
        sync_cob |= 0x40000000;
    w.var(0x1005, "COB-ID SYNC", CoType::UNSIGNED32, "rw", sync_cob);
    w.var(0x1006, "Communication cycle period", CoType::UNSIGNED32, "rw",
          net.sync == SyncMode::Period ? net.sync_period_us : 0);

    std::vector<const NodeConfig *> watched;
    for (const auto &n : net.nodes)
        if (n.supervision == Supervision::Heartbeat)
            watched.push_back(&n);
    w.begin(0x1016, "Consumer heartbeat time", 0x8, static_cast<unsigned>(watched.size()),
            static_cast<unsigned>(watched.size()));
    for (size_t i = 0; i < watched.size(); ++i)
        w.sub(0x1016, static_cast<unsigned>(i + 1), "Consumer heartbeat time", CoType::UNSIGNED32,
              "rw", watched[i]->node_id << 16 | watched[i]->heartbeat_consumer_ms);
    w.var(0x1017, "Producer heartbeat time", CoType::UNSIGNED16, "rw", net.heartbeat_producer_ms);

    w.begin(0x1018, "Identity object", 0x9, 4, 4);
    w.sub(0x1018, 1, "Vendor-ID", CoType::UNSIGNED32, "ro", 0);
    w.sub(0x1018, 2, "Product code", CoType::UNSIGNED32, "ro", 0);
    w.sub(0x1018, 3, "Revision number", CoType::UNSIGNED32, "ro", 0);
    w.sub(0x1018, 4, "Serial number", CoType::UNSIGNED32, "ro", 0);

    mirror_pdos(w, layout.rpdos, true);
    mirror_pdos(w, layout.tpdos, false);

    // NMT master that starts its slaves one by one once each is booted (CiA 302-2).
    w.var(0x1F80, "NMT startup", CoType::UNSIGNED32, "rw", 0x00000001);
    struct Array
    {
        uint16_t index;
        const char *name;
        CoType type;
        uint32_t (*value)(const NodeConfig &);
    };
    const Array arrays[] = {
        // Slave, and boot it again after an error control event; optional (not mandatory).
        {0x1F81, "Slave assignment", CoType::UNSIGNED32,
         [](const NodeConfig &) -> uint32_t { return 0x05; }},
        {0x1F84, "Device type identification", CoType::UNSIGNED32,
         [](const NodeConfig &n) { return n.device_type; }},
        {0x1F85, "Vendor identification", CoType::UNSIGNED32,
         [](const NodeConfig &n) { return n.vendor_id; }},
        {0x1F86, "Product code", CoType::UNSIGNED32,
         [](const NodeConfig &n) { return n.product_code; }},
        {0x1F87, "Revision number", CoType::UNSIGNED32,
         [](const NodeConfig &n) { return n.revision; }},
    };
    for (const Array &a : arrays)
    {
        w.begin(a.index, a.name, 0x8, static_cast<unsigned>(net.nodes.size()), 127);
        for (const auto &n : net.nodes)
            w.sub(a.index, n.node_id, a.name, a.type, "rw", a.value(n));
    }

    // One object per mapped entry, of the entry's type.
    for (const auto *list : {&layout.rpdos, &layout.tpdos})
    {
        bool in = list == &layout.rpdos;
        for (const MasterPdo &m : *list)
            for (size_t k = 0; k < m.pdo->entries.size(); ++k)
                w.var(static_cast<uint16_t>((in ? kInputObjectBase : kOutputObjectBase) +
                                            m.first_entry + k),
                      in ? "Input" : "Output", m.pdo->entries[k].type, "rw", 0, true);
    }
    return w.text(net.master_node_id);
}

std::vector<uint8_t> concise_dcf(const std::vector<SdoWrite> &sdo)
{
    std::vector<uint8_t> out;
    auto put = [&out](uint32_t v, int bytes)
    {
        for (int i = 0; i < bytes; ++i)
            out.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put(static_cast<uint32_t>(sdo.size()), 4);
    for (const auto &w : sdo)
    {
        put(w.index, 2);
        put(w.subindex, 1);
        put(static_cast<uint32_t>(w.data.size()), 4);
        out.insert(out.end(), w.data.begin(), w.data.end());
    }
    return out;
}

} // namespace canopen_plugin
