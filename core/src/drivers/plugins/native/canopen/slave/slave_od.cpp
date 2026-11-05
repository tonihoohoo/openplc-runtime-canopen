// SPDX-License-Identifier: Apache-2.0

#include "slave_od.h"

#include <algorithm>
#include <cstring>

namespace canopen_plugin
{

namespace
{

constexpr uint32_t kCobInvalid = 0x80000000u;

OD_attr_t multi_byte(OD_size_t len)
{
    return len > 1 ? static_cast<OD_attr_t>(ODA_MB) : static_cast<OD_attr_t>(0);
}

} // namespace

uint8_t *SlaveOd::store(size_t bytes)
{
    storage_.emplace_back(bytes ? bytes : 1, 0);
    return storage_.back().data();
}

uint8_t *SlaveOd::store_u8(uint8_t v)
{
    uint8_t *p = store(1);
    *p         = v;
    defaults_.push_back({p, {v}});
    return p;
}

uint8_t *SlaveOd::store_u16(uint16_t v)
{
    uint8_t *p = store(2);
    std::memcpy(p, &v, 2);
    defaults_.push_back({p, std::vector<uint8_t>(p, p + 2)});
    return p;
}

uint8_t *SlaveOd::store_u32(uint32_t v)
{
    uint8_t *p = store(4);
    std::memcpy(p, &v, 4);
    defaults_.push_back({p, std::vector<uint8_t>(p, p + 4)});
    return p;
}

void SlaveOd::add_var(uint16_t index, void *data, OD_attr_t attr, OD_size_t len)
{
    vars_.push_back(OD_obj_var_t{data, attr, len});
    pending_.push_back({index, ODT_VAR, &vars_.back(), 1, 0});
}

SlaveOd::Record &SlaveOd::add_record(uint16_t index)
{
    records_.emplace_back();
    pending_.push_back({index, ODT_REC, nullptr, 0, records_.size() - 1});
    return records_.back();
}

void SlaveOd::add_sub(Record &rec, uint8_t sub, void *data, OD_attr_t attr, OD_size_t len)
{
    rec.subs.push_back(OD_obj_record_t{data, sub, attr, len});
}

void SlaveOd::add_pdos(bool tpdo)
{
    const std::vector<SlavePdo> &pdos = tpdo ? dev_->tpdos : dev_->rpdos;
    const uint16_t comm_base          = tpdo ? 0x1800 : 0x1400;
    const uint16_t map_base           = tpdo ? 0x1A00 : 0x1600;
    const uint32_t cob_base           = tpdo ? 0x180 : 0x200;
    const OD_attr_t rw                = ODA_SDO_RW;
    for (unsigned n = 1; n <= kSlavePdos; ++n)
    {
        const SlavePdo *pdo = nullptr;
        for (const auto &p : pdos)
            if (p.number == n)
                pdo = &p;
        // PDOs 1-4 on their default COB-ID keep the bare base, which the stack adds the node ID
        // in use to.
        const uint32_t base = cob_base + 0x100 * (n - 1);
        // PDO 5-16 have no pre-defined COB-ID: off with no CAN ID until a master writes one.
        uint32_t cob = pdo ? pdo->cob_id : (kCobInvalid | (n <= 4 ? base : 0));
        if (pdo && cob == 0)
            cob = kCobInvalid;
        if (pdo && n <= 4 && (cob & 0x7FF) == base + dev_->node_id)
            cob = (cob & ~0x7FFu) | base;
        Record &rec = add_record(static_cast<uint16_t>(comm_base + n - 1));
        add_sub(rec, 0, store_u8(tpdo ? 6 : 5), ODA_SDO_R, 1);
        add_sub(rec, 1, store_u32(cob), rw | ODA_MB, 4);
        add_sub(rec, 2, store_u8(static_cast<uint8_t>(pdo ? pdo->transmission : 255)), rw, 1);
        if (tpdo)
        {
            add_sub(rec, 3, store_u16(static_cast<uint16_t>(pdo ? pdo->inhibit_100us : 0)),
                    rw | ODA_MB, 2);
            add_sub(rec, 4, store_u8(0), rw, 1);
        }
        add_sub(rec, 5, store_u16(static_cast<uint16_t>(pdo ? pdo->event_timer_ms : 0)),
                rw | ODA_MB, 2);
        if (tpdo)
            add_sub(rec, 6, store_u8(0), rw, 1);
    }
    for (unsigned n = 1; n <= kSlavePdos; ++n)
    {
        const SlavePdo *pdo = nullptr;
        for (const auto &p : pdos)
            if (p.number == n)
                pdo = &p;
        Record &rec         = add_record(static_cast<uint16_t>(map_base + n - 1));
        const uint8_t count = pdo ? static_cast<uint8_t>(pdo->entries.size()) : 0;
        add_sub(rec, 0, store_u8(count), rw, 1);
        for (uint8_t s = 1; s <= 8; ++s)
        {
            uint32_t map = 0;
            if (pdo && s <= pdo->entries.size())
            {
                const SlavePdoEntry &e = pdo->entries[s - 1];
                const SlaveObject *o   = dev_->find(e.index, e.subindex);
                const unsigned bits    = o ? (o->bytes() * 8) : 0;
                map                    = (static_cast<uint32_t>(e.index) << 16) |
                      (static_cast<uint32_t>(e.subindex) << 8) | bits;
            }
            add_sub(rec, s, store_u32(map), rw | ODA_MB, 4);
        }
    }
}

bool SlaveOd::build(const DeviceConfig &dev, uint32_t serial, std::string &problem)
{
    dev_               = &dev;
    const uint8_t node = static_cast<uint8_t>(dev.node_id);

    add_var(0x1000, store_u32(0), ODA_SDO_R | ODA_MB, 4);
    add_var(0x1001, store_u8(0), ODA_SDO_R | ODA_TPDO, 1);
    {
        Record &rec = add_record(0x1003);
        add_sub(rec, 0, store_u8(0), ODA_SDO_RW, 1);
        for (uint8_t s = 1; s <= 8; ++s)
            add_sub(rec, s, store_u32(0), ODA_SDO_R | ODA_MB, 4);
    }
    add_var(0x1005, store_u32(0x80), ODA_SDO_RW | ODA_MB, 4);
    add_var(0x1006, store_u32(0), ODA_SDO_RW | ODA_MB, 4);
    add_var(0x1007, store_u32(0), ODA_SDO_RW | ODA_MB, 4);
    if (!dev.device_name.empty())
    {
        uint8_t *p = store(dev.device_name.size());
        std::memcpy(p, dev.device_name.data(), dev.device_name.size());
        add_var(0x1008, p, ODA_SDO_R | ODA_STR, static_cast<OD_size_t>(dev.device_name.size()));
    }
    if (!dev.software_version.empty())
    {
        uint8_t *p = store(dev.software_version.size());
        std::memcpy(p, dev.software_version.data(), dev.software_version.size());
        add_var(0x100A, p, ODA_SDO_R | ODA_STR,
                static_cast<OD_size_t>(dev.software_version.size()));
    }
    add_var(0x1014, store_u32(0x80u), ODA_SDO_RW | ODA_MB, 4);
    {
        Record &rec = add_record(0x1016);
        add_sub(rec, 0, store_u8(1), ODA_SDO_R, 1);
        const uint32_t watch =
            dev.master_node_id
                ? (static_cast<uint32_t>(dev.master_node_id) << 16) | dev.master_watch_ms
                : 0;
        add_sub(rec, 1, store_u32(watch), ODA_SDO_RW | ODA_MB, 4);
    }
    add_var(0x1017, store_u16(static_cast<uint16_t>(dev.heartbeat_ms)), ODA_SDO_RW | ODA_MB, 2);
    {
        Record &rec = add_record(0x1018);
        add_sub(rec, 0, store_u8(4), ODA_SDO_R, 1);
        add_sub(rec, 1, store_u32(dev.vendor_id), ODA_SDO_R | ODA_MB, 4);
        add_sub(rec, 2, store_u32(dev.product_code), ODA_SDO_R | ODA_MB, 4);
        add_sub(rec, 3, store_u32(dev.revision), ODA_SDO_R | ODA_MB, 4);
        add_sub(rec, 4, store_u32(serial), ODA_SDO_R | ODA_MB, 4);
    }
    {
        switch (dev.on_master_lost)
        {
        case OnMasterLost::PreOperational:
            error_behaviour_ = kErrorPreOperational;
            break;
        case OnMasterLost::NoChange:
            error_behaviour_ = kErrorNoChange;
            break;
        case OnMasterLost::Stopped:
            error_behaviour_ = kErrorStopped;
            break;
        }
        uint8_t stop_state = kErrorNoChange;
        switch (dev.on_plc_stop_state)
        {
        case SlaveStopState::PreOperational:
            stop_state = kErrorPreOperational;
            break;
        case SlaveStopState::NoChange:
            stop_state = kErrorNoChange;
            break;
        case SlaveStopState::Stopped:
            stop_state = kErrorStopped;
            break;
        }
        Record &rec = add_record(0x1029);
        add_sub(rec, 0, store_u8(2), ODA_SDO_R, 1);
        error_data_ = store_u8(error_behaviour_);
        add_sub(rec, 1, error_data_, ODA_SDO_RW, 1);
        plc_stop_data_ = store_u8(stop_state);
        add_sub(rec, 2, plc_stop_data_, ODA_SDO_RW, 1);
    }
    {
        Record &rec = add_record(0x1200);
        add_sub(rec, 0, store_u8(2), ODA_SDO_R, 1);
        add_sub(rec, 1, store_u32(0x600u + node), ODA_SDO_R | ODA_MB, 4);
        add_sub(rec, 2, store_u32(0x580u + node), ODA_SDO_R | ODA_MB, 4);
    }
    add_pdos(false);
    add_pdos(true);

    comm_defaults_ = defaults_.size();

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
        app_.emplace_back();
        AppIndex &app  = app_.back();
        app.self       = this;
        app.ext.object = &app;
        app.ext.read   = OD_readOriginal;
        app.ext.write  = app_write;
        Record &rec    = add_record(index);
        add_sub(rec, 0, store_u8(objs[end - 1]->subindex), ODA_SDO_R, 1);
        for (size_t k = i; k < end; ++k)
        {
            const SlaveObject &o    = *objs[k];
            app.objects[o.subindex] = &o;
            counts_.emplace_back(0u);
            app.counts[o.subindex] = &counts_.back();
            counters_.push_back({o.index, o.subindex, &counts_.back()});
            const OD_size_t len     = static_cast<OD_size_t>(o.bytes());
            uint8_t *p              = store(len);
            if (o.string)
                std::memcpy(p, o.text.data(), o.text.size());
            else
                std::memcpy(p, &o.value, len);
            defaults_.push_back({p, std::vector<uint8_t>(p, p + len)});
            OD_attr_t attr = 0;
            if (o.exch == SlaveExchange::Constant)
                attr = ODA_SDO_R;
            else if (o.dir == SlaveDirection::FromMaster)
                attr = ODA_SDO_RW | (o.exch == SlaveExchange::Pdo ? ODA_RPDO : 0);
            else
                attr = ODA_SDO_R | (o.exch == SlaveExchange::Pdo ? ODA_TPDO : 0);
            attr |= o.string ? static_cast<OD_attr_t>(ODA_STR) : multi_byte(len);
            add_sub(rec, o.subindex, p, attr, len);
            if (o.exch == SlaveExchange::Constant)
                continue;
            Slot slot{&o, p, nullptr};
            (o.dir == SlaveDirection::FromMaster ? from_master_ : to_master_).push_back(slot);
        }
        i = end;
    }

    finish();

    // Entries of the application slots and extensions, now that the list is final.
    size_t a = 0;
    for (auto &entry : list_)
    {
        if (entry.index < 0x2000 || entry.index > 0x5FFF)
            continue;
        AppIndex &app = app_[a++];
        OD_extension_init(&entry, &app.ext);
        for (auto *slots : {&from_master_, &to_master_})
            for (auto &s : *slots)
                if (s.obj->index == entry.index)
                    s.entry = &entry;
    }
    if (OD_entry_t *e1029 = find(0x1029))
    {
        error_ext_.object = this;
        error_ext_.read   = OD_readOriginal;
        error_ext_.write  = error_write;
        OD_extension_init(e1029, &error_ext_);
    }

    config_.CNT_NMT      = 1;
    config_.ENTRY_H1017  = find(0x1017);
    config_.CNT_HB_CONS  = 1;
    config_.CNT_ARR_1016 = 1;
    config_.ENTRY_H1016  = find(0x1016);
    config_.CNT_EM       = 1;
    config_.ENTRY_H1001  = find(0x1001);
    config_.ENTRY_H1014  = find(0x1014);
    config_.ENTRY_H1015  = nullptr;
    config_.CNT_ARR_1003 = 8;
    config_.ENTRY_H1003  = find(0x1003);
    config_.CNT_SDO_SRV  = 1;
    config_.ENTRY_H1200  = find(0x1200);
    config_.CNT_SDO_CLI  = 0;
    config_.CNT_TIME     = 0;
    config_.CNT_SYNC     = 1;
    config_.ENTRY_H1005  = find(0x1005);
    config_.ENTRY_H1006  = find(0x1006);
    config_.ENTRY_H1007  = find(0x1007);
    config_.ENTRY_H1019  = nullptr;
    config_.CNT_RPDO     = kSlavePdos;
    config_.ENTRY_H1400  = find(0x1400);
    config_.ENTRY_H1600  = find(0x1600);
    config_.CNT_TPDO     = kSlavePdos;
    config_.ENTRY_H1800  = find(0x1800);
    config_.ENTRY_H1A00  = find(0x1A00);
    config_.CNT_LEDS     = 0;
    config_.CNT_GFC      = 0;
    config_.CNT_SRDO     = 0;
    config_.CNT_LSS_SLV  = dev.lss.enabled ? 1 : 0;
    config_.CNT_LSS_MST  = 0;
    config_.CNT_GTWA     = 0;
    if (!config_.ENTRY_H1017 || !config_.ENTRY_H1400 || !config_.ENTRY_H1800)
    {
        problem = "the object dictionary is incomplete";
        return false;
    }
    return true;
}

void SlaveOd::finish()
{
    std::sort(pending_.begin(), pending_.end(),
              [](const Pending &a, const Pending &b) { return a.index < b.index; });
    list_.clear();
    list_.reserve(pending_.size() + 1);
    for (const auto &p : pending_)
    {
        OD_entry_t e{};
        e.index = p.index;
        if (p.type == ODT_VAR)
        {
            e.subEntriesCount = 1;
            e.odObjectType    = ODT_VAR;
            e.odObject        = p.object;
        }
        else
        {
            Record &rec       = records_[p.record];
            e.subEntriesCount = static_cast<uint8_t>(rec.subs.size());
            e.odObjectType    = ODT_REC;
            e.odObject        = rec.subs.data();
        }
        e.extension = nullptr;
        list_.push_back(e);
    }
    list_.push_back(OD_entry_t{0x0000, 0x00, 0, nullptr, nullptr});
    od_.size = static_cast<uint16_t>(list_.size() - 1);
    od_.list = list_.data();
}

OD_entry_t *SlaveOd::find(uint16_t index)
{
    for (size_t i = 0; i + 1 < list_.size(); ++i)
        if (list_[i].index == index)
            return &list_[i];
    return nullptr;
}

void SlaveOd::restore_defaults(Restore what)
{
    const size_t first = what == Restore::Application ? comm_defaults_ : 0;
    const size_t last  = what == Restore::Communication ? comm_defaults_ : defaults_.size();
    for (size_t i = first; i < last; ++i)
        std::memcpy(defaults_[i].first, defaults_[i].second.data(), defaults_[i].second.size());
    if (error_data_)
        error_behaviour_ = *error_data_;
    written_ = true; // the scan takes the restored values
}

const std::atomic<uint32_t> *SlaveOd::write_count(uint16_t index, uint8_t subindex) const
{
    for (const Counter &c : counters_)
        if (c.index == index && c.subindex == subindex)
            return c.count;
    return nullptr;
}

bool SlaveOd::take_written()
{
    bool w   = written_;
    written_ = false;
    return w;
}

uint64_t SlaveOd::raw_value(const Slot &slot)
{
    uint64_t v = 0;
    std::memcpy(&v, slot.data, std::min<size_t>(slot.obj->bytes(), sizeof(v)));
    return v;
}

void SlaveOd::set_to_master(Slot &slot, uint64_t raw)
{
    const size_t len = slot.obj->bytes();
    if (std::memcmp(slot.data, &raw, len) == 0)
        return;
    std::memcpy(slot.data, &raw, len);
    if (slot.entry && slot.obj->exch == SlaveExchange::Pdo)
        OD_requestTPDO(slot.entry, slot.obj->subindex);
}

ODR_t SlaveOd::app_write(OD_stream_t *stream, const void *buf, OD_size_t count,
                         OD_size_t *countWritten)
{
    AppIndex *app        = static_cast<AppIndex *>(stream->object);
    const SlaveObject *o = app->objects[stream->subIndex];
    if (o && !o->string && (o->has_min || o->has_max) && count == o->bytes())
    {
        uint64_t raw = 0;
        std::memcpy(&raw, buf, count);
        if (o->has_min && compare_raw(o->type, raw, o->min) < 0)
            return ODR_VALUE_LOW;
        if (o->has_max && compare_raw(o->type, raw, o->max) > 0)
            return ODR_VALUE_HIGH;
    }
    ODR_t r = OD_writeOriginal(stream, buf, count, countWritten);
    if (r == ODR_OK && o && o->dir == SlaveDirection::FromMaster)
    {
        app->self->written_ = true;
        if (std::atomic<uint32_t> *c = app->counts[stream->subIndex])
            c->fetch_add(1, std::memory_order_relaxed);
    }
    return r;
}

ODR_t SlaveOd::error_write(OD_stream_t *stream, const void *buf, OD_size_t count,
                           OD_size_t *countWritten)
{
    SlaveOd *self = static_cast<SlaveOd *>(stream->object);
    if (stream->subIndex == 2 && count == 1)
    {
        if (*static_cast<const uint8_t *>(buf) > kErrorStopped)
            return ODR_INVALID_VALUE;
        return OD_writeOriginal(stream, buf, count, countWritten);
    }
    if (stream->subIndex == 1 && count == 1)
    {
        const uint8_t v = *static_cast<const uint8_t *>(buf);
        if (v > kErrorStopped)
            return ODR_INVALID_VALUE;
        ODR_t r = OD_writeOriginal(stream, buf, count, countWritten);
        if (r == ODR_OK)
            self->error_behaviour_ = v;
        return r;
    }
    return OD_writeOriginal(stream, buf, count, countWritten);
}

} // namespace canopen_plugin
