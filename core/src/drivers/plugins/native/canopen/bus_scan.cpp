// SPDX-License-Identifier: Apache-2.0

#include "bus_scan.h"

#include <algorithm>

#include "cJSON.h"

namespace canopen_plugin
{

constexpr unsigned BusScan::kParallel;
constexpr unsigned BusScan::kTimeoutMs;

BusScan::Device &BusScan::entry(unsigned node)
{
    auto it = std::lower_bound(devices_.begin(), devices_.end(), node,
                               [](const Device &d, unsigned id) { return d.node_id < id; });
    if (it == devices_.end() || it->node_id != node)
    {
        Device d;
        d.node_id = node;
        it        = devices_.insert(it, d);
    }
    return *it;
}

void BusScan::start(std::function<void()> progress, std::function<void()> done)
{
    progress_ = std::move(progress);
    done_     = std::move(done);
    devices_.clear();
    running_ = true;
    next_id_ = 1;
    busy_    = 0;
    for (Slot &s : slots_)
        s = Slot();
    launch();
}

void BusScan::launch()
{
    for (size_t i = 0; i < slots_.size() && running_; ++i)
    {
        if (slots_[i].node)
            continue;
        while (next_id_ == master_)
            ++next_id_;
        if (next_id_ > 127)
            break;
        slots_[i].node = next_id_++;
        slots_[i].step = 0;
        ++busy_;
        next_read(i);
    }
    if (running_ && busy_ == 0 && next_id_ > 127)
    {
        running_ = false;
        if (done_)
            done_();
    }
}

void BusScan::next_read(size_t slot)
{
    const Slot s         = slots_[slot];
    const uint16_t index = s.step == 0 ? 0x1000 : 0x1018;
    const uint8_t sub    = static_cast<uint8_t>(s.step == 0 ? 0 : s.step);
    read_(s.node, index, sub, [this, slot](Answer a, uint32_t v) { answered(slot, a, v); });
}

void BusScan::answered(size_t slot, Answer answer, uint32_t value)
{
    Slot &s = slots_[slot];
    if (!running_ || !s.node)
        return;
    if (s.step == 0)
    {
        if (answer == Answer::Timeout)
        {
            finish_id(slot);
            return;
        }
        Device &d  = entry(s.node);
        d.answered = true;
        if (answer == Answer::Value)
        {
            d.has |= 1;
            d.device_type = value;
        }
    }
    else if (answer == Answer::Value)
    {
        Device &d = entry(s.node);
        d.has |= static_cast<uint8_t>(1u << s.step);
        d.identity[static_cast<size_t>(s.step - 1)] = value;
    }
    if (s.step == 4)
    {
        finish_id(slot);
        return;
    }
    ++s.step;
    next_read(slot);
}

void BusScan::finish_id(size_t slot)
{
    slots_[slot] = Slot();
    --busy_;
    if (progress_)
        progress_();
    launch();
}

void BusScan::heard(uint32_t cob_id, const uint8_t *data, size_t len)
{
    if (!running_)
        return;
    if (cob_id >= 0x701 && cob_id <= 0x77F && len >= 1)
    {
        Device &d = entry(cob_id - 0x700);
        if ((data[0] & 0x7F) == 0)
            d.boot_up = true;
        else
            d.heartbeat = true;
    }
    else if (cob_id >= 0x081 && cob_id <= 0x0FF)
        entry(cob_id - 0x080).emcy = true;
}

std::string BusScan::devices_json() const
{
    static const char *const kIdentity[4] = {"vendor_id", "product_code", "revision",
                                             "serial_number"};
    cJSON *list                           = cJSON_CreateArray();
    for (const Device &d : devices_)
    {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "node_id", d.node_id);
        cJSON_AddBoolToObject(o, "answered", d.answered);
        cJSON *heard = cJSON_AddArrayToObject(o, "heard");
        if (d.heartbeat)
            cJSON_AddItemToArray(heard, cJSON_CreateString("heartbeat"));
        if (d.boot_up)
            cJSON_AddItemToArray(heard, cJSON_CreateString("boot_up"));
        if (d.emcy)
            cJSON_AddItemToArray(heard, cJSON_CreateString("emcy"));
        if (d.has & 1)
            cJSON_AddNumberToObject(o, "device_type", static_cast<double>(d.device_type));
        else
            cJSON_AddNullToObject(o, "device_type");
        for (size_t i = 0; i < 4; ++i)
        {
            if (d.has & (1u << (i + 1)))
                cJSON_AddNumberToObject(o, kIdentity[i], static_cast<double>(d.identity[i]));
            else
                cJSON_AddNullToObject(o, kIdentity[i]);
        }
        cJSON_AddItemToArray(list, o);
    }
    char *text      = cJSON_PrintUnformatted(list);
    std::string out = text ? text : "[]";
    cJSON_free(text);
    cJSON_Delete(list);
    return out;
}

} // namespace canopen_plugin
