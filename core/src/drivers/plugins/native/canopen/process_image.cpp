// SPDX-License-Identifier: Apache-2.0

#include "process_image.h"

#include <algorithm>
#include <cstdio>

#include <cerrno>
#include <cstring>

#include <sys/eventfd.h>
#include <unistd.h>

namespace canopen_plugin
{

namespace
{
// The eventfd calls cannot block and their failures need no handling (EAGAIN: nothing pending, or
// a counter that is already set).
void ignore_result(ssize_t) {}
} // namespace

ProcessImage::~ProcessImage()
{
    if (wake_fd_ >= 0)
        close(wake_fd_);
}

void ProcessImage::drain_wake()
{
    uint64_t n = 0;
    if (wake_fd_ >= 0)
        ignore_result(read(wake_fd_, &n, sizeof(n)));
}

namespace
{

size_t size_slot(IecSize size)
{
    switch (size)
    {
    case IecSize::X:
        return 0;
    case IecSize::B:
        return 1;
    case IecSize::W:
        return 2;
    case IecSize::D:
        return 3;
    case IecSize::L:
        return 4;
    }
    return 0;
}

} // namespace

ImageSizes ImageSizes::uniform(int entries)
{
    ImageSizes sizes;
    sizes.bounded = entries > 0;
    for (size_t i = 0; i < 5; ++i)
    {
        sizes.input[i]  = entries > 0 ? static_cast<uint32_t>(entries) : 0;
        sizes.output[i] = sizes.input[i];
    }
    return sizes;
}

uint32_t ImageSizes::capacity(const IecLocation &loc) const
{
    const size_t slot = size_slot(loc.size);
    return loc.area == IecArea::Input ? input[slot] : output[slot];
}

bool ImageSizes::contains(const IecLocation &loc) const
{
    // The runtime's journal takes a 16-bit index: nothing above it can be written.
    return loc.index <= kMaxJournalIndex && (!bounded || loc.index < capacity(loc));
}

bool slave_addresses_inside(const DeviceConfig &dev, const ImageSizes &sizes,
                            std::vector<std::string> &errors)
{
    const size_t failed = errors.size();
    auto check          = [&](const SlaveObject &o, const IecLocation &loc)
    {
        if (!sizes.contains(loc))
        {
            char obj[16];
            std::snprintf(obj, sizeof obj, "%04Xh:%02X", o.index, o.subindex);
            errors.push_back(dev.label() + ": object " + obj + " location " + loc.str() +
                             " lies outside the runtime's I/O image (" +
                             std::to_string(std::min<uint32_t>(sizes.capacity(loc),
                                                                kMaxJournalIndex + 1)) +
                             " entries in its table)");
        }
    };
    for (const SlaveObject &o : dev.objects)
    {
        if (o.packed())
            for (const IecLocation &b : o.bits)
                check(o, b);
        else if (o.exch != SlaveExchange::Constant)
            check(o, o.location);
    }
    return errors.size() == failed;
}

void ProcessImage::add_slot(const PdoEntry &e, std::vector<Slot> &slots,
                            std::vector<IecLocation> &bit_locs)
{
    Slot slot;
    slot.used = !e.dummy;
    if (e.packed())
    {
        slot.first_bit = static_cast<uint32_t>(bit_locs.size());
        slot.bits      = static_cast<uint8_t>(e.bits.size());
        bit_locs.insert(bit_locs.end(), e.bits.begin(), e.bits.end());
    }
    else
        slot.loc = e.location;
    slots.push_back(slot);
}

bool ProcessImage::build_slots(const NetworkConfig &net, const MasterLayout &layout,
                               const ImageSizes &sizes, std::vector<std::string> &errors)
{
    size_t failed = errors.size();
    auto check    = [&](const IecLocation &loc)
    {
        if (!sizes.contains(loc))
            errors.push_back(net.label() + ": location " + loc.str() +
                             " lies outside the runtime's I/O image (" +
                             std::to_string(sizes.capacity(loc)) + " entries in its table)");
    };
    auto check_entry = [&](const PdoEntry &e)
    {
        if (e.dummy)
            return;
        if (!e.packed())
            check(e.location);
        for (const IecLocation &b : e.bits)
            check(b);
    };
    in_slots_.clear();
    out_slots_.clear();
    in_bits_.clear();
    out_bits_.clear();
    for (const MasterPdo &m : layout.rpdos)
        for (const PdoEntry &e : m.pdo->entries)
        {
            check_entry(e);
            add_slot(e, in_slots_, in_bits_);
        }
    for (const MasterPdo &m : layout.tpdos)
        for (const PdoEntry &e : m.pdo->entries)
        {
            check_entry(e);
            add_slot(e, out_slots_, out_bits_);
        }
    has_state_.clear();
    state_locs_.clear();
    for (const NodeConfig &n : net.nodes)
    {
        has_state_.push_back(n.has_state_location);
        state_locs_.push_back(n.state_location);
        if (n.has_state_location)
            check(n.state_location);
    }
    return errors.size() == failed;
}

bool ProcessImage::build(const NetworkConfig &net, const MasterLayout &layout,
                         const ImageSizes &sizes, std::vector<std::string> &errors)
{
    size_t failed = errors.size();
    build_slots(net, layout, sizes, errors);
    in_count_ = in_slots_.size();
    in_work_.assign(in_slots_.size() + net.nodes.size(), 0);
    out_work_.assign(out_slots_.size(), 0);
    in_.resize(in_work_.size());
    out_.resize(out_work_.size());
    scans_.store(0, std::memory_order_release);
    if (wake_fd_ < 0)
    {
        wake_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
        if (wake_fd_ < 0)
            errors.push_back(net.label() + ": cannot create the scan wake-up (" +
                             std::strerror(errno) + ")");
    }
    return errors.size() == failed;
}

bool ProcessImage::relocate(const NetworkConfig &net, const MasterLayout &layout,
                            const ImageSizes &sizes, std::vector<std::string> &errors)
{
    const size_t failed = errors.size();
    const size_t outs   = out_slots_.size();
    const size_t nodes  = has_state_.size();
    build_slots(net, layout, sizes, errors);
    if (in_slots_.size() != in_count_ || out_slots_.size() != outs || has_state_.size() != nodes)
        errors.push_back(net.label() + ": the PDO layout differs from the running network's");
    // Zero outputs until the first scan, as with a new image; the scan side writes this buffer.
    std::fill(out_work_.begin(), out_work_.end(), 0);
    publish(out_, out_work_);
    return errors.size() == failed;
}

void ProcessImage::mark_all_down(const NetworkConfig &net, const MasterLayout &layout)
{
    for (size_t slot = 0; slot < net.nodes.size() && in_count_ + slot < in_work_.size(); ++slot)
        set_node_state(slot, 0);
    for (const MasterPdo &m : layout.rpdos)
    {
        bool zero = false;
        for (const NodeConfig &n : net.nodes)
            zero = zero || (n.node_id == m.node_id && n.on_loss == OnLoss::Zero);
        if (!zero)
            continue;
        for (size_t k = 0; k < m.pdo->entries.size() && m.first_entry + k < in_count_; ++k)
            set_input(m.first_entry + k, 0);
    }
    commit();
}

void ProcessImage::publish(TripleBuffer &buf, const std::vector<uint64_t> &work)
{
    if (!work.empty())
        std::memcpy(buf.back(), work.data(), work.size() * sizeof(uint64_t));
    buf.publish();
}

void ProcessImage::copy_to_plc(const plugin_runtime_args_t &rt)
{
    const uint64_t *in = in_.latest();
    for (size_t i = 0; i < in_slots_.size(); ++i)
    {
        const Slot &s = in_slots_[i];
        if (!s.used)
            continue;
        if (s.bits == 0)
            image_write_input(rt, s.loc, in[i]);
        for (unsigned k = 0; k < s.bits; ++k)
            image_write_input(rt, in_bits_[s.first_bit + k], (in[i] >> k) & 1);
    }
    for (size_t k = 0; k < has_state_.size(); ++k)
        if (has_state_[k])
            image_write_input(rt, state_locs_[k], in[in_slots_.size() + k]);
}

void ProcessImage::copy_from_plc(const plugin_runtime_args_t &rt)
{
    rt.image_lock();
    for (size_t i = 0; i < out_slots_.size(); ++i)
    {
        const Slot &s = out_slots_[i];
        uint64_t v    = 0;
        if (s.used && s.bits == 0)
            v = image_read_output(rt, s.loc);
        else if (s.used)
            for (unsigned k = 0; k < s.bits; ++k)
                v |= (image_read_output(rt, out_bits_[s.first_bit + k]) ? uint64_t(1) : 0) << k;
        out_work_[i] = v;
    }
    rt.image_unlock();
    publish(out_, out_work_);
    scans_.fetch_add(1, std::memory_order_acq_rel);
    const uint64_t one = 1;
    if (wake_fd_ >= 0)
        ignore_result(write(wake_fd_, &one, sizeof(one)));
}

} // namespace canopen_plugin
