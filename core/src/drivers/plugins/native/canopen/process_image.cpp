// SPDX-License-Identifier: Apache-2.0

#include "process_image.h"

#include <cstring>

namespace canopen_plugin
{

bool ProcessImage::build(const NetworkConfig &net, const MasterLayout &layout, int image_size,
                         std::vector<std::string> &errors)
{
    size_t failed = errors.size();
    auto check    = [&](const IecLocation &loc)
    {
        if (image_size > 0 && loc.index >= static_cast<uint32_t>(image_size))
            errors.push_back(net.label() + ": location " + loc.str() +
                             " lies outside the runtime's I/O image (" +
                             std::to_string(image_size) + " entries)");
    };
    in_locs_.clear();
    out_locs_.clear();
    for (const MasterPdo &m : layout.rpdos)
        for (const PdoEntry &e : m.pdo->entries)
        {
            check(e.location);
            in_locs_.push_back(e.location);
        }
    for (const MasterPdo &m : layout.tpdos)
        for (const PdoEntry &e : m.pdo->entries)
        {
            check(e.location);
            out_locs_.push_back(e.location);
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
    in_work_.assign(in_locs_.size() + net.nodes.size(), 0);
    out_work_.assign(out_locs_.size(), 0);
    in_.resize(in_work_.size());
    out_.resize(out_work_.size());
    scans_.store(0, std::memory_order_release);
    return errors.size() == failed;
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
    for (size_t i = 0; i < in_locs_.size(); ++i)
        image_write_input(rt, in_locs_[i], in[i]);
    for (size_t k = 0; k < has_state_.size(); ++k)
        if (has_state_[k])
            image_write_input(rt, state_locs_[k], in[in_locs_.size() + k]);
}

void ProcessImage::copy_from_plc(const plugin_runtime_args_t &rt)
{
    rt.image_lock();
    for (size_t i = 0; i < out_locs_.size(); ++i)
        out_work_[i] = image_read_output(rt, out_locs_[i]);
    rt.image_unlock();
    publish(out_, out_work_);
    scans_.fetch_add(1, std::memory_order_acq_rel);
}

} // namespace canopen_plugin
