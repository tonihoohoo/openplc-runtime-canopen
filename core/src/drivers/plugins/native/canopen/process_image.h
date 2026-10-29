// SPDX-License-Identifier: Apache-2.0
//
// process_image.h - the values one network exchanges with the PLC scan, passed through lock-free
// triple buffers so neither the bus thread nor the scan ever waits for the other.
//
// Inputs:  the bus thread sets values and node states, then commit() publishes a snapshot;
//          cycle_start writes the newest snapshot to %I through the runtime's journal.
// Outputs: cycle_end reads %Q under the image lock into a snapshot, publishes it and wakes the
//          bus thread through an eventfd (one non-blocking write); the bus thread takes the
//          newest one.
// Nothing here allocates, blocks or logs after build().

#ifndef CANOPEN_PROCESS_IMAGE_H
#define CANOPEN_PROCESS_IMAGE_H

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "config.h"
#include "image_io.h"
#include "master_dcf.h"
#include "plugin_types.h"

namespace canopen_plugin
{

class ProcessImage
{
  public:
    ProcessImage() = default;
    ~ProcessImage();
    ProcessImage(const ProcessImage &)            = delete;
    ProcessImage &operator=(const ProcessImage &) = delete;

    // Builds the tables in master layout order. False, with a message per location, when a
    // location lies outside an image of `image_size` entries per table (0: no limit).
    bool build(const NetworkConfig &net, const MasterLayout &layout, int image_size,
               std::vector<std::string> &errors);

    size_t inputs() const
    {
        return in_locs_.size();
    }
    size_t outputs() const
    {
        return out_locs_.size();
    }

    // ---- bus thread ----
    void set_input(size_t i, uint64_t raw)
    {
        in_work_[i] = raw;
    }
    // `slot` is the node's position in the network's node list.
    void set_node_state(size_t slot, uint8_t state)
    {
        in_work_[in_locs_.size() + slot] = state;
    }
    void commit()
    {
        publish(in_, in_work_);
    }
    const uint64_t *latest_outputs(bool *fresh = nullptr)
    {
        return out_.latest(fresh);
    }
    // Readable after each completed scan. drain_wake() clears it.
    int wake_fd() const
    {
        return wake_fd_;
    }
    void drain_wake();
    // Scans completed since build().
    uint64_t scans() const
    {
        return scans_.load(std::memory_order_acquire);
    }

    // ---- PLC scan ----
    void copy_to_plc(const plugin_runtime_args_t &rt);
    void copy_from_plc(const plugin_runtime_args_t &rt);

  private:
    static void publish(TripleBuffer &buf, const std::vector<uint64_t> &work);

    std::vector<IecLocation> in_locs_;
    std::vector<IecLocation> out_locs_;
    std::vector<bool> has_state_;
    std::vector<IecLocation> state_locs_;
    std::vector<uint64_t> in_work_;
    std::vector<uint64_t> out_work_;
    TripleBuffer in_;
    TripleBuffer out_;
    std::atomic<uint64_t> scans_{0};
    int wake_fd_ = -1;
};

} // namespace canopen_plugin

#endif // CANOPEN_PROCESS_IMAGE_H
