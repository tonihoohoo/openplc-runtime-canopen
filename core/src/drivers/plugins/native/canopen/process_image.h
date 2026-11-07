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

// Length of each %I and %Q table of the runtime's image, in its own unit (bytes for X, entries
// otherwise), indexed by IecSize position X, B, W, D, L. Unbounded: no limit is checked.
// Highest index the runtime's journal calls take (16 bits).
constexpr uint32_t kMaxJournalIndex = 0xFFFF;

struct ImageSizes
{
    bool bounded       = false;
    uint32_t input[5]  = {0, 0, 0, 0, 0};
    uint32_t output[5] = {0, 0, 0, 0, 0};

    // Every table `entries` long; 0 or less is unbounded.
    static ImageSizes uniform(int entries);
    uint32_t capacity(const IecLocation &loc) const;
    // Whether `loc` lies inside its table.
    bool contains(const IecLocation &loc) const;
};

// Whether every address of a slave's objects (packed bits included) lies inside the tables.
// False, with a message per address, when one does not.
bool slave_addresses_inside(const DeviceConfig &dev, const ImageSizes &sizes,
                            std::vector<std::string> &errors);

class ProcessImage
{
  public:
    ProcessImage() = default;
    ~ProcessImage();
    ProcessImage(const ProcessImage &)            = delete;
    ProcessImage &operator=(const ProcessImage &) = delete;

    // Builds the tables in master layout order. False, with a message per location, when a
    // location lies outside its table of `sizes`.
    bool build(const NetworkConfig &net, const MasterLayout &layout, const ImageSizes &sizes,
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
    // Every node down without a session: state 0, inputs of nodes set to zero on loss zeroed,
    // then published. `net` and `layout` are the ones build() was given.
    void mark_all_down(const NetworkConfig &net, const MasterLayout &layout);
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
    // 0 for a dummy entry: no location, read as 0 and never written to the image.
    std::vector<uint8_t> in_used_;
    std::vector<uint8_t> out_used_;
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
