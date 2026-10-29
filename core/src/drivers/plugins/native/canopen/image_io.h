// SPDX-License-Identifier: Apache-2.0
//
// Derived from canworks ec7fd59d, plugin/src/can/image_io.h.
//
// image_io.h - moving values between a protocol's bus thread and the PLC
// I/O image: a lock-free triple buffer for snapshots, and the writes into
// %I* (through the runtime's journal) and reads of %Q* the scan hooks make.

#ifndef CANOPEN_IMAGE_IO_H
#define CANOPEN_IMAGE_IO_H

#include <atomic>
#include <cstdint>
#include <vector>

#include "iec_location.h"
#include "plugin_types.h"

namespace canopen_plugin
{

// Single-producer/single-consumer triple buffer of fixed-size uint64 arrays.
class TripleBuffer
{
  public:
    void resize(size_t n);
    size_t size() const
    {
        return size_;
    }

    // Producer: the buffer to fill, then publish().
    uint64_t *back()
    {
        return bufs_[back_].data();
    }
    void publish();

    // Consumer: returns the newest published snapshot (or the previous one if
    // nothing new was published). Never blocks.
    const uint64_t *latest(bool *fresh = nullptr);

  private:
    static constexpr uint8_t kDirty = 0x4;
    std::vector<uint64_t> bufs_[3];
    size_t size_   = 0;
    uint8_t back_  = 0;
    uint8_t front_ = 1;
    std::atomic<uint8_t> middle_{2};
};

// One input location written through the runtime's journal (cycle_start),
// and one output location read (cycle_end, with the image lock held).
// Journal buffer types (journal_buffer_type_t in the runtime).
enum : int
{
    kBoolInput = 0,
    kByteInput = 3,
    kIntInput  = 5,
    kDintInput = 8,
    kLintInput = 11,
};

inline void image_write_input(const plugin_runtime_args_t &rt, const IecLocation &loc, uint64_t v)
{
    switch (loc.size)
    {
    case IecSize::X:
        rt.journal_write_bool(kBoolInput, loc.index, loc.bit, v ? 1 : 0);
        break;
    case IecSize::B:
        rt.journal_write_byte(kByteInput, loc.index, static_cast<int>(v & 0xFF));
        break;
    case IecSize::W:
        rt.journal_write_int(kIntInput, loc.index, static_cast<int>(v & 0xFFFF));
        break;
    case IecSize::D:
        rt.journal_write_dint(kDintInput, loc.index, static_cast<unsigned>(v));
        break;
    case IecSize::L:
        rt.journal_write_lint(kLintInput, loc.index, static_cast<unsigned long long>(v));
        break;
    }
}

inline uint64_t image_read_output(const plugin_runtime_args_t &rt, const IecLocation &loc)
{
    // A location the program does not declare has a NULL pointer: read 0.
    switch (loc.size)
    {
    case IecSize::X:
    {
        IEC_BOOL *p = rt.bool_output[loc.index][loc.bit];
        return p ? (*p ? 1 : 0) : 0;
    }
    case IecSize::B:
    {
        IEC_BYTE *p = rt.byte_output[loc.index];
        return p ? *p : 0;
    }
    case IecSize::W:
    {
        IEC_UINT *p = rt.int_output[loc.index];
        return p ? *p : 0;
    }
    case IecSize::D:
    {
        IEC_UDINT *p = rt.dint_output[loc.index];
        return p ? *p : 0;
    }
    case IecSize::L:
    {
        IEC_ULINT *p = rt.lint_output[loc.index];
        return p ? *p : 0;
    }
    }
    return 0;
}

} // namespace canopen_plugin

#endif // CANOPEN_IMAGE_IO_H
