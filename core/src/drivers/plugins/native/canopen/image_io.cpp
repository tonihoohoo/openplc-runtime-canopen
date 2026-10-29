// SPDX-License-Identifier: Apache-2.0
//
// Derived from canworks ec7fd59d, plugin/src/can/image_io.cpp.
//

#include "image_io.h"

#include <cstring>

namespace canopen_plugin
{

// ---------------------------------------------------------------------------
// TripleBuffer

void TripleBuffer::resize(size_t n)
{
    size_ = n;
    for (auto &b : bufs_)
        b.assign(n ? n : 1, 0);
    back_  = 0;
    front_ = 1;
    middle_.store(2, std::memory_order_relaxed);
}

void TripleBuffer::publish()
{
    // Hand the filled buffer over and take the spare one. The new back buffer
    // starts as a copy of the published one, so a producer that only updates
    // some slots keeps the others.
    uint8_t filled = back_;
    uint8_t prev   = middle_.exchange(filled | kDirty, std::memory_order_acq_rel);
    back_          = prev & 0x3;
    std::memcpy(bufs_[back_].data(), bufs_[filled].data(), bufs_[filled].size() * sizeof(uint64_t));
}

const uint64_t *TripleBuffer::latest(bool *fresh)
{
    bool got = false;
    if (middle_.load(std::memory_order_acquire) & kDirty)
    {
        uint8_t prev = middle_.exchange(front_, std::memory_order_acq_rel);
        front_       = prev & 0x3;
        got          = true;
    }
    if (fresh)
        *fresh = got;
    return bufs_[front_].data();
}

} // namespace canopen_plugin
