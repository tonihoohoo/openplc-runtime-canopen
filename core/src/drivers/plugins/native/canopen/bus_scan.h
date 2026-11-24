// SPDX-License-Identifier: Apache-2.0
//
// bus_scan.h - a client's bus scan in service mode for commissioning: every node ID 1-127 (but
// `skip`, when set) is asked for its device type (0x1000) and, when it answers, its identity
// (0x1018 sub 1-4), at most kParallel IDs at a time, one try per read. An SDO abort is an answer
// too. Heartbeat, boot-up and emergency frames heard meanwhile mark their IDs as heard. The scan
// sends no NMT command and writes nothing. The reads go through the caller, so the sequence is
// tested without a bus; it runs on the network's bus thread.

#ifndef CANOPEN_BUS_SCAN_H
#define CANOPEN_BUS_SCAN_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace canopen_plugin
{

class BusScan
{
  public:
    static constexpr unsigned kParallel  = 8;
    static constexpr unsigned kTimeoutMs = 200; // per read, for the caller's SDO client

    enum class Answer : uint8_t
    {
        Value,  // the value was read
        Abort,  // the device answered with an SDO abort
        Timeout // no answer
    };

    struct Device
    {
        unsigned node_id = 0;
        bool answered    = false;
        bool heartbeat   = false;
        bool boot_up     = false;
        bool emcy        = false;
        // Read values; `has` bit 0 device type, bits 1-4 0x1018 sub 1-4.
        uint8_t has          = 0;
        uint32_t device_type = 0;
        std::array<uint32_t, 4> identity{};
    };

    // `read` starts one read of a UNSIGNED32 object and calls `done` once with its answer; the
    // call may come before `read` returns.
    using Done = std::function<void(Answer answer, uint32_t value)>;
    using Read = std::function<void(unsigned node, uint16_t index, uint8_t subindex, Done done)>;

    // `skip`: a node ID not asked; 0 for none.
    BusScan(unsigned skip, Read read) : master_(skip), read_(std::move(read)) {}

    bool running() const
    {
        return running_;
    }
    // Starts the scan; `progress` is called after each ID's reads end, `done` once at the end.
    void start(std::function<void()> progress, std::function<void()> done);
    // A frame heard on the bus (11-bit COB-ID): 0x701-0x77F heartbeat or boot-up, 0x081-0x0FF
    // emergency.
    void heard(uint32_t cob_id, const uint8_t *data, size_t len);

    // The IDs that answered or were heard, by node ID, as a JSON array.
    std::string devices_json() const;
    const std::vector<Device> &devices() const
    {
        return devices_;
    }

  private:
    struct Slot
    {
        unsigned node = 0;
        int step      = 0; // 0 device type, 1-4 identity sub-index
    };

    Device &entry(unsigned node);
    void launch();
    void next_read(size_t slot);
    void answered(size_t slot, Answer answer, uint32_t value);
    void finish_id(size_t slot);

    unsigned master_;
    Read read_;
    std::function<void()> progress_;
    std::function<void()> done_;
    bool running_     = false;
    unsigned next_id_ = 1;
    unsigned busy_    = 0;
    std::array<Slot, kParallel> slots_{};
    std::vector<Device> devices_; // sorted by node ID
};

} // namespace canopen_plugin

#endif // CANOPEN_BUS_SCAN_H
