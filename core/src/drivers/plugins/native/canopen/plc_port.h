// SPDX-License-Identifier: Apache-2.0
//
// plc_port.h - one network's side of the PLC function blocks (plc_api.h). The PLC scan queues SDO
// and NMT requests in fixed slots and reads node states, the bus state and emergencies; the
// network's bus thread takes the requests one at a time, alternating with the online tools'
// queue, and publishes what the scan reads. The scan side never allocates, locks or blocks: slots
// change hands through atomic states, and the published values are atomics or seqlocked entries.

#ifndef CANOPEN_PLC_PORT_H
#define CANOPEN_PLC_PORT_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "plc_api.h"

namespace canopen_plugin
{

// A request taken by the bus thread.
struct PlcJob
{
    uint32_t token      = 0; // passed back to finish()
    uint8_t kind        = CANOPEN_PLC_READ;
    uint8_t node        = 0;
    uint8_t command     = 0;
    uint16_t index      = 0;
    uint8_t subindex    = 0;
    uint32_t timeout_ms = 0;
    uint32_t length     = 0;       // write
    const uint8_t *data = nullptr; // write; valid until finish()
};

class PlcPort
{
  public:
    static constexpr size_t kSlots = 16;
    static constexpr size_t kEmcy  = 32;

    // `nonce` tells this port's handles from those of an earlier PLC run, `net` is the
    // network's index (0-based) carried in them; `nodes` lists the network's node IDs.
    PlcPort(uint32_t nonce, uint8_t net, uint8_t master_node_id, const unsigned *nodes,
            size_t count);

    // PLC scan.
    uint64_t start(const canopen_plc_request &req, canopen_plc_result &res);
    int poll(uint64_t handle, canopen_plc_result &res, uint8_t *data, uint32_t cap);
    void cancel(uint64_t handle);
    bool known(unsigned node) const;
    uint8_t device_state(unsigned node) const; // canopen_plc_device_state
    uint8_t kernel_state() const;              // canopen_plc_error
    uint8_t master_node_id() const
    {
        return master_;
    }
    uint64_t emcy_cursor() const;
    int emcy_next(uint8_t node, uint64_t &cursor, canopen_plc_emcy &out) const;

    // Bus thread.
    // A session runs (true) or ended (false); ending fails every request not finished.
    void set_running(bool running);
    // Takes the oldest queued request.
    bool take(PlcJob &job);
    // Ends a request taken with take(); `data` (a read) is copied.
    void finish(uint32_t token, uint8_t error, uint8_t reason, uint32_t abort_code,
                const uint8_t *data, uint32_t length);
    // The node's NMT state byte as the master tracks it, 0 while it is not up.
    void set_nmt(unsigned node, uint8_t state);
    // canopen_plc_error of the bus state.
    void set_kernel(uint8_t error);
    void push_emcy(uint8_t node, uint16_t code, uint8_t reg, const uint8_t data[5]);

    // Handle layout: nonce, slot generation, network index, slot.
    static uint64_t make_handle(uint32_t nonce, uint16_t gen, uint8_t net, uint8_t slot)
    {
        return (static_cast<uint64_t>(nonce) << 32) | (static_cast<uint64_t>(gen) << 16) |
               (static_cast<uint64_t>(net) << 8) | slot;
    }
    static uint8_t network_of(uint64_t handle)
    {
        return static_cast<uint8_t>((handle >> 8) & 0xFF);
    }

  private:
    enum State : uint32_t
    {
        Free,
        Filling,
        Queued,
        Running,
        Ended,
        Abandoned // cancelled while running; freed when it ends
    };
    struct Slot
    {
        std::atomic<uint32_t> state{Free};
        std::atomic<uint16_t> gen{0};
        uint64_t seq = 0; // queue order, written before Queued
        canopen_plc_request req{};
        canopen_plc_result res{};
        uint8_t data[CANOPEN_PLC_MAX_DATA] = {};
    };
    struct EmcyEntry
    {
        std::atomic<uint64_t> seq{0};  // 2 * position + 1 while written, 2 * position + 2 when done
        std::atomic<uint64_t> head{0}; // node, register, code, data bytes 0-3
        std::atomic<uint8_t> tail{0};  // data byte 4
    };

    Slot *slot_of(uint64_t handle);

    const uint32_t nonce_;
    const uint8_t net_;
    const uint8_t master_;
    bool known_[128] = {};
    std::atomic<uint8_t> nmt_[128];
    std::atomic<uint8_t> kernel_{CANOPEN_PLC_NO_ERROR};
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> next_seq_{1};
    Slot slots_[kSlots];
    EmcyEntry emcy_[kEmcy];
    std::atomic<uint64_t> emcy_head_{0}; // emergencies stored so far
};

// A handle nonce (never 0), different for each call, so each PLC run's ports differ.
uint32_t new_nonce();

// The device state block value of an NMT state byte (0 while the node is not up).
uint8_t device_state_of(uint8_t nmt);

} // namespace canopen_plugin

#endif // CANOPEN_PLC_PORT_H
