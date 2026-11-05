// SPDX-License-Identifier: Apache-2.0
//
// lss_online.h - the LSS requests of a client (CiA 305): find the devices without a node ID
// (fastscan until none answers), give one device, chosen by its LSS address, a node ID, and set
// one device's bit rate. Some devices take part in a fastscan although they have a node ID, so
// at the end of a search each device found is selected on its own and asked its node ID, and
// only those without one (255) are in the result. The master's frames (0x7E5) and the devices'
// answers (0x7E4) go through the caller, so the same sequence runs on a running network and in
// service mode, where Lely's own LSS master is not started. Every sequence ends with all devices
// in the LSS waiting state.

#ifndef CANOPEN_LSS_ONLINE_H
#define CANOPEN_LSS_ONLINE_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace canopen_plugin
{

struct LssAddress4
{
    uint32_t vendor_id     = 0;
    uint32_t product_code  = 0;
    uint32_t revision      = 0;
    uint32_t serial_number = 0;

    bool operator==(const LssAddress4 &o) const
    {
        return vendor_id == o.vendor_id && product_code == o.product_code &&
               revision == o.revision && serial_number == o.serial_number;
    }
};

// The JSON object of an LSS address: {"vendor_id":..,"product_code":..,"revision":..,
// "serial_number":..}.
std::string lss_address_json(const LssAddress4 &a);

// The index of a bit rate (bit/s) in the LSS bit timing table, or -1 when it has none.
int lss_bitrate_index(unsigned bitrate);

class LssOnline
{
  public:
    using clock = std::chrono::steady_clock;
    using Frame = std::array<uint8_t, 8>;

    enum class Action : uint8_t
    {
        Find,
        SetId,
        SetBitrate
    };

    struct Request
    {
        Action action = Action::Find;
        LssAddress4 address;  // SetId, SetBitrate
        unsigned node_id = 0; // SetId
        bool store       = false;
        unsigned bitrate = 0; // SetBitrate, bit/s; always stored
    };

    struct Outcome
    {
        bool ok = false;
        // Failed: not_found (no device with the address answered), no_answer, refused,
        // store_refused (set but not stored), too_many (Find stopped at kMaxDevices), incomplete
        // (Find stopped by a device answering out of turn). Find fills `devices` in both.
        std::string reason;
        std::vector<LssAddress4> devices; // Find: the devices without a node ID
        unsigned previous_id = 0xFF;      // SetId: 255 when the device had none
        bool stored          = false;
    };

    // A device answers within this; one fastscan step nobody answers costs it once.
    static constexpr std::chrono::milliseconds kTimeout{100};
    // After an answer the next frame waits this long; answers meanwhile (more devices answering
    // the same frame) are dropped.
    static constexpr std::chrono::milliseconds kGap{20};
    // Find stops after this many devices.
    static constexpr size_t kMaxDevices = 64;

    // `send` puts one master frame (COB-ID 0x7E5) on the bus.
    explicit LssOnline(std::function<void(const Frame &)> send) : send_(std::move(send)) {}

    bool running() const
    {
        return step_ != Step::Idle;
    }
    // Starts a sequence; `found` is called for each device Find finds, `done` once at the end.
    void start(const Request &req, clock::time_point now,
               std::function<void(const LssAddress4 &)> found,
               std::function<void(const Outcome &)> done);
    // A frame of a device (COB-ID 0x7E4).
    void receive(const uint8_t *data, size_t len, clock::time_point now);
    // Sends the next queued frame and handles a timeout; called every few milliseconds.
    void tick(clock::time_point now);

  private:
    enum class Step : uint8_t
    {
        Idle,
        FindReset,    // anybody without a node ID?
        FindBit,      // one bit of one part of the address
        FindConfirm,  // the part is complete
        CheckSelect,  // a device found, selected on its own
        CheckInquire, // and asked its node ID
        Selected,     // switch selective sent, waiting for its answer
        Inquire,      // inquire node ID
        ConfigureId,
        ConfigureBitrate,
        Store
    };

    void queue(std::initializer_list<uint8_t> bytes);
    void queue_u32(uint8_t cs, uint32_t v);
    void fastscan(uint32_t id, uint8_t bit, uint8_t sub, uint8_t next);
    void expect(Step step, uint8_t cs, clock::time_point now);
    void answered(const uint8_t *data, size_t len, clock::time_point now);
    void timed_out(clock::time_point now);
    void after_id(clock::time_point now);
    void finish(bool ok, const std::string &reason);
    void store_or_finish(clock::time_point now);
    void select(const LssAddress4 &a);
    void check_next(clock::time_point now);

    std::function<void(const Frame &)> send_;
    std::function<void(const LssAddress4 &)> found_;
    std::function<void(const Outcome &)> done_;
    Request req_;
    Outcome out_;
    Step step_         = Step::Idle;
    uint8_t expect_cs_ = 0;
    clock::time_point deadline_{};
    clock::time_point hold_until_{};
    std::deque<Frame> pending_;
    // Fastscan: the address found so far, the part (0 vendor ID .. 3 serial number) and the bit.
    uint32_t id_[4] = {0, 0, 0, 0};
    uint8_t sub_    = 0;
    int bit_        = 31;
    // The devices the search found, and the one being checked.
    std::vector<LssAddress4> found_list_;
    size_t check_ = 0;
};

} // namespace canopen_plugin

#endif // CANOPEN_LSS_ONLINE_H
