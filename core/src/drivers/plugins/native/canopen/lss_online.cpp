// SPDX-License-Identifier: Apache-2.0
//
// lss_online.cpp - see lss_online.h. Command specifiers as in CiA 305: 0x04 switch state global,
// 0x40-0x43 switch state selective (answer 0x44), 0x11 configure node ID, 0x13 configure bit
// timing, 0x17 store configuration, 0x5E inquire node ID, 0x51 fastscan (answer 0x4F).
//
// The client requests are modelled on canworks f90f8a7, plugin/src/canopen/network_lss.cpp
// (Apache-2.0).

#include "lss_online.h"

#include <cstdio>

namespace canopen_plugin
{

constexpr std::chrono::milliseconds LssOnline::kTimeout;
constexpr std::chrono::milliseconds LssOnline::kGap;
constexpr size_t LssOnline::kMaxDevices;

namespace
{

constexpr uint8_t kSwitchGlobal = 0x04;
constexpr uint8_t kSelectVendor = 0x40;
constexpr uint8_t kSelectAnswer = 0x44;
constexpr uint8_t kSetId        = 0x11;
constexpr uint8_t kSetBitrate   = 0x13;
constexpr uint8_t kStore        = 0x17;
constexpr uint8_t kFastscan     = 0x51;
constexpr uint8_t kIdentify     = 0x4F;
constexpr uint8_t kInquireId    = 0x5E;

} // namespace

std::string lss_address_json(const LssAddress4 &a)
{
    return "{\"vendor_id\":" + std::to_string(a.vendor_id) +
           ",\"product_code\":" + std::to_string(a.product_code) +
           ",\"revision\":" + std::to_string(a.revision) +
           ",\"serial_number\":" + std::to_string(a.serial_number) + "}";
}

int lss_bitrate_index(unsigned bitrate)
{
    static const unsigned kTable[] = {1000000, 800000, 500000, 250000, 125000,
                                      0,       50000,  20000,  10000};
    for (int i = 0; i < static_cast<int>(sizeof kTable / sizeof kTable[0]); ++i)
        if (kTable[i] && kTable[i] == bitrate)
            return i;
    return -1;
}

void LssOnline::queue(std::initializer_list<uint8_t> bytes)
{
    Frame f{};
    size_t i = 0;
    for (uint8_t b : bytes)
        f[i++] = b;
    pending_.push_back(f);
}

void LssOnline::queue_u32(uint8_t cs, uint32_t v)
{
    queue({cs, static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v >> 16),
           static_cast<uint8_t>(v >> 24)});
}

void LssOnline::fastscan(uint32_t id, uint8_t bit, uint8_t sub, uint8_t next)
{
    queue({kFastscan, static_cast<uint8_t>(id), static_cast<uint8_t>(id >> 8),
           static_cast<uint8_t>(id >> 16), static_cast<uint8_t>(id >> 24), bit, sub, next});
}

void LssOnline::expect(Step step, uint8_t cs, clock::time_point now)
{
    step_      = step;
    expect_cs_ = cs;
    // The timeout runs from the last frame sent (tick() moves it while frames wait).
    deadline_ = now + kTimeout;
    tick(now);
}

void LssOnline::start(const Request &req, clock::time_point now,
                      std::function<void(const LssAddress4 &)> found,
                      std::function<void(const Outcome &)> done)
{
    req_        = req;
    out_        = Outcome();
    found_list_.clear();
    hold_until_ = clock::time_point{};
    found_ = std::move(found);
    done_  = std::move(done);
    pending_.clear();
    // Every device starts from the waiting state.
    queue({kSwitchGlobal, 0x00});
    if (req.action == Action::Find)
    {
        fastscan(0, 0x80, 0, 0);
        expect(Step::FindReset, kIdentify, now);
        return;
    }
    select(req.address);
    expect(Step::Selected, kSelectAnswer, now);
}

void LssOnline::select(const LssAddress4 &a)
{
    queue_u32(kSelectVendor, a.vendor_id);
    queue_u32(kSelectVendor + 1, a.product_code);
    queue_u32(kSelectVendor + 2, a.revision);
    queue_u32(kSelectVendor + 3, a.serial_number);
}

void LssOnline::check_next(clock::time_point now)
{
    if (check_ >= found_list_.size())
    {
        // The closing switch to waiting is already queued.
        out_.ok    = out_.reason.empty();
        expect_cs_ = 0;
        return;
    }
    select(found_list_[check_]);
    expect(Step::CheckSelect, kSelectAnswer, now);
}

void LssOnline::tick(clock::time_point now)
{
    if (step_ == Step::Idle)
        return;
    if (!pending_.empty())
    {
        if (now < hold_until_)
            return;
        send_(pending_.front());
        pending_.pop_front();
        deadline_ = now + kTimeout;
        return;
    }
    if (expect_cs_ == 0)
    {
        // The closing switch to waiting went out: done.
        step_ = Step::Idle;
        auto done = std::move(done_);
        found_    = nullptr;
        if (done)
            done(out_);
        return;
    }
    if (now >= deadline_)
        timed_out(now);
}

void LssOnline::receive(const uint8_t *data, size_t len, clock::time_point now)
{
    // An answer before the frame it would answer went out belongs to the frame before.
    if (step_ == Step::Idle || expect_cs_ == 0 || !pending_.empty() || len < 1 ||
        data[0] != expect_cs_)
        return;
    hold_until_ = now + kGap;
    answered(data, len, now);
}

void LssOnline::answered(const uint8_t *data, size_t len, clock::time_point now)
{
    switch (step_)
    {
    case Step::FindReset:
        for (uint32_t &v : id_)
            v = 0;
        sub_ = 0;
        bit_ = 31;
        fastscan(id_[sub_], static_cast<uint8_t>(bit_), sub_, sub_);
        expect(Step::FindBit, kIdentify, now);
        return;
    case Step::FindBit:
        // A device has this bit 0.
        after_id(now);
        return;
    case Step::FindConfirm:
        if (sub_ < 3)
        {
            ++sub_;
            bit_ = 31;
            fastscan(id_[sub_], static_cast<uint8_t>(bit_), sub_, sub_);
            expect(Step::FindBit, kIdentify, now);
            return;
        }
        {
            // The device is in the configuration state and takes no part in the next search.
            LssAddress4 a;
            a.vendor_id     = id_[0];
            a.product_code  = id_[1];
            a.revision      = id_[2];
            a.serial_number = id_[3];
            // A device answering out of turn (in the configuration state) makes the search read
            // an address of zeros or one already found: the others may be hidden behind it.
            bool ghost = a == LssAddress4();
            for (const LssAddress4 &d : found_list_)
                ghost = ghost || d == a;
            if (ghost)
            {
                out_.reason = "incomplete";
                queue({kSwitchGlobal, 0x00});
                check_ = 0;
                check_next(now);
                return;
            }
            found_list_.push_back(a);
            if (found_)
                found_(a);
            if (found_list_.size() >= kMaxDevices)
            {
                // The ones found are still checked.
                out_.reason = "too_many";
                queue({kSwitchGlobal, 0x00});
                check_ = 0;
                check_next(now);
                return;
            }
        }
        fastscan(0, 0x80, 0, 0);
        expect(Step::FindReset, kIdentify, now);
        return;
    case Step::CheckSelect:
        queue({kInquireId});
        expect(Step::CheckInquire, kInquireId, now);
        return;
    case Step::CheckInquire:
        if (len < 2 || data[1] == 0xFF)
            out_.devices.push_back(found_list_[check_]);
        queue({kSwitchGlobal, 0x00});
        ++check_;
        check_next(now);
        return;
    case Step::Selected:
        if (req_.action == Action::SetId)
        {
            queue({kInquireId});
            expect(Step::Inquire, kInquireId, now);
            return;
        }
        queue({kSetBitrate, 0x00, static_cast<uint8_t>(lss_bitrate_index(req_.bitrate))});
        expect(Step::ConfigureBitrate, kSetBitrate, now);
        return;
    case Step::Inquire:
        out_.previous_id = len > 1 ? data[1] : 0xFF;
        queue({kSetId, static_cast<uint8_t>(req_.node_id)});
        expect(Step::ConfigureId, kSetId, now);
        return;
    case Step::ConfigureId:
    case Step::ConfigureBitrate:
        if (len < 2 || data[1] != 0)
        {
            finish(false, "refused");
            return;
        }
        store_or_finish(now);
        return;
    case Step::Store:
        if (len < 2 || data[1] != 0)
        {
            finish(false, "store_refused");
            return;
        }
        out_.stored = true;
        finish(true, "");
        return;
    case Step::Idle:
        return;
    }
}

void LssOnline::timed_out(clock::time_point now)
{
    switch (step_)
    {
    case Step::FindReset:
        // Nobody without a node ID is left: each device found is checked on its own.
        queue({kSwitchGlobal, 0x00});
        check_ = 0;
        check_next(now);
        return;
    case Step::FindBit:
        // No device has this bit 0: it is 1.
        id_[sub_] |= uint32_t{1} << bit_;
        after_id(now);
        return;
    case Step::Selected:
        finish(false, "not_found");
        return;
    case Step::CheckSelect:
    case Step::CheckInquire:
    {
        // Its node ID is unknown: the device stays in the result.
        out_.devices.push_back(found_list_[check_]);
        queue({kSwitchGlobal, 0x00});
        ++check_;
        check_next(now);
        return;
    }
    default:
        finish(false, "no_answer");
        return;
    }
}

void LssOnline::after_id(clock::time_point now)
{
    if (--bit_ >= 0)
    {
        fastscan(id_[sub_], static_cast<uint8_t>(bit_), sub_, sub_);
        expect(Step::FindBit, kIdentify, now);
        return;
    }
    fastscan(id_[sub_], 0, sub_, static_cast<uint8_t>((sub_ + 1) & 3));
    expect(Step::FindConfirm, kIdentify, now);
}

void LssOnline::store_or_finish(clock::time_point now)
{
    if (req_.store || req_.action == Action::SetBitrate)
    {
        queue({kStore});
        expect(Step::Store, kStore, now);
        return;
    }
    finish(true, "");
}

void LssOnline::finish(bool ok, const std::string &reason)
{
    out_.ok     = ok;
    out_.reason = reason;
    // Every device back to waiting; a device without a node ID starts with its new one.
    queue({kSwitchGlobal, 0x00});
    expect_cs_ = 0;
}

} // namespace canopen_plugin
