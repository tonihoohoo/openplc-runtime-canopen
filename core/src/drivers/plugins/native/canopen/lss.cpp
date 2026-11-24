// SPDX-License-Identifier: Apache-2.0
//
// lss.cpp - a client's LSS requests (lss_online.h) on the network's bus: the receiver of the
// devices' answers (0x7E4), the master's frames (0x7E5) and the job's result. One LSS sequence
// runs at a time. Runs on the bus thread like the rest of the network.

#include <array>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

#include <lely/can/msg.h>
#include <lely/ev/exec.hpp>

#include "log.h"
#include "network.h"

extern "C" int can_net_send(__can_net *net, const can_msg *msg);
extern "C" __can_recv *can_recv_create(void);
extern "C" void can_recv_set_func(__can_recv *recv, int (*func)(const can_msg *, void *),
                                  void *data);
extern "C" void can_recv_start(__can_recv *recv, __can_net *net, uint_least32_t id,
                               uint_least8_t flags);

namespace canopen_plugin
{

namespace
{

constexpr unsigned kNoId = 0xFF;

std::string hex32(uint32_t v)
{
    char buf[16];
    std::snprintf(buf, sizeof buf, "0x%08X", v);
    return buf;
}

} // namespace

void Network::WatchLss()
{
    std::lock_guard<lely::util::BasicLockable> lock(*this);
    if (lss_recv_)
        return;
    lss_recv_ = can_recv_create();
    if (!lss_recv_)
    {
        log_warn("%s: cannot receive LSS answers", cfg_.label().c_str());
        return;
    }
    can_recv_set_func(lss_recv_, &Network::OnLssFrame, this);
    can_recv_start(lss_recv_, net(), 0x7E4, 0);
}

int Network::OnLssFrame(const can_msg *msg, void *data) noexcept
{
    auto *self = static_cast<Network *>(data);
    if (!self->lss_online_.running())
        return 0;
    std::array<uint8_t, 8> bytes{};
    const size_t len = msg->len < 8 ? msg->len : 8;
    for (size_t i = 0; i < len; ++i)
        bytes[i] = msg->data[i];
    // Lely holds the network's lock here, and an answer may send the next frame.
    try
    {
        lely::ev::Executor(self->exec_).post(
            [self, bytes, len] { self->lss_online_.receive(bytes.data(), len, clock::now()); });
    }
    catch (const std::exception &e)
    {
        log_error("cannot queue an LSS answer on the bus thread: %s", e.what());
    }
    return 0;
}

void Network::SendLss(const LssOnline::Frame &frame)
{
    can_msg msg = CAN_MSG_INIT;
    msg.id      = 0x7E5;
    msg.len     = 8;
    for (size_t i = 0; i < 8; ++i)
        msg.data[i] = frame[i];
    std::lock_guard<lely::util::BasicLockable> lock(*this);
    can_net_send(net(), &msg);
}

namespace
{

std::string address_text(const LssAddress4 &a)
{
    return "vendor ID " + hex32(a.vendor_id) + ", product code " + hex32(a.product_code) +
           ", revision " + hex32(a.revision) + ", serial number " + hex32(a.serial_number);
}

} // namespace

void Network::ClientLss(const ClientJob &job)
{
    const LssOnline::Request &req = job.lss;
    JobResult refused;
    refused.state = JobResult::State::Failed;
    if (lss_busy_ || lss_online_.running())
        refused.reason = "busy";
    else if (req.action == LssOnline::Action::SetBitrate && !service_ && !plc_stopped_)
        refused.reason = "needs_plc_stopped";
    else if (req.action == LssOnline::Action::SetId)
    {
        NodeState *n = Find(static_cast<uint8_t>(req.node_id));
        if (n && (n->up || n->state != 0))
            refused.reason = "node_in_use";
    }
    if (!refused.reason.empty())
    {
        client_->finish(job.id, refused);
        return;
    }
    lss_busy_         = true;
    client_busy_      = true;
    const uint64_t id = job.id;
    auto devices      = std::make_shared<std::string>();
    const std::string label = cfg_.label();
    if (req.action == LssOnline::Action::Find)
        log_info("%s: LSS search for devices without a node ID", label.c_str());
    lss_online_.start(
        req, clock::now(),
        [this, id, devices, label](const LssAddress4 &a)
        {
            if (!devices->empty())
                *devices += ",";
            *devices += lss_address_json(a);
            client_->progress(id, "\"devices\":[" + *devices + "]");
            log_info("%s: LSS search answered by the device with %s", label.c_str(),
                     address_text(a).c_str());
        },
        [this, id, req, devices, label](const LssOnline::Outcome &o)
        {
            lss_busy_    = false;
            client_busy_ = false;
            JobResult r;
            r.state  = o.ok ? JobResult::State::Done : JobResult::State::Failed;
            r.reason = o.reason;
            const std::string addr = address_text(req.address);
            switch (req.action)
            {
            case LssOnline::Action::Find:
            {
                std::string list;
                for (const LssAddress4 &d : o.devices)
                    list += (list.empty() ? "" : ",") + lss_address_json(d);
                r.json = "\"devices\":[" + list + "]";
                log_info("%s: LSS search ended, %zu device(s) found%s", label.c_str(),
                         o.devices.size(), o.ok ? "" : (" (" + o.reason + ")").c_str());
                break;
            }
            case LssOnline::Action::SetId:
                r.json = "\"previous_id\":" + std::to_string(o.previous_id) +
                         ",\"stored\":" + (o.stored ? "true" : "false");
                if (o.ok)
                    log_info("%s: LSS gave node ID %u to the device with %s (it had: %s)%s",
                             label.c_str(), req.node_id, addr.c_str(),
                             o.previous_id == kNoId ? "none"
                                                    : std::to_string(o.previous_id).c_str(),
                             o.stored ? ", stored" : "");
                else
                    log_warn("%s: LSS node ID %u for the device with %s: %s", label.c_str(),
                             req.node_id, addr.c_str(), o.reason.c_str());
                break;
            case LssOnline::Action::SetBitrate:
                r.json = std::string("\"stored\":") + (o.stored ? "true" : "false");
                if (o.ok)
                    log_info("%s: LSS set and stored %u bit/s in the device with %s; it uses the "
                             "rate after its next power cycle",
                             label.c_str(), req.bitrate, addr.c_str());
                else
                    log_warn("%s: LSS bit rate %u bit/s for the device with %s: %s",
                             label.c_str(), req.bitrate, addr.c_str(), o.reason.c_str());
                break;
            }
            client_->finish(id, r);
        });
}

} // namespace canopen_plugin
