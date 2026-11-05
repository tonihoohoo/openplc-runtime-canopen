// SPDX-License-Identifier: Apache-2.0
//
// lss.cpp - node IDs over LSS (CiA 305). At every reset communication of the master, before any
// node boots, Lely calls LssAssigner::OnStart(); the network then gives each node with an LSS
// address its node ID: switch selective (with no revision configured, the device file's revision
// first, then a slowscan over the revision), inquire the node ID, configure it only when it
// differs, store when asked, and switch every device back to waiting. A device that had another
// node ID is reset (communication) so that it boots with the new one. A node whose device did not
// answer, or that was lost, gets the assignment again before its next boot attempt. One LSS
// sequence runs at a time, the clients' LSS requests (lss_online.h) included. Runs on the bus
// thread like the rest of the network.
//
// Derived from canworks f90f8a7, plugin/src/canopen/network_lss.cpp (Apache-2.0).

#include <array>
#include <cstdio>
#include <memory>
#include <mutex>
#include <set>
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

using lely::canopen::LssAddress;
using lely::canopen::LssState;

namespace
{

constexpr unsigned kNoId = 0xFF;

std::string hex32(uint32_t v)
{
    char buf[16];
    std::snprintf(buf, sizeof buf, "0x%08X", v);
    return buf;
}

std::string address_text(const LssConfig &c)
{
    std::string s = "vendor ID " + hex32(c.vendor_id) + ", product code " + hex32(c.product_code) +
                    ", serial number " + hex32(c.serial_number);
    if (c.revision)
        s += ", revision " + hex32(c.revision);
    return s;
}

// Why an LSS request failed, in words.
std::string lss_error_text(std::error_code ec)
{
    if (ec == std::errc::timed_out)
        return "no answer";
    if (ec == std::errc::result_out_of_range)
        return "node ID out of range for the device";
    if (ec == std::errc::protocol_error)
        return "refused by the device";
    if (ec == std::errc::operation_not_permitted)
        return "LSS is not running on the master";
    return ec.message();
}

// The nodes whose node ID was stored with store on, since the plugin was loaded. A node with store
// on that is not listed gets one store even when its device already has the right ID, since LSS
// cannot tell whether that ID is in the device's memory.
std::mutex g_stored_mutex;
std::set<std::string> g_stored;

std::string stored_key(const std::string &interface, unsigned id, const LssConfig &c)
{
    return interface + "/" + std::to_string(id) + "/" + hex32(c.vendor_id) + hex32(c.product_code) +
           hex32(c.revision) + hex32(c.serial_number);
}

bool stored_before(const std::string &key)
{
    std::lock_guard<std::mutex> lock(g_stored_mutex);
    return g_stored.count(key) != 0;
}

void set_stored(const std::string &key, bool stored)
{
    std::lock_guard<std::mutex> lock(g_stored_mutex);
    if (stored)
        g_stored.insert(key);
    else
        g_stored.erase(key);
}

} // namespace

LssAssigner::LssAssigner(ev_exec_t *exec, Network &net) : LssMaster(exec, net), net_(net) {}

void LssAssigner::OnStart(std::function<void(std::error_code ec)> res) noexcept
{
    net_.LssOnStart(std::move(res));
}

void Network::LssOnStart(std::function<void(std::error_code)> res)
{
    auto job = std::make_shared<LssJob>();
    for (const auto &it : nodes_)
        if (it.second.cfg->has_lss && it.second.enabled)
            job->ids.push_back(it.first);
    if (job->ids.empty() || !running_)
    {
        res({});
        return;
    }
    // Lely's start-up waits for this; nothing else runs LSS before it.
    lss_busy_     = true;
    lss_starting_ = true;
    job->done     = [this, res]
    {
        lss_starting_ = false;
        res({});
    };
    LssRun(job);
}

bool Network::LssRecover(NodeState &node, unsigned id, clock::time_point now)
{
    if (!lss_ || lss_busy_ || now < node.lss_next)
        return false;
    auto job = std::make_shared<LssJob>();
    job->ids.push_back(id);
    lss_busy_ = true;
    LssRun(job);
    return true;
}

void Network::LssRun(std::shared_ptr<LssJob> job)
{
    if (job->next >= job->ids.size())
    {
        lss_busy_ = false;
        if (job->done)
            job->done();
        return;
    }
    const unsigned id  = job->ids[job->next];
    NodeState &n       = nodes_[id];
    const LssConfig &c = n.cfg->lss;
    n.lss_running      = true;
    auto found         = [this, job, id](std::error_code ec)
    {
        if (!ec)
        {
            LssFound(job, id);
            return;
        }
        NodeState &n = nodes_[id];
        if (ec == std::errc::timed_out)
            log_warn("%s: no device with %s answered LSS", n.cfg->label().c_str(),
                     address_text(n.cfg->lss).c_str());
        else
            log_warn("%s: LSS search failed: %s", n.cfg->label().c_str(),
                     lss_error_text(ec).c_str());
        LssDone(job, id, LssOutcome::NotFound, 0, false);
    };
    try
    {
        if (c.revision)
        {
            lss_->SubmitSwitchSelective(
                exec_, LssAddress(c.vendor_id, c.product_code, c.revision, c.serial_number), found);
            return;
        }
        // Revision unknown: the device file's first (one request), then a binary search over it
        // with the rest fixed (each step nobody answers costs one LSS timeout).
        auto search = [this, &c, found](std::error_code ec)
        {
            if (ec != std::errc::timed_out)
            {
                found(ec);
                return;
            }
            try
            {
                lss_->SubmitSlowscan(
                    exec_, LssAddress(c.vendor_id, c.product_code, 0, c.serial_number),
                    LssAddress(c.vendor_id, c.product_code, 0xFFFFFFFFu, c.serial_number),
                    [found](std::error_code ec, LssAddress) { found(ec); });
            }
            catch (const std::exception &)
            {
                found(ec);
            }
        };
        if (n.cfg->revision)
            lss_->SubmitSwitchSelective(
                exec_, LssAddress(c.vendor_id, c.product_code, n.cfg->revision, c.serial_number),
                search);
        else
            search(std::make_error_code(std::errc::timed_out));
    }
    catch (const std::exception &e)
    {
        log_error("%s: cannot start LSS: %s", n.cfg->label().c_str(), e.what());
        LssDone(job, id, LssOutcome::NotFound, 0, false);
    }
}

// The device is in LSS configuration state: inquire its node ID, set it when it differs, store it
// when asked.
void Network::LssFound(std::shared_ptr<LssJob> job, unsigned id)
{
    lss_->SubmitGetId(
        exec_,
        [this, job, id](std::error_code ec, uint8_t prev)
        {
            NodeState &n            = nodes_[id];
            const std::string label = n.cfg->label();
            if (ec)
            {
                log_warn("%s: LSS inquire node ID failed: %s", label.c_str(),
                         lss_error_text(ec).c_str());
                LssDone(job, id, LssOutcome::NotFound, 0, false);
                return;
            }
            const std::string key = stored_key(cfg_.interface, id, n.cfg->lss);
            const bool store      = n.cfg->lss.store;
            if (!store)
                set_stored(key, false);
            auto store_then = [this, job, id, prev, key, label](LssOutcome outcome)
            {
                lss_->SubmitStore(
                    exec_,
                    [this, job, id, prev, key, label, outcome](std::error_code ec)
                    {
                        if (ec)
                        {
                            log_warn("%s: LSS store configuration refused (%s); the device "
                                     "loses node ID %u at power-off",
                                     label.c_str(), lss_error_text(ec).c_str(), id);
                            LssDone(job, id, LssOutcome::StoreRefused, prev, false);
                            return;
                        }
                        set_stored(key, true);
                        log_info("%s: LSS stored node ID %u in the device", label.c_str(), id);
                        LssDone(job, id, outcome, prev, true);
                    });
            };
            if (prev == id)
            {
                if (store && !stored_before(key))
                {
                    store_then(LssOutcome::Already);
                    return;
                }
                LssDone(job, id, LssOutcome::Already, prev, false);
                return;
            }
            lss_->SubmitSetId(exec_, static_cast<uint8_t>(id),
                              [this, job, id, prev, label, store, store_then](std::error_code ec)
                              {
                                  if (ec)
                                  {
                                      log_error("%s: LSS: the device refused node ID %u (%s)",
                                                label.c_str(), id, lss_error_text(ec).c_str());
                                      LssDone(job, id, LssOutcome::Refused, prev, false);
                                      return;
                                  }
                                  const std::string was =
                                      prev == kNoId ? "none" : std::to_string(prev);
                                  log_info("%s: LSS gave the device node ID %u (it had: %s)",
                                           label.c_str(), id, was.c_str());
                                  if (store)
                                      store_then(LssOutcome::Given);
                                  else
                                      LssDone(job, id, LssOutcome::Given, prev, false);
                              });
        });
}

// Ends one node's sequence: every device back to the LSS waiting state (a device that had no node
// ID starts with its new one), a device that had another ID reset, then the next node.
void Network::LssDone(std::shared_ptr<LssJob> job, unsigned id, LssOutcome outcome,
                      unsigned previous, bool stored)
{
    NodeState &n  = nodes_[id];
    n.lss_running = false;
    status_.lss_result(id, outcome, previous, stored);
    const bool ok = outcome == LssOutcome::Given || outcome == LssOutcome::Already ||
                    outcome == LssOutcome::StoreRefused;
    if (ok)
    {
        n.lss_due     = false;
        n.lss_backoff = kRetryMin;
        n.next_poll   = clock::now(); // looked for at once with its new ID
    }
    else
    {
        n.lss_due     = true;
        n.lss_next    = clock::now() + n.lss_backoff;
        n.lss_backoff = std::min(n.lss_backoff * 2, kRetryMax);
    }
    const bool reset = ok && outcome != LssOutcome::Already && previous != kNoId && previous != id;
    ++job->next;
    try
    {
        lss_->SubmitSwitch(exec_, LssState::WAITING,
                           [this, job, reset, previous](std::error_code)
                           {
                               if (reset)
                                   SendNmt(NmtRequest::ResetCommunication,
                                           static_cast<uint8_t>(previous));
                               LssRun(job);
                           });
    }
    catch (const std::exception &e)
    {
        log_error("LSS: cannot switch devices to the waiting state: %s", e.what());
        LssRun(job);
    }
}

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
    if (lss_busy_ || lss_starting_ || lss_online_.running())
        refused.reason = "busy";
    else if (req.action == LssOnline::Action::SetBitrate && !service_)
        refused.reason = "needs_service_mode";
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
