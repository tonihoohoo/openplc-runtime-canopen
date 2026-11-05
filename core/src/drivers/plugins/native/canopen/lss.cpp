// SPDX-License-Identifier: Apache-2.0
//
// lss.cpp - node IDs over LSS (CiA 305). At every reset communication of the master, before any
// node boots, Lely calls LssAssigner::OnStart(); the network then gives each node with an LSS
// address its node ID: switch selective (with no revision configured, the device file's revision
// first, then a slowscan over the revision), inquire the node ID, configure it only when it
// differs, store when asked, and switch every device back to waiting. A device that had another
// node ID is reset (communication) so that it boots with the new one. A node whose device did not
// answer, or that was lost, gets the assignment again before its next boot attempt. One LSS
// sequence runs at a time. Runs on the bus thread like the rest of the network.
//
// Derived from canworks f90f8a7, plugin/src/canopen/network_lss.cpp (Apache-2.0).

#include <cstdio>
#include <mutex>
#include <set>
#include <string>

#include "log.h"
#include "network.h"

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

} // namespace canopen_plugin
