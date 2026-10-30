// SPDX-License-Identifier: Apache-2.0

#include "client.h"

#include <mutex>

namespace canopen_plugin
{

constexpr size_t ClientQueue::kMaxWaiting;
constexpr size_t ClientQueue::kMaxData;
constexpr std::chrono::seconds ClientQueue::kKeep;
std::atomic<uint64_t> ClientQueue::next_id_{1};

namespace
{
struct NmtName
{
    NmtRequest request;
    const char *name;
};
const NmtName kNmtNames[] = {
    {NmtRequest::Start, "start"},
    {NmtRequest::Stop, "stop"},
    {NmtRequest::PreOperational, "pre_operational"},
    {NmtRequest::ResetNode, "reset_node"},
    {NmtRequest::ResetCommunication, "reset_communication"},
};
} // namespace

const char *nmt_request_name(NmtRequest r)
{
    for (const NmtName &n : kNmtNames)
        if (n.request == r)
            return n.name;
    return "start";
}

bool parse_nmt_request(const std::string &name, NmtRequest &out)
{
    for (const NmtName &n : kNmtNames)
        if (name == n.name)
        {
            out = n.request;
            return true;
        }
    return false;
}

uint64_t ClientQueue::submit(ClientJob job, std::string &reason)
{
    std::lock_guard<PiMutex> lock(mutex_);
    expire(clock::now());
    if (!running_)
    {
        reason = "not_running";
        return 0;
    }
    if (waiting_.size() >= kMaxWaiting)
    {
        reason = "queue_full";
        return 0;
    }
    job.id = next_id_.fetch_add(1);
    jobs_[job.id];
    waiting_.push_back(std::move(job));
    return waiting_.back().id;
}

bool ClientQueue::result(uint64_t id, JobResult &out)
{
    std::lock_guard<PiMutex> lock(mutex_);
    expire(clock::now());
    auto it = jobs_.find(id);
    if (it == jobs_.end())
        return false;
    out = it->second.result;
    return true;
}

void ClientQueue::set_running(bool running)
{
    std::lock_guard<PiMutex> lock(mutex_);
    running_ = running;
    if (running)
        return;
    waiting_.clear();
    auto now = clock::now();
    for (auto &it : jobs_)
        if (it.second.result.state == JobResult::State::Pending)
        {
            it.second.result.state  = JobResult::State::Failed;
            it.second.result.reason = "not_running";
            it.second.finished      = now;
        }
}

bool ClientQueue::next(ClientJob &job)
{
    std::lock_guard<PiMutex> lock(mutex_);
    if (waiting_.empty())
        return false;
    job = std::move(waiting_.front());
    waiting_.pop_front();
    return true;
}

void ClientQueue::finish(uint64_t id, JobResult r)
{
    std::lock_guard<PiMutex> lock(mutex_);
    auto it = jobs_.find(id);
    // Already failed when the session ended.
    if (it == jobs_.end() || it->second.result.state != JobResult::State::Pending)
        return;
    it->second.result   = std::move(r);
    it->second.finished = clock::now();
}

void ClientQueue::expire(clock::time_point now)
{
    for (auto it = jobs_.begin(); it != jobs_.end();)
    {
        if (it->second.result.state != JobResult::State::Pending &&
            now - it->second.finished > kKeep)
            it = jobs_.erase(it);
        else
            ++it;
    }
}

std::string to_hex(const std::vector<uint8_t> &bytes)
{
    static const char digits[] = "0123456789abcdef";
    std::string s;
    s.reserve(bytes.size() * 2);
    for (uint8_t b : bytes)
    {
        s.push_back(digits[b >> 4]);
        s.push_back(digits[b & 0xF]);
    }
    return s;
}

bool from_hex(const std::string &text, std::vector<uint8_t> &out)
{
    if (text.size() % 2)
        return false;
    auto digit = [](char c) -> int
    {
        if (c >= '0' && c <= '9')
            return c - '0';
        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
        return -1;
    };
    out.clear();
    out.reserve(text.size() / 2);
    for (size_t i = 0; i < text.size(); i += 2)
    {
        int hi = digit(text[i]);
        int lo = digit(text[i + 1]);
        if (hi < 0 || lo < 0)
            return false;
        out.push_back(static_cast<uint8_t>(hi << 4 | lo));
    }
    return true;
}

} // namespace canopen_plugin
