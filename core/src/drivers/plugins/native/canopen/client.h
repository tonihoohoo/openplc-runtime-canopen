// SPDX-License-Identifier: Apache-2.0
//
// client.h - SDO and NMT requests from a client of the plugin's socket (the editor's online tools).
// The socket thread queues a job per request; the network's bus thread takes the jobs one at a
// time and records each result, which the client collects later with the job's id. Nothing here
// runs in the PLC scan.

#ifndef CANOPEN_CLIENT_H
#define CANOPEN_CLIENT_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "pi_mutex.h"

namespace canopen_plugin
{

enum class NmtRequest : uint8_t
{
    Start,
    Stop,
    PreOperational,
    ResetNode,
    ResetCommunication
};

const char *nmt_request_name(NmtRequest r);
bool parse_nmt_request(const std::string &name, NmtRequest &out);

struct ClientJob
{
    enum class Kind : uint8_t
    {
        Read,
        Write,
        Nmt
    };
    Kind kind        = Kind::Read;
    uint64_t id      = 0;
    unsigned node    = 0; // Nmt: 0 for every node of the network
    uint16_t index   = 0;
    uint8_t subindex = 0;
    std::vector<uint8_t> data; // Write
    NmtRequest command = NmtRequest::Start;
};

struct JobResult
{
    enum class State : uint8_t
    {
        Pending,
        Done,
        Failed
    };
    State state = State::Pending;
    std::vector<uint8_t> data; // Read
    bool has_abort      = false;
    uint32_t abort_code = 0;
    std::string reason; // Failed without an abort code: booting, not_running, ...
};

class ClientQueue
{
  public:
    using clock = std::chrono::steady_clock;

    static constexpr size_t kMaxWaiting = 16;
    static constexpr size_t kMaxData    = 65536;
    static constexpr std::chrono::seconds kKeep{30};

    // Socket thread: queues a job and returns its id, or 0 with `reason` (not_running,
    // queue_full).
    uint64_t submit(ClientJob job, std::string &reason);
    // Socket thread: the job's result; false for a job unknown or dropped after kKeep.
    bool result(uint64_t id, JobResult &out);

    // Bus thread: a session runs (true) or ended (false); ending fails every unfinished job.
    void set_running(bool running);
    // Bus thread: takes the next job.
    bool next(ClientJob &job);
    // Bus thread: records the result of a job taken with next().
    void finish(uint64_t id, JobResult r);

  private:
    struct Entry
    {
        JobResult result;
        clock::time_point finished{};
    };
    void expire(clock::time_point now);

    mutable PiMutex mutex_;
    bool running_ = false;
    std::deque<ClientJob> waiting_;
    std::map<uint64_t, Entry> jobs_;
    static std::atomic<uint64_t> next_id_;
};

// Hex text of bytes ("e803") and back; false for odd length or a non-hex digit.
std::string to_hex(const std::vector<uint8_t> &bytes);
bool from_hex(const std::string &text, std::vector<uint8_t> &out);

} // namespace canopen_plugin

#endif // CANOPEN_CLIENT_H
