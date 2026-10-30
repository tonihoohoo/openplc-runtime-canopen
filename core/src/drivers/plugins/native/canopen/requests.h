// SPDX-License-Identifier: Apache-2.0
//
// requests.h - the JSON requests of the plugin's socket (docs/CANOPEN.md): the status, SDO read
// and write, NMT commands, job results, and opening and closing service mode. This file parses and
// answers them; what they act on is behind the Requests interface, which the plugin implements.

#ifndef CANOPEN_REQUESTS_H
#define CANOPEN_REQUESTS_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "client.h"
#include "config.h"

namespace canopen_plugin
{

// The network settings a client sends to open service mode.
struct ServiceRequest
{
    std::string name = "Service"; // the network's name in the status
    std::string interface;
    unsigned bitrate        = 0;
    unsigned master_node_id = 127;
    unsigned sdo_timeout_ms = 1000;
    std::vector<unsigned> nodes;
};

enum class Lookup
{
    Ok,
    NotRunning,     // no network runs
    UnknownNetwork, // no network with that index
    UnknownNode     // the node is not one of the network's
};

class Requests
{
  public:
    virtual ~Requests() = default;
    // The status answer.
    virtual std::string status() = 0;
    // The queue of network `net`; `node` 0 stands for every node of the network.
    virtual Lookup network(size_t net, unsigned node, ClientQueue *&queue) = 0;
    // The result of a job of any network; false for an unknown job.
    virtual bool job(uint64_t id, JobResult &out) = 0;
    // Opens service mode; an empty string on success, else the refusal reason.
    virtual std::string service_open(const ServiceRequest &req) = 0;
    virtual void service_close()                                = 0;
};

// The network service mode runs: the request's interface and nodes, nothing configured or
// supervised, and no NMT command when it ends.
NetworkConfig service_network(const ServiceRequest &req);

// Answers one request line with one JSON answer.
std::string handle_request(const std::string &request, Requests &target);

} // namespace canopen_plugin

#endif // CANOPEN_REQUESTS_H
