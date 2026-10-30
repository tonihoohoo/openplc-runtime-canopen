// SPDX-License-Identifier: Apache-2.0

#include "requests.h"

#include <cmath>
#include <cstring>

#include "cJSON.h"

namespace canopen_plugin
{

namespace
{

std::string error(const char *reason)
{
    return std::string("{\"error\":\"") + reason + "\"}";
}

// An integer field within [lo, hi]; false when missing, not a whole number or out of range.
bool get_uint(const cJSON *o, const char *key, unsigned lo, unsigned hi, unsigned &out)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, key);
    if (!cJSON_IsNumber(v))
        return false;
    double d = v->valuedouble;
    if (d != std::floor(d) || d < lo || d > hi)
        return false;
    out = static_cast<unsigned>(d);
    return true;
}

const char *lookup_error(Lookup l)
{
    switch (l)
    {
    case Lookup::NotRunning:
        return "not_running";
    case Lookup::UnknownNetwork:
        return "unknown_network";
    case Lookup::UnknownNode:
        return "unknown_node";
    case Lookup::Ok:
        break;
    }
    return "bad_request";
}

std::string queue_job(const cJSON *req, Requests &target, ClientJob job, bool nmt)
{
    unsigned net = 0;
    if (!get_uint(req, "network", 0, 255, net))
        return error("bad_request");
    ClientQueue *queue = nullptr;
    Lookup l           = target.network(net, job.node, queue);
    if (l != Lookup::Ok)
        return error(lookup_error(l));
    std::string reason;
    uint64_t id = queue->submit(std::move(job), reason);
    if (!id)
        return error(reason.c_str());
    if (nmt)
        return "{\"ok\":true}";
    return "{\"job\":" + std::to_string(id) + "}";
}

std::string sdo(const cJSON *req, Requests &target, bool write)
{
    ClientJob job;
    unsigned node = 0, index = 0, sub = 0;
    if (!get_uint(req, "node", 1, 127, node) || !get_uint(req, "index", 0, 0xFFFF, index) ||
        !get_uint(req, "subindex", 0, 0xFF, sub))
        return error("bad_request");
    job.kind     = write ? ClientJob::Kind::Write : ClientJob::Kind::Read;
    job.node     = node;
    job.index    = static_cast<uint16_t>(index);
    job.subindex = static_cast<uint8_t>(sub);
    if (write)
    {
        const cJSON *data = cJSON_GetObjectItemCaseSensitive(req, "data");
        if (!cJSON_IsString(data) || !from_hex(data->valuestring, job.data) || job.data.empty())
            return error("bad_request");
        if (job.data.size() > ClientQueue::kMaxData)
            return error("too_large");
    }
    return queue_job(req, target, std::move(job), false);
}

std::string nmt(const cJSON *req, Requests &target)
{
    ClientJob job;
    job.kind         = ClientJob::Kind::Nmt;
    unsigned node    = 0;
    const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(req, "command");
    if (!get_uint(req, "node", 0, 127, node) || !cJSON_IsString(cmd) ||
        !parse_nmt_request(cmd->valuestring, job.command))
        return error("bad_request");
    job.node = node;
    return queue_job(req, target, std::move(job), true);
}

std::string job_answer(const cJSON *req, Requests &target)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(req, "job");
    if (!cJSON_IsNumber(v) || v->valuedouble < 1 || v->valuedouble != std::floor(v->valuedouble))
        return error("bad_request");
    JobResult r;
    if (!target.job(static_cast<uint64_t>(v->valuedouble), r))
        return error("unknown_job");
    switch (r.state)
    {
    case JobResult::State::Pending:
        return "{\"state\":\"pending\"}";
    case JobResult::State::Done:
        if (r.data.empty())
            return "{\"state\":\"done\"}";
        return "{\"state\":\"done\",\"data\":\"" + to_hex(r.data) + "\"}";
    case JobResult::State::Failed:
        if (r.has_abort)
            return "{\"state\":\"failed\",\"abort_code\":" + std::to_string(r.abort_code) + "}";
        return "{\"state\":\"failed\",\"reason\":\"" + r.reason + "\"}";
    }
    return error("bad_request");
}

std::string service_open(const cJSON *req, Requests &target)
{
    ServiceRequest s;
    const cJSON *itf   = cJSON_GetObjectItemCaseSensitive(req, "interface");
    const cJSON *nodes = cJSON_GetObjectItemCaseSensitive(req, "nodes");
    if (!cJSON_IsString(itf) || !*itf->valuestring || std::strlen(itf->valuestring) > 15 ||
        !get_uint(req, "bitrate", 0, 1000000, s.bitrate) ||
        (s.bitrate && !is_standard_bitrate(s.bitrate)) ||
        !get_uint(req, "master_node_id", 1, 127, s.master_node_id) ||
        !get_uint(req, "sdo_timeout_ms", 1, 60000, s.sdo_timeout_ms) || !cJSON_IsArray(nodes))
        return error("bad_request");
    s.interface = itf->valuestring;
    const cJSON *n;
    cJSON_ArrayForEach(n, nodes)
    {
        if (!cJSON_IsNumber(n) || n->valuedouble != std::floor(n->valuedouble) ||
            n->valuedouble < 1 || n->valuedouble > 127)
            return error("bad_request");
        unsigned id = static_cast<unsigned>(n->valuedouble);
        if (id == s.master_node_id)
            return error("bad_request");
        for (unsigned seen : s.nodes)
            if (seen == id)
                return error("bad_request");
        s.nodes.push_back(id);
    }
    std::string reason = target.service_open(s);
    if (!reason.empty())
        return error(reason.c_str());
    return "{\"ok\":true,\"network\":0}";
}

} // namespace

NetworkConfig service_network(const ServiceRequest &req)
{
    NetworkConfig net;
    net.name           = "Service";
    net.interface      = req.interface;
    net.bitrate        = req.bitrate;
    net.master_node_id = req.master_node_id;
    net.sdo_timeout_ms = req.sdo_timeout_ms;
    net.on_stop        = OnStop::None;
    for (unsigned id : req.nodes)
    {
        NodeConfig n;
        n.node_id = id;
        n.name    = "node " + std::to_string(id);
        net.nodes.push_back(n);
    }
    return net;
}

std::string handle_request(const std::string &request, Requests &target)
{
    cJSON *req      = cJSON_Parse(request.c_str());
    const cJSON *op = req ? cJSON_GetObjectItemCaseSensitive(req, "op") : nullptr;
    std::string answer;
    if (!req || !cJSON_IsObject(req) || !cJSON_IsString(op))
        answer = error("bad_request");
    else if (std::strcmp(op->valuestring, "status") == 0)
        answer = target.status();
    else if (std::strcmp(op->valuestring, "sdo_read") == 0)
        answer = sdo(req, target, false);
    else if (std::strcmp(op->valuestring, "sdo_write") == 0)
        answer = sdo(req, target, true);
    else if (std::strcmp(op->valuestring, "nmt") == 0)
        answer = nmt(req, target);
    else if (std::strcmp(op->valuestring, "job") == 0)
        answer = job_answer(req, target);
    else if (std::strcmp(op->valuestring, "service_open") == 0)
        answer = service_open(req, target);
    else if (std::strcmp(op->valuestring, "service_close") == 0)
    {
        target.service_close();
        answer = "{\"ok\":true}";
    }
    else
        answer = error("unknown_op");
    cJSON_Delete(req);
    return answer;
}

} // namespace canopen_plugin
