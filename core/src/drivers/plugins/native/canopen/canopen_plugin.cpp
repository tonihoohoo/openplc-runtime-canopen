// SPDX-License-Identifier: Apache-2.0
//
// canopen_plugin.cpp - the native plugin's entry points (plugin_driver.h).
//
//   init:        copies the runtime arguments, routes the log and opens the plugin's socket (one
//                idle thread); it runs at power-on even when the plugin is disabled, so nothing
//                touches a bus until a PLC start or a service mode request
//   start_loop:  closes service mode, loads conf/canopen.json, builds each network's master DCF
//                and process image and starts one bus thread per network
//   cycle_start: writes the newest inputs to %I through the journal
//   cycle_end:   reads %Q under the image lock and hands it to the bus threads
//   stop_loop:   sends each network's NMT command for a PLC stop and joins the bus threads
//   cleanup:     closes service mode and the socket
// While the PLC is stopped, a client of the socket can open one network in service mode with the
// settings it sends; it closes on request, at a PLC start, at cleanup and after kServiceIdle.
// No exception leaves an entry point: the runtime calls them as C functions.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bus.h"
#include "config.h"
#include "log.h"
#include "master_dcf.h"
#include "plugin_types.h"
#include "process_image.h"
#include "requests.h"
#include "status.h"
#include "status_server.h"

using namespace canopen_plugin;

#define CANOPEN_EXPORT extern "C" __attribute__((visibility("default")))

namespace
{

struct Running
{
    Config cfg;
    std::vector<MasterLayout> layouts;
    std::vector<std::unique_ptr<ProcessImage>> images;
    std::vector<std::unique_ptr<NetworkStatus>> statuses;
    std::vector<std::unique_ptr<Bus>> buses;
};

plugin_runtime_args_t g_args;
bool g_have_args = false;
// g_run and g_service are replaced under g_mutex; the scan hooks read g_run behind g_active.
std::mutex g_mutex;
std::unique_ptr<Running> g_run;
std::unique_ptr<Running> g_service;
std::chrono::steady_clock::time_point g_last_request;
std::atomic<bool> g_active{false};
std::unique_ptr<StatusServer> g_server;

constexpr std::chrono::minutes kServiceIdle{5};

void runtime_sink(LogLevel level, const char *msg)
{
    plugin_log_info_func_t f = nullptr;
    switch (level)
    {
    case LogLevel::Debug:
        f = g_args.log_debug;
        break;
    case LogLevel::Info:
        f = g_args.log_info;
        break;
    case LogLevel::Warn:
        f = g_args.log_warn;
        break;
    case LogLevel::Error:
        f = g_args.log_error;
        break;
    }
    if (f)
        f("%s", msg);
    else
        std::fprintf(stderr, "%s\n", msg);
}

std::string config_dir(const std::string &path)
{
    size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
}

bool write_file(const std::string &path, const std::string &text)
{
    std::ofstream out(path, std::ios::trunc);
    out << text;
    return static_cast<bool>(out);
}

std::string status_answer(const Running &run)
{
    std::vector<NetworkSnapshot> nets;
    nets.reserve(run.statuses.size());
    for (const auto &s : run.statuses)
        nets.push_back(s->snapshot());
    return status_json(nets, wall_ms());
}

// Stops the buses of a run taken out of g_run or g_service.
void stop_run(std::unique_ptr<Running> run)
{
    if (!run)
        return;
    for (auto &bus : run->buses)
        bus->stop();
    run.reset();
    release_wake_signal();
}

void close_service(const char *why)
{
    std::unique_ptr<Running> svc;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        svc = std::move(g_service);
    }
    if (!svc)
        return;
    svc->statuses.front()->set_service(false);
    stop_run(std::move(svc));
    log_info("CANopen service mode closed (%s)", why);
}

std::string open_service(const ServiceRequest &req)
{
    std::unique_ptr<Running> run(new Running);
    run->cfg.networks.push_back(service_network(req));
    const NetworkConfig &cfg = run->cfg.networks.front();
    run->layouts.push_back(master_layout(cfg));
    std::vector<std::string> errors;
    std::unique_ptr<ProcessImage> image(new ProcessImage);
    if (!image->build(cfg, run->layouts.front(), 0, errors))
        return "error";
    run->images.push_back(std::move(image));
    std::string dir =
        g_have_args ? config_dir(g_args.plugin_specific_config_file_path) : std::string("/tmp");
    std::string dcf = dir + "/canopen_service.dcf";
    if (!write_file(dcf, master_dcf(cfg, run->layouts.front())))
    {
        log_error("cannot write the service mode configuration %s", dcf.c_str());
        return "error";
    }
    run->statuses.emplace_back(new NetworkStatus(cfg));
    run->statuses.front()->set_service(true);
    run->buses.emplace_back(new Bus(cfg, run->layouts.front(), *run->images.front(),
                                    *run->statuses.front(), dcf, nullptr, nullptr, true));
    prepare_wake_signal();
    run->buses.front()->start();
    g_service = std::move(run);
    log_info("CANopen service mode open on %s at %u bit/s, %zu node(s)", req.interface.c_str(),
             req.bitrate, req.nodes.size());
    return "";
}

class PluginRequests : public Requests
{
  public:
    std::string status() override
    {
        if (g_run)
            return status_answer(*g_run);
        if (g_service)
            return status_answer(*g_service);
        return R"({"version":1,"active":false})";
    }

    Lookup network(size_t net, unsigned node, ClientQueue *&queue) override
    {
        Running *run = current();
        if (!run)
            return Lookup::NotRunning;
        if (net >= run->buses.size())
            return Lookup::UnknownNetwork;
        if (node)
        {
            bool known = false;
            for (const NodeConfig &n : run->cfg.networks[net].nodes)
                known = known || n.node_id == node;
            if (!known)
                return Lookup::UnknownNode;
        }
        queue = &run->buses[net]->client();
        return Lookup::Ok;
    }

    bool job(uint64_t id, JobResult &out) override
    {
        Running *run = current();
        if (!run)
            return false;
        for (auto &bus : run->buses)
            if (bus->client().result(id, out))
                return true;
        return false;
    }

    std::string service_open(const ServiceRequest &req) override
    {
        if (g_run)
            return "plc_running";
        if (g_service)
            return "busy";
        return open_service(req);
    }

    void service_close() override
    {
        closing_ = true;
    }

    // A close asked by the last request, done after g_mutex is released.
    bool closing_ = false;

  private:
    Running *current()
    {
        return g_run ? g_run.get() : g_service.get();
    }
};

std::string answer_request(const std::string &request)
{
    PluginRequests target;
    std::string answer;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_last_request = std::chrono::steady_clock::now();
        answer         = handle_request(request, target);
    }
    if (target.closing_)
        close_service("closed by a client");
    return answer;
}

void server_tick()
{
    bool idle;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        idle = g_service && std::chrono::steady_clock::now() - g_last_request > kServiceIdle;
    }
    if (idle)
        close_service("no request for 5 minutes");
}

int do_init(void *args)
{
    if (!args)
        return -1;
    // The runtime's copy is freed after init: keep our own.
    g_args      = *static_cast<plugin_runtime_args_t *>(args);
    g_have_args = true;
    set_log_sink(runtime_sink);
    route_lely_diagnostics();
    if (!g_server)
    {
        std::unique_ptr<StatusServer> server(
            new StatusServer(kStatusSocketPath, answer_request, server_tick));
        std::string problem;
        if (server->start(problem))
            g_server = std::move(server);
        else
            log_warn("no CANopen socket (%s); networks run without it", problem.c_str());
    }
    return 0;
}

int do_start_loop()
{
    if (g_run)
        return 0;
    close_service("the PLC starts");
    if (!g_have_args || !g_args.image_lock || !g_args.image_unlock || !g_args.journal_write_bool ||
        !g_args.journal_write_byte || !g_args.journal_write_int || !g_args.journal_write_dint ||
        !g_args.journal_write_lint)
    {
        log_error("the runtime did not provide the image and journal entry points");
        return -1;
    }

    std::unique_ptr<Running> run(new Running);
    const std::string path = g_args.plugin_specific_config_file_path;
    std::vector<std::string> errors;
    if (!load_config(path, run->cfg, errors))
    {
        for (const auto &e : errors)
            log_error("%s", e.c_str());
        log_error("%s refused; the CANopen master does not run", path.c_str());
        return -1;
    }
    for (const auto &w : run->cfg.warnings)
        log_warn("%s", w.c_str());

    const size_t count = run->cfg.networks.size();
    run->layouts.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        const NetworkConfig &net = run->cfg.networks[i];
        run->layouts.push_back(master_layout(net));
        std::unique_ptr<ProcessImage> image(new ProcessImage);
        if (!image->build(net, run->layouts.back(), g_args.buffer_size, errors))
        {
            for (const auto &e : errors)
                log_error("%s", e.c_str());
            return -1;
        }
        run->images.push_back(std::move(image));
    }
    for (size_t i = 0; i < count; ++i)
    {
        const NetworkConfig &net = run->cfg.networks[i];
        std::string dcf = config_dir(path) + "/canopen_master_" + std::to_string(i) + ".dcf";
        if (!write_file(dcf, master_dcf(net, run->layouts[i])))
        {
            log_error("%s: cannot write the master configuration %s", net.label().c_str(),
                      dcf.c_str());
            return -1;
        }
        run->statuses.emplace_back(new NetworkStatus(net));
        run->buses.emplace_back(
            new Bus(net, run->layouts[i], *run->images[i], *run->statuses.back(), dcf));
    }
    prepare_wake_signal();
    for (auto &bus : run->buses)
        bus->start();
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_run = std::move(run);
    }
    g_active.store(true, std::memory_order_release);
    log_info("CANopen master running, %zu network(s)", count);
    return 0;
}

void do_stop_loop()
{
    if (!g_run)
        return;
    g_active.store(false, std::memory_order_release);
    std::unique_ptr<Running> run;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        run = std::move(g_run);
    }
    stop_run(std::move(run));
    log_info("CANopen master stopped");
}

void report(const char *where, const char *what)
{
    log_error("%s failed: %s", where, what);
}

} // namespace

CANOPEN_EXPORT int init(void *args)
{
    try
    {
        return do_init(args);
    }
    catch (const std::exception &e)
    {
        report("init", e.what());
    }
    catch (...)
    {
        report("init", "unknown error");
    }
    return -1;
}

CANOPEN_EXPORT int start_loop(void)
{
    try
    {
        return do_start_loop();
    }
    catch (const std::exception &e)
    {
        report("start", e.what());
    }
    catch (...)
    {
        report("start", "unknown error");
    }
    try
    {
        // Bus threads that started were joined as the partial setup unwound; this clears a
        // run left from before.
        do_stop_loop();
    }
    catch (...)
    {
    }
    log_error("the CANopen master does not run");
    return -1;
}

CANOPEN_EXPORT void stop_loop(void)
{
    try
    {
        do_stop_loop();
    }
    catch (const std::exception &e)
    {
        report("stop", e.what());
    }
    catch (...)
    {
        report("stop", "unknown error");
    }
}

// The scan hooks cannot throw; the handlers only keep an exception out of the runtime.
CANOPEN_EXPORT void cycle_start(void)
{
    if (!g_active.load(std::memory_order_acquire))
        return;
    try
    {
        for (auto &image : g_run->images)
            image->copy_to_plc(g_args);
    }
    catch (...)
    {
    }
}

CANOPEN_EXPORT void cycle_end(void)
{
    if (!g_active.load(std::memory_order_acquire))
        return;
    try
    {
        for (auto &image : g_run->images)
            image->copy_from_plc(g_args);
    }
    catch (...)
    {
    }
}

CANOPEN_EXPORT void cleanup(void)
{
    try
    {
        if (g_server)
            g_server->stop();
        g_server.reset();
        close_service("cleanup");
    }
    catch (...)
    {
    }
    stop_loop();
    set_log_sink(nullptr);
}
