// SPDX-License-Identifier: Apache-2.0
//
// canopen_plugin.cpp - the native plugin's entry points (plugin_driver.h).
//
//   init:        copies the runtime arguments and routes the log; nothing else, since it runs at
//                power-on even when the plugin is disabled
//   start_loop:  loads conf/canopen.json, builds each network's master DCF and process image,
//                and starts one bus thread per network
//   cycle_start: writes the newest inputs to %I through the journal
//   cycle_end:   reads %Q under the image lock and hands it to the bus threads
//   stop_loop:   sends each network's NMT command for a PLC stop and joins the bus threads
// No exception leaves an entry point: the runtime calls them as C functions.

#include <atomic>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "bus.h"
#include "config.h"
#include "log.h"
#include "master_dcf.h"
#include "plugin_types.h"
#include "process_image.h"

using namespace canopen_plugin;

#define CANOPEN_EXPORT extern "C" __attribute__((visibility("default")))

namespace
{

struct Running
{
    Config cfg;
    std::vector<MasterLayout> layouts;
    std::vector<std::unique_ptr<ProcessImage>> images;
    std::vector<std::unique_ptr<Bus>> buses;
};

plugin_runtime_args_t g_args;
bool g_have_args = false;
std::unique_ptr<Running> g_run;
std::atomic<bool> g_active{false};

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

int do_init(void *args)
{
    if (!args)
        return -1;
    // The runtime's copy is freed after init: keep our own.
    g_args      = *static_cast<plugin_runtime_args_t *>(args);
    g_have_args = true;
    set_log_sink(runtime_sink);
    route_lely_diagnostics();
    return 0;
}

int do_start_loop()
{
    if (g_run)
        return 0;
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
        run->buses.emplace_back(new Bus(net, run->layouts[i], *run->images[i], dcf));
    }
    prepare_wake_signal();
    for (auto &bus : run->buses)
        bus->start();
    g_run = std::move(run);
    g_active.store(true, std::memory_order_release);
    log_info("CANopen master running, %zu network(s)", count);
    return 0;
}

void do_stop_loop()
{
    if (!g_run)
        return;
    g_active.store(false, std::memory_order_release);
    for (auto &bus : g_run->buses)
        bus->stop();
    g_run.reset();
    release_wake_signal();
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
    stop_loop();
    set_log_sink(nullptr);
}
