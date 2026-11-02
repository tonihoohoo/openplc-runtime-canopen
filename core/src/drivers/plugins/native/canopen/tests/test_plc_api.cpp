// SPDX-License-Identifier: Apache-2.0
//
// The block interface of the built plugin, loaded as the runtime loads it (RTLD_LOCAL from its
// path): the blocks' lookup by SONAME without loading it again, its cost per call, the answers
// with no network running and with a network whose interface is missing, and a reload of the
// plugin while the program (this test) keeps an old handle.

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#include <dlfcn.h>
#include <unistd.h>

#include "check.h"
#include "plc_api.h"
#include "plugin_types.h"

namespace
{

const char *kConfig = R"({
  "version": 4,
  "networks": [
    {"name": "a", "interface": "cotest-none0", "bitrate": 0, "master_node_id": 100,
     "nodes": [{"node_id": 2, "name": "io2", "state_location": "%IB0"}]}
  ]
})";

constexpr int kSize = 8;
IEC_BYTE g_byte_out[kSize];
IEC_BYTE *g_byte_out_ptr[kSize];
IEC_BOOL *g_bool_out_ptr[kSize][8];
IEC_UINT *g_int_out_ptr[kSize];
IEC_UDINT *g_dint_out_ptr[kSize];
IEC_ULINT *g_lint_out_ptr[kSize];

int write_bool(int, int, int, int)
{
    return 0;
}
int write_byte(int, int, int)
{
    return 0;
}
int write_int(int, int, int)
{
    return 0;
}
int write_dint(int, int, unsigned)
{
    return 0;
}
int write_lint(int, int, unsigned long long)
{
    return 0;
}
void lock() {}
void unlock() {}
void log_quiet(const char *, ...) {}

typedef const void *(*entry_t)(uint32_t);
typedef int (*init_t)(void *);
typedef int (*start_t)(void);
typedef void (*void_t)(void);

// What a block does on each call: find the loaded plugin, take its table, drop the reference.
const canopen_plc_api_v1 *lookup()
{
    void *lib = dlopen("libcanopen_plugin.so", RTLD_NOW | RTLD_NOLOAD);
    if (!lib)
        return nullptr;
    entry_t entry = reinterpret_cast<entry_t>(dlsym(lib, "canopen_plc_api"));
    const canopen_plc_api_v1 *t =
        entry ? static_cast<const canopen_plc_api_v1 *>(entry(CANOPEN_PLC_API_VERSION)) : nullptr;
    dlclose(lib);
    return t && t->size >= sizeof(canopen_plc_api_v1) ? t : nullptr;
}

} // namespace

int main()
{
    // Not loaded yet: the blocks find nothing.
    CHECK(lookup() == nullptr);

    void *plugin = dlopen(CANOPEN_PLUGIN_PATH, RTLD_LOCAL | RTLD_NOW);
    CHECK(plugin != nullptr);
    if (!plugin)
    {
        std::fprintf(stderr, "%s\n", dlerror());
        return TEST_RESULT();
    }
    const canopen_plc_api_v1 *api = lookup();
    CHECK(api != nullptr);
    if (!api)
        return TEST_RESULT();
    entry_t entry = reinterpret_cast<entry_t>(dlsym(plugin, "canopen_plc_api"));
    CHECK(entry && entry(2) == nullptr);

    // The lookup's cost, as each block call pays it.
    constexpr int kCalls = 20000;
    auto t0              = std::chrono::steady_clock::now();
    for (int i = 0; i < kCalls; ++i)
        CHECK(lookup() == api);
    double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() /
        kCalls;
    std::printf("lookup: %.2f us per call\n", us);
    CHECK(us < 50.0);

    // No network runs.
    canopen_plc_request req{};
    req.network = 1;
    req.node    = 2;
    req.kind    = CANOPEN_PLC_READ;
    req.index   = 0x1000;
    canopen_plc_result res{};
    CHECK(api->start(&req, &res) == 0 && res.error == CANOPEN_PLC_NO_DRIVER &&
          res.reason == CANOPEN_PLC_REASON_NOT_RUN);
    uint8_t state = 0;
    CHECK(api->node_state(1, 2, &state) == CANOPEN_PLC_NO_DRIVER);
    CHECK(api->poll(0x1234567800010000ull, &res, nullptr, 0) == 2 &&
          res.error == CANOPEN_PLC_NO_DRIVER);

    char dir[] = "/tmp/canopen-plcapi-XXXXXX";
    CHECK(mkdtemp(dir) != nullptr);
    const std::string conf = std::string(dir) + "/canopen.json";
    {
        std::ofstream out(conf);
        out << kConfig;
    }
    plugin_runtime_args_t rt;
    std::memset(&rt, 0, sizeof(rt));
    for (int i = 0; i < kSize; ++i)
        g_byte_out_ptr[i] = &g_byte_out[i];
    rt.bool_output        = g_bool_out_ptr;
    rt.byte_output        = g_byte_out_ptr;
    rt.int_output         = g_int_out_ptr;
    rt.dint_output        = g_dint_out_ptr;
    rt.lint_output        = g_lint_out_ptr;
    rt.image_lock         = lock;
    rt.image_unlock       = unlock;
    rt.journal_write_bool = write_bool;
    rt.journal_write_byte = write_byte;
    rt.journal_write_int  = write_int;
    rt.journal_write_dint = write_dint;
    rt.journal_write_lint = write_lint;
    rt.buffer_size        = kSize;
    rt.log_info           = log_quiet;
    rt.log_debug          = log_quiet;
    rt.log_warn           = log_quiet;
    rt.log_error          = log_quiet;
    std::snprintf(rt.plugin_specific_config_file_path, sizeof(rt.plugin_specific_config_file_path),
                  "%s", conf.c_str());

    // Two PLC runs, the plugin loaded again in between as the runtime does.
    for (int run = 0; run < 2; ++run)
    {
        init_t init  = reinterpret_cast<init_t>(dlsym(plugin, "init"));
        start_t sl   = reinterpret_cast<start_t>(dlsym(plugin, "start_loop"));
        void_t stop  = reinterpret_cast<void_t>(dlsym(plugin, "stop_loop"));
        void_t clean = reinterpret_cast<void_t>(dlsym(plugin, "cleanup"));
        CHECK(init && sl && stop && clean);
        if (!init || !sl || !stop || !clean)
            break;
        CHECK(init(&rt) == 0);
        CHECK(sl() == 0);
        api = lookup();
        CHECK(api != nullptr);
        if (!api)
            break;

        // The interface is missing: requests are refused at once, the states say why.
        CHECK(api->start(&req, &res) == 0);
        CHECK(res.error == CANOPEN_PLC_NO_DRIVER || res.error == CANOPEN_PLC_BUS_OFF);
        CHECK(api->node_state(1, 2, &state) == CANOPEN_PLC_NO_ERROR &&
              state == CANOPEN_PLC_NOT_AVAIL);
        CHECK(api->node_state(1, 3, &state) == CANOPEN_PLC_NODE_ID_UNKNOWN);
        CHECK(api->node_state(2, 2, &state) == CANOPEN_PLC_NET_ID_UNKNOWN);
        uint8_t id = 0;
        CHECK(api->local_node_id(1, &id) == CANOPEN_PLC_NO_ERROR && id == 100);
        uint64_t cursor = 0;
        canopen_plc_emcy e{};
        CHECK(api->emcy_cursor(1, &cursor) == CANOPEN_PLC_NO_ERROR);
        CHECK(api->emcy_next(1, 0, &cursor, &e) == 0);
        CHECK(api->emcy_next(1, 3, &cursor, &e) == -CANOPEN_PLC_NODE_ID_UNKNOWN);
        // A handle of an earlier load is never another request's.
        CHECK(api->poll(0x1234567800010000ull, &res, nullptr, 0) == 2 &&
              res.error == CANOPEN_PLC_NO_DRIVER && res.reason == CANOPEN_PLC_REASON_RESTART);
        api->cancel(0x1234567800010000ull);

        stop();
        CHECK(api->start(&req, &res) == 0 && res.error == CANOPEN_PLC_NO_DRIVER);
        clean();
        // The plugin may stay mapped after dlclose; the blocks do not rely on either.
        dlclose(plugin);
        plugin = dlopen(CANOPEN_PLUGIN_PATH, RTLD_LOCAL | RTLD_NOW);
        CHECK(plugin != nullptr);
        if (!plugin)
            break;
    }
    if (plugin)
        dlclose(plugin);
    std::remove(conf.c_str());
    std::remove((std::string(dir) + "/canopen_master_0.dcf").c_str());
    rmdir(dir);
    return TEST_RESULT();
}
