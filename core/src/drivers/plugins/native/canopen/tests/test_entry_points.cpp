// SPDX-License-Identifier: Apache-2.0
//
// The plugin's C entry points with an allocation failure injected at each point of start_loop: no
// exception leaves them, a failed start leaves no network running and a later start works. Also:
// service mode is refused while a start is under way (needs the plugin's socket; skipped without).

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <new>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "check.h"
#include "plugin_types.h"
#include "status_server.h"

extern "C" int init(void *args);
extern "C" int start_loop(void);
extern "C" void stop_loop(void);
extern "C" void cycle_start(void);
extern "C" void cycle_end(void);

namespace
{

// Allocations left in this thread before one fails; -1: never.
thread_local long t_fail_in = -1;

void *allocate(std::size_t n)
{
    if (t_fail_in > 0 && --t_fail_in == 0)
    {
        t_fail_in = -1;
        throw std::bad_alloc();
    }
    if (void *p = std::malloc(n ? n : 1))
        return p;
    throw std::bad_alloc();
}

} // namespace

void *operator new(std::size_t n)
{
    return allocate(n);
}
void *operator new[](std::size_t n)
{
    return allocate(n);
}
void operator delete(void *p) noexcept
{
    std::free(p);
}
void operator delete[](void *p) noexcept
{
    std::free(p);
}
void operator delete(void *p, std::size_t) noexcept
{
    std::free(p);
}
void operator delete[](void *p, std::size_t) noexcept
{
    std::free(p);
}

namespace
{

// A missing interface: the bus threads start and wait for it.
const char *kConfig = R"({
  "version": 3,
  "networks": [
    {"name": "a", "interface": "cotest-none0", "bitrate": 0, "heartbeat_producer_ms": 100,
     "sync": {"mode": "plc_cycle"},
     "nodes": [{"node_id": 2, "name": "io2", "state_location": "%IB0",
       "rpdos": [{"number": 1, "cob_id": 514, "entries": [
         {"index": 25088, "subindex": 1, "type": "UNSIGNED8", "location": "%QB0"}]}]}]},
    {"name": "b", "interface": "cotest-none1", "bitrate": 0,
     "nodes": [{"node_id": 3, "name": "io3", "state_location": "%IB1"}]}
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

int g_not_running = 0;

void log_quiet(const char *, ...) {}

void log_error(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (std::strstr(buf, "the CANopen master does not run"))
        ++g_not_running;
}

// One request over the plugin's socket, without the newline; "" when there is no socket.
std::string ask(const std::string &request)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return "";
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", canopen_plugin::kStatusSocketPath);
    std::string line = request + "\n";
    if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
        send(fd, line.data(), line.size(), 0) != static_cast<ssize_t>(line.size()))
    {
        close(fd);
        return "";
    }
    std::string answer;
    char buf[1024];
    while (answer.find('\n') == std::string::npos)
    {
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0)
            break;
        answer.append(buf, static_cast<size_t>(n));
    }
    close(fd);
    return answer;
}

const char *kServiceOpen = R"({"op":"service_open","interface":"cotest-none2","bitrate":0,)"
                           R"("master_node_id":1,"sdo_timeout_ms":500,"nodes":[2]})";

} // namespace

int main()
{
    char dir[] = "/tmp/canopen-entry-XXXXXX";
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
    rt.log_error          = log_error;
    std::snprintf(rt.plugin_specific_config_file_path, sizeof(rt.plugin_specific_config_file_path),
                  "%s", conf.c_str());
    CHECK(init(&rt) == 0);

    // Fail the 1st, 2nd, ... allocation of start_loop until it allocates no more.
    long failed = 0;
    for (long n = 1; n < 100000; ++n)
    {
        t_fail_in = n;
        int rc    = start_loop();
        bool hit  = t_fail_in == -1;
        t_fail_in = -1;
        if (!hit)
        {
            CHECK(rc == 0);
            break;
        }
        // Nothing runs after a failed start: the scan hooks do nothing.
        cycle_start();
        cycle_end();
        if (rc == 0)
            stop_loop(); // the failure was handled inside, as a refused configuration
        else
            ++failed;
    }
    std::printf("start_loop failed cleanly at %ld allocation points\n", failed);
    CHECK(failed > 10);
    CHECK(g_not_running > 0);

    cycle_start();
    cycle_end();
    stop_loop();

    // A start that waits on its configuration (a FIFO) keeps service mode closed until it ends.
    if (ask(R"({"op":"status"})").empty())
        std::printf("no plugin socket: the service-mode check during a start is skipped\n");
    else
    {
        const std::string fifo = std::string(dir) + "/slow.json";
        CHECK(mkfifo(fifo.c_str(), 0600) == 0);
        std::snprintf(rt.plugin_specific_config_file_path,
                      sizeof(rt.plugin_specific_config_file_path), "%s", fifo.c_str());
        CHECK(init(&rt) == 0);
        int rc = 0;
        std::thread starting([&rc] { rc = start_loop(); });
        usleep(200 * 1000);
        CHECK(ask(kServiceOpen).find("plc_running") != std::string::npos);
        {
            std::ofstream out(fifo);
            out << "{}";
        }
        starting.join();
        CHECK(rc == -1);
        const std::string after = ask(kServiceOpen);
        CHECK(!after.empty() && after.find("plc_running") == std::string::npos);
        ask(R"({"op":"service_close"})");
        std::remove(fifo.c_str());
    }
    std::remove(conf.c_str());
    for (int i = 0; i < 2; ++i)
        std::remove((std::string(dir) + "/canopen_master_" + std::to_string(i) + ".dcf").c_str());
    rmdir(dir);
    return TEST_RESULT();
}
