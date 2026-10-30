// SPDX-License-Identifier: Apache-2.0
//
// Derived from canworks ec7fd59d, plugin/src/can/log.cpp.
//

#include "log.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#include <lely/util/diag.h>
#include <lely/util/errnum.h>

namespace canopen_plugin
{

namespace
{

void stderr_sink(LogLevel level, const char *msg)
{
    static const char *names[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    std::fprintf(stderr, "%s %s\n", names[static_cast<int>(level)], msg);
}

LogSink g_sink = stderr_sink;
thread_local std::string t_prefix;

void vlog(LogLevel level, const char *fmt, va_list ap)
{
    char buf[1024];
    int n = std::snprintf(buf, sizeof(buf), "[CANOPEN] %s", t_prefix.c_str());
    if (n < 0 || n >= (int)sizeof(buf))
        n = 0;
    std::vsnprintf(buf + n, sizeof(buf) - n, fmt, ap);
    g_sink(level, buf);
}

} // namespace

void set_log_sink(LogSink sink)
{
    g_sink = sink ? sink : stderr_sink;
}

void set_thread_log_prefix(const std::string &prefix)
{
    t_prefix = prefix;
}
const std::string &thread_log_prefix()
{
    return t_prefix;
}

namespace
{

LogLevel from_severity(diag_severity s)
{
    switch (s)
    {
    case DIAG_DEBUG:
        return LogLevel::Debug;
    case DIAG_INFO:
        return LogLevel::Info;
    case DIAG_WARNING:
        return LogLevel::Warn;
    default:
        return LogLevel::Error;
    }
}

// Lely logs each failed boot or configuration SDO, every attempt; the plugin reports these
// failures itself, once, with the node's name.
bool boot_sdo_abort(const char *msg)
{
    return std::strncmp(msg, "SDO abort code ", 15) == 0;
}

void lely_diag(void *, diag_severity severity, int errc, const char *format, va_list ap)
{
    char msg[768];
    std::vsnprintf(msg, sizeof(msg), format, ap);
    char buf[1024];
    if (errc)
        std::snprintf(buf, sizeof(buf), "[CANOPEN] %slely: %s: %s", t_prefix.c_str(), msg,
                      errc2str(errc));
    else
        std::snprintf(buf, sizeof(buf), "[CANOPEN] %slely: %s", t_prefix.c_str(), msg);
    g_sink(boot_sdo_abort(msg) ? LogLevel::Debug : from_severity(severity), buf);
}

void lely_diag_at(void *, diag_severity severity, int errc, const floc *at, const char *format,
                  va_list ap)
{
    char msg[768];
    std::vsnprintf(msg, sizeof(msg), format, ap);
    char buf[1024];
    if (at && at->filename)
        std::snprintf(buf, sizeof(buf), "[CANOPEN] %slely: %s:%d:%d: %s", t_prefix.c_str(),
                      at->filename, at->line, at->column, msg);
    else
        std::snprintf(buf, sizeof(buf), "[CANOPEN] %slely: %s", t_prefix.c_str(), msg);
    (void)errc;
    g_sink(from_severity(severity), buf);
}

} // namespace

void route_lely_diagnostics()
{
    diag_set_handler(&lely_diag, nullptr);
    diag_at_set_handler(&lely_diag_at, nullptr);
}

#define CANOPEN_DEFINE_LOG(name, level)                                                            \
    void name(const char *fmt, ...)                                                                \
    {                                                                                              \
        va_list ap;                                                                                \
        va_start(ap, fmt);                                                                         \
        vlog(level, fmt, ap);                                                                      \
        va_end(ap);                                                                                \
    }

CANOPEN_DEFINE_LOG(log_debug, LogLevel::Debug)
CANOPEN_DEFINE_LOG(log_info, LogLevel::Info)
CANOPEN_DEFINE_LOG(log_warn, LogLevel::Warn)
CANOPEN_DEFINE_LOG(log_error, LogLevel::Error)

} // namespace canopen_plugin
