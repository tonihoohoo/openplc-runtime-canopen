// SPDX-License-Identifier: Apache-2.0
//
// Derived from canworks ec7fd59d, plugin/src/can/log.h.
//
// log.h - logging for the CANopen plugin.
//
// All messages are prefixed with "[CANOPEN]" and routed to a sink. In the
// runtime the sink is the runtime logger from plugin_runtime_args_t; in unit
// tests it is a capture buffer. Never call these from cycle_start/cycle_end.

#ifndef CANOPEN_LOG_H
#define CANOPEN_LOG_H

#include <string>

namespace canopen_plugin
{

enum class LogLevel
{
    Debug,
    Info,
    Warn,
    Error
};

using LogSink = void (*)(LogLevel level, const char *msg);

// Installs the sink. Passing nullptr restores the default (stderr).
void set_log_sink(LogSink sink);

// Text put in front of every message logged by the calling thread
// ("can0: "), for the threads of one network; "" for none.
void set_thread_log_prefix(const std::string &prefix);
const std::string &thread_log_prefix();

// Sets the calling thread's prefix for the life of the object.
class ScopedLogPrefix
{
  public:
    explicit ScopedLogPrefix(const std::string &prefix) : old_(thread_log_prefix())
    {
        set_thread_log_prefix(prefix);
    }
    ~ScopedLogPrefix()
    {
        set_thread_log_prefix(old_);
    }

  private:
    std::string old_;
};

// Routes Lely's diagnostic messages (diag()/diag_at()) to the sink.
void route_lely_diagnostics();

void log_debug(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void log_info(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void log_warn(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void log_error(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

} // namespace canopen_plugin

#endif // CANOPEN_LOG_H
