// SPDX-License-Identifier: Apache-2.0
//
// status_server.h - the plugin's local socket: a Unix stream socket that answers one JSON request
// per line with one JSON answer per line, on its own thread at normal scheduling. The handler
// builds each answer; this class is only the transport. Clients are served one at a time.

#ifndef CANOPEN_STATUS_SERVER_H
#define CANOPEN_STATUS_SERVER_H

#include <functional>
#include <string>
#include <thread>

namespace canopen_plugin
{

// Next to the runtime's own command socket.
constexpr const char *kStatusSocketPath = "/run/runtime/canopen.socket";

class StatusServer
{
  public:
    using Handler = std::function<std::string(const std::string &request)>;

    StatusServer(std::string path, Handler handler);
    ~StatusServer();

    // Binds the socket (mode 0600) and starts the thread. False, with `problem` set, when the
    // socket cannot be created; the plugin then runs without it.
    bool start(std::string &problem);
    // Stops the thread and removes the socket. Idempotent.
    void stop();

  private:
    static constexpr int kClientTimeoutMs = 1000;
    static constexpr size_t kMaxRequest   = 4096;

    void thread_main();
    void serve(int fd);

    std::string path_;
    Handler handler_;
    int listen_fd_    = -1;
    int stop_pipe_[2] = {-1, -1};
    std::thread thread_;
};

// Answers a request of the status socket: {"op": "status"} with the status, anything else with an
// error. `status` builds the status answer.
std::string handle_status_request(const std::string &request,
                                  const std::function<std::string()> &status);

} // namespace canopen_plugin

#endif // CANOPEN_STATUS_SERVER_H
