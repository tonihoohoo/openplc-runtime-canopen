// SPDX-License-Identifier: Apache-2.0

#include "status_server.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "cJSON.h"
#include "log.h"

namespace canopen_plugin
{

constexpr int StatusServer::kClientTimeoutMs;
constexpr size_t StatusServer::kMaxRequest;

StatusServer::StatusServer(std::string path, Handler handler)
    : path_(std::move(path)), handler_(std::move(handler))
{
}

StatusServer::~StatusServer()
{
    stop();
}

bool StatusServer::start(std::string &problem)
{
    if (thread_.joinable())
        return true;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (path_.size() >= sizeof(addr.sun_path))
    {
        problem = "the socket path " + path_ + " is too long";
        return false;
    }
    std::memcpy(addr.sun_path, path_.c_str(), path_.size() + 1);
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0)
    {
        problem = std::string("cannot create a socket: ") + std::strerror(errno);
        return false;
    }
    // A socket left behind by a previous run.
    unlink(path_.c_str());
    // chmod after bind, as umask() is process-wide.
    if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
        chmod(path_.c_str(), 0600) != 0 || listen(fd, 4) != 0)
    {
        problem = "cannot listen on " + path_ + ": " + std::strerror(errno);
        close(fd);
        unlink(path_.c_str());
        return false;
    }
    if (pipe2(stop_pipe_, O_CLOEXEC) != 0)
    {
        problem = std::string("cannot create a pipe: ") + std::strerror(errno);
        close(fd);
        unlink(path_.c_str());
        return false;
    }
    listen_fd_ = fd;
    thread_    = std::thread([this] { thread_main(); });
    return true;
}

void StatusServer::stop()
{
    if (thread_.joinable())
    {
        char c = 0;
        ssize_t n;
        do
            n = write(stop_pipe_[1], &c, 1);
        while (n < 0 && errno == EINTR);
        thread_.join();
    }
    for (int &fd : stop_pipe_)
    {
        if (fd >= 0)
            close(fd);
        fd = -1;
    }
    if (listen_fd_ >= 0)
    {
        close(listen_fd_);
        listen_fd_ = -1;
        unlink(path_.c_str());
    }
}

void StatusServer::thread_main()
{
    pthread_setname_np(pthread_self(), "canopen_status");
    // Normal scheduling, whatever the starting thread had: the status never competes with the bus.
    sched_param sp{};
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
    for (;;)
    {
        pollfd fds[2] = {{stop_pipe_[0], POLLIN, 0}, {listen_fd_, POLLIN, 0}};
        int rc        = poll(fds, 2, -1);
        if (rc < 0)
        {
            if (errno == EINTR)
                continue;
            log_error("status socket: poll failed: %s", std::strerror(errno));
            return;
        }
        if (fds[0].revents)
            return;
        if (!(fds[1].revents & POLLIN))
            continue;
        int client = accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC);
        if (client < 0)
            continue;
        serve(client);
        close(client);
    }
}

void StatusServer::serve(int fd)
{
    std::string buf;
    for (;;)
    {
        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos)
        {
            std::string answer = handler_(buf.substr(0, nl)) + "\n";
            buf.erase(0, nl + 1);
            size_t off = 0;
            while (off < answer.size())
            {
                ssize_t n = send(fd, answer.data() + off, answer.size() - off, MSG_NOSIGNAL);
                if (n < 0 && errno == EINTR)
                    continue;
                if (n <= 0)
                    return;
                off += static_cast<size_t>(n);
            }
        }
        if (buf.size() > kMaxRequest)
            return;
        pollfd fds[2] = {{fd, POLLIN, 0}, {stop_pipe_[0], POLLIN, 0}};
        int rc        = poll(fds, 2, kClientTimeoutMs);
        if (rc < 0 && errno == EINTR)
            continue;
        if (rc <= 0 || fds[1].revents)
            return;
        char chunk[512];
        ssize_t n = recv(fd, chunk, sizeof(chunk), 0);
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return;
        buf.append(chunk, static_cast<size_t>(n));
    }
}

std::string handle_status_request(const std::string &request,
                                  const std::function<std::string()> &status)
{
    cJSON *req      = cJSON_Parse(request.c_str());
    const cJSON *op = req ? cJSON_GetObjectItemCaseSensitive(req, "op") : nullptr;
    std::string answer;
    if (!req || !cJSON_IsObject(req))
        answer = R"({"error":"the request is not a JSON object"})";
    else if (!cJSON_IsString(op))
        answer = R"({"error":"the request has no op"})";
    else if (std::strcmp(op->valuestring, "status") == 0)
        answer = status();
    else
        answer = R"({"error":"unknown op"})";
    cJSON_Delete(req);
    return answer;
}

} // namespace canopen_plugin
