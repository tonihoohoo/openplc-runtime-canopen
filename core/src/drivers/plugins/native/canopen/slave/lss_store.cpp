// SPDX-License-Identifier: Apache-2.0

#include "lss_store.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>

#include <fcntl.h>
#include <sys/random.h>
#include <sys/stat.h>
#include <unistd.h>

#include "cJSON.h"

namespace canopen_plugin
{

namespace
{

bool in_container()
{
    if (access("/.dockerenv", F_OK) == 0)
        return true;
    const char *c = std::getenv("container");
    if ((c && *c) || std::getenv("DOCKER_CONTAINER"))
        return true;
    std::ifstream cg("/proc/1/cgroup");
    std::stringstream text;
    text << cg.rdbuf();
    const std::string s = text.str();
    for (const char *mark : {"docker", "kubepods", "/lxc/", "containerd"})
        if (s.find(mark) != std::string::npos)
            return true;
    return false;
}

bool make_dirs(const std::string &path, std::string &problem)
{
    std::string part;
    std::stringstream in(path);
    std::string item;
    if (!path.empty() && path[0] == '/')
        part = "/";
    while (std::getline(in, item, '/'))
    {
        if (item.empty())
            continue;
        part += item + "/";
        if (mkdir(part.c_str(), 0755) != 0 && errno != EEXIST)
        {
            problem = "cannot create " + part + ": " + std::strerror(errno);
            return false;
        }
    }
    return true;
}

bool get_u32(const cJSON *root, const char *key, uint32_t lo, uint32_t hi, uint32_t &out)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!v)
        return true;
    if (!cJSON_IsNumber(v) || std::floor(v->valuedouble) != v->valuedouble || v->valuedouble < lo ||
        v->valuedouble > hi)
        return false;
    out = static_cast<uint32_t>(v->valuedouble);
    return true;
}

} // namespace

std::string lss_data_dir()
{
    const char *dir = std::getenv("OPENPLC_PERSISTENT_DATA_DIR");
    if (dir && *dir)
        return dir;
    return in_container() ? "/var/run/runtime" : "/var/lib/openplc-runtime";
}

std::string lss_store_path(const std::string &interface)
{
    return lss_data_dir() + "/canopen/lss-" + interface + ".json";
}

bool lss_load(const std::string &path, LssStored &out, std::string &problem)
{
    out = LssStored();
    problem.clear();
    std::ifstream f(path);
    if (!f)
    {
        if (errno != ENOENT)
            problem = "cannot read " + path + ": " + std::strerror(errno);
        return false;
    }
    std::stringstream text;
    text << f.rdbuf();
    cJSON *root     = cJSON_Parse(text.str().c_str());
    uint32_t serial = 0, node = 0, bitrate = 0;
    bool ok = cJSON_IsObject(root) && get_u32(root, "serial", 1, 0xFFFFFFFF, serial) &&
              get_u32(root, "node_id", 1, 0xFF, node) &&
              get_u32(root, "bitrate", 10000, 1000000, bitrate) &&
              (node <= 127 || node == 0xFF || node == 0);
    cJSON_Delete(root);
    if (!ok)
    {
        problem = path + " is not a valid LSS store";
        return false;
    }
    out.serial  = serial;
    out.node_id = static_cast<uint8_t>(node);
    out.bitrate = bitrate;
    return true;
}

bool lss_save(const std::string &path, const LssStored &value, std::string &problem)
{
    const size_t slash = path.rfind('/');
    if (slash != std::string::npos && !make_dirs(path.substr(0, slash), problem))
        return false;
    std::string text;
    char buf[64];
    if (value.serial)
    {
        std::snprintf(buf, sizeof(buf), ",\"serial\":%u", static_cast<unsigned>(value.serial));
        text += buf;
    }
    if (value.node_id)
    {
        std::snprintf(buf, sizeof(buf), ",\"node_id\":%u", static_cast<unsigned>(value.node_id));
        text += buf;
    }
    if (value.bitrate)
    {
        std::snprintf(buf, sizeof(buf), ",\"bitrate\":%u", value.bitrate);
        text += buf;
    }
    text                  = "{" + (text.empty() ? text : text.substr(1)) + "}\n";
    const std::string tmp = path + ".tmp";
    int fd                = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0)
    {
        problem = "cannot write " + path + ": " + std::strerror(errno);
        return false;
    }
    bool ok =
        write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()) && fsync(fd) == 0;
    const int err = errno;
    ok            = close(fd) == 0 && ok;
    if (!ok || rename(tmp.c_str(), path.c_str()) != 0)
    {
        problem = "cannot write " + path + ": " + std::strerror(ok ? errno : err);
        unlink(tmp.c_str());
        return false;
    }
    return true;
}

uint32_t lss_random_serial()
{
    uint32_t v = 0;
    while (v == 0)
    {
        if (getrandom(&v, sizeof(v), 0) != static_cast<ssize_t>(sizeof(v)))
        {
            std::random_device rd;
            v = rd();
        }
    }
    return v;
}

} // namespace canopen_plugin
