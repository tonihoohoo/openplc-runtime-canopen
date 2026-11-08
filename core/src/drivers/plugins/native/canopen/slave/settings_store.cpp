// SPDX-License-Identifier: Apache-2.0

#include "settings_store.h"
#include "lss_store.h"
#include "slave_dcf.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#include <unistd.h>

#include "cJSON.h"

namespace canopen_plugin
{

namespace
{

void mix(uint64_t &h, const std::string &text)
{
    for (unsigned char c : text)
    {
        h ^= c;
        h *= 1099511628211ull;
    }
    h ^= 0xFF; // a separator, so "ab","c" differs from "a","bc"
    h *= 1099511628211ull;
}

bool parse_u64(const cJSON *v, uint64_t &out)
{
    if (!cJSON_IsString(v) || !v->valuestring[0])
        return false;
    char *end = nullptr;
    errno     = 0;
    out       = std::strtoull(v->valuestring, &end, 10);
    return errno == 0 && end && !*end;
}

} // namespace

unsigned settings_area(uint16_t index)
{
    if (index == 0x1014 || index == 0x1016 || index == 0x1017 || index == 0x1029 ||
        (index >= 0x1400 && index <= 0x140F) || (index >= 0x1600 && index <= 0x160F) ||
        (index >= 0x1800 && index <= 0x180F) || (index >= 0x1A00 && index <= 0x1A0F))
        return 2;
    if (index >= 0x2000 && index <= 0x5FFF)
        return 3;
    return 0;
}

std::string settings_store_path(const std::string &interface)
{
    if (!valid_interface_name(interface))
        return std::string();
    return lss_data_dir() + "/canopen/slave-" + interface + ".json";
}

std::string settings_config_hash(const DeviceConfig &dev)
{
    uint64_t h = 1469598103934665603ull;
    // The dictionary as built for node 0 and serial 0 covers the identity, heartbeat, PDOs,
    // object types, access and the switches; the strings and application defaults are set after
    // loading, so they are mixed in here.
    mix(h, slave_dcf(dev, 0, 0));
    mix(h, dev.device_name);
    mix(h, dev.hardware_version);
    mix(h, dev.software_version);
    for (const SlaveObject &o : dev.objects)
    {
        mix(h, std::to_string(o.index) + ":" + std::to_string(o.subindex));
        mix(h, std::to_string(o.value));
        mix(h, o.text);
        mix(h, std::to_string(o.min) + "," + std::to_string(o.max) + "," +
                   std::to_string(o.has_min) + std::to_string(o.has_max));
    }
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

bool settings_load(const std::string &path, SettingsStored &out, std::string &problem)
{
    out = SettingsStored();
    problem.clear();
    if (path.empty())
    {
        problem = "no valid settings file for this interface";
        return false;
    }
    errno = 0;
    std::ifstream f(path);
    if (!f)
    {
        if (errno != ENOENT)
            problem = "cannot read " + path + ": " + std::strerror(errno);
        return false;
    }
    std::stringstream text;
    text << f.rdbuf();
    cJSON *root = cJSON_Parse(text.str().c_str());
    SettingsStored value;
    bool ok = cJSON_IsObject(root);
    if (ok)
    {
        const cJSON *hash    = cJSON_GetObjectItemCaseSensitive(root, "config_hash");
        const cJSON *time    = cJSON_GetObjectItemCaseSensitive(root, "time_ms");
        const cJSON *entries = cJSON_GetObjectItemCaseSensitive(root, "entries");
        ok = cJSON_IsString(hash) && cJSON_IsNumber(time) && cJSON_IsArray(entries);
        if (ok)
        {
            value.config_hash = hash->valuestring;
            value.time_ms     = static_cast<int64_t>(time->valuedouble);
            const cJSON *e    = nullptr;
            cJSON_ArrayForEach(e, entries)
            {
                const cJSON *i = cJSON_GetObjectItemCaseSensitive(e, "index");
                const cJSON *s = cJSON_GetObjectItemCaseSensitive(e, "sub");
                SettingsEntry entry;
                uint64_t v = 0;
                if (!cJSON_IsNumber(i) || !cJSON_IsNumber(s) || i->valuedouble < 0 ||
                    i->valuedouble > 0xFFFF || s->valuedouble < 0 || s->valuedouble > 0xFF ||
                    !parse_u64(cJSON_GetObjectItemCaseSensitive(e, "value"), v))
                {
                    ok = false;
                    break;
                }
                entry.index    = static_cast<uint16_t>(i->valuedouble);
                entry.subindex = static_cast<uint8_t>(s->valuedouble);
                entry.value    = v;
                value.entries.push_back(entry);
            }
        }
    }
    cJSON_Delete(root);
    if (!ok)
    {
        problem = path + " is not a valid settings file";
        return false;
    }
    out = value;
    return true;
}

bool settings_save(const std::string &path, const SettingsStored &value, std::string &problem)
{
    problem.clear();
    if (value.entries.empty())
        return settings_remove(path, problem);
    std::string text = "{\"config_hash\":\"" + value.config_hash +
                       "\",\"time_ms\":" + std::to_string(value.time_ms) + ",\"entries\":[";
    bool first = true;
    for (const SettingsEntry &e : value.entries)
    {
        text += first ? "" : ",";
        first = false;
        text += "{\"index\":" + std::to_string(e.index) + ",\"sub\":" +
                std::to_string(e.subindex) + ",\"value\":\"" + std::to_string(e.value) + "\"}";
    }
    text += "]}\n";
    return store_write_atomic(path, text, problem);
}

bool settings_remove(const std::string &path, std::string &problem)
{
    problem.clear();
    if (path.empty())
        return true;
    if (unlink(path.c_str()) != 0 && errno != ENOENT)
    {
        problem = "cannot remove " + path + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

} // namespace canopen_plugin
