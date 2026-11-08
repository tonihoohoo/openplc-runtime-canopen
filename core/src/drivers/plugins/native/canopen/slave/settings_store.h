// SPDX-License-Identifier: Apache-2.0
//
// settings_store.h - the settings a master saved into a slave with 0x1010, kept per CAN interface
// in a small JSON file in the runtime's persistent data folder, so they survive a restart, a reboot
// and a runtime update. Only the slave thread and Slave::start() call these; never the PLC scan.

#ifndef CANOPEN_SETTINGS_STORE_H
#define CANOPEN_SETTINGS_STORE_H

#include <cstdint>
#include <string>
#include <vector>

#include "config.h"

namespace canopen_plugin
{

// One saved value: the object, and its raw little-endian bits.
struct SettingsEntry
{
    uint16_t index   = 0;
    uint8_t subindex = 0;
    uint64_t value   = 0;
};

struct SettingsStored
{
    std::string config_hash; // of the slave configuration the values were saved for
    int64_t time_ms = 0;     // wall clock of the last save
    std::vector<SettingsEntry> entries;
};

// The area of an object, 0x1010/0x1011 sub-index: 2 communication, 3 application, 0 neither.
unsigned settings_area(uint16_t index);

// <data dir>/canopen/slave-<interface>.json, empty for an unusable interface name.
std::string settings_store_path(const std::string &interface);
// A hash of everything in the slave's configuration that shapes its dictionary and defaults.
std::string settings_config_hash(const DeviceConfig &dev);

// False with a reason when the file is unreadable or invalid; `out` is then empty. A missing file
// gives an empty reason.
bool settings_load(const std::string &path, SettingsStored &out, std::string &problem);
// Atomic write; an empty entry list removes the file instead.
bool settings_save(const std::string &path, const SettingsStored &value, std::string &problem);
bool settings_remove(const std::string &path, std::string &problem);

} // namespace canopen_plugin

#endif // CANOPEN_SETTINGS_STORE_H
