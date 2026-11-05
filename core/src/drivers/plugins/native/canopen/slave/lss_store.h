// SPDX-License-Identifier: Apache-2.0
//
// lss_store.h - what an LSS master stored for a slave (node ID, bit rate) and the slave's automatic
// serial number, kept per CAN interface in a small JSON file in the runtime's persistent data
// folder, so they survive a restart, a reboot and a runtime update. Only the slave thread and
// Slave::start() call these; never the PLC scan.

#ifndef CANOPEN_LSS_STORE_H
#define CANOPEN_LSS_STORE_H

#include <cstdint>
#include <string>

namespace canopen_plugin
{

struct LssStored
{
    uint32_t serial  = 0; // 0: none kept yet
    uint8_t node_id  = 0; // 0: none stored; 1-127 or 0xFF (unconfigured)
    unsigned bitrate = 0; // bit/s; 0: none stored
};

// The runtime's persistent data folder, found as the web server finds it:
// OPENPLC_PERSISTENT_DATA_DIR, else /var/run/runtime in a container, else /var/lib/openplc-runtime.
std::string lss_data_dir();
// <data dir>/canopen/lss-<interface>.json
std::string lss_store_path(const std::string &interface);

// False with a reason when the file is missing, unreadable or not a valid store; `out` is then
// empty. A missing file gives an empty reason.
bool lss_load(const std::string &path, LssStored &out, std::string &problem);
// Writes the file atomically (temp file, fsync, rename), creating its folder. False with a reason.
bool lss_save(const std::string &path, const LssStored &value, std::string &problem);

// A random serial number, never 0.
uint32_t lss_random_serial();

} // namespace canopen_plugin

#endif // CANOPEN_LSS_STORE_H
