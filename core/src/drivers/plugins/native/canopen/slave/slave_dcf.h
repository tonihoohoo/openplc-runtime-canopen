// SPDX-License-Identifier: Apache-2.0
//
// slave_dcf.h - a CANopen slave's object dictionary as a CiA 306 DCF that Lely loads, built at
// PLC start from the slave's configuration: the communication objects from the settings and the
// application objects from the data rows.

#ifndef CANOPEN_SLAVE_DCF_H
#define CANOPEN_SLAVE_DCF_H

#include <cstdint>
#include <string>

#include "config.h"

namespace canopen_plugin
{

// The DCF of `dev` with `serial` in 0x1018 sub 4, starting at `node_id` (255: none, the slave
// waits for an LSS master). Pre-defined COB-IDs (EMCY, SDO, PDO 1-4) are given as $NODEID+base,
// so they follow a node ID given over LSS. Application values are left at 0 here; the slave
// sets them from the configuration after loading, as their raw bits.
std::string slave_dcf(const DeviceConfig &dev, uint32_t serial, unsigned node_id);

} // namespace canopen_plugin

#endif // CANOPEN_SLAVE_DCF_H
