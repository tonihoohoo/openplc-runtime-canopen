// SPDX-License-Identifier: Apache-2.0
//
// master_dcf.h - the master's own device configuration (a CiA 306 DCF that Lely loads) built from
// a network of conf/canopen.json: NMT master and slave assignment, expected identities, heartbeat
// producer and consumers, emergency consumers, SYNC, and one master PDO per node PDO. Each mapped
// entry gets a manufacturer object of its type: inputs from 0x2000, outputs from 0x3000.

#ifndef CANOPEN_MASTER_DCF_H
#define CANOPEN_MASTER_DCF_H

#include <cstdint>
#include <string>
#include <vector>

#include "config.h"

namespace canopen_plugin
{

constexpr uint16_t kInputObjectBase  = 0x2000;
constexpr uint16_t kOutputObjectBase = 0x3000;

// One master PDO and the node PDO it mirrors.
struct MasterPdo
{
    unsigned master_number = 0; // 1-based, RPDO for inputs, TPDO for outputs
    unsigned node_id       = 0;
    const PdoConfig *pdo   = nullptr;
    size_t first_entry     = 0; // index of its first entry in the master's input or output list
};

struct MasterLayout
{
    std::vector<MasterPdo> rpdos; // node TPDOs, received by the master
    std::vector<MasterPdo> tpdos; // node RPDOs, sent by the master
    size_t input_entries  = 0;
    size_t output_entries = 0;
};

MasterLayout master_layout(const NetworkConfig &net);

// The DCF text for the network's master.
std::string master_dcf(const NetworkConfig &net, const MasterLayout &layout);

// The node's configuration SDOs as a concise DCF (CiA 302-3: count, then index, sub-index, size
// and data of each entry, all little-endian).
std::vector<uint8_t> concise_dcf(const std::vector<SdoWrite> &sdo);

} // namespace canopen_plugin

#endif // CANOPEN_MASTER_DCF_H
