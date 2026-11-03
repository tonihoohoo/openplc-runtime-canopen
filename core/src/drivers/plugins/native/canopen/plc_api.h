// SPDX-License-Identifier: Apache-2.0
//
// plc_api.h - the C interface the canopen function block library calls from the PLC program.
// The blocks find canopen_plc_api() in the loaded plugin (dlopen of the SONAME with RTLD_NOLOAD,
// then dlsym). The runtime closes and opens the plugin again across PLC stops while the program
// stays loaded, so the blocks keep the table only when its flags say it stays valid; otherwise they
// look it up on each call. A handle from an earlier PLC run is answered as ended by the restart.
// Every function returns at once and is safe to call from the PLC scan: no allocation, no lock,
// no logging. Later versions only append to the table; `size` tells which fields are present.
// The block library carries a copy of these declarations; keep both in step.

#ifndef CANOPEN_PLC_API_H
#define CANOPEN_PLC_API_H

#include <stdint.h>

#define CANOPEN_PLC_API_VERSION 2u
#define CANOPEN_PLC_MAX_DATA 256u

// canopen_plc_api_v1.flags: the library is never unmapped, so the table and its functions stay
// valid for the life of the process, across PLC stops, starts and reloads of the plugin.
#define CANOPEN_PLC_FLAG_PERMANENT 0x1u

// CANOPEN_KERNEL_ERROR of the block library, in its order.
enum canopen_plc_error
{
    CANOPEN_PLC_NO_ERROR        = 0,
    CANOPEN_PLC_OTHER_ERROR     = 1, // the node aborted the transfer (abort code set)
    CANOPEN_PLC_TIMEOUT         = 2,
    CANOPEN_PLC_BUS_OFF         = 3, // bus-off, link down or the session is being reset
    CANOPEN_PLC_ERROR_PASSIVE   = 4,
    CANOPEN_PLC_INTERNAL        = 5,
    CANOPEN_PLC_WRONG_PARAMETER = 6,
    CANOPEN_PLC_NODE_ID_UNKNOWN = 7,
    CANOPEN_PLC_NET_ID_UNKNOWN  = 8,
    CANOPEN_PLC_REQUEST_ERROR   = 9, // the request cannot run now (reason set)
    CANOPEN_PLC_NO_DRIVER       = 10 // no CANopen network runs, or the request ended with the PLC
};

// Why a request failed without an abort code, for the block's error text.
enum canopen_plc_reason
{
    CANOPEN_PLC_REASON_NONE      = 0,
    CANOPEN_PLC_REASON_BOOTING   = 1, // the master is booting the node
    CANOPEN_PLC_REASON_FULL      = 2, // the network's block queue is full
    CANOPEN_PLC_REASON_NOT_RUN   = 3, // no CANopen network runs
    CANOPEN_PLC_REASON_RESTART   = 4, // the PLC stopped while the request was pending
    CANOPEN_PLC_REASON_LINK      = 5, // the link went down or was reset
    CANOPEN_PLC_REASON_TOO_LARGE = 6, // more than CANOPEN_PLC_MAX_DATA bytes (length set)
    CANOPEN_PLC_REASON_LENGTH    = 7, // a write length of 0 or above the limit
    CANOPEN_PLC_REASON_COMMAND   = 8, // an NMT command that does not exist
    CANOPEN_PLC_REASON_VERSION   = 9  // the plugin does not offer the version asked for
};

enum canopen_plc_kind
{
    CANOPEN_PLC_READ  = 0,
    CANOPEN_PLC_WRITE = 1,
    CANOPEN_PLC_NMT   = 2
};

// DEVICE_STATE of the block library, in its order.
enum canopen_plc_device_state
{
    CANOPEN_PLC_INIT            = 0,
    CANOPEN_PLC_RESET_COMM      = 1,
    CANOPEN_PLC_RESET_APP       = 2,
    CANOPEN_PLC_PRE_OPERATIONAL = 3,
    CANOPEN_PLC_STOPPED         = 4,
    CANOPEN_PLC_OPERATIONAL     = 5,
    CANOPEN_PLC_UNKNOWN         = 6,
    CANOPEN_PLC_NOT_AVAIL       = 7
};

// CANOPEN_RUN_STATE of the block library, in its order: the node's state as the master runs it.
enum canopen_plc_run_state
{
    CANOPEN_PLC_RUN_NOT_FOUND     = 0, // not booted yet (waiting, not answering or absent)
    CANOPEN_PLC_RUN_BOOTING       = 1,
    CANOPEN_PLC_RUN_FAILED        = 2, // the last boot failed; retried in the background
    CANOPEN_PLC_RUN_WAITING_START = 3, // configured, pre-operational until the program starts it
    CANOPEN_PLC_RUN_OPERATIONAL   = 4,
    CANOPEN_PLC_RUN_MANUAL        = 5, // stopped or pre-operational by the program or a client
    CANOPEN_PLC_RUN_LOST          = 6, // was up; booted again in the background
    CANOPEN_PLC_RUN_HELD          = 7, // lost and stopped until the next PLC start
    CANOPEN_PLC_RUN_DISABLED      = 8, // off
    CANOPEN_PLC_RUN_STOPPED       = 9  // the PLC stopped
};

// canopen_plc_node_status.flags
#define CANOPEN_PLC_NODE_ENABLED 0x01u       // on, as the master runs it
#define CANOPEN_PLC_NODE_OPTIONAL 0x02u      // optional
#define CANOPEN_PLC_NODE_PROGRAM_START 0x04u // started by the PLC program
#define CANOPEN_PLC_NODE_SILENT 0x08u        // mandatory and not answering

#define CANOPEN_PLC_TEXT 80u

typedef struct canopen_plc_node_status
{
    uint8_t run_state; // canopen_plc_run_state
    uint8_t flags;     // CANOPEN_PLC_NODE_*
    uint8_t reserved[2];
    char problem[CANOPEN_PLC_TEXT]; // the boot problem or loss in words while failed, lost or held
} canopen_plc_node_status;

typedef struct canopen_plc_network_status
{
    uint8_t mandatory_up; // every mandatory node that is on is operational or waits for its start
    uint8_t optional_up;  // the same for the optional nodes
    uint8_t configured;   // nodes operational, waiting for their start, or set by hand
    uint8_t operational;
    uint8_t waiting;  // waiting for the program's start
    uint8_t disabled; // off
    uint8_t faulty;   // failed, lost, held, or mandatory and not answering
    uint8_t nodes;    // configured in the network
} canopen_plc_network_status;

typedef struct canopen_plc_request
{
    uint8_t network; // counted from 1
    uint8_t node;    // NMT: 0 for every node of the network
    uint8_t kind;    // canopen_plc_kind
    uint8_t command; // NMT: the CiA 301 command specifier (1, 2, 128, 129, 130)
    uint16_t index;
    uint8_t subindex;
    uint8_t reserved;
    uint32_t timeout_ms; // 0: the network's SDO timeout
    uint32_t length;     // write: bytes in data
    const uint8_t *data; // write; copied before start returns
} canopen_plc_request;

typedef struct canopen_plc_result
{
    uint8_t error;  // canopen_plc_error
    uint8_t reason; // canopen_plc_reason
    uint16_t reserved;
    uint32_t abort_code; // CANOPEN_PLC_OTHER_ERROR, CANOPEN_PLC_TIMEOUT
    uint32_t length;     // read: bytes received (also when more than the caller could take)
} canopen_plc_result;

typedef struct canopen_plc_emcy
{
    uint8_t node;
    uint8_t error_register;
    uint16_t error_code;
    uint8_t data[5];
    uint8_t reserved[3];
    uint32_t lost; // messages skipped before this one because the reader fell behind
} canopen_plc_emcy;

typedef struct canopen_plc_api_v1
{
    uint32_t size;
    uint32_t version;
    // Queues a request. Returns its handle, or 0 with `res` saying why it cannot run.
    uint64_t (*start)(const canopen_plc_request *req, canopen_plc_result *res);
    // 0 while the request runs; 1 done, 2 failed, with `res` set. A read copies up to `cap`
    // bytes to `data`. A finished request is forgotten: poll it once.
    int (*poll)(uint64_t handle, canopen_plc_result *res, uint8_t *data, uint32_t cap);
    // Drops a request; a transfer already on the bus ends and its result is discarded.
    void (*cancel)(uint64_t handle);
    // These return a canopen_plc_error.
    int (*node_state)(uint8_t network, uint8_t node, uint8_t *state);
    int (*kernel_state)(uint8_t network, uint8_t *state);
    int (*local_node_id)(uint8_t network, uint8_t *node_id);
    // The position after the newest stored emergency of the network.
    int (*emcy_cursor)(uint8_t network, uint64_t *cursor);
    // The next emergency at or after `cursor` (of `node`, or any with 0): 1 and the message,
    // with `cursor` moved past it; 0 when there is none; a negative canopen_plc_error on a
    // failure.
    int (*emcy_next)(uint8_t network, uint8_t node, uint64_t *cursor, canopen_plc_emcy *out);
    // CANOPEN_PLC_FLAG_*; present when `size` covers it.
    uint32_t flags;
} canopen_plc_api_v1;

// Version 2 appends the node control: the table starts with version 1's.
typedef struct canopen_plc_api_v2
{
    canopen_plc_api_v1 v1;
    int (*node_status)(uint8_t network, uint8_t node, canopen_plc_node_status *out);
    int (*network_status)(uint8_t network, canopen_plc_network_status *out);
    // Asks the master to turn the node on (1) or off (0); it applies the request on its next tick
    // and reports it in node_status's CANOPEN_PLC_NODE_ENABLED. Lasts until the PLC stops.
    int (*node_enable)(uint8_t network, uint8_t node, uint8_t on);
} canopen_plc_api_v2;

#ifdef __cplusplus
#define CANOPEN_PLC_EXTERN_C extern "C"
#else
#define CANOPEN_PLC_EXTERN_C
#endif

// Exported by the plugin. Returns the table of `version` (1 or 2; version 2's table serves
// version 1 too), or NULL when it has none.
CANOPEN_PLC_EXTERN_C const void *canopen_plc_api(uint32_t version);

#endif // CANOPEN_PLC_API_H
