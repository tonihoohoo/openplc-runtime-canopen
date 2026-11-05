// SPDX-License-Identifier: Apache-2.0
//
// CO_driver_target.h - CANopenNode's target definitions for the plugin's slave (301/CO_driver.h).
// One thread runs a slave: it receives the frames, calls CO_process() and sends, and nothing else
// touches the stack's objects, so the lock macros are empty. Frames go through a SlaveCanPort,
// which is a SocketCAN socket at run time and Lely's in-process bus in the tests.

#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The stack's features. Everything else (SDO client, TIME, LSS, LEDs, storage, gateway, node
// guarding, SRDO, GFC) is left out.
#define CO_MULTIPLE_OD
#define CO_CONFIG_NMT (0)
#define CO_CONFIG_HB_CONS                                                                          \
    (CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_HB_CONS_QUERY_FUNCT | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_NODE_GUARDING (0)
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_HISTORY)
// Error register (CiA 301 7.5.2.2): the generic bit for any active error, the manufacturer bit
// for the bits from CO_EM_MANUFACTURER_START on (the PLC stop is one).
#define CO_CONFIG_ERR_CONDITION_GENERIC                                                            \
    ((em->errorStatusBits[0] | em->errorStatusBits[1] | em->errorStatusBits[2] |                   \
      em->errorStatusBits[3] | em->errorStatusBits[4] | em->errorStatusBits[5] |                   \
      em->errorStatusBits[6] | em->errorStatusBits[7] | em->errorStatusBits[8] |                   \
      em->errorStatusBits[9]) != 0U)
#define CO_CONFIG_ERR_CONDITION_MANUFACTURER                                                       \
    ((em->errorStatusBits[6] | em->errorStatusBits[7] | em->errorStatusBits[8] |                   \
      em->errorStatusBits[9]) != 0U)
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 300
#define CO_CONFIG_SDO_CLI (0)
#define CO_CONFIG_TIME (0)
#define CO_CONFIG_SYNC (CO_CONFIG_SYNC_ENABLE | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_PDO                                                                              \
    (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE | CO_CONFIG_RPDO_TIMERS_ENABLE |                \
     CO_CONFIG_TPDO_TIMERS_ENABLE | CO_CONFIG_PDO_SYNC_ENABLE | CO_CONFIG_PDO_OD_IO_ACCESS |       \
     CO_CONFIG_FLAG_OD_DYNAMIC)
// Application objects have sub-indexes up to 254; each may request its event-driven TPDO.
#define OD_FLAGS_PDO_SIZE 32U
#define CO_CONFIG_LSS (0)
#define CO_CONFIG_LEDS (0)
#define CO_CONFIG_STORAGE (0)
#define CO_CONFIG_GTW (0)
#define CO_CONFIG_GFC (0)
#define CO_CONFIG_SRDO (0)
#define CO_CONFIG_TRACE (0)
#define CO_CONFIG_DEBUG (0)
#define CO_CONFIG_CRC16 (0)

#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
    typedef uint_fast8_t bool_t;
    typedef float float32_t;
    typedef double float64_t;

    // The way out to the bus: `send` returns 0 when the frame was queued.
    typedef struct SlaveCanPort
    {
        void *ctx;
        int (*send)(void *ctx, uint32_t ident, uint8_t dlc, const uint8_t *data);
    } SlaveCanPort;

    // A received frame as the stack's receive callbacks see it.
    typedef struct
    {
        uint32_t ident; // 11-bit identifier, CO_CAN_RTR_FLAG for a remote frame
        uint8_t dlc;
        uint8_t data[8];
    } CO_CANrxMsg_t;

#define CO_CAN_RTR_FLAG 0x8000U

#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((const CO_CANrxMsg_t *)(msg))->ident))
#define CO_CANrxMsg_readDLC(msg) ((uint8_t)(((const CO_CANrxMsg_t *)(msg))->dlc))
#define CO_CANrxMsg_readData(msg) ((const uint8_t *)(((const CO_CANrxMsg_t *)(msg))->data))

    typedef struct
    {
        uint16_t ident;
        uint16_t mask;
        void *object;
        void (*CANrx_callback)(void *object, void *message);
    } CO_CANrx_t;

    typedef struct
    {
        uint32_t ident;
        uint8_t DLC;
        uint8_t data[8];
        volatile bool_t bufferFull;
        volatile bool_t syncFlag;
    } CO_CANtx_t;

    typedef struct
    {
        void *CANptr; // SlaveCanPort *
        CO_CANrx_t *rxArray;
        uint16_t rxSize;
        CO_CANtx_t *txArray;
        uint16_t txSize;
        uint16_t CANerrorStatus;
        volatile bool_t CANnormal;
        volatile bool_t useCANrxFilters;
        volatile bool_t bufferInhibitFlag;
        volatile bool_t firstCANtxMessage;
        volatile uint16_t CANtxCount;
        uint32_t errOld;
    } CO_CANmodule_t;

    typedef struct
    {
        void *addr;
        size_t len;
        uint8_t subIndexOD;
        uint8_t attr;
        void *addrNV;
    } CO_storage_entry_t;

#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                         \
    {                                                                                              \
        CO_MemoryBarrier();                                                                        \
        rxNew = (void *)1L;                                                                        \
    }
#define CO_FLAG_CLEAR(rxNew)                                                                       \
    {                                                                                              \
        CO_MemoryBarrier();                                                                        \
        rxNew = NULL;                                                                              \
    }

    // Hands one received frame to the stack's receive objects (the slave thread calls it).
    void co_slave_receive(CO_CANmodule_t *module, const CO_CANrxMsg_t *msg);

#ifdef __cplusplus
}
#endif

#endif // CO_DRIVER_TARGET_H
