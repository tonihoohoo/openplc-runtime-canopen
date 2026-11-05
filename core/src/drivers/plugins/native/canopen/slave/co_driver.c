// SPDX-License-Identifier: Apache-2.0
//
// co_driver.c - CANopenNode's CAN driver functions (301/CO_driver.h) on a SlaveCanPort. Frames
// arrive through co_slave_receive() and leave through the port's send; a frame the port cannot
// take is kept in its buffer and sent again from CO_CANmodule_process().

#include <string.h>

#include "301/CO_driver.h"

void CO_CANsetConfigurationMode(void *CANptr)
{
    (void)CANptr;
}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    CANmodule->CANnormal = true;
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule, void *CANptr, CO_CANrx_t rxArray[],
                                   uint16_t rxSize, CO_CANtx_t txArray[], uint16_t txSize,
                                   uint16_t CANbitRate)
{
    (void)CANbitRate;
    if (CANmodule == NULL || CANptr == NULL || rxArray == NULL || txArray == NULL)
        return CO_ERROR_ILLEGAL_ARGUMENT;
    CANmodule->CANptr            = CANptr;
    CANmodule->rxArray           = rxArray;
    CANmodule->rxSize            = rxSize;
    CANmodule->txArray           = txArray;
    CANmodule->txSize            = txSize;
    CANmodule->CANerrorStatus    = 0;
    CANmodule->CANnormal         = false;
    CANmodule->useCANrxFilters   = false;
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount        = 0;
    CANmodule->errOld            = 0;
    for (uint16_t i = 0; i < rxSize; i++)
    {
        rxArray[i].ident          = 0;
        rxArray[i].mask           = 0xFFFF;
        rxArray[i].object         = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (uint16_t i = 0; i < txSize; i++)
        txArray[i].bufferFull = false;
    return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL)
        CANmodule->CANnormal = false;
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident,
                                    uint16_t mask, bool_t rtr, void *object,
                                    void (*CANrx_callback)(void *object, void *message))
{
    if (CANmodule == NULL || object == NULL || CANrx_callback == NULL || index >= CANmodule->rxSize)
        return CO_ERROR_ILLEGAL_ARGUMENT;
    CO_CANrx_t *buffer     = &CANmodule->rxArray[index];
    buffer->object         = object;
    buffer->CANrx_callback = CANrx_callback;
    buffer->ident          = (uint16_t)((ident & 0x07FFU) | (rtr ? CO_CAN_RTR_FLAG : 0U));
    buffer->mask           = (uint16_t)((mask & 0x07FFU) | CO_CAN_RTR_FLAG);
    return CO_ERROR_NO;
}

CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident,
                               bool_t rtr, uint8_t noOfBytes, bool_t syncFlag)
{
    if (CANmodule == NULL || index >= CANmodule->txSize)
        return NULL;
    CO_CANtx_t *buffer = &CANmodule->txArray[index];
    buffer->ident      = (uint32_t)((ident & 0x07FFU) | (rtr ? CO_CAN_RTR_FLAG : 0U));
    buffer->DLC        = noOfBytes;
    buffer->bufferFull = false;
    buffer->syncFlag   = syncFlag;
    return buffer;
}

static int port_send(CO_CANmodule_t *CANmodule, const CO_CANtx_t *buffer)
{
    const SlaveCanPort *port = (const SlaveCanPort *)CANmodule->CANptr;
    return port->send(port->ctx, buffer->ident, buffer->DLC, buffer->data);
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    if (buffer->bufferFull)
    {
        if (!CANmodule->firstCANtxMessage)
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        return CO_ERROR_TX_OVERFLOW;
    }
    if (port_send(CANmodule, buffer) == 0)
    {
        CANmodule->firstCANtxMessage = false;
        return CO_ERROR_NO;
    }
    buffer->bufferFull = true;
    CANmodule->CANtxCount++;
    return CO_ERROR_NO;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;
    for (uint16_t i = 0; i < CANmodule->txSize; i++)
    {
        CO_CANtx_t *buffer = &CANmodule->txArray[i];
        if (buffer->bufferFull && buffer->syncFlag)
        {
            buffer->bufferFull = false;
            CANmodule->CANtxCount--;
            tpdoDeleted = 1U;
        }
    }
    if (tpdoDeleted != 0U)
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    if (CANmodule->CANtxCount == 0U)
        return;
    for (uint16_t i = 0; i < CANmodule->txSize; i++)
    {
        CO_CANtx_t *buffer = &CANmodule->txArray[i];
        if (!buffer->bufferFull)
            continue;
        if (port_send(CANmodule, buffer) != 0)
            return;
        buffer->bufferFull = false;
        CANmodule->CANtxCount--;
    }
    CANmodule->CANerrorStatus &= (uint16_t)~CO_CAN_ERRTX_OVERFLOW;
}

void co_slave_receive(CO_CANmodule_t *module, const CO_CANrxMsg_t *msg)
{
    if (module == NULL || !module->CANnormal)
        return;
    for (uint16_t i = 0; i < module->rxSize; i++)
    {
        CO_CANrx_t *buffer = &module->rxArray[i];
        if (buffer->CANrx_callback == NULL)
            continue;
        if (((msg->ident ^ buffer->ident) & buffer->mask) == 0U)
        {
            buffer->CANrx_callback(buffer->object, (void *)msg);
            break;
        }
    }
}
