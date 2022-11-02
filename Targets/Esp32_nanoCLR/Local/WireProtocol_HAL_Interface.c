//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
#include <wpUSART_Communications.h>
#include "WireProtocol_Message.h"

WP_Message inboundMessage;

#define WIRE_PROTOCOL SERIAL

#if WIRE_PROTOCOL == SERIAL
#define wp_ReadFromBuffer(ptr, size, wait_time) wp_ReadFromUsartBuffer(ptr, size, wait_time)
#define wp_WriteToBuffer(payload, size)         wp_WriteToUsartBuffer(payload, size)
#else
#define wp_ReadFromBuffer(ptr, size, wait_time) wp_ReadFromUsbBuffer(ptr, size, wait_time)
#define wp_WriteToBuffer(payload, size)         wp_WriteToUsbBuffer(payload, size)
#endif

void WP_ReceiveBytes(uint8_t **ptr, uint32_t *size)
{
    if (*size != 0) // Can get 0 for size if all header and payload comes through quickly on previous read
    {
        size_t read = wp_ReadFromBuffer(ptr, size, TX_WAIT_FOREVER);
        *ptr += read;
        *size -= read;
    }
}
uint8_t WP_TransmitMessage(WP_Message *message)
{
    wp_WriteToBuffer((uint8_t *)&message->m_header, sizeof(message->m_header));
    if (message->m_header.m_size && message->m_payload) // if there is anything on the payload send it to
                                                        // the output stream
    {
        wp_WriteToBuffer(message->m_payload, message->m_header.m_size);
    }
    return true;
}
