//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>
#include <wpUSART_Communications.h>
#include <WireProtocol.h>

WP_Message inboundMessage;

static bool Initialized = false;

#include <stdio.h>
void WP_ReceiveBytes(uint8_t **ptr, uint32_t *size)
{
    if (*size != 0) // Can get 0 for size if all header and payload comes through quickly on previous read
    {
        size_t read = wp_ReadBytes(ptr, size, TX_WAIT_FOREVER);
        *ptr += read;
        *size -= read;
    }
}
uint8_t WP_TransmitMessage(WP_Message *message)
{
    wp_WriteBytes((uint8_t *)&message->m_header, sizeof(message->m_header));
    if (message->m_header.m_size && message->m_payload) // if there is anything on the payload send it to
                                                        // the output stream
    {
        wp_WriteBytes(message->m_payload, message->m_header.m_size);
    }
    return true;
}
