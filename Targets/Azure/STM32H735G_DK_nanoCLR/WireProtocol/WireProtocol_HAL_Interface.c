#include <Debug_To_Display.h>
//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <WireProtocol.h>
#include <nanoHAL_v2.h>
#include <wpUSART_Communications.h>

WP_Message inboundMessage;

static bool Initialized = false;

void DISPLAY_HEADER(WP_Message *message);
void DISPLAY_PAYLOAD(WP_Message *message);

#include <stdio.h>

void PrintData(uint8_t *buf, int count, bool formfeed)
{
    if (formfeed)
    {
        lcd_printf("\f");
    }
    // each line is 16 bytes
    for (uint16_t i = 0; i < count; i++)
    {
        const uint8_t byte = buf[i];
        lcd_printf("%02X ", byte);
    }

    lcd_printf("|\r\n");
}



void WP_ReceiveBytes(uint8_t **ptr, uint32_t *size)
{
    if (*size != 0) // Required for the PING case where payload is 0
    {
        size_t read = wp_ReadBytes(ptr, size, TX_WAIT_FOREVER);
        *ptr += read;
        *size -= read;
    }
}
uint8_t WP_TransmitMessage(WP_Message *message)
{
    PrintData((char *)&message->m_header, sizeof(message->m_header), true);

    wp_WriteBytes((uint8_t *)&message->m_header, sizeof(message->m_header));
    if (message->m_header.m_size && message->m_payload) // if there is anything on the payload send it to
                                                        // the output stream
    {
        lcd_printf("------------------\r\n");
        PrintData((char *)&message->m_payload, message->m_header.m_size, false);
        wp_WriteBytes(message->m_payload, message->m_header.m_size);
    }
    return true;
}
