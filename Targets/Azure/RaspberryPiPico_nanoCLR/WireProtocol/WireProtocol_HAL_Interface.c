//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
/*
Board: RaspberryPiPico

   RaspberryPiPico-DK___________________     ___PC____
  |   _________________________________|    |         |
  |  |                     |           |    |         |
  |  |                     |           |    | Virtual |
  |  |   STDIO redirected  |VCP UART   |<==>| Com     |
  |  |                     |           |    | Port    |
  |  |_____________________|___________|    |         |
  |____________________________________|    |_________|

*/

#include "WireProtocol_Message.h"
#include "wp_Communications.h"
#include "WireProtocol_HAL_Interface.h"

WP_Message inboundMessage;
TX_EVENT_FLAGS_GROUP wpUartReceivedBytesEvent;

void InitWireProtocolCommunications()
{
    bool success = stdio_usb_init();
    // Create event based data synchronization
    tx_event_flags_create(&wpUartReceivedBytesEvent, "wpUart event");
}

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

int wp_ReadBytes(uint8_t **ptr, uint32_t *size, uint32_t wait_time)
{
    ULONG actual_flags;
    tx_event_flags_get(&wpUartReceivedBytesEvent, 0x1, TX_OR_CLEAR, &actual_flags, wait_time);

    int requestedSize = *size;
    int bytesRead = stdio_usb.in_chars((char *)ptr, requestedSize);
    if (bytesRead == PICO_ERROR_NO_DATA)
    {
        return 0;
    }
    else
    {
        return bytesRead;
    }
}

bool wp_WriteBytes(uint8_t *ptr, uint16_t size)
{
    // Send header
    stdio_usb.out_chars((char *)ptr, size); // 20 millisecond timeout
    return true;
}

void tud_cdc_rx_cb(uint8_t itf)
{
    int bytesAvailable = tud_cdc_n_available(0);
    tx_event_flags_set(&wpUartReceivedBytesEvent, 0x1, TX_OR);
}