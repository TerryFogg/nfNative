//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
/*
Board: RaspberryPiPico

   _ RaspberryPiPico-DK ___      ____PC___
  |   ________________     |    |         |
  |  |                |    |    | Virtual |
  |  |    VCP UART    |    |<==>| Com     |
  |  |                |    |    | Port    |
  |  |________________|    |    |         |
  |________________________|    |_________|
*/

#include "WireProtocol_HAL_Interface.h"
#include "Debug_To_Display.h"
#include "tusb.h"
#include <ctype.h>
#include <hardware/irq.h>
#include <lib/tinyusb/src/device/usbd.h>
#include "targetHAL.h"
#include "nanoHAL_v2.h"

#define PICO_ERROR_NO_DATA -3
WP_Message inboundMessage;
TX_EVENT_FLAGS_GROUP wpUsbXferEvent;


void InitWireProtocolCommunications()
{
    // Create event based data synchronization
    tx_event_flags_create(&wpUsbXferEvent, "wpProtocolXferEvent");

    // Initialize Tinyusb
    tusb_init();
}
void WP_ReceiveBytes(uint8_t **ptr, uint32_t *size)
{
    ULONG actual_flags;

    // Required for the PING case where payload 0
    if (*size != 0)
    {

//        tx_event_flags_get(&wpUsbSetupEvent, 0x1, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        while (!tud_cdc_n_connected(0))
        {
            tud_task();
        }


//        while (!tud_cdc_n_available(0))
//        {
//            tud_task();
//        }
        size_t read = 0;
        int requestedSize = *size;
        while ( (read = tud_cdc_n_read(0, (char *)*ptr, requestedSize)) == 0  )
        {
            tx_event_flags_get(&wpUsbXferEvent, 0x1, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        }
//       
//        tud_cdc_n_read_flush(0);
        *ptr += read;
        *size -= read;
    }
}
uint8_t WP_TransmitMessage(WP_Message *message)
{
    bool operationResult = false;
    uint32_t writeResult = tud_cdc_n_write(0, (uint8_t *)&message->m_header, sizeof(message->m_header));

    if (writeResult == sizeof(message->m_header))
    {
        operationResult = true;
        if (message->m_header.m_size && message->m_payload)
        {
            operationResult = false;

            while (!tud_cdc_write_available())
            {
                tud_task();
                tx_thread_relinquish();
            }

            writeResult = tud_cdc_n_write(0, (uint8_t *)message->m_payload, message->m_header.m_size);
            if (writeResult == message->m_header.m_size)
            {
                operationResult = true;
            }
        }

        //  Wait until data sent
        while (!tud_cdc_write_available())
        {
            tud_task();
            tx_thread_relinquish();
        }
    }
    tud_cdc_n_write_flush(0);
    return operationResult;
}

void WireProtocolUsb_DCD_EVENT_XFER_COMPLETE()
{
    tx_event_flags_set(&wpUsbXferEvent, 0x1, TX_OR);
}
