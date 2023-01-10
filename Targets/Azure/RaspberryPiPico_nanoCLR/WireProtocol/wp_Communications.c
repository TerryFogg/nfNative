//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
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

#include "wp_Communications.h"
#include "tusb.h"
#include "wp_CircularBuffer.h"
#include <lib/tinyusb/src/device/usbd.h>
#include <tx_api.h>

enum UsbEventTypes
{
    Setup = 1,
    Data = 2,
    Reset = 3,
    Suspend = 4,
    Resume = 5
};
enum UsbEventFlags
{
    SetupFlag = 0x00001b,
    DataFlag = 0x00010b,
    ResetFlag = 0x00100b,
    SuspendFlag = 0x01000b,
    ResumeFlag = 0x10000b
};

#define PICO_ERROR_NO_DATA -3
TX_EVENT_FLAGS_GROUP wpReceivedEvent;
TX_EVENT_FLAGS_GROUP wpReceivedEvent;

// Length of currently active TX DMA transfer
volatile size_t tx_current_len;

// Circular buffer instance for receive data
CircularBuffer_t ReceiveCircularBuffer;

// Circular buffer data array for Receive
uint8_t wp_ReceiveData[2048];

// Circular buffer instance for Transmit data
CircularBuffer_t TransmitCircularBuffer;

// Circular buffer data array for Transmit
uint8_t wp_TransmitData[2048];

static bool usbConnected = false;

void InitWireProtocolCommunications()
{
    // Initialize Tinyusb
    tusb_init();

    usbConnected = true;
    wp_InitializeBuffer(&TransmitCircularBuffer, wp_TransmitData, sizeof(wp_TransmitData));
    wp_InitializeBuffer(&ReceiveCircularBuffer, wp_ReceiveData, sizeof(wp_ReceiveData));

    // Create event based data synchronization
    tx_event_flags_create(&wpReceivedEvent, "wpReceiveDataEvent");
}
int wp_ReadBytes(uint8_t **ptr, uint32_t *size, uint32_t wait_time)
{
    ULONG actual_flags;
    uint32_t requestedSize = *size;
    ULONG read = 0;

    // Wait for a data event to be received from the USB
    bool DataReceivedEvent = false;

    while (!DataReceivedEvent)
    {
        tx_event_flags_get(&wpReceivedEvent, 0x11111b, TX_OR_CLEAR, &actual_flags, wait_time);

        tud_task();

        switch (actual_flags)
        {
            //        case SetupFlag:
            //            break;
        case DataFlag:
            tud_task();
            read = wp_ReadBuffer(&ReceiveCircularBuffer, *ptr, requestedSize);
            break;
            //        case ResetFlag:
            //            tud_task();
            //            break;
            //        case SuspendFlag:
            //            tud_task();
            //            break;
            //        case ResumeFlag:
            //            tud_task();
            //            break;
        }
    }
    // Bytes have arrived, try to read what was requested
    return read;
}
bool wp_WriteBytes(uint8_t *ptr, uint16_t size)
{
    wp_WriteBuffer(&TransmitCircularBuffer, ptr, size); // Write data to transmit buffer
    wp_StartTransmitTransfer();
    return true;
}
void wp_DataReceived(void)
{
    static size_t old_position;

    uint8_t buf[64];
    int length_data_received = tud_cdc_n_read(0, buf, sizeof(buf));

    /* Calculate current position in buffer and check for new data available */
    size_t position = ARRAY_LEN(wp_ReceiveData) - length_data_received;

    if (position != old_position)
    {
        if (position > old_position)
        {
            wp_WriteBuffer(
                &ReceiveCircularBuffer,
                &wp_ReceiveData[old_position],
                position - old_position); /* Write data to receive buffer */
        }
        else
        {
            // Processing is done in "overflow" mode..
            wp_WriteBuffer(
                &ReceiveCircularBuffer,
                &wp_ReceiveData[old_position],
                ARRAY_LEN(wp_ReceiveData) - old_position); /* Write data to receive buffer */

            if (position > 0)
            {
                wp_WriteBuffer(
                    &ReceiveCircularBuffer,
                    &wp_ReceiveData[0],
                    position); /* Write data to receive buffer */
            }
        }
        old_position = position;
        tx_event_flags_set(&wpReceivedEvent, 0x1, TX_OR);
    }
}
uint8_t wp_StartTransmitTransfer(void)
{
    uint8_t started = 0;
    size_t length;
    /* Use temporary values in case they are changed during operations */
    size_t w = TransmitCircularBuffer.w;
    size_t r = TransmitCircularBuffer.r;
    if (w > r)
    {
        length = w - r;
    }
    else if (r > w)
    {
        length = TransmitCircularBuffer.size - r;
    }
    else
    {
        length = 0;
    }

    if (tx_current_len == 0 && (tx_current_len = length) > 0)
    {
        // Wait for any previous USB Transfer to finish
        while (!tud_cdc_write_available())
        {
            tud_task();
            tx_thread_relinquish();
        }
    }
    uint32_t writeResult = tud_cdc_n_write(0, &TransmitCircularBuffer.buffer[TransmitCircularBuffer.r], tx_current_len);

    return started;
}

void WireProtocolUsb_dcd_event_rp2040(int eventType)
{
    // This runs in interrupt context,so set a flag for executing code out of interrupt context
    switch (eventType)
    {
    case Setup:
        tx_event_flags_set(&wpReceivedEvent, 0x00001b, TX_OR);
        break;
    case Data:
        wp_DataReceived();
        tx_event_flags_set(&wpReceivedEvent, 0x00010b, TX_OR);
        break;
    case Reset:
        tx_event_flags_set(&wpReceivedEvent, 0x00100b, TX_OR);
        break;
    case Suspend:
        tx_event_flags_set(&wpReceivedEvent, 0x01000b, TX_OR);
        break;
    case Resume:
        tx_event_flags_set(&wpReceivedEvent, 0x10000b, TX_OR);
        break;
    }
}
