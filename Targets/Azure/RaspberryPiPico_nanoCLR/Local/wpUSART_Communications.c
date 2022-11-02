//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

/*
Board: RaspberryPiPico

   RaspberryPiPico-DK___   ______PicoProbe ____      ___PC____
  |   _________________|   |_______            |    |         |
  |  |     USART       |   | USART |           |    |         |
  |  | (wpUSART_TX_PIN)|---|-RX    |           |    | Virtual |
  |  |                 |   |       | usb_dev_hs|<==>| Com     |
  |  | (wpUSART_RX_PIN)|---|-TX    |           |    | Port    |
  |  |_________________|   |_______|           |    |         |
  |____________________|   |___________________|    |_________|

*/
#include "wpUSART_Communications.h"

void Dma_Receive_Interrupt(void);

uint8_t wpUSART_DMA_Receive_Buffer[wpUSART_DMA_Receive_Buffer_size];
// ? RPICO ? different rules
static_assert((sizeof(wpUSART_DMA_Receive_Buffer) % 32) == 0, "Must be a multiple by 32");

// Circular buffer for receive data
CircularBuffer_t wp_UsartReceiveCircularBuffer;
uint8_t wp_ReceiveData[2048];

// Circular buffer for transmit data
CircularBuffer_t wp_UsartTransmitCircularBuffer;
uint8_t wp_TransmitData[2048];

//  Length of currently active TX DMA transfer
volatile size_t usart_tx_dma_current_len;

TX_EVENT_FLAGS_GROUP wpUartReceivedBytesEvent;

void InitWireProtocolCommunications()
{
    // Initialize transfer (TX) circular buffer
    wp_InitializeBuffer(&wp_UsartTransmitCircularBuffer, wp_TransmitData, sizeof(wp_TransmitData));

    // Initialize receive (RX) circular buffer
    wp_InitializeBuffer(&wp_UsartReceiveCircularBuffer, wp_ReceiveData, sizeof(wp_ReceiveData));

    wp_InitializeUsart();

    wp_InitializeDma();

    // Create event based data synchronization
    tx_event_flags_create(&wpUartReceivedBytesEvent, "wpUart event");
}
void wp_InitializeUsart(void)
{
    uart_inst_t *uart = wpUSART;
    uart_init(uart, wpBAUD_RATE);
    gpio_set_function(wpUSART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(wpUSART_RX_PIN, GPIO_FUNC_UART);
    // NOTE: get rid of the first junk byte
    uart_getc(uart);
}
void wp_InitializeDma(void)
{
    /// DMA uart read
    dma_channel_config rx_config = dma_channel_get_default_config(wpRXChannel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_ring(&rx_config, true, kRxBuffLengthPow);
    channel_config_set_dreq(&rx_config, wpDREQ_UART_RX);
    channel_config_set_enable(&rx_config, true);
    dma_channel_configure(wpRXChannel, &rx_config, wp_ReceiveData, &wp_UART_HW->dr, sizeof(wp_ReceiveData), true);
    dma_channel_set_irq0_enabled(wpRXChannel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, Dma_Receive_Interrupt);
    irq_set_enabled(DMA_IRQ_0, true);

    /// DMA uart write
    dma_channel_config tx_config = dma_channel_get_default_config(wpTXChannel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_ring(&tx_config, false, kTxBuffLengthPow);
    channel_config_set_dreq(&tx_config, wpDREQ_UART_TX);
    dma_channel_set_config(wpTXChannel, &tx_config, false);
    dma_channel_set_write_addr(wpTXChannel, &wp_UART_HW->dr, false);
}
void wp_UsartDataReceived(void)
{
    static size_t old_position;
    // Calculate current position into buffer and check if new data is available
    int length_data_received = dma_channel_hw_addr(wpRXChannel)->transfer_count;
    size_t position = ARRAY_LEN(wpUSART_DMA_Receive_Buffer) - length_data_received;
    if (position != old_position)
    {
        if (position > old_position)
        {
            wp_WriteBuffer(
                &wp_UsartReceiveCircularBuffer,
                &wpUSART_DMA_Receive_Buffer[old_position],
                position - old_position); /* Write data to receive buffer */
        }
        else
        {
            // Processing is done in "overflow" mode..
            wp_WriteBuffer(
                &wp_UsartReceiveCircularBuffer,
                &wpUSART_DMA_Receive_Buffer[old_position],
                ARRAY_LEN(wpUSART_DMA_Receive_Buffer) - old_position); /* Write data to receive buffer */

            if (position > 0)
            {
                wp_WriteBuffer(
                    &wp_UsartReceiveCircularBuffer,
                    &wpUSART_DMA_Receive_Buffer[0],
                    position); /* Write data to receive buffer */
            }
        }
        old_position = position;
        tx_event_flags_set(&wpUartReceivedBytesEvent, 0x1, TX_OR);
    }
}
uint8_t wp_UsartStartTxDmaTransfer(void)
{
  irq_set_enabled(wpTXChannel, false);
    uint8_t started = 0;

    size_t length;
    /* Use temporary values in case they are changed during operations */
    size_t w = wp_UsartTransmitCircularBuffer.w;
    size_t r = wp_UsartTransmitCircularBuffer.r;
    if (w > r)
    {
        length = w - r;
    }
    else if (r > w)
    {
        length = wp_UsartTransmitCircularBuffer.size - r;
    }
    else
    {
        length = 0;
    }

    if (usart_tx_dma_current_len == 0 && length > 0)
    {
        // Wait for any previous DMA to finish
        while (dma_channel_is_busy(wpTXChannel))
            ;
        dma_channel_transfer_from_buffer_now(
            wpTXChannel,
            0,
            (uint32_t)&wp_UsartTransmitCircularBuffer.buffer[wp_UsartTransmitCircularBuffer.r]);
        started = 1;
    }
    irq_set_enabled(wpTXChannel, true);

    return started;
}
bool wp_WriteToUsartBuffer(uint8_t *ptr, uint16_t size)
{
    wp_WriteBuffer(&wp_UsartTransmitCircularBuffer, ptr, size); // Write data to transmit buffer

    size_t number_bytes_waiting = wp_BufferBytesWaiting(&wp_UsartTransmitCircularBuffer);
    wp_UsartTransmitCircularBuffer.r += BUF_MIN(usart_tx_dma_current_len, number_bytes_waiting);
    if (wp_UsartTransmitCircularBuffer.r >= wp_UsartTransmitCircularBuffer.size)
    {
        wp_UsartTransmitCircularBuffer.r -= wp_UsartTransmitCircularBuffer.size;
    }
    usart_tx_dma_current_len = 0; // Clear length variable
    wp_UsartStartTxDmaTransfer(); // Start sending more data

    return true;
}
int wp_ReadFromUsartBuffer(uint8_t **ptr, uint32_t *size, uint32_t wait_time)
{
    ULONG actual_flags;
    uint32_t requestedSize = *size;
    tx_event_flags_get(&wpUartReceivedBytesEvent, 0x1, TX_OR_CLEAR, &actual_flags, wait_time);
    ULONG read = wp_ReadBuffer(
        &wp_UsartReceiveCircularBuffer,
        *ptr,
        requestedSize); // Bytes have arrived try to read what was requested
    return read;
}
uint8_t wp_InitializeBuffer(CircularBuffer_t *buffer, void *data, size_t size)
{
    if (buffer == NULL || data == NULL || size == 0)
    {
        return 0;
    }
    memset((void *)buffer, 0x00, sizeof(*buffer));
    buffer->size = size;
    buffer->buffer = data;
    memset(data, 0x00, size);
    return 1;
}
size_t wp_WriteBuffer(CircularBuffer_t *buffer, const void *data, size_t btw)
{
    size_t tocopy;
    size_t free;
    size_t size;
    const uint8_t *d = data;

    if (data == NULL || btw == 0)
    {
        return 0;
    }

    /* Calculate maximum number of bytes available to write */
    /* Use temporary values in case they are changed during operations */
    size_t w = buffer->w;
    size_t r = buffer->r;
    if (w == r)
    {
        size = buffer->size;
    }
    else if (r > w)
    {
        size = r - w;
    }
    else
    {
        size = buffer->size - (w - r);
    }
    /* buffer free size is always 1 less than actual size */
    free = size - 1;

    btw = BUF_MIN(free, btw);
    if (btw == 0)
    {
        return 0;
    }

    /* Step 1: Write data to linear part of buffer */
    tocopy = BUF_MIN(buffer->size - buffer->w, btw);
    memcpy(&buffer->buffer[buffer->w], d, tocopy);
    buffer->w += tocopy;
    btw -= tocopy;

    /* Step 2: Write data to beginning of buffer (overflow part) */
    if (btw > 0)
    {
        memcpy(buffer->buffer, &d[tocopy], btw);
        buffer->w = btw;
    }

    /* Step 3: Check end of buffer */
    if (buffer->w >= buffer->size)
    {
        buffer->w = 0;
    }
    return tocopy + btw;
}
size_t wp_ReadBuffer(CircularBuffer_t *buffer, void *data, size_t btr)
{
    size_t tocopy;
    size_t full;
    uint8_t *d = data;

    if (data == NULL || btr == 0)
    {
        return 0;
    }

    /* Calculate maximum number of bytes available to read */
    full = wp_BufferBytesWaiting(buffer);
    btr = BUF_MIN(full, btr);
    if (btr == 0)
    {
        return 0;
    }

    /* Step 1: Read data from linear part of buffer */
    tocopy = BUF_MIN(buffer->size - buffer->r, btr);
    memcpy(d, &buffer->buffer[buffer->r], tocopy);
    buffer->r += tocopy;
    btr -= tocopy;

    /* Step 2: Read data from beginning of buffer (overflow part) */
    if (btr > 0)
    {
        memcpy(&d[tocopy], buffer->buffer, btr);
        buffer->r = btr;
    }

    /* Step 3: Check end of buffer */
    if (buffer->r >= buffer->size)
    {
        buffer->r = 0;
    }
    return tocopy + btr;
}
size_t wp_BufferBytesWaiting(CircularBuffer_t *buffer)
{
    size_t size;

    /* Use temporary values in case they are changed during operations */
    size_t w = buffer->w;
    size_t r = buffer->r;
    if (w == r)
    {
        size = 0;
    }
    else if (w > r)
    {
        size = w - r;
    }
    else
    {
        size = buffer->size - (r - w);
    }
    return size;
}
void Dma_Receive_Interrupt(void)
{
  bool usartError = uart_get_hw(wpUSART)->rsr > 0;
      if (usartError) {
    hw_clear_bits(
        &uart_get_hw(wpUSART)->rsr,
        UART_UARTRSR_BITS); // Clear and re-initialize the receive ringbuffer
    wp_InitializeBuffer(&wp_UsartReceiveCircularBuffer, wp_ReceiveData,
                        sizeof(wp_ReceiveData));
    }
    else
    {
        // Clear the interrupt request.
        dma_hw->ints0 = 1u << wpRXChannel;
        // Queue up another read
        dma_channel_set_trans_count(wpRXChannel, sizeof(wp_ReceiveData), true);
        // Add received data to the circular buffer
        wp_UsartDataReceived();
    }
}
