//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include "BoardInit.h"
#include "stm32h7xx.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_utils.h"
//#include <lwrb.h>

#include "wpUSART_Communications.h"


UART_HandleTypeDef wpUartHandle;

static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;

// USART receive buffer for DMA - make sure it is in RAM accessible by the DMA controller used.
// Also, check alignment
__attribute__((section(".DMA1_RAM"))) __attribute__((aligned(32))) uint8_t wpUSART_DMA_Receive_Buffer[wpUSART_DMA_Receive_Buffer_size];

circular_buffer_t receive_circular_buffer;  // Circular buffer instance for Transmit data
uint8_t receive_circular_buffer_data[1024]; // Circular buffer data array for Receive DMA

circular_buffer_t transmit_circular_buffer;  // Circular buffer instance for Transmit data
uint8_t transmit_circular_buffer_data[1024]; // Circular buffer data array for Transmit DMA

volatile size_t usart_tx_dma_current_len; //  Length of currently active TX DMA transfer

TX_EVENT_FLAGS_GROUP wpUartReceivedBytesEvent;

void initialize_usart(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    wpUSART_PERIPHERAL_CLOCK_ENABLE;
    wpUSART_GPIO_PERIPHERAL_CLOCK_ENABLE;
    wpUSART_DMA_PERIPHERAL_CLOCK_ENABLE;

    GPIO_InitStruct.Pin = wpUSART_TX_PIN | wpUSART_RX_PIN; // UART TX/RX GPIO pin configuration and clock
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // wpUSART Receive Initialize
    LL_DMA_SetPeriphRequest(wpDMA, wpDMA_ReceiveStream, wpDMA_ReceiveMux);
    LL_DMA_SetDataTransferDirection(wpDMA, wpDMA_ReceiveStream, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(wpDMA, wpDMA_ReceiveStream, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(wpDMA, wpDMA_ReceiveStream, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(wpDMA, wpDMA_ReceiveStream, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(wpDMA, wpDMA_ReceiveStream, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetMemorySize(wpDMA, wpDMA_ReceiveStream, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphSize(wpDMA, wpDMA_ReceiveStream, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(wpDMA, wpDMA_ReceiveStream);
    LL_DMA_SetPeriphAddress(
        wpDMA,
        wpDMA_ReceiveStream,
        LL_USART_DMA_GetRegAddr(wpUSART, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(wpDMA, wpDMA_ReceiveStream, (uint32_t)wpUSART_DMA_Receive_Buffer);
    LL_DMA_SetDataLength(wpDMA, wpDMA_ReceiveStream, ARRAY_LEN(wpUSART_DMA_Receive_Buffer));

    // wpUSART Transmit Initialize
    LL_DMA_SetPeriphRequest(wpDMA, wpDMA_TransmitStream, wpDMA_TransmitMux);
    LL_DMA_SetDataTransferDirection(wpDMA, wpDMA_TransmitStream, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(wpDMA, wpDMA_TransmitStream, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(wpDMA, wpDMA_TransmitStream, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(wpDMA, wpDMA_TransmitStream, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(wpDMA, wpDMA_TransmitStream, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(wpDMA, wpDMA_TransmitStream, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(wpDMA, wpDMA_TransmitStream, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(wpDMA, wpDMA_TransmitStream);
    LL_DMA_SetPeriphAddress(
        wpDMA,
        wpDMA_TransmitStream,
        LL_USART_DMA_GetRegAddr(wpUSART, LL_USART_DMA_REG_DATA_TRANSMIT));

    LL_DMA_EnableIT_HT(wpDMA, wpDMA_ReceiveStream);  // Enable receive half transfer interrupt
    LL_DMA_EnableIT_TC(wpDMA, wpDMA_ReceiveStream);  // Enable receive transfer complete interrupt
    LL_DMA_EnableIT_TC(wpDMA, wpDMA_TransmitStream); // Enable transmit transfer complete interrupt

    // DMA interrupt initialize
    NVIC_SetPriority(wpDMA_ReceiveStreamInterrupt, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(wpDMA_ReceiveStreamInterrupt);
    NVIC_SetPriority(wpDMA_TransmitStreamInterrupt, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(wpDMA_TransmitStreamInterrupt);

    // Configure WireProtocol wpUSART
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = wpBAUD_RATE;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

    LL_USART_Disable(wpUSART); // Required to make changes, LL_USART_Init doesn't work unless disabled
    LL_USART_Init(wpUSART, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(wpUSART, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_SetRXFIFOThreshold(wpUSART, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_EnableFIFO(wpUSART);
    LL_USART_ConfigAsyncMode(wpUSART);
    LL_USART_EnableDMAReq_RX(wpUSART);
    LL_USART_EnableDMAReq_TX(wpUSART);
    LL_USART_EnableIT_IDLE(wpUSART);

    // USART interrupt, same priority as DMA channel
    NVIC_SetPriority(wpUSART_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(wpUSART_IRQn);

    LL_DMA_EnableStream(wpDMA, wpDMA_ReceiveStream); // Enable DMA receive stream
    LL_USART_Enable(wpUSART);                        // Enable the Usart

    // Polling wpUSART initialization
    while (!LL_USART_IsActiveFlag_TEACK(wpUSART) || !LL_USART_IsActiveFlag_REACK(wpUSART))
    {
    }

    // Create event based data synchronization
    tx_event_flags_create(&wpUartReceivedBytesEvent, "wpUart event");
}

bool InitWireProtocolCommunications()
{
    circular_buffer_initialize(
        &transmit_circular_buffer,
        transmit_circular_buffer_data,
        sizeof(transmit_circular_buffer_data)); // Initialize ringbuffer for TX
    circular_buffer_initialize(
        &receive_circular_buffer,
        receive_circular_buffer_data,
        sizeof(receive_circular_buffer_data)); // Initialize ringbuffer for RX
    initialize_usart();
    return true;
}
bool WritePacket(uint8_t *ptr, uint16_t size)
{
    circular_buffer_write(&transmit_circular_buffer, ptr, size); // Write data to transmit buffer
    usart_start_tx_dma_transfer();
    return true;
}
int ReadNextPacket(uint8_t **ptr, uint32_t *size, uint32_t wait_time)
{
    ULONG actual_flags;
    uint32_t requestedSize = *size;
    tx_event_flags_get(&wpUartReceivedBytesEvent, 0x1, TX_OR_CLEAR, &actual_flags, wait_time);
    ULONG read =
        circular_buffer_read(&receive_circular_buffer, *ptr, requestedSize); // Bytes have arrived try to read what was requested
    return read;
}
void DMA1_Stream0_IRQHandler(void)
{
    // Check half-transfer complete interrupt
    if (LL_DMA_IsEnabledIT_HT(wpDMA, wpDMA_ReceiveStream) && LL_DMA_IsActiveFlag_HT0(wpDMA))
    {
        LL_DMA_ClearFlag_HT0(wpDMA); // Clear half-transfer complete flag
        usart_rx_check();            // Check for data to process
    }

    // Check transfer-complete interrupt
    if (LL_DMA_IsEnabledIT_TC(wpDMA, wpDMA_ReceiveStream) && LL_DMA_IsActiveFlag_TC0(wpDMA))
    {
        LL_DMA_ClearFlag_TC0(wpDMA); // Clear transfer complete flag
        usart_rx_check();            // Check for data to process
    }
}
void DMA1_Stream1_IRQHandler(void)
{
    if (LL_DMA_IsEnabledIT_TC(wpDMA, wpDMA_TransmitStream) && LL_DMA_IsActiveFlag_TC1(wpDMA))
    {
        LL_DMA_ClearFlag_TC1(wpDMA); // Clear transfer complete flag
        size_t number_bytes_waiting = circular_buffer_bytes_waiting(&transmit_circular_buffer);
        transmit_circular_buffer.r +=
            BUF_MIN(usart_tx_dma_current_len, number_bytes_waiting);
        if (transmit_circular_buffer.r >= transmit_circular_buffer.size)
        {
            transmit_circular_buffer.r -= transmit_circular_buffer.size;
        }
//        lwrb_skip(&usart_tx_rb, usart_tx_dma_current_len); // Skip sent data, mark as read
        usart_tx_dma_current_len = 0;                      // Clear length variable
        usart_start_tx_dma_transfer();                     // Start sending more data
    }
}
#define wpUSART_IRQHANDLER() void USART3_IRQHandler(void)
wpUSART_IRQHANDLER()
// void USART3_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_IDLE(wpUSART)) // Check for IDLE line interrupt
    {
        LL_USART_ClearFlag_IDLE(wpUSART); // Clear IDLE line flag
        usart_rx_check();                 // Check for data to process
    }
}
void usart_rx_check(void)
{
    static size_t old_position;

    /* Calculate current position in buffer and check for new data available */
    int length_data_received = LL_DMA_GetDataLength(wpDMA, wpDMA_ReceiveStream);
    size_t position = ARRAY_LEN(wpUSART_DMA_Receive_Buffer) - length_data_received;

    if (position != old_position)
    {
        if (position > old_position)
        {
            usart_process_data(&wpUSART_DMA_Receive_Buffer[old_position], position - old_position);
        }
        else
        {
            // Processing is done in "overflow" mode..
            usart_process_data(&wpUSART_DMA_Receive_Buffer[old_position], ARRAY_LEN(wpUSART_DMA_Receive_Buffer) - old_position);
            if (position > 0)
            {
                usart_process_data(&wpUSART_DMA_Receive_Buffer[0], position);
            }
        }
        old_position = position;
    }
}
void usart_process_data(const void *data, size_t len)
{
    circular_buffer_write(&receive_circular_buffer, data, len); /* Write data to receive buffer */
    tx_event_flags_set(&wpUartReceivedBytesEvent, 0x1, TX_OR);
}
uint8_t usart_start_tx_dma_transfer(void)
{
    /*
     * First check if transfer is currently in-active, by examining the value of
     * usart_tx_dma_current_len variable. This variable is set before DMA transfer
     * is started and cleared in DMA TX complete interrupt. It is not necessary to
     * disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call
     * indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already
     * in-active and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len
     * != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access this function w/o
     * exclusive access protection (mutex) configured, or if application calls
     * this function from multiple interrupts.
     *
     * This example assumes worst use case scenario, hence interrupts are disabled
     * prior every check
     */
    uint8_t started = 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (usart_tx_dma_current_len == 0 &&
        (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&transmit_circular_buffer)) > 0)
    {
        // Disable channel if enabled
        LL_DMA_DisableStream(wpDMA, wpDMA_TransmitStream);

        // Clear all flags
        LL_DMA_ClearFlag_TC1(wpDMA);
        LL_DMA_ClearFlag_HT1(wpDMA);
        LL_DMA_ClearFlag_TE1(wpDMA);
        LL_DMA_ClearFlag_DME1(wpDMA);
        LL_DMA_ClearFlag_FE1(wpDMA);

        // Prepare DMA data and length
        LL_DMA_SetDataLength(wpDMA, wpDMA_TransmitStream, usart_tx_dma_current_len);
        LL_DMA_SetMemoryAddress(wpDMA, wpDMA_TransmitStream, (uint32_t)&transmit_circular_buffer.buffer[transmit_circular_buffer.r]);

        LL_DMA_EnableStream(wpDMA, wpDMA_TransmitStream);
        started = 1;
    }
    __set_PRIMASK(primask);
    return started;
}

uint8_t circular_buffer_initialize(circular_buffer_t *buffer, void *data, size_t size)
{
    if (buffer == NULL || data == NULL || size == 0)
    {
        return 0;
    }
    memset((void *)buffer, 0x00, sizeof(*buffer));
    buffer->size = size;
    buffer->buffer = data;
    return 1;
}
size_t circular_buffer_write(circular_buffer_t *buffer, const void *data, size_t btw)
{
    size_t tocopy;
    size_t free;
    const uint8_t *d = data;

    if (data == NULL || btw == 0)
    {
        return 0;
    }

    /* Calculate maximum number of bytes available to write */
    free = circular_buffer_bytes_available(buffer);
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
size_t circular_buffer_read(circular_buffer_t *buffer, void *data, size_t btr)
{
    size_t tocopy;
    size_t full;
    uint8_t *d = data;

    if (data == NULL || btr == 0)
    {
        return 0;
    }

    /* Calculate maximum number of bytes available to read */
    full = circular_buffer_bytes_waiting(buffer);
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
size_t circular_buffer_bytes_available(circular_buffer_t *buffer)
{
    size_t size;
    size_t w;
    size_t r;
    /* Use temporary values in case they are changed during operations */
    w = buffer->w;
    r = buffer->r;
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
    return size - 1;
}
size_t circular_buffer_bytes_waiting(circular_buffer_t *buffer)
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
size_t lwrb_get_linear_block_read_length(circular_buffer_t *buffer)
{
    size_t len;
    /* Use temporary values in case they are changed during operations */
    size_t w = buffer->w;
    size_t r = buffer->r;
    if (w > r)
    {
        len = w - r;
    }
    else if (r > w)
    {
        len = buffer->size - r;
    }
    else
    {
        len = 0;
    }
    return len;
}
//size_t lwrb_skip(circular_buffer_t *buffer, size_t len)
//{
//    size_t full = circular_buffer_bytes_waiting(buffer);
//    buffer->r += BUF_MIN(len, full);
//    if (buffer->r >= buffer->size)
//    {
//        buffer->r -= buffer->size;
//    }
//    return len;
//}
