//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include <lwrb.h>
#include "stm32h7xx.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_utils.h"
#include "BoardInit.h"

#include "wpUSART_Communications.h"

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

UART_HandleTypeDef wpUartHandle;
ReadPacketState CurrentReadPacketState;
WritePacketState CurrentWritePacketState;

// Place buffers used in DMA transfers in memory regions accessible by the DMA hardware.
uint8_t aTxBuffer[TXBUFFERSIZE]  __attribute__((section(".DMA2_RAM"))) __attribute__((aligned(4)));
uint8_t aRxBuffer[RXBUFFER] __attribute__((section(".DMA2_RAM"))) __attribute__((aligned(4)));

static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;
uint8_t  usart_rx_dma_buffer[2000];                                 // USART RX buffer for DMA to transfer every received byte RX (D1 RAM, accessible by DMA)
lwrb_t  usart_rx_rb;                                              // Ring buffer instance for TX data
lwrb_t  usart_tx_rb;                                              // Ring buffer instance for TX data
uint8_t usart_rx_rb_data[128];                                    // Ring buffer data array for RX DMA
uint8_t usart_tx_rb_data[128];                                    // Ring buffer data array for TX DMA
volatile size_t usart_tx_dma_current_len;                         //  Length of currently active TX DMA transfer

void  usart_init(void);

TX_EVENT_FLAGS_GROUP wpUartReceivedBytesEvent;

struct PacketsReceived
{
    long CurrentIndex;
    long ProcessedIndex;
    long NumberNotProcessed;
    long TotalPackets;
    struct {
        int None;
        int Parity_error;
        int Noise_error;
        int Frame_error;
        int Overrun_error;
        int DMA_transfer_error;
        int Receiver_Timeout_error;
    } CommsErrors;
    struct {
        uint32_t Sequence;
        bool Processed;
        ReadPacketState State;
        uint8_t Data[RXBUFFER];
    } Packets[NUMBER_PACKETS];
} CircularBuffer;
void usart_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = { 0 };
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    wpUSART_PERIPHERAL_CLOCK_ENABLE;
    wpUSART_GPIO_PERIPHERAL_CLOCK_ENABLE;
    wpUSART_DMA_PERIPHERAL_CLOCK_ENABLE;

    GPIO_InitStruct.Pin = wpUSART_TX_PIN | wpUSART_RX_PIN;                                        // UART TX/RX GPIO pin configuration and clock
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // Configure WireProtocol wpUSART
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = wpBAUD_RATE;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    
    LL_USART_Disable(wpUSART);                   // Required to make changes, LL_USART_Init doesn't work unless disabled
    LL_USART_Init(wpUSART, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(wpUSART, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_SetRXFIFOThreshold(wpUSART, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_EnableFIFO(wpUSART);
    LL_USART_ConfigAsyncMode(wpUSART);
    LL_USART_EnableDMAReq_RX(wpUSART);
    LL_USART_EnableDMAReq_TX(wpUSART);
    LL_USART_EnableIT_IDLE(wpUSART);

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
    LL_DMA_SetPeriphAddress(wpDMA, wpDMA_ReceiveStream, LL_USART_DMA_GetRegAddr(wpUSART, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(wpDMA, wpDMA_ReceiveStream, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(wpDMA, wpDMA_ReceiveStream, ARRAY_LEN(usart_rx_dma_buffer));

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
    LL_DMA_SetPeriphAddress(wpDMA, wpDMA_TransmitStream, LL_USART_DMA_GetRegAddr(wpUSART, LL_USART_DMA_REG_DATA_TRANSMIT));

    LL_DMA_EnableIT_HT(wpDMA, wpDMA_ReceiveStream);                       // Enable receive half transfer interrupt
    LL_DMA_EnableIT_TC(wpDMA, wpDMA_ReceiveStream);                       // Enable receive transfer complete interrupt
    LL_DMA_EnableIT_TC(wpDMA, wpDMA_TransmitStream);                      // Enable transmit transfer complete interrupt

    // DMA interrupt initialize
    NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    // USART interrupt, same priority as DMA channel
    NVIC_SetPriority(wpUSART_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(wpUSART_IRQn);

    LL_DMA_EnableStream(wpDMA, wpDMA_ReceiveStream);                             // Enable DMA receive stream
    LL_USART_Enable(wpUSART);                                                    // Enable the Usart

    // Polling wpUSART initialization
    while(!LL_USART_IsActiveFlag_TEACK(wpUSART) || !LL_USART_IsActiveFlag_REACK(wpUSART)) {}
    
    // Create event based data synchronization
    tx_event_flags_create(&wpUartReceivedBytesEvent, "wpUart event");
}
bool InitWireProtocolCommunications()
{
    lwrb_init(&usart_tx_rb, usart_tx_rb_data, sizeof(usart_tx_rb_data));         // Initialize ringbuff for TX
    lwrb_init(&usart_rx_rb, usart_rx_rb_data, sizeof(usart_rx_rb_data));         // Initialize ringbuff for RX
    usart_init();
    return true;
}
bool WritePacket(uint8_t* ptr, uint16_t size)
{
    lwrb_write(&usart_tx_rb, ptr, size);                                         // Write data to transmit buffer
    usart_start_tx_dma_transfer();
    return true;
}
bool ReadNextPacket(uint8_t** ptr, uint32_t* size)
{
    ULONG   actual_flags;
    uint32_t requestedSize = *size;

    tx_event_flags_get(&wpUartReceivedBytesEvent, 0x1, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
    ULONG read = lwrb_read(&usart_rx_rb, *ptr, requestedSize);         // Bytes have arrived try to read what was requested
    
        ASSERT(read <= requestedSize);
    
    *size -= read;                                                     // actual amount read could be less than requested
    *ptr += read;
    
    return true;
}
void ReadNextComplete(UART_HandleTypeDef* UartHandle)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)aRxBuffer, sizeof(aRxBuffer));                                                     // Tricky one

    CircularBuffer.TotalPackets++;
    CircularBuffer.NumberNotProcessed++;
    CircularBuffer.Packets[CircularBuffer.CurrentIndex].State = ReadFull;
    CircularBuffer.Packets[CircularBuffer.CurrentIndex].Processed = false;
    CircularBuffer.Packets[CircularBuffer.CurrentIndex].Sequence = CircularBuffer.TotalPackets;

    memcpy(CircularBuffer.Packets[CircularBuffer.CurrentIndex].Data, aRxBuffer, sizeof(aRxBuffer));

    CircularBuffer.CurrentIndex++;
    if (CircularBuffer.CurrentIndex >= NUMBER_PACKETS)
    {
        CircularBuffer.CurrentIndex = 0;
    }

}
void DMA1_Stream0_IRQHandler(void) 
{
    if (LL_DMA_IsActiveFlag_HT0(wpDMA))              // Check half-transfer complete interrupt
        {
            LL_DMA_ClearFlag_HT0(wpDMA);                                 // Clear half-transfer complete flag
            usart_rx_check();                                            // Check for data to process
        }
    if (LL_DMA_IsActiveFlag_TC0(wpDMA))              // Check transfer-complete interrupt
        {
            LL_DMA_ClearFlag_TC0(wpDMA);                                 // Clear transfer complete flag
            usart_rx_check();                                            // Check for data to process
        }
}
void DMA1_Stream1_IRQHandler(void)    
{
    if (LL_DMA_IsActiveFlag_TC1(wpDMA))                         // Check transfer complete
        {
            LL_DMA_ClearFlag_TC1(wpDMA);                                            // Clear transfer complete flag
            lwrb_skip(&usart_tx_rb, usart_tx_dma_current_len);                      // Skip sent data, mark as read
            usart_tx_dma_current_len = 0;                                           // Clear length variable
            usart_start_tx_dma_transfer();                                          // Start sending more data
        }
}
void USART3_IRQHandler(void) 
{
    
    if (LL_USART_IsActiveFlag_IDLE(wpUSART))                  // Check for IDLE line interrupt
        {
            LL_USART_ClearFlag_IDLE(wpUSART);                                     // Clear IDLE line flag
            usart_rx_check();                                                     // Check for data to process
        }

    /* Implement other events when needed */
}
void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    int length_data_received = LL_DMA_GetDataLength(wpDMA, wpDMA_ReceiveStream);
    pos = ARRAY_LEN(usart_rx_dma_buffer) - length_data_received;
    if (pos != old_pos) {
        if (pos > old_pos)
        {
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        }
        else 
        {
            // Processing is done in "overflow" mode..
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            if (pos > 0) 
            {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos;
    }
}
void usart_process_data(const void* data, size_t len) 
{
    lwrb_write(&usart_rx_rb, data, len); /* Write data to receive buffer */
    tx_event_flags_set(&wpUartReceivedBytesEvent, 0x1, TX_OR);

}
void usart_send_string(const char* str) {
    lwrb_write(&usart_tx_rb, str, strlen(str)); /* Write data to transmit buffer */
    usart_start_tx_dma_transfer();
}
uint8_t usart_start_tx_dma_transfer(void)
{
    /*
     * First check if transfer is currently in-active, by examining the value of usart_tx_dma_current_len variable.
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access this function w/o exclusive access protection (mutex) configured,
	 * or if application calls this function from multiple interrupts.
	 *
	 * This example assumes worst use case scenario, hence interrupts are disabled prior every check
     */
    uint8_t started = 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (usart_tx_dma_current_len == 0
            && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_rb)) > 0) 
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
        LL_DMA_SetMemoryAddress(wpDMA, wpDMA_TransmitStream, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_rb));

        // Start transfer
        LL_DMA_EnableStream(wpDMA, wpDMA_TransmitStream);
        started = 1;
    }
    __set_PRIMASK(primask);
    return started;
}

