#include <lwrb.h>
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include <wpUSART_Communications.h>

/*
Board: STM32H7B3I-DK

   _STM32H7B3I-DK_    _STLINK_3E_________      ___PC____ 
  |   ___________ |  |_______            |    |         |
  |  |     USART  |  | USART |           |    |         |
  |  | (PA10) TX-------RX    |           |    | Virtual |
  |  |            |  |       | usb_dev_hs|<==>| Com     |
  |  |  (PA9) RX-------TX    |           |    | Port    |
  |  |____________|  |_______|           |    |         |
  |_______________|  |___________________|    |_________|
                       
*/

UART_HandleTypeDef wpUartHandle;
ReadPacketState CurrentReadPacketState;
WritePacketState CurrentWritePacketState;

// Place buffers used in DMA transfers in memory regions accessible by the DMA hardware.
uint8_t aTxBuffer[TXBUFFERSIZE]  __attribute__((section(".DMA2_RAM"))) __attribute__((aligned(4)));
uint8_t aRxBuffer[RXBUFFER] __attribute__((section(".DMA2_RAM"))) __attribute__((aligned(4)));


static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;

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



bool InitWireProtocolCommunications()
{
        
    // Select SysClk as source of USART1 clocks
    // TODO: Check what these 3 code line do and if correct
    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit = { 0 };
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    
    // Enable the clock for the selected USART and DMA
    
    wpUSART_CLK_ENABLE();                     // SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN)     - __HAL_RCC_USART1_CLK_ENABLE()
    wpUSART_GPIO_CLK_ENABLE();                // SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);     - __HAL_RCC_GPIOA_CLK_ENABLE()
    wpDMA_CLK_ENABLE();                       // SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN)       - DMA2
        
    // UART TX/RX GPIO pin configuration and clock
    GPIO_InitStruct.Pin = wpUSART_TX_PIN | wpUSART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = wpUSART_AF;
    HAL_GPIO_Init(wpUSART_GPIO_PORT, &GPIO_InitStruct);

    
    // Receive DMA configuration
    hdma_rx.Instance = wpUSART_RX_DMA_STREAM;
    hdma_rx.Init.Request = wpUSART_RX_DMA_REQUEST;
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode = DMA_CIRCULAR;
    hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_rx))
    {
        HAL_AssertEx();
    }
    __HAL_LINKDMA(&wpUartHandle, hdmarx, hdma_rx);           // Associate the DMA handle to the the USART

    // Transmit DMA configuration
    hdma_tx.Instance = wpUSART_TX_DMA_STREAM;
    hdma_tx.Init.Request = wpUSART_TX_DMA_REQUEST;
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tx))
    {
        HAL_AssertEx();
    }
    __HAL_LINKDMA(&wpUartHandle, hdmatx, hdma_tx);           // Associate the DMA handle to the the USART

    //NVIC configuration for DMA transfer complete interrupt (USART1_RX)
    HAL_NVIC_SetPriority(wpUSART_DMA_RX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(wpUSART_DMA_RX_IRQn);

    // NVIC configuration for DMA transfer complete interrupt (USART1_TX)
    HAL_NVIC_SetPriority(wpUSART_DMA_TX_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(wpUSART_DMA_TX_IRQn);

    // NVIC for USART interrupt
    HAL_NVIC_SetPriority(wpUSART_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(wpUSART_IRQn);

    wpUartHandle.Instance = wpUSART;
    wpUartHandle.Init.BaudRate = wpBAUD_RATE;
    wpUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    wpUartHandle.Init.StopBits = UART_STOPBITS_1;
    wpUartHandle.Init.Parity = UART_PARITY_NONE;
    wpUartHandle.Init.Mode = UART_MODE_TX_RX;
    wpUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    wpUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    wpUartHandle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    wpUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&wpUartHandle) != HAL_OK)  // HAL_UART_INIT calls HAL_UART_MspInit(...) in stm32h7xx_hal_msp.c
        {
            HAL_AssertEx();
        }
    if (HAL_UARTEx_SetTxFifoThreshold(&wpUartHandle, UART_TXFIFO_THRESHOLD_7_8) != HAL_OK)
    {
        HAL_AssertEx();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&wpUartHandle, UART_RXFIFO_THRESHOLD_7_8) != HAL_OK)
    {
        HAL_AssertEx();
    }
    if (HAL_UARTEx_EnableFifoMode(&wpUartHandle) != HAL_OK)
    {
        HAL_AssertEx();
    }

    // Enable USART and DMA RX
    __HAL_DMA_ENABLE(&hdma_rx);
    __HAL_UART_ENABLE(&wpUartHandle);
    
    
    char * test = "1234567890";
    HAL_UART_Transmit_DMA(&wpUartHandle, (uint8_t*)test, 10);
    
    
    return true;
}




bool WritePacket(uint8_t* ptr, uint16_t size)
{
    memcpy(aTxBuffer, ptr, size);

    if (HAL_UART_Transmit_DMA(&wpUartHandle, (uint8_t*)aTxBuffer, size) != HAL_OK)
    {
        HAL_AssertEx();
    }
}
bool ReadNextPacket(uint8_t** ptr, uint32_t* size)
{
    
    Test();
    uint8_t * stuff = *ptr;
    if (wpUartHandle.RxState == HAL_UART_STATE_READY)
    {
        if (HAL_UART_Receive_DMA(&wpUartHandle, aRxBuffer, sizeof(aRxBuffer)) != HAL_OK)
        {
            HAL_AssertEx();
        }
    }

    for (int i = 0; i < 10; i++)
    {
        if (CircularBuffer.NumberNotProcessed > 0)
        {
            memcpy(stuff, CircularBuffer.Packets[CircularBuffer.ProcessedIndex].Data, sizeof(aRxBuffer));
            *size = 0;
            CircularBuffer.NumberNotProcessed--;
            CircularBuffer.ProcessedIndex++;
            if (CircularBuffer.ProcessedIndex >= NUMBER_PACKETS)
            {
                CircularBuffer.ProcessedIndex = 0;
            }
            return true;
        }
        else
        {
            tx_thread_sleep(25);
        }
    }
    return false;
}
void ReadNextComplete(UART_HandleTypeDef* UartHandle)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)aRxBuffer, sizeof(aRxBuffer));           // Tricky one

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


//+++++++++++++++++++++
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
uint8_t  usart_rx_dma_buffer[64];
lwrb_t  usart_rx_rb;         // Ring buffer instance for TX data
uint8_t usart_rx_rb_data[128];      // Ring buffer data array for RX DMA
lwrb_t  usart_tx_rb;          // Ring buffer instance for TX data
uint8_t usart_tx_rb_data[128];        // Ring buffer data array for TX DMA
volatile size_t usart_tx_dma_current_len;           //  Length of currently active TX DMA transfer


void WP_DMA_Receive_half_complete()
{
    __HAL_DMA_CLEAR_FLAG(&hdma_rx, wpCLEAR_HALF_TRANSFER_COMPLETE); /* Clear half-transfer complete flag */
    usart_rx_check(); /* Check for data to process */
}

void WP_DMA_Receive_complete()
{
    __HAL_DMA_CLEAR_FLAG(&hdma_rx, wpCLEAR_HALF_TRANSFER_COMPLETE); /* Clear half-transfer complete flag */
    usart_rx_check(); /* Check for data to process */
}

void WP_DMA_Transfer_complete()
{
   
    __HAL_DMA_CLEAR_FLAG(&hdma_tx, wpCLEAR_HALF_TRANSFER_COMPLETE); /* Clear half-transfer complete flag */

    lwrb_skip(&usart_tx_rb, usart_tx_dma_current_len); /* Skip sent data, mark as read */
    usart_tx_dma_current_len = 0; /* Clear length variable */
    usart_start_tx_dma_transfer(); /* Start sending more data */
}

void WP_USART_Interrupt()
{
    __HAL_USART_CLEAR_IDLEFLAG(&wpUartHandle);
    usart_rx_check(); /* Check for data to process */
}


void usart_rx_check(void) 
{
    static size_t old_pos;
    size_t pos;
    
    size_t SpaceRemaining = __HAL_DMA_GET_COUNTER(&hdma_rx);

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - SpaceRemaining;
    if (pos != old_pos) {
        /* Check change in received data */
        if (pos > old_pos) {
            /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        }
        else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos; /* Save current position as old for next transfers */
    }
}


void usart_process_data(const void* data, size_t len) 
{
    lwrb_write(&usart_rx_rb, data, len); /* Write data to receive buffer */
}


void usart_send_string(const char* str) {
    lwrb_write(&usart_tx_rb, str, strlen(str)); /* Write data to transmit buffer */
    usart_start_tx_dma_transfer();
}

uint8_t usart_start_tx_dma_transfer(void)
{
    uint32_t primask;
    uint8_t started = 0;

    /*
     * First check if transfer is currently in-active,
     * by examining the value of usart_tx_dma_current_len variable.
     *
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     *
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call,
     *         indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active
     *         and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access to this function w/o
     * exclusive access protection (mutex) configured,
	 * or if application calls this function from multiple interrupts.
	 *
	 * This example assumes worst use case scenario,
     * hence interrupts are disabled prior every check
     */
    primask = __get_PRIMASK();
    
    __disable_interrupts();
    if (usart_tx_dma_current_len == 0 && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_rb)) > 0) {
        
        /* Disable channel if enabled */

        __HAL_DMA_DISABLE(&hdma_tx);

        __HAL_DMA_CLEAR_FLAG(&hdma_tx, wpCLEAR_TRANSFER_COMPLETE);        // Clear transfer complete flag.
        __HAL_DMA_CLEAR_FLAG(&hdma_tx, wpCLEAR_HALF_TRANSFER_COMPLETE);        // Clear half transfer flag.
        __HAL_DMA_CLEAR_FLAG(&hdma_tx, wpCLEAR_TRANSFER_ERROR);        // Clear transfer error flag.
        __HAL_DMA_CLEAR_FLAG(&hdma_tx, wpCLEAR_DIRECT_MODE_ERROR);       // Clear direct mode error flag.
        __HAL_DMA_CLEAR_FLAG(&hdma_tx, wpCLEAR_FIFO_ERROR);        // Clear FIFO error flag.

        
    if(HAL_UART_Transmit_DMA(&wpUartHandle, (uint8_t *)lwrb_get_linear_block_read_address(&usart_tx_rb), usart_tx_dma_current_len) != HAL_OK)
        {
            
        }
    }
    __enable_interrupts();
    return started;
}


void Test()
{
    uint8_t state, cmd, len;
    
    
    
    char* Entry = "I have entered the test routine\n\r";
    
    HAL_UART_Transmit_DMA(&wpUartHandle, (uint8_t*)Entry, 34);

        

    while (1) {
        uint8_t b;

        /* Process RX ringbuffer */

        /* Packet format: START_BYTE, CMD, LEN[, DATA[0], DATA[len - 1]], STOP BYTE */
        /* DATA bytes are included only if LEN > 0 */
        /* An example, send sequence of these bytes: 0x55, 0x01, 0x01, 0xFF, 0xAA */

        /* Read byte by byte */

        if (lwrb_read(&usart_rx_rb, &b, 1) == 1) {
            lwrb_write(&usart_tx_rb, &b, 1); /* Write data to transmit buffer */
            usart_start_tx_dma_transfer();
            switch (state) {
            case 0: {
                    /* Wait for start byte */
                    if (b == 0x55) {
                        ++state;
                    }
                    break;
                }
            case 1: {
                    /* Check packet command */
                    cmd = b;
                    ++state;
                    break;
                }
            case 2: {
                    /* Packet data length */
                    len = b;
                    ++state;
                    if (len == 0) {
                        ++state; /* Ignore data part if len = 0 */
                    }
                    break;
                }
            case 3: {
                    /* Data for command */
                    --len; /* Decrease for received character */
                    if (len == 0) {
                        ++state;
                    }
                    break;
                }
            case 4: {
                    /* End of packet */
                    if (b == 0xAA) {
                        /* Packet is valid */

                        /* Send out response with CMD = 0xFF */
                        b = 0x55; /* Start byte */
                        lwrb_write(&usart_tx_rb, &b, 1);
                        cmd = 0xFF; /* Command = 0xFF = OK response */
                        lwrb_write(&usart_tx_rb, &cmd, 1);
                        b = 0x00; /* Len = 0 */
                        lwrb_write(&usart_tx_rb, &b, 1);
                        b = 0xAA; /* Stop byte */
                        lwrb_write(&usart_tx_rb, &b, 1);

                        /* Flush everything */
                        usart_start_tx_dma_transfer();
                    }
                    state = 0;
                    break;
                }
            }
        }

        /* Do other tasks ... */
    }
    
}
