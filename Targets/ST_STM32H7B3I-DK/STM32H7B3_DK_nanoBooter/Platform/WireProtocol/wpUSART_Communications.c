//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include <nanoHAL_v2.h>
#include <wpUSART_Communications.h>
#include "WireProtocol_Message.h"
#include <stm32h7xx.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_uart.h>
#include <BoardInit.h>

/*
Board: STM32H7B3I-DK

   _STM32H7B3I-DK_   _STLINK_3E_________      __PC___
  |   ____________|  |______            |    |       |
  |  |     USART  |  |USART |           |    |       |
  |  | (PA10) TX  |__|RX    |           |    |Virtual|
  |  |            |  |      | usb_dev_hs|<==>|Com    |
  |  |  (PA9) RX  |__|TX    |           |    |Port   |
  |  |            |  |      |           |    |_______|
  |  |       GND  |__|GND   |           |
  |  |____________|  |______|           |
  |_______________|  |__________________|

*/

UART_HandleTypeDef wpUartHandle;
ReadPacketState CurrentReadPacketState;
WritePacketState CurrentWritePacketState;
__attribute__((aligned(32))) uint8_t aTxBuffer[TXBUFFERSIZE];
__attribute__((aligned(32))) uint8_t aRxBuffer[RXBUFFER];
GPIO_InitTypeDef  GPIO_InitStruct;
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

    //-------------------------------------
    // Setup the USART - Clocks, Pins, Mode
    //-------------------------------------

    // Select SysClk as source of USART1 clocks
    // TODO: Check what these 3 code line do and if correct
    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit = { 0 };
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    // Enable the clock for the selected USART
    wpUSART_CLK_ENABLE();

    // NOTE: The following setup allows of USART RX/TX pins on different ports
    // Samples quite often ( OR pins) when pins on same port
    // UART TX GPIO pin configuration and clock
    wpUSART_TX_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = wpUSART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = wpUSART_TX_AF;
    HAL_GPIO_Init(wpUSART_TX_GPIO_PORT, &GPIO_InitStruct);

    // UART RX GPIO pin configuration and clock
    wpUSART_RX_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = wpUSART_RX_PIN;
    GPIO_InitStruct.Alternate = wpUSART_RX_AF;
    HAL_GPIO_Init(wpUSART_RX_GPIO_PORT, &GPIO_InitStruct);

    // Use USART as defined in the header file
    wpUartHandle.Instance = wpUSART;
    wpUartHandle.Init.BaudRate = wpBAUD_RATE;
    wpUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    wpUartHandle.Init.StopBits = UART_STOPBITS_1;
    wpUartHandle.Init.Parity = UART_PARITY_NONE;
    wpUartHandle.Init.Mode = UART_MODE_TX_RX;
    wpUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    wpUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    wpUartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    wpUartHandle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    wpUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    //----------------------------------------------
    // Setup the DMA to support USART communications
    //----------------------------------------------

    // Setup the DMA to use peripheral to memory mode for the USART
    DMAwp_CLK_ENABLE();

    // Receive DMA configuration
    hdma_rx.Instance = wpUSART_RX_DMA_STREAM;
    hdma_rx.Init.Request = wpUSART_RX_DMA_REQUEST;
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode = DMA_NORMAL;
    hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_rx))
    {
        HAL_AssertEx();
    }
    __HAL_LINKDMA(&wpUartHandle, hdmarx, hdma_rx); // Associate the DMA handle to the the USART
    //++++++++++++++++++++++++++++++++++++++++++++

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
    __HAL_LINKDMA(&wpUartHandle, hdmatx, hdma_tx); // Associate the DMA handle to the the USART
    //++++++++++++++++++++++++++++++++++++++++++++

    //------------------------------------------
    // Setup NVIC configuration for DMA transfer
    //------------------------------------------

    //NVIC configuration for DMA transfer complete interrupt (USART1_RX)
    HAL_NVIC_SetPriority(wpUSART_DMA_RX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(wpUSART_DMA_RX_IRQn);
    //+++++++++++++++++++++++++++++++++++++++++

    // NVIC configuration for DMA transfer complete interrupt (USART1_TX)
    HAL_NVIC_SetPriority(wpUSART_DMA_TX_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(wpUSART_DMA_TX_IRQn);
    //+++++++++++++++++++++++++++++++++++++++++

    // NVIC for USART interrupt
    HAL_NVIC_SetPriority(wpUSART_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(wpUSART_IRQn);

    if (HAL_UART_Init(&wpUartHandle) != HAL_OK)
    {
        HAL_AssertEx();
    }

    // ?????????????????
    if (HAL_UARTEx_SetTxFifoThreshold(&wpUartHandle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        HAL_AssertEx();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&wpUartHandle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        HAL_AssertEx();
    }
    if (HAL_UARTEx_DisableFifoMode(&wpUartHandle) != HAL_OK)
    {
        HAL_AssertEx();
    }


    // ?????????????????????
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
bool ReadNextPacket(uint8_t* ptr, uint16_t* size)
{
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
            memcpy(ptr, CircularBuffer.Packets[CircularBuffer.ProcessedIndex].Data, sizeof(aRxBuffer));
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
    SCB_InvalidateDCache_by_Addr(aRxBuffer, sizeof(aRxBuffer)); // Tricky one

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

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    int counter = 0;
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        switch (huart->ErrorCode)
        {
        case  HAL_UART_ERROR_NONE:
            CircularBuffer.CommsErrors.None++;
            break;
        case  HAL_UART_ERROR_PE:
            CircularBuffer.CommsErrors.Parity_error++;
            break;
        case  HAL_UART_ERROR_NE:
            CircularBuffer.CommsErrors.Noise_error++;
            break;
        case  HAL_UART_ERROR_FE:
            CircularBuffer.CommsErrors.Frame_error++;
            break;
        case  HAL_UART_ERROR_ORE:
            CircularBuffer.CommsErrors.Overrun_error++;
            break;
        case  HAL_UART_ERROR_DMA:
            CircularBuffer.CommsErrors.DMA_transfer_error++;
            break;
        case  HAL_UART_ERROR_RTO:
            CircularBuffer.CommsErrors.Receiver_Timeout_error++;
            break;
        }
    }
}
