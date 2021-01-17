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
//__attribute__((aligned(32))) uint8_t aTxBuffer[TXBUFFERSIZE];
//__attribute__((aligned(32))) uint8_t aRxBuffer[RXBUFFER];
uint8_t aTxBuffer[TXBUFFERSIZE];
uint8_t aRxBuffer[RXBUFFER];

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

    //-------------------------------------
    // Setup the USART - Clocks, Pins, Mode
    //-------------------------------------

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

    if (HAL_UART_Init(&wpUartHandle) != HAL_OK)  // HAL_UART_INIT calls HAL_UART_MspInit(...) in stm32h7xx_hal_msp.c
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
