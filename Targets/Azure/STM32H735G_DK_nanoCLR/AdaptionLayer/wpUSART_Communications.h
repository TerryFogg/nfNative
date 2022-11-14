#pragma once
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

/* Includes ------------------------------------------------------------------*/
#include "BOARD.h"

#include <stdbool.h>
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_usart.h"


#define wpUSART_DMA_Receive_Buffer_size 2048
//----------------------------------------------------------------------------------------------
// Definition for USART wire protocol receive
//----------------------------------------------------------------------------------------------
#define wpBAUD_RATE 921600

#define wpUSART USART3
#define wpUSART_IRQn USART3_IRQn
#define wpUSART_IRQHANDLER() void USART3_IRQHandler(void)
#define wpUSART_PERIPHERAL_CLOCK_ENABLE                                        \
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)

#define wpUSART_GPIO_PORT GPIOD
#define wpUSART_RX_PIN LL_GPIO_PIN_9
#define wpUSART_TX_PIN LL_GPIO_PIN_8
#define wpUSART_GPIO_PERIPHERAL_CLOCK_ENABLE                                   \
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD)

#define wpDMA DMA1
#define wpDMA_ReceiveStreamInterrupt DMA1_Stream0_IRQn
#define wpDMA_TransmitStreamInterrupt DMA1_Stream1_IRQn
#define wpUSART_DMA_PERIPHERAL_CLOCK_ENABLE                                    \
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1)

#define wpDMA_ReceiveStream LL_DMA_STREAM_0
#define wpDMA_ReceiveMux LL_DMAMUX1_REQ_USART3_RX
#define wpDMA_ReceiveStream_IRQHandler() void DMA1_Stream0_IRQHandler(void)

#define wpDMA_TransmitStream LL_DMA_STREAM_1
#define wpDMA_TransmitMux LL_DMAMUX1_REQ_USART3_TX
#define wpDMA_TransmitStream_IRQHandler() void DMA1_Stream1_IRQHandler(void)

#define BUF_MIN(x, y) ((x) < (y) ? (x) : (y))
#define BUF_MAX(x, y) ((x) > (y) ? (x) : (y))
#define ARRAY_LEN(x)  (sizeof(x) / sizeof((x)[0]))

typedef struct CircularBuffer
{
    uint8_t *buffer;
    size_t size;
    size_t r;
    size_t w;
} CircularBuffer_t;

void InitWireProtocolCommunications();
bool wp_WriteToUsartBuffer(uint8_t *ptr, uint16_t size);
int wp_ReadFromUsartBuffer(uint8_t **ptr, uint32_t *size, uint32_t wait_time);
void wp_InitializeUsart(void);
void wp_UsartDataReceived(void);
uint8_t wp_UsartStartTxDmaTransfer(void);
uint8_t wp_InitializeBuffer(CircularBuffer_t *buff, void *buffdata, size_t size);
size_t wp_WriteBuffer(CircularBuffer_t *buff, const void *data, size_t btw);
size_t wp_ReadBuffer(CircularBuffer_t *buff, void *data, size_t btr);
size_t wp_BufferBytesWaiting(CircularBuffer_t *buff);