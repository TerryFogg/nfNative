#pragma once
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_usart.h"
#include <stdbool.h>

#include "BoardInit.h"

#define wpUSART_DMA_Receive_Buffer_size 2000

#define BUF_MIN(x, y) ((x) < (y) ? (x) : (y))
#define BUF_MAX(x, y) ((x) > (y) ? (x) : (y))
#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

typedef struct CircularBuffer {
  uint8_t *buffer;
  size_t size;
  size_t r;
  size_t w;
} CircularBuffer_t;

bool InitWireProtocolCommunications();
bool wp_WriteToUsartBuffer(uint8_t *ptr, uint16_t size);
int wp_ReadFromUsartBuffer(uint8_t **ptr, uint32_t *size, uint32_t wait_time);
void ReadNextComplete(UART_HandleTypeDef *UartHandle);
void wp_InitializeUsart(void);
void wp_UsartDataReceived(void);
uint8_t wp_UsartStartTxDmaTransfer(void);
uint8_t wp_InitializeBuffer(CircularBuffer_t *buff, void *buffdata,
                            size_t size);
size_t wp_WriteBuffer(CircularBuffer_t *buff, const void *data, size_t btw);
size_t wp_ReadBuffer(CircularBuffer_t *buff, void *data, size_t btr);
size_t wp_BufferBytesWaiting(CircularBuffer_t *buff);