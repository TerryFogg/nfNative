#pragma once
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

/* Includes ------------------------------------------------------------------*/
#include "Board_PICO.h"
#include <stdbool.h>
#include <hardware/uart.h>
#include "hardware/dma.h"
#include <hardware/irq.h>
#include "pico/stdlib.h"
#include <hardware/gpio.h>
#include <tx_api.h>
#include <assert.h>

//----------------------------------------------------------------------------------------------
// Definition for USART wire protocol receive
//----------------------------------------------------------------------------------------------
// Uart 0
#define wpUSART        uart0
#define wpDREQ_UART_TX DREQ_UART0_TX
#define wpDREQ_UART_RX DREQ_UART0_RX
#define wp_UART_HW     uart0_hw
#define wpTXChannel    0
#define wpRXChannel    1
#define kRxBuffLengthPow 5
#define kTxBuffLengthPow 5

// Pins selected for TX and TX
#define wpUSART_RX_PIN PICO_DEFAULT_UART_RX_PIN
#define wpUSART_TX_PIN PICO_DEFAULT_UART_TX_PIN

#define wpBAUD_RATE 921600

#define wpUSART_DMA_Receive_Buffer_size 2048

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
void wp_InitializeDma(void);
void dma_irq_handler(void);
void wp_UsartDataReceived(void);
uint8_t wp_UsartStartTxDmaTransfer(void);
uint8_t wp_InitializeBuffer(CircularBuffer_t *buff, void *buffdata, size_t size);
size_t wp_WriteBuffer(CircularBuffer_t *buff, const void *data, size_t btw);
size_t wp_ReadBuffer(CircularBuffer_t *buff, void *data, size_t btr);
size_t wp_BufferBytesWaiting(CircularBuffer_t *buff);
