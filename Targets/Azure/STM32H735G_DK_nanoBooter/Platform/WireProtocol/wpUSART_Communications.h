//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Includes ------------------------------------------------------------------*/
#include "WireProtocol_Message.h"
#include "stm32h7xx_hal.h"
#include <BoardInit.h>
#include <nanoHAL_v2.h>
#include <stm32h7xx.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_dma.h>
#include <stm32h7xx_hal_uart.h>
#include <stm32h7xx_hal_usart.h>

// ==========================================
// Definition for USART wire protocol receive
// ==========================================

#define wpBAUD_RATE 921600

#define wpUSART                              USART3
#define wpUSART_IRQn                         USART3_IRQn
#define wpUSART_IRQHandler                   USART3_IRQHandler
#define wpUSART_GPIO_PORT                    GPIOD
#define wpUSART_RX_PIN                       GPIO_PIN_9
#define wpUSART_TX_PIN                       GPIO_PIN_8
#define wpUSART_PERIPHERAL_CLOCK_ENABLE      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)
#define wpUSART_GPIO_PERIPHERAL_CLOCK_ENABLE LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD)

#define wpDMA                               DMA1
#define wpDMA_ReceiveStreamInterrupt        DMA1_Stream0_IRQn
#define wpDMA_TransmitStreamInterrupt       DMA1_Stream1_IRQn
#define wpUSART_DMA_PERIPHERAL_CLOCK_ENABLE LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1)
#define wpDMA_ReceiveStream                 LL_DMA_STREAM_0
#define wpDMA_TransmitStream                LL_DMA_STREAM_1
#define wpDMA_ReceiveMux                    LL_DMAMUX1_REQ_USART3_RX
#define wpDMA_TransmitMux                   LL_DMAMUX1_REQ_USART3_TX

#define wpUSART_DMA_Receive_Buffer_size     2000


#define BUF_MIN(x, y) ((x) < (y) ? (x) : (y))
#define BUF_MAX(x, y) ((x) > (y) ? (x) : (y))
#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

///* Exported macro ------------------------------------------------------------*/
//#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
///* Exported functions ------------------------------------------------------- */

bool InitWireProtocolCommunications();
bool WritePacket(uint8_t *ptr, uint16_t size);
int ReadNextPacket(uint8_t **ptr, uint32_t *size, uint32_t wait_time);
void ReadNextComplete(UART_HandleTypeDef *UartHandle);

void initialize_usart(void);
void usart_rx_check(void);
void usart_process_data(const void *data, size_t len);
uint8_t usart_start_tx_dma_transfer(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);


typedef struct circular_buffer
{
    uint8_t *buffer;
    uint8_t data[1024];
    size_t size;
    size_t r;
    size_t w;
} circular_buffer_t;

uint8_t circular_buffer_initialize(circular_buffer_t *buff, void *buffdata, size_t size);
size_t circular_buffer_write(circular_buffer_t *buff, const void *data, size_t btw);
size_t circular_buffer_read(circular_buffer_t *buff, void *data, size_t btr);
size_t circular_buffer_bytes_available(circular_buffer_t *buff);
size_t circular_buffer_bytes_waiting(circular_buffer_t *buff);
size_t lwrb_get_linear_block_read_length(circular_buffer_t *buff);
//size_t lwrb_skip(circular_buffer_t *buff, size_t len);
void *lwrb_get_linear_block_write_address(circular_buffer_t *buff);
