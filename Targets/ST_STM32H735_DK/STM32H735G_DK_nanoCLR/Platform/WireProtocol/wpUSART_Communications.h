//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#pragma once

#include <stdbool.h>

/* Includes ------------------------------------------------------------------*/
#include <nanoHAL_v2.h>
#include "stm32h7xx_hal.h"
#include "WireProtocol_Message.h"
#include <stm32h7xx.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_uart.h>
#include <stm32h7xx_hal_usart.h>
#include <stm32h7xx_hal_dma.h>
#include <BoardInit.h>

// Receive buffer for DMA copy from peripheral USART
#define RXBUFFER        32

// Transmit buffer
#define TXBUFFERSIZE    32

#define NUMBER_PACKETS  4


/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

typedef enum PacketReadStates { ReadNotSet, Listening, ReadFull, ReadPartial } ReadPacketState;
typedef enum PacketWriteStates { WriteNotSet, Writing, SentFull, SentPartial } WritePacketState;

bool InitWireProtocolCommunications();
bool WritePacket(uint8_t* ptr, uint16_t size);
bool ReadNextPacket(uint8_t** ptr, uint32_t* size);
void ReadNextComplete(UART_HandleTypeDef* UartHandle);

void WP_DMA_Receive_half_complete();
void WP_DMA_Receive_complete();
void WP_DMA_Transfer_complete();
void WP_USART_Interrupt();

void usart_rx_check(void);


void    usart_process_data(const void* data, size_t len);
void    usart_send_string(const char* str);
uint8_t usart_start_tx_dma_transfer(void);


void Test();
