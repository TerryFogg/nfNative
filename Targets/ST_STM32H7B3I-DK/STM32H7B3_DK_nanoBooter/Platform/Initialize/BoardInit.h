//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#pragma once

#include <nanoCLR_Headers.h>

enum nanoBooterStatusTypes { ok = 0, communications_failure = 1 };
typedef enum nanoBooterStatusTypes eBooterStatus;

void BoardInit();
void LedsAndBoardInit();
void nanoBooterStatus(uint32_t nanoBooterState);
void Startup_Rtos();
void SystemClock_Config(void);

extern eBooterStatus nanoBooterState;

// ========================
// STM32H7B3I-DK board Leds
// ========================

#define LED_GPIO_PORT  GPIOG
#define LED_BLUE       GPIO_PIN_2
#define LED_RED        GPIO_PIN_11

// ===============================
// STM32H7B3I-DK board push button
// ===============================

#define BUTTON_USER_GPIO_PORT  GPIOC
#define BUTTON_USER_PIN        GPIO_PIN_13

// ======================================
// Definition for wpUSART clock resources
// ======================================

#define wpUSART                  USART1
#define wpBAUD_RATE              921600
#define wpUSART_CLK_ENABLE()     __HAL_RCC_USART1_CLK_ENABLE()
#define wpDMA_CLK_ENABLE()       __HAL_RCC_DMA2_CLK_ENABLE()
                                
#define wpUSART_FORCE_RESET()    __HAL_RCC_USART1_FORCE_RESET()
#define wpUSART_RELEASE_RESET()  __HAL_RCC_USART1_RELEASE_RESET()

// ==========================================
// Definition for USART wire protocol receive
// ==========================================

#define wpUSART_RX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define wpUSART_RX_PIN                GPIO_PIN_9
#define wpUSART_RX_GPIO_PORT          GPIOA
#define wpUSART_RX_AF                 GPIO_AF7_USART1
#define wpUSART_RX_DMA_REQUEST        DMA_REQUEST_USART1_RX
#define wpUSART_RX_DMA_STREAM         DMA2_Stream1
#define wpUSART_DMA_RX_IRQn           DMA2_Stream1_IRQn
#define wpUSART_DMA_RX_IRQHandler     DMA2_Stream1_IRQHandler

// ===========================================
// Definition for USART wire protocol transmit
// ===========================================

#define wpUSART_TX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define wpUSART_TX_PIN                GPIO_PIN_10
#define wpUSART_TX_GPIO_PORT          GPIOA
#define wpUSART_TX_AF                 GPIO_AF7_USART1
#define wpUSART_TX_DMA_REQUEST        DMA_REQUEST_USART1_TX
#define wpUSART_TX_DMA_STREAM         DMA2_Stream7
#define wpUSART_DMA_TX_IRQn           DMA2_Stream7_IRQn
#define wpUSART_DMA_TX_IRQHandler     DMA2_Stream7_IRQHandler

// =======================================
// Definition for USART wire protocol NVIC
// =======================================

#define wpUSART_IRQn                  USART1_IRQn
#define wpUSART_IRQHandler            USART1_IRQHandler
