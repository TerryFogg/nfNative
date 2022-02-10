#pragma once

//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>

#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h735xx.h"

enum nanoBooterStatusTypes
{
    ok = 0,
    communications_failure = 1
};
typedef enum nanoBooterStatusTypes eBooterStatus;

void BoardInit();
void Initialize_board_LEDS_and_User_Button();
bool UserButtonPressed();
void nanoBooterStatus(uint32_t nanoBooterState);
void Startup_Rtos();
void SystemClock_Config(void);

extern eBooterStatus nanoBooterState;
// ========================
// STM32H735G-DK board Leds
// ========================

#define LED_GPIO_PORT GPIOC
#define LED_GREEN     LL_GPIO_PIN_3
#define LED_RED       LL_GPIO_PIN_2

// ===============================
// STM32H7B3I-DK board push button
// ===============================

#define BUTTON_USER_GPIO_PORT GPIOC
#define BUTTON_USER_PIN       LL_GPIO_PIN_13

//----------------------------------------------------------------------------------------------
// Definition for USART wire protocol receive
//----------------------------------------------------------------------------------------------
#define wpBAUD_RATE 921600

#define wpUSART                         USART3
#define wpUSART_IRQn                    USART3_IRQn
#define wpUSART_IRQHANDLER()            void USART3_IRQHandler(void)
#define wpUSART_PERIPHERAL_CLOCK_ENABLE LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)

#define wpUSART_GPIO_PORT                    GPIOD
#define wpUSART_RX_PIN                       LL_GPIO_PIN_9
#define wpUSART_TX_PIN                       LL_GPIO_PIN_8
#define wpUSART_GPIO_PERIPHERAL_CLOCK_ENABLE LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD)

#define wpDMA                               DMA1
#define wpDMA_ReceiveStreamInterrupt        DMA1_Stream0_IRQn
#define wpDMA_TransmitStreamInterrupt       DMA1_Stream1_IRQn
#define wpUSART_DMA_PERIPHERAL_CLOCK_ENABLE LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1)

#define wpDMA_ReceiveStream              LL_DMA_STREAM_0
#define wpDMA_ReceiveMux                 LL_DMAMUX1_REQ_USART3_RX
#define wpDMA_ReceiveStream_IRQHandler() void DMA1_Stream0_IRQHandler(void)

#define wpDMA_TransmitStream              LL_DMA_STREAM_1
#define wpDMA_TransmitMux                 LL_DMAMUX1_REQ_USART3_TX
#define wpDMA_TransmitStream_IRQHandler() void DMA1_Stream1_IRQHandler(void)

//---------------------------------------------------------------------------------------------
