#pragma once

//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_uart.h>
#include <stm32h7xx_hal_usart.h>
#include <stm32h7xx_hal_dma.h>
#include "stm32h7xx_hal_tim.h"

#include <nanoHAL_v2.h>
#include <nanoCLR_Headers.h>
#include <nanoHAL_Graphics.h>

#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"
#include "usbd_dfu.h"
#include "usbd_dfu_flash.h"


enum nanoBooterStatusTypes { ok = 0, communications_failure = 1 }
;
typedef enum nanoBooterStatusTypes eBooterStatus;

void BoardInit();
void LedsAndBoardInit();
void nanoBooterStatus(uint32_t nanoBooterState);
void Startup_Rtos();
void SystemClock_Config(void);
void CPU_CACHE_Enable(void);
void Board_LED_Initialization();
void     Initialize_OPSPI_Hyperam();
void     Initialize_OPSPI_Flash();

extern eBooterStatus nanoBooterState;    
// ========================
// STM32H735G-DK board Leds
// ========================

#define LED_GPIO_PORT  GPIOC
#define LED_BLUE       GPIO_PIN_3
#define LED_RED        GPIO_PIN_2

// ===============================
// STM32H7B3I-DK board push button
// ===============================

#define BUTTON_USER_GPIO_PORT  GPIOC
#define BUTTON_USER_PIN        GPIO_PIN_13



// Definition for hyperam


#define OSPI_HYPERRAM_SIZE          24
#define OSPI_HYPERRAM_INCR_SIZE     256

/* Timing of the HyperRAM */
#define OSPI_HYPERRAM_RW_REC_TIME   3
#define OSPI_HYPERRAM_LATENCY       6

/* End address of the OSPI memory */
#define OSPI_HYPERRAM_END_ADDR      (1 << OSPI_HYPERRAM_SIZE)

/* Size of buffers */
#define BUFFERSIZE                  (COUNTOF(aTxBuffer) - 1)
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)         (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Definition for OCTOSPI2 Pins */
#define OSPI2_CS_PIN                 GPIO_PIN_12
#define OSPI2_CS_GPIO_PORT           GPIOG

#define OSPI2_CLK_PIN                GPIO_PIN_4
#define OSPI2_CLK_GPIO_PORT          GPIOF

#define OSPI2_D0_PIN                 GPIO_PIN_0
#define OSPI2_D0_GPIO_PORT           GPIOF

#define OSPI2_D1_PIN                 GPIO_PIN_1
#define OSPI2_D1_GPIO_PORT           GPIOF

#define OSPI2_D2_PIN                 GPIO_PIN_2
#define OSPI2_D2_GPIO_PORT           GPIOF

#define OSPI2_D3_PIN                 GPIO_PIN_3
#define OSPI2_D3_GPIO_PORT           GPIOF

#define OSPI2_D4_PIN                 GPIO_PIN_0
#define OSPI2_D4_GPIO_PORT           GPIOG

#define OSPI2_D5_PIN                 GPIO_PIN_1
#define OSPI2_D5_GPIO_PORT           GPIOG

#define OSPI2_D6_PIN                 GPIO_PIN_10
#define OSPI2_D6_GPIO_PORT           GPIOG

#define OSPI2_D7_PIN                 GPIO_PIN_11
#define OSPI2_D7_GPIO_PORT           GPIOG

#define OSPI2_DQS_PIN                GPIO_PIN_12
#define OSPI2_DQS_GPIO_PORT          GPIOF

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Definition for OCTOSPI2 clock resources */
#define OSPI2_CLK_ENABLE()             __HAL_RCC_OSPI2_CLK_ENABLE()
#define OSPI2_CLK_DISABLE()            __HAL_RCC_OSPI2_CLK_DISABLE()
#define OSPI2_CS_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_CLK_P_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D0_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D1_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D2_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D3_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D4_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_D5_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_D6_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_D7_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_DQS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()

#define OSP2_FORCE_RESET()             __HAL_RCC_OSPI2_FORCE_RESET()
#define OSP2_RELEASE_RESET()           __HAL_RCC_OSPI2_RELEASE_RESET()

