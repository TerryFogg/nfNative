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

void BoardInit();

void CPU_CACHE_Enable(void);
void MPU_Config(void);
void SystemClock_Config();
void USBD_Clock_Config(void);

void Board_LED_Initialization();
void Initialize_OPSPI_Hyperam();
void Initialize_OPSPI_Flash();

void System_IniRtc();
void InitializeGraphics(void);
void Initialize_AudioConnector_MEMS(void);
void Initialize_microSD(void);
void Initialize_MEMS_Microphone_Onboard(void);
void Initialize_USB();
void Initialize_Ethernet();
void Initialize_Stereo();
void Initialize_FDCAN();

void ConfigureCRC();
void System_IniRtc(void);
void Startup_Rtos();
    
// ========================
// STM32H7B3I-DK board Leds
// ========================

#define LED_GPIO_PORT  GPIOC
#define LED_BLUE       GPIO_PIN_3
#define LED_RED        GPIO_PIN_2

// ===============================
// STM32H7B3I-DK board push button
// ===============================

#define BUTTON_USER_GPIO_PORT  GPIOC
#define BUTTON_USER_PIN        GPIO_PIN_13

// ==========================================
// Definition for USART wire protocol receive
// ==========================================

#define wpUSART                                 USART3
#define wpUSART_GPIO_PORT                       GPIOD
#define wpUSART_RX_PIN                          GPIO_PIN_9
#define wpUSART_TX_PIN                          GPIO_PIN_8
#define wpUSART_PERIPHERAL_CLOCK_ENABLE         LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)
#define wpUSART_GPIO_PERIPHERAL_CLOCK_ENABLE    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD)

#define wpDMA                                   DMA1
#define wpUSART_DMA_PERIPHERAL_CLOCK_ENABLE     LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1)
#define wpDMA_ReceiveStream                     LL_DMA_STREAM_0        
#define wpDMA_TransmitStream                    LL_DMA_STREAM_1
#define wpDMA_ReceiveMux                        LL_DMAMUX1_REQ_USART3_RX
#define wpDMA_TransmitMux                       LL_DMAMUX1_REQ_USART3_TX

#define wpBAUD_RATE                             921600


//
//#define wpDMA_RX_Stream_IRQHandler  DMA1_Stream0_IRQHandler
//#define wpDMA_TX_Stream_IRQHandler  DMA1_Stream1_IRQHandler
//#define wpUSART_IRQHandler          USART3_IRQHandler

#define wpUSART_CLK_ENABLE()        __HAL_RCC_USART3_CLK_ENABLE()
#define wpDMA_CLK_ENABLE()                      __HAL_RCC_DMA1_CLK_ENABLE()
#define wpUSART_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()

#define wpUSART_FORCE_RESET()       __HAL_RCC_USART3_FORCE_RESET()
#define wpUSART_RELEASE_RESET()     __HAL_RCC_USART3_RELEASE_RESET()

#define wpUSART_AF                  GPIO_AF7_USART3

#define wpUSART_RX_DMA_REQUEST      DMA_REQUEST_USART3_RX
#define wpUSART_RX_DMA_STREAM       DMA2_Stream1
#define wpUSART_DMA_RX_IRQn         DMA2_Stream1_IRQn
#define wpUSART_DMA_RX_IRQHandler   DMA2_Stream1_IRQHandler

#define wpUSART_TX_DMA_REQUEST      DMA_REQUEST_USART3_TX
#define wpUSART_TX_DMA_STREAM       DMA2_Stream7
#define wpUSART_DMA_TX_IRQn         DMA2_Stream7_IRQn
#define wpUSART_DMA_TX_IRQHandler   DMA2_Stream7_IRQHandler

// =======================================
// Definition for USART wire protocol NVIC
// =======================================

#define wpUSART_IRQn                USART3_IRQn
#define wpUSART_IRQHandler          USART3_IRQHandler

// Definitions for DMA flags ( PER CHANNEL)
#define wpCLEAR_TRANSFER_COMPLETE       DMA_FLAG_TCIF3_7 
#define wpCLEAR_HALF_TRANSFER_COMPLETE  DMA_FLAG_HTIF3_7 
#define wpCLEAR_TRANSFER_ERROR          DMA_FLAG_TEIF3_7 
#define wpCLEAR_DIRECT_MODE_ERROR       DMA_FLAG_DMEIF3_7
#define wpCLEAR_FIFO_ERROR              DMA_FLAG_FEIF3_7 


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

