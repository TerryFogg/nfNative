#pragma once

//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include <stm32h7xx_ll_tim.h>
#include "stm32h7xx.h" // Includes    "stm32h735xx.h"
#include <stm32h7xx_ll_pwr.h>
#include <stm32h7xx_ll_rtc.h>

#include <nanoHAL_v2.h>
#include <nanoCLR_Headers.h>
#include <nanoHAL_Graphics.h>

#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"
#include "usbd_dfu.h"
#include "usbd_dfu_flash.h"
#include "tx_port.h"

#include "TargetFeatures.h"

void BoardInit();
void Initialize_DWT_Counter();

void CPU_CACHE_Enable(void);
void MPU_Config(void);
void SystemClock_Config();



// DWT is connected to the system clock
static inline void DWT_Delay_us(volatile uint32_t microsecond_delay)
{
    LL_RCC_ClocksTypeDef RCC_Clocks;
    LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
    uint32_t initial_microseconds = DWT->CYCCNT;
    uint32_t tick_rate = RCC_Clocks.SYSCLK_Frequency / 1000000;
    microsecond_delay *= tick_rate;
    while ((DWT->CYCCNT - initial_microseconds) < microsecond_delay - tick_rate)
        ;
}

static inline bool OSPI2_WaitUntilState(uint32_t Flag, FlagStatus State) {
  int loopCounter = 10; // 10 loops, 10 microseconds maximum
  while (READ_BIT(OCTOSPI2->SR, Flag) !=
         State) // Wait until flag is in expected state
  {
    DWT_Delay_us(1); // Wait a microsecond;
    loopCounter--;
    if (loopCounter == 0) {
      return false;
    }
  }
  return true;
}


#ifdef __cplusplus
extern "C"
{
#endif
    void LTDCClock_Config(void);
#ifdef __cplusplus
}
#endif
void USBD_Clock_Config(void);

void Initialize_board_LEDS();
void Initialize_OPSPI_Hyperam();
void Initialize_OPSPI_Flash();

void InitializeGraphics(void);
void Initialize_AudioConnector_MEMS(void);
void Initialize_microSD(void);
void Initialize_MEMS_Microphone_Onboard(void);
void Initialize_USB();
void Initialize_Ethernet();
void Initialize_Stereo();
void Initialize_FDCAN();
void Initialize_64bit_timer();
void System_IniRtc(void);
void Startup_Rtos();

// ========================
// STM32H735G-DK board Leds
// ========================

#define LED_GPIO_PORT GPIOC
#define LED_BLUE      GPIO_PIN_3
#define LED_RED       GPIO_PIN_2

// ===============================
// STM32H735G-DK board push button
// ===============================

#define BUTTON_USER_GPIO_PORT GPIOC
#define BUTTON_USER_PIN       GPIO_PIN_13

//----------------------------------------------------------------------------------------------
// Definition for USART wire protocol receive
//----------------------------------------------------------------------------------------------
#define wpBAUD_RATE 921600

#define wpUSART                         USART3
#define wpUSART_IRQn                    USART3_IRQn
#define wpUSART_IRQHANDLER()            void USART3_IRQHandler(void)
#define wpUSART_PERIPHERAL_CLOCK_ENABLE LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)

#define wpUSART_GPIO_PORT                    GPIOD
#define wpUSART_RX_PIN                       GPIO_PIN_9
#define wpUSART_TX_PIN                       GPIO_PIN_8
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

// Board Specific Graphics
#define LTDC_HSPOLARITY_AL          0x00000000U     /*!< Horizontal Synchronization is active low. */
#define LTDC_VSPOLARITY_AL          0x00000000U     /*!< Vertical Synchronization is active low. */
#define LTDC_DEPOLARITY_AL          0x00000000U     /*!< Data Enable, is active low. */
#define LTDC_PCPOLARITY_IPC         0x00000000U     /*!< input pixel clock. */
#define LTDC_IT_TE                  LTDC_IER_TERRIE /*!< LTDC Transfer Error Interrupt  */
#define LTDC_IT_FU                  LTDC_IER_FUIE   /*!< LTDC FIFO Underrun Interrupt   */
#define MAX_LAYER                   2U
#define LTDC_BLENDING_FACTOR2_PAxCA 0x00000007U /*!< Blending factor : Cte Alpha x Pixel Alpha*/

typedef enum
{
    HAL_LTDC_STATE_RESET = 0x00U,   /*!< LTDC not yet initialized or disabled */
    HAL_LTDC_STATE_READY = 0x01U,   /*!< LTDC initialized and ready for use   */
    HAL_LTDC_STATE_BUSY = 0x02U,    /*!< LTDC internal process is ongoing     */
    HAL_LTDC_STATE_TIMEOUT = 0x03U, /*!< LTDC Timeout state                   */
    HAL_LTDC_STATE_ERROR = 0x04U    /*!< LTDC state error                     */
} HAL_LTDC_StateTypeDef;

typedef struct
{
    uint8_t Blue; /*!< Configures the blue value.
                       This parameter must be a number between Min_Data = 0x00 and
                     Max_Data = 0xFF. */

    uint8_t Green; /*!< Configures the green value.
                        This parameter must be a number between Min_Data = 0x00
                      and Max_Data = 0xFF. */

    uint8_t Red; /*!< Configures the red value.
                      This parameter must be a number between Min_Data = 0x00 and
                    Max_Data = 0xFF. */

    uint8_t Reserved; /*!< Reserved 0xFF */
} LTDC_ColorTypeDef;
typedef struct
{
    uint32_t HSPolarity; /*!< configures the horizontal synchronization polarity.
                              This parameter can be one value of @ref
                            LTDC_HS_POLARITY */

    uint32_t VSPolarity; /*!< configures the vertical synchronization polarity.
                              This parameter can be one value of @ref
                            LTDC_VS_POLARITY */

    uint32_t DEPolarity; /*!< configures the data enable polarity.
                              This parameter can be one of value of @ref
                            LTDC_DE_POLARITY */

    uint32_t PCPolarity; /*!< configures the pixel clock polarity.
                              This parameter can be one of value of @ref
                            LTDC_PC_POLARITY */

    uint32_t HorizontalSync; /*!< configures the number of Horizontal
                                synchronization width. This parameter must be a
                                number between Min_Data = 0x000 and Max_Data =
                                0xFFF. */

    uint32_t VerticalSync; /*!< configures the number of Vertical synchronization
                              height. This parameter must be a number between
                              Min_Data = 0x000 and Max_Data = 0x7FF. */

    uint32_t AccumulatedHBP; /*!< configures the accumulated horizontal back porch
                                width. This parameter must be a number between
                                Min_Data = LTDC_HorizontalSync and Max_Data =
                                0xFFF. */

    uint32_t AccumulatedVBP; /*!< configures the accumulated vertical back porch
                                height. This parameter must be a number between
                                Min_Data = LTDC_VerticalSync and Max_Data = 0x7FF. */

    uint32_t AccumulatedActiveW; /*!< configures the accumulated active width.
                                      This parameter must be a number between Min_Data
                                    = LTDC_AccumulatedHBP and Max_Data = 0xFFF. */

    uint32_t AccumulatedActiveH; /*!< configures the accumulated active height.
                                      This parameter must be a number between Min_Data
                                    = LTDC_AccumulatedVBP and Max_Data = 0x7FF. */

    uint32_t TotalWidth; /*!< configures the total width.
                              This parameter must be a number between Min_Data =
                            LTDC_AccumulatedActiveW and Max_Data = 0xFFF. */

    uint32_t TotalHeigh; /*!< configures the total height.
                              This parameter must be a number between Min_Data =
                            LTDC_AccumulatedActiveH and Max_Data = 0x7FF. */

    LTDC_ColorTypeDef Backcolor; /*!< Configures the background color. */
} LTDC_InitTypeDef;
typedef struct
{
    uint32_t WindowX0; /*!< Configures the Window Horizontal Start Position.
                            This parameter must be a number between Min_Data =
                          0x000 and Max_Data = 0xFFF. */

    uint32_t WindowX1; /*!< Configures the Window Horizontal Stop Position.
                            This parameter must be a number between Min_Data =
                          0x000 and Max_Data = 0xFFF. */

    uint32_t WindowY0; /*!< Configures the Window vertical Start Position.
                            This parameter must be a number between Min_Data =
                          0x000 and Max_Data = 0x7FF. */

    uint32_t WindowY1; /*!< Configures the Window vertical Stop Position.
                            This parameter must be a number between Min_Data =
                          0x0000 and Max_Data = 0x7FF. */

    uint32_t PixelFormat; /*!< Specifies the pixel format.
                               This parameter can be one of value of @ref
                             LTDC_Pixelformat */

    uint32_t Alpha; /*!< Specifies the constant alpha used for blending.
                         This parameter must be a number between Min_Data = 0x00
                       and Max_Data = 0xFF. */

    uint32_t Alpha0; /*!< Configures the default alpha value.
                          This parameter must be a number between Min_Data = 0x00
                        and Max_Data = 0xFF. */

    uint32_t BlendingFactor1; /*!< Select the blending factor 1.
                                   This parameter can be one of value of @ref
                                 LTDC_BlendingFactor1 */

    uint32_t BlendingFactor2; /*!< Select the blending factor 2.
                                   This parameter can be one of value of @ref
                                 LTDC_BlendingFactor2 */

    uint32_t FBStartAdress; /*!< Configures the color frame buffer address */

    uint32_t ImageWidth; /*!< Configures the color frame buffer line length.
                              This parameter must be a number between Min_Data =
                            0x0000 and Max_Data = 0x1FFF. */

    uint32_t ImageHeight; /*!< Specifies the number of line in frame buffer.
                               This parameter must be a number between Min_Data =
                             0x000 and Max_Data = 0x7FF. */

    LTDC_ColorTypeDef Backcolor; /*!< Configures the layer background color. */
} LTDC_LayerCfgTypeDef;
typedef struct
{
    LTDC_TypeDef *Instance;                   /*!< LTDC Register base address                */
    LTDC_InitTypeDef Init;                    /*!< LTDC parameters                           */
    LTDC_LayerCfgTypeDef LayerCfg[MAX_LAYER]; /*!< LTDC Layers parameters */
    HAL_LockTypeDef Lock;                     /*!< LTDC Lock                                 */
    __IO HAL_LTDC_StateTypeDef State;         /*!< LTDC state */
    __IO uint32_t ErrorCode;                  /*!< LTDC Error code                           */
} LTDC_HandleTypeDef;

// Definition for Graphics on the board

#define TS_INT_PIN GPIO_PIN_2

//------------------
