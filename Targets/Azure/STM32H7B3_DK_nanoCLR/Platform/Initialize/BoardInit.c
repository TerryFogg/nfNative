//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <BoardInit.h>
#include "stm32h7b3i_discovery_sdram.h"

#define RTC_ASYNCH_PREDIV 0x7F      // LSE as RTC clock
#define RTC_SYNCH_PREDIV 0x00FF     // LSE as RTC clock

#ifdef HAL_RTC_MODULE_ENABLED
RTC_HandleTypeDef RtcHandle;
#endif

UART_HandleTypeDef WProtocolUart;
DMA_HandleTypeDef s_DMAHandle;


CRC_HandleTypeDef CrcHandle;


#ifdef HAL_RTC_MODULE_ENABLED
void System_IniRtc(void)
{
    RtcHandle.Instance = RTC;
    RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
    {
        // Initialization Error
        while(1)
        {
        }
    }
}
#endif // HAL_RTC_MODULE_ENABLED

void BoardInit()
{

    CPU_CACHE_Enable();
    HAL_Init();              // STM32H7xx HAL library initialization
    SystemClock_Config();    // To review: Do we need to configure the system clock from default??
    Board_LED_Initialization();
    
    BSP_SDRAM_Init(0);      // Initialized SDRAM

        // System_IniRtc();
    ConfigureCRC();
    InitializeGraphics();
}

void CPU_CACHE_Enable(void)
{
    SCB_EnableICache();         // Enable I-Cache
    SCB_EnableDCache();         // Enable D-Cache
}


// LEDs and user button of the STM32H7B3I-DK board
void Board_LED_Initialization()
{
    __GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = LED_BLUE | LED_RED;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
}
void ConfigureCRC()
{
    CrcHandle.Instance = CRC;
    CrcHandle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    CrcHandle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    CrcHandle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    HAL_CRC_Init(&CrcHandle);
}

