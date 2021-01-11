//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2017 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <nanoCLR_Headers.h>
#include <stm32h7xx_hal.h>
#include <BoardInit.h>
#include <WireProtocol_Communications.h>
#include <nanoHAL_v2.h>

#define RTC_ASYNCH_PREDIV 0x7F      // LSE as RTC clock
#define RTC_SYNCH_PREDIV 0x00FF     // LSE as RTC clock

#ifdef HAL_RTC_MODULE_ENABLED
RTC_HandleTypeDef RtcHandle;
#endif

CRC_HandleTypeDef CrcHandle;
UART_HandleTypeDef WProtocolUart;
DMA_HandleTypeDef s_DMAHandle;

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
        while (1)
        {
        }
    }
}
#endif // HAL_RTC_MODULE_ENABLED

void BoardInit()
{
    
    HAL_Init();            // STM32H7xx HAL library initialization
    SCB_EnableICache();    // Enable I-Cache
    SCB_EnableDCache();    // Enable D-Cache
    SystemClock_Config();  // To review: Do we need to configure the system clock from default??

    LedsAndBoardInit();

    bool wireProtocolResult = InitWireProtocolCommunications();

    // System_IniRtc();

    // config CRC32 unit
    CrcHandle.Instance = CRC;
    CrcHandle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    CrcHandle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    CrcHandle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;

    HAL_CRC_Init(&CrcHandle);
}

// LEDs and user button of the STM32H7B3I-DK board
void LedsAndBoardInit()
{
    __GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = LED_BLUE | LED_RED;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
}

__attribute__((noreturn));
void nanoBooterStatus(uint32_t nanoBooterState) {
    tx_thread_sleep(500);

    switch ((eBooterStatus)nanoBooterState)
    {
    case ok:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED, GPIO_PIN_SET);  // Off
        while (true) {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_RESET);
            tx_thread_sleep(500);

            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_SET);
            tx_thread_sleep(500);
        }
        break;
    case communications_failure:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_SET);  // Off
        while (true) {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED, GPIO_PIN_RESET);
            tx_thread_sleep(200);

            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED, GPIO_PIN_SET);
            tx_thread_sleep(200);
        }
        break;
    }

}
