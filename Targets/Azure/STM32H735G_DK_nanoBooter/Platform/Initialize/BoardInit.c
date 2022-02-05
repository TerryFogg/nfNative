//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <BoardInit.h>
#include "tx_port.h"
#include "stm32h735g_discovery_ospi.h"

CRC_HandleTypeDef CrcHandle;

OSPI_HandleTypeDef OSPIHandle;
OSPI_HandleTypeDef A_hospi;
eBooterStatus nanoBooterState;

void BoardInit()
{
    CPU_CACHE_Enable();
    HAL_Init(); // STM32H7xx HAL library initialization

    SystemClock_Config(); // Configure the system clock to 520 MHz
    FMC_Bank1_R->BTCR[0] &= ~FMC_BCRx_MBKEN;
    Board_LED_Initialization();
}
void CPU_CACHE_Enable(void)
{
    SCB_EnableICache(); // Enable I-Cache
                        //  SCB_EnableDCache();                                // Enable D-Cache
}
void Board_LED_Initialization()
{
    // LEDs and user button of the STM32H7B3I-DK board
    __GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = LED_BLUE | LED_RED;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
}
__attribute__((noreturn)) void nanoBooterStatus(uint32_t nanoBooterState)
{
    while (true)
    {
        switch ((eBooterStatus)nanoBooterState)
        {
            case ok:
                HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED, GPIO_PIN_SET); // Off
                while (true)
                {
                    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_RESET);
                    tx_thread_sleep(50);

                    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_SET);
                    tx_thread_sleep(50);
                }
                break;
            case communications_failure:
                HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_SET); // Off
                while (true)
                {
                    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED, GPIO_PIN_RESET);
                    tx_thread_sleep(50);

                    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED, GPIO_PIN_SET);
                    tx_thread_sleep(50);
                }
                break;
        }
    }
}

