//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <BoardInit.h>
#include "tx_port.h"

eBooterStatus nanoBooterState;

void BoardInit()
{
    SystemClock_Config(); // Configure the system clock to 520 MHz
    FMC_Bank1_R->BTCR[0] &= ~FMC_BCRx_MBKEN;
    Initialize_board_LEDS_and_User_Button();
}
void Initialize_board_LEDS_and_User_Button()
{
    // LEDs and user button use Port C of the STM32H735G-DK board
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);

    // LEDS
    LL_GPIO_InitTypeDef gpio_InitStruct = {0};
    gpio_InitStruct.Pin = LED_GREEN | LED_RED;
    gpio_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    gpio_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_GPIO_PORT, &gpio_InitStruct);

    // User Button
    LL_GPIO_InitTypeDef gpio_InitStruct_button = {0};
    gpio_InitStruct_button.Pin = BUTTON_USER_PIN;
    gpio_InitStruct_button.Mode = LL_GPIO_MODE_INPUT;
    gpio_InitStruct_button.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_InitStruct_button.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(BUTTON_USER_GPIO_PORT, &gpio_InitStruct_button);
}

bool UserButtonPressed()
{
    return LL_GPIO_IsInputPinSet(BUTTON_USER_GPIO_PORT, BUTTON_USER_PIN) == 1 ? true : false;
}

__attribute__((noreturn)) void nanoBooterStatus(uint32_t nanoBooterState)
{
    while (true)
    {
        switch ((eBooterStatus)nanoBooterState)
        {
            case ok:
                LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_GREEN);
                while (true)
                {
                    LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_GREEN);
                    tx_thread_sleep(50);
                    LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_GREEN);
                    tx_thread_sleep(50);
                }
                break;
            case communications_failure:
                LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_RED);
                while (true)
                {
                    LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_RED);
                    tx_thread_sleep(50);

                    LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_RED);
                    tx_thread_sleep(50);
                }
                break;
        }
    }
}
