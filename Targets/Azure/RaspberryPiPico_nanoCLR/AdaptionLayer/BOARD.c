
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include "BOARD.h"
#include "hardware\gpio.h"
#include "pico\binary_info\code.h"
#include "pico\multicore.h"
#include "target_board.h"

uint8_t keyA = 15;
uint8_t keyB = 17;

uint8_t up = 2;
uint8_t dowm = 18;
uint8_t left = 16;
uint8_t right = 20;

uint8_t ctrl = 3;
void Initialize_Board()
{
    SystemClock_Config(); // Configure the system clock to 520 MHz
                          //   bool success = stdio_usb_init();
    Initialize_Board_LEDS_And_Buttons();

#if (NANOCLR_AUDIO == TRUE)
    Initialize_Audio();
#endif
#if (NANOCLR_ETHERNET == TRUE)
    Initialize_Ethernet();
#endif
#if (NANOCLR_FDCAN == TRUE)
    Initialize_FDCAN();
#endif
#if (NANOCLR_MICROSD == TRUE)
    Initialize_microSD();
#endif
#if (NANOCLR_RTC == TRUE)
    Initialize_RTC();
#endif
#if (NANOCLR_USB == TRUE)
    Initialize_USB();
#endif
}

void Initialize_Board_LEDS_And_Buttons()
{
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_LED_PIN, "LED"));
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_init(keyA);
    gpio_pull_up(keyA);
    gpio_set_dir(keyA, GPIO_IN);
}
void Initialize_DWT_Counter()
{
}
void Initialize_64bit_timer()
{
    // Although we setup a "nanosecond" timer, the real resolution is less than
    // 1ns. To get 1ns resolution would require a clock speed of 1GHz.
    //
    // For this implementation, 100ns is the smallest precision.

    // Not all counters on the ARM Cortex are 32 bit
    // 32 bit counters used together to create a continuous 64 bit counter of
    // nanoseconds TIM 2  - the master, feeds TIM5 TIM 5  - the slave 2^64
    // nanosecond,  ~ 584 years before overflowing

    // In upcounting mode, the counter counts from 0 to the auto-reload value
}
void BoardLed_ON(uint32_t led)
{
    gpio_put(LED_PIN, LED_STATE_ON);
}
void BoardLed_OFF(uint32_t led)
{
    gpio_put(LED_PIN, LED_STATE_OFF);
}
void BoardLed_Toggle(uint32_t led)
{
}
bool BoardUserButton_Pressed()
{

    bool notPressed = gpio_get(keyA);

    if (notPressed)
    {
        return false;
    }
    else
    {
        return true;
    }
}
