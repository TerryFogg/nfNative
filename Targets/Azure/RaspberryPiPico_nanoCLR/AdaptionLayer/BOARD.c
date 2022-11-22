
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include "BOARD.h"
#include "target_board.h"
#include "pico\multicore.h"
#include "pico\binary_info\code.h"
#include "hardware\gpio.h"
#include "pico/stdio_usb.h"

void Initialize_Board()
{
    SystemClock_Config(); // Configure the system clock to 520 MHz


    Initialize_Board_LEDS_And_Buttons();
    Initialize_RTC();

#if (NANOCLR_GRAPHICS == TRUE)
    Initialize_Graphics();
#endif
}

void Initialize_Board_LEDS_And_Buttons()
{
  bi_decl(bi_1pin_with_name(PICO_DEFAULT_LED_PIN, "LED"));
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
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

    // Not all counters on the STM32H7 are 32 bit
    // 32 bit counters used together to create a continuous 64 bit counter of
    // nanoseconds TIM 2  - the master, feeds TIM5 TIM 5  - the slave 2^64
    // nanosecond,  ~ 584 years before overflowing

    // In upcounting mode, the counter counts from 0 to the auto-reload value
}
void BoardLed_ON(uint32_t led){
  gpio_put(LED_PIN, LED_STATE_ON);
  }
void BoardLed_OFF(uint32_t led){
  gpio_put(LED_PIN, LED_STATE_OFF);

  }
void BoardLed_Toggle(uint32_t led)
{

  }
bool BoardUserButton_Pressed()
{
    //    if (BootSel pressed)
    //    {
    //        return true;
    //    }
    //    else
    //    {
    //        return false;
    //    }
    return false;
}

