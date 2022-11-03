#pragma once

//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include <nanoHAL_v2.h>
#include <nanoCLR_Headers.h>
#include <nanoHAL_Graphics.h>

#include "tx_port.h"

#include "TargetFeatures.h"

typedef struct __nfpack HardFaultReporting {
  int count;
} HardFaultReporting;


void Initialize_Board();
void Initialize_RTC();
void Initialize_DWT_Counter();
void USBD_Clock_Config(void);
void Initialize_Board_LEDS_And_Buttons();
void Initialize_64bit_timer();
void CPU_CACHE_Enable(void);
void MPU_Config(void);
void SystemClock_Config();
void Startup_Rtos();
void BoardLed_ON(uint32_t led);
void BoardLed_OFF(uint32_t led);
void BoardLed_Toggle(uint32_t led);
bool BoardUserButton_Pressed();
static inline uint32_t Get_SYSTICK();

// DWT is connected to the system clock
static inline void DWT_Delay_us(volatile uint32_t microsecond_delay)
{
  
    //// TODO - At boot time not relevant, but Important  WRAP AROUND counter !!!! to fix
    //LL_RCC_ClocksTypeDef RCC_Clocks;
    //LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
    //uint32_t initial_microseconds = DWT->CYCCNT;
    //uint32_t tick_rate = RCC_Clocks.SYSCLK_Frequency / 1000000;
    //microsecond_delay *= tick_rate;
    //while ((DWT->CYCCNT - initial_microseconds) < microsecond_delay - tick_rate)
    //    ;
}

static inline uint32_t Get_SYSTICK()
{
    return tx_time_get();
}


#ifdef __cplusplus
extern "C"
{
#endif
    void LTDCClock_Config(void);
#ifdef __cplusplus
}
#endif

// ========================
// PICO board Leds
// ========================

#define LED_GPIO_PORT GPIOC
#define LED_GREEN     LL_GPIO_PIN_3
#define LED_RED       LL_GPIO_PIN_2

// ===============================
// STM32H735G-DK board push button
// ===============================

#define BUTTON_USER_GPIO_PORT GPIOC
#define BUTTON_USER_PIN       LL_GPIO_PIN_13

// Board Specific Graphics
#define LTDC_HSPOLARITY_AL          0x00000000U     // Horizontal Synchronization is active low.
#define LTDC_VSPOLARITY_AL          0x00000000U     // Vertical Synchronization is active low.
#define LTDC_DEPOLARITY_AL          0x00000000U     // Data Enable, is active low.
#define LTDC_PCPOLARITY_IPC         0x00000000U     // input pixel clock.
#define LTDC_IT_TE                  LTDC_IER_TERRIE // LTDC Transfer Error Interrupt
#define LTDC_IT_FU                  LTDC_IER_FUIE   // LTDC FIFO Underrun Interrupt
#define MAX_LAYER                   2U
#define LTDC_BLENDING_FACTOR2_PAxCA 0x00000007U     // Blending factor : Cte Alpha x Pixel Alpha

// Definition for Graphics on the board

#define TS_INT_PIN LL_GPIO_PIN_2

