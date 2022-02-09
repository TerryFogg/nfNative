//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <BoardInit.h>


void init2();

void BoardInit()
{
    CPU_CACHE_Enable();
    //  MPU_Config();

    SystemClock_Config();                    // Configure the system clock to 520 MHz
    FMC_Bank1_R->BTCR[0] &= ~FMC_BCRx_MBKEN; // Disabling FMC Bank1 ? To prevent this CortexM7
                                             // speculative read accesses on FMC bank1, it is
                                             // recommended to disable it when it is not used
    Initialize_DWT_Counter();                // Counter used for microsecond delays (blocking)
    Initialize_board_LEDS();
    Initialize_OPSPI_Hyperam();
  //   init2();
    Initialize_OPSPI_Flash();
    Initialize_RTC();
    Initialize_CRC();
    InitializeGraphics();
    Initialize_Audio_Features();
    Initialize_microSD();
    Initialize_USB();
    Initialize_Ethernet();
    Initialize_FDCAN();
}
void CPU_CACHE_Enable(void)
{
    SCB_EnableICache(); // Enable I-Cache
                        // SCB_EnableDCache(); // Enable D-Cache
}
void Initialize_board_LEDS()
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
void Initialize_DWT_Counter()
{
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // Disable TRC
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable TRC
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;            // Disable clock cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable  clock cycle counter
    DWT->CYCCNT = 0;                                 // Reset the clock cycle counter value
}
void MPU_Config(void)
{
    //  MPU_Region_InitTypeDef MPU_InitStruct;
    //
    //  /* Disable the MPU */
    //  HAL_MPU_Disable();
    //
    //  /* Configure the MPU attributes as write through for OctoSPI RAM */
    //  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    //  MPU_InitStruct.BaseAddress = OSPI_RAM_WRITE_READ_ADDR;
    //  MPU_InitStruct.Size = MPU_REGION_SIZE_16MB;
    //  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    //  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    //  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    //  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    //  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    //  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    //  MPU_InitStruct.SubRegionDisable = 0x00;
    //  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    //  HAL_MPU_ConfigRegion(&MPU_InitStruct);
    //
    //  /* Enable the MPU */
    //  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
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

    int TIM_CLK = 520; // MHZ

    LL_RCC_ClocksTypeDef RCC_Clocks;
    LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

    // Compute the prescaler value to have input to be 100ns pulses
    int32_t nanosecond100_per_second = 10000000;
    uint32_t prescaler_value = RCC_Clocks.SYSCLK_Frequency / nanosecond100_per_second;

    LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
    // n addition to all these clock sources, the timer should be clocked with
    // the APBx clock  (page 10. AN4013)
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetPrescaler(TIM2, prescaler_value);
    LL_TIM_EnableMasterSlaveMode(TIM2);

    // Update: the update event is selected as trigger output (TRGO).
    // This timer is used as a prescaler for a slave timer;
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
    LL_TIM_DisableMasterSlaveMode(TIM2);

    LL_TIM_SetSlaveMode(TIM5, LL_TIM_SLAVEMODE_TRIGGER);
    LL_TIM_SetClockDivision(TIM5, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_SetCounterMode(TIM5, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetTriggerInput(TIM5, TIM_TS_ITR1); // TRGO output from TIM2
    LL_TIM_EnableExternalClock(TIM5);

    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM5);

    //    do
    //    {
    //        uint32_t lower32bits = LL_TIM_GetCounter(TIM2);
    //        uint32_t upper32bits = LL_TIM_GetCounter(TIM5);
    //        uint64_t nanoSeconds = ((nanoSeconds & upper32bits) << 32) &
    //        lower32bits;
    //    } while (true);
}

