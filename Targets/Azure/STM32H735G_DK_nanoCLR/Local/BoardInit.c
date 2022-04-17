//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include <BoardInit.h>

void BoardInit()
{
    CPU_CACHE_Enable();
    //MPU_Config();
    SystemClock_Config();                    // Configure the system clock to 520 MHz
    FMC_Bank1_R->BTCR[0] &= ~FMC_BCRx_MBKEN; // Disabling FMC Bank1 ? To prevent this CortexM7
                                             // speculative read accesses on FMC bank1, it is
                                             // recommended to disable it when it is not used
    Initialize_DWT_Counter();                // Counter used for microsecond delays (blocking)
    Initialize_board_LEDS();
    Initialize_OCTOSPI2_Hyperam();
    Initialize_OPSPI_Flash();
    Initialize_RTC();
}
void CPU_CACHE_Enable(void)
{
    SCB_EnableICache(); // Enable I-Cache
    //SCB_EnableDCache(); // Enable D-Cache
}
void Initialize_board_LEDS()
{
    // LEDs and user button of the STM32H735G-DK Board
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_GPIO_InitTypeDef gpio_InitStruct = {0};
    gpio_InitStruct.Pin = LED_GREEN | LED_RED;
    gpio_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    gpio_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_GPIO_PORT, &gpio_InitStruct);
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
    // The MPU can be used to protect up to sixteen memory regions.
    // Each region in turn can have eight subregions, if the region is at least
    // 256 bytes.
    //
    // The subregions are always of equal size, and can be enabled or disabled by
    // a subregion number. Because the  minimum region size is driven by the cache
    // line length of 32 bytes, eight subregions of 32 bytes corresponds to a 256
    // byte size. The regions are numbered 0 to 15. There is also a default region
    // with an id of 1. All of the (0 -15) memory regions take priority over the
    // default region. The regions can overlap,and can be nested.
    // Region 15 has the highest priority and region 0 the lowest.

    //
    // Modifies the following registers
    //
    // MPU->RNR  - region
    // MPU->RBAR - Base Address
    //
    // MPU->RASR  register bits
    // Bits      Name  Description
    // 28        XN    Execute never
    // 26:24     AP    Data access permission field(RO, RW or No  access)

    //____________________________________________________________________________________
    // 21:19     TEX   Type extension field ( Cache properties and shareability)
    // 17        C     Cacheable
    // 16        B     Bufferable
    //
    // 18        S     Shareable (The S field is equivalent to non-cacheable memory.)
    // *S bit - (Shareable/Not Shareable)
    // /---------------------------------------------------------------------------------\
    // | TEX | C | B | Memory Type      | Description                        | Shareable |
    // | ----| --| --| -----------------| --------------------               |           |
    // | 000 | 0 | 0 | Strongly Ordered | Strongly Ordered                   | Yes       |
    // | 000 | 0 | 1 | Device           | Shared Device                      |           |
    // | 000 | 1 | 0 | Normal           | Write through, no write allocate   | *S bit    |
    // | 000 | 1 | 1 | Normal           | Write-back, no write allocate      | *S bit    |
    // | 001 | 0 | 0 | Normal           | Non-cacheable                      | *S bit    |
    // | 001 | 0 | 1 | Reserved         | Reserved                           | Reserved  |
    // | 001 | 1 | 0 | Undefined        | Undefined                          | Undefined |
    // | 001 | 1 | 1 | Normal           | Write-back, write and read allocate| *S bit    |
    // | 010 | 0 | 0 | Device           | Non-shareable device               | No        |
    // | 010 | 0 | 1 | Reserved         | Reserved                           | Reserved  |
    // \---------------------------------------------------------------------------------/

    // 15:8      SRD   Subregion disabled.For each subregion 1 = disabled, 0 = enabled.
    // 5:1       SIZE  Specifies the size of the MPU protection region.

    // Write-back: the cache does not write the cache contents to the memory until a clean operation is done.
    // Write-through: triggers a write to the memory as soon as the contents on the cache line are written to. This
    // is safer for the data coherency, but it requires more bus accesses. In practice, the write to the memory is
    // done in the background and has a little effect unless the same cache set is being accessed repeatedly and
    // very quickly. It is always a tradeoff.

    LL_MPU_Disable();

    uint32_t Region = LL_MPU_REGION_NUMBER0;
    uint32_t SubRegionDisable = 0;
    uint32_t Address = D2_AHBSRAM_BASE;                       // On CHIP Ram designated as .dma_buffer
    uint32_t Attributes = LL_MPU_REGION_NUMBER0 |             // Use Region 0
                          LL_MPU_REGION_SIZE_32KB |           // 32KB
                          LL_MPU_REGION_FULL_ACCESS |         // Full permissions
                          LL_MPU_INSTRUCTION_ACCESS_DISABLE | // Data buffers, no code execution
                                                              // --------------------------------
                          LL_MPU_TEX_LEVEL0 |                 //
                          LL_MPU_ACCESS_SHAREABLE |           //
                          LL_MPU_ACCESS_CACHEABLE |           //
                          LL_MPU_ACCESS_NOT_BUFFERABLE;       //

    LL_MPU_ConfigRegion(Region, SubRegionDisable, Address, Attributes);
    LL_MPU_EnableRegion(LL_MPU_REGION_NUMBER0);

    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
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
    // In addition to all these clock sources, the timer should be clocked with
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
    LL_TIM_SetTriggerInput(TIM5, LL_TIM_TS_ITR1); // TRGO output from TIM2
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
