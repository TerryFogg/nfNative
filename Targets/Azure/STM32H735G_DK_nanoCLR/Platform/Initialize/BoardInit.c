//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <BoardInit.h>


void BoardInit()
{
    CPU_CACHE_Enable();
    //  MPU_Config();

    //   HAL_InitTick() is called from HAL_Init() to setup 1ms tick rate for
    //   HAL_DELAY()
    HAL_Init(); // STM32H7xx HAL library initialization

    SystemClock_Config();                    // Configure the system clock to 520 MHz
    FMC_Bank1_R->BTCR[0] &= ~FMC_BCRx_MBKEN; // Disabling FMC Bank1 ? To prevent this CortexM7
                                             // speculative read accesses on FMC bank1, it is
                                             // recommended to disable it when it is not used

    Board_LED_Initialization();
    Initialize_OPSPI_Hyperam();

    // init2();
    Initialize_OPSPI_Flash();
    // Can we use the following code in an update to flash only?
    HAL_FLASH_Unlock();

    Configure_RTC();
    ConfigureCRC();
    InitializeGraphics();
    Initialize_AudioConnector_MEMS();
    Initialize_microSD();
    Initialize_MEMS_Microphone_Onboard();
    Initialize_USB();
    Initialize_Ethernet();
    Initialize_Stereo();
    Initialize_FDCAN();
}
void CPU_CACHE_Enable(void)
{
    SCB_EnableICache(); // Enable I-Cache
    // SCB_EnableDCache(); // Enable D-Cache
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
void ConfigureCRC()
{

    // At boot time (reset) the following are the defaults
    // that may need to be changed
    // LL_CRC_DEFAULT_CRC32_POLY
    // LL_CRC_POLYLENGTH_32B
    // LL_CRC_DEFAULT_CRC_INITVALUE
    // LL_CRC_INDATA_REVERSE_NONE
    // LL_CRC_OUTDATA_REVERSE_NONE
    // Enable peripheral clock for CRC
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_CRC);
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
void Initialize_OPSPI_Flash()
{
    return;
}
void Initialize_AudioConnector_MEMS()
{
    return;
}
void Initialize_microSD()
{
    SDMMC_InitTypeDef Init;
    Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    Init.BusWide = SDMMC_BUS_WIDE_4B;
    Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    Init.ClockDiv = 0;

    uint32_t tmpreg = 0;
    /* Set SDMMC configuration parameters */
    tmpreg |= (Init.ClockEdge | Init.ClockPowerSave | Init.BusWide | Init.HardwareFlowControl | Init.ClockDiv);
    /* Write to SDMMC CLKCR */
    MODIFY_REG(SDMMC1->CLKCR, CLKCR_CLEAR_MASK, tmpreg);
}

void Initialize_MEMS_Microphone_Onboard()
{
    return;
}

USBD_HandleTypeDef hUsbDeviceHS;
extern USBD_DescriptorsTypeDef DFU_Desc;

void Initialize_USB()
{
    USBD_Clock_Config();
    //        USBD_Init(&hUsbDeviceHS, &DFU_Desc, DEVICE_HS);
    //        USBD_RegisterClass(&hUsbDeviceHS, &USBD_DFU);
    //        USBD_DFU_RegisterMedia(&hUsbDeviceHS, &USBD_DFU_Flash_fops);
    //        USBD_Start(&hUsbDeviceHS);
}
void Initialize_Ethernet()
{
    return;
}
void Initialize_Stereo()
{
    return;
}
void Initialize_FDCAN()
{
    return;
}

void nanosecond64bitTimer()
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

//-----------------------------
