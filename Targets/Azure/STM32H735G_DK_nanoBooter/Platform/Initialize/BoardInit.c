//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <BoardInit.h>
#include "tx_port.h"
#include "stm32h735g_discovery_ospi.h"

UART_HandleTypeDef WProtocolUart;
DMA_HandleTypeDef s_DMAHandle;

CRC_HandleTypeDef CrcHandle;
uint8_t CmdCplt, TxCplt, StatusMatch, RxCplt;

OSPI_HandleTypeDef OSPIHandle;
OSPI_HandleTypeDef A_hospi;
 

#define RTC_ASYNCH_PREDIV 0x7F      // LSE as RTC clock
#define RTC_SYNCH_PREDIV 0x00FF     // LSE as RTC clock
eBooterStatus nanoBooterState;

void BoardInit()
{
    CPU_CACHE_Enable();
    //  MPU_Config();
    
      //   HAL_InitTick() is called from HAL_Init() to setup 1ms tick rate for HAL_DELAY()
      HAL_Init();                                           // STM32H7xx HAL library initialization

      SystemClock_Config();                                // Configure the system clock to 520 MHz
      FMC_Bank1_R->BTCR[0] &= ~FMC_BCRx_MBKEN;
    Board_LED_Initialization();
    Initialize_OPSPI_Hyperam();

    Initialize_OPSPI_Flash();
    // Can we use the following code in an update to flash only?
    HAL_FLASH_Unlock();

}

void CPU_CACHE_Enable(void)
{
    SCB_EnableICache();                                // Enable I-Cache
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

void Initialize_OPSPI_Hyperam()
{
    int32_t ret;
    static MDMA_HandleTypeDef mdma_handle;
    GPIO_InitTypeDef GPIO_InitStruct;
    uint16_t reg;
    
    OSPI_HandleTypeDef opspi_config = { 0 };

    __HAL_RCC_OSPI2_CLK_ENABLE();
    __HAL_RCC_MDMA_CLK_ENABLE();
    
    __HAL_RCC_GPIOF_CLK_ENABLE();                                  // Ports F and G have pins for the OctoSPI[2]
    __HAL_RCC_GPIOG_CLK_ENABLE();
        
    __HAL_RCC_OSPI2_FORCE_RESET();                                 // Reset the OctoSPI memory interface
    __HAL_RCC_OSPI2_RELEASE_RESET();

    // OctoSPI CS GPIO pin configuration
    GPIO_InitStruct.Pin       = OSPI_RAM_CS_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPIM_P2;   
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // OctoSPI DQS GPIO pin configuration
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Pin       = OSPI_RAM_DQS_PIN;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // OctoSPI CLK GPIO pin configuration
    GPIO_InitStruct.Pin       = OSPI_RAM_CLK_PIN;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // OctoSPI D0,D1,D2,D3 GPIO pin configuration
    GPIO_InitStruct.Pin       = OSPI_RAM_D0_PIN | OSPI_RAM_D1_PIN | OSPI_RAM_D2_PIN | OSPI_RAM_D3_PIN;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // OctoSPI D4,D5,D7 GPIO pin configuration
    GPIO_InitStruct.Pin       = OSPI_RAM_D4_PIN | OSPI_RAM_D5_PIN | OSPI_RAM_D7_PIN;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // OctoSPI D6 GPIO pin configuration
    GPIO_InitStruct.Pin       = OSPI_RAM_D6_PIN;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // Not Required ?????
    // --------------------    
   
    // Configure the OctoSPI DMA
    //    mdma_handle.Init.Request                  = MDMA_REQUEST_SW;
    //    mdma_handle.Init.TransferTriggerMode      = MDMA_BLOCK_TRANSFER;
    //    mdma_handle.Init.Priority                 = MDMA_PRIORITY_HIGH;
    //    mdma_handle.Init.SourceInc                = MDMA_SRC_INC_WORD;
    //    mdma_handle.Init.DestinationInc           = MDMA_DEST_INC_WORD;
    //    mdma_handle.Init.SourceDataSize           = MDMA_SRC_DATASIZE_WORD;
    //    mdma_handle.Init.DestDataSize             = MDMA_DEST_DATASIZE_WORD;
    //    mdma_handle.Init.DataAlignment            = MDMA_DATAALIGN_PACKENABLE;
    //    mdma_handle.Init.SourceBurst              = MDMA_SOURCE_BURST_SINGLE;
    //    mdma_handle.Init.DestBurst                = MDMA_DEST_BURST_SINGLE;
    //    mdma_handle.Init.BufferTransferLength     = 128;
    //    mdma_handle.Init.SourceBlockAddressOffset = 0;
    //    mdma_handle.Init.DestBlockAddressOffset   = 0;
    //
    //    mdma_handle.Instance = MDMA_Channel0;
    //
    //    __HAL_LINKDMA(&opspi_config, hmdma, mdma_handle);
    //    (void) HAL_MDMA_Init(&mdma_handle);
    //
    //    /* Enable and set priority of the OctoSPI and DMA interrupts */
    //    HAL_NVIC_SetPriority(OCTOSPI2_IRQn, BSP_OSPI_RAM_IT_PRIORITY, 0);
    //    HAL_NVIC_SetPriority(MDMA_IRQn, BSP_OSPI_RAM_DMA_IT_PRIORITY, 0);
    //
    //    HAL_NVIC_EnableIRQ(OCTOSPI2_IRQn);
    //    HAL_NVIC_EnableIRQ(MDMA_IRQn);        
    // 
    
    
        // HyperRAM clock configuration [ OPSPI2 ]
        //  PLL2_VCO Input = HSE_VALUE/PLL3M = 5 Mhz
        //  PLL2_VCO Output = PLL2_VCO Input * PLL3N = 400 Mhz
        //  PLLOSPICLK = PLL2_VCO Output/PLL2R = 400/2 = 200 Mhz
        //  OSPI clock frequency = PLLOSPICLK = 200 Mhz
    
        RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
    PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
    PeriphClkInitStruct.PLL2.PLL2M = 5;
    PeriphClkInitStruct.PLL2.PLL2N = 80;
    PeriphClkInitStruct.PLL2.PLL2P = 5;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    
    HAL_StatusTypeDef status;
    opspi_config.Instance = OCTOSPI2;
    opspi_config.Init.FifoThreshold         = 4;
    opspi_config.Init.DualQuad              = HAL_OSPI_DUALQUAD_DISABLE;
    opspi_config.Init.MemoryType            = HAL_OSPI_MEMTYPE_HYPERBUS;
    opspi_config.Init.DeviceSize            = (uint32_t)POSITION_VAL(S70KL1281_RAM_SIZE);
    opspi_config.Init.ChipSelectHighTime    = 4;
    opspi_config.Init.FreeRunningClock      = HAL_OSPI_FREERUNCLK_DISABLE;
    opspi_config.Init.ClockMode             = HAL_OSPI_CLOCK_MODE_0;
    opspi_config.Init.WrapSize              = HAL_OSPI_WRAP_NOT_SUPPORTED;
    opspi_config.Init.ClockPrescaler        = 4U;                                                          // OctoSPI clock = 200 MHz / ClockPrescaler = 100 MHz
    opspi_config.Init.SampleShifting        = HAL_OSPI_SAMPLE_SHIFTING_NONE;
    opspi_config.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
    opspi_config.Init.Refresh               = 250;                                                         // 4us @100MHz
    opspi_config.Init.ChipSelectBoundary    = 23;                                                          // memory die boundary 2^23=8MBs
    opspi_config.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    HAL_OSPI_Init(&opspi_config);

    OSPI_HyperbusCfgTypeDef sHyperbusCfg;
    sHyperbusCfg.RWRecoveryTime   = RW_RECOVERY_TIME;
    sHyperbusCfg.AccessTime       = OPTIMAL_FIXED_INITIAL_LATENCY;
    sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
    sHyperbusCfg.LatencyMode      = HAL_OSPI_FIXED_LATENCY;
    HAL_OSPI_HyperbusCfg(&opspi_config, &sHyperbusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    
    S70KL1281_ReadCfgReg0(&opspi_config, &reg);                                                            // Reading the configuration of the HyperRAM
    SET_BIT(reg, S70KL1281_CR0_FLE);                                                                       // Latency Type
    MODIFY_REG(reg, (uint16_t) S70KL1281_CR0_IL, (uint16_t) OPTIMAL_FIXED_INITIAL_LATENCY_REG_VAL);
    SET_BIT(reg, S70KL1281_CR0_HBE);                                                                       // Burst type

    MODIFY_REG(reg, (uint16_t) S70KL1281_CR0_BLENGTH, (uint16_t) BSP_OSPI_RAM_BURST_32_BYTES);             // Burst length
    S70KL1281_WriteCfgReg0(&opspi_config, reg, HAL_OSPI_FIXED_LATENCY, DEFAULT_INITIAL_LATENCY);
    S70KL1281_EnableMemoryMappedMode(&opspi_config);
}

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disable the MPU */
    HAL_MPU_Disable();
    
    
    
    //
    //    /* Setup AXI SRAM, SRAM1 and SRAM2 in Write-back */
    //    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    //    MPU_InitStruct.BaseAddress      = D1_AXISRAM_BASE;
    //    MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
    //    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    //    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    //    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    //    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    //    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    //    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    //    MPU_InitStruct.SubRegionDisable = 0x00;
    //    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    //    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    //
    //    /* Setup SDRAM in SO */
    //    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    //    MPU_InitStruct.BaseAddress      = OCTOSPI2_BASE;
    //    MPU_InitStruct.Size             = MPU_REGION_SIZE_256MB;
    //    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    //    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    //    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    //    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    //    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
    //    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    //    MPU_InitStruct.SubRegionDisable = 0x00;
    //    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    //    HAL_MPU_ConfigRegion(&MPU_InitStruct);

        /* Setup SDRAM in Write-back (Buffers) */
    MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = OCTOSPI2_BASE;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_16MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
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
    return;
}
void Initialize_MEMS_Microphone_Onboard()
{
    return;
}

__attribute__((noreturn))
void nanoBooterStatus(uint32_t nanoBooterState)
{
    while (true)
    {
        switch ((eBooterStatus)nanoBooterState)
        {
        case ok:
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED, GPIO_PIN_SET);    // Off
            while(true)
            {
                HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_RESET);
                tx_thread_sleep(50);

                HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_SET);
                tx_thread_sleep(50);
            }
            break;
        case communications_failure:
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE, GPIO_PIN_SET);    // Off
            while(true)
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


//
// #define  TICK_INT_PRIORITY            (0UL) /*!< tick interrupt priority */

//static TIM_HandleTypeDef htim6;
//
//// This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
//void TIM6_DAC_IRQHandler()
//{
//    HAL_IncTick();
//}
//
//
//// Configure TIM6 as a time base source with 1ms time base and dedicated tick interrupt priority.
//
//void  HAL_InitTick()
//{
//    RCC_ClkInitTypeDef    clkconfig;
//    uint32_t              uwTimclock, uwAPB1Prescaler;
//
//    uint32_t              uwPrescalerValue;
//    uint32_t              pFLatency;
//    /*Configure the TIM6 IRQ priority */
//    if (TickPriority < (1UL << __NVIC_PRIO_BITS))
//    {
//        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0U);
//
//        /* Enable the TIM6 global Interrupt */
//        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
//        uwTickPrio = TickPriority;
//    }
//    else
//    {
//        return HAL_ERROR;
//    }
//
//    /* Enable TIM6 clock */
//    __HAL_RCC_TIM6_CLK_ENABLE();
//
//    /* Get clock configuration */
//    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
//
//    /* Get APB1 prescaler */
//    uwAPB1Prescaler = clkconfig.APB1CLKDivider;
//    /* Compute TIM6 clock */
//    if (uwAPB1Prescaler == RCC_HCLK_DIV1)
//    {
//        uwTimclock = HAL_RCC_GetPCLK1Freq();
//    }
//    else
//    {
//        uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
//    }
//
//    /* Compute the prescaler value to have TIM6 counter clock equal to 1MHz */
//    uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);
//
//    /* Initialize TIM6 */
//    htim6.Instance = TIM6;
//
//    /* Initialize TIMx peripheral as follow:
//    + Period = [(TIM6CLK/1000) - 1]. to have a (1/1000) s time base.
//    + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
//    + ClockDivision = 0
//    + Counter direction = Up
//    */
//    htim6.Init.Period = (1000000U / 1000U) - 1U;
//    htim6.Init.Prescaler = uwPrescalerValue;
//    htim6.Init.ClockDivision = 0;
//    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
//
//    if (HAL_TIM_Base_Init(&htim6) == HAL_OK)
//    {
//        /* Start the TIM time Base generation in interrupt mode */
//        return HAL_TIM_Base_Start_IT(&htim6);
//    }
//
//    /* Return function status */
//    return HAL_ERROR;
//}
//
///**
//  * @brief  Suspend Tick increment.
//  * @note   Disable the tick increment by disabling TIM6 update interrupt.
//  * @param  None
//  * @retval None
//  */
//void HAL_SuspendTick(void)
//{
//    /* Disable TIM6 update Interrupt */
//    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
//}
//
///**
//  * @brief  Resume Tick increment.
//  * @note   Enable the tick increment by Enabling TIM6 update interrupt.
//  * @param  None
//  * @retval None
//  */
//void HAL_ResumeTick(void)
//{
//    /* Enable TIM6 Update interrupt */
//    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
//}


