#include "stm32h7xx.h"
#include <stm32h7xx_hal_mdma.h>
#include <stm32h7xx_hal_ospi.h>
#include <stm32h735g_discovery_ospi.h>
#include <stm32h7xx_hal_gpio.h>

void init2()
{
//    OSPI_HandleTypeDef hospi_ram1[OSPI_RAM_INSTANCES_NUMBER] = {0};
//
//    MX_OSPI_InitTypeDef ospi_init;
//
//    //    // Fill config structure
//    //    ospi_init.ClockPrescaler = 2U; /* OctoSPI clock = 200 MHz /
//    //    ClockPrescaler = 100 MHz */ ospi_init.MemorySize =
//    //    (uint32_t)POSITION_VAL(S70KL1281_RAM_SIZE); ospi_init.SampleShifting =
//    //    HAL_OSPI_SAMPLE_SHIFTING_NONE;
//    //
//    //    // STM32 OSPI Clock configuration
//    //    MX_OSPI_ClockConfig(&hospi_ram1[0]);
//    //
//    //    // STM32 OSPI interface initialization
//    //    MX_OSPI_RAM_Init(&hospi_ram1[0], &ospi_init);
//    //
//    //    // Configure the memory
//    //    BSP_OSPI_RAM_ConfigHyperRAM(0, BSP_OSPI_RAM_FIXED_LATENCY,
//    //    BSP_OSPI_RAM_LINEAR_BURST, BSP_OSPI_RAM_BURST_32_BYTES);
//
//    {
//        int32_t ret;
//        static MDMA_HandleTypeDef mdma_handle;
//        GPIO_InitTypeDef GPIO_InitStruct;
//        uint16_t reg;
//
//        //    OSPI_HandleTypeDef hospi_ram1[OSPI_RAM_INSTANCES_NUMBER] = { 0 };
//
//        // Enable the OctoSPI memory interface, DMA and GPIO clocks
//        __HAL_RCC_OSPI2_CLK_ENABLE();
//        __HAL_RCC_MDMA_CLK_ENABLE();
//
//        __HAL_RCC_GPIOF_CLK_ENABLE(); // Ports F and G have pins for the
//                                      // OctoSPI[2]
//        __HAL_RCC_GPIOG_CLK_ENABLE();
//
//        __HAL_RCC_OSPI2_FORCE_RESET(); // Reset the OctoSPI memory interface
//        __HAL_RCC_OSPI2_RELEASE_RESET();
//
//        // OctoSPI CS GPIO pin configuration
//        GPIO_InitStruct.Pin = OSPI_RAM_CS_PIN;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_PULLUP;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//        GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPIM_P2;
//        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//
//        // OctoSPI DQS GPIO pin configuration
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Pin = OSPI_RAM_DQS_PIN;
//        GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
//        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//
//        // OctoSPI CLK GPIO pin configuration
//        GPIO_InitStruct.Pin = OSPI_RAM_CLK_PIN;
//        GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
//        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//
//        // OctoSPI D1 GPIO pin configuration
//        GPIO_InitStruct.Pin = OSPI_RAM_D0_PIN | OSPI_RAM_D1_PIN | OSPI_RAM_D2_PIN | OSPI_RAM_D3_PIN;
//        GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
//        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//
//        // OctoSPI D4 GPIO pin configuration
//        GPIO_InitStruct.Pin = OSPI_RAM_D4_PIN | OSPI_RAM_D5_PIN | OSPI_RAM_D7_PIN;
//        GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
//        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//
//        // OctoSPI D6 GPIO pin configuration
//        GPIO_InitStruct.Pin = OSPI_RAM_D6_PIN;
//        GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPIM_P2;
//        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//
//        // Configure the OctoSPI DMA
//        mdma_handle.Init.Request = MDMA_REQUEST_SW;
//        mdma_handle.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
//        mdma_handle.Init.Priority = MDMA_PRIORITY_HIGH;
//        mdma_handle.Init.SourceInc = MDMA_SRC_INC_WORD;
//        mdma_handle.Init.DestinationInc = MDMA_DEST_INC_WORD;
//        mdma_handle.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
//        mdma_handle.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
//        mdma_handle.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
//        mdma_handle.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
//        mdma_handle.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
//        mdma_handle.Init.BufferTransferLength = 128;
//        mdma_handle.Init.SourceBlockAddressOffset = 0;
//        mdma_handle.Init.DestBlockAddressOffset = 0;
//
//        mdma_handle.Instance = MDMA_Channel0;
//
//        __HAL_LINKDMA(&hospi_ram1[0], hmdma, mdma_handle);
//        (void)HAL_MDMA_Init(&mdma_handle);
//
//        /* Enable and set priority of the OctoSPI and DMA interrupts */
//        HAL_NVIC_SetPriority(OCTOSPI2_IRQn, BSP_OSPI_RAM_IT_PRIORITY, 0);
//        HAL_NVIC_SetPriority(MDMA_IRQn, BSP_OSPI_RAM_DMA_IT_PRIORITY, 0);
//
//        HAL_NVIC_EnableIRQ(OCTOSPI2_IRQn);
//        HAL_NVIC_EnableIRQ(MDMA_IRQn);
//
//        /* HyperRAM clock configuration [ OPSPI2 ] */
//        /*      PLL2_VCO Input = HSE_VALUE/PLL3M = 5 Mhz */
//        /*      PLL2_VCO Output = PLL2_VCO Input * PLL3N = 400 Mhz */
//        /*      PLLOSPICLK = PLL2_VCO Output/PLL2R = 400/2 = 200 Mhz */
//        /*      OSPI clock frequency = PLLOSPICLK = 200 Mhz */
//
//        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
//        PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
//        PeriphClkInitStruct.PLL2.PLL2M = 5;
//        PeriphClkInitStruct.PLL2.PLL2N = 80;
//        PeriphClkInitStruct.PLL2.PLL2P = 5;
//        PeriphClkInitStruct.PLL2.PLL2Q = 2;
//        PeriphClkInitStruct.PLL2.PLL2R = 2;
//        PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
//        PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
//        PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
//        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
//
//        HAL_StatusTypeDef status;
//        hospi_ram1[0].Instance = OCTOSPI2;
//        hospi_ram1[0].Init.FifoThreshold = 4;
//        hospi_ram1[0].Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
//        hospi_ram1[0].Init.MemoryType = HAL_OSPI_MEMTYPE_HYPERBUS;
//        hospi_ram1[0].Init.DeviceSize = (uint32_t)POSITION_VAL(S70KL1281_RAM_SIZE);
//        hospi_ram1[0].Init.ChipSelectHighTime = 4;
//        hospi_ram1[0].Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
//        hospi_ram1[0].Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
//        hospi_ram1[0].Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
//        hospi_ram1[0].Init.ClockPrescaler = 2U; // OctoSPI clock = 200 MHz / ClockPrescaler = 100 MHz
//        hospi_ram1[0].Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
//        hospi_ram1[0].Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
//        hospi_ram1[0].Init.Refresh = 400;           // 4us @100MHz
//        hospi_ram1[0].Init.ChipSelectBoundary = 23; // memory die boundary 2^23=8MBs
//        hospi_ram1[0].Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
//
//        HAL_OSPI_Init(&hospi_ram1[0]);
//
//        OSPI_HyperbusCfgTypeDef sHyperbusCfg;
//        sHyperbusCfg.RWRecoveryTime = RW_RECOVERY_TIME;
//        sHyperbusCfg.AccessTime = DEFAULT_INITIAL_LATENCY;
//        sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
//        sHyperbusCfg.LatencyMode = HAL_OSPI_FIXED_LATENCY;
//
//        HAL_OSPI_HyperbusCfg(&hospi_ram1[0], &sHyperbusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
//
//        S70KL1281_ReadCfgReg0(&hospi_ram1[0],
//                              &reg);     // Reading the configuration of the HyperRAM
//        SET_BIT(reg, S70KL1281_CR0_FLE); // Latency Type
//        MODIFY_REG(reg, (uint16_t)S70KL1281_CR0_IL, (uint16_t)OPTIMAL_FIXED_INITIAL_LATENCY_REG_VAL);
//        SET_BIT(reg, S70KL1281_CR0_HBE); // Burst type
//
//        MODIFY_REG(reg, (uint16_t)S70KL1281_CR0_BLENGTH,
//                   (uint16_t)BSP_OSPI_RAM_BURST_32_BYTES); // Burst length
//        S70KL1281_WriteCfgReg0(&hospi_ram1[0], reg, HAL_OSPI_FIXED_LATENCY, DEFAULT_INITIAL_LATENCY);
//        HAL_OSPI_Init(&hospi_ram1[0]);
//        sHyperbusCfg.RWRecoveryTime = RW_RECOVERY_TIME;
//        sHyperbusCfg.AccessTime = OPTIMAL_FIXED_INITIAL_LATENCY;
//        sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
//        sHyperbusCfg.LatencyMode = (uint32_t)BSP_OSPI_RAM_FIXED_LATENCY;
//
//        HAL_OSPI_HyperbusCfg(&hospi_ram1[0], &sHyperbusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
//
//        // The memory access can be configured in memory mapped mode
//        S70KL1281_EnableMemoryMappedMode(&hospi_ram1[0]);
//    }
}
