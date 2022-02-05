//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "TargetFeatures.h"
#include "HyperbusRam.h"

void init2(void);

#define HAL_OSPI_FLAG_BUSY OCTOSPI_SR_BUSY /*!< Busy flag: operation is ongoing */

int32_t S70KL1281_ReadCfgReg0(OSPI_HandleTypeDef *Ctx, uint16_t *Value);
int32_t S70KL1281_WriteCfgReg0(OSPI_HandleTypeDef *Ctx, uint16_t Value, uint32_t LatencyMode, uint32_t InitialLatency);
int32_t S70KL1281_EnableMemoryMappedMode(OSPI_HandleTypeDef *Ctx);
HAL_StatusTypeDef HAL_OSPI_HyperbusCmd(OSPI_HandleTypeDef *hospi, OSPI_HyperbusCmdTypeDef *cmd, uint32_t Timeout);
HAL_StatusTypeDef HAL_OSPI_Init(OSPI_HandleTypeDef *hospi);
HAL_StatusTypeDef HAL_OSPI_MemoryMapped(OSPI_HandleTypeDef *hospi, OSPI_MemoryMappedTypeDef *cfg);
HAL_StatusTypeDef HAL_OSPI_MemoryMapped(OSPI_HandleTypeDef *hospi, OSPI_MemoryMappedTypeDef *cfg);
static HAL_StatusTypeDef OSPI_WaitFlagStateUntilTimeout(
    OSPI_HandleTypeDef *hospi,
    uint32_t Flag,
    FlagStatus State,
    uint32_t Tickstart,
    uint32_t Timeout);
HAL_StatusTypeDef HAL_OSPI_SetTimeout(OSPI_HandleTypeDef *hospi, uint32_t Timeout);
HAL_StatusTypeDef HAL_OSPI_HyperbusCfg(OSPI_HandleTypeDef *hospi, OSPI_HyperbusCfgTypeDef *cfg, uint32_t Timeout);
HAL_StatusTypeDef HAL_OSPI_Transmit(OSPI_HandleTypeDef *hospi, uint8_t *pData, uint32_t Timeout);
HAL_StatusTypeDef HAL_OSPI_Receive(OSPI_HandleTypeDef *hospi, uint8_t *pData, uint32_t Timeout);

void init2()
{
    //    OSPI_HandleTypeDef hospi_ram1[OSPI_RAM_INSTANCES_NUMBER] = {0};
    //
    //    MX_OSPI_InitTypeDef ospi_init;
    //
    //    //    // Fill config structure
    //    //    ospi_init.ClockPrescaler = 2U; /* OctoSPI clock = 200 MHz /
    //    ClockPrescaler = 100 MHz */
    //    //    ospi_init.MemorySize = (uint32_t)POSITION_VAL(S70KL1281_RAM_SIZE);
    //    //    ospi_init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
    //    //
    //    //    // STM32 OSPI Clock configuration
    //    //    MX_OSPI_ClockConfig(&hospi_ram1[0]);
    //    //
    //    //    // STM32 OSPI interface initialization
    //    //    MX_OSPI_RAM_Init(&hospi_ram1[0], &ospi_init);
    //    //
    //    //    // Configure the memory
    //    //    BSP_OSPI_RAM_ConfigHyperRAM(0, BSP_OSPI_RAM_FIXED_LATENCY,
    //    BSP_OSPI_RAM_LINEAR_BURST,
    //    //    BSP_OSPI_RAM_BURST_32_BYTES);
    //
    //    {
    //        int32_t ret;
    //        static MDMA_HandleTypeDef mdma_handle;
    //        GPIO_InitTypeDef GPIO_InitStruct;
    //        uint16_t reg;
    //
    //        //    OSPI_HandleTypeDef hospi_ram1[OSPI_RAM_INSTANCES_NUMBER] = { 0
    //        };
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
    //        GPIO_InitStruct.Pin = OSPI_RAM_D0_PIN | OSPI_RAM_D1_PIN |
    //        OSPI_RAM_D2_PIN | OSPI_RAM_D3_PIN; GPIO_InitStruct.Alternate =
    //        GPIO_AF9_OCTOSPIM_P2; HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    //
    //        // OctoSPI D4 GPIO pin configuration
    //        GPIO_InitStruct.Pin = OSPI_RAM_D4_PIN | OSPI_RAM_D5_PIN |
    //        OSPI_RAM_D7_PIN; GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
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
    //        hospi_ram1[0].Init.DeviceSize =
    //        (uint32_t)POSITION_VAL(S70KL1281_RAM_SIZE);
    //        hospi_ram1[0].Init.ChipSelectHighTime = 4;
    //        hospi_ram1[0].Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
    //        hospi_ram1[0].Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
    //        hospi_ram1[0].Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    //        hospi_ram1[0].Init.ClockPrescaler = 2U; // OctoSPI clock = 200 MHz /
    //        ClockPrescaler = 100 MHz hospi_ram1[0].Init.SampleShifting =
    //        HAL_OSPI_SAMPLE_SHIFTING_NONE;
    //        hospi_ram1[0].Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
    //        hospi_ram1[0].Init.Refresh = 400;           // 4us @100MHz
    //        hospi_ram1[0].Init.ChipSelectBoundary = 23; // memory die boundary
    //        2^23=8MBs hospi_ram1[0].Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    //
    //        HAL_OSPI_Init(&hospi_ram1[0]);
    //
    //        OSPI_HyperbusCfgTypeDef sHyperbusCfg;
    //        sHyperbusCfg.RWRecoveryTime = RW_RECOVERY_TIME;
    //        sHyperbusCfg.AccessTime = DEFAULT_INITIAL_LATENCY;
    //        sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
    //        sHyperbusCfg.LatencyMode = HAL_OSPI_FIXED_LATENCY;
    //
    //        HAL_OSPI_HyperbusCfg(&hospi_ram1[0], &sHyperbusCfg,
    //        HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    //
    //        S70KL1281_ReadCfgReg0(&hospi_ram1[0],
    //                              &reg);     // Reading the configuration of the
    //                              HyperRAM
    //        SET_BIT(reg, S70KL1281_CR0_FLE); // Latency Type
    //        MODIFY_REG(reg, (uint16_t)S70KL1281_CR0_IL,
    //        (uint16_t)OPTIMAL_FIXED_INITIAL_LATENCY_REG_VAL); SET_BIT(reg,
    //        S70KL1281_CR0_HBE); // Burst type
    //
    //        MODIFY_REG(reg, (uint16_t)S70KL1281_CR0_BLENGTH,
    //                   (uint16_t)BSP_OSPI_RAM_BURST_32_BYTES); // Burst length
    //        S70KL1281_WriteCfgReg0(&hospi_ram1[0], reg, HAL_OSPI_FIXED_LATENCY,
    //        DEFAULT_INITIAL_LATENCY); HAL_OSPI_Init(&hospi_ram1[0]);
    //        sHyperbusCfg.RWRecoveryTime = RW_RECOVERY_TIME;
    //        sHyperbusCfg.AccessTime = OPTIMAL_FIXED_INITIAL_LATENCY;
    //        sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
    //        sHyperbusCfg.LatencyMode = (uint32_t)BSP_OSPI_RAM_FIXED_LATENCY;
    //
    //        HAL_OSPI_HyperbusCfg(&hospi_ram1[0], &sHyperbusCfg,
    //        HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    //
    //        // The memory access can be configured in memory mapped mode
    //        S70KL1281_EnableMemoryMappedMode(&hospi_ram1[0]);
}

void Initialize_OPSPI_Hyperam()
{
    int32_t ret;
    static MDMA_HandleTypeDef mdma_handle;
    GPIO_InitTypeDef GPIO_InitStruct;
    uint16_t reg;

    OSPI_HandleTypeDef opspi_config = {0};

    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_OSPI2);
    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_MDMA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF); //  Ports F and G have
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG); //  pins for the OctoSPI[2]
    LL_AHB3_GRP1_ForceReset(LL_AHB3_GRP1_PERIPH_OSPI2);  // Reset the OctoSPI memory interface
    LL_AHB3_GRP1_ReleaseReset(LL_AHB3_GRP1_PERIPH_OSPI2);

    // OctoSPI CS GPIO pin configuration
    GPIO_InitStruct.Pin = OSPI_RAM_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // OctoSPI DQS GPIO pin configuration
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = OSPI_RAM_DQS_PIN;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // OctoSPI CLK GPIO pin configuration
    GPIO_InitStruct.Pin = OSPI_RAM_CLK_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // OctoSPI D0,D1,D2,D3 GPIO pin configuration
    GPIO_InitStruct.Pin = OSPI_RAM_D0_PIN | OSPI_RAM_D1_PIN | OSPI_RAM_D2_PIN | OSPI_RAM_D3_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // OctoSPI D4,D5,D7 GPIO pin configuration
    GPIO_InitStruct.Pin = OSPI_RAM_D4_PIN | OSPI_RAM_D5_PIN | OSPI_RAM_D7_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // OctoSPI D6 GPIO pin configuration
    GPIO_InitStruct.Pin = OSPI_RAM_D6_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPIM_P2;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // HyperRAM clock configuration [ OPSPI2 ]
    //  PLL2_VCO Input = HSE_VALUE/PLL3M = 5 Mhz
    //  PLL2_VCO Output = PLL2_VCO Input * PLL3N = 400 Mhz
    //  PLLOSPICLK = PLL2_VCO Output/PLL2R = 400/2 = 200 Mhz
    //  OSPI clock frequency = PLLOSPICLK = 200 Mhz

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
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
    opspi_config.Init.FifoThreshold = 4;
    opspi_config.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
    opspi_config.Init.MemoryType = HAL_OSPI_MEMTYPE_HYPERBUS;
    opspi_config.Init.DeviceSize = (uint32_t)POSITION_VAL(S70KL1281_RAM_SIZE);
    opspi_config.Init.ChipSelectHighTime = 4;
    opspi_config.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
    opspi_config.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
    opspi_config.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    opspi_config.Init.ClockPrescaler = 4U; // OctoSPI clock = 200 MHz / ClockPrescaler = 100 MHz
    opspi_config.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
    opspi_config.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
    opspi_config.Init.Refresh = 250;           // 4us @100MHz
    opspi_config.Init.ChipSelectBoundary = 23; // memory die boundary 2^23=8MBs
    opspi_config.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    HAL_OSPI_Init(&opspi_config);

    OSPI_HyperbusCfgTypeDef sHyperbusCfg;
    sHyperbusCfg.RWRecoveryTime = RW_RECOVERY_TIME;
    sHyperbusCfg.AccessTime = OPTIMAL_FIXED_INITIAL_LATENCY;
    sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
    sHyperbusCfg.LatencyMode = HAL_OSPI_FIXED_LATENCY;
    HAL_OSPI_HyperbusCfg(&opspi_config, &sHyperbusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    S70KL1281_ReadCfgReg0(&opspi_config,
                          &reg);     // Reading the configuration of the HyperRAM
    SET_BIT(reg, S70KL1281_CR0_FLE); // Latency Type
    MODIFY_REG(reg, (uint16_t)S70KL1281_CR0_IL, (uint16_t)OPTIMAL_FIXED_INITIAL_LATENCY_REG_VAL);
    SET_BIT(reg, S70KL1281_CR0_HBE); // Burst type

    MODIFY_REG(reg, (uint16_t)S70KL1281_CR0_BLENGTH,
               (uint16_t)BSP_OSPI_RAM_BURST_32_BYTES); // Burst length
    S70KL1281_WriteCfgReg0(&opspi_config, reg, HAL_OSPI_FIXED_LATENCY, DEFAULT_INITIAL_LATENCY);
    S70KL1281_EnableMemoryMappedMode(&opspi_config);
}

int32_t S70KL1281_ReadCfgReg0(OSPI_HandleTypeDef *Ctx, uint16_t *Value)
{
    OSPI_HyperbusCmdTypeDef sCommand;

    /* Initialize the read command */
    sCommand.AddressSpace = HAL_OSPI_REGISTER_ADDRESS_SPACE;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
    sCommand.Address = S70KL1281_CR0_ADDRESS;
    sCommand.DQSMode = HAL_OSPI_DQS_ENABLE;
    sCommand.NbData = 2U;

    /* Configure the command */
    if (HAL_OSPI_HyperbusCmd(Ctx, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return S70KL1281_ERROR;
    }

    /* Reception of the data */
    if (HAL_OSPI_Receive(Ctx, (uint8_t *)Value, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return S70KL1281_ERROR;
    }

    return S70KL1281_OK;
}
int32_t S70KL1281_WriteCfgReg0(OSPI_HandleTypeDef *Ctx, uint16_t Value, uint32_t LatencyMode, uint32_t InitialLatency)
{
    OSPI_HyperbusCfgTypeDef sHyperbusCfg;
    OSPI_HyperbusCmdTypeDef sCommand;

    /* Reconfigure peripheral as no write latency to write in registers */
    sHyperbusCfg.RWRecoveryTime = RW_RECOVERY_TIME;
    sHyperbusCfg.AccessTime = InitialLatency;
    sHyperbusCfg.WriteZeroLatency = HAL_OSPI_NO_LATENCY_ON_WRITE;
    sHyperbusCfg.LatencyMode = LatencyMode;

    if (HAL_OSPI_HyperbusCfg(Ctx, &sHyperbusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return S70KL1281_ERROR;
    }

    /* Initialize the write command */
    sCommand.AddressSpace = HAL_OSPI_REGISTER_ADDRESS_SPACE;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
    sCommand.Address = S70KL1281_CR0_ADDRESS;
    sCommand.DQSMode = HAL_OSPI_DQS_ENABLE;
    sCommand.NbData = 2U;

    /* Configure the command */
    if (HAL_OSPI_HyperbusCmd(Ctx, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return S70KL1281_ERROR;
    }

    /* Transmission of the data */
    if (HAL_OSPI_Transmit(Ctx, (uint8_t *)(&Value), HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return S70KL1281_ERROR;
    }

    /* Reconfigure peripheral for correct write access */
    sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;

    if (HAL_OSPI_HyperbusCfg(Ctx, &sHyperbusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return S70KL1281_ERROR;
    }

    return S70KL1281_OK;
}
int32_t S70KL1281_EnableMemoryMappedMode(OSPI_HandleTypeDef *Ctx)
{
    OSPI_HyperbusCmdTypeDef sCommand;
    OSPI_MemoryMappedTypeDef sMemMappedCfg;

    /* OctoSPI Hyperbus command configuration */
    sCommand.AddressSpace = HAL_OSPI_MEMORY_ADDRESS_SPACE;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
    sCommand.Address = 0;
    sCommand.DQSMode = HAL_OSPI_DQS_ENABLE;
    sCommand.NbData = 1;

    if (HAL_OSPI_HyperbusCmd(Ctx, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return S70KL1281_ERROR;
    }

    /* OctoSPI activation of memory-mapped mode */
    sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

    if (HAL_OSPI_MemoryMapped(Ctx, &sMemMappedCfg) != HAL_OK)
    {
        return S70KL1281_ERROR;
    }

    return S70KL1281_OK;
}
HAL_StatusTypeDef HAL_OSPI_HyperbusCmd(OSPI_HandleTypeDef *hospi, OSPI_HyperbusCmdTypeDef *cmd, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();

    /* Check the parameters of the hyperbus command structure */
    assert_param(IS_OSPI_ADDRESS_SPACE(cmd->AddressSpace));
    assert_param(IS_OSPI_ADDRESS_SIZE(cmd->AddressSize));
    assert_param(IS_OSPI_NUMBER_DATA(cmd->NbData));
    assert_param(IS_OSPI_DQS_MODE(cmd->DQSMode));

    /* Check the state of the driver */
    if ((hospi->State == HAL_OSPI_STATE_READY) && (hospi->Init.MemoryType == HAL_OSPI_MEMTYPE_HYPERBUS))
    {
        /* Wait till busy flag is reset */
        status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, tickstart, Timeout);

        if (status == HAL_OK)
        {
            /* Re-initialize the value of the functional mode */
            MODIFY_REG(hospi->Instance->CR, OCTOSPI_CR_FMODE, 0U);

            /* Configure the address space in the DCR1 register */
            MODIFY_REG(hospi->Instance->DCR1, OCTOSPI_DCR1_MTYP_0, cmd->AddressSpace);

            /* Configure the CCR and WCCR registers with the address size and the
               following configuration :
               - DQS signal enabled (used as RWDS)
               - DTR mode enabled on address and data
               - address and data on 8 lines */
            WRITE_REG(
                hospi->Instance->CCR,
                (cmd->DQSMode | OCTOSPI_CCR_DDTR | OCTOSPI_CCR_DMODE_2 | cmd->AddressSize | OCTOSPI_CCR_ADDTR |
                 OCTOSPI_CCR_ADMODE_2));
            WRITE_REG(
                hospi->Instance->WCCR,
                (cmd->DQSMode | OCTOSPI_WCCR_DDTR | OCTOSPI_WCCR_DMODE_2 | cmd->AddressSize | OCTOSPI_WCCR_ADDTR |
                 OCTOSPI_WCCR_ADMODE_2));

            /* Configure the DLR register with the number of data */
            WRITE_REG(hospi->Instance->DLR, (cmd->NbData - 1U));

            /* Configure the AR register with the address value */
            WRITE_REG(hospi->Instance->AR, cmd->Address);

            /* Update the state */
            hospi->State = HAL_OSPI_STATE_CMD_CFG;
        }
    }
    else
    {
        status = HAL_ERROR;
        hospi->ErrorCode = HAL_OSPI_ERROR_INVALID_SEQUENCE;
    }

    /* Return function status */
    return status;
}
HAL_StatusTypeDef HAL_OSPI_Init(OSPI_HandleTypeDef *hospi)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t tickstart = HAL_GetTick();

    /* Check the OSPI handle allocation */
    if (hospi == NULL)
    {
        status = HAL_ERROR;
        /* No error code can be set set as the handler is null */
    }
    else
    {
        /* Check the parameters of the initialization structure */
        assert_param(IS_OSPI_FIFO_THRESHOLD(hospi->Init.FifoThreshold));
        assert_param(IS_OSPI_DUALQUAD_MODE(hospi->Init.DualQuad));
        assert_param(IS_OSPI_MEMORY_TYPE(hospi->Init.MemoryType));
        assert_param(IS_OSPI_DEVICE_SIZE(hospi->Init.DeviceSize));
        assert_param(IS_OSPI_CS_HIGH_TIME(hospi->Init.ChipSelectHighTime));
        assert_param(IS_OSPI_FREE_RUN_CLK(hospi->Init.FreeRunningClock));
        assert_param(IS_OSPI_CLOCK_MODE(hospi->Init.ClockMode));
        assert_param(IS_OSPI_WRAP_SIZE(hospi->Init.WrapSize));
        assert_param(IS_OSPI_CLK_PRESCALER(hospi->Init.ClockPrescaler));
        assert_param(IS_OSPI_SAMPLE_SHIFTING(hospi->Init.SampleShifting));
        assert_param(IS_OSPI_DHQC(hospi->Init.DelayHoldQuarterCycle));
        assert_param(IS_OSPI_CS_BOUNDARY(hospi->Init.ChipSelectBoundary));
        assert_param(IS_OSPI_CKCSHT(hospi->Init.ClkChipSelectHighTime));
        assert_param(IS_OSPI_DLYBYP(hospi->Init.DelayBlockBypass));
        assert_param(IS_OSPI_MAXTRAN(hospi->Init.MaxTran));

        /* Initialize error code */
        hospi->ErrorCode = HAL_OSPI_ERROR_NONE;

        /* Check if the state is the reset state */
        if (hospi->State == HAL_OSPI_STATE_RESET)
        {

            /* Configure the default timeout for the OSPI memory access */
            (void)HAL_OSPI_SetTimeout(hospi, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

            /* Configure memory type, device size, chip select high time, clocked chip
             * select high time, delay block bypass, free running clock, clock mode */
            MODIFY_REG(
                hospi->Instance->DCR1,
                (OCTOSPI_DCR1_MTYP | OCTOSPI_DCR1_DEVSIZE | OCTOSPI_DCR1_CSHT | OCTOSPI_DCR1_CKCSHT |
                 OCTOSPI_DCR1_DLYBYP | OCTOSPI_DCR1_FRCK | OCTOSPI_DCR1_CKMODE),
                (hospi->Init.MemoryType | ((hospi->Init.DeviceSize - 1U) << OCTOSPI_DCR1_DEVSIZE_Pos) |
                 ((hospi->Init.ChipSelectHighTime - 1U) << OCTOSPI_DCR1_CSHT_Pos) |
                 (hospi->Init.ClkChipSelectHighTime << OCTOSPI_DCR1_CKCSHT_Pos) | hospi->Init.DelayBlockBypass |
                 hospi->Init.ClockMode));

            /* Configure wrap size */
            MODIFY_REG(hospi->Instance->DCR2, OCTOSPI_DCR2_WRAPSIZE, hospi->Init.WrapSize);

            /* Configure chip select boundary and maximun transfer */
            hospi->Instance->DCR3 =
                ((hospi->Init.ChipSelectBoundary << OCTOSPI_DCR3_CSBOUND_Pos) |
                 (hospi->Init.MaxTran << OCTOSPI_DCR3_MAXTRAN_Pos));

            /* Configure refresh */
            hospi->Instance->DCR4 = hospi->Init.Refresh;

            /* Configure FIFO threshold */
            MODIFY_REG(
                hospi->Instance->CR,
                OCTOSPI_CR_FTHRES,
                ((hospi->Init.FifoThreshold - 1U) << OCTOSPI_CR_FTHRES_Pos));

            /* Wait till busy flag is reset */
            status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, tickstart, hospi->Timeout);

            if (status == HAL_OK)
            {
                /* Configure clock prescaler */
                MODIFY_REG(
                    hospi->Instance->DCR2,
                    OCTOSPI_DCR2_PRESCALER,
                    ((hospi->Init.ClockPrescaler - 1U) << OCTOSPI_DCR2_PRESCALER_Pos));

                /* Configure Dual Quad mode */
                MODIFY_REG(hospi->Instance->CR, OCTOSPI_CR_DQM, hospi->Init.DualQuad);

                /* Configure sample shifting and delay hold quarter cycle */
                MODIFY_REG(
                    hospi->Instance->TCR,
                    (OCTOSPI_TCR_SSHIFT | OCTOSPI_TCR_DHQC),
                    (hospi->Init.SampleShifting | hospi->Init.DelayHoldQuarterCycle));

                /* Enable OctoSPI */
                __HAL_OSPI_ENABLE(hospi);

                /* Enable free running clock if needed : must be done after OSPI enable
                 */

                if (hospi->Init.FreeRunningClock == HAL_OSPI_FREERUNCLK_ENABLE)
                {
                    SET_BIT(hospi->Instance->DCR1, OCTOSPI_DCR1_FRCK);
                }

                /* Initialize the OSPI state */
                if (hospi->Init.MemoryType == HAL_OSPI_MEMTYPE_HYPERBUS)
                {
                    hospi->State = HAL_OSPI_STATE_HYPERBUS_INIT;
                }
                else
                {
                    hospi->State = HAL_OSPI_STATE_READY;
                }
            }
        }
    }

    /* Return function status */
    return status;
}
HAL_StatusTypeDef HAL_OSPI_MemoryMapped(OSPI_HandleTypeDef *hospi, OSPI_MemoryMappedTypeDef *cfg)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();

    /* Check the parameters of the memory-mapped configuration structure */
    assert_param(IS_OSPI_TIMEOUT_ACTIVATION(cfg->TimeOutActivation));

    /* Check the state */
    if (hospi->State == HAL_OSPI_STATE_CMD_CFG)
    {
        /* Wait till busy flag is reset */
        status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, tickstart, hospi->Timeout);

        if (status == HAL_OK)
        {
            /* Update state */
            hospi->State = HAL_OSPI_STATE_BUSY_MEM_MAPPED;

            if (cfg->TimeOutActivation == HAL_OSPI_TIMEOUT_COUNTER_ENABLE)
            {
                assert_param(IS_OSPI_TIMEOUT_PERIOD(cfg->TimeOutPeriod));

                /* Configure register */
                WRITE_REG(hospi->Instance->LPTR, cfg->TimeOutPeriod);

                /* Clear flags related to interrupt */
                __HAL_OSPI_CLEAR_FLAG(hospi, HAL_OSPI_FLAG_TO);

                /* Enable the timeout interrupt */
                __HAL_OSPI_ENABLE_IT(hospi, HAL_OSPI_IT_TO);
            }

            /* Configure CR register with functional mode as memory-mapped */
            MODIFY_REG(
                hospi->Instance->CR,
                (OCTOSPI_CR_TCEN | OCTOSPI_CR_FMODE),
                (cfg->TimeOutActivation | OSPI_FUNCTIONAL_MODE_MEMORY_MAPPED));
        }
    }
    else
    {
        status = HAL_ERROR;
        hospi->ErrorCode = HAL_OSPI_ERROR_INVALID_SEQUENCE;
    }

    /* Return function status */
    return status;
}
HAL_StatusTypeDef HAL_OSPI_HyperbusCfg(OSPI_HandleTypeDef *hospi, OSPI_HyperbusCfgTypeDef *cfg, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t state;
    uint32_t tickstart = HAL_GetTick();

    /* Check the parameters of the hyperbus configuration structure */
    assert_param(IS_OSPI_RW_RECOVERY_TIME(cfg->RWRecoveryTime));
    assert_param(IS_OSPI_ACCESS_TIME(cfg->AccessTime));
    assert_param(IS_OSPI_WRITE_ZERO_LATENCY(cfg->WriteZeroLatency));
    assert_param(IS_OSPI_LATENCY_MODE(cfg->LatencyMode));

    /* Check the state of the driver */
    state = hospi->State;
    if ((state == HAL_OSPI_STATE_HYPERBUS_INIT) || (state == HAL_OSPI_STATE_READY))
    {
        /* Wait till busy flag is reset */
        status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, tickstart, Timeout);

        if (status == HAL_OK)
        {
            /* Configure Hyperbus configuration Latency register */
            WRITE_REG(
                hospi->Instance->HLCR,
                ((cfg->RWRecoveryTime << OCTOSPI_HLCR_TRWR_Pos) | (cfg->AccessTime << OCTOSPI_HLCR_TACC_Pos) |
                 cfg->WriteZeroLatency | cfg->LatencyMode));

            /* Update the state */
            hospi->State = HAL_OSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_ERROR;
        hospi->ErrorCode = HAL_OSPI_ERROR_INVALID_SEQUENCE;
    }

    /* Return function status */
    return status;
}
static HAL_StatusTypeDef OSPI_WaitFlagStateUntilTimeout(
    OSPI_HandleTypeDef *hospi,
    uint32_t Flag,
    FlagStatus State,
    uint32_t Tickstart,
    uint32_t Timeout)
{
    /* Wait until flag is in expected state */
    while ((__HAL_OSPI_GET_FLAG(hospi, Flag)) != State)
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                hospi->State = HAL_OSPI_STATE_ERROR;
                hospi->ErrorCode |= HAL_OSPI_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_OSPI_SetTimeout(OSPI_HandleTypeDef *hospi, uint32_t Timeout)
{
    hospi->Timeout = Timeout;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_OSPI_Transmit(OSPI_HandleTypeDef *hospi, uint8_t *pData, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();
    __IO uint32_t *data_reg = &hospi->Instance->DR;

    /* Check the data pointer allocation */
    if (pData == NULL)
    {
        status = HAL_ERROR;
        hospi->ErrorCode = HAL_OSPI_ERROR_INVALID_PARAM;
    }
    else
    {
        /* Check the state */
        if (hospi->State == HAL_OSPI_STATE_CMD_CFG)
        {
            /* Configure counters and size */
            hospi->XferCount = READ_REG(hospi->Instance->DLR) + 1U;
            hospi->XferSize = hospi->XferCount;
            hospi->pBuffPtr = pData;

            /* Configure CR register with functional mode as indirect write */
            MODIFY_REG(hospi->Instance->CR, OCTOSPI_CR_FMODE, OSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

            do
            {
                /* Wait till fifo threshold flag is set to send data */
                status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_FT, SET, tickstart, Timeout);

                if (status != HAL_OK)
                {
                    break;
                }

                *((__IO uint8_t *)data_reg) = *hospi->pBuffPtr;
                hospi->pBuffPtr++;
                hospi->XferCount--;
            } while (hospi->XferCount > 0U);

            if (status == HAL_OK)
            {
                /* Wait till transfer complete flag is set to go back in idle state */
                status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_TC, SET, tickstart, Timeout);

                if (status == HAL_OK)
                {
                    /* Clear transfer complete flag */
                    __HAL_OSPI_CLEAR_FLAG(hospi, HAL_OSPI_FLAG_TC);

                    /* Update state */
                    hospi->State = HAL_OSPI_STATE_READY;
                }
            }
        }
        else
        {
            status = HAL_ERROR;
            hospi->ErrorCode = HAL_OSPI_ERROR_INVALID_SEQUENCE;
        }
    }

    /* Return function status */
    return status;
}
HAL_StatusTypeDef HAL_OSPI_Receive(OSPI_HandleTypeDef *hospi, uint8_t *pData, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();
    __IO uint32_t *data_reg = &hospi->Instance->DR;
    uint32_t addr_reg = hospi->Instance->AR;
    uint32_t ir_reg = hospi->Instance->IR;

    /* Check the data pointer allocation */
    if (pData == NULL)
    {
        status = HAL_ERROR;
        hospi->ErrorCode = HAL_OSPI_ERROR_INVALID_PARAM;
    }
    else
    {
        /* Check the state */
        if (hospi->State == HAL_OSPI_STATE_CMD_CFG)
        {
            /* Configure counters and size */
            hospi->XferCount = READ_REG(hospi->Instance->DLR) + 1U;
            hospi->XferSize = hospi->XferCount;
            hospi->pBuffPtr = pData;

            /* Configure CR register with functional mode as indirect read */
            MODIFY_REG(hospi->Instance->CR, OCTOSPI_CR_FMODE, OSPI_FUNCTIONAL_MODE_INDIRECT_READ);

            /* Trig the transfer by re-writing address or instruction register */
            if (hospi->Init.MemoryType == HAL_OSPI_MEMTYPE_HYPERBUS)
            {
                WRITE_REG(hospi->Instance->AR, addr_reg);
            }
            else
            {
                if (READ_BIT(hospi->Instance->CCR, OCTOSPI_CCR_ADMODE) != HAL_OSPI_ADDRESS_NONE)
                {
                    WRITE_REG(hospi->Instance->AR, addr_reg);
                }
                else
                {
                    WRITE_REG(hospi->Instance->IR, ir_reg);
                }
            }

            do
            {
                /* Wait till fifo threshold or transfer complete flags are set to read
                 * received data */
                status = OSPI_WaitFlagStateUntilTimeout(
                    hospi,
                    (HAL_OSPI_FLAG_FT | HAL_OSPI_FLAG_TC),
                    SET,
                    tickstart,
                    Timeout);

                if (status != HAL_OK)
                {
                    break;
                }

                *hospi->pBuffPtr = *((__IO uint8_t *)data_reg);
                hospi->pBuffPtr++;
                hospi->XferCount--;
            } while (hospi->XferCount > 0U);

            if (status == HAL_OK)
            {
                /* Wait till transfer complete flag is set to go back in idle state */
                status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_TC, SET, tickstart, Timeout);

                if (status == HAL_OK)
                {
                    /* Clear transfer complete flag */
                    __HAL_OSPI_CLEAR_FLAG(hospi, HAL_OSPI_FLAG_TC);

                    /* Update state */
                    hospi->State = HAL_OSPI_STATE_READY;
                }
            }
        }
        else
        {
            status = HAL_ERROR;
            hospi->ErrorCode = HAL_OSPI_ERROR_INVALID_SEQUENCE;
        }
    }

    /* Return function status */
    return status;
}
