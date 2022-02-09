//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "TargetFeatures.h"
#include "HyperbusRam.h"
#include "BoardInit.h"

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
HAL_StatusTypeDef HAL_OSPI_HyperbusCfg(OSPI_HandleTypeDef *hospi, OSPI_HyperbusCfgTypeDef *cfg, uint32_t Timeout);
HAL_StatusTypeDef HAL_OSPI_Transmit(OSPI_HandleTypeDef *hospi, uint8_t *pData, uint32_t Timeout);
HAL_StatusTypeDef HAL_OSPI_Receive(OSPI_HandleTypeDef *hospi, uint8_t *pData, uint32_t Timeout);

void Initialize_OPSPI_Hyperam()
{
    int32_t ret;
    static MDMA_HandleTypeDef mdma_handle;
    GPIO_InitTypeDef GPIO_InitStruct;
    uint16_t reg;

    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_OSPI2);
    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_MDMA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF); //  Ports F and G have
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG); //  pins for the OctoSPI[2]
    LL_AHB3_GRP1_ForceReset(LL_AHB3_GRP1_PERIPH_OSPI2);  // Reset the OctoSPI memory interface
    LL_AHB3_GRP1_ReleaseReset(LL_AHB3_GRP1_PERIPH_OSPI2);

    LL_GPIO_InitTypeDef gpio_InitStruct = {0};

    // Common pin settings
    gpio_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_InitStruct.Pull = LL_GPIO_PULL_UP;
    gpio_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

    // GPIO_InitStruct.Pin = OSPI_RAM_CS_PIN;
    gpio_InitStruct.Alternate = LL_GPIO_AF_3;
    gpio_InitStruct.Pin = OSPI_RAM_CS_PIN; // Chip Select
    LL_GPIO_Init(GPIOG, &gpio_InitStruct);

    // OctoSPI DQS GPIO pin configuration
    gpio_InitStruct.Alternate = LL_GPIO_AF_9; // GPIO_AF9_OCTOSPIM_P2
    gpio_InitStruct.Pin = OSPI_RAM_DQS_PIN;   // DQS
    LL_GPIO_Init(GPIOF, &gpio_InitStruct);

    // OctoSPI CLK GPIO pin configuration
    gpio_InitStruct.Alternate = LL_GPIO_AF_9; // GPIO_AF9_OCTOSPIM_P2
    gpio_InitStruct.Pin = OSPI_RAM_CLK_PIN;   // Clock
    LL_GPIO_Init(GPIOF, &gpio_InitStruct);

    // OctoSPI D0,D1,D2,D3 GPIO pin configuration
    gpio_InitStruct.Alternate = LL_GPIO_AF_9; // GPIO_AF9_OCTOSPIM_P2
    gpio_InitStruct.Pin = OSPI_RAM_D0_PIN | OSPI_RAM_D1_PIN | OSPI_RAM_D2_PIN | OSPI_RAM_D3_PIN;
    LL_GPIO_Init(GPIOF, &gpio_InitStruct);

    // OctoSPI D4,D5,D7 GPIO pin configuration
    gpio_InitStruct.Alternate = LL_GPIO_AF_9; // GPIO_AF9_OCTOSPIM_P2
    gpio_InitStruct.Pin = OSPI_RAM_D4_PIN | OSPI_RAM_D5_PIN | OSPI_RAM_D7_PIN;
    LL_GPIO_Init(GPIOG, &gpio_InitStruct);

    gpio_InitStruct.Alternate = LL_GPIO_AF_3; // GPIO_AF9_OCTOSPIM_P2
    gpio_InitStruct.Pin = OSPI_RAM_D6_PIN;
    LL_GPIO_Init(GPIOG, &gpio_InitStruct);

    // HyperRAM clock configuration [ OPSPI2 ]
    //  PLL2_VCO Input = HSE_VALUE/PLL3M = 5 Mhz
    //  PLL2_VCO Output = PLL2_VCO Input * PLL3N = 400 Mhz
    //  PLLOSPICLK = PLL2_VCO Output/PLL2R = 400/2 = 200 Mhz
    //  OSPI clock frequency = PLLOSPICLK = 200 Mhz

    LL_RCC_SetOSPIClockSource(LL_RCC_OSPI_CLKSOURCE_PLL2R);
    LL_RCC_PLL2P_Enable();
    LL_RCC_PLL2Q_Enable();
    LL_RCC_PLL2R_Enable();
    LL_RCC_PLL2_SetM(5);
    LL_RCC_PLL2_SetN(80);
    LL_RCC_PLL2_SetP(5);
    LL_RCC_PLL2_SetQ(2);
    LL_RCC_PLL2_SetR(2);
    LL_RCC_PLL2FRACN_Disable();
    LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLINPUTRANGE_1_2);
    LL_RCC_PLL2_Enable();
    while (LL_RCC_PLL2_IsReady() != 1)
    {
    }

    HAL_StatusTypeDef status;
    OSPI_HandleTypeDef hopspi_config = {0};
    hopspi_config.Instance = OCTOSPI2;
    hopspi_config.Init.FifoThreshold = 4;
    hopspi_config.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
    hopspi_config.Init.MemoryType = HAL_OSPI_MEMTYPE_HYPERBUS;
    hopspi_config.Init.DeviceSize = (uint32_t)POSITION_VAL(S70KL1281_RAM_SIZE);
    hopspi_config.Init.ChipSelectHighTime = 4;
    hopspi_config.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
    hopspi_config.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
    hopspi_config.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    hopspi_config.Init.ClockPrescaler = 2U; // OctoSPI clock = 200 MHz / ClockPrescaler = 100 MHz
    hopspi_config.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
    hopspi_config.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
    hopspi_config.Init.Refresh = 400;           // 4us @100MHz
    hopspi_config.Init.ChipSelectBoundary = 23; // memory die boundary 2^23=8MBs
    hopspi_config.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    ////

    /* Configure memory type, device size, chip select high time, clocked chip
     * select high time, delay block bypass, free running clock, clock mode */
    MODIFY_REG(
        hopspi_config.Instance->DCR1,
        (OCTOSPI_DCR1_MTYP | OCTOSPI_DCR1_DEVSIZE | OCTOSPI_DCR1_CSHT | OCTOSPI_DCR1_CKCSHT | OCTOSPI_DCR1_DLYBYP |
         OCTOSPI_DCR1_FRCK | OCTOSPI_DCR1_CKMODE),
        (hopspi_config.Init.MemoryType | ((hopspi_config.Init.DeviceSize - 1U) << OCTOSPI_DCR1_DEVSIZE_Pos) |
         ((hopspi_config.Init.ChipSelectHighTime - 1U) << OCTOSPI_DCR1_CSHT_Pos) |
         (hopspi_config.Init.ClkChipSelectHighTime << OCTOSPI_DCR1_CKCSHT_Pos) | hopspi_config.Init.DelayBlockBypass |
         hopspi_config.Init.ClockMode));

    /* Configure wrap size */
    MODIFY_REG(hopspi_config.Instance->DCR2, OCTOSPI_DCR2_WRAPSIZE, hopspi_config.Init.WrapSize);

    /* Configure chip select boundary and maximun transfer */
    hopspi_config.Instance->DCR3 =
        ((hopspi_config.Init.ChipSelectBoundary << OCTOSPI_DCR3_CSBOUND_Pos) |
         (hopspi_config.Init.MaxTran << OCTOSPI_DCR3_MAXTRAN_Pos));

    /* Configure refresh */
    hopspi_config.Instance->DCR4 = hopspi_config.Init.Refresh;

    /* Configure FIFO threshold */
    MODIFY_REG(
        hopspi_config.Instance->CR,
        OCTOSPI_CR_FTHRES,
        ((hopspi_config.Init.FifoThreshold - 1U) << OCTOSPI_CR_FTHRES_Pos));

    /* Wait till busy flag is reset */
    uint32_t tickstart = HAL_GetTick();
    status =
        OSPI_WaitFlagStateUntilTimeout(&hopspi_config, HAL_OSPI_FLAG_BUSY, RESET, tickstart, hopspi_config.Timeout);

    /* Configure clock prescaler */
    MODIFY_REG(
        hopspi_config.Instance->DCR2,
        OCTOSPI_DCR2_PRESCALER,
        ((hopspi_config.Init.ClockPrescaler - 1U) << OCTOSPI_DCR2_PRESCALER_Pos));

    /* Configure Dual Quad mode */
    MODIFY_REG(hopspi_config.Instance->CR, OCTOSPI_CR_DQM, hopspi_config.Init.DualQuad);

    /* Configure sample shifting and delay hold quarter cycle */
    MODIFY_REG(
        hopspi_config.Instance->TCR,
        (OCTOSPI_TCR_SSHIFT | OCTOSPI_TCR_DHQC),
        (hopspi_config.Init.SampleShifting | hopspi_config.Init.DelayHoldQuarterCycle));

    /* Enable OctoSPI */
    __HAL_OSPI_ENABLE(&hopspi_config);

    /* Enable free running clock if needed : must be done after OSPI enable
     */

    if (hopspi_config.Init.FreeRunningClock == HAL_OSPI_FREERUNCLK_ENABLE)
    {
        SET_BIT(hopspi_config.Instance->DCR1, OCTOSPI_DCR1_FRCK);
    }

    OSPI_HyperbusCfgTypeDef sHyperbusCfg;
    sHyperbusCfg.RWRecoveryTime = RW_RECOVERY_TIME;
    sHyperbusCfg.AccessTime = OPTIMAL_FIXED_INITIAL_LATENCY;
    sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
    sHyperbusCfg.LatencyMode = HAL_OSPI_FIXED_LATENCY;

    tickstart = HAL_GetTick();

    /* Wait till busy flag is reset */
    status = OSPI_WaitFlagStateUntilTimeout(&hopspi_config, HAL_OSPI_FLAG_BUSY,
                                            RESET, tickstart,
                                            HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    /* Configure Hyperbus configuration Latency register */
    WRITE_REG(hopspi_config.Instance->HLCR,
              ((sHyperbusCfg.RWRecoveryTime << OCTOSPI_HLCR_TRWR_Pos) |
               (sHyperbusCfg.AccessTime << OCTOSPI_HLCR_TACC_Pos) |
               sHyperbusCfg.WriteZeroLatency | sHyperbusCfg.LatencyMode));

    S70KL1281_ReadCfgReg0(&hopspi_config, &reg); // Reading the configuration of the HyperRAM

    OSPI_HyperbusCmdTypeDef sCommand;

    /* Initialize the read command */
    sCommand.AddressSpace = HAL_OSPI_REGISTER_ADDRESS_SPACE;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
    sCommand.Address = S70KL1281_CR0_ADDRESS;
    sCommand.DQSMode = HAL_OSPI_DQS_ENABLE;
    sCommand.NbData = 2U;

//    /* Configure the command */
//    HAL_OSPI_HyperbusCmd(&hopspi_config, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    /* Reception of the data */
    HAL_OSPI_Receive(&hopspi_config, (uint8_t *)&sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    SET_BIT(reg, S70KL1281_CR0_FLE);             // Latency Type
    MODIFY_REG(reg, (uint16_t)S70KL1281_CR0_IL, (uint16_t)OPTIMAL_FIXED_INITIAL_LATENCY_REG_VAL);
    SET_BIT(reg, S70KL1281_CR0_HBE); // Burst type

    MODIFY_REG(reg, (uint16_t)S70KL1281_CR0_BLENGTH,
               (uint16_t)BSP_OSPI_RAM_BURST_32_BYTES); // Burst length
    S70KL1281_WriteCfgReg0(&hopspi_config, reg, HAL_OSPI_FIXED_LATENCY, DEFAULT_INITIAL_LATENCY);
    
    
    
    S70KL1281_EnableMemoryMappedMode(&hopspi_config);
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
    HAL_OSPI_HyperbusCmd(Ctx, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    /* Reception of the data */
    HAL_OSPI_Receive(Ctx, (uint8_t *)Value, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
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

    HAL_OSPI_HyperbusCfg(Ctx, &sHyperbusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    /* Initialize the write command */
    sCommand.AddressSpace = HAL_OSPI_REGISTER_ADDRESS_SPACE;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
    sCommand.Address = S70KL1281_CR0_ADDRESS;
    sCommand.DQSMode = HAL_OSPI_DQS_ENABLE;
    sCommand.NbData = 2U;

    /* Configure the command */
    HAL_OSPI_HyperbusCmd(Ctx, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    /* Transmission of the data */
    HAL_OSPI_Transmit(Ctx, (uint8_t *)(&Value), HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    /* Reconfigure peripheral for correct write access */
    sHyperbusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;

    HAL_OSPI_HyperbusCfg(Ctx, &sHyperbusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

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

    HAL_OSPI_HyperbusCmd(Ctx, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    /* OctoSPI activation of memory-mapped mode */
    sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

    HAL_OSPI_MemoryMapped(Ctx, &sMemMappedCfg);

    return S70KL1281_OK;
}
HAL_StatusTypeDef HAL_OSPI_HyperbusCmd(OSPI_HandleTypeDef *hospi, OSPI_HyperbusCmdTypeDef *cmd, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();

    /* Wait till busy flag is reset */
    status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, tickstart, Timeout);

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

    /* Return function status */
    return status;
}
HAL_StatusTypeDef HAL_OSPI_Init(OSPI_HandleTypeDef *hospi)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t tickstart = HAL_GetTick();

    /* Configure memory type, device size, chip select high time, clocked chip
     * select high time, delay block bypass, free running clock, clock mode */
    MODIFY_REG(
        hospi->Instance->DCR1,
        (OCTOSPI_DCR1_MTYP | OCTOSPI_DCR1_DEVSIZE | OCTOSPI_DCR1_CSHT | OCTOSPI_DCR1_CKCSHT | OCTOSPI_DCR1_DLYBYP |
         OCTOSPI_DCR1_FRCK | OCTOSPI_DCR1_CKMODE),
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
    MODIFY_REG(hospi->Instance->CR, OCTOSPI_CR_FTHRES, ((hospi->Init.FifoThreshold - 1U) << OCTOSPI_CR_FTHRES_Pos));

    /* Wait till busy flag is reset */
    status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, tickstart, hospi->Timeout);

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

    /* Return function status */
    return status;
}
HAL_StatusTypeDef HAL_OSPI_MemoryMapped(OSPI_HandleTypeDef *hospi, OSPI_MemoryMappedTypeDef *cfg)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();

    /* Wait till busy flag is reset */
    status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, tickstart, hospi->Timeout);

    if (cfg->TimeOutActivation == HAL_OSPI_TIMEOUT_COUNTER_ENABLE)
    {

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

    /* Return function status */
    return status;
}
HAL_StatusTypeDef HAL_OSPI_HyperbusCfg(OSPI_HandleTypeDef *hospi, OSPI_HyperbusCfgTypeDef *cfg, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();

    /* Wait till busy flag is reset */
    status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, tickstart, Timeout);

    /* Configure Hyperbus configuration Latency register */
    WRITE_REG(
        hospi->Instance->HLCR,
        ((cfg->RWRecoveryTime << OCTOSPI_HLCR_TRWR_Pos) | (cfg->AccessTime << OCTOSPI_HLCR_TACC_Pos) |
         cfg->WriteZeroLatency | cfg->LatencyMode));

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
HAL_StatusTypeDef HAL_OSPI_Transmit(OSPI_HandleTypeDef *hospi, uint8_t *pData, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();
    __IO uint32_t *data_reg = &hospi->Instance->DR;

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

        *((__IO uint8_t *)data_reg) = *hospi->pBuffPtr;
        hospi->pBuffPtr++;
        hospi->XferCount--;
    } while (hospi->XferCount > 0U);

    /* Wait till transfer complete flag is set to go back in idle state */
    OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_TC, SET, tickstart, Timeout);

    /* Clear transfer complete flag */
    __HAL_OSPI_CLEAR_FLAG(hospi, HAL_OSPI_FLAG_TC);

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

    /* Configure counters and size */
    hospi->XferCount = READ_REG(hospi->Instance->DLR) + 1U;
    hospi->XferSize = hospi->XferCount;
    hospi->pBuffPtr = pData;

    /* Configure CR register with functional mode as indirect read */
    MODIFY_REG(hospi->Instance->CR, OCTOSPI_CR_FMODE, OSPI_FUNCTIONAL_MODE_INDIRECT_READ);

    //    /* Trig the transfer by re-writing address or instruction register */
    //    if (hospi->Init.MemoryType == HAL_OSPI_MEMTYPE_HYPERBUS)
    //    {
    WRITE_REG(hospi->Instance->AR, addr_reg);
    //    }
    //    else
    //    {
    //        if (READ_BIT(hospi->Instance->CCR, OCTOSPI_CCR_ADMODE) != HAL_OSPI_ADDRESS_NONE)
    //        {
    //            WRITE_REG(hospi->Instance->AR, addr_reg);
    //        }
    //        else
    //        {
    //            WRITE_REG(hospi->Instance->IR, ir_reg);
    //        }
    //    }

    do
    {
        /* Wait till fifo threshold or transfer complete flags are set to read
         * received data */
        status = OSPI_WaitFlagStateUntilTimeout(hospi, (HAL_OSPI_FLAG_FT | HAL_OSPI_FLAG_TC), SET, tickstart, Timeout);

        *hospi->pBuffPtr = *((__IO uint8_t *)data_reg);
        hospi->pBuffPtr++;
        hospi->XferCount--;
    } while (hospi->XferCount > 0U);

    /* Wait till transfer complete flag is set to go back in idle state */
    status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_TC, SET, tickstart, Timeout);

    /* Clear transfer complete flag */
    __HAL_OSPI_CLEAR_FLAG(hospi, HAL_OSPI_FLAG_TC);

    /* Return function status */
    return status;
}
