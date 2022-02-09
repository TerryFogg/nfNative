#include <stm32h7xx_ll_rcc.h>
#include <stm32h7xx_ll_system.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_pwr.h>
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include "BoardInit.h"

#include <stm32h7xx_hal.h>

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 520000000 (CPU Clock)
 *            HCLK(Hz)                       = 260000000 (AXI and AHBs Clock)
 *            AHB Prescaler                  = 2
 *            D1 APB3 Prescaler              = 2 (APB3 Clock  130MHz)
 *            D2 APB1 Prescaler              = 2 (APB1 Clock  130MHz)
 *            D2 APB2 Prescaler              = 2 (APB2 Clock  130MHz)
 *            D3 APB4 Prescaler              = 2 (APB4 Clock  130MHz)
 *            HSE Frequency(Hz)              = 25000000
 *            PLL_M                          = 5
 *            PLL_N                          = 104
 *            PLL_P                          = 1
 *            PLL_Q                          = 4
 *            PLL_R                          = 2
 *            VDD(V)                         = 3.3
 *            Flash Latency(WS)              = 3
 * @param  None
 * @retval None
 */

void SystemClock_Config(void)
{
    /*!< Supply configuration update enable */
    LL_PWR_ConfigSupply(LL_PWR_DIRECT_SMPS_SUPPLY);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
    while (LL_PWR_IsActiveFlag_VOS() == 0)
    {
    }

    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();
    while (LL_RCC_HSE_IsReady() != 1)
    {
    }

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);

    // Main PLL configuration and activation
    LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
    LL_RCC_PLL1P_Enable();
    LL_RCC_PLL1Q_Enable();
    LL_RCC_PLL1R_Enable();
    LL_RCC_PLL1FRACN_Disable();
    LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_1_2);
    LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL1_SetM(5);
    LL_RCC_PLL1_SetN(104);
    LL_RCC_PLL1_SetP(1);
    LL_RCC_PLL1_SetQ(4);
    LL_RCC_PLL1_SetR(2);
    LL_RCC_PLL1_Enable();
    while (LL_RCC_PLL1_IsReady() != 1)
    {
    }

    // Setup peripheral prescalers
    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

    /* Set PLL1 as System Clock Source */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
    {
    }

    /*
      Note : The activation of the I/O Compensation Cell is recommended with communication  interfaces
              (GPIO, SPI, FMC, OSPI ...)  when  operating at  high frequencies (please refer to product datasheet)
              The I/O Compensation Cell activation  procedure requires :
      - The activation of the CSI clock
      - The activation of the SYSCFG clock
      - Enabling the I/O Compensation Cell : setting bit[0] of register SYSCFG_CCCSR

        To do this please uncomment the following code
        */

    LL_RCC_CSI_Enable();
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG); // System clock enable
    LL_SYSCFG_EnableCompensationCell();
}

void LTDCClock_Config(void)
{
    // RK043FN48H LCD clock configuration
    // LCD clock configuration
    // PLL3_VCO Input = HSE_VALUE/PLL3M = 4 Mhz
    // PLL3_VCO Output = PLL3_VCO Input * PLL3N = 800 Mhz
    // PLLLCDCLK = PLL3_VCO Output/PLL3R = 800/83 = 9.63 Mhz
    // LTDC clock frequency = PLLLCDCLK = 9.63 Mhz
    LL_RCC_PLL3_Disable();
    LL_RCC_PLL3_SetM(5);
    LL_RCC_PLL3_SetN(160);
    LL_RCC_PLL3_SetP(2);
    LL_RCC_PLL3_SetQ(2);
    LL_RCC_PLL3_SetR(83);
    LL_RCC_PLL3_SetFRACN(0);
    LL_RCC_PLL3_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL3_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_1_2);
    LL_RCC_PLL3_Enable();
}

