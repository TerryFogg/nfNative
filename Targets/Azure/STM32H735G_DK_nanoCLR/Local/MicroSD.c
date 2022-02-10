//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include "MicroSD.h"
#include "stm32h735xx.h"
#include "Register_Modifications.h"
#include "TargetFeatures.h"

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
