#pragma once

#include "stdio.h"

#define SDMMC_CLOCK_EDGE_RISING             ((uint32_t)0x00000000U)
#define SDMMC_CLOCK_POWER_SAVE_DISABLE      ((uint32_t)0x00000000U)
#define SDMMC_BUS_WIDE_4B                   SDMMC_CLKCR_WIDBUS_0
#define SDMMC_HARDWARE_FLOW_CONTROL_DISABLE ((uint32_t)0x00000000U)
#define CLKCR_CLEAR_MASK                                                                                               \
    ((uint32_t)(SDMMC_CLKCR_CLKDIV | SDMMC_CLKCR_PWRSAV | SDMMC_CLKCR_WIDBUS | SDMMC_CLKCR_NEGEDGE | SDMMC_CLKCR_HWFC_EN | SDMMC_CLKCR_DDR | SDMMC_CLKCR_BUSSPEED | SDMMC_CLKCR_SELCLKRX))

typedef struct
{
    uint32_t ClockEdge; /*!< Specifies the SDMMC_CCK clock transition on which Data and
                           Command change. This parameter can be a value of @ref
                           SDMMC_LL_Clock_Edge                 */

    uint32_t ClockPowerSave; /*!< Specifies whether SDMMC Clock output is enabled or
                                  disabled when the bus is idle.
                                  This parameter can be a value of @ref
                                SDMMC_LL_Clock_Power_Save           */

    uint32_t BusWide; /*!< Specifies the SDMMC bus width.
                           This parameter can be a value of @ref SDMMC_LL_Bus_Wide */

    uint32_t HardwareFlowControl; /*!< Specifies whether the SDMMC hardware flow
                                     control is enabled or disabled. This
                                     parameter can be a value of @ref
                                     SDMMC_LL_Hardware_Flow_Control      */

    uint32_t ClockDiv; /*!< Specifies the clock frequency of the SDMMC controller.
                            This parameter can be a value between Min_Data = 0 and
                          Max_Data = 1023   */

#if (USE_SD_TRANSCEIVER != 0U)
    uint32_t TranceiverPresent; /*!< Specifies if there is a 1V8
                                   Tranceiver/Switcher. This parameter can be a
                                   value of @ref SDMMC_LL_TRANCEIVER_PRESENT */
#endif                          /* USE_SD_TRANSCEIVER */
} SDMMC_InitTypeDef;
