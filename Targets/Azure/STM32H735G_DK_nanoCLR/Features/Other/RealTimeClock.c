//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "TargetFeatures.h"
#include <stm32h7xx_ll_pwr.h>
#include "stm32h7xx_ll_rcc.h"
#include <stm32h7xx_ll_rtc.h>

void Initialize_RTC(void)
{

    // Enable the PWR Clock and Enable access to the backup domain
    LL_PWR_EnableBkUpAccess();

    // STMH735G-DK has an external 32.768Khz clock
    // Use this as the RTC clock source

    if (LL_RCC_LSE_IsReady() == 0) // Enable LSE only if disabled
    {
        LL_RCC_ForceBackupDomainReset();
        LL_RCC_ReleaseBackupDomainReset();
        LL_RCC_LSE_Enable();
        while (LL_RCC_LSE_IsReady() != 1)
        {
        }
        LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
        LL_RCC_EnableRTC();
    }

    // Enter in initialization mode
    LL_RTC_DisableWriteProtection(RTC);
    LL_RTC_EnableInitMode(RTC);
    while (LL_RTC_IsActiveFlag_INIT(RTC) != 1) // Initialization mode set?
    {
    }

    // Configure RTC prescaler and RTC data registers

    // LSE Frequency = 32768Hz

#define RTC_ASYNCH_PREDIV ((uint32_t)0x7F)   //  ck_apre=LSEFreq/(ASYNC prediv + 1) = 256Hz
#define RTC_SYNCH_PREDIV  ((uint32_t)0x00FF) // ck_spre=ck_apre/(SYNC prediv + 1) = 1 Hz

    LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_AMPM); // Set Hour Format
    LL_RTC_SetAsynchPrescaler(RTC, RTC_ASYNCH_PREDIV); // (value according to source clock)
    LL_RTC_SetSynchPrescaler(RTC, RTC_SYNCH_PREDIV);   // (value according to source clock)

    // Exit of initialization mode
    LL_RTC_DisableInitMode(RTC);
    LL_RTC_ClearFlag_RS(RTC);                // Clear RSF flag
    while (LL_RTC_IsActiveFlag_RS(RTC) != 1) // synchronised
    {
    }
    LL_RTC_EnableWriteProtection(RTC); // ??  if Shadow registers are enabled

    // FUTURE ? /* Set OutPut */
    // LL_RTC_SetAlarmOutEvent(RTC, LL_RTC_ALARMOUT_DISABLE);
    // LL_RTC_SetOutputPolarity(RTC, LL_RTC_OUTPUTPOLARITY_PIN_HIGH);
    // LL_RTC_SetAlarmOutputType(RTC, LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN);
}
