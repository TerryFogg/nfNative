//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_Power.h>
#include <nanoHAL_v2.h>
#include <target_platform.h>
#include <target_common.h>
#include "stm32h735xx.h"


uint32_t WakeupReasonStore;

inline void CPU_Reset()
{
    NVIC_SystemReset();
}

inline bool CPU_IsSoftRebootSupported()
{
    return true;
}

// CPU sleep is not currently implemented in this target
inline void CPU_Sleep(SLEEP_LEVEL_type level, uint64_t wakeEvents)
{
    (void)level;
    (void)wakeEvents;
}

void CPU_SetPowerMode(PowerLevel_type powerLevel)
{
    // default to false
    bool success = false;

    switch (powerLevel)
    {
    case PowerLevel__Off:
        break;
    default:
        // all the other power modes are unsupported here
        break;
    }
}
