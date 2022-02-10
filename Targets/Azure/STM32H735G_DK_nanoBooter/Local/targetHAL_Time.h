//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#ifndef _TARGET_HAL_TIME_H_
#define _TARGET_HAL_TIME_H_


static inline uint32_t DMT_GetTick()
{
    return 1;
}

#define HAL_Time_CurrentSysTicks DMT_GetTick

#endif //_TARGET_HAL_TIME_H_
