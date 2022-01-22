//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoPAL.h>
#include "tx_api.h"

static TX_TIMER nextEventTimer;

static void NextEventTimer_Callback(ULONG input)
{
    (void)input;
    
    // this call also schedules the next one, if there is one
    HAL_COMPLETION::DequeueAndExec();
}

HRESULT Time_Initialize()
{
    tx_timer_create(&nextEventTimer, (char*)"NextEventTimer", NextEventTimer_Callback, 10, 10, 10, TX_AUTO_ACTIVATE);

    return S_OK;
}

HRESULT Time_Uninitialize()
{
    tx_timer_delete(&nextEventTimer);
    return S_OK;
}

void Time_SetCompare(uint64_t compareValueTicks)
{
    if (compareValueTicks == 0)
    {
        // compare value is 0 so dequeue and schedule immediately 
        HAL_COMPLETION::DequeueAndExec();
    }
    else if (compareValueTicks == HAL_COMPLETION_IDLE_VALUE)
    {
        // wait for infinity, don't need to do anything here
    }    
    else
    {
        if (HAL_Time_CurrentTime() >= compareValueTicks) 
        { 
            // already missed the event, dequeue and execute immediately 
            HAL_COMPLETION::DequeueAndExec();
        }
        else
        {
            
            tx_timer_deactivate(&nextEventTimer);

            // compareValueTicks is the time (in sys ticks) that is being requested to fire an HAL_COMPLETION::DequeueAndExec()
            // need to subtract the current system time to set when the timer will fire
            compareValueTicks -= HAL_Time_CurrentTime();
            
            if (compareValueTicks == 0) 
            {
                // compare value is 0 so dequeue and execute immediately
                // no need to call the timer
                HAL_COMPLETION::DequeueAndExec();
                return;
            }
            // Update the value
            tx_timer_change(&nextEventTimer, compareValueTicks, compareValueTicks);
        }
    }
}
