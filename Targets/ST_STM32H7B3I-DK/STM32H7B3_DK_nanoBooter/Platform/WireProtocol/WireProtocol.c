//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>
#include "wpUSART_Communications.h"
#include "BoardInit.h"
#include <tx_api.h>


__attribute__((noreturn))
void ReceiverThread_entry(uint32_t parameter)
{
    (void)parameter;

    if (InitWireProtocolCommunications() == true)  // NOTE: Don't call  Scheduler type calls in this module
        {
            nanoBooterState = ok;
        }
    else
    {
        nanoBooterState = communications_failure;
    }

    tx_thread_sleep(50);

    // loop until thread receives a request to terminate
    while(1)
    {
        tx_thread_sleep(50);    // Relinquish control and allow some time for the lower priority nanBooter status display.
    }
}
