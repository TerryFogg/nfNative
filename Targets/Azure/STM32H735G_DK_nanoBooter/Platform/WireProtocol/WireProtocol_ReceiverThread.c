//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>
#include <WireProtocol_Message.h>
#include "wpUSART_Communications.h"
#include "BoardInit.h"
#include <tx_api.h>

extern WP_Message inboundMessage;

__attribute__((noreturn))
void ReceiverThread_entry(uint32_t parameter)
{
    (void)parameter;

    if (InitWireProtocolCommunications() == true)  // NOTE: Don't call  Scheduler type calls in this module
    {
        //
    }
    else
    {
        //
    }

    tx_thread_sleep(50);

    // loop until thread receives a request to terminate
    while (1)
    {
        WP_Message_Initialize(&inboundMessage);
        WP_Message_PrepareReception(&inboundMessage);
        WP_Message_Process(&inboundMessage);
    }
}

void WP_Message_PrepareReception_Platform()
{
    // empty on purpose, nothing to configure
}

