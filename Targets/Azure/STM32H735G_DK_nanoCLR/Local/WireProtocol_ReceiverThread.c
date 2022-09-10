//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <WireProtocol_Message.h>
#include "wpUSART_Communications.h"
#include "Board_STM32H735G-DK.h"
#include <tx_api.h>

extern WP_Message inboundMessage;

__attribute__((noreturn)) void ReceiverThread_entry(uint32_t parameter)
{
    (void)parameter;
    InitWireProtocolCommunications(); // NOTE: Don't call  Scheduler type calls in this module
    tx_thread_sleep(50);

    while (1) // loop until thread receives a request to terminate
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
