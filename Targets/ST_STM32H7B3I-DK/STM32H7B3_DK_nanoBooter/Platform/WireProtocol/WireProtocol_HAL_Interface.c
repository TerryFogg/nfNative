//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>
#include <WireProtocol.h>
#include <WireProtocol_Message.h>
#include <wpUSART_Communications.h>
#include <WireProtocol_HAL_Interface.h>
#include <stm32h7xx_hal.h>
#include <targetHAL.h>

WP_Message inboundMessage;

static bool Initialized = false;


#include <stdio.h>
void WP_ReceiveBytes(uint8_t **ptr, uint32_t *size)
{
	ReadNextPacket(ptr, size);
}

uint8_t WP_TransmitMessage(WP_Message* message)
{
	bool result = WritePacket((uint8_t*)&message->m_header, sizeof(message->m_header));
	return (result == true ? 1 : 0);
}
