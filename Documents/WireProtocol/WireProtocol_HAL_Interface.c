//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>
#include <wp_hal.h>
#include <wpUSART_Communications.h>

WP_Message inboundMessage;

static bool Initialized = false;


#include <stdio.h>
uint8_t WP_ReceiveBytes(uint8_t* ptr, uint16_t* size)
{
	bool result = ReadNextPacket(ptr, size);
	return (result == true ? 1 : 0);
}

uint8_t WP_TransmitMessage(WP_Message* message)
{
	bool result = WritePacket((uint8_t*)&message->m_header, sizeof(message->m_header));
	return (result == true ? 1 : 0);
}
