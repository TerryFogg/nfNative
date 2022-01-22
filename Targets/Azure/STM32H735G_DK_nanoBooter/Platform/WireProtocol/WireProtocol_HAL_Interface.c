//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <WireProtocol.h>
#include <WireProtocol_HAL_Interface.h>
#include <WireProtocol_Message.h>
#include <nanoHAL_v2.h>
#include <stm32h7xx_hal.h>
#include <targetHAL.h>
#include <wpUSART_Communications.h>

WP_Message inboundMessage;

static bool Initialized = false;

void WP_ReceiveBytes(uint8_t **ptr, uint32_t *size) 
{
  uint32_t wait_time = TX_WAIT_FOREVER;
  if (*size != 0) // Can get 0 for size if all header and payload comes through quickly
  {
    size_t read = ReadNextPacket(ptr, size, wait_time);
    *ptr += read;
    *size -= read;
  }
}

uint8_t WP_TransmitMessage(WP_Message *message) 
{
  WritePacket((uint8_t *)&message->m_header, sizeof(message->m_header));
  // if there is anything on the payload send it to the output stream
  if (message->m_header.m_size && message->m_payload) 
  {
    WritePacket(message->m_payload, message->m_header.m_size);
  }
  return true;
}
