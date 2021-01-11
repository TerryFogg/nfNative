#pragma once

#include <stdbool.h>

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"



// Receive buffer for DMA copy from peripheral USART
#define RXBUFFER        32

// Transmit buffer - just one packet
#define TXBUFFERSIZE                     32

#define NUMBER_PACKETS         4


/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */


typedef enum PacketReadStates { ReadNotSet, Listening, ReadFull, ReadPartial } ReadPacketState;
typedef enum PacketWriteStates { WriteNotSet, Writing, SentFull, SentPartial } WritePacketState;

bool InitWireProtocolCommunications();
void SystemClock_Config(void);

bool WritePacket(uint8_t* ptr, uint16_t size);
bool ReadNextPacket(uint8_t* ptr, uint16_t* size);
