#pragma once

#include <stdbool.h>
#include <stdint-gcc.h>

void InitWireProtocolCommunications();
bool wp_WriteBytes(uint8_t *ptr, uint16_t size);
int wp_ReadBytes(uint8_t **ptr, uint32_t *size, uint32_t wait_time);
void wp_InitializeUSB();
void wp_DataReceived(void);
uint8_t wp_StartTransmitTransfer(void);
