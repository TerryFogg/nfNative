#pragma once

#include <stdio.h>
#include <stdbool.h>
#include "stm32h7xx.h"

#define FLASHWORD 8

#define EmbeddedFlashTimeout 50000

bool EmbeddedFlashUnlock(void);
bool EmbeddedFlashLock(void);
bool EmbeddedFlash_DeploymentErase();
bool EmbeddedFlashWriteUserOptionsBits(uint8_t *buffer);
bool EmbeddedFlashReadUserOptionsBits(uint8_t *buffer);
bool EmbeddedFlashWrite(uint32_t startAddress, uint32_t lengthInBytes, uint8_t *buffer);
void EmbeddedFlashReadBytes(uint32_t startAddress, uint32_t length, uint8_t *buffer);
uint32_t GetSector(uint32_t Address);
