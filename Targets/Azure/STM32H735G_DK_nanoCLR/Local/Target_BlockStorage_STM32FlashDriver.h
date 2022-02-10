#pragma once

//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <stdio.h>
#include <stdbool.h>
#include "stm32h735xx.h"
#include <nanoPAL_BlockStorage.h>
#include "Register_Modifications.h"

#define MIN(a, b) (a < b ? a : b)

#define STM32_NON_VOLATILE_OPTION_BYTES_SIZE 2048
#define NUMBER_OF_BYTES_IN_FLASH_WORD (FLASH_NB_32BITWORD_IN_FLASHWORD * 4U)

#define FLASHWORD    8
#define FLASH_BANK_1 0x01U

#define FLASH_KEY1           0x45670123U
#define FLASH_KEY2           0xCDEF89ABU
#define FLASH_OPT_KEY1       0x08192A3BU
#define FLASH_OPT_KEY2       0x4C5D6E7FU

//#define FLASH_TYPEERASE_SECTORS 0x00U // Sectors erase only
//
#define FLASH_VOLTAGE_RANGE_4 FLASH_CR_PSIZE // Flash program/erase by 64 bits
#define FLASH_FLAG_QW_BANK1   FLASH_SR_QW    // Wait Queue on Bank 1 flag
#define FLASH_FLAG_ALL_ERRORS_BANK1                                                                                    \
    (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR | FLASH_SR_RDPERR |       \
     FLASH_SR_RDSERR | FLASH_SR_SNECCERR | FLASH_SR_DBECCERR | FLASH_SR_CRCRDERR) // All Bank 1 error flags
#define FLASH_FLAG_EOP_BANK1 FLASH_SR_EOP                                         // End Of Program on Bank 1 flag

#define FLASH_TIMEOUT_VALUE 50000U /* 50 s */

#ifdef __cplusplus
extern "C"
{
#endif
    bool STM32FlashDriver_InitializeDevice(void *);
    bool STM32FlashDriver_UninitializeDevice(void *);
    DeviceBlockInfo *STM32FlashDriver_GetDeviceInfo(void *);
    bool STM32FlashDriver_Read(void *, ByteAddress startAddress, unsigned int numBytes, unsigned char *buffer);
    bool STM32FlashDriver_Write(
        void *,
        ByteAddress startAddress,
        unsigned int numBytes,
        unsigned char *buffer,
        bool readModifyWrite);
    bool STM32FlashDriver_IsBlockErased(void *, ByteAddress blockAddress, unsigned int length);
    bool STM32FlashDriver_EraseBlock(void *, ByteAddress address);

    bool EmbeddedFlashUnlock(void);
    bool EmbeddedFlashLock(void);
    bool EmbeddedFlash_DeploymentErase();
    bool EmbeddedFlashWrite(uint32_t startAddress, uint32_t lengthInBytes, uint8_t *buffer);
    void WriteFlashWord(uint32_t *dest_addr, uint32_t *buffer);
    bool EmbeddedFlashReadBytes(uint32_t startAddress, uint32_t length,
                                uint8_t *buffer);
    bool WaitForLastOperation( uint32_t Bank);
    bool EmbeddedFlashErase(uint32_t sector);
    uint32_t GetSector(uint32_t Address);

#ifdef __cplusplus
}
#endif
