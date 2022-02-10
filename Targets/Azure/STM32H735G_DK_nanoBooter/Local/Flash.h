#pragma once

#include <stdio.h>
#include <stdbool.h>
#include "stm32h735xx.h"

#define FLASHWORD    8
#define FLASH_BANK_1 0x01U /*!< Bank 1   */

#define EmbeddedFlashTimeout      50000
#define FLASH_KEY1                0x45670123U
#define FLASH_KEY2                0xCDEF89ABU
#define FLASH_TYPEERASE_SECTORS   0x00U            /*!< Sectors erase only          */
#define FLASH_TYPEERASE_MASSERASE 0x01U            /*!< Flash Mass erase activation */
#define FLASH_VOLTAGE_RANGE_3     FLASH_CR_PSIZE_1 /*!< Flash program/erase by 32 bits */
#define FLASH_FLAG_QW_BANK1       FLASH_SR_QW      /*!< Wait Queue on Bank 1 flag */
#define FLASH_FLAG_ALL_ERRORS_BANK1                                            \
  (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR |    \
   FLASH_SR_OPERR | FLASH_SR_RDPERR | FLASH_SR_RDSERR | FLASH_SR_SNECCERR |    \
   FLASH_SR_DBECCERR | FLASH_SR_CRCRDERR) /*!< All Bank 1 error flags */
#define FLASH_FLAG_EOP_BANK1             FLASH_SR_EOP       /*!< End Of Program on Bank 1 flag                  */
#define FLASH_CLEAR_FLAG_BANK1(__FLAG__) WRITE_REG(FLASH->CCR1, (__FLAG__))
#define FLASH_GET_FLAG_BANK1(__FLAG__)   (READ_BIT(FLASH->SR1, (__FLAG__)) == (__FLAG__))

#define FLASH_TIMEOUT_VALUE 50000U /* 50 s */

#define __HAL_FLASH_GET_FLAG(__FLAG__) __HAL_FLASH_GET_FLAG_BANK1(__FLAG__)

typedef struct
{
    uint32_t TypeErase; /*!< Mass erase or sector Erase.
                             This parameter can be a value of @ref
                           FLASHEx_Type_Erase */

    uint32_t Banks; /*!< Select banks to erase when Mass erase is enabled.
                         This parameter must be a value of @ref FLASHEx_Banks */

    uint32_t Sector; /*!< Initial FLASH sector to erase when Mass erase is disabled
                          This parameter must be a value of @ref FLASH_Sectors */

    uint32_t NbSectors; /*!< Number of sectors to be erased.
                             This parameter must be a value between 1 and (max
                           number of sectors - value of Initial sector)*/

    uint32_t VoltageRange; /*!< The device voltage range which defines the erase
                              parallelism This parameter must be a value of @ref
                              FLASHEx_Voltage_Range */

} FLASH_EraseInitTypeDef;

bool EmbeddedFlashUnlock(void);
bool EmbeddedFlashLock(void);
bool EmbeddedFlash_DeploymentErase();
bool EmbeddedFlashWriteUserOptionsBits(uint8_t *buffer);
bool EmbeddedFlashReadUserOptionsBits(uint8_t *buffer);
bool EmbeddedFlashWrite(uint32_t startAddress, uint32_t lengthInBytes, uint8_t *buffer);
void EmbeddedFlashReadBytes(uint32_t startAddress, uint32_t length, uint8_t *buffer);

void WaitForLastOperation(uint32_t Timeout, uint32_t Bank);
void FLASH_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);

uint32_t GetSector(uint32_t Address);
