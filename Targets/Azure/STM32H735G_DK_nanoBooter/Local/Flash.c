
#include "Flash.h"

#include "stdio.h"
#include "string.h"
#include <targetHAL.h>
#include "Register_Modifications.h"

bool EmbeddedFlashUnlock(void)
{
    //  The application software must not unlock a register that is already
    //  unlocked, otherwise this register will remain locked until next system
    //  reset.Similar constraints apply to bank erase requests.
    if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != 0U)
    {
        /* Authorize the FLASH Bank1 Registers access */
        WRITE_REG(FLASH->KEYR1, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR1, FLASH_KEY2);

        /* Verify Flash Bank1 is unlocked */
        if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != 0U)
        {
            return false;
        }
    }
    return true;
}
bool EmbeddedFlashLock(void)
{
    // Set the LOCK Bit to lock the FLASH Bank1 Control Register access
    SET_BIT(FLASH->CR1, FLASH_CR_LOCK);

    // Verify Flash Bank1 is locked
    if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) == 0U)
    {
        return false;
    }
    return true;
}
bool EmbeddedFlashWrite(uint32_t startAddress, uint32_t lengthInBytes, uint8_t *DataAddress)
{
    // STM32H735 flash write capability has
    //  - Single Flash word write (256-bit granularity)
    //  - The application can decide to write as little as 8 bits to a Flash word.
    //    In this case, a force write mechanism to the 256 bits + ECC is used.
    //
    // It is not recommended to overwrite a Flash word that is not virgin.
    // (ie. not previously erased),which requires erasing the entire sector.

    __IO uint32_t *dest_addr = (__IO uint32_t *)(startAddress);
    __IO uint32_t *src_addr = (__IO uint32_t *)(DataAddress);

    int number_of_256bit_words = lengthInBytes / (FLASH_NB_32BITWORD_IN_FLASHWORD * 4);
    int remaining_bytes = lengthInBytes - (number_of_256bit_words * 32);

    EmbeddedFlashUnlock();
    SET_BIT(FLASH->CR1, FLASH_CR_PG); // Enable internal buffer for write operations

    for (int flash_word_index = 0; flash_word_index < (number_of_256bit_words + 1); flash_word_index++)
    {
        WaitForLastOperation(EmbeddedFlashTimeout, FLASH_BANK_1);
        bool full_word_write = (flash_word_index == number_of_256bit_words && remaining_bytes != 0);
        if (full_word_write)
        {
            {
                // Program the full flash word ( 256-bits)
                int row_index = 8;
                __ISB();
                __DSB();
                do
                {
                    *dest_addr = *src_addr;
                    dest_addr++;
                    src_addr++;
                    row_index--;
                } while (row_index != 0U);
            }
            __ISB();
            __DSB();
        }
        else
        {
            {
                // Program the remaining bytes
                int byte_index = remaining_bytes;
                uint8_t *dest_byte_pointer = (uint8_t *)dest_addr;
                uint8_t *src_byte_pointer = (uint8_t *)src_addr;
                __ISB();
                __DSB();
                do
                {
                    *dest_byte_pointer = *src_byte_pointer;
                    dest_byte_pointer++;
                    src_byte_pointer++;
                    byte_index--;
                } while (byte_index != 0U);
                __ISB();
                __DSB();
            }
            SET_BIT(FLASH->CR1, FLASH_CR_FW); // Force write mechanism
                                              // The unwritten bits are automatically set to 1.
        }
    }
    CLEAR_BIT(FLASH->CR1, FLASH_CR_PG); // Disable internal buffer for write operations
    EmbeddedFlashUnlock();
    return true;
}
bool EmbeddedFlash_DeploymentErase()
{
    uint32_t StartSector = 5;
    uint32_t EndSector = 7;
    uint32_t SECTORError;

    uint32_t FirstSector = GetSector(__deployment_start__);
    uint32_t NbOfSectors = GetSector(__deployment_end__) - FirstSector + 1;

    EmbeddedFlashUnlock();

    static FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FirstSector;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.NbSectors = NbOfSectors;
    FLASH_Erase(&EraseInitStruct, &SECTORError);
    EmbeddedFlashLock();

    return 0;
}
void EmbeddedFlashReadBytes(uint32_t startAddress, uint32_t length, uint8_t *buffer)
{
    __IO uint8_t *cursor = (__IO uint8_t *)startAddress;
    __IO uint8_t *endAddress = (__IO uint8_t *)(startAddress + length);
    while (cursor < endAddress)
    {
        *buffer++ = *cursor++;
    }
}
bool EmbeddedFlashWriteUserOptionsBits(uint8_t *buffer)
{
    return true;
}
bool EmbeddedFlashReadUserOptionsBits(uint8_t *buffer)
{
    return true;
}
uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if (Address < (FLASH_BASE + FLASH_BANK_SIZE))
    {
        sector = (Address - FLASH_BASE) / FLASH_SECTOR_SIZE;
    }
    else
    {
        sector = (Address - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_SECTOR_SIZE;
    }

    return sector;
}
void WaitForLastOperation(uint32_t Timeout, uint32_t Bank)
{
    /* Wait for the FLASH operation to complete by polling on QW flag to be reset.
       Even if the FLASH operation fails, the QW flag will be reset and an error
       flag will be set */

    uint32_t bsyflag = FLASH_FLAG_QW_BANK1;
    uint32_t errorflag = FLASH->SR1 & FLASH_FLAG_ALL_ERRORS_BANK1;

    while (FLASH_GET_FLAG_BANK1(bsyflag))
    {
    }
    /* In case of error reported in Flash SR1 or SR2 register */
    if ((errorflag & 0x7FFFFFFFU) != 0U)
    {
        WRITE_REG(FLASH->CCR1, errorflag); /* Clear error programming flags */
    }
    FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_EOP_BANK1); /* Clear FLASH End of Operation pending bit */
}

void FLASH_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError)
{
    uint32_t sector_index;

    WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE, FLASH_BANK_1);

    if (pEraseInit->TypeErase == FLASH_TYPEERASE_MASSERASE) // Mass erase to be done
    {
        /* Set Program/erase VoltageRange for Bank1 */
        FLASH->CR1 &= (~FLASH_CR_PSIZE);
        FLASH->CR1 |= pEraseInit->VoltageRange;        // Voltage range gives size
        FLASH->CR1 |= (FLASH_CR_BER | FLASH_CR_START); // Erase Bank1

        /* Wait for last operation to be completed on Bank 1 */
        if ((pEraseInit->Banks & FLASH_BANK_1) == FLASH_BANK_1)
        {
            WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE, FLASH_BANK_1);
            FLASH->CR1 &= (~FLASH_CR_BER);
        }
    }
    else
    {
        /*Initialization of SectorError variable*/
        *SectorError = 0xFFFFFFFFU;

        /* Erase by sector by sector to be done*/
        for (sector_index = pEraseInit->Sector; sector_index < (pEraseInit->NbSectors + pEraseInit->Sector);
             sector_index++)
        {
            /* Reset Program/erase VoltageRange and Sector Number for Bank1 */
            FLASH->CR1 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
            FLASH->CR1 |=
                (FLASH_CR_SER | pEraseInit->VoltageRange | (sector_index << FLASH_CR_SNB_Pos) | FLASH_CR_START);
            WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE, FLASH_BANK_1);
            FLASH->CR1 &= (~(FLASH_CR_SER | FLASH_CR_SNB)); // Operation is completed, disable the SER Bit
        }
    }
}
