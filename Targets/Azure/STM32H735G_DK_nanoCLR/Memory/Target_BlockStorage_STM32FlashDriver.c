//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <Target_BlockStorage_STM32FlashDriver.h>

bool STM32FlashDriver_InitializeDevice(void *context)
{
    (void)context;

    // nothing to do here
    return true;
}
bool STM32FlashDriver_UninitializeDevice(void *context)
{
    (void)context;

    // nothing to do here
    return true;
}
DeviceBlockInfo *STM32FlashDriver_GetDeviceInfo(void *context)
{
    MEMORY_MAPPED_NOR_BLOCK_CONFIG *config = context;
    return config->BlockConfig.BlockDeviceInformation;
}
bool STM32FlashDriver_Read(void *context, ByteAddress startAddress, unsigned int numBytes, unsigned char *buffer)
{
    (void)context;
    EmbeddedFlashReadBytes(startAddress, numBytes, buffer);
    return true;
}
bool STM32FlashDriver_Write(
    void *context,
    ByteAddress startAddress,
    unsigned int numBytes,
    unsigned char *buffer,
    bool readModifyWrite)
{
    (void)context;
    (void)readModifyWrite;

    return EmbeddedFlashWrite(startAddress, numBytes, buffer);
}
bool STM32FlashDriver_IsBlockErased(void *context, ByteAddress blockAddress, unsigned int length)
{
    // For STM32H735IG

    // NOTE:
    // From the reference manual  "RM0468 Rev 3" , page 165/3357
    // If the application software reads back a word that has been erased, all
    // the bits will be read at 1, without ECC error
    //
    // TODO Need to check performance, which is quicker, Reading 128K bytes for
    // 1's and no ECC error or just erase?

    // It is recommended not to overwrite a byte that is not a "virgin", (erased)
    // At the moment, always return false. The Block(128K sector) is not erased.

    return false;
}
bool STM32FlashDriver_EraseBlock(void *context, ByteAddress address)
{
    // Minimum erase size on STM32H735IG is 1 sector (128K bytes)
    uint32_t sector = GetSector(address);
    EmbeddedFlashErase(sector);
    return true;
}
bool EmbeddedFlashErase(uint32_t sector)
{
    bool success;
    EmbeddedFlashUnlock();
    if (WaitForLastOperation(FLASH_BANK_1)) // Flash unlocked?
    {
        FLASH->CR1 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB); // Reset Program/erase VoltageRange
                                                        // and Sector Number for Bank1
        FLASH->CR1 |= (FLASH_CR_SER | FLASH_VOLTAGE_RANGE_4 | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_START);
        success = WaitForLastOperation(FLASH_BANK_1);
        FLASH->CR1 &= (~(FLASH_CR_SER | FLASH_CR_SNB)); // Operation is completed, disable the SER Bit
        EmbeddedFlashLock();
    }
    return success;
}
bool EmbeddedFlashUnlock(void)
{
    //  The application software must not unlock a register that is already
    //  unlocked, otherwise this register will remain locked until next system
    //  reset.Similar constraints apply to bank erase requests.
    if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != 0U)
    {
        WRITE_REG(FLASH->KEYR1,
                  FLASH_KEY1); // Authorize the FLASH Bank1 Registers access
        WRITE_REG(FLASH->KEYR1, FLASH_KEY2);
        if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != 0U) // Verify Flash Bank1 is unlocked
        {
            return false;
        }
    }
    return true;
}
bool EmbeddedFlashLock(void)
{
    SET_BIT(FLASH->CR1, FLASH_CR_LOCK);            // Lock FLASH Bank1 Control Register access
    if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) == 0U) // Verify Flash Bank1 is locked
    {
        return false;
    }
    return true;
}

static int entry = 0;

#if defined(nothing)
bool EmbeddedFlashWrite(uint32_t flashStartAddress, uint32_t lengthInBytes, uint8_t *buffer)
{
    // This routine only writes complete flash words and preserves existing data

    // STM32H735 flash write capability has
    //  - Single Flash word write (256-bit granularity)
    //  - The application can decide to write as little as 8 bits to a Flash word.
    //    In this case, a force write mechanism to the 256 bits + ECC is used.
    //
    // It is not recommended to overwrite a Flash word that is not "virgin".
    // (ie. not previously erased), which requires erasing the entire sector.

    uint8_t flash_word_buffer[NUMBER_OF_BYTES_IN_FLASH_WORD]; // 256Bit flash word buffer
    uint8_t *data_buffer = buffer;
    uint8_t *flash_address = (uint8_t *)(flashStartAddress);
    uint32_t byte_offset_into_flash_word;

    if (entry == 1)
    {
        EmbeddedFlashUnlock();
    }
    entry++;

    // Check the starting address and calculate if it is aligned on a flash word
    // boundary
    byte_offset_into_flash_word = ((uint32_t)flash_address - FLASH_BANK1_BASE) % NUMBER_OF_BYTES_IN_FLASH_WORD;

    if (byte_offset_into_flash_word > 0) // If the starting address is not aligned on a flash word boundary
    {
        uint32_t flashWordBoundary = (uint32_t)flash_address - byte_offset_into_flash_word;
//        EmbeddedFlashReadBytes(
//            flashWordBoundary,             // Address aligned to flash word boundary
//            NUMBER_OF_BYTES_IN_FLASH_WORD, // Read the complete flash word
//            flash_word_buffer);            // Store into a temporary buffer
        int newBytesToWrite = NUMBER_OF_BYTES_IN_FLASH_WORD - byte_offset_into_flash_word;
        memcpy(&flash_word_buffer[byte_offset_into_flash_word], data_buffer, newBytesToWrite);
        WriteFlashWord((uint32_t *)flashWordBoundary, (uint32_t *)flash_word_buffer);
        flash_address += newBytesToWrite;
        lengthInBytes -= newBytesToWrite;
        data_buffer += newBytesToWrite;
    }

    // Current flash address is now aligned on the flash word boundary
    // Calculate the number of full flash words and any remaining bytes.
    int number_flash_words = lengthInBytes / NUMBER_OF_BYTES_IN_FLASH_WORD;
    int remaining_bytes = lengthInBytes % NUMBER_OF_BYTES_IN_FLASH_WORD;

    for (int flash_word_index = 0; flash_word_index < number_flash_words; flash_word_index++)
    {
        memcpy(flash_word_buffer, data_buffer, NUMBER_OF_BYTES_IN_FLASH_WORD);
        WriteFlashWord((uint32_t *)flash_address, (uint32_t *)flash_word_buffer);
        flash_address += NUMBER_OF_BYTES_IN_FLASH_WORD;
        lengthInBytes -= NUMBER_OF_BYTES_IN_FLASH_WORD;
        data_buffer += NUMBER_OF_BYTES_IN_FLASH_WORD;
    }

    // Use the force write mechanism to the 256 bits to recreate the ECC
    // It is not recommended to overwrite a Flash word that is not virgin.
    // The result may lead to an inconsistent ECC code that will be systematically reported by the
    // embedded Flash memory,

    if (remaining_bytes > 0) // Write final remaining bytes merging with the flash word
    {
        EmbeddedFlashReadBytes((uint32_t)flash_address, NUMBER_OF_BYTES_IN_FLASH_WORD, flash_word_buffer);
        int finalBytesToWrite = lengthInBytes;
        memcpy(flash_word_buffer, data_buffer, finalBytesToWrite);
        WriteFlashWord((uint32_t *)flash_address, (uint32_t *)flash_word_buffer);
        lengthInBytes -= finalBytesToWrite;
    }

    return true;
}
#endif

bool EmbeddedFlashWrite(uint32_t startAddress, uint32_t lengthInBytes, uint8_t *DataAddress)
{
    // STM32H735 flash write capability has
    //  - Single Flash word write (256-bit granularity)
    //  - The application can decide to write as little as 8 bits to a Flash word.
    //    In this case, a force write mechanism to the 256 bits + ECC is used.
    //
    // It is not recommended to overwrite a Flash word that is not "virgin".
    // (ie. not previously erased), which requires erasing the entire sector.

    __IO uint32_t *dest_addr = (__IO uint32_t *)(startAddress);
    __IO uint32_t *src_addr = (__IO uint32_t *)(DataAddress);

    int number_of_256bit_words = lengthInBytes / NUMBER_OF_BYTES_IN_FLASH_WORD;
    int remaining_bytes = lengthInBytes % NUMBER_OF_BYTES_IN_FLASH_WORD;

    EmbeddedFlashUnlock();

    SET_BIT(FLASH->CR1,
            FLASH_CR_PG); // Enable internal buffer for write operations

    if (number_of_256bit_words != 0) // Program the full flash word ( 256-bits)
    {
        for (int flash_word_index = 0; flash_word_index < number_of_256bit_words; flash_word_index++)
        {
            WaitForLastOperation(FLASH_BANK_1);
            int row_index = FLASH_NB_32BITWORD_IN_FLASHWORD;
            __ISB();
            __DSB();
            do
            {
                *dest_addr = *src_addr;
                dest_addr++;
                src_addr++;
                row_index--;
            } while (row_index != 0);
            __ISB();
            __DSB();
        }
    }

    if (remaining_bytes != 0)
    {
        __IO uint8_t *dest_addr_8_bit = (__IO uint8_t *)(dest_addr);
        __IO uint8_t *src_addr_8_bit = (__IO uint8_t *)(src_addr);
        WaitForLastOperation(FLASH_BANK_1);
        __ISB();
        __DSB();
        do
        {
            *dest_addr_8_bit = *src_addr_8_bit;
            dest_addr_8_bit++;
            src_addr_8_bit++;
            remaining_bytes--;
        } while (remaining_bytes != 0);
        __ISB();
        __DSB();
        SET_BIT(
            FLASH->CR1,
            FLASH_CR_FW); // Force write mechanism
                          // The unwritten bits are automatically set to 1.
    }

    CLEAR_BIT(FLASH->CR1,
              FLASH_CR_PG); // Disable internal buffer for write operations

    EmbeddedFlashLock();
    return true;
}
void WriteFlashWord(uint32_t *aligned_dest_addr, uint32_t *buffer_256bits)
{
    // dest_addr must be aligned on flash word boundary
    // Buffer must contain exactly 32 bytes ( 256 bits)

    // NOTE: Calling method must lock and unlock the flash before calling

    EmbeddedFlashUnlock();
    SET_BIT(FLASH->CR1, FLASH_CR_PG); // Enable internal buffer for write
    WaitForLastOperation(FLASH_BANK_1);
    int row_index = FLASH_NB_32BITWORD_IN_FLASHWORD;
    __ISB();
    __DSB();
    do
    {
        *aligned_dest_addr = *buffer_256bits;
        aligned_dest_addr++;
        buffer_256bits++;
        row_index--;
    } while (row_index != 0);
    __ISB();
    __DSB();
    CLEAR_BIT(FLASH->CR1, FLASH_CR_PG); // Disable internal buffer for write

    EmbeddedFlashLock();
}

bool EmbeddedFlashReadBytes(uint32_t startAddress, uint32_t length, uint8_t *buffer)
{
    __IO uint8_t *cursor = (__IO uint8_t *)startAddress;
    __IO uint8_t *endAddress = (__IO uint8_t *)(startAddress + length);
    while (cursor < endAddress)
    {
        *buffer++ = *cursor++;
    }
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
bool WaitForLastOperation(uint32_t Bank)
{
    // Wait for the FLASH operation to complete by polling on QW flag to be reset.
    // Even if the FLASH operation fails, the QW flag will be reset and an error
    // flag will be set

    uint32_t errorflag = FLASH->SR1 & FLASH_FLAG_ALL_ERRORS_BANK1;
    while (READ_BIT(FLASH->SR1, FLASH_FLAG_QW_BANK1))
    {
        // TODO : (timeout code here) is this necessary?
    }
    if ((errorflag & 0x7FFFFFFFU) != 0U) // In case of error reported in Flash SR1 or SR2 register
    {
        WRITE_REG(FLASH->CCR1, errorflag); // Clear error programming flags
        return false;
    }
    WRITE_REG(FLASH->CCR1,
              FLASH_FLAG_EOP_BANK1); // Clear FLASH End of Operation pending bit
    return true;
}
