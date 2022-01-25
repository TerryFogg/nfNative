//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>
#include <Target_BlockStorage_STM32FlashDriver.h>

#define FLASH_ERASED_WORD ((uint32_t)0xFFFFFFFF)

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

    volatile uint8_t *cursor = (volatile uint8_t *)startAddress;
    volatile uint8_t *endAddress = (volatile uint8_t *)(startAddress + numBytes);

    // copy contents from flash to buffer starting from the start address
    while (cursor < endAddress)
    {
        *buffer++ = *cursor++;
    }

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

    return true;
}

bool STM32FlashDriver_IsBlockErased(void *context, ByteAddress blockAddress, unsigned int length)
{
    (void)context;

    uint32_t *cursor = (uint32_t *)blockAddress;
    uint32_t *endAddress = (uint32_t *)(blockAddress + length);

    // an erased flash address has to read FLASH_ERASED_WORD
    // OK to check by word (32 bits) because the erase is performed by 'page' whose size is word multiple
    while (cursor < endAddress)
    {
        if (*cursor++ != FLASH_ERASED_WORD)
        {
            // found an address with something other than FLASH_ERASED_WORD!!
            return false;
        }
    }

    // reached here so the segment must be erased
    return true;
}

bool STM32FlashDriver_EraseBlock(void *context, ByteAddress address)
{
    (void)context;

    return true;
}
