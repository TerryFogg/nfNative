#include <targetHAL.h>
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "FlashDriver.h"
#include "hardware/flash.h"
#include <tx_port.h>

bool FlashDriver_InitializeDevice(void *context)
{
    (void)context;

    // nothing to do here
    return true;
}
bool FlashDriver_UninitializeDevice(void *context)
{
    (void)context;

    // nothing to do here
    return true;
}
DeviceBlockInfo *FlashDriver_GetDeviceInfo(void *context)
{
    MEMORY_MAPPED_NOR_BLOCK_CONFIG *config = context;
    return config->BlockConfig.BlockDeviceInformation;
}
bool FlashDriver_Read(void *context, ByteAddress startAddress,
                      unsigned int numBytes, unsigned char *buffer)
{
    (void)context;
    // Read each byte, if it fails the MCU will fault and not return
    uint8_t *cursor = (uint8_t *)startAddress;
    uint8_t *endAddress = (uint8_t *)(startAddress + numBytes);
    while (cursor < endAddress)
    {
        *buffer++ = *cursor++;
    }
    return true;
}
bool FlashDriver_Write(void *context, ByteAddress startAddress,
                       unsigned int numBytes, unsigned char *buffer,
                       bool readModifyWrite)
{
    (void)context;
    (void)readModifyWrite;

    uint32_t flashOffset = startAddress - XIP_MAIN_BASE;
    int numberOfPages = numBytes / FLASH_PAGE_SIZE;
    
    GLOBAL_LOCK()
    
    // By design, all data is aligned and multiple of Flash page size, 
    // except the remaining bytes as the end of the stream
    if (numBytes < FLASH_PAGE_SIZE)
    {
        flash_range_program(flashOffset, buffer, numBytes);
    }
    else
    {
        for (int iPage = 0; iPage < numberOfPages; iPage++)
        {
            flash_range_program(flashOffset, buffer, FLASH_PAGE_SIZE);
            flashOffset += FLASH_PAGE_SIZE;
            buffer += FLASH_PAGE_SIZE;
        }
    }

    GLOBAL_UNLOCK()
    
    return true;
}
bool FlashDriver_IsBlockErased(void *context, ByteAddress blockAddress,
                               unsigned int length)
{
    return false;
}
// On the RP2040, flash blocks are 4096 bytes, and all operations are aligned to and in multiples of 4096 bytes
bool FlashDriver_EraseBlock(void *context, ByteAddress address)
{
    uint32_t flashOffset = address - XIP_MAIN_BASE;
    
    GLOBAL_LOCK()
    
    flash_range_erase(flashOffset, FLASH_SECTOR_SIZE);
    
    GLOBAL_UNLOCK()
    
    return true;
}
