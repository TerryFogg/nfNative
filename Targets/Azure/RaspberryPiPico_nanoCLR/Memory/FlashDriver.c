// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "hardware/flash.h"
#include "FlashDriver.h"

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
    flash_range_program(startAddress, buffer, FLASH_PAGE_SIZE);
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
    flash_range_erase(address, FLASH_SECTOR_SIZE);

    return true;
}
