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
    return false;
}
bool STM32FlashDriver_EraseBlock(void *context, ByteAddress address)
{
    return true;
}
bool EmbeddedFlashErase(uint32_t sector)
{
    return true;
}
bool EmbeddedFlashUnlock(void)
{
    return true;
}
bool EmbeddedFlashLock(void)
{
}

static int entry = 0;


bool EmbeddedFlashWrite(uint32_t startAddress, uint32_t lengthInBytes, uint8_t *DataAddress)
{
    return true;
}
void WriteFlashWord(uint32_t *aligned_dest_addr, uint32_t *buffer_256bits)
{
}

bool EmbeddedFlashReadBytes(uint32_t startAddress, uint32_t length, uint8_t *buffer)
{
    return true;
}
uint32_t GetSector(uint32_t Address)
{
    return 1;
}
bool WaitForLastOperation(uint32_t Bank)
{
    return true;
}
