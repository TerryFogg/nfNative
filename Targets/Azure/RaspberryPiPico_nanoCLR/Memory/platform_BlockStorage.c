//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoPAL_BlockStorage.h>
#include <FlashDriver.h>

// map here the Block Storage Interface to the STM32 driver
IBlockStorageDevice Flash_BlockStorageInterface =
{                          
    &FlashDriver_InitializeDevice,
    &FlashDriver_UninitializeDevice,
    &FlashDriver_GetDeviceInfo,
    &FlashDriver_Read,
    &FlashDriver_Write,
    NULL,
    &FlashDriver_IsBlockErased,    
    &FlashDriver_EraseBlock,
    NULL,
    NULL
};
