#pragma once

//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoPAL_BlockStorage.h>
#include <stdbool.h>
#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif
bool FlashDriver_InitializeDevice(void *);
bool FlashDriver_UninitializeDevice(void *);
DeviceBlockInfo *FlashDriver_GetDeviceInfo(void *);
bool FlashDriver_Read(void *, ByteAddress startAddress,unsigned int numBytes, unsigned char *buffer);
bool FlashDriver_Write(void *, ByteAddress startAddress,
                            unsigned int numBytes, unsigned char *buffer,bool readModifyWrite);
bool FlashDriver_IsBlockErased(void*, ByteAddress blockAddress, unsigned int length);
bool FlashDriver_EraseBlock(void *, ByteAddress address);

bool EmbeddedFlashUnlock(void);
bool EmbeddedFlashLock(void);
bool EmbeddedFlash_DeploymentErase();

#ifdef __cplusplus
}
#endif
