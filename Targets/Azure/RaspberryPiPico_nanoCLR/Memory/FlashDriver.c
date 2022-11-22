//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "FlashDriver.h"

bool FlashDriver_InitializeDevice(void *context) {
  (void)context;

  // nothing to do here
  return true;
}
bool FlashDriver_UninitializeDevice(void *context) {
  (void)context;

  // nothing to do here
  return true;
}
DeviceBlockInfo *FlashDriver_GetDeviceInfo(void *context) {
  MEMORY_MAPPED_NOR_BLOCK_CONFIG *config = context;
  return config->BlockConfig.BlockDeviceInformation;
}
bool FlashDriver_Read(void *context, ByteAddress startAddress,
                      unsigned int numBytes, unsigned char *buffer) {
  (void)context;
  return true;
}
bool FlashDriver_Write(void *context, ByteAddress startAddress,
                       unsigned int numBytes, unsigned char *buffer,
                       bool readModifyWrite) {
  (void)context;
  (void)readModifyWrite;
  return true;
}
bool FlashDriver_IsBlockErased(void *context, ByteAddress blockAddress,
                               unsigned int length) {
  return false;
}
bool FlashDriver_EraseBlock(void *context, ByteAddress address) { return true; }
