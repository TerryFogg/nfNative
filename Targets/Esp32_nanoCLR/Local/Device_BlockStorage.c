//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_Types.h>
#include <nanoPAL_BlockStorage.h>

BlockStorageDevice Device_BlockStorage;

// Block Size 128KB (Smallest erase size)
const BlockRange BlockRange1[] = {
    {BlockRange_BLOCKTYPE_CODE, 0, 4},
    {BlockRange_BLOCKTYPE_DEPLOYMENT, 5, 7},
};

const BlockRegionInfo BlockRegions[] = {
    {
        (BlockRegionAttribute_ProgramWidthIs256bits),
        0x08000000,                        // start address for block region
        8,                                 // total number of blocks in this region
        0x20000,                           // total number of bytes per block
        ARRAYSIZE_CONST_EXPR(BlockRange1), // size
        BlockRange1,                       // Address
    },
};

const DeviceBlockInfo Device_BlockInfo = {
    (MediaAttribute_SupportsXIP),       // STM32 flash memory is XIP
    2,                                  // UINT32 BytesPerSector
    ARRAYSIZE_CONST_EXPR(BlockRegions), // UINT32 NumRegions;
    (BlockRegionInfo *)BlockRegions,    // const BlockRegionInfo* pRegions;
};

MEMORY_MAPPED_NOR_BLOCK_CONFIG Device_BlockStorageConfig = {
    {
        // BLOCK_CONFIG
        {
            0,     // GPIO_PIN             Pin;
            false, // BOOL                 ActiveState;
        },
        (DeviceBlockInfo *)&Device_BlockInfo, // BlockDeviceinfo
    },
    {
        // CPU_MEMORY_CONFIG
        0,          // UINT8  CPU_MEMORY_CONFIG::ChipSelect;
        true,       // UINT8  CPU_MEMORY_CONFIG::ReadOnly;
        0,          // UINT32 CPU_MEMORY_CONFIG::WaitStates;
        0,          // UINT32 CPU_MEMORY_CONFIG::ReleaseCounts;
        16,         // UINT32 CPU_MEMORY_CONFIG::BitWidth;
        0x08000000, // UINT32 CPU_MEMORY_CONFIG::BaseAddress;
        0x00200000, // UINT32 CPU_MEMORY_CONFIG::SizeInBytes;
        0,          // UINT8  CPU_MEMORY_CONFIG::XREADYEnable
        0,          // UINT8  CPU_MEMORY_CONFIG::ByteSignalsForRead
        0,          // UINT8  CPU_MEMORY_CONFIG::ExternalBufferEnable
    },
    0, // UINT32 ChipProtection;
    0, // UINT32 ManufacturerCode;
    0, // UINT32 DeviceCode;
};
