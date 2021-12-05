//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>
#include <nanoPAL_BlockStorage.h>
#include "wpUSART_Communications.h"
#include "BoardInit.h"

int main(void)
{
    BoardInit();

    InitBootClipboard();

    // initialize block storage list,devices and configuration manager
    // in CLR this is called in nanoHAL_Initialize()
    // for nanoBooter we have to init it in order to provide the flash map for Monitor_FlashSectorMap command
    BlockStorageList_Initialize();
    BlockStorage_AddDevices();
    ConfigurationManager_Initialize();
    Startup_Rtos();
}


