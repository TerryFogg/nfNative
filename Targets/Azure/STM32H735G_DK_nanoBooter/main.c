//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoPAL_BlockStorage.h>
#include "BoardInit.h"
#include "LaunchCLR.h"

extern uint32_t __nanoCLR__;

int main(void)
{
    BoardInit();
    InitBootClipboard();

    if (IsToRemainInBooter() || UserButtonPressed() || !ValidCLRImage())
    {
        BlockStorageList_Initialize();
        BlockStorage_AddDevices();
        ConfigurationManager_Initialize();
        Startup_Rtos();
    }
    else
    {
      LaunchCLR();
    }
}
