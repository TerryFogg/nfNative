// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.

#include <targetHAL.h>
#include <nanoHAL_v2.h>
#include <nanoCLR_Application.h>
#include <nanoPAL_BlockStorage.h>

__attribute__((noreturn)) void CLRStartupThread(void const *argument)
{
    CLR_SETTINGS *clrSettings = (CLR_SETTINGS *)argument;
    nanoHAL_Initialize_C();              // Initialize nanoHAL
    ClrStartup(*clrSettings);
    
    while (1)                            // loop until thread receives a request to terminate
        {
            OS_DELAY(500);                    // this function never returns
        }
}
