//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "BOARD.h"
#include <nanoHAL_v2.h>
#include <nanoPAL_BlockStorage.h>

bool g_waitForDebuggerRequested = false;
extern HardFaultReporting g_HardFault;

int main(void)
{
    Initialize_Board();
    g_waitForDebuggerRequested = BoardUserButton_Pressed();
    Startup_Rtos();
}
