// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.

#include <nanoCLR_Application.h>
#include <nanoHAL_v2.h>
#include <nanoPAL_BlockStorage.h>
#include <targetHAL.h>

__attribute__((noreturn)) void CLRStartupThread(void const *argument) {

  bool userRequestedWaitForDebugger = (bool)argument; // not used

  // CLR settings to launch CLR thread
  CLR_SETTINGS clrSettings;
  (void)memset(&clrSettings, 0, sizeof(CLR_SETTINGS));
  clrSettings.MaxContextSwitches = 50;
  clrSettings.EnterDebuggerLoopAfterExit = true;
  clrSettings.WaitForDebugger = userRequestedWaitForDebugger;

  nanoHAL_Initialize_C(); // Initialize nanoHAL
  ClrStartup(clrSettings);

  while (1) // loop until thread receives a request to terminate
  {
    OS_DELAY(500); // this function never returns
  }
}
