//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>
#include <nanoHAL_Boot.h>
#include <nanoHAL_Capabilites.h>
#include <nanoPAL_BlockStorage.h>
#include <targetHAL.h>
#include <stm32h7xx_hal.h>
#include "WireProtocol_communications.h"
#include "BoardInit.h"
#include <LaunchCLR.h>
#include <cmsis_utils.h>

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

    if (InitWireProtocolCommunications() == true)  // NOTE: Don't call  Scheduler type calls in this module
    {
        nanoBooterState = ok;
    }
    else
    {
        nanoBooterState = communications_failure;
    }

    // report successfull nanoBooter execution
    ReportSuccessfullNanoBooter();

    // Mechanism to allow nanoBooter to run and not boot nanoCLR
    bool nanoBooterRequestedByUserButton = (HAL_GPIO_ReadPin(BUTTON_USER_GPIO_PORT, BUTTON_USER_PIN) == GPIO_PIN_RESET);  // Button pushed
    bool nanoBooterRequested = IsToRemainInBooter() || nanoBooterRequestedByUserButton;
    if (!nanoBooterRequested)
    {
        if (CheckValidCLRImage((uint32_t)&__nanoImage_end__))
        {
            LaunchCLR((uint32_t)&__nanoImage_end__);
        }
    }

    Startup_Rtos();
}


