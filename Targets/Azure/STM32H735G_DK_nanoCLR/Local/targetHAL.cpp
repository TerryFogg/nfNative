//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoPAL.h>
#include <nanoHAL_Time.h>
#include <nanoHAL_Types.h>
#include <target_platform.h>
#include <nanoPAL_Events.h>
#include <nanoPAL_BlockStorage.h>
#include <nanoHAL_ConfigurationManager.h>
#include <nanoHAL_Graphics.h>
#include <nanoHAL_v2.h>
#include <platform_target_capabilities.h>
#include <TargetFeatures.h>

// global mutex protecting the internal state of the interpreter, including event flags
// mutex_t interpreterGlobalMutex;

// because nanoHAL_Initialize/Uninitialize needs to be called in both C and C++ we need a proxy to allow it to be called
// in 'C'
extern "C"
{

    void nanoHAL_Initialize_C()
    {
        nanoHAL_Initialize();
    }

    void nanoHAL_Uninitialize_C()
    {
        nanoHAL_Uninitialize();
    }
}

void nanoHAL_Initialize()
{
    // initialize global mutex
    // chMtxObjectInit(&interpreterGlobalMutex);

    Initialize_Audio_Features();
    Initialize_microSD();
    Initialize_USB();
    Initialize_Ethernet();
    Initialize_FDCAN();

    PalEvent_Initialize();

    HAL_CONTINUATION::InitializeList();
    HAL_COMPLETION::InitializeList();

    BlockStorageList_Initialize();
    BlockStorage_AddDevices();
    BlockStorageList_InitializeDevices();
    unsigned char *heapStart = NULL;
    unsigned int heapSize = 0;

    ::HeapLocation(heapStart, heapSize);
    memset(heapStart, 0, heapSize);

//    ConfigurationManager_Initialize();

    Events_Initialize();

    CPU_GPIO_Initialize();

    // Initialise Network Stack
    Network_Initialize();
}

void nanoHAL_Uninitialize()
{
    // release the global mutex, just in case it's locked somewhere
    // chMtxUnlock(&interpreterGlobalMutex);

    // TODO check for s_rebootHandlers
    // for(int i = 0; i< ARRAYSIZE(s_rebootHandlers); i++)
    // {
    //     if(s_rebootHandlers[i] != NULL)
    //     {
    //         s_rebootHandlers[i]();
    //     }
    //     else
    //     {
    //         break;
    //     }
    // }

    SOCKETS_CloseConnections();


    BlockStorageList_UnInitializeDevices();

    // need to be sure that:
    // - all mutexes for drivers that use them are released
    // - all drivers are stopped

    CPU_GPIO_Uninitialize();

    Events_Uninitialize();

    HAL_CONTINUATION::Uninitialize();
    HAL_COMPLETION::Uninitialize();
}



void HAL_AssertEx()
{
    __asm("BKPT #0\n");
    while (true)
    {
        /*nop*/
    }
}

#if !defined(BUILD_RTM)

void HARD_Breakpoint()
{
    __asm("BKPT #0\n");
    while (true)
    {
        /*nop*/
    }
}
;

#endif // !defined(BUILD_RTM)

// provide platform level "weak" implementations for all capabilities
__nfweak TARGET_IFU_CAPABLE(false);

// STM32 default capability is JTAG update
// declared as "weak" to allow targets to provide hard implementation
__nfweak GET_TARGET_CAPABILITIES(TargetCapabilities_JtagUpdate);
