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
#include <tx_api.h>
#include <LaunchCLR.h>
#include <cmsis_utils.h>


extern TX_EVENT_FLAGS_GROUP wpUartEvent;

// byte pool configuration and definitions
#define DEFAULT_BYTE_POOL_SIZE 4096
TX_BYTE_POOL byte_pool_0;
uint8_t memory_area[DEFAULT_BYTE_POOL_SIZE];

// threads definitions and configurations

// receiver thread
#define RECEIVER_THREAD_STACK_SIZE 2048
#define RECEIVER_THREAD_PRIORITY   4

TX_THREAD receiverThread;
uint32_t receiverThreadStack[RECEIVER_THREAD_STACK_SIZE / sizeof(uint32_t)];
extern void ReceiverThread_entry(uint32_t parameter);

// blink thread
#define BLINK_THREAD_STACK_SIZE 1024
#define BLINK_THREAD_PRIORITY   5

TX_THREAD blinkThread;
uint32_t blinkThreadStack[BLINK_THREAD_STACK_SIZE / sizeof(uint32_t)];

eBooterStatus nanoBooterState;  // Global to this module


void BlinkThread_entry(uint32_t parameter)
{
    (void)parameter;

    while (1)
    {
        tx_thread_sleep(50);
    }
}
void tx_application_define(void* first_unused_memory)
{
    (void)first_unused_memory;
    uint16_t status;

    systick_interval_set(TX_TIMER_TICKS_PER_SECOND);

    // Create a byte memory pool from which to allocate the thread stacks.
    tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEFAULT_BYTE_POOL_SIZE);


    // Create blink thread
    status = tx_thread_create(&blinkThread,             // Pointer to a thread control block.
        "Blink Thread",           // Pointer to the name of the thread.
        nanoBooterStatus,         // Specifies the initial C function for thread execution
        nanoBooterState,          // A 32-bit value that is passed to the thread's entry function when it first executes
        blinkThreadStack,         // Starting address of the stack's memory area.
        BLINK_THREAD_STACK_SIZE,  // Number bytes in the stack memory area. 
        BLINK_THREAD_PRIORITY,    // Numerical priority of thread.
        BLINK_THREAD_PRIORITY,    // Highest priority level (0 through (TX_MAX_PRIORITIES-1)) of disabled preemption.
        TX_NO_TIME_SLICE,         // Number of timer-ticks this thread is allowed to run before other ready threads of the same priority are given a chance to run.
        TX_AUTO_START);           // Specifies whether the thread starts immediately or is placed in a suspended state. 

    if (status != TX_SUCCESS)
    {
        while (1)
        {
        }
    }
    /*
        // Create receiver thread
        status = tx_thread_create( &receiverThread,             // Pointer to a thread control block.
                                   "Receiver Thread",			// Pointer to the name of the thread.
                                   ReceiverThread_entry,		// Specifies the initial C function for thread execution
                                   0,							// A 32-bit value that is passed to the thread's entry function when it first executes
                                   receiverThreadStack,			// Starting address of the stack's memory area.
                                   RECEIVER_THREAD_STACK_SIZE,	// Number bytes in the stack memory area.
                                   RECEIVER_THREAD_PRIORITY,	// Numerical priority of thread.
                                   RECEIVER_THREAD_PRIORITY,	// Highest priority level (0 through (TX_MAX_PRIORITIES-1)) of disabled preemption.
                                   TX_NO_TIME_SLICE,			// Number of timer-ticks this thread is allowed to run before other ready threads of the same priority are given a chance to run.
                                   TX_AUTO_START);				// Specifies whether the thread starts immediately or is placed in a suspended state.

                                   */
    if (status != TX_SUCCESS)
    {
        while (1)
        {
        }
    }

    // create UART event group
    status = tx_event_flags_create(&wpUartEvent, "wpUart event");
    if (status != TX_SUCCESS)
    {
        while (1)
        {
        }
    }
}


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
    tx_kernel_enter();  // Enter the ThreadX kernel

}


