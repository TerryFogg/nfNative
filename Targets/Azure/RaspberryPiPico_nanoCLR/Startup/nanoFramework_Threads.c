#include "BOARD.h"
#include <nanoCLR_Application.h>
#include <tx_api.h>
#include <CLR_Startup_Thread.h>
#include <Debug_To_Display.h>

// byte pool configuration and definitions
#define DEFAULT_BYTE_POOL_SIZE 16000
#define CLR_THREAD_STACK_SIZE 6000
#define RECEIVER_THREAD_STACK_SIZE 4096

extern bool g_waitForDebuggerRequested;

TX_BYTE_POOL byte_pool_0;
uint8_t memory_area[DEFAULT_BYTE_POOL_SIZE];

// Thread definitions and configurations

// 1. Receiver thread
#define RECEIVER_THREAD_PRIORITY 4

TX_THREAD receiverThread;
TX_THREAD CLRThread;

uint32_t receiverThreadStack[RECEIVER_THREAD_STACK_SIZE / sizeof(uint32_t)];
extern void ReceiverThread_entry(uint32_t parameter);

// 2. CLR thread
#define CLR_THREAD_PRIORITY 5

uint32_t CLRThreadStack[CLR_THREAD_STACK_SIZE / sizeof(uint32_t)];
extern void CLRStartupThread(uint32_t parameter);

void tx_application_define(void *first_unused_memory)
{
    (void)first_unused_memory;
    uint16_t status;
    CHAR *pointer = TX_NULL;

    tx_byte_pool_create(
        &byte_pool_0,
        "byte pool 0",
        memory_area,
        DEFAULT_BYTE_POOL_SIZE); // Create a byte memory pool from which to allocate the thread stacks.
    tx_byte_allocate(
        &byte_pool_0,
        (VOID **)&pointer,
        RECEIVER_THREAD_STACK_SIZE,
        TX_NO_WAIT); /* Allocate the stack for thread 0.  */

    // Create receiver thread
    status = tx_thread_create(
        &receiverThread,      // Pointer to a thread control block.
        "Receiver Thread",    // Pointer to the name of the thread.
        ReceiverThread_entry, // Specifies the initial C function for thread execution
        0,                    // A 32-bit value that is passed to the thread's entry function when it first executes
        receiverThreadStack,  // Starting address of the stack's memory area.
        RECEIVER_THREAD_STACK_SIZE, // Number bytes in the stack memory area.
        RECEIVER_THREAD_PRIORITY,   // Numerical priority of thread.
        RECEIVER_THREAD_PRIORITY,   // Highest priority level (0 through (TX_MAX_PRIORITIES-1)) of disabled preemption.
        TX_NO_TIME_SLICE, // Number of timer-ticks this thread is allowed to run before other ready threads of the same
                          // priority are given a chance to run.
        TX_AUTO_START);   // Specifies whether the thread starts immediately or is placed in a suspended state.
    if (status != TX_SUCCESS)
    {
        while (1)
        {
        }
    }

    tx_byte_allocate(
        &byte_pool_0,
        (VOID **)&pointer,
        CLR_THREAD_STACK_SIZE,
        TX_NO_WAIT); /* Allocate the stack for thread 0.  */

    // Create CLR thread
    status = tx_thread_create(
        &CLRThread,       // Pointer to a thread control block.
        "CLR_Thread",     // Pointer to the name of the thread.
        CLRStartupThread, // Specifies the initial C function for thread
                          // execution
        g_waitForDebuggerRequested,       // A 32-bit value that is passed to the thread's entry
                               // function when it first executes
        CLRThreadStack,        // Starting address of the stack's memory area.
        CLR_THREAD_STACK_SIZE, // Number bytes in the stack memory area.
        CLR_THREAD_PRIORITY,   // Numerical priority of thread.
        CLR_THREAD_PRIORITY,   // Highest priority level (0 through (TX_MAX_PRIORITIES-1)) of disabled preemption.
        TX_NO_TIME_SLICE, // Number of timer-ticks this thread is allowed to run before other ready threads of the same
                          // priority are given a chance to run.
        TX_AUTO_START);   // Specifies whether the thread starts immediately or is placed in a suspended state.

    if (status != TX_SUCCESS)
    {
        while (1)
        {
        }
    }
}
void Startup_Rtos()
{
    tx_kernel_enter();
}
