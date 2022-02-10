#include <BoardInit.h>
#include <tx_api.h>

// byte pool configuration and definitions
#define DEFAULT_BYTE_POOL_SIZE 6000
#define BLINK_THREAD_STACK_SIZE 1024
#define RECEIVER_THREAD_STACK_SIZE 1024

TX_BYTE_POOL byte_pool_0;
uint8_t memory_area[DEFAULT_BYTE_POOL_SIZE];

// Thread definitions and configurations

// 1. Receiver thread
#define RECEIVER_THREAD_PRIORITY   4

TX_THREAD receiverThread;
uint32_t receiverThreadStack[RECEIVER_THREAD_STACK_SIZE / sizeof(uint32_t)];
extern void ReceiverThread_entry(uint32_t parameter);

// 2. BLINK thread
#define BLINK_THREAD_PRIORITY   5

TX_THREAD blinkThread;
uint32_t blinkThreadStack[BLINK_THREAD_STACK_SIZE / sizeof(uint32_t)];
extern void nanoBooterStatus(uint32_t parameter);


void tx_application_define(void* first_unused_memory)
{
    (void)first_unused_memory;
    uint16_t status;
    CHAR    *pointer = TX_NULL;
 
    
    tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEFAULT_BYTE_POOL_SIZE);                   // Create a byte memory pool from which to allocate the thread stacks.
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, RECEIVER_THREAD_STACK_SIZE, TX_NO_WAIT); /* Allocate the stack for thread 0.  */
    
    
    // Create receiver thread
    status = tx_thread_create(&receiverThread,                     // Pointer to a thread control block.
                "Receiver Thread",                                 // Pointer to the name of the thread.
                   ReceiverThread_entry,                           // Specifies the initial C function for thread execution
                       0,                                          // A 32-bit value that is passed to the thread's entry function when it first executes
    receiverThreadStack,                                           // Starting address of the stack's memory area.
                      RECEIVER_THREAD_STACK_SIZE,                  // Number bytes in the stack memory area.
                             RECEIVER_THREAD_PRIORITY,             // Numerical priority of thread.
                           RECEIVER_THREAD_PRIORITY,               // Highest priority level (0 through (TX_MAX_PRIORITIES-1)) of disabled preemption.
                           TX_NO_TIME_SLICE,                       // Number of timer-ticks this thread is allowed to run before other ready threads of the same priority are given a chance to run.
                   TX_AUTO_START);                                 // Specifies whether the thread starts immediately or is placed in a suspended state.
               if(status != TX_SUCCESS)
    {
        while (1)
        {
        }
    }
    
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, BLINK_THREAD_STACK_SIZE, TX_NO_WAIT); /* Allocate the stack for thread 0.  */
   
        
    // Create blink thread
status = tx_thread_create(&blinkThread,                      // Pointer to a thread control block.
    "Blink Thread",                                          // Pointer to the name of the thread.
    nanoBooterStatus,                                        // Specifies the initial C function for thread execution
    nanoBooterState,                                         // A 32-bit value that is passed to the thread's entry function when it first executes
    blinkThreadStack,                                        // Starting address of the stack's memory area.
    BLINK_THREAD_STACK_SIZE,                                 // Number bytes in the stack memory area. 
    BLINK_THREAD_PRIORITY,                                   // Numerical priority of thread.
    BLINK_THREAD_PRIORITY,                                   // Highest priority level (0 through (TX_MAX_PRIORITIES-1)) of disabled preemption.
    TX_NO_TIME_SLICE,                                        // Number of timer-ticks this thread is allowed to run before other ready threads of the same priority are given a chance to run.
    TX_AUTO_START);                     // Specifies whether the thread starts immediately or is placed in a suspended state. 

if(status != TX_SUCCESS)
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


