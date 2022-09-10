//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoCLR_Headers.h>
#include <Board_STM32H735G-DK.h>

#ifdef STM32H735xx
#include "stm32h735xx.h"
#elif STM32H7B3xx
#include "stm32h7B3xx.h"
#endif

// hard fault handler for Cortex-M, except M0

// dev note: on all the following the variables need to be declared as volatile so they don't get optimized out by the
// linker dev note: the pragma below is to ignore the warning because the variables aren't actually being used despite
// needing to remain there for debug

// See http://infocenter.arm.com/help/topic/com.arm.doc.dui0552a/BABBGBEC.html
typedef enum
{
    Reset = 1,
    NMI = 2,
    HardFault = 3,
    MemManage = 4,
    BusFault = 5,
    UsageFault = 6,
} FaultType;
// Generic ARM register
typedef void *regarm_t;

// This structure represents the stack frame saved during an interrupt handler.
struct port_extctx
{
    regarm_t spsr_irq;
    regarm_t lr_irq;
    regarm_t r0;
    regarm_t r1;
    regarm_t r2;
    regarm_t r3;
    regarm_t r12;
    regarm_t lr_usr;
};

struct HardFaultReporting __attribute__((section(".hard_fault_count"))) g_HardFault;

void NMI_Handler(void)
{
    while (1)
        ;
}
void BusFault_Handler(void) __attribute__((alias("HardFault_Handler")));
void HardFault_Handler(void)
{

    // Copy to local variables (not pointers) to allow GDB "i loc" to directly show the info
    struct port_extctx ctx;

    // Get thread context. Contains main registers including PC and LR
    memcpy(&ctx, (void *)__get_PSP(), sizeof(struct port_extctx));
    (void)ctx;

    // Interrupt status register: Which interrupt have we encountered, e.g. HardFault?
    volatile FaultType faultType = (FaultType)__get_IPSR();

    // these are not available in all the STM32 series

#ifdef DEBUG
    // Flags about hardfault / busfault
    // See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/Cihdjcfc.html for reference
    volatile bool isFaultPrecise = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 1) ? true : false);
    volatile bool isFaultImprecise = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 2) ? true : false);
    volatile bool isFaultOnUnstacking = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 3) ? true : false);
    volatile bool isFaultOnStacking = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 4) ? true : false);
    volatile bool isFaultAddressValid = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 7) ? true : false);

    // Register summaries : https://developer.arm.com/documentation/ddi0489/b/debug/about-debug/debug-register-summary

    // Debug Fault Status Register
    // 31                                          5         4        3         2      1        0
    // --------------------------------------------------------------------------------------------
    // |           RESERVED                         | EXTERNAL | VCATCH | DWTTRAP | BKPT | HALTED |
    // --------------------------------------------------------------------------------------------
    //
    volatile unsigned long _DFSR = (*((volatile unsigned long *)(0xE000ED30)));
#define DFSR_EXTERNAL 16
#define DFSR_VCATCH   5
#define DFSR_DWTTRAP  4
#define DFSR_BKPT     2
#define DFSR_HALTED   1
    volatile bool ExternalDebugRequestDebugEvent = ((_DFSR & DFSR_EXTERNAL) == 1);
    volatile bool VectorCatchTriggered = ((_DFSR & DFSR_VCATCH) == 1);
    volatile bool AtLeastOneDebugEventGeneratedByTheDWT = ((_DFSR & DFSR_DWTTRAP) == 1);
    volatile bool AtLeastOneBreakpointDebugEvent = ((_DFSR & DFSR_BKPT) == 1);
    volatile bool HaltRequestDebugEvent = ((_DFSR & DFSR_HALTED) == 1);

    // Auxiliary Fault Status Register
    // Provides implementation-specific fault status information and control.
    // The contents of this register are IMPLEMENTATION DEFINED.
    volatile unsigned long _AFSR = (*((volatile unsigned long *)(0xE000ED3C)));

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values

    // Hard Fault Status Register
    // --------------------------------
    //    31       30     29                                          2      1        0
    // -------------------------------------------------------------------------------------
    // |DEBUGEVT|FORCED|                                                | VECTTBL | Reserved
    // -------------------------------------------------------------------------------------
    volatile unsigned long _HFSR = (*((volatile unsigned long *)(0xE000ED2C)));
#define DEBUGEVT 0x8000
#define FORCED   0x4000
#define VECTTBL  0x0002
    volatile bool DebugEventHasOccurred = ((_DFSR & DEBUGEVT) == 1);
    volatile bool ProcessorHasEscalatedAConfigurablePriorityExceptionToHardFault = ((_DFSR & FORCED) == 1);
    volatile bool VectorTableReadFaultHasOccurred = ((_DFSR & VECTTBL) == 1);

    // MemManage Fault Address Register
    // --------------------------------
    // Data address for an MPU fault.This is the location addressed by an attempted load or store
    // access that was faulted.The MemManage Status Register shows the cause of the fault, and
    // whether MMFAR.ADDRESS is valid.When an unaligned access faults, the address is the
    // actual address that faulted.Because an access might be split into multiple parts, each
    // aligned, this address can be any offset in the range of the requested size.
    // In implementations without unique BFARand MFAR registers, the value of this register is
    // UNKNOWN if BFSR.BFARVALID is set.
    volatile unsigned long _MMAR = (*((volatile unsigned long *)(0xE000ED34)));

    // For HardFault/BusFault this is the address that was accessed causing the error
    volatile uint32_t faultAddress = SCB->BFAR;

#endif

    // Record Hard Fault
    g_HardFault.count += 1;

    // forces a breakpoint causing the debugger to stop
    // if no debugger is attached this is ignored
    __asm volatile("BKPT #0\n");

    // If no debugger connected, just reset the board
    NVIC_SystemReset();
}
void UsageFault_Handler(void)
{

    // Copy to local variables (not pointers) to allow GDB "i loc" to directly show the info
    // Get thread context. Contains main registers including PC and LR
    struct port_extctx ctx;
    memcpy(&ctx, (void *)__get_PSP(), sizeof(struct port_extctx));
    (void)ctx;

    // Interrupt status register: Which interrupt have we encountered, e.g. HardFault?
    FaultType faultType = (FaultType)__get_IPSR();
    (void)faultType;

    // these are not available in all the STM32 series
#if defined(STM32L4XX_HAL_VERSION)

    // Flags about hardfault / busfault
    // See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/Cihdjcfc.html for reference
    volatile bool isUndefinedInstructionFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 0) ? true : false);
    volatile bool isEPSRUsageFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 1) ? true : false);
    volatile bool isInvalidPCFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 2) ? true : false);
    volatile bool isNoCoprocessorFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 3) ? true : false);
    volatile bool isUnalignedAccessFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 8) ? true : false);
    volatile bool isDivideByZeroFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 9) ? true : false);

#endif

    // forces a breakpoint causing the debugger to stop
    // if no debugger is attached this is ignored
    __asm volatile("BKPT #0\n");

    // If no debugger connected, just reset the board
    NVIC_SystemReset();
}
void MemManage_Handler(void)
{

    // Copy to local variables (not pointers) to allow GDB "i loc" to directly show the info
    // Get thread context. Contains main registers including PC and LR
    struct port_extctx ctx;
    memcpy(&ctx, (void *)__get_PSP(), sizeof(struct port_extctx));
    (void)ctx;

    // Interrupt status register: Which interrupt have we encountered, e.g. HardFault?
    FaultType faultType = (FaultType)__get_IPSR();
    (void)faultType;

    // these are not available in all the STM32 series
#if defined(STM32L4XX_HAL_VERSION)

    // For HardFault/BusFault this is the address that was accessed causing the error
    volatile uint32_t faultAddress = SCB->MMFAR;

    // Flags about hardfault / busfault
    // See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/Cihdjcfc.html for reference
    volatile bool isInstructionAccessViolation = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 0) ? true : false);
    volatile bool isDataAccessViolation = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 1) ? true : false);
    volatile bool isExceptionUnstackingFault = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 3) ? true : false);
    volatile bool isExceptionStackingFault = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 4) ? true : false);
    volatile bool isFaultAddressValid = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 7) ? true : false);

#endif

    // forces a breakpoint causing the debugger to stop
    // if no debugger is attached this is ignored
    __asm volatile("BKPT #0\n");

    // If no debugger connected, just reset the board
    NVIC_SystemReset();
}
