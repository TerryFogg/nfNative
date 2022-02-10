//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
#include "LaunchCLR.h"
#include <vectors.h>

bool ValidCLRImage()
{
    // Check for valid nanoCLR signature

    return true;
}
void LaunchCLR()
{

  extern void *__nanoCLR__[];
  ((void (*)())__nanoCLR__[1])();
  
  //    vectors_t *nanoCLRVectorTable = (vectors_t *)__nanoCLR__;      // load
  //    nanoCLR vector table irq_vector_t JumpToNanoCLR =
  //    nanoCLRVectorTable->ResetHandler; // Start at Reset Handler ISR
  //    __asm volatile("CPSID i\n");                                   //
  //    Disable interrupts
  //    __set_MSP((uint32_t)nanoCLRVectorTable->InitStack);            // need
  //    to set stack pointer from CLR vector table JumpToNanoCLR();
}
