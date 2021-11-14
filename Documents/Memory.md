
## Azure/nanoFramework memory notes ##


ThreadX requires between 2 KBytes and 20 KBytes of Read Only Memory (ROM) on the target. It also requires another 1 to 2 KBytes of the target's Random Access Memory (RAM) for the ThreadX system stack and other global data structures.


### <b>Notes</b>
 
#### ITCM-RAM (64 KB up to 256kB) ####
1. Decreases critical task execution time, compared to code execution from Flash memory. 
  - This feature can be activated using 
  ```
  #pragma location = ".itcmram"' to be placed above function declaration.
  ```
2. If the application is using the DTCM/ITCM memories 
 - 0x20000000/ 0x0000000: not cacheable and only accessible by the Cortex M7 and the MDMA
 - No need for cache maintenance when the Cortex M7 and the MDMA access these RAMs.
  
3. If the application needs to use DMA (or other masters) based access or requires more RAM, then the user has to:
   - Use a non TCM SRAM. 
     - (example : D1 AXI-SRAM  0x24000000).
   - Add a cache maintenance mechanism to ensure the cache coherence between CPU and other masters (DMAs,DMA2D,LTDC,MDMA).
   - The addresses and the size of cacheable buffers (shared between CPU and other masters) must be properly defined to be aligned to L1-CACHE line size (32 bytes).
4. It is recommended to enable the cache and maintain its coherence:
     - Depending on the use case it is also possible to configure the cache attributes using the MPU.
     - Please refer to the **AN4838** "Managing memory protection unit (MPU) in STM32 MCUs".
     - Please refer to the **AN4839** "Level 1 cache on STM32F7 Series"
  
#### ThreadX usage hints ####

 1. ThreadX uses the Systick as time base and configured with 100 ticks/sec by default. ( 1 tick = 10 milliseconds)
    - It is mandatory that the HAL uses a separate time base through the TIM IPs.
 2. ThreadX is disabling all interrupts during kernel start-up to avoid any unexpected behavior
    - All system related calls (HAL, BSP) should be done either at the beginning of the application or inside the thread entry functions.
 3. ThreadX offers the "tx_application_define()" function and should be used to create all applications ThreadX related resource, bu tno (STM32 HAL or BSP).
    - Automatically called by the tx_kernel_enter() API.
   - Using dynamic memory allocation requires changes to the linker file.
 4. ThreadX needs to pass a pointer to the first free memory location in RAM to the tx_application_define() function, using the "first_unused_memory" argument.
   - This require changes in the linker files to expose this memory location.
 
5. The simplest way to provide memory for ThreadX is to define a new section, see ._threadx_heap..
   - ._threadx_heap must be located between the .bss and the ._user_heap_stack sections in the linker script.	 
  
	``` 
    ._threadx_heap :
      {
         . = ALIGN(8);
         __RAM_segment_used_end__ = .;
         . = . + 64K;
         . = ALIGN(8);
       } >RAM_D1 AT> RAM_D1
	``` 
 6. The "tx_initialize_low_level.s" should be also modified to enable the "USE_DYNAMIC_MEMORY_ALLOCATION" flag.


### Memory protection via Azure RTOS ThreadX Modules ###
An add-on product called Azure RTOS ThreadX Modules enables one or more application threads to be bundled into a “Module” that can be dynamically loaded and run (or executed in place) on the target.

Modules enable field upgrade, bug fixing, and program partitioning to allow large applications to occupy only the memory needed by active threads.

Modules also have a separate address space from Azure RTOS ThreadX itself. 
This enables Azure RTOS ThreadX to place memory protection (via MPU or MMU) around the Module such that accidental access outside the module will not be able to corrupt any other software component.



# STM32H7B3 #



<img src="..\Documents\Notes\nanoFrameworkArchitecture.svg">




 ### Setting up for different variants of a MCU ###


Alias names can be added to existing memory regions created with the MEMORY command. Each name corresponds to at most one memory region.

REGION_ALIAS(alias, region)
The REGION_ALIAS function creates an alias name alias for the memory region region. This allows a flexible mapping of output sections to memory regions. An example follows.

Suppose we have an application for embedded systems which come with various memory storage devices. All have a general purpose, volatile memory RAM that allows code execution or data storage. Some may have a read-only, non-volatile memory ROM that allows code execution and read-only data access. The last variant is a read-only, non-volatile memory ROM2 with read-only data access and no code execution capability. We have four output sections:

.text program code;
.rodata read-only data;
.data read-write initialized data;
.bss read-write zero initialized data.
The goal is to provide a linker command file that contains a system independent part defining the output sections and a system dependent part mapping the output sections to the memory regions available on the system. Our embedded systems come with three different memory setups A, B and C:

Section	Variant A	Variant B	Variant C
.text	RAM	ROM	ROM
.rodata	RAM	ROM	ROM2
.data	RAM	RAM/ROM	RAM/ROM2
.bss	RAM	RAM	RAM
The notation RAM/ROM or RAM/ROM2 means that this section is loaded into region ROM or ROM2 respectively. Please note that the load address of the .data section starts in all three variants at the end of the .rodata section.

The base linker script that deals with the output sections follows. It includes the system dependent linkcmds.memory file that describes the memory layout:

```
INCLUDE linkcmds.memory

SECTIONS
  {
    .text :
      {
        *(.text)
      } > REGION_TEXT
    .rodata :
      {
        *(.rodata)
        rodata_end = .;
      } > REGION_RODATA
    .data : AT (rodata_end)
      {
        data_start = .;
        *(.data)
      } > REGION_DATA
    data_size = SIZEOF(.data);
    data_load_start = LOADADDR(.data);
    .bss :
      {
        *(.bss)
      } > REGION_BSS
  }
```
Now we need three different linkcmds.memory files to define memory regions and alias names. The content of linkcmds.memory for the three variants A, B and C:

A
Here everything goes into the RAM.
```
MEMORY
  {
    RAM : ORIGIN = 0, LENGTH = 4M
  }

REGION_ALIAS("REGION_TEXT", RAM);
REGION_ALIAS("REGION_RODATA", RAM);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
```
B
Program code and read-only data go into the ROM. Read-write data goes into the RAM. An image of the initialized data is loaded into the ROM and will be copied during system start into the RAM.
```
MEMORY
  {
    ROM : ORIGIN = 0, LENGTH = 3M
    RAM : ORIGIN = 0x10000000, LENGTH = 1M
  }

REGION_ALIAS("REGION_TEXT", ROM);
REGION_ALIAS("REGION_RODATA", ROM);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
```
C
Program code goes into the ROM. Read-only data goes into the ROM2. Read-write data goes into the RAM. An image of the initialized data is loaded into the ROM2 and will be copied during system start into the RAM.
```
MEMORY
  {
    ROM : ORIGIN = 0, LENGTH = 2M
    ROM2 : ORIGIN = 0x10000000, LENGTH = 1M
    RAM : ORIGIN = 0x20000000, LENGTH = 1M
  }

REGION_ALIAS("REGION_TEXT", ROM);
REGION_ALIAS("REGION_RODATA", ROM2);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
It is possible to write a common system initialization routine to copy the .data section from ROM or ROM2 into the RAM if necessary:

#include <string.h>

extern char data_start [];
extern char data_size [];
extern char data_load_start [];

void copy_data(void)
{
  if (data_start != data_load_start)
    {
      memcpy(data_start, data_load_start, (size_t) data_size);
    }
}
```


### Thread Stack Area ###
Each thread must have its own stack for saving the context of its last execution and compiler use. Most C compilers use the stack for making function calls and for temporarily allocating local variables. Figure 6 shows a typical thread's stack.

Where a thread stack is located in memory is up to the application. The stack area is specified during thread creation and can be located anywhere in the target's address space. This is an important feature because it allows applications to improve performance of important threads by placing their stack in high-speed RAM.

Stack Memory Area (example)

A thread's stack area must be large enough to accommodate worst-case function call nesting, local variable allocation, and saving its last execution context.

The minimum stack size, TX_MINIMUM_STACK, is defined by ThreadX. A stack of this size supports saving a thread's context and minimum amount of function calls and local variable allocation.

After the application is debugged, it is possible to tune the thread stack sizes if memory is scarce. A favorite trick is to preset all stack areas with an easily identifiable data pattern like (0xEFEF) prior to creating the threads. After the application has been thoroughly put through its paces, the stack areas can be examined to see how much stack was actually used by finding the area of the stack where the data pattern is still intact. Figure 7 shows a stack preset to 0xEFEF after thorough thread execution.



**Example for STM32H735IGKX**
```
/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                1024Kbytes FLASH and 560Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
** All rights reserved.</center></h2>
**
** This software component is licensed by ST under BSD 3-Clause license,
** the "License"; You may not use this file except in compliance with the
** License. You may obtain a copy of the License at:
**                        opensource.org/licenses/BSD-3-Clause
**
****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200 ;      /* required amount of heap  */
_Min_Stack_Size = 0x400 ; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
  ITCMRAM (xrw)    : ORIGIN = 0x00000000,   LENGTH = 64K
  DTCMRAM (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH    (rx)    : ORIGIN = 0x08000000,   LENGTH = 1024K
  RAM_D1  (xrw)    : ORIGIN = 0x24000000,   LENGTH = 320K
  RAM_D2  (xrw)    : ORIGIN = 0x30000000,   LENGTH = 32K
  RAM_D3  (xrw)    : ORIGIN = 0x38000000,   LENGTH = 16K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM_D1 AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D1

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
```
