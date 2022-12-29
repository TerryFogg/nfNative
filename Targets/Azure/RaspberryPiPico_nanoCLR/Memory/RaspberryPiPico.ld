/*
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
*/
/* Note:                                            */
/* SYS>>   Bare metal code or RTOS                  */
/* CLR>>   Use by Common Language Runtime           */

/*       /------------------------------------\     */
/*       |  Entry Point                       |     */  ENTRY(_entry_point)
/*       \------------------------------------/     */

/*       /------------------------------------\     */
/*       |  PARAMETERS                        |     */   
/*       | ================================== |     */
/*       |                                    |     */   
/*       |                                    |     */   
/*       |------------------------------------|     */   
/*       | The flash sector must be aligned to|SYS>>*/  flash_sector_size  = 4096 ; 
/*       | a 4096-byte sector and count must  |     */
/*       | be a multiple of 4096 bytes.       |     */
/*       |                                    |     */   
/*       | Each flash write must be aligned   |SYS>>*/  flash_page_size    = 256;
/*       | to a 256-byte boundary and count   |     */
/*       | must be a multiple of 256.         |     */
/*       |                                    |     */   
/*       | Number of flash pages on the RP2040|SYS>>*/  Total_flash_page_count   = 512;
/*       |------------------------------------|     */
/*       | Configuration to store             |     */   
/*       | persistent information             |     */   
/*       | Use the minimum erase size 1 sector|CLR>>*/  config_size =  1;
/*       | Keep in sync with                  |     */   
/*       | "targetHAL_ConfigurationManger.cpp"|     */   
/*       |                                    |     */   
/*       |                                    |     */   
/*       |  Graphics Frame buffer not used    |     */  graphics_Frame_Buffer_not_used = -77; 
/*       |  Direct drawing from graphics      |     */  
/*       |  screen memory                     |     */   
/*       |                                    |     */   
/*       |                                    |     */   
/*       \------------------------------------/     */


MEMORY    /*   Usage by nanoFramework */
{
/*       /------------------------------------\     */
/*       | Raspberry Pi Pico Physical memory  |     */
/*       | =================================  |     */
/*       |                                    |     */   
/*       | 0x00000000 : 16KB ROM              |SYS>>*/  /* Factory System Memory */
/*       |                                    |     */  /* The ROM contents are fixed at the time the silicon is */
/*       |                                    |     */  /* manufactured. It contains:                            */
/*       |                                    |     */  /*  • Initial startup routine                            */
/*       |                                    |     */  /*  • Flash boot sequence                                */
/*       |                                    |     */  /*  • Flash programming routines                         */
/*       |                                    |     */  /*  • USB mass storage device with UF2 support           */
/*       |                                    |     */  /*  • Utility libraries such as fast floating point      */
/*       |                                    |     */   
/*       |------------------------------------|     */
/*       | SRAM 256KB addressed as striped    |     */
/*       | memory OR non striped memory       |     */   
/*       |....................................|     */
/*       | STRIPED Bank     : 256KB           |SYS>>*/  RAM_STRIPED (xrw) : ORIGIN = 0x20000000, LENGTH = 256k  
/*       |      OR                            |     */ 
/*       | Non striped memory                 |     */   
/*       |....................................|     */
/*       | NON STRIPED Bank : 64KB            |SYS>>*/  RAM_NONSTRIPED_0  (xrw) : ORIGIN = 0x21000000, LENGTH = 64k  
/*       | NON STRIPED Bank : 64KB            |SYS>>*/  RAM_NONSTRIPED_1  (xrw) : ORIGIN = 0x21010000, LENGTH = 64k  
/*       | NON STRIPED Bank : 64KB            |SYS>>*/  RAM_NONSTRIPED_2  (xrw) : ORIGIN = 0x21020000, LENGTH = 64k  
/*       | NON STRIPED Bank : 64KB            |SYS>>*/  RAM_NONSTRIPED_3  (xrw) : ORIGIN = 0x21030000, LENGTH = 64k  
/*       |                                    |     */ 
/*       | Additional 4kB regions             |     */   
/*       | Software may choose to use these   |     */   
/*       | for per-core purposes,stack and    |     */   
/*       | frequently-executed code           |     */   
/*       | Bank : 4KB                         |SYS>>*/  SCRATCH_X   (xrw) : ORIGIN = 0x20040000, LENGTH = 4k  
/*       | Bank : 4KB                         |SYS>>*/  SCRATCH_Y   (xrw) : ORIGIN = 0x20041000, LENGTH = 4k  
/*       |                                    |     */   
/*       |------------------------------------|     */
/*       | Dedicated RAM blocks that may be   |     */
/*       | used in some circumstances:        |     */   
/*       |                                    |     */   
/*       | If flash XIP caching is disabled   |     */   
/*       | 0x15000000 : 4kB                   |SYS>>*/  XIP_CACHING (xrw) : ORIGIN = 0x15000000,   LENGTH = 16k
/*       |                                    |     */   
/*       | If USB is not used                 |     */   
/*       | 0x50100000 : 4kB                   |SYS>>*/  DPRAM       (xrw) : ORIGIN = 0x50100000,   LENGTH = 4k
/*       |                                    |     */   
/*       |------------------------------------|     */
/*       | External Flash                     |     */
/*       | accessed via the QSPI interface    |     */
/*       | with address space to 16MB.        |     */   
/*       | RP2040 has 2048K of QSPI flash     |     */   
/*       |                                    |     */   
/*       | 0x10000000 : 256KB                 |SYS>>*/  FLASH        (rx) : ORIGIN = 0x10000000,   LENGTH = 2048k
/*       | Flash has 512 pages of 4096 Bytes  |     */   
/*       | on the RP2040, smallest erase 1    |     */ 
/*       | Sector,4096 bytes aligned on       |     */ 
/*       | 4096 byte boundary                 |     */ 
/*       |------------------------------------|     */
/*       |                                    |     */
/*       |                                    |     */
/*       | nanoFramework                      |     */
/*       |====================================|     */
/*       |                                    |     */
/*       | FLASH LAYOUT                       |     */
/*       |------------------------------------|     */
/*       | Stage 2 boot is prepended at the   |     */   
/*       | start                              |     */   
/*       | .................................. |     */
/*       | Sectors counting from 0            |     */
/*       | .................................. |     */
/*       | Sector  0 (First 256 bytes)        |SYS>>*/  STAGE2_BOOT  (rx)  : ORIGIN = ORIGIN(FLASH), LENGTH = 252 + 4  /* contains prepended bs2_default_padded_checksummed.S */
/*       | Sectors   0 - 159                  |CLR>>*/  nanoCLR      (rx)  : ORIGIN = ORIGIN(STAGE2_BOOT) + LENGTH(STAGE2_BOOT), LENGTH = 160*flash_sector_size - LENGTH(STAGE2_BOOT)
/*       | Sectors 160 - 160                  |CLR>>*/  config       (rx)  : ORIGIN = ORIGIN(nanoCLR)     + LENGTH(nanoCLR)    , LENGTH = config_size*flash_sector_size
/*       | Sectors 161 - 511                  |CLR>>*/  deployment   (rx)  : ORIGIN = ORIGIN(config)      + LENGTH(config)     , LENGTH = Total_flash_page_count*flash_page_size - LENGTH(STAGE2_BOOT) - LENGTH(nanoCLR) - LENGTH(config)
/*       |                                    |     */
/*       |------------------------------------|     */
/*       |                                    |     */
/*       | RAM LAYOUT                         |     */
/*       |------------------------------------|     */
/*       |                                    |     */
/*       | Native code ram (Azure, nativeCLR) |CLR>>*/  RAM            (xrw) : ORIGIN = ORIGIN(RAM_STRIPED), LENGTH = 80k
/*       | CLR managed heap ram               |CLR>>*/  CLR_RAM        (xrw) : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), LENGTH = 70k
/*       | nanoCLR graphics bitmaps,          |CLR>>*/  graphics_ram   (xrw) : ORIGIN = ORIGIN(CLR_RAM) + LENGTH(CLR_RAM), LENGTH = LENGTH(RAM_STRIPED) - ( LENGTH(RAM) + LENGTH(CLR_RAM) )
/*       | drawing surfaces                   |     */
/*       |                                    |     */
/*       | Video frame buffer                 |CLR>>*/  /* NO FRAME BUFFER - use SPI buffer to transfer graphics */
/*       \------------------------------------/     */
}
        
/*       /------------------------------------\     */
/*       |  VARIABLES REFERENCED BY C/C++     |     */   
/*       | ================================== |     */
/*       | The CLR managed heap for CLR       |CLR>>*/  HeapBegin             = ORIGIN(CLR_RAM); 
/*       | objects                            |CLR>>*/  HeapEnd               = ORIGIN(CLR_RAM) + LENGTH(CLR_RAM); 
/*       |------------------------------------|     */   
/*       | CLR Image                          |CLR>>*/  __nanoImage_start__   = ORIGIN(nanoCLR); 
/*       |                                    |CLR>>*/  __nanoImage_end__     = ORIGIN(nanoCLR) + LENGTH(nanoCLR); 
/*       |------------------------------------|     */   
/*       | Flash region where the C#          |CLR>>*/  __deployment_start__  = ORIGIN(deployment); 
/*       | Portable Executable (PE) file      |CLR>>*/  __deployment_end__    = ORIGIN(deployment) + LENGTH(deployment); 
/*       | is stored                          |CLR>>*/  __deployment_size__   = LENGTH(deployment); 
/*       |------------------------------------|     */   
/*       | Small area of persistent storage   |     */  __nanoConfig_start__  = ORIGIN(config);
/*       |                                    |     */  __nanoConfig_end__    = ORIGIN(config) + LENGTH(config);
/*       |------------------------------------|     */   
/*       | Bit pixel copy of the screen       |CLR>>*/  Graphics_frame_buffer = graphics_Frame_Buffer_not_used;
/*       | Updated by DMA                     |     */    
/*       |------------------------------------|     */   
/*       | Bitmap objects created by C#       |CLR>>*/  Graphics_managed_heap = ORIGIN(graphics_ram); 
/*       | and drawn into                     |CLR>>*/  Graphics_managed_end  = ORIGIN(graphics_ram) + LENGTH(graphics_ram);   
/*       \------------------------------------/     */



/* Define output sections */
SECTIONS
{

  /* ----------------------------------------*/
  /* The startup code goes first into FLASH  */
  /* ----------------------------------------*/
  /* Second stage bootloader is prepended to the image. It must be 256 bytes   */
  /* and check summed. It is usually built by the boot_stage2 target           */
  /* in the Raspberry Pi Pico SDK                                              */

    .flash_begin : {
        __flash_binary_start = .;
    } > FLASH

    .boot2 : {
        __boot2_start__ = .;
        KEEP (*(.boot2))
        __boot2_end__ = .;
    } > FLASH

     ASSERT(__boot2_end__ - __boot2_start__ == 256, "ERROR: Pico second stage bootloader must be 256 bytes in size")

    /* The second stage will always enter the image at the start of .text.   */
    /* The debugger will use the ELF entry point, which is the _entry_point  */
    /* symbol if present, otherwise defaults to start of .text               */
    /* This can be used to transfer control back to the boot rom on debugger */
    /* launches only, to perform proper flash setup                          */
    /*                                                                       */

    .text : {
        __logical_binary_start = .;
        KEEP (*(.vectors))
        KEEP (*(.binary_info_header))
        __binary_info_header_end = .;
        KEEP (*(.reset))
        /* TODO revisit this now memset/memcpy/float in ROM */
        /* bit of a hack right now to exclude all floating point and time critical (e.g. memset, memcpy) code from
         * FLASH ... we will include any thing excluded here in .data below by default */
        *(.init)
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .text*)
        *(.fini)
        /* Pull all c'tors into .text */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        /* Followed by destructors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.eh_frame*)
        . = ALIGN(4);
    } > FLASH

  /*-----------------------------------------*/
  /* Read only data, constants, strings etc. */
  /*-----------------------------------------*/
    .rodata : {
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .rodata*)
        . = ALIGN(4);
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.flashdata*)))
        . = ALIGN(4);
    } > FLASH

  /* --------------------------------------------------------------------------------------*/
  /* ARM special sections                                                                  */
  /* Each function that can throw an exception will have entries in the following sections */
  /* used for C++ and for C programs that try to examine backtraces                        */
  /* --------------------------------------------------------------------------------------*/
  .ARM.extab :                                   /* extab names sections containing exception unwinding information */
  {
   *(.ARM.extab* .gnu.linkonce.armextab.*) 
   } >FLASH
  
  .ARM : 
  {
    __exidx_start = .;                            /* exidx is used to contain index entries for stack unwinding */
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } >FLASH
    __exidx_end = .;

  /* --------------------------------------- */
  /* Machine inspectable binary information  */
  /* --------------------------------------- */
    . = ALIGN(4);
    __binary_info_start = .;
    .binary_info :
    {
        KEEP(*(.binary_info.keep.*))
        *(.binary_info.*)
    } > FLASH
    __binary_info_end = .;
    . = ALIGN(4);

    /* End of .text-like segments */
    __etext = .;

   .ram_vector_table (COPY): {
        *(.ram_vector_table)
    } > RAM

  /* -----------------------------------------------------------------------*/
  /* Holds initialized data that contributes to the program's memory image  */
  /* -----------------------------------------------------------------------*/
    .data : {
        __data_start__ = .;
        *(vtable)

        *(.time_critical*)

        /* remaining .text and .rodata; i.e. stuff we exclude above because we want it in RAM */
        *(.text*)
        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);

        *(.data*)

        . = ALIGN(4);
        *(.after_data.*)
        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__mutex_array_start = .);
        KEEP(*(SORT(.mutex_array.*)))
        KEEP(*(.mutex_array))
        PROVIDE_HIDDEN (__mutex_array_end = .);

  /* ----------------------------------*/
  /*   Initialization of C/C++ objects
  /* ----------------------------------*/

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(SORT(.preinit_array.*)))
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

  /* ----------------------------------------------------------------------*/
  /* Tear down of C/C++ objects                                            */
  /* C++ runtime destructors for static variables, C designated finalizers */
  /* ----------------------------------------------------------------------*/
        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.jcr)
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;
    } > RAM AT> FLASH

     .uninitialized_data (COPY): {
        . = ALIGN(4);
        *(.uninitialized_data*)
    } > RAM

    /* Start and end symbols must be word-aligned */
    .scratch_x : {
        __scratch_x_start__ = .;
        *(.scratch_x.*)
        . = ALIGN(4);
        __scratch_x_end__ = .;
    } > SCRATCH_X AT > FLASH
    __scratch_x_source__ = LOADADDR(.scratch_x);

    .scratch_y : {
        __scratch_y_start__ = .;
        *(.scratch_y.*)
        . = ALIGN(4);
        __scratch_y_end__ = .;
    } > SCRATCH_Y AT > FLASH
    __scratch_y_source__ = LOADADDR(.scratch_y);
 
  /* -------------------------------------------------------------------*/
  /* This section holds uninitialized data                             */
  /* By definition, and through the code in startup_st32h735igks.s, the */
  /* system initializes the data with zeros at startup                  */
  /* -------------------------------------------------------------------*/
  .bss :       
  {
    /* This is used by the startup in order to initialize the .bss section */
        . = ALIGN(4);
        __bss_start__ = .;
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .heap (COPY):
    {
        __end__ = .;
        end = __end__;
        *(.heap*)
        __HeapLimit = .;
    } > RAM

/* -----------------------------------------------------------------*/
/* Not using "DYNAMIC_MEMORY" so no need for a ._threadx_heap here.
/* -----------------------------------------------------------------*/

    /* .stack*_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later
     *
     * stack1 section may be empty/missing if platform_launch_core1 is not used */

    /* By default we put 
       core 0 stack at the end of scratch Y
       so that if core 1 stack is not used then all 
       of SCRATCH_X is free.
     */
    .stack1_dummy (COPY):
    {
        *(.stack1*)
    } > SCRATCH_X
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SCRATCH_Y

    .flash_end : {
        __flash_binary_end = .;
    } > FLASH

    /* stack limit is poorly named, but historically is maximum heap ptr */
    __StackLimit = ORIGIN(RAM) + LENGTH(RAM);
    __StackOneTop = ORIGIN(SCRATCH_X) + LENGTH(SCRATCH_X);
    __StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);
    __StackOneBottom = __StackOneTop - SIZEOF(.stack1_dummy);
    __StackBottom = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed")

    ASSERT( __binary_info_header_end - __logical_binary_start <= 256, "Binary info must be in first 256 bytes of the binary")
    
    
    /* Add additional assert(s) on extra code below */
}

