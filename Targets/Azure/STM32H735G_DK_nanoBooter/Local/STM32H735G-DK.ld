/*
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
*/

/* Entry Point */
ENTRY(Reset_Handler)

 /* set stack pointer */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* Highest address of the user mode stack */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200 ;      /* required amount of heap  */
_Min_Stack_Size = 0x400 ; /* required amount of stack */

__main_stack_size__     = 0x400;
__process_stack_size__  = 0x800;
__crt_heap_size__       = 0x800;


/* The STM32H7 device consists of three bus matrix domains */
/* - D1, D2 and D3 */
/* - The D1 and D2 are connected through bus bridges, both can also access data in D3 domain. */
/* - There is no connection from D3 domain to D1 or D2 domain. */
/* The DMA1 and DMA2 controllers are located in D2 domain and can access almost all memories */
/* with exception of ITCM and DTCM RAM (located at 0x20000000).*/
/* From performance point of view it is better to put DMA buffers inside D2 domain  */
/* since the D2-to-D1 bridge can add additional delay.*/

/* BDMA controller is located in D3 domain and can access only SRAM4 and backup SRAM in the D3 domain. */
/* MDMA controller is located in D1 domain and can access all memories, including ITCM/DTCM. */
/* Some peripherals are D1 or D2 domain while others are D3 */


_nanoBooterImageSize = 35K;
_nanoCLRImageSize = 550K;

MEMORY
{
/*--------------------------------------*/
/* STM32H735G-DK Physical memory        */
/* location and size for reference      |   Usage by nanoFramework                               */
/*--------------------------------------|--------------------------------------------------------*/
/*                                      */   
/* ITCM-RAM  can be used to host        */
/* critical real-time code such as      */
/* interrupt handlers                   */
/*--------------------------------------*/
/* 0x00000000 : 64K ITCM SRAM           */   ramvt      (wx) : org = 0x00000000, LENGTH = 0                     /* initial RAM address is reserved for a copy of the vector table */
/*                                      */   /* To be investigated */
/*                                      */ 
/*                                      */ 
/* 0x08FFF000 : 1K Flash User OTP       */  /* Possible one time programmable use ?*/
/* 0x08FFFC00 : 512 Flash STM Read      */  /* System Not useable */
/*                                      */ 
/*--------------------------------------*/
/* Smallest erase 1 Sector, so align    */ 
/* deployment on sector boundary        */ 
/* 8 x Sectors, each 128K               */ 
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* 0x08000000 : 1024K AXI Flash         */  FLASH      (rx) : ORIGIN = 0x08000000,         LENGTH = 1024k
/*  Sector 0 : 128K                     */  nanoBooter (rx) : ORIGIN = ORIGIN(FLASH),      LENGTH = _nanoBooterImageSize
/*  Sector 1 : 128K                     */  nanoCLR    (rx) : ORIGIN = ORIGIN(nanoBooter) + LENGTH(nanoBooter), LENGTH = _nanoCLRImageSize
/*  Sector 2 : 128K                     */  
/*  Sector 3 : 128K                     */  
/*  Sector 4 : 128K                     */  
/*                                          Config parameters are stored in a 2K user option byte area that is accessed by registers
/*                                      */  config     (rw) : ORIGIN = 0, LENGTH = 0  /* NOT USED for consistency with other platforms only */
/*                                      */
/*  Sector 5 : 128K                     */  deployment (rx) : ORIGIN = 0x08000000 + (3*128K), LENGTH = (3*128K)
/*  Sector 6 : 128K                     */ 
/*  Sector 7 : 128K                     */ 
/*                                      */
/*                                      */
/*  Bootloader or Secure bootloader and */ 
/*  secure library if option secure     */ 
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* 0x1FF00000 : 128K System memory      */  /* 128 Kbytes contain the factory bootloader firmware */
/*                                      */ 
/*                                      */
/* DTCM-RAM can be used as read-write   */
/* segment to host critical real-time   */
/* data (such as stack and heap)        */
/*--------------------------------------*/
/* 0x20000000 : 128K DTCM SRAM          */  /* To be investigated */
/*                                      */ 
/*--------------------------------------*/
/* Power Domain 1                       */  /* Use both RAM banks contiguously */
/* 0x24000000 : 192K AXI SRAM           */  RAM_D1   (xrw) : ORIGIN = 0x24000000,   LENGTH = 320K
/* 0x24030000 : 128K AXI SRAM           */  
/*                                      */  
/*--------------------------------------*/
/* Power Domain 2                       */  /* Use both RAM banks contiguously, accessible by DMA 1 and DMA 2 controllers */
/* 0x30000000 : 16K AHB SRAM1           */  RAM_D2   (xrw) : ORIGIN = 0x30000000,     LENGTH = 32K
/* 0x30040000 : 16K AHB SRAM2           */  RAM_DMA  (xrw) : ORIGIN = ORIGIN(RAM_D2), LENGTH = LENGTH(RAM_D2)
/*                                      */  
/*--------------------------------------*/
/* Power Domain 3                       */
/* Retains power when D1 and D2 domain  */
/* enter DStandby mode                  */
/* 0x38000000 : 16K SRAM4               */  RAM_D4     (xrw) : ORIGIN = 0x38000000,     LENGTH = 16K
/*                                      */  bootclpbrd (xrw) : ORIGIN = ORIGIN(RAM_D4), LENGTH = 48
/*                                      */  
/*--------------------------------------*/
/* With a battery connected to the      */
/* VBAT pin,the backup SRAM can be used */
/* to retain data during low-power mode */
/*--------------------------------------*/
/* 0x38800000 : 4K Backup SRAM          */  /* Possible RAM backup */
/*                                      */ 
/*--------------------------------------*/
/* 0x70000000 : 16M OCTOSPI2 (Hyperram) */  OCTOSPI2_RAM (xrw) : ORIGIN = 0x70000000,                                  LENGTH = 16M
/*                                      */  graphics_vfb (xrw) : ORIGIN = 0x70000000,                                  LENGTH = 255k     /* 272*400*2 - pixels */
/*                                      */  graphics_ram (xrw) : ORIGIN = ORIGIN(graphics_vfb) + LENGTH(graphics_vfb), LENGTH = 8M       /* Working bitmaps etc */
/*                                      */  external_ram (xrw) : ORIGIN = ORIGIN(graphics_ram) + LENGTH(graphics_ram), LENGTH = LENGTH(OCTOSPI2_RAM) - LENGTH(graphics_vfb) - LENGTH(graphics_ram)
/*                                      */
/*                                      */ 
/*                                      */  
/*                                      */  
/*                                      */  
/*--------------------------------------*/
/* 0x09000000 : 64M OCTOSPI1 (Flash)    */  /* Uses for OCTOSPI1 Flash? */
/*                                      */  /* Local drive? */
/*                                      */  /* Load static resources? */
/*                                      */
/*                                      */ 
/*                                      */ 
/*--------------------------------------*/
}



REGION_ALIAS("SECTION_FOR_BOOTCLIPBOARD", bootclpbrd);        /* boot clipboard */

/* External Ram */
REGION_ALIAS("GRAPHICS_HEAP_RAM", graphics_ram);              /* nanoFramework graphics heap */
REGION_ALIAS("GRAPHICS_VIDEO_FRAME_BUFFER", graphics_vfb);    /* nanoFramework graphics video frame buffer */
REGION_ALIAS("CLR_MANAGED_HEAP_RAM", external_ram);           /* nanoFramework CLR managed heap */

/* Regions accessible by DMA1 and DMA 2 controllers */
REGION_ALIAS("DMA_ACCESSIBLE", RAM_D2);




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


  /* Not using "DYNAMIC_MEMORY" so no need for a ._threadx_heap here.

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
        __RAM_segment_used_end__ = .;
  } >RAM_D1

  .dma_buffer : /* DMA1 and DMA2 memory accessible and in same domain, highest performance */
{
  *(.dma_buffer)
} >RAM_DMA

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }


}


__nanoImage_start__     = ORIGIN(FLASH);
__nanoImage_size__      = LENGTH(FLASH);
__nanoImage_end__       = __nanoImage_start__ + __nanoImage_size__;

__nanoConfig_start__    = ORIGIN(config);
__nanoConfig_size__     = LENGTH(config);
__nanoConfig_end__      = __nanoConfig_start__ + __nanoConfig_size__;

__deployment_start__    = ORIGIN(deployment);
__deployment_size__     = LENGTH(deployment);
__deployment_end__      = __deployment_start__ + __deployment_size__;


__nanoCLR__ = 0x08000000 + 0x4000;



/* Include additional rules*/
INCLUDE Local\rules_bootclipboard.ld
