/*
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
*/


/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200 ;      /* required amount of heap  */
_Min_Stack_Size = 0x400 ; /* required amount of stack */

__main_stack_size__     = 0x400;
__process_stack_size__  = 0x800;
__crt_heap_size__       = 0x800;

/* Specify the memory areas */
MEMORY
{
/*---------------------------------------------------------------------------------------------------------------*/
 /* STM32H735IG Physical memory */
 /*---------------------------------------------------------------------------------------------------------------*/
 flash_system    (r) : ORIGIN = 0x1FF000000,LENGTH = 128K            /* System flash definitions for reference */

 FLASH_1 (xrw) : ORIGIN = 0x08000000, LENGTH = 1024K
     flash_bank_1    (xrw) : ORIGIN = 0x08000000, LENGTH = 1024K     /* 8 Sectors of 128k */

 FLASH_EXTERNAL_1 (xrw) : ORIGIN = 0x09000000, LENGTH = 64M          /* STM32H735G-DK on board external to SOC */
     octospi1_flash    (xrw) : ORIGIN = 0x09000000, LENGTH = 64M     /* OSPI flash */

 RAM_INSTRUCTION_CACHE        (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
     itcm_ram        (xrw) : ORIGIN = 0x00000000, LENGTH = 64K

 RAM_DATA_CACHE (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
     dtcm_ram        (xrw) : ORIGIN = 0x20000000, LENGTH = 128K  
 
 RAM_DOMAIN_1 (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
     axi_sram1       (xrw) : ORIGIN = 0x24000000, LENGTH = 192K      /* NOTE: This RAM can be re-mapped as extra ITCM RAM */
     axi_sram2       (xrw) : ORIGIN = 0x24030000, LENGTH = 128K

 RAM_DOMAIN_2 (xrw) : ORIGIN = 0x24000000, LENGTH = 32K
     ahb_sram1       (xrw) : ORIGIN = 0x30000000, LENGTH = 16K
     ahb_sram2       (xrw) : ORIGIN = 0x30040000, LENGTH = 16K

 RAM_DOMAIN_3 (xrw) : ORIGIN = 0x38000000, LENGTH = 16K
     ahb_sram4       (xrw) : ORIGIN = 0x38000000, LENGTH = 16K

     backup_ram      (xrw) : ORIGIN = 0x30010000, LENGTH = 4k    
 
 RAM_EXTERNAL_1 (xrw) : ORIGIN = 0x70000000, LENGTH = 16M            /* STM32H735G-DK on board external to SOC */
     octospi2_RAM     (xrw) : ORIGIN = 0x70000000, LENGTH = 16M             /* Hyperam */
/*----------------------------------------------------------------------------------------------------------------*/


  ITCMRAM (xrw)    : ORIGIN = 0x00000000,   LENGTH = 64K
      ramvt      (wx) : org = 0x00000000, len = 0                     /* initial RAM address is reserved for a copy of the vector table */
        config     (rw) : org = 0x00000000, len = 0                     /* space reserved for configuration block */

  DTCMRAM (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH    (rx)    : ORIGIN = 0x08000000,   LENGTH = 1024k
  
  RAM_D1  (xrw)    : ORIGIN = 0x24000000,   LENGTH = 192K
  RAM_D2  (xrw)    : ORIGIN = 0x24030000,   LENGTH = 128K

  RAM_D3  (xrw)    : ORIGIN = 0x30000000,   LENGTH = 16K
  RAM_D4  (xrw)    : ORIGIN = 0x30040000,   LENGTH = 16K


  graphics_vfb (xrw): ORIGIN = ORIGIN(RAM_EXTERNAL_1), len = 255k                /* One screen size frame buffer for video output mode  (480x272x2) bytes ( 16 bits RGB565 )*/
  graphics_ram (xrw): ORIGIN = ORIGIN(graphics_vfb) + LENGTH(graphics_vfb), LENGTH = 8M       /* For storing expanding images,jpeg etc */
  external_ram (xrw): ORIGIN = ORIGIN(graphics_ram) + LENGTH(graphics_ram), LENGTH = LENGTH(RAM_EXTERNAL_1) - (LENGTH(graphics_vfb) + LENGTH(graphics_ram) )

  bootclpbrd   (wx)  : ORIGIN = ORIGIN(RAM_D3), LENGTH = 48

}

REGION_ALIAS("SECTION_FOR_BOOTCLIPBOARD", bootclpbrd);        /* boot clipboard */

/* External Ram */
REGION_ALIAS("GRAPHICS_HEAP_RAM", graphics_ram);              /* nanoFramework graphics heap */
REGION_ALIAS("GRAPHICS_VIDEO_FRAME_BUFFER", graphics_vfb);    /* nanoFramework graphics video frame buffer */
REGION_ALIAS("CLR_MANAGED_HEAP_RAM", external_ram);           /* nanoFramework CLR managed heap */


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




  INCLUDE Platform\Memory\rules_clr.ld
  INCLUDE Platform\Memory\rules_graphics.ld
