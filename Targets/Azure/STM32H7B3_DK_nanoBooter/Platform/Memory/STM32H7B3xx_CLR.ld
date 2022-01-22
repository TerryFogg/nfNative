/*
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) STMicroelectronics. All rights reserved.
// See LICENSE file in the project root for full license information.
*/


/* Set the RAM segment used end for threadx */
/* __RAM_segment_used_end__ = 0;  */

total_flash     = 2048K;
nanoBooter_size = 60K;
nanoCLR_size    = 512K;
config_size     = 2K;

/* Specify the memory areas */


/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);	/* end of "RAM_D1" Ram type memory */

_Min_Heap_Size = 0x1000;	/* required amount of heap  */
_Min_Stack_Size = 0x1000;	/* required amount of stack */

/* Memories definition */
MEMORY
{
/*--------------------------------------------------------------------|*/
/* Physical memory location and size for reference                    |*/
                                                                    /*|*/
  /* NOT accessible by DMA1 or DMA2 */                              /*|*/
  itcm_ram        (xrw) : ORIGIN = 0x00000000, LENGTH = 64K         /*|*/
  flash_otp       (xrw) : ORIGIN = 0x08FFF000, LENGTH = 1K          /*|*/
  flash_stm       (xrw) : ORIGIN = 0x08FFF800, LENGTH = 512         /*|*/
  flash_bank_1    (xrw) : ORIGIN = 0x08000000, LENGTH = 1024K       /*|*/
  flash_bank_2    (xrw) : ORIGIN = 0x8FFF0000, LENGTH = 1024K       /*|*/
  flash_system_1  (xrw) : ORIGIN = 0x1FF000000,LENGTH = 64K         /*|*/
  flash_system_2  (xrw) : ORIGIN = 0x1FF10000, LENGTH = 44K         /*|*/
  dtcm_ram        (xrw) : ORIGIN = 0x20000000, LENGTH = 128K        /*|*/
                                                                    /*|*/
  /* accessible by DMA1 and DMA2 */                                 /*|*/
  axi_sram1       (xrw) : ORIGIN = 0x24000000, LENGTH = 256K        /*|*/
  axi_sram2       (xrw) : ORIGIN = 0x24040000, LENGTH = 384K        /*|*/
  axi_sram3       (xrw) : ORIGIN = 0x240A0000, LENGTH = 384K        /*|*/
  ahb_sram1       (xrw) : ORIGIN = 0x30000000, LENGTH = 64K         /*|*/
  ahb_sram2       (xrw) : ORIGIN = 0x30010000, LENGTH = 64K         /*|*/
  srd_ram         (xrw) : ORIGIN = 0x30000000, LENGTH = 32k         /*|*/
  backup_ram      (xrw) : ORIGIN = 0x30010000, LENGTH = 4k          /*|*/
                                                                    /*|*/
  /*STM32H7B3I-DK on board external to SOC */                       /*|*/
  octospi1_flash  (xrw) : ORIGIN = 0x90000000, LENGTH = 64M         /*|*/
  sdram_bank1     (xrw) : ORIGIN = 0xC0000000, LENGTH = 32M         /*|*/
                                                                    /*|*/
  /* Overlays : Contiguous block names */                           /*|*/
  flash_bank_1_2  (xrw) : ORIGIN = 0x08000000, LENGTH = 2048K       /*|*/
  axi_ram_1_2_3   (xrw) : ORIGIN = 0x24000000, LENGTH = 1024K       /*|*/
  ahb_ram_1_2     (xrw) : ORIGIN = 0x30000000, LENGTH = 128K        /*|*/
                                                                    /*|*/
/*   NOTE: flash_system_1,flash_system_2,flash_stm are not usable     |*/
/*         for applications                                           |*/
/*--------------------------------------------------------------------|*/

  /* ======================== */
  /* Application memory usage */
  /* ======================== */
  /* flash */
  nanoBooter   (rx)  : ORIGIN = 0x08000000, LENGTH = nanoBooter_size
  config       (rw)  : ORIGIN = ORIGIN(nanoBooter) + LENGTH(nanoBooter), LENGTH = config_size
  nanoCLR      (rx)  : ORIGIN = ORIGIN(config) + LENGTH(config), LENGTH = nanoCLR_size
  deployment   (rx)  : ORIGIN = ORIGIN(nanoCLR) + LENGTH(nanoCLR), LENGTH = total_flash - LENGTH(nanoBooter) - LENGTH(config) - LENGTH(nanoCLR)

  /* ram */
  ram2         (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
  bootclpbrd   (wx)  : ORIGIN = 0x20000000, LENGTH = 48
  ram0         (xrw) : ORIGIN = 0x20000030, LENGTH = 96K-48

  graphics_ram : org = 0x2407F800, len = 255k                /* One screen size (480x272x2) of working ram*/
  graphics_vfb : org = 0xC0F44800, len = 255k                /* One screen size frame buffer for video output mode  (480x272x2) bytes ( 16 bits RGB565 )*/

/*--------------------------------------------------------------------|*/
/* Memory usage as defined in example from x-cube-azrtos-h7 */

  DTCMRAM         (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  ITCMRAM         (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
  RAM_D1          (xrw) : ORIGIN = 0x24000000, LENGTH = 1024K
  RAM_D2          (xrw) : ORIGIN = 0x30000000, LENGTH = 128K
  RAM_D3          (xrw) : ORIGIN = 0x38000000, LENGTH = 32K

  FLASH            (rx) : ORIGIN = 0x8000000,  LENGTH = 70K  /* enough for nanoBooter */
/*--------------------------------------------------------------------|*/

}

/* RAM region to be used for the boot clipboard.*/
REGION_ALIAS("SECTION_FOR_BOOTCLIPBOARD", bootclpbrd);
REGION_ALIAS("DMA1_RAM", ahb_sram1);
REGION_ALIAS("DMA2_RAM", ahb_sram2);

/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data into "FLASH" Rom type memory */
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

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : {
  . = ALIGN(4);
  *(.ARM.extab* .gnu.linkonce.armextab.*)
  . = ALIGN(4);
  } >FLASH

  .ARM : {
	. = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
	. = ALIGN(4);
  } >FLASH

  .preinit_array     :
  {
	. = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
	. = ALIGN(4);
  } >FLASH
  
  .init_array :
  {
	. = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
	. = ALIGN(4);
  } >FLASH

  .fini_array :
  {
	. = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
	. = ALIGN(4);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM_D1" Ram type memory */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM_D1 AT> FLASH

  
  /* Uninitialized data section into "RAM_D1" Ram type memory */
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

  /* User_heap_stack section, used to check that there is enough "RAM_D1" Ram  type memory left */
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
  
     .tcp_sec (NOLOAD) : {
     
    . = ABSOLUTE(0x24018000);
    *(.RxDecripSection) 
    
    . = ABSOLUTE(0x24018060);
    *(.TxDecripSection)
    
  } >RAM_D1 AT> FLASH 

    .nx_section 0x24018200 (NOLOAD): {

    *(.NXDriverSection)

  } >RAM_D1

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* Include additional rules*/
INCLUDE Platform\Memory\rules_code.ld
INCLUDE Platform\Memory\rules_bootclipboard.ld
