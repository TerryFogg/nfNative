/*
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
*/

total_flash     = 2048K;
nanoBooter_size = 60K;
nanoCLR_size    = 512K;
config_size     = 2K;
__main_stack_size__     = 0x400;
__process_stack_size__  = 0x800;
__crt_heap_size__       = 0x3B000;
_Min_Stack_Size = 0x1000;	/* required amount of stack */
_Min_Heap_Size = 0x1000;	/* required amount of heap  */


sdram_size = 16M;

/* Memories definition */
MEMORY
{
      /*--------------------------------------------------------*/
      /* System flash definitions for reference */
      /*--------------------------------------------------------*/
      flash_otp       (xrw) : ORIGIN = 0x08FFF000, LENGTH = 1K    
      flash_stm       (xrw) : ORIGIN = 0x08FFF800, LENGTH = 512   
      flash_system_1  (xrw) : ORIGIN = 0x1FF000000,LENGTH = 64K   
      flash_system_2  (xrw) : ORIGIN = 0x1FF10000, LENGTH = 44K 
      /*--------------------------------------------------------*/
      
      FLASH_1 (xrw) : ORIGIN = 0x08000000, LENGTH = 2048K
          flash_bank_1    (xrw) : ORIGIN = 0x08000000, LENGTH = 1024K          /* 127 Sectors of 8k */
          flash_bank_2    (xrw) : ORIGIN = 0x8FFF0000, LENGTH = 1024K          /* 127 Sectors of 8k */

      FLASH_EXTERNAL_1 (xrw) : ORIGIN = 0x09000000, LENGTH = 64M               /* STM32H7B3I-DK on board external to SOC */
          octospi1_flash  (xrw) : ORIGIN = 0x90000000, LENGTH = 64M            /* OSPI flash */

      RAM_INSTRUCTION_CACHE        (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
          itcm_ram        (xrw) : ORIGIN = 0x00000000, LENGTH = 64K

      RAM_DATA_CACHE (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
          dtcm_ram        (xrw) : ORIGIN = 0x20000000, LENGTH = 128K  

      RAM_DOMAIN_1 (xrw) : ORIGIN = 0x24000000, LENGTH = 1024K
          axi_sram1       (xrw) : ORIGIN = 0x24000000, LENGTH = 256K  
          axi_sram2       (xrw) : ORIGIN = 0x24040000, LENGTH = 384K  
          axi_sram3       (xrw) : ORIGIN = 0x240A0000, LENGTH = 384K  

      RAM_DOMAIN_2 (xrw) : ORIGIN = 0x30000000, LENGTH = 128K
          ahb_sram1       (xrw) : ORIGIN = 0x30000000, LENGTH = 64K   
          ahb_sram2       (xrw) : ORIGIN = 0x30010000, LENGTH = 64K   

      RAM_DOMAIN_SRD (xrw) : ORIGIN = 0x38000000, LENGTH = 32k
          srd_ram         (xrw) : ORIGIN = 0x38000000, LENGTH = 32k   
          backup_ram      (xrw) : ORIGIN = 0x38800000, LENGTH = 4k    
      
      RAM_EXTERNAL_1 (xrw) : ORIGIN = 0xD0000000, LENGTH = 16M          /* STM32H7B3I-DK on board external to SOC */
          sdram     (xrw) : ORIGIN = 0xD0000000, LENGTH = 16M   

      
      /* ======================== */
      /* Application memory usage */
      /* ======================== */

      /* FLASH definitions */
      nanoBooter   (rx)  : ORIGIN = ORIGIN(FLASH_1), LENGTH = nanoBooter_size
      config       (rw)  : ORIGIN = ORIGIN(nanoBooter) + LENGTH(nanoBooter), LENGTH = config_size
      nanoCLR      (rx)  : ORIGIN = ORIGIN(config) + LENGTH(config), LENGTH = nanoCLR_size
      FLASH       (xrw)  : ORIGIN = ORIGIN(nanoCLR) + LENGTH(nanoCLR), LENGTH = LENGTH(FLASH_1) - (LENGTH(nanoBooter) + LENGTH(config) + LENGTH(nanoCLR) )

      ramvt        (wx) : org = 0x00000000, len = 0                         /* initial RAM address is reserved for a copy of the vector table */

      /* RAM definitions */
      bootclpbrd   (wx)  : ORIGIN = ORIGIN(RAM_DOMAIN_2), LENGTH = 48
  
      graphics_vfb (xrw): ORIGIN = ORIGIN(RAM_EXTERNAL_1), len = 255k                /* One screen size frame buffer for video output mode  (480x272x2) bytes ( 16 bits RGB565 )*/
      graphics_ram (xrw): ORIGIN = ORIGIN(graphics_vfb) + LENGTH(graphics_vfb), LENGTH = 8M       /* For storing expanding images,jpeg etc */
      external_ram (xrw): ORIGIN = ORIGIN(graphics_ram) + LENGTH(graphics_ram), LENGTH = RAM_EXTERNAL_1 - (LENGTH(graphics_vfb) + LENGTH(graphics_ram) )


      /* Overlays : Contiguous block names */                     
      flash_bank_1_2  (xrw) : ORIGIN = 0x08000000, LENGTH = 2048K 
      axi_ram_1_2_3   (xrw) : ORIGIN = 0x24000000, LENGTH = 1024K 
      ahb_ram_1_2     (xrw) : ORIGIN = 0x30000000, LENGTH = 128K 
}



/* Specify the memory areas */
/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_DOMAIN_1) + LENGTH(RAM_DOMAIN_1);	/* end of "RAM_DOMAIN_1" Ram type memory */


/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", FLASH);
REGION_ALIAS("VECTORS_FLASH_LMA", FLASH);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", FLASH);
REGION_ALIAS("XTORS_FLASH_LMA", FLASH);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", FLASH);
REGION_ALIAS("TEXT_FLASH_LMA", FLASH);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", FLASH);
REGION_ALIAS("RODATA_FLASH_LMA", FLASH);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", FLASH);
REGION_ALIAS("VARIOUS_FLASH_LMA", FLASH);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", FLASH);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", RAM_DATA_CACHE);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", RAM_DATA_CACHE);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", RAM_DOMAIN_1);
REGION_ALIAS("DATA_RAM_LMA", RAM_DOMAIN_1);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", RAM_DATA_CACHE);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", external_ram);

/* RAM region to be used for the nanoFramework CLR managed heap.*/
REGION_ALIAS("CLR_MANAGED_HEAP_RAM", external_ram);

/* RAM region to be used for the nanoFramework graphics heap.*/
REGION_ALIAS("GRAPHICS_HEAP_RAM", graphics_ram);

/* RAM region to be used for the nanoFramework graphics heap.*/
REGION_ALIAS("GRAPHICS_VIDEO_FRAME_BUFFER", graphics_vfb);

/* RAM region to be used for the boot clipboard.*/
REGION_ALIAS("SECTION_FOR_BOOTCLIPBOARD", bootclpbrd);


/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */
        . = ALIGN(4);
  } >FLASH_1

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
  } >FLASH_1

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
        . = ALIGN(4);
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
  } >FLASH_1

  .ARM.extab   : 
  {
      . = ALIGN(4);
      *(.ARM.extab* .gnu.linkonce.armextab.*)
      . = ALIGN(4);
  } >FLASH_1

  .ARM : 
  {
	    . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
	    . = ALIGN(4);
  } >FLASH_1

  .preinit_array     :
  {
	    . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
	    . = ALIGN(4);
  } >FLASH_1
  
  .init_array :
  {
	    . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
	    . = ALIGN(4);
  } >FLASH_1

  .fini_array :
  {
	    . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
	    . = ALIGN(4);
  } >FLASH_1

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM_DOMAIN_1" Ram type memory */
  .data : 
  {
        . = ALIGN(4);
        _sdata = .;        /* create a global symbol at data start */
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */

        . = ALIGN(4);
        _edata = .;        /* define a global symbol at data end */
  } >RAM_DOMAIN_1 AT> flash_bank_1_2

  
  /* Uninitialized data section into "RAM_DOMAIN_1" Ram type memory */
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
  } >RAM_DOMAIN_1

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
  


  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/*--------------------------------------------*/
/* Code rules inclusion                       */
/*--------------------------------------------*/
INCLUDE Platform\Memory\rules_bootclipboard.ld
INCLUDE Platform\Memory\rules_clr.ld
INCLUDE Platform\Memory\rules_code.ld
INCLUDE Platform\Memory\rules_data.ld
INCLUDE Platform\Memory\rules_graphics.ld
INCLUDE Platform\Memory\rules_stack.ld




/* Set the RAM segment used end for threadx */
__RAM_segment_used_end__ = 0; 