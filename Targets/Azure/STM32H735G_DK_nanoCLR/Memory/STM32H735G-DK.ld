/*
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
*/

/* Note:                                  */
/* SYS>>   Bare metal code or RTOS        */
/* CLR>>   Use by Common Language Runtime */

/*       /------------------------------------\     */
/*       |   Addresses required by            |     */   
/*       |   startup_stm32h735igkx.s          |     */   
/*       | ================================== |     */
/*       |  Entry Point                       |SYS>>*/  ENTRY(Reset_Handler)
/*       |------------------------------------|     */
/*       |  Address of the .data section      |     */  
/*       |  Copy from flash to RAM            |     */  
/*       |  at startup                        |     */   
/*       | =================================  |     */
/*       |                                    |SYS>>*/  _sidata = LOADADDR(.data);
/*       |  Start                             |SYS>>*/  /*_sdata , defined in .data section below */
/*       |  End                               |SYS>>*/  /*_edata , defined in .data section below */
/*       |------------------------------------|     */
/*       |  Address of the .bss section       |     */  
/*       |  Zero fill at startup              |     */   
/*       | ================================== |     */
/*       |  Start                             |SYS>>*/  /*_sbss  , defined in .bss section below */
/*       |  End                               |SYS>>*/  /*_ebss  , defined in .bss section below */
/*       |------------------------------------|     */
/*       |  Stack pointer address             |SYS>>*/  _estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);   
/*       |  Set to top of RAM                 |     */   
/*       |  Stack works down                  |     */   
/*       \------------------------------------/     */

/*       /------------------------------------\     */
/*       |  PARAMETERS                        |     */   
/*       | ================================== |     */
/*       | See section ._user_heap_stack      |     */
/*       | Generate a link error if the heap  |     */
/*       | and stack don't fit into RAM       |     */   
/*       | Required heap size                 |SYS>>*/  _Min_Heap_Size = 0x200; 
/*       | Required stack size                |SYS>>*/  _Min_Stack_Size = 0x400; 
/*       |------------------------------------|     */   
/*       | The flash sector size is 128K      |CLR>>*/  flash_sector_size  = 0x20000 ; 
/*       | Each flash word is 256 bits,       |CLR>>*/  flash_word_size    = 32;
/*       | (32 bytes). Smaller writes than    |     */
/*       | this requires force write with     |     */
/*       | the remaining bytes set to 1's     |     */
/*       |------------------------------------|     */
/*       | Configuration are to store         |     */   
/*       | persistent information             |     */   
/*       |                                    |CLR>>*/  config_size =  20K;
/*       | Keep in sync with                  |     */   
/*       | "targetHAL_ConfigurationManger.cpp"|     */   
/*       \------------------------------------/     */


MEMORY    /*   Usage by nanoFramework */
{
/*       /------------------------------------\     */
/*       | STM32H735G-DK Physical memory      |     */
/*       | =================================  |     */
/*       |                                    |     */   
/*       | 0x00000000 : 64K ITCM SRAM         |...>>*/  /* FUTURE USE*/  
/*       | ITCM-RAM  can be used to host      |     */
/*       | critical real-time code such as    |     */
/*       | interrupt handlers                 |     */
/*       |                                    |     */   
/*       |------------------------------------|     */
/*       |                                    |     */   
/*       |------------------------------------|     */
/*       | 0x08FFF000 : 1K Flash User OTP     |...>>*/  /* FUTURE USE*/
/*       | Possible one time programmable use |     */
/*       |------------------------------------|     */
/*       |                                    |     */   
/*       |------------------------------------|     */
/*       | 0x08FFFC00 : 512 Flash STM Read    |     */  /* Factory System Memory */
/*       |      System Not useable            |     */ 
/*       |------------------------------------|     */
/*       |                                    |     */   
/*       |------------------------------------|     */
/*       | 0x08000000 : 1024K AXI Flash       |SYS>>*/  FLASH      (rx) : ORIGIN = 0x08000000,   LENGTH = 1024k
/*       | Single Flash Bank                  |     */   
/*       | Sector size 128K                   |     */   
/*       | Smallest erase 1 Sector, so align  |     */ 
/*       | deployment on sector boundary      |     */ 
/*       | 8 x Sectors, each 128K             |     */ 
/*       |....................................|     */
/*       | Sector 0 : 128K                    |CLR>>*/  nanoCLR    (rx) : ORIGIN = ORIGIN(FLASH), LENGTH = (5*flash_sector_size)
/*       | Sector 1 : 128K                    |     */
/*       | Sector 2 : 128K                    |     */
/*       | Sector 3 : 128K                    |     */
/*       | Sector 4 : 128K                    |     */
/*       |....................................|     */
/*       | Sector 5 : 128K                    |CLR>>*/  deployment (rx) : ORIGIN = ORIGIN(nanoCLR) + LENGTH(nanoCLR), LENGTH = 3*flash_sector_size
/*       | Sector 6 : 128K                    |     */
/*       | Sector 7 : 128K                    |     */
/*       |                                    |     */
/*       | 0x080FFFFF : FLASH END             |     */
/*       |------------------------------------|     */
/*       |                                    |     */ 
/*       |------------------------------------|     */
/*       | 0x1FF00000 : 128K System memory    |SYS>>*/  /* SYSTEM Not available for user use*/ 
/*       | Bootloader or Secure bootloader and|     */ 
/*       | secure library if option secure    |     */ 
/*       |------------------------------------|     */
/*       |                                    |     */ 
/*       |------------------------------------|     */
/*       | 0x20000000 : 128K DTCM SRAM        |CLR>>*/  /* FUTURE USE */  
/*       | DTCM-RAM can be used as read-write |     */
/*       | segment to host critical real-time |     */
/*       | data (such as stack and heap)      |     */
/*       |                                    |     */
/*       |------------------------------------|     */
/*       |                                    |     */ 
/*       |------------------------------------|     */
/*       | Power Domain 1                     |     */
/*       | 0x24000000 : 192K AXI SRAM         |SYS>>*/  RAM_D1     (xrw) : ORIGIN = 0x24000000,   LENGTH = 320K   
/*       | 0x24030000 : 128K AXI SRAM         |     */
/*       | Use both RAM banks contiguously    |     */ 
/*       |                                    |     */ 
/*       | Used by                            |     */ 
/*       |   .data                            |     */ 
/*       |   .bss                             |     */ 
/*       |   .user_heap                       |     */ 
/*       |------------------------------------|     */
/*       |                                    |     */ 
/*       |------------------------------------|     */
/*       | Power Domain 2                     |     */
/*       | 0x30000000 : 16K AHB SRAM1         |SYS>>*/  RAM_D2     (xrw) : ORIGIN = 0x30000000,   LENGTH = 32K  
/*       | 0x30040000 : 16K AHB SRAM2         |     */
/*       | Use both RAM banks contiguously    |     */ 
/*       |                                    |     */ 
/*       | Accessible by DMA 1 and            |CLR>>*/  RAM_DMA    (xrw) : ORIGIN = ORIGIN(RAM_D2), LENGTH = LENGTH(RAM_D2)
/*       | DMA 2 controllers                  |     */ 
/*       |------------------------------------|     */
/*       |                                    |     */ 
/*       |------------------------------------|     */
/*       | Power Domain 3                     |     */
/*       | 0x38000000 : 16K SRAM4             |SYS>>*/  RAM_D4     (xrw) : ORIGIN = 0x38000000,     LENGTH = 16K
/*       | Retains power when D1 and D2       |     */
/*       | domain enter DStandby mode         |     */
/*       |                                    |     */ 
/*       |                                    |CLR>>*/  HardFaultCount (xrw) : ORIGIN = ORIGIN(RAM_D4), LENGTH = 32
/*       |------------------------------------|     */
/*       |                                    |     */ 
/*       |------------------------------------|     */
/*       | 0x38800000 : 4K Backup SRAM        |CLR>>*/  /* FUTURE USE */
/*       | With a battery connected to the    |     */
/*       | VBAT pin,the backup SRAM can       |     */
/*       | be used to retain data during      |     */
/*       | low-power mode                     |     */
/*       |------------------------------------|     */
/*       |                                    |     */ 
/*       |------------------------------------|     */
/*       | 0x70000000 : 16M OCTOSPI2          |SYS>>*/  OCTOSPI2_RAM (xrw) : ORIGIN = 0x70000000, LENGTH = 16M
/*       | ON STM32735G-DK (Hyperram)         |     */
/*       |                                    |     */
/*       | Video frame buffer                 |CLR>>*/  graphics_vfb (xrw) : ORIGIN = ORIGIN(OCTOSPI2_RAM), LENGTH = 255k
/*       | Sized to the display on the        |     */
/*       | STM32H735G-DK board                |     */
/*       |                                    |     */
/*       | nanoCLR graphics bitmaps,          |CLR>>*/  graphics_ram (xrw) : ORIGIN = ORIGIN(graphics_vfb) + LENGTH(graphics_vfb), LENGTH = 8M
/*       | drawing surfaces                   |     */
/*       |                                    |     */
/*       | nanoCLR managed heap               |CLR>>*/  external_ram (xrw) : ORIGIN = ORIGIN(graphics_ram) + LENGTH(graphics_ram), LENGTH = LENGTH(OCTOSPI2_RAM) - LENGTH(graphics_vfb) - LENGTH(graphics_ram)
/*       |------------------------------------|     */
/*       |                                    |     */ 
/*       |------------------------------------|     */
/*       | 0x09000000 : 64M OCTOSPI1 (Flash)  |SYS>>*/  OCTOSPI1_FLASH (xrw) : ORIGIN = 0x09000000, LENGTH = 64M
/*       |                                    |CLR>>*/  config     (rx) : ORIGIN = ORIGIN(OCTOSPI1_FLASH), LENGTH = (config_size)
/*       |                                    |     */
/*       \------------------------------------/     */
}
        
/*       /------------------------------------\     */
/*       |  VARIABLES REFERENCED BY C/C++     |     */   
/*       | ================================== |     */
/*       | The CLR managed heap for CLR       |CLR>>*/  HeapBegin             = ORIGIN(external_ram); 
/*       | objects                            |CLR>>*/  HeapEnd               = ORIGIN(external_ram) + LENGTH(external_ram); 
/*       |------------------------------------|     */   
/*       | CLR Image                          |CLR>>*/  __nanoImage_start__   = ORIGIN(nanoCLR); 
/*       |                                    |CLR>>*/  __nanoImage_end__     = LENGTH(nanoCLR); 
/*       |------------------------------------|     */   
/*       | Flash region where the C#          |CLR>>*/  __deployment_start__  = ORIGIN(deployment); 
/*       | Portable Executable (PE) file      |CLR>>*/  __deployment_size__   = LENGTH(deployment); 
/*       | is stored                          |CLR>>*/  __deployment_end__    = __deployment_start__ + __deployment_size__; 
/*       |------------------------------------|     */   
/*       | Small area of persistent storage   |     */  __nanoConfig_start__  = ORIGIN(config);
/*       |                                    |     */  __nanoConfig_end__    = ORIGIN(config) + config_size;
/*       |------------------------------------|     */   
/*       | Bit pixel copy of the screen       |CLR>>*/  Graphics_frame_buffer = ORIGIN(graphics_vfb); 
/*       | Updated by DMA                     |     */    
/*       |------------------------------------|     */   
/*       | Bitmap objects created by C#       |CLR>>*/  Graphics_managed_heap = ORIGIN(graphics_ram); 
/*       | and drawn into                     |CLR>>*/  Graphics_managed_end  = ORIGIN(graphics_ram) + LENGTH(graphics_ram);   
/*       |                                    |     */  
/*       | Used during debugging to stop      |     */  Hard_Fault_Count      = ORIGIN(HardFaultCount);
/*       | reboot loop if faulting            |     */  
/*       \------------------------------------/     */



/* Define output sections */
SECTIONS
{
  /*----------------------------------------*/
  /* The startup code goes first into FLASH */
  /*----------------------------------------*/
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /*-----------------------------------------------------*/
  /* The program code and other data goes into FLASH     */
  /* executable instructions, of a program               */
  /* glue arm to thumb code                              */
  /* glue thumb to arm code                              */
  /* the system arranges to execute the code in this     */
  /* section before calling the main program entry point */
  /*-----------------------------------------------------*/
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbol at end of code */
  } >FLASH

  /*-----------------------------------------*/
  /* Read only data, constants, strings etc. */
  /*-----------------------------------------*/
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* --------------------------------------------------------------------------------------*/
  /* ARM special sections                                                                  */
  /* Each function that can throw an exception will have entries in the following sections */
  /* used for C++ and for C programs that try to examine backtraces                        */
  /* --------------------------------------------------------------------------------------*/
  .ARM.extab :                 /* extab names sections containing exception unwinding information */
  {
   *(.ARM.extab* .gnu.linkonce.armextab.*) 
   } >FLASH
  
  .ARM : 
  {
    __exidx_start = .;         /*exidx is used to contain index entries for stack unwinding */
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH


  /* ----------------------------------*/
  /*   Initialization of C/C++ objects
  /* ----------------------------------*/
  .preinit_array :
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
  
  /* ----------------------------------------------------------------------*/
  /* Tear down of C/C++ objects                                            */
  /* C++ runtime destructors for static variables, C designated finalizers */
  /* ----------------------------------------------------------------------*/
  .fini_array : /*  */
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH
  

  /* -----------------------------------------------------------------------*/
  /* Holds initialized data that contributes to the program's memory image  */
  /* -----------------------------------------------------------------------*/
  .data :   
  {
    . = ALIGN(4);
    _sdata = .;        /* Start address for the .data section */
                       /* Used in LoopCopyDataInit:, "startup_st,32h735igks.s"
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */
    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM_D1 AT> FLASH
  
  /* -------------------------------------------------------------------*/
  /* This section holds uninitialized data                             */
  /* By definition, and through the code in startup_st32h735igks.s, the */
  /* system initializes the data with zeros at startup                  */
  /* -------------------------------------------------------------------*/
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

/* -----------------------------------------------------------------*/
/* Not using "DYNAMIC_MEMORY" so no need for a ._threadx_heap here.
/* -----------------------------------------------------------------*/

/* ------------------------------------------------------------------*/
/* Application defined for use as memory accessible by DMA1 and DMA2 */
/* ------------------------------------------------------------------*/
  .dma_buffer :
{
  *(.dma_buffer)
} >RAM_DMA

/* ---------------------------------------------------------------------*/
/* User_heap_stack section, used to check that there is enough RAM left */
/* ---------------------------------------------------------------------*/
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


  /* -----------------------------------------------*/
  /* Remove information from the standard libraries */
  /* -----------------------------------------------*/
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}

