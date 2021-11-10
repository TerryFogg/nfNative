/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", flash_bank_2);
REGION_ALIAS("VECTORS_FLASH_LMA", flash_bank_1);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", flash_bank_2);
REGION_ALIAS("XTORS_FLASH_LMA", flash_bank_1);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", flash_bank_2);
REGION_ALIAS("TEXT_FLASH_LMA", flash_bank_1);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", flash_bank_1);
REGION_ALIAS("RODATA_FLASH_LMA", flash_bank_1);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", flash_bank_2);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash_bank_1);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash_bank_1);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", dtcm_ram);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", dtcm_ram);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash_bank_1);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", dtcm_ram);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

/* RAM region to be used for the nanoFramework CLR managed heap.*/
REGION_ALIAS("CLR_MANAGED_HEAP_RAM", sdram_bank1);

/* RAM region to be used for the nanoFramework graphics heap.*/
REGION_ALIAS("GRAPHICS_HEAP_RAM", graphics_ram);

/* RAM region to be used for the nanoFramework graphics heap.*/
REGION_ALIAS("GRAPHICS_VIDEO_FRAME_BUFFER", graphics_vfb);

/* RAM region to be used for the boot clipboard.*/
REGION_ALIAS("SECTION_FOR_BOOTCLIPBOARD", bootclpbrd);

/* Memory that can be accessed by DMA 1 and 2 */
REGION_ALIAS("DMA1_RAM", ahb_sram1);
REGION_ALIAS("DMA2_RAM", ahb_sram2);
