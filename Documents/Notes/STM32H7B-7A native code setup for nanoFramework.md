







### Appendix - A : DMA transfer with accessible memory and cache consistency in STM32H7 (Memory-to-Peripheral)
-------------------------------------------------------------------------------------------

Points to note when writing DMA enabled peripherals on the STM32H7.

-   The stm32H7 does not have access to the DTCM area from the DMA controller

-   The Cortex-M7 cores have an L1 cache, so you need to be aware of cache
    inconsistencies when transferring DMA.

**Legend:** \- : Not accessible   x : Accessible

This table can be found in reference manual RM0455, (Table 2)


| Cortex-M7        | AXIM | AHBP | ITCM | DTCM | SDMMC1-AXI | MDMA-AXI | MDMA-AHBS | DMA2D | LTDC | GFX-MMU | DMA1-MEM | DMA1-PERIPH | DMA2-MEM | DMA2-PERIPH | SDMMC2-AHB | OTG_HS | BDMA1-AHB | BDMA2-AHB |
|------------------|------|------|------|------|------------|----------|-----------|-------|------|---------|----------|-------------|----------|-------------|------------|--------|-----------|-----------|
| Bus slave/type   |      |      |      |      |            |          |           |       |      |         |          |             |          |             |            |        |           |           |
| ITCM             | \-   | \-   | x    | \-   | \-         | \-       | x         | \-    | \-   | \-      | \-       | \-          | \-       | \-          | \-         | \-     | \-        | \-        |
| DTCM             | \-   | \-   | \-   | x    | \-         | \-       | x         | \-    | \-   | \-      | \-       | \-          | \-       | \-          | \-         | \-     | \-        | \-        |
| Flash memory     | x    | \-   | \-   | \-   | x          | x        | x         | \-    | x    | x       | x        | x           | x        | x           | x          | x      | x         | \-        | 
| AXI SRAM1        | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | x       | x        | **[X]**     | x        | **[X]**     | x          | x      | \-        | \-        |
| AXI SRAM2        | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | x       | x        | **[X]**     | x        | **[X]**     | x          | x      | \-        | \-        |
| AXI SRAM3        | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | x       | x        | **[X]**     | x        | **[X]**     | x          | x      | \-        | \-        |
| GFX -MMU         | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | \-      | \-       | \-          | \-       | \-          | \-         | \-     | \-        | \-        |
| OCTOSPI1         | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | x       | x        | x           | x        | x           | x          | x      | \-        | \-        |
| OCTOSPI2         | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | x       | x        | x           | x        | x           | x          | x      | \-        | \-        |
| OTFDEC1          | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | x       | x        | x           | x        | x           | x          | x      | \-        | \-        |
| OTFDEC2(4)       | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | x       | x        | x           | x        | x           | x          | x      | \-        | \-        |
| FMC              | x    | \-   | \-   | \-   | x          | x        | \-        | x     | x    | x       | x        | x           | x        | x           | x          | x      | \-        | \-        |
| AHB SRAM1        | x    | \-   | \-   | \-   | \-         | x        | \-        | x     | x    | x       | x        | **[X]**     | x        | **[X]**     | x          | x      | x         | \-        |
| AHB SRAM2        | x    | \-   | \-   | \-   | \-         | x        | \-        | x     | x    | x       | x        | **[X]**     | x        | **[X]**     | x          | x      | x         | \-        |
| AHB1 peripherals | \-   | x    | \-   | \-   | \-         | x        | \-        | x     | \-   | \-      | x        | x           | x        | x           | x          | \-     | x         | \-        |
| APB1 peripherals | \-   | x    | \-   | \-   | \-         | x        | \-        | x     | \-   | \-      | x        | x           | x        | x           | x          | \-     | x         | \-        |
| APB2 peripherals | \-   | x    | \-   | \-   | \-         | x        | \-        | x     | \-   | \-      | x        | x           | x        | x           | x          | \-     | x         | \-        |
| AHB2 peripherals | \-   | x    | \-   | \-   | \-         | \-       | \-        | \-    | \-   | \-      | x        | x           | x        | x           | x          | \-     | \-        | \-        |
| AHB3 peripherals | x    | \-   | \-   | \-   | \-         | x        | \-        | \-    | \-   | \-      | \-       | \-          | \-       | \-          | \-         | \-     | \-        | \-        |
| APB3 peripherals | x    | \-   | \-   | \-   | \-         | x        | \-        | \-    | \-   | \-      | \-       | \-          | \-       | \-          | \-         | \-     | \-        | \-        |
| AHB4 peripherals | x    | \-   | \-   | \-   | \-         | x        | \-        | \-    | \-   | \-      | x        | x           | x        | x           | x          | \-     | \-        | x         |
| APB4 peripherals | x    | \-   | \-   | \-   | \-         | x        | \-        | \-    | \-   | \-      | x        | x           | x        | x           | x          | \-     | \-        | x         |
| SRD SRAM         | x    | \-   | \-   | \-   | \-         | x        | \-        | \-    | \-   | \-      | x        | **[X]**      | x       | **[X]**     | x          | \-     | \-        | x         |
| Backup RAM       | x    | \-   | \-   | \-   | \-         | x        | \-        | \-    | \-   | \-      | x        | x           | x        | x           | x          | \-     | \-        | x         |



#### Defining buffers for use in nanoFramework native


Using the USART and DMA to support the serial wire protocol, any transmit and receive buffers must be accessible to the DMA controller.
From the table above,DMA1 and DMA2 memory to peripheral and peripheral to memory must be one of the accessible memory regions.

The memory section of the linker can be setup with a named section that is accessible to DMA1 and DMA2.
This example will use the 64K block starting at 0x30000000 named ahb_sram1

```
 /* Specify the memory areas */
MEMORY
{
/*-------------------------------------------------------------------*/
/* Physical memory location and size for reference                 |*/
                                                                 /*|*/
  /* NOT accessible by DMA1 or DMA2 */                           /*|*/
  itcm_ram        (xrw) : ORIGIN = 0x00000000, LENGTH = 64K      /*|*/
  flash_otp       (xrw) : ORIGIN = 0x08FFF000, LENGTH = 1K       /*|*/
  flash_stm       (xrw) : ORIGIN = 0x08FFF800, LENGTH = 512      /*|*/
  flash_bank_1    (xrw) : ORIGIN = 0x08000000, LENGTH = 1024K    /*|*/
  flash_bank_2    (xrw) : ORIGIN = 0x8FFF0000, LENGTH = 1024K    /*|*/
  flash_system_1  (xrw) : ORIGIN = 0x1FF000000, LENGTH = 64K     /*|*/
  flash_system_2  (xrw) : ORIGIN = 0x1FF10000, LENGTH = 44K      /*|*/
  dtcm_ram        (xrw) : ORIGIN = 0x20000000, LENGTH = 128K     /*|*/
                                                                 /*|*/
  /* accessible by DMA1 and DMA2 */                              /*|*/
  axi_sram1       (xrw) : ORIGIN = 0x24000000, LENGTH = 256K     /*|*/
  axi_sram2       (xrw) : ORIGIN = 0x24040000, LENGTH = 384K     /*|*/
  axi_sram3       (xrw) : ORIGIN = 0x240A0000, LENGTH = 384K     /*|*/
  ahb_sram1       (xrw) : ORIGIN = 0x30000000, LENGTH = 64K      /*|*/
  ahb_sram2       (xrw) : ORIGIN = 0x30010000, LENGTH = 64K      /*|*/
  srd_ram         (xrw) : ORIGIN = 0x30000000, LENGTH = 32k      /*|*/
  backup_ram      (xrw) : ORIGIN = 0x30010000, LENGTH = 4k       /*|*/
                                                                 /*|*/
  /*STM32H7B3I-DK on board external to SOC */                    /*|*/
  octospi1_flash  (xrw) : ORIGIN = 0x90000000, LENGTH = 64M      /*|*/
  sdram_bank1     (xrw) : ORIGIN = 0xC0000000, LENGTH = 32M      /*|*/
                                                                 /*|*/
  /* Overlays : Contiguous block names */                        /*|*/
  flash_bank_1_2 (xrw) : ORIGIN = 0x08000000, LENGTH = 2048K     /*|*/
  axi_ram_1_2_3  (xrw) : ORIGIN = 0x24000000, LENGTH = 1024K     /*|*/
  ahb_ram_1_2    (xrw) : ORIGIN = 0x30000000, LENGTH = 128K      /*|*/
                                                                 /*|*/
  NOTE: flash_system_1,flash_system_2,flash_stm are not usuable  /*|*/
        for applications                                         /*|*/
/*------------------------------------------------------------------*/

  /* ======================== */
  /* Application memory usage */
  /* ======================== */
  flash       (rx)  : ORIGIN = 0x08000000, LENGTH = 64k
  ram1        (xrw) : ORIGIN = 0x10000000, LENGTH = 64k
  ram2        (xrw) : ORIGIN = 0x20000000, LENGTH = 64k
  dma1_ram    (xrw) : ORIGIN = ahb_sram1, LENGTH = 64k
}
```


##### Buffer creation in code

The STM32 DMA requires buffers to be aligned on 4 byte boundaries and the buffer size must be an integer number of 32 bits
In this example ahb_sram1 has been chosen for the location of the buffer.
```
// e.g.
#define BUFFERSIZE  32  ; // where BUFFERSIZE/4 == 0 and no remainder
uint8_t buffer[BUFFERSIZE] __attribute__ ((section(".dma1_ram"))) __attribute__ ((aligned (4)));
```

