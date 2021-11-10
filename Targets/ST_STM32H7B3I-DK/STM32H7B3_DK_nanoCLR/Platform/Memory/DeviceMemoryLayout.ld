MEMORY
{
     /*---------------------------------------------------------*/
     /* Physical memory location and size for reference         */
     /*---------------------------------------------------------*/
      /* NOT accessible by DMA1 or DMA2 */                        
      itcm_ram        (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
      
      flash_otp       (xrw) : ORIGIN = 0x08FFF000, LENGTH = 1K    
      flash_stm       (xrw) : ORIGIN = 0x08FFF800, LENGTH = 512   
      flash_bank_1    (xrw) : ORIGIN = 0x08000000, LENGTH = 1024K 
      flash_bank_2    (xrw) : ORIGIN = 0x8FFF0000, LENGTH = 1024K 
      flash_system_1  (xrw) : ORIGIN = 0x1FF000000,LENGTH = 64K   
      flash_system_2  (xrw) : ORIGIN = 0x1FF10000, LENGTH = 44K 
      
      dtcm_ram        (xrw) : ORIGIN = 0x20000000, LENGTH = 128K  
                                                              
      /* accessible by DMA1 and DMA2 */                           
      axi_sram1       (xrw) : ORIGIN = 0x24000000, LENGTH = 256K  
      axi_sram2       (xrw) : ORIGIN = 0x24040000, LENGTH = 384K  
      axi_sram3       (xrw) : ORIGIN = 0x240A0000, LENGTH = 384K  
      ahb_sram1       (xrw) : ORIGIN = 0x30000000, LENGTH = 64K   
      ahb_sram2       (xrw) : ORIGIN = 0x30010000, LENGTH = 64K   
      srd_ram         (xrw) : ORIGIN = 0x30000000, LENGTH = 32k   
      backup_ram      (xrw) : ORIGIN = 0x30010000, LENGTH = 4k    
                                                              
      /*STM32H7B3I-DK on board external to SOC */                 
      octospi1_flash  (xrw) : ORIGIN = 0x90000000, LENGTH = 64M
      
      sdram_bank1     (xrw) : ORIGIN = 0xC0000000, LENGTH = 32M   


                                                              
      /* Overlays : Contiguous block names */                     
      flash_bank_1_2  (xrw) : ORIGIN = 0x08000000, LENGTH = 2048K 
      axi_ram_1_2_3   (xrw) : ORIGIN = 0x24000000, LENGTH = 1024K 
      ahb_ram_1_2     (xrw) : ORIGIN = 0x30000000, LENGTH = 128K 
      
    /*-----------------------------------------------------------*/
    /*   NOTE:                                                   */                                                  
    /*         flash_system_1, flash_system_2, flash_stm         */           
    /*         are not usable for applications                   */           
    /*-----------------------------------------------------------*/

         nanoBooter   (rx)  : ORIGIN = flash_bank_1_2, LENGTH = nanoBooter_size

}



MEMORY
{
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

MEMORY
{
      /* ram */
      ram2         (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
      bootclpbrd   (wx)  : ORIGIN = 0x20000000, LENGTH = 48
      ram0         (xrw) : ORIGIN = 0x20000030, LENGTH = 96K-48

      graphics_ram : org = 0x2407F800, len = 255k                /* One screen size (480x272x2) of working ram*/
      graphics_vfb : org = 0xC0F44800, len = 255k                /* One screen size frame buffer for video output mode  (480x272x2) bytes ( 16 bits RGB565 )*/


      ramvt      (wx) : org = 0x00000000, len = 0                         /* initial RAM address is reserved for a copy of the vector table */

}