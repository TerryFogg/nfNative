
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



<img src=".\\Documents\Notes\nanoFrameworkArchitecture.svg">
