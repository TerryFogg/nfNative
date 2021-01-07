### Graphics on nanoCLR

##### History
The graphics were originally written for the .netMF ( Micro Framework).
The code was imported and restructed to allow the display controller code to be separated from the controllers physical interface.
The separation allows re-use of the display code that contains the information to initialize a vendor 
display controller from the interface for the controller which may be parallel, SPI or MIPI.
> The complete separation of controller and interface has not completely being achieved ( to be revisited)

#### Graphics memory overview
 The code as imported relies heavily on bitmaps created in memory. 
Graphics primitives draw into the bitmaps before the bitmap is flushed to the 
controller via a display buffer where the final results are created before 
"flushing" the display buffer to the controller.

The netMF memory for graphics allocated space from the CLR memory and if there was not enough CLR memory then used memory from a simple heap if it was configured.
In the port to nanoFramework, the use of CLR memory was not included for now, and a dedicated graphics memory area is used.

>The advantage of using a dedicated graphics memory region I see 
> * Allocation and expanding a large jpeg will not crash the system but only affect the graphics ( which may be handled gracefully).
> * Graphics memory can be placed into PSRAM or external RAM ( slower), while allowing c# code to execute from faster internal limited ram.

> NOTE: This area of memory management should be reviewed or revisited at some stage in the future
 
##### Graphics RAM
The system requires a RAM area where bitmaps are created, JPEG's/Gif/Png are expanded.
In C# a bitmap is created and primitives are used to write into the bitmap. 
The simple WPF system library also uses the same system,
where a window has a drawing context (dc) and this dc has a bitmap where the primitives are written into.
For systems with small amounts of RAM care must be taken when creating bitmaps.
The simple primitive examples use one bit map of one physical screen size to limit the size of RAM required.
Multiple bitmaps, windows Jpegs soon consume a large amount of RAM.


##### Display Buffer
Not all systems will require a display buffer.

###### Controllers without memory
Some Display controllers do not have any memory of their own (this maybe not true of present day ones )
For MCU with graphics controller, e.g. STM32 - LTDC, a display buffer may be
needed depending on the mode and display controller used.

>The example STM32F769I_Discovery board uses a display buffer in video mode.
> The OTM8009a controller on the board does have memory and could be in command mode
> The current implementation does not use this mode ( although it can be a power saving improvement)

Data is moved from a graphics area to the display controller during a "flush".
During the flush some colour code conversion can take place to match the
16-bit colour code or nanoCLR with what is requried by the display controller.

###### Controllers with memory (MGRAM)
For an SPI display controller, these have their own memory, and can move the data from the 
graphics area directly to the SPI display controller, usually in large SPI block writes for performance.
Spi controllers with their own memory (MGRAM) do not need a display buffer.
Colour conversion can take place during the SPI write, reorganising the bit pattern as required by the display controller.


###### Graphics colour space

When working with pixels, colours are represented as 24-bit colours in C\# and are
converted and stored as 16-bit values to conserve memory. Each pixel colour is interpolated from the
24-bit value to the 16 bit value.



##### C\# colours are written into a 24-bit colour code space and stored in memory in the following format, and stored as 16 bits

|         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
|---------|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|
| Bits    | 31 | 30 | 29 | 28 | 27 | 26 | 25 | 24 | 23 | 22 | 21 | 20 | 19 | 18 | 17 | 16 | 15 | 14 | 13 | 12 | 11 | 10 | 9  | 8  | 7  | 6  | 5  | 4  | 3  | 2  | 1  | 0  |
| 24-bit  | \- | \- | \- | \- | \- | \- | \- | \- | B7 | B6 | B5 | B4 | B3 | B2 | B1 | B0 | G7 | G6 | G5 | G4 | G3 | G2 | G1 | G0 | R7 | R6 | R5 | R4 | R3 | R2 | R1 | R0 |
| 16-bit  |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    | B4 | B3 | B2 | B1 | B0 | G5 | G4 | G3 | G2 | G1 | G0 | R4 | R3 | R2 | R1 | R0 |

>During output to the physical display, it may be necessary to reformat the pixel
>when a bitmap is "flushed" to the display frame buffer, or during the transfer
>of the data to a display that has physical on board gRam ( Graphics ram).

*Examples: OTM8009a*

The OTM8009a can be use in video mode or command mode ( using the on board gRam)

| OTM8009A - Supports 24-bit, 18-bit and 16 bit colour formats. |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
|---------------------------------------------------------------|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|
| 24-bit                                                        | 31 | 30 | 29 | 28 | 27 | 26 | 25 | 24 | 23 | 22 | 21 | 20 | 19 | 18 | 17 | 16 | 15 | 14 | 13 | 12 | 11 | 10 | 9  | 8  | 7  | 6  | 5  | 4  | 3  | 2  | 1  | 0  |
|                                                               | \- | \- | \- | \- | \- | \- | \- | \- | R7 | R6 | R5 | R4 | R3 | R2 | R1 | R0 | G7 | G6 | G5 | G4 | G3 | G2 | G1 | G0 | B7 | B6 | B5 | B4 | B3 | B2 | B1 | B0 |
| 18-bit                                                        | \- | \- | \- | \- | \- | \- | R5 | R4 | R3 | R2 | R1 | R0 | G5 | G4 | G3 | G2 | G1 | G0 | B5 | B4 | B3 | B2 | B1 | B0 |    |    |    |    |    |    |    |    |
| 16-bit                                                        | R4 | R3 | R2 | R1 | R0 | G5 | G4 | G3 | G2 | G1 | G0 | B4 | B3 | B2 | B1 | B0 |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |


### Adding a new Display Controller and interface

#### Displays
A file containing the display code for a controller should be placed in *src->nanoFramework.Graphics->Graphics->Displays*
The Display.h have the definitions of routines that are expected by the graphics code.
> Typically the code should be written to be usefull across all platforms
> 
#### Display Interfaces
One of the following should be chosen for the platform.

**Generic Interface**

Code that can be used across platforms, 
> e.g. SPI using common calling code.

A file containing the display interface code for a controller should be placed in *src->nanoFramework.Graphics->Graphics->DisplayInterfaces*
The DisplayInterface.h has the definitions of routines that are expected by the graphics code.

**Specific Interface**

A file containing the display interface code for a controller should be placed in *targets->...**board**->nanoCLR->nanoFramework.Graphics
The DisplayInterface.h has the definitions of routines that are expected by the graphics code.





