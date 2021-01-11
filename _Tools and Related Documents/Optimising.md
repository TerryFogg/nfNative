Optimising the size of the ARM Cortex devices

**Enable compiler optimization**

To start with, instruct the compiler to optimize for size in release mode
builds. You do this using the -Os compiler command line option. This produces a
release build that is optimized for small size.

If you want to code-size optimize debug mode builds (i.e., builds with debug
instrumentation), use the -Og compiler command line option instead. This
produces a debug build that is optimized for small size. The code cannot be
optimized as much as in release mode, as the binary must still be debuggable –
but it becomes a lot smaller than without -Og.

Commercially polished GNU tools, like Atollic TrueSTUDIO, provide simple GUI
options to enable different types of compiler optimization levels.

**Enable linker dead code and dead data removal**

The compiler translates the C/C++ source code file being compiled into an
intermediate object file, that is later fed into the linker for final
processing. The job of the linker is to combine code and data from many object
files into one flashable output binary file.

Because the compiler only compiles one source code file at a time, it cannot
know if code or data in the file is unused or not. Sure, it knows if a function
in the file is called by another function in the same file. But if the function
in a file is not called by any other function in the same file, the compiler has
no way of knowing if the function is called by a function in some other file, or
if the function is unused and can be removed.

This knowledge is only available at link time; in the final phase where the
linker combines all the intermediate object files and resolves any
interdependencies across the files. This gives the linker the power to remove
unused code and data objects from the output binary flash file, thus saving
valuable memory and reduce waste. But the compiler can’t do it.

The only problem is the GNU tool chain do not enable this linker feature by
default. Dead code and dead data removal must be enabled manually. Thus, many
rookie developers trying out the GNU tools end up with massive binary output
files, as neither unused application functions, nor unused library functions,
are removed.

Commercially polished distributions of the GNU ARM compiler often handle this.
The Atollic TrueSTUDIO IDE, for example, have dead code and dead data enabled by
default. If you haven’t taken the smooth path of using a commercially polished
GNU GCC for ARM distribution, like Atollic TrueSTUDIO, you will have to enable
this yourself.

To enable dead code removal, all the C/C++ source code files must be compiled
using the command line option -ffunction-sections. Furthermore, the linker must
be launched with the linker command line option --gc-sections.

To enable dead data removal, all the C/C++ source code files must be compiled
using the command line option -fdata-sections. Furthermore, the linker must be
launched with the linker command line option --gc-sections.

Beware that any 3rd party software libraries that you receive in binary format
may not be built in this way. This means that any unused functions in the
library might not be removed by the linker, and your flashable binary output
file may be bloated.

**Disable RTTI and exception handling**

If you use C++, make sure to disable RTTI and exception handling, as these can
produce a lot of extra code. Use the compiler options -fno-rtti and
-fno-exceptions to do this.

Atollic TrueSTUDIO does this by default, although you can easily re-configure
this should you want to override this behavior for full C++ functionality.

**Select a compact C runtime library**

The GNU compiler traditionally ships with the Newlib C runtime library by
default. This library is standards compliant, but not compact enough for most
Cortex-M based projects. You can use the Newlib Nano library for smaller code
size.

Additionally, Atollic TrueSTUDIO contains a super-compact “tiny printf”
implementation for further code-size reductions.

**Try Link Time Optimization (LTO)**

As I outlined above, the C compiler traditionally compiles one source code file
at a time, and translate it to a corresponding intermediate object file. The
linker finishes the process by combining all intermediate object files,
resolving any interdependencies, and producing a flashable binary output file.

A consequence of this architecture is that the C compiler has to treat each C
file as an individual sandbox during compilation. When it applies various
optimization techniques, it can only do so on the file itself – without
considering how optimizations could be improved if it knew the contents of the
other files.

And so, there is one more trick the toolchain can do to reduce the code-size of
your project. That is to optimize the code globally, i.e. also across different
C files, not only within the individual C files.

This is now possible with the GNU compiler toolchain, using a re-sequencing
technique called LTO (Link Time Optimization). With LTO, some optimization
algorithms are moved from the compiler to the linker – where they can be applied
to the whole code base, not just locally inside each file in isolation. This
enables the toolchain to optimize also across files, not just within them. This
can reduce the code size further.

You can use the command line option -flto on both the compiler and linker to
enable LTO. Keep in mind, however, that this is a reasonably new addition to the
GNU tool chain that has not been proven in the field for years yet. Your mileage
may thus wary. Since LTO is an experimental feature at this time, Atollic do not
provide any quality guarantee or support related to LTO. But it can be worth
exploring this further if you are aggressively trying to reduce the code size of
your ARM Cortex-M project, and are prepared to be at the forefront of testing
new technologies.

**Summary**

The GNU GCC compiler for ARM Cortex-M is a rock solid, proven C/C++ compiler
with strong optimization. But with additional knowledge and hand-crafted
tweaking, you might get a lot better results compared to if you use the default
settings – alternatively, you can choose a commercially polished ARM Cortex-M
IDE, like Atollic TrueSTUDIO, that handles most of these issues automatically.

Do you want to print out a convenient checklist? Get the GNU GCC codesize
checklist:

**Using GNU GCC on ARM Cortex devices: Placing code and data on special memory
addresses using the GNU LD linker**

Posted by Magnus Unemyr on May 7, 2015 10:04:00 AM

-   **Share**

Many modern microcontroller devices have more than one memory region, and you
may want to locate code or data on fixed memory addresses in any of those memory
regions, for various reasons. It is possible to use the linker script in
the TrueSTUDIO C/C++ IDE and other GNU/GCC-based ARM Cortex-M development tools
to precisely place the code in different memory areas.

To do that modify the .ld-linker script file memory regions. Here is an example
of a linker script file containing the following memory areas:

MEMORY  
{  
FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K  
RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 16K  
MEMORY_B1 (rx) : ORIGIN = 0x60000000, LENGTH = 0K  
}

Add a new area by editing the linker configuration file. In this example, the
IP-Code region is added.

MEMORY  
{  
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 64K  
IP_CODE (x)     : ORIGIN = 0x08010000, LENGTH = 64K  
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 8K  
MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K  
}

Place the following code a bit further down in the script, between the .data {
... } and the .bss { ... } section:

.ip_code :  
{  
\*(.IP_Code\*);  
} \> IP_CODE

This change tells the linker to place all sections named .IP_Code\* into the
IP_CODE memory region that is specified to start at target memory address:
0x8010000.

In the C-code, tell the compiler which functions should go to this section by
adding \__attribute__((section(".IP_Code"))) before the function declaration.
Example:

\__attribute__((section(".IP_Code"))) int placed_logic()  
{  
/\* TODO - Add your application code here \*/  
return 1;  
}

The placed_logic()-function are now placed in the IP_CODE memory region by the
linker.  
It is also possible to use the GNU LD linker to position data on individual
memory addresses in a similar manner.

The first step in order to place variables at a specified address in memory is
to create a new memory region in the linker script (the .ld-file). Take a look
at an example of a linker script file containing the following memory areas:

MEMORY  
{  
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 128K  
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 16K  
MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K  
}

A new memory region should be added by editing the file. In this example add the
MYVARS region.

MEMORY  
{  
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 64K  
MYVARS (x)      : ORIGIN = 0x08010000, LENGTH = 64K  
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 8K  
MEMORY_B1 (rx) : ORIGIN = 0x60000000, LENGTH = 0K  
}

Now the memory section should be added. Place the following a bit further down
in the script, between the .data { ... } and the .bss { ... } section:

.myvars :  
{  
\*(.myvars\*);  
} \> MYVARS

This tells the linker to place all sections named .myvars\* from input into the
.myvars output section in the MYVARS memory region, which is specified to start
at target memory address: 0x8010000. A section can be called almost anything
except some predefined names such as data.

Now the variables need to be put in that region.

To be certain the order will stay the same when they are spread over multiple
source code files, add each variable to a section of its own. Then map the order
of the variables in the linker script.  
So for example, the C code could be:

\__attribute__((section(".myvars.VERSION_NUMBER"))) uint32_tVERSION_NUMBER;  
\__attribute__((section(".myvars.CRC"))) uint32_t CRC;  
\__attribute__((section(".myvars.BUILD_ID"))) uint16_t BUILD_ID;  
\__attribute__((section(".myvars.OTHER_VAR"))) uint8_t OTHER_VAR;

Moreover, then decide the order of the variables in the memory in the linker
script by adding the specially named sections like:

.myvars :  
{  
\*(.myvars.VERSION_NUMBER)  
\*(.myvars.CRC)  
\*(.myvars.BUILD_ID)  
\*(.myvars\*);  
} \> MYVARS

By using the methods outlined above, it is easy to ensure code or data are
linked to specific memory addresses on Cortex-M devices like STM32, Kinetis,
LPC, EFM32 or XMC.

If you want to read more on ARM Cortex development and debugging using GNU
tools, read this whitepaper:

**How to develop and debug BOOTLOADER + APPLICATION systems on ARM Cortex-M
devices**

Posted by Magnus Unemyr on Apr 27, 2015 4:21:00 PM

-   **Share**

As we all painfully have come to learn - firmware is rarely bug-free and new
requirements are usually added over time. And so there is a need for a method to
upgrade the firmware of a shipped product when defects are found or new
functionality is needed. From a logistics point of view, a product recall with a
factory upgrade might not be a feasible option.

By separating the application logic from the booting process, developers can
design a system that allows end-users or service technicians to upgrade the
application software with a newer and presumably better version later on,
without the need for recalls or factory upgrades. This can be done using a
bootloader. But what is a bootloader and how is it implemented and debugged on
an ARM Cortex-M device like STM32 or Kinetis? The post also links to a
bootloader code example for STM32F4-Discovery.

To start with, what is a bootloader? The general definition from Wikipedia is:

*“A bootloader is a computer program that loads the main operating system or
runtime environment for the computer after completion of the self-tests.” -
Wikipedia*

In ARM Cortex-M microcontroller land (for example using STM32, Kinetis, EFM32 or
LPC devices) we take this as our definition instead:

*“A bootloader enriches the capabilities of the microcontroller and makes them a
self-programmable device”*

Essentially, what happens is we create one small software program (the
bootloader) that boots the system and then hand over execution to the larger
application program (the application logic). But this alone really doesn’t add
any extra benefits compared to including the boot logic in the application
software itself.

The trick here is to add one more feature to the bootloader – the capability to
download new application software versions using some communications interface
(TCP/IP, UART, USB, CAN, SD-cards with a file system, or whatever is suitable)
and start to use the upgraded version of the application software instead of the
old one.

We thus end up with a 2-module system performing a 2-step startup; one module
that is never upgraded after factory delivery (the small bootloader), and one
module (the large application software) that can be upgraded any number of times
“in-the-field” to add bug fixes or feature extensions, without any need for
recalls and factory upgrades.

The thinking here of course is the small bootloader is easier to keep
“bug-free”, while the large application software module is more likely to
contain bugs or feature limitations that must be addressed with in-the-field
upgrades during the lifespan of the product. Hence, the bootloader cannot be
upgraded by the end-user or service technician himself, but the application
software can.

And so, creating a Cortex-M system using a BOOTLOADER + APPLICATION approach
requires the following:

-   Constructing and building the static bootloader application

-   Constructing and building the replaceable/upgradeable main software
    application

-   Handling interaction between the bootloader and the main application in the
    Cortex-M target system (in particular execution control handover)

-   Handling debugging of the above system, in particular during execution
    handover

Matters of importance here are the memory layout of the devices, as both the
bootloader and application software needs to be stored in separated areas of the
Flash memory in order not to interfere with each other, what communications
interface to use to download new application versions using the downloader, and
how to store the updated application software in Flash (i.e. flash programming).
In practice, you will need two separate Cortex-M projects (for example developed
using the Atollic TrueSTUDIO IDE for ARM) loaded into the same device:

-   The Bootloader application needs to be stored on certain Flash memory
    addresses, and handle CPU booting, including responding to the
    power-on-reset interrupt, initializing the C runtime environment, and
    performing required hardware setup like enabling chip select signals,
    configuring DRAM refresh etc. It also needs to accept new updated software
    versions using some communications interface, and store the new software
    version in Flash for later use.

-   The Application must be stored in a different area of the Flash memory, and
    most likely need to avoid doing configurations already made by the
    bootloader (remove H/W configurations made from ready-made example projects
    not intended for bootloader use for example), and obviously contains the
    application logic itself.

Debugging imposes special problems in a BOOTLOADER + APPLICATION setup. You can
typically debug the bootloader using most embedded ARM Cortex IDE’s, but debug
problems arise when execution is handed over from the bootloader to the
application software.

The debugger (that started the debug session using the bootloader project) knows
nothing about the C/C++ symbols in the binary image of the application software
stored in the Flash memory of the device, and so you are kind of naked or blind
when debugging the application software module after the bootloader have handed
over execution control to the application software.  You will not be able to do
basic debug tasks like single stepping source code, set breakpoints on source
code lines, or see C/C++ variables in the watch view, etc. This imposes
unacceptable debug limitations to embedded developers and needs a solution.

Essentially the debugger needs to be aware of both binary images (and their ELF
files containing debug info) at the same time. What we do here is really to
debug two completely different IDE projects/application binary images at the
same time, in the same debugger. And so, highly powerful and flexible Cortex-M
debuggers like Atollic TrueSTUDIO is needed for this setup, in conjunction with
a good debugger probe like ST-LINK or SEGGER J-Link/SEGGER J-Trace.

To learn more on how to develop and debug a BOOATLOADER + APPLICATION system on
Cortex-M devices like STM32, Kinetis, EFM32 or LPC, read this free training
presentation:

Read our bootloader training presentation!

It is also possible to download a code example built for an STM32F4-Discovery
board. The zip-file contains two TrueSTUDIO projects, one for the bootloader and
one for the application. Download the example here!

For more information on ARM Cortex development and debugging in general, read
this white paper:
