/*
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2006..2015 Giovanni Di Sirio.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
*/

SECTIONS
{

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

    .data : ALIGN(4)
    {
        _sdata = .;        /* create a global symbol at data start */
        PROVIDE(_textdata = LOADADDR(.data));
        PROVIDE(_data = .);
       __textdata_base__ = LOADADDR(.data);
       __data_base__ = .;
        *(.data)
        *(.data.*)
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
        __data_end__ = .;
    } > DATA_RAM AT > DATA_RAM_LMA

    .bss (NOLOAD) : ALIGN(4)
    {
        /* This is used by the startup in order to initialize the .bss section */
        _sbss = .;         /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        __bss_base__ = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
           _ebss = .;         /* define a global symbol at bss end */
     __bss_end__ = .;
        PROVIDE(end = .);
    } > BSS_RAM
}