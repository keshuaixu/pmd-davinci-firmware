/******************************************************************************
 *
 * linkPMD.ld - Linker script for PMD C-Motion Engine applications
 *
 * Copyright (c) 2009 Performance Motion Devices, Inc.  
 *
 *****************************************************************************/

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00040000, LENGTH = 256K
      RAM (rwx) : ORIGIN = 0x04016000, LENGTH = 8K
}

SECTIONS
{
    .text :
    {
	KEEP(*(header))
	FILL(0xFF)
	. = 0x18;
        KEEP(*(filename))
        KEEP(*(fileversion))
	. = 0x20;
	KEEP(*(date))
	. = 0x40;
        KEEP(*(funcstart))
	. = 0x60;
        KEEP(*(functable))
	. = . + 4;
        KEEP(*(funcstack))
	. = . + 4;
        KEEP(*(funcnames))
        *(.text*)
	*(.rodata*)
	. = ALIGN(8);
        _etext = .;
    } > FLASH

    /* initialized data in RAM which is copied in startup.c */
    .data : 
    {
        _data = .;
        *(.data)
        *(.data*)
	. = ALIGN(8);
        _edata = .;
    } > RAM AT > FLASH

    /* uninitialized data in RAM */
    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
	. = ALIGN(8);
	end = .;
	_end = .;
        _ebss = .;
    } > RAM

}

