/**
 * Standard Bare-Metal Linker Script for STM32F103C8T6
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM); /* Stack end */

SECTIONS{
    /* 1. The Vector Table goes first into FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* 2. The Program Code and Constants go into FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.text)           /* Standard functions */
        *(.text.*)         /* Isolated functions (-ffunction-sections) */
        *(.rodata)         /* Read-only constants */
        *(.rodata.*)       /* Isolated constants (-fdata-sections) */
        . = ALIGN(4);
    } > FLASH

    /* 3. Global C++ Constructors go into FLASH */
    .init_array :
    {
        . = ALIGN(4);
        _sinit = .;
        KEEP(*(.init_array))
        KEEP(*(.init_array.*))
        _einit = .;
        . = ALIGN(4);
    } > FLASH

    /* Used by startup.cpp to initialize SRAM data */
    _sidata = LOADADDR(.data);

    /* 4. Initialized Variables go into SRAM (but are copied from FLASH) */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT>FLASH

    /* 5. Uninitialized Variables (Zeroed out) go into SRAM */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)          /* Standard C uninitialized globals */
        . = ALIGN(4);
        _ebss = .;
        end = .;           /* Defines the end of used SRAM */
    } > SRAM
}