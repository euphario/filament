/*
 * Linker script for BPI-R4 bare-metal kernel
 *
 * Memory layout for MT7988A:
 * - DRAM starts at 0x40000000
 * - We load at 0x46000000 (U-Boot convention)
 * - Stack grows downward from load address
 */

ENTRY(_start)

MEMORY
{
    /*
     * RAM region where U-Boot loads us
     * 0x46000000 is the default loadaddr in U-Boot
     * We allocate 16MB for the kernel
     */
    RAM (rwx) : ORIGIN = 0x46000000, LENGTH = 16M
}

SECTIONS
{
    . = ORIGIN(RAM);

    /* Code section - must be first for entry point */
    .text : {
        KEEP(*(.text.boot))    /* Boot code first */
        *(.text .text.*)
    } > RAM

    /* Read-only data */
    .rodata : ALIGN(4K) {
        *(.rodata .rodata.*)
    } > RAM

    /* Initialized data */
    .data : ALIGN(4K) {
        *(.data .data.*)
    } > RAM

    /* BSS - zero-initialized data */
    .bss : ALIGN(4K) {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    } > RAM

    /* Stack - 64KB, grows downward */
    . = ALIGN(16);
    . = . + 64K;
    __stack_top = .;

    /* Heap starts after stack */
    __heap_start = .;

    /* Discard unneeded sections */
    /DISCARD/ : {
        *(.comment)
        *(.note*)
        *(.eh_frame*)
    }
}

/* Export symbols for assembly and Rust */
__kernel_start = ORIGIN(RAM);
__kernel_end = .;
