/*
 * Linker script for BPI-R4 bare-metal kernel
 *
 * Memory layout for MT7988A:
 * - DRAM starts at 0x40000000
 * - U-Boot loads us at physical 0x46000000
 * - Kernel runs at virtual 0xFFFF000046000000 (TTBR1)
 *
 * The kernel is linked at the TTBR1 virtual address so that all
 * symbol references (including vtables) contain virtual addresses.
 * Early boot code calculates the phys/virt offset and enables MMU
 * before any Rust code runs.
 *
 * Physical: 0x0000_0000_4600_0000
 * Virtual:  0xFFFF_0000_4600_0000
 */

ENTRY(_start)

/* Physical load address (where U-Boot puts us) */
KERNEL_PHYS_BASE = 0x46000000;

/* Virtual address base (TTBR1 upper half) */
KERNEL_VIRT_BASE = 0xFFFF000000000000;

MEMORY
{
    /*
     * RAM region - linked at TTBR1 virtual address
     * Physical load address is 0x46000000
     * Virtual address is 0xFFFF000046000000
     */
    RAM (rwx) : ORIGIN = KERNEL_VIRT_BASE + KERNEL_PHYS_BASE, LENGTH = 16M
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

/* Physical base for boot code (used to compute phys/virt offset) */
__kernel_phys_base = KERNEL_PHYS_BASE;
__kernel_virt_base = KERNEL_VIRT_BASE;
