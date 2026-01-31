/*
 * Linker script for QEMU virt machine
 *
 * Memory layout for QEMU virt:
 * - DRAM starts at 0x40000000
 * - QEMU -kernel loads at 0x40080000 (after firmware area)
 * - Kernel runs at virtual 0xFFFF000040080000 (TTBR1)
 *
 * The kernel is linked at the TTBR1 virtual address so that all
 * symbol references (including vtables) contain virtual addresses.
 * Early boot code calculates the phys/virt offset and enables MMU
 * before any Rust code runs.
 *
 * Physical: 0x0000_0000_4008_0000
 * Virtual:  0xFFFF_0000_4008_0000
 */

ENTRY(_start)

/* Physical load address (where QEMU loads us with -kernel) */
KERNEL_PHYS_BASE = 0x40080000;

/* Virtual address base (TTBR1 upper half) */
KERNEL_VIRT_BASE = 0xFFFF000000000000;

MEMORY
{
    /*
     * RAM region - linked at TTBR1 virtual address
     * Physical load address is 0x40080000
     * Virtual address is 0xFFFF000040080000
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

    /* Early boot stack - 16KB, grows downward.
     * Only used until PMM init, then switched to a PMM-allocated stack
     * with a proper guard page. Keep small to minimize kernel image size. */
    . = ALIGN(16);
    . = . + 16K;
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
