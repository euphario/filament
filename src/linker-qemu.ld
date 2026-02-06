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
 *
 * CRITICAL: The ELF must have PhysAddr = 0x40080000 so QEMU loads
 * the kernel at the correct physical address. Use AT() to set this.
 */

ENTRY(_start)

/* Physical load address (where QEMU loads us with -kernel) */
KERNEL_PHYS_BASE = 0x40080000;

/* Virtual address base (TTBR1 upper half) */
KERNEL_VIRT_BASE = 0xFFFF000000000000;

/* Combined virtual start */
KERNEL_VIRT_START = KERNEL_VIRT_BASE + KERNEL_PHYS_BASE;

MEMORY
{
    /*
     * RAM region - linked at TTBR1 virtual address
     * Physical load address is 0x40080000
     * Virtual address is 0xFFFF000040080000
     */
    RAM (rwx) : ORIGIN = KERNEL_VIRT_START, LENGTH = 16M
}

SECTIONS
{
    . = ORIGIN(RAM);

    /* Code section - must be first for entry point
     * AT() specifies the physical load address for QEMU */
    .text : AT(KERNEL_PHYS_BASE) {
        KEEP(*(.text.boot))    /* Boot code first */
        *(.text .text.*)
    } > RAM

    /* Read-only data */
    . = ALIGN(4K);
    .rodata : AT(KERNEL_PHYS_BASE + (ADDR(.rodata) - KERNEL_VIRT_START)) {
        *(.rodata .rodata.*)
    } > RAM

    /* Initialized data */
    . = ALIGN(4K);
    .data : AT(KERNEL_PHYS_BASE + (ADDR(.data) - KERNEL_VIRT_START)) {
        *(.data .data.*)
    } > RAM

    /* BSS - zero-initialized data (no load data needed) */
    . = ALIGN(4K);
    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    } > RAM

    /* Guard page (4KB) — left unmapped by MMU, catches stack overflow */
    . = ALIGN(4K);
    __boot_stack_guard = .;
    . = . + 4K;
    /* Early boot stack - 16KB, grows downward from __stack_top.
     * Only used until PMM init, then switched to a PMM-allocated stack. */
    __boot_stack_bottom = .;
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
