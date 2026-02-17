/* Linker script for BPI-R4 kernel user programs */

ENTRY(_start)

/* User programs load at 0x40010000 by default */
/* Override with --defsym=USER_BASE=0xADDRESS if needed */
USER_BASE = DEFINED(USER_BASE) ? USER_BASE : 0x40010000;

SECTIONS {
    . = USER_BASE;

    /* Code section - must start with _start */
    .text : {
        *(.text._start)
        *(.text .text.*)
    }

    /* Read-only data - page aligned to avoid overlapping with .text */
    .rodata : ALIGN(4K) {
        *(.rodata .rodata.*)
    }

    /* Initialized data - page aligned */
    .data : ALIGN(4K) {
        *(.data .data.*)
    }

    /* Zero-initialized data */
    .bss : ALIGN(4K) {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    }

    /* Discard unnecessary sections */
    /DISCARD/ : {
        *(.comment)
        *(.note*)
        *(.eh_frame*)
        *(.ARM.exidx*)
        *(.ARM.attributes*)
    }
}
