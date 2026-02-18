/* Linker script for BPI-R4 kernel user programs */

ENTRY(_start)

/* PIE: base at 0, kernel ELF loader applies random offset at load time */
USER_BASE = 0;

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
