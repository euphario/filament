/*
 * AArch64 bare-metal startup code for BPI-R4
 *
 * This code runs immediately after U-Boot jumps to us.
 * U-Boot leaves us at EL2 with MMU off.
 */

.section .text.boot
.global _start

_start:
    /*
     * Disable interrupts (DAIF = Debug, Async, IRQ, FIQ)
     */
    msr     daifset, #0xf

    /*
     * Check which CPU core we're on
     * Only core 0 should continue, others go to sleep
     */
    mrs     x0, mpidr_el1
    and     x0, x0, #0xff
    cbnz    x0, secondary_hang

    /*
     * Check current exception level
     */
    mrs     x0, currentel
    lsr     x0, x0, #2
    and     x0, x0, #0x3

    cmp     x0, #2
    b.eq    el2_setup
    cmp     x0, #1
    b.eq    el1_entry
    b       hang                // EL3 or EL0 - shouldn't happen

/*
 * EL2 Setup - Configure hypervisor and drop to EL1
 */
el2_setup:
    /*
     * Configure HCR_EL2 (Hypervisor Configuration Register)
     * RW (bit 31) = 1: EL1 is AArch64
     */
    mov     x0, #(1 << 31)      // RW bit - EL1 is AArch64
    msr     hcr_el2, x0

    /*
     * Configure SPSR_EL2 for EL1 entry:
     * - M[3:0] = 0b0101 (EL1h - EL1 using SP_EL1)
     * - DAIF = 0b1111 (all interrupts masked)
     * - Bits: D(9) A(8) I(7) F(6) M[4:0](0-4)
     */
    mov     x0, #0b1111000101   // DAIF masked, EL1h mode
    msr     spsr_el2, x0

    /*
     * Set ELR_EL2 to el1_entry - where we continue after eret
     */
    adr     x0, el1_entry
    msr     elr_el2, x0

    /*
     * Enable access to counters from EL1
     */
    mrs     x0, cnthctl_el2
    orr     x0, x0, #3          // EL1PCTEN, EL1PCEN
    msr     cnthctl_el2, x0
    msr     cntvoff_el2, xzr    // Virtual offset = 0

    /*
     * No coprocessor traps
     */
    mov     x0, #0x33ff
    msr     cptr_el2, x0

    /*
     * Drop to EL1
     */
    isb
    eret

/*
 * EL1 Entry Point
 */
el1_entry:
    /*
     * Set up exception vectors (before anything else can fault)
     * We use the TTBR1 (kernel) virtual address so exception vectors remain
     * accessible after switching TTBR0 to user address space.
     * Physical address ~= 0x4600_xxxx, TTBR1 VA = 0xFFFF_0000_4600_xxxx
     */
    adr     x0, exception_vectors
    // Load KERNEL_VIRT_BASE (0xFFFF_0000_0000_0000) using explicit encoding
    movz    x1, #0xFFFF, lsl #48    // x1 = 0xFFFF000000000000
    orr     x0, x0, x1              // Convert to kernel VA
    msr     vbar_el1, x0

    /*
     * Enable FPU/SIMD (CPACR_EL1)
     * FPEN bits [21:20] = 0b11 enables EL0 and EL1 access
     */
    mov     x0, #(3 << 20)
    msr     cpacr_el1, x0
    isb

    /*
     * Set up the stack pointer
     */
    ldr     x0, =__stack_top
    mov     sp, x0

    /*
     * Zero out BSS section
     */
    ldr     x0, =__bss_start
    ldr     x1, =__bss_end
bss_loop:
    cmp     x0, x1
    b.ge    bss_done
    str     xzr, [x0], #8
    b       bss_loop

bss_done:
    /*
     * Jump to Rust entry point
     */
    bl      kmain

hang:
    wfe
    b       hang

secondary_hang:
    wfe
    b       secondary_hang

/*
 * Exception Vector Table
 * Must be aligned to 2KB (0x800)
 * Each vector is 128 bytes (32 instructions max)
 */
.balign 0x800
exception_vectors:

/* Current EL with SP_EL0 */
.balign 0x80
curr_el_sp0_sync:
    b       exception_handler
.balign 0x80
curr_el_sp0_irq:
    b       exception_handler
.balign 0x80
curr_el_sp0_fiq:
    b       exception_handler
.balign 0x80
curr_el_sp0_serror:
    b       exception_handler

/* Current EL with SP_ELx */
.balign 0x80
curr_el_spx_sync:
    b       exception_handler
.balign 0x80
curr_el_spx_irq:
    b       irq_handler
.balign 0x80
curr_el_spx_fiq:
    b       exception_handler
.balign 0x80
curr_el_spx_serror:
    b       exception_handler

/* Lower EL using AArch64 */
.balign 0x80
lower_el_aarch64_sync:
    /* Debug: write 'L' to UART using TTBR1 address (TTBR0 has user page tables) */
    movz    x10, #0x1100, lsl #16   // 0x11000000
    movz    x11, #0xFFFF, lsl #48   // KERNEL_VIRT_BASE
    orr     x10, x10, x11           // 0xFFFF000011000000
    mov     x11, #'L'
    str     w11, [x10]
    /* Check if this is an SVC (syscall) */
    mrs     x9, esr_el1
    lsr     x9, x9, #26         // Extract EC (exception class)
    cmp     x9, #0x15           // EC 0x15 = SVC from AArch64
    b.eq    svc_handler
    b       exception_handler   // Not SVC, use generic handler
.balign 0x80
lower_el_aarch64_irq:
    b       irq_handler
.balign 0x80
lower_el_aarch64_fiq:
    b       exception_handler
.balign 0x80
lower_el_aarch64_serror:
    b       exception_handler

/* Lower EL using AArch32 */
.balign 0x80
lower_el_aarch32_sync:
    b       exception_handler
.balign 0x80
lower_el_aarch32_irq:
    b       exception_handler
.balign 0x80
lower_el_aarch32_fiq:
    b       exception_handler
.balign 0x80
lower_el_aarch32_serror:
    b       exception_handler

/*
 * Simple exception handler - just prints and hangs
 */
exception_handler:
    stp     x29, x30, [sp, #-16]!
    stp     x0, x1, [sp, #-16]!

    mrs     x0, esr_el1
    mrs     x1, elr_el1
    mrs     x2, far_el1
    bl      exception_handler_rust

    ldp     x0, x1, [sp], #16
    ldp     x29, x30, [sp], #16
    b       hang

/*
 * IRQ handler - save context, call Rust, restore and return
 */
irq_handler:
    /* Save all caller-saved registers */
    stp     x29, x30, [sp, #-16]!
    stp     x27, x28, [sp, #-16]!
    stp     x25, x26, [sp, #-16]!
    stp     x23, x24, [sp, #-16]!
    stp     x21, x22, [sp, #-16]!
    stp     x19, x20, [sp, #-16]!
    stp     x17, x18, [sp, #-16]!
    stp     x15, x16, [sp, #-16]!
    stp     x13, x14, [sp, #-16]!
    stp     x11, x12, [sp, #-16]!
    stp     x9, x10, [sp, #-16]!
    stp     x7, x8, [sp, #-16]!
    stp     x5, x6, [sp, #-16]!
    stp     x3, x4, [sp, #-16]!
    stp     x1, x2, [sp, #-16]!
    str     x0, [sp, #-8]!

    /* Call Rust IRQ handler */
    bl      irq_handler_rust

    /* Restore registers */
    ldr     x0, [sp], #8
    ldp     x1, x2, [sp], #16
    ldp     x3, x4, [sp], #16
    ldp     x5, x6, [sp], #16
    ldp     x7, x8, [sp], #16
    ldp     x9, x10, [sp], #16
    ldp     x11, x12, [sp], #16
    ldp     x13, x14, [sp], #16
    ldp     x15, x16, [sp], #16
    ldp     x17, x18, [sp], #16
    ldp     x19, x20, [sp], #16
    ldp     x21, x22, [sp], #16
    ldp     x23, x24, [sp], #16
    ldp     x25, x26, [sp], #16
    ldp     x27, x28, [sp], #16
    ldp     x29, x30, [sp], #16

    /* Return from exception */
    eret

/*
 * SVC (Syscall) handler
 * Arguments are in x0-x5, syscall number in x8
 * Return value goes in x0
 */
svc_handler:
    /* Save user TTBR0 and switch to kernel identity mapping */
    mrs     x9, ttbr0_el1           // Save user TTBR0 in x9
    adrp    x10, KERNEL_TTBR0
    ldr     x10, [x10, :lo12:KERNEL_TTBR0]
    msr     ttbr0_el1, x10          // Switch to kernel TTBR0
    isb

    /* Save all caller-saved registers to stack (including x9 with user TTBR0) */
    stp     x29, x30, [sp, #-16]!
    stp     x27, x28, [sp, #-16]!
    stp     x25, x26, [sp, #-16]!
    stp     x23, x24, [sp, #-16]!
    stp     x21, x22, [sp, #-16]!
    stp     x19, x20, [sp, #-16]!
    stp     x17, x18, [sp, #-16]!
    stp     x15, x16, [sp, #-16]!
    stp     x13, x14, [sp, #-16]!
    stp     x11, x12, [sp, #-16]!
    stp     x9, x10, [sp, #-16]!    // x9 = user TTBR0
    stp     x7, x8, [sp, #-16]!
    stp     x5, x6, [sp, #-16]!
    stp     x3, x4, [sp, #-16]!
    stp     x1, x2, [sp, #-16]!
    str     x0, [sp, #-8]!

    /*
     * Call Rust syscall handler
     * syscall_handler_rust(arg0, arg1, arg2, arg3, arg4, arg5, unused, num)
     * Arguments already in x0-x5, need to pass x8 as syscall number
     * x6 = unused (0), x7 = syscall number (was in x8)
     */
    mov     x6, #0              // unused
    mov     x7, x8              // syscall number
    bl      syscall_handler_rust

    /* Return value is in x0 - store it where we saved original x0 */
    str     x0, [sp]            // Overwrite saved x0 with return value

    /* Restore all registers including modified x0 */
    ldr     x0, [sp], #8        // x0 now has syscall return value
    ldp     x1, x2, [sp], #16
    ldp     x3, x4, [sp], #16
    ldp     x5, x6, [sp], #16
    ldp     x7, x8, [sp], #16
    ldp     x9, x10, [sp], #16  // x9 = user TTBR0
    ldp     x11, x12, [sp], #16
    ldp     x13, x14, [sp], #16
    ldp     x15, x16, [sp], #16
    ldp     x17, x18, [sp], #16
    ldp     x19, x20, [sp], #16
    ldp     x21, x22, [sp], #16
    ldp     x23, x24, [sp], #16
    ldp     x25, x26, [sp], #16
    ldp     x27, x28, [sp], #16
    ldp     x29, x30, [sp], #16

    /* Restore user TTBR0 before returning */
    msr     ttbr0_el1, x9
    isb

    /* Return to user mode */
    eret
