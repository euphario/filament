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

/*
 * Secondary CPU entry point
 * Called via PSCI CPU_ON or when mailbox is set
 */
.global secondary_start
secondary_start:
    /* Disable interrupts */
    msr     daifset, #0xf

    /* Get CPU ID */
    mrs     x0, mpidr_el1
    and     x0, x0, #0xff

    /* Check if we're at EL2, drop to EL1 if needed */
    mrs     x1, currentel
    lsr     x1, x1, #2
    and     x1, x1, #0x3
    cmp     x1, #2
    b.ne    secondary_el1_entry

    /* EL2 -> EL1 setup for secondary */
    mov     x1, #(1 << 31)
    msr     hcr_el2, x1
    mov     x1, #0b1111000101
    msr     spsr_el2, x1
    adr     x1, secondary_el1_entry
    msr     elr_el2, x1
    mrs     x1, cnthctl_el2
    orr     x1, x1, #3
    msr     cnthctl_el2, x1
    msr     cntvoff_el2, xzr
    mov     x1, #0x33ff
    msr     cptr_el2, x1
    eret

secondary_el1_entry:
    /* x0 still has CPU ID */

    /* Load stack pointer from per-CPU stacks */
    /* Stack address = CPU_STACKS + (cpu_id + 1) * CPU_STACK_SIZE */
    ldr     x1, =CPU_STACKS
    mov     x2, #(16 * 1024)        // CPU_STACK_SIZE
    add     x3, x0, #1
    mul     x2, x2, x3
    add     sp, x1, x2

    /* Set up VBAR_EL1 to kernel's exception vectors */
    ldr     x1, =exception_vectors
    /* Convert to kernel virtual address */
    mov     x2, #0xffff000000000000
    orr     x1, x1, x2
    msr     vbar_el1, x1

    /* Call Rust secondary CPU entry with CPU ID in x0 */
    bl      secondary_cpu_entry

    /* Should not return, but if it does, halt */
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
    /* Switch to kernel TTBR0 first (user TTBR0 doesn't map kernel/UART) */
    adrp    x9, KERNEL_TTBR0
    ldr     x9, [x9, :lo12:KERNEL_TTBR0]
    msr     ttbr0_el1, x9
    isb

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
 * IRQ handler - check if from user mode and handle accordingly
 */
irq_handler:
    /* Check if IRQ came from EL0 (user mode) by examining SPSR_EL1.M[3:0] */
    mrs     x9, spsr_el1
    and     x9, x9, #0xf            // Extract M[3:0] - exception level
    cbnz    x9, irq_from_kernel     // If not 0, came from EL1 (kernel)
    b       irq_from_user           // Came from EL0 (user)

/*
 * IRQ from kernel mode - simple save/restore on kernel stack
 */
irq_from_kernel:
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

    /* Call Rust IRQ handler (no preemption for kernel IRQs) */
    mov     x0, #0                  // from_user = false
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
 * IRQ from user mode - save to trap frame, potentially switch tasks
 * Similar to svc_handler but for asynchronous preemption
 */
irq_from_user:
    /*
     * CRITICAL: Switch TTBR0 to kernel identity mapping BEFORE any stack access!
     * Use TPIDR_EL1 as scratch to save x9.
     */
    msr     tpidr_el1, x9           // Save original x9 to scratch register

    /* Switch to kernel TTBR0 */
    adrp    x9, KERNEL_TTBR0
    ldr     x9, [x9, :lo12:KERNEL_TTBR0]
    msr     ttbr0_el1, x9
    isb

    /* Now kernel stack is accessible - save x10, x11, x12 to stack */
    stp     x10, x11, [sp, #-16]!
    str     x12, [sp, #-8]!

    /* Get pointer to current task's trap frame */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]

    /* Save all general purpose registers to trap frame */
    stp     x0, x1, [x9, #0]        // x0=0, x1=8
    stp     x2, x3, [x9, #16]       // x2=16, x3=24
    stp     x4, x5, [x9, #32]       // x4=32, x5=40
    stp     x6, x7, [x9, #48]       // x6=48, x7=56
    str     x8, [x9, #64]           // x8=64
    /* Get original x9-x12 from their temp locations */
    mrs     x10, tpidr_el1          // x10 = original x9
    ldr     x11, [sp], #8           // x11 = original x12
    str     x11, [x9, #96]          // x12=96
    ldp     x11, x12, [sp], #16     // x11 = original x10, x12 = original x11
    stp     x10, x11, [x9, #72]     // x9=72, x10=80
    str     x12, [x9, #88]          // x11=88
    str     x13, [x9, #104]         // x13=104
    stp     x14, x15, [x9, #112]    // x14=112, x15=120
    stp     x16, x17, [x9, #128]    // x16=128, x17=136
    stp     x18, x19, [x9, #144]    // x18=144, x19=152
    stp     x20, x21, [x9, #160]    // x20=160, x21=168
    stp     x22, x23, [x9, #176]    // x22=176, x23=184
    stp     x24, x25, [x9, #192]    // x24=192, x25=200
    stp     x26, x27, [x9, #208]    // x26=208, x27=216
    stp     x28, x29, [x9, #224]    // x28=224, x29=232
    str     x30, [x9, #240]         // x30=240

    /* Save special registers */
    mrs     x10, sp_el0
    mrs     x11, elr_el1
    mrs     x12, spsr_el1
    str     x10, [x9, #248]         // sp_el0
    str     x11, [x9, #256]         // elr_el1
    str     x12, [x9, #264]         // spsr_el1

    /* Call Rust IRQ handler with from_user = true */
    mov     x0, #1                  // from_user = true
    bl      irq_handler_rust

    /* irq_handler_rust may have scheduled a different task */
    /* Reload CURRENT_TRAP_FRAME (may have changed!) */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]

    /* Restore special registers from trap frame */
    ldr     x10, [x9, #248]         // sp_el0
    ldr     x11, [x9, #256]         // elr_el1
    ldr     x12, [x9, #264]         // spsr_el1
    msr     sp_el0, x10
    msr     elr_el1, x11
    msr     spsr_el1, x12

    /* Load CURRENT_TTBR0 (may be different task's address space) */
    adrp    x10, CURRENT_TTBR0
    ldr     x10, [x10, :lo12:CURRENT_TTBR0]
    msr     ttbr0_el1, x10
    isb
    tlbi    vmalle1
    dsb     sy
    isb

    /* Convert x9 (trap frame pointer) from physical to TTBR1 virtual address */
    movz    x10, #0xFFFF, lsl #48
    orr     x9, x9, x10

    /* Restore all general purpose registers from trap frame */
    ldp     x0, x1, [x9, #0]
    ldp     x2, x3, [x9, #16]
    ldp     x4, x5, [x9, #32]
    ldp     x6, x7, [x9, #48]
    ldr     x8, [x9, #64]
    /* Skip x9 for now, load x10-x30 */
    ldr     x10, [x9, #80]
    ldr     x11, [x9, #88]
    ldp     x12, x13, [x9, #96]
    ldp     x14, x15, [x9, #112]
    ldp     x16, x17, [x9, #128]
    ldp     x18, x19, [x9, #144]
    ldp     x20, x21, [x9, #160]
    ldp     x22, x23, [x9, #176]
    ldp     x24, x25, [x9, #192]
    ldp     x26, x27, [x9, #208]
    ldp     x28, x29, [x9, #224]
    ldr     x30, [x9, #240]
    /* Finally restore x9 */
    ldr     x9, [x9, #72]

    /* Return to user mode (possibly different task!) */
    eret

/*
 * SVC (Syscall) handler
 * Saves full user state to CURRENT_TRAP_FRAME, handles syscall,
 * then restores from CURRENT_TRAP_FRAME (which may be a different task after scheduling)
 */
svc_handler:
    /*
     * CRITICAL: Switch TTBR0 to kernel identity mapping BEFORE any stack access!
     * The kernel stack is at a physical address that isn't mapped in user TTBR0.
     * Use TPIDR_EL1 as scratch to save x9.
     */
    msr     tpidr_el1, x9           // Save original x9 to scratch register

    /* Switch to kernel TTBR0 - adrp generates TTBR1 address (PC-relative) */
    adrp    x9, KERNEL_TTBR0
    ldr     x9, [x9, :lo12:KERNEL_TTBR0]
    msr     ttbr0_el1, x9
    isb

    /* Now kernel stack is accessible - save x10, x11, x12 to stack */
    stp     x10, x11, [sp, #-16]!
    str     x12, [sp, #-8]!

    /* Get pointer to current task's trap frame */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]

    /* Save all general purpose registers to trap frame */
    /* TrapFrame layout: x0(0) x1(8) x2(16) ... x30(240), sp_el0(248), elr_el1(256), spsr_el1(264) */
    stp     x0, x1, [x9, #0]        // x0=0, x1=8
    stp     x2, x3, [x9, #16]       // x2=16, x3=24
    stp     x4, x5, [x9, #32]       // x4=32, x5=40
    stp     x6, x7, [x9, #48]       // x6=48, x7=56
    str     x8, [x9, #64]           // x8=64
    /* Get original x9-x12 from their temp locations */
    mrs     x10, tpidr_el1          // x10 = original x9
    ldr     x11, [sp], #8           // x11 = original x12
    str     x11, [x9, #96]          // x12=96
    ldp     x11, x12, [sp], #16     // x11 = original x10, x12 = original x11
    stp     x10, x11, [x9, #72]     // x9=72, x10=80
    str     x12, [x9, #88]          // x11=88
    str     x13, [x9, #104]         // x13=104
    stp     x14, x15, [x9, #112]    // x14=112, x15=120
    stp     x16, x17, [x9, #128]    // x16=128, x17=136
    stp     x18, x19, [x9, #144]    // x18=144, x19=152
    stp     x20, x21, [x9, #160]    // x20=160, x21=168
    stp     x22, x23, [x9, #176]    // x22=176, x23=184
    stp     x24, x25, [x9, #192]    // x24=192, x25=200
    stp     x26, x27, [x9, #208]    // x26=208, x27=216
    stp     x28, x29, [x9, #224]    // x28=224, x29=232
    str     x30, [x9, #240]         // x30=240

    /* Save special registers */
    mrs     x10, sp_el0
    mrs     x11, elr_el1
    mrs     x12, spsr_el1
    str     x10, [x9, #248]         // sp_el0
    str     x11, [x9, #256]         // elr_el1
    str     x12, [x9, #264]         // spsr_el1

    /*
     * Call Rust syscall handler
     * syscall_handler_rust(arg0, arg1, arg2, arg3, arg4, arg5, unused, num)
     */
    ldp     x0, x1, [x9, #0]        // arg0, arg1
    ldp     x2, x3, [x9, #16]       // arg2, arg3
    ldp     x4, x5, [x9, #32]       // arg4, arg5
    mov     x6, #0                  // unused
    ldr     x7, [x9, #64]           // syscall number (was x8)
    bl      syscall_handler_rust

    /* syscall_handler_rust may have scheduled a different task */
    /* Reload CURRENT_TRAP_FRAME (may have changed!) */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]

    /* Store syscall return value in trap frame x0 */
    str     x0, [x9, #0]

    /* Restore special registers from trap frame */
    ldr     x10, [x9, #248]         // sp_el0
    ldr     x11, [x9, #256]         // elr_el1
    ldr     x12, [x9, #264]         // spsr_el1
    msr     sp_el0, x10
    msr     elr_el1, x11
    msr     spsr_el1, x12

    /* Load CURRENT_TTBR0 (may be different task's address space) */
    adrp    x10, CURRENT_TTBR0
    ldr     x10, [x10, :lo12:CURRENT_TTBR0]
    msr     ttbr0_el1, x10
    isb
    tlbi    vmalle1
    dsb     sy
    isb

    /* Convert x9 (trap frame pointer) from physical to TTBR1 virtual address.
     * After TTBR0 switch, we can only access kernel memory via TTBR1. */
    movz    x10, #0xFFFF, lsl #48
    orr     x9, x9, x10

    /* Restore all general purpose registers from trap frame */
    /* x9 now holds TTBR1 virtual address of trap frame */
    ldp     x0, x1, [x9, #0]        // x0=0, x1=8
    ldp     x2, x3, [x9, #16]       // x2=16, x3=24
    ldp     x4, x5, [x9, #32]       // x4=32, x5=40
    ldp     x6, x7, [x9, #48]       // x6=48, x7=56
    ldr     x8, [x9, #64]           // x8=64
    /* Skip x9 for now, load x10-x30 */
    ldr     x10, [x9, #80]          // x10=80
    ldr     x11, [x9, #88]          // x11=88
    ldp     x12, x13, [x9, #96]     // x12=96, x13=104
    ldp     x14, x15, [x9, #112]    // x14=112, x15=120
    ldp     x16, x17, [x9, #128]    // x16=128, x17=136
    ldp     x18, x19, [x9, #144]    // x18=144, x19=152
    ldp     x20, x21, [x9, #160]    // x20=160, x21=168
    ldp     x22, x23, [x9, #176]    // x22=176, x23=184
    ldp     x24, x25, [x9, #192]    // x24=192, x25=200
    ldp     x26, x27, [x9, #208]    // x26=208, x27=216
    ldp     x28, x29, [x9, #224]    // x28=224, x29=232
    ldr     x30, [x9, #240]         // x30=240
    /* Finally restore x9 */
    ldr     x9, [x9, #72]           // x9=72

    /* Return to user mode (possibly different task!) */
    eret
