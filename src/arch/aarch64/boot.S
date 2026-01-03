/*
 * AArch64 bare-metal startup code for BPI-R4
 *
 * This code runs immediately after U-Boot jumps to us.
 * U-Boot leaves us at EL2 with MMU off.
 *
 * The kernel is linked at virtual address 0xFFFF_0000_4600_0000 (TTBR1)
 * but loaded by U-Boot at physical address 0x4600_0000.
 * This boot code enables MMU before calling any Rust code, so that
 * all symbol references (including vtables) work correctly.
 *
 * Key register usage in early boot (before MMU):
 *   x28 = phys-virt offset (add to virtual addr to get physical)
 *   x27 = preserved (callee-saved)
 */

/*
 * Debug UART output macro - outputs a single character
 * Uses x10, x11 as scratch (must save/restore if needed)
 * UART0 base = 0x11000000 physical, 0xFFFF000011000000 virtual (TTBR1)
 * THR = offset 0, LSR = offset 0x14
 *
 * After MMU is enabled, we use TTBR1 virtual address so this works
 * regardless of what TTBR0 is pointing to.
 */
.macro debug_uart_char char
    // Load UART base via TTBR1: 0xFFFF_0000_1100_0000
    movz    x10, #0x1100, lsl #16   // 0x11000000
    movk    x10, #0xFFFF, lsl #48   // Add TTBR1 prefix
1:  ldr     w11, [x10, #0x14]       // Read LSR
    tbz     w11, #5, 1b             // Wait for THRE (bit 5)
    mov     w11, #\char
    str     w11, [x10, #0]          // Write to THR
.endm

.section .text.boot
.global _start

_start:
    /*
     * U-Boot passes DTB address in x0. Save it before we clobber x0.
     * Use x20 (callee-saved) to preserve across function calls.
     */
    mov     x20, x0

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
 *
 * At this point:
 * - Running at physical address ~0x46000000
 * - Symbols contain virtual addresses ~0xFFFF000046000000
 * - Must enable MMU before using any Rust code
 */
el1_entry:
    /*
     * Compute virtual-to-physical offset.
     * adr gives us the runtime (physical) address
     * ldr =symbol gives us the link-time (virtual) address
     * offset = physical - virtual (negative since virt > phys)
     *
     * To convert virtual to physical: phys = virt + offset
     */
    adr     x28, _start             // x28 = physical address of _start
    ldr     x0, =_start             // x0 = virtual address of _start
    sub     x28, x28, x0            // x28 = phys - virt (preserved in x28)

    /*
     * Enable FPU/SIMD (CPACR_EL1) - needed before any code that might use FP
     * FPEN bits [21:20] = 0b11 enables EL0 and EL1 access
     */
    mov     x0, #(3 << 20)
    msr     cpacr_el1, x0
    isb

    /*
     * Set up temporary stack for early boot (before MMU).
     * Use physical address of stack.
     */
    ldr     x0, =__stack_top
    add     x0, x0, x28             // Convert virtual to physical
    mov     sp, x0

    /*
     * Zero out BSS section (using physical addresses)
     */
    ldr     x0, =__bss_start
    add     x0, x0, x28             // Convert to physical
    ldr     x1, =__bss_end
    add     x1, x1, x28             // Convert to physical
bss_loop:
    cmp     x0, x1
    b.ge    bss_done
    str     xzr, [x0], #8
    b       bss_loop

bss_done:
    /*
     * Set up page tables and enable MMU.
     * After this, we'll be running at virtual addresses.
     * x28 still contains phys-virt offset.
     */

    /* Get physical address of page tables */
    ldr     x0, =boot_l0_ttbr0
    add     x0, x0, x28             // x0 = phys addr of TTBR0 L0
    ldr     x1, =boot_l1_ttbr0
    add     x1, x1, x28             // x1 = phys addr of TTBR0 L1
    ldr     x2, =boot_l0_ttbr1
    add     x2, x2, x28             // x2 = phys addr of TTBR1 L0
    ldr     x3, =boot_l1_ttbr1
    add     x3, x3, x28             // x3 = phys addr of TTBR1 L1

    /* Initialize TTBR0 L0: entry 0 points to L1 */
    orr     x4, x1, #3              // Table descriptor: valid + table
    str     x4, [x0]

    /* Block descriptor flags:
     * Bit 0: Valid = 1
     * Bits 4:2: AttrIndx (0 = Device, 1 = Normal)
     * Bit 10: AF (Access Flag) = 1
     * For Device: 0x401 = Valid(1) + AF(0x400) + AttrIndx=0
     * For Normal: 0x405 = Valid(1) + AF(0x400) + AttrIndx=1 (0x4)
     */

    /* Initialize TTBR0 L1: identity map first 4GB */
    /* Block 0: 0x00000000 - Device memory (peripherals) */
    mov     x4, #0x401              // Valid + AF + Device (attr 0)
    str     x4, [x1, #0]
    /* Block 1: 0x40000000 - Normal memory (DRAM) */
    mov     x4, #0x40000000
    add     x4, x4, #0x405          // Valid + AF + Normal (attr 1)
    str     x4, [x1, #8]
    /* Block 2: 0x80000000 - Normal memory (DRAM) */
    mov     x4, #0x80000000
    add     x4, x4, #0x405
    str     x4, [x1, #16]
    /* Block 3: 0xC0000000 - Normal memory (DRAM) */
    mov     x4, #0xC0000000
    add     x4, x4, #0x405
    str     x4, [x1, #24]

    /* Initialize TTBR1 L0: entry 0 points to L1 */
    orr     x4, x3, #3              // Table descriptor: valid + table
    str     x4, [x2]

    /* Initialize TTBR1 L1: map kernel virtual to physical
     * 0xFFFF_0000_0000_0000 -> 0x0000_0000_0000_0000
     * Same as identity map, just accessed via high addresses */
    /* Block 0: Device memory with UXN|PXN */
    mov     x4, #0x401              // Valid + AF + Device
    movk    x4, #0x0060, lsl #48    // UXN (bit 54) + PXN (bit 53)
    str     x4, [x3, #0]
    /* Block 1: Normal memory with UXN (kernel code) */
    mov     x4, #0x40000000
    add     x4, x4, #0x405          // Valid + AF + Normal
    movk    x4, #0x0040, lsl #48    // UXN (bit 54)
    str     x4, [x3, #8]
    /* Block 2-3: Normal memory with UXN */
    mov     x4, #0x80000000
    add     x4, x4, #0x405
    movk    x4, #0x0040, lsl #48
    str     x4, [x3, #16]
    mov     x4, #0xC0000000
    add     x4, x4, #0x405
    movk    x4, #0x0040, lsl #48
    str     x4, [x3, #24]

    /* Set MAIR_EL1: Memory Attribute Indirection Register
     * Attr0: Device-nGnRnE (0x00)
     * Attr1: Normal Write-Back (0xFF)
     * Attr2: Normal Non-Cacheable (0x44)
     */
    ldr     x4, =0x000000000044FF00
    msr     mair_el1, x4

    /* Set TCR_EL1: Translation Control Register
     * T0SZ = 16 (48-bit VA for TTBR0)
     * T1SZ = 16 (48-bit VA for TTBR1)
     * TG0 = 0 (4KB granule)
     * TG1 = 2 (4KB granule)
     * IPS = 5 (48-bit PA)
     * IRGN0/ORGN0 = Write-Back
     * SH0 = Inner Shareable
     */
    ldr     x4, =0x00000005B5103510
    msr     tcr_el1, x4

    /* Set TTBR0_EL1 (user/identity mapping) */
    msr     ttbr0_el1, x0

    /* Set TTBR1_EL1 (kernel mapping) */
    msr     ttbr1_el1, x2

    /* Ensure all writes complete */
    dsb     ish
    isb

    /* Invalidate TLBs */
    tlbi    vmalle1is
    dsb     ish
    isb

    /* Enable MMU via SCTLR_EL1 */
    mrs     x4, sctlr_el1
    orr     x4, x4, #(1 << 0)       // M: Enable MMU
    orr     x4, x4, #(1 << 2)       // C: Enable data cache
    orr     x4, x4, #(1 << 12)      // I: Enable instruction cache
    mov     x5, #(1 << 26)
    orr     x4, x4, x5              // UCI: Allow cache maint from EL0
    /* Enable PAN (Privileged Access Never) for security
     * SPAN (bit 23) = 0 means PSTATE.PAN = 1 on exception entry
     * This prevents kernel from accidentally accessing user memory */
    mov     x5, #(1 << 23)
    bic     x4, x4, x5              // Clear SPAN to enable PAN
    msr     sctlr_el1, x4
    isb

    /*
     * MMU is now enabled. We're still running at physical address
     * via the TTBR0 identity mapping. Jump to TTBR1 virtual address.
     */
    ldr     x0, =post_mmu_entry     // Virtual address
    br      x0

/*
 * Post-MMU entry point.
 * Now running at virtual address 0xFFFF_0000_46xx_xxxx.
 * Stack already set (still valid via TTBR1).
 * Can use symbols directly without offset adjustment.
 */
post_mmu_entry:
    /* Re-set stack pointer to virtual address */
    ldr     x0, =__stack_top
    mov     sp, x0

    /*
     * Set up exception vectors using virtual address.
     * Since we're now running in TTBR1 space, the symbol value is correct.
     */
    ldr     x0, =exception_vectors
    msr     vbar_el1, x0

    /* Store boot page table addresses for use by Rust MMU code */
    ldr     x0, =boot_l0_ttbr0
    ldr     x1, =boot_l0_ttbr1
    /* Convert to physical for TTBR values */
    adr     x2, _start
    ldr     x3, =_start
    sub     x2, x2, x3              // x2 = phys-virt offset

    add     x0, x0, x2              // physical addr of TTBR0 tables
    add     x1, x1, x2              // physical addr of TTBR1 tables

    /* Store in globals for Rust */
    adrp    x3, BOOT_TTBR0
    str     x0, [x3, :lo12:BOOT_TTBR0]
    adrp    x3, BOOT_TTBR1
    str     x1, [x3, :lo12:BOOT_TTBR1]

    /* Store DTB pointer (from x20, saved at _start) */
    adrp    x3, BOOT_DTB_ADDR
    str     x20, [x3, :lo12:BOOT_DTB_ADDR]

    /*
     * Jump to Rust entry point.
     * Rust code now runs with MMU enabled, all symbols valid.
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

    /* Secondary CPUs start with MMU already enabled by primary.
     * Load stack pointer from per-CPU stacks (virtual address) */
    ldr     x1, =CPU_STACKS
    mov     x2, #(16 * 1024)        // CPU_STACK_SIZE
    add     x3, x0, #1
    mul     x2, x2, x3
    add     sp, x1, x2

    /* Set up VBAR_EL1 to kernel's exception vectors (virtual address) */
    ldr     x1, =exception_vectors
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
 * Note: Kernel stack is at TTBR1 address, so we can access it directly
 * without switching TTBR0. User TTBR0 remains active.
 */
exception_handler:
    stp     x29, x30, [sp, #-16]!
    stp     x0, x1, [sp, #-16]!

    /* Debug: exception entry (UART via TTBR1) */
    debug_uart_char 'E'

    mrs     x0, esr_el1
    mrs     x1, elr_el1
    mrs     x2, far_el1
    bl      exception_handler_rust

    ldp     x0, x1, [sp], #16
    ldp     x29, x30, [sp], #16
    b       hang

/*
 * IRQ handler - check if from user mode and handle accordingly
 * NOTE: Cannot use UART debug here - TTBR0 may be user space (0x11000000 not mapped)
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
 *
 * With kernel linked at TTBR1 virtual address, vtables contain virtual
 * addresses that work directly. No need to switch TTBR0 on entry.
 * Kernel stack and trap frames are in TTBR1 space, always accessible.
 */
irq_from_user:
    /* Save x9-x12 to kernel stack (which is in TTBR1 space) */
    stp     x9, x10, [sp, #-16]!
    stp     x11, x12, [sp, #-16]!

    /* Get pointer to current task's trap frame (already a virtual address) */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]

    /* Save all general purpose registers to trap frame */
    stp     x0, x1, [x9, #0]        // x0=0, x1=8
    stp     x2, x3, [x9, #16]       // x2=16, x3=24
    stp     x4, x5, [x9, #32]       // x4=32, x5=40
    stp     x6, x7, [x9, #48]       // x6=48, x7=56
    str     x8, [x9, #64]           // x8=64
    /* Get original x9-x12 from stack */
    ldp     x10, x11, [sp], #16     // x10 = original x11, x11 = original x12
    str     x11, [x9, #96]          // x12=96
    str     x10, [x9, #88]          // x11=88
    ldp     x10, x11, [sp], #16     // x10 = original x9, x11 = original x10
    stp     x10, x11, [x9, #72]     // x9=72, x10=80
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

    /* Clear SYSCALL_SWITCHED_TASK before calling irq_exit_resched
     * so we can detect if it actually switched tasks */
    adrp    x10, SYSCALL_SWITCHED_TASK
    str     xzr, [x10, :lo12:SYSCALL_SWITCHED_TASK]

    /* Safe point: check NEED_RESCHED flag and do deferred scheduling.
     * This is where timer preemption actually happens - the IRQ handler
     * just sets the flag, this function does the actual task switch.
     * If it switches tasks, it sets SYSCALL_SWITCHED_TASK = 1. */
    bl      irq_exit_resched

    /* irq_exit_resched may have scheduled a different task */
    /* Reload CURRENT_TRAP_FRAME (may have changed!) */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]

    /* Switch TTBR0 to current task's user address space for eret.
     * Use TLBI to ensure no stale TLB entries. */
    adrp    x10, CURRENT_TTBR0
    ldr     x10, [x10, :lo12:CURRENT_TTBR0]
    msr     ttbr0_el1, x10
    isb
    tlbi    vmalle1
    dsb     ish
    isb

    /* Restore special registers from trap frame */
    ldr     x10, [x9, #248]         // sp_el0
    ldr     x11, [x9, #256]         // elr_el1
    ldr     x12, [x9, #264]         // spsr_el1
    msr     sp_el0, x10
    msr     elr_el1, x11
    msr     spsr_el1, x12

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
 *
 * With kernel linked at TTBR1 virtual address, vtables contain virtual
 * addresses that work directly. No need to switch TTBR0 on entry.
 */
svc_handler:
    /* Save x9-x12 to kernel stack (which is in TTBR1 space) */
    stp     x9, x10, [sp, #-16]!
    stp     x11, x12, [sp, #-16]!

    /* Get pointer to current task's trap frame (already a virtual address) */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]

    /* Save all general purpose registers to trap frame */
    /* TrapFrame layout: x0(0) x1(8) x2(16) ... x30(240), sp_el0(248), elr_el1(256), spsr_el1(264) */
    stp     x0, x1, [x9, #0]        // x0=0, x1=8
    stp     x2, x3, [x9, #16]       // x2=16, x3=24
    stp     x4, x5, [x9, #32]       // x4=32, x5=40
    stp     x6, x7, [x9, #48]       // x6=48, x7=56
    str     x8, [x9, #64]           // x8=64
    /* Get original x9-x12 from stack */
    ldp     x10, x11, [sp], #16     // x10 = original x11, x11 = original x12
    str     x11, [x9, #96]          // x12=96
    str     x10, [x9, #88]          // x11=88
    ldp     x10, x11, [sp], #16     // x10 = original x9, x11 = original x10
    stp     x10, x11, [x9, #72]     // x9=72, x10=80
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
    /* Clear switch flag before syscall */
    adrp    x10, SYSCALL_SWITCHED_TASK
    str     xzr, [x10, :lo12:SYSCALL_SWITCHED_TASK]

    ldp     x0, x1, [x9, #0]        // arg0, arg1
    ldp     x2, x3, [x9, #16]       // arg2, arg3
    ldp     x4, x5, [x9, #32]       // arg4, arg5
    mov     x6, #0                  // unused
    ldr     x7, [x9, #64]           // syscall number (was x8)
    bl      syscall_handler_rust

    /* Save syscall return value temporarily (x19 is callee-saved) */
    mov     x19, x0

    /* Check if syscall already switched tasks */
    adrp    x10, SYSCALL_SWITCHED_TASK
    ldr     x10, [x10, :lo12:SYSCALL_SWITCHED_TASK]
    cbnz    x10, .Lsvc_already_switched

    /* Syscall didn't switch - store return value to trap frame BEFORE
     * calling do_resched_if_needed. This is critical: if do_resched switches
     * to another task, we need the return value saved now, because x9 will
     * be reloaded to point to the new task's trap frame after the switch.
     * Note: x9 may have been clobbered by syscall_handler_rust (caller-saved),
     * so we must reload CURRENT_TRAP_FRAME here. */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]
    str     x19, [x9, #0]

    /* Now check if reschedule is needed (e.g., timer preemption) */
    bl      do_resched_if_needed

.Lsvc_already_switched:
    /* syscall_handler_rust or do_resched_if_needed may have scheduled a different task */
    /* Reload CURRENT_TRAP_FRAME (may have changed!) */
    adrp    x9, CURRENT_TRAP_FRAME
    ldr     x9, [x9, :lo12:CURRENT_TRAP_FRAME]

    /* Note: return value was already stored above if syscall didn't switch.
     * If syscall DID switch, the new task's trap frame already has its x0. */

.Lsvc_skip_retval:
    /* Switch TTBR0 to current task's user address space for eret.
     * Use TLBI to ensure no stale TLB entries. */
    adrp    x10, CURRENT_TTBR0
    ldr     x10, [x10, :lo12:CURRENT_TTBR0]
    msr     ttbr0_el1, x10
    isb
    tlbi    vmalle1
    dsb     ish
    isb

    /* Restore special registers from trap frame */
    ldr     x10, [x9, #248]         // sp_el0
    ldr     x11, [x9, #256]         // elr_el1
    ldr     x12, [x9, #264]         // spsr_el1
    msr     sp_el0, x10
    msr     elr_el1, x11
    msr     spsr_el1, x12

    /* Restore all general purpose registers from trap frame */
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

/* =============================================================================
 * Data Section - Boot Page Tables
 * =============================================================================
 * These page tables are used during early boot before Rust MMU code runs.
 * They provide identity mapping (TTBR0) and kernel mapping (TTBR1).
 *
 * L0 table: 512 entries, each covers 512GB
 * L1 table: 512 entries, each covers 1GB (block descriptors)
 *
 * We only need entry 0 in L0 (pointing to L1), and entries 0-3 in L1
 * (covering 0-4GB for peripherals and DRAM).
 */

.section .data
.balign 4096
.global boot_l0_ttbr0
boot_l0_ttbr0:
    .fill 512, 8, 0             // 512 entries, all zero initially

.balign 4096
.global boot_l1_ttbr0
boot_l1_ttbr0:
    .fill 512, 8, 0             // 512 entries, all zero initially

.balign 4096
.global boot_l0_ttbr1
boot_l0_ttbr1:
    .fill 512, 8, 0             // 512 entries, all zero initially

.balign 4096
.global boot_l1_ttbr1
boot_l1_ttbr1:
    .fill 512, 8, 0             // 512 entries, all zero initially

/* Global variables for Rust MMU code */
.section .data
.balign 8
.global BOOT_TTBR0
BOOT_TTBR0:
    .quad 0                     // Physical address of boot TTBR0 L0 table

.balign 8
.global BOOT_TTBR1
BOOT_TTBR1:
    .quad 0                     // Physical address of boot TTBR1 L0 table

.balign 8
.global BOOT_DTB_ADDR
BOOT_DTB_ADDR:
    .quad 0                     // DTB address passed from U-Boot in x0
