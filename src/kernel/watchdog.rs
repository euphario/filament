//! Watchdog NMI — All-CPU State Dump on Deadlock
//!
//! When the kernel deadlocks (all CPUs stuck spinning on a lock), normal IRQs
//! don't fire because DAIF.I is masked by spinlocks. This module uses FIQ
//! (Group 0 interrupts in GICv3) which bypasses the I-bit entirely.
//!
//! **MT7988A**: WDT DUAL_MODE fires SPI 123 as Group 0 FIQ on first timeout,
//! then does hardware reset on second timeout. The FIQ handler sends Group 0
//! SGI 1 to all other CPUs so every core dumps its state.
//!
//! **QEMU**: No hardware WDT. Software watchdog in timer tick calls `soft_nmi()`
//! when no kick for 15 seconds. This is a soft NMI (from IRQ context) so it
//! won't catch IRQ-disabled deadlocks — accepted limitation for emulation.

use core::sync::atomic::{AtomicU32, Ordering};
use crate::kernel::percpu;

/// Prevents re-entry and identifies the primary NMI handler CPU.
/// 0 = no dump in progress, non-zero = CPU ID + 1 of primary handler.
static DUMP_IN_PROGRESS: AtomicU32 = AtomicU32::new(0);

/// Bitmask of CPUs that have finished dumping. Bit N = CPU N done.
static DUMP_DONE_MASK: AtomicU32 = AtomicU32::new(0);

// ============================================================================
// NMI-safe UART output (lock-free, direct hardware register access)
// ============================================================================

/// Direct UART putc — no locks, no allocations.
/// Polls UART TX-ready bit then writes one byte.
#[inline(never)]
fn nmi_putc(byte: u8) {
    #[cfg(feature = "platform-mt7988a")]
    {
        // 16550 UART at 0x11000000 via TTBR1
        const UART_BASE: usize = 0xFFFF_0000_1100_0000;
        const THR: usize = 0x00;
        const LSR: usize = 0x14;
        unsafe {
            let lsr_ptr = UART_BASE + LSR;
            // Poll LSR THRE (bit 5)
            while (core::ptr::read_volatile(lsr_ptr as *const u32) & (1 << 5)) == 0 {
                core::hint::spin_loop();
            }
            core::ptr::write_volatile((UART_BASE + THR) as *mut u32, byte as u32);
        }
    }
    #[cfg(feature = "platform-qemu-virt")]
    {
        // PL011 UART at 0x09000000 via TTBR1
        const UART_BASE: usize = 0xFFFF_0000_0900_0000;
        const UARTDR: usize = 0x00;
        const UARTFR: usize = 0x18;
        unsafe {
            let fr_ptr = UART_BASE + UARTFR;
            // Poll UARTFR TXFF (bit 5) — wait while full
            while (core::ptr::read_volatile(fr_ptr as *const u32) & (1 << 5)) != 0 {
                core::hint::spin_loop();
            }
            core::ptr::write_volatile((UART_BASE + UARTDR) as *mut u32, byte as u32);
        }
    }
}

/// Print a byte slice directly to UART.
fn nmi_puts(s: &[u8]) {
    for &b in s {
        if b == b'\n' {
            nmi_putc(b'\r');
        }
        nmi_putc(b);
    }
}

/// Print a 64-bit value as 16 hex digits.
fn nmi_put_hex64(val: u64) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    nmi_puts(b"0x");
    for i in (0..16).rev() {
        let nibble = ((val >> (i * 4)) & 0xF) as usize;
        nmi_putc(HEX[nibble]);
    }
}

/// Print a 32-bit value as 8 hex digits.
fn nmi_put_hex32(val: u32) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    nmi_puts(b"0x");
    for i in (0..8).rev() {
        let nibble = ((val >> (i * 4)) & 0xF) as usize;
        nmi_putc(HEX[nibble]);
    }
}

/// Print a u32 as decimal.
fn nmi_put_dec(mut val: u32) {
    if val == 0 {
        nmi_putc(b'0');
        return;
    }
    let mut buf = [0u8; 10];
    let mut i = 0;
    while val > 0 {
        buf[i] = b'0' + (val % 10) as u8;
        val /= 10;
        i += 1;
    }
    while i > 0 {
        i -= 1;
        nmi_putc(buf[i]);
    }
}

// ============================================================================
// Per-CPU state dump
// ============================================================================

/// Dump one CPU's state. Reads from CpuData (no locks needed — per-CPU data
/// is always readable, and at NMI time nobody else is modifying it).
fn dump_cpu(cpu: u32, elr: u64, spsr: u64) {
    nmi_puts(b"\n--- CPU ");
    nmi_put_dec(cpu);
    nmi_puts(b" ---\n");

    // Interrupted PC and PSTATE
    nmi_puts(b"  ELR=");
    nmi_put_hex64(elr);
    nmi_puts(b"  SPSR=");
    nmi_put_hex32(spsr as u32);
    nmi_putc(b'\n');

    // Per-CPU metadata from CpuData
    if let Some(data) = percpu::try_get_cpu_data(cpu as usize) {
        nmi_puts(b"  slot=");
        nmi_put_dec(data.current_slot.load(Ordering::Relaxed));
        nmi_puts(b"  last_syscall=");
        nmi_put_dec(data.last_syscall.load(Ordering::Relaxed));
        nmi_puts(b"  ticks=");
        nmi_put_dec(data.tick_count.load(Ordering::Relaxed) as u32);
        nmi_puts(b"  idle=");
        nmi_put_dec(data.in_idle.load(Ordering::Relaxed));
        nmi_puts(b"  need_resched=");
        nmi_put_dec(data.need_resched.load(Ordering::Relaxed));
        nmi_putc(b'\n');

        // Dump trap frame contents if pointer is valid
        let tf_ptr = data.trap_frame_ptr.load(Ordering::Relaxed);
        if tf_ptr != 0 && tf_ptr >= 0xFFFF_0000_0000_0000 {
            let tf = unsafe { &*(tf_ptr as *const [u64; 34]) };
            // x0-x7 (most interesting for syscall args)
            for i in 0..8 {
                nmi_puts(b"  x");
                nmi_put_dec(i as u32);
                nmi_putc(b'=');
                nmi_put_hex64(tf[i]);
                if i % 4 == 3 {
                    nmi_putc(b'\n');
                } else {
                    nmi_putc(b' ');
                }
            }
            // x29 (FP), x30 (LR), SP_EL0, ELR from trap frame
            nmi_puts(b"  x29=");
            nmi_put_hex64(tf[29]);
            nmi_puts(b" x30=");
            nmi_put_hex64(tf[30]);
            nmi_putc(b'\n');
            nmi_puts(b"  sp_el0=");
            nmi_put_hex64(tf[31]);
            nmi_puts(b" tf_elr=");
            nmi_put_hex64(tf[32]);
            nmi_putc(b'\n');
        }
    }
}

// ============================================================================
// Group 0 SGI (NMI to other CPUs)
// ============================================================================

/// Send Group 0 SGI 1 to all other CPUs.
/// Uses ICC_SGI0R_EL1 (not SGI1R) for Group 0 delivery.
fn send_nmi_sgi_all_others() {
    // ICC_SGI0R_EL1 = S3_0_C12_C11_7
    // IRM (bit 40) = 1 → send to all PEs except self
    // INTID (bits 27:24) = 1 → SGI 1
    let val: u64 = (1u64 << 40) | (1u64 << 24);
    unsafe {
        core::arch::asm!(
            "msr S3_0_C12_C11_7, {}",
            "isb",
            in(reg) val,
        );
    }
}

/// Acknowledge a Group 0 interrupt: read ICC_IAR0_EL1, write ICC_EOIR0_EL1.
fn ack_fiq() -> u32 {
    let irq: u64;
    unsafe {
        // ICC_IAR0_EL1 = S3_0_C12_C8_0
        core::arch::asm!("mrs {}, S3_0_C12_C8_0", out(reg) irq);
    }
    let irq = irq as u32;
    unsafe {
        // ICC_EOIR0_EL1 = S3_0_C12_C8_1
        core::arch::asm!("msr S3_0_C12_C8_1, {}", in(reg) irq as u64);
    }
    irq
}

// ============================================================================
// Main FIQ handler (called from assembly)
// ============================================================================

/// FIQ handler entry point, called from boot.S fiq_handler stub.
///
/// - `elr`: ELR_EL1 at time of FIQ (interrupted PC)
/// - `spsr`: SPSR_EL1 at time of FIQ (interrupted PSTATE)
///
/// This function never returns.
#[no_mangle]
pub extern "C" fn fiq_handler_rust(elr: u64, spsr: u64) -> ! {
    let cpu = percpu::cpu_id();

    // Acknowledge the FIQ interrupt
    let irq_id = ack_fiq();
    let _ = irq_id; // used for diagnostics if needed

    // Try to become the primary dump handler (CAS 0 → cpu+1)
    let is_primary = DUMP_IN_PROGRESS
        .compare_exchange(0, cpu + 1, Ordering::AcqRel, Ordering::Acquire)
        .is_ok();

    if is_primary {
        // === Primary CPU: orchestrate the dump ===

        // Clear WDT IRQ status on MT7988A so second-half reset timer restarts
        #[cfg(feature = "platform-mt7988a")]
        crate::platform::mt7988::wdt::clear_irq_status();

        // Banner
        nmi_puts(b"\n\n!!! WATCHDOG NMI - ALL-CPU STATE DUMP !!!\n");

        // Signal all other CPUs via Group 0 SGI 1
        let num_cpus = percpu::num_online_cpus();
        if num_cpus > 1 {
            send_nmi_sgi_all_others();
        }

        // Dump self first (CPU 0 goes first as primary is typically CPU 0)
        dump_cpu(cpu, elr, spsr);
        DUMP_DONE_MASK.fetch_or(1 << cpu, Ordering::Release);

        // Wait for all other online CPUs to finish dumping
        let online = num_cpus;
        let expected_mask = (1u32 << online) - 1;
        let mut timeout = 0u32;
        while DUMP_DONE_MASK.load(Ordering::Acquire) != expected_mask {
            timeout += 1;
            if timeout > 100_000_000 {
                nmi_puts(b"\n[timeout waiting for all CPUs]\n");
                break;
            }
            core::hint::spin_loop();
        }

        // Footer
        nmi_puts(b"\n!!! END WATCHDOG NMI DUMP !!!\n");

        // Reset or halt
        #[cfg(feature = "platform-mt7988a")]
        {
            nmi_puts(b"Resetting...\n");
            // Give UART time to flush
            for _ in 0..1_000_000 {
                core::hint::spin_loop();
            }
            crate::platform::mt7988::wdt::software_reset();
        }

        #[cfg(feature = "platform-qemu-virt")]
        {
            nmi_puts(b"Halting (QEMU).\n");
            loop {
                unsafe { core::arch::asm!("wfe"); }
            }
        }
    } else {
        // === Secondary CPU: dump self and halt ===

        // Wait for our turn: CPUs dump in order 0, 1, 2, 3
        // Wait until all CPUs < us are done
        if cpu > 0 {
            let wait_mask = (1u32 << cpu) - 1;
            let mut timeout = 0u32;
            while (DUMP_DONE_MASK.load(Ordering::Acquire) & wait_mask) != wait_mask {
                timeout += 1;
                if timeout > 50_000_000 {
                    break; // Don't wait forever, just dump
                }
                core::hint::spin_loop();
            }
        }

        dump_cpu(cpu, elr, spsr);
        DUMP_DONE_MASK.fetch_or(1 << cpu, Ordering::Release);

        // Halt
        loop {
            unsafe { core::arch::asm!("wfe"); }
        }
    }
}

/// Soft NMI entry point for QEMU software watchdog.
/// Called from timer tick context when watchdog timeout fires.
/// Reads ELR/SPSR from system registers (these reflect the timer IRQ return
/// address, which is close enough for diagnostics).
#[allow(dead_code)]
pub fn soft_nmi() -> ! {
    let elr: u64;
    let spsr: u64;
    unsafe {
        core::arch::asm!("mrs {}, elr_el1", out(reg) elr);
        core::arch::asm!("mrs {}, spsr_el1", out(reg) spsr);
    }
    fiq_handler_rust(elr, spsr);
}
