//! AArch64 Architecture Support
//!
//! This module contains architecture-specific code for ARMv8-A (AArch64):
//! - Exception vectors and boot sequence (boot.S)
//! - MMU and page table management
//! - TLB (Translation Lookaside Buffer) management
//! - Synchronization primitives (barriers, IrqGuard)
//! - SMP and per-CPU state
//! - MMIO abstractions with proper barriers

use core::arch::global_asm;

pub mod mmu;
pub mod tlb;
pub mod sync;
pub mod smp;
pub mod mmio;

// Include the assembly startup code
global_asm!(include_str!("boot.S"));

// Re-exports available if needed:
// pub use sync::IrqGuard;
// pub use mmio::MmioRegion;
