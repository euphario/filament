//! AArch64 Architecture Support
//!
//! This module contains architecture-specific code for ARMv8-A (AArch64):
//! - Exception vectors and boot sequence (boot.S)
//! - MMU and page table management
//! - TLB (Translation Lookaside Buffer) management
//! - Synchronization primitives (barriers, IrqGuard)
//! - SMP and per-CPU state
//! - MMIO abstractions with proper barriers
//! - HAL implementations (Cpu, ContextSwitch)

use core::arch::global_asm;

pub mod mmu;
pub mod tlb;
pub mod sync;
pub mod smp;
pub mod mmio;
pub mod hal;

// Include the assembly startup code
global_asm!(include_str!("boot.S"));

// HAL types are used internally by arch module
// Re-exports available if needed by other modules:
// pub use hal::{Aarch64Cpu, Aarch64Context, Aarch64ContextSwitch, cpu};
