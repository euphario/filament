//! Architecture re-exports for kernel/ modules.
//!
//! All kernel code should import arch-specific items through this module
//! rather than directly from `crate::arch::aarch64::*`. Porting to a new
//! architecture requires updating only this file.

#[cfg(target_arch = "aarch64")]
pub use crate::arch::aarch64::mmu;

#[cfg(target_arch = "aarch64")]
pub use crate::arch::aarch64::tlb;

#[cfg(target_arch = "aarch64")]
pub use crate::arch::aarch64::mmio;

#[cfg(target_arch = "aarch64")]
pub use crate::arch::aarch64::sync;

#[cfg(target_arch = "aarch64")]
pub use crate::arch::aarch64::hal;
