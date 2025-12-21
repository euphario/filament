//! MT7996 WiFi Driver Library
//!
//! Userspace driver for MediaTek MT7996 WiFi 7 chipset.
//!
//! ## Initialization
//!
//! The MT7996 is a PCIe device that requires:
//! 1. BAR0 mapping for register access
//! 2. Address remapping for large register space
//! 3. Firmware loading (WM, WA, DSP)
//! 4. DMA ring setup for TX/RX
//!
//! This library implements device detection, register access,
//! and firmware loading infrastructure.

#![no_std]

pub mod regs;
pub mod device;
pub mod firmware;
pub mod mcu;
pub mod dma;

pub use regs::ChipVariant;
pub use device::{Mt7996Device, Mt7996Error};
pub use firmware::{Firmware, FirmwareError};
pub use dma::{Wfdma, PatchHeader, PatchSection};
