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
#![allow(dead_code)]  // Driver functions reserved for future use
#![allow(unused_unsafe)]  // Some unsafe blocks are for future-proofing

// Register and hardware definitions - direct translations from Linux headers
pub mod regs;           // from mt7996/regs.h
pub mod dma_defs;       // from mt76/dma.h
pub mod connac3_mac;    // from mt76/mt76_connac3_mac.h
pub mod mt76_defs;      // from mt76/mt76.h
pub mod mt7996_defs;    // from mt7996/mt7996.h
pub mod mcu_defs;       // from mt7996/mcu.h

// Driver implementation modules
pub mod device;
pub mod firmware;
pub mod mcu;
pub mod dma;

// Re-exports for common types
pub use regs::ChipVariant;
pub use device::{Mt7996Device, Mt7996Error};
pub use firmware::{Firmware, FirmwareError};
pub use dma::{Wfdma, PatchHeader, PatchSection};

// Re-export hardware definitions for convenience
pub use dma_defs::Mt76Desc;
pub use mt7996_defs::Mt7996TxqId;
