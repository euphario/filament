//! PCIe Host Controller Library
//!
//! Userspace PCIe driver with layered abstraction:
//!
//! ```text
//! ┌─────────────┐
//! │   pcied     │  Daemon (main.rs)
//! ├─────────────┤
//! │   board/    │  Board-specific (BPI-R4 slot layout)
//! ├─────────────┤
//! │    soc/     │  SoC-specific (MT7988A clocks/resets)
//! ├─────────────┤
//! │ controller  │  MediaTek Gen3 PCIe (TLP config access)
//! ├─────────────┤
//! │  config     │  Standard PCIe config space
//! ├─────────────┤
//! │   regs      │  MediaTek register definitions
//! └─────────────┘
//! ```
//!
//! ## Layer Responsibilities
//!
//! - **board**: Ties together SoC + slot layout + power control
//! - **soc**: Clock gates, PHY reset, port configuration
//! - **controller**: MAC initialization, link training, device enumeration
//! - **config**: PCIe config space read/write (TLP-based for MediaTek)
//! - **regs**: Register offsets and bit definitions
//! - **consts**: Standard PCIe vendor/device IDs

#![no_std]

pub mod soc;
pub mod board;
pub mod consts;
pub mod regs;
pub mod config;
pub mod controller;
pub mod client;

// Re-export key types
pub use soc::{SocPcie, SocError, PciePortConfig};
pub use soc::Mt7988aSoc;
pub use board::{Board, BoardError, SlotInfo, PcieInit};
pub use board::BpiR4;
pub use config::{PcieBdf, PcieDeviceId, PcieConfigSpace};
pub use controller::{PcieController, PcieError, PcieDevice, PcieDeviceList};
pub use client::{PcieClient, PcieDeviceInfo, DeviceList};

// Re-export MmioRegion from userlib for use by submodules
pub use userlib::MmioRegion;

// Re-export delay_ms from userlib (timer-based, more accurate)
pub use userlib::delay_ms;
