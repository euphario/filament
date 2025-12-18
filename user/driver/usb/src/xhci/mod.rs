//! Pure xHCI (eXtensible Host Controller Interface) implementation
//!
//! This module contains ONLY standard xHCI functionality as defined in the
//! xHCI specification. NO vendor-specific code belongs here.
//!
//! If you find yourself adding MediaTek/Synopsys/Intel specific code here,
//! STOP - it belongs in the `soc` or `phy` modules instead.
//!
//! ## What belongs here:
//! - xHCI register access (capability, operational, runtime, doorbell)
//! - TRB (Transfer Request Block) structures and builders
//! - Ring management (command, transfer, event rings)
//! - Event handling
//! - Standard xHCI commands (enable slot, address device, etc.)
//!
//! ## What does NOT belong here:
//! - PHY initialization
//! - Clock/power management
//! - Vendor-specific quirks
//! - SoC wrapper registers (IPPC, GCTL, etc.)

#![allow(dead_code)]

mod regs;
mod controller;

pub use regs::*;
pub use controller::*;

// Re-export TRB and Ring from parent (they're already pure xHCI)
pub use crate::trb::{Trb, trb_type, trb_cc};
pub use crate::ring::{Ring, EventRing, ErstEntry, RING_SIZE};
