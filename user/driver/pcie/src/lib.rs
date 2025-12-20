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

use userlib::syscall;

/// Delay in milliseconds (approximate)
/// Uses yield to let scheduler run and provide some delay
pub fn delay_ms(ms: u32) {
    // Each yield gives roughly 10ms of timer tick
    let yields = if ms < 10 { 1 } else { ms / 10 };
    for _ in 0..yields {
        syscall::yield_now();
    }
}

/// Simple MMIO region wrapper
pub struct MmioRegion {
    pub fd: i32,
    pub base: u64,
    pub size: u64,
}

impl MmioRegion {
    /// Open an MMIO region at the given physical address
    pub fn open(phys_addr: u64, size: u64) -> Option<Self> {
        let mut url_buf = [0u8; 64];
        let url_len = format_mmio_url(&mut url_buf, phys_addr, size);

        let url_str = core::str::from_utf8(&url_buf[..url_len]).ok()?;
        let fd = syscall::scheme_open(url_str, 0);
        if fd < 0 {
            return None;
        }

        // Read virtual address from kernel
        let mut virt_buf = [0u8; 8];
        let n = syscall::read(fd as u32, &mut virt_buf);
        if n != 8 {
            syscall::close(fd as u32);
            return None;
        }

        let base = u64::from_le_bytes(virt_buf);
        Some(Self { fd, base, size })
    }

    /// Get the virtual base address (for debugging)
    pub fn virt_base(&self) -> u64 {
        self.base
    }

    /// Read 32-bit value at offset
    #[inline]
    pub fn read32(&self, offset: usize) -> u32 {
        unsafe {
            core::ptr::read_volatile((self.base + offset as u64) as *const u32)
        }
    }

    /// Write 32-bit value at offset
    #[inline]
    pub fn write32(&self, offset: usize, value: u32) {
        unsafe {
            core::ptr::write_volatile((self.base + offset as u64) as *mut u32, value)
        }
    }

    /// Read 16-bit value at offset
    #[inline]
    pub fn read16(&self, offset: usize) -> u16 {
        unsafe {
            core::ptr::read_volatile((self.base + offset as u64) as *const u16)
        }
    }
}

impl Drop for MmioRegion {
    fn drop(&mut self) {
        if self.fd >= 0 {
            syscall::close(self.fd as u32);
        }
    }
}

/// Format MMIO URL: "mmio:ADDR/SIZE"
fn format_mmio_url(buf: &mut [u8], addr: u64, size: u64) -> usize {
    let prefix = b"mmio:";
    let mut pos = 0;

    for &b in prefix {
        buf[pos] = b;
        pos += 1;
    }

    pos += format_hex(&mut buf[pos..], addr);
    buf[pos] = b'/';
    pos += 1;
    pos += format_hex(&mut buf[pos..], size);

    pos
}

/// Format u64 as hex string, return length
fn format_hex(buf: &mut [u8], val: u64) -> usize {
    const HEX: &[u8] = b"0123456789abcdef";

    if val == 0 {
        buf[0] = b'0';
        return 1;
    }

    let mut v = val;
    let mut digits = 0;
    while v > 0 {
        digits += 1;
        v >>= 4;
    }

    let mut v = val;
    for i in (0..digits).rev() {
        buf[i] = HEX[(v & 0xF) as usize];
        v >>= 4;
    }

    digits
}
