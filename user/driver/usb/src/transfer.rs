//! USB Transfer Helpers
//!
//! This module provides USB transfer-related utilities that can be shared
//! across the USB stack. The actual transfer execution remains in usbd
//! due to tight coupling with hardware state.

use crate::trb::{Trb, trb_type};

// =============================================================================
// Timing Constants (microseconds unless noted)
// =============================================================================

/// Delay after port reset before addressing device (microseconds)
pub const DELAY_POST_RESET_US: u32 = 50_000;

/// Delay after ADDRESS_DEVICE command (microseconds)
pub const DELAY_POST_ADDRESS_US: u32 = 100_000;

/// Delay between poll iterations (microseconds)
pub const DELAY_POLL_INTERVAL_US: u32 = 1_000;

/// Delay before control transfer for devices behind hubs (microseconds)
pub const DELAY_DOWNSTREAM_DEVICE_US: u32 = 100_000;

/// Delay before control transfer for direct devices (microseconds)
pub const DELAY_DIRECT_DEVICE_US: u32 = 50_000;

/// Post-configuration delay (microseconds)
pub const DELAY_POST_CONFIG_US: u32 = 50_000;

/// Short inter-command delay (microseconds)
pub const DELAY_INTER_CMD_US: u32 = 10_000;

/// Maximum poll iterations for direct devices
pub const POLL_MAX_DIRECT: usize = 100;

/// Maximum poll iterations for devices behind hubs
pub const POLL_MAX_DOWNSTREAM: usize = 300;

/// Maximum poll iterations for descriptor transfers
pub const POLL_MAX_DESCRIPTOR: usize = 3000;

// =============================================================================
// xHCI Register Bit Constants
// =============================================================================

/// ERDP Event Handler Busy bit (bit 3)
pub const ERDP_EHB: u64 = 1 << 3;

/// USBSTS Event Interrupt bit (bit 3)
pub const USBSTS_EINT: u32 = 1 << 3;

/// USBSTS Port Change Detect bit (bit 4)
pub const USBSTS_PCD: u32 = 1 << 4;

/// USB Setup Packet (8 bytes)
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub struct SetupPacket {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl SetupPacket {
    /// Create a new setup packet
    pub const fn new(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Self {
        Self { request_type, request, value, index, length }
    }

    /// Convert to the 64-bit format used in Setup TRB param field
    pub fn to_trb_param(&self) -> u64 {
        (self.request_type as u64) |
        ((self.request as u64) << 8) |
        ((self.value as u64) << 16) |
        ((self.index as u64) << 32) |
        ((self.length as u64) << 48)
    }

    /// Check if this is a device-to-host (IN) transfer
    pub fn is_in(&self) -> bool {
        (self.request_type & 0x80) != 0
    }

    /// Check if this transfer has a data stage
    pub fn has_data(&self) -> bool {
        self.length > 0
    }
}

/// Transfer direction
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Direction {
    Out = 0,  // Host to device
    In = 1,   // Device to host
}

/// Control transfer stage
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControlStage {
    Setup,
    Data,
    Status,
}

/// Transfer Request Type (TRT) for Setup TRB
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TransferType {
    NoData = 0,
    OutData = 2,
    InData = 3,
}

impl TransferType {
    /// Determine transfer type from setup packet
    pub fn from_setup(setup: &SetupPacket) -> Self {
        if !setup.has_data() {
            TransferType::NoData
        } else if setup.is_in() {
            TransferType::InData
        } else {
            TransferType::OutData
        }
    }
}

/// Build a Setup Stage TRB
///
/// # Arguments
/// * `setup` - The setup packet
/// * `cycle` - Current cycle bit
pub fn build_setup_trb(setup: &SetupPacket, cycle: u32) -> Trb {
    let trt = TransferType::from_setup(setup) as u32;
    Trb {
        param: setup.to_trb_param(),
        status: 8,  // Setup packet is always 8 bytes
        control: (trb_type::SETUP << 10) | (trt << 16) | (1 << 6) | cycle,  // IDT=1
    }
}

/// Build a Data Stage TRB
///
/// # Arguments
/// * `data_phys` - Physical address of data buffer
/// * `length` - Transfer length
/// * `direction` - Transfer direction
/// * `cycle` - Current cycle bit
pub fn build_data_trb(data_phys: u64, length: u16, direction: Direction, cycle: u32) -> Trb {
    let dir_bit = if direction == Direction::In { 1 << 16 } else { 0 };
    Trb {
        param: data_phys,
        status: length as u32,
        control: (trb_type::DATA << 10) | dir_bit | cycle,
    }
}

/// Build a Status Stage TRB
///
/// # Arguments
/// * `data_direction` - Direction of the data stage (None if no data stage)
/// * `cycle` - Current cycle bit
pub fn build_status_trb(data_direction: Option<Direction>, cycle: u32) -> Trb {
    // Status direction is opposite of data direction, or IN if no data
    let dir_bit = match data_direction {
        Some(Direction::In) => 0,       // Data was IN, status is OUT
        Some(Direction::Out) => 1 << 16, // Data was OUT, status is IN
        None => 1 << 16,                // No data, status is IN
    };
    Trb {
        param: 0,
        status: 0,
        control: (trb_type::STATUS << 10) | dir_bit | (1 << 5) | cycle,  // IOC=1
    }
}

/// Build a Link TRB for ring wrap-around
///
/// # Arguments
/// * `ring_phys` - Physical address of ring start
/// * `toggle_cycle` - Whether to toggle cycle bit on wrap
/// * `cycle` - Current cycle bit
pub fn build_link_trb(ring_phys: u64, toggle_cycle: bool, cycle: u32) -> Trb {
    let tc_bit = if toggle_cycle { 1 << 1 } else { 0 };
    Trb {
        param: ring_phys,
        status: 0,
        control: (trb_type::LINK << 10) | tc_bit | cycle,
    }
}

/// Build a Normal TRB for bulk transfers
///
/// # Arguments
/// * `data_phys` - Physical address of data buffer
/// * `length` - Transfer length
/// * `ioc` - Interrupt on completion
/// * `cycle` - Current cycle bit
pub fn build_normal_trb(data_phys: u64, length: u32, ioc: bool, cycle: u32) -> Trb {
    let ioc_bit = if ioc { 1 << 5 } else { 0 };
    Trb {
        param: data_phys,
        status: length,
        control: (trb_type::NORMAL << 10) | ioc_bit | cycle,
    }
}

// =============================================================================
// Cache Maintenance Operations (ARM64)
// =============================================================================
//
// DMA buffers use cacheable memory on MT7988A. Explicit cache maintenance is
// required for coherency between CPU and DMA devices:
//
// - DC CVAC (Clean by VA to PoC): Write cache line to memory (before DMA read)
// - DC CIVAC (Clean & Invalidate): Write + invalidate (after DMA write)
// - DSB SY: Ensure memory operations complete
//
// These functions abstract the ARM-specific cache ops from driver code.

/// Memory barrier (data synchronization barrier)
///
/// Ensures all memory accesses complete before continuing.
#[inline]
pub fn dsb() {
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Instruction synchronization barrier
///
/// Flushes the pipeline and ensures all previous instructions complete.
#[inline]
pub fn isb() {
    unsafe {
        core::arch::asm!("isb", options(nostack, preserves_flags));
    }
}

/// Flush (clean) a cache line to memory
///
/// Use before DMA reads (hardware reading data that CPU wrote).
#[inline]
pub fn flush_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc cvac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Invalidate a cache line
///
/// Use before CPU reads (CPU reading data that hardware wrote via DMA).
#[inline]
pub fn invalidate_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc civac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Flush a buffer to memory (for DMA reads by hardware)
#[inline]
pub fn flush_buffer(addr: u64, size: usize) {
    const CACHE_LINE: usize = 64;
    let start = addr & !(CACHE_LINE as u64 - 1);
    let end = (addr + size as u64 + CACHE_LINE as u64 - 1) & !(CACHE_LINE as u64 - 1);
    let mut a = start;
    while a < end {
        flush_cache_line(a);
        a += CACHE_LINE as u64;
    }
    dsb();
}

/// Invalidate a buffer from cache (for CPU reads after DMA writes)
#[inline]
pub fn invalidate_buffer(addr: u64, size: usize) {
    const CACHE_LINE: usize = 64;
    let start = addr & !(CACHE_LINE as u64 - 1);
    let end = (addr + size as u64 + CACHE_LINE as u64 - 1) & !(CACHE_LINE as u64 - 1);
    dsb();
    let mut a = start;
    while a < end {
        invalidate_cache_line(a);
        a += CACHE_LINE as u64;
    }
    dsb();
    isb();
}

/// Flush a TRB to memory
#[inline]
pub unsafe fn flush_trb(trb: *const Trb) {
    flush_cache_line(trb as u64);
}

/// Flush a range of TRBs and issue memory barrier
pub unsafe fn flush_trb_range(base: *const Trb, start: usize, end: usize) {
    for i in start..=end {
        flush_trb(base.add(i));
    }
    dsb();
}
