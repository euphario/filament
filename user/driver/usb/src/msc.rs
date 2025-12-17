//! USB Mass Storage Class (BOT - Bulk-Only Transport) and SCSI

use crate::trb::Trb;
use crate::usb::DeviceContext;

// =============================================================================
// USB Mass Storage Class (BOT - Bulk-Only Transport)
// =============================================================================

/// Mass Storage Class constants
pub mod msc {
    pub const CLASS: u8 = 0x08;            // Mass Storage Class
    pub const SUBCLASS_SCSI: u8 = 0x06;    // SCSI transparent command set
    pub const PROTOCOL_BOT: u8 = 0x50;     // Bulk-Only Transport

    // Class-specific requests
    pub const GET_MAX_LUN: u8 = 0xFE;
    pub const BULK_ONLY_RESET: u8 = 0xFF;

    // CBW/CSW signatures
    pub const CBW_SIGNATURE: u32 = 0x43425355;  // 'USBC'
    pub const CSW_SIGNATURE: u32 = 0x53425355;  // 'USBS'

    // CSW status
    pub const CSW_STATUS_PASSED: u8 = 0;
    pub const CSW_STATUS_FAILED: u8 = 1;
    pub const CSW_STATUS_PHASE_ERROR: u8 = 2;
}

/// Command Block Wrapper (CBW) - 31 bytes
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Cbw {
    pub signature: u32,       // 'USBC' (0x43425355)
    pub tag: u32,             // Command tag (echoed in CSW)
    pub data_length: u32,     // Data transfer length
    pub flags: u8,            // Bit 7: Direction (0=OUT, 1=IN)
    pub lun: u8,              // Logical Unit Number
    pub cb_length: u8,        // Length of command block (1-16)
    pub cb: [u8; 16],         // Command block (SCSI command)
}

impl Cbw {
    pub fn new(tag: u32, data_length: u32, direction_in: bool, lun: u8, command: &[u8]) -> Self {
        let mut cb = [0u8; 16];
        let len = command.len().min(16);
        cb[..len].copy_from_slice(&command[..len]);

        Self {
            signature: msc::CBW_SIGNATURE,
            tag,
            data_length,
            flags: if direction_in { 0x80 } else { 0x00 },
            lun,
            cb_length: len as u8,
            cb,
        }
    }
}

/// Command Status Wrapper (CSW) - 13 bytes
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct Csw {
    pub signature: u32,       // 'USBS' (0x53425355)
    pub tag: u32,             // Command tag (from CBW)
    pub data_residue: u32,    // Difference between expected and actual data
    pub status: u8,           // Command status
}

// =============================================================================
// SCSI Commands
// =============================================================================

pub mod scsi {
    pub const TEST_UNIT_READY: u8 = 0x00;
    pub const REQUEST_SENSE: u8 = 0x03;
    pub const INQUIRY: u8 = 0x12;
    pub const READ_CAPACITY_10: u8 = 0x25;
    pub const READ_10: u8 = 0x28;
    pub const WRITE_10: u8 = 0x2A;
}

/// SCSI Inquiry Response (36 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct InquiryResponse {
    pub peripheral: u8,       // Peripheral device type & qualifier
    pub removable: u8,        // RMB (bit 7)
    pub version: u8,          // SCSI version
    pub response_format: u8,  // Response data format
    pub additional_length: u8,// Additional length
    pub flags1: u8,
    pub flags2: u8,
    pub flags3: u8,
    pub vendor: [u8; 8],      // Vendor identification
    pub product: [u8; 16],    // Product identification
    pub revision: [u8; 4],    // Product revision
}

/// SCSI Read Capacity (10) Response (8 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct ReadCapacity10Response {
    pub last_lba: u32,        // Last Logical Block Address (big-endian)
    pub block_size: u32,      // Block size in bytes (big-endian)
}

// =============================================================================
// Bulk Transfer Context - for cleaner SCSI command handling
// =============================================================================

/// Context for bulk transfers (holds all the state needed for CBW/CSW/Data)
pub struct BulkContext {
    pub slot_id: u32,
    pub bulk_in_dci: u32,
    pub bulk_out_dci: u32,
    pub bulk_in_ring: *mut Trb,
    pub bulk_in_ring_phys: u64,
    pub bulk_out_ring: *mut Trb,
    pub _bulk_out_ring_phys: u64,
    pub data_buf: *mut u8,
    pub data_phys: u64,
    pub device_ctx: *mut DeviceContext,
    // Ring enqueue positions (tracks where we are in the ring)
    pub out_enqueue: usize,
    pub in_enqueue: usize,
    // CBW tag counter
    pub tag: u32,
}

/// Result of a bulk transfer
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TransferResult {
    Success,
    ShortPacket,
    Timeout,
    Error(u32),  // Completion code
}

// Buffer offsets within the 4KB data buffer
pub const CBW_OFFSET: usize = 0;       // CBW at offset 0
pub const CSW_OFFSET: usize = 64;      // CSW at offset 64
pub const DATA_OFFSET: usize = 512;    // Data at offset 512 (aligned for DMA)

impl BulkContext {
    /// Create a new bulk context
    pub fn new(
        slot_id: u32,
        bulk_in_dci: u32,
        bulk_out_dci: u32,
        bulk_in_ring: *mut Trb,
        bulk_in_ring_phys: u64,
        bulk_out_ring: *mut Trb,
        bulk_out_ring_phys: u64,
        data_buf: *mut u8,
        data_phys: u64,
        device_ctx: *mut DeviceContext,
    ) -> Self {
        Self {
            slot_id,
            bulk_in_dci,
            bulk_out_dci,
            bulk_in_ring,
            bulk_in_ring_phys,
            bulk_out_ring,
            _bulk_out_ring_phys: bulk_out_ring_phys,
            data_buf,
            data_phys,
            device_ctx,
            out_enqueue: 0,
            in_enqueue: 0,
            tag: 1,
        }
    }

    /// Get current DCS (Dequeue Cycle State) for bulk IN endpoint from device context
    pub fn get_bulk_in_dcs(&self) -> u32 {
        unsafe {
            let dev_ctx = &*self.device_ctx;
            let bulk_in_idx = (self.bulk_in_dci - 1) as usize;
            let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];

            // Invalidate cache before reading
            let ctx_addr = bulk_in_ctx as *const _ as u64;
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!("dc civac, {}", in(reg) ctx_addr, options(nostack));
            core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));

            (core::ptr::read_volatile(&bulk_in_ctx.dw2) & 1) as u32
        }
    }

    /// Flush a buffer to main memory (for DMA)
    pub fn flush_buffer(&self, addr: u64, _size: usize) {
        unsafe {
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!("dc cvac, {}", in(reg) addr, options(nostack));
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
    }

    /// Invalidate a buffer from cache (before reading DMA data)
    pub fn invalidate_buffer(&self, addr: u64, _size: usize) {
        unsafe {
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!("dc civac, {}", in(reg) addr, options(nostack));
            core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));
        }
    }

    /// Get next tag and increment
    pub fn next_tag(&mut self) -> u32 {
        let t = self.tag;
        self.tag += 1;
        t
    }
}
