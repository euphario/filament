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
    // SCSI operation codes
    pub const TEST_UNIT_READY: u8 = 0x00;
    pub const REQUEST_SENSE: u8 = 0x03;
    pub const INQUIRY: u8 = 0x12;
    pub const READ_CAPACITY_10: u8 = 0x25;
    pub const READ_10: u8 = 0x28;
    pub const WRITE_10: u8 = 0x2A;

    /// Build a TEST UNIT READY CDB (6 bytes)
    /// Returns (cdb, data_length)
    pub fn build_test_unit_ready() -> ([u8; 10], u32) {
        ([TEST_UNIT_READY, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)
    }

    /// Build an INQUIRY CDB (6 bytes, but we use 10 for consistency)
    /// allocation_length: typically 36 bytes for standard inquiry
    /// Returns (cdb, data_length)
    pub fn build_inquiry(allocation_length: u8) -> ([u8; 10], u32) {
        ([INQUIRY, 0, 0, 0, allocation_length, 0, 0, 0, 0, 0], allocation_length as u32)
    }

    /// Build a READ CAPACITY (10) CDB
    /// Returns (cdb, data_length) - response is 8 bytes
    pub fn build_read_capacity_10() -> ([u8; 10], u32) {
        ([READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0], 8)
    }

    /// Build a READ (10) CDB
    /// lba: starting logical block address
    /// count: number of blocks to read
    /// block_size: size of each block (typically 512)
    /// Returns (cdb, data_length)
    pub fn build_read_10(lba: u32, count: u16, block_size: u32) -> ([u8; 10], u32) {
        let cdb = [
            READ_10,
            0,
            ((lba >> 24) & 0xFF) as u8,
            ((lba >> 16) & 0xFF) as u8,
            ((lba >> 8) & 0xFF) as u8,
            (lba & 0xFF) as u8,
            0,
            ((count >> 8) & 0xFF) as u8,
            (count & 0xFF) as u8,
            0,
        ];
        (cdb, count as u32 * block_size)
    }

    /// Build a WRITE (10) CDB
    /// lba: starting logical block address
    /// count: number of blocks to write
    /// block_size: size of each block (typically 512)
    /// Returns (cdb, data_length)
    pub fn build_write_10(lba: u32, count: u16, block_size: u32) -> ([u8; 10], u32) {
        let cdb = [
            WRITE_10,
            0,
            ((lba >> 24) & 0xFF) as u8,
            ((lba >> 16) & 0xFF) as u8,
            ((lba >> 8) & 0xFF) as u8,
            (lba & 0xFF) as u8,
            0,
            ((count >> 8) & 0xFF) as u8,
            (count & 0xFF) as u8,
            0,
        ];
        (cdb, count as u32 * block_size)
    }

    /// Build a REQUEST SENSE CDB
    /// allocation_length: typically 18 bytes for standard sense data
    /// Returns (cdb, data_length)
    pub fn build_request_sense(allocation_length: u8) -> ([u8; 10], u32) {
        ([REQUEST_SENSE, 0, 0, 0, allocation_length, 0, 0, 0, 0, 0], allocation_length as u32)
    }

    /// Parse READ CAPACITY (10) response (8 bytes, big-endian)
    /// Returns (last_lba, block_size)
    pub fn parse_read_capacity_10(data: &[u8]) -> Option<(u32, u32)> {
        if data.len() < 8 {
            return None;
        }
        let last_lba = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let block_size = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);
        Some((last_lba, block_size))
    }
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

impl InquiryResponse {
    /// Parse inquiry data from a byte slice
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 36 {
            return None;
        }
        let mut resp = Self::default();
        resp.peripheral = data[0];
        resp.removable = data[1];
        resp.version = data[2];
        resp.response_format = data[3];
        resp.additional_length = data[4];
        resp.flags1 = data[5];
        resp.flags2 = data[6];
        resp.flags3 = data[7];
        resp.vendor.copy_from_slice(&data[8..16]);
        resp.product.copy_from_slice(&data[16..32]);
        resp.revision.copy_from_slice(&data[32..36]);
        Some(resp)
    }

    /// Get device type (bits 0-4 of peripheral)
    pub fn device_type(&self) -> u8 {
        self.peripheral & 0x1F
    }

    /// Check if device is removable
    pub fn is_removable(&self) -> bool {
        (self.removable & 0x80) != 0
    }

    /// Get vendor string (trimmed)
    pub fn vendor_str(&self) -> &[u8] {
        trim_ascii(&self.vendor)
    }

    /// Get product string (trimmed)
    pub fn product_str(&self) -> &[u8] {
        trim_ascii(&self.product)
    }

    /// Get revision string (trimmed)
    pub fn revision_str(&self) -> &[u8] {
        trim_ascii(&self.revision)
    }
}

/// SCSI Read Capacity (10) Response (8 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct ReadCapacity10Response {
    pub last_lba: u32,        // Last Logical Block Address (big-endian)
    pub block_size: u32,      // Block size in bytes (big-endian)
}

/// SCSI Sense Data (fixed format, 18 bytes minimum)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct SenseData {
    pub response_code: u8,    // 0x70 for current errors, 0x71 for deferred
    pub segment_number: u8,
    pub sense_key: u8,        // Bits 0-3
    pub information: [u8; 4],
    pub additional_length: u8,
    pub command_specific: [u8; 4],
    pub asc: u8,              // Additional Sense Code
    pub ascq: u8,             // Additional Sense Code Qualifier
    pub fruc: u8,             // Field Replaceable Unit Code
    pub sense_key_specific: [u8; 3],
}

impl SenseData {
    /// Parse sense data from a byte slice
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 18 {
            return None;
        }
        Some(Self {
            response_code: data[0],
            segment_number: data[1],
            sense_key: data[2],
            information: [data[3], data[4], data[5], data[6]],
            additional_length: data[7],
            command_specific: [data[8], data[9], data[10], data[11]],
            asc: data[12],
            ascq: data[13],
            fruc: data[14],
            sense_key_specific: [data[15], data[16], data[17]],
        })
    }

    /// Get sense key (bits 0-3 of sense_key field)
    pub fn key(&self) -> u8 {
        self.sense_key & 0x0F
    }

    /// Check if this is a valid sense response
    pub fn is_valid(&self) -> bool {
        (self.response_code & 0x70) == 0x70
    }
}

/// SCSI sense key values
pub mod sense_key {
    pub const NO_SENSE: u8 = 0x00;
    pub const RECOVERED_ERROR: u8 = 0x01;
    pub const NOT_READY: u8 = 0x02;
    pub const MEDIUM_ERROR: u8 = 0x03;
    pub const HARDWARE_ERROR: u8 = 0x04;
    pub const ILLEGAL_REQUEST: u8 = 0x05;
    pub const UNIT_ATTENTION: u8 = 0x06;
    pub const DATA_PROTECT: u8 = 0x07;
    pub const BLANK_CHECK: u8 = 0x08;
    pub const ABORTED_COMMAND: u8 = 0x0B;
}

/// Trim trailing spaces from ASCII string
fn trim_ascii(s: &[u8]) -> &[u8] {
    let mut end = s.len();
    while end > 0 && (s[end - 1] == b' ' || s[end - 1] == 0) {
        end -= 1;
    }
    &s[..end]
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
        use crate::transfer::{invalidate_cache_line, dsb, isb};

        unsafe {
            let dev_ctx = &*self.device_ctx;
            let bulk_in_idx = (self.bulk_in_dci - 1) as usize;
            let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];

            // Invalidate cache before reading
            let ctx_addr = bulk_in_ctx as *const _ as u64;
            dsb();
            invalidate_cache_line(ctx_addr);
            dsb();
            isb();

            (core::ptr::read_volatile(&bulk_in_ctx.dw2) & 1) as u32
        }
    }

    /// Flush a buffer to main memory (for DMA)
    pub fn flush_buffer(&self, addr: u64, size: usize) {
        crate::transfer::flush_buffer(addr, size);
    }

    /// Invalidate a buffer from cache (before reading DMA data)
    pub fn invalidate_buffer(&self, addr: u64, size: usize) {
        crate::transfer::invalidate_buffer(addr, size);
    }

    /// Get next tag and increment
    pub fn next_tag(&mut self) -> u32 {
        let t = self.tag;
        self.tag += 1;
        t
    }
}
