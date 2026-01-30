//! Example Filter Layer Implementations
//!
//! Shows how layers process descriptors without copying data.

use super::{Descriptor, FilterLayer, FilterResult, desc_flags};

// =============================================================================
// Layer indices (convention)
// =============================================================================

/// Layer 0: Hardware/DMA (xHCI TRBs)
pub const LAYER_XHCI: usize = 0;
/// Layer 1: Protocol (MSC/SCSI)
pub const LAYER_MSC: usize = 1;
/// Layer 2: Block device
pub const LAYER_BLOCK: usize = 2;
/// Layer 3: Filesystem
pub const LAYER_FS: usize = 3;

// =============================================================================
// MSC Layer - SCSI over USB Mass Storage
// =============================================================================

/// MSC layer metadata stored in descriptor.layer_data[LAYER_MSC]
///
/// Packed to exactly 64 bits for transmute to/from u64.
/// Layout: [lba:32][block_count:16][scsi_opcode:8][_pad:8]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MscLayerData {
    /// Logical block address (first for alignment)
    pub lba: u32,
    /// Number of blocks
    pub block_count: u16,
    /// SCSI command (e.g., READ_10, WRITE_10)
    pub scsi_opcode: u8,
    /// Reserved
    pub _pad: u8,
}

impl MscLayerData {
    pub fn to_u64(self) -> u64 {
        // Pack manually to avoid transmute alignment issues
        (self.lba as u64)
            | ((self.block_count as u64) << 32)
            | ((self.scsi_opcode as u64) << 48)
            | ((self._pad as u64) << 56)
    }

    pub fn from_u64(val: u64) -> Self {
        Self {
            lba: val as u32,
            block_count: (val >> 32) as u16,
            scsi_opcode: (val >> 48) as u8,
            _pad: (val >> 56) as u8,
        }
    }
}

/// SCSI command codes
pub mod scsi {
    pub const READ_10: u8 = 0x28;
    pub const WRITE_10: u8 = 0x2A;
    pub const READ_CAPACITY_10: u8 = 0x25;
    pub const INQUIRY: u8 = 0x12;
}

/// MSC filter layer
///
/// Translates block requests into SCSI commands.
/// Does NOT copy data - just sets up SCSI command descriptor block (CDB).
pub struct MscLayer {
    /// Block size (typically 512)
    pub block_size: u32,
}

impl MscLayer {
    pub fn new(block_size: u32) -> Self {
        Self { block_size }
    }
}

impl FilterLayer for MscLayer {
    fn filter_down(&mut self, desc: &mut Descriptor, data_pool: *mut u8) -> FilterResult {
        // Read block layer's request from layer_data[LAYER_BLOCK]
        let block_data = desc.layer_data[LAYER_BLOCK];
        let lba = (block_data & 0xFFFFFFFF) as u32;
        let block_count = ((block_data >> 32) & 0xFFFF) as u16;

        // Determine SCSI command based on direction
        let opcode = if desc.is_write() {
            scsi::WRITE_10
        } else {
            scsi::READ_10
        };

        // Store our layer data
        let msc_data = MscLayerData {
            scsi_opcode: opcode,
            lba,
            block_count,
            _pad: 0,
        };
        desc.layer_data[LAYER_MSC] = msc_data.to_u64();

        // Build SCSI CDB in the buffer BEFORE the data
        // Layout: [CBW (31 bytes)][DATA][CSW (13 bytes)]
        // We prepend CBW before data_offset
        const CBW_SIZE: u32 = 31;

        if desc.data_offset < CBW_SIZE {
            return FilterResult::Error(1); // No room for CBW
        }

        let cbw_offset = desc.data_offset - CBW_SIZE;
        let cbw_ptr = unsafe { data_pool.add(cbw_offset as usize) };

        // Build CBW (USB Mass Storage Command Block Wrapper)
        unsafe {
            let cbw = cbw_ptr;
            // dCBWSignature
            cbw.add(0).cast::<u32>().write_volatile(0x43425355); // "USBC"
            // dCBWTag
            cbw.add(4).cast::<u32>().write_volatile(desc.id);
            // dCBWDataTransferLength
            cbw.add(8).cast::<u32>().write_volatile(desc.data_len);
            // bmCBWFlags
            cbw.add(12).write_volatile(if desc.is_write() { 0x00 } else { 0x80 });
            // bCBWLUN
            cbw.add(13).write_volatile(0);
            // bCBWCBLength
            cbw.add(14).write_volatile(10); // SCSI 10-byte command

            // SCSI CDB (READ_10 or WRITE_10)
            let cdb = cbw.add(15);
            cdb.add(0).write_volatile(opcode);
            cdb.add(1).write_volatile(0); // flags
            // LBA (big-endian)
            cdb.add(2).write_volatile((lba >> 24) as u8);
            cdb.add(3).write_volatile((lba >> 16) as u8);
            cdb.add(4).write_volatile((lba >> 8) as u8);
            cdb.add(5).write_volatile(lba as u8);
            cdb.add(6).write_volatile(0); // reserved
            // Transfer length (big-endian)
            cdb.add(7).write_volatile((block_count >> 8) as u8);
            cdb.add(8).write_volatile(block_count as u8);
            cdb.add(9).write_volatile(0); // control
        }

        // Update descriptor to include CBW
        desc.data_offset = cbw_offset;
        desc.data_len += CBW_SIZE;

        FilterResult::Continue
    }

    fn filter_up(&mut self, desc: &mut Descriptor, _data_pool: *mut u8) -> FilterResult {
        // Check CSW (Command Status Wrapper) that follows the data
        // For now, just pass through - real impl would parse CSW

        // Strip CBW from the view
        const CBW_SIZE: u32 = 31;
        desc.data_offset += CBW_SIZE;
        desc.data_len -= CBW_SIZE;

        FilterResult::Continue
    }

    fn layer_index(&self) -> usize {
        LAYER_MSC
    }

    fn name(&self) -> &'static str {
        "MSC"
    }
}

// =============================================================================
// xHCI Layer - USB Host Controller
// =============================================================================

/// xHCI layer metadata stored in descriptor.layer_data[LAYER_XHCI]
///
/// Packed to exactly 64 bits for conversion to/from u64.
/// Layout: [slot_id:8][endpoint_dci:8][flags:8][_pad:8][trb_phys:32]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct XhciLayerData {
    /// Slot ID
    pub slot_id: u8,
    /// Endpoint DCI
    pub endpoint_dci: u8,
    /// Flags
    pub flags: u8,
    pub _pad: u8,
    /// TRB physical address (for completion matching)
    pub trb_phys: u32,
}

impl XhciLayerData {
    pub fn to_u64(self) -> u64 {
        // Pack manually for clarity
        (self.slot_id as u64)
            | ((self.endpoint_dci as u64) << 8)
            | ((self.flags as u64) << 16)
            | ((self._pad as u64) << 24)
            | ((self.trb_phys as u64) << 32)
    }

    pub fn from_u64(val: u64) -> Self {
        Self {
            slot_id: val as u8,
            endpoint_dci: (val >> 8) as u8,
            flags: (val >> 16) as u8,
            _pad: (val >> 24) as u8,
            trb_phys: (val >> 32) as u32,
        }
    }
}

/// xHCI filter layer
///
/// Creates TRBs and manages USB transfers.
/// The actual TRB ring is in a separate DMA region - this layer
/// just records what TRBs were posted for completion matching.
pub struct XhciLayer {
    /// Slot ID for this device
    pub slot_id: u8,
    /// Bulk OUT endpoint DCI
    pub bulk_out_dci: u8,
    /// Bulk IN endpoint DCI
    pub bulk_in_dci: u8,
    /// Physical base of data pool (for TRB addresses)
    pub data_pool_phys: u64,
}

impl XhciLayer {
    pub fn new(slot_id: u8, bulk_out_dci: u8, bulk_in_dci: u8, data_pool_phys: u64) -> Self {
        Self {
            slot_id,
            bulk_out_dci,
            bulk_in_dci,
            data_pool_phys,
        }
    }
}

impl FilterLayer for XhciLayer {
    fn filter_down(&mut self, desc: &mut Descriptor, _data_pool: *mut u8) -> FilterResult {
        // Select endpoint based on direction
        let endpoint_dci = if desc.is_write() {
            self.bulk_out_dci
        } else {
            self.bulk_in_dci
        };

        // Calculate physical address of data
        let data_phys = self.data_pool_phys + desc.data_offset as u64;

        // Store xHCI layer data
        let xhci_data = XhciLayerData {
            slot_id: self.slot_id,
            endpoint_dci,
            flags: 0,
            _pad: 0,
            trb_phys: data_phys as u32, // Lower 32 bits for matching
        };
        desc.layer_data[LAYER_XHCI] = xhci_data.to_u64();

        // In a real implementation, this is where we'd:
        // 1. Build TRBs in the transfer ring
        // 2. Ring the doorbell
        // For now, return Pending to indicate hardware processing needed

        FilterResult::Pending
    }

    fn filter_up(&mut self, desc: &mut Descriptor, _data_pool: *mut u8) -> FilterResult {
        // Called when hardware completes
        // The completion handler already filled in transferred/error

        let _xhci_data = XhciLayerData::from_u64(desc.layer_data[LAYER_XHCI]);

        // Check for errors based on xHCI completion code
        // (In real impl, completion code would be in xhci_data.flags or similar)

        if desc.is_error() {
            return FilterResult::Error(desc.error_code);
        }

        FilterResult::Continue
    }

    fn layer_index(&self) -> usize {
        LAYER_XHCI
    }

    fn name(&self) -> &'static str {
        "xHCI"
    }
}

// =============================================================================
// Block Layer - Simple block device abstraction
// =============================================================================

/// Block device layer
///
/// Presents a simple read/write block interface to filesystem.
pub struct BlockLayer {
    pub block_size: u32,
    pub block_count: u64,
}

impl BlockLayer {
    pub fn new(block_size: u32, block_count: u64) -> Self {
        Self { block_size, block_count }
    }

    /// Create a read descriptor
    pub fn read_blocks(&self, desc: &mut Descriptor, lba: u32, count: u16) {
        // Store in layer_data: lba in lower 32 bits, count in next 16 bits
        desc.layer_data[LAYER_BLOCK] = (lba as u64) | ((count as u64) << 32);
        desc.data_len = count as u32 * self.block_size;
        desc.flags = desc_flags::VALID;
    }

    /// Create a write descriptor
    pub fn write_blocks(&self, desc: &mut Descriptor, lba: u32, count: u16) {
        desc.layer_data[LAYER_BLOCK] = (lba as u64) | ((count as u64) << 32);
        desc.data_len = count as u32 * self.block_size;
        desc.flags = desc_flags::VALID | desc_flags::WRITE;
    }
}

impl FilterLayer for BlockLayer {
    fn filter_down(&mut self, _desc: &mut Descriptor, _data_pool: *mut u8) -> FilterResult {
        // Block layer just passes through - MSC layer does the work
        FilterResult::Continue
    }

    fn filter_up(&mut self, _desc: &mut Descriptor, _data_pool: *mut u8) -> FilterResult {
        // Translate byte count to block count if needed
        FilterResult::Continue
    }

    fn layer_index(&self) -> usize {
        LAYER_BLOCK
    }

    fn name(&self) -> &'static str {
        "Block"
    }
}
