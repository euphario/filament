//! Bulk-Only Transport (BOT) structures
//!
//! This is just data layout - no logic, no hardware knowledge.

pub const CBW_SIGNATURE: u32 = 0x43425355; // 'USBC'
pub const CSW_SIGNATURE: u32 = 0x53425355; // 'USBS'

/// Command Block Wrapper (31 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Cbw {
    pub signature: u32,
    pub tag: u32,
    pub data_length: u32,
    pub flags: u8,
    pub lun: u8,
    pub cb_length: u8,
    pub cb: [u8; 16],
}

impl Cbw {
    pub fn new(tag: u32, data_length: u32, direction_in: bool, lun: u8, cdb: &[u8]) -> Self {
        let mut cb = [0u8; 16];
        let len = cdb.len().min(16);
        cb[..len].copy_from_slice(&cdb[..len]);

        Self {
            signature: CBW_SIGNATURE,
            tag,
            data_length,
            flags: if direction_in { 0x80 } else { 0x00 },
            lun,
            cb_length: len as u8,
            cb,
        }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const Self as *const u8, 31)
        }
    }
}

/// Command Status Wrapper (13 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct Csw {
    pub signature: u32,
    pub tag: u32,
    pub data_residue: u32,
    pub status: u8,
}

impl Csw {
    pub fn from_bytes(data: &[u8]) -> Self {
        if data.len() < 13 {
            return Self::default();
        }
        Self {
            signature: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
            tag: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
            data_residue: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            status: data[12],
        }
    }
}
