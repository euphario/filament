//! Partition layer - exposes partitions as BlockDevices
//!
//! Sits between raw disk and filesystem:
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  Filesystem (FAT32, ext4, etc.)         │
//! │  sees: BlockDevice                      │
//! ├─────────────────────────────────────────┤
//! │  Partition Layer                        │
//! │  in:  read_blocks/write_blocks          │
//! │  out: BlockDevice (raw disk)            │
//! │  hides: MBR/GPT, LBA offsets            │
//! ├─────────────────────────────────────────┤
//! │  Raw Disk (ScsiBlockDevice, etc.)       │
//! └─────────────────────────────────────────┘
//! ```
//!
//! Each partition becomes its own BlockDevice with LBA 0 = partition start.
//! Filesystem doesn't know it's on a partition vs raw disk.

use crate::block::{BlockDevice, BlockError};

// =============================================================================
// Partition Table Structures
// =============================================================================

/// MBR partition entry (16 bytes)
#[derive(Clone, Copy, Debug)]
pub struct MbrEntry {
    /// Boot indicator (0x80 = bootable, 0x00 = not bootable)
    pub bootable: u8,
    /// Partition type (0x0B/0x0C = FAT32, 0x07 = NTFS, 0x83 = Linux, etc.)
    pub partition_type: u8,
    /// Starting LBA
    pub start_lba: u32,
    /// Number of sectors
    pub sector_count: u32,
}

impl MbrEntry {
    /// Parse from 16-byte MBR entry
    fn from_bytes(data: &[u8]) -> Self {
        Self {
            bootable: data[0],
            partition_type: data[4],
            // CHS values at [1..4] and [5..8] - we ignore them, use LBA
            start_lba: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            sector_count: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
        }
    }

    /// Is this entry valid (non-empty)?
    pub fn is_valid(&self) -> bool {
        self.partition_type != 0 && self.sector_count > 0
    }

    /// Is this a FAT32 partition?
    pub fn is_fat32(&self) -> bool {
        matches!(self.partition_type, 0x0B | 0x0C | 0x1B | 0x1C)
    }

    /// Is this a FAT16 partition?
    pub fn is_fat16(&self) -> bool {
        matches!(self.partition_type, 0x04 | 0x06 | 0x0E | 0x14 | 0x16 | 0x1E)
    }

    /// Is this any FAT partition?
    pub fn is_fat(&self) -> bool {
        self.is_fat16() || self.is_fat32()
    }
}

/// Partition type names (for display)
pub fn partition_type_name(ptype: u8) -> &'static str {
    match ptype {
        0x00 => "Empty",
        0x01 => "FAT12",
        0x04 | 0x06 | 0x0E => "FAT16",
        0x05 | 0x0F => "Extended",
        0x07 => "NTFS/exFAT",
        0x0B | 0x0C => "FAT32",
        0x1B | 0x1C => "FAT32 (hidden)",
        0x82 => "Linux swap",
        0x83 => "Linux",
        0x8E => "Linux LVM",
        0xEE => "GPT protective",
        0xEF => "EFI System",
        _ => "Unknown",
    }
}

/// Parsed MBR
#[derive(Clone, Copy)]
pub struct Mbr {
    /// Four primary partition entries
    pub entries: [MbrEntry; 4],
    /// Is this a valid MBR? (has 0x55AA signature)
    pub valid: bool,
    /// Is this a GPT disk? (first partition is type 0xEE)
    pub is_gpt: bool,
}

impl Mbr {
    /// Parse MBR from sector 0 data (512 bytes)
    pub fn parse(sector: &[u8]) -> Self {
        // Check signature
        let valid = sector.len() >= 512 && sector[510] == 0x55 && sector[511] == 0xAA;

        // Parse partition entries (offset 446, 4 entries of 16 bytes each)
        let mut entries = [MbrEntry {
            bootable: 0,
            partition_type: 0,
            start_lba: 0,
            sector_count: 0,
        }; 4];

        if valid {
            for i in 0..4 {
                let offset = 446 + i * 16;
                entries[i] = MbrEntry::from_bytes(&sector[offset..offset + 16]);
            }
        }

        // Check if GPT (first partition is protective MBR)
        let is_gpt = valid && entries[0].partition_type == 0xEE;

        Self { entries, valid, is_gpt }
    }

    /// Find first FAT partition
    pub fn find_fat_partition(&self) -> Option<&MbrEntry> {
        self.entries.iter().find(|e| e.is_fat())
    }

    /// Find first FAT32 partition
    pub fn find_fat32_partition(&self) -> Option<&MbrEntry> {
        self.entries.iter().find(|e| e.is_fat32())
    }

    /// Iterate valid partitions
    pub fn partitions(&self) -> impl Iterator<Item = (usize, &MbrEntry)> {
        self.entries.iter().enumerate().filter(|(_, e)| e.is_valid())
    }
}

// =============================================================================
// Partition Device - wraps a BlockDevice, offsets LBAs
// =============================================================================

/// A view into a partition on a disk
///
/// Implements BlockDevice so filesystems can use it directly.
/// All LBAs are translated: partition LBA 0 = disk LBA (start_lba).
pub struct PartitionDevice<T: BlockDevice> {
    /// Underlying disk
    disk: T,
    /// Partition start LBA on disk
    start_lba: u64,
    /// Partition size in blocks
    block_count: u64,
}

impl<T: BlockDevice> PartitionDevice<T> {
    /// Create a partition view from a disk and MBR entry
    pub fn from_mbr_entry(disk: T, entry: &MbrEntry) -> Option<Self> {
        if !entry.is_valid() {
            return None;
        }
        Some(Self {
            disk,
            start_lba: entry.start_lba as u64,
            block_count: entry.sector_count as u64,
        })
    }

    /// Create a partition view with explicit bounds
    pub fn new(disk: T, start_lba: u64, block_count: u64) -> Self {
        Self { disk, start_lba, block_count }
    }

    /// Get the starting LBA on the underlying disk
    pub fn start_lba(&self) -> u64 {
        self.start_lba
    }

    /// Consume and return the underlying disk
    pub fn into_inner(self) -> T {
        self.disk
    }
}

impl<T: BlockDevice> BlockDevice for PartitionDevice<T> {
    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), BlockError> {
        // Validate LBA is within partition
        let blocks = buf.len() as u64 / self.disk.block_size() as u64;
        if lba + blocks > self.block_count {
            return Err(BlockError::OutOfRange);
        }

        // Translate to disk LBA
        let disk_lba = self.start_lba + lba;
        self.disk.read_blocks(disk_lba, buf)
    }

    fn write_blocks(&mut self, lba: u64, buf: &[u8]) -> Result<(), BlockError> {
        let blocks = buf.len() as u64 / self.disk.block_size() as u64;
        if lba + blocks > self.block_count {
            return Err(BlockError::OutOfRange);
        }

        let disk_lba = self.start_lba + lba;
        self.disk.write_blocks(disk_lba, buf)
    }

    fn block_size(&self) -> u32 {
        self.disk.block_size()
    }

    fn block_count(&self) -> u64 {
        self.block_count
    }
}

// =============================================================================
// Partition Discovery Helper
// =============================================================================

/// Read and parse partition table from a block device
pub fn read_partition_table<T: BlockDevice>(disk: &mut T) -> Option<Mbr> {
    let block_size = disk.block_size() as usize;
    if block_size < 512 {
        return None;
    }

    // Read sector 0
    let mut sector = [0u8; 512];
    // Need to read full block if block_size > 512
    if block_size > 512 {
        let mut full_block = [0u8; 4096];
        if disk.read_blocks(0, &mut full_block[..block_size]).is_err() {
            return None;
        }
        sector.copy_from_slice(&full_block[..512]);
    } else {
        if disk.read_blocks(0, &mut sector).is_err() {
            return None;
        }
    }

    let mbr = Mbr::parse(&sector);
    if mbr.valid {
        Some(mbr)
    } else {
        None
    }
}
