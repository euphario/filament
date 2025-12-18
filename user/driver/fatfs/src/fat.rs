//! FAT Filesystem Parser
//!
//! Parses FAT12/16/32 boot sectors and provides filesystem navigation.

/// FAT filesystem type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FatType {
    Fat12,
    Fat16,
    Fat32,
}

/// Parsed FAT filesystem information
#[derive(Debug)]
pub struct FatFilesystem {
    pub fat_type: FatType,
    pub bytes_per_sector: u16,
    pub sectors_per_cluster: u8,
    pub reserved_sectors: u16,
    pub num_fats: u8,
    pub root_entry_count: u16,    // 0 for FAT32
    pub total_sectors: u32,
    pub fat_size: u32,            // Sectors per FAT
    pub root_dir_sector: u32,     // First sector of root dir (FAT12/16)
    pub data_start_sector: u32,   // First sector of data area
    pub total_clusters: u32,
    pub root_cluster: u32,        // FAT32 only
    pub partition_start: u32,     // LBA of partition start (for partitioned disks)
}

impl FatFilesystem {
    /// Parse boot sector and create filesystem info
    pub fn mount(boot_sector: &[u8], _block_size: u32, _block_count: u64) -> Option<Self> {
        if boot_sector.len() < 512 {
            return None;
        }

        // Check for valid boot signature
        if boot_sector[510] != 0x55 || boot_sector[511] != 0xAA {
            return None;
        }

        // Parse BPB (BIOS Parameter Block)
        let bytes_per_sector = u16::from_le_bytes([boot_sector[11], boot_sector[12]]);
        let sectors_per_cluster = boot_sector[13];
        let reserved_sectors = u16::from_le_bytes([boot_sector[14], boot_sector[15]]);
        let num_fats = boot_sector[16];
        let root_entry_count = u16::from_le_bytes([boot_sector[17], boot_sector[18]]);
        let total_sectors_16 = u16::from_le_bytes([boot_sector[19], boot_sector[20]]);
        let fat_size_16 = u16::from_le_bytes([boot_sector[22], boot_sector[23]]);
        let total_sectors_32 = u32::from_le_bytes([
            boot_sector[32], boot_sector[33], boot_sector[34], boot_sector[35]
        ]);

        // Validate BPB
        if bytes_per_sector == 0 || sectors_per_cluster == 0 || num_fats == 0 {
            return None;
        }
        if bytes_per_sector != 512 && bytes_per_sector != 1024 &&
           bytes_per_sector != 2048 && bytes_per_sector != 4096 {
            return None;
        }

        // Total sectors
        let total_sectors = if total_sectors_16 != 0 {
            total_sectors_16 as u32
        } else {
            total_sectors_32
        };

        // FAT size
        let fat_size = if fat_size_16 != 0 {
            fat_size_16 as u32
        } else {
            // FAT32: read from extended BPB
            u32::from_le_bytes([
                boot_sector[36], boot_sector[37], boot_sector[38], boot_sector[39]
            ])
        };

        // Root directory size in sectors (FAT12/16 only)
        let root_dir_sectors = ((root_entry_count as u32 * 32) + (bytes_per_sector as u32 - 1))
            / bytes_per_sector as u32;

        // First data sector
        let root_dir_sector = reserved_sectors as u32 + (num_fats as u32 * fat_size);
        let data_start_sector = root_dir_sector + root_dir_sectors;

        // Total data sectors and clusters
        let data_sectors = total_sectors - data_start_sector;
        let total_clusters = data_sectors / sectors_per_cluster as u32;

        // Determine FAT type based on cluster count
        let fat_type = if total_clusters < 4085 {
            FatType::Fat12
        } else if total_clusters < 65525 {
            FatType::Fat16
        } else {
            FatType::Fat32
        };

        // FAT32 root cluster
        let root_cluster = if fat_type == FatType::Fat32 {
            u32::from_le_bytes([
                boot_sector[44], boot_sector[45], boot_sector[46], boot_sector[47]
            ])
        } else {
            0
        };

        Some(FatFilesystem {
            fat_type,
            bytes_per_sector,
            sectors_per_cluster,
            reserved_sectors,
            num_fats,
            root_entry_count,
            total_sectors,
            fat_size,
            root_dir_sector,
            data_start_sector,
            total_clusters,
            root_cluster,
            partition_start: 0, // Set by caller if needed
        })
    }

    /// Convert cluster number to absolute sector number (includes partition offset)
    pub fn cluster_to_sector(&self, cluster: u32) -> u32 {
        // Clusters start at 2 (0 and 1 are reserved)
        self.partition_start + self.data_start_sector + (cluster - 2) * self.sectors_per_cluster as u32
    }

    /// Get absolute sector for root directory
    pub fn root_dir_absolute_sector(&self) -> u32 {
        self.partition_start + self.root_dir_sector
    }

    /// Get the next cluster in the chain from the FAT
    pub fn next_cluster(&self, _fat_data: &[u8], cluster: u32) -> Option<u32> {
        // TODO: implement FAT table reading
        // For now, just return None (end of chain)
        if cluster >= 2 && cluster < self.total_clusters + 2 {
            None
        } else {
            None
        }
    }
}

/// Directory entry (32 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct DirEntry {
    pub name: [u8; 11],
    pub attr: u8,
    pub reserved: u8,
    pub create_time_tenths: u8,
    pub create_time: u16,
    pub create_date: u16,
    pub access_date: u16,
    pub cluster_hi: u16,
    pub modify_time: u16,
    pub modify_date: u16,
    pub cluster_lo: u16,
    pub file_size: u32,
}

impl DirEntry {
    /// Check if entry is free (never used)
    pub fn is_free(&self) -> bool {
        self.name[0] == 0
    }

    /// Check if entry is deleted
    pub fn is_deleted(&self) -> bool {
        self.name[0] == 0xE5
    }

    /// Check if entry is a long filename entry
    pub fn is_lfn(&self) -> bool {
        self.attr == 0x0F
    }

    /// Check if entry is a directory
    pub fn is_directory(&self) -> bool {
        (self.attr & 0x10) != 0
    }

    /// Get the starting cluster
    pub fn start_cluster(&self) -> u32 {
        ((self.cluster_hi as u32) << 16) | (self.cluster_lo as u32)
    }

    /// Get the 8.3 filename as a string slice
    pub fn name_83(&self) -> ([u8; 12], usize) {
        let mut result = [0u8; 12];
        let mut len = 0;

        // Copy base name (trim trailing spaces)
        for i in 0..8 {
            if self.name[i] != 0x20 {
                result[len] = self.name[i];
                len += 1;
            }
        }

        // Add extension if present
        if self.name[8] != 0x20 {
            result[len] = b'.';
            len += 1;
            for i in 8..11 {
                if self.name[i] != 0x20 {
                    result[len] = self.name[i];
                    len += 1;
                }
            }
        }

        (result, len)
    }
}

// Directory entry attributes
pub const ATTR_READ_ONLY: u8 = 0x01;
pub const ATTR_HIDDEN: u8 = 0x02;
pub const ATTR_SYSTEM: u8 = 0x04;
pub const ATTR_VOLUME_ID: u8 = 0x08;
pub const ATTR_DIRECTORY: u8 = 0x10;
pub const ATTR_ARCHIVE: u8 = 0x20;
pub const ATTR_LONG_NAME: u8 = 0x0F;
