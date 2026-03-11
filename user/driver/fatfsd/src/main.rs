//! FAT16/FAT32 Filesystem Driver
//!
//! Read-write FAT16/FAT32 driver that serves the VFS protocol over DataPort.
//! Consumes a partition block port, provides a VFS protocol port.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │  vfsd (connects as consumer)       │
//! │  VFS protocol: full read/write     │
//! ├─────────────────────────────────────┤
//! │  fatfsd (this driver)              │
//! │  provides: fat0: (Filesystem)      │
//! │  consumes: part0: (Partition)       │
//! ├─────────────────────────────────────┤
//! │  partd                             │
//! │  provides: part0: (Partition)      │
//! └─────────────────────────────────────┘
//! ```
//!
//! ## Flow
//!
//! 1. Spawned by partd via devd rules when Partition port appears
//! 2. Gets SpawnContext → connects to partition DataPort (consumer)
//! 3. Reads BPB at sector 0, detects FAT16 or FAT32
//! 4. Caches FAT table (windowed for FAT32)
//! 5. Creates VFS DataPort (provider), registers as Filesystem port
//! 6. Serves VFS requests: OPEN, READ, WRITE, READDIR, STAT, MKDIR, UNLINK, CLOSE

#![no_std]
#![no_main]

use libsys::syscall;
use libos::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockPortConfig, bus_msg, ConfigKey,
    PortInfo, PortClass, port_subclass,
};
use libos::bus_runtime::driver_main;
use libos::ring::{IoSqe, io_status, POOL_SLOT_SIZE};
use libos::vfs_proto::{fs_op, open_flags, file_type, vfs_error, VfsDirEntry, VfsStat};
use libos::{uinfo, unotice, udebug, uerror};

const FAT_CACHE_ENTRIES: usize = 8192;
const MAX_OPEN_FILES: usize = 16;
/// Scratch buffer for batch reads — sized to one pool slot (2048 bytes).
const FAT_READ_BUF_SIZE: usize = 2048;

// =============================================================================
// Open file tracking
// =============================================================================

#[derive(Clone, Copy)]
struct OpenFile {
    in_use: bool,
    is_dir: bool,
    start_cluster: u32,
    size: u32,
    /// LBA of the directory sector containing this file's entry (for size updates).
    dir_entry_lba: u64,
    /// Byte offset within that sector (0..511) of the 32-byte dir entry.
    dir_entry_offset: u16,
}

impl OpenFile {
    const fn empty() -> Self {
        Self {
            in_use: false, is_dir: false, start_cluster: 0, size: 0,
            dir_entry_lba: 0, dir_entry_offset: 0,
        }
    }
}

// =============================================================================
// FAT16/FAT32 Driver
// =============================================================================

struct FatfsDriver {
    /// Consumer port (block reads from partition)
    consumer_port: Option<PortId>,
    /// VFS protocol provider port
    vfs_port: Option<PortId>,

    // BPB fields (shared)
    bytes_per_sector: u16,
    sectors_per_cluster: u8,
    reserved_sectors: u16,
    num_fats: u8,
    root_entry_count: u16,
    total_sectors: u32,
    fat_size_sectors: u32,

    // FAT32-specific
    is_fat32: bool,
    root_cluster: u32,

    // Derived layout
    fat_start_lba: u32,
    root_dir_start_lba: u32,
    root_dir_sectors: u32,
    data_start_lba: u32,

    // FAT cache — windowed: stores entries [fat_cache_base .. fat_cache_base + 8192)
    fat_cache: [u32; FAT_CACHE_ENTRIES],
    fat_cache_base: u32,
    fat_cache_valid: bool,

    // FAT write support — dirty tracking (1 bit per cache entry, 128 u64 words)
    fat_cache_dirty: [u64; FAT_CACHE_ENTRIES / 64],
    /// Next cluster index to check when allocating (avoids O(n) scan every time)
    free_cluster_hint: u32,

    // Open files
    open_files: [OpenFile; MAX_OPEN_FILES],

    // Port name derived from partition
    port_name: [u8; 32],
    port_name_len: usize,

    // Scratch buffer for batch FAT sector reads
    fat_read_buf: [u8; FAT_READ_BUF_SIZE],
}

impl FatfsDriver {
    const fn new() -> Self {
        Self {
            consumer_port: None,
            vfs_port: None,
            bytes_per_sector: 512,
            sectors_per_cluster: 0,
            reserved_sectors: 0,
            num_fats: 0,
            root_entry_count: 0,
            total_sectors: 0,
            fat_size_sectors: 0,
            is_fat32: false,
            root_cluster: 0,
            fat_start_lba: 0,
            root_dir_start_lba: 0,
            root_dir_sectors: 0,
            data_start_lba: 0,
            fat_cache: [0u32; FAT_CACHE_ENTRIES],
            fat_cache_base: 0,
            fat_cache_valid: false,
            fat_cache_dirty: [0u64; FAT_CACHE_ENTRIES / 64],
            free_cluster_hint: 2,
            open_files: [OpenFile::empty(); MAX_OPEN_FILES],
            port_name: [0; 32],
            port_name_len: 0,
            fat_read_buf: [0u8; FAT_READ_BUF_SIZE],
        }
    }

    // =========================================================================
    // Block I/O helpers
    // =========================================================================

    /// Read a single sector.
    fn read_block(&self, lba: u64, buf: &mut [u8], ctx: &mut dyn BusCtx) -> bool {
        let port_id = match self.consumer_port {
            Some(id) => id,
            None => return false,
        };
        read_sectors(port_id, lba, buf, ctx)
    }

    fn write_block(&self, lba: u64, data: &[u8], ctx: &mut dyn BusCtx) -> bool {
        let port_id = match self.consumer_port {
            Some(id) => id,
            None => return false,
        };
        write_sectors(port_id, lba, data, ctx)
    }

    // =========================================================================
    // FAT16/FAT32 initialization
    // =========================================================================

    fn parse_bpb(&mut self, sector: &[u8]) -> bool {
        if sector.len() < 512 {
            return false;
        }

        // Check for valid jump instruction
        if sector[0] != 0xEB && sector[0] != 0xE9 {
            uerror!("fatfsd", "invalid_bpb_jump"; byte = sector[0] as u32);
            return false;
        }

        self.bytes_per_sector = u16::from_le_bytes([sector[11], sector[12]]);
        self.sectors_per_cluster = sector[13];
        self.reserved_sectors = u16::from_le_bytes([sector[14], sector[15]]);
        self.num_fats = sector[16];
        self.root_entry_count = u16::from_le_bytes([sector[17], sector[18]]);

        let total_sectors_16 = u16::from_le_bytes([sector[19], sector[20]]);
        let total_sectors_32 = u32::from_le_bytes([sector[32], sector[33], sector[34], sector[35]]);
        self.total_sectors = if total_sectors_16 != 0 { total_sectors_16 as u32 } else { total_sectors_32 };

        // Detect FAT16 vs FAT32: BPB_FATSz16 at offset 22-23
        let fat_size_16 = u16::from_le_bytes([sector[22], sector[23]]);
        if fat_size_16 != 0 {
            self.fat_size_sectors = fat_size_16 as u32;
            self.is_fat32 = false;
        } else {
            // FAT32: BPB_FATSz32 at offset 36-39
            let fat_size_32 = u32::from_le_bytes([sector[36], sector[37], sector[38], sector[39]]);
            if fat_size_32 == 0 {
                uerror!("fatfsd", "invalid_bpb_fat_size";);
                return false;
            }
            self.fat_size_sectors = fat_size_32;
            self.is_fat32 = true;
            // BPB_RootClus at offset 44-47
            self.root_cluster = u32::from_le_bytes([sector[44], sector[45], sector[46], sector[47]]);
        }

        // Validate common fields
        if self.bytes_per_sector == 0 || self.sectors_per_cluster == 0 || self.num_fats == 0 {
            uerror!("fatfsd", "invalid_bpb_zero";);
            return false;
        }

        // Compute layout
        self.fat_start_lba = self.reserved_sectors as u32;
        self.root_dir_start_lba = self.fat_start_lba + (self.num_fats as u32 * self.fat_size_sectors);
        if self.is_fat32 {
            // FAT32: no fixed root directory region
            self.root_dir_sectors = 0;
        } else {
            self.root_dir_sectors = ((self.root_entry_count as u32 * 32) + self.bytes_per_sector as u32 - 1)
                / self.bytes_per_sector as u32;
        }
        self.data_start_lba = self.root_dir_start_lba + self.root_dir_sectors;

        // Validate cluster count matches expected FAT type
        let data_sectors = self.total_sectors.saturating_sub(self.data_start_lba);
        let cluster_count = data_sectors / self.sectors_per_cluster as u32;
        if self.is_fat32 {
            if cluster_count < 65525 {
                uerror!("fatfsd", "fat32_too_few_clusters"; clusters = cluster_count);
                return false;
            }
            uinfo!("fatfsd", "fat32_parsed";
                bps = self.bytes_per_sector as u32,
                spc = self.sectors_per_cluster as u32,
                root_cluster = self.root_cluster,
                clusters = cluster_count);
        } else {
            if cluster_count < 4085 || cluster_count >= 65525 {
                uerror!("fatfsd", "not_fat16"; clusters = cluster_count);
                return false;
            }
            udebug!("fatfsd", "fat16_parsed";
                bps = self.bytes_per_sector as u32,
                spc = self.sectors_per_cluster as u32,
                root_entries = self.root_entry_count as u32,
                clusters = cluster_count);
        }

        true
    }

    /// Load a window of FAT entries starting at `base_cluster` into the cache.
    /// For FAT16: 2 bytes per entry. For FAT32: 4 bytes per entry (masked to 28 bits).
    ///
    /// Reads all needed FAT sectors in a single batch I/O operation.
    fn cache_fat_window(&mut self, base_cluster: u32, ctx: &mut dyn BusCtx) -> bool {
        let entry_size: u32 = if self.is_fat32 { 4 } else { 2 };
        let bps = self.bytes_per_sector as u32;

        // How many entries exist in the FAT from base_cluster onward?
        let fat_bytes = self.fat_size_sectors * bps;
        let total_fat_entries = fat_bytes / entry_size;
        if base_cluster >= total_fat_entries {
            return false;
        }
        let entries_to_cache = (total_fat_entries - base_cluster).min(FAT_CACHE_ENTRIES as u32) as usize;

        // Which FAT sector range do we need?
        let byte_offset = base_cluster * entry_size;
        let first_fat_sector = byte_offset / bps;
        let offset_in_first_sector = (byte_offset % bps) as usize;

        let bytes_needed = offset_in_first_sector as u32 + entries_to_cache as u32 * entry_size;
        let sectors_needed = (bytes_needed + bps - 1) / bps;
        let sectors_to_read = sectors_needed.min(self.fat_size_sectors - first_fat_sector);

        // Read in chunks that fit one pool slot (POOL_SLOT_SIZE = 2048).
        // Each chunk reads up to `sectors_per_chunk` contiguous sectors.
        let port_id = match self.consumer_port {
            Some(id) => id,
            None => return false,
        };

        let chunk_bytes = POOL_SLOT_SIZE as usize;
        let sectors_per_chunk = chunk_bytes / bps as usize;
        let mut cached = 0;
        let mut sector = 0u32;
        let mut first_chunk = true;

        while sector < sectors_to_read && cached < entries_to_cache {
            let remaining_sectors = sectors_to_read - sector;
            let n = (remaining_sectors as usize).min(sectors_per_chunk);
            let read_bytes = n * bps as usize;
            let lba = self.fat_start_lba as u64 + (first_fat_sector + sector) as u64;

            if !read_sectors(port_id, lba, &mut self.fat_read_buf[..read_bytes], ctx) {
                uerror!("fatfsd", "fat_read_failed"; sector = first_fat_sector + sector, count = n as u32);
                return false;
            }

            // Parse entries from this chunk
            let start = if first_chunk { offset_in_first_sector } else { 0 };
            first_chunk = false;
            let es = entry_size as usize;
            let mut off = start;
            while off + es <= read_bytes && cached < entries_to_cache {
                if self.is_fat32 {
                    let val = u32::from_le_bytes([
                        self.fat_read_buf[off], self.fat_read_buf[off + 1],
                        self.fat_read_buf[off + 2], self.fat_read_buf[off + 3],
                    ]);
                    self.fat_cache[cached] = val & 0x0FFF_FFFF;
                } else {
                    let val = u16::from_le_bytes([self.fat_read_buf[off], self.fat_read_buf[off + 1]]);
                    self.fat_cache[cached] = val as u32;
                }
                cached += 1;
                off += es;
            }

            sector += n as u32;
        }

        self.fat_cache_base = base_cluster;
        self.fat_cache_valid = true;
        udebug!("fatfsd", "fat_cached"; base = base_cluster, entries = cached as u32, sectors = sectors_to_read);
        true
    }

    fn next_cluster(&mut self, cluster: u32, ctx: &mut dyn BusCtx) -> Option<u32> {
        // Check if cluster is within cached window; reload if not
        if !self.fat_cache_valid
            || cluster < self.fat_cache_base
            || (cluster - self.fat_cache_base) as usize >= FAT_CACHE_ENTRIES
        {
            if !self.cache_fat_window(cluster, ctx) {
                return None;
            }
        }
        let idx = (cluster - self.fat_cache_base) as usize;
        let next = self.fat_cache[idx];
        let eoc = if self.is_fat32 { 0x0FFF_FFF8 } else { 0xFFF8 };
        if next >= eoc {
            None // End of chain
        } else if next < 2 {
            None // Invalid
        } else {
            Some(next)
        }
    }

    fn cluster_to_lba(&self, cluster: u32) -> u64 {
        self.data_start_lba as u64 + (cluster as u64 - 2) * self.sectors_per_cluster as u64
    }

    fn cluster_size(&self) -> u32 {
        self.sectors_per_cluster as u32 * self.bytes_per_sector as u32
    }

    // =========================================================================
    // FAT mutation (write support)
    // =========================================================================

    fn mark_fat_dirty(&mut self, cache_idx: usize) {
        let word = cache_idx / 64;
        let bit = cache_idx % 64;
        self.fat_cache_dirty[word] |= 1u64 << bit;
    }

    fn is_fat_dirty(&self, cache_idx: usize) -> bool {
        let word = cache_idx / 64;
        let bit = cache_idx % 64;
        (self.fat_cache_dirty[word] >> bit) & 1 != 0
    }

    /// Write a FAT entry in the cache and mark it dirty.
    fn set_fat_entry(&mut self, cluster: u32, value: u32) {
        if cluster < self.fat_cache_base {
            return;
        }
        let idx = (cluster - self.fat_cache_base) as usize;
        if idx >= FAT_CACHE_ENTRIES {
            return;
        }
        self.fat_cache[idx] = if self.is_fat32 { value & 0x0FFF_FFFF } else { value };
        self.mark_fat_dirty(idx);
    }

    /// Flush all dirty FAT cache entries to disk.
    ///
    /// Groups dirty entries by sector, reads each sector, patches the modified
    /// entries, and writes it back to both FAT copies.
    fn flush_fat(&mut self, ctx: &mut dyn BusCtx) -> bool {
        let entry_size: u32 = if self.is_fat32 { 4 } else { 2 };
        let bps = self.bytes_per_sector as u32;
        let entries_per_sector = bps / entry_size;
        let mut sector_buf = [0u8; 512];

        // Scan dirty words to find which sectors need flushing
        let mut idx = 0usize;
        while idx < FAT_CACHE_ENTRIES {
            let word = idx / 64;
            if word >= self.fat_cache_dirty.len() {
                break;
            }
            if self.fat_cache_dirty[word] == 0 {
                idx += 64;
                continue;
            }

            // Found a dirty word — process each set bit
            let mut bits = self.fat_cache_dirty[word];
            while bits != 0 {
                let bit = bits.trailing_zeros() as usize;
                let cache_idx = word * 64 + bit;
                bits &= bits - 1; // Clear lowest set bit

                let cluster = self.fat_cache_base + cache_idx as u32;

                // Which FAT sector does this cluster entry live in?
                let byte_offset = cluster as u64 * entry_size as u64;
                let fat_sector = (byte_offset / bps as u64) as u32;
                let offset_in_sector = (byte_offset % bps as u64) as usize;

                // Read the FAT sector
                let fat_lba = self.fat_start_lba as u64 + fat_sector as u64;
                if !self.read_block(fat_lba, &mut sector_buf[..bps as usize], ctx) {
                    uerror!("fatfsd", "flush_fat_read_failed"; sector = fat_sector);
                    return false;
                }

                // Patch ALL dirty entries that fall in this same sector
                let sector_base_cluster = (fat_sector as u64 * bps as u64 / entry_size as u64) as u32;
                let sector_end_cluster = sector_base_cluster + entries_per_sector;

                for cl in sector_base_cluster..sector_end_cluster {
                    if cl < self.fat_cache_base {
                        continue;
                    }
                    let ci = (cl - self.fat_cache_base) as usize;
                    if ci >= FAT_CACHE_ENTRIES || !self.is_fat_dirty(ci) {
                        continue;
                    }
                    let off = ((cl - sector_base_cluster) * entry_size) as usize;
                    if self.is_fat32 {
                        // Preserve upper 4 bits of existing entry
                        let existing = u32::from_le_bytes([
                            sector_buf[off], sector_buf[off+1],
                            sector_buf[off+2], sector_buf[off+3],
                        ]);
                        let val = (existing & 0xF000_0000) | (self.fat_cache[ci] & 0x0FFF_FFFF);
                        sector_buf[off..off+4].copy_from_slice(&val.to_le_bytes());
                    } else {
                        let val = self.fat_cache[ci] as u16;
                        sector_buf[off..off+2].copy_from_slice(&val.to_le_bytes());
                    }
                    // Clear dirty bit
                    let w = ci / 64;
                    let b = ci % 64;
                    self.fat_cache_dirty[w] &= !(1u64 << b);
                }

                // Write to FAT copy 1
                if !self.write_block(fat_lba, &sector_buf[..bps as usize], ctx) {
                    uerror!("fatfsd", "flush_fat_write_failed"; sector = fat_sector);
                    return false;
                }

                // Write to FAT copy 2 (if present)
                if self.num_fats > 1 {
                    let fat2_lba = fat_lba + self.fat_size_sectors as u64;
                    if !self.write_block(fat2_lba, &sector_buf[..bps as usize], ctx) {
                        uerror!("fatfsd", "flush_fat2_write_failed"; sector = fat_sector);
                        // Non-fatal: FAT1 is the primary
                    }
                }
            }
        }
        true
    }

    /// Allocate a free cluster. Returns the cluster number or None if full.
    fn alloc_cluster(&mut self, ctx: &mut dyn BusCtx) -> Option<u32> {
        let entry_size: u32 = if self.is_fat32 { 4 } else { 2 };
        let total_fat_entries = (self.fat_size_sectors * self.bytes_per_sector as u32) / entry_size;
        // Clusters 0 and 1 are reserved
        let max_cluster = total_fat_entries.min(self.total_sectors / self.sectors_per_cluster as u32 + 2);

        let start = self.free_cluster_hint.max(2);

        // Scan from hint to end, then wrap around from 2 to hint
        for pass in 0..2 {
            let (from, to) = if pass == 0 { (start, max_cluster) } else { (2, start) };

            let mut cluster = from;
            while cluster < to {
                // Ensure this cluster is in the cache window
                if !self.fat_cache_valid
                    || cluster < self.fat_cache_base
                    || (cluster - self.fat_cache_base) as usize >= FAT_CACHE_ENTRIES
                {
                    // Flush dirty entries before sliding the window
                    if !self.flush_fat(ctx) {
                        return None;
                    }
                    if !self.cache_fat_window(cluster, ctx) {
                        return None;
                    }
                }

                let idx = (cluster - self.fat_cache_base) as usize;
                if self.fat_cache[idx] == 0 {
                    // Free cluster found — mark as end-of-chain
                    let eoc = if self.is_fat32 { 0x0FFF_FFFF } else { 0xFFFF };
                    self.fat_cache[idx] = eoc;
                    self.mark_fat_dirty(idx);
                    self.free_cluster_hint = cluster + 1;
                    return Some(cluster);
                }
                cluster += 1;
            }
        }
        None // Disk full
    }

    /// Link `new_cluster` to the end of the chain ending at `last_cluster`.
    fn extend_chain(&mut self, last_cluster: u32, new_cluster: u32, ctx: &mut dyn BusCtx) -> bool {
        // Ensure last_cluster is in cache
        if !self.fat_cache_valid
            || last_cluster < self.fat_cache_base
            || (last_cluster - self.fat_cache_base) as usize >= FAT_CACHE_ENTRIES
        {
            if !self.flush_fat(ctx) { return false; }
            if !self.cache_fat_window(last_cluster, ctx) { return false; }
        }
        self.set_fat_entry(last_cluster, new_cluster);
        true
    }

    /// Free an entire cluster chain starting at `cluster`.
    fn free_chain(&mut self, mut cluster: u32, ctx: &mut dyn BusCtx) -> bool {
        loop {
            if cluster < 2 {
                break;
            }

            // Ensure cluster is in cache
            if !self.fat_cache_valid
                || cluster < self.fat_cache_base
                || (cluster - self.fat_cache_base) as usize >= FAT_CACHE_ENTRIES
            {
                if !self.flush_fat(ctx) { return false; }
                if !self.cache_fat_window(cluster, ctx) { return false; }
            }

            let idx = (cluster - self.fat_cache_base) as usize;
            let next = self.fat_cache[idx];
            self.fat_cache[idx] = 0; // Mark as free
            self.mark_fat_dirty(idx);

            // Update hint if this cluster is earlier
            if cluster < self.free_cluster_hint {
                self.free_cluster_hint = cluster;
            }

            let eoc = if self.is_fat32 { 0x0FFF_FFF8 } else { 0xFFF8 };
            if next >= eoc || next < 2 {
                break;
            }
            cluster = next;
        }
        true
    }

    // =========================================================================
    // FAT16 directory entry parsing
    // =========================================================================

    /// Parse an 8.3 directory entry at offset in sector data.
    /// Returns (name, name_len, is_dir, start_cluster, file_size) or None if invalid.
    fn parse_dir_entry(&self, data: &[u8], offset: usize) -> Option<([u8; 12], usize, bool, u32, u32)> {
        if offset + 32 > data.len() {
            return None;
        }
        let entry = &data[offset..offset + 32];

        // Check for end of directory
        if entry[0] == 0x00 {
            return None;
        }
        // Skip deleted entries
        if entry[0] == 0xE5 {
            return Some(([0; 12], 0, false, 0, 0)); // Signal to skip
        }
        // Skip long name entries
        if entry[11] & 0x0F == 0x0F {
            return Some(([0; 12], 0, false, 0, 0)); // Signal to skip
        }
        // Skip volume label
        if entry[11] & 0x08 != 0 {
            return Some(([0; 12], 0, false, 0, 0)); // Signal to skip
        }

        let is_dir = entry[11] & 0x10 != 0;
        // FAT32: high 16 bits at offset 20-21, low 16 bits at offset 26-27
        // FAT16: offset 20-21 is zero, so this works for both
        let cluster_hi = u16::from_le_bytes([entry[20], entry[21]]) as u32;
        let cluster_lo = u16::from_le_bytes([entry[26], entry[27]]) as u32;
        let start_cluster = (cluster_hi << 16) | cluster_lo;
        let file_size = u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]);

        // Convert 8.3 name to readable form
        let mut name = [0u8; 12];
        let mut pos = 0;

        // Base name (trim trailing spaces)
        let mut base_end = 8;
        while base_end > 0 && entry[base_end - 1] == b' ' {
            base_end -= 1;
        }
        for i in 0..base_end {
            name[pos] = to_lower(entry[i]);
            pos += 1;
        }

        // Extension (trim trailing spaces)
        let mut ext_end = 11;
        while ext_end > 8 && entry[ext_end - 1] == b' ' {
            ext_end -= 1;
        }
        if ext_end > 8 {
            name[pos] = b'.';
            pos += 1;
            for i in 8..ext_end {
                name[pos] = to_lower(entry[i]);
                pos += 1;
            }
        }

        Some((name, pos, is_dir, start_cluster, file_size))
    }

    // =========================================================================
    // File handle management
    // =========================================================================

    fn alloc_handle(&mut self) -> Option<u32> {
        for i in 0..MAX_OPEN_FILES {
            if !self.open_files[i].in_use {
                self.open_files[i].in_use = true;
                return Some(i as u32);
            }
        }
        None
    }

    fn free_handle(&mut self, handle: u32) {
        if (handle as usize) < MAX_OPEN_FILES {
            self.open_files[handle as usize] = OpenFile::empty();
        }
    }

    fn get_file(&self, handle: u32) -> Option<&OpenFile> {
        let idx = handle as usize;
        if idx < MAX_OPEN_FILES && self.open_files[idx].in_use {
            Some(&self.open_files[idx])
        } else {
            None
        }
    }

    // =========================================================================
    // VFS request handlers
    // =========================================================================

    fn handle_vfs_open(&mut self, sqe: &IoSqe, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };

        let flags = sqe.param as u32;

        // Read path from pool
        let mut path_buf = [0u8; 256];
        let path_len = (sqe.data_len as usize).min(256);
        let path = {
            if let Some(port) = ctx.block_port(vfs_id) {
                if let Some(slice) = port.pool_slice(sqe.data_offset, path_len as u32) {
                    path_buf[..path_len].copy_from_slice(slice);
                    &path_buf[..path_len]
                } else {
                    &[]
                }
            } else {
                &[]
            }
        };

        if path.is_empty() {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
            return;
        }

        // Strip leading slash
        let path = if !path.is_empty() && path[0] == b'/' {
            &path[1..]
        } else {
            path
        };

        let is_dir_open = (flags & open_flags::DIR) != 0;

        // Root directory
        if path.is_empty() || path == b"." {
            if let Some(handle) = self.alloc_handle() {
                self.open_files[handle as usize].is_dir = true;
                self.open_files[handle as usize].start_cluster = 0; // root dir
                self.open_files[handle as usize].size = 0;

                Self::complete_vfs_result(ctx, vfs_id, sqe.tag, handle, 0);
            } else {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::TOO_MANY);
            }
            return;
        }

        let is_create = (flags & open_flags::CREATE) != 0;
        let is_trunc = (flags & open_flags::TRUNC) != 0;

        // Search root directory for the file
        match self.find_in_root(path, ctx) {
            Some((is_dir, start_cluster, file_size, entry_lba, entry_off)) => {
                if is_dir_open && !is_dir {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_DIR);
                    return;
                }

                let (final_cluster, final_size) = if is_trunc && !is_dir {
                    // Truncate: free existing cluster chain, update dir entry
                    if start_cluster >= 2 {
                        self.free_chain(start_cluster, ctx);
                        self.flush_fat(ctx);
                    }
                    self.update_dir_entry_size(entry_lba, entry_off, 0, ctx);
                    // Allocate a fresh cluster for the now-empty file
                    let cl = self.alloc_cluster(ctx).unwrap_or(0);
                    if cl >= 2 {
                        self.flush_fat(ctx);
                        // Update cluster in dir entry
                        self.update_dir_entry_cluster(entry_lba, entry_off, cl, ctx);
                    }
                    (cl, 0)
                } else {
                    (start_cluster, file_size)
                };

                if let Some(handle) = self.alloc_handle() {
                    let f = &mut self.open_files[handle as usize];
                    f.is_dir = is_dir;
                    f.start_cluster = final_cluster;
                    f.size = final_size;
                    f.dir_entry_lba = entry_lba;
                    f.dir_entry_offset = entry_off;
                    Self::complete_vfs_result(ctx, vfs_id, sqe.tag, handle, 0);
                } else {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::TOO_MANY);
                }
            }
            None => {
                if !is_create {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
                    return;
                }

                // CREATE: allocate cluster and create directory entry
                let cluster = match self.alloc_cluster(ctx) {
                    Some(c) => c,
                    None => {
                        Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NO_SPACE);
                        return;
                    }
                };
                if !self.flush_fat(ctx) {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                    return;
                }

                let (entry_lba, entry_off) = match self.create_dir_entry(0, path, false, cluster, ctx) {
                    Some(pos) => pos,
                    None => {
                        Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                        return;
                    }
                };

                if let Some(handle) = self.alloc_handle() {
                    let f = &mut self.open_files[handle as usize];
                    f.is_dir = false;
                    f.start_cluster = cluster;
                    f.size = 0;
                    f.dir_entry_lba = entry_lba;
                    f.dir_entry_offset = entry_off;
                    Self::complete_vfs_result(ctx, vfs_id, sqe.tag, handle, 0);
                } else {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::TOO_MANY);
                }
            }
        }
    }

    fn handle_vfs_read(&mut self, sqe: &IoSqe, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };
        let handle = sqe.param as u32;
        let file_offset = sqe.lba;
        let buf_offset = sqe.data_offset;
        let buf_len = sqe.data_len;

        let file = match self.get_file(handle) {
            Some(f) => *f,
            None => {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
                return;
            }
        };

        if file.is_dir {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IS_DIR);
            return;
        }

        // Clamp read to file size
        let remaining = if file_offset < file.size as u64 {
            (file.size as u64 - file_offset) as u32
        } else {
            0
        };
        let to_read = buf_len.min(remaining);

        if to_read == 0 {
            Self::complete_vfs_result(ctx, vfs_id, sqe.tag, 0, 0);
            return;
        }

        // Follow cluster chain to find the starting cluster for this offset
        let cluster_sz = self.cluster_size();
        let start_cluster_index = (file_offset / cluster_sz as u64) as u32;
        let offset_in_cluster = (file_offset % cluster_sz as u64) as u32;

        let mut cluster = file.start_cluster;
        for _ in 0..start_cluster_index {
            match self.next_cluster(cluster, ctx) {
                Some(c) => cluster = c,
                None => {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                    return;
                }
            }
        }

        // Read data cluster by cluster, chunked to fit pool slots
        let mut total_read = 0u32;
        let mut cluster_offset = offset_in_cluster;
        let cluster_sz = self.cluster_size() as usize;
        let bps = self.bytes_per_sector as usize;
        let chunk_bytes = POOL_SLOT_SIZE as usize;
        let port_id = match self.consumer_port {
            Some(id) => id,
            None => {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                return;
            }
        };

        while total_read < to_read {
            let base_lba = self.cluster_to_lba(cluster);
            let start_in_cluster = cluster_offset as usize;
            let needed = (to_read - total_read) as usize;
            let available = cluster_sz - start_in_cluster;
            let cluster_read = needed.min(available);

            // Read this cluster's data in pool-slot-sized chunks
            let mut done = 0usize;
            while done < cluster_read {
                let pos = start_in_cluster + done;
                let first_sector = pos / bps;
                let byte_start = first_sector * bps;
                let remain = cluster_read - done;
                let byte_end = (pos + remain + bps - 1) / bps * bps;
                let io_len = (byte_end - byte_start).min(chunk_bytes).min(cluster_sz - byte_start);
                let io_lba = base_lba + first_sector as u64;

                if !read_sectors(port_id, io_lba, &mut self.fat_read_buf[..io_len], ctx) {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                    return;
                }

                let src_start = pos - byte_start;
                let copy_len = remain.min(io_len - src_start);
                if let Some(port) = ctx.block_port(vfs_id) {
                    let dst_offset = buf_offset + total_read + done as u32;
                    port.pool_write(dst_offset, &self.fat_read_buf[src_start..src_start + copy_len]);
                }
                done += copy_len;
            }

            total_read += cluster_read as u32;
            cluster_offset = 0;

            if total_read < to_read {
                match self.next_cluster(cluster, ctx) {
                    Some(c) => cluster = c,
                    None => break,
                }
            }
        }

        Self::complete_vfs_result(ctx, vfs_id, sqe.tag, 0, total_read);
    }

    fn handle_vfs_readdir(&mut self, sqe: &IoSqe, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };
        let handle = sqe.param as u32;
        let buf_offset = sqe.data_offset;
        let buf_len = sqe.data_len;

        let file = match self.get_file(handle) {
            Some(f) => *f,
            None => {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
                return;
            }
        };

        if !file.is_dir {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_DIR);
            return;
        }

        let max_entries = buf_len as usize / VfsDirEntry::SIZE;
        let mut entry_count = 0u32;
        let mut total_bytes = 0u32;

        // Resolve root sentinel: cluster 0 means root directory
        let start_cluster = if file.start_cluster == 0 {
            if self.is_fat32 { self.root_cluster } else { 0 }
        } else {
            file.start_cluster
        };

        if start_cluster == 0 {
            // FAT16 root directory - fixed region at root_dir_start_lba
            self.readdir_linear(
                self.root_dir_start_lba as u64,
                self.root_dir_sectors,
                vfs_id,
                buf_offset,
                max_entries,
                &mut entry_count,
                &mut total_bytes,
                ctx,
            );
        } else {
            // Cluster chain (FAT32 root dir or any subdirectory)
            let mut cluster = start_cluster;
            loop {
                let lba = self.cluster_to_lba(cluster);
                let sectors = self.sectors_per_cluster as u32;
                self.readdir_linear(
                    lba,
                    sectors,
                    vfs_id,
                    buf_offset + total_bytes,
                    max_entries - entry_count as usize,
                    &mut entry_count,
                    &mut total_bytes,
                    ctx,
                );

                match self.next_cluster(cluster, ctx) {
                    Some(c) => cluster = c,
                    None => break,
                }
            }
        }

        Self::complete_vfs_result(ctx, vfs_id, sqe.tag, entry_count, total_bytes);
    }

    /// Read directory entries from linear sectors.
    fn readdir_linear(
        &self,
        start_lba: u64,
        sector_count: u32,
        vfs_id: PortId,
        buf_offset: u32,
        max_entries: usize,
        entry_count: &mut u32,
        total_bytes: &mut u32,
        ctx: &mut dyn BusCtx,
    ) {
        let mut sector_buf = [0u8; 512];

        for sec in 0..sector_count {
            if *entry_count as usize >= max_entries {
                break;
            }

            let lba = start_lba + sec as u64;
            if !self.read_block(lba, &mut sector_buf[..self.bytes_per_sector as usize], ctx) {
                break;
            }

            let entries_per_sector = self.bytes_per_sector as usize / 32;
            for i in 0..entries_per_sector {
                if *entry_count as usize >= max_entries {
                    break;
                }

                let offset = i * 32;
                match self.parse_dir_entry(&sector_buf, offset) {
                    Some((_, 0, _, _, _)) => continue, // Skip deleted/LFN/volume
                    Some((name, name_len, is_dir, _cluster, size)) => {
                        let mut dir_entry = VfsDirEntry::empty();
                        dir_entry.set_name(&name[..name_len]);
                        dir_entry.file_type = if is_dir { file_type::DIR } else { file_type::FILE };
                        dir_entry.size = size;

                        // Write to VFS pool
                        let write_offset = buf_offset + *total_bytes;
                        let mut entry_buf = [0u8; VfsDirEntry::SIZE];
                        dir_entry.write_to(&mut entry_buf, 0);
                        if let Some(port) = ctx.block_port(vfs_id) {
                            port.pool_write(write_offset, &entry_buf);
                        }

                        *entry_count += 1;
                        *total_bytes += VfsDirEntry::SIZE as u32;
                    }
                    None => return, // End of directory (0x00 marker)
                }
            }
        }
    }

    fn handle_vfs_stat(&mut self, sqe: &IoSqe, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };
        let handle = sqe.param as u32;

        let file = match self.get_file(handle) {
            Some(f) => *f,
            None => {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
                return;
            }
        };

        let stat = VfsStat {
            size: file.size as u64,
            file_type: if file.is_dir { file_type::DIR } else { file_type::FILE },
            _pad: [0; 7],
        };

        let mut stat_buf = [0u8; VfsStat::SIZE];
        stat.write_to(&mut stat_buf, 0);
        if let Some(port) = ctx.block_port(vfs_id) {
            port.pool_write(sqe.data_offset, &stat_buf);
        }

        Self::complete_vfs_result(ctx, vfs_id, sqe.tag, 0, VfsStat::SIZE as u32);
    }

    fn handle_vfs_close(&mut self, sqe: &IoSqe, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };
        let handle = sqe.param as u32;
        self.free_handle(handle);
        Self::complete_vfs_result(ctx, vfs_id, sqe.tag, 0, 0);
    }

    fn handle_vfs_write(&mut self, sqe: &IoSqe, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };
        let handle = sqe.param as u32;
        let file_offset = sqe.lba;
        let data_pool_offset = sqe.data_offset;
        let data_len = sqe.data_len;

        let file = match self.get_file(handle) {
            Some(f) => *f,
            None => {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
                return;
            }
        };

        if file.is_dir {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IS_DIR);
            return;
        }

        if data_len == 0 {
            Self::complete_vfs_result(ctx, vfs_id, sqe.tag, 0, 0);
            return;
        }

        let cluster_sz = self.cluster_size();
        let bps = self.bytes_per_sector as usize;
        let spc = self.sectors_per_cluster as u32;

        // Walk to the cluster at file_offset, extending chain as needed
        let target_cluster_idx = (file_offset / cluster_sz as u64) as u32;

        // If file has no cluster yet (empty file after CREATE), allocate one
        let mut start_cluster = file.start_cluster;
        if start_cluster < 2 {
            match self.alloc_cluster(ctx) {
                Some(c) => {
                    start_cluster = c;
                    self.open_files[handle as usize].start_cluster = c;
                    // Update dir entry with new cluster
                    self.update_dir_entry_cluster(
                        file.dir_entry_lba, file.dir_entry_offset, c, ctx,
                    );
                    self.flush_fat(ctx);
                }
                None => {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NO_SPACE);
                    return;
                }
            }
        }

        let mut cluster = start_cluster;
        let mut last_cluster = cluster;

        for _ in 0..target_cluster_idx {
            last_cluster = cluster;
            match self.next_cluster(cluster, ctx) {
                Some(c) => cluster = c,
                None => {
                    // Need to extend chain
                    let new_c = match self.alloc_cluster(ctx) {
                        Some(c) => c,
                        None => {
                            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NO_SPACE);
                            return;
                        }
                    };
                    if !self.extend_chain(last_cluster, new_c, ctx) || !self.flush_fat(ctx) {
                        Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                        return;
                    }
                    cluster = new_c;
                }
            }
        }

        // Now write data sector by sector
        let mut total_written = 0u32;
        let mut cluster_offset = (file_offset % cluster_sz as u64) as u32;
        let mut sector_buf = [0u8; 512];
        let mut src_buf = [0u8; 512];
        last_cluster = cluster;

        while total_written < data_len {
            let sector_in_cluster = cluster_offset / self.bytes_per_sector as u32;
            let offset_in_sector = cluster_offset % self.bytes_per_sector as u32;
            let lba_base = self.cluster_to_lba(cluster);

            for sec in sector_in_cluster..spc {
                if total_written >= data_len {
                    break;
                }

                let sec_lba = lba_base + sec as u64;
                let start = if sec == sector_in_cluster { offset_in_sector as usize } else { 0 };
                let available = bps - start;
                let needed = (data_len - total_written) as usize;
                let copy_len = available.min(needed);

                // If partial sector write, read-modify-write
                if start > 0 || copy_len < bps {
                    if !self.read_block(sec_lba, &mut sector_buf[..bps], ctx) {
                        Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                        return;
                    }
                }

                // Read source data from VFS pool
                if let Some(port) = ctx.block_port(vfs_id) {
                    let src_offset = data_pool_offset + total_written;
                    if let Some(slice) = port.pool_slice(src_offset, copy_len as u32) {
                        src_buf[..copy_len].copy_from_slice(slice);
                    }
                }
                sector_buf[start..start + copy_len].copy_from_slice(&src_buf[..copy_len]);

                if !self.write_block(sec_lba, &sector_buf[..bps], ctx) {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                    return;
                }

                total_written += copy_len as u32;
            }

            cluster_offset = 0;

            // Move to next cluster if more data to write
            if total_written < data_len {
                last_cluster = cluster;
                match self.next_cluster(cluster, ctx) {
                    Some(c) => cluster = c,
                    None => {
                        let new_c = match self.alloc_cluster(ctx) {
                            Some(c) => c,
                            None => {
                                // Partial write — update size and return what we wrote
                                break;
                            }
                        };
                        if !self.extend_chain(last_cluster, new_c, ctx) || !self.flush_fat(ctx) {
                            break;
                        }
                        cluster = new_c;
                    }
                }
            }
        }

        // Update file size if it grew
        let end_offset = file_offset as u32 + total_written;
        if end_offset > file.size {
            self.open_files[handle as usize].size = end_offset;
            self.update_dir_entry_size(file.dir_entry_lba, file.dir_entry_offset, end_offset, ctx);
        }

        Self::complete_vfs_result(ctx, vfs_id, sqe.tag, 0, total_written);
    }

    fn handle_vfs_mkdir(&mut self, sqe: &IoSqe, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };

        // Read path from pool
        let mut path_buf = [0u8; 256];
        let path_len = (sqe.data_len as usize).min(256);
        let path = {
            if let Some(port) = ctx.block_port(vfs_id) {
                if let Some(slice) = port.pool_slice(sqe.data_offset, path_len as u32) {
                    path_buf[..path_len].copy_from_slice(slice);
                    &path_buf[..path_len]
                } else {
                    &[]
                }
            } else {
                &[]
            }
        };

        if path.is_empty() {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
            return;
        }

        let path = if !path.is_empty() && path[0] == b'/' { &path[1..] } else { path };

        // Check if already exists
        if self.find_in_root(path, ctx).is_some() {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::EXISTS);
            return;
        }

        // Allocate a cluster for the new directory
        let cluster = match self.alloc_cluster(ctx) {
            Some(c) => c,
            None => {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NO_SPACE);
                return;
            }
        };
        if !self.flush_fat(ctx) {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
            return;
        }

        // Zero-fill the directory cluster
        let bps = self.bytes_per_sector as usize;
        let mut sector_buf = [0u8; 512];
        sector_buf[..bps].fill(0);
        let lba_base = self.cluster_to_lba(cluster);
        for sec in 0..self.sectors_per_cluster as u32 {
            if !self.write_block(lba_base + sec as u64, &sector_buf[..bps], ctx) {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                return;
            }
        }

        // Write . and .. entries
        let dot_name: [u8; 11] = *b".          ";
        let dotdot_name: [u8; 11] = *b"..         ";
        let root_cl = if self.is_fat32 { self.root_cluster } else { 0 };

        Self::format_dir_entry(&mut sector_buf, 0, &dot_name, 0x10, cluster, 0);
        Self::format_dir_entry(&mut sector_buf, 32, &dotdot_name, 0x10, root_cl, 0);
        if !self.write_block(lba_base, &sector_buf[..bps], ctx) {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
            return;
        }

        // Create entry in root directory
        match self.create_dir_entry(0, path, true, cluster, ctx) {
            Some(_) => Self::complete_vfs_result(ctx, vfs_id, sqe.tag, 0, 0),
            None => Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR),
        }
    }

    fn handle_vfs_unlink(&mut self, sqe: &IoSqe, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };

        // Read path from pool
        let mut path_buf = [0u8; 256];
        let path_len = (sqe.data_len as usize).min(256);
        let path = {
            if let Some(port) = ctx.block_port(vfs_id) {
                if let Some(slice) = port.pool_slice(sqe.data_offset, path_len as u32) {
                    path_buf[..path_len].copy_from_slice(slice);
                    &path_buf[..path_len]
                } else {
                    &[]
                }
            } else {
                &[]
            }
        };

        if path.is_empty() {
            Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
            return;
        }

        let path = if !path.is_empty() && path[0] == b'/' { &path[1..] } else { path };

        match self.find_in_root(path, ctx) {
            Some((is_dir, cluster, _size, entry_lba, entry_off)) => {
                if is_dir {
                    // TODO: check directory is empty before deleting
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IS_DIR);
                    return;
                }
                if self.delete_dir_entry(entry_lba, entry_off, cluster, ctx) {
                    Self::complete_vfs_result(ctx, vfs_id, sqe.tag, 0, 0);
                } else {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                }
            }
            None => {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
            }
        }
    }

    // =========================================================================
    // Directory write helpers
    // =========================================================================

    /// Convert a filename to 8.3 format. Returns None if the name is too long or invalid.
    fn to_8_3(name: &[u8]) -> Option<[u8; 11]> {
        let mut result = [b' '; 11];

        // Find the last dot for extension splitting
        let dot_pos = name.iter().rposition(|&b| b == b'.');
        let (base, ext) = match dot_pos {
            Some(pos) => (&name[..pos], &name[pos+1..]),
            None => (name, &[] as &[u8]),
        };

        if base.is_empty() || base.len() > 8 || ext.len() > 3 {
            return None;
        }

        for (i, &b) in base.iter().enumerate() {
            result[i] = to_upper(b);
        }
        for (i, &b) in ext.iter().enumerate() {
            result[8 + i] = to_upper(b);
        }
        Some(result)
    }

    /// Write a 32-byte directory entry into a sector buffer at `offset`.
    fn format_dir_entry(buf: &mut [u8], offset: usize, name_8_3: &[u8; 11], attr: u8, cluster: u32, size: u32) {
        let e = &mut buf[offset..offset + 32];
        e[..11].copy_from_slice(name_8_3);
        e[11] = attr;
        e[12..20].fill(0); // Reserved, create time/date, access date
        e[20..22].copy_from_slice(&((cluster >> 16) as u16).to_le_bytes());
        e[22..26].fill(0); // Write time/date (TODO: real timestamps)
        e[26..28].copy_from_slice(&(cluster as u16).to_le_bytes());
        e[28..32].copy_from_slice(&size.to_le_bytes());
    }

    /// Find a free slot in a directory and create an entry.
    /// Returns (entry_lba, entry_offset_in_sector) on success.
    fn create_dir_entry(
        &mut self,
        parent_cluster: u32,
        name: &[u8],
        is_dir: bool,
        initial_cluster: u32,
        ctx: &mut dyn BusCtx,
    ) -> Option<(u64, u16)> {
        let name_8_3 = Self::to_8_3(name)?;
        let attr: u8 = if is_dir { 0x10 } else { 0x20 }; // DIR or ARCHIVE

        let bps = self.bytes_per_sector as usize;
        let entries_per_sector = bps / 32;
        let mut sector_buf = [0u8; 512];

        // Resolve parent: cluster 0 = root directory
        if parent_cluster == 0 && !self.is_fat32 {
            // FAT16 root — fixed region
            for sec in 0..self.root_dir_sectors {
                let lba = self.root_dir_start_lba as u64 + sec as u64;
                if !self.read_block(lba, &mut sector_buf[..bps], ctx) {
                    return None;
                }
                for i in 0..entries_per_sector {
                    let off = i * 32;
                    if sector_buf[off] == 0x00 || sector_buf[off] == 0xE5 {
                        Self::format_dir_entry(&mut sector_buf, off, &name_8_3, attr, initial_cluster, 0);
                        if !self.write_block(lba, &sector_buf[..bps], ctx) {
                            return None;
                        }
                        return Some((lba, off as u16));
                    }
                }
            }
            return None; // Root directory full (FAT16 has fixed size)
        }

        // Cluster-chain directory (FAT32 root or any subdirectory)
        let start = if parent_cluster == 0 { self.root_cluster } else { parent_cluster };
        let mut cluster = start;
        let mut last_cluster = cluster;

        loop {
            let lba_base = self.cluster_to_lba(cluster);
            for sec in 0..self.sectors_per_cluster as u32 {
                let lba = lba_base + sec as u64;
                if !self.read_block(lba, &mut sector_buf[..bps], ctx) {
                    return None;
                }
                for i in 0..entries_per_sector {
                    let off = i * 32;
                    if sector_buf[off] == 0x00 || sector_buf[off] == 0xE5 {
                        Self::format_dir_entry(&mut sector_buf, off, &name_8_3, attr, initial_cluster, 0);
                        if !self.write_block(lba, &sector_buf[..bps], ctx) {
                            return None;
                        }
                        return Some((lba, off as u16));
                    }
                }
            }
            last_cluster = cluster;
            match self.next_cluster(cluster, ctx) {
                Some(c) => cluster = c,
                None => break,
            }
        }

        // No free slot found — extend the directory by one cluster
        let new_cluster = self.alloc_cluster(ctx)?;
        if !self.extend_chain(last_cluster, new_cluster, ctx) {
            return None;
        }
        if !self.flush_fat(ctx) {
            return None;
        }

        // Zero-fill the new cluster
        sector_buf[..bps].fill(0);
        let lba_base = self.cluster_to_lba(new_cluster);
        for sec in 0..self.sectors_per_cluster as u32 {
            if !self.write_block(lba_base + sec as u64, &sector_buf[..bps], ctx) {
                return None;
            }
        }

        // Write entry at first slot of new cluster
        Self::format_dir_entry(&mut sector_buf, 0, &name_8_3, attr, initial_cluster, 0);
        if !self.write_block(lba_base, &sector_buf[..bps], ctx) {
            return None;
        }
        Some((lba_base, 0))
    }

    /// Update the file size in the on-disk directory entry.
    fn update_dir_entry_size(&self, lba: u64, offset: u16, new_size: u32, ctx: &mut dyn BusCtx) -> bool {
        let bps = self.bytes_per_sector as usize;
        let mut sector_buf = [0u8; 512];
        if !self.read_block(lba, &mut sector_buf[..bps], ctx) {
            return false;
        }
        let off = offset as usize;
        sector_buf[off+28..off+32].copy_from_slice(&new_size.to_le_bytes());
        self.write_block(lba, &sector_buf[..bps], ctx)
    }

    /// Update the start cluster in the on-disk directory entry.
    fn update_dir_entry_cluster(&self, lba: u64, offset: u16, cluster: u32, ctx: &mut dyn BusCtx) -> bool {
        let bps = self.bytes_per_sector as usize;
        let mut sector_buf = [0u8; 512];
        if !self.read_block(lba, &mut sector_buf[..bps], ctx) {
            return false;
        }
        let off = offset as usize;
        // High 16 bits at offset 20-21
        sector_buf[off+20..off+22].copy_from_slice(&((cluster >> 16) as u16).to_le_bytes());
        // Low 16 bits at offset 26-27
        sector_buf[off+26..off+28].copy_from_slice(&(cluster as u16).to_le_bytes());
        self.write_block(lba, &sector_buf[..bps], ctx)
    }

    /// Mark a directory entry as deleted (0xE5) and free its cluster chain.
    fn delete_dir_entry(&mut self, lba: u64, offset: u16, cluster: u32, ctx: &mut dyn BusCtx) -> bool {
        let bps = self.bytes_per_sector as usize;
        let mut sector_buf = [0u8; 512];
        if !self.read_block(lba, &mut sector_buf[..bps], ctx) {
            return false;
        }
        sector_buf[offset as usize] = 0xE5;
        if !self.write_block(lba, &sector_buf[..bps], ctx) {
            return false;
        }
        if cluster >= 2 {
            if !self.free_chain(cluster, ctx) {
                return false;
            }
            if !self.flush_fat(ctx) {
                return false;
            }
        }
        true
    }

    // =========================================================================
    // Root directory search
    // =========================================================================

    /// Search root directory for a file/dir by name (case-insensitive 8.3 match).
    /// Returns (is_dir, start_cluster, file_size, entry_lba, entry_offset).
    fn find_in_root(
        &mut self, name: &[u8], ctx: &mut dyn BusCtx,
    ) -> Option<(bool, u32, u32, u64, u16)> {
        let mut sector_buf = [0u8; 512];
        let bps = self.bytes_per_sector as usize;
        let entries_per_sector = bps / 32;

        if self.is_fat32 {
            let mut cluster = self.root_cluster;
            loop {
                let base_lba = self.cluster_to_lba(cluster);
                for sec in 0..self.sectors_per_cluster as u32 {
                    let lba = base_lba + sec as u64;
                    if !self.read_block(lba, &mut sector_buf[..bps], ctx) {
                        return None;
                    }
                    for i in 0..entries_per_sector {
                        let offset = i * 32;
                        match self.parse_dir_entry(&sector_buf, offset) {
                            Some((_, 0, _, _, _)) => continue,
                            Some((entry_name, entry_len, is_dir, cl, size)) => {
                                if name_eq_ci(&entry_name[..entry_len], name) {
                                    return Some((is_dir, cl, size, lba, offset as u16));
                                }
                            }
                            None => return None,
                        }
                    }
                }
                match self.next_cluster(cluster, ctx) {
                    Some(c) => cluster = c,
                    None => break,
                }
            }
        } else {
            for sec in 0..self.root_dir_sectors {
                let lba = self.root_dir_start_lba as u64 + sec as u64;
                if !self.read_block(lba, &mut sector_buf[..bps], ctx) {
                    return None;
                }
                for i in 0..entries_per_sector {
                    let offset = i * 32;
                    match self.parse_dir_entry(&sector_buf, offset) {
                        Some((_, 0, _, _, _)) => continue,
                        Some((entry_name, entry_len, is_dir, cluster, size)) => {
                            if name_eq_ci(&entry_name[..entry_len], name) {
                                return Some((is_dir, cluster, size, lba, offset as u16));
                            }
                        }
                        None => return None,
                    }
                }
            }
        }
        None
    }

    // =========================================================================
    // Completion helpers
    // =========================================================================

    fn complete_vfs_error(ctx: &mut dyn BusCtx, port_id: PortId, tag: u32, error: u32) {
        if let Some(port) = ctx.block_port(port_id) {
            port.complete_error_with_result(tag, io_status::IO_ERROR as u16, error);
            port.notify();
        }
    }

    fn complete_vfs_result(ctx: &mut dyn BusCtx, port_id: PortId, tag: u32, result: u32, transferred: u32) {
        if let Some(port) = ctx.block_port(port_id) {
            port.complete_with_result(tag, result, transferred);
            port.notify();
        }
    }

    // =========================================================================
    // VFS ring request processing
    // =========================================================================

    fn process_vfs_requests(&mut self, ctx: &mut dyn BusCtx) {
        let vfs_id = match self.vfs_port {
            Some(id) => id,
            None => return,
        };

        // Collect pending requests
        let mut requests: [Option<IoSqe>; 8] = [None; 8];
        let mut req_count = 0;

        if let Some(port) = ctx.block_port(vfs_id) {
            while req_count < 8 {
                if let Some(sqe) = port.recv_request() {
                    requests[req_count] = Some(sqe);
                    req_count += 1;
                } else {
                    break;
                }
            }
        }

        // Process each request
        for i in 0..req_count {
            if let Some(sqe) = requests[i].take() {
                match sqe.opcode {
                    fs_op::OPEN => self.handle_vfs_open(&sqe, ctx),
                    fs_op::READ => self.handle_vfs_read(&sqe, ctx),
                    fs_op::WRITE => self.handle_vfs_write(&sqe, ctx),
                    fs_op::READDIR => self.handle_vfs_readdir(&sqe, ctx),
                    fs_op::STAT => self.handle_vfs_stat(&sqe, ctx),
                    fs_op::CLOSE => self.handle_vfs_close(&sqe, ctx),
                    fs_op::MKDIR => self.handle_vfs_mkdir(&sqe, ctx),
                    fs_op::UNLINK => self.handle_vfs_unlink(&sqe, ctx),
                    _ => {
                        Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                    }
                }
            }
        }

        // Process sidechannel queries
        if let Some(port) = ctx.block_port(vfs_id) {
            while let Some(entry) = port.poll_side_request() {
                use libos::ring::side_msg;
                match entry.msg_type {
                    side_msg::QUERY_GEOMETRY => {
                        // Not a block device - return error
                        let mut eol = entry;
                        eol.status = libos::ring::side_status::EOL;
                        port.notify();
                    }
                    _ => {
                        port.notify();
                    }
                }
            }
        }
    }

    // =========================================================================
    // Info formatting (for QUERY_INFO)
    // =========================================================================

    /// Core initialization: connect to partition, read FAT16, create VFS port.
    ///
    /// Called from reset() via spawn context discovery.
    fn do_init(&mut self, shmem_id: u32, source_name: &[u8], ctx: &mut dyn BusCtx) -> bool {
        // Connect to partition DataPort
        match ctx.connect_block_port(shmem_id) {
            Ok(port_id) => {
                self.consumer_port = Some(port_id);

                // Query geometry
                if let Some(port) = ctx.block_port(port_id) {
                    if let Some(geo) = port.query_geometry() {
                        self.bytes_per_sector = geo.block_size as u16;
                    }
                }

                // Read BPB (sector 0)
                let mut bpb_buf = [0u8; 512];
                if !self.read_block(0, &mut bpb_buf, ctx) {
                    uerror!("fatfsd", "bpb_read_failed";);
                    return false;
                }

                if !self.parse_bpb(&bpb_buf) {
                    uerror!("fatfsd", "invalid_bpb";);
                    return false;
                }
                // Cache initial FAT window (starting at cluster 0)
                if !self.cache_fat_window(0, ctx) {
                    uerror!("fatfsd", "fat_cache_failed";);
                    return false;
                }

                // Create VFS DataPort (provider)
                let config = BlockPortConfig {
                    ring_size: 64,
                    side_size: 8,
                    pool_size: 256 * 1024,
                };

                match ctx.create_block_port(config) {
                    Ok(port_id) => {
                        if let Some(port) = ctx.block_port(port_id) {
                            port.set_public();
                            let vfs_shmem_id = port.shmem_id();
                            self.vfs_port = Some(port_id);

                            // Simple local port name — bus_runtime prepends the
                            // full parent path (e.g. block:0/fat:0/mnt:0).
                            let pname = b"mnt:0";
                            let pname_len = pname.len();

                            self.port_name[..pname_len].copy_from_slice(&pname[..pname_len]);
                            self.port_name_len = pname_len;

                            // Register as Filesystem port with devd using unified PortInfo
                            let mut info = PortInfo::new(&pname[..pname_len], PortClass::Filesystem);
                            info.port_subclass = port_subclass::FS_FAT;
                            if let Err(_) = ctx.register_port_with_info(&info, vfs_shmem_id) {
                                uerror!("fatfsd", "port_register_failed"; shmem_id = vfs_shmem_id);
                            }

                            // Register mount with devd2.
                            // Use mount.path from config if set, otherwise fallback
                            // to heuristic from source_name.
                            let mut mount_prefix = [0u8; 65];
                            let prefix_len = if self.port_name_len > 0 {
                                // Config-set mount path (e.g., "/mnt/nvme")
                                let n = self.port_name_len.min(64);
                                mount_prefix[..n].copy_from_slice(&self.port_name[..n]);
                                let mut pos = n;
                                // Ensure trailing slash
                                if pos > 0 && mount_prefix[pos - 1] != b'/' && pos < 65 {
                                    mount_prefix[pos] = b'/';
                                    pos += 1;
                                }
                                pos
                            } else {
                                // Fallback: /mnt/<source_name>/
                                let mut pos = 0;
                                mount_prefix[pos] = b'/'; pos += 1;
                                let m = b"mnt/";
                                mount_prefix[pos..pos + m.len()].copy_from_slice(m);
                                pos += m.len();
                                let clen = source_name.len().min(65 - pos - 1);
                                mount_prefix[pos..pos + clen].copy_from_slice(&source_name[..clen]);
                                pos += clen;
                                if pos < 65 && mount_prefix[pos - 1] != b'/' {
                                    mount_prefix[pos] = b'/'; pos += 1;
                                }
                                pos
                            };
                            if let Err(_) = ctx.register_mount(&mount_prefix[..prefix_len], vfs_shmem_id) {
                                uerror!("fatfsd", "mount_register_failed";);
                            }
                            return true;
                        }
                    }
                    Err(e) => {
                        uerror!("fatfsd", "vfs_port_create_failed";);
                    }
                }
            }
            Err(e) => {
                uerror!("fatfsd", "partition_connect_failed";);
            }
        }
        false
    }


    fn format_info(&self) -> [u8; 256] {
        let mut buf = [0u8; 256];
        let mut pos = 0;

        let append = |buf: &mut [u8], pos: &mut usize, s: &[u8]| {
            let len = s.len().min(buf.len() - *pos);
            buf[*pos..*pos + len].copy_from_slice(&s[..len]);
            *pos += len;
        };

        if self.is_fat32 {
            append(&mut buf, &mut pos, b"FAT32 Filesystem Driver\n");
        } else {
            append(&mut buf, &mut pos, b"FAT16 Filesystem Driver\n");
        }
        append(&mut buf, &mut pos, b"  Port: ");
        append(&mut buf, &mut pos, &self.port_name[..self.port_name_len]);
        append(&mut buf, &mut pos, b"\n  Cluster size: ");
        let mut num_buf = [0u8; 16];
        let n = format_u32(&mut num_buf, self.cluster_size());
        append(&mut buf, &mut pos, &num_buf[..n]);
        append(&mut buf, &mut pos, b" bytes\n  Root entries: ");
        let n = format_u32(&mut num_buf, self.root_entry_count as u32);
        append(&mut buf, &mut pos, &num_buf[..n]);
        append(&mut buf, &mut pos, b"\n");

        buf
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for FatfsDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // Use spawn context to discover which partition we were spawned for
        let spawn_ctx = ctx.spawn_context().map_err(|_| {
            uerror!("fatfsd", "no_spawn_context";);
            BusError::Internal
        })?;

        let port_name = spawn_ctx.port_name();

        // Copy port name before borrowing ctx again
        let mut name_buf = [0u8; 64];
        let name_len = port_name.len().min(64);
        name_buf[..name_len].copy_from_slice(&port_name[..name_len]);

        // Discover partition shmem_id via devd (uses port_id from spawn context)
        let shmem_id = ctx.discover_port().map_err(|_| {
            uerror!("fatfsd", "discover_partition_failed";);
            BusError::Internal
        })?;

        if !self.do_init(shmem_id, &name_buf[..name_len], ctx) {
            uerror!("fatfsd", "init_failed";);
            return Err(BusError::Internal);
        }

        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                let info = self.format_info();
                let info_len = info.iter().rposition(|&b| b != 0).map(|p| p + 1).unwrap_or(0);
                let _ = ctx.respond_info(msg.seq_id, &info[..info_len]);
                Disposition::Handled
            }

            _ => Disposition::Forward,
        }
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        if self.vfs_port == Some(port) {
            self.process_vfs_requests(ctx);
        }
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], _ctx: &mut dyn BusCtx) -> usize {
        match key {
            b"mount.path" => {
                let len = value.len().min(self.port_name.len());
                self.port_name[..len].copy_from_slice(&value[..len]);
                self.port_name_len = len;
                copy_to_buf(buf, 0, b"OK")
            }
            _ => 0,
        }
    }

    fn config_keys(&self) -> &[ConfigKey] {
        FAT_CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        match key {
            b"fs.type" => copy_to_buf(buf, 0, if self.is_fat32 { b"FAT32" as &[u8] } else { b"FAT16" }),
            b"fs.cluster_size" => {
                let mut tmp = [0u8; 16];
                let len = format_u32(&mut tmp, self.cluster_size());
                copy_to_buf(buf, 0, &tmp[..len])
            }
            b"fs.total_sectors" => {
                let mut tmp = [0u8; 16];
                let len = format_u32(&mut tmp, self.total_sectors);
                copy_to_buf(buf, 0, &tmp[..len])
            }
            b"fs.sectors_per_cluster" => {
                let mut tmp = [0u8; 16];
                let len = format_u32(&mut tmp, self.sectors_per_cluster as u32);
                copy_to_buf(buf, 0, &tmp[..len])
            }
            b"fs.fat_copies" => {
                let mut tmp = [0u8; 16];
                let len = format_u32(&mut tmp, self.num_fats as u32);
                copy_to_buf(buf, 0, &tmp[..len])
            }
            b"fs.readonly" => copy_to_buf(buf, 0, b"false"),
            b"mount.path" => copy_to_buf(buf, 0, &self.port_name[..self.port_name_len]),
            b"stats.open_files" => {
                let count = self.open_files.iter().filter(|f| f.in_use).count() as u32;
                let mut tmp = [0u8; 16];
                let len = format_u32(&mut tmp, count);
                copy_to_buf(buf, 0, &tmp[..len])
            }
            _ => 0,
        }
    }
}

const FAT_CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey::read_only(b"fs.type"),
    ConfigKey::read_only(b"fs.cluster_size"),
    ConfigKey::read_only(b"fs.total_sectors"),
    ConfigKey::read_only(b"fs.sectors_per_cluster"),
    ConfigKey::read_only(b"fs.fat_copies"),
    ConfigKey::read_only(b"fs.readonly"),
    ConfigKey::read_write(b"mount.path"),
    ConfigKey::read_only(b"stats.open_files"),
];

fn copy_to_buf(buf: &mut [u8], pos: usize, src: &[u8]) -> usize {
    let len = src.len().min(buf.len().saturating_sub(pos));
    buf[pos..pos + len].copy_from_slice(&src[..len]);
    pos + len
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: FatfsDriver = FatfsDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"fatfsd", driver);
}

// =============================================================================
// Block I/O (freestanding — no &self, avoids borrow conflicts with scratch buffers)
// =============================================================================

/// Read contiguous sectors into `buf` via a single ring submission.
fn read_sectors(port_id: PortId, lba: u64, buf: &mut [u8], ctx: &mut dyn BusCtx) -> bool {
    let port = match ctx.block_port(port_id) {
        Some(p) => p,
        None => return false,
    };

    let len = buf.len() as u32;
    let offset = match port.alloc(len) {
        Some(o) => o,
        None => {
            uerror!("fatfsd", "read_alloc_failed"; lba = lba, len = len);
            return false;
        }
    };

    let tag = match port.submit_read(lba, offset, len) {
        Ok(t) => t,
        Err(_) => {
            uerror!("fatfsd", "read_submit_failed"; lba = lba, len = len);
            port.free(offset);
            return false;
        }
    };

    port.notify();

    for _ in 0..100 {
        if let Some(cqe) = port.poll_completion() {
            if cqe.tag == tag {
                if cqe.status == io_status::OK as u16 {
                    if let Some(pool_slice) = port.pool_slice(offset, len) {
                        buf.copy_from_slice(pool_slice);
                        port.free(offset);
                        return true;
                    }
                }
                uerror!("fatfsd", "read_status_failed"; lba = lba, status = cqe.status as u32);
                port.free(offset);
                return false;
            }
            // Wrong tag — discard stale completion
            udebug!("fatfsd", "read_stale_cqe"; expected = tag, got = cqe.tag);
        }
        port.wait(10);
    }
    uerror!("fatfsd", "read_timeout"; lba = lba, tag = tag);
    port.free(offset);
    false
}

/// Write contiguous sectors from `data` via a single ring submission.
fn write_sectors(port_id: PortId, lba: u64, data: &[u8], ctx: &mut dyn BusCtx) -> bool {
    let port = match ctx.block_port(port_id) {
        Some(p) => p,
        None => return false,
    };

    let len = data.len() as u32;
    let offset = match port.alloc(len) {
        Some(o) => o,
        None => return false,
    };

    port.pool_write(offset, data);

    let tag = match port.submit_write(lba, offset, len) {
        Ok(t) => t,
        Err(_) => { port.free(offset); return false; }
    };

    port.notify();

    for _ in 0..100 {
        if let Some(cqe) = port.poll_completion() {
            if cqe.tag == tag {
                port.free(offset);
                return cqe.status == io_status::OK as u16;
            }
        }
        port.wait(10);
    }
    port.free(offset);
    false
}

// =============================================================================
// Helpers
// =============================================================================

fn to_lower(c: u8) -> u8 {
    if c >= b'A' && c <= b'Z' { c + 32 } else { c }
}

fn to_upper(c: u8) -> u8 {
    if c >= b'a' && c <= b'z' { c - 32 } else { c }
}

fn name_eq_ci(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    for i in 0..a.len() {
        if to_lower(a[i]) != to_lower(b[i]) {
            return false;
        }
    }
    true
}

fn format_u32(buf: &mut [u8], val: u32) -> usize {
    if val == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut n = val;
    let mut len = 0;
    while n > 0 { len += 1; n /= 10; }
    n = val;
    for i in (0..len).rev() {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    len
}
