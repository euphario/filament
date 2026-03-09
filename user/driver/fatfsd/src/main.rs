//! FAT16/FAT32 Filesystem Driver
//!
//! Read-only FAT16/FAT32 driver that serves the VFS protocol over DataPort.
//! Consumes a partition block port, provides a VFS protocol port.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │  vfsd (connects as consumer)       │
//! │  VFS protocol: OPEN, READ, READDIR │
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
//! 6. Serves VFS requests: OPEN, READ, READDIR, STAT, CLOSE

#![no_std]
#![no_main]

use libsys::syscall;
use libos::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockPortConfig, bus_msg, ConfigKey,
    PortInfo, PortClass, port_subclass,
};
use libos::bus_runtime::driver_main;
use libos::ring::{IoSqe, io_status};
use libos::vfs_proto::{fs_op, open_flags, file_type, vfs_error, VfsDirEntry, VfsStat};
use libos::{uinfo, unotice, udebug, uerror};

const FAT_CACHE_ENTRIES: usize = 8192;
const MAX_OPEN_FILES: usize = 16;

// =============================================================================
// Open file tracking
// =============================================================================

#[derive(Clone, Copy)]
struct OpenFile {
    in_use: bool,
    is_dir: bool,
    start_cluster: u32,
    size: u32,
}

impl OpenFile {
    const fn empty() -> Self {
        Self { in_use: false, is_dir: false, start_cluster: 0, size: 0 }
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

    // Open files
    open_files: [OpenFile; MAX_OPEN_FILES],

    // Port name derived from partition
    port_name: [u8; 32],
    port_name_len: usize,
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
            open_files: [OpenFile::empty(); MAX_OPEN_FILES],
            port_name: [0; 32],
            port_name_len: 0,
        }
    }

    // =========================================================================
    // Block I/O helper (same pattern as partd)
    // =========================================================================

    fn read_block(&self, lba: u64, buf: &mut [u8], ctx: &mut dyn BusCtx) -> bool {
        let port_id = match self.consumer_port {
            Some(id) => id,
            None => return false,
        };

        let port = match ctx.block_port(port_id) {
            Some(p) => p,
            None => return false,
        };

        let len = buf.len() as u32;
        let offset = match port.alloc(len) {
            Some(o) => o,
            None => return false,
        };

        let tag = match port.submit_read(lba, offset, len) {
            Ok(t) => t,
            Err(_) => return false,
        };

        port.notify();

        // Wait for completion (kernel-backed wait, yields to scheduler)
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
                    port.free(offset);
                    return false;
                }
            }
            port.wait(10);
        }
        port.free(offset);
        false
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
    fn cache_fat_window(&mut self, base_cluster: u32, ctx: &mut dyn BusCtx) -> bool {
        let entry_size: u32 = if self.is_fat32 { 4 } else { 2 };
        let bps = self.bytes_per_sector as u32;
        let entries_per_sector = bps / entry_size;

        // Calculate total entries in the FAT
        let fat_bytes = self.fat_size_sectors * bps;
        let total_fat_entries = fat_bytes / entry_size;
        let entries_to_cache = (total_fat_entries - base_cluster).min(FAT_CACHE_ENTRIES as u32) as usize;

        // Calculate which FAT sector contains base_cluster
        let byte_offset = base_cluster * entry_size;
        let first_fat_sector = byte_offset / bps;
        let offset_in_first_sector = (byte_offset % bps) as usize;

        let mut sector_buf = [0u8; 512];
        let mut cached = 0;

        let sectors_needed = ((entries_to_cache as u32 * entry_size + bps - 1) / bps) + 1;
        for s in 0..sectors_needed {
            if cached >= entries_to_cache {
                break;
            }
            let sector_idx = first_fat_sector + s;
            if sector_idx >= self.fat_size_sectors {
                break;
            }

            let lba = self.fat_start_lba as u64 + sector_idx as u64;
            if !self.read_block(lba, &mut sector_buf[..bps as usize], ctx) {
                uerror!("fatfsd", "fat_read_failed"; sector = sector_idx);
                return false;
            }

            let start = if s == 0 { offset_in_first_sector } else { 0 };
            let mut off = start;
            while off + entry_size as usize <= bps as usize && cached < entries_to_cache {
                if self.is_fat32 {
                    let val = u32::from_le_bytes([
                        sector_buf[off], sector_buf[off + 1],
                        sector_buf[off + 2], sector_buf[off + 3],
                    ]);
                    self.fat_cache[cached] = val & 0x0FFF_FFFF;
                } else {
                    let val = u16::from_le_bytes([sector_buf[off], sector_buf[off + 1]]);
                    self.fat_cache[cached] = val as u32;
                }
                cached += 1;
                off += entry_size as usize;
            }
        }

        self.fat_cache_base = base_cluster;
        self.fat_cache_valid = true;
        udebug!("fatfsd", "fat_cached"; base = base_cluster, entries = cached as u32);
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

        // Search root directory for the file
        match self.find_in_root(path, ctx) {
            Some((is_dir, start_cluster, file_size)) => {
                if is_dir_open && !is_dir {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_DIR);
                    return;
                }
                if let Some(handle) = self.alloc_handle() {
                    self.open_files[handle as usize].is_dir = is_dir;
                    self.open_files[handle as usize].start_cluster = start_cluster;
                    self.open_files[handle as usize].size = file_size;
                    Self::complete_vfs_result(ctx, vfs_id, sqe.tag, handle, 0);
                } else {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::TOO_MANY);
                }
            }
            None => {
                Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::NOT_FOUND);
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

        // Read data cluster by cluster
        let mut total_read = 0u32;
        let mut cluster_offset = offset_in_cluster;
        let mut sector_buf = [0u8; 512];

        while total_read < to_read {
            let lba = self.cluster_to_lba(cluster);
            let sectors_in_cluster = self.sectors_per_cluster as u32;

            let sector_in_cluster = cluster_offset / self.bytes_per_sector as u32;
            let offset_in_sector = cluster_offset % self.bytes_per_sector as u32;

            for sec in sector_in_cluster..sectors_in_cluster {
                if total_read >= to_read {
                    break;
                }

                let sec_lba = lba + sec as u64;
                if !self.read_block(sec_lba, &mut sector_buf[..self.bytes_per_sector as usize], ctx) {
                    Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::IO_ERROR);
                    return;
                }

                let start = if sec == sector_in_cluster { offset_in_sector as usize } else { 0 };
                let available = self.bytes_per_sector as usize - start;
                let needed = (to_read - total_read) as usize;
                let copy_len = available.min(needed);

                // Copy to VFS pool
                if let Some(port) = ctx.block_port(vfs_id) {
                    let dst_offset = buf_offset + total_read;
                    port.pool_write(dst_offset, &sector_buf[start..start + copy_len]);
                }

                total_read += copy_len as u32;
            }

            cluster_offset = 0; // Only first cluster has an offset

            // Next cluster
            if total_read < to_read {
                match self.next_cluster(cluster, ctx) {
                    Some(c) => cluster = c,
                    None => break, // End of chain
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

    // =========================================================================
    // Root directory search
    // =========================================================================

    /// Search root directory for a file/dir by name (case-insensitive 8.3 match).
    /// Returns (is_dir, start_cluster, file_size).
    fn find_in_root(&mut self, name: &[u8], ctx: &mut dyn BusCtx) -> Option<(bool, u32, u32)> {
        let mut sector_buf = [0u8; 512];

        if self.is_fat32 {
            // FAT32: root directory is a cluster chain
            let mut cluster = self.root_cluster;
            loop {
                let lba = self.cluster_to_lba(cluster);
                for sec in 0..self.sectors_per_cluster as u32 {
                    if !self.read_block(lba + sec as u64, &mut sector_buf[..self.bytes_per_sector as usize], ctx) {
                        return None;
                    }
                    let entries_per_sector = self.bytes_per_sector as usize / 32;
                    for i in 0..entries_per_sector {
                        let offset = i * 32;
                        match self.parse_dir_entry(&sector_buf, offset) {
                            Some((_, 0, _, _, _)) => continue,
                            Some((entry_name, entry_len, is_dir, cl, size)) => {
                                if name_eq_ci(&entry_name[..entry_len], name) {
                                    return Some((is_dir, cl, size));
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
            // FAT16: fixed root directory region
            for sec in 0..self.root_dir_sectors {
                let lba = self.root_dir_start_lba as u64 + sec as u64;
                if !self.read_block(lba, &mut sector_buf[..self.bytes_per_sector as usize], ctx) {
                    return None;
                }
                let entries_per_sector = self.bytes_per_sector as usize / 32;
                for i in 0..entries_per_sector {
                    let offset = i * 32;
                    match self.parse_dir_entry(&sector_buf, offset) {
                        Some((_, 0, _, _, _)) => continue,
                        Some((entry_name, entry_len, is_dir, cluster, size)) => {
                            if name_eq_ci(&entry_name[..entry_len], name) {
                                return Some((is_dir, cluster, size));
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
                    fs_op::READDIR => self.handle_vfs_readdir(&sqe, ctx),
                    fs_op::STAT => self.handle_vfs_stat(&sqe, ctx),
                    fs_op::CLOSE => self.handle_vfs_close(&sqe, ctx),
                    _ => {
                        // Unsupported op (WRITE, MKDIR, etc. - read only)
                        Self::complete_vfs_error(ctx, vfs_id, sqe.tag, vfs_error::READ_ONLY);
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
    fn do_init(&mut self, shmem_id: u32, source_name: &[u8], mount_path: Option<&[u8]>, ctx: &mut dyn BusCtx) -> bool {
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

                            // Derive mount name from mount_path context (preferred)
                            // or fall back to old heuristic: "part0:" → "mnt/part0:"
                            let mut pname = [0u8; 32];
                            let pname_len = if let Some(path) = mount_path {
                                let copy_len = path.len().min(31);
                                pname[..copy_len].copy_from_slice(&path[..copy_len]);
                                let mut pos = copy_len;
                                // Ensure name ends with ':'
                                if pos > 0 && pname[pos - 1] != b':' && pos < 32 {
                                    pname[pos] = b':';
                                    pos += 1;
                                }
                                pos
                            } else {
                                let prefix = b"mnt/";
                                let mut pos = prefix.len();
                                pname[..pos].copy_from_slice(prefix);
                                let copy_len = source_name.len().min(32 - pos);
                                pname[pos..pos + copy_len].copy_from_slice(&source_name[..copy_len]);
                                pos += copy_len;
                                // Ensure name ends with ':'
                                if pos > 0 && pname[pos - 1] != b':' && pos < 32 {
                                    pname[pos] = b':';
                                    pos += 1;
                                }
                                pos
                            };

                            self.port_name[..pname_len].copy_from_slice(&pname[..pname_len]);
                            self.port_name_len = pname_len;

                            // Register as Filesystem port with devd using unified PortInfo
                            let mut info = PortInfo::new(&pname[..pname_len], PortClass::Filesystem);
                            info.port_subclass = port_subclass::FS_FAT;
                            if let Err(_) = ctx.register_port_with_info(&info, vfs_shmem_id) {
                                uerror!("fatfsd", "port_register_failed"; shmem_id = vfs_shmem_id);
                            }

                            // Register mount with devd so clients can resolve paths
                            // Strip trailing ':' from port name for mount prefix,
                            // and prepend '/' for a proper path prefix
                            let clean_name = if pname_len > 0 && pname[pname_len - 1] == b':' {
                                &pname[..pname_len - 1]
                            } else {
                                &pname[..pname_len]
                            };
                            let mut mount_prefix = [0u8; 65];
                            mount_prefix[0] = b'/';
                            let clen = clean_name.len().min(64);
                            mount_prefix[1..1 + clen].copy_from_slice(&clean_name[..clen]);
                            let prefix_len = 1 + clen;
                            // Ensure trailing slash
                            let prefix_len = if prefix_len < 65 && mount_prefix[prefix_len - 1] != b'/' {
                                mount_prefix[prefix_len] = b'/';
                                prefix_len + 1
                            } else {
                                prefix_len
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
            append(&mut buf, &mut pos, b"FAT32 Filesystem Driver (read-only)\n");
        } else {
            append(&mut buf, &mut pos, b"FAT16 Filesystem Driver (read-only)\n");
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

        // Check for mount.path from context KV (set by rule template expansion)
        let mut mount_path_buf = [0u8; 64];
        let mount_path_len = if let Some(path) = spawn_ctx.get(b"mount.path") {
            let l = path.len().min(64);
            mount_path_buf[..l].copy_from_slice(&path[..l]);
            l
        } else { 0 };

        // Discover partition shmem_id via devd (uses port_id from spawn context)
        let shmem_id = ctx.discover_port().map_err(|_| {
            uerror!("fatfsd", "discover_partition_failed";);
            BusError::Internal
        })?;

        let mount_path = if mount_path_len > 0 {
            Some(&mount_path_buf[..mount_path_len])
        } else {
            None
        };

        if !self.do_init(shmem_id, &name_buf[..name_len], mount_path, ctx) {
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

    fn config_keys(&self) -> &[ConfigKey] {
        FAT_CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        match key {
            b"type" => copy_to_buf(buf, 0, if self.is_fat32 { b"FAT32" as &[u8] } else { b"FAT16" }),
            b"mount" => copy_to_buf(buf, 0, &self.port_name[..self.port_name_len]),
            b"cluster_size" => {
                let mut tmp = [0u8; 16];
                let len = format_u32(&mut tmp, self.cluster_size());
                copy_to_buf(buf, 0, &tmp[..len])
            }
            b"total_sectors" => {
                let mut tmp = [0u8; 16];
                let len = format_u32(&mut tmp, self.total_sectors);
                copy_to_buf(buf, 0, &tmp[..len])
            }
            b"open_files" => {
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
    ConfigKey::read_only(b"type"),
    ConfigKey::read_only(b"mount"),
    ConfigKey::read_only(b"cluster_size"),
    ConfigKey::read_only(b"total_sectors"),
    ConfigKey::read_only(b"open_files"),
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
// Helpers
// =============================================================================

fn to_lower(c: u8) -> u8 {
    if c >= b'A' && c <= b'Z' { c + 32 } else { c }
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
