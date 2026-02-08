//! FAT16 Filesystem Driver
//!
//! Read-only FAT16 driver that serves the VFS protocol over DataPort.
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
//! 3. Reads BPB at sector 0, validates FAT16
//! 4. Caches FAT table
//! 5. Creates VFS DataPort (provider), registers as Filesystem port
//! 6. Serves VFS requests: OPEN, READ, READDIR, STAT, CLOSE

#![no_std]
#![no_main]

use userlib::syscall;
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockPortConfig, bus_msg,
    PortInfo, PortClass, port_subclass,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::{IoSqe, SideEntry, io_status, side_msg, side_status};
use userlib::vfs_proto::{fs_op, open_flags, file_type, vfs_error, VfsDirEntry, VfsStat};
use userlib::{uinfo, uerror};

const FAT_CACHE_ENTRIES: usize = 8192;
const MAX_OPEN_FILES: usize = 16;

// =============================================================================
// Open file tracking
// =============================================================================

#[derive(Clone, Copy)]
struct OpenFile {
    in_use: bool,
    is_dir: bool,
    start_cluster: u16,
    size: u32,
}

impl OpenFile {
    const fn empty() -> Self {
        Self { in_use: false, is_dir: false, start_cluster: 0, size: 0 }
    }
}

// =============================================================================
// FAT16 Driver
// =============================================================================

struct FatfsDriver {
    /// Consumer port (block reads from partition)
    consumer_port: Option<PortId>,
    /// VFS protocol provider port
    vfs_port: Option<PortId>,

    // FAT16 BPB fields
    bytes_per_sector: u16,
    sectors_per_cluster: u8,
    reserved_sectors: u16,
    num_fats: u8,
    root_entry_count: u16,
    total_sectors: u32,
    fat_size_sectors: u16,

    // Derived layout
    fat_start_lba: u32,
    root_dir_start_lba: u32,
    root_dir_sectors: u32,
    data_start_lba: u32,

    // FAT cache (2 bytes per entry for FAT16)
    fat_cache: [u16; FAT_CACHE_ENTRIES],
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
            fat_start_lba: 0,
            root_dir_start_lba: 0,
            root_dir_sectors: 0,
            data_start_lba: 0,
            fat_cache: [0u16; FAT_CACHE_ENTRIES],
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
                            return true;
                        }
                    }
                    return false;
                }
            }
            port.wait(10);
        }
        false
    }

    // =========================================================================
    // FAT16 initialization
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

        self.fat_size_sectors = u16::from_le_bytes([sector[22], sector[23]]);

        // Validate FAT16
        if self.bytes_per_sector == 0 || self.sectors_per_cluster == 0 || self.num_fats == 0 {
            uerror!("fatfsd", "invalid_bpb_zero";);
            return false;
        }

        if self.fat_size_sectors == 0 {
            uerror!("fatfsd", "invalid_bpb_fat32";);
            return false;
        }

        // Compute layout
        self.fat_start_lba = self.reserved_sectors as u32;
        self.root_dir_start_lba = self.fat_start_lba + (self.num_fats as u32 * self.fat_size_sectors as u32);
        self.root_dir_sectors = ((self.root_entry_count as u32 * 32) + self.bytes_per_sector as u32 - 1)
            / self.bytes_per_sector as u32;
        self.data_start_lba = self.root_dir_start_lba + self.root_dir_sectors;

        // Validate cluster count is FAT16 range (4085..65525)
        let data_sectors = self.total_sectors.saturating_sub(self.data_start_lba);
        let cluster_count = data_sectors / self.sectors_per_cluster as u32;
        if cluster_count < 4085 || cluster_count >= 65525 {
            uerror!("fatfsd", "not_fat16"; clusters = cluster_count);
            return false;
        }

        uinfo!("fatfsd", "fat16_parsed";
            bps = self.bytes_per_sector as u32,
            spc = self.sectors_per_cluster as u32,
            root_entries = self.root_entry_count as u32,
            clusters = cluster_count);

        true
    }

    fn cache_fat(&mut self, ctx: &mut dyn BusCtx) -> bool {
        let fat_bytes = self.fat_size_sectors as u32 * self.bytes_per_sector as u32;
        let entries_to_cache = (fat_bytes / 2).min(FAT_CACHE_ENTRIES as u32) as usize;

        // Read FAT sector by sector
        let mut sector_buf = [0u8; 512];
        let entries_per_sector = self.bytes_per_sector as usize / 2;

        let mut cached = 0;
        for sector_offset in 0..self.fat_size_sectors as u32 {
            if cached >= entries_to_cache {
                break;
            }

            let lba = self.fat_start_lba as u64 + sector_offset as u64;
            if !self.read_block(lba, &mut sector_buf[..self.bytes_per_sector as usize], ctx) {
                uerror!("fatfsd", "fat_read_failed"; sector = sector_offset);
                return false;
            }

            for i in 0..entries_per_sector {
                if cached >= entries_to_cache {
                    break;
                }
                let offset = i * 2;
                self.fat_cache[cached] = u16::from_le_bytes([
                    sector_buf[offset], sector_buf[offset + 1],
                ]);
                cached += 1;
            }
        }

        self.fat_cache_valid = true;
        uinfo!("fatfsd", "fat_cached"; entries = cached as u32);
        true
    }

    fn next_cluster(&self, cluster: u16) -> Option<u16> {
        if !self.fat_cache_valid || cluster as usize >= FAT_CACHE_ENTRIES {
            return None;
        }
        let next = self.fat_cache[cluster as usize];
        if next >= 0xFFF8 {
            None // End of chain
        } else if next < 2 {
            None // Invalid
        } else {
            Some(next)
        }
    }

    fn cluster_to_lba(&self, cluster: u16) -> u64 {
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
    fn parse_dir_entry(&self, data: &[u8], offset: usize) -> Option<([u8; 12], usize, bool, u16, u32)> {
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
        let start_cluster = u16::from_le_bytes([entry[26], entry[27]]);
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
            match self.next_cluster(cluster) {
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
                match self.next_cluster(cluster) {
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

        if file.start_cluster == 0 {
            // Root directory - read from root_dir_start_lba
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
            // Subdirectory - follow cluster chain
            let mut cluster = file.start_cluster;
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

                match self.next_cluster(cluster) {
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
    fn find_in_root(&self, name: &[u8], ctx: &mut dyn BusCtx) -> Option<(bool, u16, u32)> {
        let mut sector_buf = [0u8; 512];

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
                    None => return None, // End of directory
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
                use userlib::ring::side_msg;
                match entry.msg_type {
                    side_msg::QUERY_GEOMETRY => {
                        // Not a block device - return error
                        let mut eol = entry;
                        eol.status = userlib::ring::side_status::EOL;
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
    /// Called from init() (spawn context path) or from ATTACH_DISK command.
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
                    uerror!("fatfsd", "invalid_fat16";);
                    return false;
                }
                // Cache FAT
                if !self.cache_fat(ctx) {
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

                            // Derive mount name from partition port name
                            // e.g., "part0:" → "mnt/part0:"
                            let mut pname = [0u8; 32];
                            let pname_len = {
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
                            let _ = ctx.register_port_with_info(&info, vfs_shmem_id);

                            uinfo!("fatfsd", "vfs_port_registered"; shmem_id = vfs_shmem_id);

                            // Register mount with vfsd via side-channel
                            self.register_mount_with_vfsd(vfs_shmem_id, &pname[..pname_len], ctx);
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

    /// Register this mount with vfsd by sending REGISTER_MOUNT via side-channel.
    ///
    /// Discovers vfsd's DataPort via devd-query, connects, and sends the
    /// REGISTER_MOUNT side-channel entry with our VFS DataPort shmem_id
    /// and mount name.
    fn register_mount_with_vfsd(&mut self, vfs_shmem_id: u32, port_name: &[u8], ctx: &mut dyn BusCtx) {
        // Discover vfsd's DataPort shmem_id
        let vfsd_shmem_id = match ctx.discover_port(b"vfs:") {
            Ok(id) => id,
            Err(e) => {
                uerror!("fatfsd", "discover_vfs_failed";);
                return;
            }
        };

        // Connect to vfsd's DataPort
        let vfsd_port = match ctx.connect_block_port(vfsd_shmem_id) {
            Ok(id) => id,
            Err(e) => {
                uerror!("fatfsd", "vfsd_connect_failed";);
                return;
            }
        };

        // Build REGISTER_MOUNT side-channel entry
        // Payload: [0..4] fs_shmem_id, [4] name_len, [5..24] mount_name
        let mut entry = SideEntry::default();
        entry.msg_type = side_msg::REGISTER_MOUNT;
        entry.status = side_status::REQUEST;
        entry.payload[0..4].copy_from_slice(&vfs_shmem_id.to_le_bytes());

        // Strip trailing ":" from port name for mount name
        let clean_name = if !port_name.is_empty() && port_name[port_name.len() - 1] == b':' {
            &port_name[..port_name.len() - 1]
        } else {
            port_name
        };
        let name_len = clean_name.len().min(19);
        entry.payload[4] = name_len as u8;
        entry.payload[5..5 + name_len].copy_from_slice(&clean_name[..name_len]);

        // Send via side-channel
        if let Some(port) = ctx.block_port(vfsd_port) {
            if port.side_send(&entry) {
                port.notify();
                uinfo!("fatfsd", "mount_registered"; shmem_id = vfs_shmem_id);
            } else {
                uerror!("fatfsd", "mount_send_failed";);
            }
        }
    }

    fn format_info(&self) -> [u8; 256] {
        let mut buf = [0u8; 256];
        let mut pos = 0;

        let append = |buf: &mut [u8], pos: &mut usize, s: &[u8]| {
            let len = s.len().min(buf.len() - *pos);
            buf[*pos..*pos + len].copy_from_slice(&s[..len]);
            *pos += len;
        };

        append(&mut buf, &mut pos, b"FAT16 Filesystem Driver (read-only)\n");
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

        // Discover partition shmem_id via devd
        let shmem_id = ctx.discover_port(&name_buf[..name_len]).map_err(|_| {
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
            bus_msg::ATTACH_DISK => {
                let shmem_id = msg.read_u32(0);
                uinfo!("fatfsd", "attach_disk"; shmem_id = shmem_id);

                // Extract source name from payload
                let source_end = (msg.payload_len as usize).min(240);
                let source_start = 16;
                let source_name = if source_start < source_end {
                    let name = &msg.payload[source_start..source_end];
                    let len = name.iter().position(|&b| b == 0).unwrap_or(name.len());
                    &msg.payload[source_start..source_start + len]
                } else {
                    b"fat0:" as &[u8]
                };

                self.do_init(shmem_id, source_name, ctx);
                Disposition::Handled
            }

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
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: FatfsDriver = FatfsDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"fatfsd", FatfsDriverWrapper(driver));
}

struct FatfsDriverWrapper(&'static mut FatfsDriver);

impl Driver for FatfsDriverWrapper {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        self.0.data_ready(port, ctx)
    }
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
