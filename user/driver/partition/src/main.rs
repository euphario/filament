//! Partition Driver
//!
//! Composable block layer that reads MBR/GPT, discovers partitions, and serves
//! block requests with LBA translation.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  fatfs (spawned per partition)          │
//! │  connects to: part0:, part1:, ...       │
//! ├─────────────────────────────────────────┤
//! │  partition (this driver)                │
//! │  provides: part0:, part1:, ...          │
//! │  consumes: disk0: (or msc0:, nvme0:)    │
//! ├─────────────────────────────────────────┤
//! │  usbd / nvmed (block device)            │
//! │  provides: disk0: (Block type)          │
//! └─────────────────────────────────────────┘
//! ```
//!
//! ## Flow
//!
//! 1. devd spawns this driver when a Block port is registered
//! 2. Driver connects to the Block port (disk0:) as a client
//! 3. Reads sector 0 (MBR) to discover partitions
//! 4. Registers partition ports with devd (triggers fatfs spawn)
//! 5. Serves block requests on partition ports:
//!    - Translates partition LBA to disk LBA (+ partition start)
//!    - Forwards to underlying block device

#![no_std]
#![no_main]

use userlib::syscall::{self, Handle, LogLevel};
use userlib::devd::{DevdClient, DevdCommand, PortType, DriverState};
use userlib::blk_client::BlockClient;
use userlib::blk::{BlkHeader, BlkRead, BlkInfoResp, BlkData, msg, error};
use userlib::ipc::{Port, Mux, MuxFilter};
use userlib::error::SysError;
use userlib::data_port::DataPort;
use userlib::ring::io_status;

// =============================================================================
// Cache Management (for DMA coherency)
// =============================================================================

/// Data synchronization barrier
#[inline]
fn dsb() {
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Invalidate a cache line
#[inline]
fn invalidate_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc civac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Invalidate a buffer from cache (before CPU reads data written by DMA)
#[inline]
fn invalidate_buffer(addr: u64, size: usize) {
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
}

// =============================================================================
// Constants
// =============================================================================

const MAX_PARTITIONS: usize = 4;
const MAX_CLIENTS: usize = 4;

// =============================================================================
// Logging
// =============================================================================

macro_rules! plog {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[partition] ";
        let mut buf = [0u8; 128];
        buf[..PREFIX.len()].copy_from_slice(PREFIX);
        let mut pos = PREFIX.len();
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &b in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = b; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Info, &buf[..pos]);
    }};
}

macro_rules! perror {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[partition] ERROR: ";
        let mut buf = [0u8; 128];
        buf[..PREFIX.len()].copy_from_slice(PREFIX);
        let mut pos = PREFIX.len();
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &b in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = b; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Error, &buf[..pos]);
    }};
}

// =============================================================================
// MBR Structures
// =============================================================================

/// MBR partition entry (16 bytes)
#[derive(Clone, Copy, Debug)]
struct MbrEntry {
    bootable: u8,
    partition_type: u8,
    start_lba: u32,
    sector_count: u32,
}

impl MbrEntry {
    fn from_bytes(data: &[u8]) -> Self {
        Self {
            bootable: data[0],
            partition_type: data[4],
            start_lba: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            sector_count: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
        }
    }

    fn is_valid(&self) -> bool {
        self.partition_type != 0 && self.sector_count > 0
    }

    fn is_fat(&self) -> bool {
        matches!(self.partition_type,
            0x01 | 0x04 | 0x06 | 0x0B | 0x0C | 0x0E |
            0x14 | 0x16 | 0x1B | 0x1C | 0x1E)
    }
}

fn partition_type_name(ptype: u8) -> &'static str {
    match ptype {
        0x00 => "Empty",
        0x01 => "FAT12",
        0x04 | 0x06 | 0x0E => "FAT16",
        0x05 | 0x0F => "Extended",
        0x07 => "NTFS/exFAT",
        0x0B | 0x0C => "FAT32",
        0x82 => "Linux swap",
        0x83 => "Linux",
        0xEE => "GPT protective",
        0xEF => "EFI System",
        _ => "Unknown",
    }
}

/// Parsed MBR
struct Mbr {
    entries: [MbrEntry; 4],
    valid: bool,
}

impl Mbr {
    fn parse(sector: &[u8]) -> Self {
        let valid = sector.len() >= 512 && sector[510] == 0x55 && sector[511] == 0xAA;

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

        Self { entries, valid }
    }
}

// =============================================================================
// Partition Info
// =============================================================================

#[derive(Clone, Copy)]
struct PartitionInfo {
    /// Partition index (0-3)
    index: u8,
    /// Partition type from MBR
    partition_type: u8,
    /// Start LBA on disk
    start_lba: u64,
    /// Number of sectors
    sector_count: u64,
    /// Port handle for this partition
    port: Option<Handle>,
    /// Connected client handle (if any)
    client: Option<Handle>,
}

impl PartitionInfo {
    const fn empty() -> Self {
        Self {
            index: 0,
            partition_type: 0,
            start_lba: 0,
            sector_count: 0,
            port: None,
            client: None,
        }
    }
}

// =============================================================================
// Partition Driver
// =============================================================================

struct PartitionDriver {
    /// Connection to devd
    devd_client: Option<DevdClient>,
    /// Connection to underlying block device (legacy IPC)
    block_client: Option<BlockClient>,
    /// Connection to underlying block device (DataPort ring protocol)
    data_port: Option<DataPort>,
    /// Using DataPort (true) or legacy IPC (false)
    use_data_port: bool,
    /// Block size from underlying device
    block_size: u32,
    /// Total blocks from underlying device
    total_blocks: u64,
    /// Discovered partitions
    partitions: [PartitionInfo; MAX_PARTITIONS],
    partition_count: usize,
    /// Parent disk port name
    disk_name: [u8; 32],
    disk_name_len: usize,
}

impl PartitionDriver {
    const fn new() -> Self {
        Self {
            devd_client: None,
            block_client: None,
            data_port: None,
            use_data_port: false,
            block_size: 512,
            total_blocks: 0,
            partitions: [PartitionInfo::empty(); MAX_PARTITIONS],
            partition_count: 0,
            disk_name: [0; 32],
            disk_name_len: 0,
        }
    }

    fn init(&mut self) -> Result<(), SysError> {
        plog!("connecting to devd-query:");

        // Connect to devd
        let client = DevdClient::connect()?;
        self.devd_client = Some(client);

        plog!("connected to devd");
        Ok(())
    }

    /// Connect to the block device via DataPort (ring protocol)
    fn connect_to_disk_dataport(&mut self, shmem_id: u32) -> Result<(), SysError> {
        plog!("connecting to disk via DataPort shmem_id={}", shmem_id);

        // Connect to DataPort
        let mut port = DataPort::connect(shmem_id)?;

        // Query geometry via sidechannel
        match port.query_geometry() {
            Some(info) => {
                plog!("DataPort geometry: {} bytes/sector, {} sectors",
                    info.block_size, info.block_count);
                self.block_size = info.block_size;
                self.total_blocks = info.block_count;
            }
            None => {
                // Fall back to defaults
                plog!("DataPort: using default geometry (512 bytes, querying via read)");
                self.block_size = 512;
                self.total_blocks = 0; // Will be filled in later if needed
            }
        }

        self.data_port = Some(port);
        self.use_data_port = true;

        plog!("connected via DataPort (ring protocol)");
        Ok(())
    }

    /// Connect to the block device via legacy IPC
    fn connect_to_disk_ipc(&mut self, disk_name: &[u8]) -> Result<(), SysError> {
        plog!("connecting to block device via IPC: {}",
            core::str::from_utf8(disk_name).unwrap_or("?"));

        // Save disk name
        let len = disk_name.len().min(32);
        self.disk_name[..len].copy_from_slice(&disk_name[..len]);
        self.disk_name_len = len;

        // Connect to block device
        let mut block = BlockClient::connect(disk_name)
            .map_err(|_| SysError::ConnectionRefused)?;

        // Get device info
        let (block_size, block_count) = block.info()
            .map_err(|_| SysError::IoError)?;

        plog!("block device: {} bytes/sector, {} sectors", block_size, block_count);

        self.block_size = block_size;
        self.total_blocks = block_count;
        self.block_client = Some(block);
        self.use_data_port = false;

        Ok(())
    }

    /// Connect to the block device (tries DataPort first, falls back to IPC)
    fn connect_to_disk(&mut self, disk_name: &[u8]) -> Result<(), SysError> {
        plog!("connecting to block device: {}",
            core::str::from_utf8(disk_name).unwrap_or("?"));

        // Save disk name
        let len = disk_name.len().min(32);
        self.disk_name[..len].copy_from_slice(&disk_name[..len]);
        self.disk_name_len = len;

        // Try DataPort first (shmem_id 2 is qemu-usbd's DataPort)
        // TODO: Query devd for shmem_id associated with port name
        const DISK0_SHMEM_ID: u32 = 2;

        if disk_name == b"disk0:" {
            match self.connect_to_disk_dataport(DISK0_SHMEM_ID) {
                Ok(()) => return Ok(()),
                Err(e) => {
                    plog!("DataPort connect failed ({:?}), trying IPC", e);
                }
            }
        }

        // Fall back to legacy IPC
        self.connect_to_disk_ipc(disk_name)
    }

    /// Read a block using DataPort or legacy IPC
    fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), SysError> {
        if self.use_data_port {
            let port = self.data_port.as_mut().ok_or(SysError::ConnectionRefused)?;

            // Allocate space in the pool
            let len = buf.len() as u32;
            let offset = port.alloc(len).ok_or(SysError::OutOfMemory)?;

            // Submit read request
            let tag = match port.submit_read(lba, offset, len) {
                Some(t) => t,
                None => {
                    perror!("DataPort submit_read failed");
                    return Err(SysError::IoError);
                }
            };

            // Notify provider and poll for completion
            port.notify();

            // Poll for completion with timeout
            // Note: port.wait() uses shmem.wait() which doesn't support timeout properly
            // Use polling with sleep instead
            for _ in 0..1000 {
                if let Some(cqe) = port.poll_cq() {
                    if cqe.tag == tag {
                        if cqe.status == io_status::OK {
                            // Invalidate cache before reading DMA data
                            // DMA writes directly to memory, bypassing CPU cache
                            if let Some(pool_slice) = port.pool_slice(offset, len) {
                                // Invalidate cache lines covering this region
                                invalidate_buffer(pool_slice.as_ptr() as u64, len as usize);
                                // Now safe to read from CPU
                                buf.copy_from_slice(pool_slice);
                                // TODO: Add pool allocator free() to reclaim space
                                return Ok(());
                            } else {
                                return Err(SysError::IoError);
                            }
                        } else {
                            perror!("DataPort read failed: status={}", cqe.status);
                            return Err(SysError::IoError);
                        }
                    }
                }
                // Brief sleep to avoid busy-loop
                userlib::syscall::sleep_us(1000);
            }
            // Timeout
            perror!("DataPort read timeout");
            Err(SysError::Timeout)
        } else {
            let block = self.block_client.as_mut().ok_or(SysError::ConnectionRefused)?;
            // BlockClient::read_block expects fixed-size array
            let mut sector = [0u8; 512];
            block.read_block(lba, &mut sector).map_err(|_| SysError::IoError)?;
            let copy_len = buf.len().min(512);
            buf[..copy_len].copy_from_slice(&sector[..copy_len]);
            Ok(())
        }
    }

    /// Read MBR and discover partitions
    fn scan_partitions(&mut self) -> Result<usize, SysError> {
        // Read MBR (sector 0)
        let mut mbr_buf = [0u8; 512];
        self.read_block(0, &mut mbr_buf)?;

        let mbr = Mbr::parse(&mbr_buf);
        if !mbr.valid {
            perror!("invalid MBR (no 0x55AA signature)");
            return Err(SysError::InvalidArgument);
        }

        plog!("MBR valid, scanning partitions...");

        // Process partition entries
        self.partition_count = 0;
        for (i, entry) in mbr.entries.iter().enumerate() {
            if entry.is_valid() {
                plog!("  part{}: type={:#04x} ({}) start={} size={}",
                    i, entry.partition_type, partition_type_name(entry.partition_type),
                    entry.start_lba, entry.sector_count);

                if self.partition_count < MAX_PARTITIONS {
                    self.partitions[self.partition_count] = PartitionInfo {
                        index: i as u8,
                        partition_type: entry.partition_type,
                        start_lba: entry.start_lba as u64,
                        sector_count: entry.sector_count as u64,
                        port: None,
                        client: None,
                    };
                    self.partition_count += 1;
                }
            }
        }

        plog!("found {} partition(s)", self.partition_count);
        Ok(self.partition_count)
    }

    /// Register partition ports with devd
    fn register_partitions(&mut self) -> Result<(), SysError> {
        let client = self.devd_client.as_mut().ok_or(SysError::ConnectionRefused)?;
        let disk_name = &self.disk_name[..self.disk_name_len];

        for i in 0..self.partition_count {
            // Build port name: part0:, part1:, etc.
            let mut name = [0u8; 8];
            name[0..4].copy_from_slice(b"part");
            name[4] = b'0' + i as u8;
            name[5] = b':';
            let name_len = 6;

            plog!("registering part{} with parent {}", i,
                core::str::from_utf8(disk_name).unwrap_or("?"));

            // Register partition port with devd
            match client.register_port(&name[..name_len], PortType::Partition, Some(disk_name)) {
                Ok(()) => {
                    plog!("registered part{}", i);
                }
                Err(e) => {
                    perror!("failed to register part{}: {:?}", i, e);
                }
            }
        }

        Ok(())
    }

    /// Create partition service ports and set up the mux
    fn create_service_ports(&mut self, mux: &Mux) -> Result<(), SysError> {
        for i in 0..self.partition_count {
            // Build port name: part0:, part1:, etc.
            let mut name = [0u8; 8];
            name[0..4].copy_from_slice(b"part");
            name[4] = b'0' + i as u8;
            name[5] = b':';
            let name_len = 6;

            match Port::register(&name[..name_len]) {
                Ok(port) => {
                    let handle = port.handle();
                    plog!("created port for part{}", i);
                    let _ = mux.add(handle, MuxFilter::Readable);
                    self.partitions[i].port = Some(handle);
                    // Leak the port so it stays alive
                    core::mem::forget(port);
                }
                Err(e) => {
                    perror!("failed to create port for part{}: {:?}", i, e);
                }
            }
        }

        Ok(())
    }

    /// Handle a block request from a partition client
    fn handle_block_request(
        &mut self,
        part_idx: usize,
        client_handle: Handle,
        req_buf: &[u8],
        req_len: usize,
    ) {
        if req_len < BlkHeader::SIZE {
            return;
        }

        let header = match BlkHeader::from_bytes(req_buf) {
            Some(h) => h,
            None => return,
        };

        match header.msg_type {
            msg::BLK_INFO => {
                self.handle_info_request(part_idx, client_handle, header.seq_id);
            }
            msg::BLK_READ => {
                if let Some(req) = BlkRead::from_bytes(req_buf) {
                    self.handle_read_request(part_idx, client_handle, &req);
                }
            }
            msg::BLK_WRITE => {
                // Not implemented yet
                self.send_error(client_handle, header.seq_id, error::IO);
            }
            _ => {
                plog!("unknown message type: {:#x}", header.msg_type);
            }
        }
    }

    fn handle_info_request(&mut self, part_idx: usize, client: Handle, seq_id: u32) {
        let part = &self.partitions[part_idx];

        // Build response
        let resp = BlkInfoResp::new(seq_id, self.block_size, part.sector_count);
        let _ = syscall::write(client, &resp.to_bytes());
    }

    fn handle_read_request(&mut self, part_idx: usize, client: Handle, req: &BlkRead) {
        let part = &self.partitions[part_idx];

        // Validate request
        if req.lba + req.count as u64 > part.sector_count {
            perror!("read beyond partition: lba={} count={} max={}",
                req.lba, req.count, part.sector_count);
            self.send_error(client, req.header.seq_id, error::IO);
            return;
        }

        // Translate LBA to disk LBA
        let disk_lba = part.start_lba + req.lba;

        // Read into data buffer
        // NOTE: IPC payload limit is 576 bytes, so we limit to 1 sector
        // (512 data + 16 header = 528 bytes fits in 576)
        let mut data_buf = [0u8; 512];
        let count = req.count.min(1);

        match self.read_block(disk_lba, &mut data_buf[..count as usize * self.block_size as usize]) {
            Ok(()) => {
                let bytes_read = count as usize * self.block_size as usize;
                // Build response with data
                let resp = BlkData::new(req.header.seq_id, bytes_read as u32, error::OK);
                let mut resp_buf = [0u8; 512 + BlkData::HEADER_SIZE];
                if let Some(total_len) = resp.write_to(&mut resp_buf, &data_buf[..bytes_read]) {
                    let _ = syscall::write(client, &resp_buf[..total_len]);
                } else {
                    self.send_error(client, req.header.seq_id, error::IO);
                }
            }
            Err(e) => {
                perror!("read failed at disk LBA {}: {:?}", disk_lba, e);
                self.send_error(client, req.header.seq_id, error::IO);
            }
        }
    }

    fn send_error(&self, client: Handle, seq_id: u32, err: i32) {
        let resp = BlkData::new(seq_id, 0, err);
        let mut buf = [0u8; BlkData::HEADER_SIZE];
        if let Some(len) = resp.write_to(&mut buf, &[]) {
            let _ = syscall::write(client, &buf[..len]);
        }
    }

    /// Find partition by port handle
    fn find_partition_by_port(&self, handle: Handle) -> Option<usize> {
        for i in 0..self.partition_count {
            if self.partitions[i].port == Some(handle) {
                return Some(i);
            }
        }
        None
    }

    /// Find partition by client handle
    fn find_partition_by_client(&self, handle: Handle) -> Option<usize> {
        for i in 0..self.partition_count {
            if self.partitions[i].client == Some(handle) {
                return Some(i);
            }
        }
        None
    }

    /// Main service loop
    fn run(&mut self) -> ! {
        plog!("starting partition driver");

        // First, try to get spawn context from devd (tells us which port triggered our spawn)
        let spawn_port: Option<([u8; 64], usize)> = if let Some(client) = self.devd_client.as_mut() {
            match client.get_spawn_context() {
                Ok(Some((name, len, _ptype))) => {
                    plog!("spawn context: port={}", core::str::from_utf8(&name[..len]).unwrap_or("?"));
                    Some((name, len))
                }
                Ok(None) => {
                    plog!("no spawn context (manual start?)");
                    None
                }
                Err(e) => {
                    plog!("get_spawn_context failed: {:?}", e);
                    None
                }
            }
        } else {
            None
        };

        // Try spawn context port first, then fall back to probing
        let mut connected = false;

        if let Some((name, len)) = spawn_port {
            plog!("connecting to spawn context port");
            for retry in 0..20 {
                match self.connect_to_disk(&name[..len]) {
                    Ok(()) => {
                        connected = true;
                        break;
                    }
                    Err(_) => {
                        if retry < 19 {
                            syscall::sleep_ms(100);
                        }
                    }
                }
            }
        }

        // Fall back to probing common disk names
        if !connected {
            let disk_names: [&[u8]; 3] = [b"disk0:", b"msc0:", b"nvme0:"];

            for disk_name in &disk_names {
                plog!("probing: {}", core::str::from_utf8(disk_name).unwrap_or("?"));

                for retry in 0..20 {
                    match self.connect_to_disk(disk_name) {
                        Ok(()) => {
                            connected = true;
                            break;
                        }
                        Err(_) => {
                            if retry < 19 {
                                syscall::sleep_ms(100);
                            }
                        }
                    }
                }
                if connected {
                    break;
                }
            }
        }

        if !connected {
            perror!("failed to connect to any block device");
            syscall::exit(1);
        }

        // Scan partitions
        if let Err(e) = self.scan_partitions() {
            perror!("failed to scan partitions: {:?}", e);
            syscall::exit(1);
        }

        if self.partition_count == 0 {
            plog!("no partitions found, exiting");
            syscall::exit(0);
        }

        // Register partitions with devd
        if let Err(e) = self.register_partitions() {
            perror!("failed to register partitions: {:?}", e);
        }

        // Report ready state
        if let Some(client) = self.devd_client.as_mut() {
            let _ = client.report_state(DriverState::Ready);
        }

        // Create mux for event loop
        let mux = match Mux::new() {
            Ok(m) => m,
            Err(_) => {
                perror!("failed to create mux");
                syscall::exit(1);
            }
        };

        // Create partition service ports
        if let Err(e) = self.create_service_ports(&mux) {
            perror!("failed to create service ports: {:?}", e);
        }

        // Add devd channel to mux
        if let Some(client) = &self.devd_client {
            if let Some(handle) = client.handle() {
                let _ = mux.add(handle, MuxFilter::Readable);
            }
        }

        plog!("entering service loop");

        // Service loop
        loop {
            let event = match mux.wait() {
                Ok(e) => e,
                Err(_) => {
                    syscall::sleep_ms(10);
                    continue;
                }
            };

            // Check for new connection on partition ports
            if let Some(part_idx) = self.find_partition_by_port(event.handle) {
                // Accept connection
                let mut buf = [0u8; 8];
                if let Ok(n) = syscall::read(event.handle, &mut buf) {
                    if n >= 4 {
                        let ch_handle = Handle(u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]));
                        plog!("new client on part{}", part_idx);

                        // Close old client if any
                        if let Some(old) = self.partitions[part_idx].client {
                            let _ = mux.remove(old);
                            let _ = syscall::close(old);
                        }

                        // Track new client
                        let _ = mux.add(ch_handle, MuxFilter::Readable);
                        self.partitions[part_idx].client = Some(ch_handle);
                    }
                }
                continue;
            }

            // Check for request from partition client
            if let Some(part_idx) = self.find_partition_by_client(event.handle) {
                let mut req_buf = [0u8; 64];
                match syscall::read(event.handle, &mut req_buf) {
                    Ok(len) if len > 0 => {
                        self.handle_block_request(part_idx, event.handle, &req_buf, len);
                    }
                    Ok(0) | Err(_) => {
                        // Client disconnected
                        plog!("client disconnected from part{}", part_idx);
                        let _ = mux.remove(event.handle);
                        let _ = syscall::close(event.handle);
                        self.partitions[part_idx].client = None;
                    }
                    _ => {}
                }
                continue;
            }

            // Devd message (spawn commands, etc.)
            if let Some(client) = &mut self.devd_client {
                if client.handle() == Some(event.handle) {
                    if let Ok(Some(cmd)) = client.poll_command() {
                        match cmd {
                            DevdCommand::SpawnChild { seq_id, binary, binary_len, .. } => {
                                // Spawn the child process
                                let binary_str = match core::str::from_utf8(&binary[..binary_len]) {
                                    Ok(s) => s,
                                    Err(_) => {
                                        let _ = client.ack_spawn(seq_id, -1, 0);
                                        continue;
                                    }
                                };

                                plog!("spawning child: {}", binary_str);
                                let pid = syscall::exec(binary_str);
                                if pid > 0 {
                                    plog!("spawned child PID: {}", pid);
                                    let _ = client.ack_spawn(seq_id, 0, pid as u32);
                                } else {
                                    perror!("failed to spawn {}: {}", binary_str, -pid);
                                    let _ = client.ack_spawn(seq_id, pid as i32, 0);
                                }
                            }
                            DevdCommand::StopChild { child_pid, .. } => {
                                plog!("stopping child PID: {}", child_pid);
                                let _ = syscall::kill(child_pid);
                            }
                        }
                    }
                }
            }
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: PartitionDriver = PartitionDriver::new();

#[unsafe(no_mangle)]
fn main() {
    syscall::klog(LogLevel::Info, b"[partition] starting");

    // SAFETY: Single-threaded userspace driver, no concurrent access
    let driver = unsafe { &mut *(&raw mut DRIVER) };

    if let Err(e) = driver.init() {
        perror!("init failed: {:?}", e);
        syscall::exit(1);
    }

    driver.run()
}
