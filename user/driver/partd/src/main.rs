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
//! 2. Receives ATTACH_DISK command via bus framework
//! 3. Connects to disk DataPort, reads MBR, reports partitions
//! 4. Receives REGISTER_PARTITION, creates per-partition DataPort
//! 5. Serves block requests with LBA translation (zero-copy path)

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel};
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockTransport, BlockPortConfig, bus_msg,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::io_status;
use userlib::query::{PartitionInfoMsg, part_scheme};

// =============================================================================
// Constants
// =============================================================================

const MAX_PARTITIONS: usize = 4;

// =============================================================================
// Logging
// =============================================================================

macro_rules! plog {
    ($($arg:tt)*) => { /* disabled */ };
}

macro_rules! perror {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[partd] ERROR: ";
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

struct Mbr {
    entries: [MbrEntry; 4],
    valid: bool,
}

impl Mbr {
    fn parse(sector: &[u8]) -> Self {
        let valid = sector.len() >= 512 && sector[510] == 0x55 && sector[511] == 0xAA;
        let mut entries = [MbrEntry {
            bootable: 0, partition_type: 0, start_lba: 0, sector_count: 0,
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
    index: u8,
    partition_type: u8,
    start_lba: u64,
    sector_count: u64,
}

impl PartitionInfo {
    const fn empty() -> Self {
        Self { index: 0, partition_type: 0, start_lba: 0, sector_count: 0 }
    }
}

// =============================================================================
// Partition Driver (Bus Framework)
// =============================================================================

struct PartitionDriver {
    /// Consumer port ID (connection to underlying disk DataPort)
    consumer_port: Option<PortId>,
    /// Provider port ID (DataPort we expose to fatfs)
    provider_port: Option<PortId>,
    /// Block size from underlying device
    block_size: u32,
    /// Total blocks from underlying device
    total_blocks: u64,
    /// Discovered partitions
    partitions: [PartitionInfo; MAX_PARTITIONS],
    partition_count: usize,
    /// Disk shmem_id (saved for reporting)
    disk_shmem_id: u32,
    /// Parent disk port name
    disk_name: [u8; 32],
    disk_name_len: usize,
}

impl PartitionDriver {
    const fn new() -> Self {
        Self {
            consumer_port: None,
            provider_port: None,
            block_size: 512,
            total_blocks: 0,
            partitions: [PartitionInfo::empty(); MAX_PARTITIONS],
            partition_count: 0,
            disk_shmem_id: 0,
            disk_name: [0; 32],
            disk_name_len: 0,
        }
    }

    /// Read a block from the consumer (disk) port
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

        // Poll for completion
        for _ in 0..1000 {
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
            syscall::sleep_us(1000);
        }
        false
    }

    /// Read MBR and discover partitions
    fn scan_partitions(&mut self, ctx: &mut dyn BusCtx) -> usize {
        let mut mbr_buf = [0u8; 512];
        if !self.read_block(0, &mut mbr_buf, ctx) {
            perror!("failed to read MBR");
            return 0;
        }

        let mbr = Mbr::parse(&mbr_buf);
        if !mbr.valid {
            perror!("invalid MBR (no 0x55AA signature)");
            return 0;
        }

        self.partition_count = 0;
        for (i, entry) in mbr.entries.iter().enumerate() {
            if entry.is_valid() && self.partition_count < MAX_PARTITIONS {
                plog!("  part{}: type={:#04x} ({}) start={} size={}",
                    i, entry.partition_type, partition_type_name(entry.partition_type),
                    entry.start_lba, entry.sector_count);
                self.partitions[self.partition_count] = PartitionInfo {
                    index: i as u8,
                    partition_type: entry.partition_type,
                    start_lba: entry.start_lba as u64,
                    sector_count: entry.sector_count as u64,
                };
                self.partition_count += 1;
            }
        }
        self.partition_count
    }

    /// Process ring requests from fatfs on the provider port
    fn process_ring_requests(&mut self, ctx: &mut dyn BusCtx) {
        let provider_id = match self.provider_port {
            Some(id) => id,
            None => return,
        };

        // Collect pending SQ requests
        let mut requests: [Option<userlib::ring::IoSqe>; 8] = [None; 8];
        let mut req_count = 0;

        if let Some(port) = ctx.block_port(provider_id) {
            while req_count < 8 {
                if let Some(sqe) = port.recv_request() {
                    requests[req_count] = Some(sqe);
                    req_count += 1;
                } else {
                    break;
                }
            }
        }

        // Process collected requests
        for i in 0..req_count {
            if let Some(sqe) = requests[i].take() {
                match sqe.opcode {
                    userlib::ring::io_op::READ => {
                        self.handle_ring_read(&sqe, ctx);
                    }
                    _ => {
                        if let Some(port) = ctx.block_port(provider_id) {
                            port.complete_error(sqe.tag, userlib::ring::io_status::INVALID);
                            port.notify();
                        }
                    }
                }
            }
        }

        // Process sidechannel queries
        let mut queries: [Option<userlib::ring::SideEntry>; 4] = [None; 4];
        let mut query_count = 0;

        if let Some(port) = ctx.block_port(provider_id) {
            while query_count < 4 {
                if let Some(entry) = port.poll_side_request() {
                    queries[query_count] = Some(entry);
                    query_count += 1;
                } else {
                    break;
                }
            }
        }

        for i in 0..query_count {
            if let Some(entry) = queries[i].take() {
                use userlib::ring::side_msg;
                match entry.msg_type {
                    side_msg::QUERY_GEOMETRY => {
                        let info = userlib::bus::BlockGeometry {
                            block_size: self.block_size,
                            block_count: if self.partition_count > 0 {
                                self.partitions[0].sector_count
                            } else {
                                0
                            },
                            max_transfer: 64 * 1024,
                        };
                        if let Some(port) = ctx.block_port(provider_id) {
                            port.respond_geometry(&entry, &info);
                            port.notify();
                        }
                    }
                    _ => {
                        // Unknown query - complete with error
                        if let Some(port) = ctx.block_port(provider_id) {
                            let mut eol = entry;
                            eol.status = userlib::ring::side_status::EOL;
                            // side_send not available via trait, use complete_error as signal
                            port.notify();
                        }
                    }
                }
            }
        }
    }

    /// Handle a ring READ request from fatfs
    fn handle_ring_read(&mut self, sqe: &userlib::ring::IoSqe, ctx: &mut dyn BusCtx) {
        let provider_id = match self.provider_port {
            Some(id) => id,
            None => return,
        };
        let consumer_id = match self.consumer_port {
            Some(id) => id,
            None => {
                if let Some(port) = ctx.block_port(provider_id) {
                    port.complete_error(sqe.tag, io_status::IO_ERROR);
                    port.notify();
                }
                return;
            }
        };

        // Partition index from param field
        let part_idx = sqe.param as usize;
        if part_idx >= self.partition_count {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::INVALID);
                port.notify();
            }
            return;
        }

        let part = &self.partitions[part_idx];

        // Validate LBA
        let sectors = (sqe.data_len / self.block_size) as u64;
        if sqe.lba + sectors > part.sector_count {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::IO_ERROR);
                port.notify();
            }
            return;
        }

        // Translate LBA
        let disk_lba = part.start_lba + sqe.lba;

        // Allocate in consumer's pool
        let consumer_offset = match ctx.block_port(consumer_id).and_then(|p| p.alloc(sqe.data_len)) {
            Some(o) => o,
            None => {
                if let Some(port) = ctx.block_port(provider_id) {
                    port.complete_error(sqe.tag, io_status::IO_ERROR);
                    port.notify();
                }
                return;
            }
        };

        // Submit read to disk
        let consumer_tag = match ctx.block_port(consumer_id)
            .and_then(|p| p.submit_read(disk_lba, consumer_offset, sqe.data_len).ok())
        {
            Some(t) => t,
            None => {
                if let Some(port) = ctx.block_port(provider_id) {
                    port.complete_error(sqe.tag, io_status::IO_ERROR);
                    port.notify();
                }
                return;
            }
        };

        if let Some(port) = ctx.block_port(consumer_id) {
            port.notify();
        }

        // Poll for completion from disk
        let mut transferred = 0u32;
        let mut success = false;

        for _ in 0..1000 {
            if let Some(port) = ctx.block_port(consumer_id) {
                if let Some(cqe) = port.poll_completion() {
                    if cqe.tag == consumer_tag {
                        if cqe.status == io_status::OK as u16 {
                            transferred = cqe.transferred;
                            success = true;
                        }
                        break;
                    }
                }
            }
            syscall::sleep_us(1000);
        }

        if !success {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::IO_ERROR);
                port.notify();
            }
            return;
        }

        // Copy from consumer pool to provider pool (the ONE copy in the chain)
        let copy_ok = {
            let src_info = ctx.block_port(consumer_id).and_then(|p| {
                p.pool_slice(consumer_offset, transferred).map(|s| {
                    (s.as_ptr(), s.len())
                })
            });

            if let Some((src_ptr, src_len)) = src_info {
                let dst_info = ctx.block_port(provider_id).and_then(|p| {
                    p.pool_slice_mut(sqe.data_offset, transferred).map(|d| {
                        (d.as_mut_ptr(), d.len())
                    })
                });

                if let Some((dst_ptr, dst_len)) = dst_info {
                    let copy_len = src_len.min(dst_len);
                    unsafe {
                        core::ptr::copy_nonoverlapping(src_ptr, dst_ptr, copy_len);
                    }
                    true
                } else {
                    false
                }
            } else {
                false
            }
        };

        if let Some(port) = ctx.block_port(provider_id) {
            if copy_ok {
                port.complete_ok(sqe.tag, transferred);
            } else {
                port.complete_error(sqe.tag, io_status::IO_ERROR);
            }
            port.notify();
        }
    }

    /// Format info text for introspection queries
    fn format_info(&self) -> [u8; 512] {
        let mut buf = [0u8; 512];
        let mut pos = 0;

        let append = |buf: &mut [u8], pos: &mut usize, s: &[u8]| {
            let len = s.len().min(buf.len() - *pos);
            buf[*pos..*pos + len].copy_from_slice(&s[..len]);
            *pos += len;
        };

        append(&mut buf, &mut pos, b"Partition Driver (bus framework)\n");
        append(&mut buf, &mut pos, b"  Disk: ");
        append(&mut buf, &mut pos, &self.disk_name[..self.disk_name_len]);
        append(&mut buf, &mut pos, b" (");
        let mut num_buf = [0u8; 16];
        let num_len = format_u32(&mut num_buf, self.block_size);
        append(&mut buf, &mut pos, &num_buf[..num_len]);
        append(&mut buf, &mut pos, b" bytes/sector)\n\nPartitions: ");
        let num_len = format_u32(&mut num_buf, self.partition_count as u32);
        append(&mut buf, &mut pos, &num_buf[..num_len]);
        append(&mut buf, &mut pos, b"\n");

        for i in 0..self.partition_count {
            let p = &self.partitions[i];
            let type_name = match p.partition_type {
                0x01 | 0x04 | 0x06 | 0x0E => b"FAT16" as &[u8],
                0x0B | 0x0C => b"FAT32",
                0x07 => b"NTFS",
                0x82 => b"Linux swap",
                0x83 => b"Linux",
                0xEE => b"GPT",
                _ => b"Unknown",
            };
            let size_mb = (p.sector_count * self.block_size as u64) / (1024 * 1024);

            append(&mut buf, &mut pos, b"  part");
            let num_len = format_u32(&mut num_buf, i as u32);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b": ");
            append(&mut buf, &mut pos, type_name);
            append(&mut buf, &mut pos, b" (0x");
            let num_len = format_hex(&mut num_buf, p.partition_type as u32, 2);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b"), LBA ");
            let num_len = format_u64(&mut num_buf, p.start_lba);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b", ");
            let num_len = format_u64(&mut num_buf, size_mb);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b" MB\n");
        }

        buf
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for PartitionDriver {
    fn init(&mut self, _ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // Nothing to do — wait for ATTACH_DISK command from devd
        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::ATTACH_DISK => {
                let shmem_id = msg.read_u32(0);
                let block_size = msg.read_u32(4);
                let block_count = msg.read_u64(8);
                // Source name starts at offset 16
                let source_end = (msg.payload_len as usize).min(240);
                let source_start = 16;
                let source_name = if source_start < source_end {
                    &msg.payload[source_start..source_end]
                } else {
                    &[]
                };
                let source_len = source_name.iter().position(|&b| b == 0).unwrap_or(source_name.len());

                syscall::klog(LogLevel::Info, b"[partd] AttachDisk");

                // Save disk info
                self.disk_shmem_id = shmem_id;
                let copy_len = source_len.min(32);
                self.disk_name[..copy_len].copy_from_slice(&source_name[..copy_len]);
                self.disk_name_len = copy_len;

                // Connect to disk DataPort
                match ctx.connect_block_port(shmem_id) {
                    Ok(port_id) => {
                        self.consumer_port = Some(port_id);

                        // Query geometry
                        if let Some(port) = ctx.block_port(port_id) {
                            if let Some(geo) = port.query_geometry() {
                                self.block_size = geo.block_size;
                                self.total_blocks = geo.block_count;
                            } else {
                                self.block_size = if block_size > 0 { block_size } else { 512 };
                                self.total_blocks = block_count;
                            }
                        }

                        // Scan partitions
                        let count = self.scan_partitions(ctx);

                        if count > 0 {
                            // Build partition info for devd
                            let mut part_infos = [PartitionInfoMsg::new(); MAX_PARTITIONS];
                            for i in 0..count {
                                let p = &self.partitions[i];
                                part_infos[i] = PartitionInfoMsg {
                                    index: p.index,
                                    part_type: p.partition_type,
                                    bootable: 0,
                                    _pad: 0,
                                    start_lba: p.start_lba,
                                    size_sectors: p.sector_count,
                                    guid: [0; 16],
                                    label: [0; 36],
                                };
                            }

                            // Report to devd
                            if let Err(_) = ctx.report_partitions(
                                self.disk_shmem_id,
                                part_scheme::MBR,
                                &part_infos[..count],
                            ) {
                                perror!("failed to report partitions");
                            }
                        }
                    }
                    Err(e) => {
                        perror!("connect to disk failed: {:?}", e);
                    }
                }

                Disposition::Handled
            }

            bus_msg::REGISTER_PARTITION => {
                let index = msg.read_u8(0) as usize;
                let name = msg.read_bytes(1, 32);
                let name_len = name.iter().position(|&b| b == 0).unwrap_or(name.len());
                let part_name = &msg.payload[1..1 + name_len];

                // Create provider DataPort for this partition
                let config = BlockPortConfig {
                    ring_size: 64,
                    side_size: 8,
                    pool_size: 256 * 1024,
                };

                match ctx.create_block_port(config) {
                    Ok(port_id) => {
                        if let Some(port) = ctx.block_port(port_id) {
                            port.set_public();
                            let shmem_id = port.shmem_id();
                            self.provider_port = Some(port_id);

                            // Notify devd
                            let _ = ctx.partition_ready(part_name, shmem_id);
                        }
                    }
                    Err(e) => {
                        perror!("create partition DataPort failed: {:?}", e);
                    }
                }

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
        if self.provider_port == Some(port) {
            self.process_ring_requests(ctx);
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: PartitionDriver = PartitionDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"partd", PartitionDriverWrapper(driver));
}

/// Wrapper to pass &'static mut PartitionDriver to driver_main
struct PartitionDriverWrapper(&'static mut PartitionDriver);

impl Driver for PartitionDriverWrapper {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.init(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        self.0.data_ready(port, ctx)
    }
}

// =============================================================================
// Helper Functions for Formatting
// =============================================================================

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

fn format_u64(buf: &mut [u8], val: u64) -> usize {
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

fn format_hex(buf: &mut [u8], val: u32, min_digits: usize) -> usize {
    const HEX_CHARS: &[u8] = b"0123456789ABCDEF";
    if val == 0 {
        for i in 0..min_digits { buf[i] = b'0'; }
        return min_digits;
    }
    let mut n = val;
    let mut len = 0;
    while n > 0 { len += 1; n /= 16; }
    let actual_len = len.max(min_digits);
    for i in 0..(actual_len - len) { buf[i] = b'0'; }
    n = val;
    for i in (actual_len - len..actual_len).rev() {
        buf[i] = HEX_CHARS[(n % 16) as usize];
        n /= 16;
    }
    actual_len
}
