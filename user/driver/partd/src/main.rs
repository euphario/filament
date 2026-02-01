//! Partition Driver
//!
//! Composable block layer that reads MBR/GPT, discovers partitions, and serves
//! block requests with LBA translation. Supports multiple disks concurrently.
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
//! 2. Receives ATTACH_DISK command via bus framework (one per disk)
//! 3. Connects to disk DataPort, reads MBR, reports partitions
//! 4. Receives REGISTER_PARTITION, creates per-partition DataPort
//! 5. Serves block requests with LBA translation (zero-copy path)

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel};
use userlib::uerror;
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockPortConfig, bus_msg,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::io_status;
use userlib::query::{PartitionInfoMsg, part_scheme};

// =============================================================================
// Constants
// =============================================================================

const MAX_DISKS: usize = 4;
const MAX_PARTITIONS_PER_DISK: usize = 4;

// =============================================================================
// MBR Structures
// =============================================================================

#[derive(Clone, Copy, Debug)]
struct MbrEntry {
    _bootable: u8,
    partition_type: u8,
    start_lba: u32,
    sector_count: u32,
}

impl MbrEntry {
    fn from_bytes(data: &[u8]) -> Self {
        Self {
            _bootable: data[0],
            partition_type: data[4],
            start_lba: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            sector_count: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
        }
    }

    fn is_valid(&self) -> bool {
        self.partition_type != 0 && self.sector_count > 0
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
            _bootable: 0, partition_type: 0, start_lba: 0, sector_count: 0,
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
// Per-Disk State
// =============================================================================

struct DiskEntry {
    /// Consumer port (connection to underlying disk DataPort)
    consumer_port: PortId,
    /// Provider port (DataPort exposed to fatfs)
    provider_port: Option<PortId>,
    /// Block size from underlying device
    block_size: u32,
    /// Discovered partitions
    partitions: [PartitionInfo; MAX_PARTITIONS_PER_DISK],
    partition_count: usize,
    /// Parent disk port name
    name: [u8; 32],
    name_len: usize,
}

// =============================================================================
// Inflight Request Tracking
// =============================================================================

const MAX_INFLIGHT: usize = 16;

#[derive(Clone, Copy)]
struct InflightRequest {
    /// Tag from fatfsd's SQE (used to complete back to fatfsd)
    provider_tag: u32,
    /// Tag from our submit to usbd (used to match CQE)
    consumer_tag: u32,
    /// Offset in consumer pool where data will land
    consumer_offset: u32,
    /// Offset in provider pool where data should go
    provider_offset: u32,
    /// Which disk this request belongs to
    disk_idx: u8,
    /// True if this is a write (no copy needed on completion)
    is_write: bool,
}

// =============================================================================
// Partition Driver (Bus Framework)
// =============================================================================

struct PartitionDriver {
    disks: [Option<DiskEntry>; MAX_DISKS],
    disk_count: usize,
    inflight: [Option<InflightRequest>; MAX_INFLIGHT],
    inflight_count: usize,
}

impl PartitionDriver {
    const fn new() -> Self {
        Self {
            disks: [None, None, None, None],
            disk_count: 0,
            inflight: [None; MAX_INFLIGHT],
            inflight_count: 0,
        }
    }

    /// Find disk by provider port ID
    fn find_disk_by_provider(&self, port: PortId) -> Option<usize> {
        for (i, slot) in self.disks.iter().enumerate() {
            if let Some(d) = slot {
                if d.provider_port == Some(port) {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Find disk by consumer port ID
    fn find_disk_by_consumer(&self, port: PortId) -> Option<usize> {
        for (i, slot) in self.disks.iter().enumerate() {
            if let Some(d) = slot {
                if d.consumer_port == port {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Read a block from a specific disk's consumer port
    fn read_block(&self, disk_idx: usize, lba: u64, buf: &mut [u8], ctx: &mut dyn BusCtx) -> bool {
        let consumer_id = match &self.disks[disk_idx] {
            Some(d) => d.consumer_port,
            None => return false,
        };

        let port = match ctx.block_port(consumer_id) {
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

    /// Read MBR and discover partitions for a specific disk
    fn scan_partitions(&mut self, disk_idx: usize, ctx: &mut dyn BusCtx) -> usize {
        let mut mbr_buf = [0u8; 512];
        if !self.read_block(disk_idx, 0, &mut mbr_buf, ctx) {
            uerror!("partd", "mbr_read_failed";);
            return 0;
        }

        let mbr = Mbr::parse(&mbr_buf);
        if !mbr.valid {
            uerror!("partd", "mbr_invalid";);
            return 0;
        }

        let disk = match &mut self.disks[disk_idx] {
            Some(d) => d,
            None => return 0,
        };

        disk.partition_count = 0;
        for (i, entry) in mbr.entries.iter().enumerate() {
            if entry.is_valid() && disk.partition_count < MAX_PARTITIONS_PER_DISK {
                disk.partitions[disk.partition_count] = PartitionInfo {
                    index: i as u8,
                    partition_type: entry.partition_type,
                    start_lba: entry.start_lba as u64,
                    sector_count: entry.sector_count as u64,
                };
                disk.partition_count += 1;
            }
        }

        let count = disk.partition_count;
        count
    }

    /// Process ring requests from fatfs on a provider port
    fn process_ring_requests(&mut self, disk_idx: usize, ctx: &mut dyn BusCtx) {
        let provider_id = match &self.disks[disk_idx] {
            Some(d) => match d.provider_port {
                Some(id) => id,
                None => return,
            },
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
                        self.handle_ring_read(disk_idx, &sqe, ctx);
                    }
                    userlib::ring::io_op::WRITE => {
                        self.handle_ring_write(disk_idx, &sqe, ctx);
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
                        let disk = match &self.disks[disk_idx] {
                            Some(d) => d,
                            None => continue,
                        };
                        let info = userlib::bus::BlockGeometry {
                            block_size: disk.block_size,
                            block_count: if disk.partition_count > 0 {
                                disk.partitions[0].sector_count
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
                        if let Some(port) = ctx.block_port(provider_id) {
                            port.notify();
                        }
                    }
                }
            }
        }
    }

    /// Handle a ring READ request from fatfs — async submit, no polling.
    ///
    /// Translates LBA, submits to consumer port, stores inflight entry.
    /// Completion arrives later via `process_completions()` when the consumer
    /// port fires a data_ready callback.
    fn handle_ring_read(&mut self, disk_idx: usize, sqe: &userlib::ring::IoSqe, ctx: &mut dyn BusCtx) {
        let (provider_id, consumer_id, block_size, partition_count) = match &self.disks[disk_idx] {
            Some(d) => (
                match d.provider_port { Some(id) => id, None => return },
                d.consumer_port,
                d.block_size,
                d.partition_count,
            ),
            None => return,
        };

        // Check inflight capacity
        if self.inflight_count >= MAX_INFLIGHT {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::IO_ERROR);
                port.notify();
            }
            return;
        }

        // Partition index from param field
        let part_idx = sqe.param as usize;
        if part_idx >= partition_count {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::INVALID);
                port.notify();
            }
            return;
        }

        let (start_lba, sector_count) = match &self.disks[disk_idx] {
            Some(d) => (d.partitions[part_idx].start_lba, d.partitions[part_idx].sector_count),
            None => return,
        };

        // Validate LBA
        let sectors = (sqe.data_len / block_size) as u64;
        if sqe.lba + sectors > sector_count {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::IO_ERROR);
                port.notify();
            }
            return;
        }

        // Translate LBA
        let disk_lba = start_lba + sqe.lba;

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

        // Store inflight entry — completion arrives via process_completions()
        for slot in &mut self.inflight {
            if slot.is_none() {
                *slot = Some(InflightRequest {
                    provider_tag: sqe.tag,
                    consumer_tag,
                    consumer_offset,
                    provider_offset: sqe.data_offset,
                    disk_idx: disk_idx as u8,
                    is_write: false,
                });
                self.inflight_count += 1;
                return;
            }
        }

        // Should not reach here (checked capacity above), but handle gracefully
        if let Some(port) = ctx.block_port(provider_id) {
            port.complete_error(sqe.tag, io_status::IO_ERROR);
            port.notify();
        }
    }

    /// Handle a ring WRITE request — copy data to consumer pool, submit write.
    fn handle_ring_write(&mut self, disk_idx: usize, sqe: &userlib::ring::IoSqe, ctx: &mut dyn BusCtx) {
        let (provider_id, consumer_id, block_size, partition_count) = match &self.disks[disk_idx] {
            Some(d) => (
                match d.provider_port { Some(id) => id, None => return },
                d.consumer_port,
                d.block_size,
                d.partition_count,
            ),
            None => return,
        };

        if self.inflight_count >= MAX_INFLIGHT {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::IO_ERROR);
                port.notify();
            }
            return;
        }

        let part_idx = sqe.param as usize;
        if part_idx >= partition_count {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::INVALID);
                port.notify();
            }
            return;
        }

        let (start_lba, sector_count) = match &self.disks[disk_idx] {
            Some(d) => (d.partitions[part_idx].start_lba, d.partitions[part_idx].sector_count),
            None => return,
        };

        let sectors = (sqe.data_len / block_size) as u64;
        if sqe.lba + sectors > sector_count {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::IO_ERROR);
                port.notify();
            }
            return;
        }

        let disk_lba = start_lba + sqe.lba;

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

        // Copy data from provider pool to consumer pool before submitting write
        let copy_ok = {
            let src_info = ctx.block_port(provider_id).and_then(|p| {
                p.pool_slice(sqe.data_offset, sqe.data_len).map(|s| {
                    (s.as_ptr(), s.len())
                })
            });

            if let Some((src_ptr, src_len)) = src_info {
                let dst_info = ctx.block_port(consumer_id).and_then(|p| {
                    p.pool_slice_mut(consumer_offset, sqe.data_len).map(|d| {
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

        if !copy_ok {
            if let Some(port) = ctx.block_port(provider_id) {
                port.complete_error(sqe.tag, io_status::IO_ERROR);
                port.notify();
            }
            return;
        }

        // Submit write to disk
        let consumer_tag = match ctx.block_port(consumer_id)
            .and_then(|p| p.submit_write(disk_lba, consumer_offset, sqe.data_len).ok())
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

        for slot in &mut self.inflight {
            if slot.is_none() {
                *slot = Some(InflightRequest {
                    provider_tag: sqe.tag,
                    consumer_tag,
                    consumer_offset,
                    provider_offset: sqe.data_offset,
                    disk_idx: disk_idx as u8,
                    is_write: true,
                });
                self.inflight_count += 1;
                return;
            }
        }

        if let Some(port) = ctx.block_port(provider_id) {
            port.complete_error(sqe.tag, io_status::IO_ERROR);
            port.notify();
        }
    }

    /// Process completions from a consumer port (CQEs from usbd/nvmed).
    ///
    /// For each CQE, matches the inflight entry by consumer_tag, copies data
    /// from consumer pool to provider pool, and completes back to fatfsd.
    fn process_completions(&mut self, disk_idx: usize, ctx: &mut dyn BusCtx) {
        let consumer_id = match &self.disks[disk_idx] {
            Some(d) => d.consumer_port,
            None => return,
        };

        // Drain all available CQEs
        loop {
            let cqe = match ctx.block_port(consumer_id).and_then(|p| p.poll_completion()) {
                Some(c) => c,
                None => break,
            };

            // Find matching inflight entry
            let mut found = None;
            for (i, slot) in self.inflight.iter_mut().enumerate() {
                if let Some(req) = slot {
                    if req.consumer_tag == cqe.tag {
                        found = Some((i, *req));
                        *slot = None;
                        self.inflight_count -= 1;
                        break;
                    }
                }
            }

            let (_, req) = match found {
                Some(f) => f,
                None => continue, // Stale or unknown CQE
            };

            let provider_id = match &self.disks[req.disk_idx as usize] {
                Some(d) => match d.provider_port {
                    Some(id) => id,
                    None => continue,
                },
                None => continue,
            };

            if cqe.status != io_status::OK as u16 {
                if let Some(port) = ctx.block_port(provider_id) {
                    port.complete_error(req.provider_tag, io_status::IO_ERROR);
                    port.notify();
                }
                continue;
            }

            if req.is_write {
                // Write completions: data was already copied before submit, just complete
                if let Some(port) = ctx.block_port(provider_id) {
                    port.complete_ok(req.provider_tag, cqe.transferred);
                    port.notify();
                }
            } else {
                // Read completions: copy data from consumer pool to provider pool
                let copy_ok = {
                    let src_info = ctx.block_port(consumer_id).and_then(|p| {
                        p.pool_slice(req.consumer_offset, cqe.transferred).map(|s| {
                            (s.as_ptr(), s.len())
                        })
                    });

                    if let Some((src_ptr, src_len)) = src_info {
                        let dst_info = ctx.block_port(provider_id).and_then(|p| {
                            p.pool_slice_mut(req.provider_offset, cqe.transferred).map(|d| {
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
                        port.complete_ok(req.provider_tag, cqe.transferred);
                    } else {
                        port.complete_error(req.provider_tag, io_status::IO_ERROR);
                    }
                    port.notify();
                }
            }
        }

        // Reset consumer pool when all inflight requests are done
        if self.inflight_count == 0 {
            if let Some(port) = ctx.block_port(consumer_id) {
                port.reset_pool();
            }
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
        let mut num_buf = [0u8; 16];
        append(&mut buf, &mut pos, b"  Disks: ");
        let num_len = format_u32(&mut num_buf, self.disk_count as u32);
        append(&mut buf, &mut pos, &num_buf[..num_len]);
        append(&mut buf, &mut pos, b"\n");

        for (di, slot) in self.disks.iter().enumerate() {
            let d = match slot {
                Some(d) => d,
                None => continue,
            };
            append(&mut buf, &mut pos, b"\n  Disk ");
            let num_len = format_u32(&mut num_buf, di as u32);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b": ");
            append(&mut buf, &mut pos, &d.name[..d.name_len]);
            append(&mut buf, &mut pos, b" (");
            let num_len = format_u32(&mut num_buf, d.block_size);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b" bytes/sector, ");
            let num_len = format_u32(&mut num_buf, d.partition_count as u32);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b" partitions)\n");

            for i in 0..d.partition_count {
                let p = &d.partitions[i];
                let type_name = match p.partition_type {
                    0x01 | 0x04 | 0x06 | 0x0E => b"FAT16" as &[u8],
                    0x0B | 0x0C => b"FAT32",
                    0x07 => b"NTFS",
                    0x82 => b"Linux swap",
                    0x83 => b"Linux",
                    0xEE => b"GPT",
                    _ => b"Unknown",
                };
                let size_mb = (p.sector_count * d.block_size as u64) / (1024 * 1024);

                append(&mut buf, &mut pos, b"    part");
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
        }

        buf
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for PartitionDriver {
    fn init(&mut self, _ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // Nothing to do — wait for ATTACH_DISK commands from devd
        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::ATTACH_DISK => {
                let shmem_id = msg.read_u32(0);
                let block_size = msg.read_u32(4);
                let _block_count = msg.read_u64(8);
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

                // Find an empty disk slot
                let disk_idx = match self.disks.iter().position(|s| s.is_none()) {
                    Some(i) => i,
                    None => {
                        uerror!("partd", "too_many_disks";);
                        return Disposition::Handled;
                    }
                };

                // Connect to disk DataPort
                match ctx.connect_block_port(shmem_id) {
                    Ok(port_id) => {
                        let mut name = [0u8; 32];
                        let copy_len = source_len.min(32);
                        name[..copy_len].copy_from_slice(&source_name[..copy_len]);

                        let mut bs = if block_size > 0 { block_size } else { 512 };

                        // Query geometry
                        if let Some(port) = ctx.block_port(port_id) {
                            if let Some(geo) = port.query_geometry() {
                                bs = geo.block_size;
                            }
                        }

                        self.disks[disk_idx] = Some(DiskEntry {
                            consumer_port: port_id,
                            provider_port: None,
                            block_size: bs,
                            partitions: [PartitionInfo::empty(); MAX_PARTITIONS_PER_DISK],
                            partition_count: 0,
                            name,
                            name_len: copy_len,
                        });
                        self.disk_count += 1;

                        // Scan partitions
                        let count = self.scan_partitions(disk_idx, ctx);

                        if count > 0 {
                            // Build partition info for devd
                            let mut part_infos = [PartitionInfoMsg::new(); MAX_PARTITIONS_PER_DISK];
                            if let Some(d) = &self.disks[disk_idx] {
                                for i in 0..count {
                                    let p = &d.partitions[i];
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
                            }

                            // Report to devd
                            if let Err(_) = ctx.report_partitions(
                                shmem_id,
                                part_scheme::MBR,
                                &part_infos[..count],
                            ) {
                                uerror!("partd", "report_failed";);
                            }
                        }
                    }
                    Err(_) => {
                        uerror!("partd", "disk_connect_failed";);
                    }
                }

                Disposition::Handled
            }

            bus_msg::REGISTER_PARTITION => {
                let _index = msg.read_u8(0) as usize;
                let name = msg.read_bytes(1, 32);
                let name_len = name.iter().position(|&b| b == 0).unwrap_or(name.len());
                let part_name = &msg.payload[1..1 + name_len];

                // Find the disk that has a partition at this index without a provider port yet.
                // We look for the most recently added disk that needs a provider port for this
                // partition index, since REGISTER_PARTITION arrives in order after REPORT_PARTITIONS.
                let disk_idx = self.disks.iter().enumerate().rev()
                    .find_map(|(i, slot)| {
                        if let Some(d) = slot {
                            if d.provider_port.is_none() && d.partition_count > 0 {
                                return Some(i);
                            }
                        }
                        None
                    });

                let disk_idx = match disk_idx {
                    Some(i) => i,
                    None => {
                        uerror!("partd", "no_disk_for_partition";);
                        return Disposition::Handled;
                    }
                };

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

                            if let Some(d) = &mut self.disks[disk_idx] {
                                d.provider_port = Some(port_id);
                            }

                            // Notify devd
                            let _ = ctx.partition_ready(part_name, shmem_id);
                        }
                    }
                    Err(_) => {
                        uerror!("partd", "dataport_create_failed";);
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
        if let Some(disk_idx) = self.find_disk_by_provider(port) {
            // SQEs from fatfsd — translate and forward to consumer
            self.process_ring_requests(disk_idx, ctx);
        } else if let Some(disk_idx) = self.find_disk_by_consumer(port) {
            // CQEs from usbd — match inflight, copy data, complete to fatfsd
            self.process_completions(disk_idx, ctx);
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
