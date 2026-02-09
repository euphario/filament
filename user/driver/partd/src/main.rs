//! Partition Driver
//!
//! Per-disk block layer that reads MBR, discovers partitions, and serves
//! block requests with LBA translation. One partd instance per disk.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  fatfs (spawned per partition)          │
//! │  connects to: 0:fat, 1:ext2, ...        │
//! ├─────────────────────────────────────────┤
//! │  partd (this driver, one per disk)      │
//! │  provides: 0:fat, 1:ext2, ...           │
//! │  consumes: usb0:msc (or nvme0:)         │
//! ├─────────────────────────────────────────┤
//! │  usbd / nvmed (block device)            │
//! │  provides: usb0:msc / nvme0: (Block)    │
//! └─────────────────────────────────────────┘
//! ```
//!
//! ## Flow
//!
//! 1. devd rule spawns partd when a Block(BLOCK_RAW) port appears
//! 2. In reset(), partd queries spawn context to find the trigger port
//! 3. Discovers and connects to disk DataPort, reads MBR
//! 4. Registers per-partition ports (Block with filesystem subclass)
//! 5. Serves block requests with LBA translation (zero-copy path)

#![no_std]
#![no_main]

use userlib::syscall;
use userlib::uerror;
use userlib::uinfo;
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockPortConfig, BlockMetadata, PortInfo, PortClass, port_subclass, bus_msg,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::io_status;

// =============================================================================
// Constants
// =============================================================================

const MAX_DISKS: usize = 1;
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
            disks: [None],
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
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // 1. Get spawn context — partd is spawned per-disk by PORT_RULES
        //    Copy port name to local buffer before further ctx calls (avoids borrow conflict)
        let mut port_name_buf = [0u8; 64];
        let port_name_len;
        {
            let spawn_ctx = ctx.spawn_context()?;
            let pn = spawn_ctx.port_name();
            port_name_len = pn.len().min(64);
            port_name_buf[..port_name_len].copy_from_slice(&pn[..port_name_len]);
        }
        let port_name = &port_name_buf[..port_name_len];

        // Copy port name for disk entry (shorter buffer)
        let mut name = [0u8; 32];
        let name_len = port_name_len.min(32);
        name[..name_len].copy_from_slice(&port_name[..name_len]);

        uinfo!("partd", "attach"; port = core::str::from_utf8(port_name).unwrap_or("?"));

        // 2. Discover the disk DataPort shmem_id via devd
        let shmem_id = ctx.discover_port(port_name)?;

        // 3. Connect to the disk DataPort as consumer
        let consumer_port = ctx.connect_block_port(shmem_id)?;

        // Query block size from geometry
        let mut block_size: u32 = 512;
        if let Some(port) = ctx.block_port(consumer_port) {
            if let Some(geo) = port.query_geometry() {
                block_size = geo.block_size;
            }
        }

        self.disks[0] = Some(DiskEntry {
            consumer_port,
            provider_port: None,
            block_size,
            partitions: [PartitionInfo::empty(); MAX_PARTITIONS_PER_DISK],
            partition_count: 0,
            name,
            name_len,
        });
        self.disk_count = 1;

        // 4. Scan MBR
        let count = self.scan_partitions(0, ctx);

        if count == 0 {
            uerror!("partd", "no_partitions";);
            return Ok(());
        }

        // 5. Create provider DataPort and register a port for each partition
        let config = BlockPortConfig {
            ring_size: 64,
            side_size: 8,
            pool_size: 256 * 1024,
        };

        match ctx.create_block_port(config) {
            Ok(port_id) => {
                // Get shmem_id and set public — release borrow before register_port_with_info
                let provider_shmem_id = if let Some(port) = ctx.block_port(port_id) {
                    port.set_public();
                    port.shmem_id()
                } else {
                    uerror!("partd", "dataport_access_failed";);
                    return Ok(());
                };

                if let Some(d) = &mut self.disks[0] {
                    d.provider_port = Some(port_id);
                }

                // Register a port for each discovered partition
                if let Some(d) = &self.disks[0] {
                    for i in 0..d.partition_count {
                        let p = &d.partitions[i];

                        // Map MBR type to port_subclass
                        let subclass = mbr_type_to_subclass(p.partition_type);

                        // Build partition port name: "N:type"
                        let type_suffix = mbr_type_to_suffix(p.partition_type);
                        let mut pname = [0u8; 32];
                        let mut plen = 0;
                        // Partition index digit
                        pname[0] = b'0' + (i as u8);
                        plen += 1;
                        pname[plen] = b':';
                        plen += 1;
                        let slen = type_suffix.len().min(pname.len() - plen);
                        pname[plen..plen + slen].copy_from_slice(&type_suffix[..slen]);
                        plen += slen;

                        let mut info = PortInfo::new(&pname[..plen], PortClass::Block);
                        info.port_subclass = subclass;
                        info.set_parent(port_name);
                        info.set_block_metadata(BlockMetadata {
                            size_bytes: p.sector_count * block_size as u64,
                            sector_size: block_size,
                            partition_index: i as u8,
                            flags: 0,
                            _reserved: [0; 10],
                        });

                        let _ = ctx.register_port_with_info(&info, provider_shmem_id);

                        uinfo!("partd", "partition";
                            idx = i as u32,
                            mbr_type = p.partition_type as u32,
                            start_lba = p.start_lba,
                            sectors = p.sector_count);
                    }
                }
            }
            Err(_) => {
                uerror!("partd", "dataport_create_failed";);
            }
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
// Helper Functions for Formatting
// =============================================================================

/// Map MBR partition type byte to port_subclass constant.
fn mbr_type_to_subclass(mbr_type: u8) -> u16 {
    match mbr_type {
        0x01 => port_subclass::BLOCK_FAT12,
        0x06 => port_subclass::BLOCK_FAT16,
        0x0B => port_subclass::BLOCK_FAT32,
        0x0C => port_subclass::BLOCK_FAT32_LBA,
        0x83 => port_subclass::BLOCK_LINUX,
        _ => 0,
    }
}

/// Map MBR partition type byte to a short port name suffix.
fn mbr_type_to_suffix(mbr_type: u8) -> &'static [u8] {
    match mbr_type {
        0x01 | 0x06 | 0x0B | 0x0C => b"fat",
        0x83 => b"ext2",
        _ => b"blk",
    }
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
