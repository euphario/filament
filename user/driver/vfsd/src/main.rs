//! Virtual Filesystem Daemon — Mount Table Manager
//!
//! Owns a shared memory mount table that shell/apps read directly.
//! FS drivers (fatfsd) register mounts via side-channel on the DataPort.
//! Shell connects directly to FS drivers' DataPorts for I/O — no routing
//! through vfsd.
//!
//! ## Architecture
//!
//! ```text
//! fatfsd ──[REGISTER_MOUNT side-channel]──► vfsd (writes to mount table)
//! shell  ──[reads mount table shmem]──────► gets fs_shmem_id
//!        ──[connects to fs DataPort]──────► fatfsd (direct I/O)
//! ```
//!
//! ## Mount table layout (in DataPort pool)
//!
//! Offset 0: MountTable header (32 bytes)
//! Offset 32: MountEntry[0] (32 bytes)
//! Offset 64: MountEntry[1] (32 bytes)
//! ...
//! Total: 288 bytes for 8 mounts

#![no_std]
#![no_main]

use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockTransport, BlockPortConfig, bus_msg,
    PortInfo, PortClass, port_subclass,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::side_msg;
use userlib::vfs_proto::{MountTable, MountEntry};
use userlib::{uinfo, uerror};

const MAX_MOUNTS: usize = MountTable::MAX_MOUNTS;

// =============================================================================
// VFS Driver
// =============================================================================

struct VfsDriver {
    /// DataPort for side-channel registration (pool holds mount table)
    reg_port: Option<PortId>,
    /// Local mount count (mirrors shmem header)
    mount_count: usize,
    /// Local version counter
    version: u32,
}

impl VfsDriver {
    const fn new() -> Self {
        Self {
            reg_port: None,
            mount_count: 0,
            version: 0,
        }
    }

    /// Handle a REGISTER_MOUNT side-channel message from a FS driver.
    ///
    /// Payload layout (24 bytes):
    ///   [0..4]: fs_shmem_id (u32 LE) — the FS driver's DataPort
    ///   [4]:    mount_name_len (u8)
    ///   [5..24]: mount_name (up to 19 bytes, e.g. "fat0")
    fn handle_register_mount(&mut self, payload: &[u8; 24], ctx: &mut dyn BusCtx) {
        let port_id = match self.reg_port {
            Some(id) => id,
            None => return,
        };

        let fs_shmem_id = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        let name_len = (payload[4] as usize).min(19);
        let mount_name = &payload[5..5 + name_len];

        if self.mount_count >= MAX_MOUNTS {
            uerror!("vfsd", "mount_table_full";);
            return;
        }

        // Build prefix: "/name/"
        let mut prefix = [0u8; 24];
        prefix[0] = b'/';
        let plen = name_len.min(22);
        prefix[1..1 + plen].copy_from_slice(&mount_name[..plen]);
        prefix[1 + plen] = b'/';
        let prefix_len = 2 + plen;

        // Find empty slot and write entry to pool
        let port = match ctx.block_port(port_id) {
            Some(p) => p,
            None => return,
        };

        // Read current entries to find an empty slot
        let mut raw = [0u8; MountTable::TOTAL_SIZE];
        if let Some(slice) = port.pool_slice(0, MountTable::TOTAL_SIZE as u32) {
            raw[..MountTable::TOTAL_SIZE].copy_from_slice(slice);
        }

        let mut slot = None;
        for i in 0..MAX_MOUNTS {
            if let Some(entry) = MountTable::read_entry(&raw, i) {
                if entry.in_use == 0 {
                    slot = Some(i);
                    break;
                }
            }
        }

        let slot_idx = match slot {
            Some(i) => i,
            None => {
                uerror!("vfsd", "no_empty_slot";);
                return;
            }
        };

        // Write entry
        let entry = MountEntry {
            in_use: 1,
            prefix_len: prefix_len as u8,
            _pad: [0; 2],
            fs_shmem_id,
            prefix,
        };

        MountTable::write_entry(&mut raw, slot_idx, &entry);

        // Update header
        self.mount_count += 1;
        self.version += 1;
        let header = MountTable {
            version: self.version,
            count: self.mount_count as u32,
            _pad: [0; 24],
        };
        header.write_to_raw(&mut raw);

        // Write back to pool
        port.pool_write(0, &raw[..MountTable::TOTAL_SIZE]);

        uinfo!("vfsd", "mounted";
            shmem_id = fs_shmem_id,
            slot = slot_idx as u32);
    }

    fn format_info(&self, ctx: &mut dyn BusCtx) -> [u8; 256] {
        let mut buf = [0u8; 256];
        let mut pos = 0;

        let append = |buf: &mut [u8], pos: &mut usize, s: &[u8]| {
            let len = s.len().min(buf.len() - *pos);
            buf[*pos..*pos + len].copy_from_slice(&s[..len]);
            *pos += len;
        };

        append(&mut buf, &mut pos, b"VFS Mount Table Manager\nMounts: ");
        let mut num_buf = [0u8; 8];
        let n = format_u32(&mut num_buf, self.mount_count as u32);
        append(&mut buf, &mut pos, &num_buf[..n]);
        append(&mut buf, &mut pos, b"\n");

        // Read entries from pool
        if let Some(port_id) = self.reg_port {
            if let Some(port) = ctx.block_port(port_id) {
                let mut raw = [0u8; MountTable::TOTAL_SIZE];
                if let Some(slice) = port.pool_slice(0, MountTable::TOTAL_SIZE as u32) {
                    raw[..MountTable::TOTAL_SIZE].copy_from_slice(slice);
                }
                for i in 0..MAX_MOUNTS {
                    if let Some(entry) = MountTable::read_entry(&raw, i) {
                        if entry.in_use != 0 {
                            append(&mut buf, &mut pos, b"  ");
                            append(&mut buf, &mut pos, entry.prefix_bytes());
                            append(&mut buf, &mut pos, b" shmem=");
                            let n = format_u32(&mut num_buf, entry.fs_shmem_id);
                            append(&mut buf, &mut pos, &num_buf[..n]);
                            append(&mut buf, &mut pos, b"\n");
                        }
                    }
                }
            }
        }

        buf
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for VfsDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // Create DataPort for side-channel registration
        // Pool holds the mount table (288 bytes needed, use 4096 for alignment)
        let config = BlockPortConfig {
            ring_size: 8,
            side_size: 8,
            pool_size: 4096,
        };

        match ctx.create_block_port(config) {
            Ok(port_id) => {
                if let Some(port) = ctx.block_port(port_id) {
                    port.set_public();
                    let shmem_id = port.shmem_id();
                    self.reg_port = Some(port_id);

                    // Initialize mount table in pool (all zeros = empty)
                    let header = MountTable {
                        version: 0,
                        count: 0,
                        _pad: [0; 24],
                    };
                    let mut raw = [0u8; MountTable::TOTAL_SIZE];
                    header.write_to_raw(&mut raw);
                    port.pool_write(0, &raw);

                    // Register as "vfs:" Filesystem port using unified PortInfo
                    let mut info = PortInfo::new(b"vfs:", PortClass::Filesystem);
                    info.port_subclass = port_subclass::FS_RAMFS;
                    let _ = ctx.register_port_with_info(&info, shmem_id);

                    uinfo!("vfsd", "mount_table_ready"; shmem_id = shmem_id);
                }
            }
            Err(e) => {
                uerror!("vfsd", "create_port_failed";);
                return Err(e);
            }
        }

        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                let info = self.format_info(ctx);
                let info_len = info.iter().rposition(|&b| b != 0).map(|p| p + 1).unwrap_or(0);
                let _ = ctx.respond_info(msg.seq_id, &info[..info_len]);
                Disposition::Handled
            }

            _ => Disposition::Forward,
        }
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        if self.reg_port != Some(port) {
            return;
        }

        // Process side-channel messages (REGISTER_MOUNT from FS drivers)
        loop {
            let entry = match ctx.block_port(port) {
                Some(p) => match p.poll_side_request() {
                    Some(e) => e,
                    None => break,
                },
                None => break,
            };

            if entry.msg_type == side_msg::REGISTER_MOUNT {
                self.handle_register_mount(&entry.payload, ctx);
            }
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: VfsDriver = VfsDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"vfsd", VfsDriverWrapper(driver));
}

struct VfsDriverWrapper(&'static mut VfsDriver);

impl Driver for VfsDriverWrapper {
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
