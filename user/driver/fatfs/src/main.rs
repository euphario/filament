//! FAT Filesystem Driver
//!
//! Userspace FAT16/FAT32 filesystem driver that reads from block storage.
//! Connects to partition driver (part0:) via block IPC protocol.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  fatfs (this driver)                    │
//! │  connects to: part0: (partition)        │
//! ├─────────────────────────────────────────┤
//! │  partition driver                       │
//! │  provides: part0:, part1:, ...          │
//! │  translates LBA, forwards to disk0:     │
//! ├─────────────────────────────────────────┤
//! │  usbd / qemu-usbd (block device)        │
//! │  provides: disk0: (raw disk)            │
//! └─────────────────────────────────────────┘
//! ```
//!
//! ## Flow
//!
//! 1. fatfs connects to part0: block port (partition)
//! 2. Reads boot sector (sector 0 of partition), parses FAT parameters
//! 3. Registers with vfsd at /mnt/fatN
//! 4. Handles FS_LIST, FS_READ requests by reading FAT structures

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel, Handle};
use userlib::ipc::{Channel, Mux, MuxFilter};
use userlib::devd::{DevdClient, DriverState};
use userlib::blk_client::BlockClient;
use userlib::blk::BlkError;
use userlib::data_port::DataPort;
use userlib::ring::io_status;
use userlib::vfs::{
    FsRegister, VfsHeader, fs_type, msg, error,
    ListDir, ReadFile, MakeDir, Remove, WriteFile,
    DirEntries, DirEntry, FileData, file_type,
    Result as VfsResult,
};
use userlib::error::SysError;
use userlib::sync::SingleThreadCell;

// =============================================================================
// Logging - uses centralized macros from userlib
// =============================================================================

macro_rules! flog {
    ($($arg:tt)*) => { userlib::klog_info!("fatfs", $($arg)*) };
}

macro_rules! ferror {
    ($($arg:tt)*) => { userlib::klog_error!("fatfs", $($arg)*) };
}

// =============================================================================
// FAT Filesystem Structures
// =============================================================================

/// FAT filesystem type
#[derive(Clone, Copy, Debug, PartialEq)]
enum FatType {
    Fat16,
    Fat32,
    Unknown,
}

/// FAT BIOS Parameter Block (parsed from boot sector)
#[derive(Clone, Copy)]
struct FatBpb {
    /// Bytes per sector (usually 512)
    bytes_per_sector: u16,
    /// Sectors per cluster (1, 2, 4, 8, 16, 32, 64, or 128)
    sectors_per_cluster: u8,
    /// Reserved sectors before FAT (usually 1 for FAT16, 32 for FAT32)
    reserved_sectors: u16,
    /// Number of FAT copies (usually 2)
    num_fats: u8,
    /// Root entry count (FAT16 only, 0 for FAT32)
    root_entry_count: u16,
    /// Total sectors (16-bit, 0 if > 65535)
    total_sectors_16: u16,
    /// Sectors per FAT (FAT16)
    fat_size_16: u16,
    /// Total sectors (32-bit)
    total_sectors_32: u32,
    /// Sectors per FAT (FAT32)
    fat_size_32: u32,
    /// Root directory cluster (FAT32 only)
    root_cluster: u32,
}

impl FatBpb {
    fn from_boot_sector(sector: &[u8]) -> Option<Self> {
        if sector.len() < 512 {
            return None;
        }
        // Check boot signature
        if sector[510] != 0x55 || sector[511] != 0xAA {
            return None;
        }
        Some(Self {
            bytes_per_sector: u16::from_le_bytes([sector[11], sector[12]]),
            sectors_per_cluster: sector[13],
            reserved_sectors: u16::from_le_bytes([sector[14], sector[15]]),
            num_fats: sector[16],
            root_entry_count: u16::from_le_bytes([sector[17], sector[18]]),
            total_sectors_16: u16::from_le_bytes([sector[19], sector[20]]),
            fat_size_16: u16::from_le_bytes([sector[22], sector[23]]),
            total_sectors_32: u32::from_le_bytes([sector[32], sector[33], sector[34], sector[35]]),
            fat_size_32: u32::from_le_bytes([sector[36], sector[37], sector[38], sector[39]]),
            root_cluster: u32::from_le_bytes([sector[44], sector[45], sector[46], sector[47]]),
        })
    }

    fn fat_size(&self) -> u32 {
        if self.fat_size_16 != 0 {
            self.fat_size_16 as u32
        } else {
            self.fat_size_32
        }
    }

    fn total_sectors(&self) -> u32 {
        if self.total_sectors_16 != 0 {
            self.total_sectors_16 as u32
        } else {
            self.total_sectors_32
        }
    }
}

/// FAT filesystem state
#[derive(Clone)]
struct FatState {
    /// FAT type (FAT16 or FAT32)
    fat_type: FatType,
    /// Parsed BPB
    bpb: FatBpb,
    /// First FAT sector
    fat_start: u32,
    /// First data sector
    data_start: u32,
    /// Root directory sector (FAT16 only)
    root_dir_sector: u32,
    /// Root directory sectors (FAT16 only)
    root_dir_sectors: u32,
}

impl FatState {
    fn from_bpb(bpb: FatBpb) -> Self {
        let fat_start = bpb.reserved_sectors as u32;
        let root_dir_sectors = ((bpb.root_entry_count as u32 * 32) + (bpb.bytes_per_sector as u32 - 1))
            / bpb.bytes_per_sector as u32;
        let data_start = fat_start + (bpb.num_fats as u32 * bpb.fat_size()) + root_dir_sectors;

        // Determine FAT type by cluster count
        let total_data_sectors = bpb.total_sectors()
            .saturating_sub(fat_start)
            .saturating_sub(bpb.num_fats as u32 * bpb.fat_size())
            .saturating_sub(root_dir_sectors);
        let total_clusters = total_data_sectors / bpb.sectors_per_cluster as u32;

        let fat_type = if total_clusters < 4085 {
            FatType::Unknown // FAT12, not supported
        } else if total_clusters < 65525 {
            FatType::Fat16
        } else {
            FatType::Fat32
        };

        Self {
            fat_type,
            bpb,
            fat_start,
            data_start,
            root_dir_sector: fat_start + bpb.num_fats as u32 * bpb.fat_size(),
            root_dir_sectors,
        }
    }

    /// Convert cluster number to sector number
    fn cluster_to_sector(&self, cluster: u32) -> u32 {
        self.data_start + (cluster - 2) * self.bpb.sectors_per_cluster as u32
    }

    /// Get the root directory first sector
    fn root_dir_first_sector(&self) -> u32 {
        match self.fat_type {
            FatType::Fat16 => self.root_dir_sector,
            FatType::Fat32 => self.cluster_to_sector(self.bpb.root_cluster),
            _ => 0,
        }
    }
}

/// FAT directory entry (32 bytes)
#[derive(Clone, Copy)]
struct FatDirEntry {
    name: [u8; 11],
    attr: u8,
    cluster_hi: u16,
    cluster_lo: u16,
    size: u32,
}

impl FatDirEntry {
    fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 32 {
            return None;
        }
        Some(Self {
            name: [
                data[0], data[1], data[2], data[3], data[4], data[5], data[6],
                data[7], data[8], data[9], data[10],
            ],
            attr: data[11],
            cluster_hi: u16::from_le_bytes([data[20], data[21]]),
            cluster_lo: u16::from_le_bytes([data[26], data[27]]),
            size: u32::from_le_bytes([data[28], data[29], data[30], data[31]]),
        })
    }

    fn is_free(&self) -> bool {
        self.name[0] == 0x00 || self.name[0] == 0xE5
    }

    fn is_end(&self) -> bool {
        self.name[0] == 0x00
    }

    fn is_lfn(&self) -> bool {
        self.attr == 0x0F
    }

    fn is_directory(&self) -> bool {
        (self.attr & 0x10) != 0
    }

    fn is_volume_label(&self) -> bool {
        (self.attr & 0x08) != 0
    }

    fn is_hidden(&self) -> bool {
        (self.attr & 0x02) != 0
    }

    fn first_cluster(&self) -> u32 {
        ((self.cluster_hi as u32) << 16) | (self.cluster_lo as u32)
    }

    /// Convert 8.3 name to display name (without trailing spaces)
    fn display_name(&self, buf: &mut [u8]) -> usize {
        let mut len = 0;

        // Copy base name (first 8 chars), trimming trailing spaces
        let mut base_end = 8;
        while base_end > 0 && self.name[base_end - 1] == b' ' {
            base_end -= 1;
        }
        for i in 0..base_end {
            if len < buf.len() {
                buf[len] = self.name[i];
                len += 1;
            }
        }

        // Copy extension (last 3 chars), trimming trailing spaces
        let mut ext_end = 11;
        while ext_end > 8 && self.name[ext_end - 1] == b' ' {
            ext_end -= 1;
        }
        if ext_end > 8 {
            if len < buf.len() {
                buf[len] = b'.';
                len += 1;
            }
            for i in 8..ext_end {
                if len < buf.len() {
                    buf[len] = self.name[i];
                    len += 1;
                }
            }
        }

        len
    }
}

// =============================================================================
// FAT Driver
// =============================================================================

struct FatfsDriver {
    /// Connection to devd
    devd_client: Option<DevdClient>,
    /// Connection to vfsd
    vfs_channel: Option<Channel>,
    /// Connection to block device (legacy IPC)
    block_client: Option<BlockClient>,
    /// Connection to block device (DataPort ring protocol - zero-copy)
    data_port: Option<DataPort>,
    /// Using DataPort (true) or legacy BlockClient (false)
    use_data_port: bool,
    /// FAT filesystem state
    fat_state: Option<FatState>,
    /// Instance number (for mount point naming, derived from PID)
    instance: u8,
}

impl FatfsDriver {
    const fn new() -> Self {
        Self {
            devd_client: None,
            vfs_channel: None,
            block_client: None,
            data_port: None,
            use_data_port: false,
            fat_state: None,
            instance: 0,
        }
    }

    fn init(&mut self) -> Result<(), SysError> {
        let pid = syscall::getpid();
        self.instance = (pid % 100) as u8;

        flog!("connecting to devd (instance {})", self.instance);

        let client = DevdClient::connect()?;
        self.devd_client = Some(client);

        flog!("connected to devd");

        // Connect to block device
        self.connect_to_block_device()?;

        // Connect to vfsd
        self.connect_to_vfsd()?;

        Ok(())
    }

    /// Try to connect to block device via DataPort (zero-copy path)
    fn try_connect_dataport(&mut self) -> Result<(), SysError> {
        // Get spawn context to find which partition port we should connect to
        let port_name: Option<([u8; 64], usize)> = if let Some(client) = self.devd_client.as_mut() {
            match client.get_spawn_context() {
                Ok(Some((name, len, _ptype))) => Some((name, len)),
                _ => None,
            }
        } else {
            None
        };

        // Query devd for the port's DataPort shmem_id
        let shmem_id = if let Some((name, len)) = port_name {
            if let Some(client) = self.devd_client.as_mut() {
                match client.query_port_shmem_id(&name[..len]) {
                    Ok(Some(id)) => {
                        flog!("devd returned shmem_id={} for {}",
                            id, core::str::from_utf8(&name[..len]).unwrap_or("?"));
                        id
                    }
                    Ok(None) => {
                        flog!("port {} has no DataPort", core::str::from_utf8(&name[..len]).unwrap_or("?"));
                        return Err(SysError::NotFound);
                    }
                    Err(e) => {
                        flog!("devd query failed: {:?}", e);
                        return Err(e);
                    }
                }
            } else {
                return Err(SysError::ConnectionRefused);
            }
        } else {
            flog!("no spawn context for DataPort discovery");
            return Err(SysError::NotFound);
        };

        flog!("trying DataPort connection shmem_id={}", shmem_id);

        let mut port = DataPort::connect(shmem_id)?;

        // Query geometry
        match port.query_geometry() {
            Some(info) => {
                flog!("DataPort geometry: {} bytes/sector, {} sectors",
                    info.block_size, info.block_count);
            }
            None => {
                flog!("DataPort: geometry query failed, using defaults");
            }
        }

        self.data_port = Some(port);
        self.use_data_port = true;
        Ok(())
    }

    /// Read blocks via DataPort (zero-copy path)
    fn read_blocks_dataport(&mut self, lba: u64, count: u32, buf: &mut [u8]) -> Result<(), SysError> {
        let port = self.data_port.as_mut().ok_or(SysError::ConnectionRefused)?;

        let len = (count * 512) as u32;
        let offset = port.alloc(len).ok_or(SysError::OutOfMemory)?;

        // Use partition index 0 in param field
        let tag = {
            let t = port.next_tag();
            let sqe = userlib::ring::IoSqe {
                opcode: userlib::ring::io_op::READ,
                flags: 0,
                priority: 0,
                tag: t,
                lba,
                data_offset: offset,
                data_len: len,
                param: 0, // partition 0
            };
            if !port.submit(&sqe) {
                return Err(SysError::IoError);
            }
            t
        };

        port.notify();

        // Poll for completion
        for _ in 0..1000 {
            if let Some(cqe) = port.poll_cq() {
                if cqe.tag == tag {
                    if cqe.status == io_status::OK {
                        if let Some(slice) = port.pool_slice(offset, len) {
                            let copy_len = buf.len().min(len as usize);
                            buf[..copy_len].copy_from_slice(&slice[..copy_len]);
                            return Ok(());
                        }
                    }
                    return Err(SysError::IoError);
                }
            }
            userlib::syscall::sleep_us(1000);
        }

        ferror!("DataPort read timeout");
        Err(SysError::Timeout)
    }

    /// Unified read method - uses DataPort or legacy IPC depending on connection type
    fn read_sectors(&mut self, lba: u64, count: u32, buf: &mut [u8]) -> Result<(), SysError> {
        if self.use_data_port {
            self.read_blocks_dataport(lba, count, buf)
        } else if let Some(block) = self.block_client.as_mut() {
            block.read_blocks(lba, count, buf).map(|_| ()).map_err(|_| SysError::IoError)
        } else {
            Err(SysError::ConnectionRefused)
        }
    }

    fn connect_to_block_device(&mut self) -> Result<(), SysError> {
        flog!("connecting to block device");

        // Try DataPort first (zero-copy path)
        if self.try_connect_dataport().is_ok() {
            flog!("connected via DataPort (zero-copy)");

            // Read boot sector via DataPort
            let mut boot_sector = [0u8; 512];
            self.read_blocks_dataport(0, 1, &mut boot_sector)?;

            // Parse BPB and set up FAT state
            let bpb = match FatBpb::from_boot_sector(&boot_sector) {
                Some(b) => b,
                None => {
                    ferror!("invalid boot sector");
                    return Err(SysError::IoError);
                }
            };

            flog!("BPB: {} bytes/sector, {} sectors/cluster", bpb.bytes_per_sector, bpb.sectors_per_cluster);

            let fat_state = FatState::from_bpb(bpb);
            flog!("FAT type: {:?}", fat_state.fat_type);

            if fat_state.fat_type == FatType::Unknown {
                ferror!("unsupported FAT type");
                return Err(SysError::IoError);
            }

            self.fat_state = Some(fat_state);
            return Ok(());
        }

        flog!("DataPort failed, trying legacy IPC");

        // First, try to get spawn context from devd (tells us which port triggered our spawn)
        let port_name: Option<([u8; 64], usize)> = if let Some(client) = self.devd_client.as_mut() {
            match client.get_spawn_context() {
                Ok(Some((name, len, _ptype))) => {
                    flog!("spawn context: port={}", core::str::from_utf8(&name[..len]).unwrap_or("?"));
                    Some((name, len))
                }
                Ok(None) => {
                    flog!("no spawn context (manual start?)");
                    None
                }
                Err(e) => {
                    flog!("get_spawn_context failed: {:?}", e);
                    None
                }
            }
        } else {
            None
        };

        // Try to connect to the spawn context port first, then fall back to probing
        let mut block = if let Some((name, len)) = port_name {
            match BlockClient::connect(&name[..len]) {
                Ok(b) => b,
                Err(e) => {
                    flog!("failed to connect to spawn context port: {:?}", e);
                    // Fall through to probing
                    self.probe_block_device()?
                }
            }
        } else {
            // No spawn context - probe for available block devices
            self.probe_block_device()?
        };

        // Get block device info
        let (block_size, block_count) = match block.info() {
            Ok(info) => info,
            Err(e) => {
                ferror!("failed to get block info: {:?}", e);
                return Err(SysError::IoError);
            }
        };

        flog!("block device: {} sectors, {} bytes/sector", block_count, block_size);

        // Read boot sector
        let mut boot_sector = [0u8; 512];
        if let Err(e) = block.read_blocks(0, 1, &mut boot_sector) {
            ferror!("failed to read boot sector: {:?}", e);
            return Err(SysError::IoError);
        }

        // Parse BPB
        let bpb = match FatBpb::from_boot_sector(&boot_sector) {
            Some(b) => b,
            None => {
                ferror!("invalid boot sector");
                return Err(SysError::IoError);
            }
        };

        flog!("BPB: {} bytes/sector, {} sectors/cluster", bpb.bytes_per_sector, bpb.sectors_per_cluster);
        flog!("BPB: {} reserved, {} FATs, {} FAT size", bpb.reserved_sectors, bpb.num_fats, bpb.fat_size());

        let fat_state = FatState::from_bpb(bpb);

        flog!("FAT type: {:?}", fat_state.fat_type);
        flog!("root dir sector: {}", fat_state.root_dir_first_sector());

        if fat_state.fat_type == FatType::Unknown {
            ferror!("unsupported FAT type (FAT12?)");
            return Err(SysError::IoError);
        }

        self.fat_state = Some(fat_state);
        self.block_client = Some(block);

        Ok(())
    }

    /// Probe for available block devices (fallback when no spawn context)
    fn probe_block_device(&self) -> Result<BlockClient, SysError> {
        // Try common port names in order
        let port_names: [&[u8]; 4] = [b"part0:", b"disk0:", b"part1:", b"msc0:"];

        for port_name in &port_names {
            flog!("probing {}", core::str::from_utf8(port_name).unwrap_or("?"));

            // Retry a few times in case driver isn't ready yet
            for retry in 0..10 {
                match BlockClient::connect(port_name) {
                    Ok(b) => {
                        flog!("connected to {}", core::str::from_utf8(port_name).unwrap_or("?"));
                        return Ok(b);
                    }
                    Err(_) => {
                        if retry < 9 {
                            syscall::sleep_ms(100);
                        }
                    }
                }
            }
        }

        ferror!("no block device found");
        Err(SysError::NotFound)
    }

    fn connect_to_vfsd(&mut self) -> Result<(), SysError> {
        flog!("connecting to vfsd");

        let mut channel = Channel::connect(b"vfs:")?;
        flog!("connected to vfsd");

        // Build mount point path: /mnt/fat0, /mnt/fat1, etc.
        let mut mount_path = [0u8; 16];
        let prefix = b"/mnt/fat";
        mount_path[..prefix.len()].copy_from_slice(prefix);
        mount_path[prefix.len()] = b'0' + self.instance;
        let mount_len = prefix.len() + 1;

        // Determine fs_type from FAT state
        let fs = match &self.fat_state {
            Some(s) if s.fat_type == FatType::Fat32 => fs_type::FAT32,
            _ => fs_type::FAT16,
        };

        // Send FS_REGISTER message
        let reg = FsRegister::new(1, fs);
        let mut buf = [0u8; 64];
        if let Some(len) = reg.write_to(&mut buf, &mount_path[..mount_len], b"") {
            if let Err(e) = channel.send(&buf[..len]) {
                ferror!("FS_REGISTER send failed: {:?}", e);
                return Err(e);
            }

            let _ = userlib::ipc::wait_one(channel.handle());
            let mut resp_buf = [0u8; 32];
            match channel.recv(&mut resp_buf) {
                Ok(n) if n >= 8 => {
                    if let Some(header) = VfsHeader::from_bytes(&resp_buf[..n]) {
                        if header.msg_type == msg::RESULT {
                            let code = i32::from_le_bytes([
                                resp_buf[8], resp_buf[9], resp_buf[10], resp_buf[11]
                            ]);
                            if code == error::OK {
                                let path_str = core::str::from_utf8(&mount_path[..mount_len]).unwrap_or("?");
                                flog!("registered with vfsd at {}", path_str);
                            } else {
                                ferror!("FS_REGISTER failed: code={}", code);
                            }
                        }
                    }
                }
                Ok(_) => ferror!("FS_REGISTER response too short"),
                Err(e) => ferror!("FS_REGISTER recv failed: {:?}", e),
            }
        }

        self.vfs_channel = Some(channel);
        Ok(())
    }

    fn run(&mut self) -> ! {
        flog!("fatfs driver starting");

        // Report ready state to devd
        if let Some(ref mut client) = self.devd_client {
            let _ = client.report_state(DriverState::Ready);
            flog!("reported ready, entering service loop");
        }

        // Set up mux for event-driven I/O
        let mux = match Mux::new() {
            Ok(m) => m,
            Err(e) => {
                ferror!("mux creation failed: {:?}", e);
                syscall::exit(1);
            }
        };

        // Get handles
        let devd_handle = self.devd_client.as_ref().map(|c| c.handle()).flatten();
        let vfs_handle = self.vfs_channel.as_ref().map(|c| c.handle());

        // Add handles to mux
        if let Some(h) = devd_handle {
            let _ = mux.add(h, MuxFilter::Readable);
        }
        if let Some(h) = vfs_handle {
            let _ = mux.add(h, MuxFilter::Readable);
        }

        // Event loop
        loop {
            let event = match mux.wait() {
                Ok(e) => e,
                Err(_) => {
                    syscall::sleep_ms(10);
                    continue;
                }
            };

            // Check if it's from vfsd
            if Some(event.handle) == vfs_handle {
                self.handle_vfs_message();
                continue;
            }

            // Check if it's from devd (ignore for now)
            if Some(event.handle) == devd_handle {
                if let Some(ref mut client) = self.devd_client {
                    let _ = client.poll_command();
                }
            }
        }
    }

    fn handle_vfs_message(&mut self) {
        let channel = match self.vfs_channel.as_mut() {
            Some(c) => c,
            None => return,
        };

        let mut buf = [0u8; 256];
        let len = match channel.recv(&mut buf) {
            Ok(n) => n,
            Err(_) => return,
        };

        if len < VfsHeader::SIZE {
            return;
        }

        let header = match VfsHeader::from_bytes(&buf[..len]) {
            Some(h) => h,
            None => return,
        };

        match header.msg_type {
            msg::FS_LIST => self.handle_fs_list(header.seq_id, &buf[..len]),
            msg::FS_READ => self.handle_fs_read(header.seq_id, &buf[..len]),
            msg::FS_WRITE => self.handle_fs_write(header.seq_id, &buf[..len]),
            msg::FS_MKDIR => self.handle_fs_mkdir(header.seq_id, &buf[..len]),
            msg::FS_REMOVE => self.handle_fs_remove(header.seq_id, &buf[..len]),
            msg::RESULT => {}
            _ => flog!("unknown message type: {}", header.msg_type),
        }
    }

    fn handle_fs_list(&mut self, seq_id: u32, buf: &[u8]) {
        let channel_handle = match self.vfs_channel.as_ref() {
            Some(c) => c.handle(),
            None => return,
        };

        let (_req, path) = match ListDir::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        flog!("FS_LIST: {:?}", core::str::from_utf8(path));

        // Read root directory
        let entries = self.read_directory(path);

        // Build response
        let mut resp_buf = [0u8; 512];

        // Count valid entries
        let mut entry_count = 0;
        for (name, ftype, _) in &entries {
            if *ftype != 0 || name[0] != 0 {
                entry_count += 1;
            } else {
                break;
            }
        }

        let resp = DirEntries::new(seq_id, entry_count as u16, false);
        let header_len = match resp.write_header(&mut resp_buf) {
            Some(n) => n,
            None => return,
        };

        let mut offset = header_len;
        for (name, ftype, size) in &entries[..entry_count] {
            let entry = DirEntry::new(*ftype, *size);
            if let Some(entry_len) = entry.write_to(&mut resp_buf[offset..], name) {
                offset += entry_len;
            }
        }

        let _ = syscall::write(channel_handle, &resp_buf[..offset]);
    }

    /// Read directory entries from FAT filesystem
    fn read_directory(&mut self, path: &[u8]) -> [([u8; 13], u8, u64); 16] {
        let mut entries: [([u8; 13], u8, u64); 16] = [([0u8; 13], 0, 0); 16];
        let mut count = 0;

        let fat_state = match &self.fat_state {
            Some(s) => s.clone(),
            None => return entries,
        };

        // For now, only support root directory
        if path != b"/" && path != b"" {
            return entries;
        }

        // Read root directory sectors
        // NOTE: IPC payload limit is 576 bytes, so we can only read 1 sector at a time
        // (512 data + 16 header = 528 bytes fits in 576)
        let root_sector = fat_state.root_dir_first_sector();
        let sectors_to_read = if fat_state.fat_type == FatType::Fat16 {
            fat_state.root_dir_sectors.min(4)
        } else {
            fat_state.bpb.sectors_per_cluster as u32
        };

        let mut sector_buf = [0u8; 4096];
        let mut total_read = 0usize;

        // Read one sector at a time due to IPC size limit (for legacy path)
        // DataPort can handle larger reads but we keep consistent behavior
        for i in 0..sectors_to_read {
            let offset = i as usize * 512;
            if self.read_sectors(root_sector as u64 + i as u64, 1, &mut sector_buf[offset..offset + 512]).is_err() {
                if i == 0 {
                    ferror!("failed to read root directory sector {}", i);
                    return entries;
                }
                break; // Partial read is OK
            }
            total_read += 512;
        }

        let read_len = total_read;

        // Parse directory entries
        let mut offset = 0;
        while offset + 32 <= read_len && count < 16 {
            let entry = match FatDirEntry::from_bytes(&sector_buf[offset..offset + 32]) {
                Some(e) => e,
                None => break,
            };

            if entry.is_end() {
                break;
            }

            if entry.is_free() || entry.is_lfn() || entry.is_volume_label() || entry.is_hidden() {
                offset += 32;
                continue;
            }

            // Convert name
            let mut name_buf = [0u8; 13];
            let name_len = entry.display_name(&mut name_buf);

            let ftype = if entry.is_directory() {
                file_type::DIRECTORY
            } else {
                file_type::FILE
            };

            entries[count] = (name_buf, ftype, entry.size as u64);
            count += 1;
            offset += 32;
        }

        entries
    }

    fn handle_fs_read(&mut self, seq_id: u32, buf: &[u8]) {
        let channel_handle = match self.vfs_channel.as_ref() {
            Some(c) => c.handle(),
            None => return,
        };

        let (req, path) = match ReadFile::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        flog!("FS_READ: {:?}", core::str::from_utf8(path));

        // IPC payload limit is ~576 bytes, FileData header is 16 bytes
        // Limit data to 512 bytes per response (one sector, fits comfortably)
        const MAX_DATA_PER_RESPONSE: u32 = 512;
        let limited_len = req.len.min(MAX_DATA_PER_RESPONSE);

        // Find file in root directory and read it
        match self.read_file(path, req.offset, limited_len) {
            Some((data, data_len)) => {
                // Response buffer: 16 byte header + up to 512 bytes data
                let mut resp_buf = [0u8; 512 + 64];
                let eof = data_len < limited_len as usize;
                let resp = FileData::new(seq_id, data_len as u32, eof);
                if let Some(len) = resp.write_to(&mut resp_buf, &data[..data_len]) {
                    let _ = syscall::write(channel_handle, &resp_buf[..len]);
                }
            }
            None => {
                let resp = VfsResult::new(seq_id, error::NOT_FOUND);
                let _ = syscall::write(channel_handle, &resp.to_bytes());
            }
        }
    }

    /// Read file data from FAT filesystem
    /// Returns (data buffer, actual length read)
    fn read_file(&mut self, path: &[u8], offset: u64, max_len: u32) -> Option<([u8; 4096], usize)> {
        let fat_state = match &self.fat_state {
            Some(s) => s.clone(),
            None => return None,
        };

        // Strip leading slash
        let filename = if path.starts_with(b"/") {
            &path[1..]
        } else {
            path
        };

        // Read root directory to find file
        let root_sector = fat_state.root_dir_first_sector();
        let sectors_to_read = if fat_state.fat_type == FatType::Fat16 {
            fat_state.root_dir_sectors.min(4)
        } else {
            fat_state.bpb.sectors_per_cluster as u32
        };

        let mut sector_buf = [0u8; 4096];
        let read_len = (sectors_to_read as usize * 512).min(4096);

        // Read directory one sector at a time for consistency
        for i in 0..sectors_to_read.min(8) {
            let buf_offset = i as usize * 512;
            if self.read_sectors(root_sector as u64 + i as u64, 1, &mut sector_buf[buf_offset..buf_offset + 512]).is_err() {
                if i == 0 {
                    return None;
                }
                break;
            }
        }

        // Find the file
        let mut file_entry: Option<FatDirEntry> = None;
        let mut dir_offset = 0;
        while dir_offset + 32 <= read_len {
            let entry = match FatDirEntry::from_bytes(&sector_buf[dir_offset..dir_offset + 32]) {
                Some(e) => e,
                None => break,
            };

            if entry.is_end() {
                break;
            }

            if entry.is_free() || entry.is_lfn() || entry.is_volume_label() || entry.is_directory() {
                dir_offset += 32;
                continue;
            }

            // Compare names (case-insensitive)
            let mut name_buf = [0u8; 13];
            let name_len = entry.display_name(&mut name_buf);

            if name_len == filename.len() && names_match(&name_buf[..name_len], filename) {
                file_entry = Some(entry);
                break;
            }

            dir_offset += 32;
        }

        let entry = file_entry?;

        if entry.first_cluster() < 2 {
            return None; // Invalid cluster
        }

        // Read file data from first cluster
        let cluster_sector = fat_state.cluster_to_sector(entry.first_cluster());
        let cluster_size = fat_state.bpb.sectors_per_cluster as u32 * fat_state.bpb.bytes_per_sector as u32;

        // For now, just read first cluster (up to 4KB)
        let mut file_buf = [0u8; 4096];
        let read_sectors = (cluster_size / 512).min(8);

        // Read file data one sector at a time for consistency
        for i in 0..read_sectors {
            let buf_offset = i as usize * 512;
            if self.read_sectors(cluster_sector as u64 + i as u64, 1, &mut file_buf[buf_offset..buf_offset + 512]).is_err() {
                if i == 0 {
                    return None;
                }
                break;
            }
        }

        // Apply offset and length limits
        let file_size = entry.size as usize;
        let start = (offset as usize).min(file_size);
        let end = (start + max_len as usize).min(file_size).min(4096);

        // Copy to output buffer with offset applied
        let mut result = [0u8; 4096];
        let len = end.saturating_sub(start);
        result[..len].copy_from_slice(&file_buf[start..end]);

        Some((result, len))
    }

    fn handle_fs_write(&mut self, seq_id: u32, _buf: &[u8]) {
        let channel = match self.vfs_channel.as_ref() {
            Some(c) => c,
            None => return,
        };

        flog!("FS_WRITE: not implemented (read-only)");
        let resp = VfsResult::new(seq_id, error::PERMISSION_DENIED);
        let _ = channel.send(&resp.to_bytes());
    }

    fn handle_fs_mkdir(&mut self, seq_id: u32, _buf: &[u8]) {
        let channel = match self.vfs_channel.as_ref() {
            Some(c) => c,
            None => return,
        };

        flog!("FS_MKDIR: not implemented (read-only)");
        let resp = VfsResult::new(seq_id, error::PERMISSION_DENIED);
        let _ = channel.send(&resp.to_bytes());
    }

    fn handle_fs_remove(&mut self, seq_id: u32, _buf: &[u8]) {
        let channel = match self.vfs_channel.as_ref() {
            Some(c) => c,
            None => return,
        };

        flog!("FS_REMOVE: not implemented (read-only)");
        let resp = VfsResult::new(seq_id, error::PERMISSION_DENIED);
        let _ = channel.send(&resp.to_bytes());
    }
}

/// Case-insensitive name comparison
fn names_match(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    for i in 0..a.len() {
        let ca = if a[i] >= b'a' && a[i] <= b'z' { a[i] - 32 } else { a[i] };
        let cb = if b[i] >= b'a' && b[i] <= b'z' { b[i] - 32 } else { b[i] };
        if ca != cb {
            return false;
        }
    }
    true
}

// =============================================================================
// Main
// =============================================================================

static DRIVER: SingleThreadCell<FatfsDriver> = SingleThreadCell::new();

#[unsafe(no_mangle)]
fn main() {
    syscall::klog(LogLevel::Info, b"[fatfs] starting");

    DRIVER.init(FatfsDriver::new());

    {
        let mut driver = DRIVER.borrow_mut();
        if let Err(e) = driver.init() {
            ferror!("init failed: {:?}", e);
            syscall::exit(1);
        }
    }

    // Run loop
    DRIVER.borrow_mut().run()
}
