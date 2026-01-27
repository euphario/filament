//! FAT Filesystem Driver
//!
//! Userspace FAT12/16/32 filesystem driver that reads from block storage.
//! Spawned by devd when a Partition port is registered.
//!
//! ## Flow
//!
//! 1. partition driver registers partX: ports
//! 2. devd spawns fatfs for each partition
//! 3. fatfs probes partition for FAT signature
//! 4. If FAT found, registers fsX: port for VFS access
//!
//! ## Current Status
//!
//! This is a stub that integrates with the new devd bidirectional protocol.
//! Full FAT implementation pending.

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel, Handle};
use userlib::ipc::{Channel, Mux, MuxFilter};
use userlib::devd::{DevdClient, DriverState};
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
// FAT Driver
// =============================================================================

struct FatfsDriver {
    /// Connection to devd
    devd_client: Option<DevdClient>,
    /// Connection to vfsd
    vfs_channel: Option<Channel>,
    /// Instance number (for mount point naming, derived from PID)
    instance: u8,
}

impl FatfsDriver {
    const fn new() -> Self {
        Self {
            devd_client: None,
            vfs_channel: None,
            instance: 0,
        }
    }

    fn init(&mut self) -> Result<(), SysError> {
        // Use PID as unique instance number
        // PIDs are small and unique, so they work well for mount point naming
        let pid = syscall::getpid();
        self.instance = (pid % 100) as u8;  // Keep it small for path

        flog!("connecting to devd (instance {})", self.instance);

        let client = DevdClient::connect()?;
        self.devd_client = Some(client);

        flog!("connected to devd");

        // Connect to vfsd
        self.connect_to_vfsd()?;

        Ok(())
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

        // Send FS_REGISTER message
        let reg = FsRegister::new(1, fs_type::FAT32); // seq_id=1, assuming FAT32 for now
        let mut buf = [0u8; 64];
        if let Some(len) = reg.write_to(&mut buf, &mount_path[..mount_len], b"") {
            if let Err(e) = channel.send(&buf[..len]) {
                ferror!("FS_REGISTER send failed: {:?}", e);
                return Err(e);
            }

            // Wait for response (block until readable)
            let _ = userlib::ipc::wait_one(channel.handle());
            let mut resp_buf = [0u8; 32];
            match channel.recv(&mut resp_buf) {
                Ok(n) if n >= 8 => {
                    // Parse response header
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

    fn probe_partition(&mut self) -> Result<(), SysError> {
        // TODO: Connect to partition port and read boot sector
        // For now, just log that we're probing
        flog!("probing partition for FAT signature");

        // TODO: Read sector 0, check for FAT signature
        // If FAT found:
        //   - Parse BPB (BIOS Parameter Block)
        //   - Determine FAT type (12/16/32)
        //   - Register fsX: port with devd

        flog!("FAT probe: not yet implemented");
        Ok(())
    }

    fn run(&mut self) -> ! {
        flog!("fatfs driver starting");

        // Probe the partition
        if let Err(e) = self.probe_partition() {
            ferror!("probe failed: {:?}", e);
        }

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

            // Check if it's from devd (ignore for now - no spawn handling needed)
            if Some(event.handle) == devd_handle {
                // Just drain the message
                let mut buf = [0u8; 64];
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
            msg::RESULT => {
                // Ignore RESULT messages (response to our FS_REGISTER)
            }
            _ => {
                flog!("unknown message type: {}", header.msg_type);
            }
        }
    }

    fn handle_fs_list(&mut self, seq_id: u32, buf: &[u8]) {
        let channel = match self.vfs_channel.as_ref() {
            Some(c) => c,
            None => return,
        };

        let (_req, path) = match ListDir::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        flog!("FS_LIST: {:?}", core::str::from_utf8(path));

        // Return test directory entries
        // TODO: Actually read FAT directory from block device
        let entries = [
            ("README.TXT", file_type::FILE, 1234u64),
            ("DOCS", file_type::DIRECTORY, 0),
            ("CONFIG.SYS", file_type::FILE, 256),
        ];

        // Build response
        let mut resp_buf = [0u8; 256];
        let resp = DirEntries::new(seq_id, entries.len() as u16, false);
        let header_len = match resp.write_header(&mut resp_buf) {
            Some(n) => n,
            None => return,
        };

        let mut offset = header_len;
        for (name, ftype, size) in &entries {
            let entry = DirEntry::new(*ftype, *size);
            if let Some(entry_len) = entry.write_to(&mut resp_buf[offset..], name.as_bytes()) {
                offset += entry_len;
            }
        }

        let _ = channel.send(&resp_buf[..offset]);
    }

    fn handle_fs_read(&mut self, seq_id: u32, buf: &[u8]) {
        let channel = match self.vfs_channel.as_ref() {
            Some(c) => c,
            None => return,
        };

        let (_req, path) = match ReadFile::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        flog!("FS_READ: {:?}", core::str::from_utf8(path));

        // Return stub file content based on filename
        // TODO: Actually read from FAT filesystem
        let content: &[u8] = if path == b"/README.TXT" || path == b"README.TXT" {
            b"Welcome to BPI-R4 FAT filesystem!\n\nThis is a stub implementation.\nReal FAT reading coming soon.\n"
        } else if path == b"/CONFIG.SYS" || path == b"CONFIG.SYS" {
            b"DEVICE=STUB.SYS\nBUFFERS=16\n"
        } else {
            // File not found - send error result
            let resp = userlib::vfs::Result::new(seq_id, error::NOT_FOUND);
            let _ = channel.send(&resp.to_bytes());
            return;
        };

        // Build FILE_DATA response
        let mut resp_buf = [0u8; 4096 + 64];
        let resp = FileData::new(seq_id, content.len() as u32, true);
        if let Some(len) = resp.write_to(&mut resp_buf, content) {
            let _ = channel.send(&resp_buf[..len]);
        }
    }

    fn handle_fs_write(&mut self, seq_id: u32, buf: &[u8]) {
        let channel = match self.vfs_channel.as_ref() {
            Some(c) => c,
            None => return,
        };

        let (_req, path, data) = match WriteFile::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        flog!("FS_WRITE: {:?} ({} bytes)", core::str::from_utf8(path), data.len());

        // Stub: pretend write succeeded
        // TODO: Actually write to FAT filesystem
        let resp = VfsResult::new(seq_id, error::OK);
        let _ = channel.send(&resp.to_bytes());
    }

    fn handle_fs_mkdir(&mut self, seq_id: u32, buf: &[u8]) {
        let channel = match self.vfs_channel.as_ref() {
            Some(c) => c,
            None => return,
        };

        let (_req, path) = match MakeDir::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        flog!("FS_MKDIR: {:?}", core::str::from_utf8(path));

        // Stub: pretend mkdir succeeded
        // TODO: Actually create directory in FAT filesystem
        let resp = VfsResult::new(seq_id, error::OK);
        let _ = channel.send(&resp.to_bytes());
    }

    fn handle_fs_remove(&mut self, seq_id: u32, buf: &[u8]) {
        let channel = match self.vfs_channel.as_ref() {
            Some(c) => c,
            None => return,
        };

        let (_req, path) = match Remove::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        flog!("FS_REMOVE: {:?}", core::str::from_utf8(path));

        // Stub: pretend remove succeeded
        // TODO: Actually remove from FAT filesystem
        let resp = VfsResult::new(seq_id, error::OK);
        let _ = channel.send(&resp.to_bytes());
    }
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

    // Run loop - borrow for entire lifetime
    DRIVER.borrow_mut().run()
}
