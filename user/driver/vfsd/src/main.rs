//! VFS Daemon - Virtual Filesystem Service
//!
//! Provides unified filesystem namespace for shell and other clients.
//! Coordinates with fatfs instances for actual filesystem operations.
//!
//! ## Architecture
//!
//! ```text
//! shell ←→ vfsd ←→ fatfs (part0)
//!               ←→ fatfs (part1)
//! ```
//!
//! ## Responsibilities
//!
//! - Mount table management (path → fatfs mapping)
//! - Path resolution (which fatfs handles which path)
//! - File operations coordination (read, write, copy)
//! - Single transfer buffer for copies (simple v1)

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel, Handle};
use userlib::ipc::{Port, Mux, MuxFilter};
use userlib::vfs::{
    VfsHeader, msg, error, file_type,
    ListDir, ReadFile, CopyFile, Stat, MakeDir, Remove, WriteFile,
    DirEntries, DirEntry, FileData, FileInfo, Result as VfsResult,
    FsRegister,
};
use userlib::devd::{DevdClient, DriverState};
use userlib::error::SysError;
use userlib::sync::SingleThreadCell;

// =============================================================================
// Logging - uses centralized macros from userlib
// =============================================================================

macro_rules! vlog {
    ($($arg:tt)*) => { userlib::klog_info!("vfsd", $($arg)*) };
}

macro_rules! verror {
    ($($arg:tt)*) => { userlib::klog_error!("vfsd", $($arg)*) };
}

// =============================================================================
// Constants
// =============================================================================

const MAX_MOUNTS: usize = 8;
const MAX_CLIENTS: usize = 8;
const TRANSFER_BUF_SIZE: usize = 65536; // 64KB transfer buffer

// =============================================================================
// State Enums
// =============================================================================

/// Client connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClientState {
    /// Slot is empty, available for reuse
    Empty,
    /// Client connected, channel active
    Connected,
    /// Client disconnected, pending cleanup
    Disconnected,
}

/// Filesystem mount state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountState {
    /// Slot is empty
    Empty,
    /// Filesystem registered, channel active
    Mounted,
    /// Unmount requested, pending cleanup
    Unmounting,
}

// =============================================================================
// Mount Entry
// =============================================================================

#[derive(Clone, Copy)]
struct MountEntry {
    /// Mount point path (e.g., "/mnt/part0")
    path: [u8; 64],
    path_len: u8,
    /// Filesystem type
    fs_type: u8,
    /// Handle to fatfs channel
    handle: Handle,
    /// Mount state
    state: MountState,
}

impl MountEntry {
    const fn empty() -> Self {
        Self {
            path: [0u8; 64],
            path_len: 0,
            fs_type: 0,
            handle: Handle::INVALID,
            state: MountState::Empty,
        }
    }

    fn is_active(&self) -> bool {
        self.state == MountState::Mounted
    }

    fn path_str(&self) -> &str {
        core::str::from_utf8(&self.path[..self.path_len as usize]).unwrap_or("")
    }

    fn matches(&self, path: &[u8]) -> bool {
        if !self.is_active() || self.path_len == 0 {
            return false;
        }
        let mount_path = &self.path[..self.path_len as usize];
        if path.len() < mount_path.len() {
            return false;
        }
        // Path must start with mount point
        if &path[..mount_path.len()] != mount_path {
            return false;
        }
        // Must be exact match or followed by /
        if path.len() == mount_path.len() {
            return true;
        }
        path[mount_path.len()] == b'/'
    }

    fn relative_path<'a>(&self, path: &'a [u8]) -> &'a [u8] {
        let mount_len = self.path_len as usize;
        if path.len() <= mount_len {
            return b"/";
        }
        &path[mount_len..]
    }
}

// =============================================================================
// Client Entry
// =============================================================================

#[derive(Clone, Copy)]
struct ClientEntry {
    handle: Handle,
    is_fs: bool,  // true = fatfs, false = shell/app
    mount_idx: u8, // if is_fs, which mount this serves
    state: ClientState,
}

impl ClientEntry {
    const fn empty() -> Self {
        Self {
            handle: Handle::INVALID,
            is_fs: false,
            mount_idx: 0,
            state: ClientState::Empty,
        }
    }

    fn is_active(&self) -> bool {
        self.state == ClientState::Connected
    }
}

// =============================================================================
// VFS Daemon
// =============================================================================

struct Vfsd {
    /// Service port (stored to prevent resource leak)
    port: Option<Port>,
    /// Event multiplexer
    mux: Option<Mux>,
    /// Mount table
    mounts: [MountEntry; MAX_MOUNTS],
    mount_count: usize,
    /// Connected clients
    clients: [ClientEntry; MAX_CLIENTS],
    /// Transfer buffer (for file copies)
    transfer_buf: [u8; TRANSFER_BUF_SIZE],
    /// Message buffers
    recv_buf: [u8; 512],
    send_buf: [u8; 512],
}

impl Vfsd {
    const fn new() -> Self {
        Self {
            port: None,
            mux: None,
            mounts: [const { MountEntry::empty() }; MAX_MOUNTS],
            mount_count: 0,
            clients: [const { ClientEntry::empty() }; MAX_CLIENTS],
            transfer_buf: [0u8; TRANSFER_BUF_SIZE],
            recv_buf: [0u8; 512],
            send_buf: [0u8; 512],
        }
    }

    /// Get the port handle, if port is registered.
    fn port_handle(&self) -> Handle {
        self.port.as_ref().map(|p| p.handle()).unwrap_or(Handle::INVALID)
    }

    fn init(&mut self) -> Result<(), SysError> {
        vlog!("initializing");

        // Create mux for event-driven I/O
        let mux = Mux::new()?;

        // Register vfs: port
        let port = Port::register(b"vfs:")?;
        vlog!("port registered: vfs:");

        // Add port to mux
        mux.add(port.handle(), MuxFilter::Readable)?;

        // Store port properly (no more mem::forget leak!)
        self.port = Some(port);
        self.mux = Some(mux);

        // Connect to devd and report ready state
        match DevdClient::connect() {
            Ok(mut client) => {
                let _ = client.report_state(DriverState::Ready);
                vlog!("connected to devd");
            }
            Err(e) => {
                // Non-fatal - vfsd can still work without devd connection
                vlog!("devd connect failed: {:?} (continuing anyway)", e);
            }
        }

        vlog!("ready");
        Ok(())
    }

    fn run(&mut self) -> ! {
        loop {
            // Wait for events
            let event = match self.mux.as_ref().and_then(|m| m.wait().ok()) {
                Some(e) => e,
                None => {
                    syscall::sleep_ms(10);
                    continue;
                }
            };

            // Check if it's a new connection on the port
            if event.handle == self.port_handle() {
                self.handle_new_connection();
                continue;
            }

            // Check if it's a message from a client
            self.handle_client_message(event.handle);
        }
    }

    fn handle_new_connection(&mut self) {
        // Accept the connection by reading from port handle
        // Kernel returns [handle: u32, client_pid: u32] (8 bytes)
        let mut buf = [0u8; 8];
        let ch_handle = match syscall::read(self.port_handle(), &mut buf) {
            Ok(n) if n >= 4 => {
                let raw = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                Handle(raw)
            }
            _ => return,
        };

        vlog!("new connection");

        // Find empty client slot
        let slot = match self.clients.iter().position(|c| c.state == ClientState::Empty) {
            Some(s) => s,
            None => {
                verror!("no client slots available");
                // Close the channel we just accepted
                let _ = syscall::close(ch_handle);
                return;
            }
        };

        // Add channel to mux
        if let Some(ref mux) = self.mux {
            if let Err(e) = mux.add(ch_handle, MuxFilter::Readable) {
                verror!("mux.add failed for client slot {}: {:?}", slot, e);
                let _ = syscall::close(ch_handle);
                return;
            }
        }

        // Store handle
        self.clients[slot] = ClientEntry {
            handle: ch_handle,
            is_fs: false,
            mount_idx: 0,
            state: ClientState::Connected,
        };
    }

    fn handle_client_message(&mut self, handle: Handle) {
        // Find client by handle
        let slot = match self.clients.iter().position(|c| c.is_active() && c.handle == handle) {
            Some(s) => s,
            None => return,
        };

        // Receive message using raw syscall
        let len = match syscall::read(handle, &mut self.recv_buf) {
            Ok(n) => n,
            Err(SysError::WouldBlock) => return,
            Err(SysError::PeerClosed) => {
                self.remove_client(slot);
                return;
            }
            Err(_) => {
                self.remove_client(slot);
                return;
            }
        };

        if len < VfsHeader::SIZE {
            return;
        }

        // Parse header
        let header = match VfsHeader::from_bytes(&self.recv_buf[..len]) {
            Some(h) => h,
            None => return,
        };

        // Copy recv_buf to avoid borrow issues
        let mut msg_buf = [0u8; 512];
        msg_buf[..len].copy_from_slice(&self.recv_buf[..len]);

        // Dispatch based on message type
        match header.msg_type {
            // Filesystem registration (from fatfs)
            msg::FS_REGISTER => self.handle_fs_register(slot, &msg_buf[..len]),

            // Shell commands
            msg::LIST_DIR => self.handle_list_dir(slot, header.seq_id, &msg_buf[..len]),
            msg::READ_FILE => self.handle_read_file(slot, header.seq_id, &msg_buf[..len]),
            msg::WRITE_FILE => self.handle_write_file(slot, header.seq_id, &msg_buf[..len]),
            msg::STAT => self.handle_stat(slot, header.seq_id, &msg_buf[..len]),
            msg::COPY_FILE => self.handle_copy_file(slot, header.seq_id, &msg_buf[..len]),
            msg::MAKE_DIR => self.handle_make_dir(slot, header.seq_id, &msg_buf[..len]),
            msg::REMOVE => self.handle_remove(slot, header.seq_id, &msg_buf[..len]),

            _ => {
                vlog!("unknown message type: {}", header.msg_type);
            }
        }
    }

    fn handle_fs_register(&mut self, client_slot: usize, buf: &[u8]) {
        let (reg, mount_point, _port_name) = match FsRegister::from_bytes(buf) {
            Some(r) => r,
            None => {
                verror!("invalid FS_REGISTER message");
                return;
            }
        };

        // Find empty mount slot
        let mount_slot = match self.mounts.iter().position(|m| m.state == MountState::Empty) {
            Some(s) => s,
            None => {
                verror!("no mount slots available");
                self.send_result(client_slot, reg.header.seq_id, error::IO_ERROR);
                return;
            }
        };

        // Set up mount entry
        let mount = &mut self.mounts[mount_slot];
        let len = mount_point.len().min(64);
        mount.path[..len].copy_from_slice(&mount_point[..len]);
        mount.path_len = len as u8;
        mount.fs_type = reg.fs_type;
        mount.state = MountState::Mounted;

        // Take handle from client - fatfs channel becomes mount's channel
        mount.handle = self.clients[client_slot].handle;

        // Mark client as filesystem
        self.clients[client_slot].is_fs = true;
        self.clients[client_slot].mount_idx = mount_slot as u8;

        self.mount_count += 1;

        let path_str = core::str::from_utf8(mount_point).unwrap_or("?");
        vlog!("mounted: {} (type={})", path_str, reg.fs_type);

        self.send_result(client_slot, reg.header.seq_id, error::OK);
    }

    fn handle_list_dir(&mut self, client_slot: usize, seq_id: u32, buf: &[u8]) {
        let (_req, path) = match ListDir::from_bytes(buf) {
            Some(r) => r,
            None => {
                self.send_result(client_slot, seq_id, error::INVALID_PATH);
                return;
            }
        };

        // Special case: list /mnt to show mounted filesystems
        if path == b"/mnt" || path == b"/mnt/" {
            self.list_mounts(client_slot, seq_id);
            return;
        }

        // Find mount for this path
        let mount_idx = match self.find_mount(path) {
            Some(idx) => idx,
            None => {
                self.send_result(client_slot, seq_id, error::NOT_MOUNTED);
                return;
            }
        };

        // Get relative path within mount
        let mount = &self.mounts[mount_idx];
        let rel_path = mount.relative_path(path);

        // Forward FS_LIST to fatfs
        let fs_handle = mount.handle;
        if !fs_handle.is_valid() {
            self.send_result(client_slot, seq_id, error::IO_ERROR);
            return;
        }

        // Build FS_LIST request
        let req = ListDir {
            header: VfsHeader::new(msg::FS_LIST, seq_id),
            path_len: rel_path.len() as u16,
            _pad: 0,
        };
        let mut req_buf = [0u8; 256];
        if let Some(len) = req.write_to(&mut req_buf, rel_path) {
            // Send to fatfs
            if syscall::write(fs_handle, &req_buf[..len]).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            // Wait for response from fatfs
            if userlib::ipc::wait_one(fs_handle).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            let mut resp_buf = [0u8; 512];
            match syscall::read(fs_handle, &mut resp_buf) {
                Ok(n) if n > 0 => {
                    // Forward response to client
                    self.send_to_client(client_slot, &resp_buf[..n]);
                }
                _ => {
                    self.send_result(client_slot, seq_id, error::IO_ERROR);
                }
            }
        } else {
            self.send_result(client_slot, seq_id, error::INVALID_PATH);
        }
    }


    fn list_mounts(&mut self, client_slot: usize, seq_id: u32) {
        // Count active mounts
        let count = self.mounts.iter().filter(|m| m.is_active()).count() as u16;

        let resp = DirEntries::new(seq_id, count, false);
        if resp.write_header(&mut self.send_buf).is_none() {
            return;
        }

        let mut offset = DirEntries::HEADER_SIZE;

        for mount in &self.mounts {
            if !mount.is_active() {
                continue;
            }

            // Extract last component of mount path as name
            let path = mount.path_str();
            let name = path.rsplit('/').next().unwrap_or(path);

            let entry = DirEntry::new(file_type::DIRECTORY, 0);
            if let Some(entry_len) = entry.write_to(&mut self.send_buf[offset..], name.as_bytes()) {
                offset += entry_len;
            }
        }

        self.send_to_client(client_slot, &self.send_buf[..offset]);
    }

    fn handle_read_file(&mut self, client_slot: usize, seq_id: u32, buf: &[u8]) {
        let (req, path) = match ReadFile::from_bytes(buf) {
            Some(r) => r,
            None => {
                self.send_result(client_slot, seq_id, error::INVALID_PATH);
                return;
            }
        };

        // Find mount for this path
        let mount_idx = match self.find_mount(path) {
            Some(idx) => idx,
            None => {
                self.send_result(client_slot, seq_id, error::NOT_MOUNTED);
                return;
            }
        };

        // Get relative path within mount
        let mount = &self.mounts[mount_idx];
        let rel_path = mount.relative_path(path);
        let fs_handle = mount.handle;

        if !fs_handle.is_valid() {
            self.send_result(client_slot, seq_id, error::IO_ERROR);
            return;
        }

        // Build FS_READ request to forward to fatfs
        let fs_req = ReadFile {
            header: VfsHeader::new(msg::FS_READ, seq_id),
            path_len: rel_path.len() as u16,
            _pad: 0,
            offset: req.offset,
            len: req.len,
            _pad2: 0,
        };
        let mut req_buf = [0u8; 256];
        if let Some(len) = fs_req.write_to(&mut req_buf, rel_path) {
            // Send to fatfs
            if syscall::write(fs_handle, &req_buf[..len]).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            // Wait for response from fatfs
            if userlib::ipc::wait_one(fs_handle).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            // Use transfer buffer for larger responses
            let mut resp_buf = [0u8; 4096 + 64];
            match syscall::read(fs_handle, &mut resp_buf) {
                Ok(n) if n > 0 => {
                    // Forward response to client
                    self.send_to_client(client_slot, &resp_buf[..n]);
                }
                _ => {
                    self.send_result(client_slot, seq_id, error::IO_ERROR);
                }
            }
        } else {
            self.send_result(client_slot, seq_id, error::INVALID_PATH);
        }
    }

    fn handle_stat(&mut self, client_slot: usize, seq_id: u32, buf: &[u8]) {
        let (_req, path) = match Stat::from_bytes(buf) {
            Some(r) => r,
            None => {
                self.send_result(client_slot, seq_id, error::INVALID_PATH);
                return;
            }
        };

        // Special case: /mnt is a directory
        if path == b"/mnt" || path == b"/mnt/" {
            let resp = FileInfo::new(seq_id, file_type::DIRECTORY, 0);
            self.send_to_client(client_slot, &resp.to_bytes());
            return;
        }

        // Find mount for this path
        let mount_idx = match self.find_mount(path) {
            Some(idx) => idx,
            None => {
                self.send_result(client_slot, seq_id, error::NOT_FOUND);
                return;
            }
        };

        // TODO: Forward to fatfs
        // For now, return generic file
        let resp = FileInfo::new(seq_id, file_type::FILE, 0);
        self.send_to_client(client_slot, &resp.to_bytes());

        let _ = mount_idx;
    }

    fn handle_copy_file(&mut self, client_slot: usize, seq_id: u32, buf: &[u8]) {
        let (_req, src_path, dst_path) = match CopyFile::from_bytes(buf) {
            Some(r) => r,
            None => {
                self.send_result(client_slot, seq_id, error::INVALID_PATH);
                return;
            }
        };

        let src_mount = self.find_mount(src_path);
        let dst_mount = self.find_mount(dst_path);

        if src_mount.is_none() || dst_mount.is_none() {
            self.send_result(client_slot, seq_id, error::NOT_MOUNTED);
            return;
        }

        // TODO: Implement actual copy
        // 1. Tell src fatfs to read into transfer_buf
        // 2. Tell dst fatfs to write from transfer_buf
        // 3. Repeat until EOF

        vlog!("copy: {:?} -> {:?} (not implemented)",
            core::str::from_utf8(src_path),
            core::str::from_utf8(dst_path));

        self.send_result(client_slot, seq_id, error::IO_ERROR);
    }

    fn handle_make_dir(&mut self, client_slot: usize, seq_id: u32, buf: &[u8]) {
        let (_req, path) = match MakeDir::from_bytes(buf) {
            Some(r) => r,
            None => {
                self.send_result(client_slot, seq_id, error::INVALID_PATH);
                return;
            }
        };

        // Find mount for this path
        let mount_idx = match self.find_mount(path) {
            Some(idx) => idx,
            None => {
                self.send_result(client_slot, seq_id, error::NOT_MOUNTED);
                return;
            }
        };

        // Get relative path and forward to fatfs
        let mount = &self.mounts[mount_idx];
        let rel_path = mount.relative_path(path);
        let fs_handle = mount.handle;

        if !fs_handle.is_valid() {
            self.send_result(client_slot, seq_id, error::IO_ERROR);
            return;
        }

        // Build FS_MKDIR request
        let fs_req = MakeDir {
            header: VfsHeader::new(msg::FS_MKDIR, seq_id),
            path_len: rel_path.len() as u16,
            _pad: 0,
        };
        let mut req_buf = [0u8; 256];
        if let Some(len) = fs_req.write_to(&mut req_buf, rel_path) {
            if syscall::write(fs_handle, &req_buf[..len]).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            if userlib::ipc::wait_one(fs_handle).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            let mut resp_buf = [0u8; 32];
            match syscall::read(fs_handle, &mut resp_buf) {
                Ok(n) if n > 0 => {
                    self.send_to_client(client_slot, &resp_buf[..n]);
                }
                _ => {
                    self.send_result(client_slot, seq_id, error::IO_ERROR);
                }
            }
        } else {
            self.send_result(client_slot, seq_id, error::INVALID_PATH);
        }
    }

    fn handle_remove(&mut self, client_slot: usize, seq_id: u32, buf: &[u8]) {
        let (req, path) = match Remove::from_bytes(buf) {
            Some(r) => r,
            None => {
                self.send_result(client_slot, seq_id, error::INVALID_PATH);
                return;
            }
        };

        // Find mount for this path
        let mount_idx = match self.find_mount(path) {
            Some(idx) => idx,
            None => {
                self.send_result(client_slot, seq_id, error::NOT_MOUNTED);
                return;
            }
        };

        // Get relative path and forward to fatfs
        let mount = &self.mounts[mount_idx];
        let rel_path = mount.relative_path(path);
        let fs_handle = mount.handle;

        if !fs_handle.is_valid() {
            self.send_result(client_slot, seq_id, error::IO_ERROR);
            return;
        }

        // Build FS_REMOVE request
        let fs_req = Remove {
            header: VfsHeader::new(msg::FS_REMOVE, seq_id),
            path_len: rel_path.len() as u16,
            flags: req.flags,
            _pad: 0,
        };
        let mut req_buf = [0u8; 256];
        if let Some(len) = fs_req.write_to(&mut req_buf, rel_path) {
            if syscall::write(fs_handle, &req_buf[..len]).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            if userlib::ipc::wait_one(fs_handle).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            let mut resp_buf = [0u8; 32];
            match syscall::read(fs_handle, &mut resp_buf) {
                Ok(n) if n > 0 => {
                    self.send_to_client(client_slot, &resp_buf[..n]);
                }
                _ => {
                    self.send_result(client_slot, seq_id, error::IO_ERROR);
                }
            }
        } else {
            self.send_result(client_slot, seq_id, error::INVALID_PATH);
        }
    }

    fn handle_write_file(&mut self, client_slot: usize, seq_id: u32, buf: &[u8]) {
        let (req, path, data) = match WriteFile::from_bytes(buf) {
            Some(r) => r,
            None => {
                self.send_result(client_slot, seq_id, error::INVALID_PATH);
                return;
            }
        };

        // Find mount for this path
        let mount_idx = match self.find_mount(path) {
            Some(idx) => idx,
            None => {
                self.send_result(client_slot, seq_id, error::NOT_MOUNTED);
                return;
            }
        };

        // Get relative path and forward to fatfs
        let mount = &self.mounts[mount_idx];
        let rel_path = mount.relative_path(path);
        let fs_handle = mount.handle;

        if !fs_handle.is_valid() {
            self.send_result(client_slot, seq_id, error::IO_ERROR);
            return;
        }

        // Build FS_WRITE request
        let fs_req = WriteFile {
            header: VfsHeader::new(msg::FS_WRITE, seq_id),
            path_len: rel_path.len() as u16,
            flags: req.flags,
            _pad: 0,
            offset: req.offset,
            data_len: data.len() as u32,
            _pad2: 0,
        };
        let mut req_buf = [0u8; 4096 + 256];
        if let Some(len) = fs_req.write_to(&mut req_buf, rel_path, data) {
            if syscall::write(fs_handle, &req_buf[..len]).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            if userlib::ipc::wait_one(fs_handle).is_err() {
                self.send_result(client_slot, seq_id, error::IO_ERROR);
                return;
            }

            let mut resp_buf = [0u8; 32];
            match syscall::read(fs_handle, &mut resp_buf) {
                Ok(n) if n > 0 => {
                    self.send_to_client(client_slot, &resp_buf[..n]);
                }
                _ => {
                    self.send_result(client_slot, seq_id, error::IO_ERROR);
                }
            }
        } else {
            self.send_result(client_slot, seq_id, error::INVALID_PATH);
        }
    }

    fn find_mount(&self, path: &[u8]) -> Option<usize> {
        // Find longest matching mount point
        let mut best_match: Option<usize> = None;
        let mut best_len = 0;

        for (idx, mount) in self.mounts.iter().enumerate() {
            if mount.matches(path) && mount.path_len as usize > best_len {
                best_match = Some(idx);
                best_len = mount.path_len as usize;
            }
        }

        best_match
    }

    fn send_result(&mut self, client_slot: usize, seq_id: u32, code: i32) {
        let resp = VfsResult::new(seq_id, code);
        self.send_to_client(client_slot, &resp.to_bytes());
    }

    fn send_to_client(&self, client_slot: usize, data: &[u8]) {
        let client = &self.clients[client_slot];
        if !client.is_active() {
            return;
        }

        // If client is a filesystem, its channel is in the mount
        let handle = if client.is_fs {
            let mount_idx = client.mount_idx as usize;
            if mount_idx < MAX_MOUNTS && self.mounts[mount_idx].is_active() {
                self.mounts[mount_idx].handle
            } else {
                return;
            }
        } else {
            client.handle
        };

        let _ = syscall::write(handle, data);
    }

    fn remove_client(&mut self, slot: usize) {
        let client = &self.clients[slot];
        if !client.is_active() {
            return;
        }

        // If this was a filesystem, deactivate its mount
        if client.is_fs {
            let mount_idx = client.mount_idx as usize;
            if mount_idx < MAX_MOUNTS {
                let mount = &mut self.mounts[mount_idx];
                if mount.is_active() {
                    vlog!("unmounted: {}", mount.path_str());
                    mount.state = MountState::Empty;
                    mount.handle = Handle::INVALID;
                    self.mount_count = self.mount_count.saturating_sub(1);
                }
            }
        }

        // Remove from mux
        if let Some(ref mux) = self.mux {
            let _ = mux.remove(client.handle);
        }

        // Close the handle
        let _ = syscall::close(client.handle);

        self.clients[slot] = ClientEntry::empty();
    }
}

// =============================================================================
// Entry Point
// =============================================================================

static VFSD: SingleThreadCell<Vfsd> = SingleThreadCell::new();

#[unsafe(no_mangle)]
fn main() {
    syscall::klog(LogLevel::Info, b"[vfsd] starting");

    VFSD.init(Vfsd::new());

    {
        let mut vfsd = VFSD.borrow_mut();
        if let Err(e) = vfsd.init() {
            verror!("init failed: {:?}", e);
            syscall::exit(1);
        }
    }

    // Run loop - borrow for entire lifetime
    VFSD.borrow_mut().run()
}
