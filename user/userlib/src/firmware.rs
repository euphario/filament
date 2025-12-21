//! Firmware Loading Protocol
//!
//! IPC protocol between firmware consumers (e.g., wifid) and the
//! filesystem driver (fatfs) for loading firmware from USB storage.
//!
//! ## Protocol
//!
//! 1. Consumer creates shared memory buffer
//! 2. Consumer connects to "fatfs" port
//! 3. Consumer sends FirmwareRequest with filename and shmem_id
//! 4. fatfs reads file into shared memory
//! 5. fatfs sends FirmwareReply with size or error

use crate::syscall;

/// Maximum filename length
pub const MAX_FILENAME: usize = 128;

/// Port name for fatfs service
pub const FATFS_PORT: &[u8] = b"fatfs";

/// Firmware request message
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FirmwareRequest {
    /// Filename to load (null-terminated)
    pub filename: [u8; MAX_FILENAME],
    /// Shared memory ID where file should be written
    pub shmem_id: u32,
    /// Maximum size to read
    pub max_size: u32,
    /// PID of requester (for shmem_allow)
    pub requester_pid: u32,
}

impl FirmwareRequest {
    pub fn new(filename: &[u8], shmem_id: u32, max_size: u32) -> Self {
        let mut req = Self {
            filename: [0u8; MAX_FILENAME],
            shmem_id,
            max_size,
            requester_pid: syscall::getpid(),
        };
        let len = core::cmp::min(filename.len(), MAX_FILENAME - 1);
        req.filename[..len].copy_from_slice(&filename[..len]);
        req
    }

    pub fn filename_str(&self) -> &[u8] {
        let end = self.filename.iter().position(|&b| b == 0).unwrap_or(MAX_FILENAME);
        &self.filename[..end]
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<Self>(),
            )
        }
    }

    pub fn from_bytes(data: &[u8]) -> Option<&Self> {
        if data.len() >= core::mem::size_of::<Self>() {
            Some(unsafe { &*(data.as_ptr() as *const Self) })
        } else {
            None
        }
    }
}

/// Firmware reply message
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FirmwareReply {
    /// Size of file read, or negative error code
    /// Errors: -1 = not found, -2 = read error, -3 = too large
    pub result: i64,
}

impl FirmwareReply {
    pub fn success(size: usize) -> Self {
        Self { result: size as i64 }
    }

    pub fn error(code: i64) -> Self {
        Self { result: code }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<Self>(),
            )
        }
    }

    pub fn from_bytes(data: &[u8]) -> Option<&Self> {
        if data.len() >= core::mem::size_of::<Self>() {
            Some(unsafe { &*(data.as_ptr() as *const Self) })
        } else {
            None
        }
    }

    pub fn is_success(&self) -> bool {
        self.result >= 0
    }

    pub fn size(&self) -> Option<usize> {
        if self.result >= 0 {
            Some(self.result as usize)
        } else {
            None
        }
    }
}

/// Error codes
pub mod error {
    pub const NOT_FOUND: i64 = -1;
    pub const READ_ERROR: i64 = -2;
    pub const TOO_LARGE: i64 = -3;
    pub const NO_MEMORY: i64 = -4;
    pub const CONNECT_FAILED: i64 = -5;
    pub const TIMEOUT: i64 = -6;
}

/// Client for loading firmware from fatfs
pub struct FirmwareClient {
    channel: u32,
    /// fatfs server's PID (for shmem_allow)
    server_pid: u32,
}

impl FirmwareClient {
    /// Connect to fatfs service
    pub fn connect() -> Option<Self> {
        let result = syscall::port_connect(FATFS_PORT);
        if result < 0 {
            return None;
        }
        let channel = result as u32;

        // Get server's PID by sending a ping request
        // (Server PID is encoded in high bits of accept result, or we query it)
        // For now, assume server PID is returned in first handshake
        let mut pid_buf = [0u8; 4];

        // Send empty request to trigger handshake
        let ping = [0u8; 4]; // PID query
        if syscall::send(channel, &ping) < 0 {
            syscall::channel_close(channel);
            return None;
        }

        // Receive server PID
        if syscall::receive(channel, &mut pid_buf) < 0 {
            syscall::channel_close(channel);
            return None;
        }

        let server_pid = u32::from_le_bytes(pid_buf);

        Some(Self { channel, server_pid })
    }

    /// Load firmware into provided shared memory
    /// Caller must have created shmem already
    /// Returns size on success, negative error on failure
    pub fn load(&self, filename: &[u8], shmem_id: u32, max_size: u32) -> i64 {
        // Allow server to access our shmem
        syscall::shmem_allow(shmem_id, self.server_pid);

        let request = FirmwareRequest::new(filename, shmem_id, max_size);

        // Send request
        if syscall::send(self.channel, request.as_bytes()) < 0 {
            return error::CONNECT_FAILED;
        }

        // Wait for reply
        let mut reply_buf = [0u8; core::mem::size_of::<FirmwareReply>()];
        let n = syscall::receive(self.channel, &mut reply_buf);
        if n < 0 {
            return error::READ_ERROR;
        }

        match FirmwareReply::from_bytes(&reply_buf) {
            Some(reply) => reply.result,
            None => error::READ_ERROR,
        }
    }

    /// Load firmware, allocating shared memory automatically
    /// Returns (vaddr, paddr, size, shmem_id) on success
    pub fn load_alloc(&self, filename: &[u8], max_size: usize) -> Result<(u64, u64, usize, u32), i64> {
        // Allocate shared memory
        let mut vaddr: u64 = 0;
        let mut paddr: u64 = 0;
        let shmem_id = syscall::shmem_create(max_size, &mut vaddr, &mut paddr);
        if shmem_id < 0 {
            return Err(error::NO_MEMORY);
        }
        let shmem_id = shmem_id as u32;

        // Load the firmware
        let result = self.load(filename, shmem_id, max_size as u32);
        if result < 0 {
            syscall::shmem_destroy(shmem_id);
            return Err(result);
        }

        Ok((vaddr, paddr, result as usize, shmem_id))
    }

    /// Get the server's PID (useful for debugging)
    pub fn server_pid(&self) -> u32 {
        self.server_pid
    }
}

impl Drop for FirmwareClient {
    fn drop(&mut self) {
        syscall::channel_close(self.channel);
    }
}
