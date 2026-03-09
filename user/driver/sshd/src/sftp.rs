//! SFTP v3 Protocol Handler
//!
//! Implements the SSH File Transfer Protocol (draft-ietf-secsh-filexfer-02)
//! over an SSH channel. Bridges SFTP operations to VfsClient.
//!
//! Packet format: type(u8) + request_id(u32) + payload
//! (The 4-byte length prefix is handled by the SSH channel layer in main.rs.)

extern crate alloc;
use alloc::vec::Vec;

use libos::vfs_client::{VfsClient, VfsError};
use libos::vfs_proto::{open_flags, file_type, VfsDirEntry, VfsStat};
use libos::udebug;

// =============================================================================
// SFTP Message Types (draft-ietf-secsh-filexfer-02 § 3)
// =============================================================================

mod fxp {
    pub const INIT: u8 = 1;
    pub const VERSION: u8 = 2;
    pub const OPEN: u8 = 3;
    pub const CLOSE: u8 = 4;
    pub const READ: u8 = 5;
    pub const WRITE: u8 = 6;
    pub const LSTAT: u8 = 7;
    pub const FSTAT: u8 = 8;
    pub const SETSTAT: u8 = 9;
    pub const OPENDIR: u8 = 11;
    pub const READDIR: u8 = 12;
    pub const REMOVE: u8 = 13;
    pub const MKDIR: u8 = 14;
    pub const RMDIR: u8 = 15;
    pub const REALPATH: u8 = 16;
    pub const STAT: u8 = 17;
    pub const STATUS: u8 = 101;
    pub const HANDLE: u8 = 102;
    pub const DATA: u8 = 103;
    pub const NAME: u8 = 104;
    pub const ATTRS: u8 = 105;
}

// SFTP status codes
mod status {
    pub const OK: u32 = 0;
    pub const EOF: u32 = 1;
    pub const NO_SUCH_FILE: u32 = 2;
    pub const PERMISSION_DENIED: u32 = 3;
    pub const FAILURE: u32 = 4;
    pub const OP_UNSUPPORTED: u32 = 8;
}

// SFTP pflags (open flags)
mod pflags {
    pub const READ: u32 = 0x01;
    pub const WRITE: u32 = 0x02;
    pub const CREAT: u32 = 0x08;
    pub const TRUNC: u32 = 0x10;
}

// SFTP file attribute flags
mod attr_flags {
    pub const SIZE: u32 = 0x01;
    pub const PERMISSIONS: u32 = 0x04;
}

const MAX_HANDLES: usize = 32;

// =============================================================================
// Handle table
// =============================================================================

#[derive(Clone, Copy)]
struct SftpHandle {
    in_use: bool,
    vfs_handle: u32,
    is_dir: bool,
    /// Track whether readdir has been called (for EOF on second call).
    readdir_done: bool,
}

impl SftpHandle {
    const fn empty() -> Self {
        Self { in_use: false, vfs_handle: 0, is_dir: false, readdir_done: false }
    }
}

// =============================================================================
// SFTP Handler
// =============================================================================

pub struct SftpHandler {
    vfs: VfsClient,
    handles: [SftpHandle; MAX_HANDLES],
    initialized: bool,
}

impl SftpHandler {
    pub fn new() -> Option<Self> {
        let vfs = VfsClient::discover().ok()?;
        Some(Self {
            vfs,
            handles: [SftpHandle::empty(); MAX_HANDLES],
            initialized: false,
        })
    }

    /// Process one SFTP packet (without length prefix) and write the response.
    pub fn process(&mut self, data: &[u8], out: &mut Vec<u8>) {
        if data.is_empty() {
            return;
        }

        let msg_type = data[0];
        let body = &data[1..];

        match msg_type {
            fxp::INIT => self.handle_init(body, out),
            fxp::OPEN => self.handle_open(body, out),
            fxp::CLOSE => self.handle_close(body, out),
            fxp::READ => self.handle_read(body, out),
            fxp::WRITE => self.handle_write(body, out),
            fxp::STAT | fxp::LSTAT => self.handle_stat(body, out),
            fxp::FSTAT => self.handle_fstat(body, out),
            fxp::OPENDIR => self.handle_opendir(body, out),
            fxp::READDIR => self.handle_readdir(body, out),
            fxp::REMOVE => self.handle_remove(body, out),
            fxp::MKDIR => self.handle_mkdir(body, out),
            fxp::REALPATH => self.handle_realpath(body, out),
            fxp::SETSTAT | fxp::RMDIR => {
                // Parse request_id and return unsupported
                if let Some((id, _)) = read_u32(body) {
                    write_status(out, id, status::OP_UNSUPPORTED, b"not supported");
                }
            }
            _ => {
                if let Some((id, _)) = read_u32(body) {
                    write_status(out, id, status::OP_UNSUPPORTED, b"unknown request");
                }
            }
        }
    }

    // =========================================================================
    // Handlers
    // =========================================================================

    fn handle_init(&mut self, body: &[u8], out: &mut Vec<u8>) {
        // INIT: version(u32)
        let _version = if body.len() >= 4 {
            u32::from_be_bytes([body[0], body[1], body[2], body[3]])
        } else {
            3
        };

        // Reply: VERSION with version 3
        out.push(fxp::VERSION);
        push_u32(out, 3);
        self.initialized = true;

        udebug!("sshd", "sftp_init"; client_ver = _version);
    }

    fn handle_open(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (filename, rest) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };
        let (pflags_val, _rest) = match read_u32(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };
        // attrs follow but we ignore them

        // Convert SFTP pflags to VFS open_flags
        let mut flags = 0u32;
        if pflags_val & pflags::WRITE != 0 {
            flags |= open_flags::RDWR;
        }
        if pflags_val & pflags::CREAT != 0 {
            flags |= open_flags::CREATE;
        }
        if pflags_val & pflags::TRUNC != 0 {
            flags |= open_flags::TRUNC;
        }

        match self.vfs.open(filename, flags) {
            Ok(handle) => {
                let slot = match self.alloc_slot() {
                    Some(s) => s,
                    None => {
                        let _ = self.vfs.close(handle);
                        write_status(out, id, status::FAILURE, b"too many handles");
                        return;
                    }
                };
                self.handles[slot].vfs_handle = handle;
                self.handles[slot].is_dir = false;
                write_handle(out, id, slot as u32);
            }
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_close(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (handle_bytes, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad handle"); return; }
        };

        let slot = match parse_handle(handle_bytes) {
            Some(s) if s < MAX_HANDLES && self.handles[s].in_use => s,
            _ => { write_status(out, id, status::FAILURE, b"invalid handle"); return; }
        };

        let _ = self.vfs.close(self.handles[slot].vfs_handle);
        self.handles[slot] = SftpHandle::empty();
        write_status(out, id, status::OK, b"");
    }

    fn handle_read(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (handle_bytes, rest) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };
        let (offset, rest) = match read_u64(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };
        let (len, _) = match read_u32(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };

        let slot = match parse_handle(handle_bytes) {
            Some(s) if s < MAX_HANDLES && self.handles[s].in_use => s,
            _ => { write_status(out, id, status::FAILURE, b"invalid handle"); return; }
        };

        let read_len = (len as usize).min(32768); // Cap at 32K
        let mut buf = alloc::vec![0u8; read_len];

        match self.vfs.read(self.handles[slot].vfs_handle, offset, &mut buf) {
            Ok(0) => write_status(out, id, status::EOF, b""),
            Ok(n) => {
                out.push(fxp::DATA);
                push_u32(out, id);
                push_string(out, &buf[..n]);
            }
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_write(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (handle_bytes, rest) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };
        let (offset, rest) = match read_u64(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };
        let (data, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };

        let slot = match parse_handle(handle_bytes) {
            Some(s) if s < MAX_HANDLES && self.handles[s].in_use => s,
            _ => { write_status(out, id, status::FAILURE, b"invalid handle"); return; }
        };

        match self.vfs.write(self.handles[slot].vfs_handle, offset, data) {
            Ok(_n) => write_status(out, id, status::OK, b""),
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_stat(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (path, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };

        // Open, stat, close
        match self.vfs.open(path, 0) {
            Ok(handle) => {
                match self.vfs.stat(handle) {
                    Ok(st) => {
                        let _ = self.vfs.close(handle);
                        write_attrs(out, id, &st);
                    }
                    Err(e) => {
                        let _ = self.vfs.close(handle);
                        write_vfs_error(out, id, e);
                    }
                }
            }
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_fstat(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (handle_bytes, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };

        let slot = match parse_handle(handle_bytes) {
            Some(s) if s < MAX_HANDLES && self.handles[s].in_use => s,
            _ => { write_status(out, id, status::FAILURE, b"invalid handle"); return; }
        };

        match self.vfs.stat(self.handles[slot].vfs_handle) {
            Ok(st) => write_attrs(out, id, &st),
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_opendir(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (path, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };

        match self.vfs.open(path, open_flags::DIR) {
            Ok(handle) => {
                let slot = match self.alloc_slot() {
                    Some(s) => s,
                    None => {
                        let _ = self.vfs.close(handle);
                        write_status(out, id, status::FAILURE, b"too many handles");
                        return;
                    }
                };
                self.handles[slot].vfs_handle = handle;
                self.handles[slot].is_dir = true;
                self.handles[slot].readdir_done = false;
                write_handle(out, id, slot as u32);
            }
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_readdir(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (handle_bytes, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };

        let slot = match parse_handle(handle_bytes) {
            Some(s) if s < MAX_HANDLES && self.handles[s].in_use && self.handles[s].is_dir => s,
            _ => { write_status(out, id, status::FAILURE, b"invalid handle"); return; }
        };

        // SFTP readdir is called repeatedly; return EOF on second call
        if self.handles[slot].readdir_done {
            write_status(out, id, status::EOF, b"");
            return;
        }

        let mut entries = [VfsDirEntry::empty(); 64];
        match self.vfs.readdir(self.handles[slot].vfs_handle, &mut entries) {
            Ok(0) => {
                self.handles[slot].readdir_done = true;
                write_status(out, id, status::EOF, b"");
            }
            Ok(count) => {
                self.handles[slot].readdir_done = true;

                // Build NAME response
                out.push(fxp::NAME);
                push_u32(out, id);
                push_u32(out, count as u32);

                for i in 0..count {
                    let name = entries[i].name_bytes();
                    // filename
                    push_string(out, name);
                    // longname (ls -l format, simplified)
                    let is_dir = entries[i].file_type == file_type::DIR;
                    let mut longname = Vec::with_capacity(name.len() + 32);
                    if is_dir {
                        longname.extend_from_slice(b"drwxr-xr-x   1 root root ");
                    } else {
                        longname.extend_from_slice(b"-rw-r--r--   1 root root ");
                    }
                    // File size
                    push_decimal_ascii(&mut longname, entries[i].size as u64);
                    longname.push(b' ');
                    longname.extend_from_slice(b"Jan  1  2026 ");
                    longname.extend_from_slice(name);
                    push_string(out, &longname);

                    // attrs
                    let attr_flags = attr_flags::SIZE | attr_flags::PERMISSIONS;
                    push_u32(out, attr_flags);
                    push_u64(out, entries[i].size as u64);
                    let perms: u32 = if is_dir { 0o40755 } else { 0o100644 };
                    push_u32(out, perms);
                }
            }
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_remove(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (path, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };

        match self.vfs.unlink(path) {
            Ok(()) => write_status(out, id, status::OK, b""),
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_mkdir(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (path, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };
        // attrs follow but we ignore them

        match self.vfs.mkdir(path) {
            Ok(()) => write_status(out, id, status::OK, b""),
            Err(e) => write_vfs_error(out, id, e),
        }
    }

    fn handle_realpath(&mut self, body: &[u8], out: &mut Vec<u8>) {
        let (id, rest) = match read_u32(body) {
            Some(r) => r,
            None => return,
        };
        let (path, _) = match read_string(rest) {
            Some(r) => r,
            None => { write_status(out, id, status::FAILURE, b"bad request"); return; }
        };

        // Simple canonicalization: normalize "." to "/" and return path as-is
        let canonical = if path == b"." || path.is_empty() {
            b"/" as &[u8]
        } else {
            path
        };

        // NAME response with count=1
        out.push(fxp::NAME);
        push_u32(out, id);
        push_u32(out, 1); // count
        push_string(out, canonical); // filename
        push_string(out, canonical); // longname
        // Empty attrs
        push_u32(out, 0); // flags = 0 (no attributes)
    }

    // =========================================================================
    // Handle management
    // =========================================================================

    fn alloc_slot(&mut self) -> Option<usize> {
        for i in 0..MAX_HANDLES {
            if !self.handles[i].in_use {
                self.handles[i].in_use = true;
                return Some(i);
            }
        }
        None
    }
}

// =============================================================================
// Wire format helpers
// =============================================================================

fn read_u32(data: &[u8]) -> Option<(u32, &[u8])> {
    if data.len() < 4 { return None; }
    let val = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
    Some((val, &data[4..]))
}

fn read_u64(data: &[u8]) -> Option<(u64, &[u8])> {
    if data.len() < 8 { return None; }
    let val = u64::from_be_bytes([
        data[0], data[1], data[2], data[3],
        data[4], data[5], data[6], data[7],
    ]);
    Some((val, &data[8..]))
}

fn read_string(data: &[u8]) -> Option<(&[u8], &[u8])> {
    if data.len() < 4 { return None; }
    let len = u32::from_be_bytes([data[0], data[1], data[2], data[3]]) as usize;
    if data.len() < 4 + len { return None; }
    Some((&data[4..4 + len], &data[4 + len..]))
}

fn push_u32(out: &mut Vec<u8>, val: u32) {
    out.extend_from_slice(&val.to_be_bytes());
}

fn push_u64(out: &mut Vec<u8>, val: u64) {
    out.extend_from_slice(&val.to_be_bytes());
}

fn push_string(out: &mut Vec<u8>, data: &[u8]) {
    push_u32(out, data.len() as u32);
    out.extend_from_slice(data);
}

/// Write an SSH_FXP_STATUS response.
fn write_status(out: &mut Vec<u8>, id: u32, code: u32, msg: &[u8]) {
    out.push(fxp::STATUS);
    push_u32(out, id);
    push_u32(out, code);
    push_string(out, msg); // error message
    push_string(out, b""); // language tag
}

/// Write an SSH_FXP_HANDLE response.
fn write_handle(out: &mut Vec<u8>, id: u32, slot: u32) {
    out.push(fxp::HANDLE);
    push_u32(out, id);
    // Handle is a 4-byte opaque string containing the slot number
    push_string(out, &slot.to_be_bytes());
}

/// Write an SSH_FXP_ATTRS response.
fn write_attrs(out: &mut Vec<u8>, id: u32, st: &VfsStat) {
    out.push(fxp::ATTRS);
    push_u32(out, id);
    let flags = attr_flags::SIZE | attr_flags::PERMISSIONS;
    push_u32(out, flags);
    push_u64(out, st.size);
    let perms: u32 = if st.file_type == file_type::DIR { 0o40755 } else { 0o100644 };
    push_u32(out, perms);
}

/// Convert VfsError to SFTP status.
fn write_vfs_error(out: &mut Vec<u8>, id: u32, e: VfsError) {
    let (code, msg): (u32, &[u8]) = match e {
        VfsError::NotFound => (status::NO_SUCH_FILE, b"no such file"),
        VfsError::Permission => (status::PERMISSION_DENIED, b"permission denied"),
        VfsError::NoSpace => (status::FAILURE, b"no space left"),
        VfsError::NotDir => (status::FAILURE, b"not a directory"),
        VfsError::IsDir => (status::FAILURE, b"is a directory"),
        VfsError::Exists => (status::FAILURE, b"file exists"),
        VfsError::ReadOnly => (status::FAILURE, b"read-only filesystem"),
        VfsError::TooMany => (status::FAILURE, b"too many open files"),
        VfsError::Timeout => (status::FAILURE, b"timeout"),
        _ => (status::FAILURE, b"I/O error"),
    };
    write_status(out, id, code, msg);
}

/// Parse an SFTP handle (4-byte BE slot number).
fn parse_handle(data: &[u8]) -> Option<usize> {
    if data.len() != 4 { return None; }
    Some(u32::from_be_bytes([data[0], data[1], data[2], data[3]]) as usize)
}

/// Push a decimal ASCII representation of a u64.
fn push_decimal_ascii(out: &mut Vec<u8>, val: u64) {
    if val == 0 {
        out.push(b'0');
        return;
    }
    let mut tmp = [0u8; 20];
    let mut n = val;
    let mut len = 0;
    while n > 0 {
        tmp[len] = b'0' + (n % 10) as u8;
        n /= 10;
        len += 1;
    }
    for i in (0..len).rev() {
        out.push(tmp[i]);
    }
}
