//! TFTP Server (RFC 1350)
//!
//! Minimal stop-and-wait TFTP over UDP port 69.
//! Serves files from ramfs (initrd binaries) and accepts writes to ramfs.
//! Single transfer at a time.

use smoltcp::wire::{IpAddress, IpEndpoint, Ipv4Address};

use userlib::syscall::RamfsListEntry;

/// TFTP opcodes (RFC 1350).
const OP_RRQ: u16 = 1;
const OP_WRQ: u16 = 2;
const OP_DATA: u16 = 3;
const OP_ACK: u16 = 4;
const OP_ERROR: u16 = 5;

/// TFTP error codes.
const ERR_NOT_FOUND: u16 = 1;
const ERR_ACCESS: u16 = 2;
const ERR_DISK_FULL: u16 = 3;
const ERR_ILLEGAL_OP: u16 = 4;

/// TFTP block size (standard).
const BLOCK_SIZE: usize = 512;

/// Max file size for reads (64 KB — sufficient for initrd binaries listing).
const MAX_READ_SIZE: usize = 64 * 1024;

/// Max file size for writes (64 KB).
const MAX_WRITE_SIZE: usize = 64 * 1024;

/// TFTP transfer state.
#[derive(Clone, Copy, PartialEq, Eq)]
enum TftpState {
    Idle,
    Reading,
    Writing,
}

/// TFTP server handling one transfer at a time.
pub struct TftpServer {
    state: TftpState,
    remote: IpEndpoint,
    block_num: u16,
    /// Read buffer (file content being sent).
    read_buf: [u8; MAX_READ_SIZE],
    read_len: usize,
    read_offset: usize,
    /// Write buffer (file content being received).
    write_buf: [u8; MAX_WRITE_SIZE],
    write_len: usize,
    /// Filename for write operations.
    write_name: [u8; 64],
    write_name_len: usize,
}

impl TftpServer {
    pub const fn new() -> Self {
        Self {
            state: TftpState::Idle,
            remote: IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::UNSPECIFIED), 0),
            block_num: 0,
            read_buf: [0u8; MAX_READ_SIZE],
            read_len: 0,
            read_offset: 0,
            write_buf: [0u8; MAX_WRITE_SIZE],
            write_len: 0,
            write_name: [0u8; 64],
            write_name_len: 0,
        }
    }

    /// Process a received UDP datagram on port 69.
    /// Returns response bytes to send back (if any), written into `out`.
    pub fn handle(&mut self, data: &[u8], remote: IpEndpoint, out: &mut [u8]) -> usize {
        if data.len() < 2 {
            return 0;
        }

        let opcode = u16::from_be_bytes([data[0], data[1]]);

        match opcode {
            OP_RRQ => self.handle_rrq(data, remote, out),
            OP_WRQ => self.handle_wrq(data, remote, out),
            OP_DATA => self.handle_data(data, remote, out),
            OP_ACK => self.handle_ack(data, remote, out),
            _ => Self::build_error(out, ERR_ILLEGAL_OP, b"Unknown opcode"),
        }
    }

    /// Handle Read Request.
    fn handle_rrq(&mut self, data: &[u8], remote: IpEndpoint, out: &mut [u8]) -> usize {
        if self.state != TftpState::Idle {
            return Self::build_error(out, ERR_ACCESS, b"Transfer in progress");
        }

        let filename = match Self::parse_filename(&data[2..]) {
            Some(f) => f,
            None => return Self::build_error(out, ERR_ILLEGAL_OP, b"Bad request"),
        };

        // Try to load file content
        let loaded = self.load_file(filename);
        if !loaded {
            return Self::build_error(out, ERR_NOT_FOUND, b"File not found");
        }

        self.state = TftpState::Reading;
        self.remote = remote;
        self.block_num = 1;
        self.read_offset = 0;

        self.build_data_block(out)
    }

    /// Handle Write Request.
    fn handle_wrq(&mut self, data: &[u8], remote: IpEndpoint, out: &mut [u8]) -> usize {
        if self.state != TftpState::Idle {
            return Self::build_error(out, ERR_ACCESS, b"Transfer in progress");
        }

        let filename = match Self::parse_filename(&data[2..]) {
            Some(f) => f,
            None => return Self::build_error(out, ERR_ILLEGAL_OP, b"Bad request"),
        };

        // Store filename for when write completes
        let name_len = filename.len().min(self.write_name.len());
        self.write_name[..name_len].copy_from_slice(&filename.as_bytes()[..name_len]);
        self.write_name_len = name_len;

        self.state = TftpState::Writing;
        self.remote = remote;
        self.block_num = 0;
        self.write_len = 0;

        // ACK block 0 to accept the write
        Self::build_ack(out, 0)
    }

    /// Handle incoming DATA packet (during write).
    fn handle_data(&mut self, data: &[u8], remote: IpEndpoint, out: &mut [u8]) -> usize {
        if self.state != TftpState::Writing || remote != self.remote {
            return 0;
        }

        if data.len() < 4 {
            return 0;
        }

        let block = u16::from_be_bytes([data[2], data[3]]);
        let payload = &data[4..];

        if block != self.block_num + 1 {
            // Duplicate or out-of-order — re-ACK current block
            return Self::build_ack(out, self.block_num);
        }

        // Append payload to write buffer
        let space = MAX_WRITE_SIZE - self.write_len;
        if payload.len() > space {
            self.state = TftpState::Idle;
            return Self::build_error(out, ERR_DISK_FULL, b"File too large");
        }

        self.write_buf[self.write_len..self.write_len + payload.len()]
            .copy_from_slice(payload);
        self.write_len += payload.len();
        self.block_num = block;

        // Last block if payload < 512 bytes
        if payload.len() < BLOCK_SIZE {
            // Transfer complete — file is in write_buf[..write_len]
            self.state = TftpState::Idle;
        }

        Self::build_ack(out, block)
    }

    /// Handle ACK packet (during read).
    fn handle_ack(&mut self, data: &[u8], remote: IpEndpoint, out: &mut [u8]) -> usize {
        if self.state != TftpState::Reading || remote != self.remote {
            return 0;
        }

        if data.len() < 4 {
            return 0;
        }

        let acked_block = u16::from_be_bytes([data[2], data[3]]);

        if acked_block != self.block_num {
            return 0; // Not the block we're waiting for
        }

        // Advance offset
        self.read_offset += BLOCK_SIZE;

        // Check if transfer is complete
        if self.read_offset >= self.read_len {
            // Also complete if previous block was exactly 512 bytes,
            // but we already sent it — check if there's more
            if self.read_offset > self.read_len
                || (self.read_len > 0 && self.read_len % BLOCK_SIZE != 0)
            {
                self.state = TftpState::Idle;
                return 0;
            }
            // Need to send a zero-length final block
        }

        self.block_num += 1;
        self.build_data_block(out)
    }

    /// Build a DATA block from the read buffer at current offset.
    fn build_data_block(&self, out: &mut [u8]) -> usize {
        if out.len() < 4 {
            return 0;
        }

        out[0..2].copy_from_slice(&OP_DATA.to_be_bytes());
        out[2..4].copy_from_slice(&self.block_num.to_be_bytes());

        let remaining = self.read_len.saturating_sub(self.read_offset);
        let chunk_len = remaining.min(BLOCK_SIZE).min(out.len() - 4);

        if chunk_len > 0 {
            out[4..4 + chunk_len].copy_from_slice(
                &self.read_buf[self.read_offset..self.read_offset + chunk_len],
            );
        }

        4 + chunk_len
    }

    /// Build an ACK packet.
    fn build_ack(out: &mut [u8], block: u16) -> usize {
        if out.len() < 4 {
            return 0;
        }
        out[0..2].copy_from_slice(&OP_ACK.to_be_bytes());
        out[2..4].copy_from_slice(&block.to_be_bytes());
        4
    }

    /// Build an ERROR packet.
    fn build_error(out: &mut [u8], code: u16, msg: &[u8]) -> usize {
        let total = 4 + msg.len() + 1; // opcode + error_code + msg + NUL
        if out.len() < total {
            return 0;
        }
        out[0..2].copy_from_slice(&OP_ERROR.to_be_bytes());
        out[2..4].copy_from_slice(&code.to_be_bytes());
        out[4..4 + msg.len()].copy_from_slice(msg);
        out[4 + msg.len()] = 0; // NUL terminator
        total
    }

    /// Parse a NUL-terminated filename from a request packet.
    /// Returns the filename as a str (up to the first NUL).
    fn parse_filename(data: &[u8]) -> Option<&str> {
        let nul_pos = data.iter().position(|&b| b == 0)?;
        if nul_pos == 0 {
            return None;
        }
        core::str::from_utf8(&data[..nul_pos]).ok()
    }

    /// Load a file into the read buffer.
    /// Supports virtual files:
    ///   "ls" or "/" — list ramfs entries
    ///   Any other name — search ramfs for matching entry
    fn load_file(&mut self, name: &str) -> bool {
        // Strip leading path separators
        let name = name.trim_start_matches('/');

        if name == "ls" || name.is_empty() {
            return self.load_ramfs_listing();
        }

        // No raw file read from ramfs available yet — return listing for any request
        // Future: VfsClient integration for real file reads
        false
    }

    /// Generate a listing of ramfs entries (like 'ls /bin').
    fn load_ramfs_listing(&mut self) -> bool {
        let mut entries = [RamfsListEntry::empty(); 32];
        let count = userlib::syscall::ramfs_list(&mut entries);
        if count < 0 {
            return false;
        }

        let mut pos = 0;
        let header = b"# ramfs contents\n";
        if pos + header.len() <= MAX_READ_SIZE {
            self.read_buf[pos..pos + header.len()].copy_from_slice(header);
            pos += header.len();
        }

        for i in 0..count as usize {
            let entry = &entries[i];
            let name_bytes = entry.name_str();
            let size = entry.size as u32;

            // Format: "name  size\n"
            let mut size_buf = [0u8; 16];
            let size_len = format_u32(size, &mut size_buf);

            let line_len = name_bytes.len() + 2 + size_len + 1; // name + "  " + size + "\n"
            if pos + line_len > MAX_READ_SIZE {
                break;
            }

            self.read_buf[pos..pos + name_bytes.len()].copy_from_slice(name_bytes);
            pos += name_bytes.len();
            self.read_buf[pos..pos + 2].copy_from_slice(b"  ");
            pos += 2;
            self.read_buf[pos..pos + size_len].copy_from_slice(&size_buf[..size_len]);
            pos += size_len;
            self.read_buf[pos] = b'\n';
            pos += 1;
        }

        self.read_len = pos;
        true
    }
}

/// Format a u32 as decimal into a buffer. Returns number of bytes written.
fn format_u32(mut val: u32, buf: &mut [u8]) -> usize {
    if val == 0 {
        if !buf.is_empty() {
            buf[0] = b'0';
        }
        return 1;
    }

    let mut tmp = [0u8; 10];
    let mut i = 0;
    while val > 0 {
        tmp[i] = b'0' + (val % 10) as u8;
        val /= 10;
        i += 1;
    }

    let len = i.min(buf.len());
    for j in 0..len {
        buf[j] = tmp[i - 1 - j];
    }
    len
}
