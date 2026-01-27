//! Block Device Client
//!
//! Wrapper for block device IPC that provides a simple API for reading/writing blocks.
//! Used by fatfs to read from disk.

use crate::ipc::Channel;
use crate::blk::{
    BlkHeader, BlkInfo, BlkInfoResp, BlkRead, BlkData, BlkWrite, BlkResult,
    BlkError, msg, error,
};
use crate::error::SysError;

/// Block device client - connects to a block device port
pub struct BlockClient {
    channel: Channel,
    seq_id: u32,
    /// Cached block size (set after info() call)
    block_size: u32,
    /// Cached block count (set after info() call)
    block_count: u64,
}

impl BlockClient {
    /// Connect to a block device by port name (e.g., "disk0:")
    pub fn connect(port_name: &[u8]) -> Result<Self, BlkError> {
        let channel = Channel::connect(port_name).map_err(|_| BlkError::NotReady)?;
        Ok(Self {
            channel,
            seq_id: 1,
            block_size: 0,
            block_count: 0,
        })
    }

    fn next_seq(&mut self) -> u32 {
        let seq = self.seq_id;
        self.seq_id = self.seq_id.wrapping_add(1);
        seq
    }

    /// Get block device info (block_size, block_count)
    /// Also caches the values for block_size() and block_count() methods.
    pub fn info(&mut self) -> Result<(u32, u64), BlkError> {
        let seq = self.next_seq();
        let req = BlkInfo::new(seq);

        // Send request
        if self.channel.send(&req.to_bytes()).is_err() {
            return Err(BlkError::Io);
        }

        // Wait for response
        let _ = crate::ipc::wait_one(self.channel.handle());

        // Receive response
        let mut buf = [0u8; BlkInfoResp::SIZE];
        match self.channel.recv(&mut buf) {
            Ok(n) if n >= BlkInfoResp::SIZE => {
                if let Some(resp) = BlkInfoResp::from_bytes(&buf) {
                    if resp.header.msg_type == msg::BLK_INFO_RESP {
                        // Cache values
                        self.block_size = resp.block_size;
                        self.block_count = resp.block_count;
                        return Ok((resp.block_size, resp.block_count));
                    }
                }
                Err(BlkError::InvalidMessage)
            }
            _ => Err(BlkError::Io),
        }
    }

    /// Get cached block size (call info() first)
    pub fn block_size(&self) -> u32 {
        self.block_size
    }

    /// Get cached block count (call info() first)
    pub fn block_count(&self) -> u64 {
        self.block_count
    }

    /// Read blocks at LBA into buffer
    ///
    /// Buffer must be large enough for `count * block_size` bytes.
    pub fn read_blocks(&mut self, lba: u64, count: u32, buf: &mut [u8]) -> Result<usize, BlkError> {
        let seq = self.next_seq();
        let req = BlkRead::new(seq, lba, count);

        // Send request
        if self.channel.send(&req.to_bytes()).is_err() {
            return Err(BlkError::Io);
        }

        // Wait for response
        let _ = crate::ipc::wait_one(self.channel.handle());

        // Receive response into internal buffer (header + data)
        // Response format: 16-byte BlkData header + data
        // We need a buffer large enough for both
        let mut resp_buf = [0u8; 4096 + BlkData::HEADER_SIZE];
        match self.channel.recv(&mut resp_buf) {
            Ok(n) if n >= BlkData::HEADER_SIZE => {
                if let Some((resp, data)) = BlkData::from_bytes(&resp_buf[..n]) {
                    if resp.header.msg_type == msg::BLK_DATA {
                        if resp.error != error::OK {
                            return Err(BlkError::from_i32(resp.error));
                        }
                        // Copy data to caller's buffer
                        let data_len = data.len().min(buf.len());
                        buf[..data_len].copy_from_slice(&data[..data_len]);
                        return Ok(data_len);
                    }
                }
                Err(BlkError::InvalidMessage)
            }
            _ => Err(BlkError::Io),
        }
    }

    /// Read a single block at LBA into a 512-byte buffer
    pub fn read_block(&mut self, lba: u64, buf: &mut [u8; 512]) -> Result<(), BlkError> {
        self.read_blocks(lba, 1, buf)?;
        Ok(())
    }

    /// Write blocks at LBA from buffer
    pub fn write_blocks(&mut self, lba: u64, count: u32, data: &[u8]) -> Result<(), BlkError> {
        let seq = self.next_seq();
        let req = BlkWrite::new(seq, lba, count);

        // Build message
        let mut msg_buf = [0u8; 4096];
        let len = match req.write_to(&mut msg_buf, data) {
            Some(l) => l,
            None => return Err(BlkError::InvalidMessage),
        };

        // Send request
        if self.channel.send(&msg_buf[..len]).is_err() {
            return Err(BlkError::Io);
        }

        // Wait for response
        let _ = crate::ipc::wait_one(self.channel.handle());

        // Receive result
        let mut buf = [0u8; BlkResult::SIZE];
        match self.channel.recv(&mut buf) {
            Ok(n) if n >= BlkResult::SIZE => {
                if let Some(resp) = BlkResult::from_bytes(&buf) {
                    if resp.header.msg_type == msg::BLK_RESULT {
                        if resp.error != error::OK {
                            return Err(BlkError::from_i32(resp.error));
                        }
                        return Ok(());
                    }
                }
                Err(BlkError::InvalidMessage)
            }
            _ => Err(BlkError::Io),
        }
    }
}
