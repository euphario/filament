//! Block Device Client (Ring Buffer Version)
//!
//! Provides a clean abstraction for block device access via usbd.
//! Uses shared ring buffers for high-performance zero-copy transfers.
//!
//! # Example
//! ```
//! let mut block = BlockClient::connect(b"usb")?;
//! let (block_size, block_count) = block.get_info()?;
//!
//! // Read sectors - data is available via block.data()
//! block.read(lba, count)?;
//! let data = block.data();
//! ```

use userlib::ring::{BlockRing, BlockRequest, BlockResponse};
use userlib::syscall;
use crate::transfer::invalidate_buffer;

/// Block device client
///
/// Connects to usbd and provides block-level read/write access.
/// Uses shared ring buffers for zero-copy transfers.
pub struct BlockClient {
    /// Shared ring buffer
    ring: BlockRing,
    /// Block size (cached from get_info)
    block_size: u32,
    /// Total block count (cached from get_info)
    block_count: u64,
    /// Next request tag
    next_tag: u32,
}

// BlockClient is Send because ring buffer access is synchronized
unsafe impl Send for BlockClient {}

impl BlockClient {
    /// Connect to block device server via the specified port name
    /// Returns None if connection or ring setup fails
    pub fn connect(port_name: &[u8]) -> Option<Self> {
        // Connect to server
        let channel = syscall::port_connect(port_name);
        if channel < 0 {
            return None;
        }
        let channel = channel as u32;

        Self::setup_with_channel(channel)
    }

    /// Connect using an existing channel (for when connection is done externally)
    pub fn from_channel(channel: u32) -> Option<Self> {
        Self::setup_with_channel(channel)
    }

    /// Common setup after channel is established
    fn setup_with_channel(channel: u32) -> Option<Self> {
        let mut msg_buf = [0u8; 64];

        // Send our PID to server
        let my_pid = syscall::getpid() as u32;
        msg_buf[..4].copy_from_slice(&my_pid.to_le_bytes());
        syscall::send(channel, &msg_buf[..4]);

        // Wait for shmem_id from server
        for _ in 0..100 {
            let recv_len = syscall::receive(channel, &mut msg_buf);
            if recv_len >= 4 {
                break;
            }
            syscall::yield_now();
        }

        let shmem_id = u32::from_le_bytes([msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3]]);
        if shmem_id == 0 {
            return None;
        }

        // Map the ring buffer - channel is no longer needed after this
        let ring = BlockRing::map(shmem_id)?;

        Some(Self {
            ring,
            block_size: 512,  // Default, updated by get_info()
            block_count: 0,
            next_tag: 1,
        })
    }

    /// Get next request tag
    fn next_tag(&mut self) -> u32 {
        let tag = self.next_tag;
        self.next_tag = self.next_tag.wrapping_add(1);
        if self.next_tag == 0 {
            self.next_tag = 1;
        }
        tag
    }

    /// Wait for a completion with the given tag
    fn wait_completion(&self, expected_tag: u32) -> Option<BlockResponse> {
        for _ in 0..500 {
            // Check for pending completions
            if self.ring.cq_pending() > 0 {
                if let Some(resp) = self.ring.next_completion() {
                    if resp.tag == expected_tag {
                        return Some(resp);
                    }
                    // Wrong tag - could be out-of-order, but for now just continue
                }
            }

            // Wait for notification
            if !self.ring.wait(100) {
                // Timeout - check again
                syscall::yield_now();
            }
        }
        None
    }

    /// Get block device info (block_size, block_count)
    /// Also caches the values internally
    pub fn get_info(&mut self) -> Option<(u32, u64)> {
        let tag = self.next_tag();
        let req = BlockRequest::info(tag);

        if !self.ring.submit(&req) {
            return None;
        }
        self.ring.notify();

        let resp = self.wait_completion(tag)?;
        if resp.status != 0 {
            return None;
        }

        self.block_size = resp.block_size;
        self.block_count = resp.block_count;
        Some((resp.block_size, resp.block_count))
    }

    /// Read blocks from the device
    ///
    /// Data is stored in the ring's data buffer and can be accessed via `data()`.
    /// Returns the number of bytes read, or None on error.
    ///
    /// # Arguments
    /// * `lba` - Logical Block Address to start reading from
    /// * `count` - Number of blocks to read
    /// * `buf_offset` - Offset into the data buffer (default 0)
    pub fn read_at(&mut self, lba: u64, count: u32, buf_offset: u32) -> Option<usize> {
        let max_bytes = self.ring.data_size();
        let requested_bytes = (count as usize) * (self.block_size as usize);
        if buf_offset as usize + requested_bytes > max_bytes {
            return None;
        }

        let tag = self.next_tag();
        let req = BlockRequest::read(tag, lba, count, buf_offset);

        if !self.ring.submit(&req) {
            return None;
        }
        self.ring.notify();

        let resp = self.wait_completion(tag)?;
        if resp.status != 0 {
            return None;
        }

        // Invalidate CPU cache for the data region - DMA wrote to physical memory
        // but CPU may have stale data in cache
        let bytes_read = resp.bytes as usize;
        if bytes_read > 0 {
            let data = self.ring.data();
            let data_addr = data.as_ptr() as u64 + buf_offset as u64;
            invalidate_buffer(data_addr, bytes_read);
        }

        Some(bytes_read)
    }

    /// Read blocks from the device (offset 0)
    pub fn read(&mut self, lba: u64, count: u32) -> Option<usize> {
        self.read_at(lba, count, 0)
    }

    /// Read a single sector
    pub fn read_sector(&mut self, lba: u64) -> Option<usize> {
        self.read(lba, 1)
    }

    /// Write blocks to the device
    ///
    /// Data should be placed in the ring's data buffer before calling.
    /// Use `data_mut()` to access the buffer.
    ///
    /// # Arguments
    /// * `lba` - Logical Block Address to start writing to
    /// * `count` - Number of blocks to write
    /// * `buf_offset` - Offset into the data buffer where data is located
    pub fn write_at(&mut self, lba: u64, count: u32, buf_offset: u32) -> Option<usize> {
        let max_bytes = self.ring.data_size();
        let requested_bytes = (count as usize) * (self.block_size as usize);
        if buf_offset as usize + requested_bytes > max_bytes {
            return None;
        }

        let tag = self.next_tag();
        let req = BlockRequest::write(tag, lba, count, buf_offset);

        if !self.ring.submit(&req) {
            return None;
        }
        self.ring.notify();

        let resp = self.wait_completion(tag)?;
        if resp.status != 0 {
            return None;
        }

        Some(resp.bytes as usize)
    }

    /// Write blocks to the device (offset 0)
    pub fn write(&mut self, lba: u64, count: u32) -> Option<usize> {
        self.write_at(lba, count, 0)
    }

    /// Get a slice of the data buffer after a read operation
    ///
    /// The slice is valid until the next read operation.
    pub fn data(&self) -> &[u8] {
        self.ring.data()
    }

    /// Get a mutable slice of the data buffer (for writing data before write operation)
    pub fn data_mut(&mut self) -> &mut [u8] {
        self.ring.data_mut()
    }

    /// Get a slice of specific length from the data buffer
    pub fn data_slice(&self, len: usize) -> &[u8] {
        let data = self.ring.data();
        let actual_len = core::cmp::min(len, data.len());
        &data[..actual_len]
    }

    /// Copy data from ring buffer to provided slice
    pub fn copy_to(&self, dst: &mut [u8], len: usize) {
        let data = self.ring.data();
        let copy_len = core::cmp::min(len, core::cmp::min(dst.len(), data.len()));
        dst[..copy_len].copy_from_slice(&data[..copy_len]);
    }

    /// Get the block size (typically 512)
    pub fn block_size(&self) -> u32 {
        self.block_size
    }

    /// Get the total block count
    pub fn block_count(&self) -> u64 {
        self.block_count
    }

    /// Get the data buffer capacity in bytes
    pub fn buffer_size(&self) -> usize {
        self.ring.data_size()
    }

    /// Get the maximum number of blocks that can be read at once
    pub fn max_blocks_per_read(&self) -> u32 {
        (self.ring.data_size() / self.block_size as usize) as u32
    }

    /// Get the physical address of the data buffer (for DMA)
    pub fn data_phys(&self) -> u64 {
        self.ring.data_phys()
    }
}
