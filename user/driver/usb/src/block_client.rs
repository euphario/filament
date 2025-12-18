//! Block Device Client
//!
//! Provides a clean abstraction for block device access via usbd.
//! Handles IPC communication and DMA buffer management internally.
//!
//! # Example
//! ```
//! let mut block = BlockClient::connect("usb")?;
//! let (block_size, block_count) = block.get_info()?;
//!
//! // Read sectors - data is available via block.data()
//! block.read(lba, count)?;
//! let data = block.data();
//! ```

use crate::{
    UsbRequest, UsbStatus, UsbMessageHeader,
    BlockInfoResponse,
    BlockReadDmaRequest, BlockReadDmaResponse,
    delay_ms,
};

/// Default DMA buffer size (32KB = 64 sectors)
pub const DMA_BUFFER_SIZE: usize = 32768;

/// Block device client
///
/// Connects to usbd and provides block-level read/write access.
/// Uses zero-copy DMA internally for efficient transfers.
pub struct BlockClient {
    channel: u32,
    msg_buf: [u8; 128],
    dma_virt: *mut u8,
    dma_phys: u64,
    dma_size: usize,
    block_size: u32,
    block_count: u64,
}

// BlockClient needs to be Send for potential future async use
// The raw pointers are to DMA memory we own exclusively
unsafe impl Send for BlockClient {}

impl BlockClient {
    /// Connect to usbd via the specified port name
    /// Returns None if connection or DMA allocation fails
    pub fn connect(port_name: &[u8]) -> Option<Self> {
        // Connect to usbd
        let channel = userlib::syscall::port_connect(port_name);
        if channel < 0 {
            return None;
        }
        let channel = channel as u32;

        // Allocate DMA buffer
        let mut dma_phys: u64 = 0;
        let dma_virt = userlib::syscall::mmap_dma(DMA_BUFFER_SIZE, &mut dma_phys);
        if dma_virt < 0 {
            return None;
        }

        Some(Self {
            channel,
            msg_buf: [0u8; 128],
            dma_virt: dma_virt as *mut u8,
            dma_phys,
            dma_size: DMA_BUFFER_SIZE,
            block_size: 512,  // Default, updated by get_info()
            block_count: 0,
        })
    }

    /// Connect using an existing channel (for when connection is done externally)
    pub fn from_channel(channel: u32) -> Option<Self> {
        // Allocate DMA buffer
        let mut dma_phys: u64 = 0;
        let dma_virt = userlib::syscall::mmap_dma(DMA_BUFFER_SIZE, &mut dma_phys);
        if dma_virt < 0 {
            return None;
        }

        Some(Self {
            channel,
            msg_buf: [0u8; 128],
            dma_virt: dma_virt as *mut u8,
            dma_phys,
            dma_size: DMA_BUFFER_SIZE,
            block_size: 512,
            block_count: 0,
        })
    }

    /// Get block device info (block_size, block_count)
    /// Also caches the values internally
    pub fn get_info(&mut self) -> Option<(u32, u64)> {
        // Build request
        let header = UsbMessageHeader::new(UsbRequest::BlockInfo, 0);
        let header_bytes = header.to_bytes();
        self.msg_buf[..UsbMessageHeader::SIZE].copy_from_slice(&header_bytes);

        // Send request
        if userlib::syscall::send(self.channel, &self.msg_buf[..UsbMessageHeader::SIZE]) < 0 {
            return None;
        }

        // Receive response
        let expected_type = UsbRequest::BlockInfo as u8;
        for _ in 0..100 {
            let recv_result = userlib::syscall::receive(self.channel, &mut self.msg_buf);
            if recv_result < 1 {
                delay_ms(10);
                userlib::syscall::yield_now();
                continue;
            }

            let resp_type = self.msg_buf[0];
            if resp_type == expected_type && recv_result >= BlockInfoResponse::SIZE as isize {
                let resp: BlockInfoResponse = unsafe {
                    let mut arr = [0u8; BlockInfoResponse::SIZE];
                    arr.copy_from_slice(&self.msg_buf[..BlockInfoResponse::SIZE]);
                    core::mem::transmute(arr)
                };

                if resp.status != UsbStatus::Ok {
                    return None;
                }

                self.block_size = resp.block_size;
                self.block_count = resp.block_count;
                return Some((resp.block_size, resp.block_count));
            }
        }

        None
    }

    /// Read blocks from the device
    ///
    /// Data is stored in the internal DMA buffer and can be accessed via `data()`.
    /// Returns the number of bytes read, or None on error.
    ///
    /// # Arguments
    /// * `lba` - Logical Block Address to start reading from
    /// * `count` - Number of blocks to read (max 64 for 32KB buffer)
    pub fn read(&mut self, lba: u64, count: u32) -> Option<usize> {
        let max_blocks = self.dma_size / self.block_size as usize;
        if count == 0 || count as usize > max_blocks {
            return None;
        }

        // Build request
        let header = UsbMessageHeader::new(UsbRequest::BlockReadDma, BlockReadDmaRequest::SIZE as u32);
        let header_bytes = header.to_bytes();
        self.msg_buf[..UsbMessageHeader::SIZE].copy_from_slice(&header_bytes);

        let req = BlockReadDmaRequest::new(0, lba, count, self.dma_phys);
        let req_bytes = req.to_bytes();
        self.msg_buf[UsbMessageHeader::SIZE..UsbMessageHeader::SIZE + BlockReadDmaRequest::SIZE]
            .copy_from_slice(&req_bytes);

        // Send request
        let msg_len = UsbMessageHeader::SIZE + BlockReadDmaRequest::SIZE;
        if userlib::syscall::send(self.channel, &self.msg_buf[..msg_len]) < 0 {
            return None;
        }

        // Wait for response
        let expected_type = UsbRequest::BlockReadDma as u8;
        for _ in 0..100 {
            let recv_result = userlib::syscall::receive(self.channel, &mut self.msg_buf);
            if recv_result < 1 {
                delay_ms(10);
                userlib::syscall::yield_now();
                continue;
            }

            let resp_type = self.msg_buf[0];
            if resp_type == expected_type && recv_result >= BlockReadDmaResponse::SIZE as isize {
                let resp: BlockReadDmaResponse = unsafe {
                    let mut arr = [0u8; BlockReadDmaResponse::SIZE];
                    arr.copy_from_slice(&self.msg_buf[..BlockReadDmaResponse::SIZE]);
                    core::mem::transmute(arr)
                };

                if resp.status != UsbStatus::Ok {
                    return None;
                }

                // Invalidate cache to see DMA-written data
                self.invalidate_cache(resp.bytes_read as usize);

                return Some(resp.bytes_read as usize);
            }
        }

        None
    }

    /// Read a single sector
    pub fn read_sector(&mut self, lba: u64) -> Option<usize> {
        self.read(lba, 1)
    }

    /// Get a slice of the data buffer after a read operation
    ///
    /// The slice is valid until the next read operation.
    pub fn data(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.dma_virt, self.dma_size) }
    }

    /// Get a slice of specific length from the data buffer
    pub fn data_slice(&self, len: usize) -> &[u8] {
        let actual_len = core::cmp::min(len, self.dma_size);
        unsafe { core::slice::from_raw_parts(self.dma_virt, actual_len) }
    }

    /// Copy data from DMA buffer to provided slice
    pub fn copy_to(&self, dst: &mut [u8], len: usize) {
        let copy_len = core::cmp::min(len, core::cmp::min(dst.len(), self.dma_size));
        unsafe {
            core::ptr::copy_nonoverlapping(self.dma_virt, dst.as_mut_ptr(), copy_len);
        }
    }

    /// Get the block size (typically 512)
    pub fn block_size(&self) -> u32 {
        self.block_size
    }

    /// Get the total block count
    pub fn block_count(&self) -> u64 {
        self.block_count
    }

    /// Get the DMA buffer capacity in bytes
    pub fn buffer_size(&self) -> usize {
        self.dma_size
    }

    /// Get the maximum number of blocks that can be read at once
    pub fn max_blocks_per_read(&self) -> u32 {
        (self.dma_size / self.block_size as usize) as u32
    }

    /// Invalidate cache for DMA buffer (internal use)
    fn invalidate_cache(&self, len: usize) {
        unsafe {
            for offset in (0..len).step_by(64) {
                let addr = self.dma_virt.add(offset) as u64;
                core::arch::asm!("dc civac, {addr}", addr = in(reg) addr, options(nostack, preserves_flags));
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
    }
}
