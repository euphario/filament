//! Block Device IPC Client
//!
//! Communicates with usbd to read blocks from USB mass storage devices.

use userlib::{println, syscall};
use usb::{
    UsbRequest, UsbStatus, UsbMessageHeader,
    BlockReadRequest, BlockReadResponse, BlockInfoResponse,
    BlockWriteRequest, BlockWriteResponse,
    USB_MSG_MAX_SIZE, delay_ms,
};

/// Block device client (communicates with usbd via IPC)
pub struct BlockDevice {
    channel: u32,
    msg_buf: [u8; USB_MSG_MAX_SIZE],
}

impl BlockDevice {
    /// Create a new block device client
    pub fn new(channel: u32) -> Self {
        Self {
            channel,
            msg_buf: [0u8; USB_MSG_MAX_SIZE],
        }
    }

    /// Get block device info (block_size, block_count)
    pub fn get_info(&mut self) -> Option<(u32, u64)> {
        // Drain any stale messages first
        self.drain_stale_messages();

        // Build request
        let header = UsbMessageHeader::new(UsbRequest::BlockInfo, 0);
        let header_bytes = header.to_bytes();
        self.msg_buf[..UsbMessageHeader::SIZE].copy_from_slice(&header_bytes);

        // Send request
        let send_result = syscall::send(self.channel, &self.msg_buf[..UsbMessageHeader::SIZE]);
        if send_result < 0 {
            return None;
        }

        // Receive response (with response type filtering)
        let expected_type = UsbRequest::BlockInfo as u8;
        for _ in 0..100 {
            let recv_result = syscall::receive(self.channel, &mut self.msg_buf);
            if recv_result < 1 {
                // No more messages, wait 10ms
                delay_ms(10);
                syscall::yield_now();
                continue;
            }

            // Check response type (first byte)
            let resp_type = self.msg_buf[0];
            if resp_type == expected_type && recv_result >= BlockInfoResponse::SIZE as isize {
                // Parse response
                let resp_bytes: [u8; BlockInfoResponse::SIZE] = {
                    let mut arr = [0u8; BlockInfoResponse::SIZE];
                    arr.copy_from_slice(&self.msg_buf[..BlockInfoResponse::SIZE]);
                    arr
                };
                let resp: BlockInfoResponse = unsafe { core::mem::transmute(resp_bytes) };

                if resp.status != UsbStatus::Ok {
                    return None;
                }

                return Some((resp.block_size, resp.block_count));
            } else {
                // Stale message, discard and try again
                println!("[fatfs] Discarding stale message type 0x{:02x}, expected 0x{:02x}", resp_type, expected_type);
            }
        }

        None
    }

    /// Drain any stale messages from the channel
    fn drain_stale_messages(&mut self) {
        loop {
            let recv_result = syscall::receive(self.channel, &mut self.msg_buf);
            if recv_result <= 0 {
                break; // No more messages
            }
            println!("[fatfs] Drained stale message type 0x{:02x}, {} bytes", self.msg_buf[0], recv_result);
        }
    }

    /// Read a single sector (512 bytes) from the device
    pub fn read_sector(&mut self, lba: u64, buf: &mut [u8; 512]) -> bool {
        self.read_blocks(lba, 1, buf)
    }

    /// Read multiple blocks from the device
    pub fn read_blocks(&mut self, lba: u64, count: u32, buf: &mut [u8]) -> bool {
        if count == 0 || count > 64 {
            return false; // Limit to 64 blocks per request
        }

        // Drain any stale messages first
        self.drain_stale_messages();

        // Build request
        let header = UsbMessageHeader::new(UsbRequest::BlockRead, BlockReadRequest::SIZE as u32);
        let header_bytes = header.to_bytes();
        self.msg_buf[..UsbMessageHeader::SIZE].copy_from_slice(&header_bytes);

        let req = BlockReadRequest::new(0, lba, count); // slot_id 0 for now
        let req_bytes = req.to_bytes();
        self.msg_buf[UsbMessageHeader::SIZE..UsbMessageHeader::SIZE + BlockReadRequest::SIZE]
            .copy_from_slice(&req_bytes);

        // Send request
        let msg_len = UsbMessageHeader::SIZE + BlockReadRequest::SIZE;
        let send_result = syscall::send(self.channel, &self.msg_buf[..msg_len]);
        if send_result < 0 {
            return false;
        }

        // Receive response (with response type filtering)
        // USB transfers can take a while, so we need to wait longer
        let expected_type = UsbRequest::BlockRead as u8;
        for _ in 0..100 {
            let recv_result = syscall::receive(self.channel, &mut self.msg_buf);
            if recv_result < 1 {
                // No more messages, wait 10ms for USB transfer to complete
                delay_ms(10);
                syscall::yield_now();
                continue;
            }

            // Check response type (first byte)
            let resp_type = self.msg_buf[0];
            if resp_type == expected_type && recv_result >= BlockReadResponse::SIZE as isize {
                // Parse response header
                let resp_bytes: [u8; BlockReadResponse::SIZE] = {
                    let mut arr = [0u8; BlockReadResponse::SIZE];
                    arr.copy_from_slice(&self.msg_buf[..BlockReadResponse::SIZE]);
                    arr
                };
                let resp: BlockReadResponse = unsafe { core::mem::transmute(resp_bytes) };

                if resp.status != UsbStatus::Ok {
                    println!("[fatfs] read_blocks: status={:?}", resp.status);
                    return false;
                }

                // Copy data to output buffer
                let bytes_read = resp.bytes_read as usize;
                let copy_len = core::cmp::min(bytes_read, buf.len());
                let data_start = BlockReadResponse::SIZE;
                buf[..copy_len].copy_from_slice(&self.msg_buf[data_start..data_start + copy_len]);

                return true;
            } else {
                // Stale message, discard and try again
                println!("[fatfs] read_blocks: Discarding stale message type 0x{:02x}, expected 0x{:02x}", resp_type, expected_type);
            }
        }

        println!("[fatfs] read_blocks: Timeout waiting for response");
        false
    }

    /// Write a single sector (512 bytes) to the device
    pub fn write_sector(&mut self, lba: u64, buf: &[u8; 512]) -> bool {
        self.write_blocks(lba, 1, buf)
    }

    /// Write multiple blocks to the device
    pub fn write_blocks(&mut self, lba: u64, count: u32, buf: &[u8]) -> bool {
        if count == 0 || count > 1 {
            return false; // Limit to 1 block per request due to IPC size limit
        }

        // Drain any stale messages first
        self.drain_stale_messages();

        // Build request header
        let header = UsbMessageHeader::new(
            UsbRequest::BlockWrite,
            (BlockWriteRequest::SIZE + buf.len()) as u32
        );
        let header_bytes = header.to_bytes();
        self.msg_buf[..UsbMessageHeader::SIZE].copy_from_slice(&header_bytes);

        // Add BlockWriteRequest
        let req = BlockWriteRequest::new(0, lba, count); // slot_id 0 for now
        let req_bytes = req.to_bytes();
        let req_offset = UsbMessageHeader::SIZE;
        self.msg_buf[req_offset..req_offset + BlockWriteRequest::SIZE]
            .copy_from_slice(&req_bytes);

        // Add data after request
        let data_offset = req_offset + BlockWriteRequest::SIZE;
        self.msg_buf[data_offset..data_offset + buf.len()].copy_from_slice(buf);

        // Send request
        let msg_len = data_offset + buf.len();
        let send_result = syscall::send(self.channel, &self.msg_buf[..msg_len]);
        if send_result < 0 {
            println!("[fatfs] write_blocks: send failed: {}", send_result);
            return false;
        }

        // Receive response (with response type filtering)
        let expected_type = UsbRequest::BlockWrite as u8;
        for _ in 0..100 {
            let recv_result = syscall::receive(self.channel, &mut self.msg_buf);
            if recv_result < 1 {
                delay_ms(10);
                syscall::yield_now();
                continue;
            }

            // Check response type (first byte)
            let resp_type = self.msg_buf[0];
            if resp_type == expected_type && recv_result >= BlockWriteResponse::SIZE as isize {
                // Parse response
                let resp_bytes: [u8; BlockWriteResponse::SIZE] = {
                    let mut arr = [0u8; BlockWriteResponse::SIZE];
                    arr.copy_from_slice(&self.msg_buf[..BlockWriteResponse::SIZE]);
                    arr
                };
                let resp: BlockWriteResponse = unsafe { core::mem::transmute(resp_bytes) };

                if resp.status != UsbStatus::Ok {
                    println!("[fatfs] write_blocks: status={:?}", resp.status);
                    return false;
                }

                return true;
            } else {
                println!("[fatfs] write_blocks: Discarding stale message type 0x{:02x}", resp_type);
            }
        }

        println!("[fatfs] write_blocks: Timeout waiting for response");
        false
    }
}
