//! Block Service - serves block read/write requests to clients
//!
//! Handles BLK_INFO, BLK_READ, BLK_WRITE requests from filesystem drivers.

use userlib::syscall::{self, Handle};
use userlib::blk::{
    BlkHeader, BlkInfoResp, BlkRead, BlkData, BlkResult,
    msg, error,
};

use crate::log;

/// Stored partition info for block service
#[derive(Clone, Copy)]
pub struct PartitionInfo {
    pub start_lba: u64,
    pub block_count: u64,
    pub block_size: u32,
    pub slot_id: u8,
    pub bulk_in_dci: u8,
    pub bulk_out_dci: u8,
}

impl PartitionInfo {
    pub fn empty() -> Self {
        Self {
            start_lba: 0,
            block_count: 0,
            block_size: 512,
            slot_id: 0,
            bulk_in_dci: 0,
            bulk_out_dci: 0,
        }
    }

    pub fn is_valid(&self) -> bool {
        self.block_count > 0
    }
}

/// Parse and dispatch a block service request
/// Returns (msg_type, seq_id, Option<BlkRead>) for the caller to handle
pub fn parse_request(buf: &[u8], len: usize) -> Option<(u16, u32, Option<BlkRead>)> {
    if len < BlkHeader::SIZE {
        return None;
    }

    let header = BlkHeader::from_bytes(&buf[..len])?;

    match header.msg_type {
        msg::BLK_INFO => Some((msg::BLK_INFO, header.seq_id, None)),
        msg::BLK_READ => {
            let req = BlkRead::from_bytes(&buf[..len])?;
            Some((msg::BLK_READ, header.seq_id, Some(req)))
        }
        msg::BLK_WRITE => Some((msg::BLK_WRITE, header.seq_id, None)),
        _ => None,
    }
}

/// Send BLK_INFO_RESP
pub fn send_info_response(channel_handle: Handle, seq_id: u32, info: &PartitionInfo) {
    let resp = BlkInfoResp::new(seq_id, info.block_size, info.block_count);
    let _ = syscall::write(channel_handle, &resp.to_bytes());
}

/// Send BLK_DATA response with block data
/// NOTE: IPC payload limit is 576 bytes, so we can only send 1 sector
/// (512 data + 16 header = 528 bytes fits in 576)
pub fn send_read_response(channel_handle: Handle, seq_id: u32, data: &[u8]) {
    // Build response: header + data
    let mut resp_buf = [0u8; 512 + 16]; // 512 bytes data + 16 byte header
    let data_len = data.len().min(512);

    // Header
    let header = BlkHeader::new(msg::BLK_DATA, seq_id);
    resp_buf[0..8].copy_from_slice(&header.to_bytes());
    resp_buf[8..12].copy_from_slice(&(data_len as u32).to_le_bytes());
    resp_buf[12..16].copy_from_slice(&(error::OK).to_le_bytes());

    // Data
    resp_buf[16..16 + data_len].copy_from_slice(&data[..data_len]);

    let _ = syscall::write(channel_handle, &resp_buf[..16 + data_len]);
}

/// Send error response
pub fn send_error_response(channel_handle: Handle, seq_id: u32, err: i32) {
    let resp = BlkResult::new(seq_id, err);
    let _ = syscall::write(channel_handle, &resp.to_bytes());
}
