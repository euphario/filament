//! Miscellaneous Operations Backend Implementation
//!
//! Implements the `MiscOps` trait for logging and ramfs operations.

use crate::kernel::traits::misc_ops::MiscOps;
use crate::kernel::error::KernelError;

pub struct KernelMiscOps;

impl KernelMiscOps {
    pub const fn new() -> Self {
        Self
    }
}

impl MiscOps for KernelMiscOps {
    fn write_user_log(&self, caller_pid: u32, level: u8, msg: &[u8]) -> i64 {
        let level = match crate::klog::Level::from_u8(level) {
            Some(l) => l,
            None => return KernelError::InvalidArg.to_errno(),
        };

        if msg.is_empty() || msg.len() > 256 {
            return KernelError::InvalidArg.to_errno();
        }

        let mut builder = crate::klog::RecordBuilder::new();
        builder.header(level);
        builder.subsys("user");

        // Parse [name] prefix for event name
        let (event, _rest) = if msg.starts_with(b"[") {
            if let Some(end) = msg.iter().position(|&c| c == b']') {
                let name = core::str::from_utf8(&msg[1..end]).unwrap_or("msg");
                (name, &msg[end + 1..])
            } else {
                ("msg", msg)
            }
        } else {
            ("msg", msg)
        };

        builder.event(event);
        builder.ctx_count(0);

        if let Ok(text) = core::str::from_utf8(msg) {
            let trimmed = text.trim();
            if trimmed.is_empty() && level == crate::klog::Level::Error {
                crate::kwarn!("syscall", "empty_klog"; pid = caller_pid as u64, len = msg.len() as u64);
            }
            builder.kv_count(2);
            builder.kv("pid", caller_pid as u64);
            builder.kv("text", text);
        } else {
            builder.kv_count(1);
            builder.kv("pid", caller_pid as u64);
        }

        builder.finish();
        msg.len() as i64
    }

    fn read_log_record(&self, buf: &mut [u8]) -> i64 {
        let mut record_buf = [0u8; crate::klog::MAX_RECORD_SIZE];

        let record_len = {
            let mut ring = crate::klog::LOG_RING.lock();
            ring.read(&mut record_buf)
        };

        let Some(len) = record_len else {
            return 0;
        };

        let text_len = crate::klog::format_record(&record_buf[..len], buf);
        if text_len == 0 {
            return 0;
        }

        text_len as i64
    }

    fn write_raw_record(&self, record: &[u8]) -> i64 {
        if record.len() < crate::klog::RecordHeader::SIZE || record.len() > crate::klog::MAX_RECORD_SIZE {
            return KernelError::InvalidArg.to_errno();
        }

        // Validate total_len matches
        let total_len = u16::from_le_bytes([record[0], record[1]]) as usize;
        if total_len != record.len() {
            return KernelError::InvalidArg.to_errno();
        }

        // Validate level
        let level = record[6];
        if crate::klog::Level::from_u8(level).is_none() {
            return KernelError::InvalidArg.to_errno();
        }

        // Re-stamp timestamp from kernel clock
        let mut stamped = [0u8; crate::klog::MAX_RECORD_SIZE];
        stamped[..record.len()].copy_from_slice(record);
        let ts = crate::klog::timestamp_ms();
        stamped[2] = ts as u8;
        stamped[3] = (ts >> 8) as u8;
        stamped[4] = (ts >> 16) as u8;
        stamped[5] = (ts >> 24) as u8;

        {
            let mut ring = crate::klog::LOG_RING.lock();
            ring.write(&stamped[..record.len()]);
        }

        record.len() as i64
    }

    fn sleep_until(&self, deadline: u64) -> Result<(), KernelError> {
        use crate::kernel::{sched, task::WaitReason};
        if !sched::wait_and_reschedule(WaitReason::Timer, deadline) {
            return Err(KernelError::InvalidArg);
        }
        Ok(())
    }

    fn list_ramfs(&self, buf: &mut [abi::RamfsListEntry], max: usize) -> usize {
        let ramfs = crate::ramfs::ramfs();
        let count = ramfs.len().min(max);

        for i in 0..count {
            if let Some(entry) = ramfs.get(i) {
                let mut list_entry = abi::RamfsListEntry {
                    name: [0u8; 100],
                    size: entry.size as u64,
                    file_type: if entry.is_dir() { 1 } else { 0 },
                    _pad: [0u8; 7],
                };
                let name_len = entry.name.iter().position(|&c| c == 0).unwrap_or(100).min(100);
                list_entry.name[..name_len].copy_from_slice(&entry.name[..name_len]);
                buf[i] = list_entry;
            }
        }

        count
    }
}

/// Global kernel misc operations backend
pub static MISC_OPS_BACKEND: KernelMiscOps = KernelMiscOps::new();

/// Get a reference to the global misc operations backend
pub fn misc_ops_backend() -> &'static dyn MiscOps {
    &MISC_OPS_BACKEND
}
