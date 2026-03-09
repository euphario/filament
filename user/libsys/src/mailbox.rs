//! Mailbox — typed wrapper around the shared 4KB mailbox page.
//!
//! The mailbox page carries one-shot birth context for newly spawned children.
//! The MailboxHeader at offset 0..64 contains spawn metadata (port name,
//! port class, capabilities, parent PID, etc.). KVs follow at offset 64+.
//!
//! Runtime parent↔child communication uses SupervisionQueue (SuperQ), not
//! the mailbox page. The mailbox is read once at child startup and not used
//! for ongoing IPC.

use abi::{
    Handle, MailboxHeader,
    MAILBOX_MAGIC,
};
use crate::syscall;
use crate::error::SysError;

/// Mapped mailbox page for reading birth context at child startup.
pub struct Mailbox {
    ptr: *mut u8,
    #[allow(dead_code)]
    handle: Handle,
}

impl Mailbox {
    /// Map a mailbox shmem handle into this process's address space.
    pub fn from_handle(handle: Handle) -> Result<Self, SysError> {
        let addr = syscall::map(handle, 0)?;
        if addr == 0 {
            return Err(SysError::OutOfMemory);
        }
        Ok(Self { ptr: addr as *mut u8, handle })
    }

    /// Read the mailbox header (offset 0..64).
    pub fn header(&self) -> &MailboxHeader {
        unsafe { &*(self.ptr as *const MailboxHeader) }
    }

    /// Mutable access to the mailbox header.
    pub fn header_mut(&mut self) -> &mut MailboxHeader {
        unsafe { &mut *(self.ptr as *mut MailboxHeader) }
    }

    /// Check if this looks like a valid mailbox.
    pub fn is_valid(&self) -> bool {
        self.header().magic == MAILBOX_MAGIC
    }

    /// Raw pointer to the start of the mailbox page (for building content before spawn).
    pub fn raw_ptr(&self) -> *const u8 {
        self.ptr
    }

    /// Read spawn context KVs from the mailbox.
    ///
    /// KVs are stored after the header (offset 64+) in the format:
    ///   key_len(u8) + key + value_len(u8) + value
    /// repeated kv_count times.
    ///
    /// Returns the number of KVs read and fills the output arrays.
    pub fn read_kvs(
        &self,
        keys: &mut [[u8; 32]; 4],
        key_lens: &mut [u8; 4],
        values: &mut [[u8; 64]; 4],
        value_lens: &mut [u8; 4],
    ) -> usize {
        let hdr = self.header();
        let kv_count = (hdr.kv_count as usize).min(4);
        if kv_count == 0 {
            return 0;
        }

        // KVs start after device entries (offset 64 + 16 * device_count)
        let dev_size = hdr.device_count as usize * 16;
        let kv_start = 64 + dev_size;
        let page = unsafe { core::slice::from_raw_parts(self.ptr, 4096) };

        let mut pos = kv_start;
        let mut count = 0;
        for i in 0..kv_count {
            if pos >= 2048 { break; }
            let klen = page[pos] as usize;
            pos += 1;
            if pos + klen > 2048 { break; }
            let kl = klen.min(32);
            keys[i][..kl].copy_from_slice(&page[pos..pos + kl]);
            key_lens[i] = kl as u8;
            pos += klen;

            if pos >= 2048 { break; }
            let vlen = page[pos] as usize;
            pos += 1;
            if pos + vlen > 2048 { break; }
            let vl = vlen.min(64);
            values[i][..vl].copy_from_slice(&page[pos..pos + vl]);
            value_lens[i] = vl as u8;
            pos += vlen;
            count += 1;
        }
        count
    }
}
