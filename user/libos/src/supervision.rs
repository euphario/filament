//! Supervision Handle — userspace wrapper for kernel SupervisionQueue
//!
//! SupervisionHandle wraps the kernel-side SupervisionQueue object,
//! providing typed send/recv for SupervisionNote messages.
//!
//! Parent end (SupervisionParent): reads from `up` ring (child→parent),
//! writes to `down` ring (parent→child).
//!
//! Child end (SupervisionChild): reads from `down` ring (parent→child),
//! writes to `up` ring (child→parent).

use libsys::error::SysError;
use libsys::ipc::ObjHandle;
use abi::{Handle, SupervisionNote};

/// Wrapper around a kernel SupervisionQueue handle.
///
/// Works for both parent and child ends — the kernel dispatches
/// reads/writes to the correct ring direction based on the object type.
pub struct SupervisionHandle {
    handle: Handle,
}

impl SupervisionHandle {
    /// Wrap a raw handle (e.g., from exec_with_mailbox return value
    /// or from Handle::SUPERVISION slot 4 for child end).
    pub fn from_handle(handle: Handle) -> Self {
        Self { handle }
    }

    /// Send a supervision note to the peer.
    ///
    /// Parent→child: enqueues in `down` ring.
    /// Child→parent: enqueues in `up` ring.
    pub fn send(&self, note: &SupervisionNote) -> Result<(), SysError> {
        let bytes = unsafe {
            core::slice::from_raw_parts(
                note as *const SupervisionNote as *const u8,
                core::mem::size_of::<SupervisionNote>(),
            )
        };
        libsys::syscall::write(self.handle, bytes)?;
        Ok(())
    }

    /// Receive a supervision note from the peer (blocking).
    ///
    /// Uses wait_one() to block until data is available, then reads.
    /// Same pattern as Channel blocking reads — consistent kernel design.
    ///
    /// Parent: dequeues from `up` ring (child→parent).
    /// Child: dequeues from `down` ring (parent→child).
    pub fn recv(&self) -> Result<SupervisionNote, SysError> {
        loop {
            match self.try_recv()? {
                Some(note) => return Ok(note),
                None => {
                    libsys::ipc::wait_one(self.handle())?;
                }
            }
        }
    }

    /// Try to receive a supervision note (non-blocking).
    ///
    /// Returns Ok(Some(note)) if data was available, Ok(None) if empty.
    pub fn try_recv(&self) -> Result<Option<SupervisionNote>, SysError> {
        let mut note = SupervisionNote::empty();
        let bytes = unsafe {
            core::slice::from_raw_parts_mut(
                &mut note as *mut SupervisionNote as *mut u8,
                core::mem::size_of::<SupervisionNote>(),
            )
        };
        match libsys::syscall::try_read(self.handle, bytes) {
            Ok(Some(_)) => Ok(Some(note)),
            Ok(None) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Get the underlying handle for Mux registration.
    pub fn handle(&self) -> ObjHandle {
        Handle(self.handle.raw())
    }

    /// Get the raw Handle value.
    pub fn raw_handle(&self) -> Handle {
        self.handle
    }

    /// Send raw bytes as one or more FORWARD notes.
    ///
    /// Messages ≤ 120 bytes fit in a single note. Larger messages are
    /// fragmented: flags bit 0 (HAS_MORE) indicates continuation notes.
    /// All fragments share the same `seq` value.
    pub fn send_forward(&self, data: &[u8], seq: u16) -> Result<(), SysError> {
        if data.is_empty() {
            return Ok(());
        }
        let total_chunks = (data.len() + 119) / 120;
        let mut offset = 0;
        for i in 0..total_chunks {
            let chunk_len = (data.len() - offset).min(120);
            let mut note = SupervisionNote::empty();
            note.note_type = abi::supervision_note::FORWARD;
            note.flags = if i + 1 < total_chunks { 1 } else { 0 }; // HAS_MORE
            note.seq = seq;
            note.len = chunk_len as u16;
            note.payload[..chunk_len].copy_from_slice(&data[offset..offset + chunk_len]);
            self.send(&note)?;
            offset += chunk_len;
        }
        Ok(())
    }

    /// Receive one complete message from FORWARD notes (blocking).
    ///
    /// Reads notes and reassembles fragments into `buf`. Returns the
    /// total number of bytes or None if the first note is not a FORWARD.
    pub fn recv_forward(&self, buf: &mut [u8]) -> Result<Option<usize>, SysError> {
        let note = self.try_recv()?;
        let note = match note {
            Some(n) => n,
            None => return Ok(None),
        };
        if note.note_type != abi::supervision_note::FORWARD {
            return Ok(None);
        }
        let len = (note.len as usize).min(120).min(buf.len());
        buf[..len].copy_from_slice(&note.payload[..len]);
        let mut pos = len;

        // Read continuation fragments
        if note.flags & 1 != 0 {
            loop {
                let cont = self.recv()?;
                let clen = (cont.len as usize).min(120);
                let n = clen.min(buf.len().saturating_sub(pos));
                if n > 0 {
                    buf[pos..pos + n].copy_from_slice(&cont.payload[..n]);
                    pos += n;
                }
                if cont.flags & 1 == 0 {
                    break;
                }
            }
        }
        Ok(Some(pos))
    }
}

impl Drop for SupervisionHandle {
    fn drop(&mut self) {
        let _ = libsys::syscall::close(self.handle);
    }
}
