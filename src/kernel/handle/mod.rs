//! Unified Handle System
//!
//! This module provides a unified abstraction for kernel resources.
//! Each task has a HandleTable containing handles to kernel objects.
//!
//! ## Design
//!
//! - Handle: User-visible 32-bit value with embedded generation counter
//! - HandleRights: Capability bits controlling allowed operations
//! - KernelObject: Enum of all handle types (channels, timers, etc.)
//! - HandleTable: Per-task table of handle entries
//!
//! ## Handle Format
//!
//! ```text
//! [generation:8][index:24]
//! ```
//!
//! The generation counter prevents stale handle attacks (TOCTOU).

pub mod traits;
pub mod channel;
pub mod timer;
pub mod console;
pub mod klog;
pub mod child_exit;
pub mod shmem;
pub mod port;

// Re-export commonly used types
pub use traits::{Waitable, Closable, WaitResult, WaitFilter, CloseAction};
pub use channel::ChannelObject;
pub use timer::TimerObject;
pub use console::ConsoleObject;
pub use klog::KlogObject;
pub use child_exit::ChildExitObject;
pub use shmem::ShmemObject;
pub use port::PortObject;

// ============================================================================
// Handle Type - imported from abi crate (single source of truth)
// ============================================================================

/// Maximum handles per task
pub const MAX_HANDLES: usize = 64;

/// User-visible handle with embedded generation counter
/// Imported from abi crate for consistency across kernel/userspace
pub use abi::Handle;

/// Error codes and HandleRights - imported from ABI (single source of truth)
use abi::errno;
pub use abi::HandleRights;

// ============================================================================
// Kernel Objects
// ============================================================================

/// The kernel object referenced by a handle
///
/// This enum holds the kernel-side state for each handle type.
/// Each variant wraps a specific object type.
pub enum KernelObject {
    /// IPC channel endpoint
    Channel(ChannelObject),

    /// Timer handle
    Timer(TimerObject),

    /// Console I/O (stdin/stdout/stderr)
    Console(ConsoleObject),

    /// Kernel log buffer (for logd)
    Klog(KlogObject),

    /// Child process exit notification (for devd)
    ChildExit(ChildExitObject),

    /// Shared memory region monitoring (for consoled)
    Shmem(ShmemObject),

    /// Named port (listening endpoint)
    Port(PortObject),

    // Future: Irq, Task, Ramfs, Mmio, Scheme
}

impl KernelObject {
    /// Get object type as string (for debugging)
    pub fn type_name(&self) -> &'static str {
        match self {
            KernelObject::Channel(_) => "channel",
            KernelObject::Timer(_) => "timer",
            KernelObject::Console(_) => "console",
            KernelObject::Klog(_) => "klog",
            KernelObject::ChildExit(_) => "child_exit",
            KernelObject::Shmem(_) => "shmem",
            KernelObject::Port(_) => "port",
        }
    }

    /// Register a waker so task is woken when this object has events
    pub fn register_waker(&mut self, task_id: super::task::TaskId) {
        match self {
            KernelObject::Channel(ch) => ch.register_waker(task_id),
            KernelObject::Port(port) => port.register_waker(task_id),
            KernelObject::Console(con) => con.register_waker(task_id),
            KernelObject::ChildExit(ce) => ce.register_waker(task_id),
            KernelObject::Shmem(sh) => sh.register_waker(task_id),
            // Timer and Klog don't need wakers - polled via deadline/queue
            KernelObject::Timer(_) => {}
            KernelObject::Klog(_) => {}
        }
    }

    /// Unregister a waker
    pub fn unregister_waker(&mut self, task_id: super::task::TaskId) {
        match self {
            KernelObject::Channel(ch) => ch.unregister_waker(task_id),
            KernelObject::Port(port) => port.unregister_waker(task_id),
            KernelObject::Console(con) => con.unregister_waker(task_id),
            KernelObject::ChildExit(ce) => ce.unregister_waker(task_id),
            KernelObject::Shmem(sh) => sh.unregister_waker(task_id),
            KernelObject::Timer(_) => {}
            KernelObject::Klog(_) => {}
        }
    }
}

// ============================================================================
// Handle Entry
// ============================================================================

/// A single entry in the handle table
/// Blocking mode for I/O operations
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum BlockingMode {
    /// Operations block until complete or data available
    #[default]
    Blocking,
    /// Operations return EAGAIN immediately if would block
    NonBlocking,
}

impl BlockingMode {
    /// Check if in blocking mode
    pub fn is_blocking(&self) -> bool {
        matches!(self, BlockingMode::Blocking)
    }

    /// Check if in non-blocking mode
    pub fn is_nonblocking(&self) -> bool {
        matches!(self, BlockingMode::NonBlocking)
    }
}

pub struct HandleEntry {
    /// Generation counter (must match Handle for validity)
    pub generation: u8,

    /// Rights granted to this handle
    pub rights: HandleRights,

    /// The kernel object this handle refers to
    pub object: KernelObject,

    /// Current file offset (for seekable objects)
    pub offset: u64,

    /// Blocking mode for I/O operations
    pub blocking_mode: BlockingMode,
}

impl HandleEntry {
    /// Create a new handle entry
    pub fn new(generation: u8, object: KernelObject, rights: HandleRights) -> Self {
        Self {
            generation,
            rights,
            object,
            offset: 0,
            blocking_mode: BlockingMode::Blocking,
        }
    }

    /// Check if handle is in non-blocking mode
    pub fn is_nonblocking(&self) -> bool {
        self.blocking_mode.is_nonblocking()
    }
}

// ============================================================================
// Handle Table
// ============================================================================

/// Handle error codes
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HandleError {
    /// Handle index out of range or invalid
    InvalidHandle,
    /// Generation mismatch (stale handle)
    StaleHandle,
    /// Operation not permitted for this handle's rights
    PermissionDenied,
    /// Object type doesn't support this operation
    NotSupported,
    /// Handle table is full
    NoSpace,
    /// Peer closed (for channels)
    PeerClosed,
    /// Would block (non-blocking mode)
    WouldBlock,
}

impl HandleError {
    /// Convert to errno-style negative value
    pub fn to_errno(&self) -> i64 {
        match self {
            HandleError::InvalidHandle => errno::EBADF as i64,
            HandleError::StaleHandle => errno::EBADF as i64,
            HandleError::PermissionDenied => errno::EPERM as i64,
            HandleError::NotSupported => errno::ENOSYS as i64,
            HandleError::NoSpace => errno::ENOSPC as i64,
            HandleError::PeerClosed => errno::EPIPE as i64,
            HandleError::WouldBlock => errno::EAGAIN as i64,
        }
    }
}

/// Per-task handle table
///
/// Stores up to MAX_HANDLES entries, with generation counters
/// to prevent stale handle reuse.
pub struct HandleTable {
    /// Handle entries (None = free slot)
    entries: [Option<HandleEntry>; MAX_HANDLES],

    /// Generation counter per slot (survives across allocations)
    generations: [u8; MAX_HANDLES],
}

impl HandleTable {
    /// Create a new empty handle table
    pub const fn new() -> Self {
        Self {
            entries: [const { None }; MAX_HANDLES],
            generations: [1u8; MAX_HANDLES], // Start at 1 so 0 is always invalid
        }
    }

    /// Allocate a new handle for an object
    ///
    /// Returns the handle if successful, None if table is full.
    pub fn alloc(&mut self, object: KernelObject, rights: HandleRights) -> Option<Handle> {
        // Find first free slot (skip slot 0 - reserved for INVALID)
        for i in 1..MAX_HANDLES {
            if self.entries[i].is_none() {
                let gen = self.generations[i];
                self.entries[i] = Some(HandleEntry::new(gen, object, rights));
                return Some(Handle::new(i, gen));
            }
        }
        None
    }

    /// Get entry by handle (validates generation)
    pub fn get(&self, handle: Handle) -> Result<&HandleEntry, HandleError> {
        let idx = handle.index();
        if idx >= MAX_HANDLES {
            return Err(HandleError::InvalidHandle);
        }

        match &self.entries[idx] {
            Some(entry) if entry.generation == handle.generation() => Ok(entry),
            Some(_) => Err(HandleError::StaleHandle),
            None => Err(HandleError::InvalidHandle),
        }
    }

    /// Get entry mutably
    pub fn get_mut(&mut self, handle: Handle) -> Result<&mut HandleEntry, HandleError> {
        let idx = handle.index();
        if idx >= MAX_HANDLES {
            return Err(HandleError::InvalidHandle);
        }

        match &mut self.entries[idx] {
            Some(entry) if entry.generation == handle.generation() => Ok(entry),
            Some(_) => Err(HandleError::StaleHandle),
            None => Err(HandleError::InvalidHandle),
        }
    }

    /// Close a handle
    ///
    /// Increments generation counter to invalidate any copies of this handle.
    pub fn close(&mut self, handle: Handle) -> Result<KernelObject, HandleError> {
        let idx = handle.index();
        if idx >= MAX_HANDLES {
            return Err(HandleError::InvalidHandle);
        }

        // Verify generation
        if let Some(ref entry) = self.entries[idx] {
            if entry.generation != handle.generation() {
                return Err(HandleError::StaleHandle);
            }
        } else {
            return Err(HandleError::InvalidHandle);
        }

        // Take the entry and bump generation
        let entry = match self.entries[idx].take() {
            Some(e) => e,
            None => return Err(HandleError::InvalidHandle), // Defensive: should never happen
        };
        self.generations[idx] = self.generations[idx].wrapping_add(1);
        // Skip 0 generation (reserved for INVALID)
        if self.generations[idx] == 0 {
            self.generations[idx] = 1;
        }

        Ok(entry.object)
    }

    /// Duplicate a handle with (possibly reduced) rights
    pub fn duplicate(&mut self, handle: Handle, new_rights: HandleRights) -> Result<Handle, HandleError> {
        // Validate source handle
        let (object_clone, effective_rights) = {
            let entry = self.get(handle)?;
            if !entry.rights.has(HandleRights::DUPLICATE) {
                return Err(HandleError::PermissionDenied);
            }
            // Can only reduce rights, not expand
            let effective = entry.rights.and(new_rights);
            // Clone the object (we need to implement Clone for KernelObject types)
            // For now, this only works for some object types
            let obj_clone = match &entry.object {
                KernelObject::Channel(ch) => KernelObject::Channel(ch.clone()),
                KernelObject::Timer(t) => KernelObject::Timer(t.clone()),
                KernelObject::Console(c) => KernelObject::Console(c.clone()),
                KernelObject::Klog(k) => KernelObject::Klog(k.clone()),
                KernelObject::ChildExit(ce) => KernelObject::ChildExit(ce.clone()),
                KernelObject::Shmem(s) => KernelObject::Shmem(s.clone()),
                KernelObject::Port(_) => return Err(HandleError::NotSupported), // Ports cannot be duplicated
            };
            (obj_clone, effective)
        };

        self.alloc(object_clone, effective_rights)
            .ok_or(HandleError::NoSpace)
    }

    /// Close all handles (for task cleanup)
    pub fn close_all(&mut self, owner_pid: u32) {
        for i in 1..MAX_HANDLES {
            if let Some(entry) = self.entries[i].take() {
                // Bump generation
                self.generations[i] = self.generations[i].wrapping_add(1);
                if self.generations[i] == 0 {
                    self.generations[i] = 1;
                }

                // Perform cleanup based on object type
                match entry.object {
                    KernelObject::Channel(ch) => {
                        ch.close(owner_pid);
                    }
                    KernelObject::Timer(_) => {
                        // Timers don't need special cleanup
                    }
                    KernelObject::Console(_) => {
                        // Console doesn't need cleanup
                    }
                    KernelObject::Klog(_) => {
                        // Klog handles don't need special cleanup
                    }
                    KernelObject::ChildExit(_) => {
                        // ChildExit handles don't need special cleanup
                    }
                    KernelObject::Shmem(_) => {
                        // Shmem handles don't need special cleanup
                        // The actual shmem region is managed separately
                    }
                    KernelObject::Port(port) => {
                        // Unregister the port from the global registry
                        let action = port.close(owner_pid);
                        if let CloseAction::UnregisterPort { port_id } = action {
                            crate::kernel::ipc::port_unregister_by_channel(port_id);
                        }
                    }
                }
            }
        }
    }

    /// Check if a handle is valid
    pub fn is_valid(&self, handle: Handle) -> bool {
        self.get(handle).is_ok()
    }

    /// Count allocated handles
    pub fn count(&self) -> usize {
        self.entries.iter().filter(|e| e.is_some()).count()
    }

    /// Get mutable iterator over all entries (for kernel-side notifications)
    ///
    /// Used by child_exit notification to iterate parent's handles.
    pub fn entries_mut(&mut self) -> impl Iterator<Item = &mut Option<HandleEntry>> {
        self.entries.iter_mut()
    }
}

impl Default for HandleTable {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_handle_encoding() {
        let h = Handle::new(42, 7);
        assert_eq!(h.index(), 42);
        assert_eq!(h.generation(), 7);

        let h2 = Handle::new(0x00FFFFFF, 255);
        assert_eq!(h2.index(), 0x00FFFFFF);
        assert_eq!(h2.generation(), 255);
    }

    #[test]
    fn test_handle_invalid() {
        assert!(!Handle::INVALID.is_valid());
        assert!(Handle::new(1, 1).is_valid());
    }

    #[test]
    fn test_rights_operations() {
        let rw = HandleRights::READ.or(HandleRights::WRITE);
        assert!(rw.has(HandleRights::READ));
        assert!(rw.has(HandleRights::WRITE));
        assert!(!rw.has(HandleRights::WAIT));

        let restricted = rw.and(HandleRights::READ);
        assert!(restricted.has(HandleRights::READ));
        assert!(!restricted.has(HandleRights::WRITE));
    }
}
