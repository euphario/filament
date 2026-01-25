//! Object Service - Central Owner of Object Tables
//!
//! This module implements the ObjectService which owns all per-task object tables.
//! This is the key architectural change: object tables are NOT in the Task struct,
//! they're owned by ObjectService with its own lock.
//!
//! # Architecture
//!
//! ```text
//! ┌────────────────────────────────────────────────────────────────┐
//! │                    Syscall Layer (thin)                        │
//! │    sys_open() → validate args → ObjectService.open()           │
//! │    sys_read() → validate args → ObjectService.read()           │
//! └───────────────────────────────┬────────────────────────────────┘
//!                                 │
//! ┌───────────────────────────────▼────────────────────────────────┐
//! │                    ObjectService                               │
//! │    SpinLock<[Option<HandleTable>; MAX_TASKS]>                  │
//! │    - Owns all object tables (indexed by slot, not task_id)    │
//! │    - Blocking read() implementation                            │
//! │    - Wakes tasks when events occur                             │
//! └────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Key Design
//!
//! - **ObjectService owns object tables** - Not in Task struct
//! - **Own lock** - Not nested inside scheduler lock
//! - **Blocking read()** - Loop: try_read → subscribe → block → retry
//! - **Mux for multiplexing** - The only way to wait on multiple handles

use crate::kernel::lock::SpinLock;
use crate::kernel::task::{TaskId, MAX_TASKS};
use crate::kernel::object::{HandleTable, Handle, Object, ObjectType, ConsoleType};
use crate::kernel::object::handle::HandleEntry;

// ============================================================================
// Error Types
// ============================================================================

/// Errors from ObjectService operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjError {
    /// Invalid handle (wrong generation or not found)
    BadHandle,
    /// Object type mismatch
    TypeMismatch,
    /// Operation not supported for this object type
    NotSupported,
    /// Would block (internal - not returned to userspace)
    WouldBlock,
    /// Peer closed
    Closed,
    /// Out of memory
    OutOfMemory,
    /// Out of handles
    OutOfHandles,
    /// Invalid argument
    InvalidArg,
    /// Permission denied
    PermissionDenied,
    /// Task not found
    TaskNotFound,
    /// Already exists
    AlreadyExists,
}

impl ObjError {
    /// Convert to errno for syscall return
    pub fn to_errno(self) -> i64 {
        match self {
            ObjError::BadHandle => -9,      // EBADF
            ObjError::TypeMismatch => -22,  // EINVAL
            ObjError::NotSupported => -38,  // ENOSYS
            ObjError::WouldBlock => -11,    // EAGAIN (internal only)
            ObjError::Closed => -32,        // EPIPE
            ObjError::OutOfMemory => -12,   // ENOMEM
            ObjError::OutOfHandles => -24,  // EMFILE
            ObjError::InvalidArg => -22,    // EINVAL
            ObjError::PermissionDenied => -1,  // EPERM
            ObjError::TaskNotFound => -3,   // ESRCH
            ObjError::AlreadyExists => -17, // EEXIST
        }
    }

    /// Convert from errno
    pub fn from_errno(e: i64) -> Self {
        match e {
            -9 => ObjError::BadHandle,
            -22 => ObjError::InvalidArg,
            -38 => ObjError::NotSupported,
            -11 => ObjError::WouldBlock,
            -32 => ObjError::Closed,
            -12 => ObjError::OutOfMemory,
            -24 => ObjError::OutOfHandles,
            -13 => ObjError::PermissionDenied,
            -3 => ObjError::TaskNotFound,
            -17 => ObjError::AlreadyExists,
            _ => ObjError::InvalidArg, // Default
        }
    }
}

// ============================================================================
// Read Attempt Result (for blocking implementation)
// ============================================================================

/// Result of a non-blocking read attempt
pub enum ReadAttempt {
    /// Data successfully read
    Success(usize),
    /// Object is closed, no more data
    Closed,
    /// No data available, need to block
    NeedBlock,
}

// ============================================================================
// ObjectService
// ============================================================================

/// Extract slot index from TaskId
///
/// PID format: bits[7:0] = slot + 1 (1-16), bits[31:8] = generation
/// Returns None if invalid (slot 0 is reserved)
fn slot_from_task_id(task_id: TaskId) -> Option<usize> {
    let slot_bits = (task_id & 0xFF) as usize;
    if slot_bits == 0 || slot_bits > MAX_TASKS {
        return None;
    }
    Some(slot_bits - 1)
}

/// Central service that owns all object tables
///
/// This is the single source of truth for object state. Tasks don't have
/// object tables - ObjectService does.
///
/// Uses a fixed-size array indexed by slot (extracted from task_id).
/// This avoids heap allocation in a no_std kernel.
pub struct ObjectService {
    /// Per-slot object tables (indexed by slot, not task_id)
    tables: SpinLock<[Option<HandleTable>; MAX_TASKS]>,
}

impl ObjectService {
    /// Create a new ObjectService
    pub const fn new() -> Self {
        const NONE: Option<HandleTable> = None;
        Self {
            tables: SpinLock::new([NONE; MAX_TASKS]),
        }
    }

    // ========================================================================
    // Task Lifecycle
    // ========================================================================

    /// Create object table for a new task
    ///
    /// Called when a task is created. Creates an empty HandleTable
    /// (or one with stdin/stdout/stderr for user tasks).
    pub fn create_task_table(&self, task_id: TaskId, is_user: bool) {
        let slot = match slot_from_task_id(task_id) {
            Some(s) => s,
            None => return, // Invalid task_id
        };

        let table = if is_user {
            HandleTable::new_with_stdio()
        } else {
            HandleTable::new()
        };

        let mut tables = self.tables.lock();
        tables[slot] = Some(table);
    }

    /// Remove object table for an exiting task
    ///
    /// Called when a task exits. Closes all handles and removes the table.
    /// Returns the list of objects that were closed (for cleanup).
    pub fn remove_task_table(&self, task_id: TaskId) -> Option<HandleTable> {
        let slot = slot_from_task_id(task_id)?;
        let mut tables = self.tables.lock();
        tables[slot].take()
    }

    /// Check if a task has an object table
    pub fn task_exists(&self, task_id: TaskId) -> bool {
        let slot = match slot_from_task_id(task_id) {
            Some(s) => s,
            None => return false,
        };
        let tables = self.tables.lock();
        tables[slot].is_some()
    }

    // ========================================================================
    // Handle Operations (Phase 1 - just wrappers, no blocking yet)
    // ========================================================================

    /// Allocate a new handle in a task's object table
    pub fn alloc_handle(
        &self,
        task_id: TaskId,
        object_type: ObjectType,
        object: Object,
    ) -> Result<Handle, ObjError> {
        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        table.alloc(object_type, object).ok_or(ObjError::OutOfHandles)
    }

    /// Get a handle entry (immutable)
    pub fn get_entry<F, R>(&self, task_id: TaskId, handle: Handle, f: F) -> Result<R, ObjError>
    where
        F: FnOnce(&HandleEntry) -> R,
    {
        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;
        let tables = self.tables.lock();
        let table = tables[slot].as_ref().ok_or(ObjError::TaskNotFound)?;
        let entry = table.get(handle).ok_or(ObjError::BadHandle)?;
        Ok(f(entry))
    }

    /// Get a handle entry (mutable)
    pub fn get_entry_mut<F, R>(&self, task_id: TaskId, handle: Handle, f: F) -> Result<R, ObjError>
    where
        F: FnOnce(&mut HandleEntry) -> R,
    {
        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let entry = table.get_mut(handle).ok_or(ObjError::BadHandle)?;
        Ok(f(entry))
    }

    /// Close a handle
    pub fn close_handle(&self, task_id: TaskId, handle: Handle) -> Result<Object, ObjError> {
        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        table.close(handle).ok_or(ObjError::BadHandle)
    }

    /// Access the full table for a task (for complex operations like Mux polling)
    pub fn with_table<F, R>(&self, task_id: TaskId, f: F) -> Result<R, ObjError>
    where
        F: FnOnce(&HandleTable) -> R,
    {
        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;
        let tables = self.tables.lock();
        let table = tables[slot].as_ref().ok_or(ObjError::TaskNotFound)?;
        Ok(f(table))
    }

    /// Access the full table mutably for a task
    pub fn with_table_mut<F, R>(&self, task_id: TaskId, f: F) -> Result<R, ObjError>
    where
        F: FnOnce(&mut HandleTable) -> R,
    {
        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        Ok(f(table))
    }

    // ========================================================================
    // Object Operations (Phase 3 - will implement blocking read/write)
    // ========================================================================

    // ========================================================================
    // Console Operations (Phase 3 - First migration)
    // ========================================================================

    /// Open a console handle (stdin/stdout/stderr)
    ///
    /// Returns handle on success, error on failure.
    pub fn open_console(&self, task_id: TaskId, console_type: ConsoleType) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, ConsoleObject};

        let obj = Object::Console(ConsoleObject::new(console_type));
        let obj_type = match console_type {
            ConsoleType::Stdin => ObjectType::Stdin,
            ConsoleType::Stdout => ObjectType::Stdout,
            ConsoleType::Stderr => ObjectType::Stderr,
        };

        self.alloc_handle(task_id, obj_type, obj)
    }

    /// Read from console (stdin only)
    ///
    /// Non-blocking: returns data if available, WouldBlock otherwise.
    /// The caller (syscall layer) handles blocking.
    ///
    /// MIGRATION NOTE: Uses task.object_table for storage during Phase 3.
    pub fn read_console(
        &self,
        task_id: TaskId,
        handle: Handle,
        buf: &mut [u8],
    ) -> Result<ReadAttempt, ObjError> {
        use crate::platform::current::uart;
        use crate::kernel::ipc::traits::Subscriber;
        use crate::kernel::object::{Object, Pollable};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let entry = table.get_mut(handle).ok_or(ObjError::BadHandle)?;

        let Object::Console(ref mut c) = entry.object else {
            return Err(ObjError::TypeMismatch);
        };

        // Only stdin is readable
        if !matches!(c.console_type(), ConsoleType::Stdin) {
            return Err(ObjError::NotSupported);
        }

        if buf.is_empty() {
            return Ok(ReadAttempt::Success(0));
        }

        // Read from UART RX buffer
        let mut bytes_read = 0;
        for byte in buf.iter_mut() {
            if let Some(b) = uart::rx_buffer_read() {
                *byte = b;
                bytes_read += 1;
            } else {
                break;
            }
        }

        if bytes_read > 0 {
            Ok(ReadAttempt::Success(bytes_read))
        } else {
            // Register for wake when input arrives (IRQ-based)
            let sub = Subscriber {
                task_id,
                generation: crate::kernel::task::Scheduler::generation_from_pid(task_id),
            };
            c.subscribe(sub);
            Ok(ReadAttempt::NeedBlock)
        }
    }

    /// Write to console (stdout/stderr only)
    ///
    /// Always completes immediately (UART is buffered).
    pub fn write_console(
        &self,
        task_id: TaskId,
        handle: Handle,
        buf: &[u8],
    ) -> Result<usize, ObjError> {
        use crate::platform::current::uart;
        use crate::kernel::object::Object;

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;
        let tables = self.tables.lock();
        let table = tables[slot].as_ref().ok_or(ObjError::TaskNotFound)?;
        let entry = table.get(handle).ok_or(ObjError::BadHandle)?;

        let Object::Console(ref c) = entry.object else {
            return Err(ObjError::TypeMismatch);
        };

        // Only stdout/stderr are writable
        match c.console_type() {
            ConsoleType::Stdout | ConsoleType::Stderr => {
                if buf.is_empty() {
                    return Ok(0);
                }
                uart::write_buffered(buf);
                uart::flush_buffer();
                Ok(buf.len())
            }
            ConsoleType::Stdin => Err(ObjError::NotSupported),
        }
    }

    // ========================================================================
    // Timer Operations (Phase 3)
    // ========================================================================

    /// Open a timer handle
    ///
    /// Returns handle on success, error on failure.
    pub fn open_timer(&self, task_id: TaskId) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, TimerObject};

        let obj = Object::Timer(TimerObject::new());
        self.alloc_handle(task_id, ObjectType::Timer, obj)
    }
    // ========================================================================
    // Port Operations (Phase 3)
    // ========================================================================

    /// Open (register) a named port for listening
    ///
    /// Returns handle on success.
    pub fn open_port(&self, task_id: TaskId, name: &[u8]) -> Result<Handle, ObjError> {
        use crate::kernel::ipc;
        use crate::kernel::object::{Object, PortObject};

        if name.is_empty() || name.len() > 32 {
            return Err(ObjError::InvalidArg);
        }

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Check limit via scheduler (separate from object table access)
        let can_create = crate::kernel::task::with_scheduler(|sched| {
            sched.task(slot).map(|t| t.can_create_port()).unwrap_or(false)
        });

        if !can_create {
            return Err(ObjError::OutOfHandles);
        }

        // Register port with IPC backend (no lock held)
        let (port_id, _listen_channel) = ipc::port_register(name, task_id)
            .map_err(|e| match e {
                ipc::IpcError::PortExists { .. } => ObjError::AlreadyExists,
                _ => ObjError::OutOfMemory,
            })?;

        // Allocate handle in ObjectService tables
        let mut tables = self.tables.lock();

        let table = tables[slot].as_mut().ok_or_else(|| {
            let _ = ipc::port_unregister(port_id, task_id);
            ObjError::TaskNotFound
        })?;

        let obj = Object::Port(PortObject::new(port_id, name));
        let handle = match table.alloc(ObjectType::Port, obj) {
            Some(h) => h,
            None => {
                let _ = ipc::port_unregister(port_id, task_id);
                return Err(ObjError::OutOfHandles);
            }
        };

        // Drop tables lock before scheduler access
        drop(tables);

        // Update limit counter via scheduler
        crate::kernel::task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                task.add_port();
            }
        });

        Ok(handle)
    }

    /// Read from port (accept connection)
    ///
    /// Returns (channel_handle, client_pid) on success.
    /// Returns NeedBlock if no pending connections.
    pub fn read_port(
        &self,
        task_id: TaskId,
        handle: Handle,
    ) -> Result<(ReadAttempt, Option<(Handle, TaskId)>), ObjError> {
        use crate::kernel::ipc;
        use crate::kernel::object::{Object, ChannelObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Get port_id from object table
        let port_id = {
            let tables = self.tables.lock();
            let table = tables[slot].as_ref().ok_or(ObjError::TaskNotFound)?;
            let entry = table.get(handle).ok_or(ObjError::BadHandle)?;

            let Object::Port(ref p) = entry.object else {
                return Err(ObjError::TypeMismatch);
            };

            if !p.is_listening() {
                return Ok((ReadAttempt::Closed, None));
            }

            p.port_id()
        };

        // Accept connection from IPC backend (no lock held)
        match ipc::port_accept(port_id, task_id) {
            Ok((server_channel, client_pid)) => {
                // Allocate channel handle in ObjectService tables
                let mut tables = self.tables.lock();
                let table = tables[slot].as_mut().ok_or_else(|| {
                    let _ = ipc::close_channel(server_channel, task_id);
                    ObjError::TaskNotFound
                })?;

                let ch_obj = Object::Channel(ChannelObject::new(server_channel));
                match table.alloc(ObjectType::Channel, ch_obj) {
                    Some(ch_handle) => {
                        drop(tables);
                        // Update limit counter via scheduler
                        crate::kernel::task::with_scheduler(|sched| {
                            if let Some(task) = sched.task_mut(slot) {
                                task.add_channel();
                            }
                        });
                        Ok((ReadAttempt::Success(8), Some((ch_handle, client_pid))))
                    }
                    None => {
                        let _ = ipc::close_channel(server_channel, task_id);
                        Err(ObjError::OutOfHandles)
                    }
                }
            }
            Err(ipc::IpcError::NoPending) => Ok((ReadAttempt::NeedBlock, None)),
            Err(ipc::IpcError::Closed) => Ok((ReadAttempt::Closed, None)),
            Err(_) => Err(ObjError::BadHandle),
        }
    }

    // ========================================================================
    // Channel Operations (Phase 3)
    // ========================================================================

    /// Create a channel pair
    ///
    /// Returns (handle_a, handle_b) on success.
    pub fn open_channel_pair(&self, task_id: TaskId) -> Result<(Handle, Handle), ObjError> {
        use crate::kernel::ipc;
        use crate::kernel::object::{Object, ChannelObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Per-process limit check via scheduler
        let channel_count = crate::kernel::task::with_scheduler(|sched| {
            sched.task(slot).map(|t| t.channel_count).unwrap_or(u16::MAX)
        });
        if channel_count + 2 > crate::kernel::task::MAX_CHANNELS_PER_TASK {
            return Err(ObjError::OutOfHandles);
        }

        // Create channel pair via IPC backend (no lock held)
        let (ch_a, ch_b) = ipc::create_channel_pair(task_id, task_id)
            .map_err(|_| ObjError::OutOfMemory)?;

        // Allocate handles in ObjectService tables
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or_else(|| {
            let _ = ipc::close_channel(ch_a, task_id);
            let _ = ipc::close_channel(ch_b, task_id);
            ObjError::TaskNotFound
        })?;

        // Create and allocate handle A
        let obj_a = Object::Channel(ChannelObject::new(ch_a));
        let handle_a = table.alloc(ObjectType::Channel, obj_a)
            .ok_or_else(|| {
                let _ = ipc::close_channel(ch_a, task_id);
                let _ = ipc::close_channel(ch_b, task_id);
                ObjError::OutOfHandles
            })?;

        // Create and allocate handle B
        let obj_b = Object::Channel(ChannelObject::new(ch_b));
        let handle_b = match table.alloc(ObjectType::Channel, obj_b) {
            Some(h) => h,
            None => {
                let _ = table.close(handle_a);
                let _ = ipc::close_channel(ch_a, task_id);
                let _ = ipc::close_channel(ch_b, task_id);
                return Err(ObjError::OutOfHandles);
            }
        };

        drop(tables);

        // Track resource usage via scheduler
        crate::kernel::task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                task.add_channel();
                task.add_channel();
            }
        });

        Ok((handle_a, handle_b))
    }

    /// Connect to a named port
    ///
    /// Returns channel handle on success.
    pub fn open_channel_connect(
        &self,
        task_id: TaskId,
        port_name: &[u8],
    ) -> Result<Handle, ObjError> {
        use crate::kernel::ipc::{self, waker};
        use crate::kernel::object::{Object, ChannelObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Per-process limit check via scheduler
        let can_create = crate::kernel::task::with_scheduler(|sched| {
            sched.task(slot).map(|t| t.can_create_channel()).unwrap_or(false)
        });
        if !can_create {
            return Err(ObjError::OutOfHandles);
        }

        // Connect to port via IPC backend (no lock held)
        let (client_channel, wake_list, port_owner) = ipc::port_connect(port_name, task_id)
            .map_err(|e| match e {
                ipc::IpcError::PortNotFound => ObjError::BadHandle,
                _ => ObjError::InvalidArg,
            })?;

        // Wake the server waiting for connections (IPC backend subscribers)
        waker::wake(&wake_list, ipc::WakeReason::Accepted);

        // Also wake PortObject subscriber if the port owner is watching via Mux
        // (The PortObject.subscriber is set by read_mux_via_service)
        let port_subscriber = if let Some(owner_slot) = slot_from_task_id(port_owner) {
            let tables = self.tables.lock();
            let mut found_sub = None;
            if let Some(Some(owner_table)) = tables.get(owner_slot) {
                // Find the PortObject for this port name and get its subscriber
                for (_handle, entry) in owner_table.iter() {
                    if let Object::Port(ref port_obj) = entry.object {
                        if port_obj.name_matches(port_name) {
                            found_sub = port_obj.subscriber();
                            break;
                        }
                    }
                }
            }
            found_sub
        } else {
            None
        };
        // Wake outside the lock
        if let Some(sub) = port_subscriber {
            waker::wake_pid(sub.task_id);
        }
        // Note: no subscriber is normal if port not yet added to mux

        // Allocate handle in ObjectService tables
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or_else(|| {
            let _ = ipc::close_channel(client_channel, task_id);
            ObjError::TaskNotFound
        })?;

        let obj = Object::Channel(ChannelObject::new(client_channel));
        let handle = match table.alloc(ObjectType::Channel, obj) {
            Some(h) => h,
            None => {
                let _ = ipc::close_channel(client_channel, task_id);
                return Err(ObjError::OutOfHandles);
            }
        };

        drop(tables);

        // Track resource usage via scheduler
        crate::kernel::task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                task.add_channel();
            }
        });

        Ok(handle)
    }

    /// Read from channel (receive message)
    pub fn read_channel(
        &self,
        task_id: TaskId,
        handle: Handle,
        buf: &mut [u8],
    ) -> Result<ReadAttempt, ObjError> {
        use crate::kernel::ipc;
        use crate::kernel::object::Object;

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Get channel info from object table
        let channel_id = {
            let tables = self.tables.lock();
            let table = tables[slot].as_ref().ok_or(ObjError::TaskNotFound)?;
            let entry = table.get(handle).ok_or(ObjError::BadHandle)?;

            let Object::Channel(ref ch) = entry.object else {
                return Err(ObjError::TypeMismatch);
            };

            // Check state
            if ch.is_closed() && !ipc::channel_has_messages(ch.channel_id()) {
                return Ok(ReadAttempt::Closed);
            }

            let channel_id = ch.channel_id();
            if channel_id == 0 {
                return Ok(ReadAttempt::Closed);
            }

            channel_id
        };
        // tables lock dropped here

        // Receive from IPC backend (outside lock)
        match ipc::receive(channel_id, task_id) {
            Ok(msg) => {
                let payload = msg.payload_slice();
                let copy_len = core::cmp::min(payload.len(), buf.len());
                buf[..copy_len].copy_from_slice(&payload[..copy_len]);
                Ok(ReadAttempt::Success(copy_len))
            }
            Err(ipc::IpcError::WouldBlock) => Ok(ReadAttempt::NeedBlock),
            Err(ipc::IpcError::PeerClosed) | Err(ipc::IpcError::Closed) => {
                Ok(ReadAttempt::Closed)
            }
            Err(_) => Err(ObjError::BadHandle),
        }
    }

    /// Write to channel (send message)
    pub fn write_channel(
        &self,
        task_id: TaskId,
        handle: Handle,
        buf: &[u8],
    ) -> Result<usize, ObjError> {
        use crate::kernel::ipc::{self, waker};
        use crate::kernel::object::Object;

        if buf.len() > ipc::MAX_INLINE_PAYLOAD {
            return Err(ObjError::InvalidArg);
        }

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Get channel info from object table
        let channel_id = {
            let tables = self.tables.lock();
            let table = tables[slot].as_ref().ok_or(ObjError::TaskNotFound)?;
            let entry = table.get(handle).ok_or(ObjError::BadHandle)?;

            let Object::Channel(ref ch) = entry.object else {
                return Err(ObjError::TypeMismatch);
            };

            // Check state
            if !ch.is_open() {
                return Err(ObjError::Closed);
            }

            let channel_id = ch.channel_id();
            if channel_id == 0 {
                return Err(ObjError::Closed);
            }

            channel_id
        };
        // tables lock dropped here

        // Create and send message (outside lock)
        let msg = ipc::Message::data(task_id, buf);

        match ipc::send(channel_id, msg, task_id) {
            Ok(wake_list) => {
                waker::wake(&wake_list, ipc::WakeReason::Readable);
                Ok(buf.len())
            }
            Err(ipc::IpcError::QueueFull) => Err(ObjError::WouldBlock),
            Err(ipc::IpcError::PeerClosed) | Err(ipc::IpcError::Closed) => {
                Err(ObjError::Closed)
            }
            Err(_) => Err(ObjError::BadHandle),
        }
    }

    // ========================================================================
    // Mux Operations
    // ========================================================================

    /// Create a new Mux
    ///
    /// Returns a handle to the mux on success.
    pub fn open_mux(&self, task_id: TaskId) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, MuxObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let obj = Object::Mux(MuxObject::new());

        table.alloc(ObjectType::Mux, obj).ok_or(ObjError::OutOfHandles)
    }

    // ========================================================================
    // Process Operations
    // ========================================================================

    /// Create a Process object to watch another process
    ///
    /// The target_pid is the PID to watch. Returns a handle that can be
    /// read to wait for the target process to exit.
    pub fn open_process(&self, task_id: TaskId, target_pid: TaskId) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, ProcessObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // First, check target task state via scheduler
        let exit_code = crate::kernel::task::with_scheduler(|sched| {
            if let Some(target_slot) = sched.slot_by_pid(target_pid) {
                if let Some(target) = sched.task(target_slot) {
                    target.state().exit_code()
                } else {
                    Some(-1) // Slot exists but task is None
                }
            } else {
                // Task not found - may have already exited and been reaped
                Some(-1)
            }
        });
        // scheduler lock dropped here

        // Create ProcessObject
        let mut proc_obj = ProcessObject::new(target_pid);
        if let Some(code) = exit_code {
            proc_obj.set_exit_code(Some(code));
        }
        let obj = Object::Process(proc_obj);

        // Allocate handle in object table
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        table.alloc(ObjectType::Process, obj).ok_or(ObjError::OutOfHandles)
    }

    // ========================================================================
    // Shmem Operations
    // ========================================================================

    /// Create a new shared memory region
    ///
    /// Returns a handle to the shmem on success.
    pub fn open_shmem_create(&self, task_id: TaskId, size: usize) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, ShmemObject};
        use crate::kernel::shmem;

        if size == 0 {
            return Err(ObjError::InvalidArg);
        }

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Create shmem region (outside lock)
        let (shmem_id, vaddr, paddr) = shmem::create(task_id, size)
            .map_err(ObjError::from_errno)?;

        // Allocate handle in object table
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or_else(|| {
            let _ = shmem::destroy(shmem_id, task_id);
            ObjError::TaskNotFound
        })?;

        let obj = Object::Shmem(ShmemObject::new(shmem_id, paddr, size, vaddr));

        match table.alloc(ObjectType::Shmem, obj) {
            Some(handle) => Ok(handle),
            None => {
                let _ = shmem::destroy(shmem_id, task_id);
                Err(ObjError::OutOfHandles)
            }
        }
    }

    /// Open an existing shared memory region
    ///
    /// Returns a handle to the shmem on success.
    /// Open an existing shared memory region
    ///
    /// Returns a handle to the shmem on success.
    pub fn open_shmem_existing(&self, task_id: TaskId, shmem_id: u32) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, ShmemObject};
        use crate::kernel::shmem;

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Map existing shmem region (outside lock)
        let (vaddr, paddr) = shmem::map(task_id, shmem_id)
            .map_err(ObjError::from_errno)?;
        let size = shmem::get_size(shmem_id).unwrap_or(0);

        // Allocate handle in object table
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or_else(|| {
            let _ = shmem::unmap(task_id, shmem_id);
            ObjError::TaskNotFound
        })?;

        let obj = Object::Shmem(ShmemObject::new(shmem_id, paddr, size, vaddr));

        match table.alloc(ObjectType::Shmem, obj) {
            Some(handle) => Ok(handle),
            None => {
                let _ = shmem::unmap(task_id, shmem_id);
                Err(ObjError::OutOfHandles)
            }
        }
    }

    // ========================================================================
    // DmaPool Operations
    // ========================================================================

    /// Allocate DMA-capable memory
    ///
    /// Returns a handle to the DMA pool on success.
    pub fn open_dma_pool(
        &self,
        task_id: TaskId,
        size: usize,
        use_high: bool,
    ) -> Result<Handle, ObjError> {
        use crate::kernel::dma_pool;
        use crate::kernel::object::{Object, DmaPoolObject};

        if size == 0 {
            return Err(ObjError::InvalidArg);
        }

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Allocate from appropriate pool (outside lock)
        let paddr = if use_high {
            dma_pool::alloc_high(size).map_err(ObjError::from_errno)?
        } else {
            dma_pool::alloc(size).map_err(ObjError::from_errno)?
        };

        // Map into process (outside lock)
        let vaddr = if use_high {
            dma_pool::map_into_process_high(task_id, paddr, size)
                .map_err(ObjError::from_errno)?
        } else {
            dma_pool::map_into_process(task_id, paddr, size)
                .map_err(ObjError::from_errno)?
        };

        // Allocate handle in object table
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let obj = Object::DmaPool(DmaPoolObject::new(paddr, size, vaddr));

        table.alloc(ObjectType::DmaPool, obj).ok_or(ObjError::OutOfHandles)
    }

    // ========================================================================
    // MMIO Operations
    // ========================================================================

    /// Map MMIO region
    ///
    /// Returns a handle to the MMIO mapping on success.
    pub fn open_mmio(
        &self,
        task_id: TaskId,
        phys_addr: u64,
        size: usize,
    ) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, MmioObject};

        if size == 0 {
            return Err(ObjError::InvalidArg);
        }

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        // Map MMIO into process via scheduler
        let vaddr = crate::kernel::task::with_scheduler(|sched| {
            let task = sched.task_mut(slot).ok_or(ObjError::TaskNotFound)?;
            task.mmap_device(phys_addr, size).ok_or(ObjError::OutOfMemory)
        })?;
        // scheduler lock dropped here

        // Allocate handle in object table
        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let obj = Object::Mmio(MmioObject::new(phys_addr, size, vaddr));

        table.alloc(ObjectType::Mmio, obj).ok_or(ObjError::OutOfHandles)
    }

    // ========================================================================
    // Klog Operations
    // ========================================================================

    /// Create a kernel log reader
    ///
    /// Returns a handle to the klog reader on success.
    pub fn open_klog(&self, task_id: TaskId) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, KlogObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let obj = Object::Klog(KlogObject::new());

        table.alloc(ObjectType::Klog, obj).ok_or(ObjError::OutOfHandles)
    }

    // ========================================================================
    // PCI Bus Operations
    // ========================================================================

    /// Create a PCI bus handle for device enumeration
    pub fn open_pci_bus(&self, task_id: TaskId, bdf_filter: u32) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, PciBusObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let obj = Object::PciBus(PciBusObject::new(bdf_filter));

        table.alloc(ObjectType::PciBus, obj).ok_or(ObjError::OutOfHandles)
    }

    /// Create a PCI device handle for config access
    pub fn open_pci_device(&self, task_id: TaskId, bdf: u32) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, PciDeviceObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let obj = Object::PciDevice(PciDeviceObject::new(bdf));

        table.alloc(ObjectType::PciDevice, obj).ok_or(ObjError::OutOfHandles)
    }

    /// Allocate MSI vectors for a PCI device
    pub fn open_msi(
        &self,
        task_id: TaskId,
        bdf: u32,
        first_irq: u32,
        count: u8,
    ) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, MsiObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let obj = Object::Msi(MsiObject::new(bdf, first_irq, count));

        table.alloc(ObjectType::Msi, obj).ok_or(ObjError::OutOfHandles)
    }

    /// Create a bus list handle
    pub fn open_bus_list(&self, task_id: TaskId) -> Result<Handle, ObjError> {
        use crate::kernel::object::{Object, BusListObject};

        let slot = slot_from_task_id(task_id).ok_or(ObjError::TaskNotFound)?;

        let mut tables = self.tables.lock();
        let table = tables[slot].as_mut().ok_or(ObjError::TaskNotFound)?;
        let obj = Object::BusList(BusListObject::new());

        table.alloc(ObjectType::BusList, obj).ok_or(ObjError::OutOfHandles)
    }

    // ========================================================================
    // Timer Checking (for scheduler tick)
    // ========================================================================

    /// Check TimerObjects for a task and collect fired subscribers
    ///
    /// Called during scheduler tick to find TimerObjects that have fired.
    /// Returns subscribers that should be woken.
    pub fn check_timers_for_task(
        &self,
        task_id: TaskId,
        current_tick: u64,
        max_subscribers: usize,
    ) -> (crate::kernel::ipc::waker::WakeList, usize) {
        use crate::kernel::object::Object;

        let slot = match slot_from_task_id(task_id) {
            Some(s) => s,
            None => return (crate::kernel::ipc::waker::WakeList::new(), 0),
        };

        let mut tables = self.tables.lock();
        let table = match tables[slot].as_mut() {
            Some(t) => t,
            None => return (crate::kernel::ipc::waker::WakeList::new(), 0),
        };

        let mut wake_list = crate::kernel::ipc::waker::WakeList::new();
        let mut count = 0;

        for entry in table.entries_mut() {
            if count >= max_subscribers {
                break;
            }
            if let Object::Timer(ref mut t) = entry.object {
                if t.check(current_tick) {
                    if let Some(sub) = t.subscriber() {
                        wake_list.push(sub);
                        count += 1;
                    }
                }
            }
        }

        (wake_list, count)
    }

    // ========================================================================
    // Child Exit Notification
    // ========================================================================

    /// Notify a parent task's ProcessObject handles about a child exit
    ///
    /// This method can be called outside the scheduler lock.
    /// Returns a wake list for the caller to process.
    pub fn notify_child_exit(
        &self,
        parent_id: TaskId,
        child_pid: TaskId,
        exit_code: i32,
    ) -> crate::kernel::ipc::waker::WakeList {
        use crate::kernel::object::Object;

        let slot = match slot_from_task_id(parent_id) {
            Some(s) => s,
            None => return crate::kernel::ipc::waker::WakeList::new(),
        };

        let mut tables = self.tables.lock();
        let table = match tables[slot].as_mut() {
            Some(t) => t,
            None => return crate::kernel::ipc::waker::WakeList::new(),
        };

        // Iterate and update ProcessObjects watching this child
        let mut wake_list = crate::kernel::ipc::waker::WakeList::new();
        for entry in table.entries_mut() {
            if let Object::Process(ref mut proc_obj) = entry.object {
                if proc_obj.pid() == child_pid {
                    proc_obj.set_exit_code(Some(exit_code));
                    if let Some(sub) = proc_obj.subscriber() {
                        wake_list.push(sub);
                    }
                }
            }
        }

        wake_list
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global ObjectService instance
///
/// This is the single source of truth for all object tables.
/// All syscalls go through this service.
static OBJECT_SERVICE: ObjectService = ObjectService::new();

/// Get a reference to the global ObjectService
pub fn object_service() -> &'static ObjectService {
    &OBJECT_SERVICE
}

// ============================================================================
// Tests
// ============================================================================

// Note: Tests disabled for no_std kernel. Test ObjectService logic via
// integration tests or QEMU-based testing.
//
// The key behaviors to verify:
// - create_task_table() creates tables indexed by slot (extracted from task_id)
// - remove_task_table() removes and returns the table
// - alloc_handle() allocates in the correct task's table
// - close_handle() closes handles correctly
// - User tasks get stdin/stdout/stderr pre-allocated
