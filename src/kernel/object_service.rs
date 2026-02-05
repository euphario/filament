//! Object Service - Central Owner of Object Tables
//!
//! Implements the ObjectService which owns all per-task object tables using
//! per-task locks for deadlock-free concurrent access.
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
//! │    [SpinLock<Option<HandleTable>>; MAX_TASKS]                  │
//! │    - One lock per task slot (eliminates global lock deadlocks) │
//! │    - TableGuard proves lock is held (type-safe)                │
//! │    - Blocking read() implementation                            │
//! │    - Wakes tasks when events occur                             │
//! └────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Key Design
//!
//! - **Per-task locks** - Each task slot has its own SpinLock
//! - **TableGuard** - Type-safe proof of lock ownership
//! - **Lock ordering** - `lock_table_ordered()` prevents ABBA deadlocks
//! - **Continuation pattern** - Operations release lock before waking

use crate::kernel::lock::{SpinLock, SpinLockGuard};
use crate::kernel::task::{TaskId, MAX_TASKS};
use crate::kernel::object::{HandleTable, Handle, Object, ObjectType, ConsoleType};
use crate::kernel::object::handle::HandleEntry;
use crate::kernel::error::KernelError;

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
// TableGuard - Lock Token
// ============================================================================

/// Guard/token proving a task's table lock is held.
///
/// Dropping this guard releases the lock. The guard provides access to the
/// table and records which slot is locked for ordering verification.
pub struct TableGuard<'a> {
    /// The actual spinlock guard
    guard: SpinLockGuard<'a, Option<HandleTable>>,
    /// Which slot this guards (for ordering checks)
    slot: usize,
}

impl<'a> TableGuard<'a> {
    /// Get immutable reference to the table
    pub fn table(&self) -> Result<&HandleTable, KernelError> {
        self.guard.as_ref().ok_or(KernelError::NoProcess)
    }

    /// Get mutable reference to the table
    pub fn table_mut(&mut self) -> Result<&mut HandleTable, KernelError> {
        self.guard.as_mut().ok_or(KernelError::NoProcess)
    }

    /// Get the slot index this guard protects
    pub fn slot(&self) -> usize {
        self.slot
    }

    /// Check if this guard holds a valid table
    pub fn has_table(&self) -> bool {
        self.guard.is_some()
    }

    /// Take the table out of the guard (for remove_task_table)
    pub fn take(&mut self) -> Option<HandleTable> {
        self.guard.take()
    }

    /// Put a table into the guard (for create_task_table)
    pub fn put(&mut self, table: HandleTable) {
        *self.guard = Some(table);
    }
}

// ============================================================================
// ObjectService
// ============================================================================

/// Extract slot index from TaskId
///
/// PID format: bits[8:0] = slot + 1 (1-256), bits[31:9] = generation
/// Returns None if invalid (slot 0 is reserved for idle)
fn slot_from_task_id(task_id: TaskId) -> Option<usize> {
    let slot_bits = (task_id & 0x1FF) as usize;
    if slot_bits == 0 || slot_bits > MAX_TASKS {
        return None;
    }
    Some(slot_bits - 1)
}

/// Central service that owns all object tables.
///
/// Uses per-task locks for deadlock-free concurrent access.
/// Each task slot has its own SpinLock, eliminating the class of
/// same-lock re-acquisition deadlocks.
pub struct ObjectService {
    /// Per-slot object tables with individual locks
    tables: [SpinLock<Option<HandleTable>>; MAX_TASKS],
}

impl ObjectService {
    /// Create a new ObjectService with per-task locks
    pub const fn new() -> Self {
        const LOCKED_NONE: SpinLock<Option<HandleTable>> = SpinLock::new(None);
        Self {
            tables: [LOCKED_NONE; MAX_TASKS],
        }
    }

    // ========================================================================
    // Lock Acquisition
    // ========================================================================

    /// Acquire lock on a single task's table.
    ///
    /// Returns a TableGuard that proves the lock is held.
    pub fn lock_table(&self, task_id: TaskId) -> Result<TableGuard<'_>, KernelError> {
        let slot = slot_from_task_id(task_id).ok_or(KernelError::NoProcess)?;
        let guard = self.tables[slot].lock();
        Ok(TableGuard { guard, slot })
    }

    /// Lock a table by slot index directly (internal use)
    fn lock_slot(&self, slot: usize) -> Result<TableGuard<'_>, KernelError> {
        if slot >= MAX_TASKS {
            return Err(KernelError::NoProcess);
        }
        let guard = self.tables[slot].lock();
        Ok(TableGuard { guard, slot })
    }

    /// Acquire lock on a second table, enforcing slot ordering.
    ///
    /// The new slot must be strictly greater than the held slot.
    /// This prevents ABBA deadlocks when multiple tables need to be locked.
    ///
    /// # Errors
    /// Returns `KernelError::InvalidArg` if trying to lock a lower or equal slot.
    pub fn lock_table_ordered(
        &self,
        held: &TableGuard<'_>,
        task_id: TaskId,
    ) -> Result<TableGuard<'_>, KernelError> {
        let slot = slot_from_task_id(task_id).ok_or(KernelError::NoProcess)?;
        if slot <= held.slot {
            // Lock order violation - would cause ABBA deadlock
            return Err(KernelError::InvalidArg);
        }
        let guard = self.tables[slot].lock();
        Ok(TableGuard { guard, slot })
    }

    /// Try to lock a table without blocking.
    ///
    /// Returns Ok(None) if lock is already held by another CPU.
    pub fn try_lock_table(&self, task_id: TaskId) -> Result<Option<TableGuard<'_>>, KernelError> {
        let slot = slot_from_task_id(task_id).ok_or(KernelError::NoProcess)?;
        match self.tables[slot].try_lock() {
            Some(guard) => Ok(Some(TableGuard { guard, slot })),
            None => Ok(None),
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
        if let Ok(mut guard) = self.lock_table(task_id) {
            let table = if is_user {
                HandleTable::new_with_stdio()
            } else {
                HandleTable::new()
            };
            guard.put(table);
        }
    }

    /// Remove object table for an exiting task
    ///
    /// Called when a task exits. Closes all handles and removes the table.
    pub fn remove_task_table(&self, task_id: TaskId) -> Option<HandleTable> {
        self.lock_table(task_id).ok()?.take()
    }

    /// Check if a task has an object table
    pub fn task_exists(&self, task_id: TaskId) -> bool {
        self.lock_table(task_id)
            .map(|g| g.has_table())
            .unwrap_or(false)
    }

    // ========================================================================
    // Handle Operations (with guards)
    // ========================================================================

    /// Allocate handle in a locked table
    pub fn alloc_handle_with(
        guard: &mut TableGuard<'_>,
        object_type: ObjectType,
        object: Object,
    ) -> Result<Handle, KernelError> {
        let table = guard.table_mut()?;
        table.alloc(object_type, object).ok_or(KernelError::OutOfHandles)
    }

    /// Close handle in a locked table
    pub fn close_handle_with(
        guard: &mut TableGuard<'_>,
        handle: Handle,
    ) -> Result<Object, KernelError> {
        let table = guard.table_mut()?;
        table.close(handle).ok_or(KernelError::BadHandle)
    }

    /// Get entry from a locked table
    pub fn get_entry_with<'b>(
        guard: &'b TableGuard<'b>,
        handle: Handle,
    ) -> Result<&'b HandleEntry, KernelError> {
        let table = guard.table()?;
        table.get(handle).ok_or(KernelError::BadHandle)
    }

    /// Get mutable entry from a locked table
    pub fn get_entry_mut_with<'b>(
        guard: &'b mut TableGuard<'b>,
        handle: Handle,
    ) -> Result<&'b mut HandleEntry, KernelError> {
        let table = guard.table_mut()?;
        table.get_mut(handle).ok_or(KernelError::BadHandle)
    }

    // ========================================================================
    // Convenience Methods (lock + operate + unlock)
    // ========================================================================

    /// Allocate a new handle in a task's object table
    pub fn alloc_handle(
        &self,
        task_id: TaskId,
        object_type: ObjectType,
        object: Object,
    ) -> Result<Handle, KernelError> {
        let mut guard = self.lock_table(task_id)?;
        Self::alloc_handle_with(&mut guard, object_type, object)
    }

    /// Get a handle entry (immutable)
    pub fn get_entry<F, R>(&self, task_id: TaskId, handle: Handle, f: F) -> Result<R, KernelError>
    where
        F: FnOnce(&HandleEntry) -> R,
    {
        let guard = self.lock_table(task_id)?;
        let table = guard.table()?;
        let entry = table.get(handle).ok_or(KernelError::BadHandle)?;
        Ok(f(entry))
    }

    /// Get a handle entry (mutable)
    pub fn get_entry_mut<F, R>(&self, task_id: TaskId, handle: Handle, f: F) -> Result<R, KernelError>
    where
        F: FnOnce(&mut HandleEntry) -> R,
    {
        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let entry = table.get_mut(handle).ok_or(KernelError::BadHandle)?;
        Ok(f(entry))
    }

    /// Close a handle
    pub fn close_handle(&self, task_id: TaskId, handle: Handle) -> Result<Object, KernelError> {
        let mut guard = self.lock_table(task_id)?;
        Self::close_handle_with(&mut guard, handle)
    }

    /// Access the full table for a task (for complex operations like Mux polling)
    pub fn with_table<F, R>(&self, task_id: TaskId, f: F) -> Result<R, KernelError>
    where
        F: FnOnce(&HandleTable) -> R,
    {
        let guard = self.lock_table(task_id)?;
        let table = guard.table()?;
        Ok(f(table))
    }

    /// Access the full table mutably for a task
    pub fn with_table_mut<F, R>(&self, task_id: TaskId, f: F) -> Result<R, KernelError>
    where
        F: FnOnce(&mut HandleTable) -> R,
    {
        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        Ok(f(table))
    }

    // ========================================================================
    // Console Operations
    // ========================================================================

    /// Open a console handle (stdin/stdout/stderr)
    pub fn open_console(&self, task_id: TaskId, console_type: ConsoleType) -> Result<Handle, KernelError> {
        use crate::kernel::object::ConsoleObject;

        let obj = Object::Console(ConsoleObject::new(console_type));
        let obj_type = match console_type {
            ConsoleType::Stdin => ObjectType::Stdin,
            ConsoleType::Stdout => ObjectType::Stdout,
            ConsoleType::Stderr => ObjectType::Stderr,
        };

        self.alloc_handle(task_id, obj_type, obj)
    }

    // ========================================================================
    // Timer Operations
    // ========================================================================

    /// Open a timer handle
    pub fn open_timer(&self, task_id: TaskId) -> Result<Handle, KernelError> {
        use crate::kernel::object::TimerObject;

        let obj = Object::Timer(TimerObject::new());
        self.alloc_handle(task_id, ObjectType::Timer, obj)
    }

    // ========================================================================
    // Port Operations
    // ========================================================================

    /// Open (register) a named port for listening
    pub fn open_port(&self, task_id: TaskId, name: &[u8]) -> Result<Handle, KernelError> {
        use crate::kernel::ipc;
        use crate::kernel::object::PortObject;

        if name.is_empty() || name.len() > 32 {
            return Err(KernelError::InvalidArg);
        }

        let slot = slot_from_task_id(task_id).ok_or(KernelError::NoProcess)?;

        // Check limit via scheduler (separate from object table access)
        let can_create = crate::kernel::task::with_scheduler(|sched| {
            sched.task(slot).map(|t| t.can_create_port()).unwrap_or(false)
        });

        if !can_create {
            return Err(KernelError::OutOfHandles);
        }

        // Register port with IPC backend (no lock held)
        let (port_id, _listen_channel) = ipc::port_register(name, task_id)
            .map_err(|e| match e {
                ipc::IpcError::PortExists { .. } => KernelError::Exists,
                _ => KernelError::OutOfMemory,
            })?;

        // Allocate handle in object table
        let mut guard = self.lock_table(task_id)?;
        let table = match guard.table_mut() {
            Ok(t) => t,
            Err(_) => {
                let _ = ipc::port_unregister(port_id, task_id);
                return Err(KernelError::NoProcess);
            }
        };

        let obj = Object::Port(PortObject::new(port_id, name));
        let handle = match table.alloc(ObjectType::Port, obj) {
            Some(h) => h,
            None => {
                let _ = ipc::port_unregister(port_id, task_id);
                return Err(KernelError::OutOfHandles);
            }
        };

        // Drop table lock before scheduler access
        drop(guard);

        // Update limit counter via scheduler
        crate::kernel::task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                task.add_port();
            }
        });

        Ok(handle)
    }

    /// Read from port (accept connection)
    pub fn read_port(
        &self,
        task_id: TaskId,
        handle: Handle,
    ) -> Result<(ReadAttempt, Option<(Handle, TaskId)>), KernelError> {
        use crate::kernel::ipc;
        use crate::kernel::object::ChannelObject;

        let slot = slot_from_task_id(task_id).ok_or(KernelError::NoProcess)?;

        // Get port_id from object table
        let port_id = {
            let guard = self.lock_table(task_id)?;
            let table = guard.table()?;
            let entry = table.get(handle).ok_or(KernelError::BadHandle)?;

            let Object::Port(ref p) = entry.object else {
                return Err(KernelError::InvalidArg);
            };

            if !p.is_listening() {
                return Ok((ReadAttempt::Closed, None));
            }

            p.port_id()
        };
        // guard dropped here

        // Accept connection from IPC backend (no lock held)
        match ipc::port_accept(port_id, task_id) {
            Ok((server_channel, client_pid)) => {
                // Allocate channel handle
                let mut guard = self.lock_table(task_id)?;
                let table = match guard.table_mut() {
                    Ok(t) => t,
                    Err(_) => {
                        let _ = ipc::close_channel(server_channel, task_id);
                        return Err(KernelError::NoProcess);
                    }
                };

                let ch_obj = Object::Channel(ChannelObject::new(server_channel));
                match table.alloc(ObjectType::Channel, ch_obj) {
                    Some(ch_handle) => {
                        drop(guard);
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
                        Err(KernelError::OutOfHandles)
                    }
                }
            }
            Err(ipc::IpcError::NoPending) => Ok((ReadAttempt::NeedBlock, None)),
            Err(ipc::IpcError::Closed) => Ok((ReadAttempt::Closed, None)),
            Err(_) => Err(KernelError::BadHandle),
        }
    }

    // ========================================================================
    // Channel Operations
    // ========================================================================

    /// Create a channel pair
    pub fn open_channel_pair(&self, task_id: TaskId) -> Result<(Handle, Handle), KernelError> {
        use crate::kernel::ipc;
        use crate::kernel::object::ChannelObject;

        let slot = slot_from_task_id(task_id).ok_or(KernelError::NoProcess)?;

        // Per-process limit check via scheduler
        let channel_count = crate::kernel::task::with_scheduler(|sched| {
            sched.task(slot).map(|t| t.channel_count).unwrap_or(u16::MAX)
        });
        if channel_count + 2 > crate::kernel::task::MAX_CHANNELS_PER_TASK {
            return Err(KernelError::OutOfHandles);
        }

        // Create channel pair via IPC backend (no lock held)
        let (ch_a, ch_b) = ipc::create_channel_pair(task_id, task_id)
            .map_err(|_| KernelError::OutOfMemory)?;

        // Allocate handles
        let mut guard = self.lock_table(task_id)?;
        let table = match guard.table_mut() {
            Ok(t) => t,
            Err(_) => {
                let _ = ipc::close_channel(ch_a, task_id);
                let _ = ipc::close_channel(ch_b, task_id);
                return Err(KernelError::NoProcess);
            }
        };

        // Create and allocate handle A
        let obj_a = Object::Channel(ChannelObject::new(ch_a));
        let handle_a = match table.alloc(ObjectType::Channel, obj_a) {
            Some(h) => h,
            None => {
                let _ = ipc::close_channel(ch_a, task_id);
                let _ = ipc::close_channel(ch_b, task_id);
                return Err(KernelError::OutOfHandles);
            }
        };

        // Create and allocate handle B
        let obj_b = Object::Channel(ChannelObject::new(ch_b));
        let handle_b = match table.alloc(ObjectType::Channel, obj_b) {
            Some(h) => h,
            None => {
                let _ = table.close(handle_a);
                let _ = ipc::close_channel(ch_a, task_id);
                let _ = ipc::close_channel(ch_b, task_id);
                return Err(KernelError::OutOfHandles);
            }
        };

        drop(guard);

        // Track resource usage via scheduler
        crate::kernel::task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                task.add_channel();
                task.add_channel();
            }
        });

        Ok((handle_a, handle_b))
    }

    /// Connect to a named port (accesses two tables safely with per-task locks)
    pub fn open_channel_connect(
        &self,
        task_id: TaskId,
        port_name: &[u8],
    ) -> Result<Handle, KernelError> {
        use crate::kernel::ipc::{self, waker};
        use crate::kernel::object::ChannelObject;

        let slot = slot_from_task_id(task_id).ok_or(KernelError::NoProcess)?;

        // Per-process limit check via scheduler
        let can_create = crate::kernel::task::with_scheduler(|sched| {
            sched.task(slot).map(|t| t.can_create_channel()).unwrap_or(false)
        });
        if !can_create {
            return Err(KernelError::OutOfHandles);
        }

        // Connect to port via IPC backend (no lock held)
        let (client_channel, wake_list, port_owner) = ipc::port_connect(port_name, task_id)
            .map_err(|e| match e {
                ipc::IpcError::PortNotFound => KernelError::BadHandle,
                _ => KernelError::InvalidArg,
            })?;

        // Wake the server waiting for connections (IPC backend subscribers)
        waker::wake(&wake_list, ipc::WakeReason::Accepted);

        // Get port subscriber from owner's table (if owner != caller)
        let owner_slot = slot_from_task_id(port_owner);
        let port_subscriber = match owner_slot {
            Some(os) if os != slot => {
                // Different task - lock owner's table
                let owner_guard = self.lock_slot(os)?;
                let mut found_sub = None;
                if let Ok(table) = owner_guard.table() {
                    for (_h, entry) in table.iter() {
                        if let Object::Port(ref p) = entry.object {
                            if p.name_matches(port_name) {
                                found_sub = p.subscriber();
                                break;
                            }
                        }
                    }
                }
                found_sub
                // owner_guard dropped here
            }
            _ => None,
        };

        // Wake subscriber (no locks held)
        if let Some(sub) = port_subscriber {
            waker::wake_pid(sub.task_id);
        }

        // Allocate handle in caller's table
        let mut caller_guard = self.lock_table(task_id)?;
        let table = match caller_guard.table_mut() {
            Ok(t) => t,
            Err(_) => {
                let _ = ipc::close_channel(client_channel, task_id);
                return Err(KernelError::NoProcess);
            }
        };

        let obj = Object::Channel(ChannelObject::new(client_channel));
        let handle = match table.alloc(ObjectType::Channel, obj) {
            Some(h) => h,
            None => {
                let _ = ipc::close_channel(client_channel, task_id);
                return Err(KernelError::OutOfHandles);
            }
        };

        drop(caller_guard);

        // Track resource usage via scheduler
        crate::kernel::task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                task.add_channel();
            }
        });

        Ok(handle)
    }

    // ========================================================================
    // Mux Operations
    // ========================================================================

    /// Create a new Mux
    pub fn open_mux(&self, task_id: TaskId) -> Result<Handle, KernelError> {
        use crate::kernel::object::MuxObject;

        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let obj = Object::Mux(MuxObject::new());

        table.alloc(ObjectType::Mux, obj).ok_or(KernelError::OutOfHandles)
    }

    // ========================================================================
    // Process Operations
    // ========================================================================

    /// Create a Process object to watch another process
    pub fn open_process(&self, task_id: TaskId, target_pid: TaskId) -> Result<Handle, KernelError> {
        use crate::kernel::object::ProcessObject;

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
        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        table.alloc(ObjectType::Process, obj).ok_or(KernelError::OutOfHandles)
    }

    // ========================================================================
    // Shmem Operations
    // ========================================================================

    /// Create a new shared memory region
    pub fn open_shmem_create(&self, task_id: TaskId, size: usize) -> Result<(Handle, u32), KernelError> {
        use crate::kernel::object::ShmemObject;
        use crate::kernel::shmem;

        if size == 0 {
            return Err(KernelError::InvalidArg);
        }

        // Create shmem region (outside lock)
        let (shmem_id, vaddr, paddr) = shmem::create(task_id, size)
            .map_err(KernelError::from_errno)?;

        // Allocate handle in object table
        let mut guard = self.lock_table(task_id)?;
        let table = match guard.table_mut() {
            Ok(t) => t,
            Err(_) => {
                let _ = shmem::destroy(shmem_id, task_id);
                return Err(KernelError::NoProcess);
            }
        };

        let obj = Object::Shmem(ShmemObject::new(shmem_id, paddr, size, vaddr));

        match table.alloc(ObjectType::Shmem, obj) {
            Some(handle) => Ok((handle, shmem_id)),
            None => {
                let _ = shmem::destroy(shmem_id, task_id);
                Err(KernelError::OutOfHandles)
            }
        }
    }

    /// Open an existing shared memory region
    pub fn open_shmem_existing(&self, task_id: TaskId, shmem_id: u32) -> Result<Handle, KernelError> {
        use crate::kernel::object::ShmemObject;
        use crate::kernel::shmem;

        // Map existing shmem region (outside lock)
        let (vaddr, paddr) = shmem::map(task_id, shmem_id)
            .map_err(KernelError::from_errno)?;
        let size = match shmem::get_size(shmem_id) {
            Some(s) => s,
            None => {
                let _ = shmem::unmap(task_id, shmem_id);
                return Err(KernelError::NotFound);
            }
        };

        // Allocate handle in object table
        let mut guard = self.lock_table(task_id)?;
        let table = match guard.table_mut() {
            Ok(t) => t,
            Err(_) => {
                let _ = shmem::unmap(task_id, shmem_id);
                return Err(KernelError::NoProcess);
            }
        };

        let obj = Object::Shmem(ShmemObject::new(shmem_id, paddr, size, vaddr));

        match table.alloc(ObjectType::Shmem, obj) {
            Some(handle) => Ok(handle),
            None => {
                let _ = shmem::unmap(task_id, shmem_id);
                Err(KernelError::OutOfHandles)
            }
        }
    }

    // ========================================================================
    // DmaPool Operations
    // ========================================================================

    /// Allocate DMA-capable memory
    ///
    /// flags bit 0: use_high - allocate from high memory pool (>4GB)
    /// flags bit 1: streaming - use cacheable memory (requires explicit cache sync)
    pub fn open_dma_pool(
        &self,
        task_id: TaskId,
        size: usize,
        use_high: bool,
        streaming: bool,
    ) -> Result<Handle, KernelError> {
        use crate::kernel::dma_pool;
        use crate::kernel::object::DmaPoolObject;

        if size == 0 {
            return Err(KernelError::InvalidArg);
        }

        // Allocate from appropriate pool (outside lock)
        let paddr = if use_high {
            dma_pool::alloc_high(size).map_err(KernelError::from_errno)?
        } else {
            dma_pool::alloc(size).map_err(KernelError::from_errno)?
        };

        // Map into process (outside lock)
        // Choose mapping mode based on streaming flag
        let vaddr = if use_high {
            // High memory pool doesn't support streaming mode yet
            dma_pool::map_into_process_high(task_id, paddr, size)
                .map_err(KernelError::from_errno)?
        } else if streaming {
            // Streaming: cacheable memory, requires explicit cache sync
            dma_pool::map_into_process_streaming(task_id, paddr, size)
                .map_err(KernelError::from_errno)?
        } else {
            // Coherent: non-cacheable memory, no cache sync needed
            dma_pool::map_into_process(task_id, paddr, size)
                .map_err(KernelError::from_errno)?
        };

        // Allocate handle in object table
        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let obj = Object::DmaPool(DmaPoolObject::new(paddr, size, vaddr));

        table.alloc(ObjectType::DmaPool, obj).ok_or(KernelError::OutOfHandles)
    }

    // ========================================================================
    // MMIO Operations
    // ========================================================================

    /// Map MMIO region
    pub fn open_mmio(
        &self,
        task_id: TaskId,
        phys_addr: u64,
        size: usize,
    ) -> Result<Handle, KernelError> {
        use crate::kernel::object::MmioObject;

        if size == 0 {
            return Err(KernelError::InvalidArg);
        }

        let slot = slot_from_task_id(task_id).ok_or(KernelError::NoProcess)?;

        // Map MMIO into process via scheduler
        let vaddr = crate::kernel::task::with_scheduler(|sched| {
            let task = sched.task_mut(slot).ok_or(KernelError::NoProcess)?;
            task.mmap_device(phys_addr, size).ok_or(KernelError::OutOfMemory)
        })?;
        // scheduler lock dropped here

        // Allocate handle in object table
        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let obj = Object::Mmio(MmioObject::new(phys_addr, size, vaddr));

        table.alloc(ObjectType::Mmio, obj).ok_or(KernelError::OutOfHandles)
    }

    // ========================================================================
    // Ring Operations (high-performance IPC)
    // ========================================================================

    /// Create a new Ring IPC object
    pub fn open_ring(
        &self,
        task_id: TaskId,
        ring: crate::kernel::object::RingObject,
        shmem_id: u32,
    ) -> Result<Handle, KernelError> {
        // Allocate handle in object table
        let mut guard = self.lock_table(task_id)?;
        let table = match guard.table_mut() {
            Ok(t) => t,
            Err(_) => {
                // Clean up shmem on failure
                let _ = crate::kernel::shmem::destroy(shmem_id, task_id);
                return Err(KernelError::NoProcess);
            }
        };

        let obj = Object::Ring(ring);

        match table.alloc(ObjectType::Ring, obj) {
            Some(handle) => Ok(handle),
            None => {
                let _ = crate::kernel::shmem::destroy(shmem_id, task_id);
                Err(KernelError::OutOfHandles)
            }
        }
    }

    // ========================================================================
    // Klog Operations
    // ========================================================================

    /// Create a kernel log reader
    pub fn open_klog(&self, task_id: TaskId) -> Result<Handle, KernelError> {
        use crate::kernel::object::KlogObject;

        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let obj = Object::Klog(KlogObject::new());

        table.alloc(ObjectType::Klog, obj).ok_or(KernelError::OutOfHandles)
    }

    // ========================================================================
    // PCI Bus Operations
    // ========================================================================

    /// Create a PCI bus handle for device enumeration
    pub fn open_pci_bus(&self, task_id: TaskId, bdf_filter: u32) -> Result<Handle, KernelError> {
        use crate::kernel::object::PciBusObject;

        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let obj = Object::PciBus(PciBusObject::new(bdf_filter));

        table.alloc(ObjectType::PciBus, obj).ok_or(KernelError::OutOfHandles)
    }

    /// Create a PCI device handle for config access
    pub fn open_pci_device(&self, task_id: TaskId, bdf: u32) -> Result<Handle, KernelError> {
        use crate::kernel::object::PciDeviceObject;

        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let obj = Object::PciDevice(PciDeviceObject::new(bdf));

        table.alloc(ObjectType::PciDevice, obj).ok_or(KernelError::OutOfHandles)
    }

    /// Allocate MSI vectors for a PCI device
    pub fn open_msi(
        &self,
        task_id: TaskId,
        bdf: u32,
        first_irq: u32,
        count: u8,
    ) -> Result<Handle, KernelError> {
        use crate::kernel::object::MsiObject;

        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let obj = Object::Msi(MsiObject::new(bdf, first_irq, count));

        table.alloc(ObjectType::Msi, obj).ok_or(KernelError::OutOfHandles)
    }

    /// Create a bus list handle
    pub fn open_bus_list(&self, task_id: TaskId) -> Result<Handle, KernelError> {
        use crate::kernel::object::BusListObject;

        let mut guard = self.lock_table(task_id)?;
        let table = guard.table_mut()?;
        let obj = Object::BusList(BusListObject::new());

        table.alloc(ObjectType::BusList, obj).ok_or(KernelError::OutOfHandles)
    }

    // ========================================================================
    // Timer Checking (for scheduler tick)
    // ========================================================================

    /// Check TimerObjects for a task and collect fired subscribers
    pub fn check_timers_for_task(
        &self,
        task_id: TaskId,
        current_tick: u64,
        max_subscribers: usize,
    ) -> (crate::kernel::ipc::waker::WakeList, usize) {
        let slot = match slot_from_task_id(task_id) {
            Some(s) => s,
            None => return (crate::kernel::ipc::waker::WakeList::new(), 0),
        };

        let mut guard = match self.lock_slot(slot) {
            Ok(g) => g,
            Err(_) => return (crate::kernel::ipc::waker::WakeList::new(), 0),
        };

        let table = match guard.table_mut() {
            Ok(t) => t,
            Err(_) => return (crate::kernel::ipc::waker::WakeList::new(), 0),
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
    pub fn notify_child_exit(
        &self,
        parent_id: TaskId,
        child_pid: TaskId,
        exit_code: i32,
    ) -> crate::kernel::ipc::waker::WakeList {
        let slot = match slot_from_task_id(parent_id) {
            Some(s) => s,
            None => return crate::kernel::ipc::waker::WakeList::new(),
        };

        let mut guard = match self.lock_slot(slot) {
            Ok(g) => g,
            Err(_) => return crate::kernel::ipc::waker::WakeList::new(),
        };

        let table = match guard.table_mut() {
            Ok(t) => t,
            Err(_) => return crate::kernel::ipc::waker::WakeList::new(),
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

    // ========================================================================
    // Channel Wake (for unified subscriber model)
    // ========================================================================

    /// Wake subscribers on a ChannelObject's WaitQueue
    ///
    /// Called after ipc::send() or ipc::close_channel() returns PeerInfo.
    /// Scans the peer task's handle table for a ChannelObject matching
    /// the given channel_id, then fires its WaitQueue with the event.
    ///
    /// Safe to call even if the peer has already closed — the scan simply
    /// finds no matching ChannelObject and returns an empty WakeList.
    pub fn wake_channel(
        &self,
        task_id: TaskId,
        channel_id: u32,
        event: u8,
    ) -> crate::kernel::ipc::waker::WakeList {
        let slot = match slot_from_task_id(task_id) {
            Some(s) => s,
            None => return crate::kernel::ipc::waker::WakeList::new(),
        };

        let mut guard = match self.lock_slot(slot) {
            Ok(g) => g,
            Err(_) => return crate::kernel::ipc::waker::WakeList::new(),
        };

        let table = match guard.table_mut() {
            Ok(t) => t,
            Err(_) => return crate::kernel::ipc::waker::WakeList::new(),
        };

        for entry in table.entries_mut() {
            if let Object::Channel(ref mut ch) = entry.object {
                if ch.channel_id() == channel_id {
                    return ch.wait_queue_mut().wake(event);
                }
            }
        }

        crate::kernel::ipc::waker::WakeList::new()
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global ObjectService instance
static OBJECT_SERVICE: ObjectService = ObjectService::new();

/// Get a reference to the global ObjectService
pub fn object_service() -> &'static ObjectService {
    &OBJECT_SERVICE
}
