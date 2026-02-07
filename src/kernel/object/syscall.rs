//! The Five Syscalls
//!
//! Everything goes through these five entry points.
//!
//! # Architecture
//!
//! This module implements the unified 5-syscall interface that is the heart
//! of the microkernel. All scheduler access goes through `task::with_scheduler`
//! which provides safe access to the scheduler with proper locking.
//!
//! # The 5 Syscalls
//!
//! | Syscall | Purpose |
//! |---------|---------|
//! | open | Create object, return handle |
//! | read | Read from object |
//! | write | Write to object |
//! | map | Map object to memory |
//! | close | Release handle |

use super::{Object, ObjectType, Handle, ConsoleType, Pollable};
use crate::kernel::error::KernelError;
use crate::kernel::task;
use crate::kernel::uaccess;
use crate::platform::current::uart;
use crate::kernel::ipc;
use crate::kernel::ipc::waker;
use crate::kernel::shmem;
use crate::kernel::object_service::object_service;
// ============================================================================
// Helpers
// ============================================================================

/// Get current task's ID. Returns None if no task in current slot.
fn get_current_task_id() -> Option<u32> {
    task::with_scheduler(|sched| {
        let slot = task::current_slot();
        sched.task(slot).map(|t| t.id)
    })
}

// ============================================================================
// open - Create a new handle
// ============================================================================

/// Open a new object
///
/// type_id: ObjectType discriminant
/// params_ptr: Type-specific parameters
/// params_len: Length of params
///
/// Returns: handle (positive) or error (negative)
pub fn open(type_id: u32, params_ptr: u64, params_len: usize) -> i64 {
    let Some(obj_type) = ObjectType::from_u32(type_id) else {
        return KernelError::InvalidArg.to_errno();
    };

    // Dispatch to type-specific open
    match obj_type {
        ObjectType::Channel => open_channel(params_ptr, params_len),
        ObjectType::Timer => open_timer(params_ptr, params_len),
        ObjectType::Process => open_process(params_ptr, params_len),
        ObjectType::Port => open_port(params_ptr, params_len),
        ObjectType::Shmem => open_shmem(params_ptr, params_len),
        ObjectType::DmaPool => open_dma_pool(params_ptr, params_len),
        ObjectType::Mmio => open_mmio(params_ptr, params_len),
        ObjectType::Stdin => open_console(super::ConsoleType::Stdin),
        ObjectType::Stdout => open_console(super::ConsoleType::Stdout),
        ObjectType::Stderr => open_console(super::ConsoleType::Stderr),
        ObjectType::Klog => open_klog(params_ptr, params_len),
        ObjectType::Mux => open_mux(params_ptr, params_len),
        ObjectType::PciBus => open_pci_bus(params_ptr, params_len),
        ObjectType::PciDevice => open_pci_device(params_ptr, params_len),
        ObjectType::Msi => open_msi(params_ptr, params_len),
        ObjectType::BusList => open_bus_list(params_ptr, params_len),
        ObjectType::Ring => open_ring(params_ptr, params_len),
        ObjectType::Bus => open_bus_create(params_ptr, params_len),
    }
}

// ============================================================================
// read - Read from handle
// ============================================================================

/// Read from a handle
///
/// For channels: receive message
/// For timer: wait until tick, return timestamp
/// For process: wait until exit, return exit code
/// For port: accept connection, return new handle
/// For mux: wait for event, return (handle, event_type)
///
/// Returns: bytes read (positive) or error (negative)
pub fn read(handle_raw: u32, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::kernel::object_service::object_service;

    let handle = Handle::from_raw(handle_raw);

    // Get current task ID
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Use ObjectService's tables for dispatch
    let result = object_service().with_table_mut(task_id, |table| {
        // Check handle validity and get type
        let Some(entry) = table.get(handle) else {
            return Err(KernelError::BadHandle);
        };
        if !entry.object_type.is_readable() {
            return Err(KernelError::NotSupported);
        }
        Ok(entry.object_type)
    });

    let obj_type = match result {
        Ok(Ok(t)) => t,
        Ok(Err(e)) => return e.to_errno(),
        Err(_) => return KernelError::BadHandle.to_errno(),
    };

    // Handle Mux specially - needs access to multiple handles
    if obj_type == ObjectType::Mux {
        return read_mux_via_service(task_id, handle, buf_ptr, buf_len);
    }

    // Handle Port specially - needs to allocate a new handle (can't nest locks)
    if obj_type == ObjectType::Port {
        return read_port_via_service(task_id, handle, buf_ptr, buf_len);
    }

    // NOTE: Channel read is non-blocking (returns WouldBlock if no data).
    // Use Mux to wait for data, or call wait_one() then recv().
    // A proper blocking recv API should be added via a separate syscall flag.

    // For other types, get mutable ref and dispatch
    let result = object_service().with_table_mut(task_id, |table| {
        let Some(entry) = table.get_mut(handle) else {
            return KernelError::BadHandle.to_errno();
        };

        // Dispatch to type-specific read
        match &mut entry.object {
            Object::Channel(ch) => read_channel(ch, buf_ptr, buf_len),
            Object::Timer(t) => read_timer(t, buf_ptr, buf_len, task_id),
            Object::Process(p) => read_process(p, buf_ptr, buf_len, task_id),
            Object::Port(_) => unreachable!(), // Handled above
            Object::Shmem(s) => read_shmem(s, buf_ptr, buf_len, task_id),
            Object::Console(c) => read_console(c, buf_ptr, buf_len, task_id),
            Object::Klog(k) => read_klog(k, buf_ptr, buf_len),
            Object::Mux(_) => unreachable!(), // Handled above
            Object::PciBus(p) => read_pci_bus(p, buf_ptr, buf_len),
            Object::PciDevice(d) => read_pci_device(d, buf_ptr, buf_len, task_id),
            Object::Msi(m) => read_msi(m, buf_ptr, buf_len),
            Object::BusList(b) => read_bus_list(b, buf_ptr, buf_len),
            Object::Ring(r) => read_ring(r, buf_ptr, buf_len, task_id),
            Object::DmaPool(d) => read_dma_pool(d, buf_ptr, buf_len),
            _ => KernelError::NotSupported.to_errno(),
        }
    });

    match result {
        Ok(errno) => errno,
        Err(_) => KernelError::BadHandle.to_errno(),
    }
}

// ============================================================================
// sys_write - Write to handle
// ============================================================================

/// Write to a handle
///
/// For channels: send message
/// For timer: set deadline (buf = u64 nanoseconds)
/// For mux: add/remove watch
///
/// Returns: bytes written (positive) or error (negative)
pub fn write(handle_raw: u32, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::kernel::object_service::object_service;

    let handle = Handle::from_raw(handle_raw);

    // Get current task ID
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Pending actions to perform after table lock is released.
    // Channel wake and shmem notify both need to acquire per-task table locks,
    // so they must not run while the caller's table lock is held (deadlock if peer == self).
    enum PendingAction {
        None,
        ShmemNotify(u32, u32), // (shmem_id, caller_pid)
        ChannelWake(ChannelWriteResult),
    }

    // Use ObjectService's tables for dispatch
    let result = object_service().with_table_mut(task_id, |table| {
        let Some(entry) = table.get_mut(handle) else {
            return (KernelError::BadHandle.to_errno(), PendingAction::None);
        };

        if !entry.object_type.is_writable() {
            return (KernelError::NotSupported.to_errno(), PendingAction::None);
        }

        // Dispatch to type-specific write
        match &mut entry.object {
            Object::Channel(ch) => {
                let r = write_channel(ch, buf_ptr, buf_len);
                let errno = r.errno;
                if r.peer.is_some() {
                    (errno, PendingAction::ChannelWake(r))
                } else {
                    (errno, PendingAction::None)
                }
            }
            Object::Timer(t) => (write_timer(t, buf_ptr, buf_len), PendingAction::None),
            Object::Shmem(s) => {
                let shmem_id = s.shmem_id();
                let res = write_shmem(s, buf_ptr, buf_len, task_id);
                // Check if this was a NOTIFY command (buf_len == 1 and first byte == 1)
                // If so, return shmem_id for deferred notify_shmem_objects call
                let pending = if buf_len == 1 {
                    let mut cmd = [0u8; 1];
                    if uaccess::copy_from_user(&mut cmd, buf_ptr).is_ok() && cmd[0] == 1 {
                        PendingAction::ShmemNotify(shmem_id, task_id)
                    } else {
                        PendingAction::None
                    }
                } else {
                    PendingAction::None
                };
                (res, pending)
            }
            Object::Console(c) => (write_console(c, buf_ptr, buf_len), PendingAction::None),
            Object::Mux(m) => (write_mux(m, buf_ptr, buf_len), PendingAction::None),
            Object::PciDevice(d) => (write_pci_device(d, buf_ptr, buf_len, task_id), PendingAction::None),
            Object::Ring(r) => (write_ring(r, buf_ptr, buf_len, task_id), PendingAction::None),
            _ => (KernelError::NotSupported.to_errno(), PendingAction::None),
        }
    });

    match result {
        Ok((errno, pending)) => {
            // Handle deferred actions AFTER table lock is released
            match pending {
                PendingAction::ShmemNotify(shmem_id, caller_pid) => {
                    notify_shmem_objects(shmem_id, caller_pid);
                }
                PendingAction::ChannelWake(r) => {
                    if let Some(peer) = r.peer {
                        // Wake peer's ChannelObject via ObjectService WaitQueue
                        let wake_list = object_service().wake_channel(
                            peer.task_id, peer.channel_id, super::poll::READABLE,
                        );
                        waker::wake(&wake_list, ipc::WakeReason::Readable);
                    }
                    // Kernel bus channels are dispatched synchronously
                    if let Some((buf, len, channel_id)) = r.bus_data {
                        crate::kernel::bus::process_bus_message(channel_id, &buf[..len]);
                    }
                }
                PendingAction::None => {}
            }
            errno
        }
        Err(_) => KernelError::BadHandle.to_errno(),
    }
}

// ============================================================================
// sys_map - Map handle to memory
// ============================================================================

/// Map a memory handle into address space
///
/// Only valid for: Shmem, DmaPool, Mmio
///
/// Returns: virtual address (positive) or error (negative)
pub fn map(handle_raw: u32, flags: u32) -> i64 {
    use crate::kernel::object_service::object_service;

    let handle = Handle::from_raw(handle_raw);
    let _ = flags; // For future use (read-only, etc.)

    // Get current task ID
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Use ObjectService's tables for dispatch
    let result = object_service().with_table_mut(task_id, |table| {
        let Some(entry) = table.get_mut(handle) else {
            return KernelError::BadHandle.to_errno();
        };

        if !entry.object_type.is_mappable() {
            return KernelError::NotSupported.to_errno();
        }

        // Dispatch to type-specific map
        match &mut entry.object {
            Object::Shmem(s) => map_shmem(s, task_id),
            Object::DmaPool(d) => map_dma_pool(d, task_id),
            Object::Mmio(m) => map_mmio(m, task_id),
            Object::Ring(r) => map_ring(r, task_id),
            _ => KernelError::NotSupported.to_errno(),
        }
    });

    match result {
        Ok(vaddr) => {
            // Validate: a successful map must return a non-zero address.
            // User heap starts at 0x50000000, so 0 is never valid.
            if vaddr > 0 {
                vaddr
            } else {
                KernelError::NoSpace.to_errno()
            }
        }
        Err(_) => KernelError::BadHandle.to_errno(),
    }
}

// ============================================================================
// sys_close - Close a handle
// ============================================================================

/// Close a handle
///
/// Returns: 0 on success, negative error
pub fn close(handle_raw: u32) -> i64 {
    use crate::kernel::object_service::object_service;

    let handle = Handle::from_raw(handle_raw);

    // Get current task ID and slot
    let (task_id, slot) = match task::with_scheduler(|sched| {
        let slot = task::current_slot();
        sched.task(slot).map(|t| (t.id, slot))
    }) {
        Some((id, slot)) => (id, slot),
        None => return KernelError::BadHandle.to_errno(),
    };

    // Close handle via ObjectService
    let object = match object_service().close_handle(task_id, handle) {
        Ok(obj) => obj,
        Err(_) => return KernelError::BadHandle.to_errno(),
    };

    // Extract info needed for cleanup before scheduler lock
    let (is_channel, is_port, is_shmem) = match &object {
        Object::Channel(_) => (true, false, false),
        Object::Port(_) => (false, true, false),
        Object::Shmem(_) => (false, false, true),
        _ => (false, false, false),
    };

    // Type-specific cleanup (outside scheduler lock)
    match object {
        Object::Channel(ch) => close_channel(ch, task_id),
        Object::Port(p) => close_port(p, task_id),
        Object::Shmem(s) => close_shmem(s, task_id),
        Object::DmaPool(d) => close_dma_pool(d, task_id),
        Object::Mmio(m) => close_mmio(m, task_id),
        Object::Ring(r) => close_ring(r, task_id),
        _ => {} // No cleanup needed
    }

    // Update resource counters via scheduler
    if is_channel || is_port || is_shmem {
        task::with_scheduler(|sched| {
            let Some(task) = sched.task_mut(slot) else {
                return; // Task gone, counters don't matter
            };
            if is_channel { task.remove_channel(); }
            if is_port { task.remove_port(); }
            if is_shmem { task.remove_shmem(); }
        });
    }

    0
}

// ============================================================================
// Type-specific implementations (stubs for now)
// ============================================================================

fn open_channel(params_ptr: u64, params_len: usize) -> i64 {
    // Two modes:
    // 1. params_len == 0: Create channel pair
    // 2. params_len > 0: Connect to named port
    //
    // For channel pair: Returns both handles packed: (handle_a << 32) | handle_b
    // For port connect: Returns single channel handle

    use crate::kernel::object_service::object_service;

    // Get current task ID
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Check if this is a port connect (params contains port name)
    if params_len > 0 && params_len <= 32 {
        // Copy port name from user
        let mut name_buf = [0u8; 32];
        if uaccess::copy_from_user(&mut name_buf[..params_len], params_ptr).is_err() {
            return KernelError::BadAddress.to_errno();
        }

        // Delegate to ObjectService
        match object_service().open_channel_connect(task_id, &name_buf[..params_len]) {
            Ok(handle) => handle.raw() as i64,
            Err(e) => e.to_errno(),
        }
    } else {
        // Create channel pair
        match object_service().open_channel_pair(task_id) {
            Ok((handle_a, handle_b)) => {
                // Pack both handles into return value
                ((handle_a.raw() as i64) << 32) | (handle_b.raw() as i64)
            }
            Err(e) => e.to_errno(),
        }
    }
}

fn open_timer(_params_ptr: u64, _params_len: usize) -> i64 {
    // Get current task ID
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_timer(task_id) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_process(params_ptr: u64, params_len: usize) -> i64 {
    // Params: u32 PID to watch (4 bytes)
    if params_len != 4 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy PID from user
    let mut pid_bytes = [0u8; 4];
    if uaccess::copy_from_user(&mut pid_bytes, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let target_pid = u32::from_le_bytes(pid_bytes);

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_process(task_id, target_pid) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_port(params_ptr: u64, params_len: usize) -> i64 {
    // params contains the port name
    if params_len == 0 || params_len > 32 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy port name from user
    let mut name_buf = [0u8; 32];
    if uaccess::copy_from_user(&mut name_buf[..params_len], params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }

    // Get current task ID
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_port(task_id, &name_buf[..params_len]) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_shmem(params_ptr: u64, params_len: usize) -> i64 {
    // Params format depends on length:
    // - 8 bytes: Create new (size:u64)
    // - 4 bytes: Open existing (shmem_id:u32)
    match params_len {
        8 => open_shmem_create(params_ptr),
        4 => open_shmem_existing(params_ptr),
        _ => KernelError::InvalidArg.to_errno(),
    }
}

fn open_shmem_create(params_ptr: u64) -> i64 {
    // Copy size from user
    let mut size_bytes = [0u8; 8];
    if uaccess::copy_from_user(&mut size_bytes, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let size = u64::from_le_bytes(size_bytes) as usize;

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    // Returns (handle, shmem_id) - encode both in return value
    // Lower 32 bits: handle, Upper 32 bits: shmem_id
    use crate::kernel::object_service::object_service;
    match object_service().open_shmem_create(task_id, size) {
        Ok((handle, shmem_id)) => {
            ((shmem_id as i64) << 32) | (handle.raw() as i64)
        }
        Err(e) => e.to_errno(),
    }
}

fn open_shmem_existing(params_ptr: u64) -> i64 {
    // Copy shmem_id from user
    let mut id_bytes = [0u8; 4];
    if uaccess::copy_from_user(&mut id_bytes, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let shmem_id = u32::from_le_bytes(id_bytes);

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_shmem_existing(task_id, shmem_id) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_dma_pool(params_ptr: u64, params_len: usize) -> i64 {
    // Params: u64 size, u32 flags (16 bytes)
    // flags bit 0: 1 = high memory pool, 0 = low memory pool
    // flags bit 1: 1 = streaming (cacheable), 0 = coherent (non-cacheable)
    if params_len != 16 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy params from user
    let mut params = [0u8; 16];
    if uaccess::copy_from_user(&mut params, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    // Use array indexing (infallible for known-size arrays)
    let size = u64::from_le_bytes([
        params[0], params[1], params[2], params[3],
        params[4], params[5], params[6], params[7],
    ]) as usize;
    let flags = u32::from_le_bytes([params[8], params[9], params[10], params[11]]);
    let use_high = (flags & 1) != 0;
    let streaming = (flags & 2) != 0;

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_dma_pool(task_id, size, use_high, streaming) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_mmio(params_ptr: u64, params_len: usize) -> i64 {
    // Params: u64 phys_addr, u64 size (16 bytes)
    if params_len != 16 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy params from user
    let mut params = [0u8; 16];
    if uaccess::copy_from_user(&mut params, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    // Use array indexing (infallible for known-size arrays)
    let phys_addr = u64::from_le_bytes([
        params[0], params[1], params[2], params[3],
        params[4], params[5], params[6], params[7],
    ]);
    let size = u64::from_le_bytes([
        params[8], params[9], params[10], params[11],
        params[12], params[13], params[14], params[15],
    ]) as usize;

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_mmio(task_id, phys_addr, size) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_console(console_type: super::ConsoleType) -> i64 {
    // Get current task ID
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_console(task_id, console_type) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_klog(_params_ptr: u64, _params_len: usize) -> i64 {
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_klog(task_id) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_mux(_params_ptr: u64, _params_len: usize) -> i64 {
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_mux(task_id) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_pci_bus(params_ptr: u64, params_len: usize) -> i64 {
    // Params: u32 bdf_filter (4 bytes) - 0 for all devices
    // BDF format: bus << 8 | dev << 3 | func
    let bdf_filter = if params_len == 0 {
        0 // Default: all devices
    } else if params_len == 4 {
        let mut bdf_bytes = [0u8; 4];
        if uaccess::copy_from_user(&mut bdf_bytes, params_ptr).is_err() {
            return KernelError::BadAddress.to_errno();
        }
        u32::from_le_bytes(bdf_bytes)
    } else {
        return KernelError::InvalidArg.to_errno();
    };

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_pci_bus(task_id, bdf_filter) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_pci_device(params_ptr: u64, params_len: usize) -> i64 {
    use crate::kernel::caps::Capabilities;
    use crate::kernel::pci::{self, PciBdf};

    // Params: u32 bdf (4 bytes)
    if params_len != 4 {
        return KernelError::InvalidArg.to_errno();
    }

    // Require RAW_DEVICE capability
    if let Err(e) = super::super::syscall::require_capability(Capabilities::RAW_DEVICE) {
        return e;
    }

    let mut bdf_bytes = [0u8; 4];
    if uaccess::copy_from_user(&mut bdf_bytes, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let bdf = u32::from_le_bytes(bdf_bytes);

    // Check device exists
    let bdf_typed = PciBdf::from_u32(bdf);
    if pci::find_by_bdf(bdf_typed).is_none() {
        return KernelError::NotFound.to_errno();
    }

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_pci_device(task_id, bdf) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_msi(params_ptr: u64, params_len: usize) -> i64 {
    use crate::kernel::caps::Capabilities;
    use crate::kernel::pci::{self, PciBdf};

    // Params: u32 bdf (4 bytes) + u8 count (1 byte) = 5 bytes
    if params_len != 5 {
        return KernelError::InvalidArg.to_errno();
    }

    // Require IRQ_CLAIM capability
    if let Err(e) = super::super::syscall::require_capability(Capabilities::IRQ_CLAIM) {
        return e;
    }

    let mut params = [0u8; 5];
    if uaccess::copy_from_user(&mut params, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let bdf = u32::from_le_bytes([params[0], params[1], params[2], params[3]]);
    let count = params[4];

    let bdf_typed = PciBdf::from_u32(bdf);

    // Check device exists and caller owns it
    let dev = match pci::find_by_bdf(bdf_typed) {
        Some(d) => d,
        None => return KernelError::NotFound.to_errno(),
    };

    // Get current pid via safe scheduler access
    let task_id = task::with_scheduler(|sched| {
        sched.current_task().map(|t| t.id).unwrap_or(0)
    });

    if dev.owner() != task_id {
        return KernelError::PermDenied.to_errno();
    }

    // Check device supports MSI
    if !dev.has_msi() && !dev.has_msix() {
        return KernelError::NotSupported.to_errno();
    }

    // Allocate vectors
    let first_irq = match pci::msi_alloc(bdf_typed, count) {
        Ok(irq) => irq,
        Err(pci::PciError::NoMsiVectors) => return KernelError::NoSpace.to_errno(),
        Err(_) => return KernelError::Io.to_errno(),
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_msi(task_id, bdf, first_irq, count) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_bus_create(params_ptr: u64, params_len: usize) -> i64 {
    use crate::kernel::bus;
    use crate::kernel::caps;

    if params_len != core::mem::size_of::<abi::BusCreateInfo>() {
        return KernelError::InvalidArg.to_errno();
    }

    // Check caller has CAP_BUS_CREATE
    let has_cap = task::with_scheduler(|sched| {
        let slot = task::current_slot();
        sched.task(slot).map(|t| t.has_capability(caps::Capabilities::BUS_CREATE)).unwrap_or(false)
    });
    if !has_cap {
        return KernelError::PermDenied.to_errno();
    }

    // Check bus creation is not locked
    if bus::is_bus_creation_locked() {
        return KernelError::PermDenied.to_errno();
    }

    // Copy BusCreateInfo from user memory
    let mut info_bytes = [0u8; 32];
    if uaccess::copy_from_user(&mut info_bytes, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let info = unsafe { &*(info_bytes.as_ptr() as *const abi::BusCreateInfo) };

    // Create the bus (kernel_pid = 0 for kernel-owned ports)
    match bus::create_bus(info, 0) {
        Ok(()) => 0, // Success, no handle returned
        Err(e) => e.to_errno() as i64,
    }
}

fn open_bus_list(_params_ptr: u64, _params_len: usize) -> i64 {
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_bus_list(task_id) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

// Read implementations
fn read_channel(ch: &mut super::ChannelObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Check state first (queries ipc backend)
    if ch.is_closed() && !ipc::channel_has_messages(ch.channel_id()) {
        return KernelError::PeerClosed.to_errno();
    }

    let channel_id = ch.channel_id();
    if channel_id == 0 {
        return KernelError::PeerClosed.to_errno();
    }

    let pid = task::with_scheduler(|sched| {
        match sched.current_task() {
            Some(t) => t.id,
            None => 0,
        }
    });
    if pid == 0 {
        return KernelError::BadHandle.to_errno();
    }

    // Receive message from ipc
    match ipc::receive(channel_id, pid) {
        Ok(msg) => {
            let payload = msg.payload_slice();
            let copy_len = core::cmp::min(payload.len(), buf_len);

            // Copy to user buffer
            if uaccess::copy_to_user(buf_ptr, &payload[..copy_len]).is_err() {
                return KernelError::BadAddress.to_errno();
            }

            copy_len as i64
        }
        Err(ipc::IpcError::WouldBlock) => KernelError::WouldBlock.to_errno(),
        Err(ipc::IpcError::PeerClosed) | Err(ipc::IpcError::Closed) => {
            // Backend already knows about peer close
            KernelError::PeerClosed.to_errno()
        }
        Err(_) => KernelError::BadHandle.to_errno(),
    }
}

fn read_timer(t: &mut super::TimerObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    use super::TimerState;
    use crate::platform::current::timer;

    // Use hardware counter - deadline is in counter units (set via deadline_ns)
    let current_counter = timer::counter();

    match t.state() {
        TimerState::Disarmed => {
            // Timer not armed - return error
            KernelError::InvalidArg.to_errno()
        }
        TimerState::Armed => {
            if current_counter >= t.deadline() {
                // Timer has fired - use transition method
                let _ = t.fire();

                // Write timestamp to buffer if provided
                if buf_ptr != 0 && buf_len >= 8 {
                    let ts_bytes = current_counter.to_le_bytes();
                    if uaccess::copy_to_user(buf_ptr, &ts_bytes).is_err() {
                        return KernelError::BadAddress.to_errno();
                    }
                }

                // Handle recurring timer - re-arm with new deadline
                if t.interval() > 0 {
                    let new_deadline = current_counter + t.interval();
                    t.arm(new_deadline, t.interval());
                }

                8 // Return bytes written
            } else {
                // Not yet fired - register waker and return WouldBlock
                let sub = ipc::traits::Subscriber {
                    task_id,
                    generation: task::Scheduler::generation_from_pid(task_id),
                };
                t.wait_queue_mut().subscribe(super::Waiter::from_subscriber(sub, super::types::filter::READABLE));
                KernelError::WouldBlock.to_errno()
            }
        }
        TimerState::Fired => {
            // Already fired - return timestamp
            if buf_ptr != 0 && buf_len >= 8 {
                let ts_bytes = t.deadline().to_le_bytes();
                if uaccess::copy_to_user(buf_ptr, &ts_bytes).is_err() {
                    return KernelError::BadAddress.to_errno();
                }
            }
            // Consume the fired state
            t.disarm();
            8
        }
    }
}

fn read_process(p: &mut super::ProcessObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    use crate::kernel::ipc::traits::Subscriber;

    // Check if we already have the exit code cached
    if let Some(code) = p.exit_code {
        // Return the exit code
        if buf_ptr != 0 && buf_len >= 4 {
            let code_bytes = code.to_le_bytes();
            if uaccess::copy_to_user(buf_ptr, &code_bytes).is_err() {
                return KernelError::BadAddress.to_errno();
            }
        }
        return 4; // Return 4 bytes written
    }

    // Lazy check: see if target task has exited since we last checked
    let exit_code = task::with_scheduler(|sched| {
        if let Some(target_slot) = sched.slot_by_pid(p.pid) {
            if let Some(target) = sched.task(target_slot) {
                target.state().exit_code()
            } else {
                Some(-1) // Slot exists but task is None (shouldn't happen)
            }
        } else {
            // Task not found - PID no longer valid, task was reaped
            Some(-1)
        }
    });

    if let Some(code) = exit_code {
        // Target has exited - cache and return
        p.exit_code = Some(code);
        if buf_ptr != 0 && buf_len >= 4 {
            let code_bytes = code.to_le_bytes();
            if uaccess::copy_to_user(buf_ptr, &code_bytes).is_err() {
                return KernelError::BadAddress.to_errno();
            }
        }
        return 4;
    }

    // Target still running - register subscriber and return WouldBlock
    let sub = Subscriber {
        task_id,
        generation: task::Scheduler::generation_from_pid(task_id),
    };
    p.wait_queue_mut().subscribe(super::Waiter::from_subscriber(sub, super::types::filter::READABLE));
    KernelError::WouldBlock.to_errno()
}

/// Read from channel with blocking - waits until data arrives or peer closes
///
/// This is the proper blocking recv() implementation. Instead of returning WouldBlock,
/// it subscribes to the channel, blocks the task, and waits for data.
fn read_channel_blocking(task_id: u32, handle: Handle, buf_ptr: u64, buf_len: usize) -> i64 {
    // Main loop - keep trying until we have data or peer closes
    loop {
        // Phase 1: Get channel_id and check if closed
        let channel_id = {
            let result = object_service().with_table(task_id, |table| {
                let Some(entry) = table.get(handle) else {
                    return Err(KernelError::BadHandle);
                };
                let Object::Channel(ref ch) = entry.object else {
                    return Err(KernelError::BadHandle);
                };
                Ok(ch.channel_id())
            });

            match result {
                Ok(Ok(id)) => id,
                Ok(Err(e)) => return e.to_errno(),
                Err(_) => return KernelError::BadHandle.to_errno(),
            }
        };

        if channel_id == 0 {
            return KernelError::PeerClosed.to_errno();
        }

        // Phase 2: Check if channel is closed or has data
        // (Subscription is handled by ChannelObject.subscribe() via WaitQueue)
        let is_closed = ipc::channel_is_closed(channel_id);
        let has_messages = ipc::channel_has_messages(channel_id);

        if is_closed && !has_messages {
            // Peer closed and no remaining messages
            return KernelError::PeerClosed.to_errno();
        }

        if has_messages {
            // Try to receive the message
            match ipc::receive(channel_id, task_id) {
                Ok(msg) => {
                    let payload = msg.payload_slice();
                    let copy_len = core::cmp::min(payload.len(), buf_len);

                    if uaccess::copy_to_user(buf_ptr, &payload[..copy_len]).is_err() {
                        return KernelError::BadAddress.to_errno();
                    }

                    return copy_len as i64;
                }
                Err(ipc::IpcError::WouldBlock) => {
                    // Race: message was consumed by another thread, continue to block
                }
                Err(ipc::IpcError::PeerClosed) | Err(ipc::IpcError::Closed) => {
                    return KernelError::PeerClosed.to_errno();
                }
                Err(_) => {
                    return KernelError::BadHandle.to_errno();
                }
            }
        }

        // Phase 4: No data available - block the task
        // Phase 5: Atomically block and context switch to another task.
        // Uses sleep_and_reschedule to prevent SMP race where another CPU
        // wakes+schedules us before our kernel context is saved.
        if !crate::kernel::sched::sleep_and_reschedule(
            crate::kernel::task::SleepReason::ChannelRecv
        ) {
            return KernelError::WouldBlock.to_errno();
        }
        // When we're woken, we'll loop back and try to receive again
    }
}

/// Read from port via service - accepts connection and allocates channel handle
///
/// Must be called OUTSIDE any with_table_mut closure to avoid deadlock,
/// since allocating the channel handle requires taking the tables lock.
fn read_port_via_service(task_id: u32, handle: Handle, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::kernel::object_service::object_service;

    // Need at least 4 bytes for handle
    if buf_ptr == 0 || buf_len < 4 {
        return KernelError::InvalidArg.to_errno();
    }

    // Phase 1: Get port_id and check state (quick lock)
    let port_id = {
        let result = object_service().with_table(task_id, |table| {
            let Some(entry) = table.get(handle) else {
                return Err(KernelError::BadHandle);
            };
            match &entry.object {
                Object::Port(p) => {
                    if !p.is_listening() {
                        return Err(KernelError::PeerClosed);
                    }
                    Ok(p.port_id())
                }
                _ => Err(KernelError::BadHandle),
            }
        });
        match result {
            Ok(Ok(id)) => id,
            Ok(Err(e)) => return e.to_errno(),
            Err(_) => return KernelError::BadHandle.to_errno(),
        }
    };

    // Phase 2: Accept connection (no ObjectService lock held)
    let (server_channel, client_pid) = match ipc::port_accept(port_id, task_id) {
        Ok(result) => result,
        Err(ipc::IpcError::NoPending) => return KernelError::WouldBlock.to_errno(),
        Err(ipc::IpcError::Closed) => return KernelError::PeerClosed.to_errno(),
        Err(_) => return KernelError::BadHandle.to_errno(),
    };

    // Phase 3: Allocate channel handle (takes lock again - safe, not nested)
    let ch_obj = Object::Channel(super::ChannelObject::new(server_channel));
    let new_handle = match object_service().with_table_mut(task_id, |table| {
        table.alloc(ObjectType::Channel, ch_obj)
    }) {
        Ok(Some(h)) => h,
        Ok(None) | Err(_) => {
            // Close the channel we couldn't wrap
            let _ = ipc::close_channel(server_channel, task_id);
            return KernelError::NoSpace.to_errno();
        }
    };

    // Phase 4: Write results to user
    let handle_bytes = new_handle.raw().to_le_bytes();
    if uaccess::copy_to_user(buf_ptr, &handle_bytes).is_err() {
        return KernelError::BadAddress.to_errno();
    }

    if buf_len >= 8 {
        let pid_bytes = client_pid.to_le_bytes();
        if uaccess::copy_to_user(buf_ptr + 4, &pid_bytes).is_err() {
            return KernelError::BadAddress.to_errno();
        }
        return 8;
    }
    4 // Wrote 4 bytes (handle only)
}

// Note: Port reads are handled via read_port_via_service() to avoid nested lock issues
// when allocating the channel handle. See that function for the implementation.

fn read_shmem(s: &mut super::ShmemObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    // buf_len >= 16: Metadata query - returns paddr (u64) + size (u64)
    // buf_len < 16: WAIT operation with optional timeout
    if buf_len >= 16 {
        let mut info = [0u8; 16];
        info[0..8].copy_from_slice(&s.paddr().to_le_bytes());
        info[8..16].copy_from_slice(&(s.size() as u64).to_le_bytes());
        if uaccess::copy_to_user(buf_ptr, &info).is_err() {
            return KernelError::BadAddress.to_errno();
        }
        return 16;
    }

    // WAIT operation - params: optional timeout_ms (u32), default 0 = infinite
    let timeout_ms = if buf_len >= 4 {
        let mut timeout_bytes = [0u8; 4];
        if uaccess::copy_from_user(&mut timeout_bytes, buf_ptr).is_err() {
            return KernelError::BadAddress.to_errno();
        }
        u32::from_le_bytes(timeout_bytes)
    } else {
        0 // Infinite wait
    };

    match shmem::wait(task_id, s.shmem_id(), timeout_ms) {
        Ok(()) => 0,
        Err(-11) => {
            // EAGAIN = successfully set up wait, task is now blocked.
            // Return 0 so assembly stores 0 to trap_frame.x0.
            // When do_resched_if_needed runs, it will see task is blocked and reschedule.
            // When we wake up, eret returns to userspace with x0=0 (success).
            0
        }
        Err(e) => e,
    }
}

fn read_dma_pool(d: &mut super::DmaPoolObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Read returns DMA pool info: physical address (u64) + size (u64) = 16 bytes
    if buf_len < 16 {
        return KernelError::InvalidArg.to_errno();
    }

    let mut info = [0u8; 16];
    info[0..8].copy_from_slice(&d.paddr().to_le_bytes());
    info[8..16].copy_from_slice(&(d.size() as u64).to_le_bytes());

    if uaccess::copy_to_user(buf_ptr, &info).is_err() {
        return KernelError::BadAddress.to_errno();
    }

    16 // Return bytes written
}

fn read_console(c: &mut super::ConsoleObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    use super::Pollable;
    use crate::kernel::ipc::traits::Subscriber;

    // Only stdin is readable
    match c.console_type {
        ConsoleType::Stdin => {
            if buf_len == 0 {
                return 0;
            }
            // Read from UART RX buffer (non-blocking for now)
            let mut kernel_buf = [0u8; 256];
            let max_read = core::cmp::min(buf_len, 256);
            let mut bytes_read = 0;

            for i in 0..max_read {
                if let Some(byte) = uart::rx_buffer_read() {
                    kernel_buf[i] = byte;
                    bytes_read += 1;
                } else {
                    break;
                }
            }

            if bytes_read == 0 {
                // Register for wake when input arrives (fixes audit #13)
                let sub = Subscriber {
                    task_id,
                    generation: task::Scheduler::generation_from_pid(task_id),
                };
                c.subscribe(sub);
                return KernelError::WouldBlock.to_errno();
            }

            // Copy to user
            if uaccess::copy_to_user(buf_ptr, &kernel_buf[..bytes_read]).is_err() {
                return KernelError::BadAddress.to_errno();
            }

            bytes_read as i64
        }
        ConsoleType::Stdout | ConsoleType::Stderr => KernelError::NotSupported.to_errno(),
    }
}

fn read_klog(k: &mut super::KlogObject, buf_ptr: u64, buf_len: usize) -> i64 {
    let _ = (k, buf_ptr, buf_len);
    KernelError::NotSupported.to_errno()
}

fn read_mux(_m: &mut super::MuxObject, _buf_ptr: u64, _buf_len: usize, _task_id: u32) -> i64 {
    // This shouldn't be called - Mux is handled specially in read()
    KernelError::NotSupported.to_errno()
}

fn read_pci_bus(p: &mut super::PciBusObject, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::kernel::pci;

    if buf_ptr == 0 {
        // Just return device count
        return pci::device_count() as i64;
    }

    let entry_size = core::mem::size_of::<abi::PciEnumEntry>();
    let max = buf_len / entry_size;
    if max == 0 {
        return 0;
    }

    let bdf_filter = p.bdf_filter();

    // Fill PciEnumEntry array from kernel registry
    let count = pci::with_devices(|registry| {
        let mut written = 0usize;
        for i in 0..registry.len() {
            if written >= max {
                break;
            }
            let Some(dev) = registry.get(i) else { continue };

            // Apply BDF filter if non-zero
            if bdf_filter != 0 {
                let packed_bdf = ((dev.bdf.bus as u32) << 8)
                    | ((dev.bdf.device as u32) << 3)
                    | (dev.bdf.function as u32);
                if packed_bdf != bdf_filter {
                    continue;
                }
            }

            let entry = abi::PciEnumEntry {
                bdf: dev.bdf.to_u32(),
                vendor_id: dev.vendor_id,
                device_id: dev.device_id,
                class_code: (dev.class_code << 8) | (dev.revision as u32),
                bar0_addr: dev.bar0_addr,
                bar0_size: dev.bar0_size as u32,
                msi_cap: dev.msi_cap,
                msix_cap: dev.msix_cap,
                _reserved: [0; 2],
            };

            // Copy single entry to userspace
            let src = unsafe {
                core::slice::from_raw_parts(
                    &entry as *const abi::PciEnumEntry as *const u8,
                    entry_size,
                )
            };
            let dst = buf_ptr + (written * entry_size) as u64;
            if uaccess::copy_to_user(dst, src).is_err() {
                return written;
            }
            written += 1;
        }
        written
    });

    count as i64
}

fn read_pci_device(d: &mut super::PciDeviceObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    use crate::kernel::pci::{self, PciBdf};

    // Params: offset:u16 (2 bytes) + size:u8 (1 byte) = 3 bytes
    if buf_len < 3 {
        return KernelError::InvalidArg.to_errno();
    }

    let mut params = [0u8; 3];
    if uaccess::copy_from_user(&mut params, buf_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let offset = u16::from_le_bytes([params[0], params[1]]);
    let size = params[2];

    // Validate offset and size
    const PCI_CONFIG_MAX: u16 = 4096;
    if offset >= PCI_CONFIG_MAX || (size != 1 && size != 2 && size != 4) {
        return KernelError::InvalidArg.to_errno();
    }
    if (offset as u32) + (size as u32) > PCI_CONFIG_MAX as u32 {
        return KernelError::InvalidArg.to_errno();
    }
    // Check alignment
    if (size == 2 && (offset & 1) != 0) || (size == 4 && (offset & 3) != 0) {
        return KernelError::InvalidArg.to_errno();
    }

    let bdf = PciBdf::from_u32(d.bdf());

    // Check device exists and caller has access
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return KernelError::NotFound.to_errno(),
    };

    // Check ownership (0 = unclaimed, anyone can read)
    let owner = dev.owner();
    if owner != 0 && owner != task_id {
        return KernelError::PermDenied.to_errno();
    }

    match pci::config_read32(bdf, offset & !0x3) {
        Ok(val32) => {
            // Extract the requested portion
            match size {
                1 => {
                    let shift = (offset & 3) * 8;
                    ((val32 >> shift) & 0xFF) as i64
                }
                2 => {
                    let shift = (offset & 2) * 8;
                    ((val32 >> shift) & 0xFFFF) as i64
                }
                4 => val32 as i64,
                _ => KernelError::InvalidArg.to_errno(),
            }
        }
        Err(_) => KernelError::Io.to_errno(),
    }
}

fn read_msi(m: &mut super::MsiObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Return MSI info: first_irq:u32 + count:u8 + bdf:u32 = 9 bytes
    if buf_len < 9 {
        return KernelError::InvalidArg.to_errno();
    }

    let mut out = [0u8; 9];
    out[0..4].copy_from_slice(&m.first_irq().to_le_bytes());
    out[4] = m.count();
    out[5..9].copy_from_slice(&m.bdf().to_le_bytes());

    if uaccess::copy_to_user(buf_ptr, &out).is_err() {
        return KernelError::BadAddress.to_errno();
    }

    9
}

fn read_bus_list(b: &mut super::BusListObject, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::kernel::bus::{BusInfo, get_bus_list, get_bus_count};

    if buf_ptr == 0 {
        // Just return count
        return get_bus_count() as i64;
    }

    let max = buf_len / core::mem::size_of::<BusInfo>();
    if max == 0 {
        return 0;
    }

    // Create temporary buffer (max 16 buses)
    let mut temp = [BusInfo::empty(); 16];
    let count = get_bus_list(&mut temp[..max.min(16)]);

    // Skip already-read entries
    let cursor = b.cursor() as usize;
    if cursor >= count {
        return 0; // No more entries
    }

    let remaining = count - cursor;
    let to_copy = remaining.min(max.min(16 - cursor));

    // Copy to userspace
    let copy_size = to_copy * core::mem::size_of::<BusInfo>();
    let src_bytes = unsafe {
        core::slice::from_raw_parts(temp[cursor..].as_ptr() as *const u8, copy_size)
    };
    if uaccess::copy_to_user(buf_ptr, src_bytes).is_err() {
        return KernelError::BadAddress.to_errno();
    }

    // Advance cursor
    for _ in 0..to_copy {
        b.advance();
    }

    to_copy as i64
}

/// Mux read via ObjectService - blocks until events are ready
///
/// Uses ObjectService's tables instead of task.object_table.
/// Loops until events are ready, sleeping between attempts.
/// Mux read: single-pass event-driven blocking.
///
/// # Design
///
/// Three phases, no loop:
///   1. Subscribe + Poll (atomic, under table lock)
///   2. Block (task sleeps — zero CPU, woken by event or timeout)
///   3. Woken — re-poll and return
///
/// # Returns
///   - N > 0: N events written to buffer
///   - -ETIMEDOUT (-110): Timeout fired, no events (periodic work opportunity)
///   - -EAGAIN (-11): Spurious wake, no events (userspace retries)
///   - -EBADF: Bad handle
///   - -EFAULT: Bad user address
fn read_mux_via_service(task_id: crate::kernel::task::TaskId, mux_handle: Handle, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::kernel::error::KernelError;
    use crate::kernel::ipc::traits::Subscriber;
    use crate::kernel::object_service::object_service;

    let slot = task::current_slot();
    let subscriber = Subscriber {
        task_id,
        generation: task::Scheduler::generation_from_pid(task_id),
    };

    // ── Phase 1: Subscribe + Poll (single lock, atomic) ──
    //
    // CRITICAL: Subscribe and poll in one locked section. If we subscribe,
    // release the lock, then poll separately, an event can arrive in the gap.
    // The waker finds our subscriber but we're not blocked yet — wake is lost.
    let subscribe_and_poll_result = object_service().with_table_mut(task_id, |table| {
        // Validate mux handle
        let Some(mux_entry) = table.get(mux_handle) else {
            return Err(KernelError::BadHandle);
        };
        let Object::Mux(ref mux) = mux_entry.object else {
            return Err(KernelError::BadHandle);
        };

        // Snapshot watches and timeout before mutating
        let timeout_ns = mux.timeout_ns();
        let mut watches = [(None::<super::MuxWatch>, 0u32); super::MAX_MUX_WATCHES];
        for i in 0..super::MAX_MUX_WATCHES {
            if let Some(watch) = mux.watches()[i] {
                let watched_handle = Handle::from_raw(watch.handle());
                let channel_id = if let Some(entry) = table.get(watched_handle) {
                    if let Object::Channel(ch) = &entry.object { ch.channel_id() } else { 0 }
                } else { 0 };
                watches[i] = (Some(watch), channel_id);
            }
        }

        // Subscribe to mux itself
        let waiter = super::Waiter::from_subscriber(subscriber, super::types::filter::READABLE);
        if let Some(mux_entry) = table.get_mut(mux_handle) {
            if let Object::Mux(ref mut mux) = mux_entry.object {
                mux.wait_queue_mut().subscribe(waiter);
            }
        }

        // Subscribe to each watched object + compute earliest timer deadline
        let mut earliest_deadline: u64 = u64::MAX;

        for watch_opt in &watches {
            let Some(watch) = watch_opt.0 else { continue };
            let _channel_id = watch_opt.1;
            let watched_handle = Handle::from_raw(watch.handle());

            if let Some(entry) = table.get_mut(watched_handle) {
                match &mut entry.object {
                    Object::Channel(ch) => {
                        ch.wait_queue_mut().subscribe(super::Waiter::from_subscriber(subscriber, watch.filter()));
                    }
                    Object::Port(p) => { p.wait_queue_mut().subscribe(super::Waiter::from_subscriber(subscriber, watch.filter())); }
                    Object::Timer(t) => {
                        t.wait_queue_mut().subscribe(super::Waiter::from_subscriber(subscriber, watch.filter()));
                        if t.deadline() > 0 && t.deadline() < earliest_deadline {
                            earliest_deadline = t.deadline();
                        }
                    }
                    Object::Console(c) => {
                        c.wait_queue_mut().subscribe(super::Waiter::from_subscriber(subscriber, watch.filter()));
                        if matches!(c.console_type(), ConsoleType::Stdin) {
                            uart::block_for_input(task_id);
                        }
                    }
                    Object::Process(p) => { p.wait_queue_mut().subscribe(super::Waiter::from_subscriber(subscriber, watch.filter())); }
                    Object::Shmem(s) => { s.set_mux_subscriber(Some(subscriber)); }
                    _ => {}
                }
            }
        }

        // Incorporate mux timeout into earliest_deadline
        if timeout_ns > 0 {
            let abs_deadline = crate::platform::current::timer::deadline_ns(timeout_ns);
            if abs_deadline < earliest_deadline {
                earliest_deadline = abs_deadline;
            }
        }

        // Poll all watched objects (still holding the lock)
        let mut events = [super::MuxEvent::empty(); super::MAX_MUX_EVENTS];
        let mut event_count = 0usize;
        let max_events = buf_len / core::mem::size_of::<super::MuxEvent>();
        let current_tick = crate::platform::current::timer::counter();

        for watch_opt in &watches {
            if event_count >= max_events { break; }
            let Some(watch) = watch_opt.0 else { continue };
            let channel_id = watch_opt.1;
            let watched_handle = Handle::from_raw(watch.handle());

            let Some(entry) = table.get_mut(watched_handle) else { continue };

            let ready = poll_watched_object_mut(entry, watch.filter(), channel_id, current_tick);

            if ready {
                events[event_count] = super::MuxEvent {
                    handle: abi::Handle::from_raw(watch.handle()),
                    event: watch.filter(),
                    _pad: [0; 3],
                };
                event_count += 1;
            }
        }

        Ok((watches, earliest_deadline, timeout_ns, events, event_count))
    });

    let (watches, earliest_deadline, timeout_ns, events, event_count) = match subscribe_and_poll_result {
        Ok(Ok(result)) => result,
        Ok(Err(e)) => return e.to_errno(),
        Err(_) => return KernelError::BadHandle.to_errno(),
    };

    // Events already ready → return immediately (no blocking needed)
    if event_count > 0 {
        uart::clear_blocked_if_pid(task_id);
        return copy_mux_events_to_user(buf_ptr, &events, event_count);
    }

    // ── Phase 2: Block (task sleeps until event or timeout) ──
    // NOTE: Mux blocking cannot use sleep_and_reschedule() because it needs
    // a post-block re-poll to catch events that arrived between the poll and
    // the state transition. The blocking and rescheduling must be separate
    // steps to allow this race-detection window.
    let transition_ok = task::with_scheduler(|sched| {
        // First pass: check state and do transition
        let ok = {
            let Some(task) = sched.task_mut(slot) else {
                return false;
            };

            // Handle task state at block point
            let cpu = crate::kernel::percpu::cpu_id();
            match task.state() {
                task::TaskState::Running { .. } => {}
                task::TaskState::Ready => {
                    if task.set_running(cpu).is_err() {
                        return false;
                    }
                }
                task::TaskState::Sleeping { .. } | task::TaskState::Waiting { .. } => {
                    return true; // Already blocked
                }
                _ => return false,
            }

            if earliest_deadline == u64::MAX {
                task.set_sleeping(crate::kernel::task::SleepReason::EventLoop).is_ok()
            } else {
                task.set_waiting(crate::kernel::task::WaitReason::TimedEvent, earliest_deadline).is_ok()
            }
        }; // task borrow ends here

        if ok {
            if earliest_deadline != u64::MAX {
                sched.note_deadline(earliest_deadline);
            }
            // CRITICAL: Prevent simple preemption race — another CPU must not
            // schedule this task via trap-frame swap while our kernel stack is live.
            if let Some(task) = sched.task_mut(slot) {
                task.mark_context_saved();
            }
        }
        ok
    });

    if !transition_ok {
        return KernelError::InvalidArg.to_errno();
    }

    // Post-block race check: poll once more while blocked to catch events
    // that arrived between our poll and the state transition.
    let late_events_or_woken = {
        let late_found = object_service().with_table(task_id, |table| {
            for watch_opt in &watches {
                let Some(watch) = watch_opt.0 else { continue };
                let channel_id = watch_opt.1;
                let watched_handle = Handle::from_raw(watch.handle());
                let Some(entry) = table.get(watched_handle) else { continue };
                if poll_watched_object(&entry.object, watch.filter(), channel_id) {
                    return true;
                }
            }
            false
        }).unwrap_or(false);

        let task_woken = task::with_scheduler(|sched| {
            sched.task(slot).map(|t| !t.is_blocked()).unwrap_or(false)
        });

        late_found || task_woken
    };

    if late_events_or_woken {
        // Self-wake: transition back to Running via state machine
        task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                let cpu = crate::kernel::percpu::cpu_id();
                if !task.state().is_running() {
                    crate::transition_or_log!(task, wake);
                    if *task.state() == task::TaskState::Ready {
                        crate::transition_or_evict!(task, set_running, cpu);
                    }
                }
                // Clear flag since we're NOT context switching (self-wake path)
                task.mark_context_restored();
            }
        });
    } else {
        // No late events, still blocked → reschedule (task is OFF the CPU)
        crate::kernel::sched::reschedule();

        // Woken by event or timeout. Ensure Running.
        task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                let cpu = crate::kernel::percpu::cpu_id();
                if *task.state() == task::TaskState::Ready {
                    crate::transition_or_evict!(task, set_running, cpu);
                }
                // Reset liveness — proves task is responsive
                let current_tick = crate::platform::current::timer::logical_ticks();
                task.record_activity(current_tick);
                task.reset_liveness_if_implicit_pong();
            }
        });
    }

    // ── Phase 3: Woken — re-poll and return ──
    uart::clear_blocked_if_pid(task_id);

    let repoll_result = object_service().with_table_mut(task_id, |table| {
        let mut events = [super::MuxEvent::empty(); super::MAX_MUX_EVENTS];
        let mut event_count = 0usize;
        let max_events = buf_len / core::mem::size_of::<super::MuxEvent>();
        let current_tick = crate::platform::current::timer::counter();

        for watch_opt in &watches {
            if event_count >= max_events { break; }
            let Some(watch) = watch_opt.0 else { continue };
            let channel_id = watch_opt.1;
            let watched_handle = Handle::from_raw(watch.handle());

            let Some(entry) = table.get_mut(watched_handle) else { continue };

            if poll_watched_object_mut(entry, watch.filter(), channel_id, current_tick) {
                events[event_count] = super::MuxEvent {
                    handle: abi::Handle::from_raw(watch.handle()),
                    event: watch.filter(),
                    _pad: [0; 3],
                };
                event_count += 1;
            }
        }

        (events, event_count)
    });

    let (events, event_count) = match repoll_result {
        Ok((e, c)) => (e, c),
        Err(_) => return KernelError::BadHandle.to_errno(),
    };

    if event_count > 0 {
        return copy_mux_events_to_user(buf_ptr, &events, event_count);
    }

    // No events after wake — distinguish timeout from spurious wake
    if timeout_ns > 0 && crate::platform::current::timer::is_expired(earliest_deadline) {
        return KernelError::Timeout.to_errno();
    }
    KernelError::WouldBlock.to_errno()
}

// ── Mux helpers ──

/// Copy MuxEvents to user buffer. Returns event_count as i64.
fn copy_mux_events_to_user(buf_ptr: u64, events: &[super::MuxEvent; super::MAX_MUX_EVENTS], count: usize) -> i64 {
    let bytes = count * core::mem::size_of::<super::MuxEvent>();
    let event_bytes = unsafe {
        core::slice::from_raw_parts(events.as_ptr() as *const u8, bytes)
    };
    if uaccess::copy_to_user(buf_ptr, event_bytes).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    count as i64
}

/// Poll a watched object (immutable, for late-poll race check).
fn poll_watched_object(obj: &Object, filter: u8, channel_id: u32) -> bool {
    use super::types::filter as f;

    match obj {
        Object::Channel(_) => {
            let is_closed = channel_id == 0
                || !ipc::channel_exists(channel_id)
                || ipc::channel_is_closed(channel_id);
            let has_messages = channel_id != 0 && ipc::channel_has_messages(channel_id);
            ((filter & f::CLOSED) != 0 && is_closed)
                || ((filter & f::READABLE) != 0 && (has_messages || is_closed))
        }
        Object::Port(p) => {
            (filter & f::READABLE) != 0 && ipc::port_has_pending(p.port_id())
        }
        Object::Timer(t) => {
            if (filter & f::READABLE) != 0 {
                t.state() == super::TimerState::Fired
                    || (t.state() == super::TimerState::Armed
                        && crate::platform::current::timer::is_expired(t.deadline()))
            } else {
                false
            }
        }
        Object::Console(c) => {
            match c.console_type() {
                ConsoleType::Stdin => (filter & f::READABLE) != 0 && uart::rx_buffer_has_data(),
                ConsoleType::Stdout | ConsoleType::Stderr => (filter & f::WRITABLE) != 0,
            }
        }
        Object::Process(p) => {
            (filter & f::READABLE) != 0 && p.poll(f::READABLE).is_ready()
        }
        Object::Ring(r) => {
            r.poll(filter).is_ready()
        }
        _ => false,
    }
}

/// Poll a watched object (mutable, can consume timer/shmem state).
///
/// For Timer and Shmem objects, this consumes the ready state (fires/acknowledges).
/// For all other object types, delegates to the immutable poll_watched_object().
fn poll_watched_object_mut(entry: &mut super::handle::HandleEntry, filter: u8, channel_id: u32, current_tick: u64) -> bool {
    use super::types::filter as f;

    match &mut entry.object {
        Object::Timer(t) => {
            if (filter & f::READABLE) != 0 {
                let state = t.state();
                if state == super::TimerState::Fired {
                    t.consume(current_tick);
                    true
                } else if state == super::TimerState::Armed && crate::platform::current::timer::is_expired(t.deadline()) {
                    t.fire();
                    t.consume(current_tick);
                    true
                } else {
                    false
                }
            } else {
                false
            }
        }
        Object::Shmem(s) => {
            if (filter & f::READABLE) != 0 && s.is_notified() {
                s.acknowledge();
                true
            } else {
                false
            }
        }
        _ => poll_watched_object(&entry.object, filter, channel_id),
    }
}

// Write implementations
/// Pending wake action from write_channel, processed after table lock is released.
/// This avoids deadlocking when peer is in the same process (same per-task table lock).
struct ChannelWriteResult {
    errno: i64,
    peer: Option<ipc::PeerInfo>,
    is_bus_channel: bool,
    bus_data: Option<([u8; ipc::MAX_INLINE_PAYLOAD], usize, u32)>, // (buf, len, channel_id)
}

fn write_channel(ch: &mut super::ChannelObject, buf_ptr: u64, buf_len: usize) -> ChannelWriteResult {
    let fail = |e: i64| ChannelWriteResult { errno: e, peer: None, is_bus_channel: false, bus_data: None };

    // Check state - can only write to Open channel (queries ipc backend)
    if !ch.is_open() {
        return fail(KernelError::PeerClosed.to_errno());
    }

    let channel_id = ch.channel_id();
    if channel_id == 0 {
        return fail(KernelError::PeerClosed.to_errno());
    }

    // Limit message size
    if buf_len > ipc::MAX_INLINE_PAYLOAD {
        return fail(KernelError::InvalidArg.to_errno());
    }

    let pid = task::with_scheduler(|sched| {
        match sched.current_task() {
            Some(t) => t.id,
            None => 0,
        }
    });
    if pid == 0 {
        return fail(KernelError::BadHandle.to_errno());
    }

    // Copy from user buffer (use MAX_INLINE_PAYLOAD to support block transfers)
    let mut kernel_buf = [0u8; ipc::MAX_INLINE_PAYLOAD];
    let copy_len = core::cmp::min(buf_len, kernel_buf.len());
    if uaccess::copy_from_user(&mut kernel_buf[..copy_len], buf_ptr).is_err() {
        return fail(KernelError::BadAddress.to_errno());
    }

    // Create message and send via ipc
    let msg = ipc::Message::data(pid, &kernel_buf[..copy_len]);

    // Check before send — avoids re-acquiring channel table lock after send
    let is_bus_channel = ipc::is_kernel_dispatch(channel_id);

    match ipc::send(channel_id, msg, pid) {
        Ok(peer) => {
            // Return peer info for deferred wake (after table lock is released)
            let bus_data = if is_bus_channel {
                Some((kernel_buf, copy_len, channel_id))
            } else {
                None
            };
            ChannelWriteResult {
                errno: copy_len as i64,
                peer: Some(peer),
                is_bus_channel,
                bus_data,
            }
        }
        Err(ipc::IpcError::QueueFull) => fail(KernelError::WouldBlock.to_errno()),
        Err(ipc::IpcError::PeerClosed) | Err(ipc::IpcError::Closed) => {
            // Backend already knows about peer close
            fail(KernelError::PeerClosed.to_errno())
        }
        Err(_) => fail(KernelError::BadHandle.to_errno()),
    }
}

fn write_timer(t: &mut super::TimerObject, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::platform::current::timer;

    // Format: [deadline_ns: u64] or [deadline_ns: u64, interval_ns: u64]
    if buf_len < 8 {
        return KernelError::InvalidArg.to_errno();
    }

    let mut buf = [0u8; 16];
    let copy_len = buf_len.min(16);
    if uaccess::copy_from_user(&mut buf[..copy_len], buf_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }

    let deadline_ns = u64::from_le_bytes([buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]]);

    if deadline_ns == 0 {
        // Disarm timer using transition method
        t.disarm();
        return 0;
    }

    // Use unified clock API to create deadline
    let deadline = timer::deadline_ns(deadline_ns);

    // Check for interval (recurring timer)
    let freq = timer::frequency();
    let interval = if buf_len >= 16 && freq > 0 {
        let interval_ns = u64::from_le_bytes([buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]]);
        if interval_ns > 0 {
            // Convert interval to counter ticks (same unit as deadline)
            ((interval_ns as u128 * freq as u128) / 1_000_000_000) as u64
        } else {
            0
        }
    } else {
        0
    };

    // Arm timer using transition method
    t.arm(deadline, interval);

    // Notify scheduler about this deadline so it wakes us when timer fires
    task::with_scheduler(|sched| {
        sched.note_deadline(deadline);
    });

    0
}

fn write_shmem(s: &mut super::ShmemObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    // Write commands:
    // - cmd=0 + peer_pid:u32 = ALLOW (grant access to peer)
    // - cmd=1 = NOTIFY (wake waiters)
    if buf_len < 1 {
        return KernelError::InvalidArg.to_errno();
    }

    let mut cmd_byte = [0u8; 1];
    if uaccess::copy_from_user(&mut cmd_byte, buf_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }

    match cmd_byte[0] {
        0 => {
            // ALLOW: need peer_pid
            if buf_len < 5 {
                return KernelError::InvalidArg.to_errno();
            }
            let mut pid_bytes = [0u8; 4];
            if uaccess::copy_from_user(&mut pid_bytes, buf_ptr + 1).is_err() {
                return KernelError::BadAddress.to_errno();
            }
            let peer_pid = u32::from_le_bytes(pid_bytes);

            match shmem::allow(task_id, s.shmem_id(), peer_pid) {
                Ok(()) => 0,
                Err(e) => e,
            }
        }
        1 => {
            // NOTIFY: wake waiters
            // NOTE: We return a special value to indicate notify_shmem_objects should be
            // called AFTER the ObjectService lock is released. The actual notification
            // is done by shmem::notify() and the return value is handled by the caller.
            let shmem_id = s.shmem_id();

            let result = shmem::notify(task_id, shmem_id);

            // Return result with high bit set to indicate pending notify_shmem_objects
            // We encode: (shmem_id << 32) | result, but since we return i64,
            // we use a simpler approach: return (result | PENDING_NOTIFY_FLAG)
            // and store shmem_id in the ShmemObject for later retrieval.
            // Actually, we'll use a different approach: write_shmem_notify returns
            // both the result AND the shmem_id, and the caller handles notify_shmem_objects.
            match result {
                Ok(woken) => {
                    // Encode: positive result in low bits, shmem_id in high bits
                    // We return a special struct or just return the result and let caller
                    // do the notify based on checking if it was a NOTIFY command.
                    // Simplest: just return the result; caller checks if buf_len==1 && cmd==1
                    woken as i64
                }
                Err(e) => e,
            }
        }
        2 => {
            // SET_PUBLIC: make region accessible by any process
            match shmem::set_public(task_id, s.shmem_id()) {
                Ok(()) => 0,
                Err(e) => e,
            }
        }
        _ => KernelError::InvalidArg.to_errno(),
    }
}

/// Mark ShmemObjects as notified for mux integration
/// Called when shmem::notify() is invoked - sets notified flag on all tasks' ShmemObjects
/// (except the caller) and wakes their mux subscribers
fn notify_shmem_objects(shmem_id: u32, caller_pid: u32) {
    use crate::kernel::object_service::object_service;
    use crate::kernel::ipc::traits::Subscriber;

    // Get the list of pids that have access to this shmem
    let allowed_pids = shmem::mapping_get_pids(shmem_id);

    // Collect subscribers to wake (outside lock)
    let mut to_wake: [Option<Subscriber>; 8] = [None; 8];
    let mut wake_count = 0;

    // For each allowed pid (except caller), find their ShmemObject and set notified
    for pid in allowed_pids {
        if pid == 0 || pid == caller_pid {
            continue;
        }

        let sub = object_service().with_table_mut(pid, |table| {
            // Find ShmemObject with matching shmem_id
            // NOTE: Must use iter_mut() instead of manual Handle::from_raw() loop,
            // because from_raw(i) creates handle with generation=0, but entries
            // have generation>=1, so get_mut() always fails due to generation mismatch.
            for (_handle, entry) in table.iter_mut() {
                if let Object::Shmem(ref mut s) = entry.object {
                    if s.shmem_id() == shmem_id {
                        s.notify();
                        return s.mux_subscriber();
                    }
                }
            }
            None
        });

        match &sub {
            Ok(Some(s)) => {
                if wake_count < to_wake.len() {
                    to_wake[wake_count] = Some(*s);
                    wake_count += 1;
                }
            }
            Ok(None) | Err(_) => {}
        }
    }

    // Wake subscribers outside the table lock
    for sub_opt in &to_wake {
        if let Some(sub) = sub_opt {
            task::with_scheduler(|sched| {
                if let Some(slot) = sched.slot_by_pid(sub.task_id) {
                    let _ = sched.wake_task(slot);
                }
            });
        }
    }
}

/// Mark a specific task's ShmemObject as notified
/// Used by shmem cleanup to notify owner when a mapper dies
///
/// NOTE: Does NOT wake the task directly to avoid deadlock when called from
/// reap_terminated (which holds scheduler lock). The task will be woken by
/// event delivery or its next mux poll.
pub fn mark_shmem_notified_for_task(task_pid: u32, shmem_id: u32) {
    use crate::kernel::object_service::object_service;

    let _ = object_service().with_table_mut(task_pid, |table| {
        // Find ShmemObject with matching shmem_id
        for (_handle, entry) in table.iter_mut() {
            if let Object::Shmem(ref mut s) = entry.object {
                if s.shmem_id() == shmem_id {
                    s.notify();
                    return;
                }
            }
        }
    });
}

fn write_console(c: &mut super::ConsoleObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Only stdout and stderr are writable
    match c.console_type {
        ConsoleType::Stdout | ConsoleType::Stderr => {
            if buf_len == 0 {
                return 0;
            }
            // Limit buffer size to 512 bytes per write to keep stack usage bounded.
            // Userspace can issue multiple writes for larger outputs.
            let write_len = buf_len.min(512);
            let mut kernel_buf = [0u8; 512];
            if uaccess::copy_from_user(&mut kernel_buf[..write_len], buf_ptr).is_err() {
                return KernelError::BadAddress.to_errno();
            }
            // Write to UART and flush
            uart::write_buffered(&kernel_buf[..write_len]);
            uart::flush_buffer();
            write_len as i64
        }
        ConsoleType::Stdin => KernelError::NotSupported.to_errno(),
    }
}

fn write_mux(m: &mut super::MuxObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Format: [op:1][filter:1][pad:2][handle:4]
    // op: 0=Add, 1=Remove
    // filter: 0=Readable, 1=Writable, 2=Closed
    if buf_len < 8 {
        return KernelError::InvalidArg.to_errno();
    }

    let mut buf = [0u8; 8];
    if uaccess::copy_from_user(&mut buf, buf_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }

    let op = buf[0];
    let filter = buf[1];
    let handle = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);

    match op {
        0 => {
            // Add watch
            for slot in &mut m.watches {
                if slot.is_none() {
                    *slot = Some(super::MuxWatch { handle, filter });
                    return 0;
                }
            }
            KernelError::NoSpace.to_errno()
        }
        1 => {
            // Remove watch
            for slot in &mut m.watches {
                if let Some(w) = slot {
                    if w.handle == handle {
                        *slot = None;
                        return 0;
                    }
                }
            }
            KernelError::NotFound.to_errno()
        }
        2 => {
            // Set timeout: [op:1][pad:3][timeout_ms:4]
            let timeout_ms = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
            m.set_timeout_ns(if timeout_ms == 0 { 0 } else { timeout_ms as u64 * 1_000_000 });
            0
        }
        _ => KernelError::InvalidArg.to_errno(),
    }
}

fn write_pci_device(d: &mut super::PciDeviceObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    use crate::kernel::pci::{self, PciBdf};

    // Params: offset:u16 (2) + size:u8 (1) + value:u32 (4) = 7 bytes
    if buf_len < 7 {
        return KernelError::InvalidArg.to_errno();
    }

    let mut params = [0u8; 7];
    if uaccess::copy_from_user(&mut params, buf_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let offset = u16::from_le_bytes([params[0], params[1]]);
    let size = params[2];
    let value = u32::from_le_bytes([params[3], params[4], params[5], params[6]]);

    // Validate offset and size
    const PCI_CONFIG_MAX: u16 = 4096;
    if offset >= PCI_CONFIG_MAX || (size != 1 && size != 2 && size != 4) {
        return KernelError::InvalidArg.to_errno();
    }
    if (offset as u32) + (size as u32) > PCI_CONFIG_MAX as u32 {
        return KernelError::InvalidArg.to_errno();
    }
    // Check alignment
    if (size == 2 && (offset & 1) != 0) || (size == 4 && (offset & 3) != 0) {
        return KernelError::InvalidArg.to_errno();
    }

    let bdf = PciBdf::from_u32(d.bdf());

    // Check device exists and caller owns it
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return KernelError::NotFound.to_errno(),
    };

    if dev.owner() != task_id {
        return KernelError::PermDenied.to_errno();
    }

    // For partial writes, do read-modify-write
    let aligned_offset = offset & !0x3;

    let result = if size == 4 {
        pci::config_write32(bdf, aligned_offset, value)
    } else {
        // Read current value
        let current = match pci::config_read32(bdf, aligned_offset) {
            Ok(v) => v,
            Err(_) => return KernelError::Io.to_errno(),
        };

        let (mask, shift) = match size {
            1 => (0xFFu32, ((offset & 3) * 8) as u32),
            2 => (0xFFFFu32, ((offset & 2) * 8) as u32),
            _ => return KernelError::InvalidArg.to_errno(),
        };

        let new_val = (current & !(mask << shift)) | ((value & mask) << shift);
        pci::config_write32(bdf, aligned_offset, new_val)
    };

    match result {
        Ok(()) => 0,
        Err(_) => KernelError::Io.to_errno(),
    }
}

// Map implementations
fn map_shmem(s: &mut super::ShmemObject, task_id: u32) -> i64 {
    // If already mapped, return the existing vaddr
    if s.vaddr() != 0 {
        return s.vaddr() as i64;
    }

    // Map the region (shouldn't normally happen since open() already maps)
    match shmem::map(task_id, s.shmem_id()) {
        Ok((vaddr, _paddr)) => {
            s.set_vaddr(vaddr);
            vaddr as i64
        }
        Err(e) => e,
    }
}

fn map_dma_pool(d: &mut super::DmaPoolObject, _task_id: u32) -> i64 {
    // DMA pool is already mapped during open(), return the vaddr
    // Use read() on the handle to get the physical address
    d.vaddr() as i64
}

fn map_mmio(m: &mut super::MmioObject, _task_id: u32) -> i64 {
    // MMIO is already mapped during open(), just return the vaddr
    m.vaddr() as i64
}

// Close implementations
fn close_channel(ch: super::ChannelObject, owner: u32) {
    // Only close if not already closed (queries ipc backend)
    if !ch.is_closed() && ch.channel_id() != 0 {
        if let Ok(peer_opt) = ipc::close_channel(ch.channel_id(), owner) {
            // Wake peer's ChannelObject via ObjectService WaitQueue
            if let Some(peer) = peer_opt {
                let wake_list = object_service().wake_channel(
                    peer.task_id, peer.channel_id, super::poll::CLOSED,
                );
                waker::wake(&wake_list, ipc::WakeReason::Closed);
            }
        }
    }
}

fn close_port(p: super::PortObject, owner: u32) {
    // Only close if listening
    if p.is_listening() && p.port_id() != 0 {
        if let Ok(wake_list) = ipc::port_unregister(p.port_id(), owner) {
            // Wake any subscribers
            waker::wake(&wake_list, ipc::WakeReason::Closed);
        }
    }
}

fn close_shmem(s: super::ShmemObject, owner: u32) {
    // Unmap from caller's address space if mapped
    if s.vaddr() != 0 {
        let _ = shmem::unmap(owner, s.shmem_id());
    }
    // Destroy the region (shmem module handles reference counting)
    let _ = shmem::destroy(s.shmem_id(), owner);
}

fn close_dma_pool(d: super::DmaPoolObject, owner: u32) {
    // DMA pool uses bump allocation - no free function
    // Memory is reclaimed when process exits and page tables are freed
    // The physical DMA memory is not freed (kernel pool is long-lived)
    let _ = (d, owner);
}

fn close_mmio(m: super::MmioObject, owner: u32) {
    // MMIO mappings are part of the process's address space
    // They are unmapped when the process exits and page tables are freed
    // No explicit unmap function - device memory is usually needed for process lifetime
    let _ = (m, owner);
}

// ============================================================================
// Ring IPC (high-performance typed messages)
// ============================================================================

/// Open a new Ring
///
/// Params format: RingParams (4 bytes: tx_config:u16, rx_config:u16)
/// Returns: handle (positive) or error (negative)
fn open_ring(params_ptr: u64, params_len: usize) -> i64 {
    use crate::kernel::object_service::object_service;

    // Params: RingParams (tx: u16, rx: u16) = 4 bytes
    if params_len != 4 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy params from user
    let mut params = [0u8; 4];
    if uaccess::copy_from_user(&mut params, params_ptr).is_err() {
        return KernelError::BadAddress.to_errno();
    }
    let tx_config = u16::from_le_bytes([params[0], params[1]]);
    let rx_config = u16::from_le_bytes([params[2], params[3]]);

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return KernelError::BadHandle.to_errno();
    };

    // Calculate total size: header (64) + tx_ring + rx_ring
    let tx_size = abi::ring_config::ring_size(tx_config);
    let rx_size = abi::ring_config::ring_size(rx_config);
    let total_size = 64 + tx_size + rx_size;

    // Allocate physical memory for the ring using shmem
    // shmem::create returns (shmem_id, vaddr, paddr)
    let (shmem_id, _vaddr, paddr) = match shmem::create(task_id, total_size) {
        Ok(result) => result,
        Err(_) => return KernelError::NoSpace.to_errno(),
    };

    // Create RingObject and add to handle table
    let mut ring = super::RingObject::new_creator(paddr, total_size, tx_config, rx_config);
    ring.set_shmem_id(shmem_id);

    match object_service().open_ring(task_id, ring, shmem_id) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => {
            let _ = shmem::destroy(task_id, shmem_id);
            e.to_errno()
        }
    }
}

/// Read from Ring - wait for data to be available
///
/// For Ring objects, read() is used for synchronization:
/// - Blocks until TX ring has data (or peer writes something)
/// - Returns immediately if data is already available
///
/// buf_ptr: ignored (userspace reads ring directly after mapping)
/// buf_len: ignored
///
/// Returns: 0 on success, negative error
fn read_ring(r: &mut super::RingObject, _buf_ptr: u64, _buf_len: usize, task_id: u32) -> i64 {
    use crate::kernel::ipc::traits::Subscriber;

    // If not mapped, can't be ready
    if r.vaddr() == 0 {
        return KernelError::NotSupported.to_errno();
    }

    // Ring is always "ready" - userspace manages head/tail
    // The read() syscall is only for waking up blocked readers
    // Subscribe for notification if peer writes
    let sub = Subscriber::simple(task_id);
    r.subscribe(sub);

    0 // Success - userspace will read ring directly
}

/// Write to Ring - notify peer
///
/// For Ring objects, write() is used to signal the peer:
/// - After writing data to TX ring, call write() to wake peer
/// - buf contains optional notification data (usually empty)
///
/// Returns: 0 on success, negative error
fn write_ring(r: &mut super::RingObject, _buf_ptr: u64, _buf_len: usize, _task_id: u32) -> i64 {
    use crate::kernel::ipc::waker::WakeList;

    // Wake peer if subscribed
    if let Some(sub) = r.subscriber() {
        let mut wake_list = WakeList::new();
        wake_list.push(sub);
        waker::wake(&wake_list, ipc::WakeReason::Readable);
    }

    0 // Success
}

/// Map Ring shared memory into task's address space
fn map_ring(r: &mut super::RingObject, _task_id: u32) -> i64 {
    // Already mapped?
    if r.vaddr() != 0 {
        return r.vaddr() as i64;
    }

    // Map the ring's backing shared memory
    let paddr = r.paddr();
    let size = r.size();

    // Use mmap_shmem_dma to map the physical address into userspace (non-cacheable)
    let vaddr = match task::with_scheduler(|sched| {
        let slot = task::current_slot();
        if let Some(task) = sched.task_mut(slot) {
            task.mmap_shmem_dma(paddr, size)
        } else {
            None
        }
    }) {
        Some(v) if v != 0 => v,
        _ => return KernelError::NoSpace.to_errno(),
    };

    r.set_vaddr(vaddr);

    // Initialize the RingHeader if we're the creator
    if r.is_creator() {
        // Zero-initialize the header
        unsafe {
            let header_ptr = vaddr as *mut u8;
            core::ptr::write_bytes(header_ptr, 0, 64);

            // Set the config fields
            let header = vaddr as *mut abi::RingHeader;
            (*header).tx_config = r.tx_config();
            (*header).rx_config = r.rx_config();
        }
    }

    vaddr as i64
}

/// Close Ring and cleanup
fn close_ring(r: super::RingObject, owner: u32) {
    // If mapped, unmap from caller's address space
    if r.vaddr() != 0 {
        task::with_scheduler(|sched| {
            let slot = task::current_slot();
            if let Some(task) = sched.task_mut(slot) {
                task.munmap(r.vaddr(), r.size());
            }
        });
    }

    // Wake any subscribers
    if let Some(sub) = r.subscriber() {
        let mut wake_list = waker::WakeList::new();
        wake_list.push(sub);
        waker::wake(&wake_list, ipc::WakeReason::Closed);
    }

    // Free the backing shmem region if this is the creator side
    if r.is_creator() && r.shmem_id() != 0 {
        let _ = shmem::destroy(r.shmem_id(), owner);
    }
}
