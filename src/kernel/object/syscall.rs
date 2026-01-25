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

use super::{Object, ObjectType, Handle, ConsoleType, Pollable, HasSubscriber};
use super::types::Error;
use crate::kernel::task;
use crate::kernel::uaccess;
use crate::platform::current::uart;
use crate::kernel::ipc;
use crate::kernel::ipc::waker;
use crate::kernel::shmem;

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
        return Error::InvalidArg.to_errno();
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
        return Error::BadHandle.to_errno();
    };

    // Use ObjectService's tables for dispatch
    let result = object_service().with_table_mut(task_id, |table| {
        // Check handle validity and get type
        let Some(entry) = table.get(handle) else {
            return Err(Error::BadHandle);
        };
        if !entry.object_type.is_readable() {
            return Err(Error::NotSupported);
        }
        Ok(entry.object_type)
    });

    let obj_type = match result {
        Ok(Ok(t)) => t,
        Ok(Err(e)) => return e.to_errno(),
        Err(_) => return Error::BadHandle.to_errno(),
    };

    // Handle Mux specially - needs access to multiple handles
    if obj_type == ObjectType::Mux {
        return read_mux_via_service(task_id, handle, buf_ptr, buf_len);
    }

    // Handle Port specially - needs to allocate a new handle (can't nest locks)
    if obj_type == ObjectType::Port {
        return read_port_via_service(task_id, handle, buf_ptr, buf_len);
    }

    // For other types, get mutable ref and dispatch
    let result = object_service().with_table_mut(task_id, |table| {
        let Some(entry) = table.get_mut(handle) else {
            return Error::BadHandle.to_errno();
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
            _ => Error::NotSupported.to_errno(),
        }
    });

    match result {
        Ok(errno) => errno,
        Err(_) => Error::BadHandle.to_errno(),
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
        return Error::BadHandle.to_errno();
    };

    // Use ObjectService's tables for dispatch
    let result = object_service().with_table_mut(task_id, |table| {
        let Some(entry) = table.get_mut(handle) else {
            return Error::BadHandle.to_errno();
        };

        if !entry.object_type.is_writable() {
            return Error::NotSupported.to_errno();
        }

        // Dispatch to type-specific write
        match &mut entry.object {
            Object::Channel(ch) => write_channel(ch, buf_ptr, buf_len),
            Object::Timer(t) => write_timer(t, buf_ptr, buf_len),
            Object::Shmem(s) => write_shmem(s, buf_ptr, buf_len, task_id),
            Object::Console(c) => write_console(c, buf_ptr, buf_len),
            Object::Mux(m) => write_mux(m, buf_ptr, buf_len),
            Object::PciDevice(d) => write_pci_device(d, buf_ptr, buf_len, task_id),
            _ => Error::NotSupported.to_errno(),
        }
    });

    match result {
        Ok(errno) => errno,
        Err(_) => Error::BadHandle.to_errno(),
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
        return Error::BadHandle.to_errno();
    };

    // Use ObjectService's tables for dispatch
    let result = object_service().with_table_mut(task_id, |table| {
        let Some(entry) = table.get_mut(handle) else {
            return Error::BadHandle.to_errno();
        };

        if !entry.object_type.is_mappable() {
            return Error::NotSupported.to_errno();
        }

        // Dispatch to type-specific map
        match &mut entry.object {
            Object::Shmem(s) => map_shmem(s, task_id),
            Object::DmaPool(d) => map_dma_pool(d, task_id),
            Object::Mmio(m) => map_mmio(m, task_id),
            _ => Error::NotSupported.to_errno(),
        }
    });

    match result {
        Ok(errno) => errno,
        Err(_) => Error::BadHandle.to_errno(),
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
        None => return Error::BadHandle.to_errno(),
    };

    // Close handle via ObjectService
    let object = match object_service().close_handle(task_id, handle) {
        Ok(obj) => obj,
        Err(_) => return Error::BadHandle.to_errno(),
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
        return Error::BadHandle.to_errno();
    };

    // Check if this is a port connect (params contains port name)
    if params_len > 0 && params_len <= 32 {
        // Copy port name from user
        let mut name_buf = [0u8; 32];
        if uaccess::copy_from_user(&mut name_buf[..params_len], params_ptr).is_err() {
            return Error::BadAddress.to_errno();
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
        return Error::BadHandle.to_errno();
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
        return Error::InvalidArg.to_errno();
    }

    // Copy PID from user
    let mut pid_bytes = [0u8; 4];
    if uaccess::copy_from_user(&mut pid_bytes, params_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }
    let target_pid = u32::from_le_bytes(pid_bytes);

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return Error::BadHandle.to_errno();
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
        return Error::InvalidArg.to_errno();
    }

    // Copy port name from user
    let mut name_buf = [0u8; 32];
    if uaccess::copy_from_user(&mut name_buf[..params_len], params_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }

    // Get current task ID
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return Error::BadHandle.to_errno();
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
        _ => Error::InvalidArg.to_errno(),
    }
}

fn open_shmem_create(params_ptr: u64) -> i64 {
    // Copy size from user
    let mut size_bytes = [0u8; 8];
    if uaccess::copy_from_user(&mut size_bytes, params_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }
    let size = u64::from_le_bytes(size_bytes) as usize;

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return Error::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_shmem_create(task_id, size) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_shmem_existing(params_ptr: u64) -> i64 {
    // Copy shmem_id from user
    let mut id_bytes = [0u8; 4];
    if uaccess::copy_from_user(&mut id_bytes, params_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }
    let shmem_id = u32::from_le_bytes(id_bytes);

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return Error::BadHandle.to_errno();
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
    if params_len != 16 {
        return Error::InvalidArg.to_errno();
    }

    // Copy params from user
    let mut params = [0u8; 16];
    if uaccess::copy_from_user(&mut params, params_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }
    // Use array indexing (infallible for known-size arrays)
    let size = u64::from_le_bytes([
        params[0], params[1], params[2], params[3],
        params[4], params[5], params[6], params[7],
    ]) as usize;
    let flags = u32::from_le_bytes([params[8], params[9], params[10], params[11]]);
    let use_high = (flags & 1) != 0;

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return Error::BadHandle.to_errno();
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_dma_pool(task_id, size, use_high) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_mmio(params_ptr: u64, params_len: usize) -> i64 {
    // Params: u64 phys_addr, u64 size (16 bytes)
    if params_len != 16 {
        return Error::InvalidArg.to_errno();
    }

    // Copy params from user
    let mut params = [0u8; 16];
    if uaccess::copy_from_user(&mut params, params_ptr).is_err() {
        return Error::BadAddress.to_errno();
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
        return Error::BadHandle.to_errno();
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
        return Error::BadHandle.to_errno();
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
        return Error::BadHandle.to_errno();
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
        return Error::BadHandle.to_errno();
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
            return Error::BadAddress.to_errno();
        }
        u32::from_le_bytes(bdf_bytes)
    } else {
        return Error::InvalidArg.to_errno();
    };

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return Error::BadHandle.to_errno();
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
        return Error::InvalidArg.to_errno();
    }

    // Require RAW_DEVICE capability
    if let Err(e) = super::super::syscall::require_capability(Capabilities::RAW_DEVICE) {
        return e;
    }

    let mut bdf_bytes = [0u8; 4];
    if uaccess::copy_from_user(&mut bdf_bytes, params_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }
    let bdf = u32::from_le_bytes(bdf_bytes);

    // Check device exists
    let bdf_typed = PciBdf::from_u32(bdf);
    if pci::find_by_bdf(bdf_typed).is_none() {
        return Error::NotFound.to_errno();
    }

    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return Error::BadHandle.to_errno();
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
        return Error::InvalidArg.to_errno();
    }

    // Require IRQ_CLAIM capability
    if let Err(e) = super::super::syscall::require_capability(Capabilities::IRQ_CLAIM) {
        return e;
    }

    let mut params = [0u8; 5];
    if uaccess::copy_from_user(&mut params, params_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }
    let bdf = u32::from_le_bytes([params[0], params[1], params[2], params[3]]);
    let count = params[4];

    let bdf_typed = PciBdf::from_u32(bdf);

    // Check device exists and caller owns it
    let dev = match pci::find_by_bdf(bdf_typed) {
        Some(d) => d,
        None => return Error::NotFound.to_errno(),
    };

    // Get current pid via safe scheduler access
    let task_id = task::with_scheduler(|sched| {
        sched.current_task().map(|t| t.id).unwrap_or(0)
    });

    if dev.owner() != task_id {
        return Error::PermDenied.to_errno();
    }

    // Check device supports MSI
    if !dev.has_msi() && !dev.has_msix() {
        return Error::NotSupported.to_errno();
    }

    // Allocate vectors
    let first_irq = match pci::msi_alloc(bdf_typed, count) {
        Ok(irq) => irq,
        Err(pci::PciError::NoMsiVectors) => return Error::NoSpace.to_errno(),
        Err(_) => return Error::Io.to_errno(),
    };

    // Delegate to ObjectService
    use crate::kernel::object_service::object_service;
    match object_service().open_msi(task_id, bdf, first_irq, count) {
        Ok(handle) => handle.raw() as i64,
        Err(e) => e.to_errno(),
    }
}

fn open_bus_list(_params_ptr: u64, _params_len: usize) -> i64 {
    let task_id = get_current_task_id();

    let Some(task_id) = task_id else {
        return Error::BadHandle.to_errno();
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
        return Error::PeerClosed.to_errno();
    }

    let channel_id = ch.channel_id();
    if channel_id == 0 {
        return Error::PeerClosed.to_errno();
    }

    let pid = task::with_scheduler(|sched| {
        match sched.current_task() {
            Some(t) => t.id,
            None => 0,
        }
    });
    if pid == 0 {
        return Error::BadHandle.to_errno();
    }

    // Receive message from ipc
    match ipc::receive(channel_id, pid) {
        Ok(msg) => {
            let payload = msg.payload_slice();
            let copy_len = core::cmp::min(payload.len(), buf_len);

            // Copy to user buffer
            if uaccess::copy_to_user(buf_ptr, &payload[..copy_len]).is_err() {
                return Error::BadAddress.to_errno();
            }

            copy_len as i64
        }
        Err(ipc::IpcError::WouldBlock) => Error::WouldBlock.to_errno(),
        Err(ipc::IpcError::PeerClosed) | Err(ipc::IpcError::Closed) => {
            // Backend already knows about peer close
            Error::PeerClosed.to_errno()
        }
        Err(_) => Error::BadHandle.to_errno(),
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
            Error::InvalidArg.to_errno()
        }
        TimerState::Armed => {
            if current_counter >= t.deadline() {
                // Timer has fired - use transition method
                let _ = t.fire();

                // Write timestamp to buffer if provided
                if buf_ptr != 0 && buf_len >= 8 {
                    let ts_bytes = current_counter.to_le_bytes();
                    if uaccess::copy_to_user(buf_ptr, &ts_bytes).is_err() {
                        return Error::BadAddress.to_errno();
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
                t.set_subscriber(Some(ipc::traits::Subscriber {
                    task_id,
                    generation: task::Scheduler::generation_from_pid(task_id),
                }));
                Error::WouldBlock.to_errno()
            }
        }
        TimerState::Fired => {
            // Already fired - return timestamp
            if buf_ptr != 0 && buf_len >= 8 {
                let ts_bytes = t.deadline().to_le_bytes();
                if uaccess::copy_to_user(buf_ptr, &ts_bytes).is_err() {
                    return Error::BadAddress.to_errno();
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
                return Error::BadAddress.to_errno();
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
                return Error::BadAddress.to_errno();
            }
        }
        return 4;
    }

    // Target still running - register subscriber and return WouldBlock
    p.subscriber = Some(Subscriber {
        task_id,
        generation: task::Scheduler::generation_from_pid(task_id),
    });
    Error::WouldBlock.to_errno()
}

/// Read from port via service - accepts connection and allocates channel handle
///
/// Must be called OUTSIDE any with_table_mut closure to avoid deadlock,
/// since allocating the channel handle requires taking the tables lock.
fn read_port_via_service(task_id: u32, handle: Handle, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::kernel::object_service::object_service;

    // Need at least 4 bytes for handle
    if buf_ptr == 0 || buf_len < 4 {
        return Error::InvalidArg.to_errno();
    }

    // Phase 1: Get port_id and check state (quick lock)
    let port_id = {
        let result = object_service().with_table(task_id, |table| {
            let Some(entry) = table.get(handle) else {
                return Err(Error::BadHandle);
            };
            match &entry.object {
                Object::Port(p) => {
                    if !p.is_listening() {
                        return Err(Error::PeerClosed);
                    }
                    Ok(p.port_id())
                }
                _ => Err(Error::BadHandle),
            }
        });
        match result {
            Ok(Ok(id)) => id,
            Ok(Err(e)) => return e.to_errno(),
            Err(_) => return Error::BadHandle.to_errno(),
        }
    };

    // Phase 2: Accept connection (no ObjectService lock held)
    let (server_channel, client_pid) = match ipc::port_accept(port_id, task_id) {
        Ok(result) => result,
        Err(ipc::IpcError::NoPending) => return Error::WouldBlock.to_errno(),
        Err(ipc::IpcError::Closed) => return Error::PeerClosed.to_errno(),
        Err(_) => return Error::BadHandle.to_errno(),
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
            return Error::NoSpace.to_errno();
        }
    };

    // Phase 4: Write results to user
    let handle_bytes = new_handle.raw().to_le_bytes();
    if uaccess::copy_to_user(buf_ptr, &handle_bytes).is_err() {
        return Error::BadAddress.to_errno();
    }

    if buf_len >= 8 {
        let pid_bytes = client_pid.to_le_bytes();
        if uaccess::copy_to_user(buf_ptr + 4, &pid_bytes).is_err() {
            return Error::BadAddress.to_errno();
        }
        return 8;
    }
    4 // Wrote 4 bytes (handle only)
}

// Note: Port reads are handled via read_port_via_service() to avoid nested lock issues
// when allocating the channel handle. See that function for the implementation.

fn read_shmem(s: &mut super::ShmemObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    // Read = WAIT operation
    // Params: optional timeout_ms (u32) in buffer, default 0 = infinite
    let timeout_ms = if buf_len >= 4 {
        let mut timeout_bytes = [0u8; 4];
        if uaccess::copy_from_user(&mut timeout_bytes, buf_ptr).is_err() {
            return Error::BadAddress.to_errno();
        }
        u32::from_le_bytes(timeout_bytes)
    } else {
        0 // Infinite wait
    };

    match shmem::wait(task_id, s.shmem_id(), timeout_ms) {
        Ok(()) => 0,
        Err(-11) => {
            // EAGAIN = blocked, pre-store success return value
            task::with_scheduler(|sched| {
                if let Some(task) = sched.current_task_mut() {
                    task.set_deferred_return(0);
                }
            });
            Error::WouldBlock.to_errno()
        }
        Err(e) => e,
    }
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
                return Error::WouldBlock.to_errno();
            }

            // Copy to user
            if uaccess::copy_to_user(buf_ptr, &kernel_buf[..bytes_read]).is_err() {
                return Error::BadAddress.to_errno();
            }

            bytes_read as i64
        }
        ConsoleType::Stdout | ConsoleType::Stderr => Error::NotSupported.to_errno(),
    }
}

fn read_klog(k: &mut super::KlogObject, buf_ptr: u64, buf_len: usize) -> i64 {
    let _ = (k, buf_ptr, buf_len);
    Error::NotSupported.to_errno()
}

fn read_mux(_m: &mut super::MuxObject, _buf_ptr: u64, _buf_len: usize, _task_id: u32) -> i64 {
    // This shouldn't be called - Mux is handled specially in read()
    Error::NotSupported.to_errno()
}

fn read_pci_bus(p: &mut super::PciBusObject, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::kernel::bus::{DeviceInfo, get_device_list, get_device_count};

    if buf_ptr == 0 {
        // Just return count
        return get_device_count() as i64;
    }

    let max = buf_len / core::mem::size_of::<DeviceInfo>();
    if max == 0 {
        return 0;
    }

    // Create temporary buffer (max 16 devices)
    let mut temp = [DeviceInfo::empty(); 16];
    let count = get_device_list(&mut temp[..max.min(16)]);

    // Filter by BDF if needed
    // NOTE: BDF filtering requires DeviceInfo to have a BDF field or
    // a separate PCIe-specific enumeration API. Currently, all devices
    // pass through regardless of filter. This is acceptable since:
    // - bdf_filter=0 is the common case (enumerate all)
    // - Userspace can filter results itself if needed
    let bdf_filter = p.bdf_filter();
    let filtered_count = if bdf_filter == 0 {
        count
    } else {
        // BDF filtering not implemented - would need DeviceInfo.bdf field
        // For now, just copy all devices and let userspace filter
        count
    };

    // Copy to userspace
    let copy_size = filtered_count * core::mem::size_of::<DeviceInfo>();
    let src_bytes = unsafe {
        core::slice::from_raw_parts(temp.as_ptr() as *const u8, copy_size)
    };
    if uaccess::copy_to_user(buf_ptr, src_bytes).is_err() {
        return Error::BadAddress.to_errno();
    }

    filtered_count as i64
}

fn read_pci_device(d: &mut super::PciDeviceObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    use crate::kernel::pci::{self, PciBdf};

    // Params: offset:u16 (2 bytes) + size:u8 (1 byte) = 3 bytes
    if buf_len < 3 {
        return Error::InvalidArg.to_errno();
    }

    let mut params = [0u8; 3];
    if uaccess::copy_from_user(&mut params, buf_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }
    let offset = u16::from_le_bytes([params[0], params[1]]);
    let size = params[2];

    // Validate offset and size
    const PCI_CONFIG_MAX: u16 = 4096;
    if offset >= PCI_CONFIG_MAX || (size != 1 && size != 2 && size != 4) {
        return Error::InvalidArg.to_errno();
    }
    if (offset as u32) + (size as u32) > PCI_CONFIG_MAX as u32 {
        return Error::InvalidArg.to_errno();
    }
    // Check alignment
    if (size == 2 && (offset & 1) != 0) || (size == 4 && (offset & 3) != 0) {
        return Error::InvalidArg.to_errno();
    }

    let bdf = PciBdf::from_u32(d.bdf());

    // Check device exists and caller has access
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return Error::NotFound.to_errno(),
    };

    // Check ownership (0 = unclaimed, anyone can read)
    let owner = dev.owner();
    if owner != 0 && owner != task_id {
        return Error::PermDenied.to_errno();
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
                _ => Error::InvalidArg.to_errno(),
            }
        }
        Err(_) => Error::Io.to_errno(),
    }
}

fn read_msi(m: &mut super::MsiObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Return MSI info: first_irq:u32 + count:u8 + bdf:u32 = 9 bytes
    if buf_len < 9 {
        return Error::InvalidArg.to_errno();
    }

    let mut out = [0u8; 9];
    out[0..4].copy_from_slice(&m.first_irq().to_le_bytes());
    out[4] = m.count();
    out[5..9].copy_from_slice(&m.bdf().to_le_bytes());

    if uaccess::copy_to_user(buf_ptr, &out).is_err() {
        return Error::BadAddress.to_errno();
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
        return Error::BadAddress.to_errno();
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
fn read_mux_via_service(task_id: crate::kernel::task::TaskId, mux_handle: Handle, buf_ptr: u64, buf_len: usize) -> i64 {
    use super::types::filter;
    use crate::kernel::ipc::traits::Subscriber;
    use crate::kernel::object_service::object_service;

    let slot = task::current_slot();
    let subscriber = Subscriber {
        task_id,
        generation: task::Scheduler::generation_from_pid(task_id),
    };

    // Main loop - keep trying until we have events
    loop {
        crate::kdebug!("mux", "poll_start"; task_id = task_id);

        // Phase 1: Collect watch list and channel IDs from ObjectService tables
        let watches: [(Option<super::MuxWatch>, u32); super::MAX_MUX_WATCHES] = {
            let result = object_service().with_table(task_id, |table| {
                let Some(mux_entry) = table.get(mux_handle) else {
                    return Err(Error::BadHandle);
                };
                let Object::Mux(ref mux) = mux_entry.object else {
                    return Err(Error::BadHandle);
                };

                let mut w = [(None, 0u32); super::MAX_MUX_WATCHES];
                for i in 0..super::MAX_MUX_WATCHES {
                    if let Some(watch) = mux.watches[i] {
                        let watched_handle = Handle::from_raw(watch.handle);
                        let channel_id = if let Some(entry) = table.get(watched_handle) {
                            if let Object::Channel(ch) = &entry.object { ch.channel_id() } else { 0 }
                        } else { 0 };
                        w[i] = (Some(watch), channel_id);
                    }
                }
                Ok(w)
            });

            match result {
                Ok(Ok(w)) => w,
                Ok(Err(e)) => return e.to_errno(),
                Err(_) => return Error::BadHandle.to_errno(),
            }
        };

        // Phase 2+3: Subscribe AND poll in single lock to avoid race
        //
        // CRITICAL: Subscribe and poll must happen atomically. If we subscribe,
        // release the lock, then poll separately, an event can arrive between
        // subscribe and poll. The waker will find our subscriber but we're not
        // blocked yet, so the wake fails. Then we poll, find nothing, and block
        // - but we already missed the wake.
        //
        // By doing both in a single locked section, either:
        // - The event arrives BEFORE we subscribe → poll will see it
        // - The event arrives AFTER we poll → we'll be blocked and wake succeeds
        let subscribe_and_poll_result = object_service().with_table_mut(task_id, |table| {
            // Subscribe to mux itself
            if let Some(mux_entry) = table.get_mut(mux_handle) {
                if let Object::Mux(ref mut mux) = mux_entry.object {
                    mux.set_subscriber(Some(subscriber));
                }
            }

            // Compute earliest timer deadline for tickless wake
            let mut earliest_deadline: u64 = u64::MAX;

            // Subscribe to each watched object
            for watch_opt in &watches {
                let Some(watch) = watch_opt.0 else { continue };
                let channel_id = watch_opt.1;
                let watched_handle = Handle::from_raw(watch.handle);

                if let Some(entry) = table.get_mut(watched_handle) {
                    match &mut entry.object {
                        Object::Channel(_) => {
                            if channel_id != 0 {
                                let _ = ipc::subscribe(channel_id, task_id, subscriber, ipc::WakeReason::Readable);
                            }
                        }
                        Object::Port(p) => { p.set_subscriber(Some(subscriber)); }
                        Object::Timer(t) => {
                            t.set_subscriber(Some(subscriber));
                            if t.deadline() > 0 && t.deadline() < earliest_deadline {
                                earliest_deadline = t.deadline();
                            }
                        }
                        Object::Console(c) => {
                            c.set_subscriber(Some(subscriber));
                            if matches!(c.console_type(), ConsoleType::Stdin) {
                                uart::block_for_input(task_id);
                            }
                        }
                        Object::Process(p) => { p.set_subscriber(Some(subscriber)); }
                        _ => {}
                    }
                }
            }

            // Now poll for ready events (still holding the lock)
            // We use mutable access so we can transition timer state when fired
            let mut events = [super::MuxEvent::empty(); super::MAX_MUX_EVENTS];
            let mut event_count = 0usize;
            let max_events = buf_len / core::mem::size_of::<super::MuxEvent>();
            let current_tick = crate::platform::current::timer::counter();

            for watch_opt in &watches {
                if event_count >= max_events { break; }
                let Some(watch) = watch_opt.0 else { continue };
                let channel_id = watch_opt.1;
                let watched_handle = Handle::from_raw(watch.handle);

                let Some(entry) = table.get_mut(watched_handle) else { continue };

                let ready = match &mut entry.object {
                    Object::Channel(_) => {
                        if (watch.filter & filter::READABLE) != 0 {
                            channel_id != 0 && ipc::channel_has_messages(channel_id)
                        } else if (watch.filter & filter::CLOSED) != 0 {
                            channel_id == 0
                        } else {
                            false
                        }
                    }
                    Object::Port(p) => {
                        (watch.filter & filter::READABLE) != 0 && ipc::port_has_pending(p.port_id())
                    }
                    Object::Timer(t) => {
                        if (watch.filter & filter::READABLE) != 0 {
                            // Check timer state:
                            // - Armed + expired: transition to Fired and return ready
                            // - Fired: already fired (by check_timeouts), consume and return ready
                            // - Disarmed or Armed but not expired: not ready
                            let state = t.state();
                            if state == super::TimerState::Fired {
                                // Already fired by check_timeouts - consume and return ready
                                t.consume(current_tick);
                                true
                            } else if state == super::TimerState::Armed && crate::platform::current::timer::is_expired(t.deadline()) {
                                // Fire now and consume
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
                    Object::Console(c) => {
                        match c.console_type {
                            ConsoleType::Stdin => (watch.filter & filter::READABLE) != 0 && uart::rx_buffer_has_data(),
                            ConsoleType::Stdout | ConsoleType::Stderr => (watch.filter & filter::WRITABLE) != 0,
                        }
                    }
                    Object::Process(p) => {
                        (watch.filter & filter::READABLE) != 0 && p.poll(filter::READABLE).is_ready()
                    }
                    _ => false,
                };

                if ready {
                    events[event_count] = super::MuxEvent {
                        handle: abi::Handle::from_raw(watch.handle),
                        event: watch.filter,
                        _pad: [0; 3],
                    };
                    event_count += 1;
                }
            }

            (earliest_deadline, events, event_count)
        });

        let (earliest_deadline, events, event_count) = match subscribe_and_poll_result {
            Ok((d, e, c)) => (d, e, c),
            Err(_) => return Error::BadHandle.to_errno(),
        };

        // Phase 4: Events ready → return to userspace
        if event_count > 0 {
            // Clear stdin blocked registration so next poll can re-register
            uart::clear_blocked_if_pid(task_id);

            let bytes = event_count * core::mem::size_of::<super::MuxEvent>();
            let event_bytes = unsafe {
                core::slice::from_raw_parts(events.as_ptr() as *const u8, bytes)
            };
            if uaccess::copy_to_user(buf_ptr, event_bytes).is_err() {
                return Error::BadAddress.to_errno();
            }
            return event_count as i64;
        }

        // Phase 5: Transition to blocked state
        //
        // RACE CONDITION: Between releasing the tables lock (above) and transitioning
        // to Sleeping, an event may arrive. The waker will call wake_task(), but since
        // we're still Running, it returns false (wake ignored). We then block and miss
        // the event.
        //
        // SOLUTION: After transitioning to Sleeping/Waiting, do one more poll. If events
        // are ready, wake ourselves (proper state machine transition: Sleeping -> Ready -> Running)
        // and return the events. This keeps all state changes going through the state machine.
        let transition_ok = task::with_scheduler(|sched| {
            let Some(task) = sched.task_mut(slot) else {
                return false;
            };

            // If task was preempted (Ready state), transition back to Running first.
            // This can happen if a timer interrupt preempted us between the poll and here.
            if *task.state() == task::TaskState::Ready {
                if task.set_running().is_err() {
                    crate::kerror!("mux", "set_running_failed"; task_id = task_id);
                    return false;
                }
            }

            let state_name = task.state().name();

            if earliest_deadline == u64::MAX {
                let result = task.set_sleeping(crate::kernel::task::SleepReason::EventLoop);
                if result.is_err() {
                    crate::kerror!("mux", "sleep_failed"; task_id = task_id, state = state_name);
                }
                result.is_ok()
            } else {
                if task.set_waiting(crate::kernel::task::WaitReason::TimedEvent, earliest_deadline).is_ok() {
                    sched.note_deadline(earliest_deadline);
                    true
                } else {
                    crate::kerror!("mux", "wait_failed"; task_id = task_id, state = state_name);
                    false
                }
            }
        });

        if !transition_ok {
            // State transition failed - shouldn't happen normally
            crate::kerror!("mux", "transition_failed"; task_id = task_id);
            return Error::InvalidArg.to_errno();
        }

        // Now we're in Sleeping/Waiting state. Poll once more to catch events
        // that arrived in the window between our poll and blocking.
        let late_poll_found_events = {
            let mut found = false;
            let _ = object_service().with_table(task_id, |table| {
                for watch_opt in &watches {
                    let Some(watch) = watch_opt.0 else { continue };
                    let channel_id = watch_opt.1;
                    let watched_handle = Handle::from_raw(watch.handle);
                    let Some(entry) = table.get(watched_handle) else { continue };

                    let ready = match &entry.object {
                        Object::Channel(_) => {
                            if (watch.filter & filter::READABLE) != 0 {
                                channel_id != 0 && ipc::channel_has_messages(channel_id)
                            } else if (watch.filter & filter::CLOSED) != 0 {
                                channel_id == 0
                            } else {
                                false
                            }
                        }
                        Object::Port(p) => {
                            (watch.filter & filter::READABLE) != 0 && ipc::port_has_pending(p.port_id())
                        }
                        Object::Timer(t) => {
                            // Check timer state - Fired or (Armed + expired) means ready
                            // (We don't consume here - just checking if there's an event to return)
                            if (watch.filter & filter::READABLE) != 0 {
                                let state = t.state();
                                state == super::TimerState::Fired ||
                                    (state == super::TimerState::Armed &&
                                     crate::platform::current::timer::is_expired(t.deadline()))
                            } else {
                                false
                            }
                        }
                        Object::Console(c) => {
                            match c.console_type {
                                ConsoleType::Stdin => (watch.filter & filter::READABLE) != 0 && uart::rx_buffer_has_data(),
                                ConsoleType::Stdout | ConsoleType::Stderr => (watch.filter & filter::WRITABLE) != 0,
                            }
                        }
                        Object::Process(p) => {
                            (watch.filter & filter::READABLE) != 0 && p.poll(filter::READABLE).is_ready()
                        }
                        _ => false,
                    };

                    if ready {
                        found = true;
                        break;
                    }
                }
                Ok::<(), Error>(())
            });
            found
        };

        // Check BOTH: late_poll found events OR task was woken (state changed)
        //
        // Race condition: A waker might wake us (Sleeping -> Ready) after we
        // transitioned to Sleeping but the late_poll didn't see the event data.
        // In that case, late_poll_found_events is false but task is no longer blocked.
        // We must check the task state to catch this case.
        //
        // We check !is_blocked() rather than == Ready because there's another race:
        // 1. We set Sleeping
        // 2. Shell sends message, wakes us (Sleeping -> Ready)
        // 3. Scheduler preempts, switches to us (Ready -> Running)
        // 4. We check state - it's Running, not Ready!
        // If we only checked == Ready, we'd miss this and go back to sleep.
        //
        // Note: We only reach here if set_sleeping/set_waiting succeeded (transition_ok),
        // so being unblocked means we were woken, not that the transition failed.
        let task_was_woken = task::with_scheduler(|sched| {
            sched.task(slot)
                .map(|t| !t.is_blocked())
                .unwrap_or(false)
        });

        if late_poll_found_events || task_was_woken {
            // Events arrived OR we were woken by someone else.
            // Transition to Running via proper state machine.
            task::with_scheduler(|sched| {
                if let Some(task) = sched.task_mut(slot) {
                    // If Sleeping/Waiting -> wake to Ready
                    // If already Ready -> this is a no-op (logged by macro)
                    crate::transition_or_log!(task, wake);
                    // Ready -> Running
                    crate::transition_or_evict!(task, set_running);
                }
            });
            // Loop back to poll and return the events
            continue;
        }

        // No late events and still blocked - safe to reschedule.
        //
        // When reschedule() is called with a blocked task:
        // - If another task is Ready, we do a kernel context switch to it
        // - If no tasks are Ready, we enter idle loop (WFI)
        // - Either way, when we return here, we've been woken and should poll again
        crate::kernel::sched::reschedule();

        // When we return here, we've been switched back to this task.
        // The scheduler's context_switch mechanism ensures we resume here.
        // Ensure we're Running (state machine: was Ready when scheduled back).
        let post_state = task::with_scheduler(|sched| {
            if let Some(task) = sched.task_mut(slot) {
                // After being switched back, task should be Running (set by scheduler)
                // or Ready (if just woken). Handle both cases.
                if *task.state() == task::TaskState::Ready {
                    crate::transition_or_evict!(task, set_running);
                }

                // Reset liveness state - this proves the task is alive.
                // Liveness uses implicit pong for EventLoop sleep: waking the task
                // and waiting for a syscall. But since we're in a blocking syscall
                // loop, we never make a NEW syscall - we just continue looping.
                // Reset liveness here to prove we're responsive.
                let current_tick = crate::platform::current::timer::logical_ticks();
                task.record_activity(current_tick);
                task.reset_liveness_if_implicit_pong();

                task.state().name()
            } else {
                "none"
            }
        });
        crate::kdebug!("mux", "woke_up"; task_id = task_id, state = post_state);
    }
}

// Write implementations
fn write_channel(ch: &mut super::ChannelObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Check state - can only write to Open channel (queries ipc backend)
    if !ch.is_open() {
        return Error::PeerClosed.to_errno();
    }

    let channel_id = ch.channel_id();
    if channel_id == 0 {
        return Error::PeerClosed.to_errno();
    }

    // Limit message size
    if buf_len > ipc::MAX_INLINE_PAYLOAD {
        return Error::InvalidArg.to_errno();
    }

    let pid = task::with_scheduler(|sched| {
        match sched.current_task() {
            Some(t) => t.id,
            None => 0,
        }
    });
    if pid == 0 {
        return Error::BadHandle.to_errno();
    }

    // Copy from user buffer
    let mut kernel_buf = [0u8; 256];
    let copy_len = core::cmp::min(buf_len, kernel_buf.len());
    if uaccess::copy_from_user(&mut kernel_buf[..copy_len], buf_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }

    // Create message and send via ipc
    let msg = ipc::Message::data(pid, &kernel_buf[..copy_len]);

    match ipc::send(channel_id, msg, pid) {
        Ok(wake_list) => {
            // Wake any subscribers waiting for readable events
            waker::wake(&wake_list, ipc::WakeReason::Readable);
            copy_len as i64
        }
        Err(ipc::IpcError::QueueFull) => Error::WouldBlock.to_errno(),
        Err(ipc::IpcError::PeerClosed) | Err(ipc::IpcError::Closed) => {
            // Backend already knows about peer close
            Error::PeerClosed.to_errno()
        }
        Err(_) => Error::BadHandle.to_errno(),
    }
}

fn write_timer(t: &mut super::TimerObject, buf_ptr: u64, buf_len: usize) -> i64 {
    use crate::platform::current::timer;

    // Format: [deadline_ns: u64] or [deadline_ns: u64, interval_ns: u64]
    if buf_len < 8 {
        return Error::InvalidArg.to_errno();
    }

    let mut buf = [0u8; 16];
    let copy_len = buf_len.min(16);
    if uaccess::copy_from_user(&mut buf[..copy_len], buf_ptr).is_err() {
        return Error::BadAddress.to_errno();
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
        return Error::InvalidArg.to_errno();
    }

    let mut cmd_byte = [0u8; 1];
    if uaccess::copy_from_user(&mut cmd_byte, buf_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }

    match cmd_byte[0] {
        0 => {
            // ALLOW: need peer_pid
            if buf_len < 5 {
                return Error::InvalidArg.to_errno();
            }
            let mut pid_bytes = [0u8; 4];
            if uaccess::copy_from_user(&mut pid_bytes, buf_ptr + 1).is_err() {
                return Error::BadAddress.to_errno();
            }
            let peer_pid = u32::from_le_bytes(pid_bytes);

            match shmem::allow(task_id, s.shmem_id(), peer_pid) {
                Ok(()) => 0,
                Err(e) => e,
            }
        }
        1 => {
            // NOTIFY: wake waiters
            match shmem::notify(task_id, s.shmem_id()) {
                Ok(woken) => woken as i64,
                Err(e) => e,
            }
        }
        _ => Error::InvalidArg.to_errno(),
    }
}

fn write_console(c: &mut super::ConsoleObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Only stdout and stderr are writable
    match c.console_type {
        ConsoleType::Stdout | ConsoleType::Stderr => {
            if buf_len == 0 {
                return 0;
            }
            // Limit buffer size
            if buf_len > 4096 {
                return Error::InvalidArg.to_errno();
            }
            // Copy from user
            let mut kernel_buf = [0u8; 4096];
            if uaccess::copy_from_user(&mut kernel_buf[..buf_len], buf_ptr).is_err() {
                return Error::BadAddress.to_errno();
            }
            // Write to UART and flush
            uart::write_buffered(&kernel_buf[..buf_len]);
            uart::flush_buffer();
            buf_len as i64
        }
        ConsoleType::Stdin => Error::NotSupported.to_errno(),
    }
}

fn write_mux(m: &mut super::MuxObject, buf_ptr: u64, buf_len: usize) -> i64 {
    // Format: [op:1][filter:1][pad:2][handle:4]
    // op: 0=Add, 1=Remove
    // filter: 0=Readable, 1=Writable, 2=Closed
    if buf_len < 8 {
        return Error::InvalidArg.to_errno();
    }

    let mut buf = [0u8; 8];
    if uaccess::copy_from_user(&mut buf, buf_ptr).is_err() {
        return Error::BadAddress.to_errno();
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
            Error::NoSpace.to_errno()
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
            Error::NotFound.to_errno()
        }
        _ => Error::InvalidArg.to_errno(),
    }
}

fn write_pci_device(d: &mut super::PciDeviceObject, buf_ptr: u64, buf_len: usize, task_id: u32) -> i64 {
    use crate::kernel::pci::{self, PciBdf};

    // Params: offset:u16 (2) + size:u8 (1) + value:u32 (4) = 7 bytes
    if buf_len < 7 {
        return Error::InvalidArg.to_errno();
    }

    let mut params = [0u8; 7];
    if uaccess::copy_from_user(&mut params, buf_ptr).is_err() {
        return Error::BadAddress.to_errno();
    }
    let offset = u16::from_le_bytes([params[0], params[1]]);
    let size = params[2];
    let value = u32::from_le_bytes([params[3], params[4], params[5], params[6]]);

    // Validate offset and size
    const PCI_CONFIG_MAX: u16 = 4096;
    if offset >= PCI_CONFIG_MAX || (size != 1 && size != 2 && size != 4) {
        return Error::InvalidArg.to_errno();
    }
    if (offset as u32) + (size as u32) > PCI_CONFIG_MAX as u32 {
        return Error::InvalidArg.to_errno();
    }
    // Check alignment
    if (size == 2 && (offset & 1) != 0) || (size == 4 && (offset & 3) != 0) {
        return Error::InvalidArg.to_errno();
    }

    let bdf = PciBdf::from_u32(d.bdf());

    // Check device exists and caller owns it
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return Error::NotFound.to_errno(),
    };

    if dev.owner() != task_id {
        return Error::PermDenied.to_errno();
    }

    // For partial writes, do read-modify-write
    let aligned_offset = offset & !0x3;

    let result = if size == 4 {
        pci::config_write32(bdf, aligned_offset, value)
    } else {
        // Read current value
        let current = match pci::config_read32(bdf, aligned_offset) {
            Ok(v) => v,
            Err(_) => return Error::Io.to_errno(),
        };

        let (mask, shift) = match size {
            1 => (0xFFu32, ((offset & 3) * 8) as u32),
            2 => (0xFFFFu32, ((offset & 2) * 8) as u32),
            _ => return Error::InvalidArg.to_errno(),
        };

        let new_val = (current & !(mask << shift)) | ((value & mask) << shift);
        pci::config_write32(bdf, aligned_offset, new_val)
    };

    match result {
        Ok(()) => 0,
        Err(_) => Error::Io.to_errno(),
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

fn map_dma_pool(d: &mut super::DmaPoolObject, task_id: u32) -> i64 {
    let _ = (d, task_id);
    Error::NotSupported.to_errno()
}

fn map_mmio(m: &mut super::MmioObject, task_id: u32) -> i64 {
    let _ = (m, task_id);
    Error::NotSupported.to_errno()
}

// Close implementations
fn close_channel(ch: super::ChannelObject, owner: u32) {
    // Only close if not already closed (queries ipc backend)
    if !ch.is_closed() && ch.channel_id() != 0 {
        if let Ok(wake_list) = ipc::close_channel(ch.channel_id(), owner) {
            // Wake any subscribers waiting on this channel
            waker::wake(&wake_list, ipc::WakeReason::Closed);
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
