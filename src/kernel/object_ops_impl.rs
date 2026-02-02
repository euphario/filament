//! Object Operations Backend Implementation
//!
//! Implements the `ObjectOps` trait by delegating to `object::syscall`.
//!
//! NOTE: The unified syscall dispatch (100-104) currently calls
//! `object::syscall::open/read/write/map/close` directly for zero overhead.
//! This wrapper exists for the trait architecture (testability via mocks)
//! and is accessed via `SyscallContext::object()`.
//!
//! # The 5 Syscalls
//!
//! | Syscall | Method | Purpose |
//! |---------|--------|---------|
//! | open | create | Create object, return handle |
//! | read | read | Read from object |
//! | write | write | Write to object |
//! | map | map | Map object to memory |
//! | close | close | Release handle |

use crate::kernel::traits::object_ops::{
    ObjectOps, ObjectType, Handle, ReadResult, MapResult,
};
use crate::kernel::error::KernelError;
use crate::kernel::traits::task::TaskId;
use crate::kernel::traits::waker::Subscriber;
use crate::kernel::object;

// ============================================================================
// Kernel Object Operations Implementation
// ============================================================================

/// Kernel object operations backend implementation
///
/// Delegates to the existing object module syscall functions.
pub struct KernelObjectOps;

impl KernelObjectOps {
    /// Create a new kernel object operations instance
    pub const fn new() -> Self {
        Self
    }
}

/// Convert object module error to KernelError
fn convert_error(errno: i64) -> KernelError {
    KernelError::from_errno(errno)
}

impl ObjectOps for KernelObjectOps {
    fn create(
        &self,
        _task_id: TaskId,
        obj_type: ObjectType,
        _flags: u32,
        arg: u64,
    ) -> Result<Handle, KernelError> {
        // Convert trait ObjectType to object module ObjectType
        let obj_type_num = match obj_type {
            ObjectType::Channel => 0,
            ObjectType::Timer => 1,
            ObjectType::Process => 2,
            ObjectType::Port => 3,
            ObjectType::Shmem => 4,
            ObjectType::DmaPool => 5,
            ObjectType::Mmio => 6,
            ObjectType::Stdin => 7,
            ObjectType::Stdout => 8,
            ObjectType::Stderr => 9,
            ObjectType::Klog => 10,
            ObjectType::Mux => 11,
            ObjectType::PciBus => 12,
            ObjectType::PciDevice => 13,
            ObjectType::Msi => 14,
            ObjectType::BusList => 15,
            ObjectType::Ring => 16,
        };

        // Call the existing open syscall
        // Note: The existing implementation uses current_slot() internally
        // Future: Pass task_id through properly
        let result = object::syscall::open(obj_type_num, arg, 0);

        if result >= 0 {
            Ok(result as Handle)
        } else {
            Err(convert_error(result))
        }
    }

    fn read(
        &self,
        _task_id: TaskId,
        handle: Handle,
        buf: &mut [u8],
        _flags: u32,
    ) -> Result<ReadResult, KernelError> {
        // Call the existing read syscall
        // Note: buf is in kernel space, but the current implementation expects user pointers
        // This is a temporary bridge - we pass the kernel buffer pointer directly
        // which works because we're in the same address space
        let buf_ptr = buf.as_mut_ptr() as u64;
        let result = object::syscall::read(handle, buf_ptr, buf.len());

        if result >= 0 {
            Ok(ReadResult {
                bytes_read: result as usize,
                // peer_pid not available from read() return value — port accept
                // encodes the channel handle, not the peer PID, in the result.
                // The actual syscall path (100-104) handles this correctly via
                // object::syscall::read which returns the raw result to userspace.
                peer_pid: None,
                extra: 0,
            })
        } else {
            Err(convert_error(result))
        }
    }

    fn write(
        &self,
        _task_id: TaskId,
        handle: Handle,
        buf: &[u8],
        _flags: u32,
    ) -> Result<usize, KernelError> {
        // Call the existing write syscall
        let buf_ptr = buf.as_ptr() as u64;
        let result = object::syscall::write(handle, buf_ptr, buf.len());

        if result >= 0 {
            Ok(result as usize)
        } else {
            Err(convert_error(result))
        }
    }

    fn map(&self, _task_id: TaskId, handle: Handle) -> Result<MapResult, KernelError> {
        // Call the existing map syscall
        let result = object::syscall::map(handle, 0);

        if result >= 0 {
            Ok(MapResult {
                vaddr: result as u64,
                // paddr and size not available from map() return value — it returns
                // only vaddr as i64. The actual syscall path (103) returns this raw
                // value to userspace. Userspace drivers that need paddr use the
                // separate DMA pool output pointers or shmem map which writes both.
                paddr: 0,
                size: 0,
            })
        } else {
            Err(convert_error(result))
        }
    }

    fn close(&self, _task_id: TaskId, handle: Handle) -> Result<(), KernelError> {
        // Call the existing close syscall
        let result = object::syscall::close(handle);

        if result >= 0 {
            Ok(())
        } else {
            Err(convert_error(result))
        }
    }

    fn subscribe(
        &self,
        _task_id: TaskId,
        _handle: Handle,
        _sub: Subscriber,
    ) -> Result<(), KernelError> {
        // Subscriptions are handled internally by object types (Channel, Port,
        // Timer, Mux) via their Pollable trait. The subscribe/unsubscribe cycle
        // is managed by read_mux_via_service in object::syscall, not here.
        Ok(())
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel object operations backend
pub static OBJECT_OPS_BACKEND: KernelObjectOps = KernelObjectOps::new();

/// Get a reference to the global object operations backend
pub fn object_ops_backend() -> &'static dyn ObjectOps {
    &OBJECT_OPS_BACKEND
}
