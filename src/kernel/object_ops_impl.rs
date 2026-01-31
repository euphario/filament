//! Object Operations Backend Implementation
//!
//! Implements the `ObjectOps` trait - the heart of the microkernel.
//! This wraps the existing object/syscall.rs logic but provides a clean
//! trait boundary.
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
                peer_pid: None, // TODO: Extract from port accept
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
                paddr: 0, // TODO: Return physical address for DMA
                size: 0,  // TODO: Return mapped size
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
        // TODO: Implement subscription for blocking operations
        // For now, subscriptions are handled internally by the object types
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
