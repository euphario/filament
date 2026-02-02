//! Kernel Trait Boundaries
//!
//! This module defines the contracts between kernel subsystems.
//! These traits are the ONLY way subsystems communicate - no direct struct access.
//!
//! # Design Philosophy
//!
//! 1. **Single Source of Truth**: Traits define state, implementations own it
//! 2. **No Caching Across Boundaries**: Always query through the trait
//! 3. **Swappable Implementations**: Mock for testing, real for production
//! 4. **Dependency Injection**: Subsystems receive trait objects, not concrete types
//!
//! # Module Structure
//!
//! - [`ipc`] - IPC primitives (channels, ports)
//! - [`memory`] - Memory allocation and shared memory
//! - [`waker`] - Task waking abstraction
//!
//! # Usage Example
//!
//! ```ignore
//! // Syscall layer receives trait objects
//! fn sys_read(backend: &dyn IpcBackend, handle: Handle) -> Result<Message, Error> {
//!     let channel = backend.get_channel(handle)?;
//!     channel.receive()
//! }
//!
//! // Testing with mock
//! #[test]
//! fn test_receive() {
//!     let mock = MockIpcBackend::new();
//!     mock.inject_message(handle, Message::new(b"test"));
//!     let result = sys_read(&mock, handle);
//!     assert_eq!(result.unwrap().payload(), b"test");
//! }
//! ```

// Existing subsystem traits
pub mod ipc;
pub mod memory;
pub mod waker;
pub mod sched;
pub mod process;
pub mod addrspace;
pub mod task;

// New syscall-facing traits (microkernel interface)
pub mod syscall_ctx;
pub mod object_ops;
pub mod raw_object_ops;
pub mod memory_ops;
pub mod process_ops;
pub mod user_access;
pub mod misc_ops;

// Re-exports for convenient access - existing traits
#[allow(unused_imports)]
pub use ipc::{
    IpcChannel, IpcPort, IpcBackend, IpcError as TraitIpcError,
    ChannelState, PortState, Message as TraitMessage,
};
#[allow(unused_imports)]
pub use memory::{PhysicalAllocator, SharedMemory, ShmemError, MappingTracker};
#[allow(unused_imports)]
pub use waker::{Waker, WakeReason, Subscriber};
#[allow(unused_imports)]
pub use sched::{SchedulerBackend, TaskStateInfo, SchedError};
#[allow(unused_imports)]
pub use process::{ProcessBackend, ProcessStateInfo, ProcessInfo};
#[allow(unused_imports)]
pub use addrspace::{AddressSpaceBackend, Asid, VirtAddr, PhysAddr, PageFlags, MemoryType, AddrSpaceError};
#[allow(unused_imports)]
pub use task::{TaskOperations, TaskId, ResourceCounts, Capabilities, TaskError};

// Re-exports for new syscall-facing traits
#[allow(unused_imports)]
pub use syscall_ctx::{SyscallContext, SyscallError};
#[allow(unused_imports)]
pub use object_ops::{ObjectOps, ObjectType, ObjectError, Handle, ReadResult, MapResult};
// ObjectError is now a type alias for crate::kernel::error::KernelError
#[allow(unused_imports)]
pub use memory_ops::{MemoryOps, MemoryError};
#[allow(unused_imports)]
pub use process_ops::{ProcessOps, ProcessError as ProcOpsError};
#[allow(unused_imports)]
pub use user_access::{UserAccess, UserAccessExt, UAccessError};
#[allow(unused_imports)]
pub use misc_ops::MiscOps;
