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

pub mod ipc;
pub mod memory;
pub mod waker;
pub mod sched;

// Re-exports for convenient access - will be used once migration completes
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
