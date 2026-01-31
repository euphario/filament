//! Kernel Core
//!
//! This module contains the portable kernel core:
//! - Task and process management
//! - Scheduler
//! - System call handling
//! - IPC (channels, ports)
//! - Memory management (PMM, address spaces)
//! - ELF loader
//!
//! Code in this module should be architecture and platform independent.

pub mod traits; // Trait boundaries for swappable subsystems
pub mod waker_impl; // Waker trait implementation
pub mod task;
pub mod task_impl; // TaskOperations trait implementation
pub mod sched;
pub mod sched_impl; // SchedulerBackend trait implementation
pub mod storm;  // Syscall storm protection
pub mod process;
pub mod process_impl; // ProcessBackend trait implementation
pub mod syscall;
pub mod memory;  // User memory mapping types
pub(crate) mod ipc;    // IPC primitives - internal, use object/ for public API
pub mod shmem;
pub mod shmem_impl; // ShmemBackend trait implementation
pub mod dma_pool;
pub mod elf;
pub mod pmm;
pub mod pmm_impl; // PhysicalAllocator trait implementation
pub mod addrspace;
pub mod addrspace_impl; // AddressSpaceBackend trait implementation
pub mod uaccess;
pub mod caps;
pub mod lock;
pub mod percpu;
pub mod pci;
pub mod bus;
pub mod security_log;
pub mod fdt;
pub mod liveness;
pub mod hw_poll;
pub mod error; // Unified KernelError type
pub mod object;
pub mod irq;
pub mod object_service;
pub mod idle;

// New syscall-facing trait implementations
pub mod syscall_ctx_impl;
pub mod object_ops_impl;
pub mod memory_ops_impl;
pub mod process_ops_impl;
pub mod user_access_impl;
