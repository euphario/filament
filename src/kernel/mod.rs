//! Kernel Core
//!
//! This module contains the portable kernel core:
//! - Task and process management
//! - Scheduler
//! - System call handling
//! - IPC (channels, ports, events)
//! - Scheme system (userspace drivers)
//! - Memory management (PMM, address spaces)
//! - ELF loader
//!
//! Code in this module should be architecture and platform independent.

pub mod task;
pub mod process;
pub mod syscall;
pub mod ipc;
pub mod scheme;
pub mod port;
pub mod event;
pub mod fd;
pub mod shmem;
pub mod elf;
pub mod pmm;
pub mod addrspace;
pub mod uaccess;
pub mod log;
pub mod caps;
pub mod lock;
pub mod percpu;
pub mod pci;
pub mod bus;
pub mod security_log;
