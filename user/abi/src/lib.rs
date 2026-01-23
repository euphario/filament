//! Kernel ABI - Shared types between kernel and userspace
//!
//! This crate defines all types that cross the kernel/userspace boundary.
//! Both the kernel and userlib depend on this crate to ensure consistency.
//!
//! # Rules
//! - All structs must be `#[repr(C)]` for stable layout
//! - No heap allocation (no_std compatible)
//! - No dependencies on kernel or userlib internals

#![no_std]

// ============================================================================
// Syscall Numbers
// ============================================================================

/// Syscall numbers - must match kernel src/kernel/syscall/mod.rs
pub mod syscall {
    // Core process/memory (0-11)
    pub const EXIT: u64 = 0;
    pub const DEBUG_WRITE: u64 = 1;
    pub const YIELD: u64 = 2;
    pub const GETPID: u64 = 3;
    pub const MMAP: u64 = 4;
    pub const MUNMAP: u64 = 5;
    // 6-7: reserved
    pub const SPAWN: u64 = 8;
    pub const WAIT: u64 = 9;
    pub const GETTIME: u64 = 10;
    pub const SLEEP: u64 = 11;

    // Process control (31-37)
    pub const EXEC: u64 = 31;
    pub const DAEMONIZE: u64 = 32;
    pub const KILL: u64 = 33;
    pub const PS_INFO: u64 = 34;
    pub const SET_LOG_LEVEL: u64 = 35;
    pub const MMAP_DMA: u64 = 36;
    pub const RESET: u64 = 37;

    // Shared memory (39-47)
    pub const SHMEM_CREATE: u64 = 39;
    pub const SHMEM_MAP: u64 = 40;
    pub const SHMEM_ALLOW: u64 = 41;
    pub const SHMEM_WAIT: u64 = 42;
    pub const SHMEM_NOTIFY: u64 = 43;
    pub const SHMEM_DESTROY: u64 = 44;
    pub const SHMEM_UNMAP: u64 = 47;

    // PCI/Bus (51-76)
    pub const PCI_CONFIG_READ: u64 = 51;
    pub const PCI_CONFIG_WRITE: u64 = 52;
    pub const PCI_MSI_ALLOC: u64 = 54;
    pub const SIGNAL_ALLOW: u64 = 56;
    pub const BUS_LIST: u64 = 59;
    pub const MMAP_DEVICE: u64 = 60;
    pub const DMA_POOL_CREATE: u64 = 61;
    pub const DMA_POOL_CREATE_HIGH: u64 = 62;
    pub const RAMFS_LIST: u64 = 63;
    pub const EXEC_MEM: u64 = 64;
    pub const KLOG_READ: u64 = 65;
    pub const GET_CAPABILITIES: u64 = 66;
    pub const CPU_STATS: u64 = 68;
    pub const EXEC_WITH_CAPS: u64 = 74;
    pub const DEVICE_LIST: u64 = 75;
    pub const KLOG: u64 = 76;

    // Unified interface (100-104) - THE 5 SYSCALLS
    pub const OPEN: u64 = 100;
    pub const READ: u64 = 101;
    pub const WRITE: u64 = 102;
    pub const MAP: u64 = 103;
    pub const CLOSE: u64 = 104;
}

// ============================================================================
// Object Types (for unified syscall interface)
// ============================================================================

/// Object types for the unified open/read/write/map/close interface
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObjectType {
    /// Bidirectional IPC channel
    Channel = 0,
    /// Timer - write deadline, read waits for tick
    Timer = 1,
    /// Child process - read waits for exit
    Process = 2,
    /// Named port - read accepts connections
    Port = 3,
    /// Shared memory region (mappable)
    Shmem = 4,
    /// DMA-capable memory pool (mappable)
    DmaPool = 5,
    /// Device MMIO region (mappable)
    Mmio = 6,
    /// Console stdin
    Stdin = 7,
    /// Console stdout
    Stdout = 8,
    /// Console stderr
    Stderr = 9,
    /// Kernel log buffer
    Klog = 10,
    /// Event multiplexer
    Mux = 11,
    /// PCIe bus access (device enumeration)
    PciBus = 12,
    /// PCI device handle (for config read/write)
    PciDevice = 13,
    /// MSI vector allocation
    Msi = 14,
    /// Bus enumeration
    BusList = 15,
}

impl ObjectType {
    pub fn from_u32(v: u32) -> Option<Self> {
        match v {
            0 => Some(ObjectType::Channel),
            1 => Some(ObjectType::Timer),
            2 => Some(ObjectType::Process),
            3 => Some(ObjectType::Port),
            4 => Some(ObjectType::Shmem),
            5 => Some(ObjectType::DmaPool),
            6 => Some(ObjectType::Mmio),
            7 => Some(ObjectType::Stdin),
            8 => Some(ObjectType::Stdout),
            9 => Some(ObjectType::Stderr),
            10 => Some(ObjectType::Klog),
            11 => Some(ObjectType::Mux),
            12 => Some(ObjectType::PciBus),
            13 => Some(ObjectType::PciDevice),
            14 => Some(ObjectType::Msi),
            15 => Some(ObjectType::BusList),
            _ => None,
        }
    }

    /// Can this type be memory-mapped?
    pub fn is_mappable(&self) -> bool {
        matches!(self, ObjectType::Shmem | ObjectType::DmaPool | ObjectType::Mmio)
    }

    /// Does this type support read()?
    pub fn is_readable(&self) -> bool {
        !matches!(self, ObjectType::Stdout | ObjectType::Stderr | ObjectType::DmaPool | ObjectType::Mmio)
    }

    /// Does this type support write()?
    pub fn is_writable(&self) -> bool {
        !matches!(self, ObjectType::Stdin | ObjectType::Klog | ObjectType::DmaPool | ObjectType::Mmio | ObjectType::Process | ObjectType::BusList | ObjectType::Msi)
    }
}

// ============================================================================
// Process Information
// ============================================================================

/// Process info structure for ps_info syscall
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ProcessInfo {
    pub pid: u32,
    pub ppid: u32,
    pub state: u8,
    pub liveness_status: u8,
    pub _pad: [u8; 2],
    pub activity_age_ms: u32,
    pub name: [u8; 16],
}

impl ProcessInfo {
    pub const fn empty() -> Self {
        Self {
            pid: 0,
            ppid: 0,
            state: 0,
            liveness_status: 0,
            _pad: [0; 2],
            activity_age_ms: 0,
            name: [0; 16],
        }
    }

    pub fn state_str(&self) -> &'static str {
        match self.state {
            0 => "Ready",
            1 => "Running",
            2 => "Sleeping",
            3 => "Waiting",
            4 => "Exiting",
            5 => "Dying",
            6 => "Dead",
            _ => "Unknown",
        }
    }
}

/// Process state values
pub mod process_state {
    pub const READY: u8 = 0;
    pub const RUNNING: u8 = 1;
    pub const SLEEPING: u8 = 2;
    pub const WAITING: u8 = 3;
    pub const EXITING: u8 = 4;
    pub const DYING: u8 = 5;
    pub const DEAD: u8 = 6;
}

/// Liveness status values (kernel ping/pong system)
pub mod liveness_status {
    pub const NORMAL: u8 = 0;
    pub const PING_SENT: u8 = 1;
    pub const CLOSE_PENDING: u8 = 2;
}

// ============================================================================
// Log Levels
// ============================================================================

/// Log levels for klog syscall
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogLevel {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
    Trace = 4,
}

pub mod log_level {
    pub const ERROR: u8 = 0;
    pub const WARN: u8 = 1;
    pub const INFO: u8 = 2;
    pub const DEBUG: u8 = 3;
    pub const TRACE: u8 = 4;
}

// ============================================================================
// Memory Protection Flags
// ============================================================================

pub mod prot {
    pub const READ: u32 = 1;
    pub const WRITE: u32 = 2;
    pub const EXEC: u32 = 4;
}

// ============================================================================
// Handle
// ============================================================================

/// User-visible handle to kernel objects
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Handle(pub u32);

impl Handle {
    pub const INVALID: Handle = Handle(0);
    /// Pre-allocated stdin handle
    pub const STDIN: Handle = Handle((1 << 24) | 1);
    /// Pre-allocated stdout handle
    pub const STDOUT: Handle = Handle((1 << 24) | 2);
    /// Pre-allocated stderr handle
    pub const STDERR: Handle = Handle((1 << 24) | 3);

    #[inline]
    pub const fn new(index: usize, generation: u8) -> Self {
        Self(((generation as u32) << 24) | (index as u32 & 0x00FF_FFFF))
    }

    #[inline]
    pub const fn index(&self) -> usize {
        (self.0 & 0x00FF_FFFF) as usize
    }

    #[inline]
    pub const fn generation(&self) -> u8 {
        (self.0 >> 24) as u8
    }

    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.0 != 0
    }

    #[inline]
    pub const fn raw(&self) -> u32 {
        self.0
    }

    #[inline]
    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }
}

// ============================================================================
// Handle Rights
// ============================================================================

/// Rights associated with a handle (capability bits)
///
/// Rights can only be reduced when duplicating, never expanded.
/// This follows the capability model for security.
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct HandleRights(u16);

impl HandleRights {
    /// No rights
    pub const NONE: Self = Self(0);

    /// Can read/receive data
    pub const READ: Self = Self(1 << 0);

    /// Can write/send data
    pub const WRITE: Self = Self(1 << 1);

    /// Can wait on this handle
    pub const WAIT: Self = Self(1 << 2);

    /// Can signal/notify via this handle
    pub const SIGNAL: Self = Self(1 << 3);

    /// Can close this handle
    pub const CLOSE: Self = Self(1 << 4);

    /// Can transfer this handle to another task
    pub const TRANSFER: Self = Self(1 << 5);

    /// Can duplicate this handle
    pub const DUPLICATE: Self = Self(1 << 6);

    /// Can map memory (for shmem, mmio)
    pub const MAP: Self = Self(1 << 7);

    /// All rights
    pub const FULL: Self = Self(0xFF);

    /// Read-only preset
    pub const READ_ONLY: Self = Self(Self::READ.0 | Self::WAIT.0 | Self::CLOSE.0);

    /// Write-only preset
    pub const WRITE_ONLY: Self = Self(Self::WRITE.0 | Self::CLOSE.0);

    /// Read-write preset
    pub const READ_WRITE: Self = Self(Self::READ.0 | Self::WRITE.0 | Self::WAIT.0 | Self::CLOSE.0);

    /// Check if a specific right is present
    #[inline]
    pub const fn has(&self, right: HandleRights) -> bool {
        (self.0 & right.0) == right.0
    }

    /// Combine rights (OR)
    #[inline]
    pub const fn or(&self, other: HandleRights) -> Self {
        Self(self.0 | other.0)
    }

    /// Restrict rights (AND)
    #[inline]
    pub const fn and(&self, other: HandleRights) -> Self {
        Self(self.0 & other.0)
    }

    /// Raw value
    #[inline]
    pub const fn raw(&self) -> u16 {
        self.0
    }

    /// From raw value
    #[inline]
    pub const fn from_raw(raw: u16) -> Self {
        Self(raw)
    }
}

// ============================================================================
// Error Codes
// ============================================================================

/// Standard errno values returned by syscalls
pub mod errno {
    pub const SUCCESS: i32 = 0;
    pub const EPERM: i32 = -1;         // Operation not permitted
    pub const ENOENT: i32 = -2;        // No such file or directory
    pub const ESRCH: i32 = -3;         // No such process
    pub const EINTR: i32 = -4;         // Interrupted system call
    pub const EIO: i32 = -5;           // I/O error
    pub const ENXIO: i32 = -6;         // No such device or address
    pub const ENOEXEC: i32 = -8;       // Exec format error
    pub const EBADF: i32 = -9;         // Bad file descriptor
    pub const ECHILD: i32 = -10;       // No child processes
    pub const EAGAIN: i32 = -11;       // Resource temporarily unavailable
    pub const ENOMEM: i32 = -12;       // Out of memory
    pub const EACCES: i32 = -13;       // Permission denied
    pub const EFAULT: i32 = -14;       // Bad address
    pub const EBUSY: i32 = -16;        // Device or resource busy
    pub const EEXIST: i32 = -17;       // File/resource already exists
    pub const ENODEV: i32 = -19;       // No such device
    pub const EINVAL: i32 = -22;       // Invalid argument
    pub const ENFILE: i32 = -23;       // Too many open files in system
    pub const ENOSPC: i32 = -28;       // No space left
    pub const EROFS: i32 = -30;        // Read-only filesystem
    pub const EPIPE: i32 = -32;        // Broken pipe
    pub const ENOSYS: i32 = -38;       // Function not implemented
    pub const EMSGSIZE: i32 = -90;     // Message too long
    pub const EOPNOTSUPP: i32 = -95;   // Operation not supported
    pub const ECONNRESET: i32 = -104;  // Connection reset
    pub const ETIMEDOUT: i32 = -110;   // Connection timed out
    pub const ECONNREFUSED: i32 = -111; // Connection refused
}

// ============================================================================
// Mux Types (Event Multiplexer)
// ============================================================================

/// Filter for Mux events (bitmask - can combine multiple)
pub mod mux_filter {
    pub const READABLE: u8 = 1 << 0;
    pub const WRITABLE: u8 = 1 << 1;
    pub const CLOSED: u8 = 1 << 2;
    pub const ERROR: u8 = 1 << 3;
}

/// Filter for Mux events (enum for simple cases)
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MuxFilter {
    Readable = 1,
    Writable = 2,
    Closed = 4,
}

/// Event from multiplexer
/// MUST match exactly between kernel and userspace
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MuxEvent {
    pub handle: Handle,
    pub event: u8,
    pub _pad: [u8; 3],
}

impl MuxEvent {
    pub const fn empty() -> Self {
        Self { handle: Handle::INVALID, event: 0, _pad: [0; 3] }
    }

    /// Check if this event indicates readable
    pub fn is_readable(&self) -> bool {
        (self.event & mux_filter::READABLE) != 0
    }

    /// Check if this event indicates writable
    pub fn is_writable(&self) -> bool {
        (self.event & mux_filter::WRITABLE) != 0
    }

    /// Check if this event indicates closed
    pub fn is_closed(&self) -> bool {
        (self.event & mux_filter::CLOSED) != 0
    }
}

/// Mux commands (written to mux handle)
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MuxCommand {
    Add = 1,
    Remove = 2,
    Modify = 3,
}

/// Add/modify watch request (for write to mux)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MuxAddWatch {
    pub command: u8,
    pub filter: u8,
    pub _pad: [u8; 2],
    pub handle: u32,
}
