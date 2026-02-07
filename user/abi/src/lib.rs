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
    pub const GETPID: u64 = 3;
    pub const MMAP: u64 = 4;
    pub const MUNMAP: u64 = 5;
    pub const SPAWN: u64 = 8;
    pub const WAIT: u64 = 9;
    pub const GETTIME: u64 = 10;
    pub const SLEEP: u64 = 11;

    // Process control
    pub const EXEC: u64 = 31;
    pub const DAEMONIZE: u64 = 32;
    pub const KILL: u64 = 33;
    pub const PS_INFO: u64 = 34;
    pub const SET_LOG_LEVEL: u64 = 35;
    pub const RESET: u64 = 37;
    pub const SHUTDOWN: u64 = 38;

    // Misc
    pub const RAMFS_LIST: u64 = 63;
    pub const EXEC_MEM: u64 = 64;
    pub const KLOG_READ: u64 = 65;
    pub const GET_CAPABILITIES: u64 = 66;
    pub const EXEC_WITH_CAPS: u64 = 74;
    pub const KLOG: u64 = 76;
    pub const KLOG_WRITE: u64 = 77;

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
    /// Ring buffer IPC (high-performance typed messages)
    Ring = 16,
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
            16 => Some(ObjectType::Ring),
            _ => None,
        }
    }

    /// Can this type be memory-mapped?
    pub fn is_mappable(&self) -> bool {
        matches!(self, ObjectType::Shmem | ObjectType::DmaPool | ObjectType::Mmio | ObjectType::Ring)
    }

    /// Does this type support read()?
    /// DmaPool: read returns paddr + size (16 bytes)
    pub fn is_readable(&self) -> bool {
        !matches!(self, ObjectType::Stdout | ObjectType::Stderr | ObjectType::Mmio)
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
    pub cpu: u8,
    pub _pad: u8,
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
            cpu: 0xFF,
            _pad: 0,
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
    pub const EMFILE: i32 = -24;       // Too many open files per process
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

// ============================================================================
// Ring IPC (High-Performance Typed Message Passing)
// ============================================================================

/// Ring configuration - 16 bits encoding size parameters
///
/// Bits 0-3:  Ring size shift  (actual = 1 << (12 + n)) = 4KB to 512KB
/// Bits 4-7:  Slot size shift  (actual = 1 << (5 + n))  = 32B to 4KB
/// Bits 8-15: Reserved/flags
pub type RingConfig = u16;

/// Ring configuration helpers
pub mod ring_config {
    use super::RingConfig;

    /// Extract ring size in bytes from config
    #[inline]
    pub const fn ring_size(cfg: RingConfig) -> usize {
        1 << (12 + (cfg & 0x0F) as usize)
    }

    /// Extract slot size in bytes from config
    #[inline]
    pub const fn slot_size(cfg: RingConfig) -> usize {
        1 << (5 + ((cfg >> 4) & 0x0F) as usize)
    }

    /// Calculate number of slots in ring
    #[inline]
    pub const fn slot_count(cfg: RingConfig) -> usize {
        ring_size(cfg) / slot_size(cfg)
    }

    /// Create config from ring shift and slot shift
    #[inline]
    pub const fn make(ring_shift: u8, slot_shift: u8) -> RingConfig {
        (ring_shift as u16) | ((slot_shift as u16) << 4)
    }
}

/// Preset ring configurations
pub mod ring_presets {
    use super::RingConfig;
    use super::ring_config::make;

    /// Console TX: 64KB ring, 256B slots (256 slots) - shell output bursts
    pub const CONSOLE_TX: RingConfig = make(4, 3);

    /// Console RX: 4KB ring, 64B slots (64 slots) - keyboard input
    pub const CONSOLE_RX: RingConfig = make(0, 1);

    /// Balanced: 16KB ring, 128B slots (128 slots) - general IPC
    pub const BALANCED: RingConfig = make(2, 2);

    /// Bulk: 256KB ring, 512B slots (512 slots) - large transfers
    pub const BULK: RingConfig = make(6, 4);

    /// Tiny: 4KB ring, 32B slots (128 slots) - minimal footprint
    pub const TINY: RingConfig = make(0, 0);
}

/// Parameters for opening a Ring object
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RingParams {
    /// TX ring config (producer → consumer direction)
    pub tx: RingConfig,
    /// RX ring config (consumer → producer direction)
    pub rx: RingConfig,
}

impl RingParams {
    /// Console preset: asymmetric (large TX, small RX)
    pub const CONSOLE: Self = Self {
        tx: ring_presets::CONSOLE_TX,
        rx: ring_presets::CONSOLE_RX,
    };

    /// Balanced preset: symmetric
    pub const BALANCED: Self = Self {
        tx: ring_presets::BALANCED,
        rx: ring_presets::BALANCED,
    };
}

/// Ring header in shared memory (cache-line aligned)
///
/// Layout in shared memory:
/// ```text
/// [RingHeader][TX Ring buffer][RX Ring buffer]
/// ```
#[repr(C, align(64))]
#[derive(Debug)]
pub struct RingHeader {
    /// TX ring: producer head (written by producer)
    pub tx_head: core::sync::atomic::AtomicU32,
    /// TX ring: consumer tail (written by consumer)
    pub tx_tail: core::sync::atomic::AtomicU32,
    /// RX ring: producer head (written by consumer, who is RX producer)
    pub rx_head: core::sync::atomic::AtomicU32,
    /// RX ring: consumer tail (written by producer, who is RX consumer)
    pub rx_tail: core::sync::atomic::AtomicU32,
    /// TX ring config (immutable after creation)
    pub tx_config: RingConfig,
    /// RX ring config (immutable after creation)
    pub rx_config: RingConfig,
    /// Flags for signaling
    pub flags: core::sync::atomic::AtomicU32,
    /// Reserved for future use
    pub _reserved: [u8; 40],
}

/// Ring flags for wake signaling
pub mod ring_flags {
    /// Producer wants wake when space available
    pub const TX_WAIT: u32 = 1 << 0;
    /// Consumer wants wake when data available
    pub const RX_WAIT: u32 = 1 << 1;
}

// ============================================================================
// Pipe Messages (Typed Data for Ring IPC)
// ============================================================================

/// Maximum filename length in pipe messages
pub const PIPE_NAME_MAX: usize = 64;

/// Entry kind for directory entries
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntryKind {
    File = 0,
    Directory = 1,
    Symlink = 2,
    Device = 3,
    Unknown = 255,
}

/// Directory entry - used by ls, find, etc.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DirEntry {
    pub name: [u8; PIPE_NAME_MAX],
    pub size: u64,
    pub kind: EntryKind,
    pub permissions: u16,
    pub _pad: [u8; 5],
}

impl DirEntry {
    pub const fn empty() -> Self {
        Self {
            name: [0; PIPE_NAME_MAX],
            size: 0,
            kind: EntryKind::Unknown,
            permissions: 0,
            _pad: [0; 5],
        }
    }

    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&b| b == 0).unwrap_or(PIPE_NAME_MAX);
        core::str::from_utf8(&self.name[..len]).unwrap_or("")
    }
}

/// Process info for ps output
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PipeProcessInfo {
    pub pid: u32,
    pub ppid: u32,
    pub state: u8,
    pub _pad: [u8; 3],
    pub name: [u8; 32],
}

impl PipeProcessInfo {
    pub const fn empty() -> Self {
        Self {
            pid: 0,
            ppid: 0,
            state: 0,
            _pad: [0; 3],
            name: [0; 32],
        }
    }
}

/// Maximum text chunk size
pub const PIPE_TEXT_MAX: usize = 192;

/// Raw text chunk - fallback for unstructured data
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TextChunk {
    pub len: u16,
    pub _pad: [u8; 2],
    pub data: [u8; PIPE_TEXT_MAX],
}

impl TextChunk {
    pub const fn empty() -> Self {
        Self {
            len: 0,
            _pad: [0; 2],
            data: [0; PIPE_TEXT_MAX],
        }
    }

    pub fn from_bytes(data: &[u8]) -> Self {
        let mut chunk = Self::empty();
        let len = data.len().min(PIPE_TEXT_MAX);
        chunk.data[..len].copy_from_slice(&data[..len]);
        chunk.len = len as u16;
        chunk
    }

    pub fn as_bytes(&self) -> &[u8] {
        let len = (self.len as usize).min(PIPE_TEXT_MAX);
        &self.data[..len]
    }
}

/// Table column descriptor
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TableColumn {
    pub name: [u8; 16],
    pub width: u8,
    pub align: u8,  // 0=left, 1=right, 2=center
    pub _pad: [u8; 2],
}

impl TableColumn {
    pub const fn empty() -> Self {
        Self {
            name: [0; 16],
            width: 0,
            align: 0,
            _pad: [0; 2],
        }
    }
}

/// Table start marker with column definitions
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TableStart {
    pub column_count: u8,
    pub _pad: [u8; 3],
    pub columns: [TableColumn; 8],
}

impl TableStart {
    pub const fn empty() -> Self {
        Self {
            column_count: 0,
            _pad: [0; 3],
            columns: [TableColumn::empty(); 8],
        }
    }
}

/// Error message
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PipeError {
    pub code: i32,
    pub msg_len: u16,
    pub _pad: [u8; 2],
    pub msg: [u8; 64],
}

impl PipeError {
    pub const fn empty() -> Self {
        Self {
            code: 0,
            msg_len: 0,
            _pad: [0; 2],
            msg: [0; 64],
        }
    }
}

/// Pipe message type tag
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PipeMessageType {
    /// Raw text (fallback)
    Text = 1,
    /// Directory entry
    DirEntry = 2,
    /// Process info
    ProcessInfo = 3,
    /// Table start marker
    TableStart = 4,
    /// Table end marker
    TableEnd = 5,
    /// Error message
    Error = 6,
    /// End of stream
    Eof = 7,
}

/// Maximum pipe message size (fits in 256B slot with header)
pub const PIPE_MESSAGE_MAX: usize = 248;

// ============================================================================
// PCI BDF Encoding (Single Source of Truth)
// ============================================================================

/// Standard PCI Bus/Device/Function encoding.
///
/// This is the canonical BDF format used across the kernel/userspace boundary.
/// Both the kernel's `PciBdf` and userspace drivers must use these functions.
///
/// Bit layout of a packed BDF u32:
///   bits 15..8:  bus (0-255)
///   bits 7..3:   device (0-31)
///   bits 2..0:   function (0-7)
///
/// The upper 16 bits are reserved (zero for standard PCI).
pub mod pci_bdf {
    /// Pack bus/device/function into a u32.
    #[inline]
    pub const fn pack(bus: u8, device: u8, function: u8) -> u32 {
        ((bus as u32) << 8) | ((device as u32 & 0x1F) << 3) | (function as u32 & 0x07)
    }

    /// Extract bus from packed BDF.
    #[inline]
    pub const fn bus(bdf: u32) -> u8 {
        ((bdf >> 8) & 0xFF) as u8
    }

    /// Extract device from packed BDF.
    #[inline]
    pub const fn device(bdf: u32) -> u8 {
        ((bdf >> 3) & 0x1F) as u8
    }

    /// Extract function from packed BDF.
    #[inline]
    pub const fn function(bdf: u32) -> u8 {
        (bdf & 0x07) as u8
    }
}

// ============================================================================
// PCI Enumeration (Kernel → Userspace)
// ============================================================================

/// PCI device entry returned by kernel enumeration
///
/// Used by read(PciBus) to return the list of discovered PCI devices.
/// 28 bytes per entry, packed for efficient transfer.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PciEnumEntry {
    /// Packed BDF — see [`pci_bdf`] for encoding.
    pub bdf: u32,
    /// PCI vendor ID
    pub vendor_id: u16,
    /// PCI device ID
    pub device_id: u16,
    /// Packed class: class(8) | subclass(8) | prog_if(8) | revision(8)
    pub class_code: u32,
    /// First MMIO BAR physical address
    pub bar0_addr: u64,
    /// First MMIO BAR size in bytes
    pub bar0_size: u32,
    /// MSI capability offset (0 = not supported)
    pub msi_cap: u8,
    /// MSI-X capability offset (0 = not supported)
    pub msix_cap: u8,
    /// Reserved for future use
    pub _reserved: [u8; 2],
}

impl PciEnumEntry {
    pub const fn empty() -> Self {
        Self {
            bdf: 0,
            vendor_id: 0,
            device_id: 0,
            class_code: 0,
            bar0_addr: 0,
            bar0_size: 0,
            msi_cap: 0,
            msix_cap: 0,
            _reserved: [0; 2],
        }
    }

    /// Extract bus number from packed BDF
    pub const fn bus(&self) -> u8 {
        pci_bdf::bus(self.bdf)
    }

    /// Extract device number from packed BDF
    pub const fn device(&self) -> u8 {
        pci_bdf::device(self.bdf)
    }

    /// Extract function number from packed BDF
    pub const fn function(&self) -> u8 {
        pci_bdf::function(self.bdf)
    }

    /// Extract base class from class_code
    pub const fn base_class(&self) -> u8 {
        ((self.class_code >> 24) & 0xFF) as u8
    }

    /// Extract subclass from class_code
    pub const fn subclass(&self) -> u8 {
        ((self.class_code >> 16) & 0xFF) as u8
    }

    /// Extract prog_if from class_code
    pub const fn prog_if(&self) -> u8 {
        ((self.class_code >> 8) & 0xFF) as u8
    }

    /// Extract revision from class_code
    pub const fn revision(&self) -> u8 {
        (self.class_code & 0xFF) as u8
    }
}

// ============================================================================
// Bus Types (Kernel → Userspace)
// ============================================================================

/// Bus type constants for BusInfo
pub mod bus_type {
    /// PCIe root port
    pub const PCIE: u8 = 0;
    /// USB host controller (xHCI)
    pub const USB: u8 = 1;
    /// Platform pseudo-bus (gpio, i2c, spi, etc.) - NOT uart
    pub const PLATFORM: u8 = 2;
    /// Ethernet (native GMAC)
    pub const ETHERNET: u8 = 3;
    /// UART serial port
    pub const UART: u8 = 4;
    /// Kernel log (klog)
    pub const KLOG: u8 = 5;
}

/// Bus state constants for BusInfo
pub mod bus_state {
    /// Bus is idle, ready to be claimed
    pub const SAFE: u8 = 0;
    /// Bus has an active driver
    pub const CLAIMED: u8 = 1;
    /// Bus is resetting (driver crashed or explicit reset)
    pub const RESETTING: u8 = 2;
}

/// Bus information returned by kernel bus enumeration
///
/// Used by open(BusList) + read(BusList) to return discovered buses.
/// 48 bytes per entry, packed for efficient transfer.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct BusInfo {
    /// Bus type (see `bus_type` module)
    pub bus_type: u8,
    /// Bus index within type (e.g., 0 for pcie0)
    pub bus_index: u8,
    /// Current state (see `bus_state` module)
    pub state: u8,
    /// Padding for alignment
    pub _pad: u8,
    /// MMIO base address (from DTB/hardcoded)
    pub base_addr: u32,
    /// Owner PID (0 if no owner)
    pub owner_pid: u32,
    /// Port path (e.g., "/kernel/bus/pcie0")
    pub path: [u8; 32],
    /// Length of path string
    pub path_len: u8,
    /// Reserved for future use
    pub _reserved: [u8; 3],
}

impl BusInfo {
    pub const fn empty() -> Self {
        Self {
            bus_type: 0,
            bus_index: 0,
            state: 0,
            _pad: 0,
            base_addr: 0,
            owner_pid: 0,
            path: [0; 32],
            path_len: 0,
            _reserved: [0; 3],
        }
    }

    /// Get path as byte slice
    pub fn path_bytes(&self) -> &[u8] {
        &self.path[..self.path_len as usize]
    }
}

// ============================================================================
// RamFS Directory Entry
// ============================================================================

/// Entry returned by the RAMFS_LIST syscall.
///
/// Shared between kernel and userspace — single source of truth.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RamfsListEntry {
    /// Filename (null-terminated)
    pub name: [u8; 100],
    /// File size in bytes
    pub size: u64,
    /// File type: 0 = regular, 1 = directory
    pub file_type: u8,
    /// Padding for alignment
    pub _pad: [u8; 7],
}

const _: () = assert!(core::mem::size_of::<RamfsListEntry>() == 120);

// ============================================================================
// ABI Size Assertions — kernel/user boundary types must not silently change
// ============================================================================

const _: () = assert!(core::mem::size_of::<ProcessInfo>() == 32);
const _: () = assert!(core::mem::size_of::<MuxEvent>() == 8);
const _: () = assert!(core::mem::size_of::<PciEnumEntry>() == 32);
const _: () = assert!(core::mem::size_of::<BusInfo>() == 48);
const _: () = assert!(core::mem::size_of::<DirEntry>() == 88);
const _: () = assert!(core::mem::size_of::<PipeProcessInfo>() == 44);
const _: () = assert!(core::mem::size_of::<TextChunk>() == 196);
const _: () = assert!(core::mem::size_of::<TableStart>() == 164);
const _: () = assert!(core::mem::size_of::<PipeError>() == 72);
const _: () = assert!(core::mem::size_of::<PipeMessageHeader>() == 8);

impl RamfsListEntry {
    pub const SIZE: usize = core::mem::size_of::<Self>();

    pub const fn empty() -> Self {
        Self { name: [0; 100], size: 0, file_type: 0, _pad: [0; 7] }
    }

    pub fn name_str(&self) -> &[u8] {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(100);
        &self.name[..len]
    }
}

// ============================================================================
// Pipe Messages
// ============================================================================

/// Pipe message header (8 bytes)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PipeMessageHeader {
    pub msg_type: u8,
    pub flags: u8,
    pub len: u16,        // payload length
    pub sequence: u32,   // for ordering/debug
}

impl PipeMessageHeader {
    pub const fn new(msg_type: PipeMessageType, len: u16) -> Self {
        Self {
            msg_type: msg_type as u8,
            flags: 0,
            len,
            sequence: 0,
        }
    }
}

// ============================================================================
// Port Enumeration (Unified Device Discovery)
// ============================================================================

/// High-level device/port classification
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortClass {
    Unknown = 0,

    // Storage
    Block = 1,              // Disk, partition, ramdisk
    StorageController = 2,  // NVMe, AHCI, SCSI controller

    // Network
    Network = 3,            // Ethernet, WiFi, switch port

    // Bus controllers
    Usb = 4,                // USB host controller or device
    Pcie = 5,               // PCIe root port or endpoint

    // Filesystems
    Filesystem = 6,         // Mounted filesystem

    // Platform
    Console = 7,            // TTY, serial port
    Gpio = 8,               // GPIO bank
    I2c = 9,                // I2C bus
    Spi = 10,               // SPI bus

    // Services
    Service = 11,           // Generic service port

    // Hardware-layer classes (kernel bus ports)
    Uart = 12,              // UART controller (kernel bus)
    Klog = 13,              // Kernel log (kernel bus)
    Ethernet = 14,          // Ethernet controller (kernel bus)
}

impl PortClass {
    pub fn from_u16(v: u16) -> Option<Self> {
        match v {
            0 => Some(PortClass::Unknown),
            1 => Some(PortClass::Block),
            2 => Some(PortClass::StorageController),
            3 => Some(PortClass::Network),
            4 => Some(PortClass::Usb),
            5 => Some(PortClass::Pcie),
            6 => Some(PortClass::Filesystem),
            7 => Some(PortClass::Console),
            8 => Some(PortClass::Gpio),
            9 => Some(PortClass::I2c),
            10 => Some(PortClass::Spi),
            11 => Some(PortClass::Service),
            12 => Some(PortClass::Uart),
            13 => Some(PortClass::Klog),
            14 => Some(PortClass::Ethernet),
            _ => None,
        }
    }
}

/// Port lifecycle state
///
/// All ports have an explicit state. Rules only fire on Ready transitions.
///
/// State machine:
///   Initialize ──→ Ready ──→ Disconnect
///                    ↑            │
///                    └─ Resetting ←┘
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortState {
    /// Port exists, owner setting up (hardware reset, driver init)
    Initialize = 0,
    /// Port is live, owner can process commands
    Ready = 1,
    /// Owner exited (clean or crash), port removed
    Disconnect = 2,
    /// Hardware/driver resetting, will return to Ready
    Resetting = 3,
}

impl PortState {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(PortState::Initialize),
            1 => Some(PortState::Ready),
            2 => Some(PortState::Disconnect),
            3 => Some(PortState::Resetting),
            _ => None,
        }
    }

    pub fn is_ready(&self) -> bool {
        matches!(self, PortState::Ready)
    }
}

/// Port subclass constants (class-specific values)
pub mod port_subclass {
    // Block subclasses (partition types from MBR/GPT)
    pub const BLOCK_RAW: u16 = 0x00;        // Unpartitioned disk
    pub const BLOCK_FAT12: u16 = 0x01;
    pub const BLOCK_FAT16: u16 = 0x06;
    pub const BLOCK_FAT32: u16 = 0x0b;
    pub const BLOCK_FAT32_LBA: u16 = 0x0c;
    pub const BLOCK_LINUX: u16 = 0x83;
    pub const BLOCK_GPT: u16 = 0xee;

    // USB subclasses (from interface class)
    pub const USB_XHCI: u16 = 0x30;
    pub const USB_HUB: u16 = 0x09;
    pub const USB_MSC: u16 = 0x08;
    pub const USB_HID: u16 = 0x03;

    // Network subclasses
    pub const NET_ETHERNET: u16 = 0x00;
    pub const NET_WIFI: u16 = 0x01;
    pub const NET_SWITCH_PORT: u16 = 0x02;
    pub const NET_SWITCH: u16 = 0x10;  // L2 switch (spawns switchd)
    pub const NET_BRIDGE_GROUP: u16 = 0x03;  // Bridge group port (spawns ipd per group)

    // Storage controller subclasses
    pub const STORAGE_NVME: u16 = 0x02;
    pub const STORAGE_AHCI: u16 = 0x06;

    // Filesystem subclasses
    pub const FS_FAT: u16 = 0x01;
    pub const FS_EXT4: u16 = 0x02;
    pub const FS_RAMFS: u16 = 0x10;

    // Console subclasses
    pub const CONSOLE_SERIAL: u16 = 0x00;  // Serial console (UART)
    pub const CONSOLE_VT: u16 = 0x01;      // Virtual terminal
}

/// Port capability bitflags
pub mod port_caps {
    pub const DMA: u16       = 1 << 0;   // Can do DMA
    pub const IRQ: u16       = 1 << 1;   // Has interrupt
    pub const MMIO: u16      = 1 << 2;   // Has MMIO region
    pub const READONLY: u16  = 1 << 3;   // Read-only device
    pub const REMOVABLE: u16 = 1 << 4;   // Hot-pluggable
    pub const INTERNAL: u16  = 1 << 5;   // Not user-facing
    pub const BOOTABLE: u16  = 1 << 6;   // Can boot from
    pub const MULTIDRV: u16  = 1 << 7;   // Supports multiple drivers
}

/// Maximum port name length
pub const PORT_NAME_MAX: usize = 32;

/// Type-safe metadata for different port classes
#[repr(C)]
#[derive(Clone, Copy)]
pub union PortMetadata {
    /// Block device metadata
    pub block: BlockMetadata,
    /// Network port metadata
    pub network: NetworkMetadata,
    /// USB device metadata
    pub usb: UsbMetadata,
    /// Generic bytes (for classes without specific metadata)
    pub raw: [u8; 24],
}

/// Block device metadata
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlockMetadata {
    /// Total size in bytes
    pub size_bytes: u64,
    /// Sector size (typically 512)
    pub sector_size: u32,
    /// Partition index (0 = whole disk)
    pub partition_index: u8,
    /// Flags
    pub flags: u8,
    /// Reserved
    pub _reserved: [u8; 10],
}

/// Network port metadata
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct NetworkMetadata {
    /// MAC address (6 bytes)
    pub mac: [u8; 6],
    /// Port number (for switches)
    pub port_number: u8,
    /// Flags
    pub flags: u8,
    /// Link speed in Mbps (0 = unknown)
    pub link_speed_mbps: u16,
    /// VLAN ID (0 = none)
    pub vlan_id: u16,
    /// Reserved
    pub _reserved: [u8; 12],
}

/// USB device metadata
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct UsbMetadata {
    /// USB device address
    pub device_addr: u8,
    /// USB speed (1=low, 2=full, 3=high, 4=super)
    pub speed: u8,
    /// Interface class
    pub iface_class: u8,
    /// Interface subclass
    pub iface_subclass: u8,
    /// Interface protocol
    pub iface_protocol: u8,
    /// Configuration number
    pub config_num: u8,
    /// Interface number
    pub iface_num: u8,
    /// Flags
    pub flags: u8,
    /// Reserved
    pub _reserved: [u8; 16],
}

impl BlockMetadata {
    pub const fn empty() -> Self {
        Self {
            size_bytes: 0,
            sector_size: 512,
            partition_index: 0,
            flags: 0,
            _reserved: [0; 10],
        }
    }
}

impl NetworkMetadata {
    pub const fn empty() -> Self {
        Self {
            mac: [0; 6],
            link_speed_mbps: 0,
            port_number: 0,
            vlan_id: 0,
            flags: 0,
            _reserved: [0; 12],
        }
    }
}

impl UsbMetadata {
    pub const fn empty() -> Self {
        Self {
            device_addr: 0,
            speed: 0,
            iface_class: 0,
            iface_subclass: 0,
            iface_protocol: 0,
            config_num: 0,
            iface_num: 0,
            flags: 0,
            _reserved: [0; 16],
        }
    }
}

/// Unified port enumeration record
///
/// Used by both kernel (hardware discovery) and drivers (sub-device enumeration).
/// This is the single source of truth for device information flowing through devd.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PortInfo {
    /// Port name (becomes the IPC port name, null-terminated)
    pub name: [u8; PORT_NAME_MAX],
    /// Length of name (excluding null terminator)
    pub name_len: u8,

    /// What kind of port is this?
    pub port_class: PortClass,

    /// Subclass for finer matching (class-specific, see port_subclass)
    pub port_subclass: u16,

    /// Parent port name (empty = root level)
    pub parent: [u8; PORT_NAME_MAX],
    /// Length of parent name
    pub parent_len: u8,

    /// Hardware vendor ID (0 = not applicable)
    pub vendor_id: u16,
    /// Hardware device ID (0 = not applicable)
    pub device_id: u16,

    /// Capability flags (see port_caps)
    pub caps: u16,

    /// Type-safe metadata (interpretation depends on port_class)
    pub metadata: PortMetadata,

    /// Reserved for future use
    pub _reserved: [u8; 4],
}

// Size assertion: ensure PortInfo has a stable size
const _: () = assert!(core::mem::size_of::<PortInfo>() == 112);
const _: () = assert!(core::mem::size_of::<PortMetadata>() == 24);
const _: () = assert!(core::mem::size_of::<BlockMetadata>() == 24);
const _: () = assert!(core::mem::size_of::<NetworkMetadata>() == 24);
const _: () = assert!(core::mem::size_of::<UsbMetadata>() == 24);

impl PortInfo {
    /// Create an empty PortInfo
    pub const fn empty() -> Self {
        Self {
            name: [0; PORT_NAME_MAX],
            name_len: 0,
            port_class: PortClass::Unknown,
            port_subclass: 0,
            parent: [0; PORT_NAME_MAX],
            parent_len: 0,
            vendor_id: 0,
            device_id: 0,
            caps: 0,
            metadata: PortMetadata { raw: [0; 24] },
            _reserved: [0; 4],
        }
    }

    /// Create a new PortInfo with the given name and class
    pub fn new(name: &[u8], class: PortClass) -> Self {
        let mut info = Self::empty();
        info.set_name(name);
        info.port_class = class;
        info
    }

    /// Set the port name
    pub fn set_name(&mut self, name: &[u8]) {
        let len = name.len().min(PORT_NAME_MAX - 1);
        self.name[..len].copy_from_slice(&name[..len]);
        self.name[len] = 0; // null terminate
        self.name_len = len as u8;
    }

    /// Get the port name as a byte slice
    pub fn name_bytes(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    /// Set the parent port name
    pub fn set_parent(&mut self, parent: &[u8]) {
        let len = parent.len().min(PORT_NAME_MAX - 1);
        self.parent[..len].copy_from_slice(&parent[..len]);
        self.parent[len] = 0;
        self.parent_len = len as u8;
    }

    /// Get the parent name as a byte slice
    pub fn parent_bytes(&self) -> &[u8] {
        &self.parent[..self.parent_len as usize]
    }

    /// Check if this port has a specific capability
    pub fn has_cap(&self, cap: u16) -> bool {
        (self.caps & cap) != 0
    }

    /// Set block metadata (only valid if port_class == Block)
    pub fn set_block_metadata(&mut self, meta: BlockMetadata) {
        self.metadata = PortMetadata { block: meta };
    }

    /// Set network metadata (only valid if port_class == Network)
    pub fn set_network_metadata(&mut self, meta: NetworkMetadata) {
        self.metadata = PortMetadata { network: meta };
    }

    /// Set USB metadata (only valid if port_class == Usb)
    pub fn set_usb_metadata(&mut self, meta: UsbMetadata) {
        self.metadata = PortMetadata { usb: meta };
    }

    /// Get block metadata (caller must verify port_class == Block)
    pub unsafe fn block_metadata(&self) -> &BlockMetadata {
        &self.metadata.block
    }

    /// Get network metadata (caller must verify port_class == Network)
    pub unsafe fn network_metadata(&self) -> &NetworkMetadata {
        &self.metadata.network
    }

    /// Get USB metadata (caller must verify port_class == Usb)
    pub unsafe fn usb_metadata(&self) -> &UsbMetadata {
        &self.metadata.usb
    }
}
