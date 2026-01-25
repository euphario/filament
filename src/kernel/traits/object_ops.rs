//! Object Operations Trait
//!
//! This trait defines the unified 5-syscall interface that is the heart
//! of the microkernel. All IPC, timers, shared memory, etc. go through
//! these operations.
//!
//! # The 5 Syscalls
//!
//! | Syscall | Method | Purpose |
//! |---------|--------|---------|
//! | open | create | Create object, return handle |
//! | read | read | Read from object (receive msg, accept conn, poll mux) |
//! | write | write | Write to object (send msg, arm timer, add to mux) |
//! | map | map | Map object to memory (shmem, DMA, MMIO) |
//! | close | close | Release handle |

use super::task::TaskId;
use super::waker::Subscriber;

/// Handle to a kernel object
pub type Handle = u32;

/// Object type enumeration
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
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
    /// PCIe bus access
    PciBus = 12,
    /// PCI device handle
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
}

/// Error type for object operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectError {
    /// Handle not found or invalid
    BadHandle,
    /// Invalid argument
    InvalidArgument,
    /// Operation would block
    WouldBlock,
    /// Out of memory / no slots
    OutOfMemory,
    /// Permission denied
    PermissionDenied,
    /// Object type doesn't support this operation
    NotSupported,
    /// Peer closed / connection reset
    Closed,
    /// Already exists
    AlreadyExists,
    /// Not found
    NotFound,
    /// Message too large
    MessageTooLarge,
}

impl ObjectError {
    pub fn to_errno(self) -> i64 {
        match self {
            ObjectError::BadHandle => -9,          // EBADF
            ObjectError::InvalidArgument => -22,   // EINVAL
            ObjectError::WouldBlock => -11,        // EAGAIN
            ObjectError::OutOfMemory => -12,       // ENOMEM
            ObjectError::PermissionDenied => -1,   // EPERM
            ObjectError::NotSupported => -95,      // EOPNOTSUPP
            ObjectError::Closed => -104,           // ECONNRESET
            ObjectError::AlreadyExists => -17,     // EEXIST
            ObjectError::NotFound => -2,           // ENOENT
            ObjectError::MessageTooLarge => -90,   // EMSGSIZE
        }
    }
}

/// Result of a read operation
pub struct ReadResult {
    /// Number of bytes read
    pub bytes_read: usize,
    /// For port accept: peer PID
    pub peer_pid: Option<TaskId>,
    /// Additional data (handle for accepted connection, etc.)
    pub extra: u64,
}

/// Result of a map operation
pub struct MapResult {
    /// Virtual address in task's address space
    pub vaddr: u64,
    /// Physical address (for DMA)
    pub paddr: u64,
    /// Size of mapping
    pub size: usize,
}

/// Trait for unified object operations
///
/// This is the heart of the microkernel. All IPC, timers, shared memory,
/// and device access goes through this interface.
///
/// # Contract
///
/// 1. Handles are task-local (same handle value in different tasks refers to different objects)
/// 2. Operations are type-checked against object type
/// 3. All operations are thread-safe
/// 4. Blocking operations use the subscriber mechanism for waking
pub trait ObjectOps: Send + Sync {
    /// Create a new object and return its handle
    ///
    /// # Arguments
    /// * `task_id` - Calling task
    /// * `obj_type` - Type of object to create
    /// * `flags` - Type-specific flags
    /// * `arg` - Type-specific argument
    ///
    /// # Object-specific behavior
    /// - Channel: Creates channel pair, arg is ignored
    /// - Port: arg points to name buffer
    /// - Timer: No special args
    /// - Shmem: arg is size
    /// - etc.
    fn create(
        &self,
        task_id: TaskId,
        obj_type: ObjectType,
        flags: u32,
        arg: u64,
    ) -> Result<Handle, ObjectError>;

    /// Read from an object
    ///
    /// # Arguments
    /// * `task_id` - Calling task
    /// * `handle` - Object handle
    /// * `buf` - Buffer to read into
    /// * `flags` - Operation flags (NONBLOCK, etc.)
    ///
    /// # Object-specific behavior
    /// - Channel: Receive message
    /// - Port: Accept connection
    /// - Timer: Wait for tick (returns nothing)
    /// - Mux: Poll and return ready handles
    /// - Stdin: Read input
    /// - Klog: Read kernel log record
    fn read(
        &self,
        task_id: TaskId,
        handle: Handle,
        buf: &mut [u8],
        flags: u32,
    ) -> Result<ReadResult, ObjectError>;

    /// Write to an object
    ///
    /// # Arguments
    /// * `task_id` - Calling task
    /// * `handle` - Object handle
    /// * `buf` - Buffer to write
    /// * `flags` - Operation flags
    ///
    /// # Object-specific behavior
    /// - Channel: Send message
    /// - Timer: Set deadline (buf contains u64 deadline)
    /// - Mux: Add/remove watch
    /// - Stdout/Stderr: Write output
    fn write(
        &self,
        task_id: TaskId,
        handle: Handle,
        buf: &[u8],
        flags: u32,
    ) -> Result<usize, ObjectError>;

    /// Map an object to memory
    ///
    /// # Arguments
    /// * `task_id` - Calling task
    /// * `handle` - Object handle
    ///
    /// # Object-specific behavior
    /// - Shmem: Map shared memory to address space
    /// - DmaPool: Map DMA buffer
    /// - Mmio: Map device registers
    fn map(&self, task_id: TaskId, handle: Handle) -> Result<MapResult, ObjectError>;

    /// Close an object handle
    ///
    /// Releases the handle and decrements reference count.
    /// Object is destroyed when last handle is closed.
    fn close(&self, task_id: TaskId, handle: Handle) -> Result<(), ObjectError>;

    /// Subscribe to object events for waking
    ///
    /// Used by blocking operations to register for wakeup.
    fn subscribe(
        &self,
        task_id: TaskId,
        handle: Handle,
        sub: Subscriber,
    ) -> Result<(), ObjectError>;
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

/// Mock ObjectOps implementation for unit testing
///
/// This allows testing code that depends on ObjectOps without a real kernel.
/// Configure behavior via the builder methods.
///
/// # Example
/// ```ignore
/// let mock = MockObjectOps::new()
///     .with_create_result(Ok(42))
///     .with_read_result(Ok(ReadResult { bytes_read: 10, peer_pid: None }));
/// test_code_that_uses_object_ops(&mock);
/// ```
#[cfg(any(test, feature = "mock"))]
pub struct MockObjectOps {
    create_result: Result<Handle, ObjectError>,
    read_result: Result<ReadResult, ObjectError>,
    write_result: Result<usize, ObjectError>,
    map_result: Result<MapResult, ObjectError>,
    close_result: Result<(), ObjectError>,
    subscribe_result: Result<(), ObjectError>,
}

#[cfg(any(test, feature = "mock"))]
impl MockObjectOps {
    /// Create a new mock with default (success) results
    pub const fn new() -> Self {
        Self {
            create_result: Ok(1), // Default handle
            read_result: Ok(ReadResult { bytes_read: 0, peer_pid: None }),
            write_result: Ok(0),
            map_result: Ok(MapResult { vaddr: 0, paddr: 0, size: 0 }),
            close_result: Ok(()),
            subscribe_result: Ok(()),
        }
    }

    /// Configure create() result
    pub const fn with_create_result(mut self, result: Result<Handle, ObjectError>) -> Self {
        self.create_result = result;
        self
    }

    /// Configure read() result
    pub const fn with_read_result(mut self, result: Result<ReadResult, ObjectError>) -> Self {
        self.read_result = result;
        self
    }

    /// Configure write() result
    pub const fn with_write_result(mut self, result: Result<usize, ObjectError>) -> Self {
        self.write_result = result;
        self
    }

    /// Configure map() result
    pub const fn with_map_result(mut self, result: Result<MapResult, ObjectError>) -> Self {
        self.map_result = result;
        self
    }

    /// Configure close() result
    pub const fn with_close_result(mut self, result: Result<(), ObjectError>) -> Self {
        self.close_result = result;
        self
    }

    /// Configure subscribe() result
    pub const fn with_subscribe_result(mut self, result: Result<(), ObjectError>) -> Self {
        self.subscribe_result = result;
        self
    }
}

#[cfg(any(test, feature = "mock"))]
impl ObjectOps for MockObjectOps {
    fn create(&self, _task_id: TaskId, _obj_type: ObjectType, _flags: u32, _arg: u64) -> Result<Handle, ObjectError> {
        self.create_result
    }

    fn read(&self, _task_id: TaskId, _handle: Handle, _buf: &mut [u8], _flags: u32) -> Result<ReadResult, ObjectError> {
        self.read_result
    }

    fn write(&self, _task_id: TaskId, _handle: Handle, _buf: &[u8], _flags: u32) -> Result<usize, ObjectError> {
        self.write_result
    }

    fn map(&self, _task_id: TaskId, _handle: Handle) -> Result<MapResult, ObjectError> {
        self.map_result
    }

    fn close(&self, _task_id: TaskId, _handle: Handle) -> Result<(), ObjectError> {
        self.close_result
    }

    fn subscribe(&self, _task_id: TaskId, _handle: Handle, _sub: Subscriber) -> Result<(), ObjectError> {
        self.subscribe_result
    }
}
