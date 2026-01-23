//! Task Operations Trait Definitions
//!
//! These traits define task-level operations not covered by SchedulerBackend.
//! This enables mock implementations for testing without hardware.
//!
//! # Design Philosophy
//!
//! SchedulerBackend handles scheduling operations (wake, sleep, yield).
//! TaskOperations handles task-level operations:
//! - Resource limits (channels, ports, shmem counts)
//! - Capabilities and signal permissions
//! - Event queuing
//! - Timer management
//!
//! # Usage
//!
//! ```ignore
//! // Check if task can create a channel
//! if task_backend().can_create_channel(task_id) {
//!     task_backend().add_channel(task_id)?;
//!     // create the channel...
//! }
//!
//! // Check capability
//! if task_backend().has_capability(task_id, Capability::PciAccess) {
//!     // allow PCI operation...
//! }
//! ```

/// Task ID type (matches Pid from process module)
pub type TaskId = u32;

// ============================================================================
// Resource Tracking
// ============================================================================

/// Resource counts for a task
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ResourceCounts {
    /// Number of channels created
    pub channels: u16,
    /// Number of ports registered
    pub ports: u16,
    /// Number of shmem regions mapped
    pub shmem: u16,
}

// ============================================================================
// Capabilities
// ============================================================================

/// Capability flags for task permissions
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct Capabilities(pub u64);

impl Capabilities {
    /// No capabilities
    pub const NONE: Capabilities = Capabilities(0);

    /// Can access PCI configuration space
    pub const PCI_ACCESS: u64 = 1 << 0;
    /// Can map device MMIO regions
    pub const MMIO_MAP: u64 = 1 << 1;
    /// Can allocate DMA buffers
    pub const DMA_ALLOC: u64 = 1 << 2;
    /// Can kill other processes
    pub const KILL: u64 = 1 << 3;
    /// Can access kernel log
    pub const KLOG: u64 = 1 << 4;
    /// Can spawn processes
    pub const SPAWN: u64 = 1 << 5;
    /// Can register ports (services)
    pub const PORT_REGISTER: u64 = 1 << 6;
    /// Can access bus controller
    pub const BUS_ACCESS: u64 = 1 << 7;

    /// Create from raw bits
    pub const fn from_bits(bits: u64) -> Self {
        Capabilities(bits)
    }

    /// Get raw bits
    pub const fn bits(&self) -> u64 {
        self.0
    }

    /// Check if this has a specific capability
    pub const fn has(&self, cap: u64) -> bool {
        (self.0 & cap) != 0
    }

    /// Add a capability
    pub const fn with(self, cap: u64) -> Self {
        Capabilities(self.0 | cap)
    }

    /// Remove a capability
    pub const fn without(self, cap: u64) -> Self {
        Capabilities(self.0 & !cap)
    }

    /// Check if all of the given capabilities are present
    pub const fn has_all(&self, caps: u64) -> bool {
        (self.0 & caps) == caps
    }
}

impl Default for Capabilities {
    fn default() -> Self {
        Capabilities::NONE
    }
}

// ============================================================================
// Error Type
// ============================================================================

/// Task operation errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskError {
    /// Task not found
    NotFound,
    /// Resource limit reached
    LimitReached,
    /// Invalid operation for current state
    InvalidState,
    /// Permission denied
    PermissionDenied,
    /// Signal allowlist full
    AllowlistFull,
    /// Timer not found
    TimerNotFound,
    /// Event queue full
    EventQueueFull,
}

impl TaskError {
    pub fn to_errno(self) -> i64 {
        match self {
            TaskError::NotFound => -3,         // ESRCH
            TaskError::LimitReached => -12,    // ENOMEM
            TaskError::InvalidState => -22,    // EINVAL
            TaskError::PermissionDenied => -1, // EPERM
            TaskError::AllowlistFull => -28,   // ENOSPC
            TaskError::TimerNotFound => -2,    // ENOENT
            TaskError::EventQueueFull => -11,  // EAGAIN
        }
    }
}

// ============================================================================
// Task Operations Trait
// ============================================================================

/// Trait for task-level operations
///
/// Provides operations on individual tasks that are not related to scheduling.
/// This complements SchedulerBackend which handles wake/sleep/yield.
///
/// # Contract
///
/// 1. All operations are thread-safe (internal locking)
/// 2. TaskId 0 is typically the kernel (no user task)
/// 3. Resource limits are enforced per-task
/// 4. Capabilities are inherited at spawn time
pub trait TaskOperations: Send + Sync {
    // ========================================================================
    // Resource Limits
    // ========================================================================

    /// Check if task can create another channel
    fn can_create_channel(&self, task_id: TaskId) -> bool;

    /// Increment task's channel count
    fn add_channel(&self, task_id: TaskId) -> Result<(), TaskError>;

    /// Decrement task's channel count
    fn remove_channel(&self, task_id: TaskId);

    /// Check if task can register another port
    fn can_create_port(&self, task_id: TaskId) -> bool;

    /// Increment task's port count
    fn add_port(&self, task_id: TaskId) -> Result<(), TaskError>;

    /// Decrement task's port count
    fn remove_port(&self, task_id: TaskId);

    /// Check if task can create another shmem mapping
    fn can_create_shmem(&self, task_id: TaskId) -> bool;

    /// Increment task's shmem count
    fn add_shmem(&self, task_id: TaskId) -> Result<(), TaskError>;

    /// Decrement task's shmem count
    fn remove_shmem(&self, task_id: TaskId);

    /// Get current resource counts for task
    fn get_resource_counts(&self, task_id: TaskId) -> Option<ResourceCounts>;

    // ========================================================================
    // Capabilities
    // ========================================================================

    /// Get task's capabilities
    fn get_capabilities(&self, task_id: TaskId) -> Option<Capabilities>;

    /// Check if task has a specific capability
    fn has_capability(&self, task_id: TaskId, cap: u64) -> bool;

    /// Set task's capabilities (for spawn)
    fn set_capabilities(&self, task_id: TaskId, caps: Capabilities) -> Result<(), TaskError>;

    // ========================================================================
    // Signal Permissions
    // ========================================================================

    /// Check if task can receive signals from sender
    fn can_receive_signal_from(&self, task_id: TaskId, sender: TaskId) -> bool;

    /// Allow task to receive signals from sender
    fn allow_signals_from(&self, task_id: TaskId, sender: TaskId) -> Result<(), TaskError>;

    // ========================================================================
    // Events
    // ========================================================================

    /// Check if task has pending events
    fn has_pending_events(&self, task_id: TaskId) -> bool;

    /// Get count of pending events
    fn pending_event_count(&self, task_id: TaskId) -> usize;

    // ========================================================================
    // Timers
    // ========================================================================

    /// Arm a timer for the task
    ///
    /// # Arguments
    /// * `task_id` - Target task
    /// * `timer_id` - Timer slot (0-7)
    /// * `deadline` - Tick when timer fires
    /// * `interval` - Interval for recurring (0 = one-shot)
    fn arm_timer(
        &self,
        task_id: TaskId,
        timer_id: u32,
        deadline: u64,
        interval: u64,
    ) -> Result<(), TaskError>;

    /// Disarm a timer
    fn disarm_timer(&self, task_id: TaskId, timer_id: u32) -> Result<(), TaskError>;

    /// Check if timer is armed
    fn is_timer_armed(&self, task_id: TaskId, timer_id: u32) -> bool;

    // ========================================================================
    // Task Info
    // ========================================================================

    /// Get task name (32 bytes, null-terminated)
    fn get_name(&self, task_id: TaskId) -> Option<[u8; 32]>;

    /// Get task's parent ID
    fn get_parent(&self, task_id: TaskId) -> Option<TaskId>;

    /// Check if task exists
    fn exists(&self, task_id: TaskId) -> bool {
        self.get_name(task_id).is_some()
    }
}
