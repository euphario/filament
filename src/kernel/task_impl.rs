//! Task Operations Backend Implementation
//!
//! Implements the `TaskOperations` trait by delegating to the existing
//! task module. This provides a clean trait boundary for testing without
//! changing the internal implementation.
//!
//! # Thread Safety
//!
//! All operations go through `with_scheduler()` which holds the scheduler
//! lock with IRQ-save semantics, ensuring proper serialization on SMP systems.

use crate::kernel::traits::task::{
    TaskOperations, TaskId, ResourceCounts, Capabilities as TraitCapabilities, TaskError,
};
use crate::kernel::task::{self, tcb::MAX_TIMERS_PER_TASK};
use crate::kernel::caps::Capabilities as KernelCapabilities;

// ============================================================================
// Type Conversions
// ============================================================================

/// Convert kernel Capabilities to trait Capabilities
fn convert_caps_to_trait(caps: KernelCapabilities) -> TraitCapabilities {
    TraitCapabilities::from_bits(caps.bits())
}

/// Convert trait Capabilities to kernel Capabilities
fn convert_caps_from_trait(caps: TraitCapabilities) -> KernelCapabilities {
    KernelCapabilities::from_bits(caps.bits())
}

// ============================================================================
// Kernel Task Operations Implementation
// ============================================================================

/// Kernel task operations backend implementation
///
/// A zero-sized type that implements `TaskOperations` by delegating to the
/// global scheduler via `with_scheduler()`.
pub struct KernelTaskOperations;

impl KernelTaskOperations {
    /// Create a new kernel task operations instance
    pub const fn new() -> Self {
        Self
    }
}

impl TaskOperations for KernelTaskOperations {
    // ========================================================================
    // Resource Limits
    // ========================================================================

    fn can_create_channel(&self, task_id: TaskId) -> bool {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.can_create_channel())
                .unwrap_or(false)
        })
    }

    fn add_channel(&self, task_id: TaskId) -> Result<(), TaskError> {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    if t.can_create_channel() {
                        t.add_channel();
                        return Ok(());
                    } else {
                        return Err(TaskError::LimitReached);
                    }
                }
            }
            Err(TaskError::NotFound)
        })
    }

    fn remove_channel(&self, task_id: TaskId) {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    t.remove_channel();
                }
            }
        });
    }

    fn can_create_port(&self, task_id: TaskId) -> bool {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.can_create_port())
                .unwrap_or(false)
        })
    }

    fn add_port(&self, task_id: TaskId) -> Result<(), TaskError> {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    if t.can_create_port() {
                        t.add_port();
                        return Ok(());
                    } else {
                        return Err(TaskError::LimitReached);
                    }
                }
            }
            Err(TaskError::NotFound)
        })
    }

    fn remove_port(&self, task_id: TaskId) {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    t.remove_port();
                }
            }
        });
    }

    fn can_create_shmem(&self, task_id: TaskId) -> bool {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.can_create_shmem())
                .unwrap_or(false)
        })
    }

    fn add_shmem(&self, task_id: TaskId) -> Result<(), TaskError> {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    if t.can_create_shmem() {
                        t.add_shmem();
                        return Ok(());
                    } else {
                        return Err(TaskError::LimitReached);
                    }
                }
            }
            Err(TaskError::NotFound)
        })
    }

    fn remove_shmem(&self, task_id: TaskId) {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    t.remove_shmem();
                }
            }
        });
    }

    fn get_resource_counts(&self, task_id: TaskId) -> Option<ResourceCounts> {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| ResourceCounts {
                    channels: t.channel_count,
                    ports: t.port_count,
                    shmem: t.shmem_count,
                })
        })
    }

    // ========================================================================
    // Capabilities
    // ========================================================================

    fn get_capabilities(&self, task_id: TaskId) -> Option<TraitCapabilities> {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| convert_caps_to_trait(t.capabilities))
        })
    }

    fn has_capability(&self, task_id: TaskId, cap: u64) -> bool {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.capabilities.has(KernelCapabilities::from_bits(cap)))
                .unwrap_or(false)
        })
    }

    fn set_capabilities(&self, task_id: TaskId, caps: TraitCapabilities) -> Result<(), TaskError> {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    t.set_capabilities(convert_caps_from_trait(caps));
                    return Ok(());
                }
            }
            Err(TaskError::NotFound)
        })
    }

    // ========================================================================
    // Signal Permissions
    // ========================================================================

    fn can_receive_signal_from(&self, task_id: TaskId, sender: TaskId) -> bool {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.can_receive_signal_from(sender))
                .unwrap_or(false)
        })
    }

    fn allow_signals_from(&self, task_id: TaskId, sender: TaskId) -> Result<(), TaskError> {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    if t.allow_signals_from(sender) {
                        return Ok(());
                    } else {
                        return Err(TaskError::AllowlistFull);
                    }
                }
            }
            Err(TaskError::NotFound)
        })
    }

    // ========================================================================
    // Events
    // ========================================================================

    fn has_pending_events(&self, task_id: TaskId) -> bool {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.event_queue.has_events())
                .unwrap_or(false)
        })
    }

    fn pending_event_count(&self, task_id: TaskId) -> usize {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.event_queue.len())
                .unwrap_or(0)
        })
    }

    // ========================================================================
    // Timers
    // ========================================================================

    fn arm_timer(
        &self,
        task_id: TaskId,
        timer_id: u32,
        deadline: u64,
        interval: u64,
    ) -> Result<(), TaskError> {
        if timer_id as usize >= MAX_TIMERS_PER_TASK {
            return Err(TaskError::TimerNotFound);
        }

        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    t.timers[timer_id as usize].id = timer_id;
                    t.timers[timer_id as usize].deadline = deadline;
                    t.timers[timer_id as usize].interval = interval;
                    // Note the deadline for tickless optimization
                    sched.note_deadline(deadline);
                    return Ok(());
                }
            }
            Err(TaskError::NotFound)
        })
    }

    fn disarm_timer(&self, task_id: TaskId, timer_id: u32) -> Result<(), TaskError> {
        if timer_id as usize >= MAX_TIMERS_PER_TASK {
            return Err(TaskError::TimerNotFound);
        }

        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(t) = sched.task_mut(slot) {
                    t.timers[timer_id as usize].deadline = 0;
                    t.timers[timer_id as usize].interval = 0;
                    return Ok(());
                }
            }
            Err(TaskError::NotFound)
        })
    }

    fn is_timer_armed(&self, task_id: TaskId, timer_id: u32) -> bool {
        if timer_id as usize >= MAX_TIMERS_PER_TASK {
            return false;
        }

        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.timers[timer_id as usize].is_active())
                .unwrap_or(false)
        })
    }

    // ========================================================================
    // Task Info
    // ========================================================================

    fn get_name(&self, task_id: TaskId) -> Option<[u8; 32]> {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| {
                    // Task has 16 byte name, trait expects 32
                    let mut name = [0u8; 32];
                    name[..16].copy_from_slice(&t.name);
                    name
                })
        })
    }

    fn get_parent(&self, task_id: TaskId) -> Option<TaskId> {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.parent_id)
        })
    }

    fn exists(&self, task_id: TaskId) -> bool {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id).is_some()
        })
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel task operations backend
pub static TASK_BACKEND: KernelTaskOperations = KernelTaskOperations::new();

/// Get a reference to the global task operations backend
pub fn task_backend() -> &'static dyn TaskOperations {
    &TASK_BACKEND
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

#[cfg(test)]
pub mod mock {
    use super::*;
    use core::cell::RefCell;

    /// Maximum tasks in mock
    const MAX_MOCK_TASKS: usize = 16;
    /// Maximum signal allowlist entries
    const MAX_ALLOWLIST: usize = 8;
    /// Maximum timers per task
    const MAX_TIMERS: usize = 8;

    /// Mock task info
    #[derive(Clone)]
    struct MockTaskInfo {
        id: TaskId,
        parent_id: TaskId,
        name: [u8; 32],
        channels: u16,
        ports: u16,
        shmem: u16,
        capabilities: TraitCapabilities,
        signal_allowlist: [TaskId; MAX_ALLOWLIST],
        signal_allowlist_count: usize,
        event_count: usize,
        timers: [(u64, u64); MAX_TIMERS], // (deadline, interval)
    }

    impl Default for MockTaskInfo {
        fn default() -> Self {
            Self {
                id: 0,
                parent_id: 0,
                name: [0u8; 32],
                channels: 0,
                ports: 0,
                shmem: 0,
                capabilities: TraitCapabilities::NONE,
                signal_allowlist: [0; MAX_ALLOWLIST],
                signal_allowlist_count: 0,
                event_count: 0,
                timers: [(0, 0); MAX_TIMERS],
            }
        }
    }

    /// Mock task operations for testing
    pub struct MockTaskOperations {
        tasks: RefCell<[Option<MockTaskInfo>; MAX_MOCK_TASKS]>,
        max_channels: u16,
        max_ports: u16,
        max_shmem: u16,
    }

    impl MockTaskOperations {
        /// Create a new mock with default limits
        pub fn new() -> Self {
            const NONE: Option<MockTaskInfo> = None;
            Self {
                tasks: RefCell::new([NONE; MAX_MOCK_TASKS]),
                max_channels: 32,
                max_ports: 4,
                max_shmem: 16,
            }
        }

        /// Create a mock with custom limits
        pub fn with_limits(max_channels: u16, max_ports: u16, max_shmem: u16) -> Self {
            const NONE: Option<MockTaskInfo> = None;
            Self {
                tasks: RefCell::new([NONE; MAX_MOCK_TASKS]),
                max_channels,
                max_ports,
                max_shmem,
            }
        }

        /// Add a task to the mock
        pub fn add_task(&self, task_id: TaskId, parent_id: TaskId, name: &str) -> bool {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if slot.is_none() {
                    let mut task = MockTaskInfo::default();
                    task.id = task_id;
                    task.parent_id = parent_id;
                    let name_bytes = name.as_bytes();
                    let copy_len = core::cmp::min(name_bytes.len(), 31);
                    task.name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);
                    *slot = Some(task);
                    return true;
                }
            }
            false
        }

        /// Remove a task from the mock
        pub fn remove_task(&self, task_id: TaskId) -> bool {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        *slot = None;
                        return true;
                    }
                }
            }
            false
        }

        /// Set capabilities for a task
        pub fn set_task_capabilities(&self, task_id: TaskId, caps: TraitCapabilities) -> bool {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        t.capabilities = caps;
                        return true;
                    }
                }
            }
            false
        }

        /// Add pending events to a task
        pub fn add_events(&self, task_id: TaskId, count: usize) -> bool {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        t.event_count += count;
                        return true;
                    }
                }
            }
            false
        }

        /// Get all task IDs
        pub fn task_ids(&self) -> Vec<TaskId> {
            let tasks = self.tasks.borrow();
            tasks.iter()
                .filter_map(|slot| slot.as_ref().map(|t| t.id))
                .collect()
        }
    }

    impl Default for MockTaskOperations {
        fn default() -> Self {
            Self::new()
        }
    }

    impl TaskOperations for MockTaskOperations {
        fn can_create_channel(&self, task_id: TaskId) -> bool {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return t.channels < self.max_channels;
                    }
                }
            }
            false
        }

        fn add_channel(&self, task_id: TaskId) -> Result<(), TaskError> {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        if t.channels < self.max_channels {
                            t.channels += 1;
                            return Ok(());
                        } else {
                            return Err(TaskError::LimitReached);
                        }
                    }
                }
            }
            Err(TaskError::NotFound)
        }

        fn remove_channel(&self, task_id: TaskId) {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id && t.channels > 0 {
                        t.channels -= 1;
                        return;
                    }
                }
            }
        }

        fn can_create_port(&self, task_id: TaskId) -> bool {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return t.ports < self.max_ports;
                    }
                }
            }
            false
        }

        fn add_port(&self, task_id: TaskId) -> Result<(), TaskError> {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        if t.ports < self.max_ports {
                            t.ports += 1;
                            return Ok(());
                        } else {
                            return Err(TaskError::LimitReached);
                        }
                    }
                }
            }
            Err(TaskError::NotFound)
        }

        fn remove_port(&self, task_id: TaskId) {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id && t.ports > 0 {
                        t.ports -= 1;
                        return;
                    }
                }
            }
        }

        fn can_create_shmem(&self, task_id: TaskId) -> bool {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return t.shmem < self.max_shmem;
                    }
                }
            }
            false
        }

        fn add_shmem(&self, task_id: TaskId) -> Result<(), TaskError> {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        if t.shmem < self.max_shmem {
                            t.shmem += 1;
                            return Ok(());
                        } else {
                            return Err(TaskError::LimitReached);
                        }
                    }
                }
            }
            Err(TaskError::NotFound)
        }

        fn remove_shmem(&self, task_id: TaskId) {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id && t.shmem > 0 {
                        t.shmem -= 1;
                        return;
                    }
                }
            }
        }

        fn get_resource_counts(&self, task_id: TaskId) -> Option<ResourceCounts> {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return Some(ResourceCounts {
                            channels: t.channels,
                            ports: t.ports,
                            shmem: t.shmem,
                        });
                    }
                }
            }
            None
        }

        fn get_capabilities(&self, task_id: TaskId) -> Option<TraitCapabilities> {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return Some(t.capabilities);
                    }
                }
            }
            None
        }

        fn has_capability(&self, task_id: TaskId, cap: u64) -> bool {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return t.capabilities.has(cap);
                    }
                }
            }
            false
        }

        fn set_capabilities(&self, task_id: TaskId, caps: TraitCapabilities) -> Result<(), TaskError> {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        t.capabilities = caps;
                        return Ok(());
                    }
                }
            }
            Err(TaskError::NotFound)
        }

        fn can_receive_signal_from(&self, task_id: TaskId, sender: TaskId) -> bool {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        // If allowlist is empty, allow all
                        if t.signal_allowlist_count == 0 {
                            return true;
                        }
                        // Parent always allowed
                        if sender == t.parent_id && t.parent_id != 0 {
                            return true;
                        }
                        // Check allowlist
                        for i in 0..t.signal_allowlist_count {
                            if t.signal_allowlist[i] == sender {
                                return true;
                            }
                        }
                        return false;
                    }
                }
            }
            false
        }

        fn allow_signals_from(&self, task_id: TaskId, sender: TaskId) -> Result<(), TaskError> {
            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        // Check if already in list
                        for i in 0..t.signal_allowlist_count {
                            if t.signal_allowlist[i] == sender {
                                return Ok(());
                            }
                        }
                        // Add if space
                        if t.signal_allowlist_count < MAX_ALLOWLIST {
                            t.signal_allowlist[t.signal_allowlist_count] = sender;
                            t.signal_allowlist_count += 1;
                            return Ok(());
                        } else {
                            return Err(TaskError::AllowlistFull);
                        }
                    }
                }
            }
            Err(TaskError::NotFound)
        }

        fn has_pending_events(&self, task_id: TaskId) -> bool {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return t.event_count > 0;
                    }
                }
            }
            false
        }

        fn pending_event_count(&self, task_id: TaskId) -> usize {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return t.event_count;
                    }
                }
            }
            0
        }

        fn arm_timer(
            &self,
            task_id: TaskId,
            timer_id: u32,
            deadline: u64,
            interval: u64,
        ) -> Result<(), TaskError> {
            if timer_id as usize >= MAX_TIMERS {
                return Err(TaskError::TimerNotFound);
            }

            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        t.timers[timer_id as usize] = (deadline, interval);
                        return Ok(());
                    }
                }
            }
            Err(TaskError::NotFound)
        }

        fn disarm_timer(&self, task_id: TaskId, timer_id: u32) -> Result<(), TaskError> {
            if timer_id as usize >= MAX_TIMERS {
                return Err(TaskError::TimerNotFound);
            }

            let mut tasks = self.tasks.borrow_mut();
            for slot in tasks.iter_mut() {
                if let Some(ref mut t) = slot {
                    if t.id == task_id {
                        t.timers[timer_id as usize] = (0, 0);
                        return Ok(());
                    }
                }
            }
            Err(TaskError::NotFound)
        }

        fn is_timer_armed(&self, task_id: TaskId, timer_id: u32) -> bool {
            if timer_id as usize >= MAX_TIMERS {
                return false;
            }

            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return t.timers[timer_id as usize].0 != 0;
                    }
                }
            }
            false
        }

        fn get_name(&self, task_id: TaskId) -> Option<[u8; 32]> {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return Some(t.name);
                    }
                }
            }
            None
        }

        fn get_parent(&self, task_id: TaskId) -> Option<TaskId> {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return Some(t.parent_id);
                    }
                }
            }
            None
        }

        fn exists(&self, task_id: TaskId) -> bool {
            let tasks = self.tasks.borrow();
            for slot in tasks.iter() {
                if let Some(ref t) = slot {
                    if t.id == task_id {
                        return true;
                    }
                }
            }
            false
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use super::mock::MockTaskOperations;

    #[test]
    fn test_mock_task_creation() {
        let mock = MockTaskOperations::new();
        assert!(mock.add_task(1, 0, "init"));
        assert!(mock.exists(1));
        assert!(!mock.exists(2));
    }

    #[test]
    fn test_mock_task_removal() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");
        assert!(mock.exists(1));
        assert!(mock.remove_task(1));
        assert!(!mock.exists(1));
    }

    #[test]
    fn test_channel_resource_limits() {
        let mock = MockTaskOperations::with_limits(2, 4, 16);
        mock.add_task(1, 0, "test");

        // Can create first channel
        assert!(mock.can_create_channel(1));
        assert!(mock.add_channel(1).is_ok());

        // Can create second channel
        assert!(mock.can_create_channel(1));
        assert!(mock.add_channel(1).is_ok());

        // At limit - can't create third
        assert!(!mock.can_create_channel(1));
        assert!(matches!(mock.add_channel(1), Err(TaskError::LimitReached)));

        // Remove one - can create again
        mock.remove_channel(1);
        assert!(mock.can_create_channel(1));
    }

    #[test]
    fn test_port_resource_limits() {
        let mock = MockTaskOperations::with_limits(32, 2, 16);
        mock.add_task(1, 0, "test");

        assert!(mock.add_port(1).is_ok());
        assert!(mock.add_port(1).is_ok());
        assert!(matches!(mock.add_port(1), Err(TaskError::LimitReached)));
    }

    #[test]
    fn test_shmem_resource_limits() {
        let mock = MockTaskOperations::with_limits(32, 4, 2);
        mock.add_task(1, 0, "test");

        assert!(mock.add_shmem(1).is_ok());
        assert!(mock.add_shmem(1).is_ok());
        assert!(matches!(mock.add_shmem(1), Err(TaskError::LimitReached)));
    }

    #[test]
    fn test_resource_counts() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");

        mock.add_channel(1).unwrap();
        mock.add_channel(1).unwrap();
        mock.add_port(1).unwrap();
        mock.add_shmem(1).unwrap();
        mock.add_shmem(1).unwrap();
        mock.add_shmem(1).unwrap();

        let counts = mock.get_resource_counts(1).unwrap();
        assert_eq!(counts.channels, 2);
        assert_eq!(counts.ports, 1);
        assert_eq!(counts.shmem, 3);
    }

    #[test]
    fn test_capabilities() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");

        // Initially no capabilities
        let caps = mock.get_capabilities(1).unwrap();
        assert_eq!(caps.bits(), 0);

        // Set some capabilities
        let new_caps = TraitCapabilities::from_bits(
            TraitCapabilities::PCI_ACCESS | TraitCapabilities::DMA_ALLOC
        );
        mock.set_capabilities(1, new_caps).unwrap();

        // Verify has_capability
        assert!(mock.has_capability(1, TraitCapabilities::PCI_ACCESS));
        assert!(mock.has_capability(1, TraitCapabilities::DMA_ALLOC));
        assert!(!mock.has_capability(1, TraitCapabilities::KILL));
    }

    #[test]
    fn test_signal_permissions_empty_allowlist() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "receiver");
        mock.add_task(2, 0, "sender");

        // Empty allowlist = allow all
        assert!(mock.can_receive_signal_from(1, 2));
        assert!(mock.can_receive_signal_from(1, 999));
    }

    #[test]
    fn test_signal_permissions_with_allowlist() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "receiver");
        mock.add_task(2, 0, "allowed");
        mock.add_task(3, 0, "blocked");

        // Add sender 2 to allowlist
        mock.allow_signals_from(1, 2).unwrap();

        // Only sender 2 can send
        assert!(mock.can_receive_signal_from(1, 2));
        assert!(!mock.can_receive_signal_from(1, 3));
    }

    #[test]
    fn test_signal_permissions_parent_always_allowed() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "parent");
        mock.add_task(2, 1, "child"); // Parent is 1

        // Add some other sender to create a non-empty allowlist
        mock.allow_signals_from(2, 99).unwrap();

        // Parent (1) should still be allowed even if not in allowlist
        assert!(mock.can_receive_signal_from(2, 1));
    }

    #[test]
    fn test_signal_allowlist_full() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "receiver");

        // Fill up the allowlist
        for i in 10..18 {
            assert!(mock.allow_signals_from(1, i).is_ok());
        }

        // Should be full now
        assert!(matches!(mock.allow_signals_from(1, 100), Err(TaskError::AllowlistFull)));
    }

    #[test]
    fn test_pending_events() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");

        assert!(!mock.has_pending_events(1));
        assert_eq!(mock.pending_event_count(1), 0);

        mock.add_events(1, 3);
        assert!(mock.has_pending_events(1));
        assert_eq!(mock.pending_event_count(1), 3);
    }

    #[test]
    fn test_timer_operations() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");

        // Initially not armed
        assert!(!mock.is_timer_armed(1, 0));

        // Arm timer
        mock.arm_timer(1, 0, 1000, 100).unwrap();
        assert!(mock.is_timer_armed(1, 0));

        // Disarm timer
        mock.disarm_timer(1, 0).unwrap();
        assert!(!mock.is_timer_armed(1, 0));
    }

    #[test]
    fn test_timer_invalid_id() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");

        // Timer ID 8 is out of range (0-7 valid)
        assert!(matches!(mock.arm_timer(1, 8, 1000, 0), Err(TaskError::TimerNotFound)));
        assert!(matches!(mock.disarm_timer(1, 8), Err(TaskError::TimerNotFound)));
    }

    #[test]
    fn test_multiple_timers() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");

        // Arm multiple timers
        mock.arm_timer(1, 0, 1000, 0).unwrap();
        mock.arm_timer(1, 3, 2000, 500).unwrap();
        mock.arm_timer(1, 7, 3000, 0).unwrap();

        assert!(mock.is_timer_armed(1, 0));
        assert!(!mock.is_timer_armed(1, 1));
        assert!(!mock.is_timer_armed(1, 2));
        assert!(mock.is_timer_armed(1, 3));
        assert!(mock.is_timer_armed(1, 7));
    }

    #[test]
    fn test_task_name() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test_task");

        let name = mock.get_name(1).unwrap();
        assert_eq!(&name[..9], b"test_task");
        assert_eq!(name[9], 0); // Null terminated
    }

    #[test]
    fn test_task_parent() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "parent");
        mock.add_task(2, 1, "child");

        assert_eq!(mock.get_parent(1), Some(0));
        assert_eq!(mock.get_parent(2), Some(1));
    }

    #[test]
    fn test_nonexistent_task() {
        let mock = MockTaskOperations::new();

        // All operations on non-existent task should fail gracefully
        assert!(!mock.can_create_channel(999));
        assert!(matches!(mock.add_channel(999), Err(TaskError::NotFound)));
        assert!(mock.get_resource_counts(999).is_none());
        assert!(mock.get_capabilities(999).is_none());
        assert!(!mock.has_capability(999, TraitCapabilities::PCI_ACCESS));
        assert!(!mock.can_receive_signal_from(999, 1));
        assert!(matches!(mock.allow_signals_from(999, 1), Err(TaskError::NotFound)));
        assert!(!mock.has_pending_events(999));
        assert!(matches!(mock.arm_timer(999, 0, 1000, 0), Err(TaskError::NotFound)));
        assert!(mock.get_name(999).is_none());
        assert!(mock.get_parent(999).is_none());
        assert!(!mock.exists(999));
    }

    #[test]
    fn test_resource_removal_underflow() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");

        // Remove when count is 0 - should not underflow
        mock.remove_channel(1);
        mock.remove_port(1);
        mock.remove_shmem(1);

        let counts = mock.get_resource_counts(1).unwrap();
        assert_eq!(counts.channels, 0);
        assert_eq!(counts.ports, 0);
        assert_eq!(counts.shmem, 0);
    }
}
