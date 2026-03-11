//! Task Operations Backend Implementation
//!
//! Implements the `TaskOperations` trait by delegating to the ProcessTable.
//! All metadata queries (resource counts, capabilities, signals, names)
//! go through the ProcessTable lock — the scheduler lock is never touched.

use crate::kernel::traits::task::{
    TaskOperations, TaskId, ResourceCounts, Capabilities as TraitCapabilities, TaskError,
};
use crate::kernel::task;
use crate::kernel::caps::Capabilities as KernelCapabilities;
use crate::kernel::process;

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

/// Resolve a PID to a scheduler slot index.
fn slot_for(task_id: TaskId) -> Option<usize> {
    task::with_scheduler(|sched| sched.slot_by_pid(task_id))
}

// ============================================================================
// Kernel Task Operations Implementation
// ============================================================================

/// Kernel task operations backend implementation
///
/// A zero-sized type that implements `TaskOperations` by delegating to the
/// ProcessTable for all metadata queries.
pub struct KernelTaskOperations;

impl KernelTaskOperations {
    pub const fn new() -> Self {
        Self
    }
}

impl TaskOperations for KernelTaskOperations {
    // ========================================================================
    // Resource Limits
    // ========================================================================

    fn can_create_channel(&self, task_id: TaskId) -> bool {
        slot_for(task_id)
            .and_then(|s| process::process_table().with(s, |p| p.can_create_channel()))
            .unwrap_or(false)
    }

    fn add_channel(&self, task_id: TaskId) -> Result<(), TaskError> {
        let slot = slot_for(task_id).ok_or(TaskError::NotFound)?;
        process::process_table().with_mut(slot, |p| {
            if p.can_create_channel() {
                p.add_channel();
                Ok(())
            } else {
                Err(TaskError::LimitReached)
            }
        }).unwrap_or(Err(TaskError::NotFound))
    }

    fn remove_channel(&self, task_id: TaskId) {
        if let Some(slot) = slot_for(task_id) {
            process::process_table().with_mut(slot, |p| p.remove_channel());
        }
    }

    fn can_create_port(&self, task_id: TaskId) -> bool {
        slot_for(task_id)
            .and_then(|s| process::process_table().with(s, |p| p.can_create_port()))
            .unwrap_or(false)
    }

    fn add_port(&self, task_id: TaskId) -> Result<(), TaskError> {
        let slot = slot_for(task_id).ok_or(TaskError::NotFound)?;
        process::process_table().with_mut(slot, |p| {
            if p.can_create_port() {
                p.add_port();
                Ok(())
            } else {
                Err(TaskError::LimitReached)
            }
        }).unwrap_or(Err(TaskError::NotFound))
    }

    fn remove_port(&self, task_id: TaskId) {
        if let Some(slot) = slot_for(task_id) {
            process::process_table().with_mut(slot, |p| p.remove_port());
        }
    }

    fn can_create_shmem(&self, task_id: TaskId) -> bool {
        slot_for(task_id)
            .and_then(|s| process::process_table().with(s, |p| p.can_create_shmem()))
            .unwrap_or(false)
    }

    fn add_shmem(&self, task_id: TaskId) -> Result<(), TaskError> {
        let slot = slot_for(task_id).ok_or(TaskError::NotFound)?;
        process::process_table().with_mut(slot, |p| {
            if p.can_create_shmem() {
                p.add_shmem();
                Ok(())
            } else {
                Err(TaskError::LimitReached)
            }
        }).unwrap_or(Err(TaskError::NotFound))
    }

    fn remove_shmem(&self, task_id: TaskId) {
        if let Some(slot) = slot_for(task_id) {
            process::process_table().with_mut(slot, |p| p.remove_shmem());
        }
    }

    fn get_resource_counts(&self, task_id: TaskId) -> Option<ResourceCounts> {
        slot_for(task_id).and_then(|s| {
            process::process_table().with(s, |p| ResourceCounts {
                channels: p.channel_count,
                ports: p.port_count,
                shmem: p.shmem_count,
            })
        })
    }

    // ========================================================================
    // Capabilities
    // ========================================================================

    fn get_capabilities(&self, task_id: TaskId) -> Option<TraitCapabilities> {
        slot_for(task_id).and_then(|s| {
            process::process_table().with(s, |p| convert_caps_to_trait(p.capabilities))
        })
    }

    fn has_capability(&self, task_id: TaskId, cap: u64) -> bool {
        slot_for(task_id)
            .and_then(|s| process::process_table().with(s, |p| {
                p.capabilities.has(KernelCapabilities::from_bits(cap))
            }))
            .unwrap_or(false)
    }

    fn set_capabilities(&self, task_id: TaskId, caps: TraitCapabilities) -> Result<(), TaskError> {
        let slot = slot_for(task_id).ok_or(TaskError::NotFound)?;
        process::process_table().with_mut(slot, |p| {
            p.set_capabilities(convert_caps_from_trait(caps));
        }).ok_or(TaskError::NotFound)
    }

    // ========================================================================
    // Signal Permissions
    // ========================================================================

    fn can_receive_signal_from(&self, task_id: TaskId, sender: TaskId) -> bool {
        slot_for(task_id)
            .and_then(|s| process::process_table().with(s, |p| p.can_receive_signal_from(sender)))
            .unwrap_or(false)
    }

    fn allow_signals_from(&self, task_id: TaskId, sender: TaskId) -> Result<(), TaskError> {
        let slot = slot_for(task_id).ok_or(TaskError::NotFound)?;
        process::process_table().with_mut(slot, |p| {
            p.allow_signals_from(sender);
        }).ok_or(TaskError::NotFound)
    }

    // ========================================================================
    // Task Info
    // ========================================================================

    fn get_name(&self, task_id: TaskId) -> Option<[u8; 32]> {
        slot_for(task_id).and_then(|s| {
            process::process_table().with(s, |p| {
                let mut name = [0u8; 32];
                name[..16].copy_from_slice(&p.name);
                name
            })
        })
    }

    fn get_parent(&self, task_id: TaskId) -> Option<TaskId> {
        slot_for(task_id).and_then(|s| {
            process::process_table().with(s, |p| p.parent_id)
        })
    }

    fn exists(&self, task_id: TaskId) -> bool {
        slot_for(task_id).is_some()
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
        pub fn new() -> Self {
            const NONE: Option<MockTaskInfo> = None;
            Self {
                tasks: RefCell::new([NONE; MAX_MOCK_TASKS]),
                max_channels: 32,
                max_ports: 4,
                max_shmem: 16,
            }
        }

        pub fn with_limits(max_channels: u16, max_ports: u16, max_shmem: u16) -> Self {
            const NONE: Option<MockTaskInfo> = None;
            Self {
                tasks: RefCell::new([NONE; MAX_MOCK_TASKS]),
                max_channels,
                max_ports,
                max_shmem,
            }
        }

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
                        if t.signal_allowlist_count == 0 {
                            return true;
                        }
                        if sender == t.parent_id && t.parent_id != 0 {
                            return true;
                        }
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
                        for i in 0..t.signal_allowlist_count {
                            if t.signal_allowlist[i] == sender {
                                return Ok(());
                            }
                        }
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

        assert!(mock.can_create_channel(1));
        assert!(mock.add_channel(1).is_ok());
        assert!(mock.can_create_channel(1));
        assert!(mock.add_channel(1).is_ok());
        assert!(!mock.can_create_channel(1));
        assert!(matches!(mock.add_channel(1), Err(TaskError::LimitReached)));

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

        let caps = mock.get_capabilities(1).unwrap();
        assert_eq!(caps.bits(), 0);

        let new_caps = TraitCapabilities::from_bits(
            TraitCapabilities::PCI_ACCESS | TraitCapabilities::DMA_ALLOC
        );
        mock.set_capabilities(1, new_caps).unwrap();

        assert!(mock.has_capability(1, TraitCapabilities::PCI_ACCESS));
        assert!(mock.has_capability(1, TraitCapabilities::DMA_ALLOC));
        assert!(!mock.has_capability(1, TraitCapabilities::KILL));
    }

    #[test]
    fn test_signal_permissions_empty_allowlist() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "receiver");
        mock.add_task(2, 0, "sender");

        assert!(mock.can_receive_signal_from(1, 2));
        assert!(mock.can_receive_signal_from(1, 999));
    }

    #[test]
    fn test_signal_permissions_with_allowlist() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "receiver");
        mock.add_task(2, 0, "allowed");
        mock.add_task(3, 0, "blocked");

        mock.allow_signals_from(1, 2).unwrap();

        assert!(mock.can_receive_signal_from(1, 2));
        assert!(!mock.can_receive_signal_from(1, 3));
    }

    #[test]
    fn test_signal_permissions_parent_always_allowed() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "parent");
        mock.add_task(2, 1, "child");

        mock.allow_signals_from(2, 99).unwrap();

        assert!(mock.can_receive_signal_from(2, 1));
    }

    #[test]
    fn test_signal_allowlist_full() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "receiver");

        for i in 10..18 {
            assert!(mock.allow_signals_from(1, i).is_ok());
        }

        assert!(matches!(mock.allow_signals_from(1, 100), Err(TaskError::AllowlistFull)));
    }

    #[test]
    fn test_task_name() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test_task");

        let name = mock.get_name(1).unwrap();
        assert_eq!(&name[..9], b"test_task");
        assert_eq!(name[9], 0);
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

        assert!(!mock.can_create_channel(999));
        assert!(matches!(mock.add_channel(999), Err(TaskError::NotFound)));
        assert!(mock.get_resource_counts(999).is_none());
        assert!(mock.get_capabilities(999).is_none());
        assert!(!mock.has_capability(999, TraitCapabilities::PCI_ACCESS));
        assert!(!mock.can_receive_signal_from(999, 1));
        assert!(matches!(mock.allow_signals_from(999, 1), Err(TaskError::NotFound)));
        assert!(mock.get_name(999).is_none());
        assert!(mock.get_parent(999).is_none());
        assert!(!mock.exists(999));
    }

    #[test]
    fn test_resource_removal_underflow() {
        let mock = MockTaskOperations::new();
        mock.add_task(1, 0, "test");

        mock.remove_channel(1);
        mock.remove_port(1);
        mock.remove_shmem(1);

        let counts = mock.get_resource_counts(1).unwrap();
        assert_eq!(counts.channels, 0);
        assert_eq!(counts.ports, 0);
        assert_eq!(counts.shmem, 0);
    }
}
