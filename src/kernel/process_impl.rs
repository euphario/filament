//! Process Backend Implementation
//!
//! Implements the `ProcessBackend` trait by delegating to the existing
//! process module. This provides a clean trait boundary without
//! changing the internal implementation.
//!
//! # Thread Safety
//!
//! All operations access the global process table with appropriate
//! synchronization (currently via unsafe access - proper locking needed
//! for SMP).

use crate::kernel::traits::process::{
    ProcessBackend, ProcessStateInfo, ProcessInfo, Pid,
};
use crate::kernel::process::{self, ProcessState};

// ============================================================================
// Type Conversions
// ============================================================================

/// Convert internal ProcessState to trait ProcessStateInfo
fn convert_state(state: ProcessState, exit_code: i32) -> ProcessStateInfo {
    match state {
        ProcessState::Creating => ProcessStateInfo::Creating,
        ProcessState::Ready => ProcessStateInfo::Ready,
        ProcessState::Running => ProcessStateInfo::Running,
        ProcessState::Blocked => ProcessStateInfo::Blocked,
        ProcessState::Zombie => ProcessStateInfo::Zombie { exit_code },
        ProcessState::Free => ProcessStateInfo::Free,
    }
}

/// Convert internal Process to ProcessInfo
fn convert_process(proc: &process::Process) -> ProcessInfo {
    ProcessInfo {
        pid: proc.pid,
        parent_pid: proc.parent_pid,
        state: convert_state(proc.state(), proc.exit_code),
        name: proc.name,
    }
}

// ============================================================================
// Kernel Process Backend Implementation
// ============================================================================

/// Kernel process backend implementation
///
/// A zero-sized type that implements `ProcessBackend` by delegating to the
/// global process table.
pub struct KernelProcessBackend;

impl KernelProcessBackend {
    /// Create a new kernel process backend instance
    pub const fn new() -> Self {
        Self
    }
}

impl ProcessBackend for KernelProcessBackend {
    fn current_pid(&self) -> Option<Pid> {
        unsafe {
            let table = process::process_table();
            table.current().map(|p| p.pid)
        }
    }

    fn current_info(&self) -> Option<ProcessInfo> {
        unsafe {
            let table = process::process_table();
            table.current().map(convert_process)
        }
    }

    fn get_info(&self, pid: Pid) -> Option<ProcessInfo> {
        unsafe {
            let table = process::process_table();
            table.get(pid).map(convert_process)
        }
    }

    fn get_state(&self, pid: Pid) -> Option<ProcessStateInfo> {
        unsafe {
            let table = process::process_table();
            table.get(pid).map(|p| convert_state(p.state(), p.exit_code))
        }
    }

    fn create(&self, parent: Pid, name: &str) -> Option<Pid> {
        unsafe {
            let table = process::process_table();
            table.create(parent, name)
        }
    }

    fn terminate(&self, pid: Pid, exit_code: i32) {
        unsafe {
            let table = process::process_table();
            table.terminate(pid, exit_code);
        }
    }

    fn reap(&self, pid: Pid) -> Option<i32> {
        unsafe {
            let table = process::process_table();
            table.reap(pid)
        }
    }

    fn parent_of(&self, pid: Pid) -> Option<Pid> {
        unsafe {
            let table = process::process_table();
            table.get(pid).map(|p| p.parent_pid)
        }
    }

    fn child_count(&self, pid: Pid) -> usize {
        unsafe {
            let table = process::process_table();
            let mut count = 0;
            for i in 0..64 { // MAX_PROCESSES
                if let Some(proc) = table.get(i as Pid) {
                    if proc.parent_pid == pid {
                        count += 1;
                    }
                }
            }
            count
        }
    }

    fn has_zombie_children(&self, parent_pid: Pid) -> bool {
        unsafe {
            let table = process::process_table();
            // Scan for any zombie children
            for i in 0..64 { // MAX_PROCESSES
                if let Some(proc) = table.get(i as Pid) {
                    if proc.parent_pid == parent_pid && proc.state() == ProcessState::Zombie {
                        return true;
                    }
                }
            }
            false
        }
    }

    fn wake(&self, pid: Pid) -> bool {
        unsafe {
            let table = process::process_table();
            table.wake(pid)
        }
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel process backend
pub static PROCESS_BACKEND: KernelProcessBackend = KernelProcessBackend::new();

/// Get a reference to the global process backend
pub fn process_backend() -> &'static dyn ProcessBackend {
    &PROCESS_BACKEND
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

#[cfg(test)]
pub mod mock {
    use super::*;
    use core::cell::RefCell;

    /// Mock process for testing
    #[derive(Clone)]
    pub struct MockProcess {
        pub pid: Pid,
        pub parent_pid: Pid,
        pub state: ProcessStateInfo,
        pub name: [u8; 32],
    }

    impl MockProcess {
        pub fn new(pid: Pid, parent_pid: Pid, name: &str) -> Self {
            let mut name_buf = [0u8; 32];
            let bytes = name.as_bytes();
            let len = bytes.len().min(31);
            name_buf[..len].copy_from_slice(&bytes[..len]);

            Self {
                pid,
                parent_pid,
                state: ProcessStateInfo::Creating,
                name: name_buf,
            }
        }
    }

    /// Mock process backend for testing
    pub struct MockProcessBackend {
        processes: RefCell<Vec<MockProcess>>,
        current_pid: RefCell<Option<Pid>>,
        next_pid: RefCell<Pid>,
        /// Track created processes for verification
        create_history: RefCell<Vec<(Pid, Pid, String)>>, // (new_pid, parent, name)
        /// Track terminated processes for verification
        terminate_history: RefCell<Vec<(Pid, i32)>>,
    }

    impl MockProcessBackend {
        pub fn new() -> Self {
            Self {
                processes: RefCell::new(Vec::new()),
                current_pid: RefCell::new(None),
                next_pid: RefCell::new(1),
                create_history: RefCell::new(Vec::new()),
                terminate_history: RefCell::new(Vec::new()),
            }
        }

        /// Add a pre-existing process (for test setup)
        pub fn add_process(&self, proc: MockProcess) {
            self.processes.borrow_mut().push(proc);
        }

        /// Set current process
        pub fn set_current(&self, pid: Option<Pid>) {
            *self.current_pid.borrow_mut() = pid;
        }

        /// Get create history
        pub fn create_history(&self) -> Vec<(Pid, Pid, String)> {
            self.create_history.borrow().clone()
        }

        /// Get terminate history
        pub fn terminate_history(&self) -> Vec<(Pid, i32)> {
            self.terminate_history.borrow().clone()
        }

        /// Reset all state
        pub fn reset(&self) {
            self.processes.borrow_mut().clear();
            *self.current_pid.borrow_mut() = None;
            *self.next_pid.borrow_mut() = 1;
            self.create_history.borrow_mut().clear();
            self.terminate_history.borrow_mut().clear();
        }

        /// Set a process's state (for testing state transitions)
        pub fn set_process_state(&self, pid: Pid, state: ProcessStateInfo) {
            let mut procs = self.processes.borrow_mut();
            if let Some(proc) = procs.iter_mut().find(|p| p.pid == pid) {
                proc.state = state;
            }
        }
    }

    impl ProcessBackend for MockProcessBackend {
        fn current_pid(&self) -> Option<Pid> {
            *self.current_pid.borrow()
        }

        fn current_info(&self) -> Option<ProcessInfo> {
            let current = self.current_pid()?;
            self.get_info(current)
        }

        fn get_info(&self, pid: Pid) -> Option<ProcessInfo> {
            let procs = self.processes.borrow();
            procs.iter().find(|p| p.pid == pid).map(|p| ProcessInfo {
                pid: p.pid,
                parent_pid: p.parent_pid,
                state: p.state.clone(),
                name: p.name,
            })
        }

        fn get_state(&self, pid: Pid) -> Option<ProcessStateInfo> {
            let procs = self.processes.borrow();
            procs.iter().find(|p| p.pid == pid).map(|p| p.state.clone())
        }

        fn create(&self, parent: Pid, name: &str) -> Option<Pid> {
            let pid = *self.next_pid.borrow();
            *self.next_pid.borrow_mut() += 1;

            let proc = MockProcess::new(pid, parent, name);
            self.processes.borrow_mut().push(proc);
            self.create_history.borrow_mut().push((pid, parent, name.to_string()));

            Some(pid)
        }

        fn terminate(&self, pid: Pid, exit_code: i32) {
            let mut procs = self.processes.borrow_mut();
            if let Some(proc) = procs.iter_mut().find(|p| p.pid == pid) {
                proc.state = ProcessStateInfo::Zombie { exit_code };
            }
            drop(procs);
            self.terminate_history.borrow_mut().push((pid, exit_code));
        }

        fn reap(&self, pid: Pid) -> Option<i32> {
            let mut procs = self.processes.borrow_mut();
            let idx = procs.iter().position(|p| {
                p.pid == pid && matches!(p.state, ProcessStateInfo::Zombie { .. })
            })?;

            let exit_code = match procs[idx].state {
                ProcessStateInfo::Zombie { exit_code } => exit_code,
                _ => return None,
            };

            procs.remove(idx);
            Some(exit_code)
        }

        fn parent_of(&self, pid: Pid) -> Option<Pid> {
            let procs = self.processes.borrow();
            procs.iter().find(|p| p.pid == pid).map(|p| p.parent_pid)
        }

        fn child_count(&self, pid: Pid) -> usize {
            let procs = self.processes.borrow();
            procs.iter().filter(|p| p.parent_pid == pid).count()
        }

        fn has_zombie_children(&self, parent_pid: Pid) -> bool {
            let procs = self.processes.borrow();
            procs.iter().any(|p| {
                p.parent_pid == parent_pid &&
                matches!(p.state, ProcessStateInfo::Zombie { .. })
            })
        }

        fn wake(&self, pid: Pid) -> bool {
            let mut procs = self.processes.borrow_mut();
            if let Some(proc) = procs.iter_mut().find(|p| p.pid == pid) {
                if proc.state == ProcessStateInfo::Blocked {
                    proc.state = ProcessStateInfo::Ready;
                    return true;
                }
            }
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::mock::*;

    #[test]
    fn test_mock_create_process() {
        let backend = MockProcessBackend::new();
        let pid = backend.create(0, "test_proc");

        assert!(pid.is_some());
        let pid = pid.unwrap();
        assert!(pid > 0);

        let info = backend.get_info(pid);
        assert!(info.is_some());
        let info = info.unwrap();
        assert_eq!(info.pid, pid);
        assert_eq!(info.parent_pid, 0);
        assert_eq!(info.name_str(), "test_proc");
    }

    #[test]
    fn test_mock_terminate_and_reap() {
        let backend = MockProcessBackend::new();
        let pid = backend.create(0, "dying").unwrap();

        // Make it running first
        backend.set_process_state(pid, ProcessStateInfo::Running);

        // Terminate
        backend.terminate(pid, 42);

        let state = backend.get_state(pid);
        assert!(matches!(state, Some(ProcessStateInfo::Zombie { exit_code: 42 })));

        // Reap
        let exit_code = backend.reap(pid);
        assert_eq!(exit_code, Some(42));

        // Should be gone now
        assert!(backend.get_info(pid).is_none());
    }

    #[test]
    fn test_mock_parent_child() {
        let backend = MockProcessBackend::new();

        // Create parent
        let parent_pid = backend.create(0, "parent").unwrap();

        // Create children
        let child1 = backend.create(parent_pid, "child1").unwrap();
        let child2 = backend.create(parent_pid, "child2").unwrap();

        // Verify parent
        assert_eq!(backend.parent_of(child1), Some(parent_pid));
        assert_eq!(backend.parent_of(child2), Some(parent_pid));

        // Verify child count
        assert_eq!(backend.child_count(parent_pid), 2);

        // Suppress warnings for unused variables
        let _ = (child1, child2);
    }

    #[test]
    fn test_mock_zombie_children() {
        let backend = MockProcessBackend::new();

        let parent = backend.create(0, "parent").unwrap();
        let child = backend.create(parent, "child").unwrap();

        // No zombies yet
        assert!(!backend.has_zombie_children(parent));

        // Make child a zombie
        backend.terminate(child, 0);

        // Now has zombie children
        assert!(backend.has_zombie_children(parent));

        // Reap it
        backend.reap(child);

        // No more zombie children
        assert!(!backend.has_zombie_children(parent));
    }

    #[test]
    fn test_mock_wake() {
        let backend = MockProcessBackend::new();
        let pid = backend.create(0, "sleeper").unwrap();

        // Not blocked yet - wake should fail
        assert!(!backend.wake(pid));

        // Make it blocked
        backend.set_process_state(pid, ProcessStateInfo::Blocked);

        // Now wake should succeed
        assert!(backend.wake(pid));
        assert_eq!(backend.get_state(pid), Some(ProcessStateInfo::Ready));

        // Wake again should fail (already ready)
        assert!(!backend.wake(pid));
    }

    #[test]
    fn test_mock_current_process() {
        let backend = MockProcessBackend::new();
        let pid = backend.create(0, "current").unwrap();

        // No current process yet
        assert!(backend.current_pid().is_none());

        // Set current
        backend.set_current(Some(pid));

        assert_eq!(backend.current_pid(), Some(pid));
        assert!(backend.current_info().is_some());
    }

    #[test]
    fn test_mock_create_history() {
        let backend = MockProcessBackend::new();

        backend.create(0, "first").unwrap();
        backend.create(0, "second").unwrap();

        let history = backend.create_history();
        assert_eq!(history.len(), 2);
        assert_eq!(history[0].2, "first");
        assert_eq!(history[1].2, "second");
    }

    #[test]
    fn test_mock_reset() {
        let backend = MockProcessBackend::new();

        backend.create(0, "proc1").unwrap();
        backend.set_current(Some(1));

        backend.reset();

        assert!(backend.current_pid().is_none());
        assert!(backend.get_info(1).is_none());
        assert!(backend.create_history().is_empty());
    }

    // ========================================================================
    // Additional Lifecycle Tests
    // ========================================================================

    #[test]
    fn test_nonexistent_process_operations() {
        let backend = MockProcessBackend::new();

        // Operations on nonexistent PID should fail gracefully
        assert!(backend.get_info(999).is_none());
        assert!(backend.get_state(999).is_none());
        assert!(backend.parent_of(999).is_none());
        assert_eq!(backend.child_count(999), 0);
        assert!(!backend.has_zombie_children(999));
        assert!(!backend.wake(999));

        // Reaping nonexistent should return None
        assert!(backend.reap(999).is_none());
    }

    #[test]
    fn test_terminate_twice() {
        let backend = MockProcessBackend::new();
        let pid = backend.create(0, "proc").unwrap();

        // First terminate
        backend.terminate(pid, 42);
        assert!(matches!(
            backend.get_state(pid),
            Some(ProcessStateInfo::Zombie { exit_code: 42 })
        ));

        // Second terminate should update exit code
        backend.terminate(pid, 99);
        assert!(matches!(
            backend.get_state(pid),
            Some(ProcessStateInfo::Zombie { exit_code: 99 })
        ));
    }

    #[test]
    fn test_reap_non_zombie() {
        let backend = MockProcessBackend::new();
        let pid = backend.create(0, "proc").unwrap();

        // Process is in Creating state, not Zombie
        assert!(backend.reap(pid).is_none());

        // Make it Running
        backend.set_process_state(pid, ProcessStateInfo::Running);
        assert!(backend.reap(pid).is_none());

        // Make it Blocked
        backend.set_process_state(pid, ProcessStateInfo::Blocked);
        assert!(backend.reap(pid).is_none());

        // Make it Zombie - now reap should work
        backend.terminate(pid, 0);
        assert_eq!(backend.reap(pid), Some(0));
    }

    #[test]
    fn test_multiple_zombie_children() {
        let backend = MockProcessBackend::new();
        let parent = backend.create(0, "parent").unwrap();

        // Create multiple children
        let child1 = backend.create(parent, "child1").unwrap();
        let child2 = backend.create(parent, "child2").unwrap();
        let child3 = backend.create(parent, "child3").unwrap();

        assert_eq!(backend.child_count(parent), 3);
        assert!(!backend.has_zombie_children(parent));

        // Terminate first child
        backend.terminate(child1, 1);
        assert!(backend.has_zombie_children(parent));

        // Terminate second child
        backend.terminate(child2, 2);
        assert!(backend.has_zombie_children(parent));

        // Reap first
        assert_eq!(backend.reap(child1), Some(1));
        assert!(backend.has_zombie_children(parent));  // child2 still zombie

        // Reap second
        assert_eq!(backend.reap(child2), Some(2));
        assert!(!backend.has_zombie_children(parent));  // child3 not zombie

        // Third child still alive
        assert_eq!(backend.child_count(parent), 1);

        // Cleanup
        let _ = child3;
    }

    #[test]
    fn test_terminate_history() {
        let backend = MockProcessBackend::new();
        let pid1 = backend.create(0, "proc1").unwrap();
        let pid2 = backend.create(0, "proc2").unwrap();

        backend.terminate(pid1, 42);
        backend.terminate(pid2, -1);

        let history = backend.terminate_history();
        assert_eq!(history.len(), 2);
        assert_eq!(history[0], (pid1, 42));
        assert_eq!(history[1], (pid2, -1));
    }

    #[test]
    fn test_state_transitions() {
        let backend = MockProcessBackend::new();
        let pid = backend.create(0, "proc").unwrap();

        // Creating -> Ready
        backend.set_process_state(pid, ProcessStateInfo::Ready);
        assert_eq!(backend.get_state(pid), Some(ProcessStateInfo::Ready));

        // Ready -> Running
        backend.set_process_state(pid, ProcessStateInfo::Running);
        assert_eq!(backend.get_state(pid), Some(ProcessStateInfo::Running));

        // Running -> Blocked
        backend.set_process_state(pid, ProcessStateInfo::Blocked);
        assert_eq!(backend.get_state(pid), Some(ProcessStateInfo::Blocked));

        // Blocked -> Ready (via wake)
        assert!(backend.wake(pid));
        assert_eq!(backend.get_state(pid), Some(ProcessStateInfo::Ready));
    }

    #[test]
    fn test_exists_method() {
        let backend = MockProcessBackend::new();

        // Nonexistent
        assert!(!backend.exists(999));

        // Create and check exists
        let pid = backend.create(0, "proc").unwrap();
        assert!(backend.exists(pid));

        // Terminate -> still exists (zombie)
        backend.terminate(pid, 0);
        assert!(backend.exists(pid));

        // Reap -> no longer exists
        backend.reap(pid);
        assert!(!backend.exists(pid));
    }

    #[test]
    fn test_deep_process_tree() {
        let backend = MockProcessBackend::new();

        // Create: root -> child -> grandchild -> great-grandchild
        let root = backend.create(0, "root").unwrap();
        let child = backend.create(root, "child").unwrap();
        let grandchild = backend.create(child, "grandchild").unwrap();
        let great = backend.create(grandchild, "great").unwrap();

        // Verify parent chain
        assert_eq!(backend.parent_of(great), Some(grandchild));
        assert_eq!(backend.parent_of(grandchild), Some(child));
        assert_eq!(backend.parent_of(child), Some(root));
        assert_eq!(backend.parent_of(root), Some(0));

        // Verify child counts
        assert_eq!(backend.child_count(root), 1);
        assert_eq!(backend.child_count(child), 1);
        assert_eq!(backend.child_count(grandchild), 1);
        assert_eq!(backend.child_count(great), 0);
    }

    #[test]
    fn test_process_name_truncation() {
        let backend = MockProcessBackend::new();

        // Very long name should be truncated to 31 chars
        let long_name = "this_is_a_very_long_process_name_that_exceeds_limit";
        let pid = backend.create(0, long_name).unwrap();

        let info = backend.get_info(pid).unwrap();
        let name = info.name_str();

        // Should be truncated to 31 chars (32 byte buffer, null terminated)
        assert!(name.len() <= 31);
        assert!(long_name.starts_with(name));
    }
}
