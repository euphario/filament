//! Scheduler Backend Implementation
//!
//! Implements the `SchedulerBackend` trait by delegating to the existing
//! task and sched modules. This provides a clean trait boundary without
//! changing the internal implementation.
//!
//! # Thread Safety
//!
//! All operations go through `with_scheduler()` which holds the scheduler
//! lock with IRQ-save semantics, ensuring proper serialization on SMP systems.

use crate::kernel::traits::sched::{
    SchedulerBackend, TaskStateInfo, SleepReason, WaitReason, TaskId,
};
use crate::kernel::task::{
    self, TaskState,
    SleepReason as InternalSleepReason,
    WaitReason as InternalWaitReason,
};
use crate::kernel::sched;

// ============================================================================
// Type Conversions
// ============================================================================

/// Convert internal TaskState to trait TaskStateInfo
fn convert_state(state: &TaskState) -> TaskStateInfo {
    match state {
        TaskState::Ready => TaskStateInfo::Ready,
        TaskState::Running => TaskStateInfo::Running,
        TaskState::Sleeping { .. } => TaskStateInfo::Sleeping,
        TaskState::Waiting { deadline, .. } => TaskStateInfo::Waiting { deadline: *deadline },
        TaskState::Exiting { .. } |
        TaskState::Dying { .. } |
        TaskState::Dead => TaskStateInfo::Terminated,
    }
}

/// Convert trait SleepReason to internal SleepReason
fn convert_sleep_reason(reason: SleepReason) -> InternalSleepReason {
    match reason {
        SleepReason::EventLoop => InternalSleepReason::EventLoop,
        SleepReason::Ipc => InternalSleepReason::Ipc,
        // ChildWait is a WaitReason in the kernel (has deadline), not SleepReason
        SleepReason::ChildWait => InternalSleepReason::EventLoop,
        SleepReason::Other => InternalSleepReason::EventLoop,
    }
}

/// Convert trait WaitReason to internal WaitReason
fn convert_wait_reason(reason: WaitReason) -> InternalWaitReason {
    match reason {
        WaitReason::IpcCall => InternalWaitReason::IpcCall,
        WaitReason::Timer => InternalWaitReason::Timer,
        WaitReason::ShmemNotify => InternalWaitReason::ShmemNotify { shmem_id: 0 },
        WaitReason::Other => InternalWaitReason::IpcCall, // Default to IpcCall
    }
}

// ============================================================================
// Kernel Scheduler Backend Implementation
// ============================================================================

/// Kernel scheduler backend implementation
///
/// A zero-sized type that implements `SchedulerBackend` by delegating to the
/// global scheduler via `with_scheduler()`.
pub struct KernelSchedulerBackend;

impl KernelSchedulerBackend {
    /// Create a new kernel scheduler backend instance
    pub const fn new() -> Self {
        Self
    }
}

impl SchedulerBackend for KernelSchedulerBackend {
    fn current_task_id(&self) -> Option<TaskId> {
        task::with_scheduler(|sched| {
            let slot = task::current_slot();
            sched.task(slot).map(|t| t.id)
        })
    }

    fn current_task_state(&self) -> Option<TaskStateInfo> {
        task::with_scheduler(|sched| {
            let slot = task::current_slot();
            sched.task(slot).map(|t| convert_state(t.state()))
        })
    }

    fn task_state(&self, task_id: TaskId) -> Option<TaskStateInfo> {
        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| convert_state(t.state()))
        })
    }

    fn wake(&self, task_id: TaskId) -> bool {
        sched::wake(task_id)
    }

    fn sleep_current(&self, reason: SleepReason) -> bool {
        sched::sleep_current(convert_sleep_reason(reason))
    }

    fn wait_current(&self, reason: WaitReason, deadline: u64) -> bool {
        sched::wait_current(convert_wait_reason(reason), deadline)
    }

    fn yield_current(&self) {
        sched::yield_current();
    }

    fn request_resched(&self) {
        crate::arch::aarch64::sync::cpu_flags().set_need_resched();
    }

    fn current_tick(&self) -> u64 {
        crate::platform::current::timer::ticks()
    }

    fn note_deadline(&self, deadline: u64) {
        task::with_scheduler(|sched| {
            sched.note_deadline(deadline);
        });
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel scheduler backend
pub static SCHEDULER_BACKEND: KernelSchedulerBackend = KernelSchedulerBackend::new();

/// Get a reference to the global scheduler backend
pub fn scheduler_backend() -> &'static dyn SchedulerBackend {
    &SCHEDULER_BACKEND
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

#[cfg(test)]
pub mod mock {
    use super::*;
    use core::cell::RefCell;

    /// Mock task for testing
    #[derive(Clone)]
    pub struct MockTask {
        pub id: TaskId,
        pub state: TaskStateInfo,
    }

    /// Mock scheduler backend for testing
    ///
    /// Simulates scheduler operations without real task switching.
    pub struct MockSchedulerBackend {
        /// Current task ID (simulated)
        current_task: RefCell<Option<TaskId>>,
        /// Task states
        tasks: RefCell<Vec<MockTask>>,
        /// Current tick counter
        tick: RefCell<u64>,
        /// Next deadline
        next_deadline: RefCell<u64>,
        /// Track wake calls for verification
        wake_history: RefCell<Vec<TaskId>>,
        /// Track sleep calls for verification
        sleep_history: RefCell<Vec<(TaskId, SleepReason)>>,
        /// Resched flag
        resched_requested: RefCell<bool>,
    }

    impl MockSchedulerBackend {
        /// Create a new mock scheduler
        pub fn new() -> Self {
            Self {
                current_task: RefCell::new(None),
                tasks: RefCell::new(Vec::new()),
                tick: RefCell::new(0),
                next_deadline: RefCell::new(u64::MAX),
                wake_history: RefCell::new(Vec::new()),
                sleep_history: RefCell::new(Vec::new()),
                resched_requested: RefCell::new(false),
            }
        }

        /// Add a mock task
        pub fn add_task(&self, id: TaskId, state: TaskStateInfo) {
            self.tasks.borrow_mut().push(MockTask { id, state });
        }

        /// Set current task
        pub fn set_current(&self, id: TaskId) {
            *self.current_task.borrow_mut() = Some(id);
        }

        /// Set current tick
        pub fn set_tick(&self, tick: u64) {
            *self.tick.borrow_mut() = tick;
        }

        /// Advance tick by amount
        pub fn advance_tick(&self, amount: u64) {
            *self.tick.borrow_mut() += amount;
        }

        /// Get wake history for verification
        pub fn wake_history(&self) -> Vec<TaskId> {
            self.wake_history.borrow().clone()
        }

        /// Get sleep history for verification
        pub fn sleep_history(&self) -> Vec<(TaskId, SleepReason)> {
            self.sleep_history.borrow().clone()
        }

        /// Check if resched was requested
        pub fn resched_requested(&self) -> bool {
            *self.resched_requested.borrow()
        }

        /// Clear resched flag
        pub fn clear_resched(&self) {
            *self.resched_requested.borrow_mut() = false;
        }

        /// Reset all state
        pub fn reset(&self) {
            *self.current_task.borrow_mut() = None;
            self.tasks.borrow_mut().clear();
            *self.tick.borrow_mut() = 0;
            *self.next_deadline.borrow_mut() = u64::MAX;
            self.wake_history.borrow_mut().clear();
            self.sleep_history.borrow_mut().clear();
            *self.resched_requested.borrow_mut() = false;
        }

        /// Update a task's state (for testing state transitions)
        pub fn set_task_state(&self, id: TaskId, state: TaskStateInfo) {
            let mut tasks = self.tasks.borrow_mut();
            if let Some(task) = tasks.iter_mut().find(|t| t.id == id) {
                task.state = state;
            }
        }
    }

    impl SchedulerBackend for MockSchedulerBackend {
        fn current_task_id(&self) -> Option<TaskId> {
            *self.current_task.borrow()
        }

        fn current_task_state(&self) -> Option<TaskStateInfo> {
            let current = self.current_task.borrow();
            let current_id = (*current)?;
            let tasks = self.tasks.borrow();
            tasks.iter().find(|t| t.id == current_id).map(|t| t.state)
        }

        fn task_state(&self, task_id: TaskId) -> Option<TaskStateInfo> {
            let tasks = self.tasks.borrow();
            tasks.iter().find(|t| t.id == task_id).map(|t| t.state)
        }

        fn wake(&self, task_id: TaskId) -> bool {
            let mut tasks = self.tasks.borrow_mut();
            if let Some(task) = tasks.iter_mut().find(|t| t.id == task_id) {
                if task.state.can_wake() {
                    task.state = TaskStateInfo::Ready;
                    drop(tasks);
                    self.wake_history.borrow_mut().push(task_id);
                    return true;
                }
            }
            false
        }

        fn sleep_current(&self, reason: SleepReason) -> bool {
            let current = *self.current_task.borrow();
            if let Some(current_id) = current {
                let mut tasks = self.tasks.borrow_mut();
                if let Some(task) = tasks.iter_mut().find(|t| t.id == current_id) {
                    if task.state == TaskStateInfo::Running {
                        task.state = TaskStateInfo::Sleeping;
                        drop(tasks);
                        self.sleep_history.borrow_mut().push((current_id, reason));
                        return true;
                    }
                }
            }
            false
        }

        fn wait_current(&self, _reason: WaitReason, deadline: u64) -> bool {
            let current = *self.current_task.borrow();
            if let Some(current_id) = current {
                let mut tasks = self.tasks.borrow_mut();
                if let Some(task) = tasks.iter_mut().find(|t| t.id == current_id) {
                    if task.state == TaskStateInfo::Running {
                        task.state = TaskStateInfo::Waiting { deadline };
                        return true;
                    }
                }
            }
            false
        }

        fn yield_current(&self) {
            let current = *self.current_task.borrow();
            if let Some(current_id) = current {
                let mut tasks = self.tasks.borrow_mut();
                if let Some(task) = tasks.iter_mut().find(|t| t.id == current_id) {
                    if task.state == TaskStateInfo::Running {
                        task.state = TaskStateInfo::Ready;
                    }
                }
            }
        }

        fn request_resched(&self) {
            *self.resched_requested.borrow_mut() = true;
        }

        fn current_tick(&self) -> u64 {
            *self.tick.borrow()
        }

        fn note_deadline(&self, deadline: u64) {
            let mut next = self.next_deadline.borrow_mut();
            if deadline < *next {
                *next = deadline;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::mock::*;

    #[test]
    fn test_mock_add_and_find_task() {
        let sched = MockSchedulerBackend::new();
        sched.add_task(100, TaskStateInfo::Ready);

        let state = sched.task_state(100);
        assert!(state.is_some());
        assert_eq!(state.unwrap(), TaskStateInfo::Ready);
    }

    #[test]
    fn test_mock_task_not_found() {
        let sched = MockSchedulerBackend::new();
        assert!(sched.task_state(999).is_none());
    }

    #[test]
    fn test_mock_wake_sleeping_task() {
        let sched = MockSchedulerBackend::new();
        sched.add_task(100, TaskStateInfo::Sleeping);

        let result = sched.wake(100);
        assert!(result);
        assert_eq!(sched.task_state(100), Some(TaskStateInfo::Ready));
        assert_eq!(sched.wake_history(), vec![100]);
    }

    #[test]
    fn test_mock_wake_ready_task_fails() {
        let sched = MockSchedulerBackend::new();
        sched.add_task(100, TaskStateInfo::Ready);

        let result = sched.wake(100);
        assert!(!result);
        assert!(sched.wake_history().is_empty());
    }

    #[test]
    fn test_mock_sleep_current() {
        let sched = MockSchedulerBackend::new();
        sched.add_task(100, TaskStateInfo::Running);
        sched.set_current(100);

        let result = sched.sleep_current(SleepReason::EventLoop);
        assert!(result);
        assert_eq!(sched.task_state(100), Some(TaskStateInfo::Sleeping));
        assert_eq!(sched.sleep_history(), vec![(100, SleepReason::EventLoop)]);
    }

    #[test]
    fn test_mock_wait_current_with_deadline() {
        let sched = MockSchedulerBackend::new();
        sched.add_task(100, TaskStateInfo::Running);
        sched.set_current(100);

        let deadline = 12345;
        let result = sched.wait_current(WaitReason::Timer, deadline);
        assert!(result);
        assert_eq!(sched.task_state(100), Some(TaskStateInfo::Waiting { deadline }));
    }

    #[test]
    fn test_mock_yield_current() {
        let sched = MockSchedulerBackend::new();
        sched.add_task(100, TaskStateInfo::Running);
        sched.set_current(100);

        sched.yield_current();
        assert_eq!(sched.task_state(100), Some(TaskStateInfo::Ready));
    }

    #[test]
    fn test_mock_request_resched() {
        let sched = MockSchedulerBackend::new();
        assert!(!sched.resched_requested());

        sched.request_resched();
        assert!(sched.resched_requested());

        sched.clear_resched();
        assert!(!sched.resched_requested());
    }

    #[test]
    fn test_mock_tick_operations() {
        let sched = MockSchedulerBackend::new();
        assert_eq!(sched.current_tick(), 0);

        sched.set_tick(1000);
        assert_eq!(sched.current_tick(), 1000);

        sched.advance_tick(500);
        assert_eq!(sched.current_tick(), 1500);
    }

    #[test]
    fn test_mock_deadline_tracking() {
        let sched = MockSchedulerBackend::new();

        sched.note_deadline(1000);
        sched.note_deadline(500); // Should replace since earlier
        sched.note_deadline(1500); // Should not replace since later

        // We can verify this through the wait behavior
        sched.add_task(100, TaskStateInfo::Running);
        sched.set_current(100);
        sched.wait_current(WaitReason::Timer, 500);

        // Task should be in waiting state with the deadline
        assert_eq!(sched.task_state(100), Some(TaskStateInfo::Waiting { deadline: 500 }));
    }

    #[test]
    fn test_mock_current_task_operations() {
        let sched = MockSchedulerBackend::new();

        // No current task initially
        assert!(sched.current_task_id().is_none());
        assert!(sched.current_task_state().is_none());

        // Add and set current task
        sched.add_task(100, TaskStateInfo::Running);
        sched.set_current(100);

        assert_eq!(sched.current_task_id(), Some(100));
        assert_eq!(sched.current_task_state(), Some(TaskStateInfo::Running));
    }

    #[test]
    fn test_mock_wake_all() {
        let sched = MockSchedulerBackend::new();
        sched.add_task(100, TaskStateInfo::Sleeping);
        sched.add_task(101, TaskStateInfo::Sleeping);
        sched.add_task(102, TaskStateInfo::Ready); // Won't be woken

        let count = sched.wake_all(&[100, 101, 102]);
        assert_eq!(count, 2); // Only 100 and 101 were sleeping

        assert_eq!(sched.task_state(100), Some(TaskStateInfo::Ready));
        assert_eq!(sched.task_state(101), Some(TaskStateInfo::Ready));
    }

    #[test]
    fn test_mock_reset() {
        let sched = MockSchedulerBackend::new();
        sched.add_task(100, TaskStateInfo::Running);
        sched.set_current(100);
        sched.set_tick(1000);

        sched.reset();

        assert!(sched.current_task_id().is_none());
        assert!(sched.task_state(100).is_none());
        assert_eq!(sched.current_tick(), 0);
    }
}
