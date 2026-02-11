//! Process Operations Backend Implementation
//!
//! Implements the `ProcessOps` trait for process lifecycle operations.
//! This is a minimal interface: exit, kill, spawn.

use crate::kernel::traits::process_ops::{ProcessOps, ProcessError, SpawnSource, WaitChildResult};
use crate::kernel::traits::task::TaskId;
use crate::kernel::task::{self, lifecycle};
use crate::kernel::caps::Capabilities;
use crate::kernel::elf;
use crate::{kdebug, kwarn};

// ============================================================================
// Kernel Process Operations Implementation
// ============================================================================

/// Kernel process operations backend implementation
pub struct KernelProcessOps;

impl KernelProcessOps {
    /// Create a new kernel process operations instance
    pub const fn new() -> Self {
        Self
    }
}

impl ProcessOps for KernelProcessOps {
    fn exit(&self, task_id: TaskId, code: i32) -> ! {
        use crate::kernel::microtask::{self, MicroTask};
        use crate::kernel::task::tcb::CleanupPhase;

        // Flush UART buffer so pending output from this process appears first
        while crate::platform::current::uart::has_buffered_output() {
            crate::platform::current::uart::flush_buffer();
        }

        // Phase 1: Transition state under scheduler lock, enqueue microtasks
        task::with_scheduler(|sched| {
            if let Err(e) = lifecycle::exit(sched, task_id, code) {
                kdebug!("process_ops", "exit_lifecycle_error"; pid = task_id as u64, err = e as i64);
            }

            // Set cleanup phase on the task
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(task) = sched.task_mut(slot) {
                    task.cleanup_phase = CleanupPhase::Phase1Enqueued;
                }
            }

            // Enqueue Phase 1 microtasks (lock ordering: SCHEDULER(10) → MICROTASK(15) is valid)
            // NotifyParentExit is deferred to Phase 2 — parent learns about exit
            // only after IPC cleanup, bus release, and grace period are complete.
            let _ = microtask::enqueue(MicroTask::IpcCleanup { pid: task_id });
            let _ = microtask::enqueue(MicroTask::KillChildren { pid: task_id });
        });

        // Drain microtasks immediately (outside scheduler lock)
        // This gives us the same low-latency peer notification as the old
        // do_exit_ipc_cleanup() path, but through the unified queue.
        // Higher budget at exit — task is terminating, cleanup is priority.
        microtask::drain(64);

        // If probed just exited, spawn devd now (scheduler lock is released)
        lifecycle::complete_probed_exit();

        // Enable IRQs and enter WFI. The next timer IRQ will call
        // do_resched_if_needed() which detects the Exiting state and
        // reschedules. The grace period scanner enqueues Phase 2 later.
        unsafe {
            core::arch::asm!("msr daifclr, #2"); // enable IRQs
        }
        loop {
            unsafe {
                core::arch::asm!("wfi");
            }
        }
    }

    fn kill(&self, killer: TaskId, target: TaskId) -> Result<(), ProcessError> {
        use crate::kernel::microtask::{self, MicroTask};
        use crate::kernel::task::tcb::CleanupPhase;

        // Never allow killing init (PID 1)
        if target == 1 {
            kwarn!("security", "kill_init_denied"; caller = killer as u64);
            return Err(ProcessError::PermissionDenied);
        }

        // Phase 1: Transition state under scheduler lock, enqueue microtasks
        let (result, need_resched) = task::with_scheduler(|sched| {
            match lifecycle::kill(sched, target, killer) {
                Ok(_info) => {
                    // Set cleanup phase
                    if let Some(slot) = sched.slot_by_pid(target) {
                        if let Some(task) = sched.task_mut(slot) {
                            task.cleanup_phase = CleanupPhase::Phase1Enqueued;
                        }
                    }

                    // Enqueue Phase 1 microtasks
                    // NotifyParentExit deferred to Phase 2 (after cleanup + grace period)
                    let _ = microtask::enqueue(MicroTask::IpcCleanup { pid: target });
                    let _ = microtask::enqueue(MicroTask::KillChildren { pid: target });

                    (Ok(()), target == killer)
                }
                Err(lifecycle::LifecycleError::NotFound) => (Err(ProcessError::NotFound), false),
                Err(lifecycle::LifecycleError::PermissionDenied) => {
                    kwarn!("security", "kill_denied"; caller = killer as u64, target = target as u64);
                    (Err(ProcessError::PermissionDenied), false)
                }
                Err(lifecycle::LifecycleError::InvalidState) => (Err(ProcessError::InvalidState), false),
                Err(lifecycle::LifecycleError::NoChildren) => (Err(ProcessError::InvalidState), false),
            }
        });

        // Drain microtasks immediately (outside scheduler lock)
        // Higher budget at kill — cleanup is priority.
        let drained = microtask::drain(64);
        kdebug!("process_ops", "kill_drain_done"; target = target as u64, drained = drained as u64);

        if need_resched {
            crate::kernel::sched::reschedule();
        }

        result
    }

    fn daemonize(&self, task_id: TaskId) -> Result<(), ProcessError> {
        task::with_scheduler(|sched| {
            let parent_id = if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(task) = sched.task_mut(slot) {
                    task.detach_from_parent()
                } else {
                    return Err(ProcessError::NotFound);
                }
            } else {
                return Err(ProcessError::NotFound);
            };

            if parent_id != 0 {
                if let Some(parent_slot) = sched.slot_by_pid(parent_id) {
                    if let Some(parent) = sched.task_mut(parent_slot) {
                        parent.remove_child(task_id);
                    }
                    sched.wake_task(parent_slot);
                }
            }
            Ok(())
        })
    }

    fn get_capabilities(&self, task_id: TaskId) -> Result<u64, ProcessError> {
        task::with_scheduler(|sched| {
            if let Some(slot) = sched.slot_by_pid(task_id) {
                if let Some(task) = sched.task(slot) {
                    return Ok(task.get_capabilities_bits() as u64);
                }
            }
            Err(ProcessError::NotFound)
        })
    }

    fn wait_child(&self, caller: TaskId, target_pid: i32, no_hang: bool) -> WaitChildResult {
        loop {
            let result = task::with_scheduler(|sched| {
                lifecycle::wait_child(sched, caller, target_pid, no_hang)
            });

            match result {
                lifecycle::WaitResult::Exited { pid, code } => {
                    return WaitChildResult::Exited { pid, code };
                }
                lifecycle::WaitResult::WouldBlock => {
                    if no_hang {
                        return WaitChildResult::WouldBlock;
                    }
                    // Atomically block and reschedule
                    if !crate::kernel::sched::sleep_and_reschedule(task::SleepReason::EventLoop) {
                        return WaitChildResult::NoChildren;
                    }
                    // Woken up — loop back to retry
                    continue;
                }
                lifecycle::WaitResult::NoChildren => return WaitChildResult::NoChildren,
                lifecycle::WaitResult::NotChild => return WaitChildResult::NotChild,
            }
        }
    }

    fn list_processes(&self, buf: &mut [abi::ProcessInfo], max: usize) -> usize {
        task::with_scheduler(|sched| {
            let mut count = 0usize;

            for (slot, task_opt) in sched.iter_tasks() {
                if crate::kernel::sched::is_idle_slot(slot) {
                    continue;
                }
                if count >= max {
                    break;
                }
                if let Some(task) = task_opt {
                    let current_tick = crate::platform::current::timer::counter();
                    buf[count] = abi::ProcessInfo {
                        pid: task.id,
                        ppid: task.parent_id,
                        state: task.state().state_code(),
                        liveness_status: task.get_liveness_status_code(),
                        cpu: sched.task_cpu(slot).map_or(0xFF, |c| c as u8),
                        _pad: 0,
                        activity_age_ms: task.get_activity_age_ms(current_tick),
                        name: task.name,
                    };
                    count += 1;
                }
            }

            count
        })
    }

    fn spawn(&self, parent: TaskId, source: SpawnSource) -> Result<TaskId, ProcessError> {
        match source {
            SpawnSource::ElfId(id, name) => {
                let elf_data = elf::get_elf_by_id(id).ok_or(ProcessError::NotFound)?;
                match elf::spawn_from_elf_with_parent(elf_data, name, parent) {
                    Ok((child_id, _)) => Ok(child_id),
                    Err(_) => Err(ProcessError::OutOfMemory),
                }
            }
            SpawnSource::Path(path) => {
                match elf::spawn_from_path_with_parent(path, parent) {
                    Ok((child_id, _)) => Ok(child_id),
                    Err(elf::ElfError::NotExecutable) => Err(ProcessError::NotFound),
                    Err(_) => Err(ProcessError::OutOfMemory),
                }
            }
            SpawnSource::PathWithCaps(path, caps) => {
                let kernel_caps = Capabilities::from_bits(caps.bits());
                match elf::spawn_from_path_with_caps_find(path, parent, kernel_caps) {
                    Ok((child_id, _)) => Ok(child_id),
                    Err(elf::ElfError::NotExecutable) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::BadMagic) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::WrongArch) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::Not64Bit) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::NotLittleEndian) => Err(ProcessError::InvalidElf),
                    Err(_) => Err(ProcessError::NoSlots),
                }
            }
            SpawnSource::PathWithCapsAndChannel(path, caps, channel_handle) => {
                let kernel_caps = Capabilities::from_bits(caps.bits());
                match elf::spawn_from_path_with_caps_and_channel(path, parent, kernel_caps, channel_handle) {
                    Ok((child_id, _)) => Ok(child_id),
                    Err(elf::ElfError::NotExecutable) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::BadMagic) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::WrongArch) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::Not64Bit) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::NotLittleEndian) => Err(ProcessError::InvalidElf),
                    Err(_) => Err(ProcessError::NoSlots),
                }
            }
            SpawnSource::Memory(data, name) => {
                match elf::spawn_from_elf_with_parent(data, name, parent) {
                    Ok((child_id, _)) => Ok(child_id),
                    Err(elf::ElfError::BadMagic) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::NotExecutable) => Err(ProcessError::InvalidElf),
                    Err(elf::ElfError::WrongArch) => Err(ProcessError::InvalidElf),
                    Err(_) => Err(ProcessError::OutOfMemory),
                }
            }
        }
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel process operations backend
pub static PROCESS_OPS_BACKEND: KernelProcessOps = KernelProcessOps::new();

/// Get a reference to the global process operations backend
pub fn process_ops_backend() -> &'static dyn ProcessOps {
    &PROCESS_OPS_BACKEND
}
