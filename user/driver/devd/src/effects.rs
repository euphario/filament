//! Effect Executor
//!
//! Executes side effects returned by state machines.
//! This is the ONLY place where syscalls happen for state machine logic.
//!
//! # Design
//!
//! State machines are pure - they return Effects.
//! This module executes those effects.
//! For testing, you can swap in a mock executor.

use crate::state::{Effect, Effects, LogLevel};
use userlib::syscall::{self, Handle, handle_timer_set};

/// Result of spawning a process
#[derive(Debug)]
pub enum SpawnResult {
    Success { pid: u32 },
    Failed,
}

/// Trait for executing effects - mockable for testing
pub trait EffectExecutor {
    /// Schedule a timer to fire at the given absolute time
    fn schedule_deadline(&mut self, deadline_ns: u64);

    /// Cancel any pending timer
    fn cancel_timer(&mut self);

    /// Spawn a process, returns pid on success
    fn spawn_process(&mut self, binary: &str) -> SpawnResult;

    /// Kill a process
    fn kill_process(&mut self, pid: u32);

    /// Log a message
    fn log(&mut self, level: LogLevel, context: &str, message: &str);

    /// Request a bus reset
    fn request_bus_reset(&mut self, bus_type: u8, bus_index: u8, level: u8);
}

/// Real executor that uses syscalls (handle API)
pub struct SyscallExecutor {
    /// Timer handle for scheduling deadlines
    timer_handle: Handle,
    /// Current scheduled deadline (to avoid redundant timer_set calls)
    current_deadline: u64,
}

impl SyscallExecutor {
    pub const fn new() -> Self {
        Self {
            timer_handle: Handle::INVALID,
            current_deadline: 0,
        }
    }

    /// Set the timer handle (must be called before using schedule_deadline)
    pub fn set_timer_handle(&mut self, handle: Handle) {
        self.timer_handle = handle;
    }
}

impl EffectExecutor for SyscallExecutor {
    fn schedule_deadline(&mut self, deadline_ns: u64) {
        // Only set timer if this deadline is earlier than current
        if self.current_deadline == 0 || deadline_ns < self.current_deadline {
            self.current_deadline = deadline_ns;
            let now = syscall::gettime() as u64;
            let delay = deadline_ns.saturating_sub(now);
            // Timer handle must be set before using schedule_deadline
            if self.timer_handle.is_valid() {
                let set_delay = if delay > 0 { delay } else { 1 };
                let _ = handle_timer_set(self.timer_handle, set_delay, 0);
            }
        }
    }

    fn cancel_timer(&mut self) {
        self.current_deadline = 0;
        // Note: With handle API we could potentially disarm the timer, but let it fire
    }

    fn spawn_process(&mut self, binary: &str) -> SpawnResult {
        // Build path - exec_with_caps expects &str
        // We need to construct "/bin/{binary}" as a str
        // Since we can't allocate, we use a fixed buffer and unsafe conversion
        let mut path = [0u8; 64];
        let prefix = b"/bin/";
        path[..prefix.len()].copy_from_slice(prefix);
        let binary_bytes = binary.as_bytes();
        let binary_len = binary_bytes.len().min(64 - prefix.len());
        path[prefix.len()..prefix.len() + binary_len].copy_from_slice(&binary_bytes[..binary_len]);
        let end = prefix.len() + binary_len;

        // Determine capabilities based on binary name
        let caps = match binary {
            "pcied" | "usbd" | "xhcid" => syscall::caps::BUS_DRIVER,
            "wifid" | "nvmed" | "partd" => syscall::caps::DEVICE_DRIVER,
            "fatfsd" | "vfsd" => syscall::caps::FS_DRIVER,
            "gpio" | "pwm" => syscall::caps::GPIO_DRIVER,
            "consoled" | "logd" => syscall::caps::SERVICE_DRIVER,
            _ => syscall::caps::USER_DEFAULT,
        };

        // Safe because we just built this from valid UTF-8 ("/bin/" + binary)
        let path_str = unsafe { core::str::from_utf8_unchecked(&path[..end]) };
        let result = syscall::exec_with_caps(path_str, caps);
        if result > 0 {
            SpawnResult::Success { pid: result as u32 }
        } else {
            SpawnResult::Failed
        }
    }

    fn kill_process(&mut self, pid: u32) {
        let _ = syscall::kill(pid);
    }

    fn log(&mut self, level: LogLevel, _context: &str, message: &str) {
        // Only log errors - use klog syscall
        if matches!(level, LogLevel::Error) {
            let mut buf = [0u8; 256];
            let prefix = b"[devd] ";
            let msg_bytes = message.as_bytes();
            let len = (prefix.len() + msg_bytes.len()).min(255);
            buf[..prefix.len()].copy_from_slice(prefix);
            let msg_len = (len - prefix.len()).min(msg_bytes.len());
            buf[prefix.len()..prefix.len() + msg_len].copy_from_slice(&msg_bytes[..msg_len]);
            syscall::klog(syscall::LogLevel::Error, &buf[..prefix.len() + msg_len]);
        }
    }

    fn request_bus_reset(&mut self, _bus_type: u8, _bus_index: u8, _level: u8) {
        // TODO: Send message to bus driver
    }
}

/// Execute a set of effects
pub fn execute_effects<E: EffectExecutor>(
    executor: &mut E,
    effects: Effects,
    context: &str,
) -> Option<SpawnResult> {
    let mut spawn_result = None;

    for effect in effects.iter() {
        match effect {
            Effect::ScheduleDeadline(deadline) => {
                executor.schedule_deadline(*deadline);
            }
            Effect::CancelTimer => {
                executor.cancel_timer();
            }
            Effect::SpawnProcess { binary } => {
                spawn_result = Some(executor.spawn_process(binary));
            }
            Effect::KillProcess { pid } => {
                executor.kill_process(*pid);
            }
            Effect::Log { level, message } => {
                executor.log(*level, context, message);
            }
            Effect::RequestBusReset { bus_type, bus_index, level } => {
                executor.request_bus_reset(*bus_type, *bus_index, *level);
            }
        }
    }

    spawn_result
}

// Mock executor would go here for testing with std
// For no_std testing, we'd use a simpler fixed-buffer approach
