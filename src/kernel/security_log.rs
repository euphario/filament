//! Security Event Logging
//!
//! Provides structured logging for security-relevant events.
//! All security events are logged with a [SECURITY] prefix for easy filtering.

use crate::kwarn;

/// Security event types for categorization
#[derive(Debug, Clone, Copy)]
pub enum SecurityEvent {
    /// Capability check failed
    CapabilityDenied,
    /// IRQ subscription denied (no IRQ_CLAIM)
    IrqSubscribeDenied,
    /// Signal blocked by allowlist
    SignalBlocked,
    /// Invalid or stale PID used
    InvalidPid,
    /// Resource exhaustion (subscription slots, etc.)
    ResourceExhausted,
    /// Permission denied for operation
    PermissionDenied,
}

/// Log a security event with context
///
/// All security events are logged with [SECURITY] prefix for easy grep/filtering.
pub fn log_security_event(event: SecurityEvent, pid: u32, _details: &str) {
    kwarn!("security", "event"; event_type = event as u64, pid = pid as u64);
}

/// Log capability denial with specific capability name
pub fn log_capability_denied(pid: u32, cap_name: &str, operation: &str) {
    kwarn!("security", "cap_denied"; pid = pid as u64, cap = cap_name, op = operation);
}

/// Log IRQ subscription denial
pub fn log_irq_denied(pid: u32, irq: u64) {
    kwarn!("security", "irq_denied"; pid = pid as u64, irq = irq);
}

/// Log signal blocked by allowlist
pub fn log_signal_blocked(target_pid: u32, sender_pid: u32) {
    kwarn!("security", "signal_blocked"; sender = sender_pid as u64, target = target_pid as u64);
}

/// Log invalid PID access attempt
pub fn log_invalid_pid(caller_pid: u32, target_pid: u32, operation: &str) {
    kwarn!("security", "invalid_pid"; caller = caller_pid as u64, target = target_pid as u64, op = operation);
}

/// Log resource exhaustion
pub fn log_resource_exhausted(pid: u32, resource: &str) {
    kwarn!("security", "resource_exhausted"; pid = pid as u64, resource = resource);
}
