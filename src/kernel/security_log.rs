//! Security Event Logging
//!
//! Provides structured logging for security-relevant events.
//! All security events are logged with a [SECURITY] prefix for easy filtering.

use crate::logln;

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
pub fn log_security_event(event: SecurityEvent, pid: u32, details: &str) {
    logln!("[SECURITY] {:?}: pid={} {}", event, pid, details);
}

/// Log capability denial with specific capability name
pub fn log_capability_denied(pid: u32, cap_name: &str, operation: &str) {
    log_security_event(
        SecurityEvent::CapabilityDenied,
        pid,
        // Use static string for logging
        cap_name,
    );
    logln!("[SECURITY]   operation={} cap={}", operation, cap_name);
}

/// Log IRQ subscription denial
pub fn log_irq_denied(pid: u32, irq: u64) {
    logln!("[SECURITY] IrqSubscribeDenied: pid={} irq={} (lacks IRQ_CLAIM)", pid, irq);
}

/// Log signal blocked by allowlist
pub fn log_signal_blocked(target_pid: u32, sender_pid: u32) {
    logln!("[SECURITY] SignalBlocked: sender={} target={} (not in allowlist)",
           sender_pid, target_pid);
}

/// Log invalid PID access attempt
pub fn log_invalid_pid(caller_pid: u32, target_pid: u32, operation: &str) {
    logln!("[SECURITY] InvalidPid: caller={} target={} op={}",
           caller_pid, target_pid, operation);
}

/// Log resource exhaustion
pub fn log_resource_exhausted(pid: u32, resource: &str) {
    logln!("[SECURITY] ResourceExhausted: pid={} resource={}", pid, resource);
}
