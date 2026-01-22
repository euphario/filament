//! Hardware Polling Abstraction
//!
//! Provides a unified interface for polling hardware registers with timeouts.
//! This replaces ad-hoc polling loops scattered throughout the kernel.
//!
//! ## Design Principles
//!
//! - **Time-bound**: All polls have explicit timeouts, no infinite waits
//! - **Composable**: Trait-based for dependency injection and testing
//! - **Observable**: Logging on timeout for debugging hardware issues
//!
//! ## Usage
//!
//! ```rust
//! use kernel::hw_poll::{poll_until, poll_timeout_ms};
//!
//! // Poll with iteration count
//! if !poll_until(100, 1, || (reg.read32(STATUS) & READY) != 0) {
//!     // Timeout - handle error
//! }
//!
//! // Poll with timeout in milliseconds
//! if !poll_timeout_ms(200, || (reg.read32(STATUS) & READY) != 0) {
//!     // Timeout - handle error
//! }
//! ```

use crate::arch::aarch64::mmio::delay_ms;

/// Poll a condition with a maximum iteration count.
///
/// Returns `true` if the condition became true within the limit.
/// Returns `false` if max_polls was reached (timeout).
///
/// # Arguments
/// * `max_polls` - Maximum number of poll iterations
/// * `delay_between_ms` - Delay between polls in milliseconds
/// * `check` - Function that returns true when the condition is met
///
/// # Example
/// ```rust
/// let ready = poll_until(100, 1, || (reg.read32(STATUS) & READY_BIT) != 0);
/// if !ready {
///     kerror!("hw", "poll_timeout"; reg = "STATUS");
/// }
/// ```
#[inline]
pub fn poll_until<F>(max_polls: usize, delay_between_ms: u64, check: F) -> bool
where
    F: Fn() -> bool,
{
    for _ in 0..max_polls {
        if check() {
            return true;
        }
        if delay_between_ms > 0 {
            delay_ms(delay_between_ms);
        }
    }
    false
}

/// Poll a condition with a timeout in milliseconds.
///
/// Returns `true` if the condition became true within the timeout.
/// Returns `false` if the timeout was reached.
///
/// # Arguments
/// * `timeout_ms` - Maximum time to wait in milliseconds
/// * `check` - Function that returns true when the condition is met
///
/// # Note
/// Polls approximately every 1ms for timeouts >= 10ms,
/// more frequently for shorter timeouts.
///
/// # Example
/// ```rust
/// let ready = poll_timeout_ms(200, || (reg.read32(STATUS) & CNR_BIT) == 0);
/// if !ready {
///     kerror!("usb", "cnr_timeout"; usbsts = reg.read32(USBSTS));
/// }
/// ```
#[inline]
pub fn poll_timeout_ms<F>(timeout_ms: u64, check: F) -> bool
where
    F: Fn() -> bool,
{
    // For short timeouts, poll more frequently
    let delay = if timeout_ms < 10 { 0 } else { 1 };
    let max_polls = if delay == 0 {
        timeout_ms * 10  // Rough estimate: ~100us per iteration without delay
    } else {
        timeout_ms
    };

    poll_until(max_polls as usize, delay, check)
}

/// Hardware polling trait for dependency injection and testing.
///
/// Implement this trait for hardware abstractions that need polling.
/// This allows unit tests to mock the polling behavior.
pub trait HardwarePoller {
    /// Poll until a condition is met or max iterations reached.
    ///
    /// Returns `true` if condition was met, `false` on timeout.
    fn poll_until<F>(&self, max_polls: usize, delay_ms: u64, check: F) -> bool
    where
        F: Fn() -> bool;

    /// Poll until a condition is met or timeout reached.
    ///
    /// Returns `true` if condition was met, `false` on timeout.
    fn poll_timeout_ms<F>(&self, timeout_ms: u64, check: F) -> bool
    where
        F: Fn() -> bool;
}

/// Default implementation using hardware delays.
///
/// This is the production implementation that performs actual hardware polling.
pub struct DefaultHardwarePoller;

impl HardwarePoller for DefaultHardwarePoller {
    #[inline]
    fn poll_until<F>(&self, max_polls: usize, delay_between_ms: u64, check: F) -> bool
    where
        F: Fn() -> bool,
    {
        poll_until(max_polls, delay_between_ms, check)
    }

    #[inline]
    fn poll_timeout_ms<F>(&self, timeout_ms: u64, check: F) -> bool
    where
        F: Fn() -> bool,
    {
        poll_timeout_ms(timeout_ms, check)
    }
}

/// Test implementation that allows controlling poll results.
#[cfg(test)]
pub struct MockHardwarePoller {
    /// If true, always return success immediately
    pub always_succeed: bool,
    /// Number of iterations before returning success (if always_succeed is false)
    pub succeed_after: usize,
}

#[cfg(test)]
impl HardwarePoller for MockHardwarePoller {
    fn poll_until<F>(&self, _max_polls: usize, _delay_ms: u64, _check: F) -> bool
    where
        F: Fn() -> bool,
    {
        self.always_succeed
    }

    fn poll_timeout_ms<F>(&self, _timeout_ms: u64, _check: F) -> bool
    where
        F: Fn() -> bool,
    {
        self.always_succeed
    }
}

/// Unit tests
pub fn test() {
    use crate::print_direct;
    print_direct!("  Testing hw_poll...\n");

    // Test poll_until with immediate success
    let result = poll_until(10, 0, || true);
    assert!(result);

    // Test poll_until with immediate failure (returns false after max polls)
    let counter = core::cell::Cell::new(0);
    let result = poll_until(5, 0, || {
        counter.set(counter.get() + 1);
        false
    });
    assert!(!result);
    assert_eq!(counter.get(), 5);

    // Test poll_until with success on iteration 3
    let counter = core::cell::Cell::new(0);
    let result = poll_until(10, 0, || {
        counter.set(counter.get() + 1);
        counter.get() >= 3
    });
    assert!(result);
    assert_eq!(counter.get(), 3);

    print_direct!("    [OK] hw_poll test passed\n");
}
