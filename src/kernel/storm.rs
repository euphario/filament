//! Storm Protection
//!
//! Rate limiting for syscalls and wakes to prevent denial-of-service
//! from misbehaving or malicious tasks.
//!
//! # Storm Types
//!
//! | Type | Cause | Detection | Action |
//! |------|-------|-----------|--------|
//! | Syscall storm | Tight loop calling syscalls | >N syscalls/tick | Throttle |
//! | Wake storm | Tasks ping-ponging wakes | >N wakes/tick | Rate limit |
//!
//! # Throttling
//!
//! When a task exceeds thresholds:
//! 1. First offense: Log warning, throttle for N ticks
//! 2. Repeat offense: Increase throttle duration
//! 3. Too many strikes: Evict task
//!
//! # Usage
//!
//! ```ignore
//! // At syscall entry
//! let action = task.storm.record_syscall(current_tick);
//! match action {
//!     StormAction::Allow => { /* proceed */ }
//!     StormAction::Throttle => return Err(EAGAIN),
//!     StormAction::Evict => { /* kill task */ }
//! }
//! ```

/// Configuration for storm protection
#[derive(Debug, Clone, Copy)]
pub struct StormConfig {
    /// Maximum syscalls per tick before throttling
    pub syscall_threshold: u16,
    /// Maximum wakes sent per tick before throttling
    pub wake_threshold: u16,
    /// How many ticks to throttle on first offense
    pub throttle_ticks: u64,
    /// Maximum strikes before eviction
    pub max_strikes: u8,
    /// Master enable switch
    pub enabled: bool,
}

impl StormConfig {
    /// Create default storm config
    pub const fn new() -> Self {
        Self {
            syscall_threshold: 50,    // Per tick
            wake_threshold: 50,        // Per tick
            throttle_ticks: 10,        // ~100ms at 10ms tick
            max_strikes: 3,            // 3 strikes and you're out
            enabled: true,
        }
    }

    /// Disabled storm protection (for testing)
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            ..Self::new()
        }
    }
}

impl Default for StormConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Action to take after storm check
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StormAction {
    /// Allow the operation to proceed
    Allow,
    /// Throttle the task (return EAGAIN)
    Throttle,
    /// Evict the task (too many offenses)
    Evict,
}

/// Per-task storm state
#[derive(Debug, Clone, Copy)]
pub struct StormState {
    /// Syscalls this tick
    syscall_count: u16,
    /// Wakes sent this tick
    wake_count: u16,
    /// Tick when counters were last reset
    last_reset_tick: u64,
    /// Throttled until this tick (0 = not throttled)
    throttle_until: u64,
    /// Strike count (repeated offenses)
    strikes: u8,
}

impl StormState {
    /// Create new storm state
    pub const fn new() -> Self {
        Self {
            syscall_count: 0,
            wake_count: 0,
            last_reset_tick: 0,
            throttle_until: 0,
            strikes: 0,
        }
    }

    /// Reset counters for new tick
    fn maybe_reset(&mut self, current_tick: u64) {
        if current_tick > self.last_reset_tick {
            self.syscall_count = 0;
            self.wake_count = 0;
            self.last_reset_tick = current_tick;
        }
    }

    /// Check if currently throttled
    pub fn is_throttled(&self, current_tick: u64) -> bool {
        self.throttle_until > current_tick
    }

    /// Record a syscall and check for storm
    ///
    /// Call this at syscall entry.
    pub fn record_syscall(&mut self, current_tick: u64, config: &StormConfig) -> StormAction {
        if !config.enabled {
            return StormAction::Allow;
        }

        // Check if already throttled
        if self.is_throttled(current_tick) {
            return StormAction::Throttle;
        }

        self.maybe_reset(current_tick);
        self.syscall_count = self.syscall_count.saturating_add(1);

        if self.syscall_count > config.syscall_threshold {
            self.trigger_throttle(current_tick, config)
        } else {
            StormAction::Allow
        }
    }

    /// Record a wake sent and check for storm
    ///
    /// Call this when task wakes another task.
    pub fn record_wake(&mut self, current_tick: u64, config: &StormConfig) -> StormAction {
        if !config.enabled {
            return StormAction::Allow;
        }

        // Check if already throttled
        if self.is_throttled(current_tick) {
            return StormAction::Throttle;
        }

        self.maybe_reset(current_tick);
        self.wake_count = self.wake_count.saturating_add(1);

        if self.wake_count > config.wake_threshold {
            self.trigger_throttle(current_tick, config)
        } else {
            StormAction::Allow
        }
    }

    /// Trigger throttling
    fn trigger_throttle(&mut self, current_tick: u64, config: &StormConfig) -> StormAction {
        self.strikes = self.strikes.saturating_add(1);

        if self.strikes >= config.max_strikes {
            StormAction::Evict
        } else {
            // Exponential backoff: throttle_ticks * 2^(strikes-1)
            let multiplier = 1u64 << (self.strikes - 1).min(4);
            self.throttle_until = current_tick + config.throttle_ticks * multiplier;
            StormAction::Throttle
        }
    }

    /// Clear strikes (call after task behaves well)
    pub fn clear_strikes(&mut self) {
        self.strikes = 0;
    }

    /// Get current strike count
    pub fn strikes(&self) -> u8 {
        self.strikes
    }

    /// Get syscall count this tick
    pub fn syscall_count(&self) -> u16 {
        self.syscall_count
    }

    /// Get wake count this tick
    pub fn wake_count(&self) -> u16 {
        self.wake_count
    }

    /// Get throttle deadline (0 if not throttled)
    pub fn throttle_until(&self) -> u64 {
        self.throttle_until
    }
}

impl Default for StormState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_allow_normal_usage() {
        let mut storm = StormState::new();
        let config = StormConfig::new();

        // Normal syscalls should be allowed
        for _ in 0..100 {
            assert_eq!(storm.record_syscall(1, &config), StormAction::Allow);
        }
    }

    #[test]
    fn test_throttle_on_storm() {
        let mut storm = StormState::new();
        let config = StormConfig {
            syscall_threshold: 10,
            throttle_ticks: 5,
            max_strikes: 3,
            ..StormConfig::new()
        };

        // First 10 syscalls allowed
        for _ in 0..10 {
            assert_eq!(storm.record_syscall(1, &config), StormAction::Allow);
        }

        // 11th syscall triggers throttle
        assert_eq!(storm.record_syscall(1, &config), StormAction::Throttle);
        assert_eq!(storm.strikes(), 1);

        // Throttled for next 5 ticks
        assert!(storm.is_throttled(2));
        assert!(storm.is_throttled(5));
        assert!(!storm.is_throttled(7));
    }

    #[test]
    fn test_evict_on_repeated_storms() {
        let mut storm = StormState::new();
        let config = StormConfig {
            syscall_threshold: 5,
            throttle_ticks: 1,
            max_strikes: 3,
            ..StormConfig::new()
        };

        // Strike 1
        for _ in 0..6 {
            storm.record_syscall(1, &config);
        }
        assert_eq!(storm.strikes(), 1);

        // Strike 2 (after throttle expires)
        for _ in 0..6 {
            storm.record_syscall(10, &config);
        }
        assert_eq!(storm.strikes(), 2);

        // Strike 3 - should evict
        for _ in 0..5 {
            storm.record_syscall(100, &config);
        }
        let action = storm.record_syscall(100, &config);
        assert_eq!(action, StormAction::Evict);
    }

    #[test]
    fn test_counter_reset_on_new_tick() {
        let mut storm = StormState::new();
        let config = StormConfig {
            syscall_threshold: 10,
            ..StormConfig::new()
        };

        // Fill up counter
        for _ in 0..10 {
            storm.record_syscall(1, &config);
        }

        // New tick resets counter
        assert_eq!(storm.record_syscall(2, &config), StormAction::Allow);
        assert_eq!(storm.syscall_count(), 1);
    }

    #[test]
    fn test_disabled_storm_protection() {
        let mut storm = StormState::new();
        let config = StormConfig::disabled();

        // Even excessive syscalls allowed
        for _ in 0..10000 {
            assert_eq!(storm.record_syscall(1, &config), StormAction::Allow);
        }
    }

    #[test]
    fn test_exponential_backoff() {
        let mut storm = StormState::new();
        let config = StormConfig {
            syscall_threshold: 1,
            throttle_ticks: 10,
            max_strikes: 10,
            ..StormConfig::new()
        };

        // Strike 1: throttle for 10 ticks
        storm.record_syscall(0, &config);
        storm.record_syscall(0, &config);
        assert_eq!(storm.throttle_until(), 10);

        // Strike 2: throttle for 20 ticks (10 * 2^1)
        storm.record_syscall(100, &config);
        storm.record_syscall(100, &config);
        assert_eq!(storm.throttle_until(), 120);

        // Strike 3: throttle for 40 ticks (10 * 2^2)
        storm.record_syscall(200, &config);
        storm.record_syscall(200, &config);
        assert_eq!(storm.throttle_until(), 240);
    }

    #[test]
    fn test_clear_strikes() {
        let mut storm = StormState::new();
        let config = StormConfig {
            syscall_threshold: 1,
            ..StormConfig::new()
        };

        // Accumulate strikes
        storm.record_syscall(1, &config);
        storm.record_syscall(1, &config);
        assert!(storm.strikes() > 0);

        // Clear strikes
        storm.clear_strikes();
        assert_eq!(storm.strikes(), 0);
    }

    #[test]
    fn test_wake_storm() {
        let mut storm = StormState::new();
        let config = StormConfig {
            wake_threshold: 5,
            ..StormConfig::new()
        };

        // Normal wakes allowed
        for _ in 0..5 {
            assert_eq!(storm.record_wake(1, &config), StormAction::Allow);
        }

        // 6th wake triggers throttle
        assert_eq!(storm.record_wake(1, &config), StormAction::Throttle);
    }
}
