//! Time types — mirrors `std::time`.
//!
//! `Duration` represents a span of time. `Instant` represents a point
//! in monotonic time (wraps `libsys::syscall::gettime()`).

use core::fmt;
use core::ops::{Add, AddAssign, Sub, SubAssign};

// ============================================================================
// Duration
// ============================================================================

const NANOS_PER_SEC: u32 = 1_000_000_000;
const NANOS_PER_MILLI: u32 = 1_000_000;
const NANOS_PER_MICRO: u32 = 1_000;
const MILLIS_PER_SEC: u64 = 1_000;
const MICROS_PER_SEC: u64 = 1_000_000;

/// A span of time — mirrors `std::time::Duration`.
///
/// Internally stores seconds and sub-second nanoseconds, matching std exactly.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Duration {
    secs: u64,
    nanos: u32, // always < 1_000_000_000
}

impl Duration {
    /// Zero duration.
    pub const ZERO: Self = Self { secs: 0, nanos: 0 };

    /// One second. Filament convenience constant.
    pub const SECOND: Self = Self { secs: 1, nanos: 0 };

    /// One millisecond. Filament convenience constant.
    pub const MILLISECOND: Self = Self { secs: 0, nanos: NANOS_PER_MILLI };

    /// Maximum representable duration.
    pub const MAX: Self = Self { secs: u64::MAX, nanos: NANOS_PER_SEC - 1 };

    /// Create a duration from seconds and additional nanoseconds.
    ///
    /// # Panics
    /// Panics if nanos >= 1_000_000_000 and the total overflows.
    pub const fn new(secs: u64, nanos: u32) -> Self {
        let extra_secs = (nanos / NANOS_PER_SEC) as u64;
        let nanos = nanos % NANOS_PER_SEC;
        let secs = match secs.checked_add(extra_secs) {
            Some(s) => s,
            None => panic!("Duration::new overflow"),
        };
        Self { secs, nanos }
    }

    /// Create from whole seconds.
    pub const fn from_secs(secs: u64) -> Self {
        Self { secs, nanos: 0 }
    }

    /// Create from whole milliseconds.
    pub const fn from_millis(millis: u64) -> Self {
        Self {
            secs: millis / MILLIS_PER_SEC,
            nanos: ((millis % MILLIS_PER_SEC) as u32) * NANOS_PER_MILLI,
        }
    }

    /// Create from whole microseconds.
    pub const fn from_micros(micros: u64) -> Self {
        Self {
            secs: micros / MICROS_PER_SEC,
            nanos: ((micros % MICROS_PER_SEC) as u32) * NANOS_PER_MICRO,
        }
    }

    /// Create from whole nanoseconds.
    pub const fn from_nanos(nanos: u64) -> Self {
        Self {
            secs: nanos / (NANOS_PER_SEC as u64),
            nanos: (nanos % (NANOS_PER_SEC as u64)) as u32,
        }
    }

    /// Returns the whole seconds component.
    pub const fn as_secs(&self) -> u64 {
        self.secs
    }

    /// Returns the fractional part in milliseconds (0..999).
    pub const fn subsec_millis(&self) -> u32 {
        self.nanos / NANOS_PER_MILLI
    }

    /// Returns the fractional part in microseconds (0..999_999).
    pub const fn subsec_micros(&self) -> u32 {
        self.nanos / NANOS_PER_MICRO
    }

    /// Returns the fractional part in nanoseconds (0..999_999_999).
    pub const fn subsec_nanos(&self) -> u32 {
        self.nanos
    }

    /// Total duration in milliseconds.
    pub const fn as_millis(&self) -> u128 {
        (self.secs as u128) * (MILLIS_PER_SEC as u128)
            + (self.nanos / NANOS_PER_MILLI) as u128
    }

    /// Total duration in microseconds.
    pub const fn as_micros(&self) -> u128 {
        (self.secs as u128) * (MICROS_PER_SEC as u128)
            + (self.nanos / NANOS_PER_MICRO) as u128
    }

    /// Total duration in nanoseconds.
    pub const fn as_nanos(&self) -> u128 {
        (self.secs as u128) * (NANOS_PER_SEC as u128) + self.nanos as u128
    }

    /// Total duration in nanoseconds as u64.
    ///
    /// Filament-specific: avoids u128 since kernel APIs use u64 nanoseconds.
    /// Returns `None` if the value exceeds u64::MAX (~584 years).
    pub const fn as_nanos_u64(&self) -> Option<u64> {
        match self.secs.checked_mul(NANOS_PER_SEC as u64) {
            Some(s) => s.checked_add(self.nanos as u64),
            None => None,
        }
    }

    /// Checked addition. Returns `None` on overflow.
    pub const fn checked_add(self, rhs: Duration) -> Option<Duration> {
        let mut nanos = self.nanos + rhs.nanos;
        let mut carry = 0u64;
        if nanos >= NANOS_PER_SEC {
            nanos -= NANOS_PER_SEC;
            carry = 1;
        }
        match self.secs.checked_add(rhs.secs) {
            Some(s) => match s.checked_add(carry) {
                Some(s) => Some(Duration { secs: s, nanos }),
                None => None,
            },
            None => None,
        }
    }

    /// Checked subtraction. Returns `None` if `rhs > self`.
    pub const fn checked_sub(self, rhs: Duration) -> Option<Duration> {
        let (nanos, borrow) = if self.nanos >= rhs.nanos {
            (self.nanos - rhs.nanos, 0u64)
        } else {
            (self.nanos + NANOS_PER_SEC - rhs.nanos, 1)
        };
        let rhs_total = match rhs.secs.checked_add(borrow) {
            Some(s) => s,
            None => return None,
        };
        match self.secs.checked_sub(rhs_total) {
            Some(s) => Some(Duration { secs: s, nanos }),
            None => None,
        }
    }

    /// Saturating addition. Clamps at `Duration::MAX` on overflow.
    pub const fn saturating_add(self, rhs: Duration) -> Duration {
        match self.checked_add(rhs) {
            Some(d) => d,
            None => Duration::MAX,
        }
    }

    /// Saturating subtraction. Clamps at `Duration::ZERO` on underflow.
    pub const fn saturating_sub(self, rhs: Duration) -> Duration {
        match self.checked_sub(rhs) {
            Some(d) => d,
            None => Duration::ZERO,
        }
    }

    /// Returns true if this duration is zero.
    pub const fn is_zero(&self) -> bool {
        self.secs == 0 && self.nanos == 0
    }
}

impl Add for Duration {
    type Output = Duration;
    fn add(self, rhs: Duration) -> Duration {
        self.checked_add(rhs).expect("Duration addition overflow")
    }
}

impl AddAssign for Duration {
    fn add_assign(&mut self, rhs: Duration) {
        *self = *self + rhs;
    }
}

impl Sub for Duration {
    type Output = Duration;
    fn sub(self, rhs: Duration) -> Duration {
        self.checked_sub(rhs).expect("Duration subtraction underflow")
    }
}

impl SubAssign for Duration {
    fn sub_assign(&mut self, rhs: Duration) {
        *self = *self - rhs;
    }
}

impl PartialOrd for Duration {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Duration {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.secs.cmp(&other.secs).then(self.nanos.cmp(&other.nanos))
    }
}

impl fmt::Debug for Duration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Duration({}.{:09}s)", self.secs, self.nanos)
    }
}

impl fmt::Display for Duration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.secs > 0 {
            write!(f, "{}.{:03}s", self.secs, self.nanos / NANOS_PER_MILLI)
        } else if self.nanos >= NANOS_PER_MILLI as u32 {
            write!(f, "{}ms", self.nanos / NANOS_PER_MILLI)
        } else if self.nanos >= NANOS_PER_MICRO as u32 {
            write!(f, "{}us", self.nanos / NANOS_PER_MICRO)
        } else {
            write!(f, "{}ns", self.nanos)
        }
    }
}

// ============================================================================
// Instant (requires libsys — not available in host test builds)
// ============================================================================

/// A point in monotonic time — mirrors `std::time::Instant`.
///
/// Wraps `libsys::syscall::gettime()` which returns nanoseconds
/// from an unspecified monotonic epoch.
#[cfg(feature = "target")]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Instant {
    ns: u64,
}

#[cfg(feature = "target")]
impl Instant {
    /// Returns the current time.
    pub fn now() -> Self {
        Self { ns: libsys::syscall::gettime() }
    }

    /// Returns the time elapsed since this instant.
    pub fn elapsed(&self) -> Duration {
        Self::now().duration_since(*self)
    }

    /// Returns the duration from `earlier` to `self`.
    ///
    /// # Panics
    /// Panics if `earlier` is later than `self`.
    pub fn duration_since(&self, earlier: Instant) -> Duration {
        self.checked_duration_since(earlier)
            .expect("Instant::duration_since: earlier is later than self")
    }

    /// Returns the duration from `earlier` to `self`, or `None` if
    /// `earlier` is later.
    pub fn checked_duration_since(&self, earlier: Instant) -> Option<Duration> {
        self.ns.checked_sub(earlier.ns).map(Duration::from_nanos)
    }

    /// Returns the duration from `earlier` to `self`, saturating at zero.
    pub fn saturating_duration_since(&self, earlier: Instant) -> Duration {
        self.checked_duration_since(earlier).unwrap_or(Duration::ZERO)
    }

    /// Checked addition of a duration. Returns `None` on overflow.
    pub fn checked_add(&self, duration: Duration) -> Option<Instant> {
        duration.as_nanos_u64()
            .and_then(|d| self.ns.checked_add(d))
            .map(|ns| Instant { ns })
    }

    /// Checked subtraction of a duration. Returns `None` on underflow.
    pub fn checked_sub(&self, duration: Duration) -> Option<Instant> {
        duration.as_nanos_u64()
            .and_then(|d| self.ns.checked_sub(d))
            .map(|ns| Instant { ns })
    }
}

#[cfg(feature = "target")]
impl Add<Duration> for Instant {
    type Output = Instant;
    fn add(self, rhs: Duration) -> Instant {
        self.checked_add(rhs).expect("Instant + Duration overflow")
    }
}

#[cfg(feature = "target")]
impl AddAssign<Duration> for Instant {
    fn add_assign(&mut self, rhs: Duration) {
        *self = *self + rhs;
    }
}

#[cfg(feature = "target")]
impl Sub<Duration> for Instant {
    type Output = Instant;
    fn sub(self, rhs: Duration) -> Instant {
        self.checked_sub(rhs).expect("Instant - Duration underflow")
    }
}

#[cfg(feature = "target")]
impl SubAssign<Duration> for Instant {
    fn sub_assign(&mut self, rhs: Duration) {
        *self = *self - rhs;
    }
}

#[cfg(feature = "target")]
impl Sub<Instant> for Instant {
    type Output = Duration;
    fn sub(self, rhs: Instant) -> Duration {
        self.duration_since(rhs)
    }
}

#[cfg(feature = "target")]
impl fmt::Debug for Instant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Instant({}ns)", self.ns)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duration_from_secs() {
        let d = Duration::from_secs(5);
        assert_eq!(d.as_secs(), 5);
        assert_eq!(d.subsec_nanos(), 0);
    }

    #[test]
    fn duration_from_millis() {
        let d = Duration::from_millis(2500);
        assert_eq!(d.as_secs(), 2);
        assert_eq!(d.subsec_millis(), 500);
        assert_eq!(d.subsec_nanos(), 500_000_000);
    }

    #[test]
    fn duration_from_micros() {
        let d = Duration::from_micros(1_500_000);
        assert_eq!(d.as_secs(), 1);
        assert_eq!(d.subsec_micros(), 500_000);
    }

    #[test]
    fn duration_from_nanos() {
        let d = Duration::from_nanos(3_500_000_000);
        assert_eq!(d.as_secs(), 3);
        assert_eq!(d.subsec_nanos(), 500_000_000);
    }

    #[test]
    fn duration_new_normalizes() {
        let d = Duration::new(1, 1_500_000_000);
        assert_eq!(d.as_secs(), 2);
        assert_eq!(d.subsec_nanos(), 500_000_000);
    }

    #[test]
    fn duration_as_nanos_u64() {
        let d = Duration::from_millis(1500);
        assert_eq!(d.as_nanos_u64(), Some(1_500_000_000));
    }

    #[test]
    fn duration_as_nanos_u64_overflow() {
        let d = Duration::MAX;
        assert_eq!(d.as_nanos_u64(), None);
    }

    #[test]
    fn duration_add() {
        let a = Duration::from_millis(750);
        let b = Duration::from_millis(500);
        let c = a + b;
        assert_eq!(c.as_secs(), 1);
        assert_eq!(c.subsec_millis(), 250);
    }

    #[test]
    fn duration_sub() {
        let a = Duration::from_millis(1500);
        let b = Duration::from_millis(750);
        let c = a - b;
        assert_eq!(c.as_millis(), 750);
    }

    #[test]
    fn duration_sub_with_borrow() {
        let a = Duration::new(2, 100_000_000); // 2.1s
        let b = Duration::new(1, 500_000_000); // 1.5s
        let c = a - b;
        assert_eq!(c.as_secs(), 0);
        assert_eq!(c.subsec_nanos(), 600_000_000);
    }

    #[test]
    fn duration_checked_sub_underflow() {
        let a = Duration::from_millis(100);
        let b = Duration::from_millis(200);
        assert_eq!(a.checked_sub(b), None);
    }

    #[test]
    fn duration_saturating() {
        let a = Duration::from_millis(100);
        let b = Duration::from_millis(200);
        assert_eq!(a.saturating_sub(b), Duration::ZERO);
        assert_eq!(Duration::MAX.saturating_add(Duration::SECOND), Duration::MAX);
    }

    #[test]
    fn duration_ord() {
        let a = Duration::from_millis(500);
        let b = Duration::from_millis(1000);
        assert!(a < b);

        let c = Duration::new(1, 0);
        let d = Duration::new(0, 999_999_999);
        assert!(d < c);
    }

    #[test]
    fn duration_is_zero() {
        assert!(Duration::ZERO.is_zero());
        assert!(!Duration::SECOND.is_zero());
    }

    #[test]
    fn duration_as_millis_u128() {
        let d = Duration::new(1, 500_000_000);
        assert_eq!(d.as_millis(), 1500);
    }

    #[test]
    fn duration_display() {
        use alloc::format;
        assert_eq!(format!("{}", Duration::from_secs(3)), "3.000s");
        assert_eq!(format!("{}", Duration::new(1, 500_000_000)), "1.500s");
        assert_eq!(format!("{}", Duration::from_millis(42)), "42ms");
        assert_eq!(format!("{}", Duration::from_micros(100)), "100us");
        assert_eq!(format!("{}", Duration::from_nanos(50)), "50ns");
    }
}
