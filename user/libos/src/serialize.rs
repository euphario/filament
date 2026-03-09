//! Shared serialization primitives for logging and tracing
//!
//! This module provides the common `Serialize` trait and value types used by
//! both ulog (logging) and utrace (tracing) to write structured data to ring buffers.

// ============================================================================
// Value Types for Structured Key-Value Pairs
// ============================================================================

/// Value types for key-value pairs in log/trace records
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum ValueType {
    /// Unsigned 64-bit, formatted as decimal
    U64 = 0,
    /// Signed 64-bit, formatted as signed decimal
    I64 = 1,
    /// 32-bit hex, formatted as 0x%08x
    Hex32 = 2,
    /// 64-bit hex, formatted as 0x%016x
    Hex64 = 3,
    /// Boolean, formatted as true/false
    Bool = 4,
    /// String (length-prefixed)
    Str = 5,
}

// ============================================================================
// Serialize Trait
// ============================================================================

/// Trait for serializing values directly into ring buffers.
/// This allows any lifetime for strings since bytes are copied immediately.
pub trait Serialize {
    /// Type marker for this value type
    fn type_marker(&self) -> u8;
    /// Size in bytes when serialized
    fn serialized_size(&self) -> usize;
    /// Serialize value bytes into buffer, returns bytes written
    fn serialize(&self, buf: &mut [u8]) -> usize;
}

// ============================================================================
// Serialize Implementations for Primitive Types
// ============================================================================

impl Serialize for u64 {
    fn type_marker(&self) -> u8 { ValueType::U64 as u8 }
    fn serialized_size(&self) -> usize { 8 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..8].copy_from_slice(&self.to_le_bytes());
        8
    }
}

impl Serialize for u32 {
    fn type_marker(&self) -> u8 { ValueType::U64 as u8 }
    fn serialized_size(&self) -> usize { 8 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..8].copy_from_slice(&(*self as u64).to_le_bytes());
        8
    }
}

impl Serialize for u16 {
    fn type_marker(&self) -> u8 { ValueType::U64 as u8 }
    fn serialized_size(&self) -> usize { 8 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..8].copy_from_slice(&(*self as u64).to_le_bytes());
        8
    }
}

impl Serialize for u8 {
    fn type_marker(&self) -> u8 { ValueType::U64 as u8 }
    fn serialized_size(&self) -> usize { 8 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..8].copy_from_slice(&(*self as u64).to_le_bytes());
        8
    }
}

impl Serialize for usize {
    fn type_marker(&self) -> u8 { ValueType::U64 as u8 }
    fn serialized_size(&self) -> usize { 8 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..8].copy_from_slice(&(*self as u64).to_le_bytes());
        8
    }
}

impl Serialize for i64 {
    fn type_marker(&self) -> u8 { ValueType::I64 as u8 }
    fn serialized_size(&self) -> usize { 8 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..8].copy_from_slice(&self.to_le_bytes());
        8
    }
}

impl Serialize for i32 {
    fn type_marker(&self) -> u8 { ValueType::I64 as u8 }
    fn serialized_size(&self) -> usize { 8 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..8].copy_from_slice(&(*self as i64).to_le_bytes());
        8
    }
}

impl Serialize for bool {
    fn type_marker(&self) -> u8 { ValueType::Bool as u8 }
    fn serialized_size(&self) -> usize { 1 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = if *self { 1 } else { 0 };
        1
    }
}

// String with any lifetime - the key feature!
impl Serialize for &str {
    fn type_marker(&self) -> u8 { ValueType::Str as u8 }
    fn serialized_size(&self) -> usize { 1 + self.len().min(255) }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        let len = self.len().min(255);
        buf[0] = len as u8;
        buf[1..1 + len].copy_from_slice(&self.as_bytes()[..len]);
        1 + len
    }
}

// ============================================================================
// Hex Wrapper Types (for explicit hex formatting)
// ============================================================================

/// 32-bit value that formats as hex (0x%08x)
#[derive(Clone, Copy, Debug)]
pub struct Hex32(pub u32);

/// 64-bit value that formats as hex (0x%016x)
#[derive(Clone, Copy, Debug)]
pub struct Hex64(pub u64);

impl Serialize for Hex32 {
    fn type_marker(&self) -> u8 { ValueType::Hex32 as u8 }
    fn serialized_size(&self) -> usize { 4 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..4].copy_from_slice(&self.0.to_le_bytes());
        4
    }
}

impl Serialize for Hex64 {
    fn type_marker(&self) -> u8 { ValueType::Hex64 as u8 }
    fn serialized_size(&self) -> usize { 8 }
    fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[..8].copy_from_slice(&self.0.to_le_bytes());
        8
    }
}

/// Convenience constructor for hex32
pub const fn hex32(v: u32) -> Hex32 {
    Hex32(v)
}

/// Convenience constructor for hex64
pub const fn hex64(v: u64) -> Hex64 {
    Hex64(v)
}
