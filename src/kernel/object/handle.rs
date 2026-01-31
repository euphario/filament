//! Handle and HandleTable
//!
//! Handles are user-visible references to kernel objects.
//! Each task has a HandleTable mapping handles to objects.

use super::{Object, ObjectType};
use crate::kernel::task::TaskId;

// Import Handle from abi crate - single source of truth
pub use abi::Handle;

/// Maximum handles per task
pub const MAX_HANDLES: usize = 64;

/// Entry in the handle table
pub struct HandleEntry {
    /// Generation (must match Handle)
    pub generation: u8,
    /// Object type (for quick dispatch)
    pub object_type: ObjectType,
    /// The actual object
    pub object: Object,
}

/// Per-task handle table
pub struct HandleTable {
    entries: [Option<HandleEntry>; MAX_HANDLES],
    generations: [u8; MAX_HANDLES],
}

impl HandleTable {
    pub const fn new() -> Self {
        Self {
            entries: [const { None }; MAX_HANDLES],
            generations: [1u8; MAX_HANDLES], // Start at 1 so 0 is invalid
        }
    }

    /// Create with stdin/stdout/stderr pre-allocated at handles 1, 2, 3
    pub fn new_with_stdio() -> Self {
        let mut table = Self::new();

        // Handle 1 = stdin
        table.entries[1] = Some(HandleEntry {
            generation: 1,
            object_type: super::ObjectType::Stdin,
            object: super::Object::Console(super::ConsoleObject::new(super::ConsoleType::Stdin)),
        });

        // Handle 2 = stdout
        table.entries[2] = Some(HandleEntry {
            generation: 1,
            object_type: super::ObjectType::Stdout,
            object: super::Object::Console(super::ConsoleObject::new(super::ConsoleType::Stdout)),
        });

        // Handle 3 = stderr
        table.entries[3] = Some(HandleEntry {
            generation: 1,
            object_type: super::ObjectType::Stderr,
            object: super::Object::Console(super::ConsoleObject::new(super::ConsoleType::Stderr)),
        });

        table
    }

    /// Allocate a new handle
    pub fn alloc(&mut self, object_type: ObjectType, object: Object) -> Option<Handle> {
        // Find free slot (skip 0, reserved for INVALID)
        for i in 1..MAX_HANDLES {
            if self.entries[i].is_none() {
                let gen = self.generations[i];
                self.entries[i] = Some(HandleEntry {
                    generation: gen,
                    object_type,
                    object,
                });
                return Some(Handle::new(i, gen));
            }
        }
        None
    }

    /// Get object by handle
    pub fn get(&self, handle: Handle) -> Option<&HandleEntry> {
        let idx = handle.index();
        if idx >= MAX_HANDLES {
            return None;
        }
        match &self.entries[idx] {
            Some(entry) if entry.generation == handle.generation() => Some(entry),
            _ => None,
        }
    }

    /// Get object mutably
    pub fn get_mut(&mut self, handle: Handle) -> Option<&mut HandleEntry> {
        let idx = handle.index();
        if idx >= MAX_HANDLES {
            return None;
        }
        match &mut self.entries[idx] {
            Some(entry) if entry.generation == handle.generation() => Some(entry),
            _ => None,
        }
    }

    /// Close a handle
    pub fn close(&mut self, handle: Handle) -> Option<Object> {
        let idx = handle.index();
        if idx >= MAX_HANDLES {
            return None;
        }

        // Verify generation
        if let Some(ref entry) = self.entries[idx] {
            if entry.generation != handle.generation() {
                return None;
            }
        } else {
            return None;
        }

        // Take entry and bump generation
        let entry = self.entries[idx].take()?;
        self.generations[idx] = self.generations[idx].wrapping_add(1);
        if self.generations[idx] == 0 {
            self.generations[idx] = 1;
        }

        Some(entry.object)
    }

    /// Close all handles (task cleanup)
    pub fn close_all(&mut self, owner: TaskId) -> impl Iterator<Item = Object> + '_ {
        let _ = owner; // For future use (cleanup actions)
        self.entries.iter_mut().filter_map(|entry| {
            entry.take().map(|e| e.object)
        })
    }

    /// Iterate over all entries mutably (for timer checks, etc.)
    /// NOTE: Prefer using specific operations (close_all, notify_timers, etc.)
    pub(crate) fn entries_mut(&mut self) -> impl Iterator<Item = &mut HandleEntry> {
        self.entries.iter_mut().filter_map(|entry| entry.as_mut())
    }

    /// Get iterator over all entries (for multiplexer polling)
    pub(crate) fn iter(&self) -> impl Iterator<Item = (Handle, &HandleEntry)> {
        self.entries.iter().enumerate().filter_map(|(i, entry)| {
            entry.as_ref().map(|e| (Handle::new(i, e.generation), e))
        })
    }

    /// Get mutable iterator
    pub(crate) fn iter_mut(&mut self) -> impl Iterator<Item = (Handle, &mut HandleEntry)> {
        self.entries.iter_mut().enumerate().filter_map(|(i, entry)| {
            entry.as_mut().map(|e| {
                let h = Handle::new(i, e.generation);
                (h, e)
            })
        })
    }
}

impl Default for HandleTable {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::object::{Object, ObjectType, TimerObject};

    fn make_timer() -> Object {
        Object::Timer(TimerObject::new())
    }

    #[test]
    fn test_new_table_empty() {
        let table = HandleTable::new();
        // All entries should be None
        for i in 0..MAX_HANDLES {
            let h = Handle::new(i, 1);
            assert!(table.get(h).is_none());
        }
    }

    #[test]
    fn test_alloc_returns_valid_handle() {
        let mut table = HandleTable::new();
        let handle = table.alloc(ObjectType::Timer, make_timer());
        assert!(handle.is_some());
        let h = handle.unwrap();
        assert!(h.raw() > 0); // Handle should not be 0
    }

    #[test]
    fn test_get_returns_allocated_object() {
        let mut table = HandleTable::new();
        let h = table.alloc(ObjectType::Timer, make_timer()).unwrap();

        let entry = table.get(h);
        assert!(entry.is_some());
        assert_eq!(entry.unwrap().object_type, ObjectType::Timer);
    }

    #[test]
    fn test_get_mut_returns_mutable_object() {
        let mut table = HandleTable::new();
        let h = table.alloc(ObjectType::Timer, make_timer()).unwrap();

        let entry = table.get_mut(h);
        assert!(entry.is_some());
        assert_eq!(entry.unwrap().object_type, ObjectType::Timer);
    }

    #[test]
    fn test_close_removes_object() {
        let mut table = HandleTable::new();
        let h = table.alloc(ObjectType::Timer, make_timer()).unwrap();

        // Close should return the object
        let closed = table.close(h);
        assert!(closed.is_some());

        // Get should now return None
        assert!(table.get(h).is_none());
    }

    #[test]
    fn test_close_invalid_handle() {
        let mut table = HandleTable::new();
        let invalid = Handle::new(5, 99); // Never allocated
        let closed = table.close(invalid);
        assert!(closed.is_none());
    }

    #[test]
    fn test_stale_handle_rejected() {
        let mut table = HandleTable::new();
        let h1 = table.alloc(ObjectType::Timer, make_timer()).unwrap();
        let slot = h1.index();

        // Close it
        table.close(h1);

        // Allocate again in same slot (if it reuses)
        let _h2 = table.alloc(ObjectType::Timer, make_timer()).unwrap();

        // Old handle should be stale (generation mismatch)
        assert!(table.get(h1).is_none());
    }

    #[test]
    fn test_multiple_allocations() {
        let mut table = HandleTable::new();

        let h1 = table.alloc(ObjectType::Timer, make_timer()).unwrap();
        let h2 = table.alloc(ObjectType::Timer, make_timer()).unwrap();
        let h3 = table.alloc(ObjectType::Timer, make_timer()).unwrap();

        // All should be distinct
        assert_ne!(h1.raw(), h2.raw());
        assert_ne!(h2.raw(), h3.raw());

        // All should be accessible
        assert!(table.get(h1).is_some());
        assert!(table.get(h2).is_some());
        assert!(table.get(h3).is_some());
    }

    #[test]
    fn test_table_capacity() {
        let mut table = HandleTable::new();
        let mut handles = Vec::new();

        // Allocate until full
        for _ in 0..MAX_HANDLES {
            if let Some(h) = table.alloc(ObjectType::Timer, make_timer()) {
                handles.push(h);
            } else {
                break;
            }
        }

        // Next allocation should fail
        let overflow = table.alloc(ObjectType::Timer, make_timer());
        assert!(overflow.is_none());
    }

    #[test]
    fn test_close_all() {
        let mut table = HandleTable::new();

        table.alloc(ObjectType::Timer, make_timer());
        table.alloc(ObjectType::Timer, make_timer());
        table.alloc(ObjectType::Timer, make_timer());

        // close_all returns iterator of closed objects
        let closed: Vec<_> = table.close_all(0).collect();
        assert_eq!(closed.len(), 3);
    }

    #[test]
    fn test_new_with_stdio() {
        let table = HandleTable::new_with_stdio();

        // Handles 1, 2, 3 should be stdin, stdout, stderr
        let stdin = table.get(Handle::new(1, 1));
        let stdout = table.get(Handle::new(2, 1));
        let stderr = table.get(Handle::new(3, 1));

        assert!(stdin.is_some());
        assert!(stdout.is_some());
        assert!(stderr.is_some());

        assert_eq!(stdin.unwrap().object_type, ObjectType::Stdin);
        assert_eq!(stdout.unwrap().object_type, ObjectType::Stdout);
        assert_eq!(stderr.unwrap().object_type, ObjectType::Stderr);
    }

    #[test]
    fn test_iter_empty() {
        let table = HandleTable::new();
        let count = table.iter().count();
        assert_eq!(count, 0);
    }

    #[test]
    fn test_iter_returns_all() {
        let mut table = HandleTable::new();

        table.alloc(ObjectType::Timer, make_timer());
        table.alloc(ObjectType::Timer, make_timer());

        let count = table.iter().count();
        assert_eq!(count, 2);
    }
}
