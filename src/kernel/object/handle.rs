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
            object: super::Object::Console(super::ConsoleObject {
                console_type: super::ConsoleType::Stdin,
                subscriber: None,
            }),
        });

        // Handle 2 = stdout
        table.entries[2] = Some(HandleEntry {
            generation: 1,
            object_type: super::ObjectType::Stdout,
            object: super::Object::Console(super::ConsoleObject {
                console_type: super::ConsoleType::Stdout,
                subscriber: None,
            }),
        });

        // Handle 3 = stderr
        table.entries[3] = Some(HandleEntry {
            generation: 1,
            object_type: super::ObjectType::Stderr,
            object: super::Object::Console(super::ConsoleObject {
                console_type: super::ConsoleType::Stderr,
                subscriber: None,
            }),
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
    pub fn entries_mut(&mut self) -> impl Iterator<Item = &mut HandleEntry> {
        self.entries.iter_mut().filter_map(|entry| entry.as_mut())
    }

    /// Get iterator over all entries (for multiplexer polling)
    pub fn iter(&self) -> impl Iterator<Item = (Handle, &HandleEntry)> {
        self.entries.iter().enumerate().filter_map(|(i, entry)| {
            entry.as_ref().map(|e| (Handle::new(i, e.generation), e))
        })
    }

    /// Get mutable iterator
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (Handle, &mut HandleEntry)> {
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
