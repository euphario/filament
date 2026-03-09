//! FxHash-based HashMap. Fast for small keys (integers, short strings).
//!
//! NOT cryptographic — suitable for internal data structures only.
//! Uses open addressing with linear probing.

use alloc::vec::Vec;
use core::hash::{Hash, Hasher};
use core::mem;

// ========================================================================
// FxHasher
// ========================================================================

const SEED: u64 = 0x517cc1b727220a95;

/// FxHash — multiply-xor hasher. Fast, simple, deterministic.
pub struct FxHasher {
    hash: u64,
}

impl FxHasher {
    fn new() -> Self {
        Self { hash: 0 }
    }
}

impl Hasher for FxHasher {
    fn finish(&self) -> u64 {
        self.hash
    }

    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.hash = (self.hash.rotate_left(5) ^ b as u64).wrapping_mul(SEED);
        }
    }

    fn write_u8(&mut self, i: u8) {
        self.hash = (self.hash.rotate_left(5) ^ i as u64).wrapping_mul(SEED);
    }

    fn write_u16(&mut self, i: u16) {
        self.hash = (self.hash.rotate_left(5) ^ i as u64).wrapping_mul(SEED);
    }

    fn write_u32(&mut self, i: u32) {
        self.hash = (self.hash.rotate_left(5) ^ i as u64).wrapping_mul(SEED);
    }

    fn write_u64(&mut self, i: u64) {
        self.hash = (self.hash.rotate_left(5) ^ i).wrapping_mul(SEED);
    }

    fn write_usize(&mut self, i: usize) {
        self.hash = (self.hash.rotate_left(5) ^ i as u64).wrapping_mul(SEED);
    }
}

fn fx_hash<K: Hash>(key: &K) -> u64 {
    let mut hasher = FxHasher::new();
    key.hash(&mut hasher);
    hasher.finish()
}

// ========================================================================
// HashMap
// ========================================================================

const INITIAL_CAPACITY: usize = 8;
const LOAD_FACTOR_NUM: usize = 3;
const LOAD_FACTOR_DEN: usize = 4; // grow at 75%

/// Slot state: Empty, Occupied(key, value), or Tombstone (deleted).
enum Slot<K, V> {
    Empty,
    Occupied(K, V),
    Tombstone,
}

impl<K, V> Slot<K, V> {
    fn is_empty(&self) -> bool {
        matches!(self, Slot::Empty)
    }
}

/// Open-addressing hash map with linear probing and FxHash.
pub struct HashMap<K, V> {
    slots: Vec<Slot<K, V>>,
    len: usize,
    /// Occupied + tombstones (used for load factor)
    used: usize,
}

impl<K: Hash + Eq, V> HashMap<K, V> {
    /// Create an empty HashMap.
    pub fn new() -> Self {
        Self {
            slots: Vec::new(),
            len: 0,
            used: 0,
        }
    }

    /// Create with pre-allocated capacity.
    pub fn with_capacity(cap: usize) -> Self {
        let cap = cap.next_power_of_two().max(INITIAL_CAPACITY);
        let mut slots = Vec::with_capacity(cap);
        for _ in 0..cap {
            slots.push(Slot::Empty);
        }
        Self { slots, len: 0, used: 0 }
    }

    /// Number of entries.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Returns true if empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the table capacity (0 if not yet allocated).
    pub fn capacity(&self) -> usize {
        self.slots.len()
    }

    fn mask(&self) -> usize {
        self.slots.len() - 1
    }

    fn needs_grow(&self) -> bool {
        if self.slots.is_empty() {
            return true;
        }
        self.used * LOAD_FACTOR_DEN >= self.slots.len() * LOAD_FACTOR_NUM
    }

    fn grow(&mut self) {
        let new_cap = if self.slots.is_empty() {
            INITIAL_CAPACITY
        } else {
            self.slots.len() * 2
        };

        let old_slots = mem::replace(&mut self.slots, {
            let mut v = Vec::with_capacity(new_cap);
            for _ in 0..new_cap {
                v.push(Slot::Empty);
            }
            v
        });
        self.len = 0;
        self.used = 0;

        for slot in old_slots {
            if let Slot::Occupied(k, v) = slot {
                self.insert_no_grow(k, v);
            }
        }
    }

    fn insert_no_grow(&mut self, key: K, value: V) -> Option<V> {
        let hash = fx_hash(&key);
        let mask = self.mask();
        let mut idx = hash as usize & mask;

        loop {
            match &self.slots[idx] {
                Slot::Empty | Slot::Tombstone => {
                    let was_empty = self.slots[idx].is_empty();
                    self.slots[idx] = Slot::Occupied(key, value);
                    self.len += 1;
                    if was_empty {
                        self.used += 1;
                    }
                    return None;
                }
                Slot::Occupied(k, _) => {
                    if *k == key {
                        let old = mem::replace(&mut self.slots[idx], Slot::Occupied(key, value));
                        if let Slot::Occupied(_, v) = old {
                            return Some(v);
                        }
                        unreachable!();
                    }
                }
            }
            idx = (idx + 1) & mask;
        }
    }

    /// Insert a key-value pair. Returns the previous value if the key existed.
    pub fn insert(&mut self, key: K, value: V) -> Option<V> {
        if self.needs_grow() {
            self.grow();
        }
        self.insert_no_grow(key, value)
    }

    fn find_slot(&self, key: &K) -> Option<usize> {
        if self.slots.is_empty() {
            return None;
        }
        let hash = fx_hash(key);
        let mask = self.mask();
        let mut idx = hash as usize & mask;

        loop {
            match &self.slots[idx] {
                Slot::Empty => return None,
                Slot::Occupied(k, _) if k == key => return Some(idx),
                _ => idx = (idx + 1) & mask,
            }
        }
    }

    /// Get a reference to the value for a key.
    pub fn get(&self, key: &K) -> Option<&V> {
        self.find_slot(key).and_then(|idx| match &self.slots[idx] {
            Slot::Occupied(_, v) => Some(v),
            _ => None,
        })
    }

    /// Get a mutable reference to the value for a key.
    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        self.find_slot(key).and_then(|idx| match &mut self.slots[idx] {
            Slot::Occupied(_, v) => Some(v),
            _ => None,
        })
    }

    /// Returns true if the key exists.
    pub fn contains_key(&self, key: &K) -> bool {
        self.find_slot(key).is_some()
    }

    /// Remove a key, returning the value if it existed.
    pub fn remove(&mut self, key: &K) -> Option<V> {
        let idx = self.find_slot(key)?;
        let old = mem::replace(&mut self.slots[idx], Slot::Tombstone);
        self.len -= 1;
        // `used` stays the same (tombstone still counts for load)
        match old {
            Slot::Occupied(_, v) => Some(v),
            _ => None,
        }
    }

    /// Remove all entries.
    pub fn clear(&mut self) {
        for slot in self.slots.iter_mut() {
            *slot = Slot::Empty;
        }
        self.len = 0;
        self.used = 0;
    }

    /// Iterate over (key, value) pairs.
    pub fn iter(&self) -> Iter<'_, K, V> {
        Iter { slots: &self.slots, idx: 0 }
    }

    /// Iterate over keys.
    pub fn keys(&self) -> impl Iterator<Item = &K> {
        self.iter().map(|(k, _)| k)
    }

    /// Iterate over values.
    pub fn values(&self) -> impl Iterator<Item = &V> {
        self.iter().map(|(_, v)| v)
    }

    /// Iterate over mutable values.
    pub fn values_mut(&mut self) -> impl Iterator<Item = &mut V> {
        self.slots.iter_mut().filter_map(|slot| match slot {
            Slot::Occupied(_, v) => Some(v),
            _ => None,
        })
    }
}

impl<K: Hash + Eq, V> Default for HashMap<K, V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Hash + Eq + core::fmt::Debug, V: core::fmt::Debug> core::fmt::Debug for HashMap<K, V> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_map().entries(self.iter()).finish()
    }
}

// IntoIterator for owned HashMap
impl<K, V> IntoIterator for HashMap<K, V> {
    type Item = (K, V);
    type IntoIter = IntoIter<K, V>;
    fn into_iter(self) -> IntoIter<K, V> {
        IntoIter { slots: self.slots.into_iter() }
    }
}

pub struct IntoIter<K, V> {
    slots: alloc::vec::IntoIter<Slot<K, V>>,
}

impl<K, V> Iterator for IntoIter<K, V> {
    type Item = (K, V);
    fn next(&mut self) -> Option<(K, V)> {
        loop {
            match self.slots.next()? {
                Slot::Occupied(k, v) => return Some((k, v)),
                _ => continue,
            }
        }
    }
}

// IntoIterator for &HashMap
impl<'a, K, V> IntoIterator for &'a HashMap<K, V> {
    type Item = (&'a K, &'a V);
    type IntoIter = Iter<'a, K, V>;
    fn into_iter(self) -> Iter<'a, K, V> {
        Iter { slots: &self.slots, idx: 0 }
    }
}

/// Iterator over (key, value) references.
pub struct Iter<'a, K, V> {
    slots: &'a [Slot<K, V>],
    idx: usize,
}

impl<'a, K, V> Iterator for Iter<'a, K, V> {
    type Item = (&'a K, &'a V);
    fn next(&mut self) -> Option<(&'a K, &'a V)> {
        while self.idx < self.slots.len() {
            let i = self.idx;
            self.idx += 1;
            if let Slot::Occupied(k, v) = &self.slots[i] {
                return Some((k, v));
            }
        }
        None
    }
}

/// Collect from (K, V) iterator into HashMap.
impl<K: Hash + Eq, V> core::iter::FromIterator<(K, V)> for HashMap<K, V> {
    fn from_iter<I: IntoIterator<Item = (K, V)>>(iter: I) -> Self {
        let iter = iter.into_iter();
        let (lower, _) = iter.size_hint();
        let mut map = HashMap::with_capacity(lower.max(INITIAL_CAPACITY));
        for (k, v) in iter {
            map.insert(k, v);
        }
        map
    }
}
