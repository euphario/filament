//! Lock-free SPSC ring primitives.
//!
//! [`HalfRing`] handles one direction of a byte ring in shared memory.
//! [`TypedRing`] does the same for fixed-size structured items.
//! [`NapiRing`] wraps a TypedRing with a trigger handle for NAPI-like polling.
//!
//! These are pure building blocks — no shmem allocation, no notification.
//! For complete pipes with shmem and notify/wait, see `libf::sync`.

use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering, fence};

// ============================================================================
// HalfRing — SPSC byte ring
// ============================================================================

/// One direction of a single-producer single-consumer byte ring.
///
/// Lives in shared memory. One process pushes bytes in, another pulls them
/// out. Lock-free via atomic head/tail cursors with Acquire/Release ordering.
///
/// # Safety contract
///
/// - Exactly one process may call [`push`](Self::push) (the writer).
/// - Exactly one process may call [`pull`](Self::pull) (the reader).
/// - The backing memory must remain mapped for the lifetime of this struct.
pub struct HalfRing {
    buffer: *mut u8,
    capacity: usize,
    /// Reader advances head after consuming bytes.
    head: &'static AtomicU32,
    /// Writer advances tail after producing bytes.
    tail: &'static AtomicU32,
}

impl HalfRing {
    /// Construct from raw components.
    ///
    /// # Safety
    ///
    /// - `buffer` must point to `capacity` bytes of valid, mapped memory.
    /// - `capacity` must be a power of two.
    /// - `head` and `tail` must be valid AtomicU32s that outlive this struct.
    pub unsafe fn from_raw_parts(
        buffer: *mut u8,
        capacity: usize,
        head: &'static AtomicU32,
        tail: &'static AtomicU32,
    ) -> Self {
        debug_assert!(capacity.is_power_of_two(), "ring capacity must be power of two");
        Self { buffer, capacity, head, tail }
    }

    /// Write bytes into the ring. Returns how many were accepted.
    ///
    /// Partial writes are normal — the caller should retry for the rest.
    /// Never blocks; returns 0 when the ring is full.
    pub fn push(&self, data: &[u8]) -> usize {
        // We own tail (writer side), so Relaxed is fine for our own cursor.
        // Head belongs to the reader on another core — Acquire ensures we
        // see their latest consumption before calculating free space.
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Relaxed);

        let count = self.writable_between(head, tail).min(data.len());
        if count == 0 {
            return 0;
        }

        self.copy_in((tail as usize) & self.mask(), &data[..count]);

        // Ensure bytes land in memory before we publish the new tail.
        // Without this, the reader could see the updated tail and
        // read stale (pre-write) bytes from the buffer.
        fence(Ordering::Release);
        self.tail.store(tail.wrapping_add(count as u32), Ordering::Release);

        count
    }

    /// Read bytes from the ring. Returns how many were delivered.
    ///
    /// Returns 0 if the ring is empty. Never blocks.
    pub fn pull(&self, buf: &mut [u8]) -> usize {
        // We own head (reader side), so Relaxed for our own cursor.
        // Tail belongs to the writer — Acquire ensures we see all the
        // bytes they wrote before they updated tail.
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        let count = self.readable_between(head, tail).min(buf.len());
        if count == 0 {
            return 0;
        }

        self.copy_out((head as usize) & self.mask(), &mut buf[..count]);

        // Release: let the writer see our updated head so it can
        // reclaim the space we just consumed.
        self.head.store(head.wrapping_add(count as u32), Ordering::Release);

        count
    }

    /// How many bytes can be written before the ring is full.
    ///
    /// Intended for the writer side. If the reader calls this, the result
    /// may be slightly stale but never unsafe.
    pub fn writable(&self) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Relaxed);
        self.writable_between(head, tail)
    }

    /// How many bytes are waiting to be read.
    ///
    /// Intended for the reader side. If the writer calls this, the result
    /// may be slightly stale but never unsafe.
    pub fn readable(&self) -> usize {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);
        self.readable_between(head, tail)
    }

    /// Total ring capacity.
    ///
    /// Usable capacity is `capacity - 1` because one slot is reserved
    /// to distinguish "full" from "empty".
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    // --- Private helpers ---

    fn mask(&self) -> usize {
        self.capacity - 1
    }

    /// Bytes the writer can add, given a snapshot of both cursors.
    fn writable_between(&self, head: u32, tail: u32) -> usize {
        let used = (tail.wrapping_sub(head) as usize) & self.mask();
        // One slot stays empty so we can tell "full" from "empty".
        self.capacity - 1 - used
    }

    /// Bytes the reader can consume, given a snapshot of both cursors.
    fn readable_between(&self, head: u32, tail: u32) -> usize {
        (tail.wrapping_sub(head) as usize) & self.mask()
    }

    /// Copy `data` into the ring at `pos`, wrapping around the end.
    fn copy_in(&self, pos: usize, data: &[u8]) {
        let before_wrap = data.len().min(self.capacity - pos);
        unsafe {
            ptr::copy_nonoverlapping(data.as_ptr(), self.buffer.add(pos), before_wrap);
            if before_wrap < data.len() {
                ptr::copy_nonoverlapping(
                    data.as_ptr().add(before_wrap),
                    self.buffer,
                    data.len() - before_wrap,
                );
            }
        }
    }

    /// Copy from the ring at `pos` into `buf`, wrapping around the end.
    fn copy_out(&self, pos: usize, buf: &mut [u8]) {
        let before_wrap = buf.len().min(self.capacity - pos);
        unsafe {
            ptr::copy_nonoverlapping(self.buffer.add(pos), buf.as_mut_ptr(), before_wrap);
            if before_wrap < buf.len() {
                ptr::copy_nonoverlapping(
                    self.buffer,
                    buf.as_mut_ptr().add(before_wrap),
                    buf.len() - before_wrap,
                );
            }
        }
    }
}

// ============================================================================
// TypedRing — SPSC ring for structured items
// ============================================================================

/// One direction of a single-producer single-consumer ring for fixed-size items.
///
/// Like [`HalfRing`] but carries discrete `T` values instead of raw bytes.
/// Useful for command/completion rings, event queues, or any SPSC protocol
/// with structured entries.
///
/// `T` must be `Copy` so items can be safely memcpy'd in and out of the ring.
/// For cross-process use, `T` should also be `#[repr(C)]` to guarantee layout.
///
/// # Safety contract
///
/// Same as HalfRing: one writer, one reader, memory must stay mapped.
pub struct TypedRing<T: Copy> {
    buffer: *mut T,
    /// Number of T slots (power of two).
    capacity: usize,
    head: &'static AtomicU32,
    tail: &'static AtomicU32,
    _marker: PhantomData<T>,
}

impl<T: Copy> TypedRing<T> {
    /// Construct from raw components.
    ///
    /// # Safety
    ///
    /// - `buffer` must point to `capacity` properly-aligned `T` slots.
    /// - `capacity` must be a power of two.
    /// - `head` and `tail` must be valid AtomicU32s that outlive this ring.
    pub unsafe fn from_raw_parts(
        buffer: *mut T,
        capacity: usize,
        head: &'static AtomicU32,
        tail: &'static AtomicU32,
    ) -> Self {
        debug_assert!(capacity.is_power_of_two(), "ring capacity must be power of two");
        Self { buffer, capacity, head, tail, _marker: PhantomData }
    }

    /// Push one item into the ring.
    ///
    /// Returns `true` if accepted, `false` if the ring is full.
    pub fn push(&self, item: &T) -> bool {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Relaxed);

        if self.writable_between(head, tail) == 0 {
            return false;
        }

        let pos = (tail as usize) & self.mask();
        unsafe { ptr::write(self.buffer.add(pos), *item) };

        fence(Ordering::Release);
        self.tail.store(tail.wrapping_add(1), Ordering::Release);

        true
    }

    /// Pull one item from the ring.
    ///
    /// Returns `None` if the ring is empty.
    pub fn pull(&self) -> Option<T> {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        if self.readable_between(head, tail) == 0 {
            return None;
        }

        let pos = (head as usize) & self.mask();
        let item = unsafe { ptr::read(self.buffer.add(pos)) };

        self.head.store(head.wrapping_add(1), Ordering::Release);

        Some(item)
    }

    /// Push multiple items. Returns how many were accepted.
    pub fn push_batch(&self, items: &[T]) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Relaxed);

        let count = self.writable_between(head, tail).min(items.len());
        if count == 0 {
            return 0;
        }

        let start = (tail as usize) & self.mask();
        self.typed_copy_in(start, &items[..count]);

        fence(Ordering::Release);
        self.tail.store(tail.wrapping_add(count as u32), Ordering::Release);

        count
    }

    /// Pull multiple items. Returns how many were delivered.
    pub fn pull_batch(&self, buf: &mut [T]) -> usize {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        let count = self.readable_between(head, tail).min(buf.len());
        if count == 0 {
            return 0;
        }

        let start = (head as usize) & self.mask();
        self.typed_copy_out(start, &mut buf[..count]);

        self.head.store(head.wrapping_add(count as u32), Ordering::Release);

        count
    }

    /// How many items can be pushed before the ring is full.
    pub fn writable(&self) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Relaxed);
        self.writable_between(head, tail)
    }

    /// How many items are waiting to be pulled.
    pub fn readable(&self) -> usize {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);
        self.readable_between(head, tail)
    }

    /// Total number of slots.
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Peek at the next item without consuming it.
    ///
    /// Returns `None` if the ring is empty. The item stays in the ring —
    /// call [`pull`](Self::pull) to actually consume it.
    pub fn peek(&self) -> Option<T> {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        if self.readable_between(head, tail) == 0 {
            return None;
        }

        let pos = (head as usize) & self.mask();
        let item = unsafe { ptr::read(self.buffer.add(pos)) };

        Some(item)
    }

    /// Peek at an item at a given offset from head, without consuming.
    ///
    /// `offset` is relative to the current head position (0 = head, 1 = head+1).
    /// Returns `None` if the offset is past the tail.
    pub fn peek_at(&self, offset: u32) -> Option<T> {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        let readable = self.readable_between(head, tail);
        if (offset as usize) >= readable {
            return None;
        }

        let pos = (head.wrapping_add(offset) as usize) & self.mask();
        let item = unsafe { ptr::read(self.buffer.add(pos)) };

        Some(item)
    }

    /// Advance head by `n` entries without reading them.
    ///
    /// Used for batch-ack patterns: peek at items, process them, then
    /// advance the head in one shot. Caller must ensure `n` does not
    /// exceed the number of readable items.
    pub fn advance(&self, n: u32) {
        if n == 0 {
            return;
        }
        let head = self.head.load(Ordering::Relaxed);
        self.head.store(head.wrapping_add(n), Ordering::Release);
    }

    // --- Private helpers ---

    fn mask(&self) -> usize {
        self.capacity - 1
    }

    fn writable_between(&self, head: u32, tail: u32) -> usize {
        let used = (tail.wrapping_sub(head) as usize) & self.mask();
        self.capacity - 1 - used
    }

    fn readable_between(&self, head: u32, tail: u32) -> usize {
        (tail.wrapping_sub(head) as usize) & self.mask()
    }

    fn typed_copy_in(&self, pos: usize, items: &[T]) {
        let before_wrap = items.len().min(self.capacity - pos);
        unsafe {
            ptr::copy_nonoverlapping(items.as_ptr(), self.buffer.add(pos), before_wrap);
            if before_wrap < items.len() {
                ptr::copy_nonoverlapping(
                    items.as_ptr().add(before_wrap),
                    self.buffer,
                    items.len() - before_wrap,
                );
            }
        }
    }

    fn typed_copy_out(&self, pos: usize, buf: &mut [T]) {
        let before_wrap = buf.len().min(self.capacity - pos);
        unsafe {
            ptr::copy_nonoverlapping(self.buffer.add(pos), buf.as_mut_ptr(), before_wrap);
            if before_wrap < buf.len() {
                ptr::copy_nonoverlapping(
                    self.buffer,
                    buf.as_mut_ptr().add(before_wrap),
                    buf.len() - before_wrap,
                );
            }
        }
    }
}

// ============================================================================
// NapiRing — TypedRing with NAPI-like notification
// ============================================================================

/// A [`TypedRing`] with NAPI-like notification.
///
/// Each NapiRing is one direction: one producer pushes, one consumer pulls.
/// The trigger handle (channel doorbell, IRQ, timer) tells the consumer
/// "go poll the ring."
///
/// **Producer side**: push items, call [`notify()`](Self::notify) to ring trigger.
/// **Consumer side**: register [`handle()`](Self::handle) with Mux, call
/// [`begin_poll()`](Self::begin_poll) when Mux fires, drain ring, call
/// [`end_poll()`](Self::end_poll) when empty.
pub struct NapiRing<T: Copy> {
    ring: TypedRing<T>,
    /// Trigger handle — any readable kernel object (channel, IRQ, timer).
    trigger: crate::syscall::Handle,
    /// In poll mode: trigger acked, actively draining the ring.
    polling: bool,
}

impl<T: Copy> NapiRing<T> {
    /// Create from a TypedRing and a trigger handle.
    ///
    /// The trigger is the consumer's end of a notification channel —
    /// typically from [`ChannelDoorbell::pair()`](crate::ring::ChannelDoorbell::pair).
    pub fn new(ring: TypedRing<T>, trigger: crate::syscall::Handle) -> Self {
        Self { ring, trigger, polling: false }
    }

    // --- Producer side ---

    /// Push one item into the ring. Returns `true` if accepted.
    pub fn push(&self, item: &T) -> bool {
        self.ring.push(item)
    }

    /// Push multiple items. Returns how many were accepted.
    pub fn push_batch(&self, items: &[T]) -> usize {
        self.ring.push_batch(items)
    }

    /// Ring the trigger to wake the consumer.
    ///
    /// Call after pushing items. Writes a single byte to the trigger handle.
    /// If the handle's buffer is full, the consumer already has a pending
    /// notification — safe to ignore the error.
    pub fn notify(&self) {
        let _ = crate::syscall::write(self.trigger, &[1]);
    }

    // --- Consumer side ---

    /// Pull one item from the ring.
    pub fn poll(&self) -> Option<T> {
        self.ring.pull()
    }

    /// Pull multiple items. Returns how many were delivered.
    pub fn poll_batch(&self, buf: &mut [T]) -> usize {
        self.ring.pull_batch(buf)
    }

    /// Peek at the next item without consuming it.
    pub fn peek(&self) -> Option<T> {
        self.ring.peek()
    }

    /// Handle for Mux registration.
    ///
    /// Register this with `Mux::add(handle, MuxFilter::Readable)`.
    pub fn handle(&self) -> crate::syscall::Handle {
        self.trigger
    }

    /// Enter poll mode after Mux fires.
    ///
    /// Drains pending notification bytes from the trigger so it won't
    /// re-fire spuriously. Call this, then drain the ring until empty,
    /// then call [`end_poll()`](Self::end_poll).
    pub fn begin_poll(&mut self) {
        let mut buf = [0u8; 64];
        let _ = crate::syscall::try_read(self.trigger, &mut buf);
        self.polling = true;
    }

    /// Exit poll mode.
    ///
    /// The ring is empty — the next `notify()` from the producer will
    /// fire the Mux again.
    pub fn end_poll(&mut self) {
        self.polling = false;
    }

    /// Whether we're currently in poll mode.
    pub fn is_polling(&self) -> bool {
        self.polling
    }

    // --- Shared ---

    /// How many items are waiting to be pulled.
    pub fn readable(&self) -> usize {
        self.ring.readable()
    }

    /// How many items can be pushed.
    pub fn writable(&self) -> usize {
        self.ring.writable()
    }

    /// Total number of slots.
    pub fn capacity(&self) -> usize {
        self.ring.capacity()
    }

    /// Access the underlying TypedRing.
    pub fn inner(&self) -> &TypedRing<T> {
        &self.ring
    }
}
