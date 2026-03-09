//! Shared-memory synchronization primitives.
//!
//! Lock-free building blocks for inter-process communication,
//! analogous to how `std::sync` provides intra-process coordination.
//!
//! # Primitives
//!
//! **Byte-oriented:**
//! - [`HalfRing`] — One direction of a SPSC byte ring. The core building block.
//! - [`SharedPipe`] — Bidirectional byte pipe: two HalfRings + shmem notification.
//!
//! **Typed (structured messages):**
//! - [`TypedRing`] — One direction of a SPSC ring carrying fixed-size `T` items.
//! - [`SharedChannel`] — Bidirectional typed channel: two TypedRings + shmem notification.
//!
//! # How data flows
//!
//! ```text
//! Process A (creator)          Shared Memory           Process B (opener)
//! ┌───────────────┐     ┌──────────────────────┐     ┌───────────────┐
//! │     push() ──────→  │       Ring A         ──────→ │ pull()        │
//! │     pull() ←──────  │       Ring B         ←────── │ push()        │
//! └───────────────┘     └──────────────────────┘     └───────────────┘
//! ```
//!
//! The creator's outgoing is the opener's incoming, and vice versa.
//! Both SharedPipe and SharedChannel handle this flip automatically.

use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering, fence};

use crate::time::{Duration, Instant};
use libsys::ipc::Shmem;
use libsys::syscall;

// Re-export the core ring primitives from libsys.
// These live in libsys to avoid a circular dependency (libf → libsys → libf).
// SharedPipe and SharedChannel add shmem allocation and notification on top.
pub use libsys::half_ring::{HalfRing, TypedRing};

// ============================================================================
// SharedPipe — bidirectional byte pipe over shared memory
// ============================================================================

/// Magic number at the start of a SharedPipe's memory region.
const PIPE_MAGIC: u32 = 0x50495045; // "PIPE"

/// Size of the pipe header in bytes (cache-line aligned).
const HEADER_SIZE: usize = 64;

/// Header at the start of a SharedPipe's shared memory region.
///
/// ```text
/// [PipeHeader (64B)][Ring A buffer (a_size)][Ring B buffer (b_size)]
/// ```
///
/// Ring A: creator writes, opener reads.
/// Ring B: opener writes, creator reads.
#[repr(C)]
struct PipeHeader {
    a_head: AtomicU32,
    a_tail: AtomicU32,
    a_size: u32,

    b_head: AtomicU32,
    b_tail: AtomicU32,
    b_size: u32,

    magic: u32,
    _pad: [u8; 36],
}

/// Bidirectional byte pipe over shared memory.
///
/// Two processes coordinate through a single shmem region containing
/// two ring buffers — one for each direction. The creator's "outgoing"
/// is the opener's "incoming", and vice versa.
///
/// # Variants
///
/// Use named constructors for common scenarios:
/// - [`console()`](Self::console) — Terminal I/O: large output buffer, small input.
/// - [`balanced()`](Self::balanced) — Equal both directions (16K each).
/// - [`bulk()`](Self::bulk) — High-throughput streaming (256K each).
///
/// Or [`create()`](Self::create) with explicit sizes for full control.
pub struct SharedPipe {
    /// The buffer we write into. Peer reads from this.
    outgoing: HalfRing,
    /// The buffer peer writes into. We read from this.
    incoming: HalfRing,
    shmem: Shmem,
}

impl SharedPipe {
    /// Create a new pipe with explicit buffer sizes.
    ///
    /// Both sizes must be powers of two (e.g., 4096, 65536).
    ///
    /// After creation, call `allow(peer_pid)` then send `shmem_id()` to the
    /// peer so it can `open()` the same pipe.
    pub fn create(outgoing_size: usize, incoming_size: usize) -> Option<Self> {
        assert!(outgoing_size.is_power_of_two(), "outgoing_size must be power of two");
        assert!(incoming_size.is_power_of_two(), "incoming_size must be power of two");

        let total = HEADER_SIZE + outgoing_size + incoming_size;
        let shmem = Shmem::create(total).ok()?;
        let base = shmem.vaddr() as *mut u8;

        // Zero the header, then fill in sizes and magic.
        unsafe {
            ptr::write_bytes(base, 0, HEADER_SIZE);
            let header = &mut *(base as *mut PipeHeader);
            header.a_size = outgoing_size as u32;
            header.b_size = incoming_size as u32;
            header.magic = PIPE_MAGIC;
        }

        // Ensure header writes are visible before we share the shmem_id.
        // The opener's Acquire fence pairs with this.
        fence(Ordering::Release);

        Self::from_shmem_as_creator(shmem)
    }

    // --- Named constructors for common scenarios ---

    /// Terminal I/O: 4K outgoing (keystrokes), 64K incoming (output bursts).
    ///
    /// Sized for shell-like workloads where the child (opener) produces
    /// far more output than the parent (creator) sends as input.
    /// The opener's outgoing ring is the creator's incoming — so the
    /// large buffer is `incoming_size` to give the child room to write.
    pub fn console() -> Option<Self> {
        Self::create(4096, 65536)
    }

    /// Balanced: 16K each direction.
    ///
    /// Good default for bidirectional protocols where neither side
    /// produces dramatically more data than the other.
    pub fn balanced() -> Option<Self> {
        Self::create(16384, 16384)
    }

    /// High-throughput: 256K each direction.
    ///
    /// For bulk data transfer where latency matters less than bandwidth.
    /// Uses 512K + 64B of shmem.
    pub fn bulk() -> Option<Self> {
        Self::create(262144, 262144)
    }

    /// Open an existing pipe as the remote side.
    ///
    /// The shmem_id comes from the creator (via IPC, mailbox, etc.).
    /// Returns `None` if the shmem can't be mapped or isn't a valid pipe.
    pub fn open(shmem_id: u32) -> Option<Self> {
        let shmem = Shmem::open_existing(shmem_id).ok()?;

        // Pair with the creator's Release fence — ensures we see their header.
        fence(Ordering::Acquire);

        let base = shmem.vaddr() as *mut u8;
        let header = unsafe { &*(base as *const PipeHeader) };

        if header.magic != PIPE_MAGIC {
            return None;
        }

        let a_size = header.a_size as usize;
        let b_size = header.b_size as usize;
        let a_buf = unsafe { base.add(HEADER_SIZE) };
        let b_buf = unsafe { base.add(HEADER_SIZE + a_size) };

        // Roles flip: creator's outgoing (Ring A) is our incoming,
        //             creator's incoming (Ring B) is our outgoing.
        Some(Self {
            incoming: unsafe {
                HalfRing::from_raw_parts(a_buf, a_size, &header.a_head, &header.a_tail)
            },
            outgoing: unsafe {
                HalfRing::from_raw_parts(b_buf, b_size, &header.b_head, &header.b_tail)
            },
            shmem,
        })
    }

    // --- Non-blocking data transfer ---

    /// Write bytes to the peer. Returns how many were accepted.
    ///
    /// Never blocks. Returns 0 if the outgoing ring is full.
    pub fn push(&self, data: &[u8]) -> usize {
        self.outgoing.push(data)
    }

    /// Read bytes from the peer. Returns how many were delivered.
    ///
    /// Never blocks. Returns 0 if the incoming ring is empty.
    pub fn pull(&self, buf: &mut [u8]) -> usize {
        self.incoming.pull(buf)
    }

    /// How many bytes we can write before the outgoing ring is full.
    pub fn writable(&self) -> usize {
        self.outgoing.writable()
    }

    /// How many bytes the peer has sent that we haven't read yet.
    pub fn readable(&self) -> usize {
        self.incoming.readable()
    }

    // --- Blocking data transfer ---

    /// Write all bytes, blocking until the peer drains enough space.
    ///
    /// Notifies the peer after each chunk so it can drain. Waits for
    /// the ring to be at least half empty before retrying, to avoid
    /// thrashing on small writes.
    pub fn push_all(&self, data: &[u8]) {
        let mut offset = 0;

        while offset < data.len() {
            let written = self.outgoing.push(&data[offset..]);
            offset += written;

            if offset < data.len() {
                // Ring full — tell the peer to drain, then wait for space.
                self.notify();
                self.wait_for_space();
            }
        }

        if !data.is_empty() {
            self.notify();
        }
    }

    /// Read bytes, blocking until at least one byte arrives.
    ///
    /// Always returns at least 1 byte (unless the pipe is broken).
    /// For non-blocking reads, use [`pull()`](Self::pull) instead.
    pub fn pull_blocking(&self, buf: &mut [u8]) -> usize {
        loop {
            let n = self.incoming.pull(buf);
            if n > 0 {
                return n;
            }
            // Nothing available — wait for the peer to push.
            self.wait(5000);
        }
    }

    /// Read bytes with a timeout.
    ///
    /// Returns 0 if no data arrives within `timeout_ms` milliseconds.
    pub fn pull_timeout(&self, buf: &mut [u8], timeout_ms: u32) -> usize {
        // Fast path: data already waiting.
        let n = self.incoming.pull(buf);
        if n > 0 {
            return n;
        }

        let deadline = Instant::now() + Duration::from_millis(timeout_ms as u64);
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return 0;
            }
            self.wait((remaining.as_millis() as u32).max(1));

            let n = self.incoming.pull(buf);
            if n > 0 {
                return n;
            }
        }
    }

    // --- Notification ---

    /// Wake the peer if it's blocked in `wait()`.
    ///
    /// Call this after pushing data so the peer knows to check.
    /// Also useful after pulling data so the peer can reclaim space.
    pub fn notify(&self) {
        let _ = self.shmem.notify();
    }

    /// Block until the peer calls `notify()`, or until `timeout_ms` expires.
    ///
    /// Returns `true` if woken by notification, `false` on timeout.
    /// Spurious wakeups are possible — always re-check the ring.
    pub fn wait(&self, timeout_ms: u32) -> bool {
        self.shmem.wait(timeout_ms).is_ok()
    }

    // --- Direct ring access ---

    /// The outgoing ring (we write, peer reads).
    pub fn outgoing(&self) -> &HalfRing {
        &self.outgoing
    }

    /// The incoming ring (peer writes, we read).
    pub fn incoming(&self) -> &HalfRing {
        &self.incoming
    }

    // --- Shmem plumbing ---

    /// The shmem ID to send to the peer so it can `open()` this pipe.
    pub fn shmem_id(&self) -> u32 {
        self.shmem.shmem_id()
    }

    /// The kernel handle, for registering with a Mux.
    pub fn handle(&self) -> syscall::Handle {
        self.shmem.handle()
    }

    /// Grant another process permission to map this pipe's shmem.
    pub fn allow(&self, peer_pid: u32) -> bool {
        self.shmem.allow(peer_pid).is_ok()
    }

    // --- Private helpers ---

    /// Wait until the outgoing ring has meaningful space available.
    ///
    /// Waits for at least half the ring to be free before returning.
    /// This avoids a pathological pattern where we push one byte,
    /// wake the peer, peer reads one byte, wakes us, repeat.
    fn wait_for_space(&self) {
        let half = self.outgoing.capacity() / 2;
        loop {
            self.wait(1000);
            if self.outgoing.writable() >= half {
                break;
            }
            // Peer may have missed the wake — nudge again.
            self.notify();
        }
    }

    /// Build a SharedPipe from shmem that was just created (creator side).
    fn from_shmem_as_creator(shmem: Shmem) -> Option<Self> {
        let base = shmem.vaddr() as *mut u8;
        let header = unsafe { &*(base as *const PipeHeader) };
        let a_size = header.a_size as usize;
        let a_buf = unsafe { base.add(HEADER_SIZE) };
        let b_buf = unsafe { base.add(HEADER_SIZE + a_size) };

        Some(Self {
            outgoing: unsafe {
                HalfRing::from_raw_parts(a_buf, a_size, &header.a_head, &header.a_tail)
            },
            incoming: unsafe {
                let b_size = header.b_size as usize;
                HalfRing::from_raw_parts(b_buf, b_size, &header.b_head, &header.b_tail)
            },
            shmem,
        })
    }
}

// ============================================================================
// SharedChannel — bidirectional typed channel over shared memory
// ============================================================================

/// Magic number for SharedChannel.
const CHANNEL_MAGIC: u32 = 0x43484E4C; // "CHNL"

/// Header at the start of a SharedChannel's shared memory region.
///
/// ```text
/// [ChannelHeader (64B)][Ring A slots (a_count × size_of::<T>)][Ring B slots]
/// ```
#[repr(C)]
struct ChannelHeader {
    a_head: AtomicU32,
    a_tail: AtomicU32,
    /// Number of T slots in ring A (power of two).
    a_count: u32,

    b_head: AtomicU32,
    b_tail: AtomicU32,
    /// Number of T slots in ring B (power of two).
    b_count: u32,

    /// Size of each T in bytes (for validation on open).
    item_size: u32,
    magic: u32,
    _pad: [u8; 32],
}

/// Bidirectional typed channel over shared memory.
///
/// Like [`SharedPipe`] but carries discrete `T` items instead of raw bytes.
/// Each side pushes and pulls whole items — no partial reads or framing.
///
/// # Example use cases
///
/// - Command/response protocols between driver layers
/// - Event queues between a supervisor and its children
/// - Any structured IPC where messages have a fixed layout
pub struct SharedChannel<T: Copy> {
    outgoing: TypedRing<T>,
    incoming: TypedRing<T>,
    shmem: Shmem,
}

impl<T: Copy> SharedChannel<T> {
    /// Create a new channel with the given slot counts.
    ///
    /// `outgoing_slots` and `incoming_slots` must be powers of two.
    /// The total shmem consumed is `64 + (outgoing + incoming) * size_of::<T>()`.
    pub fn create(outgoing_slots: usize, incoming_slots: usize) -> Option<Self> {
        assert!(outgoing_slots.is_power_of_two());
        assert!(incoming_slots.is_power_of_two());

        let item_size = core::mem::size_of::<T>();
        let total = HEADER_SIZE + (outgoing_slots + incoming_slots) * item_size;
        let shmem = Shmem::create(total).ok()?;
        let base = shmem.vaddr() as *mut u8;

        unsafe {
            ptr::write_bytes(base, 0, HEADER_SIZE);
            let header = &mut *(base as *mut ChannelHeader);
            header.a_count = outgoing_slots as u32;
            header.b_count = incoming_slots as u32;
            header.item_size = item_size as u32;
            header.magic = CHANNEL_MAGIC;
        }

        fence(Ordering::Release);

        let header = unsafe { &*(base as *const ChannelHeader) };
        let a_buf = unsafe { base.add(HEADER_SIZE) as *mut T };
        let b_buf = unsafe { base.add(HEADER_SIZE + outgoing_slots * item_size) as *mut T };

        Some(Self {
            outgoing: unsafe {
                TypedRing::from_raw_parts(a_buf, outgoing_slots, &header.a_head, &header.a_tail)
            },
            incoming: unsafe {
                TypedRing::from_raw_parts(b_buf, incoming_slots, &header.b_head, &header.b_tail)
            },
            shmem,
        })
    }

    /// Open an existing channel as the remote side.
    pub fn open(shmem_id: u32) -> Option<Self> {
        let shmem = Shmem::open_existing(shmem_id).ok()?;
        fence(Ordering::Acquire);

        let base = shmem.vaddr() as *mut u8;
        let header = unsafe { &*(base as *const ChannelHeader) };

        if header.magic != CHANNEL_MAGIC {
            return None;
        }

        // Validate that T matches what the creator used.
        let item_size = core::mem::size_of::<T>();
        if header.item_size != item_size as u32 {
            return None;
        }

        let a_count = header.a_count as usize;
        let b_count = header.b_count as usize;
        let a_buf = unsafe { base.add(HEADER_SIZE) as *mut T };
        let b_buf = unsafe { base.add(HEADER_SIZE + a_count * item_size) as *mut T };

        // Roles flip, same as SharedPipe.
        Some(Self {
            incoming: unsafe {
                TypedRing::from_raw_parts(a_buf, a_count, &header.a_head, &header.a_tail)
            },
            outgoing: unsafe {
                TypedRing::from_raw_parts(b_buf, b_count, &header.b_head, &header.b_tail)
            },
            shmem,
        })
    }

    // --- Non-blocking transfer ---

    /// Push one item to the peer. Returns `true` if accepted.
    pub fn push(&self, item: &T) -> bool {
        self.outgoing.push(item)
    }

    /// Pull one item from the peer. Returns `None` if empty.
    pub fn pull(&self) -> Option<T> {
        self.incoming.pull()
    }

    /// Push multiple items. Returns how many were accepted.
    pub fn push_batch(&self, items: &[T]) -> usize {
        self.outgoing.push_batch(items)
    }

    /// Pull multiple items. Returns how many were delivered.
    pub fn pull_batch(&self, buf: &mut [T]) -> usize {
        self.incoming.pull_batch(buf)
    }

    /// How many items can be pushed before the outgoing ring is full.
    pub fn writable(&self) -> usize {
        self.outgoing.writable()
    }

    /// How many items the peer has sent that we haven't read yet.
    pub fn readable(&self) -> usize {
        self.incoming.readable()
    }

    // --- Blocking transfer ---

    /// Push one item, blocking until accepted.
    pub fn push_blocking(&self, item: &T) {
        while !self.outgoing.push(item) {
            self.notify();
            self.wait(1000);
        }
        self.notify();
    }

    /// Pull one item, blocking until available.
    pub fn pull_blocking(&self) -> T {
        loop {
            if let Some(item) = self.incoming.pull() {
                return item;
            }
            self.wait(5000);
        }
    }

    /// Pull one item with timeout. Returns `None` if deadline expires.
    pub fn pull_timeout(&self, timeout_ms: u32) -> Option<T> {
        if let Some(item) = self.incoming.pull() {
            return Some(item);
        }

        let deadline = Instant::now() + Duration::from_millis(timeout_ms as u64);
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return None;
            }
            self.wait((remaining.as_millis() as u32).max(1));

            if let Some(item) = self.incoming.pull() {
                return Some(item);
            }
        }
    }

    // --- Notification ---

    pub fn notify(&self) {
        let _ = self.shmem.notify();
    }

    pub fn wait(&self, timeout_ms: u32) -> bool {
        self.shmem.wait(timeout_ms).is_ok()
    }

    // --- Direct ring access ---

    pub fn outgoing(&self) -> &TypedRing<T> {
        &self.outgoing
    }

    pub fn incoming(&self) -> &TypedRing<T> {
        &self.incoming
    }

    // --- Shmem plumbing ---

    pub fn shmem_id(&self) -> u32 {
        self.shmem.shmem_id()
    }

    pub fn handle(&self) -> syscall::Handle {
        self.shmem.handle()
    }

    pub fn allow(&self, peer_pid: u32) -> bool {
        self.shmem.allow(peer_pid).is_ok()
    }
}
