//! Simple Byte Ring Buffer for Console I/O
//!
//! A lock-free single-producer single-consumer byte stream ring buffer
//! built on shared memory. Simpler than Ring<S,C> which handles structured
//! request/response messages.
//!
//! # Architecture
//!
//! ```text
//! ┌────────────────────────────────────────────────────┐
//! │                 Shared Memory                      │
//! ├──────────────┬─────────────────────────────────────┤
//! │ ByteRingHeader│           Data Buffer              │
//! │ (64 bytes)   │         (size bytes)               │
//! │ head, tail   │ [....data....]                     │
//! └──────────────┴─────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! Producer (e.g., shell writing output):
//! ```ignore
//! let ring = ByteRing::create(4096)?;  // 4KB ring
//! ring.allow(consumer_pid)?;
//! // Send ring.shmem_id() to consumer via IPC
//!
//! ring.write(b"Hello, world!\n");
//! // Send doorbell via IPC channel
//! ```
//!
//! Consumer (e.g., consoled reading output):
//! ```ignore
//! let ring = ByteRing::map(shmem_id)?;
//!
//! // Wait for doorbell via event_wait(IpcReady)
//! let mut buf = [0u8; 256];
//! while let Some(n) = ring.read(&mut buf) {
//!     // Process buf[..n]
//! }
//! ```
//!
//! # Notification
//!
//! This ring does NOT include notification - use an IPC channel as a doorbell.
//! TODO: Add futex-based notification for lower latency.

use core::sync::atomic::{AtomicU32, Ordering};
use crate::syscall;

/// Ring buffer header - stored at start of shared memory
/// 64 bytes to align with cache line
#[repr(C, align(64))]
pub struct ByteRingHeader {
    /// Magic number for validation
    pub magic: u32,
    /// Ring data size (not including header)
    pub size: u32,
    /// Write position (producer updates)
    pub head: AtomicU32,
    /// Read position (consumer updates)
    pub tail: AtomicU32,
    /// Reserved for future use
    _reserved: [u32; 12],
}

const BYTE_RING_MAGIC: u32 = 0x42595445; // "BYTE"

impl ByteRingHeader {
    pub const SIZE: usize = 64;
}

/// Byte ring buffer handle
pub struct ByteRing {
    /// Shared memory ID
    shmem_id: u32,
    /// Virtual address of mapped memory
    base: u64,
    /// Ring data size
    size: u32,
    /// Are we the producer (creator)?
    is_producer: bool,
}

/// Error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ByteRingError {
    /// Failed to create shared memory
    ShmemCreateFailed,
    /// Failed to map shared memory
    ShmemMapFailed,
    /// Failed to allow access to peer
    AllowFailed,
    /// Invalid magic number (corrupted or wrong shmem)
    InvalidMagic,
    /// Ring is full (would block)
    WouldBlock,
}

impl ByteRing {
    /// Create a new byte ring as producer
    ///
    /// `size` is the data buffer size (will be rounded up to page size)
    pub fn create(size: usize) -> Result<Self, ByteRingError> {
        // Total size = header + data, rounded to page
        let total_size = ByteRingHeader::SIZE + size;
        let total_size = (total_size + 4095) & !4095;
        let data_size = total_size - ByteRingHeader::SIZE;

        // Create shared memory (shmem_create returns shmem_id and maps it)
        let mut vaddr: u64 = 0;
        let mut paddr: u64 = 0;
        let shmem_id = syscall::shmem_create(total_size, &mut vaddr, &mut paddr);
        if shmem_id < 0 {
            return Err(ByteRingError::ShmemCreateFailed);
        }
        if vaddr == 0 {
            return Err(ByteRingError::ShmemMapFailed);
        }
        let shmem_id = shmem_id as u32;
        let base = vaddr;

        // Initialize header
        let header = unsafe { &mut *(base as *mut ByteRingHeader) };
        header.magic = BYTE_RING_MAGIC;
        header.size = data_size as u32;
        header.head = AtomicU32::new(0);
        header.tail = AtomicU32::new(0);

        // Memory barrier to ensure header is visible
        core::sync::atomic::fence(Ordering::Release);

        Ok(Self {
            shmem_id,
            base,
            size: data_size as u32,
            is_producer: true,
        })
    }

    /// Map an existing byte ring as consumer
    pub fn map(shmem_id: u32) -> Result<Self, ByteRingError> {
        // Map the shared memory
        let mut vaddr: u64 = 0;
        let mut paddr: u64 = 0;
        let result = syscall::shmem_map(shmem_id, &mut vaddr, &mut paddr);
        if result < 0 || vaddr == 0 {
            return Err(ByteRingError::ShmemMapFailed);
        }
        let base = vaddr;

        // Memory barrier before reading header
        core::sync::atomic::fence(Ordering::Acquire);

        // Validate header
        let header = unsafe { &*(base as *const ByteRingHeader) };
        if header.magic != BYTE_RING_MAGIC {
            return Err(ByteRingError::InvalidMagic);
        }

        Ok(Self {
            shmem_id,
            base,
            size: header.size,
            is_producer: false,
        })
    }

    /// Allow another process to map this ring
    pub fn allow(&self, pid: u32) -> Result<(), ByteRingError> {
        let result = syscall::shmem_allow(self.shmem_id, pid);
        if result < 0 {
            Err(ByteRingError::AllowFailed)
        } else {
            Ok(())
        }
    }

    /// Get the shared memory ID (to send to peer)
    pub fn shmem_id(&self) -> u32 {
        self.shmem_id
    }

    /// Get pointer to header
    fn header(&self) -> &ByteRingHeader {
        unsafe { &*(self.base as *const ByteRingHeader) }
    }

    /// Get pointer to data buffer
    fn data(&self) -> *mut u8 {
        (self.base as usize + ByteRingHeader::SIZE) as *mut u8
    }

    /// Write bytes to the ring (producer only)
    ///
    /// Returns number of bytes written (may be less than input if ring is full)
    pub fn write(&self, data: &[u8]) -> usize {
        debug_assert!(self.is_producer);

        let header = self.header();
        let head = header.head.load(Ordering::Acquire);
        let tail = header.tail.load(Ordering::Acquire);
        let size = self.size;

        // Available space (leave one byte to distinguish full from empty)
        let available = if head >= tail {
            size - head + tail - 1
        } else {
            tail - head - 1
        } as usize;

        if available == 0 {
            return 0;
        }

        let to_write = data.len().min(available);
        let data_ptr = self.data();

        // Write data, handling wrap-around
        let head_usize = head as usize;
        let size_usize = size as usize;

        let first_chunk = (size_usize - head_usize).min(to_write);
        unsafe {
            core::ptr::copy_nonoverlapping(
                data.as_ptr(),
                data_ptr.add(head_usize),
                first_chunk,
            );
        }

        if to_write > first_chunk {
            // Wrap around
            unsafe {
                core::ptr::copy_nonoverlapping(
                    data.as_ptr().add(first_chunk),
                    data_ptr,
                    to_write - first_chunk,
                );
            }
        }

        // Update head
        let new_head = ((head as usize + to_write) % size_usize) as u32;
        header.head.store(new_head, Ordering::Release);

        to_write
    }

    /// Read bytes from the ring (consumer only)
    ///
    /// Returns number of bytes read, or 0 if empty
    pub fn read(&self, buf: &mut [u8]) -> usize {
        debug_assert!(!self.is_producer);

        let header = self.header();
        let head = header.head.load(Ordering::Acquire);
        let tail = header.tail.load(Ordering::Acquire);
        let size = self.size;

        // Available data
        let available = if head >= tail {
            head - tail
        } else {
            size - tail + head
        } as usize;

        if available == 0 {
            return 0;
        }

        let to_read = buf.len().min(available);
        let data_ptr = self.data();

        // Read data, handling wrap-around
        let tail_usize = tail as usize;
        let size_usize = size as usize;

        let first_chunk = (size_usize - tail_usize).min(to_read);
        unsafe {
            core::ptr::copy_nonoverlapping(
                data_ptr.add(tail_usize),
                buf.as_mut_ptr(),
                first_chunk,
            );
        }

        if to_read > first_chunk {
            // Wrap around
            unsafe {
                core::ptr::copy_nonoverlapping(
                    data_ptr,
                    buf.as_mut_ptr().add(first_chunk),
                    to_read - first_chunk,
                );
            }
        }

        // Update tail
        let new_tail = ((tail as usize + to_read) % size_usize) as u32;
        header.tail.store(new_tail, Ordering::Release);

        to_read
    }

    /// Check if ring has data available (for consumer)
    pub fn has_data(&self) -> bool {
        let header = self.header();
        let head = header.head.load(Ordering::Acquire);
        let tail = header.tail.load(Ordering::Acquire);
        head != tail
    }

    /// Check if ring has space available (for producer)
    pub fn has_space(&self) -> bool {
        let header = self.header();
        let head = header.head.load(Ordering::Acquire);
        let tail = header.tail.load(Ordering::Acquire);
        let size = self.size;

        let available = if head >= tail {
            size - head + tail - 1
        } else {
            tail - head - 1
        };
        available > 0
    }

    /// Get number of bytes available to read
    pub fn available(&self) -> usize {
        let header = self.header();
        let head = header.head.load(Ordering::Acquire);
        let tail = header.tail.load(Ordering::Acquire);
        let size = self.size;

        if head >= tail {
            (head - tail) as usize
        } else {
            (size - tail + head) as usize
        }
    }
}

// TODO: Add futex-based notification
// - Producer calls futex_wake(&header.head) after write
// - Consumer calls futex_wait(&header.head, last_head) to sleep until data
// This eliminates the need for IPC doorbell messages
