//! Console Ring - High-performance bidirectional IPC for console I/O
//!
//! Uses shared memory with lock-free ring buffers for shell <-> consoled communication.
//!
//! # Memory Layout
//!
//! ```text
//! ┌────────────────────────────────────────────────────────────────────┐
//! │ RingHeader (64 bytes)                                              │
//! │   tx_head, tx_tail, rx_head, rx_tail, tx_config, rx_config, flags  │
//! ├────────────────────────────────────────────────────────────────────┤
//! │ TX Ring Buffer (shell output -> consoled -> UART)                  │
//! │   Size: ring_config::ring_size(tx_config)                          │
//! ├────────────────────────────────────────────────────────────────────┤
//! │ RX Ring Buffer (UART -> consoled -> shell input)                   │
//! │   Size: ring_config::ring_size(rx_config)                          │
//! └────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Protocol
//!
//! - TX: Shell writes output, consoled reads and forwards to UART
//! - RX: Consoled writes UART input, shell reads
//! - Lock-free using atomic head/tail pointers
//! - Notifications via shmem notify/wait

use core::sync::atomic::Ordering;
use crate::ipc::Shmem;

// Re-export ring types from abi
pub use abi::{RingHeader, RingConfig, ring_config, ring_presets};

/// Console ring - bidirectional I/O over shared memory
pub struct ConsoleRing {
    /// Shared memory region
    shmem: Shmem,
    /// Are we the owner (creator)?
    _is_owner: bool,
    /// TX config
    tx_config: RingConfig,
    /// RX config
    rx_config: RingConfig,
}

impl ConsoleRing {
    /// Create a new console ring (server/consoled side)
    ///
    /// Uses preset configurations optimized for console I/O:
    /// - TX: Large buffer for shell output bursts
    /// - RX: Small buffer for UART input
    pub fn create() -> Option<Self> {
        Self::create_with_config(ring_presets::CONSOLE_TX, ring_presets::CONSOLE_RX)
    }

    /// Create with custom configuration
    pub fn create_with_config(tx_config: RingConfig, rx_config: RingConfig) -> Option<Self> {
        // Calculate sizes
        let tx_size = ring_config::ring_size(tx_config);
        let rx_size = ring_config::ring_size(rx_config);
        let total_size = 64 + tx_size + rx_size; // Header + TX + RX

        // Create shared memory via unified interface
        let shmem = Shmem::create(total_size).ok()?;

        // Initialize header
        let vaddr = shmem.vaddr();
        unsafe {
            let header = vaddr as *mut RingHeader;
            // Zero-init first
            core::ptr::write_bytes(header as *mut u8, 0, 64);
            // Set config
            (*header).tx_config = tx_config;
            (*header).rx_config = rx_config;
            // Head/tail already 0 from zero-init
        }

        // MEMORY BARRIER: Ensure header writes are visible to other processes
        // before we send the shmem_id. Without this, shell might map the shmem
        // and see uninitialized tx_config/rx_config (causing crash at FAR=0x10).
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);

        Some(Self {
            shmem,
            _is_owner: true,
            tx_config,
            rx_config,
        })
    }

    /// Map an existing console ring (client/shell side)
    pub fn map(shmem_id: u32) -> Option<Self> {
        // Open existing shmem via unified interface
        let shmem = Shmem::open_existing(shmem_id).ok()?;

        // MEMORY BARRIER: Ensure we see all writes from the creator (consoled)
        // before reading the header. Pairs with Release fence in create_with_config.
        core::sync::atomic::fence(core::sync::atomic::Ordering::Acquire);

        // Read config from header
        let vaddr = shmem.vaddr();
        let header = unsafe { &*(vaddr as *const RingHeader) };
        let tx_config = header.tx_config;
        let rx_config = header.rx_config;

        Some(Self {
            shmem,
            _is_owner: false,
            tx_config,
            rx_config,
        })
    }

    /// Get shmem ID (to send to peer for mapping)
    pub fn shmem_id(&self) -> u32 {
        self.shmem.shmem_id()
    }

    /// Get shmem handle (for adding to mux)
    pub fn shmem_handle(&self) -> crate::syscall::Handle {
        self.shmem.handle()
    }

    /// Allow another process to map this ring
    pub fn allow(&self, peer_pid: u32) -> bool {
        self.shmem.allow(peer_pid).is_ok()
    }

    /// Get virtual address
    fn vaddr(&self) -> u64 {
        self.shmem.vaddr()
    }

    /// Get ring header
    fn header(&self) -> &RingHeader {
        unsafe { &*(self.vaddr() as *const RingHeader) }
    }

    /// Get TX buffer pointer
    fn tx_buf(&self) -> *mut u8 {
        unsafe { (self.vaddr() as *mut u8).add(64) }
    }

    /// Get RX buffer pointer
    fn rx_buf(&self) -> *mut u8 {
        let tx_size = ring_config::ring_size(self.tx_config);
        unsafe { (self.vaddr() as *mut u8).add(64 + tx_size) }
    }

    /// Get TX buffer size
    pub fn tx_size(&self) -> usize {
        ring_config::ring_size(self.tx_config)
    }

    /// Get RX buffer size
    pub fn rx_size(&self) -> usize {
        ring_config::ring_size(self.rx_config)
    }

    // ========================================================================
    // TX Operations (shell writes, consoled reads)
    // ========================================================================

    /// Check how many bytes can be written to TX
    pub fn tx_space(&self) -> usize {
        let header = self.header();
        let head = header.tx_head.load(Ordering::Acquire);
        let tail = header.tx_tail.load(Ordering::Relaxed);
        let size = self.tx_size();
        let mask = size - 1;

        // Available space = size - 1 - used
        // Used = (tail - head) & mask
        let used = (tail.wrapping_sub(head) as usize) & mask;
        size - 1 - used
    }

    /// Check how many bytes are available to read from TX
    pub fn tx_available(&self) -> usize {
        let header = self.header();
        let head = header.tx_head.load(Ordering::Relaxed);
        let tail = header.tx_tail.load(Ordering::Acquire);
        let mask = self.tx_size() - 1;
        (tail.wrapping_sub(head) as usize) & mask
    }

    /// Write to TX ring (shell side)
    /// Returns number of bytes written
    pub fn tx_write(&self, data: &[u8]) -> usize {
        let header = self.header();
        let size = self.tx_size();
        let mask = (size - 1) as u32;

        let head = header.tx_head.load(Ordering::Acquire);
        let tail = header.tx_tail.load(Ordering::Relaxed);

        // Calculate space
        let used = tail.wrapping_sub(head) & mask;
        let space = size - 1 - used as usize;
        let to_write = data.len().min(space);

        if to_write == 0 {
            return 0;
        }

        // Write data
        let buf = self.tx_buf();
        let tail_idx = (tail & mask) as usize;

        // Handle wrap-around
        let first_chunk = to_write.min(size - tail_idx);
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(tail_idx), first_chunk);
            if first_chunk < to_write {
                core::ptr::copy_nonoverlapping(
                    data.as_ptr().add(first_chunk),
                    buf,
                    to_write - first_chunk
                );
            }
        }

        // Memory barrier before updating tail
        core::sync::atomic::fence(Ordering::Release);

        // Update tail
        let new_tail = tail.wrapping_add(to_write as u32);
        header.tx_tail.store(new_tail, Ordering::Release);

        to_write
    }

    /// Read from TX ring (consoled side)
    /// Returns number of bytes read
    pub fn tx_read(&self, buf: &mut [u8]) -> usize {
        let header = self.header();
        let size = self.tx_size();
        let mask = (size - 1) as u32;

        let head = header.tx_head.load(Ordering::Relaxed);
        let tail = header.tx_tail.load(Ordering::Acquire);

        // Calculate available
        let available = (tail.wrapping_sub(head) & mask) as usize;
        let to_read = buf.len().min(available);

        if to_read == 0 {
            return 0;
        }

        // Read data
        let ring_buf = self.tx_buf();
        let head_idx = (head & mask) as usize;

        // Handle wrap-around
        let first_chunk = to_read.min(size - head_idx);
        unsafe {
            core::ptr::copy_nonoverlapping(ring_buf.add(head_idx), buf.as_mut_ptr(), first_chunk);
            if first_chunk < to_read {
                core::ptr::copy_nonoverlapping(
                    ring_buf,
                    buf.as_mut_ptr().add(first_chunk),
                    to_read - first_chunk
                );
            }
        }

        // Update head
        let new_head = head.wrapping_add(to_read as u32);
        header.tx_head.store(new_head, Ordering::Release);

        to_read
    }

    // ========================================================================
    // RX Operations (consoled writes, shell reads)
    // ========================================================================

    /// Check how many bytes can be written to RX
    pub fn rx_space(&self) -> usize {
        let header = self.header();
        let head = header.rx_head.load(Ordering::Acquire);
        let tail = header.rx_tail.load(Ordering::Relaxed);
        let size = self.rx_size();
        let mask = size - 1;

        let used = (tail.wrapping_sub(head) as usize) & mask;
        size - 1 - used
    }

    /// Check how many bytes are available to read from RX
    pub fn rx_available(&self) -> usize {
        let header = self.header();
        let head = header.rx_head.load(Ordering::Relaxed);
        let tail = header.rx_tail.load(Ordering::Acquire);
        let mask = self.rx_size() - 1;
        (tail.wrapping_sub(head) as usize) & mask
    }

    /// Write to RX ring (consoled side)
    /// Returns number of bytes written
    pub fn rx_write(&self, data: &[u8]) -> usize {
        let header = self.header();
        let size = self.rx_size();
        let mask = (size - 1) as u32;

        let head = header.rx_head.load(Ordering::Acquire);
        let tail = header.rx_tail.load(Ordering::Relaxed);

        // Calculate space
        let used = tail.wrapping_sub(head) & mask;
        let space = size - 1 - used as usize;
        let to_write = data.len().min(space);

        if to_write == 0 {
            return 0;
        }

        // Write data
        let buf = self.rx_buf();
        let tail_idx = (tail & mask) as usize;

        // Handle wrap-around
        let first_chunk = to_write.min(size - tail_idx);
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(tail_idx), first_chunk);
            if first_chunk < to_write {
                core::ptr::copy_nonoverlapping(
                    data.as_ptr().add(first_chunk),
                    buf,
                    to_write - first_chunk
                );
            }
        }

        // Memory barrier before updating tail
        core::sync::atomic::fence(Ordering::Release);

        // Update tail
        let new_tail = tail.wrapping_add(to_write as u32);
        header.rx_tail.store(new_tail, Ordering::Release);

        to_write
    }

    /// Read from RX ring (shell side)
    /// Returns number of bytes read
    pub fn rx_read(&self, buf: &mut [u8]) -> usize {
        let header = self.header();
        let size = self.rx_size();
        let mask = (size - 1) as u32;

        let head = header.rx_head.load(Ordering::Relaxed);
        let tail = header.rx_tail.load(Ordering::Acquire);

        // Calculate available
        let available = (tail.wrapping_sub(head) & mask) as usize;
        let to_read = buf.len().min(available);

        if to_read == 0 {
            return 0;
        }

        // Read data
        let ring_buf = self.rx_buf();
        let head_idx = (head & mask) as usize;

        // Handle wrap-around
        let first_chunk = to_read.min(size - head_idx);
        unsafe {
            core::ptr::copy_nonoverlapping(ring_buf.add(head_idx), buf.as_mut_ptr(), first_chunk);
            if first_chunk < to_read {
                core::ptr::copy_nonoverlapping(
                    ring_buf,
                    buf.as_mut_ptr().add(first_chunk),
                    to_read - first_chunk
                );
            }
        }

        // Update head
        let new_head = head.wrapping_add(to_read as u32);
        header.rx_head.store(new_head, Ordering::Release);

        to_read
    }

    // ========================================================================
    // Notification
    // ========================================================================

    /// Notify peer that data is available
    pub fn notify(&self) -> u32 {
        self.shmem.notify().unwrap_or(0)
    }

    /// Wait for notification (with timeout)
    pub fn wait(&self, timeout_ms: u32) -> bool {
        self.shmem.wait(timeout_ms).is_ok()
    }

    /// Get the underlying shmem handle for use with Mux
    pub fn handle(&self) -> crate::syscall::Handle {
        self.shmem.handle()
    }
}
