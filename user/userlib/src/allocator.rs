//! Global heap allocator for userspace programs.
//!
//! Free-list allocator backed by mmap. Allocates large chunks from the kernel
//! and carves them into blocks internally. Never munmaps — the kernel reclaims
//! all memory on task exit.
//!
//! Design:
//! - Linked free list sorted by address for O(1) coalescing
//! - 16-byte block header: { size, next }
//! - All blocks 16-byte aligned (header is 16 bytes, payload at +16)
//! - Minimum block = 32 bytes (16 header + 16 usable)
//! - Chunk growth: 64KB → 128KB → 256KB → ... capped at 1MB
//! - SpinLock via AtomicBool CAS for thread safety

use core::alloc::{GlobalAlloc, Layout};
use core::cell::UnsafeCell;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::syscall;

const HEADER_SIZE: usize = 16;
const MIN_BLOCK: usize = 32; // header + 16 bytes minimum payload
const INITIAL_CHUNK: usize = 64 * 1024;
const MAX_CHUNK: usize = 1024 * 1024;
const BLOCK_ALIGN: usize = 16;

/// Free block node in the linked list.
///
/// Stored at the start of each free block. `size` is the total block size
/// including this header.
#[repr(C, align(16))]
struct FreeNode {
    size: usize,
    next: *mut FreeNode,
}

/// Spin lock using atomic CAS.
struct Lock {
    locked: AtomicBool,
}

impl Lock {
    const fn new() -> Self {
        Self { locked: AtomicBool::new(false) }
    }

    fn acquire(&self) {
        while self.locked.compare_exchange_weak(
            false, true,
            Ordering::Acquire,
            Ordering::Relaxed,
        ).is_err() {
            core::hint::spin_loop();
        }
    }

    fn release(&self) {
        self.locked.store(false, Ordering::Release);
    }
}

/// Global heap allocator.
///
/// Manages a free list of blocks carved from mmap'd chunks.
pub struct FilamentAlloc {
    lock: Lock,
    head: UnsafeCell<*mut FreeNode>,
    next_chunk: AtomicUsize,
}

// SAFETY: Access is serialized by the spin lock.
unsafe impl Sync for FilamentAlloc {}

impl FilamentAlloc {
    pub const fn new() -> Self {
        Self {
            lock: Lock::new(),
            head: UnsafeCell::new(ptr::null_mut()),
            next_chunk: AtomicUsize::new(INITIAL_CHUNK),
        }
    }

    /// Request a new chunk of memory from the kernel via mmap.
    ///
    /// Returns a pointer to the new region, or null on failure.
    /// Grows chunk size exponentially up to MAX_CHUNK.
    unsafe fn grow(&self, min_size: usize) -> *mut u8 {
        let chunk = self.next_chunk.load(Ordering::Relaxed);
        let size = if min_size > chunk { min_size } else { chunk };
        // Round up to page boundary
        let size = (size + 4095) & !4095;

        let ret = syscall::mmap(0, size, syscall::PROT_READ | syscall::PROT_WRITE);
        if ret <= 0 {
            return ptr::null_mut();
        }

        // Grow chunk size for next time (double, capped at MAX_CHUNK)
        let next = (chunk * 2).min(MAX_CHUNK);
        self.next_chunk.store(next, Ordering::Relaxed);

        let base = ret as *mut u8;

        // Create a single free block spanning the entire chunk
        let node = base as *mut FreeNode;
        (*node).size = size;
        (*node).next = ptr::null_mut();

        // Insert into free list in address order
        self.insert_free(node);

        base
    }

    /// Insert a free node into the list in address order and coalesce
    /// with adjacent blocks.
    ///
    /// SAFETY: Must be called with the lock held.
    unsafe fn insert_free(&self, node: *mut FreeNode) {
        let head_ptr = self.head.get();
        let mut prev: *mut FreeNode = ptr::null_mut();
        let mut curr = *head_ptr;

        // Find insertion point (sorted by address)
        while !curr.is_null() && (curr as usize) < (node as usize) {
            prev = curr;
            curr = (*curr).next;
        }

        // Insert node between prev and curr
        (*node).next = curr;
        if prev.is_null() {
            *head_ptr = node;
        } else {
            (*prev).next = node;
        }

        // Coalesce with next block
        if !curr.is_null() {
            let node_end = (node as usize) + (*node).size;
            if node_end == curr as usize {
                (*node).size += (*curr).size;
                (*node).next = (*curr).next;
            }
        }

        // Coalesce with previous block
        if !prev.is_null() {
            let prev_end = (prev as usize) + (*prev).size;
            if prev_end == node as usize {
                (*prev).size += (*node).size;
                (*prev).next = (*node).next;
            }
        }
    }

    /// Find and remove a block of at least `size` bytes from the free list.
    /// Splits the block if the remainder is large enough.
    ///
    /// SAFETY: Must be called with the lock held.
    unsafe fn find_block(&self, size: usize) -> *mut FreeNode {
        let head_ptr = self.head.get();
        let mut prev: *mut FreeNode = ptr::null_mut();
        let mut curr = *head_ptr;

        while !curr.is_null() {
            if (*curr).size >= size {
                // Found a fit — check if we can split
                let remainder = (*curr).size - size;
                if remainder >= MIN_BLOCK {
                    // Split: create a new free node after the allocated block
                    let split = (curr as *mut u8).add(size) as *mut FreeNode;
                    (*split).size = remainder;
                    (*split).next = (*curr).next;
                    (*curr).size = size;

                    // Replace curr with split in the list
                    if prev.is_null() {
                        *head_ptr = split;
                    } else {
                        (*prev).next = split;
                    }
                } else {
                    // Use the whole block
                    if prev.is_null() {
                        *head_ptr = (*curr).next;
                    } else {
                        (*prev).next = (*curr).next;
                    }
                }
                return curr;
            }
            prev = curr;
            curr = (*curr).next;
        }

        ptr::null_mut()
    }
}

unsafe impl GlobalAlloc for FilamentAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let align = layout.align();
        let payload_size = layout.size();

        if align <= BLOCK_ALIGN {
            // Common path: alignment ≤ 16, header provides natural alignment
            let total = (HEADER_SIZE + payload_size).max(MIN_BLOCK);
            // Round up to 16-byte boundary
            let total = (total + BLOCK_ALIGN - 1) & !(BLOCK_ALIGN - 1);

            self.lock.acquire();

            let mut block = self.find_block(total);
            if block.is_null() {
                // No block found — grow the heap and retry
                let needed = total + HEADER_SIZE; // extra for chunk overhead
                if self.grow(needed).is_null() {
                    self.lock.release();
                    return ptr::null_mut();
                }
                block = self.find_block(total);
                if block.is_null() {
                    self.lock.release();
                    return ptr::null_mut();
                }
            }

            self.lock.release();
            (block as *mut u8).add(HEADER_SIZE)
        } else {
            // Rare path: alignment > 16
            // Over-allocate so we can align the payload and store a backpointer.
            //
            // Layout: [header][padding][backptr(8 bytes)][aligned payload]
            // backptr at payload-8 stores the real block address.
            let worst_padding = align; // at most `align` bytes of padding needed
            let total = HEADER_SIZE + worst_padding + payload_size;
            let total = (total + BLOCK_ALIGN - 1) & !(BLOCK_ALIGN - 1);
            let total = total.max(MIN_BLOCK);

            self.lock.acquire();

            let mut block = self.find_block(total);
            if block.is_null() {
                let needed = total + HEADER_SIZE;
                if self.grow(needed).is_null() {
                    self.lock.release();
                    return ptr::null_mut();
                }
                block = self.find_block(total);
                if block.is_null() {
                    self.lock.release();
                    return ptr::null_mut();
                }
            }

            self.lock.release();

            // Find aligned payload address after header + backpointer space
            let block_addr = block as usize;
            let earliest_payload = block_addr + HEADER_SIZE + 8; // room for backpointer
            let aligned = (earliest_payload + align - 1) & !(align - 1);

            // Store backpointer at payload - 8
            let backptr = (aligned - 8) as *mut usize;
            *backptr = block_addr;

            aligned as *mut u8
        }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        if ptr.is_null() {
            return;
        }

        let block = if layout.align() <= BLOCK_ALIGN {
            // Common path: block header is right before payload
            ptr.sub(HEADER_SIZE) as *mut FreeNode
        } else {
            // Rare path: read backpointer at ptr - 8
            let backptr = (ptr as usize - 8) as *const usize;
            *backptr as *mut FreeNode
        };

        self.lock.acquire();
        self.insert_free(block);
        self.lock.release();
    }
}
